/*
 * Copyright (c) 2025 efogdev
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/kernel.h>
#include <zmk_esb/protocol.h>
#include <zmk_esb/channel_hop.h>
#include "channel_hop_ep.h"
#include "esb_transport.h"

#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_QUARANTINE_PERSIST) || IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_RSSI_WEIGHT)
#include <zephyr/settings/settings.h>
#endif

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(zmk_esb_chhop_ep, CONFIG_ZMK_ESB_ENDPOINT_LOG_LEVEL);

#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_HOP)

/* Compile-time invariants for the hop / rollback timing knobs. Cross-firmware
 * constraints against DONGLE_* live on the dongle side and cannot be checked
 * here (the dongle Kconfig is not visible in the endpoint build); see each
 * symbol's help text for those. */
BUILD_ASSERT(CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_HOP_WEAK_LINK_MS == 0 ||
             CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_HOP_WEAK_LINK_MS >
             CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_HOP_TX_FAIL_WINDOW_MS,
             "WEAK_LINK_MS (when nonzero) must exceed TX_FAIL_WINDOW_MS so "
             "the fast dead-link trigger fires before the weak-link path");
BUILD_ASSERT(CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_HOP_RENDEZVOUS_FALLBACK_MS == 0 ||
             CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_HOP_RENDEZVOUS_FALLBACK_MS <
             CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_HOP_COOLDOWN_MS,
             "RENDEZVOUS_FALLBACK_MS (when nonzero) must be < COOLDOWN_MS or "
             "the fallback can never fire — the cooldown gate elapses first");
BUILD_ASSERT(CONFIG_ZMK_ESB_ENDPOINT_MOTION_MAX_STALE_MS >=
             CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_HOP_POST_QUIET_MS,
             "MOTION_MAX_STALE_MS must be >= POST_QUIET_MS or accumulated "
             "motion is dropped before the post-hop quiet window even ends");
BUILD_ASSERT(CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_HOP_NEGOTIATE_INTERVAL_MS >
             CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_HOP_NEGOTIATE_RETRY_MS,
             "Steady-state NEGOTIATE_INTERVAL_MS must be larger than the "
             "fast-retry NEGOTIATE_RETRY_MS or the 'fast' path is no faster");
BUILD_ASSERT(CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_HOP_RENDEZVOUS_TIMEOUT_MS <
             CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_HOP_POST_BURST_INTERVAL_MS,
             "RENDEZVOUS_TIMEOUT_MS must be < POST_BURST_INTERVAL_MS so the "
             "rendezvous side-trip cannot stall past the next burst tick");

#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_RSSI_WEIGHT)
BUILD_ASSERT(CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_RSSI_WEIGHT_MAX >
             CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_RSSI_WEIGHT_MIN,
             "RSSI_WEIGHT_MAX must exceed RSSI_WEIGHT_MIN");
BUILD_ASSERT(CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_RSSI_EWMA_ALPHA <=
             CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_RSSI_WEIGHT_SCALE,
             "RSSI_EWMA_ALPHA must not exceed RSSI_WEIGHT_SCALE");
#endif

static struct quarantine_state m_quarantine;

/* Per-channel RSSI EWMA, stored as signed dBm (negative; higher == stronger).
 * Address-taken even when RSSI weighting is off, where the picker ignores it. */
static struct channel_rssi_state m_channel_rssi;
static uint8_t m_committed_next = CHANNEL_HOP_INVALID;
static bool m_link_up;

/* True iff the current m_committed_next has been acknowledged by the
 * dongle via CONFIRM AND nothing has happened since that could have
 * invalidated the dongle's matching view. While true, periodic
 * re-affirm PROPOSALs are suppressed — both sides already agree, so
 * the steady-state TX is pure airtime overhead. The user-traffic ACK
 * path carries a REQUEST whenever the dongle's committed_next is
 * empty (see channel_hop_dongle_after_rx), so silent state loss on
 * the dongle (reboot, speculative-hop validate fail) surfaces as a
 * REQUEST on the very next user TX and flips this flag back to false. */
static bool m_committed_synced;

/* Channel we hopped away from on the most recent hop, iff no TX_SUCCESS
 * has been observed since that hop. If a subsequent hop fires while this
 * is still set, the new channel clearly isn't working either — revert
 * here instead of picking yet another fresh candidate that might be just
 * as bad. Cleared on any TX success, on explicit revert, and on link
 * down. Single-byte read/written from ISR and workqueue; the race is
 * benign (stale read at worst skips a revert). */
static volatile uint8_t m_prev_channel = CHANNEL_HOP_INVALID;

/* Last non-default channel we held. Unlike m_prev_channel this is not
 * cleared on TX success or revert — it is the fallback target for a hop
 * that has no committed_next while already sitting on the rendezvous
 * channel, so the pair can oscillate between two predictable points.
 * Workqueue-only access (hop_work / commit_hop_bookkeeping). */
static uint8_t m_prev_known_channel = CHANNEL_HOP_INVALID;

/* Active vs idle. Starts ACTIVE on connect; transitions to IDLE after
 * IDLE_THRESHOLD_MS of no user TX. A user TX while idle sends us back to
 * active (no explicit packet — whatever the user TX is serves as the
 * "I'm awake" signal to the dongle). Only while active do we
 *   - run the negotiate work
 *   - honour the TX-fail hop trigger
 * which prevents an idle link from hopping for no reason. */
static bool m_active;

static struct k_work_delayable negotiate_work;
static struct k_work_delayable idle_check_work;
static struct k_work_delayable post_hop_burst_work;
static struct k_work_delayable hop_retry_work;
static struct k_work           hop_work;

static uint8_t m_post_hop_burst_remaining;

/* Sticky candidate for the post-hop burst: the channel post_hop_burst_work_fn
 * proposes on every tick until a CONFIRM lands. pick_candidate() is
 * randomised, so without a sticky value each tick would propose a
 * different channel and the dongle's commit-on-every-PROPOSAL semantics
 * would make committed_next oscillate across the burst window — observed
 * live as 16/93/16 across three ticks. Reset at start_post_hop_burst() and
 * at burst completion. negotiate_work_fn deliberately bypasses this cache:
 * it retries on its own cadence outside the burst, and a dead first pick
 * sticky across retries would freeze recovery on one bad channel. */
static uint8_t m_pending_candidate = CHANNEL_HOP_INVALID;

/* Counter of PROPOSAL attempts left in the current REQUEST-driven burst.
 * The dongle queues an ESB_PKT_CHANNEL_HOP_REQUEST whenever its
 * committed_next is INVALID; channel_hop_ep_on_request() seeds this with
 * a few attempts so a single dropped PROPOSAL on a weak link doesn't
 * leave the dongle waiting for the next 60 s steady-state interval. While
 * the counter is non-zero, negotiate_work_fn re-arms at the fast retry
 * cadence (NEGOTIATE_RETRY_MS) regardless of needs_commit — the dongle
 * has explicitly told us its state is empty, so re-affirming aggressively
 * is correct even if our own committed_next is valid. Decremented per
 * negotiate_work fire; arrival of a CONFIRM (channel_hop_ep_on_rx)
 * resets to zero since at that point the dongle is back in sync. */
static uint8_t m_request_burst_remaining;

/* Uptime (ms, 32-bit wrap-safe compare) until which TX-fail-triggered hops
 * are suppressed. Set when a hop commits so the endpoint lingers on the new
 * channel long enough for the dongle's silence watchdog
 * (DONGLE_CHANNEL_HOP_RX_SILENCE_MS) and validate window
 * (DONGLE_CHANNEL_HOP_VALIDATE_MS) to fire. Without this, a short burst of
 * consecutive TX fails on a dead link can bounce the endpoint off the new
 * channel before the dongle ever arrives, so the speculative hop always
 * validates against an empty channel. Zero means "no cooldown active". */
static uint32_t m_hop_cooldown_until;

/* Uptime (ms) of the most recent positive link event on the current
 * channel — either a hop commit (dongle *should* follow) or a TX_SUCCESS
 * (dongle actually did follow, or link is healthy). Used by the cooldown
 * rendezvous fallback: while cooldown is active, if no positive event has
 * occurred for RENDEZVOUS_FALLBACK_MS, the link is dead long enough to
 * justify bypassing the cooldown and forcing a rendezvous hop.
 *
 * TX_SUCCESS refreshes this to now (rather than zeroing it). Zeroing on
 * TX_SUCCESS would let one lucky packet inside the cooldown window
 * permanently disable fallback for the rest of that cooldown — even if
 * the link went dead immediately afterwards — and two such
 * success→silence cycles back-to-back could keep the endpoint from
 * firing any hop for roughly twice CHANNEL_HOP_COOLDOWN_MS. Refreshing
 * keeps the "no activity for RENDEZVOUS_FALLBACK_MS" invariant correct
 * across intermittent links.
 *
 * Zero means "no hop has ever committed" — set only by init / disconnect
 * and preserved as the fallback's "is there anything to fall back on?"
 * gate. 32-bit wrap-safe compare. */
static volatile uint32_t m_last_link_event_ms;

#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_RENDEZVOUS)
/* Set by channel_hop_ep_on_tx_fail_isr when the cooldown-fallback condition
 * triggers; consumed by hop_work_fn to bypass the normal candidate-selection
 * path and target the rendezvous channel directly. Single-bit flag, single
 * producer (ISR), single consumer (workqueue) — no lock. */
static volatile bool m_rendezvous_fallback_pending;

/* Uptime (ms, wrap-safe) until which TX-fail hops are suppressed while the
 * endpoint sits on the rendezvous channel after a fallback. Keeps the
 * endpoint a stationary target for the dongle's rollback dwell instead of
 * bouncing away after one cooldown. Cleared to 0 by the first TX_SUCCESS
 * (re-acquired) and on init / disconnect; otherwise it simply expires and
 * normal hopping resumes. Zero means "not holding". */
static volatile uint32_t m_rendezvous_hold_until;
#endif

#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_COOP_HOP)
/* Cooperative-hop state machine. Fires earlier than the existing TX-fail /
 * weak-link triggers, in the "moderately degraded" link zone where
 * retransmits are climbing but most packets still get through. Two-step
 * handshake:
 *
 *   COOP_IDLE   → no handshake in progress.
 *   COOP_OFFERED → HOP_OFFER queued/in-flight; awaiting TX_SUCCESS to
 *                  arm the commit timer. Cleared on TX_FAILED.
 *   COOP_ARMED  → both sides agreed; coop_hop_commit_work scheduled to
 *                 fire in ~hop_in_ms. Cleared by the commit work itself.
 *
 * `m_coop_hop_target` tracks the agreed channel — kept SEPARATE from
 * m_committed_next so a stale PROPOSAL/CONFIRM round-trip can't poison
 * the coop-hop target mid-handshake.
 *
 * `m_coop_hop_seq` is an 8-bit nonce; HOP_ACCEPT must echo it for the
 * endpoint to honor a counter-proposal. ESB hardware dedupes
 * retransmits, but the seq protects the application layer against
 * out-of-order CONFIRM-vs-OFFER races where a stale PROPOSAL CONFIRM
 * could otherwise arrive after a fresh OFFER and be mis-attributed.
 *
 * `m_last_tx_was_offer` is set by send_offer() right before the radio
 * transmits and cleared in TX_SUCCESS / TX_FAILED handlers. Lets the
 * TX_SUCCESS hook decide whether the just-completed TX was the OFFER
 * (and therefore that the ACK payload contained an ACCEPT).
 *
 * `m_coop_hop_cooldown_until` rate-limits coop hops independently of
 * the existing TX-fail-driven cooldown — a successful coop hop should
 * not block a subsequent speculative hop and vice versa. */
enum coop_hop_state {
    COOP_IDLE = 0,
    COOP_OFFERED,
    COOP_ARMED,
};

static volatile uint8_t m_coop_hop_state;
static uint8_t  m_coop_hop_target;
static uint8_t  m_coop_hop_seq;
static volatile bool m_last_tx_was_offer;
static uint32_t m_coop_hop_cooldown_until;

static struct k_work_delayable coop_hop_offer_work;
static struct k_work_delayable coop_hop_commit_work;
#endif /* COOP_HOP */

static bool hop_cooldown_active(void) {
    if (m_hop_cooldown_until == 0) {
        return false;
    }
    return (int32_t)(m_hop_cooldown_until - k_uptime_get_32()) > 0;
}

static void enter_idle_state(void);
static void enter_active_state(bool send_initial_proposal);
#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_COOP_HOP)
static void coop_hop_abort(void);
#endif

#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_QUARANTINE_PERSIST)
/* In-memory evidence: how many times each channel has been condemned by a
 * fail-driven hop. Distilled to the persisted avoided set every
 * PERSIST_INTERVAL_MS; aged by persist_decay_check(). */
struct hit_entry {
    uint8_t  channel;
    uint16_t hits;
    uint16_t session_hits;
};
static struct hit_entry m_hits[HIT_TABLE_SIZE];
static uint8_t m_hit_count;

static struct k_work_delayable persist_work;

/* Powered-on-time decay clock. There is no RTC, so elapsed wall time cannot
 * survive a power cycle; instead we accumulate uptime deltas (which keep
 * advancing through idle), sampled at boot, wake, and every speculative hop.
 * The accumulator is kept in whole minutes (m_decay_accum_min) and persisted,
 * so progress toward the next DECAY_S period survives reboots; m_decay_frac_ms
 * carries the sub-minute remainder (RAM only — losing <1 min on reboot is
 * harmless). m_decay_last_uptime is the previous sample, re-anchored to the
 * current uptime on boot/wrap so the reset-to-zero after a reboot — where
 * `now` jumps below the previous sample — contributes nothing. */
static uint32_t m_decay_accum_min;
static uint32_t m_decay_frac_ms;
static uint32_t m_decay_last_uptime;
static uint16_t m_saved_accum_min;

/* Serialised offender list as last rebuilt, and the copy last written to NVS.
 * The offender list is rewritten only when it changes; the decay accumulator
 * rides along in the same record but is flushed at coarse granularity so the
 * per-hop checks don't burn flash. */
static uint8_t m_off_buf[PERSIST_OFF_MAX];
static uint8_t m_off_len;
static uint8_t m_saved_off[PERSIST_OFF_MAX];
static uint8_t m_saved_off_len;

static uint8_t persist_n(void) {
    const uint8_t n = (uint8_t)ZRC_GET(PERSIST_ZRC_KEY, PERSIST_N_MAX);
    return n > PERSIST_N_MAX ? PERSIST_N_MAX : n;
}

/* DECAY_S expressed in whole minutes (the accumulator's unit); 0 disables
 * decay, and a sub-minute interval is clamped up to 1 minute. */
static uint32_t persist_decay_period_min(void) {
    const uint32_t s = CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_QUARANTINE_PERSIST_DECAY_S;
    if (s == 0) {
        return 0;
    }
    const uint32_t m = s / 60u;
    return m == 0 ? 1u : m;
}

/* Record one condemnation of `channel`. Space-Saving style: count if
 * present, append while there is room, otherwise overwrite the
 * least-offending slot and inherit its count + 1 so a genuinely bad
 * newcomer can still climb past stale entries. */
static void persist_record_hit(const uint8_t channel) {
    if (channel >= CHANNEL_HOP_CHANNEL_COUNT || channel == esb_transport_get_rendezvous_channel()) {
        return;
    }
    for (uint8_t i = 0; i < m_hit_count; i++) {
        if (m_hits[i].channel == channel) {
            if (m_hits[i].hits < UINT16_MAX) {
                m_hits[i].hits++;
            }
            if (m_hits[i].session_hits < UINT16_MAX) {
                m_hits[i].session_hits++;
            }
            return;
        }
    }
    if (m_hit_count < ARRAY_SIZE(m_hits)) {
        m_hits[m_hit_count].channel = channel;
        m_hits[m_hit_count].hits = 1;
        m_hits[m_hit_count].session_hits = 1;
        m_hit_count++;
        return;
    }
    uint8_t min_i = 0;
    for (uint8_t i = 1; i < m_hit_count; i++) {
        if (m_hits[i].hits < m_hits[min_i].hits) {
            min_i = i;
        }
    }
    m_hits[min_i].channel = channel;
    if (m_hits[min_i].hits < UINT16_MAX) {
        m_hits[min_i].hits++;
    }
    m_hits[min_i].session_hits = 1;
}

/* A channel we are deliberately reoccupying (revert / rendezvous): drop its
 * evidence and remove it from the live avoided set so the picker and its
 * buffer stop fighting the move. The next distill persists the change. */
static void persist_reclaim(const uint8_t channel) {
    for (uint8_t i = 0; i < m_hit_count; i++) {
        if (m_hits[i].channel == channel) {
            m_hits[i] = m_hits[--m_hit_count];
            break;
        }
    }
    struct persistent_quarantine *p = &m_quarantine.persistent;
    for (uint8_t i = 0; i < p->count; i++) {
        if (p->channels[i] == channel) {
            p->channels[i] = p->channels[--p->count];
            break;
        }
    }
}

/* Recompute the avoided set (and its serialised form m_off_buf) from the hit
 * table: the top persist_n() channels that have reached the promotion
 * threshold. Sub-threshold channels stay pickable so they keep being
 * re-tested between timed-quarantine cooldowns — without this gate a channel
 * would be avoided after its very first hit and could never accumulate a
 * second. Partial selection sort — N and the table are small. */
static void persist_rebuild_avoided(void) {
    const uint8_t n_max = persist_n();
    bool used[ARRAY_SIZE(m_hits)] = { false };
    uint8_t out = 0;
    size_t p = 1;

    while (out < n_max) {
        uint8_t best = 0xFF;
        for (uint8_t i = 0; i < m_hit_count; i++) {
            if (used[i] || m_hits[i].hits < CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_QUARANTINE_PERSIST_THRESHOLD || m_hits[i].channel == esb_transport_get_rendezvous_channel()) {
                continue;
            }
            if (best == 0xFF || m_hits[i].hits > m_hits[best].hits) {
                best = i;
            }
        }
        if (best == 0xFF) {
            break;
        }
        used[best] = true;
        m_quarantine.persistent.channels[out] = m_hits[best].channel;
        m_off_buf[p++] = m_hits[best].channel;
        m_off_buf[p++] = (uint8_t)(m_hits[best].hits & 0xFF);
        m_off_buf[p++] = (uint8_t)(m_hits[best].hits >> 8);
        out++;
    }
    m_quarantine.persistent.count = out;
    m_off_buf[0] = out;
    m_off_len = (uint8_t)p;
}

/* Write the full record — accumulator (minutes) + current offender list — to
 * NVS and refresh the in-RAM snapshots used for write suppression. */
static void persist_write(void) {
    uint8_t buf[PERSIST_REC_MAX];
    const uint16_t acc = m_decay_accum_min > UINT16_MAX
                       ? UINT16_MAX : (uint16_t)m_decay_accum_min;
    buf[0] = (uint8_t)(acc & 0xFF);
    buf[1] = (uint8_t)(acc >> 8);
    memcpy(&buf[2], m_off_buf, m_off_len);
    settings_save_one(PERSIST_SETTINGS_KEY, buf, 2 + m_off_len);
    memcpy(m_saved_off, m_off_buf, m_off_len);
    m_saved_off_len = m_off_len;
    m_saved_accum_min = acc;
}

static bool offenders_changed(void) {
    return m_off_len != m_saved_off_len ||
           memcmp(m_off_buf, m_saved_off, m_off_len) != 0;
}

/* Periodic distill: rebuild the avoided set and persist only if the offender
 * list actually changed (the accumulator is flushed separately). */
static void persist_work_fn(struct k_work *w) {
    ARG_UNUSED(w);
    persist_rebuild_avoided();
    if (offenders_changed()) {
        persist_write();
    }
    k_work_reschedule(&persist_work,
                      K_MSEC(CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_QUARANTINE_PERSIST_INTERVAL_MS));
}

/* Age every tracked channel by `periods` hits and drop those that reach zero.
 * A promoted offender is frozen (avoided → never re-picked → never re-hit),
 * so decay is the only thing that erodes it: once it falls back below the
 * threshold the next rebuild removes it from the avoided set and it becomes
 * pickable — and re-testable — again. */
static void persist_decay_apply(const uint32_t periods) {
    uint8_t w = 0;
    for (uint8_t i = 0; i < m_hit_count; i++) {
        if (m_hits[i].hits > periods) {
            m_hits[i].hits = (uint16_t)(m_hits[i].hits - periods);
            m_hits[i].session_hits = m_hits[i].session_hits > periods
                ? (uint16_t)(m_hits[i].session_hits - periods) : 0;
            m_hits[w++] = m_hits[i];
        }
    }
    m_hit_count = w;
}

/* Advance the powered-on-time decay clock and apply any whole DECAY_S periods
 * that have elapsed. Called at boot, on wake, and on every speculative hop.
 * Cheap and write-free unless a decay actually fires or the accumulator has
 * advanced past a coarse flush step since the last NVS write. */
static void persist_decay_check(void) {
    const uint32_t period_min = persist_decay_period_min();
    if (period_min == 0) {
        return;  /* decay disabled */
    }
    const uint32_t now = k_uptime_get_32();
    if (now >= m_decay_last_uptime) {
        m_decay_frac_ms += now - m_decay_last_uptime;
    }
    /* else: uptime reset (reboot) or 32-bit wrap — contribute nothing. */
    m_decay_last_uptime = now;
    if (m_decay_frac_ms >= 60000u) {
        m_decay_accum_min += m_decay_frac_ms / 60000u;
        m_decay_frac_ms %= 60000u;
    }

    bool fired = false;
    if (m_decay_accum_min >= period_min) {
        const uint32_t periods = m_decay_accum_min / period_min;
        m_decay_accum_min -= periods * period_min;
        persist_decay_apply(periods);
        persist_rebuild_avoided();
        fired = true;
    }

    /* Flush the accumulator to NVS at ~1/16 of a period so progress survives
     * a reboot without a write on every hop. A fired decay always writes. */
    const uint32_t flush = MAX(1u, period_min / 16u);
    if (fired || m_decay_accum_min >= (uint32_t)m_saved_accum_min + flush) {
        persist_write();
    }
}

#else
static inline void persist_record_hit(uint8_t channel) { ARG_UNUSED(channel); }
static inline void persist_reclaim(uint8_t channel) { ARG_UNUSED(channel); }
static inline void persist_decay_check(void) {}
#endif /* CHANNEL_QUARANTINE_PERSIST */

#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_RSSI_WEIGHT)
#define RSSI_SETTINGS_KEY "esb_hop/rssi"

static int8_t m_rssi_saved[CHANNEL_HOP_CHANNEL_COUNT];
static struct k_work_delayable rssi_persist_work;

static bool rssi_buf_changed(void) {
    return memcmp(m_channel_rssi.ewma, m_rssi_saved, CHANNEL_HOP_CHANNEL_COUNT) != 0;
}

static void rssi_write(void) {
    settings_save_one(RSSI_SETTINGS_KEY, m_channel_rssi.ewma, CHANNEL_HOP_CHANNEL_COUNT);
    memcpy(m_rssi_saved, m_channel_rssi.ewma, CHANNEL_HOP_CHANNEL_COUNT);
}

static void rssi_persist_work_fn(struct k_work *w) {
    ARG_UNUSED(w);
    if (rssi_buf_changed()) {
        rssi_write();
    }
    k_work_reschedule(&rssi_persist_work,
                      K_MSEC(CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_RSSI_PERSIST_INTERVAL_MS));
}
#endif /* CHANNEL_RSSI_WEIGHT */

#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_QUARANTINE_PERSIST) || IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_RSSI_WEIGHT)
static int persist_settings_load_cb(const char *name, const size_t len,
                                    const settings_read_cb read_cb, void *cb_arg) {
#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_QUARANTINE_PERSIST)
    if (settings_name_steq(name, "quar", NULL)) {
        if (len < 3 || len > PERSIST_REC_MAX) {
            LOG_DBG("unexpected quarantine record length %zu, dropping", len);
            return 0;
        }
        uint8_t buf[PERSIST_REC_MAX];
        if (read_cb(cb_arg, buf, len) < 0) {
            return 0;
        }
        m_decay_accum_min = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);

        const uint8_t *off = &buf[2];
        const size_t off_len = len - 2;
        uint8_t stored = off[0];
        if ((size_t)(1 + stored * 3) > off_len) {
            stored = (uint8_t)((off_len - 1) / 3);
        }
        for (uint8_t i = 0; i < stored; i++) {
            const uint8_t ch = off[1 + i * 3];
            const uint16_t hits = (uint16_t)off[2 + i * 3] | ((uint16_t)off[3 + i * 3] << 8);
            if (ch >= CHANNEL_HOP_CHANNEL_COUNT) {
                continue;
            }
            /* Seed the evidence table so the ranking survives the reboot. The
             * settings subsystem may replay a superseded copy of the record, so
             * merge by channel rather than appending blindly. */
            uint8_t j = 0;
            for (; j < m_hit_count; j++) {
                if (m_hits[j].channel == ch) {
                    break;
                }
            }
            if (j < m_hit_count) {
                if (hits > m_hits[j].hits) {
                    m_hits[j].hits = hits;
                }
            } else if (m_hit_count < ARRAY_SIZE(m_hits)) {
                m_hits[m_hit_count].channel = ch;
                m_hits[m_hit_count].hits = hits ? hits : 1;
                m_hits[m_hit_count].session_hits = 0;
                m_hit_count++;
            }
        }
        return 0;
    }
#endif
#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_RSSI_WEIGHT)
    if (settings_name_steq(name, "rssi", NULL)) {
        if (len != CHANNEL_HOP_CHANNEL_COUNT) {
            LOG_DBG("unexpected rssi record length %zu, dropping", len);
            return 0;
        }
        read_cb(cb_arg, m_channel_rssi.ewma, CHANNEL_HOP_CHANNEL_COUNT);
        return 0;
    }
#endif
    return 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(esb_hop, "esb_hop", NULL, persist_settings_load_cb, NULL, NULL);

static void persist_load(void) {
#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_QUARANTINE_PERSIST)
    m_hit_count = 0;
    m_off_len = 0;
    m_saved_off_len = 0;
    m_decay_accum_min = 0;
    m_decay_frac_ms = 0;
    m_decay_last_uptime = k_uptime_get_32();
    m_saved_accum_min = 0;
#if IS_ENABLED(CONFIG_ZMK_RUNTIME_CONFIG)
    zrc_register(PERSIST_ZRC_KEY, PERSIST_N_MAX, 0, PERSIST_N_MAX);
#endif
#endif
    settings_subsys_init();
    settings_load_subtree("esb_hop");
#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_QUARANTINE_PERSIST)
    /* Build the avoided set from whatever was seeded and snapshot the
     * on-flash state so the first distill / flush doesn't rewrite it. */
    persist_rebuild_avoided();
    memcpy(m_saved_off, m_off_buf, m_off_len);
    m_saved_off_len = m_off_len;
    m_saved_accum_min = m_decay_accum_min > UINT16_MAX
                      ? UINT16_MAX : (uint16_t)m_decay_accum_min;
    k_work_init_delayable(&persist_work, persist_work_fn);
    k_work_reschedule(&persist_work,
                      K_MSEC(CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_QUARANTINE_PERSIST_INTERVAL_MS));
    /* Boot check: a full period may already have elapsed across prior runs. */
    persist_decay_check();
#endif
#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_RSSI_WEIGHT)
    memcpy(m_rssi_saved, m_channel_rssi.ewma, CHANNEL_HOP_CHANNEL_COUNT);
    k_work_init_delayable(&rssi_persist_work, rssi_persist_work_fn);
    k_work_reschedule(&rssi_persist_work,
                      K_MSEC(CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_RSSI_PERSIST_INTERVAL_MS));
#endif
}
#endif /* QUARANTINE_PERSIST || RSSI_WEIGHT */

static uint8_t pick_candidate(void) {
    const uint8_t current = esb_transport_get_channel();
    return channel_hop_pick_weighted(&m_quarantine,
                                     CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_QUARANTINE_MIN_DISTANCE,
                                     current, &m_channel_rssi);
}

/* Reuse the cached pending candidate if it is still a valid target;
 * otherwise pick fresh and cache. Used only by post_hop_burst_work_fn so
 * its consecutive ticks converge on one channel within a single burst
 * window. negotiate_work_fn calls pick_candidate() directly so its own
 * retries diversify across channels — a sticky bad pick there would
 * lock recovery on a dead channel for the lifetime of the negotiation. */
static uint8_t select_pending_candidate(void) {
    const uint8_t current = esb_transport_get_channel();
    if (m_pending_candidate != CHANNEL_HOP_INVALID &&
        m_pending_candidate != current &&
        !quarantine_is(&m_quarantine, m_pending_candidate)) {
        return m_pending_candidate;
    }
    m_pending_candidate = pick_candidate();
    return m_pending_candidate;
}

static void send_proposal(const uint8_t candidate) {
    struct esb_pkt_channel_hop_proposal pkt = {
        .type     = ESB_PKT_CHANNEL_HOP_PROPOSAL,
        .proposed = candidate,
        .current  = esb_transport_get_channel(),
    };
    const int err = esb_transport_send(ESB_PIPE_DATA, (uint8_t *)&pkt, sizeof(pkt));
    if (err) {
        LOG_DBG("proposal send failed: %d", err);
    } else {
        LOG_DBG("proposed channel %u (current=%u)", candidate, pkt.current);
    }
}

static void negotiate_work_fn(struct k_work *w) {
    ARG_UNUSED(w);
    if (!m_link_up) {
        return;
    }
    if (!m_active) {
        /* Don't TX a PROPOSAL during idle — that would flip the dongle's
         * peer_idle to false, rearm its silence watchdog, and defeat the
         * IDLE handshake we just sent. But DO keep ticking so the next
         * active period doesn't have to wait the full INTERVAL_MS for a
         * fresh PROPOSAL. Without this guard, m_active flapping faster
         * than INTERVAL_MS (e.g., user types-pauses-types every few ms
         * apart) cancels-and-reschedules negotiate_work indefinitely
         * without it ever firing — leaving the dongle's committed_next
         * INVALID for arbitrarily long stretches and the silence safety
         * net unable to hop. The cost of background ticking is one work
         * dispatch per INTERVAL_MS during long idles — negligible. */
        k_work_reschedule(&negotiate_work,
                          K_MSEC(CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_HOP_NEGOTIATE_INTERVAL_MS));
        return;
    }

    bool needs_commit = false;

    /* If we have a committed next and it's still a valid non-quarantined
     * channel, keep it. Otherwise pick fresh. */
    if (m_committed_next == CHANNEL_HOP_INVALID ||
        quarantine_is(&m_quarantine, m_committed_next) ||
        m_committed_next == esb_transport_get_channel()) {
        needs_commit = true;
        /* Fresh random candidate per tick — until CONFIRM lands the dongle
         * holds a different value than we hold here, so retries must
         * diversify. Sharing post_hop_burst_work_fn's sticky cache here
         * would let one dead first pick freeze negotiation on a bad
         * channel for the rest of the recovery cycle. */
        m_committed_synced = false;
        const uint8_t candidate = pick_candidate();
        if (candidate != CHANNEL_HOP_INVALID) {
            send_proposal(candidate);
        } else {
            LOG_DBG("no candidate channel available");
        }
    } else if (!m_committed_synced) {
        /* committed_next is valid but the dongle has signalled (REQUEST)
         * or otherwise lost its view since the last CONFIRM — re-affirm
         * with the channel we hold. */
        send_proposal(m_committed_next);
    }
    /* else: both sides know and agree; skip the redundant TX. Silent
     * dongle state loss surfaces via REQUEST in the next ACK. */

    /* Fast retry while we don't yet hold a valid committed_next OR while
     * a REQUEST burst is in flight — we need committed_next in place
     * before the next TX-fail burst, and once a REQUEST has told us the
     * dongle's committed_next is empty, re-affirming aggressively is the
     * only way for our PROPOSAL to land on a weak link. Once CONFIRM
     * arrives we fall back to the slow steady-state interval. */
    bool fast_retry = needs_commit;
    if (m_request_burst_remaining > 0) {
        m_request_burst_remaining--;
        fast_retry = true;
    }
    const int reschedule_ms = fast_retry
        ? CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_HOP_NEGOTIATE_RETRY_MS
        : CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_HOP_NEGOTIATE_INTERVAL_MS;
    k_work_reschedule(&negotiate_work, K_MSEC(reschedule_ms));
}

static void idle_check_work_fn(struct k_work *w) {
    ARG_UNUSED(w);
    /* Scheduled only while active; arrival here means IDLE_THRESHOLD_MS
     * have elapsed with no user TX touching the activity timestamp. */
    if (!m_link_up || !m_active) {
        return;
    }
    enter_idle_state();
}

static void send_idle_packet(void) {
    /* Drop any queued user traffic so the IDLE goes out cleanly without
     * a tail of stale HID frames riding behind it. Use the wrapper, not
     * bare esb_flush_tx — the dropped packets' ring slots and any per-
     * send retransmit_count override they applied still need to be
     * reset, otherwise the IDLE's own TX event pops a stale slot and
     * the radio can stay stuck at an override count for the next
     * sends. */
    esb_transport_flush_tx();

    struct esb_pkt_idle pkt = { .type = ESB_PKT_IDLE };
    const int err = esb_transport_send(ESB_PIPE_DATA, (uint8_t *)&pkt, sizeof(pkt));
    if (err) {
        LOG_DBG("IDLE send failed: %d", err);
    }
}

static void enter_idle_state(void) {
    if (!m_active) {
        return;
    }
    m_active = false;
    k_work_cancel_delayable(&negotiate_work);
    k_work_cancel_delayable(&idle_check_work);
#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_COOP_HOP)
    /* Abort any in-flight cooperative-hop handshake. The link is going
     * quiet — no urgency to hop, and a half-finished OFFER/ACCEPT
     * exchange can leave the dongle armed for a hop the endpoint will
     * never make. The dongle's own disarm rule (see
     * channel_hop_dongle_note_rx_active) catches the case where it had
     * already armed; here we just stop the endpoint side. */
    coop_hop_abort();
#endif
    LOG_DBG("endpoint entering IDLE");

    /* No PROPOSAL on the way out. The original design fired a PROPOSAL
     * here to refresh committed_next "one last time before going quiet",
     * but that introduced two tightly coupled hazards:
     *
     *   1. PROPOSAL flips the dongle's peer_idle to false (every non-
     *      IDLE packet does), arming the silence watchdog AND populating
     *      committed_next with a fresh value. If the IDLE that follows
     *      is then lost, the dongle is left in the worst possible state:
     *      peer_idle=false + freshly-set committed_next = silence_work
     *      fires after RX_SILENCE_MS → speculative hop to that very
     *      target → validate-fail → rollback. A 1-byte packet drop
     *      becomes seconds of cascading desync.
     *
     *   2. PROPOSAL → CONFIRM is a request/response pair. The CONFIRM
     *      rides as the ACK payload to the *next* pipe-1 packet — which
     *      in this context is the IDLE itself. If the keyboard goes
     *      quiet (which is exactly what enter_idle_state means) and the
     *      IDLE is the sole following packet, a lost IDLE also loses
     *      the CONFIRM, leaving the two sides holding different
     *      committed_next values until the next hop or activity cycle.
     *
     * The dongle's committed_next is kept fresh during active use by
     * negotiate_work (every NEGOTIATE_INTERVAL_MS) and during hops by
     * the post-hop PROPOSAL burst — both more reliable contexts than a
     * one-shot transition packet. The triple-IDLE in send_idle_packet
     * additionally guarantees that IF a stray PROPOSAL ever races us
     * into needing CONFIRM here, multiple ACK opportunities follow. */
    send_idle_packet();
}

static void post_hop_burst_work_fn(struct k_work *w) {
    ARG_UNUSED(w);
    if (m_post_hop_burst_remaining == 0) {
        return;
    }
    /* Once the first CONFIRM has populated m_committed_next, re-affirm
     * that channel for the rest of the burst rather than picking a fresh
     * candidate each tick. The state update is asymmetric — dongle commits
     * on every PROPOSAL it receives, endpoint commits only on CONFIRMs
     * that make it back — so a fresh-pick burst lets one dropped CONFIRM
     * leave the two sides holding different "next" channels for the
     * remainder of the burst. A subsequent TX-fail hop then sends the
     * endpoint to OUR last-confirmed value while the dongle's silence
     * watchdog targets ITS later one, splitting the link. Re-affirming
     * keeps both sides anchored to the same channel. While committed_next
     * is still empty (first CONFIRM hasn't arrived yet) we fall back to
     * picking fresh — that's the situation where rebuilding the cache
     * actually matters. */
    const uint8_t current = esb_transport_get_channel();
    uint8_t candidate;
    if (m_committed_next != CHANNEL_HOP_INVALID &&
        m_committed_next != current &&
        !quarantine_is(&m_quarantine, m_committed_next)) {
        /* CONFIRM-derived ground truth — both sides agree on this. */
        candidate = m_committed_next;
    } else {
        /* No CONFIRM yet — go through the shared cache so this tick and
         * any negotiate_work tick that fires alongside us propose the
         * same channel. Quarantine is re-validated inside the helper. */
        candidate = select_pending_candidate();
    }

#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_RENDEZVOUS)
    /* Rendezvous side-trip: every Nth burst tick, send a PROPOSAL on the
     * DTS-default channel pointing the dongle to OUR current channel. The
     * dongle's rollback dwell cycle always includes the default channel,
     * so a desynced dongle that never followed our original hop will catch
     * one of these and learn where to find us — bounded recovery instead
     * of waiting for its rollback to randomly stumble onto our channel.
     * The blocking helper holds off user TX during the trip and restores
     * the active channel before returning; failures here are silent (this
     * is best-effort signalling, not a real send). */
    const uint8_t rdv_every = CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_HOP_POST_BURST_RENDEZVOUS_EVERY;
    const uint8_t rendezvous = esb_transport_get_rendezvous_channel();
    const uint8_t total = CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_HOP_POST_BURST_COUNT;
    const uint8_t tick_index = total - m_post_hop_burst_remaining;
    const bool send_on_rendezvous =
        rdv_every > 0 &&
        tick_index > 0 &&
        (tick_index % rdv_every) == 0 &&
        rendezvous != current;

    if (send_on_rendezvous) {
        struct esb_pkt_channel_hop_proposal pkt = {
            .type     = ESB_PKT_CHANNEL_HOP_PROPOSAL,
            .proposed = current,
            .current  = current,
        };
        const int err = esb_transport_send_blocking(
            rendezvous, ESB_PIPE_DATA, (uint8_t *)&pkt, sizeof(pkt),
            K_MSEC(CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_HOP_RENDEZVOUS_TIMEOUT_MS));
        LOG_DBG("rendezvous PROPOSAL on ch=%u (proposed=%u): %d",
                rendezvous, current, err);
    } else if (candidate != CHANNEL_HOP_INVALID && !m_committed_synced) {
        /* Skip once the first CONFIRM in this burst has landed: both
         * sides hold `candidate` already and the remaining ticks would
         * just re-affirm. The rendezvous side-trip above is unaffected
         * — it targets a desynced dongle on a different channel and is
         * never redundant. */
        send_proposal(candidate);
    }
#else
    if (candidate != CHANNEL_HOP_INVALID && !m_committed_synced) {
        send_proposal(candidate);
    }
#endif /* RENDEZVOUS */

    if (--m_post_hop_burst_remaining > 0) {
        k_work_reschedule(&post_hop_burst_work,
                          K_MSEC(CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_HOP_POST_BURST_INTERVAL_MS));
    } else {
        /* Burst done — clear the sticky candidate so the next burst
         * picks fresh (the link state may have changed since this burst
         * started). */
        m_pending_candidate = CHANNEL_HOP_INVALID;
        /* If the device went idle during the burst, the trailing
         * PROPOSALs we just sent flipped the dongle back to
         * peer_idle=false (every non-IDLE packet does), re-arming its
         * silence watchdog. enter_idle_state's IDLE burst landed
         * earlier in the sequence and was overwritten by these
         * PROPOSALs. Re-send IDLE now so peer_idle=true is the last
         * state the dongle sees on this channel — otherwise
         * silence_work fires after RX_SILENCE_MS and triggers a
         * speculative hop into a stale committed_next. */
        if (!m_active) {
            send_idle_packet();
        }
    }
}

static void start_post_hop_burst(void) {
    if (CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_HOP_POST_BURST_COUNT == 0) {
        return;
    }
    m_post_hop_burst_remaining = CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_HOP_POST_BURST_COUNT;
    /* Fresh burst — clear any leftover sticky candidate from a prior
     * burst that ended early or was preempted. */
    m_pending_candidate = CHANNEL_HOP_INVALID;
    /* First PROPOSAL lands after the transport's quiet window expires —
     * esb_transport_send silently drops anything sent before then. */
    k_work_reschedule(&post_hop_burst_work,
                      K_MSEC(CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_HOP_POST_QUIET_MS));
}

static void enter_active_state(const bool send_initial_proposal) {
    if (m_active) {
        /* Already active — just refresh the idle deadline. */
        k_work_reschedule(&idle_check_work,
                          K_MSEC(CONFIG_ZMK_ESB_ENDPOINT_IDLE_THRESHOLD_MS));
        return;
    }
    m_active = true;
    LOG_DBG("endpoint entering ACTIVE");
    persist_decay_check();
    k_work_reschedule(&idle_check_work,
                      K_MSEC(CONFIG_ZMK_ESB_ENDPOINT_IDLE_THRESHOLD_MS));
    if (send_initial_proposal) {
        k_work_reschedule(&negotiate_work, K_NO_WAIT);
    } else {
        /* Idle→active transition: fire negotiate at fast-retry tempo
         * (NEGOTIATE_RETRY_MS) instead of the slow INTERVAL. The dongle's
         * committed_next may have been wiped during the idle period —
         * a successful speculative-hop validation on the dongle while
         * we were silent clears its target and there's no inbound
         * PROPOSAL between then and now to refresh it. Bounding the
         * silence-watchdog blackout window to one fast-retry tick after
         * resume costs one extra PROPOSAL per active-resume — cheap
         * insurance against multi-second hops-to-nowhere on the dongle. */
        k_work_reschedule(&negotiate_work,
                          K_MSEC(CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_HOP_NEGOTIATE_RETRY_MS));
    }
}

static void hop_retry_work_fn(struct k_work *w) {
    ARG_UNUSED(w);
    /* Re-submit the hop attempt. The ESB state has had a chance to drain
     * back to IDLE or PTX_TXIDLE; set_rf_channel may now succeed. */
    k_work_submit(&hop_work);
}

/* Post-hop bookkeeping shared between the speculative-fail-driven hop
 * (hop_work_fn) and the cooperative-hop commit work. Called AFTER a
 * successful esb_transport_set_channel(target). Handles:
 *   - quarantine of the channel we just left (caller picks duration —
 *     coop hops use a much shorter hold than fail-driven hops since
 *     the trigger fires on transient degradation, not a confirmed-bad
 *     channel)
 *   - clearing committed_next (the agreed channel was just consumed)
 *   - arming the cooldown window so TX-fail triggers don't bounce off
 *     the new channel before the dongle arrives
 *   - stamping the link-event clock for the rendezvous-fallback timer
 *   - kicking off the post-hop PROPOSAL burst so the dongle can
 *     validate against real traffic
 *   - rescheduling negotiate_work at the fast-retry tempo so a fresh
 *     committed_next is built quickly
 *
 * `prev` is the channel we just left; the caller has already verified
 * the hop committed (set_channel returned 0). `quarantine_ms` is the
 * hold time to apply to `prev`.
 *
 * The "is_revert" / "is_rendezvous_fallback" branches in hop_work_fn
 * mutate state BEFORE this is called (they have additional special
 * cases — clearing quarantine on the target, etc.); this helper only
 * implements the common tail. Coop hop has no revert / rendezvous
 * concept, so it calls this directly. */
static void commit_hop_bookkeeping(const uint8_t prev, const uint32_t quarantine_ms) {
    /* Never quarantine the rendezvous channel — it must stay a usable
     * fallback for re-syncing with a desynced dongle. */
    if (prev != esb_transport_get_rendezvous_channel()) {
        quarantine_add(&m_quarantine, prev, quarantine_ms);
        m_prev_known_channel = prev;
    }

    m_committed_next = CHANNEL_HOP_INVALID;
    m_committed_synced = false;

    m_hop_cooldown_until = k_uptime_get_32() +
        CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_HOP_COOLDOWN_MS;

    {
        const uint32_t now = k_uptime_get_32();
        m_last_link_event_ms = (now == 0) ? 1 : now;
    }

    start_post_hop_burst();

    k_work_reschedule(&negotiate_work,
                      K_MSEC(CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_HOP_NEGOTIATE_RETRY_MS));
}

static void hop_work_fn(struct k_work *w) {
    ARG_UNUSED(w);
    persist_decay_check();
    const uint8_t current = esb_transport_get_channel();

    /* Snapshot once — TX_SUCCESS ISR can clear m_prev_channel while we
     * run, and we want a consistent decision below. */
    const uint8_t prev = m_prev_channel;
    const bool is_revert = (prev != CHANNEL_HOP_INVALID) && (prev != current);

    /* Cooldown rendezvous fallback (ISR set the flag): force the rendezvous
     * channel as the target, bypassing the normal candidate-selection path
     * and ignoring quarantine. The dongle's rollback dwell always includes
     * this channel, so it is the surest place to re-acquire after a hop
     * desync. We also clear quarantine on the rendezvous channel below so
     * subsequent picks may use it freely. */
    bool is_rendezvous_fallback = false;
#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_RENDEZVOUS)
    if (m_rendezvous_fallback_pending) {
        m_rendezvous_fallback_pending = false;
        is_rendezvous_fallback = true;
    }
#endif

    /* Decide the target WITHOUT mutating any persistent state. If the
     * radio hop fails, we must leave quarantine, m_prev_channel, and
     * m_committed_next untouched so the retry (or a later re-trigger
     * from the TX-fail ISR) sees the same decision inputs. */
    uint8_t target;
    if (is_rendezvous_fallback) {
        target = esb_transport_get_rendezvous_channel();
        if (target == current) {
            /* Already on the rendezvous channel — nothing to do; user TX
             * is failing for a non-desync reason (range, interference). */
            return;
        }
    } else if (is_revert) {
        /* The previous hop did not produce a single TX_SUCCESS on the new
         * channel before we hit the fail threshold again — the "new"
         * channel is no better than the one we left. Bounce back rather
         * than keep rolling the dice. */
        target = prev;
    } else {
        target = m_committed_next;
        if (target == CHANNEL_HOP_INVALID || target == current ||
            quarantine_is(&m_quarantine, target)) {
            /* No valid pre-negotiated channel (CONFIRM never arrived,
             * committed_next matched our current channel, or it has since
             * been quarantined — timed expiry or a persistent-offender
             * rebuild can condemn it after we committed). Don't
             * scatter to a random channel a desynced dongle can't predict —
             * converge on the rendezvous (default) channel, which the
             * dongle's rollback dwell always visits. If already on it, bounce
             * to the last non-default channel we held so the pair oscillates
             * between two predictable points instead of one dead rendezvous;
             * fall back to a fresh pick only when neither is available. */
            const uint8_t rendezvous = esb_transport_get_rendezvous_channel();
            if (current != rendezvous) {
                target = rendezvous;
            } else if (m_prev_known_channel != CHANNEL_HOP_INVALID &&
                       m_prev_known_channel != current) {
                target = m_prev_known_channel;
            } else {
                target = channel_hop_pick_weighted(
                    &m_quarantine,
                    CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_QUARANTINE_MIN_DISTANCE,
                    current, &m_channel_rssi);
                if (target == CHANNEL_HOP_INVALID) {
                    LOG_DBG("no candidate channel available; staying on %u", current);
                    /* Clear the fail counter so the modulo trigger in the TX
                     * ISR will rearm after the next THRESHOLD failures. */
                    esb_transport_reset_consecutive_fail();
                    return;
                }
            }
            LOG_DBG("no committed next; hop %u -> %u", current, target);
        }
    }

    /* Try the radio hop FIRST. All persistent state mutations are below,
     * guarded by success. */
    const int err = esb_transport_set_channel(target);
    if (err) {
        LOG_ERR("channel hop %u -> %u failed: %d (will retry in %ums)",
                current, target, err, (unsigned)CONFIG_ZMK_ESB_ENDPOINT_HOP_RETRY_DELAY_MS);
        /* -EBUSY here means the PTX is in PTX_TX/PTX_TX_ACK/PTX_TXIDLE and
         * we lack fast-channel-switching. Leave committed_next, m_prev,
         * and quarantine intact so the retry can make the same decision.
         * Reset the fail counter so the TX ISR's modulo trigger re-arms
         * and a new burst of fails still retriggers us. */
        esb_transport_reset_consecutive_fail();
        k_work_reschedule(&hop_retry_work, K_MSEC(CONFIG_ZMK_ESB_ENDPOINT_HOP_RETRY_DELAY_MS));
        return;
    }

    /* Hop succeeded. NOW commit state transitions. */
    if (is_rendezvous_fallback) {
        /* Rendezvous fallback: do NOT quarantine the channel we left —
         * it might be perfectly fine, we just lost sync with the dongle.
         * Clear quarantine on the rendezvous target in case a prior hop
         * had banished it. No revert arming: from here we want the dongle
         * to find us and resume normal negotiation. Also bypass the
         * shared bookkeeping's quarantine_add of `current`. */
        m_quarantine.expires_at[target] = 0;
        persist_reclaim(target);
        m_prev_channel = CHANNEL_HOP_INVALID;
        LOG_DBG("rendezvous fallback: %u -> %u (cooldown desync recovery)",
                current, target);

        m_committed_next = CHANNEL_HOP_INVALID;
        m_committed_synced = false;
        m_hop_cooldown_until = k_uptime_get_32() +
            CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_HOP_COOLDOWN_MS;
        {
            const uint32_t now = k_uptime_get_32();
            m_last_link_event_ms = (now == 0) ? 1 : now;
        }
        start_post_hop_burst();
#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_RENDEZVOUS)
        /* Park here: suppress hops until a TX_SUCCESS re-acquires the link
         * or the hold expires, so the dongle's rollback dwell has a
         * stationary target instead of a moving one. */
        if (CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_HOP_RENDEZVOUS_HOLD_MS > 0) {
            const uint32_t hold = k_uptime_get_32() +
                CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_HOP_RENDEZVOUS_HOLD_MS;
            m_rendezvous_hold_until = (hold == 0) ? 1 : hold;
        }
#endif
        k_work_reschedule(&negotiate_work,
                          K_MSEC(CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_HOP_NEGOTIATE_RETRY_MS));
        return;
    }

    if (is_revert) {
        /* The prev channel was quarantined by the previous hop; clear that
         * so we are allowed to occupy it again — both the timed quarantine
         * and any persistent-offender record. */
        m_quarantine.expires_at[target] = 0;
        persist_reclaim(target);
        LOG_DBG("no TX success since last hop; reverted %u -> %u", current, target);
    }

    /* Arm a future revert to `current` only for normal hops. A revert is
     * one-shot: if this revert also fails, the next hop must go pick a
     * fresh candidate, not bounce again. */
    m_prev_channel = is_revert ? CHANNEL_HOP_INVALID : current;

    LOG_DBG("channel hop: %u -> %u (quarantined %u for %ums)",
            current, target, current,
            (unsigned)CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_QUARANTINE_MS);

    commit_hop_bookkeeping(current, CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_QUARANTINE_MS);

    /* A fail-driven hop leaves behind a channel that just proved bad —
     * register a hit against it. Coop hops fire on transient degradation
     * rather than a confirmed-bad channel and deliberately do not. */
    persist_record_hit(current);
}

/* ----- Cooperative-hop handshake ----- */

#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_COOP_HOP)
static bool coop_hop_cooldown_active(void) {
    if (m_coop_hop_cooldown_until == 0) {
        return false;
    }
    return (int32_t)(m_coop_hop_cooldown_until - k_uptime_get_32()) > 0;
}

static uint8_t coop_hop_pick_target(void) {
    const uint8_t current = esb_transport_get_channel();

    /* Prefer the pre-negotiated channel if it is still useful — that's
     * the "may use pre-committed" path the design calls for. Falls back
     * to a fresh pick otherwise. */
    if (m_committed_next != CHANNEL_HOP_INVALID &&
        m_committed_next != current &&
        !quarantine_is(&m_quarantine, m_committed_next)) {
        return m_committed_next;
    }
    return channel_hop_pick_weighted(&m_quarantine,
                                     CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_QUARANTINE_MIN_DISTANCE,
                                     current, &m_channel_rssi);
}

static void coop_hop_offer_work_fn(struct k_work *w) {
    ARG_UNUSED(w);
    if (!m_link_up || !m_active) {
        m_coop_hop_state = COOP_IDLE;
        return;
    }
    if (m_coop_hop_state != COOP_OFFERED) {
        /* State changed underneath us (e.g. disconnect, idle) before
         * the workqueue ran. Don't TX a stale OFFER. */
        return;
    }

    const uint8_t target = coop_hop_pick_target();
    if (target == CHANNEL_HOP_INVALID || target == esb_transport_get_channel()) {
        LOG_DBG("coop hop: no candidate channel; aborting");
        m_coop_hop_state = COOP_IDLE;
        return;
    }
    m_coop_hop_target = target;
    m_coop_hop_seq++;

    struct esb_pkt_hop_offer pkt = {
        .type           = ESB_PKT_HOP_OFFER,
        .target_channel = target,
        .hop_in_ms      = (uint8_t)CONFIG_ZMK_ESB_ENDPOINT_COOP_HOP_HOP_IN_MS,
        .seq            = m_coop_hop_seq,
    };

    /* Set the offer-in-flight tag immediately before the radio fires.
     * The TX_SUCCESS handler keys off this to know that the just-
     * acknowledged ACK payload (if any) was a HOP_ACCEPT and that
     * arming the commit timer is the right action. */
    m_last_tx_was_offer = true;
    const int err = esb_transport_send(ESB_PIPE_DATA, (uint8_t *)&pkt, sizeof(pkt));
    if (err) {
        LOG_DBG("coop OFFER send failed: %d", err);
        m_last_tx_was_offer = false;
        m_coop_hop_state = COOP_IDLE;
        return;
    }
    LOG_DBG("coop hop OFFER: target=%u hop_in=%ums seq=%u",
            target, pkt.hop_in_ms, m_coop_hop_seq);
}

static void coop_hop_commit_work_fn(struct k_work *w) {
    ARG_UNUSED(w);
    if (m_coop_hop_state != COOP_ARMED) {
        /* Cancelled (disconnect, idle, abort) between scheduling and
         * firing. Don't hop. */
        return;
    }
    const uint8_t current = esb_transport_get_channel();
    const uint8_t target = m_coop_hop_target;
    if (target == CHANNEL_HOP_INVALID || target == current) {
        m_coop_hop_state = COOP_IDLE;
        return;
    }

    const int err = esb_transport_set_channel(target);
    if (err) {
        LOG_DBG("coop hop %u -> %u set_channel failed: %d (will retry in %d ms)", current, target, err, CONFIG_ZMK_ESB_ENDPOINT_HOP_RETRY_DELAY_MS);
        m_coop_hop_state = COOP_IDLE;
        /* Reuse the existing retry path — the TX-fail trigger will
         * re-fire if the link is genuinely bad. */
        k_work_reschedule(&hop_retry_work, K_MSEC(CONFIG_ZMK_ESB_ENDPOINT_HOP_RETRY_DELAY_MS));
        return;
    }

    LOG_DBG("coop hop committed: %u -> %u (quarantined %u for %ums)",
            current, target, current,
            (unsigned)CONFIG_ZMK_ESB_ENDPOINT_COOP_HOP_QUARANTINE_MS);
    m_prev_channel = CHANNEL_HOP_INVALID;
    commit_hop_bookkeeping(current,
                           CONFIG_ZMK_ESB_ENDPOINT_COOP_HOP_QUARANTINE_MS);

    /* Independent cooldown: prevents back-to-back coop hops while the
     * link settles. The TX-fail-hop cooldown set inside
     * commit_hop_bookkeeping handles the speculative path; this one
     * gates the link-degraded ISR's own re-arm. */
    m_coop_hop_cooldown_until = k_uptime_get_32() +
        CONFIG_ZMK_ESB_ENDPOINT_COOP_HOP_COOLDOWN_MS;
    m_coop_hop_state = COOP_IDLE;
}

static void coop_hop_abort(void) {
    /* Single point of teardown — used by disconnect, idle entry, and
     * other state transitions that invalidate an in-flight handshake. */
    k_work_cancel_delayable(&coop_hop_offer_work);
    k_work_cancel_delayable(&coop_hop_commit_work);
    m_coop_hop_state = COOP_IDLE;
    m_last_tx_was_offer = false;
}

void channel_hop_ep_on_link_degraded_isr(void) {
    /* Single producer (RADIO ISR), single consumer (workqueue). The
     * gates here are racy reads of state vars but the work fn re-checks
     * before sending — extra fires only cost a workqueue dispatch. */
    if (!m_link_up || !m_active) {
        return;
    }
    if (m_coop_hop_state != COOP_IDLE) {
        return;
    }
    if (hop_cooldown_active() || coop_hop_cooldown_active()) {
        return;
    }
    /* Latch state before submitting so a second ISR call can't double-
     * submit the offer work. The actual send happens in the work fn. */
    m_coop_hop_state = COOP_OFFERED;
    k_work_reschedule(&coop_hop_offer_work, K_NO_WAIT);
}
#endif /* COOP_HOP */

void channel_hop_ep_init(void) {
    quarantine_reset(&m_quarantine);
    channel_rssi_reset(&m_channel_rssi);
#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_QUARANTINE_PERSIST) || IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_RSSI_WEIGHT)
    persist_load();
#endif
    m_committed_next = CHANNEL_HOP_INVALID;
    m_committed_synced = false;
    m_prev_channel = CHANNEL_HOP_INVALID;
    m_prev_known_channel = CHANNEL_HOP_INVALID;
    m_active = false;
    m_post_hop_burst_remaining = 0;
    m_hop_cooldown_until = 0;
    m_last_link_event_ms = 0;
#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_RENDEZVOUS)
    m_rendezvous_fallback_pending = false;
    m_rendezvous_hold_until = 0;
#endif
    m_request_burst_remaining = 0;
#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_COOP_HOP)
    m_coop_hop_state = COOP_IDLE;
    m_coop_hop_target = CHANNEL_HOP_INVALID;
    m_coop_hop_seq = 0;
    m_last_tx_was_offer = false;
    m_coop_hop_cooldown_until = 0;
#endif
    k_work_init_delayable(&negotiate_work, negotiate_work_fn);
    k_work_init_delayable(&idle_check_work, idle_check_work_fn);
    k_work_init_delayable(&post_hop_burst_work, post_hop_burst_work_fn);
    k_work_init_delayable(&hop_retry_work, hop_retry_work_fn);
#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_COOP_HOP)
    k_work_init_delayable(&coop_hop_offer_work, coop_hop_offer_work_fn);
    k_work_init_delayable(&coop_hop_commit_work, coop_hop_commit_work_fn);
#endif
    k_work_init(&hop_work, hop_work_fn);
}

void channel_hop_ep_on_connected(void) {
    m_link_up = true;
    /* The rendezvous channel is only known once addresses are configured,
     * after persist_load's rebuild ran — drop it from the evidence table
     * and avoided set in case a stale NVS record promoted it. */
    persist_reclaim(esb_transport_get_rendezvous_channel());
    /* Start ACTIVE so the first PROPOSAL goes out immediately and we have
     * a committed_next available the first time TX failures hit the hop
     * threshold. idle_check_work will demote us if no HID follows. */
    enter_active_state(true);
}

void channel_hop_ep_on_disconnected(void) {
    m_link_up = false;
    m_active = false;
    m_committed_next = CHANNEL_HOP_INVALID;
    m_committed_synced = false;
    m_prev_channel = CHANNEL_HOP_INVALID;
    m_prev_known_channel = CHANNEL_HOP_INVALID;
    m_post_hop_burst_remaining = 0;
    m_hop_cooldown_until = 0;
    m_last_link_event_ms = 0;
#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_RENDEZVOUS)
    m_rendezvous_fallback_pending = false;
    m_rendezvous_hold_until = 0;
#endif
    m_request_burst_remaining = 0;
#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_COOP_HOP)
    m_coop_hop_cooldown_until = 0;
#endif
    k_work_cancel_delayable(&negotiate_work);
    k_work_cancel_delayable(&idle_check_work);
    k_work_cancel_delayable(&post_hop_burst_work);
    k_work_cancel_delayable(&hop_retry_work);
#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_COOP_HOP)
    coop_hop_abort();
#endif
}

void channel_hop_ep_note_user_tx(void) {
    if (!m_link_up) {
        return;
    }
    enter_active_state(false);
}

void channel_hop_ep_on_rx(const uint8_t *data, const uint8_t len) {
    if (len < 1) {
        return;
    }

#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_COOP_HOP)
    /* Cooperative-hop ACCEPT path. Updates the agreed target if the
     * dongle counter-proposed; the actual commit is armed by the
     * TX_SUCCESS hook on the OFFER (not here) so a stray ACCEPT for an
     * older OFFER cannot fire a hop. */
    if (data[0] == ESB_PKT_HOP_ACCEPT) {
        if (len < sizeof(struct esb_pkt_hop_accept)) {
            return;
        }
        const struct esb_pkt_hop_accept *a = (const void *)data;
        if (a->seq != m_coop_hop_seq) {
            LOG_DBG("coop ACCEPT: stale seq %u (have %u), ignoring",
                    a->seq, m_coop_hop_seq);
            return;
        }
        if (m_coop_hop_state != COOP_OFFERED && m_coop_hop_state != COOP_ARMED) {
            LOG_DBG("coop ACCEPT for state=%u, ignoring", m_coop_hop_state);
            return;
        }
        if (a->agreed_channel >= CHANNEL_HOP_CHANNEL_COUNT) {
            return;
        }
        if (a->agreed_channel != m_coop_hop_target) {
            LOG_DBG("coop ACCEPT counter-proposed: %u -> %u",
                    m_coop_hop_target, a->agreed_channel);
            m_coop_hop_target = a->agreed_channel;
        }
        return;
    }
#endif /* COOP_HOP */

    if (len < sizeof(struct esb_pkt_channel_hop_confirm)) {
        return;
    }
    if (data[0] != ESB_PKT_CHANNEL_HOP_CONFIRM) {
        return;
    }
    const struct esb_pkt_channel_hop_confirm *c = (const void *)data;
    if (c->agreed >= CHANNEL_HOP_CHANNEL_COUNT) {
        return;
    }
    /* Reject a dongle-agreed channel we quarantine. Committing it would let
     * the next TX-fail hop jump straight onto a known-bad channel (the hop
     * target is taken from m_committed_next unconditionally). Leave
     * committed_next untouched and re-seed a PROPOSAL burst so the dongle is
     * pushed toward a channel we both accept; our PROPOSALs only ever carry
     * non-quarantined picks, so a counter-proposal cannot recur on it. */
    if (quarantine_is(&m_quarantine, c->agreed)) {
        LOG_DBG("rejecting agreed channel %u: quarantined", c->agreed);
        m_committed_synced = false;
        if (m_link_up && m_active) {
            m_request_burst_remaining =
                CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_HOP_REQUEST_BURST_COUNT;
            k_work_reschedule(&negotiate_work, K_NO_WAIT);
        }
        return;
    }
    const bool was_missing = (m_committed_next == CHANNEL_HOP_INVALID);
    m_committed_next = c->agreed;
    /* Both sides now hold the same agreed channel — suppress the
     * periodic re-affirm and the post-hop burst tail until something
     * surfaces that could desync us again. */
    m_committed_synced = true;
    /* CONFIRM means the dongle has committed_next populated again, so
     * any in-flight REQUEST burst is no longer needed — drop the
     * counter so we don't keep hammering at fast retry while the link
     * is healthy. */
    m_request_burst_remaining = 0;
    LOG_DBG("committed next channel = %u (%s)",
            c->agreed, c->accepted ? "accepted" : "counter-proposed");
    /* Now that we hold a valid committed_next, drop back to the slow
     * steady-state cadence instead of the fast retry tempo. */
    if (was_missing && m_link_up && m_active) {
        k_work_reschedule(&negotiate_work,
                          K_MSEC(CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_HOP_NEGOTIATE_INTERVAL_MS));
    }
}

void channel_hop_ep_on_request(void) {
    if (!m_link_up) {
        return;
    }
    /* Don't gate on m_active. REQUEST arrives as the ACK payload to one
     * of our own pipe-1 TXs, which by definition means the endpoint just
     * transmitted user traffic — note_user_tx already flipped us active
     * before the radio fired. Even in the unlikely race where the flip
     * hasn't propagated yet, scheduling negotiate_work is idempotent and
     * its own !m_active guard reschedules instead of sending, so a
     * spurious request costs nothing.
     *
     * Seed a small burst of fast-retry attempts. A single PROPOSAL on a
     * weak link is too easy to lose, and the steady-state cadence
     * (NEGOTIATE_INTERVAL_MS = 60 s) is far too slow to recover. The
     * burst counter is consumed inside negotiate_work_fn so the
     * resulting reschedules use NEGOTIATE_RETRY_MS regardless of whether
     * our own committed_next happens to be valid. Re-seed (don't add)
     * on every REQUEST so the most recent dongle state always wins; a
     * back-to-back REQUEST stream from a busy weak link is just one
     * burst, not stacked bursts. */
    m_request_burst_remaining =
        CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_HOP_REQUEST_BURST_COUNT;
    /* Dongle just told us its committed_next is INVALID — even if our
     * own m_committed_next is still valid, the two views no longer
     * match. Force the next negotiate_work_fn / post_hop_burst_work_fn
     * to actually transmit the PROPOSAL instead of skipping it. */
    m_committed_synced = false;
    k_work_reschedule(&negotiate_work, K_NO_WAIT);
    LOG_DBG("REQUEST received — burst of %u PROPOSAL attempts",
            m_request_burst_remaining);

    /* Respond by re-proposing only — never by hopping. A REQUEST means the
     * dongle lost its pre-negotiated next channel (typically its own
     * speculative-hop validation reverted and cleared committed_next), not
     * that the current channel is bad. Triggering a hop here turns routine
     * dongle-side desync into a self-sustaining storm: the hop clears
     * committed_next on both ends, the dongle re-REQUESTs on its next ACK,
     * and the link never settles even while it is perfectly healthy.
     * Genuine bad-channel recovery stays owned by the TX-fail and weak-link
     * triggers, which fire on actual transmit failures. */
}

void channel_hop_ep_on_tx_fail_isr(void) {
#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_COOP_HOP)
    /* If the just-failed packet was a HOP_OFFER, the cooperative-hop
     * handshake is dead. Tear down state — note this fires from the
     * RADIO ISR, so we cannot cancel the work item here (k_work_cancel
     * is workqueue-thread context). The commit work guards on state ==
     * COOP_ARMED, so clearing state to IDLE is sufficient: any pending
     * commit_work that fires later will see IDLE and no-op. The dongle
     * may still be armed (PRX heard OFFER but our PTX missed the ACK);
     * the dongle's note_rx_active disarm rule handles that case. */
    if (m_last_tx_was_offer) {
        m_last_tx_was_offer = false;
        if (m_coop_hop_state == COOP_OFFERED) {
            m_coop_hop_state = COOP_IDLE;
            LOG_DBG("coop OFFER TX_FAILED, aborting handshake");
            /* Suppress the normal TX-fail trigger for THIS one event:
             * a single OFFER loss should not double-fire as a
             * speculative hop. The next TX_FAILED in the streak will
             * still be counted by the existing window/weak-link
             * machinery — no permanent suppression. */
            return;
        }
    }
#endif

    /* Only hop while we're actively sending user traffic. During idle
     * the only packets on the wire are the single ESB_PKT_IDLE we fired
     * at transition; if that one fails we do NOT want to hop — nothing
     * urgent is pending. */
    if (!m_active) {
        return;
    }
#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_RENDEZVOUS)
    /* Parked on the rendezvous channel after a fallback: hold here with all
     * hops suppressed so the dongle's rollback dwell can re-acquire a
     * stationary target. Cleared by TX_SUCCESS; otherwise expires so a dead
     * rendezvous channel is not a permanent trap. */
    if (m_rendezvous_hold_until != 0) {
        if ((int32_t)(m_rendezvous_hold_until - k_uptime_get_32()) > 0) {
            return;
        }
        m_rendezvous_hold_until = 0;
    }
#endif
    /* Post-hop cooldown: after a recent hop, give the dongle time to fire
     * its own silence watchdog (RX_SILENCE_MS) and validate the speculative
     * hop (VALIDATE_MS) before we consider hopping again. Without this a
     * short burst of TX fails on a dead link bounces us off the new channel
     * far before the dongle's silence watchdog ever arrives there, so the
     * speculative hop always validates against an empty channel. The
     * post-hop PROPOSAL burst continues firing during cooldown regardless
     * — that's precisely what gives the dongle a packet to confirm against. */
    if (hop_cooldown_active()) {
#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_RENDEZVOUS)
        /* Cooldown rendezvous fallback: if no positive link event (hop
         * commit or TX_SUCCESS) has occurred for RENDEZVOUS_FALLBACK_MS,
         * the channel is effectively dead. Force a hop to the rendezvous
         * channel — the one channel the dongle's rollback dwell is
         * guaranteed to visit — so we can re-acquire instead of running
         * out the cooldown clock on a dead link. TX_SUCCESS refreshes
         * m_last_link_event_ms so a healthy link re-arms the silence
         * clock rather than permanently disabling fallback. */
        if (CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_HOP_RENDEZVOUS_FALLBACK_MS > 0 &&
            m_last_link_event_ms != 0 &&
            !m_rendezvous_fallback_pending) {
            const uint32_t elapsed =
                k_uptime_get_32() - m_last_link_event_ms;
            if (elapsed >= CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_HOP_RENDEZVOUS_FALLBACK_MS) {
                m_rendezvous_fallback_pending = true;
                k_work_submit(&hop_work);
            }
        }
#endif
        return;
    }
    k_work_submit(&hop_work);
}

void channel_hop_ep_on_tx_success_isr(void) {
#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_COOP_HOP)
    /* If the just-acknowledged packet was a HOP_OFFER, the dongle's
     * ACCEPT rode in the ACK payload (already consumed by the RX path
     * which updated m_coop_hop_target if it counter-proposed). Arm the
     * commit timer for hop_in_ms from now — both sides flip after the
     * same delay relative to OFFER delivery. */
    if (m_last_tx_was_offer) {
        m_last_tx_was_offer = false;
        if (m_coop_hop_state == COOP_OFFERED) {
            m_coop_hop_state = COOP_ARMED;
            k_work_reschedule(&coop_hop_commit_work,
                              K_MSEC(CONFIG_ZMK_ESB_ENDPOINT_COOP_HOP_HOP_IN_MS));
        }
    }
#endif

    /* A packet made it through on the current channel — any pending
     * "revert to the previous channel" arming is no longer appropriate. */
    m_prev_channel = CHANNEL_HOP_INVALID;
#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_RENDEZVOUS)
    /* Link re-acquired — end any rendezvous hold so normal hopping resumes. */
    m_rendezvous_hold_until = 0;
#endif
    /* Refresh the silence clock rather than clearing it. If the link goes
     * dead again inside the same cooldown window, the fallback must still
     * be able to fire after RENDEZVOUS_FALLBACK_MS of fresh silence. */
    const uint32_t now = k_uptime_get_32();
    m_last_link_event_ms = (now == 0) ? 1 : now;
}

bool channel_hop_ep_is_active(void) {
    return m_active;
}

uint8_t channel_hop_ep_get_committed(void) {
    return m_committed_next;
}

uint8_t channel_hop_ep_get_quarantine_count(void) {
    return quarantine_count(&m_quarantine);
}

bool channel_hop_ep_is_quarantined(const uint8_t channel) {
    return quarantine_is(&m_quarantine, channel);
}

uint8_t channel_hop_ep_get_offenders(struct channel_hop_ep_offender *out, const uint8_t max) {
#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_QUARANTINE_PERSIST)
    if (out == NULL || max == 0) {
        return 0;
    }
    bool used[ARRAY_SIZE(m_hits)] = { false };
    uint8_t out_n = 0;
    while (out_n < max) {
        uint8_t best = 0xFF;
        for (uint8_t i = 0; i < m_hit_count; i++) {
            if (used[i]) {
                continue;
            }
            if (best == 0xFF || m_hits[i].hits > m_hits[best].hits) {
                best = i;
            }
        }
        if (best == 0xFF) {
            break;
        }
        used[best] = true;
        out[out_n].channel = m_hits[best].channel;
        out[out_n].hits = m_hits[best].hits;
        out[out_n].session_hits = m_hits[best].session_hits;
        out[out_n].persistent =
            persistent_quarantine_contains(&m_quarantine.persistent, m_hits[best].channel);
        out_n++;
    }
    return out_n;
#else
    ARG_UNUSED(out);
    ARG_UNUSED(max);
    return 0;
#endif
}

#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_RSSI_WEIGHT)
void channel_hop_ep_note_peer_rssi(const int8_t ewma) {
    /* The dongle reports RSSI as a positive magnitude (|dBm|); store the true
     * negative dBm so stronger maps to larger and the +127 no-sample sentinel
     * cannot collide with a real reading. */
    channel_rssi_update(&m_channel_rssi, esb_transport_get_channel(), (int8_t)-ewma);
}

uint8_t channel_hop_ep_get_rssi(struct channel_hop_ep_rssi *out, const uint8_t max) {
    if (out == NULL || max == 0) {
        return 0;
    }
    uint8_t n = 0;
    for (uint8_t ch = 0; ch < CHANNEL_HOP_CHANNEL_COUNT && n < max; ch++) {
        int8_t v;
        if (channel_rssi_get(&m_channel_rssi, ch, &v)) {
            out[n].channel = ch;
            out[n].ewma = v;
            n++;
        }
    }
    return n;
}
#else
void channel_hop_ep_note_peer_rssi(int8_t ewma) { ARG_UNUSED(ewma); }
uint8_t channel_hop_ep_get_rssi(struct channel_hop_ep_rssi *out, uint8_t max) {
    ARG_UNUSED(out); ARG_UNUSED(max); return 0;
}
#endif

#else /* !CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_HOP */

void channel_hop_ep_init(void) {}
void channel_hop_ep_on_connected(void) {}
void channel_hop_ep_on_disconnected(void) {}
void channel_hop_ep_note_user_tx(void) {}
void channel_hop_ep_on_rx(const uint8_t *data, uint8_t len) { ARG_UNUSED(data); ARG_UNUSED(len); }
void channel_hop_ep_on_request(void) {}
void channel_hop_ep_on_tx_fail_isr(void) {}
void channel_hop_ep_on_tx_success_isr(void) {}
uint8_t channel_hop_ep_get_committed(void) { return CHANNEL_HOP_INVALID; }
uint8_t channel_hop_ep_get_quarantine_count(void) { return 0; }
bool channel_hop_ep_is_quarantined(uint8_t channel) { ARG_UNUSED(channel); return false; }
bool channel_hop_ep_is_active(void) { return false; }
uint8_t channel_hop_ep_get_offenders(struct channel_hop_ep_offender *out, uint8_t max) {
    ARG_UNUSED(out); ARG_UNUSED(max); return 0;
}
void channel_hop_ep_note_peer_rssi(int8_t ewma) { ARG_UNUSED(ewma); }
uint8_t channel_hop_ep_get_rssi(struct channel_hop_ep_rssi *out, uint8_t max) {
    ARG_UNUSED(out); ARG_UNUSED(max); return 0;
}

#endif /* CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_HOP */
