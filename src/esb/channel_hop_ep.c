/*
 * Copyright (c) 2025 efogdev
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/kernel.h>
#include <zmk_esb/protocol.h>
#include <zmk_esb/channel_hop.h>
#include "channel_hop_ep.h"
#include "esb_transport.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(zmk_esb_chhop_ep, CONFIG_ZMK_LOG_LEVEL);

#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_HOP)

static struct quarantine_state m_quarantine;
static uint8_t m_committed_next = CHANNEL_HOP_INVALID;
static bool m_link_up;

/* Channel we hopped away from on the most recent hop, iff no TX_SUCCESS
 * has been observed since that hop. If a subsequent hop fires while this
 * is still set, the new channel clearly isn't working either — revert
 * here instead of picking yet another fresh candidate that might be just
 * as bad. Cleared on any TX success, on explicit revert, and on link
 * down. Single-byte read/written from ISR and workqueue; the race is
 * benign (stale read at worst skips a revert). */
static volatile uint8_t m_prev_channel = CHANNEL_HOP_INVALID;

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
 * channel long enough for the dongle's silence watchdog (default 275 ms) and
 * validate window (default 350 ms) to fire. Without this, 3 consecutive TX
 * fails (~30 ms on a dead link) bounce the endpoint off the new channel
 * before the dongle ever arrives, so the speculative hop always validates
 * against an empty channel. Zero means "no cooldown active". */
static uint32_t m_hop_cooldown_until;

/* Uptime (ms) of the most recent positive link event on the current
 * channel — either a hop commit (dongle *should* follow) or a TX_SUCCESS
 * (dongle actually did follow, or link is healthy). Used by the cooldown
 * rendezvous fallback: while cooldown is active, if no positive event has
 * occurred for RENDEZVOUS_FALLBACK_MS, the link is dead long enough to
 * justify bypassing the cooldown and forcing a rendezvous hop.
 *
 * Earlier revisions zeroed this on TX_SUCCESS. That made one lucky packet
 * inside the 2500 ms cooldown permanently disable fallback for the rest
 * of the cooldown — even if the link went dead immediately afterwards.
 * With two such success→silence cycles back-to-back (cooldown is 2500 ms),
 * the endpoint could sit without firing any hop for ~5 s. Refreshing to
 * now on TX_SUCCESS keeps the "no activity for RENDEZVOUS_FALLBACK_MS"
 * invariant correct across intermittent links.
 *
 * Zero still means "no hop has ever committed" — set only by init /
 * disconnect and preserved as the fallback's "is there anything to fall
 * back on?" gate. 32-bit wrap-safe compare. */
static volatile uint32_t m_last_link_event_ms;

/* Set by channel_hop_ep_on_tx_fail_isr when the cooldown-fallback condition
 * triggers; consumed by hop_work_fn to bypass the normal candidate-selection
 * path and target the rendezvous channel directly. Single-bit flag, single
 * producer (ISR), single consumer (workqueue) — no lock. */
static volatile bool m_rendezvous_fallback_pending;

/* Delay before re-submitting hop_work after a failed esb_transport_set_channel.
 * -EBUSY resolves within one or two TX slots (retransmit_delay is 470us,
 * retransmit_count=2, so ~1.5ms for a drain). 5ms is a comfortable buffer. */
#define HOP_RETRY_DELAY_MS 5

static bool hop_cooldown_active(void) {
    if (m_hop_cooldown_until == 0) {
        return false;
    }
    return (int32_t)(m_hop_cooldown_until - k_uptime_get_32()) > 0;
}

static void enter_idle_state(void);
static void enter_active_state(bool send_initial_proposal);

static uint8_t pick_candidate(void) {
    const uint8_t current = esb_transport_get_channel();
    return channel_hop_pick(&m_quarantine,
                            CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_QUARANTINE_MIN_DISTANCE,
                            current);
}

static void send_proposal(uint8_t candidate) {
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
        const uint8_t candidate = pick_candidate();
        if (candidate != CHANNEL_HOP_INVALID) {
            send_proposal(candidate);
        } else {
            LOG_WRN("no candidate channel available");
        }
    } else {
        /* Periodically re-affirm the existing committed channel so a
         * dongle that rebooted picks up our view. */
        send_proposal(m_committed_next);
    }

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
    struct esb_pkt_idle pkt = { .type = ESB_PKT_IDLE };
    /* IDLE is the dongle's sole signal to disarm its silence watchdog
     * (see channel_hop_dongle.c::channel_hop_dongle_note_rx_idle). A
     * single dropped IDLE leaves the dongle thinking the peer is still
     * active, fires silence_work after RX_SILENCE_MS, triggers a
     * speculative hop into a stale committed_next, and cascades into
     * rollback dwell — a multi-second blackout from a single 1-byte
     * packet loss. Burst three back-to-back: ESB queues all three into
     * the TX FIFO and the radio drains them in <1 ms at 2 Mbps, so
     * total link-loss probability drops by ~3 orders of magnitude with
     * negligible cost. */
    for (int i = 0; i < 3; i++) {
        const int err = esb_transport_send(ESB_PIPE_DATA, (uint8_t *)&pkt, sizeof(pkt));
        if (err) {
            LOG_DBG("IDLE send %d failed: %d", i, err);
        }
    }
}

static void enter_idle_state(void) {
    if (!m_active) {
        return;
    }
    m_active = false;
    k_work_cancel_delayable(&negotiate_work);
    k_work_cancel_delayable(&idle_check_work);
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
        candidate = m_committed_next;
    } else {
        candidate = pick_candidate();
    }

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
    } else if (candidate != CHANNEL_HOP_INVALID) {
        send_proposal(candidate);
    }

    if (--m_post_hop_burst_remaining > 0) {
        k_work_reschedule(&post_hop_burst_work,
                          K_MSEC(CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_HOP_POST_BURST_INTERVAL_MS));
    }
}

static void start_post_hop_burst(void) {
    if (CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_HOP_POST_BURST_COUNT == 0) {
        return;
    }
    m_post_hop_burst_remaining = CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_HOP_POST_BURST_COUNT;
    /* First PROPOSAL lands after the transport's quiet window expires —
     * esb_transport_send silently drops anything sent before then. */
    k_work_reschedule(&post_hop_burst_work,
                      K_MSEC(CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_HOP_POST_QUIET_MS));
}

static void enter_active_state(bool send_initial_proposal) {
    if (m_active) {
        /* Already active — just refresh the idle deadline. */
        k_work_reschedule(&idle_check_work,
                          K_MSEC(CONFIG_ZMK_ESB_ENDPOINT_IDLE_THRESHOLD_MS));
        return;
    }
    m_active = true;
    LOG_DBG("endpoint entering ACTIVE");
    k_work_reschedule(&idle_check_work,
                      K_MSEC(CONFIG_ZMK_ESB_ENDPOINT_IDLE_THRESHOLD_MS));
    if (send_initial_proposal) {
        k_work_reschedule(&negotiate_work, K_NO_WAIT);
    } else {
        /* Idle→active transition: fire negotiate at fast-retry tempo
         * (~200 ms) instead of the slow INTERVAL. The dongle's
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

static void hop_work_fn(struct k_work *w) {
    ARG_UNUSED(w);
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
    if (m_rendezvous_fallback_pending) {
        m_rendezvous_fallback_pending = false;
        is_rendezvous_fallback = true;
    }

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
        if (target == CHANNEL_HOP_INVALID || target == current) {
            /* No valid pre-negotiated channel (either CONFIRM never arrived,
             * or committed_next happened to match our current channel — e.g.
             * after a prior hop on a noisy link). Picking on the fly is
             * strictly better than staying on a bad channel: the dongle's
             * speculative hop + rollback will locate us on the new channel
             * even though it wasn't pre-confirmed. */
            target = channel_hop_pick(
                &m_quarantine,
                CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_QUARANTINE_MIN_DISTANCE,
                current);
            if (target == CHANNEL_HOP_INVALID) {
                LOG_WRN("no candidate channel available; staying on %u", current);
                /* Clear the fail counter so the modulo trigger in the TX
                 * ISR will rearm after the next THRESHOLD failures. */
                esb_transport_reset_consecutive_fail();
                return;
            }
            LOG_WRN("no committed next; picked %u on the fly", target);
        }
    }

    /* Try the radio hop FIRST. All persistent state mutations are below,
     * guarded by success. */
    const int err = esb_transport_set_channel(target);
    if (err) {
        LOG_ERR("channel hop %u -> %u failed: %d (will retry in %ums)",
                current, target, err, (unsigned)HOP_RETRY_DELAY_MS);
        /* -EBUSY here means the PTX is in PTX_TX/PTX_TX_ACK/PTX_TXIDLE and
         * we lack fast-channel-switching. Leave committed_next, m_prev,
         * and quarantine intact so the retry can make the same decision.
         * Reset the fail counter so the TX ISR's modulo trigger re-arms
         * and a new burst of fails still retriggers us. */
        esb_transport_reset_consecutive_fail();
        k_work_reschedule(&hop_retry_work, K_MSEC(HOP_RETRY_DELAY_MS));
        return;
    }

    /* Hop succeeded. NOW commit state transitions. */
    if (is_rendezvous_fallback) {
        /* Rendezvous fallback: do NOT quarantine the channel we left —
         * it might be perfectly fine, we just lost sync with the dongle.
         * Clear quarantine on the rendezvous target in case a prior hop
         * had banished it. No revert arming: from here we want the dongle
         * to find us and resume normal negotiation. */
        m_quarantine.expires_at[target] = 0;
        m_prev_channel = CHANNEL_HOP_INVALID;
        LOG_WRN("rendezvous fallback: %u -> %u (cooldown desync recovery)",
                current, target);
    } else {
        if (is_revert) {
            /* The prev channel was quarantined by the previous hop; clear that
             * so we are allowed to occupy it again. */
            m_quarantine.expires_at[target] = 0;
            LOG_WRN("no TX success since last hop; reverted %u -> %u", current, target);
        }
        quarantine_add(&m_quarantine, current,
                       CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_QUARANTINE_MS);

        /* Arm a future revert to `current` only for normal hops. A revert is
         * one-shot: if this revert also fails, the next hop must go pick a
         * fresh candidate, not bounce again. */
        m_prev_channel = is_revert ? CHANNEL_HOP_INVALID : current;

        LOG_INF("channel hop: %u -> %u (quarantined %u for %ums)",
                current, target, current,
                (unsigned)CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_QUARANTINE_MS);
    }

    m_committed_next = CHANNEL_HOP_INVALID;

    /* Arm the post-hop cooldown. During this window, TX-fail-triggered
     * hops are suppressed so the endpoint stays put long enough for the
     * dongle to fire its own silence watchdog (RX_SILENCE_MS) and validate
     * window (VALIDATE_MS). The retry path is not gated by this — a failed
     * set_channel still retries independently. */
    m_hop_cooldown_until = k_uptime_get_32() +
        CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_HOP_COOLDOWN_MS;

    /* Stamp the hop time so the cooldown-fallback path can measure how
     * long the new channel has been silent. Refreshed on every TX_SUCCESS
     * so intermittent successes don't permanently suppress the fallback
     * (they reset the "silence" clock without disarming it). */
    {
        const uint32_t now = k_uptime_get_32();
        m_last_link_event_ms = (now == 0) ? 1 : now;
    }

    /* Fire a short burst of PROPOSALs so the dongle's speculative hop
     * validates within its window. */
    start_post_hop_burst();

    /* committed_next was just cleared. Schedule negotiate_work at the
     * fast-retry tempo so we keep hammering on PROPOSAL until CONFIRM
     * arrives — *not* at the 60s steady-state interval, which would
     * strand us without a next-hop until long after the burst ended.
     * negotiate_work_fn will drop back to the slow interval on its own
     * once committed_next is populated again. */
    k_work_reschedule(&negotiate_work,
                      K_MSEC(CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_HOP_NEGOTIATE_RETRY_MS));
}

void channel_hop_ep_init(void) {
    quarantine_reset(&m_quarantine);
    m_committed_next = CHANNEL_HOP_INVALID;
    m_prev_channel = CHANNEL_HOP_INVALID;
    m_active = false;
    m_post_hop_burst_remaining = 0;
    m_hop_cooldown_until = 0;
    m_last_link_event_ms = 0;
    m_rendezvous_fallback_pending = false;
    m_request_burst_remaining = 0;
    k_work_init_delayable(&negotiate_work, negotiate_work_fn);
    k_work_init_delayable(&idle_check_work, idle_check_work_fn);
    k_work_init_delayable(&post_hop_burst_work, post_hop_burst_work_fn);
    k_work_init_delayable(&hop_retry_work, hop_retry_work_fn);
    k_work_init(&hop_work, hop_work_fn);
}

void channel_hop_ep_on_connected(void) {
    m_link_up = true;
    /* Start ACTIVE so the first PROPOSAL goes out immediately and we have
     * a committed_next available the first time TX failures hit the hop
     * threshold. idle_check_work will demote us if no HID follows. */
    enter_active_state(true);
}

void channel_hop_ep_on_disconnected(void) {
    m_link_up = false;
    m_active = false;
    m_committed_next = CHANNEL_HOP_INVALID;
    m_prev_channel = CHANNEL_HOP_INVALID;
    m_post_hop_burst_remaining = 0;
    m_hop_cooldown_until = 0;
    m_last_link_event_ms = 0;
    m_rendezvous_fallback_pending = false;
    m_request_burst_remaining = 0;
    k_work_cancel_delayable(&negotiate_work);
    k_work_cancel_delayable(&idle_check_work);
    k_work_cancel_delayable(&post_hop_burst_work);
    k_work_cancel_delayable(&hop_retry_work);
}

void channel_hop_ep_note_user_tx(void) {
    if (!m_link_up) {
        return;
    }
    enter_active_state(false);
}

void channel_hop_ep_on_rx(const uint8_t *data, uint8_t len) {
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
    const bool was_missing = (m_committed_next == CHANNEL_HOP_INVALID);
    m_committed_next = c->agreed;
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
    k_work_reschedule(&negotiate_work, K_NO_WAIT);
    LOG_INF("REQUEST received — burst of %u PROPOSAL attempts + hop trigger",
            m_request_burst_remaining);

    /* Also trigger a hop. REQUEST is positive evidence the dongle just
     * came out of a failed validation (its committed_next was cleared),
     * which strongly implies the current channel can no longer carry the
     * link reliably — otherwise the dongle's silence watchdog wouldn't
     * have fired in the first place. Re-affirming PROPOSALs alone leaves
     * us on the same dead channel; the only way out is to actually hop.
     * The funnel goes through the same TX-fail entry point, so cooldown,
     * m_active, and the rendezvous-fallback gating all keep working —
     * a REQUEST during cooldown is harmless (returns early). */
    channel_hop_ep_on_tx_fail_isr();
}

void channel_hop_ep_on_tx_fail_isr(void) {
    /* Only hop while we're actively sending user traffic. During idle
     * the only packets on the wire are the single ESB_PKT_IDLE we fired
     * at transition; if that one fails we do NOT want to hop — nothing
     * urgent is pending. */
    if (!m_active) {
        return;
    }
    /* Post-hop cooldown: after a recent hop, give the dongle time to fire
     * its own silence watchdog (RX_SILENCE_MS) and validate the speculative
     * hop (VALIDATE_MS) before we consider hopping again. Without this we
     * bounce off the new channel in ~30 ms, far before the dongle (275 ms
     * silence) ever arrives there, so the speculative hop always validates
     * against an empty channel. The post-hop PROPOSAL burst continues
     * firing during cooldown regardless — that's precisely what gives the
     * dongle a packet to confirm against. */
    if (hop_cooldown_active()) {
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
        return;
    }
    k_work_submit(&hop_work);
}

void channel_hop_ep_on_tx_success_isr(void) {
    /* A packet made it through on the current channel — any pending
     * "revert to the previous channel" arming is no longer appropriate. */
    m_prev_channel = CHANNEL_HOP_INVALID;
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

bool channel_hop_ep_is_quarantined(uint8_t channel) {
    return quarantine_is(&m_quarantine, channel);
}

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

#endif /* CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_HOP */
