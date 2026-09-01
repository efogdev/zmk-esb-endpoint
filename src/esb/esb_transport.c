/*
 * Copyright (c) 2025 efogdev
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <hal/nrf_timer.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <esb.h>
#include "esb_transport.h"
#include "radio_arbiter.h"
#include <zmk_esb/protocol.h>
#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_SHELL_RELAY)
#include "../shell/shell_relay.h"
#endif
#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_BENCH)
#include "bench.h"
#endif
#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_HOP)
#include "channel_hop_ep.h"
#endif

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(zmk_esb_transport, CONFIG_ZMK_ESB_ENDPOINT_LOG_LEVEL);

static esb_transport_cb_t m_cb;
static struct esb_payload m_rx_payload;

static struct {
    uint8_t base0[4];
    uint8_t base1[4];
    uint8_t prefixes[8];
    uint8_t channel;
    /* Channel the radio booted on (the DTS-default rendezvous channel).
     * The peer's rollback dwell cycle always includes this channel, so
     * it is the one place the keyboard can reliably reach a desynced
     * peer. Set once in set_addresses(); never overwritten by hops. */
    uint8_t boot_channel;
    bool configured;
} m_addr;

#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_RENDEZVOUS)
/* Synchronous TX state. Used by esb_transport_send_blocking() to fire one
 * packet on a transient channel and wait for the radio's TX_SUCCESS /
 * TX_FAILED before returning. While in_progress is true:
 *   - the regular TX_SUCCESS / TX_FAILED handlers short-circuit (don't
 *     touch consecutive_tx_fail or trigger channel hops — this packet was
 *     not a user-driven send on the active channel),
 *   - esb_transport_send() drops user-thread sends so they do not race
 *     onto the wrong channel.
 * Single in-flight only; callers must serialise (current sole caller is
 * the post-hop burst on the system workqueue). */
static struct k_sem    m_sync_tx_done;
static volatile bool   m_sync_tx_in_progress;
static volatile bool   m_sync_tx_result_success;
#endif

/* Double-buffered RX mailbox. The EGU event ISR fills the inactive slot and
 * publishes it by flipping rx_active; rx_work_fn reads the published slot.
 * The ISR never writes the slot the consumer just snapshotted, so a torn read
 * needs two ISR events inside one rx_work_fn pass rather than any overlap. */
static struct k_work rx_work;
static uint8_t rx_buf[2][ESB_MAX_PAYLOAD_LEN];
static uint8_t rx_len[2];
static volatile uint8_t rx_active;

static uint32_t tx_ok_count;
static uint32_t tx_fail_count;
static uint32_t tx_retried_count;      /* ACKed events with tx_attempts > 1 */
static uint32_t tx_exhausted_count;    /* packets that died after all retries */
static uint32_t consecutive_tx_fail;

/* HID-report TX rate observability. m_hid_tx_count is the lifetime count
 * of ESB_PKT_HID_REPORT sends. m_hid_tx_bucket_count accumulates inside
 * the current one-second bucket, committed to m_hid_tx_rate_hz when
 * elapsed since m_hid_tx_bucket_ms crosses HID_RATE_BUCKET_MS. The
 * getter treats m_hid_tx_rate_hz as stale once HID_RATE_STALE_MS has
 * passed without a new send updating the bucket. All updates happen
 * from the user-thread send path; reads are single-word loads. */
#define HID_RATE_BUCKET_MS 1000U
#define HID_RATE_STALE_MS  2000U
static uint32_t m_hid_tx_count;
static uint32_t m_hid_tx_bucket_count;
static uint32_t m_hid_tx_bucket_ms;
static uint16_t m_hid_tx_rate_hz;

#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_CRITICAL_MAX_RETRANSMIT)
/* Parallel ring tracking which recent sends had the override applied.
 * Independent from m_recent_noack[] (which only exists when CHANNEL_HOP is
 * compiled in) so the critical-packet feature works for BEACON/PAIR_RESP/
 * VERIFY_REQ/DISCONNECT even with channel hopping disabled. Indexed via
 * `& (SIZE - 1)`, so the depth must be a power of two. */
BUILD_ASSERT((CONFIG_ZMK_ESB_ENDPOINT_CRIT_OVERRIDE_RING_SIZE &
              (CONFIG_ZMK_ESB_ENDPOINT_CRIT_OVERRIDE_RING_SIZE - 1)) == 0,
             "CRIT_OVERRIDE_RING_SIZE must be a power of two");
static uint8_t  m_recent_critical_override[CONFIG_ZMK_ESB_ENDPOINT_CRIT_OVERRIDE_RING_SIZE];
static uint8_t  m_recent_critical_head;
static uint8_t  m_recent_critical_tail;
#endif /* CRITICAL_MAX_RETRANSMIT */

/* Pointer/scroll refund + per-send motion ring. When a mouse report whose
 * deltas are non-zero exhausts its (lower) retransmit budget, the deltas
 * are not lost: they are added (saturating to int16) to the next outgoing
 * mouse report's d_x/d_y/d_scroll_*. Buttons are deliberately excluded —
 * edge-triggered state cannot be reconstructed by accumulation, the
 * input processor's pending_buttons queue handles edge replay separately.
 *
 * m_recent_motion[] mirrors the existing override / noack rings: one slot
 * per esb_write_payload, popped one per TX event. On TX_FAILED we read the
 * deltas back out of the slot into m_motion_refund. On TX_SUCCESS we just
 * advance the tail. Same drift trade-off as m_recent_critical_override:
 * esb_flush_tx() drops queued packets without firing TX events, leaving
 * a few stale slots; the ring stays bounded so refund accumulation is
 * bounded too.
 *
 * Refund is ALSO consumed on every mouse-report send (not just on the
 * TX_FAILED that produced it) so a refund stranded by no further motion
 * isn't kept forever. Stale refunds older than
 * CONFIG_ZMK_ESB_ENDPOINT_POINTER_REFUND_STALE_MS are dropped at apply
 * time (matches the input processor's MOTION_MAX_STALE_MS rationale:
 * a long radio outage shouldn't produce a giant cursor jump when
 * traffic resumes). */
struct pointer_motion_record {
    int16_t dx;
    int16_t dy;
    int16_t scroll_x;
    int16_t scroll_y;
    uint8_t overrode;  /* radio's retransmit_count was lowered for this send */
    uint8_t hid_kind;  /* enum esb_transport_hid_kind of the matching send */
};
static struct pointer_motion_record
    m_recent_motion[CONFIG_ZMK_ESB_ENDPOINT_POINTER_MOTION_RING_SIZE];
/* uint32_t (not uint8_t like the other rings) because the Kconfig ring
 * size is not constrained to a power of 2 — uint8 wrap (mod 256) would
 * mis-index whenever 256 % SIZE != 0. % SIZE on a 32-bit counter stays
 * correct for any SIZE; counters are bumped from a single ISR + the
 * thread send path with the existing transport-wide synchronisation. */
static uint32_t m_recent_motion_head;
static uint32_t m_recent_motion_tail;

static struct {
    int32_t  dx;
    int32_t  dy;
    int32_t  scroll_x;
    int32_t  scroll_y;
    uint32_t stamp_ms;  /* 0 means empty */
} m_motion_refund;

/* Count of pointer-motion mouse reports currently sitting in the ESB TX
 * FIFO (pushed via esb_write_payload, matching TX event not yet seen).
 * Bumped by motion_ring_push for any send carrying non-zero deltas;
 * decremented when motion_ring_consume pops the matching slot. Mirrors
 * the (head - tail) gap on the motion ring but counts only mouse-with-
 * motion entries — buttons-only mouse reports and non-mouse sends push
 * zero records into the ring for symmetry but do not contribute to
 * back-pressure. Zeroed by esb_flush_tx_and_reset() (every flush abandons
 * its in-flight pointer sends along with the queue). The cap that gates
 * back-pressure is CONFIG_ZMK_ESB_ENDPOINT_POINTER_INFLIGHT_CAP. */
static uint8_t m_pointer_inflight_count;

/* HID delivery-failure resync. When an ACKed HID report dies without
 * being delivered — retry exhaustion (TX_FAILED) or a TX-FIFO flush
 * that drops it unsent — the report's kind bit is OR'd into
 * m_hid_resync_pending (from ISR or thread context) and
 * m_hid_resync_work is scheduled HID_RESYNC_DELAY_MS out. The work
 * fetches-and-clears the mask and invokes the registered consumers,
 * which re-send their current absolute report state. The delay
 * coalesces a failure burst into one resync and lets a hop's quiet
 * window arm first; k_work_schedule on an already-pending delayable
 * keeps the earlier deadline, so a storm of failures still yields one
 * callback. If the resync send itself dies, its own TX event re-sets
 * the bit — the loop is bounded at one in-flight resync per report
 * kind and self-terminates on the first TX_SUCCESS. */
static atomic_t m_hid_resync_pending;
static struct k_work_delayable m_hid_resync_work;

#define HID_RESYNC_MAX_CBS 4
static struct {
    esb_transport_hid_resync_cb_t cb;
    void *ctx;
} m_hid_resync_cbs[HID_RESYNC_MAX_CBS];
static uint8_t m_hid_resync_cb_count;

int esb_transport_register_hid_resync_cb(const esb_transport_hid_resync_cb_t cb, void *ctx) {
    if (m_hid_resync_cb_count >= HID_RESYNC_MAX_CBS) {
        return -ENOMEM;
    }
    m_hid_resync_cbs[m_hid_resync_cb_count].cb = cb;
    m_hid_resync_cbs[m_hid_resync_cb_count].ctx = ctx;
    m_hid_resync_cb_count++;
    return 0;
}

/* Callable from the RADIO ISR and thread context alike. */
static void hid_resync_request(const uint8_t kinds_mask) {
    atomic_or(&m_hid_resync_pending, kinds_mask);
    k_work_schedule(&m_hid_resync_work,
                    K_MSEC(CONFIG_ZMK_ESB_ENDPOINT_HID_RESYNC_DELAY_MS));
}

static void hid_resync_work_fn(struct k_work *work) {
    ARG_UNUSED(work);
    const uint8_t kinds = (uint8_t)atomic_clear(&m_hid_resync_pending);
    if (kinds == 0) {
        return;
    }
    for (uint8_t i = 0; i < m_hid_resync_cb_count; i++) {
        m_hid_resync_cbs[i].cb(kinds, m_hid_resync_cbs[i].ctx);
    }
}

#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_HOP)
#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_ADAPTIVE_RETRY)
/* Adaptive retransmit count state. Recomputed periodically from
 * m_link_quality_ewma_x10 so the radio spends fewer retries on a clean
 * link (lower tail latency) and more on a degraded one (survival over
 * speed). Last applied value cached to elide redundant set_retransmit_count
 * calls. Initialised at esb_transport_init to the Kconfig default, which
 * is also what esb_init_and_configure programs on slot start. */
static uint8_t m_adaptive_retransmit_count =
    CONFIG_ZMK_ESB_ENDPOINT_RETRANSMIT_COUNT;
static struct k_work_delayable m_adaptive_retry_work;
#endif /* ADAPTIVE_RETRY */

/* Time-window TX-failure tracking. m_first_fail_ms holds the uptime at
 * which the current fail window started; cleared on any TX_SUCCESS and
 * on channel change. We deliberately do NOT use a "triggered-once" latch:
 * the hopper's own cooldown (m_hop_cooldown_until in channel_hop_ep.c)
 * suppresses back-to-back hops, and a latch here interacts badly with it
 * — if the hopper returns early from cooldown the latch would stay set
 * forever (TX_SUCCESS never comes on a dead link), permanently disabling
 * the trigger. Instead, after each trigger call we re-stamp m_first_fail_ms
 * so the next call is gated by one more WINDOW_MS of continuous failures.
 * Touched from the RADIO ISR and from esb_transport_set_channel() (which
 * runs under the hop work item after esb_flush_tx() has quiesced the
 * radio), so no locking is needed. */
static uint32_t m_first_fail_ms;

/* Uptime of the most recent non-sync TX_SUCCESS. Zero before the first
 * success or after a channel change. Used by the weak-link trigger to
 * detect "no progress for too long even though some packets are getting
 * through" — the case where consecutive_tx_fail and m_first_fail_ms are
 * reset to zero by every occasional success, so the fail-window trigger
 * above never accumulates to its threshold. Stamped from the RADIO ISR;
 * single-word Cortex-M access so no lock needed (same as m_first_fail_ms). */
static uint32_t m_last_tx_success_ms;

/* Consecutive -EBUSY results from esb_set_rf_channel() across hop
 * attempts. Forcing the radio idle aborts an in-flight TX cycle, so the
 * hop path first lets the retry worker wait out transient busyness and
 * only stops TX once the streak shows the radio is not draining to IDLE
 * on its own. */
static uint8_t m_set_channel_busy_streak;

/* Post-hop quiet window deadline. Set by esb_transport_set_channel();
 * esb_transport_send() silently drops packets while now < deadline so
 * the peer has time to follow the speculative hop before we pile on
 * new traffic that would otherwise count as failures. Zero means "no
 * active quiet period". 32-bit wrap-safe compare in the send path. */
static uint32_t m_tx_quiet_until_ms;

/* Link-quality observability. The nRF ESB library reports tx_attempts on
 * every TX event (1 = first try succeeded, N+1 on a failure where N is
 * retransmit_count). m_link_quality_window is a sliding shift register —
 * one bit per ACKed TX event, set when the event needed at least one
 * retransmit. m_link_quality_count caches its popcount so the
 * threshold check is O(1) per packet. The metric is independent of the
 * TX-fail-window and weak-link triggers; it lights up much earlier (at
 * 5–10% PER) so a cooperative hop can fire while the link is still
 * carrying traffic. Reset by esb_transport_set_channel() — a hop wipes
 * link history.
 *
 * m_link_quality_ewma_x10 is the EWMA of tx_attempts on the
 * TX_SUCCESS path only (×10 for fixed-point), exposed via
 * esb_transport_get_link_quality() for shell/observability and
 * adaptive_retry. TX_FAILED still flips the popcount bit so coop-hop
 * sees retry exhaustion, but does not feed the EWMA — its
 * retransmit_count+1 sample is bounded by the radio ceiling rather
 * than the real delivery cost and would dominate the average.
 *
 * m_recent_noack[] tracks the noack flag of the most recent sends in
 * arrival order, indexed by m_recent_noack_head & (RING_SIZE - 1)
 * (depth set by LINK_QUALITY_NOACK_RING_SIZE). Each TX event
 * consumes the tail entry and skips the metric update if the matching
 * send was noack — those report tx_attempts=1 unconditionally and
 * would dilute the signal toward zero. The send-side push runs on the
 * same TX path that yields the event, so head/tail stay in sync as
 * long as we account for one push per event. */
/* Indexed via `& (SIZE - 1)`, so the depth must be a power of two. */
BUILD_ASSERT((CONFIG_ZMK_ESB_ENDPOINT_LINK_QUALITY_NOACK_RING_SIZE &
              (CONFIG_ZMK_ESB_ENDPOINT_LINK_QUALITY_NOACK_RING_SIZE - 1)) == 0,
             "LINK_QUALITY_NOACK_RING_SIZE must be a power of two");
static uint32_t m_link_quality_window;
static uint8_t  m_link_quality_count;
static uint16_t m_link_quality_ewma_x10;
static uint8_t  m_recent_noack[CONFIG_ZMK_ESB_ENDPOINT_LINK_QUALITY_NOACK_RING_SIZE];
static uint8_t  m_recent_noack_head;
static uint8_t  m_recent_noack_tail;

#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_COOP_HOP)
/* Hysteresis gate. Set when a link-degraded trigger fires; cleared when
 * popcount drops to LINK_DEGRADED_REARM. While set, threshold-cross
 * checks are skipped so a single noisy window does not retrigger
 * before the link has had a chance to settle (or the cooperative-hop
 * cooldown expires elsewhere). Single-byte read/write from RADIO ISR
 * and from set_channel — race is benign. */
static volatile bool m_link_degraded_armed;
#endif /* COOP_HOP */
#endif

/* 32-bit wrap-safe: (uint32_t)(now - last) stays correct for spans up to
 * ~24.8 days, far beyond any sensible activity threshold. Single-word access
 * on Cortex-M is atomic, so no lock is needed. */
static uint32_t m_last_activity_ms;
static bool     m_activity_seen;

/* Uptime of the most recent ESB TX event (success or fail), regardless of
 * sync_tx_in_progress / channel-hop state. Read by esb_transport_flush_tx
 * to gate the public flush on radio silence. Stamped from the RADIO ISR;
 * single-word Cortex-M access so no lock needed. 0 means "no event seen
 * yet" — treated as silence by the gate. */
static uint32_t m_last_tx_event_ms;

/* xorshift32 state for retry-delay jitter. Seeded lazily on first use from
 * k_uptime_get_32 — we cannot rely on init ordering w.r.t. the kernel
 * clock. One static counter flipped monotonically so the seed re-derives
 * if m_jitter_rng happens to land on 0. No thread-safety: caller is
 * esb_transport_send, which runs on the user thread and is not re-entered
 * (the send path is serialised by the ESB library's own locks). */
static uint32_t m_jitter_rng;

static uint16_t jittered_retransmit_delay(void) {
    if (m_jitter_rng == 0) {
        uint32_t seed = k_uptime_get_32();
        seed ^= (uint32_t)(uintptr_t)&m_jitter_rng;
        if (seed == 0) {
            seed = 0xA3C59B1Du;
        }
        m_jitter_rng = seed;
    }
    /* xorshift32 — one multiplication-free step, plenty random for jitter. */
    m_jitter_rng ^= m_jitter_rng << 13;
    m_jitter_rng ^= m_jitter_rng >> 17;
    m_jitter_rng ^= m_jitter_rng << 5;

    /* ±12.5% of base, computed as base ± (base >> 3) * (random_byte / 128).
     * Using a byte-wide mixer keeps the arithmetic tight; 12.5% is
     * enough to de-correlate from periodic interferers (WiFi beacons at
     * 102.4 ms, BLE adv at 20/30/50 ms) without extending the retry
     * slot enough to overrun the Nordic ESB library's internal timing. */
    const uint32_t base = CONFIG_ZMK_ESB_ENDPOINT_RETRANSMIT_DELAY_US;
    const uint32_t span = base >> 3;  /* 12.5% */
    const int8_t   off  = (int8_t)(m_jitter_rng & 0xFFu);  /* -128..127 */
    const int32_t  delta = ((int32_t)span * off) / 128;
    int32_t d = (int32_t)base + delta;
    if (d < 250) {
        d = 250;
    }
    if (d > UINT16_MAX) {
        d = UINT16_MAX;
    }
    return (uint16_t)d;
}

/* Peer's view of the link. Stamped from ESB_PKT_LINK_STATS ACK
 * payloads; consumed by adaptive-retransmit and future TX-power logic.
 * m_peer_rssi_valid flips true on the first LINK_STATS RX — before that
 * the values are meaningless. Single-byte loads on Cortex-M, no lock. */
static int8_t  m_peer_rssi_last;
static int8_t  m_peer_rssi_ewma;
static bool    m_peer_rssi_valid;

static void note_activity(void) {
    m_last_activity_ms = k_uptime_get_32();
    m_activity_seen = true;
}

uint32_t esb_transport_ms_since_activity(void) {
    if (!m_activity_seen) {
        return UINT32_MAX;
    }
    return (uint32_t)(k_uptime_get_32() - m_last_activity_ms);
}

#define CONSECUTIVE_WARN_THRESHOLD CONFIG_ZMK_ESB_ENDPOINT_TX_FAIL_WARN_THRESHOLD
#define CONSECUTIVE_ERR_THRESHOLD  CONFIG_ZMK_ESB_ENDPOINT_TX_FAIL_ERR_THRESHOLD

static int esb_init_and_configure(void);

static void rx_work_fn(struct k_work *w) {
    if (!m_cb) {
        return;
    }
    note_activity();
    const uint8_t slot = rx_active;
    const esb_transport_evt_t evt = {
        .type   = ESB_RX_EVT,
        .rx_buf = rx_buf[slot],
        .rx_len = rx_len[slot],
    };
    m_cb(&evt);
}

#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_HOP)
/* Pop one entry from the noack ring (in arrival order). Returns true if
 * the matching send was noack — the caller should skip the metric
 * update in that case. If the ring is empty (event arrived without a
 * matching push, e.g. on first-event-after-reset), returns false so
 * the event is counted; mis-attribution here is bounded by ring size. */
static bool consume_recent_noack(void) {
    if (m_recent_noack_tail == m_recent_noack_head) {
        return false;
    }
    const bool was_noack =
        m_recent_noack[m_recent_noack_tail & (CONFIG_ZMK_ESB_ENDPOINT_LINK_QUALITY_NOACK_RING_SIZE - 1)] != 0;
    m_recent_noack_tail++;
    return was_noack;
}

/* Shift one bit into the link-quality window and maintain the cached
 * popcount. Called from the RADIO ISR. Single-word Cortex-M reads/
 * writes on the state vars are atomic; the shell accessor takes a
 * snapshot rather than locking. */
static void link_quality_shift_window(const bool retry_bit) {
    const uint32_t old_window = m_link_quality_window;
    const uint32_t mask = (CONFIG_ZMK_ESB_ENDPOINT_LINK_QUALITY_WINDOW < 32)
        ? ((1u << CONFIG_ZMK_ESB_ENDPOINT_LINK_QUALITY_WINDOW) - 1u)
        : 0xFFFFFFFFu;
    const bool dropped_bit = (CONFIG_ZMK_ESB_ENDPOINT_LINK_QUALITY_WINDOW < 32)
        ? ((old_window >> (CONFIG_ZMK_ESB_ENDPOINT_LINK_QUALITY_WINDOW - 1)) & 1u)
        : ((old_window >> 31) & 1u);
    const uint32_t new_window = ((old_window << 1) | (retry_bit ? 1u : 0u)) & mask;
    m_link_quality_window = new_window;

    /* Popcount delta: subtract the bit shifted out (only meaningful
     * once the window has filled — but the bit there is initialized to
     * 0 anyway, so the delta is correct from boot). */
    if (retry_bit && !dropped_bit) {
        m_link_quality_count++;
    } else if (!retry_bit && dropped_bit) {
        m_link_quality_count--;
    }
}

/* TX_SUCCESS variant: shift the popcount window AND update the
 * tx_attempts EWMA. The TX_FAILED path uses link_quality_shift_window()
 * directly because retry-exhausted tx_attempts (retransmit_count+1)
 * is bounded by the radio's ceiling, not by actual delivery cost, and
 * would dominate the EWMA out of its nominal range. */
static void link_quality_record(const uint8_t tx_attempts, const bool retry_bit) {
    link_quality_shift_window(retry_bit);

    /* EWMA × 10 of attempts: ewma' = (ewma * 7 + attempts*10) / 8.
     * Initialized to 10 (== 1.0 attempts) in init / set_channel. */
    const uint16_t a10 = (uint16_t)tx_attempts * 10u;
    m_link_quality_ewma_x10 =
        (uint16_t)(((uint32_t)m_link_quality_ewma_x10 * 7u + a10) / 8u);
}

#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_COOP_HOP)
/* Weak default so the symbol resolves when channel-hop or coop-hop is
 * compiled out. The real implementation lives in channel_hop_ep.c. */
__weak void channel_hop_ep_on_link_degraded_isr(void) {}
#endif

#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_ADAPTIVE_RETRY)
/* Map the current link-quality EWMA to a target retransmit count.
 * EWMA is attempts × 10; higher means more retries are happening so
 * we raise the ceiling. Linear interpolation between the two
 * breakpoints; saturates at MIN / MAX outside. */
static uint8_t adaptive_retry_target(void) {
    const uint16_t ewma = m_link_quality_ewma_x10;
    if (ewma <= CONFIG_ZMK_ESB_ENDPOINT_ADAPTIVE_RETRY_EWMA_LOW) {
        return CONFIG_ZMK_ESB_ENDPOINT_ADAPTIVE_RETRY_COUNT_MIN;
    }
    if (ewma >= CONFIG_ZMK_ESB_ENDPOINT_ADAPTIVE_RETRY_EWMA_HIGH) {
        return CONFIG_ZMK_ESB_ENDPOINT_ADAPTIVE_RETRY_COUNT_MAX;
    }
    const uint32_t span = CONFIG_ZMK_ESB_ENDPOINT_ADAPTIVE_RETRY_EWMA_HIGH -
                          CONFIG_ZMK_ESB_ENDPOINT_ADAPTIVE_RETRY_EWMA_LOW;
    const uint32_t into = ewma - CONFIG_ZMK_ESB_ENDPOINT_ADAPTIVE_RETRY_EWMA_LOW;
    const uint32_t delta =
        (into * (CONFIG_ZMK_ESB_ENDPOINT_ADAPTIVE_RETRY_COUNT_MAX -
                 CONFIG_ZMK_ESB_ENDPOINT_ADAPTIVE_RETRY_COUNT_MIN)) / span;
    return (uint8_t)(CONFIG_ZMK_ESB_ENDPOINT_ADAPTIVE_RETRY_COUNT_MIN + delta);
}

static void adaptive_retry_work_fn(struct k_work *work) {
    ARG_UNUSED(work);
    const uint8_t want = adaptive_retry_target();
    if (want != m_adaptive_retransmit_count) {
        k_sched_lock();
        if (esb_set_retransmit_count(want) == 0) {
            m_adaptive_retransmit_count = want;
            LOG_DBG("adaptive retry: ewma_x10=%u -> count=%u",
                    m_link_quality_ewma_x10, want);
        }
        k_sched_unlock();
    }
    k_work_reschedule(&m_adaptive_retry_work,
                      K_MSEC(CONFIG_ZMK_ESB_ENDPOINT_ADAPTIVE_RETRY_INTERVAL_MS));
}
#endif /* ADAPTIVE_RETRY */
#endif

/* Restore the radio's retransmit count after a per-packet override (critical
 * or pointer) TX completes. Returns to the adaptive value if adaptive retry
 * is compiled in, otherwise the static Kconfig default. Best-effort —
 * -EBUSY just means the radio is mid-cycle on a back-to-back send; the
 * next override consumer or adaptive tick resyncs. */
static void restore_retransmit_count(void) {
    const uint8_t restore =
#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_HOP) && \
    IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_ADAPTIVE_RETRY)
        m_adaptive_retransmit_count;
#else
        (uint8_t)CONFIG_ZMK_ESB_ENDPOINT_RETRANSMIT_COUNT;
#endif
    (void)esb_set_retransmit_count(restore);
}

#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_CRITICAL_MAX_RETRANSMIT)
/* Pop one entry from the critical-override ring. Returns true if the matching
 * send pushed the radio to CRITICAL_RETRANSMIT_COUNT — the caller restores
 * the retransmit count in that case. Ring-empty returns false (benign: nothing
 * to restore). Called from the ESB event handler in lockstep with the send
 * push, so head/tail stay aligned. */
static bool consume_recent_override(void) {
    if (m_recent_critical_tail == m_recent_critical_head) {
        return false;
    }
    const bool was_override =
        m_recent_critical_override[m_recent_critical_tail & (CONFIG_ZMK_ESB_ENDPOINT_CRIT_OVERRIDE_RING_SIZE - 1)] != 0;
    m_recent_critical_tail++;
    return was_override;
}
#endif

/* Mouse body laid out as the input processor packs it (zmk_hid_mouse_report_body
 * memcpy'd into esb_pkt_hid_report.data). Local definition rather than including
 * <zmk/hid.h> here — the transport layer is otherwise oblivious to the HID
 * report shape. */
struct esb_mouse_body {
    uint8_t buttons;
    int16_t d_x;
    int16_t d_y;
    int16_t d_scroll_y;
    int16_t d_scroll_x;
} __attribute__((__packed__));

/* True if the payload bytes look like a mouse HID report packet — used to
 * gate refund + pointer-retransmit override on the send / TX-event paths. */
static inline bool is_mouse_report(const uint8_t *data, const uint8_t len) {
    return len >= (2u + sizeof(struct esb_mouse_body)) &&
           data[0] == ESB_PKT_HID_REPORT &&
           data[1] == ESB_REPORT_MOUSE;
}

static inline struct esb_mouse_body *mouse_body_of(uint8_t *pkt_data) {
    /* &pkt.data[2] in esb_pkt_hid_report = first byte of the mouse body. */
    return (struct esb_mouse_body *)(pkt_data + 2);
}

static int16_t sat16(const int32_t v) {
    if (v > INT16_MAX) return INT16_MAX;
    if (v < INT16_MIN) return INT16_MIN;
    return (int16_t)v;
}

/* Add any pending refund (saturating) into the outgoing mouse report's
 * d_x/d_y/d_scroll_*. Residual that overflowed int16 stays in the pool
 * for the next mouse send. Stale refunds (>= STALE_MS old) are dropped
 * before applying. */
static void motion_refund_apply(struct esb_mouse_body *body) {
    if (m_motion_refund.stamp_ms != 0) {
        const uint32_t age = k_uptime_get_32() - m_motion_refund.stamp_ms;
        if (age >= CONFIG_ZMK_ESB_ENDPOINT_POINTER_REFUND_STALE_MS) {
            m_motion_refund.dx = 0;
            m_motion_refund.dy = 0;
            m_motion_refund.scroll_x = 0;
            m_motion_refund.scroll_y = 0;
            m_motion_refund.stamp_ms = 0;
            return;
        }
    }
    if ((m_motion_refund.dx | m_motion_refund.dy |
         m_motion_refund.scroll_x | m_motion_refund.scroll_y) == 0) {
        return;
    }
    const int32_t total_dx = (int32_t)body->d_x + m_motion_refund.dx;
    const int32_t total_dy = (int32_t)body->d_y + m_motion_refund.dy;
    const int32_t total_sx = (int32_t)body->d_scroll_x + m_motion_refund.scroll_x;
    const int32_t total_sy = (int32_t)body->d_scroll_y + m_motion_refund.scroll_y;
    body->d_x        = sat16(total_dx);
    body->d_y        = sat16(total_dy);
    body->d_scroll_x = sat16(total_sx);
    body->d_scroll_y = sat16(total_sy);
    m_motion_refund.dx       = total_dx - body->d_x;
    m_motion_refund.dy       = total_dy - body->d_y;
    m_motion_refund.scroll_x = total_sx - body->d_scroll_x;
    m_motion_refund.scroll_y = total_sy - body->d_scroll_y;
    if ((m_motion_refund.dx | m_motion_refund.dy |
         m_motion_refund.scroll_x | m_motion_refund.scroll_y) == 0) {
        m_motion_refund.stamp_ms = 0;
    }
}

/* Push the (post-refund) motion deltas of an outgoing send onto the ring,
 * matching the slot the next TX event will consume. `overrode` records
 * whether the radio's retransmit_count was lowered for this send so the
 * matching event handler can restore. `hid_kind` classifies the payload
 * (keyboard/consumer/mouse HID report, or NONE) so a delivery failure
 * can be reported to the HID resync machinery. Non-mouse sends push a
 * zero motion record to keep head/tail in lockstep with TX events. */
static void motion_ring_push(const struct esb_mouse_body *body, const bool overrode,
                             const uint8_t hid_kind) {
    const uint32_t slot = m_recent_motion_head % CONFIG_ZMK_ESB_ENDPOINT_POINTER_MOTION_RING_SIZE;
    if (body) {
        m_recent_motion[slot].dx       = body->d_x;
        m_recent_motion[slot].dy       = body->d_y;
        m_recent_motion[slot].scroll_x = body->d_scroll_x;
        m_recent_motion[slot].scroll_y = body->d_scroll_y;
    } else {
        m_recent_motion[slot].dx = 0;
        m_recent_motion[slot].dy = 0;
        m_recent_motion[slot].scroll_x = 0;
        m_recent_motion[slot].scroll_y = 0;
    }
    m_recent_motion[slot].overrode = overrode ? 1U : 0U;
    m_recent_motion[slot].hid_kind = hid_kind;
    /* Track in-flight pointer-with-motion sends for back-pressure. Only
     * non-zero-delta pushes count — buttons-only mouse reports and non-
     * mouse sends are recorded for ring/event symmetry but should not
     * gate the input processor. Saturating cap on UINT8_MAX is paranoia:
     * any reasonable inflight cap is far smaller, and a stuck counter
     * would only over-back-pressure (correctness preserved). */
    if (body && (body->d_x | body->d_y | body->d_scroll_x | body->d_scroll_y) &&
        m_pointer_inflight_count < UINT8_MAX) {
        m_pointer_inflight_count++;
    }
    m_recent_motion_head++;
}

/* Pop one entry from the motion ring on TX event. On TX_FAILED the
 * deltas are added back to the refund pool (so the next mouse send
 * picks them up); on TX_SUCCESS the slot is just consumed. If the
 * popped slot had `overrode` set, the caller restores the radio's
 * retransmit count. Ring-empty pop is benign — flush_tx can drop a
 * push without the matching event ever arriving (same trade-off the
 * critical-override and noack rings document). */
static void motion_ring_consume(const bool failed) {
    if (m_recent_motion_tail == m_recent_motion_head) {
        return;
    }
    const uint32_t slot = m_recent_motion_tail % CONFIG_ZMK_ESB_ENDPOINT_POINTER_MOTION_RING_SIZE;
    const struct pointer_motion_record rec = m_recent_motion[slot];
    m_recent_motion_tail++;
    /* Mirror the push-side count: a slot with non-zero deltas was a
     * pointer-with-motion send, so the radio is done with one more
     * in-flight pointer payload. */
    if ((rec.dx | rec.dy | rec.scroll_x | rec.scroll_y) &&
        m_pointer_inflight_count > 0) {
        m_pointer_inflight_count--;
    }
    if (failed && (rec.dx | rec.dy | rec.scroll_x | rec.scroll_y)) {
        m_motion_refund.dx       += rec.dx;
        m_motion_refund.dy       += rec.dy;
        m_motion_refund.scroll_x += rec.scroll_x;
        m_motion_refund.scroll_y += rec.scroll_y;
        if (m_motion_refund.stamp_ms == 0) {
            const uint32_t now = k_uptime_get_32();
            m_motion_refund.stamp_ms = (now == 0) ? 1 : now;
        }
    }
    /* A failed HID report died undelivered after exhausting its retries.
     * Motion is repaired by the refund above; edge-triggered state
     * (keyboard keys, mouse buttons) cannot be — ask the owning HID
     * layer to re-send its current absolute state. Runs from the RADIO
     * ISR; hid_resync_request only does atomic_or + k_work_schedule. */
    if (failed && rec.hid_kind != ESB_HID_KIND_NONE) {
        hid_resync_request(BIT(rec.hid_kind));
    }
    if (rec.overrode) {
        restore_retransmit_count();
    }
}

/* Drop every packet still sitting in ESB's TX FIFO and resync all of the
 * per-send bookkeeping that lives in lockstep with TX events.
 *
 * esb_flush_tx() never fires the matching TX_SUCCESS / TX_FAILED for the
 * packets it drops, so the ring tails (noack, critical-override, motion)
 * stay behind their head, m_pointer_inflight_count stays inflated, and
 * any per-send retransmit_count override that a flushed packet had
 * applied is never restored — the radio would silently keep using the
 * override for every subsequent send. Every caller that flushes queued
 * traffic must therefore go through this wrapper so the next TX event
 * pops the right slot and the radio is back at the steady-state count.
 *
 * The motion refund pool is intentionally preserved: it represents
 * already-failed deltas that the next mouse send legitimately wants to
 * pick up. set_channel wipes it explicitly afterwards because cross-
 * channel motion is stale by the time the new channel is live.
 */
static void tx_bookkeeping_resync(void) {
#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_HOP)
    m_recent_noack_tail = m_recent_noack_head;
#endif
#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_CRITICAL_MAX_RETRANSMIT)
    m_recent_critical_tail = m_recent_critical_head;
#endif
    /* The flushed packets die undelivered exactly like retry-exhausted
     * ones, but their TX events never fire — harvest the HID kinds of
     * every outstanding ring slot before the tail snaps forward so the
     * owning layers re-send current state. Without this, a FIFO-full
     * flush silently eats queued key/button edges (a flushed press
     * followed by a delivered release is a keystroke the host never
     * sees). Iteration is bounded to the ring depth: if head has
     * outrun the ring (pushes without matching events), the oldest
     * slots were overwritten and only the newest RING_SIZE are real. */
    {
        uint32_t n = m_recent_motion_head - m_recent_motion_tail;
        if (n > CONFIG_ZMK_ESB_ENDPOINT_POINTER_MOTION_RING_SIZE) {
            n = CONFIG_ZMK_ESB_ENDPOINT_POINTER_MOTION_RING_SIZE;
        }
        uint8_t kinds = 0;
        for (uint32_t i = 0; i < n; i++) {
            const uint32_t slot = (m_recent_motion_head - n + i) %
                                  CONFIG_ZMK_ESB_ENDPOINT_POINTER_MOTION_RING_SIZE;
            const uint8_t k = m_recent_motion[slot].hid_kind;
            if (k != ESB_HID_KIND_NONE) {
                kinds |= (uint8_t)BIT(k);
            }
        }
        if (kinds != 0) {
            hid_resync_request(kinds);
        }
    }
    m_recent_motion_tail = m_recent_motion_head;
    m_pointer_inflight_count = 0;
    restore_retransmit_count();
}

static void esb_flush_tx_and_reset(void) {
    esb_flush_tx();
    tx_bookkeeping_resync();
}

/* Counterpart of esb_stash_tx() for the paths that park the FIFO across a
 * temporary disruption (channel hop, rendezvous side-trip) instead of
 * dropping it. Stashed packets keep their ring slots, inflight count and
 * retransmit-count overrides — their TX events still arrive once the queue
 * is restored — so unlike a flush, NO bookkeeping resync happens here.
 * The -ENOMEM fallback (FIFO refilled between stash and restore; cannot
 * happen under the k_sched_lock the callers hold, but stay defensive)
 * degrades to flush semantics: the stash is abandoned and
 * the bookkeeping
 * resynced as if the packets had been dropped. */
static void esb_restore_tx_or_drop(void) {
    if (esb_restore_tx() < 0) {
        tx_bookkeeping_resync();
    }
}

void esb_transport_flush_tx(void) {
    /* Don't yank the FIFO mid-burst. Wait until the radio has been
     * silent (no TX event) for FLUSH_QUIET_MS, or FLUSH_FORCE_MS total —
     * whichever comes first — so a packet that was about to ack gets the
     * chance to complete normally instead of being dropped. The wait is
     * bounded and the only callers (send_idle_packet, future external
     * flushers) run on the system workqueue where k_sleep is fine; ISR-
     * context flushers in this file go straight through the helper. */
#if CONFIG_ZMK_ESB_ENDPOINT_FLUSH_QUIET_MS > 0
    const uint32_t start = k_uptime_get_32();
    while (true) {
        const uint32_t now = k_uptime_get_32();
        const uint32_t since_evt = (m_last_tx_event_ms == 0)
            ? UINT32_MAX
            : (uint32_t)(now - m_last_tx_event_ms);
        if (since_evt >= CONFIG_ZMK_ESB_ENDPOINT_FLUSH_QUIET_MS) {
            break;
        }
        if ((uint32_t)(now - start) >= CONFIG_ZMK_ESB_ENDPOINT_FLUSH_FORCE_MS) {
            LOG_DBG("flush_tx: forcing after %u ms (no quiet window)",
                    (uint32_t)(now - start));
            break;
        }
        k_sleep(K_MSEC(1));
    }
#endif
    k_sched_lock();
    esb_flush_tx_and_reset();
    k_sched_unlock();
}

static void zmk_esb_transport_evt_cb(struct esb_evt const *event) {
    if (event->evt_id == ESB_EVENT_TX_SUCCESS ||
        event->evt_id == ESB_EVENT_TX_FAILED) {
        const uint32_t now = k_uptime_get_32();
        m_last_tx_event_ms = (now == 0) ? 1 : now;
    }
    switch (event->evt_id) {
    case ESB_EVENT_TX_SUCCESS:
#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_RENDEZVOUS)
        if (m_sync_tx_in_progress) {
            /* This event belongs to esb_transport_send_blocking() — it
             * sent on a different channel and the regular bookkeeping
             * (consecutive_tx_fail, hop notifications, shell relay,
             * bench) would all reach the wrong conclusions about the
             * active channel's health. Hand the result to the waiter
             * and stop. */
            m_sync_tx_result_success = true;
            m_sync_tx_in_progress = false;
            k_sem_give(&m_sync_tx_done);
            break;
        }
#endif
        tx_ok_count++;
        if (event->tx_attempts > 1) {
            tx_retried_count++;
        }
        consecutive_tx_fail = 0;
#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_HOP)
        m_first_fail_ms = 0;
        {
            const uint32_t now_ok = k_uptime_get_32();
            m_last_tx_success_ms = (now_ok == 0) ? 1 : now_ok;
        }
        channel_hop_ep_on_tx_success_isr();
        /* Link-quality metric. Skip the slot if the matching send was
         * noack — those events report tx_attempts=1 unconditionally
         * and would dilute the signal. The hop_*-cooldown tracking is
         * separate, so this metric stays live across hops; the
         * window itself is reset by set_channel so a hop wipes
         * stale history. */
        {
            const bool was_noack = consume_recent_noack();
            if (!was_noack) {
                const bool retry = (event->tx_attempts > 1);
                link_quality_record((uint8_t)event->tx_attempts, retry);

#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_COOP_HOP)
                /* Threshold check + hysteresis re-arm. Wide window
                 * fires the trigger; fast-path fires on a sudden
                 * cliff in the last 8 events. */
                if (m_link_degraded_armed) {
                    if (m_link_quality_count <=
                        CONFIG_ZMK_ESB_ENDPOINT_LINK_DEGRADED_REARM) {
                        m_link_degraded_armed = false;
                    }
                } else {
                    bool fire = false;
                    if (m_link_quality_count >=
                        CONFIG_ZMK_ESB_ENDPOINT_LINK_DEGRADED_THRESHOLD) {
                        fire = true;
                    } else {
                        const uint8_t fast = (uint8_t)__builtin_popcount(
                            m_link_quality_window & 0xFFu);
                        if (fast >=
                            CONFIG_ZMK_ESB_ENDPOINT_LINK_DEGRADED_FAST_THRESHOLD) {
                            fire = true;
                        }
                    }
                    if (fire) {
                        m_link_degraded_armed = true;
                        channel_hop_ep_on_link_degraded_isr();
                    }
                }
#endif /* COOP_HOP */
            }
        }
#endif
#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_SHELL_RELAY)
        esb_shell_relay_notify_tx();
#endif
#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_BENCH)
        esb_bench_notify_tx_success();
#endif
#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_CRITICAL_MAX_RETRANSMIT)
        if (consume_recent_override()) {
            restore_retransmit_count();
        }
#endif
        motion_ring_consume(false);
        break;

    case ESB_EVENT_RX_RECEIVED:
        while (esb_read_rx_payload(&m_rx_payload) == 0) {
            if (m_rx_payload.length > 0 &&
                m_rx_payload.length <= ESB_MAX_PAYLOAD_LEN) {
                const uint8_t slot = rx_active ^ 1U;
                rx_len[slot] = m_rx_payload.length;
                memcpy(rx_buf[slot], m_rx_payload.data, rx_len[slot]);
                rx_active = slot;
                k_work_submit(&rx_work);
            }
        }
        break;
    case ESB_EVENT_TX_FAILED:
#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_RENDEZVOUS)
        if (m_sync_tx_in_progress) {
            /* See TX_SUCCESS branch — this fail belongs to the side-trip,
             * not the active channel. Skip all hop / counter machinery so
             * a missed rendezvous PROPOSAL does not register as a failure
             * on the channel we'll be back on in microseconds. */
            m_sync_tx_result_success = false;
            m_sync_tx_in_progress = false;
            k_sem_give(&m_sync_tx_done);
            /* Bare flush, deliberately without the bookkeeping resync:
             * the only thing in the FIFO is the failed sync payload
             * (send_blocking stashed the user queue before writing it),
             * and it never pushed ring entries. A resync here would
             * desynchronise the rings from the stashed packets that
             * send_blocking is about to restore. */
            esb_flush_tx();
            break;
        }
#endif
        tx_fail_count++;
        tx_exhausted_count++;
        consecutive_tx_fail++;
        if (consecutive_tx_fail == CONSECUTIVE_WARN_THRESHOLD) {
            LOG_WRN("%u consecutive TX failures (ok=%u)", consecutive_tx_fail, tx_ok_count);
        } else if (consecutive_tx_fail == CONSECUTIVE_ERR_THRESHOLD) {
            LOG_ERR("%u consecutive TX failures (ok=%u)", consecutive_tx_fail, tx_ok_count);
        }
#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_HOP)
        /* Time-window hop trigger. First failure in a streak stamps the
         * starting uptime; subsequent failures check elapsed time against
         * the configured window. After each trigger we re-stamp the
         * timestamp so the next call is gated by another full WINDOW_MS
         * of continuous failures — this rate-limits the ISR→work-submit
         * path AND gives the hopper's cooldown a chance to expire between
         * attempts. A plain latch here would deadlock: if the hopper
         * early-returns while cooldown is active (by design, to let the
         * peer catch up on the new channel), a latch would stay set
         * forever because TX_SUCCESS never arrives on a dead link, so
         * no later streak could re-trigger. */
        if (consecutive_tx_fail == 1) {
            m_first_fail_ms = k_uptime_get_32();
            /* Ensure nonzero so the "set" check below is unambiguous even
             * on the rare boot where uptime happens to be 0 at this moment. */
            if (m_first_fail_ms == 0) {
                m_first_fail_ms = 1;
            }
        } else if (m_first_fail_ms != 0) {
            const uint32_t elapsed = k_uptime_get_32() - m_first_fail_ms;
            if (elapsed >= CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_HOP_TX_FAIL_WINDOW_MS) {
                channel_hop_ep_on_tx_fail_isr();
                /* Restart the window. If the hop succeeded, set_channel
                 * will overwrite m_first_fail_ms = 0 before we get back
                 * here. If the hop was refused (cooldown active), the
                 * restart spaces the next attempt WINDOW_MS into the
                 * future so we are not spamming the cooldown-check in
                 * the ISR at user-TX rate. */
                m_first_fail_ms = k_uptime_get_32();
                if (m_first_fail_ms == 0) {
                    m_first_fail_ms = 1;
                }
            }
        }
#if CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_HOP_WEAK_LINK_MS > 0
        /* Weak-link trigger — independent of the fail-window above, so
         * it survives intermittent successes that keep resetting
         * consecutive_tx_fail and m_first_fail_ms (which is precisely
         * the case the fail-window cannot detect). Runs as a parallel
         * `if` rather than an `else if` because once a fail streak has
         * started, m_first_fail_ms is always non-zero and would
         * unconditionally claim the chain — leaving this branch dead.
         * If both the fail-window AND the weak-link condition fire on
         * the same TX_FAILED, k_work_submit is idempotent: the work
         * gets queued at most once. After firing, restamp
         * m_last_tx_success_ms so the next fire is gated by another
         * full WEAK_LINK_MS of continued no-success — same rate-limit
         * pattern as m_first_fail_ms restamping above. Pretending
         * "success now" is the right abstraction: the variable's role
         * here is "deadline anchor for the weak-link check," and a
         * real TX_SUCCESS overwrites this within one packet if the
         * hop actually fixed the link. */
        if (m_last_tx_success_ms != 0) {
            const uint32_t since_ok =
                    k_uptime_get_32() - m_last_tx_success_ms;
            if (since_ok >= CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_HOP_WEAK_LINK_MS) {
                channel_hop_ep_on_tx_fail_isr();
                const uint32_t now_w = k_uptime_get_32();
                m_last_tx_success_ms = (now_w == 0) ? 1 : now_w;
            }
        }
#endif
        /* TX_FAILED feeds only the popcount window, not the EWMA:
         * definite retry exhaustion is a stronger "1" than a single
         * retransmit and keeps coop-hop's "how bad is it" view honest,
         * but the EWMA is the avg-tx_attempts-on-delivered-packets
         * signal that drives adaptive_retry, and a retransmit_count+1
         * spike would dominate it. The TX-fail trigger remains the real
         * escalation path. We still consume the noack slot to keep ring
         * head/tail in sync. */
        {
            (void)consume_recent_noack();
            link_quality_shift_window(true);
            /* Don't fire the coop-hop trigger from the fail branch:
             * TX-fail-window / weak-link triggers already own the
             * "really bad" path, and double-firing would race a
             * speculative hop against a coop hop. */
        }
#endif
#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_SHELL_RELAY)
        esb_shell_relay_notify_tx();
#endif
#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_BENCH)
        esb_bench_notify_tx_fail();
#endif
#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_CRITICAL_MAX_RETRANSMIT)
        if (consume_recent_override()) {
            restore_retransmit_count();
        }
#endif
        motion_ring_consume(true);
        if (esb_tx_full()) {
            esb_flush_tx_and_reset();
        }

        break;
    }
}

static int esb_init_and_configure(void) {
    struct esb_config cfg = ESB_DEFAULT_CONFIG;
    cfg.protocol           = ESB_PROTOCOL_ESB_DPL;
    cfg.mode               = ESB_MODE_PTX;
    cfg.bitrate            = ESB_BITRATE_1MBPS_BLE;
    cfg.crc                = ESB_CRC_16BIT;
    cfg.retransmit_count   = CONFIG_ZMK_ESB_ENDPOINT_RETRANSMIT_COUNT;
    cfg.retransmit_delay   = CONFIG_ZMK_ESB_ENDPOINT_RETRANSMIT_DELAY_US;
    cfg.tx_mode            = ESB_TXMODE_AUTO;
    cfg.use_fast_ramp_up   = true;
    cfg.tx_output_power    = ESB_TX_POWER_8DBM;
    cfg.selective_auto_ack = IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_HID_NOACK);
    cfg.event_handler      = zmk_esb_transport_evt_cb;
    cfg.payload_length     = ESB_MAX_PAYLOAD_LEN;

    const int err = esb_init(&cfg);
    if (err) {
        LOG_ERR("esb_init failed: %d", err);
        return err;
    }

    if (m_addr.configured) {
        esb_set_base_address_0(m_addr.base0);
        esb_set_base_address_1(m_addr.base1);
        esb_set_prefixes(m_addr.prefixes, 2);
        esb_set_rf_channel(m_addr.channel);
    }

    return 0;
}

int esb_transport_init(const esb_transport_cb_t cb) {
    m_cb = cb;
    k_work_init(&rx_work, rx_work_fn);
    k_work_init_delayable(&m_hid_resync_work, hid_resync_work_fn);
#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_RENDEZVOUS)
    k_sem_init(&m_sync_tx_done, 0, 1);
#endif
    radio_arbiter_init();
#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_HOP)
    /* Seed the EWMA at "perfect link" (1.0 attempts × 10) so the shell
     * snapshot reports a sensible value before any TX has happened. */
    m_link_quality_ewma_x10 = 10;
#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_ADAPTIVE_RETRY)
    k_work_init_delayable(&m_adaptive_retry_work, adaptive_retry_work_fn);
#endif
#endif
    return 0;
}

void esb_transport_set_addresses(const uint8_t base0[4], const uint8_t base1[4], const uint8_t prefixes[8], const uint8_t channel) {
    memcpy(m_addr.base0, base0, 4);
    memcpy(m_addr.base1, base1, 4);
    memcpy(m_addr.prefixes, prefixes, 8);
    m_addr.channel = channel;
    m_addr.boot_channel = channel;
    m_addr.configured = true;
}

uint8_t esb_transport_get_channel(void) {
    return m_addr.channel;
}

uint8_t esb_transport_get_rendezvous_channel(void) {
    return m_addr.boot_channel;
}

int esb_transport_set_channel(const uint8_t channel) {
    if (!m_addr.configured) {
        return -ENODEV;
    }
    /* The flush → (stop) → retune → bookkeeping sequence must not
     * interleave with a concurrent esb_transport_send from another
     * thread, which would re-arm the radio on the old channel mid-hop. */
    k_sched_lock();

    /* Force an LFRC calibration on every hop. The persistent HFXO hold
     * (see radio_arbiter_prepare) keeps the 32M xtal running, but the LFRC itself
     * still drifts with temperature between the calibrator's 8s hw-cal
     * windows. A hop is almost always preceded by a streak of TX failures,
     * and stale LFRC timing is one plausible contributor — run the cal
     * here so we start the new channel with freshly-trimmed LF timing.
     * force_start is async and no-ops if a cal is already running; it
     * does not block the hop. */
    z_nrf_clock_calibration_force_start();

    /* Park any queued packets instead of dropping them. Retransmits on
     * the new channel would arrive at a peer that has not yet hopped, so
     * the FIFO must be empty while we retune — but the packets themselves
     * (queued HID reports, shell bytes) are still worth delivering once
     * the peer has followed. The stash keeps them, PIDs included; they go
     * back into the FIFO below and leave the radio when the first send
     * after the post-hop quiet window kicks start_tx. Because the packets
     * are restored (their TX events still arrive), the noack/critical/
     * motion rings, the inflight count and any retransmit-count override
     * are deliberately left alone — resyncing them here (as the old flush
     * did) would desynchronise the rings from the restored queue. */
    (void)esb_stash_tx();

    /* esb_set_rf_channel() requires ESB_STATE_IDLE. With
     * CONFIG_ESB_NEVER_DISABLE_TX the radio can stay hot (PTX_TXIDLE /
     * mid TX-ACK) indefinitely under sustained traffic, so -EBUSY may
     * never clear on its own. Stopping TX outright takes time and aborts
     * the in-flight cycle, so let the hop-retry worker wait out transient
     * busyness first and force the radio idle only once the busy streak
     * shows it is not settling. */
    int err = esb_set_rf_channel(channel);
#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_HOP)
    if (err == -EBUSY &&
        ++m_set_channel_busy_streak >= CONFIG_ZMK_ESB_ENDPOINT_HOP_STOP_TX_AFTER_BUSY) {
        esb_stop_tx();
        err = esb_set_rf_channel(channel);
    }
    if (err == 0) {
        m_set_channel_busy_streak = 0;
    }
#endif
    if (err) {
        /* Retune failed — the radio is still on the old channel, where
         * the stashed packets remain valid. Put them back. */
        esb_restore_tx_or_drop();
        k_sched_unlock();
        return err;
    }
    m_addr.channel = channel;
    /* Re-queue the parked packets on the new channel. Passive: restore
     * does not kick the radio, so nothing transmits until the first
     * send after the quiet window below calls esb_write_payload, which
     * starts TX with the restored packets at the front of the queue. */
    esb_restore_tx_or_drop();
    consecutive_tx_fail = 0;
    /* Peer's RSSI view is per-link, not per-channel — the peer samples
     * over its own radio which now has to re-establish contact. Invalidate
     * so consumers don't act on stale values during the quiet window. */
    m_peer_rssi_valid = false;
#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_HOP)
    m_first_fail_ms = 0;
    /* New channel — no evidence yet that anything is alive here. The
     * weak-link trigger's != 0 gate keeps it quiet until a real
     * TX_SUCCESS re-arms the clock. */
    m_last_tx_success_ms = 0;
    /* Wipe link-quality history: the new channel deserves a fresh
     * scoreboard, and bits carried over from a degraded old channel
     * would re-trigger coop hop instantly. EWMA reset to 1.0×10. */
    m_link_quality_window = 0;
    m_link_quality_count = 0;
    m_link_quality_ewma_x10 = 10;
#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_COOP_HOP)
    m_link_degraded_armed = false;
#endif
    /* Cross-channel motion is stale by the time the new channel is live
     * (post-quiet window + peer catch-up), so additionally drop any
     * pending pointer refund — adding it to the next mouse report would
     * teleport the cursor. Matches the input processor's accumulator
     * clear at the same moment. The motion ring and inflight count are
     * left intact — the stashed packets were restored above and their
     * TX events will consume those slots; only the refund pool (already-
     * failed motion) is dropped on the channel-change path. */
    m_motion_refund.dx = 0;
    m_motion_refund.dy = 0;
    m_motion_refund.scroll_x = 0;
    m_motion_refund.scroll_y = 0;
    m_motion_refund.stamp_ms = 0;
    /* Arm the post-hop quiet window. Any send() during this interval is
     * silently dropped — the peer may still be on the old channel and
     * anything we transmit would just fail and feed the next hop trigger.
     * Saturating add: the deadline stays in uint32 wrap-safe range for
     * every realistic quiet value. */
    const uint32_t quiet_until =
        k_uptime_get_32() + CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_HOP_POST_QUIET_MS;
    m_tx_quiet_until_ms = (quiet_until == 0) ? 1 : quiet_until;
#endif
    k_sched_unlock();
    return 0;
}

void esb_transport_reset_consecutive_fail(void) {
    consecutive_tx_fail = 0;
}

void esb_transport_on_rx_link_stats(const uint8_t *data, const uint8_t len) {
    if (len < sizeof(struct esb_pkt_link_stats)) {
        return;
    }
    const struct esb_pkt_link_stats *s = (const void *)data;
    m_peer_rssi_last = s->rssi_last;
    m_peer_rssi_ewma = s->rssi_ewma;
    m_peer_rssi_valid = true;
#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_RSSI_WEIGHT)
    channel_hop_ep_note_peer_rssi(s->rssi_ewma);
#endif
}

bool esb_transport_get_peer_rssi(int8_t *last_out, int8_t *ewma_out) {
    if (!m_peer_rssi_valid) {
        return false;
    }
    if (last_out) {
        *last_out = m_peer_rssi_last;
    }
    if (ewma_out) {
        *ewma_out = m_peer_rssi_ewma;
    }
    return true;
}

bool esb_transport_is_quiet(void) {
#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_RENDEZVOUS)
    if (m_sync_tx_in_progress) {
        return true;
    }
#endif
#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_HOP)
    if (m_tx_quiet_until_ms != 0) {
        const uint32_t now = k_uptime_get_32();
        if ((int32_t)(m_tx_quiet_until_ms - now) > 0) {
            return true;
        }
    }
#endif
    return false;
}

bool esb_transport_shell_backpressure(void) {
    return esb_tx_count() >= (CONFIG_ESB_TX_FIFO_SIZE / 2);
}

bool esb_transport_pointer_backpressure(void) {
    return m_pointer_inflight_count >=
        CONFIG_ZMK_ESB_ENDPOINT_POINTER_INFLIGHT_CAP;
}

void esb_transport_get_per_stats(uint32_t *ok_out, uint32_t *retried_out,
                                 uint32_t *exhausted_out) {
    if (ok_out) {
        *ok_out = tx_ok_count;
    }
    if (retried_out) {
        *retried_out = tx_retried_count;
    }
    if (exhausted_out) {
        *exhausted_out = tx_exhausted_count;
    }
}

void esb_transport_get_hid_tx_stats(uint32_t *count_out, uint16_t *rate_hz_out) {
    if (count_out) {
        *count_out = m_hid_tx_count;
    }
    if (rate_hz_out) {
        const uint32_t now = k_uptime_get_32();
        if (m_hid_tx_bucket_ms == 0 ||
            (uint32_t)(now - m_hid_tx_bucket_ms) > HID_RATE_STALE_MS) {
            *rate_hz_out = 0;
        } else {
            *rate_hz_out = m_hid_tx_rate_hz;
        }
    }
}

int esb_transport_get_link_quality(uint8_t *retried_out, uint16_t *ewma_x10_out) {
#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_HOP)
    if (retried_out) {
        *retried_out = m_link_quality_count;
    }
    if (ewma_x10_out) {
        *ewma_x10_out = m_link_quality_ewma_x10;
    }
#else
    if (retried_out) {
        *retried_out = 0;
    }
    if (ewma_x10_out) {
        *ewma_x10_out = 10;
    }
#endif
    return 0;
}

void esb_transport_deinit(void) {
    m_addr.configured = false;
}

int esb_transport_send(const uint8_t pipe, const uint8_t *data, uint8_t len) {
    if (len > ESB_MAX_PAYLOAD_LEN) {
        return -EMSGSIZE;
    }

    const bool is_shell = (len >= 1) &&
        (data[0] == ESB_PKT_SHELL_DATA || data[0] == ESB_PKT_SHELL_POLL || data[0] == ESB_PKT_SHELL_STOP);

#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_RENDEZVOUS)
    /* Refuse any user-thread send while a synchronous side-trip is on
     * the radio. The rendezvous send temporarily flips RADIO->FREQUENCY
     * to another channel; queueing a user packet here would let the ESB
     * state machine TX it on the wrong channel. The window is bounded
     * by RENDEZVOUS_TIMEOUT_MS (a few ms). -EAGAIN (not fake success)
     * so edge-carrying callers can park the report in their pending
     * queues — this return is the authoritative version of the
     * advisory esb_transport_is_quiet() pre-check. */
    if (m_sync_tx_in_progress) {
        return -EAGAIN;
    }
#endif

    /* Sends arrive from multiple thread contexts (input thread, system
     * workqueue); the section below is a multi-step low-level ESB
     * sequence (retransmit knobs, ring pushes, FIFO write) that must not
     * interleave between threads. ISRs keep running — the vendored
     * library irq-locks its own hardware-critical parts. */
    k_sched_lock();

    /* Jitter the retransmit delay per-send. De-correlates retries from
     * periodic 2.4 GHz interferers (WiFi beacons, BLE adv) that might
     * happen to line up with the fixed base delay. Cheap — one xorshift
     * step + one register write. esb_set_retransmit_delay stores the
     * value for the radio state machine to pick up on the next TX;
     * safe from thread context. */
    (void)esb_set_retransmit_delay(jittered_retransmit_delay());

#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_HOP)
    /* Post-hop quiet window: the peer needs a moment to follow us to
     * the new channel. Refuse user traffic until the deadline — 32-bit
     * wrap-safe compare. Not queueing for CHANNEL_HOP_POST_QUIET_MS is
     * strictly better than transmitting to a channel the receiver
     * hasn't reached yet, where every attempt would just count as a
     * failure. -EAGAIN (not fake success): this check runs under the
     * same k_sched_lock that arms the window, so it is the
     * authoritative answer for callers whose advisory is_quiet()
     * pre-check raced the hop worker — they re-enqueue the report
     * instead of losing the edge. */
    if (m_tx_quiet_until_ms != 0) {
        const uint32_t now = k_uptime_get_32();
        if ((int32_t)(m_tx_quiet_until_ms - now) > 0) {
            k_sched_unlock();
            return -EAGAIN;
        }
        m_tx_quiet_until_ms = 0;
    }
#endif

#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_HID_NOACK)
    /* Only mouse reports may skip ACKs: pointer/scroll is a self-correcting stream,
     * a dropped frame costs at most a few ticks of motion. Keyboard/consumer reports
     * carry edge-triggered state — a lost release packet strands the key on the host
     * forever (re-sending the same "released" state produces no HID edge). Keep them
     * ACKed so ESB retransmits if the peer misses one. */
    const bool noack = (pipe == 1) && (len >= 2) &&
                       (data[0] == ESB_PKT_HID_REPORT) &&
                       (data[1] == ESB_REPORT_MOUSE);
#else
    const bool noack = false;
#endif
    struct esb_payload pkt = {
        .pipe   = pipe,
        .length = len,
        .noack  = noack ? 1U : 0U,
    };
    memcpy(pkt.data, data, len);

    /* Fail fast before any ring/refund bookkeeping when the FIFO is full
     * and this is a shell send. The non-shell path flushes (which resets
     * the rings), so esb_write_payload below always succeeds. The shell
     * path intentionally does not flush — flushing would drop pending
     * pointer reports and other queued shell bytes. Pushing to the
     * noack/critical/motion rings and then losing the packet to -ENOMEM
     * leaks a phantom zero slot in each ring; the next real-pointer
     * TX event then consumes the phantom and the real pointer record
     * stays stranded, never decrementing m_pointer_inflight_count.
     * Returning early before the pushes keeps head/tail in lockstep
     * with TX events; the shell relay drain loop already handles
     * -ENOMEM by re-queueing the unsent bytes. */
    if (esb_tx_full()) {
        if (is_shell) {
            k_sched_unlock();
            return -ENOMEM;
        }
        esb_flush_tx_and_reset();
    }

#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_HOP)
    /* Stamp the noack flag of this send into the link-quality ring so
     * the matching TX event handler can decide whether to count it.
     * The flush above can drop unsent packets; we accept that as one
     * skipped-but-recorded ring slot (the next event still consumes
     * the next tail entry, which now corresponds to the post-flush
     * send — at worst we mis-attribute one packet's noack-ness, which
     * the wide window absorbs). */
    m_recent_noack[m_recent_noack_head & (CONFIG_ZMK_ESB_ENDPOINT_LINK_QUALITY_NOACK_RING_SIZE - 1)] = noack ? 1U : 0U;
    m_recent_noack_head++;
#endif

    const bool is_bg_poll = (len >= 1) && (data[0] == ESB_PKT_SHELL_BG_POLL);
    if (!is_bg_poll) {
        note_activity();
    }

    if (len >= 1 && data[0] == ESB_PKT_HID_REPORT) {
        m_hid_tx_count++;
        m_hid_tx_bucket_count++;
        const uint32_t now = k_uptime_get_32();
        if (m_hid_tx_bucket_ms == 0) {
            m_hid_tx_bucket_ms = now;
        } else if ((uint32_t)(now - m_hid_tx_bucket_ms) >= HID_RATE_BUCKET_MS) {
            m_hid_tx_rate_hz = (m_hid_tx_bucket_count > UINT16_MAX)
                ? UINT16_MAX : (uint16_t)m_hid_tx_bucket_count;
            m_hid_tx_bucket_count = 0;
            m_hid_tx_bucket_ms = now;
        }
    }

#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_HOP)
    /* Anything that represents real user traffic keeps the endpoint in
     * active state. Housekeeping packets (BEACON, VERIFY_REQ, PROPOSAL,
     * IDLE itself, background polls) do not — otherwise we would never
     * settle into idle. */
    if (len >= 1) {
        const uint8_t type = data[0];
        const bool is_user_traffic =
            type == ESB_PKT_HID_REPORT ||
            type == ESB_PKT_SHELL_DATA ||
            type == ESB_PKT_SHELL_POLL ||
            type == ESB_PKT_SHELL_START ||
            type == ESB_PKT_SHELL_STOP;
        if (is_user_traffic) {
            channel_hop_ep_note_user_tx();
        }
    }
#endif

#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_CRITICAL_MAX_RETRANSMIT)
    /* Critical-packet retransmit override. For pairing, control, hop
     * coordination, and silence packets, push the radio to the hardware
     * maximum so a single dropped frame can't strand pairing or leave
     * the link on a degraded channel. The matching TX event in
     * zmk_esb_transport_evt_cb consumes the ring tail and restores the
     * count. The push runs every send (override or not) so head/tail
     * stay aligned with the TX events. */
    const bool critical_override = (len >= 1) && (
        data[0] == ESB_PKT_BEACON               ||
        data[0] == ESB_PKT_PAIR_RESP            ||
        data[0] == ESB_PKT_DISCONNECT           ||
        data[0] == ESB_PKT_VERIFY_REQ           ||
        data[0] == ESB_PKT_CHANNEL_HOP_PROPOSAL ||
        data[0] == ESB_PKT_HOP_OFFER            ||
        data[0] == ESB_PKT_IDLE);
    m_recent_critical_override[m_recent_critical_head & (CONFIG_ZMK_ESB_ENDPOINT_CRIT_OVERRIDE_RING_SIZE - 1)] =
        critical_override ? 1U : 0U;
    m_recent_critical_head++;
    if (critical_override) {
        /* Best effort: -EBUSY just means another TX is mid-cycle and
         * already locked in its retransmit_count. Our payload sits in
         * the queue; ESB picks up the new count at the next IDLE
         * transition before pulling our entry. */
        (void)esb_set_retransmit_count(
            CONFIG_ZMK_ESB_ENDPOINT_CRITICAL_RETRANSMIT_COUNT);
    }
#endif

    /* Pointer/scroll handling: apply any pending refund (from a previous
     * mouse motion TX_FAILED) to this packet's d_x/d_y/d_scroll_*
     * saturating to int16, push the post-refund deltas to the motion
     * ring so a TX_FAILED on this send can re-enter them, and lower the
     * radio's retransmit_count for motion-only sends so failures land
     * quickly and the refund pool stays current. Mouse reports that
     * carry any button bit keep the global retransmit count — buttons
     * are edge-triggered. Non-mouse sends still push a zero record so
     * the ring head/tail stay aligned with the TX event stream. The
     * pointer override is mutually exclusive with critical_override
     * (no critical packet is a mouse report) so the order with the
     * critical block above is moot. The override is also a no-op when
     * the send is noack (HID_NOACK=y): noack TX events are not retried
     * by the radio at all, but esb_set_retransmit_count is still cheap
     * and the field stays consistent for the next ACKed send. */
    /* Classify HID payloads so a delivery failure (retry exhaustion or
     * FIFO flush) can be routed back to the layer owning the report
     * state. Noack mouse sends stay NONE: the radio never reports a
     * failure for them, so a resync request could never fire anyway,
     * and flush-harvesting them would only produce spurious (if
     * idempotent) resyncs for a stream that self-corrects. */
    uint8_t hid_kind = ESB_HID_KIND_NONE;
    if (len >= 2 && data[0] == ESB_PKT_HID_REPORT && !noack) {
        switch (data[1]) {
        case ESB_REPORT_KEYBOARD: hid_kind = ESB_HID_KIND_KB;    break;
        case ESB_REPORT_CONSUMER: hid_kind = ESB_HID_KIND_CONS;  break;
        case ESB_REPORT_MOUSE:    hid_kind = ESB_HID_KIND_MOUSE; break;
        default: break;
        }
    }

    bool pointer_override = false;
    if (is_mouse_report(data, len)) {
        struct esb_mouse_body *body = mouse_body_of(pkt.data);
        motion_refund_apply(body);
        const bool has_motion =
            (body->d_x | body->d_y | body->d_scroll_x | body->d_scroll_y) != 0;
        pointer_override = has_motion && (body->buttons == 0) && !noack;
        if (pointer_override) {
            (void)esb_set_retransmit_count(
                CONFIG_ZMK_ESB_ENDPOINT_POINTER_RETRANSMIT_COUNT);
        }
        motion_ring_push(body, pointer_override, hid_kind);
    } else {
        motion_ring_push(NULL, false, hid_kind);
    }

    const int ret = esb_write_payload(&pkt);
    k_sched_unlock();
    return ret;
}

#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_RENDEZVOUS)
int esb_transport_send_blocking(const uint8_t channel, const uint8_t pipe,
                                const uint8_t *data, const uint8_t len,
                                const k_timeout_t timeout) {
    if (len > ESB_MAX_PAYLOAD_LEN) {
        return -EMSGSIZE;
    }
    if (!m_addr.configured) {
        return -ENODEV;
    }
    if (m_sync_tx_in_progress) {
        return -EBUSY;
    }

    /* esb_set_rf_channel only updates an internal struct field; the radio
     * picks up the new frequency on the next ramp-up. So a stash_tx +
     * channel_set + write_payload sequence guarantees that the next packet
     * to leave is ours, on the channel we just set. After completion the
     * inverse restore returns the radio to the active channel — and puts
     * the stashed user packets back in the queue — for the very next
     * user-thread send (which we held off via m_sync_tx_in_progress
     * during the round trip). m_addr.channel is left untouched throughout —
     * we are visiting, not committing. The setup and restore sections run
     * with preemption disabled so no other thread can slip a send onto the
     * transient channel; the lock is dropped around the semaphore wait.
     *
     * The stash keeps the parked packets' ring slots / inflight count /
     * retransmit override live (their TX events still arrive after the
     * restore), and the sync packet itself never touches the rings (both
     * ISR branches bail out on m_sync_tx_in_progress before the ring
     * consumption), so no bookkeeping resync happens anywhere on this
     * round trip. */
    k_sched_lock();
    (void)esb_stash_tx();

    const int chan_err = esb_set_rf_channel(channel);
    if (chan_err) {
        esb_restore_tx_or_drop();
        k_sched_unlock();
        return chan_err;
    }

    k_sem_reset(&m_sync_tx_done);
    m_sync_tx_result_success = false;
    m_sync_tx_in_progress = true;

    struct esb_payload pkt = {
        .pipe   = pipe,
        .length = len,
        .noack  = 0,
    };
    memcpy(pkt.data, data, len);

    const int werr = esb_write_payload(&pkt);
    if (werr) {
        m_sync_tx_in_progress = false;
        (void)esb_set_rf_channel(m_addr.channel);
        esb_restore_tx_or_drop();
        (void)esb_start_tx();
        k_sched_unlock();
        return werr;
    }
    k_sched_unlock();

    const int wait_err = k_sem_take(&m_sync_tx_done, timeout);

    /* Always restore the active channel, even on timeout — leaving the
     * radio on the rendezvous channel would silently break every
     * subsequent user TX. The flag is cleared by the ISR on completion;
     * on timeout we clear it here so the next caller is unblocked. */
    k_sched_lock();
    (void)esb_set_rf_channel(m_addr.channel);

    /* Put the parked user packets back and kick the radio so they leave
     * on the active channel now, not at the next user send. On timeout
     * the sync payload may still occupy the FIFO front mid-retransmit —
     * esb_start_tx then reports -EBUSY and the restored packets ride out
     * behind it once the radio settles; both benign. */
    esb_restore_tx_or_drop();
    (void)esb_start_tx();

    if (wait_err) {
        m_sync_tx_in_progress = false;
        k_sched_unlock();
        return -ETIMEDOUT;
    }
    k_sched_unlock();
    return m_sync_tx_result_success ? 0 : -EIO;
}
#endif /* RENDEZVOUS */

void esb_transport_on_slot_start(void) {
    radio_arbiter_prepare();

    k_sched_lock();
    const int init_err = esb_init_and_configure();
    if (init_err) {
        k_sched_unlock();
        LOG_ERR("ESB slot start: init failed (%d)", init_err);
        return;
    }

    /* esb_init() has now populated _sw_isr_table[RADIO_IRQn] via
     * irq_connect_dynamic(); retarget the vector so RADIO IRQs reach it. */
    radio_arbiter_take();

    consecutive_tx_fail = 0;
#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_HOP) && \
    IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_ADAPTIVE_RETRY)
    /* Reset adaptive state to the default (what esb_init_and_configure
     * programmed) so the worker's cached value matches reality before
     * the first EWMA sample arrives. */
    m_adaptive_retransmit_count = CONFIG_ZMK_ESB_ENDPOINT_RETRANSMIT_COUNT;
    k_work_reschedule(&m_adaptive_retry_work, K_MSEC(CONFIG_ZMK_ESB_ENDPOINT_ADAPTIVE_RETRY_INTERVAL_MS));
#endif
    k_sched_unlock();
    LOG_DBG("ESB slot start OK");
}

void esb_transport_on_slot_stop(void) {
    LOG_DBG("ESB slot stop (ok=%u fail=%u)", tx_ok_count, tx_fail_count);
    consecutive_tx_fail = 0;
#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_HOP) && \
    IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_ADAPTIVE_RETRY)
    k_work_cancel_delayable(&m_adaptive_retry_work);
#endif

    k_sched_lock();
    esb_disable();
    k_sched_unlock();

    radio_arbiter_release();
}
