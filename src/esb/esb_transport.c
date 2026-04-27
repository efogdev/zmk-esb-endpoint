/*
 * Copyright (c) 2025 efogdev
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/irq.h>
#include <zephyr/arch/arm/irq.h>
#include <zephyr/sw_isr_table.h>
#include <hal/nrf_ppi.h>
#include <hal/nrf_timer.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <zephyr/sys/onoff.h>
#include <esb.h>
#include "esb_transport.h"
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

/* nRF52833: RADIO_IRQn = 1 */
#define RADIO_IRQn 1

/*
 * BT_LL_SW_SPLIT runs a TIMER0-driven scheduler with radio events wired via
 * PPI.  Even when no adv/scan/conn role is active, stray TIMER0 CC events
 * can trigger RADIO tasks through those PPI shortcuts, clobbering ESB's
 * packet/address/channel config.
 *
 * While ESB owns the radio, clear the PPI channels the LL owns so no shortcut
 * can reach RADIO.  We deliberately do NOT mask TIMER0/RTC0 IRQs: the BT host
 * still expects to drive HCI commands (adv start/stop) while ESB is active —
 * wrapping them to no-op above keeps the host's view of the controller
 * consistent, but the controller's own TIMER/RTC ISRs must keep running so
 * pending commands can be acked, else the host thread blocks forever.
 *
 * Mask covers PPI channels 6-19 (programmable, used by LL via SW split ticker
 * and RADIO enable/disable/capture), plus fixed PPIs 22/23/25 (HCTO disable,
 * AAR trigger, CRYPT trigger) — see radio_nrf5_ppi_resources.h.
 */
#define BT_LL_PPI_MASK 0x02CFFFC0u

/*
 * On Cortex-M4 with CONFIG_CPU_CORTEX_M_HAS_VTOR, Zephyr sets SCB->VTOR to
 * the flash _vector_start — there is no RAM copy.  Writes to the flash vector
 * table are silently ignored, so patching VTOR[17] (RADIO_IRQn) at runtime
 * requires us to first create a RAM copy and redirect SCB->VTOR to it.
 *
 * After the RAM table is installed we can freely swap the RADIO entry between
 * BLE LL's radio_nrf5_isr and ESB's z_arm_irq_direct_dynamic_dispatch_reschedule
 * (which dispatches to _sw_isr_table[RADIO_IRQn] set by irq_connect_dynamic).
 *
 * nRF52833: 16 system + 48 external = 64 entries, 256-byte alignment.
 */
#define NVIC_NUM_VECTORS 64
static uint32_t m_ram_vtor[NVIC_NUM_VECTORS] __aligned(256);
static bool m_ram_vtor_installed;
static uint32_t saved_radio_vector;

static esb_transport_cb_t m_cb;
static struct esb_payload m_rx_payload;

static struct {
    uint8_t base0[4];
    uint8_t base1[4];
    uint8_t prefixes[8];
    uint8_t channel;
    /* Channel the radio booted on (the DTS-default rendezvous channel).
     * The dongle's rollback dwell cycle always includes this channel, so
     * it is the one place the keyboard can reliably reach a desynced
     * dongle. Set once in set_addresses(); never overwritten by hops. */
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

static struct k_work rx_work;
static uint8_t rx_buf[ESB_MAX_PAYLOAD_LEN];
static uint8_t rx_len;

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
 * VERIFY_REQ/DISCONNECT even with channel hopping disabled. */
#define CRIT_OVERRIDE_RING_SIZE 8
static uint8_t  m_recent_critical_override[CRIT_OVERRIDE_RING_SIZE];
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

/* Post-hop quiet window deadline. Set by esb_transport_set_channel();
 * esb_transport_send() silently drops packets while now < deadline so
 * the dongle has time to follow the speculative hop before we pile on
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
 * m_link_quality_ewma_x10 is the EWMA of tx_attempts itself (×10 for
 * fixed-point), exposed via esb_transport_get_link_quality() purely
 * for shell/observability. Updated at the same time as the popcount.
 *
 * m_recent_noack[] tracks the noack flag of the last 8 sends in arrival
 * order, indexed by m_recent_noack_head (low 3 bits). Each TX event
 * consumes the tail entry and skips the metric update if the matching
 * send was noack — those report tx_attempts=1 unconditionally and
 * would dilute the signal toward zero. The send-side push runs on the
 * same TX path that yields the event, so head/tail stay in sync as
 * long as we account for one push per event. */
#define LQ_NOACK_RING_SIZE 8
static uint32_t m_link_quality_window;
static uint8_t  m_link_quality_count;
static uint16_t m_link_quality_ewma_x10;
static uint8_t  m_recent_noack[LQ_NOACK_RING_SIZE];
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
    if (d < 100) {
        d = 100;  /* hard floor — Nordic ESB rejects values below ~50 µs */
    }
    if (d > UINT16_MAX) {
        d = UINT16_MAX;
    }
    return (uint16_t)d;
}

/* Peer's (dongle's) view of the link. Stamped from ESB_PKT_LINK_STATS ACK
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
    const esb_transport_evt_t evt = {
        .type   = ESB_RX_EVT,
        .rx_buf = rx_buf,
        .rx_len = rx_len,
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
        m_recent_noack[m_recent_noack_tail & (LQ_NOACK_RING_SIZE - 1)] != 0;
    m_recent_noack_tail++;
    return was_noack;
}

/* Shift one bit into the link-quality window, maintain the cached
 * popcount, update the EWMA. Called from the RADIO ISR. Single-word
 * Cortex-M reads/writes on the state vars are atomic; the shell
 * accessor takes a snapshot rather than locking. */
static void link_quality_record(uint8_t tx_attempts, bool retry_bit) {
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

    /* EWMA × 10 of attempts: ewma' = (ewma * 7 + attempts*10) / 8.
     * Saturating to retransmit_count+1 in case of noise. Initialized
     * to 10 (== 1.0 attempts) in init / set_channel. */
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
        if (esb_set_retransmit_count(want) == 0) {
            m_adaptive_retransmit_count = want;
            LOG_DBG("adaptive retry: ewma_x10=%u -> count=%u",
                    m_link_quality_ewma_x10, want);
        }
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
        m_recent_critical_override[m_recent_critical_tail & (CRIT_OVERRIDE_RING_SIZE - 1)] != 0;
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
 * matching event handler can restore. Non-mouse sends push a zero record
 * to keep head/tail in lockstep with TX events. */
static void motion_ring_push(const struct esb_mouse_body *body, const bool overrode) {
    const uint32_t slot = m_recent_motion_head %
        CONFIG_ZMK_ESB_ENDPOINT_POINTER_MOTION_RING_SIZE;
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
    const uint32_t slot = m_recent_motion_tail %
        CONFIG_ZMK_ESB_ENDPOINT_POINTER_MOTION_RING_SIZE;
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
static void esb_flush_tx_and_reset(void) {
    esb_flush_tx();
#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_HOP)
    m_recent_noack_tail = m_recent_noack_head;
#endif
#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_CRITICAL_MAX_RETRANSMIT)
    m_recent_critical_tail = m_recent_critical_head;
#endif
    m_recent_motion_tail = m_recent_motion_head;
    m_pointer_inflight_count = 0;
    restore_retransmit_count();
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
    esb_flush_tx_and_reset();
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
            esb_flush_tx_and_reset();
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
         * dongle catch up on the new channel), a latch would stay set
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
        /* TX_FAILED also feeds the link-quality metric: definite retry
         * exhaustion is a stronger "1" than a single retransmit. We
         * still consume the noack slot to keep ring head/tail in sync,
         * but unconditionally count it as retried — the existing
         * TX-fail trigger is the real escalation path here, the
         * metric just keeps coop-hop's view of "how bad is it" honest. */
        {
            (void)consume_recent_noack();
            link_quality_record((uint8_t)event->tx_attempts, true);
            /* Don't fire the coop-hop trigger from the fail branch:
             * TX-fail-window / weak-link triggers already own the
             * "really bad" path, and double-firing would race a
             * speculative hop against a coop hop. */
        }
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
        esb_flush_tx_and_reset();
        break;

    case ESB_EVENT_RX_RECEIVED:
        while (esb_read_rx_payload(&m_rx_payload) == 0) {
            if (m_rx_payload.length > 0 &&
                m_rx_payload.length <= ESB_MAX_PAYLOAD_LEN) {
                rx_len = m_rx_payload.length;
                memcpy(rx_buf, m_rx_payload.data, rx_len);
                k_work_submit(&rx_work);
            }
        }
        break;
    }
}

static int esb_init_and_configure(void) {
    struct esb_config cfg = ESB_DEFAULT_CONFIG;
    cfg.protocol           = ESB_PROTOCOL_ESB_DPL;
    cfg.mode               = ESB_MODE_PTX;
    cfg.bitrate            = ESB_BITRATE_2MBPS_BLE;
    cfg.crc                = ESB_CRC_16BIT;
    cfg.retransmit_count   = CONFIG_ZMK_ESB_ENDPOINT_RETRANSMIT_COUNT;
    cfg.retransmit_delay   = CONFIG_ZMK_ESB_ENDPOINT_RETRANSMIT_DELAY_US;
    cfg.tx_mode            = ESB_TXMODE_AUTO;
    /* nRF52833 fast ramp-up shortens radio RXIDLE→RX from ~140 µs to ~40 µs
     * (RADIO->MODECNF0.RU=1). Both endpoint and dongle must match — see
     * dongle/src/esb_prx.c cfg.use_fast_ramp_up. Shortens each retry's
     * airtime budget, letting the retransmit_delay cover more back-to-back
     * attempts and reducing the probability a retry window lands on a
     * periodic interferer. */
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

static void install_ram_vtor(void) {
    if (m_ram_vtor_installed) {
        return;
    }
    const uint32_t *flash_vtor = (const uint32_t *)(SCB->VTOR);
    memcpy(m_ram_vtor, flash_vtor, sizeof(m_ram_vtor));
    saved_radio_vector = m_ram_vtor[16 + RADIO_IRQn];
    const unsigned int key = irq_lock();
    __DSB();
    SCB->VTOR = (uint32_t)m_ram_vtor;
    __DSB();
    __ISB();
    irq_unlock(key);
    m_ram_vtor_installed = true;
    LOG_DBG("RAM VTOR installed at 0x%08x, saved RADIO vector=0x%08x",
            (uint32_t)m_ram_vtor, saved_radio_vector);
}

int esb_transport_init(const esb_transport_cb_t cb) {
    m_cb = cb;
    k_work_init(&rx_work, rx_work_fn);
#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_RENDEZVOUS)
    k_sem_init(&m_sync_tx_done, 0, 1);
#endif
    install_ram_vtor();
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
    /* Force an LFRC calibration on every hop. The persistent HFXO hold
     * (see hfxo_request) keeps the 32M xtal running, but the LFRC itself
     * still drifts with temperature between the calibrator's 8s hw-cal
     * windows. A hop is almost always preceded by a streak of TX failures,
     * and stale LFRC timing is one plausible contributor — run the cal
     * here so we start the new channel with freshly-trimmed LF timing.
     * force_start is async and no-ops if a cal is already running; it
     * does not block the hop. */
    z_nrf_clock_calibration_force_start();

    /* Drain any queued packets on the old channel — retransmits on the new
     * channel would arrive at a dongle that has not yet hopped. The helper
     * also drains the noack/critical/motion rings, zeroes the inflight
     * pointer count, and restores the radio's retransmit_count, so any
     * critical- or pointer-override packet that got dropped by the flush
     * does not strand the radio at the override value. */
    esb_flush_tx_and_reset();
    const int err = esb_set_rf_channel(channel);
    if (err) {
        return err;
    }
    m_addr.channel = channel;
    consecutive_tx_fail = 0;
    /* Peer's RSSI view is per-link, not per-channel — the dongle samples
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
     * (post-quiet window + dongle catch-up), so additionally drop any
     * pending pointer refund — adding it to the next mouse report would
     * teleport the cursor. Matches the input processor's accumulator
     * clear at the same moment. The motion ring tail and inflight count
     * are already wiped by esb_flush_tx_and_reset above; only the refund
     * pool is unique to the channel-change path. */
    m_motion_refund.dx = 0;
    m_motion_refund.dy = 0;
    m_motion_refund.scroll_x = 0;
    m_motion_refund.scroll_y = 0;
    m_motion_refund.stamp_ms = 0;
    /* Arm the post-hop quiet window. Any send() during this interval is
     * silently dropped — the dongle may still be on the old channel and
     * anything we transmit would just fail and feed the next hop trigger.
     * Saturating add: the deadline stays in uint32 wrap-safe range for
     * every realistic quiet value. */
    const uint32_t quiet_until =
        k_uptime_get_32() + CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_HOP_POST_QUIET_MS;
    m_tx_quiet_until_ms = (quiet_until == 0) ? 1 : quiet_until;
#endif
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

#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_RENDEZVOUS)
    /* Drop any user-thread send while a synchronous side-trip is on the
     * radio. The rendezvous send temporarily flips RADIO->FREQUENCY to
     * another channel; queueing a user packet here would let the ESB
     * state machine TX it on the wrong channel. The window is bounded
     * by RENDEZVOUS_TIMEOUT_MS (a few ms). */
    if (m_sync_tx_in_progress) {
        return 0;
    }
#endif

    /* Jitter the retransmit delay per-send. De-correlates retries from
     * periodic 2.4 GHz interferers (WiFi beacons, BLE adv) that might
     * happen to line up with the fixed base delay. Cheap — one xorshift
     * step + one register write. esb_set_retransmit_delay stores the
     * value for the radio state machine to pick up on the next TX;
     * safe from thread context. */
    (void)esb_set_retransmit_delay(jittered_retransmit_delay());

#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_HOP)
    /* Post-hop quiet window: the dongle needs a moment to follow us to
     * the new channel. Silently drop user traffic until the deadline —
     * 32-bit wrap-safe compare. Losing CHANNEL_HOP_POST_QUIET_MS worth of
     * HID reports is strictly better than queueing them for a channel the
     * receiver hasn't reached yet, where every attempt would just count
     * as a failure. */
    if (m_tx_quiet_until_ms != 0) {
        const uint32_t now = k_uptime_get_32();
        if ((int32_t)(m_tx_quiet_until_ms - now) > 0) {
            return 0;
        }
        m_tx_quiet_until_ms = 0;
    }
#endif

#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_HID_NOACK)
    /* Only mouse reports may skip ACKs: pointer/scroll is a self-correcting stream,
     * a dropped frame costs at most a few ticks of motion. Keyboard/consumer reports
     * carry edge-triggered state — a lost release packet strands the key on the host
     * forever (re-sending the same "released" state produces no HID edge). Keep them
     * ACKed so ESB retransmits if the dongle misses one. */
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

    const bool is_shell = (len >= 1) &&
        (data[0] == ESB_PKT_SHELL_DATA || data[0] == ESB_PKT_SHELL_POLL || data[0] == ESB_PKT_SHELL_STOP);
    if (!is_shell && esb_tx_full()) {
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
    m_recent_noack[m_recent_noack_head & (LQ_NOACK_RING_SIZE - 1)] =
        noack ? 1U : 0U;
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
    m_recent_critical_override[m_recent_critical_head & (CRIT_OVERRIDE_RING_SIZE - 1)] =
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
        motion_ring_push(body, pointer_override);
    } else {
        motion_ring_push(NULL, false);
    }

    return esb_write_payload(&pkt);
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
     * picks up the new frequency on the next ramp-up. So a flush_tx +
     * channel_set + write_payload sequence guarantees that the next packet
     * to leave is ours, on the channel we just set. After completion the
     * inverse restore returns the radio to the active channel for the
     * very next user-thread send (which we held off via m_sync_tx_in_progress
     * during the round trip). m_addr.channel is left untouched throughout —
     * we are visiting, not committing. */
    esb_flush_tx_and_reset();

    const int chan_err = esb_set_rf_channel(channel);
    if (chan_err) {
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
        return werr;
    }

    const int wait_err = k_sem_take(&m_sync_tx_done, timeout);

    /* Always restore the active channel, even on timeout — leaving the
     * radio on the rendezvous channel would silently break every
     * subsequent user TX. The flag is cleared by the ISR on completion;
     * on timeout we clear it here so the next caller is unblocked. */
    (void)esb_set_rf_channel(m_addr.channel);

    if (wait_err) {
        m_sync_tx_in_progress = false;
        return -ETIMEDOUT;
    }
    return m_sync_tx_result_success ? 0 : -EIO;
}
#endif /* RENDEZVOUS */

static uint32_t saved_bt_ll_ppi_chen;
static bool bt_ll_suspended;

static void bt_ll_suspend(void) {
    if (bt_ll_suspended) {
        return;
    }
    saved_bt_ll_ppi_chen = NRF_PPI->CHEN & BT_LL_PPI_MASK;
    NRF_PPI->CHENCLR = BT_LL_PPI_MASK;
    bt_ll_suspended = true;
    LOG_DBG("BT LL PPI suspended (saved CHEN=0x%08x)", saved_bt_ll_ppi_chen);
}

static void bt_ll_resume(void) {
    if (!bt_ll_suspended) {
        return;
    }
    NRF_PPI->CHENSET = saved_bt_ll_ppi_chen;
    bt_ll_suspended = false;
    LOG_DBG("BT LL PPI resumed (restored CHEN=0x%08x)", saved_bt_ll_ppi_chen);
}

/*
 * The LFRC calibrator (CONFIG_CLOCK_CONTROL_NRF_K32SRC_RC, configurable period
 * with MAX_SKIP) does onoff_request / onoff_release on the HF subsystem each
 * period — when the refcount drops back to 0 the driver issues HFCLKSTOP.
 * Every (MAX_SKIP+1) periods a full hardware calibration runs, giving the
 * RADIO just long enough to push a burst of ESB frames.  Hold a persistent HF
 * request while ESB is active so the calibrator's release going 1 -> 0 never
 * happens and HFXO stays on continuously.
 */
static struct onoff_client hfclk_cli;
static bool hfclk_held;

static int hfxo_request(void) {
    if (hfclk_held) {
        return 0;
    }
    struct onoff_manager *mgr =
        z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_HF);
    sys_notify_init_spinwait(&hfclk_cli.notify);
    const int err = onoff_request(mgr, &hfclk_cli);
    if (err < 0) {
        LOG_ERR("HFXO onoff_request failed: %d", err);
        return err;
    }
    int res;
    while (sys_notify_fetch_result(&hfclk_cli.notify, &res) == -EAGAIN) {
        k_busy_wait(10);
    }
    hfclk_held = (res == 0);
    if (!hfclk_held) {
        LOG_ERR("HFXO start failed: %d", res);
    } else {
        LOG_DBG("HFXO running (HFCLKSTAT=0x%08x)", NRF_CLOCK->HFCLKSTAT);
    }
    return res;
}

static void hfxo_release(void) {
    if (!hfclk_held) {
        return;
    }
    struct onoff_manager *mgr =
        z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_HF);
    (void)onoff_release(mgr);
    hfclk_held = false;
    LOG_DBG("HFXO released");
}

void esb_transport_on_slot_start(void) {
    LOG_DBG("ESB slot start (HFCLKSTAT=0x%08x PPI_CHEN=0x%08x)", NRF_CLOCK->HFCLKSTAT, NRF_PPI->CHEN);
    bt_ll_suspend();
    const int hfxo_err = hfxo_request();
    if (hfxo_err) {
        LOG_ERR("ESB slot start: HFXO failed (%d), radio may be unreliable", hfxo_err);
    }
    const int init_err = esb_init_and_configure();
    if (init_err) {
        LOG_ERR("ESB slot start: init failed (%d)", init_err);
        return;
    }

    /*
     * esb_init() calls irq_connect_dynamic(RADIO_IRQn, ..., radio_dynamic_irq_handler)
     * which sets _sw_isr_table[RADIO_IRQn].  But ARM_IRQ_DIRECT_DYNAMIC_CONNECT inside
     * esb_init() is a compile-time macro that does NOT update the RAM vector table at
     * runtime — the BLE LL's radio_nrf5_isr is still in VTOR[16+RADIO_IRQn].
     * Patch it now so RADIO IRQs reach ESB's handler.
     */
    {
        const unsigned int key = irq_lock();
        m_ram_vtor[16 + RADIO_IRQn] = (uint32_t) z_arm_irq_direct_dynamic_dispatch_reschedule;
        __DSB();
        irq_unlock(key);
    }

    consecutive_tx_fail = 0;
#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_HOP) && \
    IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_ADAPTIVE_RETRY)
    /* Reset adaptive state to the default (what esb_init_and_configure
     * programmed) so the worker's cached value matches reality before
     * the first EWMA sample arrives. */
    m_adaptive_retransmit_count = CONFIG_ZMK_ESB_ENDPOINT_RETRANSMIT_COUNT;
    k_work_reschedule(&m_adaptive_retry_work,
                      K_MSEC(CONFIG_ZMK_ESB_ENDPOINT_ADAPTIVE_RETRY_INTERVAL_MS));
#endif
    LOG_DBG("ESB slot start OK (VTOR[RADIO]=0x%08x)",
            m_ram_vtor[16 + RADIO_IRQn]);
}

void esb_transport_on_slot_stop(void) {
    LOG_DBG("ESB slot stop (ok=%u fail=%u)", tx_ok_count, tx_fail_count);
    consecutive_tx_fail = 0;
#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_HOP) && \
    IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_ADAPTIVE_RETRY)
    k_work_cancel_delayable(&m_adaptive_retry_work);
#endif

    esb_disable();

    if (m_ram_vtor_installed) {
        const unsigned int key = irq_lock();
        m_ram_vtor[16 + RADIO_IRQn] = saved_radio_vector;
        __DSB();
        irq_unlock(key);
        irq_enable(RADIO_IRQn);
        LOG_DBG("VTOR[RADIO] restored to 0x%08x", saved_radio_vector);
    }

    bt_ll_resume();
    hfxo_release();
}
