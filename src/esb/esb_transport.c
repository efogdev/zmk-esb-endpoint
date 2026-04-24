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
LOG_MODULE_REGISTER(zmk_esb_transport, CONFIG_ZMK_LOG_LEVEL);

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

static struct k_work rx_work;
static uint8_t rx_buf[ESB_MAX_PAYLOAD_LEN];
static uint8_t rx_len;

static uint32_t tx_ok_count;
static uint32_t tx_fail_count;
static uint32_t consecutive_tx_fail;

#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_HOP)
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
#endif

/* 32-bit wrap-safe: (uint32_t)(now - last) stays correct for spans up to
 * ~24.8 days, far beyond any sensible activity threshold. Single-word access
 * on Cortex-M is atomic, so no lock is needed. */
static uint32_t m_last_activity_ms;
static bool     m_activity_seen;

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

static void zmk_esb_transport_evt_cb(struct esb_evt const *event) {
    switch (event->evt_id) {
    case ESB_EVENT_TX_SUCCESS:
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
        tx_ok_count++;
        consecutive_tx_fail = 0;
#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_HOP)
        m_first_fail_ms = 0;
        {
            const uint32_t now_ok = k_uptime_get_32();
            m_last_tx_success_ms = (now_ok == 0) ? 1 : now_ok;
        }
        channel_hop_ep_on_tx_success_isr();
#endif
#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_SHELL_RELAY)
        esb_shell_relay_notify_tx();
#endif
#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_BENCH)
        esb_bench_notify_tx_success();
#endif
        break;

    case ESB_EVENT_TX_FAILED:
        if (m_sync_tx_in_progress) {
            /* See TX_SUCCESS branch — this fail belongs to the side-trip,
             * not the active channel. Skip all hop / counter machinery so
             * a missed rendezvous PROPOSAL does not register as a failure
             * on the channel we'll be back on in microseconds. */
            m_sync_tx_result_success = false;
            m_sync_tx_in_progress = false;
            k_sem_give(&m_sync_tx_done);
            esb_flush_tx();
            break;
        }
        tx_fail_count++;
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
#endif
#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_BENCH)
        esb_bench_notify_tx_fail();
#endif
        esb_flush_tx();
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
    cfg.use_fast_ramp_up   = false;
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
    k_sem_init(&m_sync_tx_done, 0, 1);
    install_ram_vtor();
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
     * channel would arrive at a dongle that has not yet hopped. */
    esb_flush_tx();
    const int err = esb_set_rf_channel(channel);
    if (err) {
        return err;
    }
    m_addr.channel = channel;
    consecutive_tx_fail = 0;
#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_HOP)
    m_first_fail_ms = 0;
    /* New channel — no evidence yet that anything is alive here. The
     * weak-link trigger's != 0 gate keeps it quiet until a real
     * TX_SUCCESS re-arms the clock. */
    m_last_tx_success_ms = 0;
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

void esb_transport_deinit(void) {
    m_addr.configured = false;
}

int esb_transport_send(const uint8_t pipe, const uint8_t *data, uint8_t len) {
    if (len > ESB_MAX_PAYLOAD_LEN) {
        return -EMSGSIZE;
    }

    /* Drop any user-thread send while a synchronous side-trip is on the
     * radio. The rendezvous send temporarily flips RADIO->FREQUENCY to
     * another channel; queueing a user packet here would let the ESB
     * state machine TX it on the wrong channel. The window is bounded
     * by RENDEZVOUS_TIMEOUT_MS (a few ms). */
    if (m_sync_tx_in_progress) {
        return 0;
    }

#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_HOP)
    /* Post-hop quiet window: the dongle needs a moment to follow us to
     * the new channel. Silently drop user traffic until the deadline —
     * 32-bit wrap-safe compare. Losing ~10 ms of HID reports is strictly
     * better than queueing them for a channel the receiver hasn't
     * reached yet, where every attempt would just count as a failure. */
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
        esb_flush_tx();
    }

    const bool is_bg_poll = (len >= 1) && (data[0] == ESB_PKT_SHELL_BG_POLL);
    if (!is_bg_poll) {
        note_activity();
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

    return esb_write_payload(&pkt);
}

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
    esb_flush_tx();

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
 * The LFRC calibrator (CONFIG_CLOCK_CONTROL_NRF_K32SRC_RC + default 4000ms period,
 * MAX_SKIP=1) does onoff_request / onoff_release on the HF subsystem every 4s —
 * when the refcount drops back to 0 the driver issues HFCLKSTOP.  Every 8s a
 * full hardware calibration runs, giving the RADIO just long enough to push a
 * burst of ESB frames.  Hold a persistent HF request while ESB is active so the
 * calibrator's release goes 1 -> 0 never happens and HFXO stays on continuously.
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
    LOG_DBG("ESB slot start OK (VTOR[RADIO]=0x%08x)",
            m_ram_vtor[16 + RADIO_IRQn]);
}

void esb_transport_on_slot_stop(void) {
    LOG_DBG("ESB slot stop (ok=%u fail=%u)", tx_ok_count, tx_fail_count);
    consecutive_tx_fail = 0;

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
