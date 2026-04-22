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
    bool configured;
} m_addr;

static struct k_work rx_work;
static uint8_t rx_buf[ESB_MAX_PAYLOAD_LEN];
static uint8_t rx_len;

static void rx_work_fn(struct k_work *w) {
    if (!m_cb) {
        return;
    }
    const esb_transport_evt_t evt = {
        .type   = ESB_RX_EVT,
        .rx_buf = rx_buf,
        .rx_len = rx_len,
    };
    m_cb(&evt);
}

static void zmk_esb_transport_evt_cb(struct esb_evt const *event) {
    static uint32_t tx_ok_count;
    static uint32_t tx_fail_count;

    switch (event->evt_id) {
    case ESB_EVENT_TX_SUCCESS:
        break;

    case ESB_EVENT_TX_FAILED:
        tx_fail_count++;
        if ((tx_fail_count % 10u) == 0u) {
            LOG_WRN("ESB TX_FAILED count=%u (ok=%u)", tx_fail_count, tx_ok_count);
        }
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
    cfg.bitrate            = ESB_BITRATE_2MBPS;
    cfg.crc                = ESB_CRC_8BIT;
    cfg.retransmit_count   = CONFIG_ZMK_ESB_ENDPOINT_RETRANSMIT_COUNT;
    cfg.retransmit_delay   = CONFIG_ZMK_ESB_ENDPOINT_RETRANSMIT_DELAY_US;
    cfg.tx_mode            = ESB_TXMODE_AUTO;
    cfg.use_fast_ramp_up   = true;
    cfg.selective_auto_ack = true;
    cfg.event_handler      = zmk_esb_transport_evt_cb;
    cfg.payload_length     = ESB_MAX_PAYLOAD_LEN;

    const int err = esb_init(&cfg);
    if (err) {
        return err;
    }

    if (m_addr.configured) {
        esb_set_base_address_0(m_addr.base0);
        esb_set_base_address_1(m_addr.base1);
        esb_set_prefixes(m_addr.prefixes, ARRAY_SIZE(m_addr.prefixes));
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
}

int esb_transport_init(const esb_transport_cb_t cb) {
    m_cb = cb;
    k_work_init(&rx_work, rx_work_fn);
    install_ram_vtor();
    return 0;
}

void esb_transport_set_addresses(const uint8_t base0[4], const uint8_t base1[4], const uint8_t prefixes[8], const uint8_t channel) {
    memcpy(m_addr.base0, base0, 4);
    memcpy(m_addr.base1, base1, 4);
    memcpy(m_addr.prefixes, prefixes, 8);
    m_addr.channel = channel;
    m_addr.configured = true;
}

void esb_transport_deinit(void) {
    m_cb = NULL;
    m_addr.configured = false;
}

int esb_transport_send(const uint8_t pipe, const uint8_t *data, uint8_t len) {
    if (len > ESB_MAX_PAYLOAD_LEN) {
        return -EMSGSIZE;
    }

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
    return esb_write_payload(&pkt);
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
}

static void bt_ll_resume(void) {
    if (!bt_ll_suspended) {
        return;
    }
    NRF_PPI->CHENSET = saved_bt_ll_ppi_chen;
    bt_ll_suspended = false;
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
        return err;
    }
    int res;
    while (sys_notify_fetch_result(&hfclk_cli.notify, &res) == -EAGAIN) {
        k_busy_wait(10);
    }
    hfclk_held = (res == 0);
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
}

void esb_transport_on_slot_start(void) {
    bt_ll_suspend();
    hfxo_request();
    const int err = esb_init_and_configure();
    if (err) {
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
}

void esb_transport_on_slot_stop(void) {
    esb_disable();

    if (m_ram_vtor_installed) {
        const unsigned int key = irq_lock();
        m_ram_vtor[16 + RADIO_IRQn] = saved_radio_vector;
        __DSB();
        irq_unlock(key);
        irq_enable(RADIO_IRQn);
    }

    bt_ll_resume();
    hfxo_release();
}
