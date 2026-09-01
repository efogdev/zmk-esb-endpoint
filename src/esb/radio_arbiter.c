/*
 * Copyright (c) 2025 efogdev
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/kernel.h>
#include <zephyr/irq.h>
#include <zephyr/arch/arm/irq.h>
#include <zephyr/sw_isr_table.h>
#include <hal/nrf_ppi.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <zephyr/sys/onoff.h>

#include "radio_arbiter.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(zmk_esb_radio_arbiter, CONFIG_ZMK_ESB_ENDPOINT_LOG_LEVEL);

/* nRF52833: RADIO_IRQn = 1 */
#define RADIO_IRQn 1

/*
 * BT_LL_SW_SPLIT runs a TIMER0-driven scheduler with radio events wired via
 * PPI. Even when no adv/scan/conn role is active, stray TIMER0 CC events can
 * trigger RADIO tasks through those PPI shortcuts, clobbering ESB's
 * packet/address/channel config.
 *
 * While ESB owns the radio, clear the PPI channels the LL owns so no
 * shortcut can reach RADIO. TIMER0/RTC0 IRQs are deliberately left
 * unmasked: the BT host still expects to drive HCI commands (adv
 * start/stop, silently no-op'd by zmk_ble_radio_yielded()) while ESB is
 * active, and those ISRs must keep running so pending commands get acked,
 * else the host thread blocks forever.
 *
 * Mask covers PPI channels 6-19 (programmable, used by LL via SW split
 * ticker and RADIO enable/disable/capture), plus fixed PPIs 22/23/25 (HCTO
 * disable, AAR trigger, CRYPT trigger) — see radio_nrf5_ppi_resources.h.
 */
#define BT_LL_PPI_MASK 0x02CFFFC0u

/*
 * On Cortex-M4 with CONFIG_CPU_CORTEX_M_HAS_VTOR, Zephyr sets SCB->VTOR to
 * the flash _vector_start — there is no RAM copy. Writes to the flash
 * vector table are silently ignored, so retargeting VTOR[16+RADIO_IRQn] at
 * runtime requires a RAM copy of the table with SCB->VTOR redirected to it.
 *
 * Once the RAM table is installed, the RADIO entry can be swapped between
 * the BLE link layer's radio_nrf5_isr and Zephyr's own
 * z_arm_irq_direct_dynamic_dispatch_reschedule, which dispatches to
 * _sw_isr_table[RADIO_IRQn] — the slot esb_init() populates via
 * irq_connect_dynamic().
 *
 * nRF52833: 16 system + 48 external = 64 entries, 256-byte alignment.
 */
#define NVIC_NUM_VECTORS 64
static uint32_t m_ram_vtor[NVIC_NUM_VECTORS] __aligned(256);
static bool m_ram_vtor_installed;
static uint32_t saved_radio_vector;

extern void z_arm_irq_direct_dynamic_dispatch_reschedule(void);

void radio_arbiter_init(void) {
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

void radio_arbiter_take(void) {
    const unsigned int key = irq_lock();
    m_ram_vtor[16 + RADIO_IRQn] = (uint32_t)z_arm_irq_direct_dynamic_dispatch_reschedule;
    __DSB();
    irq_unlock(key);
    LOG_DBG("VTOR[RADIO] handed to ESB (0x%08x)", m_ram_vtor[16 + RADIO_IRQn]);
}

static void radio_vector_restore(void) {
    if (!m_ram_vtor_installed) {
        return;
    }
    const unsigned int key = irq_lock();
    m_ram_vtor[16 + RADIO_IRQn] = saved_radio_vector;
    __DSB();
    irq_unlock(key);
    irq_enable(RADIO_IRQn);
    LOG_DBG("VTOR[RADIO] restored to 0x%08x", saved_radio_vector);
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
 * The LFRC calibrator (CONFIG_CLOCK_CONTROL_NRF_K32SRC_RC, configurable
 * period with MAX_SKIP) does onoff_request / onoff_release on the HF
 * subsystem each period — when the refcount drops back to 0 the driver
 * issues HFCLKSTOP. Every (MAX_SKIP+1) periods a full hardware calibration
 * runs, giving RADIO just long enough to push a burst of ESB frames. Hold a
 * persistent HF request while ESB is active so the calibrator's release
 * going 1 -> 0 never happens and HFXO stays on continuously.
 */
static struct onoff_client hfclk_cli;
static bool hfclk_held;

static int hfxo_request(void) {
    if (hfclk_held) {
        return 0;
    }
    struct onoff_manager *mgr = z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_HF);
    sys_notify_init_spinwait(&hfclk_cli.notify);
    const int err = onoff_request(mgr, &hfclk_cli);
    if (err < 0) {
        return err;
    }
    int res;
    while (sys_notify_fetch_result(&hfclk_cli.notify, &res) == -EAGAIN) {
        k_busy_wait(10);
    }
    return res;
}

static void hfxo_release(void) {
    if (!hfclk_held) {
        return;
    }
    struct onoff_manager *mgr = z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_HF);
    (void)onoff_release(mgr);
    hfclk_held = false;
    LOG_DBG("HFXO released");
}

void radio_arbiter_prepare(void) {
    LOG_DBG("radio arbiter prepare (HFCLKSTAT=0x%08x PPI_CHEN=0x%08x)", NRF_CLOCK->HFCLKSTAT, NRF_PPI->CHEN);
    bt_ll_suspend();
    const int hfxo_err = hfxo_request();
    if (hfxo_err) {
        LOG_DBG("radio arbiter prepare: HFXO failed (%d), radio may be unreliable", hfxo_err);
    }
}

void radio_arbiter_release(void) {
    radio_vector_restore();
    bt_ll_resume();
    hfxo_release();
}
