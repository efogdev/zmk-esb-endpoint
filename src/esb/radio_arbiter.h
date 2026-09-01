/*
 * Copyright (c) 2025 efogdev
 * SPDX-License-Identifier: MIT
 */

#pragma once

/* BT_LL_SW_SPLIT (Zephyr's open-source BLE link layer, the only one
 * available on this target) owns the RADIO peripheral with no coexistence
 * hooks of its own: RADIO_IRQn is bound to its ISR directly in the flash
 * vector table at build time, its scheduler drives RADIO via PPI shortcuts
 * independent of any active BLE role, and its LFRC calibrator cycles HFXO
 * on/off on its own schedule. This module is the only thing in the
 * firmware that reaches outside its own subsystem to arbitrate the radio
 * between BLE and ESB — everything else in zmk-esb-endpoint talks to ESB
 * or to ZMK's public APIs only.
 *
 * Sequencing across a takeover:
 *   radio_arbiter_init()     - once, at boot
 *   radio_arbiter_prepare()  - before ESB claims _sw_isr_table[RADIO_IRQn]
 *   ... esb_init_and_configure() ...
 *   radio_arbiter_take()     - after ESB has claimed the slot
 *   ... ESB owns the radio ...
 *   ... esb_disable() ...
 *   radio_arbiter_release()
 */

/* Relocates the interrupt vector table to RAM so RADIO_IRQn can later be
 * retargeted at runtime. Idempotent; call once, before the first
 * radio_arbiter_take(). */
void radio_arbiter_init(void);

/* Suspends the BLE link layer's PPI shortcuts to RADIO and holds HFXO on.
 * Call before handing ESB the radio, so nothing races with the vector swap
 * that follows. */
void radio_arbiter_prepare(void);

/* Retargets RADIO_IRQn's vector to the dynamic-dispatch stub, so interrupts
 * reach whatever esb_init() registered in _sw_isr_table[RADIO_IRQn]. Call
 * only after that registration has happened. */
void radio_arbiter_take(void);

/* Restores RADIO_IRQn's vector to the BLE link layer's ISR, re-enables it,
 * unmasks the PPI shortcuts, and releases the HFXO hold. */
void radio_arbiter_release(void);
