/*
 * Copyright (c) 2025 efogdev
 * SPDX-License-Identifier: MIT
 *
 * Mirrors ZMK's hid_listener semantics against a module-local report state
 * rather than the global keyboard_report / consumer_report. Decouples the
 * ESB report from hid_listener: the order in which the two listeners run
 * (link order; .event_subscription is unsorted) no longer matters, and the
 * global implicit-mod cross-release bug can't leak into the ESB stream.
 *
 * Returns ZMK_EV_EVENT_HANDLED when ESB is active + paired so hid_listener
 * is skipped and USB/BLE stays quiet; bubbles otherwise so the normal path
 * runs unchanged.
 */

#include <zephyr/kernel.h>
#include <string.h>
#include <zmk/event_manager.h>
#include <zmk/events/keycode_state_changed.h>
#include <zmk/hid.h>
#include <zmk/keys.h>
#include <dt-bindings/zmk/hid_usage_pages.h>
#include <zmk_esb/endpoint.h>
#include <zmk_esb/protocol.h>
#include "esb_transport.h"
#include "pairing.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(zmk_esb_hid, CONFIG_ZMK_ESB_ENDPOINT_LOG_LEVEL);

static struct zmk_hid_keyboard_report_body kb_body;
static struct zmk_hid_consumer_report_body cons_body;

/* Per-bit refcount for each modifier flag. explicit and implicit tracked
 * separately so ZMK's known implicit-mod cross-release bug (see inline
 * comment at hid_listener.c:81-86) doesn't affect the ESB stream. */
static uint8_t explicit_mod_refcount[8];
static uint8_t implicit_mod_refcount[8];
static zmk_mod_flags_t explicit_mods;
static zmk_mod_flags_t implicit_mods;

static void mods_inc(uint8_t *rc, zmk_mod_flags_t *out, const zmk_mod_flags_t mods) {
    for (int b = 0; b < 8; b++) {
        if (mods & BIT(b)) {
            if (rc[b]++ == 0) {
                *out |= BIT(b);
            }
        }
    }
}

static void mods_dec(uint8_t *rc, zmk_mod_flags_t *out, const zmk_mod_flags_t mods) {
    for (int b = 0; b < 8; b++) {
        if ((mods & BIT(b)) && rc[b] > 0) {
            if (--rc[b] == 0) {
                *out &= ~BIT(b);
            }
        }
    }
}

static void kb_keys_press(const uint32_t keycode) {
#if IS_ENABLED(CONFIG_ZMK_HID_REPORT_TYPE_NKRO)
    if (keycode <= ZMK_HID_KEYBOARD_NKRO_MAX_USAGE) {
        kb_body.keys[keycode / 8] |= BIT(keycode % 8);
    }
#elif IS_ENABLED(CONFIG_ZMK_HID_REPORT_TYPE_HKRO)
    for (int i = 0; i < CONFIG_ZMK_HID_KEYBOARD_REPORT_SIZE; i++) {
        if (kb_body.keys[i] == (uint8_t)keycode) {
            return;
        }
    }
    for (int i = 0; i < CONFIG_ZMK_HID_KEYBOARD_REPORT_SIZE; i++) {
        if (kb_body.keys[i] == 0) {
            kb_body.keys[i] = (uint8_t)keycode;
            return;
        }
    }
#endif
}

static void kb_keys_release(const uint32_t keycode) {
#if IS_ENABLED(CONFIG_ZMK_HID_REPORT_TYPE_NKRO)
    if (keycode <= ZMK_HID_KEYBOARD_NKRO_MAX_USAGE) {
        kb_body.keys[keycode / 8] &= ~BIT(keycode % 8);
    }
#elif IS_ENABLED(CONFIG_ZMK_HID_REPORT_TYPE_HKRO)
    for (int i = 0; i < CONFIG_ZMK_HID_KEYBOARD_REPORT_SIZE; i++) {
        if (kb_body.keys[i] == (uint8_t)keycode) {
            kb_body.keys[i] = 0;
        }
    }
#endif
}

static void cons_keys_press(const uint32_t keycode) {
    for (int i = 0; i < CONFIG_ZMK_HID_CONSUMER_REPORT_SIZE; i++) {
        if (cons_body.keys[i] == keycode) {
            return;
        }
    }
    for (int i = 0; i < CONFIG_ZMK_HID_CONSUMER_REPORT_SIZE; i++) {
        if (cons_body.keys[i] == 0) {
            cons_body.keys[i] = keycode;
            return;
        }
    }
}

static void cons_keys_release(const uint32_t keycode) {
    for (int i = 0; i < CONFIG_ZMK_HID_CONSUMER_REPORT_SIZE; i++) {
        if (cons_body.keys[i] == keycode) {
            cons_body.keys[i] = 0;
        }
    }
}

static void send_keyboard(void) {
    kb_body.modifiers = explicit_mods | implicit_mods;
    struct esb_pkt_hid_report pkt = {
        .type        = ESB_PKT_HID_REPORT,
        .report_type = ESB_REPORT_KEYBOARD,
    };
    size_t body_len = sizeof(kb_body);
    if (body_len > sizeof(pkt.data)) {
        body_len = sizeof(pkt.data);
    }
    memcpy(pkt.data, &kb_body, body_len);
    const int err = esb_transport_send(ESB_PIPE_DATA, (uint8_t *)&pkt, sizeof(pkt));
    if (err && err != -ENOMEM) {
        LOG_WRN("KB report send err %d", err);
    }
}

static void send_consumer(void) {
    struct esb_pkt_hid_report pkt = {
        .type        = ESB_PKT_HID_REPORT,
        .report_type = ESB_REPORT_CONSUMER,
    };
    size_t body_len = sizeof(cons_body);
    if (body_len > sizeof(pkt.data)) {
        body_len = sizeof(pkt.data);
    }
    memcpy(pkt.data, &cons_body, body_len);
    const int err = esb_transport_send(ESB_PIPE_DATA, (uint8_t *)&pkt, sizeof(pkt));
    if (err && err != -ENOMEM) {
        LOG_WRN("consumer report send err %d", err);
    }
}

static int hid_relay_cb(const zmk_event_t *eh) {
    const struct zmk_keycode_state_changed *ev = as_zmk_keycode_state_changed(eh);
    if (!ev) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    const bool active = zmk_esb_endpoint_is_active() && pairing_is_connected();

    /* Track state unconditionally: keeps the ESB report internally coherent
     * across profile switches for events we did observe. */
    switch (ev->usage_page) {
    case HID_USAGE_KEY:
        if (is_mod(ev->usage_page, ev->keycode)) {
            const zmk_mod_flags_t bit = BIT(ev->keycode - HID_USAGE_KEY_KEYBOARD_LEFTCONTROL);
            if (ev->state) {
                mods_inc(explicit_mod_refcount, &explicit_mods, bit);
            } else {
                mods_dec(explicit_mod_refcount, &explicit_mods, bit);
            }
        } else {
            if (ev->state) {
                kb_keys_press(ev->keycode);
                mods_inc(explicit_mod_refcount, &explicit_mods, ev->explicit_modifiers);
                mods_inc(implicit_mod_refcount, &implicit_mods, ev->implicit_modifiers);
            } else {
                kb_keys_release(ev->keycode);
                mods_dec(explicit_mod_refcount, &explicit_mods, ev->explicit_modifiers);
                mods_dec(implicit_mod_refcount, &implicit_mods, ev->implicit_modifiers);
            }
        }
        if (active) {
            send_keyboard();
        }
        break;
    case HID_USAGE_CONSUMER:
        if (ev->state) {
            cons_keys_press(ev->keycode);
            mods_inc(explicit_mod_refcount, &explicit_mods, ev->explicit_modifiers);
            mods_inc(implicit_mod_refcount, &implicit_mods, ev->implicit_modifiers);
        } else {
            cons_keys_release(ev->keycode);
            mods_dec(explicit_mod_refcount, &explicit_mods, ev->explicit_modifiers);
            mods_dec(implicit_mod_refcount, &implicit_mods, ev->implicit_modifiers);
        }
        if (active) {
            /* Consumer keycodes may carry explicit/implicit mods — push a
             * keyboard report too so modifier state stays in sync. */
            if (ev->explicit_modifiers || ev->implicit_modifiers) {
                send_keyboard();
            }
            send_consumer();
        }
        break;
    default:
        return ZMK_EV_EVENT_BUBBLE;
    }

    return active ? ZMK_EV_EVENT_HANDLED : ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(esb_relay, hid_relay_cb);
ZMK_SUBSCRIPTION(esb_relay, zmk_keycode_state_changed);
