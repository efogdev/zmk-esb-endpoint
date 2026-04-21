/*
 * Copyright (c) 2025 efogdev
 * SPDX-License-Identifier: MIT
 *
 * ZMK event listener for keyboard and consumer HID reports.
 * Fires after ZMK's hid_listener (link order), so HID state is already updated.
 */

#include <zephyr/kernel.h>
#include <zmk/event_manager.h>
#include <zmk/events/keycode_state_changed.h>
#include <zmk/hid.h>
#include <dt-bindings/zmk/hid_usage_pages.h>
#include <zmk_esb/endpoint.h>
#include <zmk_esb/protocol.h>
#include "esb_transport.h"
#include "pairing.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(zmk_esb_hid, CONFIG_ZMK_LOG_LEVEL);

static void send_keyboard(void)
{
    struct zmk_hid_keyboard_report *r = zmk_hid_get_keyboard_report();
    struct esb_pkt_hid_report pkt = {
        .type        = ESB_PKT_HID_REPORT,
        .report_type = ESB_REPORT_KEYBOARD,
    };
    size_t body_len = sizeof(r->body);
    if (body_len > sizeof(pkt.data)) {
        body_len = sizeof(pkt.data);
    }
    memcpy(pkt.data, &r->body, body_len);
    esb_transport_send(1, (uint8_t *)&pkt, sizeof(pkt));
}

static void send_consumer(void)
{
    struct zmk_hid_consumer_report *r = zmk_hid_get_consumer_report();
    struct esb_pkt_hid_report pkt = {
        .type        = ESB_PKT_HID_REPORT,
        .report_type = ESB_REPORT_CONSUMER,
    };
    size_t body_len = sizeof(r->body);
    if (body_len > sizeof(pkt.data)) {
        body_len = sizeof(pkt.data);
    }
    memcpy(pkt.data, &r->body, body_len);
    esb_transport_send(1, (uint8_t *)&pkt, sizeof(pkt));
}

static int hid_relay_cb(const zmk_event_t *eh)
{
    if (!zmk_esb_endpoint_is_active() || !pairing_is_connected()) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    const struct zmk_keycode_state_changed *ev = as_zmk_keycode_state_changed(eh);
    if (!ev) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    switch (ev->usage_page) {
    case HID_USAGE_KEY:
        send_keyboard();
        break;
    case HID_USAGE_CONSUMER:
        send_consumer();
        /* Also send updated keyboard report if modifiers changed */
        send_keyboard();
        break;
    default:
        break;
    }

    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(zmk_esb_hid_relay, hid_relay_cb);
ZMK_SUBSCRIPTION(zmk_esb_hid_relay, zmk_keycode_state_changed);
