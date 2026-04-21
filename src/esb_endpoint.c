/*
 * Copyright (c) 2025 efogdev
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_esb_endpoint

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/settings/settings.h>
#include <esb.h>

#include <zmk/event_manager.h>
#include <zmk/events/endpoint_changed.h>
#include <zmk/endpoints.h>
#include <zmk/endpoints_types.h>
#include <zmk/ble.h>

#include "timeslot_handler.h"
#include "esb_transport.h"
#include "pairing.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(zmk_esb_endpoint, CONFIG_ZMK_LOG_LEVEL);

/* True when the last BLE profile is the active endpoint */
static volatile bool esb_active;

bool zmk_esb_endpoint_is_active(void)
{
    return esb_active;
}

/* ---------- address configuration from DT ---------- */

static void configure_esb_addresses(void)
{
    static const uint8_t base_addr_0[4] = DT_INST_PROP(0, pairing_base_address);
    static const uint8_t base_addr_1[4] = DT_INST_PROP(0, data_base_address);
    static const uint8_t prefixes[8]    = {
        DT_INST_PROP(0, pairing_prefix),
        DT_INST_PROP(0, data_prefix),
        0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8,
    };

    esb_transport_set_addresses(base_addr_0, base_addr_1, prefixes,
                                DT_INST_PROP(0, esb_channel));
}

/* ---------- timeslot event callback ---------- */

static void on_timeslot_evt(timeslot_evt_t evt)
{
    if (evt == TS_EVT_STARTED) {
        esb_transport_on_slot_start();
    } else {
        esb_transport_on_slot_stop();
    }
}

/* ---------- transport RX callback ---------- */

static void on_transport_evt(const esb_transport_evt_t *evt)
{
    if (evt->type == ESB_RX_EVT) {
        pairing_on_rx(evt->rx_buf, evt->rx_len);
    }
}

/* ---------- activation / deactivation ---------- */

static void esb_endpoint_activate(void)
{
    LOG_INF("ESB endpoint activating");
    configure_esb_addresses();
    pairing_start();
    timeslot_handler_start();
    esb_active = true;
}

static void esb_endpoint_deactivate(void)
{
    LOG_INF("ESB endpoint deactivating");
    esb_active = false;
    timeslot_handler_stop();
    pairing_stop();
    esb_transport_deinit();
}

/* ---------- ZMK endpoint_changed listener ---------- */

static int endpoint_listener_cb(const zmk_event_t *eh)
{
    const struct zmk_endpoint_changed *ev = as_zmk_endpoint_changed(eh);
    if (!ev) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    const struct zmk_endpoint_instance ep = ev->endpoint;
    bool should_be_active =
        ep.transport == ZMK_TRANSPORT_BLE &&
        ep.ble.profile_index == (ZMK_BLE_PROFILE_COUNT - 1);

    if (should_be_active && !esb_active) {
        esb_endpoint_activate();
    } else if (!should_be_active && esb_active) {
        esb_endpoint_deactivate();
    }

    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(zmk_esb_endpoint_listener, endpoint_listener_cb);
ZMK_SUBSCRIPTION(zmk_esb_endpoint_listener, zmk_endpoint_changed);

/* ---------- init ---------- */

static int esb_endpoint_init(void)
{
    timeslot_handler_init(on_timeslot_evt);
    esb_transport_init(on_transport_evt);
    pairing_init();
    return 0;
}

SYS_INIT(esb_endpoint_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
