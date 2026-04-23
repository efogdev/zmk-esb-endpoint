/*
 * Copyright (c) 2025 efogdev
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/kernel.h>
#include <zephyr/settings/settings.h>
#include <hal/nrf_ficr.h>
#include "pairing.h"
#include "esb_transport.h"
#include <zmk_esb/protocol.h>
#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_SHELL_RELAY)
#include "../shell/shell_relay.h"
#endif

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(zmk_esb_pairing, CONFIG_ZMK_LOG_LEVEL);

#if IS_ENABLED(CONFIG_ZMK_ADAPTIVE_FEEDBACK)
#include <zmk_adaptive_feedback/adaptive_feedback.h>
ZAF_CUSTOM_EVENT_DEFINE(esb_dongle_paired, "esb-paired");
#endif

#define SETTINGS_KEY        "esb_ep/paired"

static pairing_state_t m_state = PAIRING_STATE_IDLE;
static uint8_t m_device_id[6];
static bool m_has_stored_peer;
static struct k_work_delayable beacon_work;

static void save_paired(const uint8_t val) {
    settings_save_one(SETTINGS_KEY, &val, sizeof(val));
    LOG_INF("ESB: pairing state saved (%u)", val);
}

static void build_device_id(void) {
    const uint32_t lo = nrf_ficr_deviceid_get(NRF_FICR, 0);
    const uint32_t hi = nrf_ficr_deviceid_get(NRF_FICR, 1);
    memcpy(&m_device_id[0], &lo, 4);
    memcpy(&m_device_id[4], &hi, 2);
}

static void send_beacon(void) {
    struct esb_pkt_beacon pkt = {
        .type = ESB_PKT_BEACON,
        .caps = ESB_CAP_KEYBOARD | ESB_CAP_CONSUMER | ESB_CAP_MOUSE,
    };
    memcpy(pkt.device_id, m_device_id, sizeof(m_device_id));
    esb_transport_send(0, (uint8_t *)&pkt, sizeof(pkt));
}

static void send_pair_resp(void) {
    struct esb_pkt_pair_resp pkt = {
        .type = ESB_PKT_PAIR_RESP,
        .caps = ESB_CAP_KEYBOARD | ESB_CAP_CONSUMER | ESB_CAP_MOUSE,
    };
    memcpy(pkt.device_id, m_device_id, sizeof(m_device_id));
    esb_transport_send(1, (uint8_t *)&pkt, sizeof(pkt));
}

static int settings_load_cb(const char *name, size_t len, const settings_read_cb read_cb, void *cb_arg) {
    uint8_t val = 0;
    read_cb(cb_arg, &val, sizeof(val));
    m_has_stored_peer = (val == 1);
    return 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(esb_pairing, "esb_ep", NULL, settings_load_cb, NULL, NULL);

static void beacon_work_fn(struct k_work *w) {
    if (m_state != PAIRING_STATE_UNPAIRED) {
        return;
    }
    send_beacon();
    k_work_reschedule(&beacon_work, K_MSEC(CONFIG_ZMK_ESB_ENDPOINT_BEACON_INTERVAL_MS));
}

void pairing_init(void) {
    build_device_id();
    k_work_init_delayable(&beacon_work, beacon_work_fn);
    settings_subsys_init();
    settings_load_subtree("esb_ep");
}

void pairing_start(void) {
    if (m_has_stored_peer) {
        LOG_DBG("ESB: known dongle, going CONNECTED");
        m_state = PAIRING_STATE_CONNECTED;
#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_SHELL_RELAY)
        esb_shell_relay_on_connected();
#endif
    } else {
        LOG_INF("ESB: no dongle, advertising BEACON");
        m_state = PAIRING_STATE_UNPAIRED;
        k_work_reschedule(&beacon_work, K_MSEC(10));
    }
}

void pairing_stop(void) {
    m_state = PAIRING_STATE_IDLE;
    k_work_cancel_delayable(&beacon_work);
#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_SHELL_RELAY)
    esb_shell_relay_on_disconnected();
#endif
}

void pairing_unpair(void) {
    m_has_stored_peer = false;
    save_paired(0);
    if (m_state != PAIRING_STATE_IDLE) {
        k_work_cancel_delayable(&beacon_work);
        m_state = PAIRING_STATE_UNPAIRED;
        k_work_reschedule(&beacon_work, K_MSEC(10));
    }
}

void pairing_on_rx(const uint8_t *data, const uint8_t len) {
    if (len < 1) {
        return;
    }

    switch (data[0]) {
    case ESB_PKT_PAIR_REQ:
        if (m_state == PAIRING_STATE_UNPAIRED) {
            LOG_DBG("ESB: PAIR_REQ received");
            k_work_cancel_delayable(&beacon_work);
            send_pair_resp();
            m_has_stored_peer = true;
            m_state = PAIRING_STATE_CONNECTED;
            LOG_INF("ESB: paired, CONNECTED");
            save_paired(1);
#if IS_ENABLED(CONFIG_ZMK_ADAPTIVE_FEEDBACK)
            zaf_custom_event_trigger(&esb_dongle_paired);
#endif
#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_SHELL_RELAY)
            esb_shell_relay_on_connected();
#endif
        }
        break;

    case ESB_PKT_DISCONNECT:
        LOG_INF("ESB: dongle disconnected");
        m_state = PAIRING_STATE_UNPAIRED;
        m_has_stored_peer = false;
        save_paired(0);
        k_work_reschedule(&beacon_work, K_MSEC(10));
#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_SHELL_RELAY)
        esb_shell_relay_on_disconnected();
#endif
        break;

#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_SHELL_RELAY)
    case ESB_PKT_SHELL_REQ:
        esb_shell_relay_on_req();
        break;

    case ESB_PKT_SHELL_DATA: {
        if (len >= 2) {
            const struct esb_pkt_shell_data *pkt = (const void *)data;
            if (pkt->len > 0 && pkt->len <= ESB_PKT_DATA_MAX) {
                esb_shell_relay_on_data(pkt->data, pkt->len);
            }
        }
        break;
    }
#endif /* CONFIG_ZMK_ESB_ENDPOINT_SHELL_RELAY */

    default:
        break;
    }
}

pairing_state_t pairing_get_state(void) {
    return m_state;
}

bool pairing_is_connected(void) {
    return m_state == PAIRING_STATE_CONNECTED;
}
