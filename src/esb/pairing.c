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
#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_HOP)
#include "channel_hop_ep.h"
#endif

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(zmk_esb_pairing, CONFIG_ZMK_LOG_LEVEL);

#if IS_ENABLED(CONFIG_ZMK_ADAPTIVE_FEEDBACK)
#include <zmk_adaptive_feedback/adaptive_feedback.h>
ZAF_CUSTOM_EVENT_DEFINE(esb_dongle_paired, "esb-paired");
#endif

#define SETTINGS_KEY        "esb_ep/paired"

/* Persisted record. A single settings_save_one keeps updates atomic. */
struct esb_ep_paired_rec {
    uint8_t paired;
    uint8_t device_id[6];
} __attribute__((__packed__));

static pairing_state_t m_state = PAIRING_STATE_IDLE;
static uint8_t m_device_id[6];
static uint8_t m_peer_device_id[6];
static bool m_has_stored_peer;
static struct k_work_delayable beacon_work;
static struct k_work_delayable verify_work;

static void save_paired(const bool paired) {
    struct esb_ep_paired_rec rec = { .paired = paired ? 1 : 0 };
    if (paired) {
        memcpy(rec.device_id, m_peer_device_id, sizeof(rec.device_id));
    }
    settings_save_one(SETTINGS_KEY, &rec, sizeof(rec));
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
    esb_transport_send(ESB_PIPE_PAIRING, (uint8_t *)&pkt, sizeof(pkt));
}

static void send_pair_resp(void) {
    struct esb_pkt_pair_resp pkt = {
        .type = ESB_PKT_PAIR_RESP,
        .caps = ESB_CAP_KEYBOARD | ESB_CAP_CONSUMER | ESB_CAP_MOUSE,
    };
    memcpy(pkt.device_id, m_device_id, sizeof(m_device_id));
    esb_transport_send(ESB_PIPE_DATA, (uint8_t *)&pkt, sizeof(pkt));
}

static int settings_load_cb(const char *name, size_t len, const settings_read_cb read_cb, void *cb_arg) {
    if (len != sizeof(struct esb_ep_paired_rec)) {
        LOG_WRN("unexpected settings record length %zu, dropping", len);
        return 0;
    }
    struct esb_ep_paired_rec rec;
    read_cb(cb_arg, &rec, sizeof(rec));
    m_has_stored_peer = (rec.paired == 1);
    memcpy(m_peer_device_id, rec.device_id, sizeof(m_peer_device_id));
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

static void verify_work_fn(struct k_work *w) {
    if (m_state != PAIRING_STATE_VERIFYING) {
        return;
    }
    struct esb_pkt_verify_req req = { .type = ESB_PKT_VERIFY_REQ };
    memcpy(req.device_id, m_device_id, sizeof(req.device_id));
    esb_transport_send(ESB_PIPE_DATA, (uint8_t *)&req, sizeof(req));
    k_work_reschedule(&verify_work, K_MSEC(CONFIG_ZMK_ESB_ENDPOINT_VERIFY_INTERVAL_MS));
}

void pairing_init(void) {
    build_device_id();
    k_work_init_delayable(&beacon_work, beacon_work_fn);
    k_work_init_delayable(&verify_work, verify_work_fn);
    settings_subsys_init();
    settings_load_subtree("esb_ep");
}

void pairing_start(void) {
    if (m_has_stored_peer) {
        LOG_DBG("known dongle %02X%02X%02X%02X%02X%02X, VERIFYING",
                m_peer_device_id[0], m_peer_device_id[1], m_peer_device_id[2],
                m_peer_device_id[3], m_peer_device_id[4], m_peer_device_id[5]);
        m_state = PAIRING_STATE_VERIFYING;
        k_work_reschedule(&verify_work, K_NO_WAIT);
    } else {
        LOG_INF("no dongle, advertising BEACON");
        m_state = PAIRING_STATE_UNPAIRED;
        k_work_reschedule(&beacon_work, K_MSEC(CONFIG_ZMK_ESB_ENDPOINT_BEACON_INITIAL_DELAY_MS));
    }
}

void pairing_stop(void) {
    m_state = PAIRING_STATE_IDLE;
    k_work_cancel_delayable(&beacon_work);
    k_work_cancel_delayable(&verify_work);
#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_SHELL_RELAY)
    esb_shell_relay_on_disconnected();
#endif
#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_HOP)
    channel_hop_ep_on_disconnected();
#endif
}

void pairing_unpair(void) {
    m_has_stored_peer = false;
    memset(m_peer_device_id, 0, sizeof(m_peer_device_id));
    save_paired(false);
    k_work_cancel_delayable(&verify_work);
    if (m_state != PAIRING_STATE_IDLE) {
        k_work_cancel_delayable(&beacon_work);
        m_state = PAIRING_STATE_UNPAIRED;
        k_work_reschedule(&beacon_work, K_MSEC(CONFIG_ZMK_ESB_ENDPOINT_BEACON_INITIAL_DELAY_MS));
    }
}

void pairing_on_rx(const uint8_t *data, const uint8_t len) {
    if (len < 1) {
        return;
    }

    switch (data[0]) {
    case ESB_PKT_PAIR_REQ:
        if (m_state == PAIRING_STATE_UNPAIRED && len >= sizeof(struct esb_pkt_pair_req)) {
            const struct esb_pkt_pair_req *req = (const void *)data;
            memcpy(m_peer_device_id, req->device_id, sizeof(m_peer_device_id));
            LOG_DBG("PAIR_REQ received");
            k_work_cancel_delayable(&beacon_work);
            send_pair_resp();
            m_has_stored_peer = true;
            m_state = PAIRING_STATE_CONNECTED;
            LOG_INF("paired with %02X%02X%02X%02X%02X%02X, connected",
                    m_peer_device_id[0], m_peer_device_id[1], m_peer_device_id[2],
                    m_peer_device_id[3], m_peer_device_id[4], m_peer_device_id[5]);
            save_paired(true);
#if IS_ENABLED(CONFIG_ZMK_ADAPTIVE_FEEDBACK)
            zaf_custom_event_trigger(&esb_dongle_paired);
#endif
#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_SHELL_RELAY)
            esb_shell_relay_on_connected();
#endif
#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_HOP)
            channel_hop_ep_on_connected();
#endif
        }
        break;

    case ESB_PKT_VERIFY_RESP:
        if (m_state == PAIRING_STATE_VERIFYING && len >= sizeof(struct esb_pkt_verify_resp)) {
            const struct esb_pkt_verify_resp *resp = (const void *)data;
            if (memcmp(resp->device_id, m_peer_device_id, sizeof(m_peer_device_id)) == 0) {
                k_work_cancel_delayable(&verify_work);
                m_state = PAIRING_STATE_CONNECTED;
                LOG_INF("dongle connected");
#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_SHELL_RELAY)
                esb_shell_relay_on_connected();
#endif
#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_HOP)
                channel_hop_ep_on_connected();
#endif
            } else {
                LOG_DBG("VERIFY_RESP mismatch: expected %02X%02X%02X%02X%02X%02X got %02X%02X%02X%02X%02X%02X",
                        m_peer_device_id[0], m_peer_device_id[1], m_peer_device_id[2],
                        m_peer_device_id[3], m_peer_device_id[4], m_peer_device_id[5],
                        resp->device_id[0], resp->device_id[1], resp->device_id[2],
                        resp->device_id[3], resp->device_id[4], resp->device_id[5]);
            }
        }
        break;

    case ESB_PKT_DISCONNECT:
        LOG_INF("dongle disconnected");
        m_state = PAIRING_STATE_UNPAIRED;
        m_has_stored_peer = false;
        memset(m_peer_device_id, 0, sizeof(m_peer_device_id));
        save_paired(false);
        k_work_cancel_delayable(&verify_work);
        k_work_reschedule(&beacon_work, K_MSEC(CONFIG_ZMK_ESB_ENDPOINT_BEACON_INITIAL_DELAY_MS));
#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_SHELL_RELAY)
        esb_shell_relay_on_disconnected();
#endif
#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_HOP)
        channel_hop_ep_on_disconnected();
#endif
        break;

    case ESB_PKT_RESYNC:
        /* Dongle rebooted and wants us to re-run the VERIFY handshake. Keep
         * the stored peer and drop back to VERIFYING; verify_work will send
         * VERIFY_REQ, which the dongle (still in STATE_VERIFYING) answers
         * with VERIFY_RESP, returning us to CONNECTED without a re-pair. */
        if (m_state == PAIRING_STATE_CONNECTED) {
            LOG_INF("dongle requested RESYNC, re-verifying");
            m_state = PAIRING_STATE_VERIFYING;
            k_work_reschedule(&verify_work, K_NO_WAIT);
#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_SHELL_RELAY)
            esb_shell_relay_on_disconnected();
#endif
#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_HOP)
            channel_hop_ep_on_disconnected();
#endif
        }
        break;

#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_HOP)
    case ESB_PKT_CHANNEL_HOP_CONFIRM:
        channel_hop_ep_on_rx(data, len);
        break;

    case ESB_PKT_CHANNEL_HOP_REQUEST:
        channel_hop_ep_on_request();
        break;
#endif

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
