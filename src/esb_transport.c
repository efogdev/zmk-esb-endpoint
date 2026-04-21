/*
 * Copyright (c) 2025 efogdev
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/kernel.h>
#include <esb.h>
#include "esb_transport.h"
#include <zmk_esb/protocol.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(zmk_esb_transport, CONFIG_ZMK_LOG_LEVEL);

static esb_transport_cb_t m_cb;
static struct esb_payload m_rx_payload;

/* TX queue: ISR-safe, used from both thread and IRQ contexts */
K_MSGQ_DEFINE(tx_msgq, sizeof(struct esb_payload),
              CONFIG_ZMK_ESB_ENDPOINT_TX_QUEUE_SIZE, 4);

/* Address config applied on each slot init */
static struct {
    uint8_t base0[4];
    uint8_t base1[4];
    uint8_t prefixes[8];
    uint8_t channel;
    bool configured;
} m_addr;

/* RX dispatch via work item (IRQ → thread) */
static struct k_work rx_work;
static uint8_t rx_buf[ESB_MAX_PAYLOAD_LEN];
static uint8_t rx_len;

static void rx_work_fn(struct k_work *w)
{
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

static void esb_event_handler(struct esb_evt const *event)
{
    struct esb_payload tmp;

    switch (event->evt_id) {
    case ESB_EVENT_TX_SUCCESS:
        k_msgq_get(&tx_msgq, &tmp, K_NO_WAIT);
        if (k_msgq_peek(&tx_msgq, &tmp) == 0) {
            esb_write_payload(&tmp);
        }
        break;

    case ESB_EVENT_TX_FAILED:
        esb_flush_tx();
        if (k_msgq_peek(&tx_msgq, &tmp) == 0) {
            esb_write_payload(&tmp);
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

static int esb_init_and_configure(void)
{
    struct esb_config cfg = ESB_DEFAULT_CONFIG;
    cfg.protocol           = ESB_PROTOCOL_ESB_DPL;
    cfg.mode               = ESB_MODE_PTX;
    cfg.bitrate            = ESB_BITRATE_2MBPS;
    cfg.crc                = ESB_CRC_16BIT;
    cfg.retransmit_count   = 3;
    cfg.retransmit_delay   = 600;
    cfg.tx_mode            = ESB_TXMODE_AUTO;
    cfg.use_fast_ramp_up   = true;
    cfg.selective_auto_ack = false;
    cfg.event_handler      = esb_event_handler;
    cfg.payload_length     = ESB_MAX_PAYLOAD_LEN;

    int err = esb_init(&cfg);
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

/* ---------- public API ---------- */

int esb_transport_init(esb_transport_cb_t cb)
{
    m_cb = cb;
    k_work_init(&rx_work, rx_work_fn);
    return 0;
}

void esb_transport_set_addresses(const uint8_t base0[4], const uint8_t base1[4],
                                 const uint8_t prefixes[8], uint8_t channel)
{
    memcpy(m_addr.base0, base0, 4);
    memcpy(m_addr.base1, base1, 4);
    memcpy(m_addr.prefixes, prefixes, 8);
    m_addr.channel = channel;
    m_addr.configured = true;
}

void esb_transport_deinit(void)
{
    m_cb = NULL;
    k_msgq_purge(&tx_msgq);
    m_addr.configured = false;
}

int esb_transport_send(uint8_t pipe, const uint8_t *data, uint8_t len)
{
    if (len > ESB_MAX_PAYLOAD_LEN) {
        return -EMSGSIZE;
    }
    struct esb_payload pkt = {
        .pipe   = pipe,
        .length = len,
    };
    memcpy(pkt.data, data, len);
    return k_msgq_put(&tx_msgq, &pkt, K_NO_WAIT);
}

void esb_transport_on_slot_start(void)
{
    int err = esb_init_and_configure();
    if (err) {
        LOG_ERR("esb init: %d", err);
        return;
    }

    struct esb_payload tmp;
    if (k_msgq_peek(&tx_msgq, &tmp) == 0) {
        esb_write_payload(&tmp);
        /* ESB_TXMODE_AUTO will start transmission automatically */
    }
}

void esb_transport_on_slot_stop(void)
{
    esb_disable();
}
