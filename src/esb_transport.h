/*
 * Copyright (c) 2025 efogdev
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    ESB_TX_EVT_SUCCESS,
    ESB_TX_EVT_FAIL,
    ESB_RX_EVT,
} esb_transport_evt_type_t;

typedef struct {
    esb_transport_evt_type_t type;
    const uint8_t *rx_buf;
    uint8_t rx_len;
} esb_transport_evt_t;

typedef void (*esb_transport_cb_t)(const esb_transport_evt_t *evt);

int  esb_transport_init(esb_transport_cb_t cb);
void esb_transport_deinit(void);
void esb_transport_set_addresses(const uint8_t base0[4], const uint8_t base1[4], const uint8_t prefixes[8], uint8_t channel);
int  esb_transport_send(uint8_t pipe, const uint8_t *data, uint8_t len);

void esb_transport_on_slot_start(void);
void esb_transport_on_slot_stop(void);
