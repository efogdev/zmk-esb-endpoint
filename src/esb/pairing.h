/*
 * Copyright (c) 2025 efogdev
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    PAIRING_STATE_IDLE,
    PAIRING_STATE_UNPAIRED,
    PAIRING_STATE_PAIRING,
    PAIRING_STATE_VERIFYING,  /* has stored peer, awaiting device_id match */
    PAIRING_STATE_CONNECTED,
} pairing_state_t;

void pairing_init(void);
void pairing_start(void);
void pairing_stop(void);
void pairing_unpair(void);
void pairing_on_rx(const uint8_t *data, uint8_t len);

pairing_state_t pairing_get_state(void);
bool pairing_is_connected(void);
