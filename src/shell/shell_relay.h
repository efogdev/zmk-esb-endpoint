/*
 * Copyright (c) 2025 efogdev
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

void esb_shell_relay_init(void);
void esb_shell_relay_on_connected(void);
void esb_shell_relay_on_disconnected(void);
void esb_shell_relay_on_req(void);
void esb_shell_relay_on_data(const uint8_t *data, uint8_t len);
bool esb_shell_relay_is_active(void);
void esb_shell_relay_notify_tx(void);
