/*
 * Copyright (c) 2025 efogdev
 * SPDX-License-Identifier: MIT
 */

#pragma once

typedef enum {
    TS_EVT_STARTED,
    TS_EVT_STOPPED,
} timeslot_evt_t;

typedef void (*timeslot_cb_t)(timeslot_evt_t evt);

void timeslot_handler_init(timeslot_cb_t cb);
void timeslot_handler_start(void);
void timeslot_handler_stop(void);
