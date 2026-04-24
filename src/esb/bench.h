/*
 * Copyright (c) 2025 efogdev
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <stdint.h>

enum esb_bench_state {
    BENCH_IDLE,
    BENCH_RUNNING,
    BENCH_STOPPING,
    BENCH_DONE,
};

struct esb_bench_result {
    uint32_t tx_ok;
    uint32_t tx_fail;
    uint32_t throughput_pkt_s;
    int8_t   rssi_avg;
    int8_t   rssi_min;
    int8_t   rssi_max;
    uint32_t rssi_samples;
};

void esb_bench_start(void);
void esb_bench_reset(void);
enum esb_bench_state esb_bench_get_state(void);
void esb_bench_get_result(struct esb_bench_result *result);
void esb_bench_on_rx(const uint8_t *data, uint8_t len);
void esb_bench_notify_tx_success(void);
void esb_bench_notify_tx_fail(void);
