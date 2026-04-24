/*
 * Copyright (c) 2025 efogdev
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/kernel.h>
#include <string.h>
#include <esb.h>
#include <zmk_esb/protocol.h>
#include "bench.h"
#include "esb_transport.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(zmk_esb_bench, CONFIG_ZMK_LOG_LEVEL);

#define BENCH_DURATION_S        CONFIG_ZMK_ESB_ENDPOINT_BENCH_DURATION_S
#define BENCH_RESULT_POLL_MS    CONFIG_ZMK_ESB_ENDPOINT_BENCH_RESULT_POLL_MS
#define BENCH_RESULT_TIMEOUT_MS CONFIG_ZMK_ESB_ENDPOINT_BENCH_RESULT_TIMEOUT_MS

static enum esb_bench_state m_state = BENCH_IDLE;
static int64_t m_start_ms;
static uint32_t m_tx_ok;
static uint32_t m_tx_fail;
static uint8_t m_seq;
static struct esb_bench_result m_result;

static void bench_tx_fn(struct k_work *w);
static void bench_complete_fn(struct k_work *w);
static void bench_result_poll_fn(struct k_work *w);
static void bench_result_timeout_fn(struct k_work *w);

static K_WORK_DEFINE(bench_tx_work, bench_tx_fn);
static K_WORK_DELAYABLE_DEFINE(bench_complete_work, bench_complete_fn);
static K_WORK_DELAYABLE_DEFINE(bench_result_poll_work, bench_result_poll_fn);
static K_WORK_DELAYABLE_DEFINE(bench_result_timeout_work, bench_result_timeout_fn);

static void bench_tx_fn(struct k_work *w) {
    ARG_UNUSED(w);
    if (m_state != BENCH_RUNNING) {
        return;
    }
    struct esb_payload pkt = {
        .pipe   = 1,
        .length = sizeof(struct esb_pkt_bench_ping),
        .noack  = 0,
    };
    struct esb_pkt_bench_ping ping = { .type = ESB_PKT_BENCH_PING };
    do {
        ping.seq = m_seq++;
        memcpy(pkt.data, &ping, sizeof(ping));
    } while (esb_write_payload(&pkt) == 0);
}

static void bench_complete_fn(struct k_work *w) {
    ARG_UNUSED(w);
    if (m_state != BENCH_RUNNING) {
        return;
    }
    k_work_cancel(&bench_tx_work);

    /* Switch to slow polling so the dongle's rx_thread can process BENCH_STOP
     * without being overwritten by a subsequent BENCH_PING. The dongle queues
     * BENCH_RESULT after the first BENCH_STOP; the ACK to the next poll
     * carries it back. */
    const struct esb_pkt_bench_stop stop = { .type = ESB_PKT_BENCH_STOP };
    esb_transport_send(ESB_PIPE_DATA, (const uint8_t *)&stop, sizeof(stop));

    m_state = BENCH_STOPPING;
    k_work_reschedule(&bench_result_poll_work, K_MSEC(BENCH_RESULT_POLL_MS));
    k_work_reschedule(&bench_result_timeout_work, K_MSEC(BENCH_RESULT_TIMEOUT_MS));
    LOG_DBG("Bench complete: polling for dongle result every %d ms", BENCH_RESULT_POLL_MS);
}

static void bench_result_poll_fn(struct k_work *w) {
    ARG_UNUSED(w);
    if (m_state != BENCH_STOPPING) {
        return;
    }
    const struct esb_pkt_bench_stop stop = { .type = ESB_PKT_BENCH_STOP };
    esb_transport_send(ESB_PIPE_DATA, (const uint8_t *)&stop, sizeof(stop));
    k_work_reschedule(&bench_result_poll_work, K_MSEC(BENCH_RESULT_POLL_MS));
}

static void bench_result_timeout_fn(struct k_work *w) {
    ARG_UNUSED(w);
    if (m_state != BENCH_STOPPING) {
        return;
    }
    k_work_cancel_delayable(&bench_result_poll_work);
    LOG_WRN("Bench: no BENCH_RESULT from dongle (timeout)");
    m_result.tx_ok            = m_tx_ok;
    m_result.tx_fail          = m_tx_fail;
    m_result.throughput_pkt_s = m_tx_ok / BENCH_DURATION_S;
    m_result.rssi_samples     = 0;
    m_state = BENCH_DONE;
}

void esb_bench_start(void) {
    m_state    = BENCH_RUNNING;
    m_start_ms = k_uptime_get();
    m_tx_ok    = 0;
    m_tx_fail  = 0;
    m_seq      = 0;
    memset(&m_result, 0, sizeof(m_result));
    k_work_submit(&bench_tx_work);
    k_work_reschedule(&bench_complete_work, K_SECONDS(BENCH_DURATION_S));
    LOG_INF("ESB benchmark started (%d s)", BENCH_DURATION_S);
}

void esb_bench_reset(void) {
    k_work_cancel(&bench_tx_work);
    k_work_cancel_delayable(&bench_complete_work);
    k_work_cancel_delayable(&bench_result_poll_work);
    k_work_cancel_delayable(&bench_result_timeout_work);
    m_state = BENCH_IDLE;
}

enum esb_bench_state esb_bench_get_state(void) {
    return m_state;
}

void esb_bench_get_result(struct esb_bench_result *result) {
    *result = m_result;
}

void esb_bench_on_rx(const uint8_t *data, const uint8_t len) {
    if (len < 1 || data[0] != ESB_PKT_BENCH_RESULT) {
        return;
    }
    if (m_state != BENCH_STOPPING) {
        return;
    }
    if (len < sizeof(struct esb_pkt_bench_result)) {
        LOG_WRN("Bench: BENCH_RESULT too short (%u)", len);
        return;
    }
    k_work_cancel_delayable(&bench_result_poll_work);
    k_work_cancel_delayable(&bench_result_timeout_work);

    const struct esb_pkt_bench_result *r = (const struct esb_pkt_bench_result *)data;
    m_result.tx_ok            = m_tx_ok;
    m_result.tx_fail          = m_tx_fail;
    m_result.throughput_pkt_s = m_tx_ok / BENCH_DURATION_S;
    m_result.rssi_avg         = r->rssi_avg;
    m_result.rssi_min         = r->rssi_min;
    m_result.rssi_max         = r->rssi_max;
    m_result.rssi_samples     = r->rx_count;
    m_state = BENCH_DONE;
    LOG_INF("Bench done: %u pkt/s, RSSI avg=%d dBm (n=%u)",
            m_result.throughput_pkt_s, m_result.rssi_avg, m_result.rssi_samples);
}

void esb_bench_notify_tx_success(void) {
    if (m_state == BENCH_RUNNING) {
        m_tx_ok++;
        k_work_submit(&bench_tx_work);
    }
}

void esb_bench_notify_tx_fail(void) {
    if (m_state == BENCH_RUNNING) {
        m_tx_fail++;
        k_work_submit(&bench_tx_work);
    }
}
