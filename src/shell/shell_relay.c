/*
 * Copyright (c) 2025 efogdev
 * SPDX-License-Identifier: MIT
 *
 * Keyboard-side ESB shell relay.
 *
 * When the dongle sends ESB_PKT_SHELL_REQ (via ACK payload), the keyboard
 * enters shell relay mode:
 *   - A periodic SHELL_POLL packet is sent every SHELL_POLL_INTERVAL_MS.
 *     The dongle uses the ACK to deliver queued user input.
 *   - Received SHELL_DATA bytes are accumulated until a newline, then executed
 *     via the Zephyr dummy shell backend (same approach as zmk-ble-shell).
 *   - Command output is fragmented into 30-byte SHELL_DATA TX packets sent
 *     back to the dongle. An output ring buffer handles backpressure when the
 *     ESB TX FIFO is full; draining resumes on TX_SUCCESS (notify_tx).
 *   - An inactivity timer (60s default) fires if no command bytes arrive;
 *     the keyboard then sends SHELL_STOP and exits shell mode.
 */

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/shell/shell_dummy.h>
#include <zephyr/sys/ring_buffer.h>
#include <string.h>

#include <zmk_esb/protocol.h>
#include "esb_transport.h"
#include "shell_relay.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(zmk_esb_shell, CONFIG_ZMK_LOG_LEVEL);

static bool m_shell_active;

static char cmd_buf[CONFIG_ZMK_ESB_ENDPOINT_SHELL_CMD_BUF_SIZE];
static size_t cmd_len;
static K_MUTEX_DEFINE(cmd_mutex);

RING_BUF_DECLARE(out_rb, CONFIG_ZMK_ESB_ENDPOINT_SHELL_OUT_BUF_SIZE);

static void shell_exec_work_fn(struct k_work *w);
static void shell_poll_work_fn(struct k_work *w);
static void inactivity_work_fn(struct k_work *w);
static void out_drain_work_fn(struct k_work *w);
static void bg_poll_work_fn(struct k_work *w);

static K_WORK_DEFINE(shell_exec_work, shell_exec_work_fn);
static K_WORK_DELAYABLE_DEFINE(shell_poll_work, shell_poll_work_fn);
static K_WORK_DELAYABLE_DEFINE(inactivity_work, inactivity_work_fn);
static K_WORK_DEFINE(out_drain_work, out_drain_work_fn);
static K_WORK_DELAYABLE_DEFINE(bg_poll_work, bg_poll_work_fn);

static void out_enqueue(const uint8_t *data, size_t len) {
    const uint32_t put = ring_buf_put(&out_rb, data, (uint32_t)len);
    if (put < len) {
        LOG_WRN("out_rb full: dropped %zu bytes", len - put);
    }
}

static const char newline[] = "\r\n";
static void send_prompt(void) {
    static const char prompt[]  = "\033[1;32m" CONFIG_ZMK_ESB_ENDPOINT_SHELL_PROMPT "\033[0m";
    out_enqueue((const uint8_t *)newline, sizeof(newline) - 1);
    out_enqueue((const uint8_t *)prompt, sizeof(prompt) - 1);
    k_work_submit(&out_drain_work);
}

static void out_drain_work_fn(struct k_work *w) {
    ARG_UNUSED(w);
    if (!m_shell_active) {
        return;
    }

    uint8_t *chunk;
    uint32_t got;
    uint32_t sent = 0;

    while ((got = ring_buf_get_claim(&out_rb, &chunk, ESB_PKT_DATA_MAX)) > 0) {
        struct esb_pkt_shell_data pkt = {
            .type = ESB_PKT_SHELL_DATA,
            .len  = (uint8_t)got,
        };
        memcpy(pkt.data, chunk, got);
        memset(pkt.data + got, 0, sizeof(pkt.data) - got);

        const int err = esb_transport_send(1, (uint8_t *)&pkt, sizeof(pkt));
        if (err == -ENOMEM) {
            LOG_DBG("shell TX FIFO full, %u bytes deferred", got);
            ring_buf_get_finish(&out_rb, 0);
            break;
        } else if (err) {
            LOG_WRN("shell TX err %d, dropping %u bytes", err, got);
            ring_buf_get_finish(&out_rb, got);
            break;
        }
        ring_buf_get_finish(&out_rb, got);
        sent += got;
    }

    if (sent > 0) {
        LOG_DBG("shell TX: drained %u bytes to dongle", sent);
    }
}

static void shell_exec_work_fn(struct k_work *w) {
    ARG_UNUSED(w);

    char cmd[CONFIG_ZMK_ESB_ENDPOINT_SHELL_CMD_BUF_SIZE];
    k_mutex_lock(&cmd_mutex, K_FOREVER);
    strncpy(cmd, cmd_buf, sizeof(cmd));
    cmd[sizeof(cmd) - 1] = '\0';
    k_mutex_unlock(&cmd_mutex);

    char *start = cmd;
    while (*start == ' ' || *start == '\t') {
        start++;
    }
    char *end = start + strlen(start);
    while (end > start && (*(end - 1) == ' ' || *(end - 1) == '\t')) {
        end--;
    }
    *end = '\0';

    LOG_DBG("ESB shell: exec '%s'", start);

    const struct shell *sh = shell_backend_dummy_get_ptr();
    shell_backend_dummy_clear_output(sh);
    const int ret = shell_execute_cmd(sh, start);

    size_t out_len;
    const char *out = shell_backend_dummy_get_output(sh, &out_len);

    LOG_DBG("ESB shell: ret=%d out_len=%zu", ret, out_len);
    if (ret == -ENOEXEC) {
        char msg[CONFIG_ZMK_ESB_ENDPOINT_SHELL_CMD_BUF_SIZE + 24];
        const int n = snprintk(msg, sizeof(msg), ";31m%s: command not found", start);
        if (n > 0) {
            out_enqueue((const uint8_t *)msg, (size_t)n);
        }
    } else {
        if (out_len > 0) {
            out_enqueue((const uint8_t *)out, out_len);
        }
        if (ret != 0) {
            out_enqueue((const uint8_t *)"\r\n", 2);
        }
    }

    send_prompt();
}

static void shell_poll_work_fn(struct k_work *w) {
    ARG_UNUSED(w);
    if (!m_shell_active) {
        return;
    }
    struct esb_pkt_shell_poll pkt = { .type = ESB_PKT_SHELL_POLL };
    const int err = esb_transport_send(1, (uint8_t *)&pkt, sizeof(pkt));
    if (err && err != -ENOMEM) {
        LOG_DBG("SHELL_POLL TX err %d", err);
    }
    k_work_reschedule(&shell_poll_work, K_MSEC(CONFIG_ZMK_ESB_ENDPOINT_SHELL_POLL_INTERVAL_MS));
}

static void inactivity_work_fn(struct k_work *w) {
    ARG_UNUSED(w);
    LOG_DBG("ESB shell relay: inactivity timeout (%ds), stopping",
            CONFIG_ZMK_ESB_ENDPOINT_SHELL_INACTIVITY_S);
    m_shell_active = false;
    k_work_cancel_delayable(&shell_poll_work);
    ring_buf_reset(&out_rb);
    struct esb_pkt_shell_stop pkt = { .type = ESB_PKT_SHELL_STOP };
    esb_transport_send(1, (uint8_t *)&pkt, sizeof(pkt));
}

static void bg_poll_work_fn(struct k_work *w) {
    ARG_UNUSED(w);
    if (m_shell_active) {
        return;
    }
    struct esb_pkt_shell_poll pkt = { .type = ESB_PKT_SHELL_POLL };
    esb_transport_send(1, (uint8_t *)&pkt, sizeof(pkt));
    k_work_reschedule(&bg_poll_work, K_MSEC(500));
}

void esb_shell_relay_init(void) {
    LOG_DBG("ESB shell relay: init");
}

void esb_shell_relay_on_connected(void) {
    LOG_DBG("ESB shell relay: connected, starting background ACK poll");
    k_work_reschedule(&bg_poll_work, K_MSEC(500));
}

void esb_shell_relay_on_disconnected(void) {
    LOG_DBG("ESB shell relay: disconnected");
    k_work_cancel_delayable(&bg_poll_work);
    if (!m_shell_active) {
        return;
    }
    m_shell_active = false;
    k_work_cancel_delayable(&shell_poll_work);
    k_work_cancel_delayable(&inactivity_work);
    ring_buf_reset(&out_rb);
}

void esb_shell_relay_on_req(void) {
    if (m_shell_active) {
        LOG_DBG("ESB shell relay: SHELL_REQ but already active");
        return;
    }
    LOG_DBG("ESB shell relay: SHELL_REQ received, starting");
    m_shell_active = true;
    cmd_len = 0;
    ring_buf_reset(&out_rb);
    k_work_cancel_delayable(&bg_poll_work);
    k_work_reschedule(&shell_poll_work, K_MSEC(20));
    if (CONFIG_ZMK_ESB_ENDPOINT_SHELL_INACTIVITY_S > 0) {
        k_work_reschedule(&inactivity_work, K_SECONDS(CONFIG_ZMK_ESB_ENDPOINT_SHELL_INACTIVITY_S));
    }
    send_prompt();
}

void esb_shell_relay_on_data(const uint8_t *data, const uint8_t len) {
    if (!m_shell_active || len == 0) {
        LOG_DBG("ESB shell: SHELL_DATA ignored (active=%d len=%u)", m_shell_active, len);
        return;
    }
    LOG_DBG("ESB shell: RX %u cmd bytes from dongle", len);
    if (CONFIG_ZMK_ESB_ENDPOINT_SHELL_INACTIVITY_S > 0) {
        k_work_reschedule(&inactivity_work, K_SECONDS(CONFIG_ZMK_ESB_ENDPOINT_SHELL_INACTIVITY_S));
    }
    k_mutex_lock(&cmd_mutex, K_FOREVER);
    for (uint8_t i = 0; i < len; i++) {
        const uint8_t b = data[i];
        if (b == '\n' || b == '\r') {
            if (cmd_len > 0) {
                cmd_buf[cmd_len] = '\0';
                cmd_len = 0;
                k_mutex_unlock(&cmd_mutex);
                LOG_DBG("ESB shell: command ready, submitting exec work");
                k_work_submit(&shell_exec_work);
                k_mutex_lock(&cmd_mutex, K_FOREVER);
            }
        } else if (cmd_len < sizeof(cmd_buf) - 1) {
            cmd_buf[cmd_len++] = (char)b;
        } else {
            LOG_WRN("ESB shell: cmd_buf overflow, byte 0x%02x dropped", b);
        }
    }
    k_mutex_unlock(&cmd_mutex);
}

bool esb_shell_relay_is_active(void) {
    return m_shell_active;
}

void esb_shell_relay_notify_tx(void) {
    if (m_shell_active && ring_buf_size_get(&out_rb) > 0) {
        LOG_DBG("ESB shell: notify_tx, %u bytes pending", ring_buf_size_get(&out_rb));
        k_work_submit(&out_drain_work);
    }
}
