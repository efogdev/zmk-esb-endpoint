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
 *   - Command output is fragmented into SHELL_DATA TX packets sent
 *     back to the dongle. An output ring buffer handles backpressure when the
 *     ESB TX FIFO is full; draining resumes on TX_SUCCESS (notify_tx).
 *   - An inactivity timer fires if no command bytes arrive;
 *     the keyboard then sends SHELL_STOP and exits shell mode.
 */

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/shell/shell_dummy.h>
#include <zephyr/sys/ring_buffer.h>
#include <string.h>

#include <zmk_esb/protocol.h>
#include <zmk_esb/endpoint.h>
#include "esb_transport.h"
#include "shell_relay.h"

#include <zephyr/logging/log.h>
#include <zephyr/logging/log_backend.h>
#include <zephyr/logging/log_output.h>
#include <zephyr/logging/log_core.h>
LOG_MODULE_REGISTER(zmk_esb_shell, CONFIG_ZMK_ESB_ENDPOINT_LOG_LEVEL);

static bool m_shell_active;

static char cmd_buf[CONFIG_ZMK_ESB_ENDPOINT_SHELL_CMD_BUF_SIZE];
static size_t cmd_len;
static K_MUTEX_DEFINE(cmd_mutex);

RING_BUF_DECLARE(out_rb, CONFIG_ZMK_ESB_ENDPOINT_SHELL_OUT_BUF_SIZE);

static void shell_exec_work_fn(struct k_work *w);
static void shell_poll_work_fn(struct k_work *w);
static void inactivity_work_fn(struct k_work *w);
static void out_drain_work_fn(struct k_work *w);
static void stdout_flush_work_fn(struct k_work *w);
static void stdout_buf_reset(void);
#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_SHELL_BG_POLL)
static void bg_poll_work_fn(struct k_work *w);
#endif

static K_WORK_DEFINE(shell_exec_work, shell_exec_work_fn);
static K_WORK_DELAYABLE_DEFINE(shell_poll_work, shell_poll_work_fn);
static K_WORK_DELAYABLE_DEFINE(inactivity_work, inactivity_work_fn);
static K_WORK_DEFINE(out_drain_work, out_drain_work_fn);
static K_WORK_DELAYABLE_DEFINE(stdout_flush_work, stdout_flush_work_fn);
#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_SHELL_BG_POLL)
static K_WORK_DELAYABLE_DEFINE(bg_poll_work, bg_poll_work_fn);
#endif

/* Coalescing buffer for the stdout hook. printk/puts feed the hook one byte
 * at a time; without coalescing every byte would produce its own
 * out_enqueue + out_drain_work submission, fragmenting the stream into
 * one ESB packet per byte. Bytes are flushed when the buffer fills or
 * after SHELL_STDOUT_COALESCE_MS of hook inactivity. */
static uint8_t stdout_buf[CONFIG_ZMK_ESB_ENDPOINT_SHELL_STDOUT_COALESCE_BYTES];
static size_t stdout_buf_len;
static struct k_spinlock stdout_buf_lock;

/* Dedicated work queue for shell_exec_work. Keeps long-running NVS-bound
 * commands (e.g. settings_load_subtree_direct) off the system workqueue so
 * shell_poll_work / out_drain_work / radio housekeeping keep firing while a
 * command runs, and gives shell_execute_cmd enough stack to absorb VLA-using
 * settings callbacks that overflow the system workqueue's 1 KB default. */
static K_THREAD_STACK_DEFINE(shell_exec_stack,
                             CONFIG_ZMK_ESB_ENDPOINT_SHELL_EXEC_STACK_SIZE);
static struct k_work_q shell_exec_q;

/* Tracks the column the dongle's CDC cursor will be on after the bytes
 * currently queued in out_rb are flushed. Used to decide whether a log
 * message needs a leading CRLF to escape the prompt or mid-typed input.
 * Only reflects what the keyboard has emitted — the dongle's local input
 * echo is not visible here, so this is best-effort. */
static bool m_at_line_start = true;

static void out_enqueue(const uint8_t *data, const size_t len) {
    const uint32_t put = ring_buf_put(&out_rb, data, (uint32_t)len);
    if (put < len) {
        LOG_WRN("out_rb full: dropped %zu bytes", len - put);
    }
    if (put > 0) {
        m_at_line_start = (data[put - 1] == '\n');
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

        const int err = esb_transport_send(ESB_PIPE_DATA, (uint8_t *)&pkt, sizeof(pkt));
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

    LOG_DBG("exec '%s'", start);

    const struct shell *sh = shell_backend_dummy_get_ptr();
    shell_backend_dummy_clear_output(sh);
    const int ret = shell_execute_cmd(sh, start);

    size_t out_len;
    const char *out = shell_backend_dummy_get_output(sh, &out_len);

    LOG_DBG("ret=%d out_len=%zu", ret, out_len);
    if (ret == -ENOEXEC) {
        char msg[CONFIG_ZMK_ESB_ENDPOINT_SHELL_CMD_BUF_SIZE + 32];
        const int n = snprintk(msg, sizeof(msg), "\r\n\033[31m%s: command not found\033[0m", start);
        if (n > 0) {
            out_enqueue((const uint8_t *)msg, (size_t)n);
        }
    } else {
        if (out_len > 0) {
            out_enqueue((const uint8_t *)out, out_len);
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
    const int err = esb_transport_send(ESB_PIPE_DATA, (uint8_t *)&pkt, sizeof(pkt));
    if (err && err != -ENOMEM) {
        LOG_DBG("SHELL_POLL TX err %d", err);
    }
    k_work_reschedule(&shell_poll_work, K_MSEC(CONFIG_ZMK_ESB_ENDPOINT_SHELL_POLL_INTERVAL_MS));
}

static void inactivity_work_fn(struct k_work *w) {
    ARG_UNUSED(w);
    LOG_DBG("inactivity timeout (%ds), stopping",
            CONFIG_ZMK_ESB_ENDPOINT_SHELL_INACTIVITY_S);
    m_shell_active = false;
    k_work_cancel_delayable(&shell_poll_work);
    stdout_buf_reset();
    ring_buf_reset(&out_rb);
    struct esb_pkt_shell_stop pkt = { .type = ESB_PKT_SHELL_STOP };
    esb_transport_send(ESB_PIPE_DATA, (uint8_t *)&pkt, sizeof(pkt));
}

#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_SHELL_BG_POLL)
static void bg_poll_work_fn(struct k_work *w) {
    ARG_UNUSED(w);
    if (m_shell_active) {
        return;
    }
    const uint32_t threshold = CONFIG_ZMK_ESB_ENDPOINT_SHELL_BG_POLL_ACTIVITY_THRESHOLD_MS;
    if (threshold == 0 || esb_transport_ms_since_activity() <= threshold) {
        struct esb_pkt_shell_bg_poll pkt = { .type = ESB_PKT_SHELL_BG_POLL };
        esb_transport_send(ESB_PIPE_DATA, (uint8_t *)&pkt, sizeof(pkt));
    }
    k_work_reschedule(&bg_poll_work, K_MSEC(CONFIG_ZMK_ESB_ENDPOINT_SHELL_BG_POLL_MS));
}
#endif

static bool in_log_emit;
static uint8_t log_line_buf[CONFIG_ZMK_ESB_ENDPOINT_SHELL_LOG_LINE_BUF_SIZE];

static int log_out_func(uint8_t *buf, const size_t size, void *ctx) {
    ARG_UNUSED(ctx);
    if (m_shell_active) {
        out_enqueue(buf, size);
        k_work_submit(&out_drain_work);
    }
    return (int)size;
}

LOG_OUTPUT_DEFINE(log_output_esb, log_out_func, log_line_buf, sizeof(log_line_buf));

static const uint32_t ESB_LOG_FLAGS =
    LOG_OUTPUT_FLAG_LEVEL | LOG_OUTPUT_FLAG_COLORS |
    LOG_OUTPUT_FLAG_TIMESTAMP | LOG_OUTPUT_FLAG_FORMAT_TIMESTAMP;

static void esb_log_backend_process(const struct log_backend *const backend,
                                    union log_msg_generic *msg) {
    ARG_UNUSED(backend);
    if (!m_shell_active || in_log_emit) {
        return;
    }
    in_log_emit = true;
    if (!m_at_line_start) {
        static const uint8_t crlf[] = "\r\n";
        out_enqueue(crlf, sizeof(crlf) - 1);
    }
    log_output_msg_process(&log_output_esb, &msg->log, ESB_LOG_FLAGS);
    in_log_emit = false;
}

static void esb_log_backend_dropped(const struct log_backend *const backend, const uint32_t cnt) {
    ARG_UNUSED(backend);
    log_output_dropped_process(&log_output_esb, cnt);
}

static void esb_log_backend_panic(const struct log_backend *const backend) {
    ARG_UNUSED(backend);
    log_output_flush(&log_output_esb);
}

static int esb_log_backend_format_set(const struct log_backend *const backend, const uint32_t type) {
    ARG_UNUSED(backend);
    ARG_UNUSED(type);
    return 0;
}

static const struct log_backend_api esb_log_backend_api = {
    .process    = esb_log_backend_process,
    .dropped    = esb_log_backend_dropped,
    .panic      = esb_log_backend_panic,
    .format_set = esb_log_backend_format_set,
};

LOG_BACKEND_DEFINE(esb_shell_log_backend, esb_log_backend_api, true);

extern void __stdout_hook_install(int (*hook)(int));

static void stdout_flush_work_fn(struct k_work *w) {
    ARG_UNUSED(w);
    uint8_t tmp[sizeof(stdout_buf)];
    const k_spinlock_key_t key = k_spin_lock(&stdout_buf_lock);
    const size_t n = stdout_buf_len;
    if (n > 0) {
        memcpy(tmp, stdout_buf, n);
        stdout_buf_len = 0;
    }
    k_spin_unlock(&stdout_buf_lock, key);
    if (n == 0 || !m_shell_active) {
        return;
    }
    out_enqueue(tmp, n);
    k_work_submit(&out_drain_work);
}

static void stdout_buf_reset(void) {
    k_work_cancel_delayable(&stdout_flush_work);
    const k_spinlock_key_t key = k_spin_lock(&stdout_buf_lock);
    stdout_buf_len = 0;
    k_spin_unlock(&stdout_buf_lock, key);
}

static int esb_stdout_hook(const int c) {
    if (!m_shell_active) {
        return c;
    }
    bool full;
    const k_spinlock_key_t key = k_spin_lock(&stdout_buf_lock);
    if (stdout_buf_len < sizeof(stdout_buf)) {
        stdout_buf[stdout_buf_len++] = (uint8_t)c;
    }
    full = (stdout_buf_len >= sizeof(stdout_buf));
    k_spin_unlock(&stdout_buf_lock, key);
    k_work_reschedule(&stdout_flush_work,
                      full ? K_NO_WAIT : K_MSEC(CONFIG_ZMK_ESB_ENDPOINT_SHELL_STDOUT_COALESCE_MS));
    return c;
}

void esb_shell_relay_init(void) {
    __stdout_hook_install(esb_stdout_hook);
    k_work_queue_init(&shell_exec_q);
    k_work_queue_start(&shell_exec_q, shell_exec_stack,
                       K_THREAD_STACK_SIZEOF(shell_exec_stack),
                       K_PRIO_PREEMPT(CONFIG_ZMK_ESB_ENDPOINT_SHELL_EXEC_PRIORITY),
                       NULL);
    k_thread_name_set(&shell_exec_q.thread, "esb_shell_exec");
}

void esb_shell_relay_on_connected(void) {
    LOG_DBG("connected, resetting shell state");
    m_shell_active = false;
    k_work_cancel_delayable(&shell_poll_work);
    k_work_cancel_delayable(&inactivity_work);
    stdout_buf_reset();
    ring_buf_reset(&out_rb);
    k_mutex_lock(&cmd_mutex, K_FOREVER);
    cmd_len = 0;
    k_mutex_unlock(&cmd_mutex);
#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_SHELL_BG_POLL)
    k_work_reschedule(&bg_poll_work, K_MSEC(CONFIG_ZMK_ESB_ENDPOINT_SHELL_BG_POLL_MS));
#endif
}

void esb_shell_relay_on_disconnected(void) {
    LOG_DBG("disconnected");
#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_SHELL_BG_POLL)
    k_work_cancel_delayable(&bg_poll_work);
#endif
    if (!m_shell_active) {
        return;
    }
    m_shell_active = false;
    k_work_cancel_delayable(&shell_poll_work);
    k_work_cancel_delayable(&inactivity_work);
    stdout_buf_reset();
    ring_buf_reset(&out_rb);
}

void esb_shell_relay_on_req(void) {
    if (m_shell_active) {
        LOG_DBG("SHELL_REQ but already active");
        return;
    }
    LOG_DBG("SHELL_REQ received, entering active shell mode");
    m_shell_active = true;
    cmd_len = 0;
    stdout_buf_reset();
    ring_buf_reset(&out_rb);
#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_SHELL_BG_POLL)
    k_work_cancel_delayable(&bg_poll_work);
#endif
    k_work_reschedule(&shell_poll_work, K_MSEC(CONFIG_ZMK_ESB_ENDPOINT_SHELL_INITIAL_POLL_DELAY_MS));
    if (CONFIG_ZMK_ESB_ENDPOINT_SHELL_INACTIVITY_S > 0) {
        k_work_reschedule(&inactivity_work, K_SECONDS(CONFIG_ZMK_ESB_ENDPOINT_SHELL_INACTIVITY_S));
    }
    send_prompt();
}

void esb_shell_relay_on_data(const uint8_t *data, const uint8_t len) {
    if (!m_shell_active || len == 0) {
        LOG_DBG("SHELL_DATA ignored (active=%d len=%u)", m_shell_active, len);
        if (!m_shell_active && len > 0) {
            /* Dongle thinks the session is live (typical case: keyboard rebooted
             * mid-session, dongle's m_active/m_keyboard_confirmed survived). Nudge
             * it with SHELL_STOP so its always-on handler resets and re-issues
             * SHELL_REQ; without this the dongle keeps queuing CDC bytes as
             * SHELL_DATA ACK payloads until the FIFO fills (-ENOMEM) and input
             * is dropped. Cooldown debounces a burst of in-flight payloads. */
            static int64_t last_nudge_ms;
            const int64_t now = k_uptime_get();
            if (now - last_nudge_ms > CONFIG_ZMK_ESB_ENDPOINT_SHELL_STOP_NUDGE_COOLDOWN_MS) {
                last_nudge_ms = now;
                struct esb_pkt_shell_stop pkt = { .type = ESB_PKT_SHELL_STOP };
                esb_transport_send(ESB_PIPE_DATA, (uint8_t *)&pkt, sizeof(pkt));
            }
        }
        return;
    }
    LOG_DBG("RX %u cmd bytes from dongle", len);
    if (CONFIG_ZMK_ESB_ENDPOINT_SHELL_INACTIVITY_S > 0) {
        k_work_reschedule(&inactivity_work, K_SECONDS(CONFIG_ZMK_ESB_ENDPOINT_SHELL_INACTIVITY_S));
    }
    bool need_drain = false;
    k_mutex_lock(&cmd_mutex, K_FOREVER);
    for (uint8_t i = 0; i < len; i++) {
        const uint8_t b = data[i];
        if (b == '\n' || b == '\r') {
            if (cmd_len > 0) {
                cmd_buf[cmd_len] = '\0';
                cmd_len = 0;
                k_mutex_unlock(&cmd_mutex);
                LOG_DBG("command ready, submitting exec work");
                k_work_submit_to_queue(&shell_exec_q, &shell_exec_work);
                k_mutex_lock(&cmd_mutex, K_FOREVER);
            }
        } else if (b == 0x7F || b == 0x08) {
            if (cmd_len > 0) {
                cmd_len--;
                static const uint8_t erase[] = "\b \b";
                out_enqueue(erase, sizeof(erase) - 1);
                need_drain = true;
            }
        } else if (cmd_len < sizeof(cmd_buf) - 1) {
            cmd_buf[cmd_len++] = (char)b;
        } else {
            LOG_WRN("cmd_buf overflow, byte 0x%02x dropped", b);
        }
    }
    k_mutex_unlock(&cmd_mutex);
    if (need_drain) {
        k_work_submit(&out_drain_work);
    }
}

bool esb_shell_relay_is_active(void) {
    return m_shell_active;
}

void esb_shell_relay_notify_tx(void) {
    if (m_shell_active && ring_buf_size_get(&out_rb) > 0) {
        LOG_DBG("notify_tx, %u bytes pending", ring_buf_size_get(&out_rb));
        k_work_submit(&out_drain_work);
    }
}

int esb_shell_relay_request(void) {
    if (!zmk_esb_endpoint_is_active()) {
        LOG_DBG("SHELL_START ignored, ESB endpoint not active");
        return -ENOTCONN;
    }
    if (m_shell_active) {
        LOG_DBG("SHELL_START ignored, already in shell mode");
        return 0;
    }
    const struct esb_pkt_shell_start pkt = { .type = ESB_PKT_SHELL_START };
    const int err = esb_transport_send(ESB_PIPE_DATA, (const uint8_t *)&pkt, sizeof(pkt));
    if (err) {
        LOG_WRN("SHELL_START TX err %d", err);
    } else {
        LOG_DBG("SHELL_START sent to dongle");
    }
    return err;
}
