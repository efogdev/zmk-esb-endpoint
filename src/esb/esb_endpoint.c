/*
 * Copyright (c) 2025 efogdev
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_esb_endpoint

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/settings/settings.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <esb.h>

#include <zmk/event_manager.h>
#include <zmk/events/ble_active_profile_changed.h>

#include "esb_transport.h"
#include "pairing.h"
#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_SHELL_RELAY)
#include "../shell/shell_relay.h"
#endif
#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_BENCH)
#include "bench.h"
#endif
#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_HOP)
#include "channel_hop_ep.h"
#include <zmk_esb/channel_hop.h>
#endif

#include <zephyr/logging/log.h>
#if IS_ENABLED(CONFIG_SHELL)
#include <zephyr/shell/shell.h>
#endif

#include "zmk/ble.h"
#include "zmk/hog.h"
LOG_MODULE_REGISTER(zmk_esb_endpoint, CONFIG_ZMK_ESB_ENDPOINT_LOG_LEVEL);

#if IS_ENABLED(CONFIG_ZMK_ADAPTIVE_FEEDBACK)
#include <zmk_adaptive_feedback/adaptive_feedback.h>
ZAF_CUSTOM_EVENT_DEFINE(esb_switched_to_dongle, "esb-switched");
#endif

static volatile bool esb_active;

K_THREAD_STACK_DEFINE(esb_ctrl_stack, CONFIG_ZMK_ESB_ENDPOINT_CTRL_STACK_SIZE);
static struct k_thread esb_ctrl_thread;

#define ESB_CMD_ACTIVATE   1
#define ESB_CMD_DEACTIVATE 2
K_MSGQ_DEFINE(esb_ctrl_msgq, sizeof(int), CONFIG_ZMK_ESB_ENDPOINT_CTRL_MSGQ_DEPTH, 4);

bool zmk_esb_endpoint_is_active(void) {
    return esb_active;
}

static void configure_esb_addresses(void) {
    static const uint8_t base_addr_0[4] = DT_INST_PROP(0, pairing_base_address);
    static const uint8_t base_addr_1[4] = DT_INST_PROP(0, data_base_address);
    static const uint8_t prefixes[8]    = {
        DT_INST_PROP(0, pairing_prefix),
        DT_INST_PROP(0, data_prefix),
        0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8,
    };

    esb_transport_set_addresses(base_addr_0, base_addr_1, prefixes, DT_INST_PROP(0, esb_channel));
}

static void on_transport_evt(const esb_transport_evt_t *evt) {
    if (evt->type == ESB_RX_EVT) {
        pairing_on_rx(evt->rx_buf, evt->rx_len);
#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_BENCH)
        esb_bench_on_rx(evt->rx_buf, evt->rx_len);
#endif
    }
}

static void disconnect_conn_cb(struct bt_conn *conn, void *data)
{
    bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
}

/* Profile has no bonded peer, so ZMK's update_advertising() permanently wants
 * ZMK_ADV_CONN. Every disconnect/endpoint/profile-change event re-submits
 * update_advertising_work, which calls bt_le_adv_start(); the BT Controller
 * then schedules adv events that clobber ESB's RADIO config. Intercept the
 * start via --wrap=bt_le_adv_start (see CMakeLists.txt) and no-op while ESB
 * owns the radio. This removes the race at the source — ZMK still thinks it
 * started adv and tracks its advertising_status accordingly, so the normal
 * BLE flow resumes once esb_active clears. */
int __real_bt_le_adv_start(const struct bt_le_adv_param *param,
                           const struct bt_data *ad, size_t ad_len,
                           const struct bt_data *sd, size_t sd_len);

int __wrap_bt_le_adv_start(const struct bt_le_adv_param *param,
                           const struct bt_data *ad, const size_t ad_len,
                           const struct bt_data *sd, const size_t sd_len) {
    if (esb_active) {
        return 0;
    }
    return __real_bt_le_adv_start(param, ad, ad_len, sd, sd_len);
}

int __real_bt_le_adv_update_data(const struct bt_data *ad, size_t ad_len,
                                 const struct bt_data *sd, size_t sd_len);

int __wrap_bt_le_adv_update_data(const struct bt_data *ad, const size_t ad_len,
                                 const struct bt_data *sd, const size_t sd_len) {
    if (esb_active) {
        return 0;
    }
    return __real_bt_le_adv_update_data(ad, ad_len, sd, sd_len);
}

/* bt_le_adv_start_legacy is a private host function (no public header); reached
 * via bt_le_adv_resume() on disconnect — wrap it too so resumes stay quiet. */
struct bt_le_ext_adv;
int __real_bt_le_adv_start_legacy(struct bt_le_ext_adv *adv,
                                  const struct bt_le_adv_param *param,
                                  const struct bt_data *ad, size_t ad_len,
                                  const struct bt_data *sd, size_t sd_len);

int __wrap_bt_le_adv_start_legacy(struct bt_le_ext_adv *adv,
                                  const struct bt_le_adv_param *param,
                                  const struct bt_data *ad, const size_t ad_len,
                                  const struct bt_data *sd, const size_t sd_len) {
    if (esb_active) {
        return 0;
    }
    return __real_bt_le_adv_start_legacy(adv, param, ad, ad_len, sd, sd_len);
}

/* While ESB owns the radio the active BLE profile has no peer (BT_ADDR_LE_ANY),
 * yet endpoints.c keeps routing HID reports to HoG: with USB unplugged and BLE
 * "not ready", get_selected_transport() falls through to DEFAULT_TRANSPORT,
 * which is BLE whenever CONFIG_ZMK_BLE=y. Each queued report then runs
 * zmk_ble_active_profile_conn(), which logs a LOG_WRN per call. Drop the
 * reports at the entrypoint so nothing enters hog_work_q and the BLE LL is
 * never touched. */
int __real_zmk_hog_send_keyboard_report(struct zmk_hid_keyboard_report_body *body);
int __wrap_zmk_hog_send_keyboard_report(struct zmk_hid_keyboard_report_body *body) {
    if (esb_active) {
        return 0;
    }
    return __real_zmk_hog_send_keyboard_report(body);
}

int __real_zmk_hog_send_consumer_report(struct zmk_hid_consumer_report_body *body);
int __wrap_zmk_hog_send_consumer_report(struct zmk_hid_consumer_report_body *body) {
    if (esb_active) {
        return 0;
    }
    return __real_zmk_hog_send_consumer_report(body);
}

#if IS_ENABLED(CONFIG_ZMK_POINTING)
int __real_zmk_hog_send_mouse_report(struct zmk_hid_mouse_report_body *body);
int __wrap_zmk_hog_send_mouse_report(struct zmk_hid_mouse_report_body *body) {
    if (esb_active) {
        return 0;
    }
    return __real_zmk_hog_send_mouse_report(body);
}
#endif

static void esb_ctrl_thread_fn(void *p1, void *p2, void *p3) {
    int cmd;
    while (1) {
        k_msgq_get(&esb_ctrl_msgq, &cmd, K_FOREVER);
        if (cmd == ESB_CMD_ACTIVATE) {
            /* Quiesce the BLE LL instead of tearing down the whole stack:
             *   - disconnect active peers (LL stops conn events)
             *   - stop advertising (LL stops adv events)
             * With no scheduled radio work the LL will not touch RADIO, and the
             * VTOR swap in esb_transport_on_slot_start() redirects the IRQ to ESB.
             * This avoids the settings-flush storm triggered by bt_disable(). */
            bt_conn_foreach(BT_CONN_TYPE_LE, disconnect_conn_cb, NULL);
            bt_le_adv_stop();
            k_sleep(K_MSEC(CONFIG_ZMK_ESB_ENDPOINT_BLE_QUIESCE_MS));
            configure_esb_addresses();
            pairing_start();
            esb_transport_on_slot_start();
            esb_active = true;
#if IS_ENABLED(CONFIG_ZMK_ADAPTIVE_FEEDBACK)
            zaf_custom_event_trigger(&esb_switched_to_dongle);
#endif
#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_SHELL_RELAY)
            esb_shell_relay_on_activate();
#endif
        } else if (cmd == ESB_CMD_DEACTIVATE) {
#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_SHELL_RELAY)
            esb_shell_relay_on_deactivate();
#endif
            esb_active = false;
            esb_transport_on_slot_stop();
            pairing_stop();
            esb_transport_deinit();
            /* Resync ZMK's static advertising_status with reality. Our wrap
             * returned 0 on the last bt_le_adv_start, so ZMK believes adv is
             * running; a bare bt_le_adv_stop() here would not flip that state
             * and update_advertising() would then no-op (desired==current==CONN),
             * leaving the new BLE slot unreachable. zmk_ble_set_device_name()
             * with CONFIG_BT_DEVICE_NAME is idempotent on bt_set_name() and
             * runs the exact reset path we need: stops adv, clears
             * advertising_status to NONE, then calls update_advertising() which
             * starts adv for real (esb_active is now false, so the wrap is
             * transparent). zmk_ble_active_profile_name() would return an empty
             * string — ZMK never populates profiles[i].name — which bt_set_name()
             * would then apply, making the device advertise without a name. */
            zmk_ble_set_device_name((char *)CONFIG_BT_DEVICE_NAME);
        }
    }
}

static int profile_listener_cb(const zmk_event_t *eh) {
    const struct zmk_ble_active_profile_changed *ev = as_zmk_ble_active_profile_changed(eh);
    if (!ev) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    const bool should_be_active = (ev->index == (ZMK_BLE_PROFILE_COUNT - 1));
    if (should_be_active && !esb_active) {
        const int cmd = ESB_CMD_ACTIVATE;
        k_msgq_put(&esb_ctrl_msgq, &cmd, K_NO_WAIT);
    } else if (!should_be_active && esb_active) {
        const int cmd = ESB_CMD_DEACTIVATE;
        k_msgq_put(&esb_ctrl_msgq, &cmd, K_NO_WAIT);
    }

    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(zmk_esb_endpoint_listener, profile_listener_cb);
ZMK_SUBSCRIPTION(zmk_esb_endpoint_listener, zmk_ble_active_profile_changed);

/* ZMK does not raise zmk_ble_active_profile_changed for the profile loaded from
 * settings at boot — the event only fires on user switches and connect/disconnect.
 * Poll once after settings have loaded so booting with the ESB slot selected
 * actually activates ESB. */
static void boot_profile_check_fn(struct k_work *work) {
    ARG_UNUSED(work);
    if (zmk_ble_active_profile_index() == (ZMK_BLE_PROFILE_COUNT - 1) && !esb_active) {
        const int cmd = ESB_CMD_ACTIVATE;
        k_msgq_put(&esb_ctrl_msgq, &cmd, K_NO_WAIT);
    }
}
static K_WORK_DELAYABLE_DEFINE(boot_profile_check_work, boot_profile_check_fn);

static int esb_endpoint_init(void) {
    esb_transport_init(on_transport_evt);
    pairing_init();
#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_SHELL_RELAY)
    esb_shell_relay_init();
#endif
#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_HOP)
    channel_hop_ep_init();
#endif

    k_thread_create(&esb_ctrl_thread, esb_ctrl_stack, K_THREAD_STACK_SIZEOF(esb_ctrl_stack),
                    esb_ctrl_thread_fn, NULL, NULL, NULL, K_PRIO_COOP(CONFIG_ZMK_ESB_ENDPOINT_CTRL_THREAD_PRIORITY), 0, K_NO_WAIT);
    k_thread_name_set(&esb_ctrl_thread, "esb_ctrl");

    k_work_schedule(&boot_profile_check_work, K_MSEC(CONFIG_ZMK_ESB_ENDPOINT_BOOT_CHECK_DELAY_MS));
    return 0;
}

SYS_INIT(esb_endpoint_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_SHELL_CMDS)
static int cmd_esb_unpair(const struct shell *sh, const size_t argc, char **argv) {
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);
    pairing_unpair();
    return 0;
}

#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_BENCH)
static int cmd_esb_bench(const struct shell *sh, const size_t argc, char **argv) {
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);
    switch (esb_bench_get_state()) {
    case BENCH_IDLE:
        if (!zmk_esb_endpoint_is_active()) {
            shell_error(sh, "ESB not active");
            return -EINVAL;
        }
        esb_bench_start();
        shell_print(sh, "ESB benchmark started (10 s)");
        break;
    case BENCH_RUNNING:
        break;
    case BENCH_STOPPING:
        shell_print(sh, "Benchmark stopping, waiting for dongle result...");
        break;
    case BENCH_DONE: {
        struct esb_bench_result r;
        esb_bench_get_result(&r);
        shell_print(sh, "Throughput: %u pkt/s  (ok=%u fail=%u)",
                    r.throughput_pkt_s, r.tx_ok, r.tx_fail);
        if (r.rssi_samples > 0) {
            shell_print(sh, "RSSI: avg=%d dBm  min=%d  max=%d  n=%u",
                        (int)r.rssi_avg, (int)r.rssi_min, (int)r.rssi_max,
                        r.rssi_samples);
        } else {
            shell_print(sh, "RSSI: n/a (no dongle response)");
        }
        esb_bench_reset();
        break;
    }
    }
    return 0;
}
#endif /* CONFIG_ZMK_ESB_ENDPOINT_BENCH */

static const char *pairing_state_str(const pairing_state_t s) {
    switch (s) {
    case PAIRING_STATE_IDLE:      return "idle";
    case PAIRING_STATE_UNPAIRED:  return "unpaired";
    case PAIRING_STATE_PAIRING:   return "pairing";
    case PAIRING_STATE_VERIFYING: return "verifying";
    case PAIRING_STATE_CONNECTED: return "connected";
    }
    return "?";
}

static int cmd_esb_status(const struct shell *sh, const size_t argc, char **argv) {
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    shell_print(sh, "pairing:  %s", pairing_state_str(pairing_get_state()));
    shell_print(sh, "active:   %s", zmk_esb_endpoint_is_active() ? "yes" : "no");

#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_HOP)
    const uint8_t cur = esb_transport_get_channel();
    const uint8_t nxt = channel_hop_ep_get_committed();
    shell_print(sh, "channel:  %u (hop %s)",
                cur, channel_hop_ep_is_active() ? "active" : "idle");
    if (nxt == CHANNEL_HOP_INVALID) {
        shell_print(sh, "next:     <none>");
    } else {
        shell_print(sh, "next:     %u", nxt);
    }
#else
    shell_print(sh, "channel:  %u", esb_transport_get_channel());
#endif

    int8_t r_last, r_ewma;
    if (esb_transport_get_peer_rssi(&r_last, &r_ewma)) {
        shell_print(sh, "rssi:     -%d dBm, ewma: -%d dBm", (int)r_last, (int)r_ewma);
    } else {
        shell_print(sh, "rssi:     n/a");
    }

#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_HOP)
    {
        uint8_t  lq_retried = 0;
        uint16_t lq_ewma_x10 = 0;
        esb_transport_get_link_quality(&lq_retried, &lq_ewma_x10);
        shell_print(sh, "link:     ewma=%u.%u attempts, retried=%u/%u in window",
                    (unsigned)(lq_ewma_x10 / 10u),
                    (unsigned)(lq_ewma_x10 % 10u),
                    (unsigned)lq_retried,
                    (unsigned)CONFIG_ZMK_ESB_ENDPOINT_LINK_QUALITY_WINDOW);
    }
#endif

    {
        uint32_t tx_ok = 0, tx_retried = 0, tx_exhausted = 0;
        esb_transport_get_per_stats(&tx_ok, &tx_retried, &tx_exhausted);
        const uint32_t tx_total = tx_ok + tx_exhausted;
        if (tx_total > 0) {
            /* per-mille so we can render one decimal without floats */
            const uint32_t per_pm   = (tx_exhausted * 1000u) / tx_total;
            const uint32_t retry_pm = (tx_ok > 0)
                ? (tx_retried * 1000u) / tx_ok : 0u;
            shell_print(sh,
                "TX:       ok=%u retried=%u (%u.%u%%) failed=%u (PER %u.%u%%)",
                (unsigned)tx_ok,
                (unsigned)tx_retried,
                (unsigned)(retry_pm / 10u), (unsigned)(retry_pm % 10u),
                (unsigned)tx_exhausted,
                (unsigned)(per_pm / 10u), (unsigned)(per_pm % 10u));
        } else {
            shell_print(sh, "TX:       ok=0 retried=0 failed=0");
        }
    }

    uint32_t hid_total = 0;
    uint16_t hid_rate  = 0;
    esb_transport_get_hid_tx_stats(&hid_total, &hid_rate);
    shell_print(sh, "HID:      %u Hz (total %u)",
                (unsigned)hid_rate, (unsigned)hid_total);

#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_HOP)
    {
        shell_print(sh, "");

        struct channel_hop_ep_offender off[16];
        const uint8_t n = channel_hop_ep_get_offenders(off, ARRAY_SIZE(off));
        char line[ARRAY_SIZE(off) * 12 + 1];
        size_t pos;
        uint8_t shown;

        pos = 0;
        shown = 0;
        for (uint8_t i = 0; i < n; i++) {
            if (!off[i].persistent) {
                continue;
            }
            pos += snprintk(line + pos, sizeof(line) - pos, "%s%u(%u)",
                            shown == 0 ? "" : " ",
                            (unsigned)off[i].channel, (unsigned)off[i].hits);
            shown++;
        }
        shell_print(sh, "persistently quarantined:  %s", shown > 0 ? line : "<none>");

        pos = 0;
        shown = 0;
        for (uint8_t i = 0; i < n; i++) {
            if (off[i].session_hits == 0) {
                continue;
            }
            pos += snprintk(line + pos, sizeof(line) - pos, "%s%u(%u)",
                            shown == 0 ? "" : " ",
                            (unsigned)off[i].channel, (unsigned)off[i].session_hits);
            shown++;
        }
        shell_print(sh, "offenders since boot:   %s", shown > 0 ? line : "<none>");
    }
#endif
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(esb_cmds,
    SHELL_CMD(unpair, NULL, "Forget paired dongle and start beaconing", cmd_esb_unpair),
    SHELL_CMD(stats, NULL, "Show available statistics", cmd_esb_status),
#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_BENCH)
    SHELL_CMD(bench, NULL, "Run ESB link benchmark (run twice: start, then read results)", cmd_esb_bench),
#endif
    SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(esb, &esb_cmds, "ESB endpoint commands", NULL);
#endif /* CONFIG_ZMK_ESB_ENDPOINT_SHELL_CMDS */
