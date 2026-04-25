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
        } else if (cmd == ESB_CMD_DEACTIVATE) {
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

#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_HOP)
static int cmd_esb_channel(const struct shell *sh, const size_t argc, char **argv) {
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);
    const uint8_t cur = esb_transport_get_channel();
    const uint8_t nxt = channel_hop_ep_get_committed();
    const uint8_t qn  = channel_hop_ep_get_quarantine_count();

    shell_print(sh, "current:   %u", cur);
    if (nxt == CHANNEL_HOP_INVALID) {
        shell_print(sh, "committed: <none>");
    } else {
        shell_print(sh, "committed: %u", nxt);
    }
    shell_print(sh, "state:     %s",
                channel_hop_ep_is_active() ? "active" : "idle");
    shell_print(sh, "quarantined channels: %u", qn);
    if (qn > 0) {
        shell_fprintf(sh, SHELL_NORMAL, "  [");
        bool first = true;
        for (uint8_t ch = 0; ch < CHANNEL_HOP_CHANNEL_COUNT; ch++) {
            if (channel_hop_ep_is_quarantined(ch)) {
                shell_fprintf(sh, SHELL_NORMAL, "%s%u", first ? "" : ", ", ch);
                first = false;
            }
        }
        shell_print(sh, "]");
    }
    return 0;
}
#endif

SHELL_STATIC_SUBCMD_SET_CREATE(esb_cmds,
    SHELL_CMD(unpair, NULL, "Forget paired dongle and start beaconing", cmd_esb_unpair),
#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_BENCH)
    SHELL_CMD(bench, NULL, "Run ESB link benchmark (run twice: start, then read results)", cmd_esb_bench),
#endif
#if IS_ENABLED(CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_HOP)
    SHELL_CMD(channel, NULL, "Show current RF channel, committed next hop, and quarantined channels", cmd_esb_channel),
#endif
    SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(esb, &esb_cmds, "ESB endpoint commands", NULL);
#endif /* CONFIG_ZMK_ESB_ENDPOINT_SHELL_CMDS */
