/*
 * Copyright (c) 2025 efogdev
 * SPDX-License-Identifier: MIT
 *
 * Input processor that captures pointer events and forwards them over ESB.
 * Must be placed in the input-processors chain of each sensor listener in DTS.
 * Returns ZMK_INPUT_PROC_CONTINUE so the normal BLE/USB path also runs.
 */

#define DT_DRV_COMPAT zmk_esb_input_processor

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/input/input.h>
#include <drivers/input_processor.h>
#include <zmk/hid.h>
#include <zmk_esb/endpoint.h>
#include <zmk_esb/protocol.h>
#include "esb_transport.h"
#include "pairing.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(zmk_esb_input_proc, CONFIG_ZMK_ESB_ENDPOINT_LOG_LEVEL);

#define ESB_IP_MOTION_MAX_STALE_MS ((uint32_t)CONFIG_ZMK_ESB_ENDPOINT_MOTION_MAX_STALE_MS)

/* Capacity of the per-instance button-edge queue used to defer reports
 * past a transport-quiet window. Sized for a press-and-release pair
 * with margin; further edges shift the oldest out so the final queued
 * state still matches d->buttons. */
#define ESB_IP_PENDING_BTN_QUEUE 4u

/* Retry cadence while waiting for esb_transport_is_quiet() to clear. */
#define ESB_IP_QUIET_RETRY_MS 1u

struct esb_ip_data {
    int32_t dx;
    int32_t dy;
    int32_t scroll_x;
    int32_t scroll_y;
    uint8_t buttons;
    /* Uptime (k_uptime_get_32) when the current accumulator started
     * collecting. Zero means "accumulator empty / just reset"; set on
     * first non-zero delta after a reset, cleared on every send. */
    uint32_t accum_start_ms;
    /* Snapshots of d->buttons captured when a button edge fell during a
     * transport-quiet window. Drained in order by retry_work once the
     * quiet window ends so the host sees each press/release edge. */
    uint8_t pending_buttons[ESB_IP_PENDING_BTN_QUEUE];
    uint8_t pending_count;
    struct k_work_delayable retry_work;
};

static void send_hid_report_raw(uint8_t buttons, int32_t dx, int32_t dy,
                                int32_t scroll_x, int32_t scroll_y) {
    const struct zmk_hid_mouse_report_body body = {
        .buttons    = buttons,
        .d_x        = (int16_t)CLAMP(dx,       INT16_MIN, INT16_MAX),
        .d_y        = (int16_t)CLAMP(dy,       INT16_MIN, INT16_MAX),
        .d_scroll_y = (int16_t)CLAMP(scroll_y, INT16_MIN, INT16_MAX),
        .d_scroll_x = (int16_t)CLAMP(scroll_x, INT16_MIN, INT16_MAX),
    };
    struct esb_pkt_hid_report pkt = {
        .type        = ESB_PKT_HID_REPORT,
        .report_type = ESB_REPORT_MOUSE,
    };
    memcpy(pkt.data, &body, sizeof(body));
    esb_transport_send(ESB_PIPE_DATA, (uint8_t *)&pkt, sizeof(pkt));
}

static void enqueue_pending_button(struct esb_ip_data *d) {
    if (d->pending_count >= ESB_IP_PENDING_BTN_QUEUE) {
        /* Drop the oldest so the queue still ends on the latest state.
         * An overrun means more edges happened in the quiet window than
         * we budgeted for; preserving the final state matters more than
         * any one intermediate edge. */
        memmove(&d->pending_buttons[0], &d->pending_buttons[1],
                ESB_IP_PENDING_BTN_QUEUE - 1u);
        d->pending_count = ESB_IP_PENDING_BTN_QUEUE - 1u;
    }
    d->pending_buttons[d->pending_count++] = d->buttons;
}

/* Replays queued button-edge snapshots in order as buttons-only HID
 * reports. Caller must verify the transport is no longer quiet, and is
 * responsible for the trailing "current state" report (with motion)
 * once the queue has drained. */
static void drain_pending_buttons(struct esb_ip_data *d) {
    for (uint8_t i = 0; i < d->pending_count; i++) {
        send_hid_report_raw(d->pending_buttons[i], 0, 0, 0, 0);
    }
    d->pending_count = 0;
}

static void retry_work_fn(struct k_work *work) {
    struct k_work_delayable *dw = k_work_delayable_from_work(work);
    struct esb_ip_data *d = CONTAINER_OF(dw, struct esb_ip_data, retry_work);

    if (esb_transport_is_quiet()) {
        k_work_reschedule(&d->retry_work, K_MSEC(ESB_IP_QUIET_RETRY_MS));
        return;
    }
    drain_pending_buttons(d);

    /* Motion that accumulated during the quiet window is stale by
     * definition: by the time the host sees it, it describes where the
     * cursor was during the stall. Clearing the accumulator here also
     * resets accum_start_ms so fresh post-quiet motion is not silently
     * dropped by the stale-motion check off the old timestamp. */
    d->dx = d->dy = d->scroll_x = d->scroll_y = 0;
    d->accum_start_ms = 0;
}

static void send_mouse_report(struct esb_ip_data *d, bool button_changed) {
    /* Quiet window (post-hop quiet, sync-TX side-trip): the transport
     * silently drops anything we hand it. Coalesce motion in the
     * accumulator and queue button edges so the host eventually sees
     * each press/release in order — sending now would lose them. */
    if (esb_transport_is_quiet()) {
        if (button_changed) {
            enqueue_pending_button(d);
            k_work_reschedule(&d->retry_work, K_MSEC(ESB_IP_QUIET_RETRY_MS));
        }
        return;
    }

    /* Quiet ended while button edges were queued: replay each as a
     * buttons-only report so the host sees every press/release in
     * order, then fall through to send the current state (which may
     * differ from the last queue entry if a button edge happened after
     * quiet ended but before this call). No-op when the queue is empty. */
    drain_pending_buttons(d);

    /* Drop motion that accumulated longer ago than MOTION_MAX_STALE_MS.
     * Happens when a long stall (post-hop quiet, sync-TX, radio issue)
     * ended and the accumulator is now holding deltas that describe
     * where the cursor WAS going, not where it should go now. Flushing
     * them produces a visible cursor jump; dropping them matches what
     * a live stream of fresh reports would do once motion resumes.
     * Button edges flush regardless — staleness affects where the click
     * lands on screen but the click itself must still happen. */
    if (!button_changed && d->accum_start_ms != 0) {
        const uint32_t age = k_uptime_get_32() - d->accum_start_ms;
        if (age > ESB_IP_MOTION_MAX_STALE_MS) {
            d->dx = d->dy = d->scroll_x = d->scroll_y = 0;
            d->accum_start_ms = 0;
            return;
        }
    }

    send_hid_report_raw(d->buttons, d->dx, d->dy, d->scroll_x, d->scroll_y);
    d->dx = d->dy = d->scroll_x = d->scroll_y = 0;
    d->accum_start_ms = 0;
}

static int esb_ip_handle_event(const struct device *dev, struct input_event *event, uint32_t param1, uint32_t param2,
                               struct zmk_input_processor_state *state) {
    if (!zmk_esb_endpoint_is_active() || !pairing_is_connected()) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    struct esb_ip_data *d = dev->data;
    bool button_changed = false;
    switch (event->type) {
    case INPUT_EV_REL:
        /* Stamp the accumulator's start uptime on first delta after a
         * flush so the staleness check in send_mouse_report has an
         * anchor. Zero is the "empty" sentinel — if k_uptime happens
         * to return 0 at this instant, bias to 1 so !=0 stays
         * truthful. */
        if (d->accum_start_ms == 0) {
            const uint32_t now = k_uptime_get_32();
            d->accum_start_ms = (now == 0) ? 1 : now;
        }
        switch (event->code) {
        case INPUT_REL_X:
            d->dx += event->value;
            break;
        case INPUT_REL_Y:
            d->dy += event->value;
            break;
        case INPUT_REL_WHEEL:
            d->scroll_y += event->value;
            break;
        case INPUT_REL_HWHEEL:
            d->scroll_x += event->value;
            break;
        default: break;
        }
        break;
    case INPUT_EV_KEY:
        if (event->code >= INPUT_BTN_0 && event->code <= INPUT_BTN_4) {
            const uint8_t btn = event->code - INPUT_BTN_0;
            const uint8_t before = d->buttons;
            if (event->value) {
                d->buttons |= BIT(btn);
            } else {
                d->buttons &= ~BIT(btn);
            }
            button_changed = (d->buttons != before);
        }
        break;
    default: break;
    }

    if (event->sync || button_changed) {
        send_mouse_report(d, button_changed);
    }

    return ZMK_INPUT_PROC_STOP;
}

static const struct zmk_input_processor_driver_api esb_ip_api = {
    .handle_event = esb_ip_handle_event,
};

static int esb_ip_init(const struct device *dev) {
    struct esb_ip_data *d = dev->data;
    memset(d, 0, sizeof(*d));
    k_work_init_delayable(&d->retry_work, retry_work_fn);
    return 0;
}

#define ESB_IP_INST(n) \
    static struct esb_ip_data esb_ip_data_##n = {}; \
    DEVICE_DT_INST_DEFINE(n, esb_ip_init, NULL, &esb_ip_data_##n, NULL, \
                          POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &esb_ip_api);

DT_INST_FOREACH_STATUS_OKAY(ESB_IP_INST)
