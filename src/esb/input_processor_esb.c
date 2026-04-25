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

/* Dropping stale motion prevents a post-quiet catch-up flush from
 * teleporting the cursor by tens of accumulated deltas. Tuned to sit
 * between one HID frame (~1 ms at 1 kHz) and the post-hop quiet window
 * (CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_HOP_POST_QUIET_MS, default 3 ms) plus
 * a few radio retries; ~20 ms catches genuine drop-outs without
 * discarding legitimate motion through normal queue pressure. */
#define ESB_IP_MOTION_MAX_STALE_MS 20u

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
};

static void send_mouse_report(struct esb_ip_data *d, bool button_changed) {
    /* Coalesce motion during post-hop quiet / sync-TX side-trip: the
     * transport would silently drop the send anyway (and we'd lose the
     * accumulated dx/dy). Button edges still need to land immediately
     * — a stuck click is worse than an oversized catch-up delta, and
     * the saturation in CLAMP below bounds the worst case. */
    if (!button_changed && esb_transport_is_quiet()) {
        return;
    }

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

    const struct zmk_hid_mouse_report_body body = {
        .buttons    = d->buttons,
        .d_x        = (int16_t)CLAMP(d->dx,       INT16_MIN, INT16_MAX),
        .d_y        = (int16_t)CLAMP(d->dy,       INT16_MIN, INT16_MAX),
        .d_scroll_y = (int16_t)CLAMP(d->scroll_y, INT16_MIN, INT16_MAX),
        .d_scroll_x = (int16_t)CLAMP(d->scroll_x, INT16_MIN, INT16_MAX),
    };

    struct esb_pkt_hid_report pkt = {
        .type        = ESB_PKT_HID_REPORT,
        .report_type = ESB_REPORT_MOUSE,
    };
    memcpy(pkt.data, &body, sizeof(body));
    esb_transport_send(ESB_PIPE_DATA, (uint8_t *)&pkt, sizeof(pkt));
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
    return 0;
}

#define ESB_IP_INST(n) \
    static struct esb_ip_data esb_ip_data_##n = {}; \
    DEVICE_DT_INST_DEFINE(n, esb_ip_init, NULL, &esb_ip_data_##n, NULL, \
                          POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &esb_ip_api);

DT_INST_FOREACH_STATUS_OKAY(ESB_IP_INST)
