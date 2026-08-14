/*
 * Copyright (c) 2025 efogdev
 * SPDX-License-Identifier: MIT
 *
 * Input processor that captures pointer events and forwards them over ESB.
 * Must be placed in the input-processors chain of each sensor listener in DTS.
 * Returns ZMK_INPUT_PROC_STOP after handling the event when the ESB endpoint
 * is active and paired; returns ZMK_INPUT_PROC_CONTINUE otherwise so other
 * transports (BLE/USB) can run.
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
#define ESB_IP_PENDING_BTN_QUEUE   ((uint8_t)CONFIG_ZMK_ESB_ENDPOINT_IP_PENDING_BTN_QUEUE)
#define ESB_IP_QUIET_RETRY_MS      ((uint32_t)CONFIG_ZMK_ESB_ENDPOINT_IP_QUIET_RETRY_MS)

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

static int send_hid_report_raw(const uint8_t buttons, const int32_t dx, const int32_t dy,
                               const int32_t scroll_x, const int32_t scroll_y) {
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
    return esb_transport_send(ESB_PIPE_DATA, (uint8_t *)&pkt, sizeof(pkt));
}

/* Caller must hold the scheduler lock: the queue is touched from the
 * input thread (event handler) and the system workqueue (retry worker,
 * transport resync callback), and k_sched_lock() is the module-wide
 * mutual exclusion between those two contexts (single core, no ISR
 * involvement — mirrors hid_relay.c's locking note). */
static void enqueue_pending_button_locked(struct esb_ip_data *d) {
    if (d->pending_count >= ESB_IP_PENDING_BTN_QUEUE) {
        /* Drop the oldest so the queue still ends on the latest state.
         * An overrun means more edges happened in the quiet window than
         * we budgeted for; preserving the final state matters more than
         * any one intermediate edge. */
        memmove(&d->pending_buttons[0], &d->pending_buttons[1], ESB_IP_PENDING_BTN_QUEUE - 1u);
        d->pending_count = ESB_IP_PENDING_BTN_QUEUE - 1u;
    }
    d->pending_buttons[d->pending_count++] = d->buttons;
}

/* Replays queued button-edge snapshots in order as buttons-only HID
 * reports. Snapshot-and-clear under the scheduler lock, send from the
 * local copy — a concurrent enqueue lands in the live queue instead of
 * being wiped, and the lock is never held across a send. Returns 0 when
 * fully drained; -EAGAIN when the transport went quiet mid-drain, with
 * the unsent tail re-parked at the queue head (ahead of anything
 * enqueued meanwhile — the tail is strictly older; on overflow the
 * oldest tail entries drop so the queue still ends on latest state).
 * The caller owns the trailing "current state" report (with motion)
 * once the queue has drained, and reschedules the retry worker on
 * -EAGAIN. */
static int drain_pending_buttons(struct esb_ip_data *d) {
    uint8_t snap[ESB_IP_PENDING_BTN_QUEUE];
    k_sched_lock();
    const uint8_t n = d->pending_count;
    memcpy(snap, d->pending_buttons, n);
    d->pending_count = 0;
    k_sched_unlock();
    for (uint8_t i = 0; i < n; i++) {
        if (send_hid_report_raw(snap[i], 0, 0, 0, 0) != -EAGAIN) {
            continue;
        }
        k_sched_lock();
        uint8_t tail = n - i;
        const uint8_t live = d->pending_count;
        if (tail + live > ESB_IP_PENDING_BTN_QUEUE) {
            const uint8_t drop = (uint8_t)(tail + live - ESB_IP_PENDING_BTN_QUEUE);
            i += drop;
            tail = (drop >= tail) ? 0u : (uint8_t)(tail - drop);
        }
        memmove(&d->pending_buttons[tail], &d->pending_buttons[0], live);
        memcpy(&d->pending_buttons[0], &snap[i], tail);
        d->pending_count = (uint8_t)(tail + live);
        k_sched_unlock();
        return -EAGAIN;
    }
    return 0;
}

static void retry_work_fn(struct k_work *work) {
    struct k_work_delayable *dw = k_work_delayable_from_work(work);
    struct esb_ip_data *d = CONTAINER_OF(dw, struct esb_ip_data, retry_work);

    if (esb_transport_is_quiet() || esb_transport_pointer_backpressure()) {
        k_work_reschedule(&d->retry_work, K_MSEC(ESB_IP_QUIET_RETRY_MS));
        return;
    }
    if (drain_pending_buttons(d) == -EAGAIN) {
        k_work_reschedule(&d->retry_work, K_MSEC(ESB_IP_QUIET_RETRY_MS));
        return;
    }

    /* Flush any motion that accumulated during the deferral window as a
     * final report before clearing the accumulator. Two cases:
     *  - Brief pointer back-pressure (inflight cap): motion is fresh and
     *    should land on the host so the user's deltas aren't dropped.
     *  - Long quiet window (post-hop quiet, sync-TX side-trip): motion
     *    is stale by the time the host would see it.
     * The MOTION_MAX_STALE_MS check distinguishes them — fresh motion
     * flushes; older motion is dropped to avoid teleporting the cursor.
     * Buttons drained above already inform the host of any edge.
     * Snapshot-and-clear under the lock so a delta arriving mid-flush
     * is not lost to the clear; on -EAGAIN the deltas (and the original
     * staleness anchor) are merged back for the next attempt. */
    k_sched_lock();
    const uint32_t start = d->accum_start_ms;
    const uint8_t  btns  = d->buttons;
    const int32_t  dx = d->dx, dy = d->dy, sx = d->scroll_x, sy = d->scroll_y;
    d->dx = d->dy = d->scroll_x = d->scroll_y = 0;
    d->accum_start_ms = 0;
    k_sched_unlock();
    if (start != 0) {
        const uint32_t age = k_uptime_get_32() - start;
        if (age <= ESB_IP_MOTION_MAX_STALE_MS && (dx | dy | sx | sy)) {
            if (send_hid_report_raw(btns, dx, dy, sx, sy) == -EAGAIN) {
                k_sched_lock();
                d->dx += dx;
                d->dy += dy;
                d->scroll_x += sx;
                d->scroll_y += sy;
                if (d->accum_start_ms == 0) {
                    d->accum_start_ms = start;
                }
                k_sched_unlock();
                k_work_reschedule(&d->retry_work, K_MSEC(ESB_IP_QUIET_RETRY_MS));
            }
        }
    }
}

static void send_mouse_report(struct esb_ip_data *d, const bool button_changed) {
    /* Defer the send when either:
     *  - the transport is quiet (post-hop quiet, sync-TX side-trip): it
     *    will silently drop whatever we hand it, or
     *  - too many pointer-motion reports are already in-flight in ESB's
     *    TX FIFO: queueing more would only buffer stale frames that
     *    later squirt out as a cursor teleport when the link recovers.
     * In both cases motion coalesces in the per-instance accumulator
     * and button edges queue so the host eventually sees each
     * press/release in order — sending now would lose them. The retry
     * worker drains the queue once the deferral condition clears. */
    if (esb_transport_is_quiet() || esb_transport_pointer_backpressure()) {
        if (button_changed) {
            k_sched_lock();
            enqueue_pending_button_locked(d);
            k_sched_unlock();
            k_work_reschedule(&d->retry_work, K_MSEC(ESB_IP_QUIET_RETRY_MS));
        }
        return;
    }

    /* Quiet ended while button edges were queued: replay each as a
     * buttons-only report so the host sees every press/release in
     * order, then fall through to send the current state (which may
     * differ from the last queue entry if a button edge happened after
     * quiet ended but before this call). No-op when the queue is empty.
     * -EAGAIN means the transport went quiet again between the advisory
     * check above and the drain — park this edge too and let the retry
     * worker replay; motion stays in the accumulator untouched. */
    if (drain_pending_buttons(d) == -EAGAIN) {
        if (button_changed) {
            k_sched_lock();
            enqueue_pending_button_locked(d);
            k_sched_unlock();
        }
        k_work_reschedule(&d->retry_work, K_MSEC(ESB_IP_QUIET_RETRY_MS));
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

    /* Snapshot-and-clear under the lock (a delta arriving mid-send must
     * not be lost to the clear), send from the copy. On -EAGAIN — the
     * quiet window armed after the advisory check — merge the deltas
     * back (with the original staleness anchor), park the button edge
     * if there was one, and let the retry worker finish. */
    k_sched_lock();
    const uint32_t start = d->accum_start_ms;
    const uint8_t  btns  = d->buttons;
    const int32_t  dx = d->dx, dy = d->dy, sx = d->scroll_x, sy = d->scroll_y;
    d->dx = d->dy = d->scroll_x = d->scroll_y = 0;
    d->accum_start_ms = 0;
    k_sched_unlock();
    if (send_hid_report_raw(btns, dx, dy, sx, sy) == -EAGAIN) {
        k_sched_lock();
        d->dx += dx;
        d->dy += dy;
        d->scroll_x += sx;
        d->scroll_y += sy;
        if (d->accum_start_ms == 0 && start != 0) {
            d->accum_start_ms = start;
        }
        if (button_changed) {
            enqueue_pending_button_locked(d);
        }
        k_sched_unlock();
        k_work_reschedule(&d->retry_work, K_MSEC(ESB_IP_QUIET_RETRY_MS));
    }
}

/* Transport delivery-failure resync: an ACKed mouse report died
 * undelivered (retry exhaustion or FIFO flush), so the host may hold a
 * stale buttons view — a lost press while still held, or a lost release
 * leaving a phantom drag. Motion recovery is owned by the transport's
 * refund pool; this re-sends only the current absolute buttons state,
 * which is idempotent. Deferral conditions park the state in the
 * pending queue for the retry worker, same as a live edge would. */
static void esb_ip_resync_cb(const uint8_t kinds, void *ctx) {
    struct esb_ip_data *d = ctx;
    if (!(kinds & ESB_HID_RESYNC_MOUSE)) {
        return;
    }
    if (!zmk_esb_endpoint_is_active() || !pairing_is_connected()) {
        return;
    }
    if (esb_transport_is_quiet() || esb_transport_pointer_backpressure() ||
        drain_pending_buttons(d) == -EAGAIN ||
        send_hid_report_raw(d->buttons, 0, 0, 0, 0) == -EAGAIN) {
        k_sched_lock();
        enqueue_pending_button_locked(d);
        k_sched_unlock();
        k_work_reschedule(&d->retry_work, K_MSEC(ESB_IP_QUIET_RETRY_MS));
    }
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
    (void)esb_transport_register_hid_resync_cb(esb_ip_resync_cb, d);
    return 0;
}

#define ESB_IP_INST(n) \
    static struct esb_ip_data esb_ip_data_##n = {}; \
    DEVICE_DT_INST_DEFINE(n, esb_ip_init, NULL, &esb_ip_data_##n, NULL, \
                          POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &esb_ip_api);

DT_INST_FOREACH_STATUS_OKAY(ESB_IP_INST)
