/*
 * Copyright (c) 2025 efogdev
 * SPDX-License-Identifier: MIT
 *
 * Mirrors ZMK's hid_listener semantics against a module-local report state
 * rather than the global keyboard_report / consumer_report. Decouples the
 * ESB report from hid_listener: the order in which the two listeners run
 * (link order; .event_subscription is unsorted) no longer matters, and the
 * global implicit-mod cross-release bug can't leak into the ESB stream.
 *
 * Returns ZMK_EV_EVENT_HANDLED when ESB is active + paired so hid_listener
 * is skipped and USB/BLE stays quiet; bubbles otherwise so the normal path
 * runs unchanged.
 */

#include <zephyr/kernel.h>
#include <string.h>
#include <zmk/event_manager.h>
#include <zmk/events/keycode_state_changed.h>
#include <zmk/hid.h>
#include <zmk/keys.h>
#include <dt-bindings/zmk/hid_usage_pages.h>
#include <zmk_esb/endpoint.h>
#include <zmk_esb/protocol.h>
#include "esb_transport.h"
#include "pairing.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(zmk_esb_hid, CONFIG_ZMK_ESB_ENDPOINT_LOG_LEVEL);

static struct zmk_hid_keyboard_report_body kb_body;
static struct zmk_hid_consumer_report_body cons_body;

/* Per-bit refcount for each modifier flag. explicit and implicit tracked
 * separately so ZMK's known implicit-mod cross-release bug (see inline
 * comment at hid_listener.c:81-86) doesn't affect the ESB stream. */
static uint8_t explicit_mod_refcount[8];
static uint8_t implicit_mod_refcount[8];
static zmk_mod_flags_t explicit_mods;
static zmk_mod_flags_t implicit_mods;

/* Per-report snapshot rings used to defer reports past a transport-quiet
 * window (post-hop quiet, sync-TX side-trip). esb_transport_send()
 * returns -EAGAIN during those windows; without these rings,
 * intermediate press/release edges that fall in the window are lost —
 * kb_body / cons_body keep evolving, so by the time the window ends only
 * the latest state remains and the host never observes the missed edges
 * (stranded modifiers, missing keystrokes). On overrun the oldest entry
 * is shifted out so the queue still ends on the latest state; final
 * steady-state stays correct, only the truncated head of a long burst
 * is lost. Same trade-off the mouse path documents in
 * input_processor_esb.c:71-82.
 *
 * Locking: the rings and the report/modifier state are touched from two
 * contexts — the ZMK event thread (hid_relay_cb) and the system
 * workqueue (hid_retry_work, the transport's HID resync callback). All
 * mutations happen under k_sched_lock(), which on this single-core part
 * is sufficient mutual exclusion between threads (no ISR touches this
 * state). The lock is never held across esb_transport_send(): drains
 * snapshot-and-clear the ring under the lock, then send from the local
 * copy, re-parking any unsent tail if the transport goes quiet
 * mid-drain. */
static struct zmk_hid_keyboard_report_body
    kb_pending[CONFIG_ZMK_ESB_ENDPOINT_HID_QUIET_KB_QUEUE_DEPTH];
static uint8_t kb_pending_count;

static struct zmk_hid_consumer_report_body
    cons_pending[CONFIG_ZMK_ESB_ENDPOINT_HID_QUIET_CONS_QUEUE_DEPTH];
static uint8_t cons_pending_count;

static struct k_work_delayable hid_retry_work;

static void mods_inc(uint8_t *rc, zmk_mod_flags_t *out, const zmk_mod_flags_t mods) {
    for (int b = 0; b < 8; b++) {
        if (mods & BIT(b)) {
            if (rc[b]++ == 0) {
                *out |= BIT(b);
            }
        }
    }
}

static void mods_dec(uint8_t *rc, zmk_mod_flags_t *out, const zmk_mod_flags_t mods) {
    for (int b = 0; b < 8; b++) {
        if ((mods & BIT(b)) && rc[b] > 0) {
            if (--rc[b] == 0) {
                *out &= ~BIT(b);
            }
        }
    }
}

static void kb_keys_press(const uint32_t keycode) {
#if IS_ENABLED(CONFIG_ZMK_HID_REPORT_TYPE_NKRO)
    if (keycode <= ZMK_HID_KEYBOARD_NKRO_MAX_USAGE) {
        kb_body.keys[keycode / 8] |= BIT(keycode % 8);
    }
#elif IS_ENABLED(CONFIG_ZMK_HID_REPORT_TYPE_HKRO)
    for (int i = 0; i < CONFIG_ZMK_HID_KEYBOARD_REPORT_SIZE; i++) {
        if (kb_body.keys[i] == (uint8_t)keycode) {
            return;
        }
    }
    for (int i = 0; i < CONFIG_ZMK_HID_KEYBOARD_REPORT_SIZE; i++) {
        if (kb_body.keys[i] == 0) {
            kb_body.keys[i] = (uint8_t)keycode;
            return;
        }
    }
#endif
}

static void kb_keys_release(const uint32_t keycode) {
#if IS_ENABLED(CONFIG_ZMK_HID_REPORT_TYPE_NKRO)
    if (keycode <= ZMK_HID_KEYBOARD_NKRO_MAX_USAGE) {
        kb_body.keys[keycode / 8] &= ~BIT(keycode % 8);
    }
#elif IS_ENABLED(CONFIG_ZMK_HID_REPORT_TYPE_HKRO)
    for (int i = 0; i < CONFIG_ZMK_HID_KEYBOARD_REPORT_SIZE; i++) {
        if (kb_body.keys[i] == (uint8_t)keycode) {
            kb_body.keys[i] = 0;
        }
    }
#endif
}

static void cons_keys_press(const uint32_t keycode) {
    for (int i = 0; i < CONFIG_ZMK_HID_CONSUMER_REPORT_SIZE; i++) {
        if (cons_body.keys[i] == keycode) {
            return;
        }
    }
    for (int i = 0; i < CONFIG_ZMK_HID_CONSUMER_REPORT_SIZE; i++) {
        if (cons_body.keys[i] == 0) {
            cons_body.keys[i] = keycode;
            return;
        }
    }
}

static void cons_keys_release(const uint32_t keycode) {
    for (int i = 0; i < CONFIG_ZMK_HID_CONSUMER_REPORT_SIZE; i++) {
        if (cons_body.keys[i] == keycode) {
            cons_body.keys[i] = 0;
        }
    }
}

static int send_hid_pkt(uint8_t report_type, const void *body, size_t body_len) {
    struct esb_pkt_hid_report pkt = {
        .type        = ESB_PKT_HID_REPORT,
        .report_type = report_type,
    };
    if (body_len > sizeof(pkt.data)) {
        body_len = sizeof(pkt.data);
    }
    memcpy(pkt.data, body, body_len);
    return esb_transport_send(ESB_PIPE_DATA, (uint8_t *)&pkt, sizeof(pkt));
}

static int send_kb_body(const struct zmk_hid_keyboard_report_body *body) {
    const int err = send_hid_pkt(ESB_REPORT_KEYBOARD, body, sizeof(*body));
    if (err && err != -ENOMEM && err != -EAGAIN) {
        LOG_WRN("KB report send err %d", err);
    }
    return err;
}

static int send_cons_body(const struct zmk_hid_consumer_report_body *body) {
    const int err = send_hid_pkt(ESB_REPORT_CONSUMER, body, sizeof(*body));
    if (err && err != -ENOMEM && err != -EAGAIN) {
        LOG_WRN("consumer report send err %d", err);
    }
    return err;
}

/* Queue-append helpers. Caller must hold the scheduler lock. */
static void enqueue_pending_kb_locked(const struct zmk_hid_keyboard_report_body *body) {
    if (kb_pending_count >= CONFIG_ZMK_ESB_ENDPOINT_HID_QUIET_KB_QUEUE_DEPTH) {
        memmove(&kb_pending[0], &kb_pending[1],
                sizeof(kb_pending[0]) *
                    (CONFIG_ZMK_ESB_ENDPOINT_HID_QUIET_KB_QUEUE_DEPTH - 1u));
        kb_pending_count = CONFIG_ZMK_ESB_ENDPOINT_HID_QUIET_KB_QUEUE_DEPTH - 1u;
    }
    kb_pending[kb_pending_count++] = *body;
}

static void enqueue_pending_cons_locked(const struct zmk_hid_consumer_report_body *body) {
    if (cons_pending_count >= CONFIG_ZMK_ESB_ENDPOINT_HID_QUIET_CONS_QUEUE_DEPTH) {
        memmove(&cons_pending[0], &cons_pending[1],
                sizeof(cons_pending[0]) *
                    (CONFIG_ZMK_ESB_ENDPOINT_HID_QUIET_CONS_QUEUE_DEPTH - 1u));
        cons_pending_count = CONFIG_ZMK_ESB_ENDPOINT_HID_QUIET_CONS_QUEUE_DEPTH - 1u;
    }
    cons_pending[cons_pending_count++] = *body;
}

/* Drain the queue in arrival order. Snapshot-and-clear under the
 * scheduler lock, send from the local copy — so a concurrent enqueue
 * from the other context lands in the (now empty) live queue instead of
 * being wiped by the count reset, and the lock is never held across a
 * send. Returns 0 when fully drained; -EAGAIN when the transport went
 * quiet mid-drain, in which case the unsent tail is re-parked at the
 * queue head (ahead of anything enqueued meanwhile — the tail is
 * strictly older) and the caller reschedules the retry worker. On
 * re-park overflow the oldest tail entries are dropped so the queue
 * still ends on the latest state. */
static int drain_pending_kb(void) {
    struct zmk_hid_keyboard_report_body snap[CONFIG_ZMK_ESB_ENDPOINT_HID_QUIET_KB_QUEUE_DEPTH];
    k_sched_lock();
    const uint8_t n = kb_pending_count;
    memcpy(snap, kb_pending, (size_t)n * sizeof(snap[0]));
    kb_pending_count = 0;
    k_sched_unlock();
    for (uint8_t i = 0; i < n; i++) {
        if (send_kb_body(&snap[i]) != -EAGAIN) {
            continue;
        }
        k_sched_lock();
        uint8_t tail = n - i;
        const uint8_t live = kb_pending_count;
        if (tail + live > CONFIG_ZMK_ESB_ENDPOINT_HID_QUIET_KB_QUEUE_DEPTH) {
            const uint8_t drop =
                (uint8_t)(tail + live - CONFIG_ZMK_ESB_ENDPOINT_HID_QUIET_KB_QUEUE_DEPTH);
            i += drop;
            tail = (drop >= tail) ? 0u : (uint8_t)(tail - drop);
        }
        memmove(&kb_pending[tail], &kb_pending[0], (size_t)live * sizeof(kb_pending[0]));
        memcpy(&kb_pending[0], &snap[i], (size_t)tail * sizeof(kb_pending[0]));
        kb_pending_count = (uint8_t)(tail + live);
        k_sched_unlock();
        return -EAGAIN;
    }
    return 0;
}

static int drain_pending_cons(void) {
    struct zmk_hid_consumer_report_body snap[CONFIG_ZMK_ESB_ENDPOINT_HID_QUIET_CONS_QUEUE_DEPTH];
    k_sched_lock();
    const uint8_t n = cons_pending_count;
    memcpy(snap, cons_pending, (size_t)n * sizeof(snap[0]));
    cons_pending_count = 0;
    k_sched_unlock();
    for (uint8_t i = 0; i < n; i++) {
        if (send_cons_body(&snap[i]) != -EAGAIN) {
            continue;
        }
        k_sched_lock();
        uint8_t tail = n - i;
        const uint8_t live = cons_pending_count;
        if (tail + live > CONFIG_ZMK_ESB_ENDPOINT_HID_QUIET_CONS_QUEUE_DEPTH) {
            const uint8_t drop =
                (uint8_t)(tail + live - CONFIG_ZMK_ESB_ENDPOINT_HID_QUIET_CONS_QUEUE_DEPTH);
            i += drop;
            tail = (drop >= tail) ? 0u : (uint8_t)(tail - drop);
        }
        memmove(&cons_pending[tail], &cons_pending[0], (size_t)live * sizeof(cons_pending[0]));
        memcpy(&cons_pending[0], &snap[i], (size_t)tail * sizeof(cons_pending[0]));
        cons_pending_count = (uint8_t)(tail + live);
        k_sched_unlock();
        return -EAGAIN;
    }
    return 0;
}

static void hid_retry_work_fn(struct k_work *work) {
    ARG_UNUSED(work);
    if (esb_transport_is_quiet()) {
        k_work_reschedule(&hid_retry_work,
                          K_MSEC(CONFIG_ZMK_ESB_ENDPOINT_HID_QUIET_RETRY_MS));
        return;
    }
    /* Drain in arrival order within each queue. Cross-queue ordering
     * (a kb edge vs a cons edge that fell in the same window) is not
     * preserved — they are independent reports for the host, and
     * media-vs-typing interleaving in a 1-3 ms window is not a real
     * use case. Deliberately not short-circuited: a re-parked kb tail
     * must not starve the cons drain. */
    const bool kb_parked = drain_pending_kb() == -EAGAIN;
    const bool cons_parked = drain_pending_cons() == -EAGAIN;
    if (kb_parked || cons_parked) {
        k_work_reschedule(&hid_retry_work,
                          K_MSEC(CONFIG_ZMK_ESB_ENDPOINT_HID_QUIET_RETRY_MS));
    }
}

static void send_keyboard(void) {
    struct zmk_hid_keyboard_report_body snap;
    k_sched_lock();
    kb_body.modifiers = explicit_mods | implicit_mods;
    snap = kb_body;
    if (esb_transport_is_quiet()) {
        enqueue_pending_kb_locked(&snap);
        k_sched_unlock();
        k_work_reschedule(&hid_retry_work,
                          K_MSEC(CONFIG_ZMK_ESB_ENDPOINT_HID_QUIET_RETRY_MS));
        return;
    }
    k_sched_unlock();
    /* Quiet ended (or never started): replay queued snapshots in order
     * so the host sees every press/release edge, then send the live
     * state. No-op when the queue is empty. The is_quiet() check above
     * is advisory — the transport can arm a quiet window between it and
     * the send, in which case -EAGAIN comes back here: park the
     * snapshot (after any re-parked drain tail, which is strictly
     * older) and let the retry worker finish the job. */
    if (drain_pending_kb() == -EAGAIN || send_kb_body(&snap) == -EAGAIN) {
        k_sched_lock();
        enqueue_pending_kb_locked(&snap);
        k_sched_unlock();
        k_work_reschedule(&hid_retry_work,
                          K_MSEC(CONFIG_ZMK_ESB_ENDPOINT_HID_QUIET_RETRY_MS));
    }
}

static void send_consumer(void) {
    struct zmk_hid_consumer_report_body snap;
    k_sched_lock();
    snap = cons_body;
    if (esb_transport_is_quiet()) {
        enqueue_pending_cons_locked(&snap);
        k_sched_unlock();
        k_work_reschedule(&hid_retry_work,
                          K_MSEC(CONFIG_ZMK_ESB_ENDPOINT_HID_QUIET_RETRY_MS));
        return;
    }
    k_sched_unlock();
    if (drain_pending_cons() == -EAGAIN || send_cons_body(&snap) == -EAGAIN) {
        k_sched_lock();
        enqueue_pending_cons_locked(&snap);
        k_sched_unlock();
        k_work_reschedule(&hid_retry_work,
                          K_MSEC(CONFIG_ZMK_ESB_ENDPOINT_HID_QUIET_RETRY_MS));
    }
}

/* Transport delivery-failure resync (fix for retry-exhausted / flushed
 * reports): an ACKed keyboard or consumer report died undelivered, so
 * the host may hold a stale edge (stuck key, missed press). Report
 * state is absolute, so re-sending the current snapshot is idempotent
 * and converges the host. Runs on the system workqueue; send_keyboard /
 * send_consumer handle a concurrent quiet window by parking in the
 * pending rings, and their own TX failure re-arms the resync — bounded
 * at one in-flight resync per kind, terminating on the first success. */
static void hid_resync_cb(const uint8_t kinds, void *ctx) {
    ARG_UNUSED(ctx);
    if (!zmk_esb_endpoint_is_active() || !pairing_is_connected()) {
        return;
    }
    if (kinds & ESB_HID_RESYNC_KB) {
        send_keyboard();
    }
    if (kinds & ESB_HID_RESYNC_CONS) {
        send_consumer();
    }
}

static int hid_relay_cb(const zmk_event_t *eh) {
    const struct zmk_keycode_state_changed *ev = as_zmk_keycode_state_changed(eh);
    if (!ev) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    const bool active = zmk_esb_endpoint_is_active() && pairing_is_connected();

    /* Falling-edge cleanup: when ESB deactivates mid-quiet, leftover
     * snapshots would later drain into a dead transport and produce
     * harmless but noisy LOG_WRN spam. Catch the edge here, cancel the
     * pending work, and clear the queues. The next reactivation starts
     * fresh. */
    static bool was_active;
    if (was_active && !active) {
        k_work_cancel_delayable(&hid_retry_work);
        k_sched_lock();
        kb_pending_count = 0;
        cons_pending_count = 0;
        k_sched_unlock();
    }
    was_active = active;

    /* Track state unconditionally: keeps the ESB report internally coherent
     * across profile switches for events we did observe. Mutations run
     * under the scheduler lock so the workqueue-side resync/retry paths
     * can never snapshot a half-updated report (e.g. a key bit set but
     * its implicit modifier not yet applied — the host would briefly see
     * the keycode without its modifier). Sends happen after unlock. */
    bool want_kb = false;
    bool want_cons = false;
    k_sched_lock();
    switch (ev->usage_page) {
    case HID_USAGE_KEY:
        if (is_mod(ev->usage_page, ev->keycode)) {
            const zmk_mod_flags_t bit = BIT(ev->keycode - HID_USAGE_KEY_KEYBOARD_LEFTCONTROL);
            if (ev->state) {
                mods_inc(explicit_mod_refcount, &explicit_mods, bit);
            } else {
                mods_dec(explicit_mod_refcount, &explicit_mods, bit);
            }
        } else {
            if (ev->state) {
                kb_keys_press(ev->keycode);
                mods_inc(explicit_mod_refcount, &explicit_mods, ev->explicit_modifiers);
                mods_inc(implicit_mod_refcount, &implicit_mods, ev->implicit_modifiers);
            } else {
                kb_keys_release(ev->keycode);
                mods_dec(explicit_mod_refcount, &explicit_mods, ev->explicit_modifiers);
                mods_dec(implicit_mod_refcount, &implicit_mods, ev->implicit_modifiers);
            }
        }
        want_kb = true;
        break;
    case HID_USAGE_CONSUMER:
        if (ev->state) {
            cons_keys_press(ev->keycode);
            mods_inc(explicit_mod_refcount, &explicit_mods, ev->explicit_modifiers);
            mods_inc(implicit_mod_refcount, &implicit_mods, ev->implicit_modifiers);
        } else {
            cons_keys_release(ev->keycode);
            mods_dec(explicit_mod_refcount, &explicit_mods, ev->explicit_modifiers);
            mods_dec(implicit_mod_refcount, &implicit_mods, ev->implicit_modifiers);
        }
        /* Consumer keycodes may carry explicit/implicit mods — push a
         * keyboard report too so modifier state stays in sync. */
        want_kb = (ev->explicit_modifiers || ev->implicit_modifiers);
        want_cons = true;
        break;
    default:
        k_sched_unlock();
        return ZMK_EV_EVENT_BUBBLE;
    }
    k_sched_unlock();

    if (active) {
        if (want_kb) {
            send_keyboard();
        }
        if (want_cons) {
            send_consumer();
        }
    }

    return active ? ZMK_EV_EVENT_HANDLED : ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(esb_relay, hid_relay_cb);
ZMK_SUBSCRIPTION(esb_relay, zmk_keycode_state_changed);

static int hid_relay_init(void) {
    k_work_init_delayable(&hid_retry_work, hid_retry_work_fn);
    (void)esb_transport_register_hid_resync_cb(hid_resync_cb, NULL);
    return 0;
}

SYS_INIT(hid_relay_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
