/*
 * Copyright (c) 2025 efogdev
 * SPDX-License-Identifier: MIT
 *
 * MPSL timeslot handler — ported and simplified from ncs-esb-ble-mpsl-demo.
 * Requests short (CONFIG_ZMK_ESB_ENDPOINT_TIMESLOT_US) back-to-back "earliest"
 * timeslots so ESB coexists with BLE without extension complexity.
 */

#include <zephyr/kernel.h>
#include <zephyr/irq.h>
#include <hal/nrf_timer.h>
#include <mpsl_timeslot.h>
#include <mpsl.h>
#include "timeslot_handler.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(zmk_esb_ts, CONFIG_ZMK_LOG_LEVEL);

#define TIMESLOT_LEN_US      CONFIG_ZMK_ESB_ENDPOINT_TIMESLOT_US
#define GUARD_US             CONFIG_ZMK_ESB_ENDPOINT_TIMESLOT_GUARD_US
#define TIMER_FIRE_US        (TIMESLOT_LEN_US - GUARD_US)
#define TIMEOUT_US           1000000UL

#define MPSL_THREAD_PRIO     CONFIG_MPSL_THREAD_COOP_PRIO
#define MPSL_STACK_SIZE      CONFIG_MAIN_STACK_SIZE

enum mpsl_api_call {
    API_OPEN_SESSION,
    API_MAKE_REQUEST,
    API_CLOSE_SESSION,
};

static timeslot_cb_t m_cb;
static volatile bool m_running;
static volatile bool m_in_slot;

K_MSGQ_DEFINE(ts_api_msgq, sizeof(enum mpsl_api_call), 8, 4);

static mpsl_timeslot_request_t ts_request_earliest = {
    .request_type = MPSL_TIMESLOT_REQ_TYPE_EARLIEST,
    .params.earliest = {
        .hfclk    = MPSL_TIMESLOT_HFCLK_CFG_NO_GUARANTEE,
        .priority = MPSL_TIMESLOT_PRIORITY_NORMAL,
        .length_us   = TIMESLOT_LEN_US,
        .timeout_us  = TIMEOUT_US,
    },
};

static mpsl_timeslot_signal_return_param_t ret_param;

void RADIO_IRQHandler(void);

static void schedule_api(enum mpsl_api_call call)
{
    int err = k_msgq_put(&ts_api_msgq, &call, K_NO_WAIT);
    if (err) {
        LOG_ERR("ts api msgq full");
    }
}

static void slot_set_active(bool active)
{
    if (active == m_in_slot) {
        return;
    }
    m_in_slot = active;
    if (m_cb) {
        m_cb(active ? TS_EVT_STARTED : TS_EVT_STOPPED);
    }
}

static mpsl_timeslot_signal_return_param_t *ts_signal_cb(
    mpsl_timeslot_session_id_t session_id, uint32_t signal)
{
    (void)session_id;
    mpsl_timeslot_signal_return_param_t *p = NULL;

    switch (signal) {
    case MPSL_TIMESLOT_SIGNAL_START:
        /* Reset RADIO from prior BLE use */
        NVIC_ClearPendingIRQ(RADIO_IRQn);
        NRF_RADIO->POWER = RADIO_POWER_POWER_Disabled << RADIO_POWER_POWER_Pos;
        NRF_RADIO->POWER = RADIO_POWER_POWER_Enabled << RADIO_POWER_POWER_Pos;
        NVIC_ClearPendingIRQ(RADIO_IRQn);

        nrf_timer_bit_width_set(NRF_TIMER0, NRF_TIMER_BIT_WIDTH_32);
        nrf_timer_cc_set(NRF_TIMER0, NRF_TIMER_CC_CHANNEL0, TIMER_FIRE_US);
        nrf_timer_int_enable(NRF_TIMER0, NRF_TIMER_INT_COMPARE0_MASK);

        slot_set_active(true);

        ret_param.callback_action = MPSL_TIMESLOT_SIGNAL_ACTION_NONE;
        p = &ret_param;
        break;

    case MPSL_TIMESLOT_SIGNAL_TIMER0:
        if (nrf_timer_event_check(NRF_TIMER0, NRF_TIMER_EVENT_COMPARE0)) {
            nrf_timer_int_disable(NRF_TIMER0, NRF_TIMER_INT_COMPARE0_MASK);
            nrf_timer_event_clear(NRF_TIMER0, NRF_TIMER_EVENT_COMPARE0);

            slot_set_active(false);

            if (m_running) {
                ret_param.callback_action = MPSL_TIMESLOT_SIGNAL_ACTION_REQUEST;
                ret_param.params.request.p_next = &ts_request_earliest;
            } else {
                ret_param.callback_action = MPSL_TIMESLOT_SIGNAL_ACTION_END;
            }
            p = &ret_param;
        }
        break;

    case MPSL_TIMESLOT_SIGNAL_RADIO:
        ret_param.callback_action = MPSL_TIMESLOT_SIGNAL_ACTION_NONE;
        p = &ret_param;
        if (m_in_slot) {
            RADIO_IRQHandler();
        } else {
            NVIC_ClearPendingIRQ(RADIO_IRQn);
            NVIC_DisableIRQ(RADIO_IRQn);
        }
        break;

    case MPSL_TIMESLOT_SIGNAL_EXTEND_FAILED:
    case MPSL_TIMESLOT_SIGNAL_OVERSTAYED:
        ret_param.callback_action = MPSL_TIMESLOT_SIGNAL_ACTION_END;
        p = &ret_param;
        slot_set_active(false);
        break;

    case MPSL_TIMESLOT_SIGNAL_CANCELLED:
    case MPSL_TIMESLOT_SIGNAL_BLOCKED:
        ret_param.callback_action = MPSL_TIMESLOT_SIGNAL_ACTION_NONE;
        p = &ret_param;
        slot_set_active(false);
        schedule_api(API_MAKE_REQUEST);
        break;

    case MPSL_TIMESLOT_SIGNAL_SESSION_IDLE:
        ret_param.callback_action = MPSL_TIMESLOT_SIGNAL_ACTION_NONE;
        p = &ret_param;
        slot_set_active(false);
        if (m_running) {
            schedule_api(API_MAKE_REQUEST);
        }
        break;

    case MPSL_TIMESLOT_SIGNAL_SESSION_CLOSED:
        ret_param.callback_action = MPSL_TIMESLOT_SIGNAL_ACTION_NONE;
        p = &ret_param;
        slot_set_active(false);
        break;

    case MPSL_TIMESLOT_SIGNAL_INVALID_RETURN:
        LOG_ERR("invalid return");
        ret_param.callback_action = MPSL_TIMESLOT_SIGNAL_ACTION_END;
        p = &ret_param;
        slot_set_active(false);
        break;

    default:
        LOG_ERR("unexpected signal %u", signal);
        break;
    }

    return p;
}

static void mpsl_nonpreemptible_thread_fn(void)
{
    int err;
    enum mpsl_api_call call;
    mpsl_timeslot_session_id_t session = 0xFF;

    while (1) {
        k_msgq_get(&ts_api_msgq, &call, K_FOREVER);
        switch (call) {
        case API_OPEN_SESSION:
            err = mpsl_timeslot_session_open(ts_signal_cb, &session);
            if (err) {
                LOG_ERR("session open: %d", err);
            }
            break;
        case API_MAKE_REQUEST:
            err = mpsl_timeslot_request(session, &ts_request_earliest);
            if (err) {
                LOG_ERR("ts request: %d", err);
            }
            break;
        case API_CLOSE_SESSION:
            err = mpsl_timeslot_session_close(session);
            if (err) {
                LOG_ERR("session close: %d", err);
            }
            break;
        }
    }
}

K_THREAD_DEFINE(mpsl_np_thread, MPSL_STACK_SIZE,
    mpsl_nonpreemptible_thread_fn, NULL, NULL, NULL,
    K_PRIO_COOP(MPSL_THREAD_PRIO), 0, 0);

void timeslot_handler_init(timeslot_cb_t cb)
{
    m_cb = cb;
}

void timeslot_handler_start(void)
{
    if (m_running) {
        return;
    }
    m_running = true;
    schedule_api(API_OPEN_SESSION);
    schedule_api(API_MAKE_REQUEST);
}

void timeslot_handler_stop(void)
{
    if (!m_running) {
        return;
    }
    m_running = false;
    /* Session closes after current slot ends (TIMER0 fires → ACTION_END → SESSION_CLOSED) */
}
