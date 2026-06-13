/*
 * Copyright (c) 2025 efogdev
 * SPDX-License-Identifier: MIT
 *
 * Endpoint-side (PTX) channel-hop manager. Owns quarantine state, the
 * committed-next-channel cache, and the negotiation / hop work queues.
 * esb_transport calls in from the TX_FAILED ISR; pairing.c calls in when
 * a CONFIRM packet arrives.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

void channel_hop_ep_init(void);

/* Called from esb_endpoint when pairing transitions into CONNECTED state
 * (fresh pair or successful VERIFY) and when the link drops, respectively. */
void channel_hop_ep_on_connected(void);
void channel_hop_ep_on_disconnected(void);

/* Called from pairing.c's RX dispatch for CHANNEL_HOP_CONFIRM packets. */
void channel_hop_ep_on_rx(const uint8_t *data, uint8_t len);

/* Called from pairing.c when a CHANNEL_HOP_REQUEST arrives (dongle's
 * committed_next is INVALID and it wants a fresh PROPOSAL). Schedules
 * negotiate_work to fire immediately so the next outbound TX carries a
 * PROPOSAL instead of waiting up to NEGOTIATE_INTERVAL_MS for the slow
 * steady-state cadence. Idempotent — back-to-back REQUESTs collapse to
 * one work dispatch. */
void channel_hop_ep_on_request(void);

/* Called from the ESB event ISR once TX has been failing continuously
 * for CHANNEL_HOP_TX_FAIL_WINDOW_MS (measured from the first failure in
 * the current streak). Submits a work item; never does the hop directly
 * (ISR-unsafe APIs involved).
 *
 * No-op if the endpoint is currently in the idle state — a failing
 * IDLE heartbeat must not trigger a hop.
 */
void channel_hop_ep_on_tx_fail_isr(void);

/* Called from ESB event ISR on TX_SUCCESS. Used to cancel the "revert to
 * previous channel" arming set up by the last hop: if at least one packet
 * got through on the new channel, the hop is considered good and any
 * future hop picks a fresh candidate rather than bouncing back. */
void channel_hop_ep_on_tx_success_isr(void);

/* Called from esb_transport_send() whenever the endpoint sends a packet
 * that represents real user traffic (HID reports, shell relay bytes).
 * Keeps the endpoint in the active state; any idle transition is
 * deferred by the IDLE_THRESHOLD_MS deadline. Cheap to call from
 * thread context; safe from ISR. */
void channel_hop_ep_note_user_tx(void);

/* Shell / debug getters. */
uint8_t channel_hop_ep_get_committed(void);
uint8_t channel_hop_ep_get_quarantine_count(void);
bool    channel_hop_ep_is_quarantined(uint8_t channel);
bool    channel_hop_ep_is_active(void);

struct channel_hop_ep_offender {
    uint8_t  channel;
    uint16_t hits;          /* total evidence, incl. the baseline restored from NVS */
    uint16_t session_hits;  /* condemnations recorded since this boot */
    bool     persistent;    /* currently in the persisted avoided set */
};

/* Fills `out` with up to `max` known-offender channels (those condemned by
 * fail-driven hops), ordered by hit count descending. Returns the number
 * written. Always 0 unless CHANNEL_QUARANTINE_PERSIST is built in. */
uint8_t channel_hop_ep_get_offenders(struct channel_hop_ep_offender *out, uint8_t max);
