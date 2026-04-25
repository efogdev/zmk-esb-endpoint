/*
 * Copyright (c) 2025 efogdev
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <zephyr/kernel.h>

typedef enum {
    ESB_TX_EVT_SUCCESS,
    ESB_TX_EVT_FAIL,
    ESB_RX_EVT,
} esb_transport_evt_type_t;

typedef struct {
    esb_transport_evt_type_t type;
    const uint8_t *rx_buf;
    uint8_t rx_len;
} esb_transport_evt_t;

typedef void (*esb_transport_cb_t)(const esb_transport_evt_t *evt);

int  esb_transport_init(esb_transport_cb_t cb);
void esb_transport_deinit(void);
void esb_transport_set_addresses(const uint8_t base0[4], const uint8_t base1[4], const uint8_t prefixes[8], uint8_t channel);
int  esb_transport_send(uint8_t pipe, const uint8_t *data, uint8_t len);

/* Current RF channel (last value applied via set_addresses or set_channel). */
uint8_t esb_transport_get_channel(void);

/* The DTS-default channel the radio booted on. Used as a stable rendezvous
 * channel for re-syncing with a desynced dongle: the dongle's rollback
 * dwell cycle always includes this channel, so a packet sent here will
 * eventually be heard even if the regular committed_next negotiation has
 * gone stale. */
uint8_t esb_transport_get_rendezvous_channel(void);

/* Change the active RF channel. Flushes the TX FIFO first and re-applies
 * the channel to the ESB radio. Safe to call from thread context while
 * the ESB slot is active; no-op with error if inactive. */
int esb_transport_set_channel(uint8_t channel);

/* Send one packet on `channel` (without changing the active channel) and
 * block until the ESB radio reports TX_SUCCESS, TX_FAILED, or `timeout`
 * elapses. Used for rendezvous side-trips during the post-hop burst.
 * Serialised: only one in-flight blocking send is allowed at a time —
 * a concurrent caller gets -EBUSY. While blocked, regular esb_transport_send
 * calls are silently dropped. Returns 0 on TX_SUCCESS, -EIO on TX_FAILED,
 * -ETIMEDOUT if the radio did not produce an event in time. */
int esb_transport_send_blocking(uint8_t channel, uint8_t pipe,
                                const uint8_t *data, uint8_t len,
                                k_timeout_t timeout);

/* Zero the consecutive-TX-fail counter. Used by the channel-hop state
 * machine to re-arm the `>= THRESHOLD` trigger after a hop attempt that
 * did not reset the counter through a successful set_channel (e.g. an
 * -EBUSY retry, or "no candidate" pick). Without this, one missed hop
 * opportunity would leave the counter monotonically growing past the
 * threshold and the hop ISR trigger permanently disabled. */
void esb_transport_reset_consecutive_fail(void);

/* Milliseconds since the most recent ESB RX or non-bg-poll TX, or UINT32_MAX
 * if no activity has been observed yet. Used by the shell relay to gate its
 * idle polling: polls only go out when some other ESB traffic has been seen
 * recently. */
uint32_t esb_transport_ms_since_activity(void);

/* Snapshot of the link-quality metric used by the cooperative-hop trigger.
 * `*retried_out` (if non-NULL) gets the popcount of the last
 * LINK_QUALITY_WINDOW ACKed packets that needed at least one ESB
 * retransmit. `*ewma_x10_out` (if non-NULL) gets the EWMA of tx_attempts
 * times 10 (so 10 == 1.0 attempts == perfect link, 25 == 2.5 attempts).
 * Returns 0 always; the args are independent so a caller can request
 * just one. Reads are non-locking single-word loads; values may shift
 * between fields but are individually consistent.
 *
 * No-op (writes 0) if CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_HOP is disabled. */
int esb_transport_get_link_quality(uint8_t *retried_out, uint16_t *ewma_x10_out);

/* True when the transport is in a state that would silently drop user
 * sends (post-hop quiet window, synchronous side-trip in flight). Allows
 * callers that accumulate state (e.g. the mouse input processor's pending
 * dx/dy) to skip emitting a report whose payload would be thrown away,
 * preserving the accumulated delta for the next emission. */
bool esb_transport_is_quiet(void);

/* Handle an inbound ESB_PKT_LINK_STATS payload. Called from pairing.c's
 * RX dispatch. Silently drops too-short frames. */
void esb_transport_on_rx_link_stats(const uint8_t *data, uint8_t len);

/* Latest peer-reported RSSI snapshot. Returns false if no LINK_STATS has
 * been received since the last connect or channel hop (outputs untouched).
 * last_out / ewma_out may be NULL. */
bool esb_transport_get_peer_rssi(int8_t *last_out, int8_t *ewma_out);

/* Cumulative lifetime PER counters. Any output pointer may be NULL.
 *  ok_out        — ACKed TX events (includes events that needed retries)
 *  retried_out   — subset of ok where tx_attempts > 1 (at least one retry)
 *  exhausted_out — TX events that exhausted retransmit_count and failed
 * Sync-TX (rendezvous side-trip) events are excluded from all three.
 * PER over any interval ≈ delta(exhausted) / (delta(ok) + delta(exhausted)). */
void esb_transport_get_per_stats(uint32_t *ok_out, uint32_t *retried_out,
                                 uint32_t *exhausted_out);

void esb_transport_on_slot_start(void);
void esb_transport_on_slot_stop(void);
