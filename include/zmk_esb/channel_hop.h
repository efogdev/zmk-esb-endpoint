/*
 * Copyright (c) 2025 efogdev
 * SPDX-License-Identifier: MIT
 *
 * Shared channel-hop algorithms used by both the keyboard (PTX) and the
 * dongle (PRX). Pure data / selection logic; no knowledge of ESB, work
 * queues, or Zephyr subsystems other than k_uptime and sys_rand32.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

/* ESB supports channels 0..100 (2400..2500 MHz in 1 MHz steps). */
#define CHANNEL_HOP_CHANNEL_COUNT 101
#define CHANNEL_HOP_INVALID 0xFF

struct quarantine_state {
    /* expires_at[ch] == uptime (ms) at which quarantine on channel ch ends.
     * Zero means "never quarantined". Channel ch is quarantined iff
     * expires_at[ch] > k_uptime_get_32(). */
    uint32_t expires_at[CHANNEL_HOP_CHANNEL_COUNT];
};

void quarantine_reset(struct quarantine_state *q);

/* Mark a channel as quarantined for at least hold_ms from now. If the
 * channel is already quarantined for longer, the existing expiry wins. */
void quarantine_add(struct quarantine_state *q, uint8_t channel, uint32_t hold_ms);

bool quarantine_is(const struct quarantine_state *q, uint8_t channel);

/* Number of currently quarantined channels. Useful for logging / shell. */
uint8_t quarantine_count(const struct quarantine_state *q);

/* Pick a new channel from [0..100] respecting quarantine and the requested
 * minimum distance from any quarantined channel. If no channel satisfies
 * the full min_distance constraint, the distance is progressively relaxed
 * (decremented by 1) until at least one candidate remains, or distance=0.
 * Also excludes `avoid_channel` (typically the current channel on the
 * caller's side, to guarantee we actually move).
 *
 * Returns CHANNEL_HOP_INVALID if no candidate exists even at distance=0
 * (e.g. every channel is quarantined and also matches avoid_channel —
 * should not happen in practice).
 */
uint8_t channel_hop_pick(const struct quarantine_state *q,
                         uint8_t min_distance,
                         uint8_t avoid_channel);
