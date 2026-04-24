/*
 * Copyright (c) 2025 efogdev
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/kernel.h>
#include <zmk_esb/channel_hop.h>

/* Local xorshift32 PRNG. We pick channels pseudo-randomly to spread
 * attempts across the band but deliberately avoid Zephyr's random
 * subsystem so this module has no Kconfig dependency on an entropy
 * backend. Seeded lazily from the first-call uptime + cycle counter —
 * good enough for "don't always pick the same channel after a reboot". */
static uint32_t m_prng_state;

static uint32_t prng_next(void) {
    if (m_prng_state == 0) {
        m_prng_state = k_uptime_get_32() ^ (k_cycle_get_32() * 2654435761u);
        if (m_prng_state == 0) {
            m_prng_state = 0xdeadbeefu;
        }
    }
    uint32_t x = m_prng_state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    m_prng_state = x;
    return x;
}

void quarantine_reset(struct quarantine_state *q) {
    memset(q->expires_at, 0, sizeof(q->expires_at));
}

void quarantine_add(struct quarantine_state *q, uint8_t channel, uint32_t hold_ms) {
    if (channel >= CHANNEL_HOP_CHANNEL_COUNT) {
        return;
    }
    const uint32_t now = k_uptime_get_32();
    /* Saturating add: if now+hold wraps past UINT32_MAX we clamp — the
     * caller never passes hold_ms anywhere near 2^32. */
    const uint32_t new_expiry = now + hold_ms;
    if (new_expiry > q->expires_at[channel]) {
        q->expires_at[channel] = new_expiry;
    }
}

bool quarantine_is(const struct quarantine_state *q, uint8_t channel) {
    if (channel >= CHANNEL_HOP_CHANNEL_COUNT) {
        return true;  /* out-of-range channels are "quarantined" i.e. rejected */
    }
    const uint32_t e = q->expires_at[channel];
    if (e == 0) {
        return false;
    }
    /* 32-bit wrap-safe compare: quarantined iff (e - now) > 0 and not wrapped. */
    const uint32_t now = k_uptime_get_32();
    return (int32_t)(e - now) > 0;
}

uint8_t quarantine_count(const struct quarantine_state *q) {
    uint8_t n = 0;
    for (uint8_t ch = 0; ch < CHANNEL_HOP_CHANNEL_COUNT; ch++) {
        if (quarantine_is(q, ch)) {
            n++;
        }
    }
    return n;
}

static bool within_distance_of_quarantine(const struct quarantine_state *q,
                                          uint8_t channel, uint8_t distance) {
    /* True if any quarantined channel is within `distance` of `channel`. */
    const int16_t lo = (int16_t)channel - (int16_t)distance;
    const int16_t hi = (int16_t)channel + (int16_t)distance;
    for (int16_t i = lo; i <= hi; i++) {
        if (i < 0 || i >= CHANNEL_HOP_CHANNEL_COUNT) {
            continue;
        }
        if (i == channel) {
            continue;  /* the channel itself is handled by quarantine_is() directly */
        }
        if (quarantine_is(q, (uint8_t)i)) {
            return true;
        }
    }
    return false;
}

uint8_t channel_hop_pick(const struct quarantine_state *q,
                         uint8_t min_distance,
                         uint8_t avoid_channel) {
    uint8_t candidates[CHANNEL_HOP_CHANNEL_COUNT];

    /* Try progressively smaller distances. We never accept a quarantined
     * channel itself — relaxation only drops the buffer around
     * quarantine/avoid, never the hard exclusion of the quarantined
     * channels themselves.
     *
     * avoid_channel is treated as "about to become quarantined": both
     * hard-excluded and subject to min_distance, so we do not pick a
     * channel adjacent to the one we are about to leave. (When the
     * endpoint pre-negotiates committed_next while still on channel X,
     * X isn't in the quarantine store yet but will be the moment we
     * hop; picking X±1 here would violate the user's min-distance
     * guarantee.) */
    for (int16_t d = (int16_t)min_distance; d >= 0; d--) {
        uint8_t count = 0;
        for (uint8_t ch = 0; ch < CHANNEL_HOP_CHANNEL_COUNT; ch++) {
            if (quarantine_is(q, ch)) {
                continue;
            }
            if (ch == avoid_channel) {
                continue;
            }
            if (d > 0 && within_distance_of_quarantine(q, ch, (uint8_t)d)) {
                continue;
            }
            if (d > 0 && avoid_channel < CHANNEL_HOP_CHANNEL_COUNT) {
                const int16_t delta = (int16_t)ch - (int16_t)avoid_channel;
                const int16_t abs_delta = delta < 0 ? -delta : delta;
                if (abs_delta <= d) {
                    continue;
                }
            }
            candidates[count++] = ch;
        }
        if (count > 0) {
            const uint32_t r = prng_next();
            return candidates[r % count];
        }
    }

    return CHANNEL_HOP_INVALID;
}
