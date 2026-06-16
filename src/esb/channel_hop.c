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

#if defined(CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_RSSI_WEIGHT)
void channel_rssi_reset(struct channel_rssi_state *r) {
    for (uint8_t ch = 0; ch < CHANNEL_HOP_CHANNEL_COUNT; ch++) {
        r->ewma[ch] = CHANNEL_RSSI_NONE;
    }
}

void channel_rssi_update(struct channel_rssi_state *r, const uint8_t channel, const int8_t sample) {
    if (channel >= CHANNEL_HOP_CHANNEL_COUNT) {
        return;
    }
    if (r->ewma[channel] == CHANNEL_RSSI_NONE) {
        r->ewma[channel] = sample;
        return;
    }
    const int32_t alpha = CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_RSSI_EWMA_ALPHA;
    const int32_t scale = CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_RSSI_WEIGHT_SCALE;
    r->ewma[channel] = (int8_t)((alpha * sample + (scale - alpha) * r->ewma[channel]) / scale);
}

bool channel_rssi_get(const struct channel_rssi_state *r, const uint8_t channel, int8_t *out) {
    if (channel >= CHANNEL_HOP_CHANNEL_COUNT || r->ewma[channel] == CHANNEL_RSSI_NONE) {
        return false;
    }
    if (out) {
        *out = r->ewma[channel];
    }
    return true;
}
#endif /* CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_RSSI_WEIGHT */

#if defined(CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_QUARANTINE_PERSIST)
bool persistent_quarantine_contains(const struct persistent_quarantine *p, uint8_t channel) {
    for (uint8_t i = 0; i < p->count; i++) {
        if (p->channels[i] == channel) {
            return true;
        }
    }
    return false;
}
#endif /* CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_QUARANTINE_PERSIST */

void quarantine_reset(struct quarantine_state *q) {
    memset(q->expires_at, 0, sizeof(q->expires_at));
#if defined(CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_QUARANTINE_PERSIST)
    q->persistent.count = 0;
#endif
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

static bool timed_quarantine_is(const struct quarantine_state *q, uint8_t channel) {
    const uint32_t e = q->expires_at[channel];
    if (e == 0) {
        return false;
    }
    /* 32-bit wrap-safe compare: quarantined iff (e - now) > 0 and not wrapped. */
    const uint32_t now = k_uptime_get_32();
    return (int32_t)(e - now) > 0;
}

bool quarantine_is(const struct quarantine_state *q, uint8_t channel) {
    if (channel >= CHANNEL_HOP_CHANNEL_COUNT) {
        return true;  /* out-of-range channels are "quarantined" i.e. rejected */
    }
#if defined(CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_QUARANTINE_PERSIST)
    if (persistent_quarantine_contains(&q->persistent, channel)) {
        return true;
    }
#endif
    return timed_quarantine_is(q, channel);
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

/* True if any TIMED-quarantined channel is within `distance` of `channel`.
 * The persistent set is excluded here — it has its own, narrower buffer in
 * channel_hop_pick(); a recently-bad channel warrants a full-width guard
 * (interference bleeds into neighbours now), a historically-bad one only a
 * half-width nudge. */
static bool within_distance_of_quarantine(const struct quarantine_state *q,
                                          uint8_t channel, uint8_t distance) {
    const int16_t lo = (int16_t)channel - (int16_t)distance;
    const int16_t hi = (int16_t)channel + (int16_t)distance;
    for (int16_t i = lo; i <= hi; i++) {
        if (i < 0 || i >= CHANNEL_HOP_CHANNEL_COUNT) {
            continue;
        }
        if (i == channel) {
            continue;  /* the channel itself is handled by quarantine_is() directly */
        }
        if (timed_quarantine_is(q, (uint8_t)i)) {
            return true;
        }
    }
    return false;
}

#if defined(CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_QUARANTINE_PERSIST)
/* True if any persistent worst-offender channel is within `distance` of
 * `channel`. Used with half the timed-quarantine min_distance. */
static bool within_distance_of_persistent(const struct quarantine_state *q,
                                          uint8_t channel, uint8_t distance) {
    const int16_t lo = (int16_t)channel - (int16_t)distance;
    const int16_t hi = (int16_t)channel + (int16_t)distance;
    for (int16_t i = lo; i <= hi; i++) {
        if (i < 0 || i >= CHANNEL_HOP_CHANNEL_COUNT || i == channel) {
            continue;
        }
        if (persistent_quarantine_contains(&q->persistent, (uint8_t)i)) {
            return true;
        }
    }
    return false;
}
#endif

#if defined(CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_RSSI_WEIGHT)
/* Draw one candidate index, biased by per-channel RSSI. Candidates with a
 * sample map linearly onto [WEIGHT_MIN, WEIGHT_MAX] (best RSSI -> MAX);
 * unsampled candidates take the midpoint. Falls back to a uniform draw when
 * no candidate carries a sample. */
static int32_t candidate_weight(const struct channel_rssi_state *rssi, const uint8_t channel,
                                const int8_t lo, const int8_t hi) {
    const int32_t w_min = CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_RSSI_WEIGHT_MIN;
    const int32_t w_max = CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_RSSI_WEIGHT_MAX;
    int8_t s;
    if (!channel_rssi_get(rssi, channel, &s) || hi == lo) {
        return (w_min + w_max) / 2;
    }
    return w_min + (w_max - w_min) * (int32_t)(s - lo) / (int32_t)(hi - lo);
}

static uint8_t weighted_index(const uint8_t *candidates, const uint8_t count,
                              const struct channel_rssi_state *rssi) {
    int8_t lo = 0, hi = 0;
    bool any = false;
    for (uint8_t i = 0; i < count; i++) {
        int8_t s;
        if (!channel_rssi_get(rssi, candidates[i], &s)) {
            continue;
        }
        if (!any) {
            lo = hi = s;
            any = true;
        } else {
            if (s < lo) lo = s;
            if (s > hi) hi = s;
        }
    }
    if (!any) {
        return (uint8_t)(prng_next() % count);
    }

    /* Two passes (sum, then walk) so no per-candidate weight buffer is needed
     * on the workqueue stack the picker runs on. */
    int32_t total = 0;
    for (uint8_t i = 0; i < count; i++) {
        total += candidate_weight(rssi, candidates[i], lo, hi);
    }
    int32_t r = (int32_t)(prng_next() % (uint32_t)total);
    for (uint8_t i = 0; i < count; i++) {
        r -= candidate_weight(rssi, candidates[i], lo, hi);
        if (r < 0) {
            return i;
        }
    }
    return (uint8_t)(count - 1);
}
#endif /* CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_RSSI_WEIGHT */

uint8_t channel_hop_pick(const struct quarantine_state *q,
                         uint8_t min_distance,
                         uint8_t avoid_channel) {
    return channel_hop_pick_weighted(q, min_distance, avoid_channel, NULL);
}

uint8_t channel_hop_pick_weighted(const struct quarantine_state *q,
                                  uint8_t min_distance,
                                  uint8_t avoid_channel,
                                  const struct channel_rssi_state *rssi) {
#if !defined(CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_RSSI_WEIGHT)
    ARG_UNUSED(rssi);
#endif
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
#if defined(CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_QUARANTINE_PERSIST)
            /* Persistent offenders get half the buffer of timed quarantine,
             * relaxing in lockstep with d so the candidate-exists guarantee
             * still holds at d=0. */
            if (d > 1 && within_distance_of_persistent(q, ch, (uint8_t)(d / 2))) {
                continue;
            }
#endif
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
#if defined(CONFIG_ZMK_ESB_ENDPOINT_CHANNEL_RSSI_WEIGHT)
            if (rssi != NULL) {
                return candidates[weighted_index(candidates, count, rssi)];
            }
#endif
            return candidates[prng_next() % count];
        }
    }

    return CHANNEL_HOP_INVALID;
}
