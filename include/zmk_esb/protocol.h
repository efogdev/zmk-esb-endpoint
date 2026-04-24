/*
 * Copyright (c) 2025 efogdev
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <stdint.h>

#define ESB_MAX_PAYLOAD_LEN  CONFIG_ESB_MAX_PAYLOAD_LENGTH
#define ESB_PKT_DATA_MAX     (CONFIG_ESB_MAX_PAYLOAD_LENGTH - 2)

/* ESB pipe numbers */
#define ESB_PIPE_PAIRING     0
#define ESB_PIPE_DATA        1

/* Packet type byte (first byte of every ESB payload) */
#define ESB_PKT_BEACON       0x01
#define ESB_PKT_PAIR_REQ     0x02
#define ESB_PKT_PAIR_RESP    0x03
#define ESB_PKT_HID_REPORT   0x06
#define ESB_PKT_DISCONNECT   0x07

/* HID report subtypes */
#define ESB_REPORT_KEYBOARD  0x00
#define ESB_REPORT_CONSUMER  0x01
#define ESB_REPORT_MOUSE     0x02

/* Capability flags in BEACON */
#define ESB_CAP_KEYBOARD     BIT(0)
#define ESB_CAP_CONSUMER     BIT(1)
#define ESB_CAP_MOUSE        BIT(2)

/*
 * BEACON: keyboard → dongle (pipe 0, broadcast)
 * Sent repeatedly while unpaired.
 */
struct esb_pkt_beacon {
    uint8_t type;        /* ESB_PKT_BEACON */
    uint8_t device_id[6];
    uint8_t caps;
} __attribute__((__packed__));

/*
 * PAIR_REQ: dongle → keyboard (in ESB ACK payload on pipe 0)
 * Dongle sends this in the ACK payload after receiving a BEACON. The dongle's
 * FICR-derived device_id is included so the keyboard can persist it and
 * verify the peer on future reconnects.
 */
struct esb_pkt_pair_req {
    uint8_t type;         /* ESB_PKT_PAIR_REQ */
    uint8_t device_id[6]; /* dongle FICR device_id */
} __attribute__((__packed__));

/*
 * PAIR_RESP: keyboard → dongle (pipe 0)
 * Confirms pairing. Dongle switches to data pipe after this.
 */
struct esb_pkt_pair_resp {
    uint8_t type;        /* ESB_PKT_PAIR_RESP */
    uint8_t device_id[6];
    uint8_t caps;
} __attribute__((__packed__));

/*
 * HID_REPORT: keyboard → dongle (pipe 1)
 * Carries a ZMK HID report body verbatim.
 */
struct esb_pkt_hid_report {
    uint8_t type;        /* ESB_PKT_HID_REPORT */
    uint8_t report_type; /* ESB_REPORT_* */
    uint8_t data[ESB_PKT_DATA_MAX]; /* HID report body, zero-padded */
} __attribute__((__packed__));

/*
 * DISCONNECT: either direction (pipe 1)
 */
struct esb_pkt_disconnect {
    uint8_t type; /* ESB_PKT_DISCONNECT */
} __attribute__((__packed__));

/* Shell relay packet types */
#define ESB_PKT_SHELL_REQ     0x08  /* dongle→keyboard (ACK payload): start shell session */
#define ESB_PKT_SHELL_STOP    0x09  /* keyboard→dongle (TX): end shell session (timeout) */
#define ESB_PKT_SHELL_DATA    0x0A  /* bidirectional: raw shell bytes */
#define ESB_PKT_SHELL_POLL    0x0B  /* keyboard→dongle (TX): poll while in active shell mode */
#define ESB_PKT_SHELL_BG_POLL 0x0F  /* keyboard→dongle (TX): keepalive while NOT in shell mode */
#define ESB_PKT_SHELL_START   0x12  /* keyboard→dongle (TX): ask dongle to open a session (equivalent to dongle pair-button short-press while paired) */

struct esb_pkt_shell_req {
    uint8_t type; /* ESB_PKT_SHELL_REQ */
} __attribute__((__packed__));

struct esb_pkt_shell_stop {
    uint8_t type; /* ESB_PKT_SHELL_STOP */
} __attribute__((__packed__));

struct esb_pkt_shell_poll {
    uint8_t type; /* ESB_PKT_SHELL_POLL */
} __attribute__((__packed__));

struct esb_pkt_shell_bg_poll {
    uint8_t type; /* ESB_PKT_SHELL_BG_POLL */
} __attribute__((__packed__));

struct esb_pkt_shell_start {
    uint8_t type; /* ESB_PKT_SHELL_START */
} __attribute__((__packed__));

/*
 * SHELL_DATA: bidirectional shell bytes.
 *   dongle→keyboard: via ESB ACK payload, carries user input
 *   keyboard→dongle: via normal ESB TX, carries shell command output
 */
struct esb_pkt_shell_data {
    uint8_t type;                  /* ESB_PKT_SHELL_DATA */
    uint8_t len;                   /* valid bytes in data[] */
    uint8_t data[ESB_PKT_DATA_MAX]; /* raw bytes, only data[0..len-1] are valid */
} __attribute__((__packed__));

/* Mutual identity verification on reconnect. Each side persists the peer's
 * FICR device_id at pair time and re-validates on every boot:
 *   keyboard → dongle: VERIFY_REQ carries the keyboard's own device_id.
 *                      Dongle forwards HID only after the sender id matches
 *                      its stored peer. On mismatch or unknown peer the
 *                      dongle replies with DISCONNECT instead of VERIFY_RESP.
 *   dongle → keyboard: VERIFY_RESP (in ACK) carries the dongle's live
 *                      device_id. Keyboard transitions to CONNECTED only on
 *                      a matching id. */
#define ESB_PKT_VERIFY_REQ   0x10  /* keyboard→dongle (TX): identity challenge */
#define ESB_PKT_VERIFY_RESP  0x11  /* dongle→keyboard (ACK): identity response */

struct esb_pkt_verify_req {
    uint8_t type;         /* ESB_PKT_VERIFY_REQ */
    uint8_t device_id[6]; /* keyboard's live FICR device_id */
} __attribute__((__packed__));

struct esb_pkt_verify_resp {
    uint8_t type;         /* ESB_PKT_VERIFY_RESP */
    uint8_t device_id[6]; /* dongle's live FICR device_id */
} __attribute__((__packed__));

/*
 * RESYNC: dongle → keyboard (ACK payload, pipe 1)
 * Sent when the dongle boots into STATE_VERIFYING with a stored peer but the
 * keyboard is still in PAIRING_STATE_CONNECTED from a previous session (so it
 * sends HID reports instead of VERIFY_REQ). Nudges the keyboard back to
 * VERIFYING without wiping the stored peer, after which the normal
 * VERIFY_REQ/VERIFY_RESP handshake re-establishes the session.
 */
#define ESB_PKT_RESYNC       0x13  /* dongle→keyboard (ACK): please re-verify */

struct esb_pkt_resync {
    uint8_t type; /* ESB_PKT_RESYNC */
} __attribute__((__packed__));

/* Benchmark packet types */
#define ESB_PKT_BENCH_PING   0x0C  /* keyboard→dongle: benchmark probe */
#define ESB_PKT_BENCH_STOP   0x0D  /* keyboard→dongle: end benchmark, request result */
#define ESB_PKT_BENCH_RESULT 0x0E  /* dongle→keyboard (ACK payload): benchmark results */

struct esb_pkt_bench_ping {
    uint8_t type; /* ESB_PKT_BENCH_PING */
    uint8_t seq;  /* wrapping sequence number */
    char random[ESB_PKT_DATA_MAX];
} __attribute__((__packed__));

struct esb_pkt_bench_stop {
    uint8_t type; /* ESB_PKT_BENCH_STOP */
} __attribute__((__packed__));

struct esb_pkt_bench_result {
    uint8_t  type;      /* ESB_PKT_BENCH_RESULT */
    int8_t   rssi_avg;  /* average RSSI in dBm */
    int8_t   rssi_min;
    int8_t   rssi_max;
    uint32_t rx_count;  /* number of BENCH_PING packets received at dongle */
} __attribute__((__packed__));

/* Channel hop negotiation + idle gating.
 *
 *   PROPOSAL / CONFIRM: pre-agreement on the next channel. Endpoint
 *     proposes periodically while active; dongle confirms or counter-
 *     proposes. Both sides cache the result so it is ready when the
 *     endpoint decides to hop.
 *
 *   IDLE: endpoint → dongle. "No user input for a while; expect radio
 *     silence." Dongle disarms its silence watchdog on receipt; while
 *     the peer is known-idle, the dongle will never hop just because
 *     no packets are arriving. A re-send is fired periodically so the
 *     dongle re-enters idle mode even if the first IDLE was dropped.
 *     On the next non-IDLE packet (HID, VERIFY, PROPOSAL, etc.) the
 *     dongle flips back to active mode and rearms the watchdog.
 *
 * The endpoint itself only ever hops when it is active — it never fires
 * the hop trigger for packets sent during idle, so consecutive TX fails
 * on IDLE retries cannot strand an idle link.
 */
#define ESB_PKT_CHANNEL_HOP_PROPOSAL 0x14  /* keyboard→dongle (TX, pipe 1) */
#define ESB_PKT_CHANNEL_HOP_CONFIRM  0x15  /* dongle→keyboard (ACK, pipe 1) */
#define ESB_PKT_CHANNEL_HOP_REQUEST  0x16  /* dongle→keyboard (ACK, pipe 1): send a PROPOSAL now */
#define ESB_PKT_IDLE                 0x17  /* keyboard→dongle (TX, pipe 1) */

struct esb_pkt_channel_hop_proposal {
    uint8_t type;      /* ESB_PKT_CHANNEL_HOP_PROPOSAL */
    uint8_t proposed;  /* candidate next channel (0..100) */
    uint8_t current;   /* sender's currently active channel */
} __attribute__((__packed__));

struct esb_pkt_channel_hop_confirm {
    uint8_t type;      /* ESB_PKT_CHANNEL_HOP_CONFIRM */
    uint8_t agreed;    /* final agreed channel (== proposed on accept) */
    uint8_t accepted;  /* 1 if agreed equals proposed, 0 if counter-proposal */
} __attribute__((__packed__));

/* REQUEST: dongle → keyboard (ACK payload, pipe 1).
 * Queued whenever the dongle's committed_next is INVALID — most often
 * right after a failed speculative-hop validation cleared it. The
 * endpoint's steady-state PROPOSAL cadence is 60 s (kept slow for
 * battery), so without an explicit nudge a desynced dongle would sit
 * with no recovery target, fire silence_work to no effect, and burn
 * rollback cycles searching the band. On REQUEST the endpoint fires
 * a PROPOSAL immediately. */
struct esb_pkt_channel_hop_request {
    uint8_t type;      /* ESB_PKT_CHANNEL_HOP_REQUEST */
} __attribute__((__packed__));

struct esb_pkt_idle {
    uint8_t type;      /* ESB_PKT_IDLE */
} __attribute__((__packed__));
