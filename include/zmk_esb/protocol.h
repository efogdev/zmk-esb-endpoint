/*
 * Copyright (c) 2025 efogdev
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <stdint.h>

#define ESB_MAX_PAYLOAD_LEN  CONFIG_ESB_MAX_PAYLOAD_LENGTH
#define ESB_PKT_DATA_MAX     (CONFIG_ESB_MAX_PAYLOAD_LENGTH - 2)

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
 * Dongle sends this in the ACK payload after receiving a BEACON.
 */
struct esb_pkt_pair_req {
    uint8_t type; /* ESB_PKT_PAIR_REQ */
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
#define ESB_PKT_SHELL_REQ   0x08  /* dongle→keyboard (ACK payload): start shell session */
#define ESB_PKT_SHELL_STOP  0x09  /* keyboard→dongle (TX): end shell session (timeout) */
#define ESB_PKT_SHELL_DATA  0x0A  /* bidirectional: raw shell bytes */
#define ESB_PKT_SHELL_POLL  0x0B  /* keyboard→dongle (TX): poll for queued input */

struct esb_pkt_shell_req {
    uint8_t type; /* ESB_PKT_SHELL_REQ */
} __attribute__((__packed__));

struct esb_pkt_shell_stop {
    uint8_t type; /* ESB_PKT_SHELL_STOP */
} __attribute__((__packed__));

struct esb_pkt_shell_poll {
    uint8_t type; /* ESB_PKT_SHELL_POLL */
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
