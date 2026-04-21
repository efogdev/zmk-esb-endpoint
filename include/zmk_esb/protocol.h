/*
 * Copyright (c) 2025 efogdev
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <stdint.h>

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
} __packed;

/*
 * PAIR_REQ: dongle → keyboard (in ESB ACK payload on pipe 0)
 * Dongle sends this in the ACK payload after receiving a BEACON.
 */
struct esb_pkt_pair_req {
    uint8_t type; /* ESB_PKT_PAIR_REQ */
} __packed;

/*
 * PAIR_RESP: keyboard → dongle (pipe 0)
 * Confirms pairing. Dongle switches to data pipe after this.
 */
struct esb_pkt_pair_resp {
    uint8_t type;        /* ESB_PKT_PAIR_RESP */
    uint8_t device_id[6];
    uint8_t caps;
} __packed;

/*
 * HID_REPORT: keyboard → dongle (pipe 1)
 * Carries a ZMK HID report body verbatim.
 */
struct esb_pkt_hid_report {
    uint8_t type;        /* ESB_PKT_HID_REPORT */
    uint8_t report_type; /* ESB_REPORT_* */
    uint8_t data[30];    /* HID report body, zero-padded */
} __packed;

/*
 * DISCONNECT: either direction (pipe 1)
 */
struct esb_pkt_disconnect {
    uint8_t type; /* ESB_PKT_DISCONNECT */
} __packed;

#define ESB_MAX_PAYLOAD_LEN 32
