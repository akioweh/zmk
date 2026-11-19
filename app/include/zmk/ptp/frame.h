/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */
#pragma once

#include <stddef.h>
#include <zephyr/sys/util.h>
#include <zmk/ptp.h>

/* Sequence, scan time, first covered sequence, count/buttons, then up to five
 * (ID/confidence, little-endian X/Y) records. BLE omits unused slots; wired
 * envelopes retain the maximum-size object. Not a host HID report. */
#define ZMK_PTP_SPLIT_HEADER_SIZE 7
#define ZMK_PTP_SPLIT_FRAME_SIZE 32
struct zmk_ptp_split_contact {
    uint8_t id_confidence; /* ID in bits 1..7; confidence in bit 0. */
    uint16_t x, y;
} __packed;

/* Multibyte wire fields are little endian, including coordinates. */
struct zmk_ptp_split_frame {
    uint16_t sequence, scan_time, first_sequence;
    uint8_t count_buttons; /* Count in bits 0..2; buttons in bits 3..5. */
    struct zmk_ptp_split_contact contacts[ZMK_PTP_MAX_CONTACTS];
} __packed;

BUILD_ASSERT(offsetof(struct zmk_ptp_split_frame, contacts) == ZMK_PTP_SPLIT_HEADER_SIZE);
BUILD_ASSERT(sizeof(struct zmk_ptp_split_contact) == 5);
BUILD_ASSERT(sizeof(struct zmk_ptp_split_frame) == ZMK_PTP_SPLIT_FRAME_SIZE);

int zmk_ptp_validate_frame(const struct zmk_ptp_frame *frame);
void zmk_ptp_note_activity(const struct zmk_ptp_frame *frame);
size_t zmk_ptp_split_size(const struct zmk_ptp_split_frame *wire);
int zmk_ptp_split_unpack(struct zmk_ptp_split_frame *wire, const void *data, size_t length);
void zmk_ptp_split_encode(struct zmk_ptp_split_frame *wire, uint16_t sequence,
                          const struct zmk_ptp_frame *frame);
int zmk_ptp_split_decode(const struct zmk_ptp_split_frame *wire, uint16_t *sequence,
                         struct zmk_ptp_frame *frame);
