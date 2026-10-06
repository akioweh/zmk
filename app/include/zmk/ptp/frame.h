/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */
#pragma once

#include <zmk/ptp.h>

/* Internal, sensor-neutral split payload: sequence, scan time, count/buttons,
 * five (ID/confidence, little-endian X, little-endian Y) records. Not HID. */
#define ZMK_PTP_SPLIT_FRAME_SIZE 30
struct zmk_ptp_split_frame {
    uint8_t data[ZMK_PTP_SPLIT_FRAME_SIZE];
};

int zmk_ptp_validate_frame(const struct zmk_ptp_frame *frame);
void zmk_ptp_split_encode(struct zmk_ptp_split_frame *wire, uint16_t sequence,
                          const struct zmk_ptp_frame *frame);
int zmk_ptp_split_decode(const struct zmk_ptp_split_frame *wire, uint16_t *sequence,
                         struct zmk_ptp_frame *frame);
