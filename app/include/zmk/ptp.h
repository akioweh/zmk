/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

/** One active contact. IDs are stable slots in [0, CONFIG_ZMK_TRACKPAD_FINGERS). */
struct zmk_ptp_contact {
    uint8_t id;
    uint16_t x;
    uint16_t y;
    bool confidence;
};

/** A complete snapshot, not a delta. Missing contacts are automatically lifted. */
struct zmk_ptp_frame {
    /* Sensor scan time in 100 us units, wrapping at 65536. */
    uint16_t scan_time;
    uint8_t contact_count;
    /* Bits 0..2: integrated, external primary, external secondary buttons. */
    uint8_t buttons;
    struct zmk_ptp_contact contacts[CONFIG_ZMK_TRACKPAD_FINGERS];
};

/**
 * Submit one complete frame to the selected endpoint from thread context.
 *
 * The core copies the frame, preserves lift coordinates, and sends one whole
 * HID report. Returns 0 when the transport accepts it, or a negative errno.
 * On failure the previous accepted contact state is retained: retry with a
 * complete snapshot. -EAGAIN indicates contention or suspension; -ENODEV
 * indicates no endpoint, and BLE can return -EMSGSIZE until MTU negotiation.
 * Do not reuse an ID for a different contact until an accepted snapshot omits it.
 */
int zmk_ptp_submit_frame(const struct zmk_ptp_frame *frame);

/** Release contacts/buttons. Transport-failed releases are deferred; retry -EAGAIN on contention.
 */
int zmk_ptp_release(void);

/** Convenience timestamp for drivers without a hardware scan clock. */
uint16_t zmk_ptp_scan_time(void);
