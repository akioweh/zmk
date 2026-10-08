/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#define ZMK_PTP_MAX_CONTACTS 5

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
    struct zmk_ptp_contact contacts[ZMK_PTP_MAX_CONTACTS];
};

/**
 * Submit one complete frame from thread context, on either split role.
 *
 * Central/unibody: send one HID report to the selected endpoint, preserving
 * lift coordinates and previous accepted state on transport failure.
 * Peripheral: copy and forward one contact event using the active split
 * transport. The latest physical snapshot is cached/retried even offline;
 * sequence gaps let the central cancel a lost contact lifetime.
 * Returns 0 on local transport admission, not remote delivery acknowledgment.
 * Retry negative errors with a complete snapshot. -EAGAIN indicates contention
 * or suspension; -ENODEV indicates no endpoint/link; BLE can return -EMSGSIZE
 * until MTU negotiation. Do not reuse an ID until an accepted snapshot omits it.
 */
int zmk_ptp_submit_frame(const struct zmk_ptp_frame *frame);

/** Release contacts/buttons. Transport-failed releases are deferred; retry -EAGAIN on contention.
 */
int zmk_ptp_release(void);

/** Contact/button/confidence state, excluding coordinates and scan time. */
uint16_t zmk_ptp_frame_state(const struct zmk_ptp_frame *frame);

/** Convenience timestamp for drivers without a hardware scan clock. */
uint16_t zmk_ptp_scan_time(void);
