/*
 * Copyright (c) 2026 The ZMK Contributors
 * SPDX-License-Identifier: MIT
 */
#pragma once
#include <zmk/ptp.h>

/* Known geometry: horizontal, lift, vertical, lift, two diagonals, lift.
 * IDs remain stable and every reuse has a preceding empty snapshot. */
static inline struct zmk_ptp_frame zmk_ptp_test_frame(uint32_t step) {
    struct zmk_ptp_frame f = {.scan_time = step * 100};
    uint32_t phase = step % 450;
    if (phase >= 400 || (phase >= 100 && phase < 150) || (phase >= 250 && phase < 300)) {
        return f;
    }
    uint32_t i = phase < 100 ? phase : phase < 250 ? phase - 150 : phase - 300;
    f.contact_count = phase >= 300 ? 2 : 1;
    uint16_t x = CONFIG_ZMK_TRACKPAD_LOGICAL_X / 4 + i * (CONFIG_ZMK_TRACKPAD_LOGICAL_X / 2) / 99;
    uint16_t y = CONFIG_ZMK_TRACKPAD_LOGICAL_Y / 4 + i * (CONFIG_ZMK_TRACKPAD_LOGICAL_Y / 2) / 99;
    f.contacts[0] = (struct zmk_ptp_contact){
        .id = 0,
        .confidence = true,
        .x = phase >= 150 && phase < 250 ? CONFIG_ZMK_TRACKPAD_LOGICAL_X / 2 : x,
        .y = phase < 100 ? CONFIG_ZMK_TRACKPAD_LOGICAL_Y / 2 : y};
    if (f.contact_count == 2) {
        f.contacts[1] = (struct zmk_ptp_contact){.id = 1,
                                                 .confidence = true,
                                                 .x = CONFIG_ZMK_TRACKPAD_LOGICAL_X - x,
                                                 .y = CONFIG_ZMK_TRACKPAD_LOGICAL_Y - y};
    }
    return f;
}
