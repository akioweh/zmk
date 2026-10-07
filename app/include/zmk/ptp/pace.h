/*
 * Copyright (c) 2026 The ZMK Contributors
 * SPDX-License-Identifier: MIT
 */
#pragma once
#include <zephyr/kernel.h>

/* Only isolated pending motion waits; a following transition flushes it.
 * Account for the granted interval in microseconds, not rounded milliseconds.
 * The caller records last only after successful transport admission. */
static inline k_ticks_t zmk_ptp_pace_wait(bool motion, bool sent, int64_t last, int64_t now,
                                          uint32_t interval_us) {
#if IS_ENABLED(CONFIG_ZMK_BLE_PTP_PACING)
    if (motion && sent) {
        uint32_t us = interval_us ? interval_us + CONFIG_ZMK_BLE_PTP_PACE_MARGIN_US
                                  : CONFIG_ZMK_BLE_PTP_PACE_FALLBACK_US;
        /* Tick conversion is unsigned: cast before subtraction, or an expired
         * deadline wraps and schedules motion effectively forever. */
        int64_t period = k_us_to_ticks_ceil64(us);
        return MAX(INT64_C(0), last + period - now);
    }
#endif
    return 0;
}
