/*
 * Copyright (c) 2026 The ZMK Contributors
 * SPDX-License-Identifier: MIT
 */
#pragma once
#include <errno.h>
#include <zephyr/kernel.h>
#include <zmk/ptp/transport.h>

/* Host compatibility belongs to HoG, not to the generic feature encoder. */
static inline int zmk_ptp_get_ble_feature(struct zmk_endpoint_instance endpoint, uint8_t id,
                                          uint8_t *data, size_t size) {
    bool pad = IS_ENABLED(CONFIG_ZMK_BLE_PTP_FEATURE_PAD_BYTE);
    if (!size && pad) {
        return -EMSGSIZE;
    }
    int len = zmk_ptp_get_feature(endpoint, id, data, size - pad);
    if (len >= 0 && pad) {
        data[len++] = 0;
    }
    return len;
}

/* Only isolated pending motion waits while a notification is in flight;
 * a following transition flushes it immediately. */
static inline bool zmk_ptp_pace_wait(bool motion, unsigned int in_flight) {
#if IS_ENABLED(CONFIG_ZMK_BLE_PTP_PACING)
    return motion && in_flight > 0;
#else
    return false;
#endif
}
