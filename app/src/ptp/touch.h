/*
 * Copyright (c) 2026 The ZMK Contributors
 * SPDX-License-Identifier: MIT
 */
#pragma once

#include <zmk/ptp.h>

#if IS_ENABLED(CONFIG_ZMK_PTP_TOUCH)
void zmk_ptp_touch_update(const struct zmk_ptp_frame *frame);
#else
static inline void zmk_ptp_touch_update(const struct zmk_ptp_frame *frame) {}
#endif
