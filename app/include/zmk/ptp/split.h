/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */
#pragma once

#include <zephyr/sys/util.h>
#include <zmk/ptp/frame.h>

#if IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
int zmk_ptp_split_receive(uint8_t source, const struct zmk_ptp_split_frame *wire);
void zmk_ptp_split_reset(void);
/* Unconfident cleanup on source loss. Admission failures are deferred;
 * only lock contention needs a caller retry. */
int zmk_ptp_cancel(void);
#else
void zmk_ptp_split_resume(void);
#endif
