/*
 * Copyright (c) 2026 The ZMK Contributors
 * SPDX-License-Identifier: MIT
 */
#define DT_DRV_COMPAT zmk_ptp_touch

#include <zephyr/device.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include "touch.h"

/* Only touch state enters the legacy input pipeline, never motion or clicks. */
BUILD_ASSERT(DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 1,
             "One logical PTP touch-state device is supported");
DEVICE_DT_INST_DEFINE(0, NULL, NULL, NULL, NULL, POST_KERNEL,
                      CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, NULL);

static atomic_t touching;
static bool reported;
static void report_touch(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(touch_work, report_touch);

static void report_touch(struct k_work *work) {
    bool state = atomic_get(&touching);
    if (state == reported) {
        return;
    }
    int err = input_report_key(DEVICE_DT_INST_GET(0), INPUT_BTN_TOUCH, state,
                               true, K_NO_WAIT);
    if (!err) {
        reported = state;
    }
    /* Retry queue backpressure, including releases, without blocking PTP.
     * Re-read the latest physical state rather than replaying stale touches. */
    if (err || state != atomic_get(&touching)) {
        k_work_reschedule(&touch_work, K_MSEC(1));
    }
}

void zmk_ptp_touch_update(const struct zmk_ptp_frame *frame) {
    bool state = false;
    if (frame) {
        for (int i = 0; i < frame->contact_count; i++) {
            state |= frame->contacts[i].confidence;
        }
    }
    if (atomic_set(&touching, state) != state) {
        k_work_reschedule(&touch_work, K_NO_WAIT);
    }
}
