/*
 * Copyright (c) 2026 The ZMK Contributors
 * SPDX-License-Identifier: MIT
 */
#include <errno.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include "test_source.h"

/* Diagnostic image only. Use the real producer/split/host path, never a second
 * HID interface. Keep a rejected observation intact so no lift is swallowed. */
K_THREAD_STACK_DEFINE(source_stack, 1536);
static struct k_work_q source_queue;
static uint32_t step;
static struct zmk_ptp_frame frame;
static bool pending;
static void generate(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(source_work, generate);

static void generate(struct k_work *work) {
    if (!pending) {
        frame = zmk_ptp_test_frame(step);
        frame.scan_time = zmk_ptp_scan_time();
        pending = true;
    }
    int err = zmk_ptp_submit_frame(&frame);
    bool retry = err == -EAGAIN || err == -ENOMSG || err == -ENOSPC || err == -ENOMEM;
    if (!retry) {
        step++;
        pending = false; /* Offline observations still enter the producer cache. */
    }
    k_work_schedule_for_queue(&source_queue, &source_work, K_MSEC(retry ? 5 : 10));
}

static int source_init(void) {
    k_work_queue_start(&source_queue, source_stack, K_THREAD_STACK_SIZEOF(source_stack),
                       K_PRIO_PREEMPT(5), NULL);
    k_work_schedule_for_queue(&source_queue, &source_work, K_SECONDS(2));
    return 0;
}
SYS_INIT(source_init, APPLICATION, 90);
