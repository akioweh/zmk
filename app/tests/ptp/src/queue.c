/*
 * Copyright (c) 2026 The ZMK Contributors
 * SPDX-License-Identifier: MIT
 */
#include <errno.h>
#include <zephyr/ztest.h>
#include <zmk/ptp/queue.h>
#include <zmk/ptp/frame.h>
#include <zmk/ptp/transport.h>
#include "hog.h"
#include "test_source.h"
#include <zephyr/sys/byteorder.h>

struct item {
    struct zmk_ptp_queue_header header;
    int position;
};
K_MSGQ_DEFINE(queue, sizeof(struct item), 4, 4);
static uint32_t ticket;

static int put(int position, uint16_t state, bool motion) {
    uint32_t next = ++ticket;
    struct item item = {.header = {.ticket = next,
                                   .state = state,
                                   .first_sequence = next,
                                   .motion = motion},
                        .position = position},
                scratch;
    return zmk_ptp_queue_put(&queue, &item, &scratch);
}

static void before(void *fixture) {
    k_msgq_purge(&queue);
    ticket = 0;
}

ZTEST(ptp_queue, test_transition_then_latest_motion) {
    zassert_ok(put(1, 1, false)); /* Initial down is an immutable anchor. */
    zassert_ok(put(2, 1, true));
    for (int i = 3; i <= 100; i++) {
        zassert_equal(put(i, 1, true), 1);
    }
    zassert_equal(k_msgq_num_used_get(&queue), 2);
    struct item item;
    zassert_ok(k_msgq_get(&queue, &item, K_NO_WAIT));
    zassert_equal(item.position, 1);
    zassert_ok(k_msgq_get(&queue, &item, K_NO_WAIT));
    zassert_equal(item.position, 100);
    zassert_equal(item.header.first_sequence, 2); /* Coverage includes all replaced motion. */
}

ZTEST(ptp_queue, test_lift_retouch_and_full_queue) {
    zassert_ok(put(1, 1, false));
    zassert_ok(put(2, 1, true));
    zassert_ok(put(3, 0, false)); /* Lift. */
    zassert_ok(put(4, 1, false)); /* Re-touch of same ID is not the old lifetime. */
    zassert_equal(put(5, 1, true), -ENOMSG);
    for (int i = 1; i <= 4; i++) {
        struct item item;
        zassert_ok(k_msgq_get(&queue, &item, K_NO_WAIT));
        zassert_equal(item.position, i);
    }
}

ZTEST(ptp_queue, test_full_queue_can_replace_motion_but_not_cross_state) {
    for (int i = 1; i <= 3; i++) {
        zassert_ok(put(i, i, false));
    }
    zassert_ok(put(4, 3, true));
    zassert_equal(put(5, 3, true), 1);
    zassert_equal(put(6, 4, true), -ENOMSG);
    struct item item;
    for (int i = 1; i <= 3; i++) {
        zassert_ok(k_msgq_get(&queue, &item, K_NO_WAIT));
        zassert_equal(item.position, i);
    }
    zassert_ok(k_msgq_get(&queue, &item, K_NO_WAIT));
    zassert_equal(item.position, 5);
}

ZTEST(ptp_queue, test_unlocked_head_overwrite_does_not_dequeue_new_observation) {
    zassert_ok(put(1, 1, true));
    struct item copied, removed;
    zassert_ok(k_msgq_peek(&queue, &copied));
    zassert_equal(put(2, 1, true), 1); /* Arrives while the old copy is being sent. */
    zassert_false(zmk_ptp_queue_get(&queue, copied.header.ticket, &removed));
    zassert_ok(k_msgq_peek(&queue, &copied));
    zassert_equal(copied.position, 2);
    zassert_true(zmk_ptp_queue_get(&queue, copied.header.ticket, &removed));
    zassert_equal(removed.position, 2);
    zassert_ok(put(3, 1, false));
    zassert_ok(k_msgq_peek(&queue, &copied));
    zassert_ok(put(4, 1, true)); /* Appending a tail must not invalidate the head. */
    zassert_true(zmk_ptp_queue_get(&queue, copied.header.ticket, &removed));
    zassert_equal(removed.position, 3);
}

ZTEST(ptp_queue, test_sender_coverage_cannot_bridge_failed_admission) {
    struct item old = {
        .header = {.ticket = 1, .state = 1, .motion = true, .first_sequence = 10, .sequence = 10},
        .position = 10};
    struct item next = {
        .header = {.ticket = 2, .state = 1, .motion = true, .first_sequence = 12, .sequence = 12},
        .position = 12};
    struct item scratch;
    zassert_ok(zmk_ptp_queue_put(&queue, &old, &scratch));
    /* Observation 11 could have been a failed lift; never mark it covered. */
    zassert_ok(zmk_ptp_queue_put(&queue, &next, &scratch));
    zassert_equal(k_msgq_num_used_get(&queue), 2);
    zassert_ok(k_msgq_get(&queue, &scratch, K_NO_WAIT));
    zassert_equal(scratch.header.first_sequence, 10);
    zassert_ok(k_msgq_get(&queue, &scratch, K_NO_WAIT));
    zassert_equal(scratch.header.first_sequence, 12);
    old.header.sequence = 0xffff;
    old.header.first_sequence = 0xfffe;
    next.header.sequence = 0;
    next.header.first_sequence = 0;
    zassert_ok(zmk_ptp_queue_put(&queue, &old, &scratch));
    zassert_equal(zmk_ptp_queue_put(&queue, &next, &scratch), 1);
    zassert_ok(k_msgq_get(&queue, &scratch, K_NO_WAIT));
    zassert_equal(scratch.header.first_sequence, 0xfffe);
}

ZTEST(ptp_queue, test_hid_and_source_state_ignore_coordinates_not_lifts) {
    struct zmk_ptp_frame f = {
        .contact_count = 1, .contacts = {{.id = 3, .confidence = true}}, .buttons = 5};
    struct zmk_ptp_report r = {
        .count_buttons = 1 | (5 << 4),
        .contacts = {{.flags_id = (3 << 2) | ZMK_PTP_TIP | ZMK_PTP_CONFIDENCE}}};
    bool lift;
    uint16_t state = zmk_ptp_report_state(&r, &lift);
    zassert_false(lift);
    zassert_equal(state, zmk_ptp_frame_state(&f));
    r.contacts[0].x = 100;
    r.scan_time = 200;
    zassert_equal(state, zmk_ptp_report_state(&r, &lift));
    r.contacts[0].flags_id &= ~ZMK_PTP_CONFIDENCE;
    zassert_not_equal(state, zmk_ptp_report_state(&r, &lift));
    r.contacts[0].flags_id &= ~ZMK_PTP_TIP;
    zmk_ptp_report_state(&r, &lift);
    zassert_true(lift);
    f.contact_count = 2;
    f.contacts[1] = f.contacts[0];
    f.contacts[1].id = 1;
    uint16_t a = zmk_ptp_frame_state(&f);
    struct zmk_ptp_contact swap = f.contacts[0];
    f.contacts[0] = f.contacts[1];
    f.contacts[1] = swap;
    zassert_equal(a, zmk_ptp_frame_state(&f)); /* Order does not create a fake transition. */
}

ZTEST(ptp_queue, test_compact_lengths_and_sequence_coverage_wrap) {
    struct zmk_ptp_split_frame wire, decoded;
    struct zmk_ptp_frame f = {0}, received;
    uint16_t seq;
    for (int n = 0; n <= 5; n++) {
        f.contact_count = n;
        for (int i = 0; i < n; i++) {
            f.contacts[i] = (struct zmk_ptp_contact){.id = i, .confidence = true};
        }
        zmk_ptp_split_encode(&wire, 1, &f);
        wire.first_sequence = sys_cpu_to_le16(0xfffc);
        size_t size = 7 + 5 * n;
        zassert_equal(zmk_ptp_split_size(&wire), size);
        zassert_ok(zmk_ptp_split_unpack(&decoded, &wire, size));
        uint8_t packet[sizeof(wire) + 1] __aligned(2);
        memcpy(packet + 1, &wire, size); /* Received ATT values need not be aligned. */
        zassert_ok(zmk_ptp_split_unpack(&decoded, packet + 1, size));
        zassert_mem_equal(&decoded, &wire, sizeof(wire));
        zassert_ok(zmk_ptp_split_decode(&decoded, &seq, &received));
        zassert_equal(received.contact_count, n);
        zassert_equal(zmk_ptp_split_unpack(&decoded, &wire, size - 1), -EINVAL);
        if (n < 5) {
            zassert_equal(zmk_ptp_split_unpack(&decoded, &wire, size + 1), -EINVAL);
        }
    }
    zassert_equal(zmk_ptp_split_unpack(&decoded, NULL, 7), -EINVAL);
    zassert_equal(zmk_ptp_split_unpack(&decoded, &wire, 33), -EINVAL);
    wire.first_sequence = sys_cpu_to_le16(2); /* Coverage cannot begin in the future. */
    zassert_equal(zmk_ptp_split_decode(&wire, &seq, &received), -EINVAL);
    wire.count_buttons = 0x80;
    zassert_equal(zmk_ptp_split_unpack(&decoded, &wire, 7), -EINVAL);
    wire.count_buttons = 6;
    zassert_equal(zmk_ptp_split_unpack(&decoded, &wire, 32), -EINVAL);
}

ZTEST(ptp_queue, test_motion_pace_and_transition_bypass) {
    int64_t last = 100;
    zassert_equal(zmk_ptp_pace_wait(false, true, last, last, 7500), 0);
    zassert_equal(zmk_ptp_pace_wait(true, false, last, last, 7500), 0);
#if IS_ENABLED(CONFIG_ZMK_BLE_PTP_PACING)
    k_ticks_t period = k_us_to_ticks_ceil64(7500 + CONFIG_ZMK_BLE_PTP_PACE_MARGIN_US);
    zassert_equal(zmk_ptp_pace_wait(true, true, last, last, 7500), period);
    zassert_equal(zmk_ptp_pace_wait(true, true, last, last + period - 1, 7500), 1);
    zassert_equal(zmk_ptp_pace_wait(true, true, last, last + period, 7500), 0);
    zassert_equal(zmk_ptp_pace_wait(true, true, last, last + period + 1, 7500), 0);
    zassert_equal(zmk_ptp_pace_wait(true, true, last, last + period + 100000, 7500), 0);
    zassert_equal(zmk_ptp_pace_wait(true, true, last, last, 15000),
                  k_us_to_ticks_ceil64(15000 + CONFIG_ZMK_BLE_PTP_PACE_MARGIN_US));
    zassert_equal(zmk_ptp_pace_wait(true, true, last, last, 0),
                  k_us_to_ticks_ceil64(CONFIG_ZMK_BLE_PTP_PACE_FALLBACK_US));
#else
    zassert_equal(zmk_ptp_pace_wait(true, true, last, last, 7500), 0);
#endif
}

ZTEST(ptp_queue, test_synthetic_strokes_and_lifetime_boundaries) {
    for (uint32_t step = 0; step < 900; step++) {
        struct zmk_ptp_frame f = zmk_ptp_test_frame(step);
        zassert_ok(zmk_ptp_validate_frame(&f));
        zassert_equal(f.scan_time, (uint16_t)(step * 100));
        uint32_t phase = step % 450;
        if (phase < 100) {
            zassert_equal(f.contact_count, 1);
            zassert_equal(f.contacts[0].y, CONFIG_ZMK_TRACKPAD_LOGICAL_Y / 2);
        } else if (phase >= 150 && phase < 250) {
            zassert_equal(f.contact_count, 1);
            zassert_equal(f.contacts[0].x, CONFIG_ZMK_TRACKPAD_LOGICAL_X / 2);
        } else if (phase >= 300 && phase < 400) {
            zassert_equal(f.contact_count, 2);
            zassert_equal(f.contacts[0].x + f.contacts[1].x, CONFIG_ZMK_TRACKPAD_LOGICAL_X);
            zassert_equal(f.contacts[0].y + f.contacts[1].y, CONFIG_ZMK_TRACKPAD_LOGICAL_Y);
        } else {
            zassert_equal(f.contact_count, 0);
        }
    }
}

ZTEST_SUITE(ptp_queue, NULL, NULL, before, NULL, NULL);
