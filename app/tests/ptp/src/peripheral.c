/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */
#include <errno.h>
#include <zephyr/irq_offload.h>
#include <zephyr/ztest.h>
#include <zmk/ptp/split.h>
#include <zmk/split/peripheral.h>

static struct zmk_split_transport_peripheral_event events[64];
static int count, error;

int zmk_activity_note(void) { return 0; }

int zmk_split_peripheral_report_event(const struct zmk_split_transport_peripheral_event *event) {
    if (error) {
        return error;
    }
    zassert_true(count < ARRAY_SIZE(events));
    events[count++] = *event;
    return 0;
}

static struct zmk_ptp_frame one(void) {
    return (struct zmk_ptp_frame){.scan_time = 0xffff,
                                  .contact_count = 1,
                                  .contacts = {{.id = 4, .x = 123, .y = 456, .confidence = true}},
                                  .buttons = 7};
}

static struct zmk_ptp_frame last(uint16_t *sequence) {
    struct zmk_ptp_frame frame;
    zassert_true(count > 0);
    zassert_equal(events[count - 1].type, ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_CONTACT_FRAME);
    zassert_ok(zmk_ptp_split_decode(&events[count - 1].data.contact_frame, sequence, &frame));
    return frame;
}

static void before(void *fixture) {
    error = 0;
    zassert_ok(zmk_ptp_release());
    k_sleep(K_MSEC(160)); /* Drain the idle retries. */
    count = 0;
}

ZTEST(ptp_peripheral, test_uniform_api_one_atomic_event) {
    struct zmk_ptp_frame frame = one();
    frame.contact_count = 5;
    for (int i = 0; i < 5; i++) {
        frame.contacts[i] =
            (struct zmk_ptp_contact){.id = i, .x = 123 + i, .y = 456, .confidence = i != 3};
    }
    zassert_ok(zmk_ptp_submit_frame(&frame));
    zassert_equal(count, 1); /* Not twenty scalar input events. */
    uint16_t sequence;
    struct zmk_ptp_frame received = last(&sequence);
    zassert_equal(received.contact_count, 5);
    zassert_equal(received.scan_time, 0xffff);
    zassert_equal(received.buttons, 7);
    zassert_equal(received.contacts[4].x, 127);
    zassert_false(received.contacts[3].confidence);
    frame.contacts[4].x = 999; /* Submission copied the snapshot. */
    k_sleep(K_MSEC(70));
    uint16_t heartbeat_sequence;
    received = last(&heartbeat_sequence);
    zassert_equal(heartbeat_sequence, sequence);
    zassert_equal(received.contacts[4].x, 127);
}

ZTEST(ptp_peripheral, test_offline_lift_replaces_cached_hold) {
    struct zmk_ptp_frame frame = one();
    zassert_ok(zmk_ptp_submit_frame(&frame));
    uint16_t sequence;
    last(&sequence);
    error = -ENODEV;
    zassert_equal(zmk_ptp_release(), -ENODEV);
    k_sleep(K_MSEC(70));
    error = 0;
    zmk_ptp_split_resume();
    k_sleep(K_MSEC(20));
    uint16_t resumed_sequence;
    struct zmk_ptp_frame resumed = last(&resumed_sequence);
    zassert_equal(resumed_sequence, (uint16_t)(sequence + 1));
    zassert_equal(resumed.contact_count, 0);
    zassert_equal(resumed.buttons, 0);
    k_sleep(K_MSEC(160));
    int idle_count = count;
    k_sleep(K_MSEC(160));
    zassert_equal(count, idle_count); /* No permanent idle radio traffic. */
}

ZTEST(ptp_peripheral, test_failed_admission_and_validation) {
    struct zmk_ptp_frame frame = one();
    error = -ENOSPC;
    zassert_equal(zmk_ptp_submit_frame(&frame), -ENOSPC);
    error = 0;
    k_sleep(K_MSEC(70)); /* Latest physical snapshot is retried. */
    uint16_t sequence;
    zassert_equal(last(&sequence).contacts[0].x, 123);
    int accepted = count;
    frame.contact_count = 6;
    zassert_equal(zmk_ptp_submit_frame(&frame), -EINVAL);
    zassert_equal(zmk_ptp_submit_frame(NULL), -EINVAL);
    zassert_equal(count, accepted);
}

static void from_isr(const void *unused) {
    struct zmk_ptp_frame frame = one();
    zassert_equal(zmk_ptp_submit_frame(&frame), -EWOULDBLOCK);
    zassert_equal(zmk_ptp_release(), -EWOULDBLOCK);
}

ZTEST(ptp_peripheral, test_isr_rejected) {
    irq_offload(from_isr, NULL);
    zassert_equal(count, 0);
}

ZTEST_SUITE(ptp_peripheral, NULL, NULL, before, NULL, NULL);
