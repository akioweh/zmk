/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <errno.h>
#include <string.h>
#include <zephyr/irq_offload.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/ztest.h>
#include <zmk/endpoints.h>
#include "hid.h"

static const struct zmk_endpoint_instance usb = {.transport = ZMK_TRANSPORT_USB};
static const struct zmk_endpoint_instance ble0 = {.transport = ZMK_TRANSPORT_BLE,
                                                  .ble.profile_index = 0};
static const struct zmk_endpoint_instance ble1 = {.transport = ZMK_TRANSPORT_BLE,
                                                  .ble.profile_index = 1};
static struct {
    struct zmk_endpoint_instance endpoint;
    struct zmk_ptp_report report;
} sent[32];
static int sent_count;
static int usb_error;
static int ble_error;

bool zmk_endpoint_instance_eq(struct zmk_endpoint_instance a, struct zmk_endpoint_instance b) {
    return a.transport == b.transport &&
           (a.transport != ZMK_TRANSPORT_BLE || a.ble.profile_index == b.ble.profile_index);
}

int zmk_endpoint_instance_to_index(struct zmk_endpoint_instance endpoint) {
    return endpoint.transport == ZMK_TRANSPORT_BLE ? 2 + endpoint.ble.profile_index
                                                   : endpoint.transport;
}

static int record(struct zmk_endpoint_instance endpoint, const struct zmk_ptp_report *report,
                  int error) {
    if (error) {
        return error;
    }
    zassert_true(sent_count < ARRAY_SIZE(sent));
    sent[sent_count].endpoint = endpoint;
    sent[sent_count++].report = *report;
    return 0;
}

int zmk_ptp_usb_send_report(const struct zmk_ptp_report *report) {
    return record(usb, report, usb_error);
}

int zmk_ptp_hog_send_report(struct zmk_endpoint_instance endpoint,
                            const struct zmk_ptp_report *report) {
    return record(endpoint, report, ble_error);
}

static struct zmk_ptp_frame one(uint8_t id, uint16_t x, uint16_t y) {
    return (struct zmk_ptp_frame){.scan_time = 0xbeef,
                                  .contact_count = 1,
                                  .contacts = {{.id = id, .x = x, .y = y, .confidence = true}}};
}

static struct zmk_ptp_report last(void) {
    zassert_true(sent_count > 0);
    return sent[sent_count - 1].report;
}

static void before(void *fixture) {
    usb_error = ble_error = 0;
    zmk_ptp_set_endpoint((struct zmk_endpoint_instance){0});
    zmk_ptp_reset_endpoint(usb);
    for (int i = 0; i < ZMK_BLE_PROFILE_COUNT; i++) {
        zmk_ptp_reset_endpoint(
            (struct zmk_endpoint_instance){.transport = ZMK_TRANSPORT_BLE, .ble.profile_index = i});
    }
    /* Deferred reset runs on the real Zephyr system workqueue. */
    k_sleep(K_MSEC(20));
    sent_count = 0;
    zmk_ptp_set_endpoint(usb);
}

ZTEST(ptp, test_encoding_and_lifts) {
    struct zmk_ptp_frame frame = one(0, 0x123, 0x456);
    frame.contact_count = 2;
    frame.contacts[1] = (struct zmk_ptp_contact){.id = 3, .x = 12, .y = 34, .confidence = true};
    frame.buttons = 5;
    zassert_ok(zmk_ptp_submit_frame(&frame));
    struct zmk_ptp_report report = last();
    zassert_equal(sizeof(report), 29);
    zassert_equal(report.report_id, ZMK_PTP_REPORT_ID_INPUT);
    zassert_equal(report.count_buttons, 0x52);
    zassert_equal(report.contacts[0].flags_id, 3);
    zassert_equal(report.contacts[1].flags_id, (3 << 2) | 3);
    const uint8_t *bytes = (const uint8_t *)&report;
    zassert_equal(bytes[2], 0x23);
    zassert_equal(bytes[3], 0x01);
    zassert_equal(bytes[sizeof(report) - 3], 0xef);
    zassert_equal(bytes[sizeof(report) - 2], 0xbe);

    frame = one(3, 50, 60);
    zassert_ok(zmk_ptp_submit_frame(&frame));
    report = last();
    zassert_equal(report.count_buttons, 2); /* One lift plus one active contact. */
    zassert_equal(report.contacts[0].flags_id, ZMK_PTP_CONFIDENCE);
    zassert_equal(sys_le16_to_cpu(report.contacts[0].x), 0x123);
    zassert_equal(sys_le16_to_cpu(report.contacts[0].y), 0x456);
    zassert_equal(report.contacts[1].flags_id, (3 << 2) | 3);
    zassert_ok(zmk_ptp_submit_frame(&frame));
    zassert_equal(last().count_buttons, 1); /* Lift is not repeated. */
    zassert_ok(zmk_ptp_release());
    report = last();
    zassert_equal(report.count_buttons, 1);
    zassert_equal(report.contacts[0].flags_id, (3 << 2) | ZMK_PTP_CONFIDENCE);
    zassert_equal(sys_le16_to_cpu(report.contacts[0].x), 50);
    zassert_ok(zmk_ptp_release());
}

ZTEST(ptp, test_validation_and_time_wrap) {
    struct zmk_ptp_frame frame = one(0, 10, 20);
    zassert_equal(zmk_ptp_submit_frame(NULL), -EINVAL);
    frame.contact_count = CONFIG_ZMK_TRACKPAD_FINGERS + 1;
    zassert_equal(zmk_ptp_submit_frame(&frame), -EINVAL);
    frame.contact_count = 2;
    frame.contacts[1] = frame.contacts[0];
    zassert_equal(zmk_ptp_submit_frame(&frame), -EINVAL);
    frame = one(CONFIG_ZMK_TRACKPAD_FINGERS, 10, 20);
    zassert_equal(zmk_ptp_submit_frame(&frame), -EINVAL);
    frame = one(0, CONFIG_ZMK_TRACKPAD_LOGICAL_X + 1, 20);
    zassert_equal(zmk_ptp_submit_frame(&frame), -EINVAL);
    frame = one(0, 10, CONFIG_ZMK_TRACKPAD_LOGICAL_Y + 1);
    zassert_equal(zmk_ptp_submit_frame(&frame), -EINVAL);
    frame = one(0, 10, 20);
    frame.buttons = 8;
    zassert_equal(zmk_ptp_submit_frame(&frame), -EINVAL);
    zassert_equal(sent_count, 0);
    frame.buttons = 0;
    frame.scan_time = UINT16_MAX;
    zassert_ok(zmk_ptp_submit_frame(&frame));
    frame.scan_time = 0;
    zassert_ok(zmk_ptp_submit_frame(&frame));
    zassert_equal(last().scan_time, 0);
    zmk_ptp_set_endpoint((struct zmk_endpoint_instance){0});
    zassert_equal(zmk_ptp_submit_frame(&frame), -ENODEV);
}

ZTEST(ptp, test_all_five_contacts_and_lifts_fit) {
    struct zmk_ptp_frame frame = {.contact_count = CONFIG_ZMK_TRACKPAD_FINGERS};
    for (int i = 0; i < CONFIG_ZMK_TRACKPAD_FINGERS; i++) {
        frame.contacts[i] = (struct zmk_ptp_contact){.id = i, .x = i * 100, .confidence = true};
    }
    zassert_ok(zmk_ptp_submit_frame(&frame));
    zassert_equal(last().count_buttons, CONFIG_ZMK_TRACKPAD_FINGERS);
    frame.contact_count = 0;
    zassert_ok(zmk_ptp_submit_frame(&frame));
    struct zmk_ptp_report report = last();
    zassert_equal(report.count_buttons, CONFIG_ZMK_TRACKPAD_FINGERS);
    for (int i = 0; i < CONFIG_ZMK_TRACKPAD_FINGERS; i++) {
        zassert_equal(report.contacts[i].flags_id, (i << 2) | ZMK_PTP_CONFIDENCE);
        zassert_equal(sys_le16_to_cpu(report.contacts[i].x), i * 100);
    }
}

ZTEST(ptp, test_confidence_is_sticky_until_lift) {
    struct zmk_ptp_frame frame = one(0, 10, 20);
    frame.contacts[0].confidence = false;
    zassert_ok(zmk_ptp_submit_frame(&frame));
    frame.contacts[0].confidence = true;
    zassert_ok(zmk_ptp_submit_frame(&frame));
    zassert_equal(last().contacts[0].flags_id, ZMK_PTP_TIP);
    zassert_ok(zmk_ptp_release());
    zassert_ok(zmk_ptp_submit_frame(&frame));
    zassert_equal(last().contacts[0].flags_id, ZMK_PTP_TIP | ZMK_PTP_CONFIDENCE);
}

ZTEST(ptp, test_failed_frame_does_not_advance_state) {
    struct zmk_ptp_frame frame = one(0, 10, 20);
    zassert_ok(zmk_ptp_submit_frame(&frame));
    usb_error = -EAGAIN;
    frame.contacts[0].x = 100;
    zassert_equal(zmk_ptp_submit_frame(&frame), -EAGAIN);
    zassert_equal(sent_count, 1);
    usb_error = 0;
    zassert_ok(zmk_ptp_release());
    zassert_equal(sys_le16_to_cpu(last().contacts[0].x), 10);
}

ZTEST(ptp, test_endpoint_change_releases_original_destination) {
    struct zmk_ptp_frame frame = one(0, 10, 20);
    zassert_ok(zmk_ptp_submit_frame(&frame));
    zmk_ptp_set_endpoint(ble1);
    zassert_equal(sent_count, 2);
    zassert_true(zmk_endpoint_instance_eq(sent[1].endpoint, usb));
    zassert_equal(sent[1].report.contacts[0].flags_id, ZMK_PTP_CONFIDENCE);
    frame = one(2, 30, 40);
    zassert_ok(zmk_ptp_submit_frame(&frame));
    zassert_true(zmk_endpoint_instance_eq(sent[2].endpoint, ble1));
    zmk_ptp_set_endpoint(ble0);
    zassert_true(zmk_endpoint_instance_eq(sent[3].endpoint, ble1));
    zassert_equal(sent[3].report.contacts[0].flags_id, (2 << 2) | ZMK_PTP_CONFIDENCE);
}

ZTEST(ptp, test_failed_old_release_does_not_block_new_host) {
    struct zmk_ptp_frame frame = one(0, 10, 20);
    zassert_ok(zmk_ptp_submit_frame(&frame));
    usb_error = -EAGAIN;
    zmk_ptp_set_endpoint(ble1);
    zassert_ok(zmk_ptp_submit_frame(&frame));
    zassert_true(zmk_endpoint_instance_eq(sent[1].endpoint, ble1));
    usb_error = 0;
    k_sleep(K_MSEC(60));
    zassert_true(zmk_endpoint_instance_eq(sent[sent_count - 1].endpoint, usb));
    zassert_equal(last().contacts[0].flags_id, ZMK_PTP_CONFIDENCE);
}

ZTEST(ptp, test_features_and_per_host_selective_reporting) {
    uint8_t data[256];
    zassert_equal(zmk_ptp_get_feature(usb, ZMK_PTP_REPORT_ID_CAPABILITIES, data, sizeof(data)), 1);
    zassert_equal(data[0], CONFIG_ZMK_TRACKPAD_FINGERS | (CONFIG_ZMK_TRACKPAD_PAD_TYPE << 4));
    zassert_equal(zmk_ptp_get_feature(usb, ZMK_PTP_REPORT_ID_CERTIFICATION, data, sizeof(data)),
                  256);
    for (int i = 0; i < sizeof(data); i++) {
        zassert_equal(data[i], 0);
    }
    zassert_equal(zmk_ptp_get_feature(usb, ZMK_PTP_REPORT_ID_CAPABILITIES, data, 0), -EMSGSIZE);
    zassert_equal(zmk_ptp_get_feature(usb, 0xff, data, sizeof(data)), -ENOTSUP);
    data[0] = 0;
    zassert_equal(zmk_ptp_set_feature(usb, ZMK_PTP_REPORT_ID_MODE, data, 1), -ENOTSUP);
    data[0] = ZMK_PTP_MODE_TOUCHPAD;
    zassert_ok(zmk_ptp_set_feature(usb, ZMK_PTP_REPORT_ID_MODE, data, 1));
    zassert_equal(zmk_ptp_set_feature(usb, ZMK_PTP_REPORT_ID_MODE, data, 2), -EINVAL);
    data[0] = 8;
    zassert_equal(zmk_ptp_set_feature(usb, ZMK_PTP_REPORT_ID_SELECTIVE, data, 1), -EINVAL);
    zassert_equal(zmk_ptp_set_feature(usb, ZMK_PTP_REPORT_ID_CAPABILITIES, data, 1), -ENOTSUP);

    struct zmk_ptp_frame frame = one(0, 10, 20);
    frame.buttons = 7;
    zassert_ok(zmk_ptp_submit_frame(&frame));
    data[0] = ZMK_PTP_BUTTONS;
    zassert_ok(zmk_ptp_set_feature(usb, ZMK_PTP_REPORT_ID_SELECTIVE, data, 1));
    zassert_equal(last().contacts[0].flags_id, ZMK_PTP_CONFIDENCE);
    zassert_ok(zmk_ptp_submit_frame(&frame));
    zassert_equal(last().count_buttons, 0x70);
    zassert_equal(zmk_ptp_get_feature(ble1, ZMK_PTP_REPORT_ID_SELECTIVE, data, 1), 1);
    zassert_equal(data[0], 3);
    data[0] = ZMK_PTP_SURFACE;
    zassert_ok(zmk_ptp_set_feature(usb, ZMK_PTP_REPORT_ID_SELECTIVE, data, 1));
    zassert_ok(zmk_ptp_submit_frame(&frame));
    zassert_equal(last().count_buttons, 1);
    data[0] = 0;
    zassert_ok(zmk_ptp_set_feature(usb, ZMK_PTP_REPORT_ID_SELECTIVE, data, 1));
    int before_count = sent_count;
    zassert_ok(zmk_ptp_submit_frame(&frame));
    zassert_equal(sent_count, before_count); /* Neither type of input is selected. */
}

ZTEST(ptp, test_suspend_resume_and_reset) {
    struct zmk_ptp_frame frame = one(0, 10, 20);
    zassert_ok(zmk_ptp_submit_frame(&frame));
    zassert_ok(zmk_ptp_set_suspended(usb, true));
    frame.contact_count = 0;
    zassert_equal(zmk_ptp_submit_frame(&frame), -EAGAIN);
    zassert_ok(zmk_ptp_set_suspended(usb, false));
    zassert_ok(zmk_ptp_submit_frame(&frame));
    zassert_equal(last().contacts[0].flags_id, ZMK_PTP_CONFIDENCE);
    uint8_t data = 0;
    zassert_ok(zmk_ptp_set_feature(usb, ZMK_PTP_REPORT_ID_SELECTIVE, &data, 1));
    zmk_ptp_reset_endpoint(usb);
    k_sleep(K_MSEC(20));
    zassert_equal(zmk_ptp_get_feature(usb, ZMK_PTP_REPORT_ID_SELECTIVE, &data, 1), 1);
    zassert_equal(data, 3);
    struct zmk_ptp_report report;
    zassert_ok(zmk_ptp_get_report(usb, &report));
    zassert_equal(report.count_buttons, 0);
    zassert_equal(report.report_id, ZMK_PTP_REPORT_ID_INPUT);
}

static int isr_result;
static void submit_in_isr(const void *unused) { isr_result = zmk_ptp_submit_frame(NULL); }

ZTEST(ptp, test_isr_calls_are_rejected) {
    irq_offload(submit_in_isr, NULL);
    zassert_equal(isr_result, -EWOULDBLOCK);
    zassert_equal(sent_count, 0);
}

ZTEST(ptp, test_descriptor_matches_wire_layout_and_units) {
    uint32_t size = 0, count = 0, id = 0, unit = 0;
    uint32_t input_bits[9] = {0}, feature_bits[9] = {0};
    int xy_fields = 0, scan_fields = 0;
    for (size_t pos = 0; pos < sizeof(zmk_ptp_hid_report_desc);) {
        uint8_t prefix = zmk_ptp_hid_report_desc[pos++];
        uint8_t length = prefix & 3;
        length = length == 3 ? 4 : length;
        zassert_true(pos + length <= sizeof(zmk_ptp_hid_report_desc));
        uint32_t value = 0;
        for (int i = 0; i < length; i++) {
            value |= (uint32_t)zmk_ptp_hid_report_desc[pos++] << (8 * i);
        }
        int type = (prefix >> 2) & 3;
        int tag = prefix >> 4;
        if (type == 1) {
            if (tag == 7) {
                size = value;
            }
            if (tag == 8) {
                id = value;
            }
            if (tag == 9) {
                count = value;
            }
            if (tag == 6) {
                unit = value;
            }
        } else if (type == 0 && (tag == 8 || tag == 11)) {
            zassert_true(id < ARRAY_SIZE(input_bits));
            if (tag == 8) {
                input_bits[id] += size * count;
                if (size == 16 && unit == 0x11) {
                    xy_fields++;
                } else if (size == 16 && unit == 0x1001) {
                    scan_fields++;
                } else {
                    zassert_equal(unit, 0);
                }
            } else {
                zassert_equal(unit, 0);
                feature_bits[id] += size * count;
            }
        }
    }
    zassert_equal(input_bits[ZMK_PTP_REPORT_ID_INPUT], (sizeof(struct zmk_ptp_report) - 1) * 8);
    zassert_equal(xy_fields, 2 * CONFIG_ZMK_TRACKPAD_FINGERS);
    zassert_equal(scan_fields, 1);
    zassert_equal(feature_bits[ZMK_PTP_REPORT_ID_CAPABILITIES], 8);
    zassert_equal(feature_bits[ZMK_PTP_REPORT_ID_CERTIFICATION], 256 * 8);
    zassert_equal(feature_bits[ZMK_PTP_REPORT_ID_MODE], 8);
    zassert_equal(feature_bits[ZMK_PTP_REPORT_ID_SELECTIVE], 8);
}

ZTEST_SUITE(ptp, NULL, NULL, before, NULL, NULL);
