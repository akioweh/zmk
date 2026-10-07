/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */
#include <errno.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zmk/ptp/frame.h>
#include <zmk/ptp/transport.h>
#include <zmk/activity.h>

uint16_t zmk_ptp_scan_time(void) {
    return (uint16_t)(k_ticks_to_us_floor64(k_uptime_ticks()) / 100);
}

int zmk_ptp_validate_frame(const struct zmk_ptp_frame *frame) {
    if (k_is_in_isr()) {
        return -EWOULDBLOCK;
    }
    if (!frame || frame->contact_count > CONFIG_ZMK_TRACKPAD_FINGERS || (frame->buttons & ~7)) {
        return -EINVAL;
    }
    uint8_t seen = 0;
    for (int i = 0; i < frame->contact_count; i++) {
        const struct zmk_ptp_contact *contact = &frame->contacts[i];
        if (contact->id >= CONFIG_ZMK_TRACKPAD_FINGERS || (seen & BIT(contact->id)) ||
            contact->x > CONFIG_ZMK_TRACKPAD_LOGICAL_X ||
            contact->y > CONFIG_ZMK_TRACKPAD_LOGICAL_Y) {
            return -EINVAL;
        }
        seen |= BIT(contact->id);
    }
    return 0;
}

void zmk_ptp_note_activity(const struct zmk_ptp_frame *frame) {
    bool intentional = frame->buttons != 0;
    for (int i = 0; i < frame->contact_count; i++) {
        intentional |= frame->contacts[i].confidence;
    }
    if (intentional) {
        zmk_activity_note();
    }
}

uint16_t zmk_ptp_frame_state(const struct zmk_ptp_frame *frame) {
    uint16_t state = frame->buttons << 10;
    for (int i = 0; i < frame->contact_count; i++) {
        state |= BIT(frame->contacts[i].id);
        if (frame->contacts[i].confidence) {
            state |= BIT(frame->contacts[i].id + 5);
        }
    }
    return state;
}

uint16_t zmk_ptp_report_state(const struct zmk_ptp_report *report, bool *lift) {
    uint16_t state = (report->count_buttons >> 4) << 10;
    *lift = false;
    for (int i = 0; i < (report->count_buttons & 0x0f); i++) {
        uint8_t flags = report->contacts[i].flags_id;
        if (!(flags & ZMK_PTP_TIP)) {
            *lift = true;
            continue;
        }
        uint8_t id = flags >> 2;
        state |= BIT(id);
        if (flags & ZMK_PTP_CONFIDENCE) {
            state |= BIT(id + 5);
        }
    }
    return state;
}

size_t zmk_ptp_split_size(const struct zmk_ptp_split_frame *wire) {
    return ZMK_PTP_SPLIT_HEADER_SIZE + 5 * (wire->data[6] & 7);
}

int zmk_ptp_split_unpack(struct zmk_ptp_split_frame *wire, const void *data, size_t length) {
    if (!data || length < ZMK_PTP_SPLIT_HEADER_SIZE || length > sizeof(*wire)) {
        return -EINVAL;
    }
    const uint8_t *bytes = data;
    size_t expected = ZMK_PTP_SPLIT_HEADER_SIZE + 5 * (bytes[6] & 7);
    if ((bytes[6] & 0xc0) || (bytes[6] & 7) > CONFIG_ZMK_TRACKPAD_FINGERS || length != expected) {
        return -EINVAL;
    }
    memset(wire, 0, sizeof(*wire));
    memcpy(wire, data, length);
    return 0;
}

void zmk_ptp_split_encode(struct zmk_ptp_split_frame *wire, uint16_t sequence,
                          const struct zmk_ptp_frame *frame) {
    memset(wire, 0, sizeof(*wire));
    sys_put_le16(sequence, wire->data);
    sys_put_le16(frame->scan_time, &wire->data[2]);
    sys_put_le16(sequence, &wire->data[4]); /* No skipped observations yet. */
    wire->data[6] = frame->contact_count | (frame->buttons << 3);
    for (int i = 0; i < frame->contact_count; i++) {
        uint8_t *p = &wire->data[7 + i * 5];
        p[0] = (frame->contacts[i].id << 1) | frame->contacts[i].confidence;
        sys_put_le16(frame->contacts[i].x, p + 1);
        sys_put_le16(frame->contacts[i].y, p + 3);
    }
}

int zmk_ptp_split_decode(const struct zmk_ptp_split_frame *wire, uint16_t *sequence,
                         struct zmk_ptp_frame *frame) {
    if ((wire->data[6] & 0xc0) || (wire->data[6] & 7) > CONFIG_ZMK_TRACKPAD_FINGERS ||
        (uint16_t)(sys_get_le16(wire->data) - sys_get_le16(wire->data + 4)) > INT16_MAX) {
        return -EINVAL;
    }
    *sequence = sys_get_le16(wire->data);
    *frame = (struct zmk_ptp_frame){
        .scan_time = sys_get_le16(&wire->data[2]),
        .contact_count = wire->data[6] & 7,
        .buttons = (wire->data[6] >> 3) & 7,
    };
    for (int i = 0; i < frame->contact_count; i++) {
        const uint8_t *p = &wire->data[7 + i * 5];
        frame->contacts[i] = (struct zmk_ptp_contact){
            .id = p[0] >> 1,
            .confidence = p[0] & 1,
            .x = sys_get_le16(p + 1),
            .y = sys_get_le16(p + 3),
        };
    }
    return zmk_ptp_validate_frame(frame);
}
