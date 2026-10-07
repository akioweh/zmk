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
    uint16_t state = frame->buttons << (2 * ZMK_PTP_MAX_CONTACTS);
    for (int i = 0; i < frame->contact_count; i++) {
        state |= BIT(frame->contacts[i].id);
        if (frame->contacts[i].confidence) {
            state |= BIT(frame->contacts[i].id + ZMK_PTP_MAX_CONTACTS);
        }
    }
    return state;
}

size_t zmk_ptp_split_size(const struct zmk_ptp_split_frame *wire) {
    return ZMK_PTP_SPLIT_HEADER_SIZE + sizeof(wire->contacts[0]) * (wire->count_buttons & 7);
}

int zmk_ptp_split_unpack(struct zmk_ptp_split_frame *wire, const void *data, size_t length) {
    if (!data || length < ZMK_PTP_SPLIT_HEADER_SIZE || length > sizeof(*wire)) {
        return -EINVAL;
    }
    const struct zmk_ptp_split_frame *input = data;
    if ((input->count_buttons & 0xc0) || (input->count_buttons & 7) > CONFIG_ZMK_TRACKPAD_FINGERS ||
        length != zmk_ptp_split_size(input)) {
        return -EINVAL;
    }
    memset(wire, 0, sizeof(*wire));
    memcpy(wire, data, length);
    return 0;
}

void zmk_ptp_split_encode(struct zmk_ptp_split_frame *wire, uint16_t sequence,
                          const struct zmk_ptp_frame *frame) {
    memset(wire, 0, sizeof(*wire));
    wire->sequence = wire->first_sequence = sys_cpu_to_le16(sequence);
    wire->scan_time = sys_cpu_to_le16(frame->scan_time);
    wire->count_buttons = frame->contact_count | (frame->buttons << 3);
    for (int i = 0; i < frame->contact_count; i++) {
        const struct zmk_ptp_contact *contact = &frame->contacts[i];
        wire->contacts[i] = (struct zmk_ptp_split_contact){
            .id_confidence = (contact->id << 1) | contact->confidence,
            .x = sys_cpu_to_le16(contact->x),
            .y = sys_cpu_to_le16(contact->y),
        };
    }
}

int zmk_ptp_split_decode(const struct zmk_ptp_split_frame *wire, uint16_t *sequence,
                         struct zmk_ptp_frame *frame) {
    if ((wire->count_buttons & 0xc0) || (wire->count_buttons & 7) > CONFIG_ZMK_TRACKPAD_FINGERS ||
        (uint16_t)(sys_le16_to_cpu(wire->sequence) - sys_le16_to_cpu(wire->first_sequence)) >
            INT16_MAX) {
        return -EINVAL;
    }
    *sequence = sys_le16_to_cpu(wire->sequence);
    *frame = (struct zmk_ptp_frame){
        .scan_time = sys_le16_to_cpu(wire->scan_time),
        .contact_count = wire->count_buttons & 7,
        .buttons = (wire->count_buttons >> 3) & 7,
    };
    for (int i = 0; i < frame->contact_count; i++) {
        const struct zmk_ptp_split_contact *contact = &wire->contacts[i];
        frame->contacts[i] = (struct zmk_ptp_contact){
            .id = contact->id_confidence >> 1,
            .confidence = contact->id_confidence & 1,
            .x = sys_le16_to_cpu(contact->x),
            .y = sys_le16_to_cpu(contact->y),
        };
    }
    return zmk_ptp_validate_frame(frame);
}
