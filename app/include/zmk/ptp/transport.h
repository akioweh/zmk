/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <stddef.h>
#include <zephyr/toolchain.h>
#include <zephyr/sys/util.h>
#include <zmk/endpoints_types.h>
#include <zmk/ptp.h>

/* Report IDs retain Pete Johanson's PTP prototype's numbering. */
#define ZMK_PTP_REPORT_ID_INPUT 8
#define ZMK_PTP_REPORT_ID_CAPABILITIES 4
#define ZMK_PTP_REPORT_ID_CERTIFICATION 5
#define ZMK_PTP_REPORT_ID_MODE 6
#define ZMK_PTP_REPORT_ID_SELECTIVE 7

#define ZMK_PTP_CONFIDENCE 0x01
#define ZMK_PTP_TIP 0x02
#define ZMK_PTP_MODE_TOUCHPAD 3
#define ZMK_PTP_SURFACE 0x01
#define ZMK_PTP_BUTTONS 0x02

/* Wire layout shared by USB and HoG; coordinates/time are little endian. */
struct zmk_ptp_wire_contact {
    uint8_t flags_id;
    uint16_t x;
    uint16_t y;
} __packed;

struct zmk_ptp_report {
    uint8_t report_id;
    struct zmk_ptp_wire_contact contacts[CONFIG_ZMK_TRACKPAD_FINGERS];
    uint16_t scan_time;
    uint8_t count_buttons;
} __packed;

/* Stable contact/button state for queue coalescing; explicit lifts are barriers. */
static inline uint16_t zmk_ptp_report_state(const struct zmk_ptp_report *report, bool *lift) {
    uint16_t state = (report->count_buttons >> 4) << (2 * ZMK_PTP_MAX_CONTACTS);
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
            state |= BIT(id + ZMK_PTP_MAX_CONTACTS);
        }
    }
    return state;
}

/* Core lifecycle hooks and HID backend helpers, not the driver interface. */
void zmk_ptp_set_endpoint(struct zmk_endpoint_instance endpoint);
void zmk_ptp_reset_endpoint(struct zmk_endpoint_instance endpoint);
int zmk_ptp_set_suspended(struct zmk_endpoint_instance endpoint, bool suspended);
int zmk_ptp_get_report(struct zmk_endpoint_instance endpoint, struct zmk_ptp_report *report);
/* Feature data excludes the Report ID (USB adds it; HoG does not). */
int zmk_ptp_get_feature(struct zmk_endpoint_instance endpoint, uint8_t id, uint8_t *data,
                        size_t size);
int zmk_ptp_set_feature(struct zmk_endpoint_instance endpoint, uint8_t id, const uint8_t *data,
                        size_t size);

#if IS_ENABLED(CONFIG_ZMK_USB)
int zmk_ptp_usb_send_report(const struct zmk_ptp_report *report);
void zmk_ptp_usb_reset(void);
#endif
#if IS_ENABLED(CONFIG_ZMK_BLE)
int zmk_ptp_hog_send_report(struct zmk_endpoint_instance endpoint,
                            const struct zmk_ptp_report *report);
#endif
