/*
 * Copyright (c) 2020 The ZMK Contributors
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 *
 * Adapted from Pete Johanson's PTP USB backend.
 */

#include <errno.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zmk/usb.h>
#include "hid.h"

static const struct device *hid_dev;
static K_SEM_DEFINE(hid_ready, 1, 1);
static uint8_t control_report[257];
static const struct zmk_endpoint_instance usb_endpoint = {.transport = ZMK_TRANSPORT_USB};

BUILD_ASSERT(CONFIG_USB_HID_DEVICE_COUNT >= 2, "PTP needs a second USB HID interface");
BUILD_ASSERT(CONFIG_HID_INTERRUPT_EP_MPS >= sizeof(struct zmk_ptp_report),
             "PTP input report must fit one USB interrupt packet");

static void in_ready(const struct device *dev) { k_sem_give(&hid_ready); }

void zmk_ptp_usb_reset(void) {
    /* A reset may abort an IN transfer without its completion callback. */
    k_sem_give(&hid_ready);
    zmk_ptp_reset_endpoint(usb_endpoint);
}

static int get_report(const struct device *dev, struct usb_setup_packet *setup, int32_t *len,
                      uint8_t **data) {
    uint8_t id = setup->wValue & 0xff;
    uint8_t type = setup->wValue >> 8;
    int size;
    if (type == 1 && id == ZMK_PTP_REPORT_ID_INPUT) {
        struct zmk_ptp_report report;
        int err = zmk_ptp_get_report(usb_endpoint, &report);
        if (err) {
            return err;
        }
        memcpy(control_report, &report, sizeof(report));
        size = sizeof(report);
    } else if (type == 3) {
        control_report[0] = id;
        size =
            zmk_ptp_get_feature(usb_endpoint, id, control_report + 1, sizeof(control_report) - 1);
        if (size < 0) {
            return size;
        }
        size++;
    } else {
        return -ENOTSUP;
    }
    *data = control_report;
    *len = size;
    return 0;
}

static int set_report(const struct device *dev, struct usb_setup_packet *setup, int32_t *len,
                      uint8_t **data) {
    uint8_t id = setup->wValue & 0xff;
    if ((setup->wValue >> 8) != 3) {
        return -ENOTSUP;
    }
    if (*len != 2 || !data || !*data || (*data)[0] != id) {
        return -EINVAL;
    }
    return zmk_ptp_set_feature(usb_endpoint, id, *data + 1, *len - 1);
}

static const struct hid_ops ops = {
    .int_in_ready = in_ready,
    .get_report = get_report,
    .set_report = set_report,
};

int zmk_ptp_usb_send_report(const struct zmk_ptp_report *report) {
    if (!hid_dev || !device_is_ready(hid_dev) || !zmk_usb_is_hid_ready()) {
        return -ENODEV;
    }
    if (zmk_usb_get_status() == USB_DC_SUSPEND) {
        /* A cleanup/release must not wake an old, inactive host. */
        bool activity = (report->count_buttons & 0x70) != 0;
        for (int i = 0; i < (report->count_buttons & 0x0f); i++) {
            activity |= (report->contacts[i].flags_id & ZMK_PTP_TIP) != 0;
        }
        if (activity) {
            usb_wakeup_request();
        }
        return -EAGAIN;
    }
    if (k_sem_take(&hid_ready, K_MSEC(30))) {
        return -EAGAIN;
    }
    uint32_t written = 0;
    int err = hid_int_ep_write(hid_dev, (const uint8_t *)report, sizeof(*report), &written);
    if (err || written != sizeof(*report)) {
        k_sem_give(&hid_ready);
        return err ? err : -EIO;
    }
    return 0;
}

static int ptp_usb_init(void) {
    hid_dev = device_get_binding("HID_1");
    if (!hid_dev || !device_is_ready(hid_dev)) {
        return -ENODEV;
    }
    usb_hid_register_device(hid_dev, zmk_ptp_hid_report_desc, sizeof(zmk_ptp_hid_report_desc),
                            &ops);
#if IS_ENABLED(CONFIG_ZMK_USB_BOOT)
    usb_hid_set_proto_code(hid_dev, HID_BOOT_IFACE_CODE_NONE);
#endif
    return usb_hid_init(hid_dev);
}

SYS_INIT(ptp_usb_init, APPLICATION, CONFIG_ZMK_USB_HID_INIT_PRIORITY);
