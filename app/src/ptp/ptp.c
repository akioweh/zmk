/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <errno.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/byteorder.h>
#include <zmk/endpoints.h>
#include <zmk/ptp/transport.h>
#include <zmk/ptp/frame.h>
#include "touch.h"
#if IS_ENABLED(CONFIG_ZMK_SPLIT)
#include <zmk/ptp/split.h>
#endif

struct ptp_host {
    struct zmk_endpoint_instance endpoint;
    struct zmk_ptp_contact contacts[CONFIG_ZMK_TRACKPAD_FINGERS];
    struct zmk_ptp_report report;
    uint8_t active;
    uint8_t selective;
    bool suspended;
    bool release_pending;
};

static struct ptp_host hosts[ZMK_ENDPOINT_COUNT] = {
    [0 ... ZMK_ENDPOINT_COUNT - 1] =
        {
            .report.report_id = ZMK_PTP_REPORT_ID_INPUT,
            .selective = ZMK_PTP_SURFACE | ZMK_PTP_BUTTONS,
        },
};
static struct zmk_endpoint_instance selected;
#if IS_ENABLED(CONFIG_ZMK_SPLIT)
static atomic_t selected_index = -1;
#endif
static K_MUTEX_DEFINE(ptp_lock);
ATOMIC_DEFINE(reset_pending, ZMK_ENDPOINT_COUNT);

static void recovery_work(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(recovery, recovery_work);

BUILD_ASSERT(sizeof(struct zmk_ptp_report) == 5 * CONFIG_ZMK_TRACKPAD_FINGERS + 4);

static int host_index(struct zmk_endpoint_instance endpoint) {
    switch (endpoint.transport) {
#if IS_ENABLED(CONFIG_ZMK_USB)
    case ZMK_TRANSPORT_USB:
        return zmk_endpoint_instance_to_index(endpoint);
#endif
#if IS_ENABLED(CONFIG_ZMK_BLE)
    case ZMK_TRANSPORT_BLE:
        if (endpoint.ble.profile_index >= 0 && endpoint.ble.profile_index < ZMK_BLE_PROFILE_COUNT) {
            return zmk_endpoint_instance_to_index(endpoint);
        }
        break;
#endif
    default:
        break;
    }
    return -ENODEV;
}

static int send_report(struct ptp_host *host, const struct zmk_ptp_report *report) {
    if (host->suspended) {
        return -EAGAIN;
    }
    switch (host->endpoint.transport) {
#if IS_ENABLED(CONFIG_ZMK_USB)
    case ZMK_TRANSPORT_USB:
        return zmk_ptp_usb_send_report(report);
#endif
#if IS_ENABLED(CONFIG_ZMK_BLE)
    case ZMK_TRANSPORT_BLE:
        return zmk_ptp_hog_send_report(host->endpoint, report);
#endif
    default:
        return -ENODEV;
    }
}

static struct zmk_ptp_wire_contact encode_contact(struct zmk_ptp_contact contact, bool tip) {
    return (struct zmk_ptp_wire_contact){
        .flags_id = (contact.id << 2) | (contact.confidence ? ZMK_PTP_CONFIDENCE : 0) |
                    (tip ? ZMK_PTP_TIP : 0),
        .x = sys_cpu_to_le16(contact.x),
        .y = sys_cpu_to_le16(contact.y),
    };
}

/* Called under ptp_lock. No accepted state is changed on a transport error. */
static int submit(struct ptp_host *host, const struct zmk_ptp_frame *frame) {
    if (!host->selective && !host->active && !(host->report.count_buttons & 0x70)) {
        return 0; /* Both selective-reporting switches are off. */
    }
    struct zmk_ptp_contact contacts[CONFIG_ZMK_TRACKPAD_FINGERS] = {0};
    uint8_t active = 0;
    if (host->selective & ZMK_PTP_SURFACE) {
        for (int i = 0; i < frame->contact_count; i++) {
            struct zmk_ptp_contact contact = frame->contacts[i];
            if (host->active & BIT(contact.id)) {
                /* A rejected/palm contact stays unconfident for its lifetime. */
                contact.confidence &= host->contacts[contact.id].confidence;
            }
            contacts[contact.id] = contact;
            active |= BIT(contact.id);
        }
    }

    struct zmk_ptp_report report = {
        .report_id = ZMK_PTP_REPORT_ID_INPUT,
        .scan_time = sys_cpu_to_le16(frame->scan_time),
    };
    int count = 0;
    /* Emit explicit lifts once, at their last accepted coordinates. */
    for (int id = 0; id < CONFIG_ZMK_TRACKPAD_FINGERS; id++) {
        if ((host->active & BIT(id)) && !(active & BIT(id))) {
            report.contacts[count++] = encode_contact(host->contacts[id], false);
        }
    }
    for (int id = 0; id < CONFIG_ZMK_TRACKPAD_FINGERS; id++) {
        if (active & BIT(id)) {
            report.contacts[count++] = encode_contact(contacts[id], true);
        }
    }
    uint8_t buttons = (host->selective & ZMK_PTP_BUTTONS) ? frame->buttons : 0;
    report.count_buttons = count | (buttons << 4);
    int err = send_report(host, &report);
    if (!err) {
        host->active = active;
        memcpy(host->contacts, contacts, sizeof(contacts));
        host->report = report;
    }
    return err;
}

static int release(struct ptp_host *host) {
    if (!host->active && !(host->report.count_buttons & 0x70)) {
        host->release_pending = false;
        return 0;
    }
    /* Synthetic cleanup has no new sensor scan. Keep the last source clock,
     * which may belong to a peripheral with an independent boot offset. */
    struct zmk_ptp_frame frame = {.scan_time = sys_le16_to_cpu(host->report.scan_time)};
    int err = submit(host, &frame);
    host->release_pending = err != 0;
    return err;
}

static void apply_reset(int index) {
    if (atomic_test_and_clear_bit(reset_pending, index)) {
        struct zmk_endpoint_instance endpoint = hosts[index].endpoint;
        hosts[index] = (struct ptp_host){
            .endpoint = endpoint,
            .report.report_id = ZMK_PTP_REPORT_ID_INPUT,
            .selective = ZMK_PTP_SURFACE | ZMK_PTP_BUTTONS,
        };
    }
}

static void recovery_work(struct k_work *work) {
    if (k_mutex_lock(&ptp_lock, K_NO_WAIT)) {
        k_work_reschedule(&recovery, K_MSEC(50));
        return;
    }
    bool retry = false;
    for (int i = 0; i < ZMK_ENDPOINT_COUNT; i++) {
        struct ptp_host *host = &hosts[i];
        apply_reset(i);
        if (host->release_pending && !host->suspended && release(host)) {
            retry = true;
        }
    }
    k_mutex_unlock(&ptp_lock);
    if (retry) {
        k_work_reschedule(&recovery, K_MSEC(50));
    }
}

static int submit_frame(const struct zmk_ptp_frame *frame, const uint32_t *epoch) {
    int validation = zmk_ptp_validate_frame(frame);
    if (validation) {
        return validation;
    }
    zmk_ptp_note_activity(frame);
    if (k_mutex_lock(&ptp_lock, K_NO_WAIT)) {
        return -EAGAIN;
    }
#if IS_ENABLED(CONFIG_ZMK_SPLIT)
    /* Admission linearizes here with endpoint changes. Already admitted
     * transfers cannot be withdrawn; invalidated bridge handoffs are rejected. */
    if (epoch && !zmk_ptp_split_epoch_is_current(*epoch)) {
        k_mutex_unlock(&ptp_lock);
        return -ESTALE;
    }
#endif
    zmk_ptp_touch_update(frame);
    int index = host_index(selected);
    int err = -ENODEV;
    if (index >= 0) {
        struct ptp_host *host = &hosts[index];
        apply_reset(index);
        err = host->release_pending ? release(host) : 0;
        if (!err) {
            err = submit(host, frame);
        }
    }
    k_mutex_unlock(&ptp_lock);
    return err;
}

int zmk_ptp_submit_frame(const struct zmk_ptp_frame *frame) { return submit_frame(frame, NULL); }

#if IS_ENABLED(CONFIG_ZMK_SPLIT)
int zmk_ptp_submit_split_frame(const struct zmk_ptp_frame *frame, uint32_t epoch) {
    return submit_frame(frame, &epoch);
}
#endif

static int release_selected(bool cancel) {
    if (k_is_in_isr()) {
        return -EWOULDBLOCK;
    }
    if (k_mutex_lock(&ptp_lock, K_NO_WAIT)) {
        return -EAGAIN;
    }
    zmk_ptp_touch_update(NULL);
    int index = host_index(selected);
    if (index >= 0) {
        apply_reset(index);
    }
    if (cancel && index >= 0) {
        for (int i = 0; i < CONFIG_ZMK_TRACKPAD_FINGERS; i++) {
            hosts[index].contacts[i].confidence = false;
        }
    }
    int err = index >= 0 ? release(&hosts[index]) : 0;
    if (err) {
        k_work_reschedule(&recovery, K_MSEC(50));
    }
    k_mutex_unlock(&ptp_lock);
    /* Cancellation owns deferred cleanup; the bridge need only retry contention. */
    return cancel ? 0 : err;
}

int zmk_ptp_release(void) { return release_selected(false); }

int zmk_ptp_cancel(void) { return release_selected(true); }

void zmk_ptp_set_endpoint(struct zmk_endpoint_instance endpoint) {
    k_mutex_lock(&ptp_lock, K_FOREVER);
    if (!zmk_endpoint_instance_eq(selected, endpoint)) {
        int old = host_index(selected);
        if (old >= 0 && release(&hosts[old])) {
            k_work_reschedule(&recovery, K_MSEC(50));
        }
#if IS_ENABLED(CONFIG_ZMK_SPLIT)
        zmk_ptp_split_reset();
#endif
        selected = endpoint;
        int index = host_index(endpoint);
#if IS_ENABLED(CONFIG_ZMK_SPLIT)
        atomic_set(&selected_index, index);
#endif
        if (index >= 0) {
            hosts[index].endpoint = endpoint;
        }
    }
    k_mutex_unlock(&ptp_lock);
}

void zmk_ptp_reset_endpoint(struct zmk_endpoint_instance endpoint) {
    int index = host_index(endpoint);
    if (index >= 0) {
        atomic_set_bit(reset_pending, index);
#if IS_ENABLED(CONFIG_ZMK_SPLIT)
        if (index == atomic_get(&selected_index)) {
            zmk_ptp_split_reset();
        }
#endif
        k_work_reschedule(&recovery, K_NO_WAIT);
    }
}

int zmk_ptp_set_suspended(struct zmk_endpoint_instance endpoint, bool suspended) {
    int index = host_index(endpoint);
    if (index < 0) {
        return index;
    }
    if (k_mutex_lock(&ptp_lock, K_NO_WAIT)) {
        return -EAGAIN;
    }
    apply_reset(index);
    hosts[index].suspended = suspended;
    k_mutex_unlock(&ptp_lock);
    if (!suspended) {
        k_work_reschedule(&recovery, K_NO_WAIT);
    }
    return 0;
}

int zmk_ptp_get_report(struct zmk_endpoint_instance endpoint, struct zmk_ptp_report *report) {
    int index = host_index(endpoint);
    if (index < 0 || !report) {
        return -EINVAL;
    }
    /* HID callbacks must never wait for a sender holding ptp_lock. */
    if (k_mutex_lock(&ptp_lock, K_NO_WAIT)) {
        return -EAGAIN;
    }
    apply_reset(index);
    *report = hosts[index].report;
    k_mutex_unlock(&ptp_lock);
    return 0;
}

int zmk_ptp_get_feature(struct zmk_endpoint_instance endpoint, uint8_t id, uint8_t *data,
                        size_t size) {
    int index = host_index(endpoint);
    if (index < 0 || !data) {
        return -EINVAL;
    }
    size_t len = id == ZMK_PTP_REPORT_ID_CERTIFICATION ? 256 : 1;
    if (size < len) {
        return -EMSGSIZE;
    }
    if (k_mutex_lock(&ptp_lock, K_NO_WAIT)) {
        return -EAGAIN;
    }
    apply_reset(index);
    int err = len;
    switch (id) {
    case ZMK_PTP_REPORT_ID_CAPABILITIES:
        data[0] = CONFIG_ZMK_TRACKPAD_FINGERS | (CONFIG_ZMK_TRACKPAD_PAD_TYPE << 4);
        break;
    case ZMK_PTP_REPORT_ID_CERTIFICATION:
        memset(data, 0, len); /* Uncertified, not a forged certification blob. */
        break;
    case ZMK_PTP_REPORT_ID_MODE:
        data[0] = ZMK_PTP_MODE_TOUCHPAD;
        break;
    case ZMK_PTP_REPORT_ID_SELECTIVE:
        data[0] = hosts[index].selective;
        break;
    default:
        err = -ENOTSUP;
        break;
    }
    k_mutex_unlock(&ptp_lock);
    return err;
}

int zmk_ptp_get_ble_feature(struct zmk_endpoint_instance endpoint, uint8_t id, uint8_t *data,
                            size_t size) {
    bool pad = IS_ENABLED(CONFIG_ZMK_BLE_PTP_FEATURE_PAD_BYTE);
    if (!size && pad) {
        return -EMSGSIZE;
    }
    int len = zmk_ptp_get_feature(endpoint, id, data, size - pad);
    if (len >= 0 && pad) {
        data[len++] = 0;
    }
    return len;
}

int zmk_ptp_set_feature(struct zmk_endpoint_instance endpoint, uint8_t id, const uint8_t *data,
                        size_t size) {
    int index = host_index(endpoint);
    if (index < 0 || !data || size != 1) {
        return -EINVAL;
    }
    if (id == ZMK_PTP_REPORT_ID_MODE) {
        /* Native-only: never claim to provide an unimplemented mouse fallback. */
        return data[0] == ZMK_PTP_MODE_TOUCHPAD ? 0 : -ENOTSUP;
    }
    if (id != ZMK_PTP_REPORT_ID_SELECTIVE) {
        return -ENOTSUP;
    }
    if (data[0] & ~(ZMK_PTP_SURFACE | ZMK_PTP_BUTTONS)) {
        return -EINVAL;
    }
    if (k_mutex_lock(&ptp_lock, K_NO_WAIT)) {
        return -EAGAIN;
    }
    apply_reset(index);
    struct ptp_host *host = &hosts[index];
    if (host->selective != data[0]) {
        /* Release any old contacts/buttons before changing reporting policy. */
        if (release(host)) {
            k_work_reschedule(&recovery, K_MSEC(50));
        }
        host->selective = data[0];
    }
    k_mutex_unlock(&ptp_lock);
    return 0;
}
