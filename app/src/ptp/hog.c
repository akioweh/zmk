/*
 * Copyright (c) 2020 The ZMK Contributors
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 *
 * Adapted from Pete Johanson's PTP HoG backend.
 */

#include <errno.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zmk/ble.h>
#include <zmk/ptp/queue.h>
#include "hid.h"

struct report_ref {
    uint8_t id;
    uint8_t type;
} __packed;

static const uint8_t hid_info[] = {0x11, 0x01, 0, 0x03};
static const struct report_ref input_ref = {ZMK_PTP_REPORT_ID_INPUT, 1};
static const struct report_ref capabilities_ref = {ZMK_PTP_REPORT_ID_CAPABILITIES, 3};
static const struct report_ref certification_ref = {ZMK_PTP_REPORT_ID_CERTIFICATION, 3};
static const struct report_ref mode_ref = {ZMK_PTP_REPORT_ID_MODE, 3};
static const struct report_ref selective_ref = {ZMK_PTP_REPORT_ID_SELECTIVE, 3};
static atomic_t suspended[ZMK_BLE_PROFILE_COUNT];
static atomic_ptr_t bound_conns[ZMK_BLE_PROFILE_COUNT];
static void notify_work(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(notifier, notify_work);

static int connection_endpoint(struct bt_conn *conn, struct zmk_endpoint_instance *endpoint) {
    int profile = zmk_ble_profile_index(bt_conn_get_dst(conn));
    if (profile < 0 || profile >= ZMK_BLE_PROFILE_COUNT) {
        return -ENODEV;
    }
    *endpoint = (struct zmk_endpoint_instance){.transport = ZMK_TRANSPORT_BLE,
                                               .ble.profile_index = profile};
    return 0;
}

static ssize_t read_info(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
                         uint16_t len, uint16_t offset) {
    return bt_gatt_attr_read(conn, attr, buf, len, offset, hid_info, sizeof(hid_info));
}

static ssize_t read_map(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
                        uint16_t len, uint16_t offset) {
    return bt_gatt_attr_read(conn, attr, buf, len, offset, zmk_ptp_hid_report_desc,
                             sizeof(zmk_ptp_hid_report_desc));
}

static ssize_t read_ref(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
                        uint16_t len, uint16_t offset) {
    return bt_gatt_attr_read(conn, attr, buf, len, offset, attr->user_data,
                             sizeof(struct report_ref));
}

static ssize_t read_input(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
                          uint16_t len, uint16_t offset) {
    struct zmk_endpoint_instance endpoint;
    struct zmk_ptp_report report;
    if (connection_endpoint(conn, &endpoint) || zmk_ptp_get_report(endpoint, &report)) {
        return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
    }
    /* HoG report values exclude the Report ID. */
    return bt_gatt_attr_read(conn, attr, buf, len, offset, (uint8_t *)&report + 1,
                             sizeof(report) - 1);
}

static ssize_t read_feature(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
                            uint16_t len, uint16_t offset) {
    const struct report_ref *ref = attr->user_data;
    struct zmk_endpoint_instance endpoint;
    uint8_t data[256];
    if (connection_endpoint(conn, &endpoint)) {
        return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
    }
    int size = zmk_ptp_get_feature(endpoint, ref->id, data, sizeof(data));
    if (size < 0) {
        return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
    }
    return bt_gatt_attr_read(conn, attr, buf, len, offset, data, size);
}

static ssize_t write_feature(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
                             uint16_t len, uint16_t offset, uint8_t flags) {
    const struct report_ref *ref = attr->user_data;
    struct zmk_endpoint_instance endpoint;
    if (offset) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }
    if (len != 1) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }
    if (connection_endpoint(conn, &endpoint)) {
        return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
    }
    int err = zmk_ptp_set_feature(endpoint, ref->id, buf, len);
    if (err) {
        return BT_GATT_ERR(err == -EAGAIN ? BT_ATT_ERR_UNLIKELY : BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }
    return len;
}

static ssize_t write_control(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
                             uint16_t len, uint16_t offset, uint8_t flags) {
    struct zmk_endpoint_instance endpoint;
    if (offset) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }
    if (len != 1) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }
    uint8_t value = *(const uint8_t *)buf;
    if (value > 1) {
        return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }
    if (connection_endpoint(conn, &endpoint) || zmk_ptp_set_suspended(endpoint, value == 0)) {
        return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
    }
    atomic_set(&suspended[endpoint.ble.profile_index], value == 0);
    if (value == 1) {
        k_work_reschedule(&notifier, K_NO_WAIT);
    }
    return len;
}

static void ccc_changed(const struct bt_gatt_attr *attr, uint16_t value) {}

#define FEATURE_RO(ref)                                                                            \
    BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_REPORT, BT_GATT_CHRC_READ, BT_GATT_PERM_READ_ENCRYPT,      \
                           read_feature, NULL, (void *)&ref),                                      \
        BT_GATT_DESCRIPTOR(BT_UUID_HIDS_REPORT_REF, BT_GATT_PERM_READ_ENCRYPT, read_ref, NULL,     \
                           (void *)&ref)
#define FEATURE_RW(ref)                                                                            \
    BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_REPORT, BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,            \
                           BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT, read_feature,   \
                           write_feature, (void *)&ref),                                           \
        BT_GATT_DESCRIPTOR(BT_UUID_HIDS_REPORT_REF, BT_GATT_PERM_READ_ENCRYPT, read_ref, NULL,     \
                           (void *)&ref)

BT_GATT_SERVICE_DEFINE(
    ptp_svc, BT_GATT_PRIMARY_SERVICE(BT_UUID_HIDS),
    BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_INFO, BT_GATT_CHRC_READ, BT_GATT_PERM_READ, read_info, NULL,
                           NULL),
    BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_REPORT_MAP, BT_GATT_CHRC_READ, BT_GATT_PERM_READ_ENCRYPT,
                           read_map, NULL, NULL),
    BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_REPORT, BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ_ENCRYPT, read_input, NULL, NULL),
    BT_GATT_CCC(ccc_changed, BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT),
    BT_GATT_DESCRIPTOR(BT_UUID_HIDS_REPORT_REF, BT_GATT_PERM_READ_ENCRYPT, read_ref, NULL,
                       (void *)&input_ref),
    FEATURE_RO(capabilities_ref), FEATURE_RO(certification_ref), FEATURE_RW(mode_ref),
    FEATURE_RW(selective_ref),
    BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_CTRL_POINT, BT_GATT_CHRC_WRITE_WITHOUT_RESP,
                           BT_GATT_PERM_WRITE_ENCRYPT, NULL, write_control, NULL));

/* Value attribute, not the characteristic declaration; no conditional attrs precede it. */
#define PTP_INPUT_ATTR 6

struct queued_report {
    struct zmk_ptp_queue_header header;
    /* A retained connection reference pins the original peer/session, not just a profile slot. */
    struct bt_conn *conn;
    struct zmk_ptp_report report;
};

/* A suspended old host must not block the newly selected host's reports. */
static struct k_msgq reports[ZMK_BLE_PROFILE_COUNT];
static char report_buffers[ZMK_BLE_PROFILE_COUNT][CONFIG_ZMK_BLE_PTP_REPORT_QUEUE_SIZE *
                                                  sizeof(struct queued_report)] __aligned(4);
static struct k_spinlock report_lock;
static uint32_t report_ticket;
static struct bt_conn *last_conn[ZMK_BLE_PROFILE_COUNT];
static uint16_t last_state[ZMK_BLE_PROFILE_COUNT];
static bool report_seen[ZMK_BLE_PROFILE_COUNT];

static int connection_ready(struct bt_conn *conn) {
    struct bt_conn_info info;
    if (bt_conn_get_info(conn, &info) || info.state != BT_CONN_STATE_CONNECTED) {
        return -ENOTCONN;
    }
    if (!bt_gatt_is_subscribed(conn, &ptp_svc.attrs[PTP_INPUT_ATTR], BT_GATT_CCC_NOTIFY)) {
        return -EACCES;
    }
    if (bt_gatt_get_mtu(conn) < sizeof(struct zmk_ptp_report) - 1 + 3) {
        return -EMSGSIZE;
    }
    return 0;
}

static void notify_work(struct k_work *work) {
    bool retry = false;
    for (int profile = 0; profile < ZMK_BLE_PROFILE_COUNT; profile++) {
        struct queued_report entry, scratch;
        while (true) {
            k_spinlock_key_t key = k_spin_lock(&report_lock);
            bool ready = k_msgq_peek(&reports[profile], &entry) == 0;
            if (ready) {
                bt_conn_ref(entry.conn);
            }
            k_spin_unlock(&report_lock, key);
            if (!ready) {
                break;
            }
            int err = connection_ready(entry.conn);
            if (!err && atomic_get(&suspended[profile])) {
                bt_conn_unref(entry.conn);
                break; /* Resume/disconnect will schedule us again. */
            }
            if (!err) {
                struct bt_gatt_notify_params params = {
                    .attr = &ptp_svc.attrs[PTP_INPUT_ATTR],
                    .data = (const uint8_t *)&entry.report + 1,
                    .len = sizeof(entry.report) - 1,
                };
                /* System workqueue context makes resource exhaustion non-blocking. */
                err = bt_gatt_notify_cb(entry.conn, &params);
                if (err && err != -ENOTCONN) {
                    bt_conn_unref(entry.conn);
                    retry = true;
                    break; /* Retain transitions, but allow pending motion to be refreshed. */
                }
            }
            bt_conn_unref(entry.conn);
            /* Ended HID sessions are never replayed to a new peer. */
            key = k_spin_lock(&report_lock);
            bool removed = zmk_ptp_queue_get(&reports[profile], entry.header.ticket, &scratch);
            k_spin_unlock(&report_lock, key);
            if (removed) {
                bt_conn_unref(scratch.conn);
            }
        }
    }
    if (retry) {
        k_work_reschedule(&notifier, K_MSEC(5));
    }
}

int zmk_ptp_hog_send_report(struct zmk_endpoint_instance endpoint,
                            const struct zmk_ptp_report *report) {
    int profile = endpoint.ble.profile_index;
    if (profile < 0 || profile >= ZMK_BLE_PROFILE_COUNT) {
        return -EINVAL;
    }
    struct bt_conn *conn = bt_conn_lookup_addr_le(BT_ID_DEFAULT, zmk_ble_profile_address(profile));
    if (!conn) {
        return -ENODEV;
    }
    struct bt_conn *old = atomic_ptr_set(&bound_conns[profile], bt_conn_ref(conn));
    if (old) {
        if (old != conn) {
            zmk_ptp_reset_endpoint(endpoint);
        }
        bt_conn_unref(old);
    }
    int err = connection_ready(conn);
    if (err) {
        if (atomic_ptr_cas(&bound_conns[profile], conn, NULL)) {
            bt_conn_unref(conn);
        }
        bt_conn_unref(conn);
        return err;
    }
    struct queued_report entry = {.conn = conn, .report = *report}, scratch;
    bool lift;
    uint16_t state = zmk_ptp_report_state(report, &lift);
    k_spinlock_key_t key = k_spin_lock(&report_lock);
    entry.header = (struct zmk_ptp_queue_header){
        .ticket = ++report_ticket,
        .state = state,
        .motion = !lift && report_seen[profile] && last_conn[profile] == conn &&
                  last_state[profile] == state,
    };
    err = zmk_ptp_queue_put(&reports[profile], &entry, &scratch);
    if (err >= 0) {
        report_seen[profile] = true;
        last_conn[profile] = conn;
        last_state[profile] = state;
    }
    k_spin_unlock(&report_lock, key);
    if (err < 0) {
        bt_conn_unref(conn);
        return -EAGAIN; /* Transition backpressure never overwrites a lifecycle boundary. */
    }
    if (err == 1) {
        bt_conn_unref(scratch.conn);
    }
    k_work_reschedule(&notifier, K_NO_WAIT);
    return 0;
}

static void disconnected(struct bt_conn *conn, uint8_t reason) {
    k_spinlock_key_t key = k_spin_lock(&report_lock);
    for (int i = 0; i < ZMK_BLE_PROFILE_COUNT; i++) {
        if (last_conn[i] == conn) {
            report_seen[i] = false;
            last_conn[i] = NULL;
        }
    }
    k_spin_unlock(&report_lock, key);
    int profile = zmk_ble_profile_index(bt_conn_get_dst(conn));
    if (profile >= 0 && profile < ZMK_BLE_PROFILE_COUNT) {
        atomic_clear(&suspended[profile]);
        zmk_ptp_reset_endpoint((struct zmk_endpoint_instance){.transport = ZMK_TRANSPORT_BLE,
                                                              .ble.profile_index = profile});
    }
    for (int i = 0; i < ZMK_BLE_PROFILE_COUNT; i++) {
        if (atomic_ptr_cas(&bound_conns[i], conn, NULL)) {
            atomic_clear(&suspended[i]);
            zmk_ptp_reset_endpoint((struct zmk_endpoint_instance){.transport = ZMK_TRANSPORT_BLE,
                                                                  .ble.profile_index = i});
            bt_conn_unref(conn);
        }
    }
    k_work_reschedule(&notifier, K_NO_WAIT);
}

BT_CONN_CB_DEFINE(ptp_conn_callbacks) = {.disconnected = disconnected};

static int ptp_hog_init(void) {
    for (int i = 0; i < ZMK_BLE_PROFILE_COUNT; i++) {
        k_msgq_init(&reports[i], report_buffers[i], sizeof(struct queued_report),
                    CONFIG_ZMK_BLE_PTP_REPORT_QUEUE_SIZE);
    }
    return 0;
}

SYS_INIT(ptp_hog_init, APPLICATION, CONFIG_ZMK_BLE_INIT_PRIORITY);
