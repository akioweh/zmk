/*
 * Copyright (c) 2026 The ZMK Contributors
 * SPDX-License-Identifier: MIT
 */
#include <zephyr/ztest.h>
#include <zephyr/fff.h>

/* Both split roles share this harness; the central header omits this peripheral hook. */
void zmk_ptp_split_resume(void);

/* Compile the production workers, GATT attributes and connection callbacks.
 * Private queue assertions avoid adding production-only test entry points. */
#include "../../../src/ptp/hog.c"
#include "../../../src/split/bluetooth/service.c"

DEFINE_FFF_GLOBALS;
FAKE_VALUE_FUNC(int, zmk_ptp_get_report, struct zmk_endpoint_instance, struct zmk_ptp_report *);
FAKE_VALUE_FUNC(int, zmk_ptp_get_feature, struct zmk_endpoint_instance, uint8_t, uint8_t *, size_t);
FAKE_VALUE_FUNC(int, zmk_ptp_set_feature, struct zmk_endpoint_instance, uint8_t, const uint8_t *,
                size_t);
FAKE_VALUE_FUNC(int, zmk_ptp_set_suspended, struct zmk_endpoint_instance, bool);
FAKE_VOID_FUNC(zmk_ptp_reset_endpoint, struct zmk_endpoint_instance);
FAKE_VOID_FUNC(zmk_ptp_split_resume);
FAKE_VALUE_FUNC(int, zmk_activity_note);
FAKE_VALUE_FUNC(int, zmk_physical_layouts_select, uint8_t);
FAKE_VALUE_FUNC(int, zmk_physical_layouts_get_selected);
FAKE_VALUE_FUNC(struct zmk_split_transport_peripheral *, zmk_split_transport_peripheral_bt);
FAKE_VALUE_FUNC(int, zmk_split_transport_peripheral_command_handler,
                const struct zmk_split_transport_peripheral *,
                struct zmk_split_transport_central_command);
FAKE_VALUE_FUNC(ssize_t, bt_gatt_attr_read_service, struct bt_conn *, const struct bt_gatt_attr *,
                void *, uint16_t, uint16_t);
FAKE_VALUE_FUNC(ssize_t, bt_gatt_attr_read_chrc, struct bt_conn *, const struct bt_gatt_attr *,
                void *, uint16_t, uint16_t);
FAKE_VALUE_FUNC(ssize_t, bt_gatt_attr_read_ccc, struct bt_conn *, const struct bt_gatt_attr *,
                void *, uint16_t, uint16_t);
FAKE_VALUE_FUNC(ssize_t, bt_gatt_attr_write_ccc, struct bt_conn *, const struct bt_gatt_attr *,
                const void *, uint16_t, uint16_t, uint8_t);

struct bt_conn {
    bt_addr_le_t addr;
    bool connected;
    int refs;
};
/* A reconnect has the same address but a different connection object. */
static struct bt_conn peers[3];
static struct bt_conn *split_peer;
static int admission_error;
static uint16_t mtu = 65;
static bool subscribed = true;
static unsigned int sent_count, completion_index;
static struct {
    struct bt_conn *conn;
    bt_gatt_complete_func_t complete;
    void *user_data;
    uint8_t bytes[ZMK_PTP_SPLIT_FRAME_SIZE];
    uint16_t len;
} sent[32];

struct bt_conn *bt_conn_ref(struct bt_conn *conn) {
    zassert_not_null(conn);
    conn->refs++;
    return conn;
}
void bt_conn_unref(struct bt_conn *conn) {
    zassert_true(conn->refs > 0);
    conn->refs--;
}
const bt_addr_le_t *bt_conn_get_dst(const struct bt_conn *conn) { return &conn->addr; }
int bt_conn_get_info(const struct bt_conn *conn, struct bt_conn_info *info) {
    *info = (struct bt_conn_info){.state = conn->connected ? BT_CONN_STATE_CONNECTED
                                                           : BT_CONN_STATE_DISCONNECTED};
    return 0;
}
bt_security_t bt_conn_get_security(const struct bt_conn *conn) { return BT_SECURITY_L2; }
int zmk_ble_profile_index(const bt_addr_le_t *addr) { return addr->a.val[0]; }
bt_addr_le_t *zmk_ble_profile_address(uint8_t index) {
    static bt_addr_le_t addresses[3];
    addresses[index].a.val[0] = index;
    return &addresses[index];
}
struct bt_conn *bt_conn_lookup_addr_le(uint8_t id, const bt_addr_le_t *addr) {
    for (int i = 0; i < ARRAY_SIZE(peers); i++) {
        if (peers[i].connected && !bt_addr_le_cmp(&peers[i].addr, addr)) {
            return bt_conn_ref(&peers[i]);
        }
    }
    return NULL;
}
void bt_conn_foreach(enum bt_conn_type type, void (*func)(struct bt_conn *, void *), void *data) {
    if (split_peer) {
        func(split_peer, data);
    }
}
uint16_t bt_gatt_get_mtu(struct bt_conn *conn) { return mtu; }
bool bt_gatt_is_subscribed(struct bt_conn *conn, const struct bt_gatt_attr *attr, uint16_t ccc) {
    return subscribed;
}
ssize_t bt_gatt_attr_read(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
                          uint16_t len, uint16_t offset, const void *value, uint16_t value_len) {
    if (offset > value_len) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }
    len = MIN(len, value_len - offset);
    memcpy(buf, (const uint8_t *)value + offset, len);
    return len;
}
struct bt_gatt_attr *bt_gatt_find_by_uuid(const struct bt_gatt_attr *attrs, uint16_t count,
                                          const struct bt_uuid *uuid) {
    zassert_equal(uuid->type, BT_UUID_TYPE_128);
    for (int i = 0; i < count; i++) {
        if (attrs[i].uuid->type == BT_UUID_TYPE_128 &&
            !memcmp(BT_UUID_128(attrs[i].uuid)->val, BT_UUID_128(uuid)->val, 16)) {
            return (struct bt_gatt_attr *)&attrs[i];
        }
    }
    return NULL;
}
int bt_gatt_notify_cb(struct bt_conn *conn, struct bt_gatt_notify_params *params) {
    zassert_true(k_current_get() == &k_sys_work_q.thread,
                 "Production notifier must run on the system workqueue");
    if (!conn || !conn->connected) {
        return -ENOTCONN;
    }
    if (admission_error) {
        return admission_error;
    }
    zassert_not_null(params->func);
    zassert_true(sent_count < ARRAY_SIZE(sent));
    zassert_true(params->len <= sizeof(sent[0].bytes));
    sent[sent_count].conn = bt_conn_ref(conn);
    sent[sent_count].complete = params->func;
    sent[sent_count].user_data = params->user_data;
    sent[sent_count].len = params->len;
    memcpy(sent[sent_count++].bytes, params->data, params->len);
    return 0;
}

/* Zephyr invokes completion on its system workqueue, not on the submitting thread. */
static void complete_work(struct k_work *work) {
    unsigned int i = completion_index;
    zassert_true(i < sent_count);
    zassert_not_null(sent[i].complete);
    sent[i].complete(sent[i].conn, sent[i].user_data);
    sent[i].complete = NULL;
    bt_conn_unref(sent[i].conn);
}
K_WORK_DEFINE(completer, complete_work);
static void complete(unsigned int i) {
    completion_index = i;
    k_work_submit(&completer);
    k_sleep(K_MSEC(10));
}
static void disconnect_peer(struct bt_conn *conn) {
    conn->connected = false;
    /* Use the production callbacks, including queued-connection cleanup. */
    disconnected(conn, 0);
    contact_disconnected(conn, 0);
    k_sleep(K_MSEC(10));
}
static void clean(void) {
    admission_error = 0;
    for (int i = 0; i < ARRAY_SIZE(peers); i++) {
        disconnect_peer(&peers[i]);
    }
    for (unsigned int i = 0; i < sent_count; i++) {
        if (sent[i].complete) {
            complete(i);
        }
    }
    k_sleep(K_MSEC(10));
    for (int i = 0; i < ARRAY_SIZE(peers); i++) {
        zassert_equal(peers[i].refs, 0, "Leaked connection %d", i);
    }
    for (int i = 0; i < ARRAY_SIZE(profiles); i++) {
        zassert_equal(profiles[i].in_flight, 0);
        zassert_equal(k_msgq_num_used_get(&profiles[i].reports), 0);
    }
    zassert_equal(contact_in_flight, 0);
    zassert_equal(k_msgq_num_used_get(&contact_msgq), 0);
    sent_count = 0;
}
static void before(void *fixture) {
    clean();
    memset(peers, 0, sizeof(peers));
    peers[2].addr.a.val[0] = 1;
    peers[0].connected = true;
    split_peer = &peers[0];
    subscribed = true;
    mtu = 65;
}
static void after(void *fixture) { clean(); }

static int send_frame(bool split, unsigned int sequence, uint16_t x, bool down) {
    struct zmk_ptp_frame frame = {
        .scan_time = sequence * 100,
        .contact_count = down ? 1 : 0,
        .contacts = {{.id = 0, .x = x, .y = 100, .confidence = true}},
    };
    if (split) {
        struct zmk_split_transport_peripheral_event event = {
            .type = ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_CONTACT_FRAME,
        };
        zmk_ptp_split_encode(&event.data.contact_frame, sequence, &frame);
        return zmk_split_transport_peripheral_bt_report_event(&event);
    }
    struct zmk_ptp_report report = {
        .report_id = ZMK_PTP_REPORT_ID_INPUT,
        .scan_time = sys_cpu_to_le16(frame.scan_time),
        .count_buttons = 1,
        .contacts = {{.flags_id = ZMK_PTP_CONFIDENCE | (down ? ZMK_PTP_TIP : 0),
                      .x = sys_cpu_to_le16(x),
                      .y = sys_cpu_to_le16(100)}},
    };
    return zmk_ptp_hog_send_report(
        (struct zmk_endpoint_instance){.transport = ZMK_TRANSPORT_BLE, .ble.profile_index = 0},
        &report);
}
static unsigned int in_flight(bool split) {
    return split ? contact_in_flight : profiles[0].in_flight;
}
static uint16_t sent_x(bool split, unsigned int i) {
    return sys_get_le16(sent[i].bytes + (split ? ZMK_PTP_SPLIT_HEADER_SIZE + 1 : 1));
}

ZTEST(ptp_ble, test_completion_releases_latest_motion_on_both_links) {
    for (int split = 0; split <= 1; split++) {
        before(NULL);
        zassert_ok(send_frame(split, 1, 10, true));
        k_sleep(K_MSEC(10));
        zassert_equal(sent_count, 1);
        zassert_equal(in_flight(split), 1);
        zassert_ok(send_frame(split, 2, 20, true));
        zassert_ok(send_frame(split, 3, 30, true));
        k_sleep(K_MSEC(20));
        zassert_equal(sent_count, 1); /* Wait for completion, not an elapsed timer. */
        complete(0);
        zassert_equal(sent_count, 2);
        zassert_equal(sent_x(split, 1), 30);
        zassert_equal(in_flight(split), 1);
        if (split) {
            zassert_equal(sys_get_le16(sent[1].bytes), 3);
            zassert_equal(sys_get_le16(sent[1].bytes + 4), 2); /* Coalesced coverage. */
        }
        complete(1);
        zassert_equal(in_flight(split), 0);
    }
}

ZTEST(ptp_ble, test_lift_flushes_motion_before_completion_on_both_links) {
    for (int split = 0; split <= 1; split++) {
        before(NULL);
        zassert_ok(send_frame(split, 1, 10, true));
        k_sleep(K_MSEC(10));
        zassert_ok(send_frame(split, 2, 20, true));
        zassert_ok(send_frame(split, 3, 20, false));
        k_sleep(K_MSEC(10));
        zassert_equal(sent_count, 3);
        zassert_equal(sent_x(split, 1), 20);
        zassert_equal(in_flight(split), 3);
        if (split) {
            zassert_equal(sent[2].bytes[6] & 7, 0);
        } else {
            zassert_false(sent[2].bytes[0] & ZMK_PTP_TIP);
        }
        /* Completion order must not matter to the outstanding count. */
        complete(2);
        complete(0);
        complete(1);
        zassert_equal(in_flight(split), 0);
    }
}

ZTEST(ptp_ble, test_admission_failure_rolls_back_and_retries_on_both_links) {
    const int errors[] = {-ENOMEM, -EAGAIN, -ENOBUFS};
    for (int split = 0; split <= 1; split++) {
        for (int e = 0; e < ARRAY_SIZE(errors); e++) {
            before(NULL);
            admission_error = errors[e];
            zassert_ok(send_frame(split, 1, 10, true));
            k_sleep(K_MSEC(20));
            zassert_equal(sent_count, 0);
            zassert_equal(in_flight(split), 0);
            /* Birth is retained, only later motion can be replaced. */
            zassert_ok(send_frame(split, 2, 20, true));
            zassert_ok(send_frame(split, 3, 30, true));
            admission_error = 0;
            k_sleep(K_MSEC(20));
            zassert_equal(sent_count, 1);
            zassert_equal(sent_x(split, 0), 10);
            complete(0);
            zassert_equal(sent_count, 2);
            zassert_equal(sent_x(split, 1), 30);
        }
    }
}

ZTEST(ptp_ble, test_disconnect_and_late_completion_cannot_unblock_new_session) {
    for (int split = 0; split <= 1; split++) {
        before(NULL);
        zassert_ok(send_frame(split, 1, 10, true));
        k_sleep(K_MSEC(10));
        zassert_ok(send_frame(split, 2, 20, true));
        k_sleep(K_MSEC(10));
        disconnect_peer(&peers[0]);
        zassert_equal(in_flight(split), 0);
        peers[1].connected = true; /* Same profile/address, distinct session. */
        split_peer = &peers[1];
        zassert_ok(send_frame(split, 3, 30, true));
        k_sleep(K_MSEC(10));
        zassert_equal(sent_count, 2);
        zassert_equal(sent_x(split, 1), 30);
        zassert_equal(sent[1].conn, &peers[1]);
        zassert_ok(send_frame(split, 4, 40, true));
        complete(0); /* The old connection's completion arrives late. */
        zassert_equal(in_flight(split), 1);
        zassert_equal(sent_count, 2);
        complete(1);
        zassert_equal(sent_count, 3);
        zassert_equal(sent_x(split, 2), 40);
    }
}

ZTEST(ptp_ble, test_host_suspend_does_not_block_other_profile) {
    zassert_ok(send_frame(false, 1, 10, true));
    k_sleep(K_MSEC(10));
    uint8_t suspend = 0;
    zassert_equal(write_control(&peers[0], NULL, &suspend, 1, 0, 0), 1);
    zassert_ok(send_frame(false, 2, 20, true));
    complete(0);
    zassert_equal(sent_count, 1);
    peers[2].connected = true;
    struct zmk_ptp_report report = {
        .report_id = ZMK_PTP_REPORT_ID_INPUT,
        .count_buttons = 1,
        .contacts = {{.flags_id = ZMK_PTP_CONFIDENCE | ZMK_PTP_TIP}},
    };
    zassert_ok(zmk_ptp_hog_send_report(
        (struct zmk_endpoint_instance){.transport = ZMK_TRANSPORT_BLE, .ble.profile_index = 1},
        &report));
    k_sleep(K_MSEC(10));
    zassert_equal(sent_count, 2);
    zassert_equal(sent[1].conn, &peers[2]);
    uint8_t resume = 1;
    zassert_equal(write_control(&peers[0], NULL, &resume, 1, 0, 0), 1);
    k_sleep(K_MSEC(10));
    zassert_equal(sent_count, 3);
    zassert_equal(sent_x(false, 2), 20);
}

ZTEST_SUITE(ptp_ble, NULL, NULL, before, after, NULL);
