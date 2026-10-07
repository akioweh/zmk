/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */
#include <errno.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zmk/ptp/split.h>
#include <zmk/ptp/queue.h>
#include <zephyr/sys/byteorder.h>

/* A lease detects a lost final lift, including wired links without detect GPIO.
 * Only active snapshots need continuous heartbeats; idle snapshots get two retries. */
#define HEARTBEAT_MS 50
#define LEASE_MS 300

#if IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)

struct queued_frame {
    struct zmk_ptp_queue_header header;
    struct zmk_ptp_frame frame;
    uint32_t epoch;
};
K_MSGQ_DEFINE(frames, sizeof(struct queued_frame), CONFIG_ZMK_TRACKPAD_SPLIT_QUEUE_SIZE, 4);
static struct k_spinlock lock;
static uint32_t epoch;
static uint16_t sequence;
static bool sequence_valid, cancel_pending;
static int64_t expires;
static struct zmk_ptp_split_frame latest;
static uint32_t ticket;
static uint16_t latest_state;
static void forward(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(forward_work, forward);

/* Called under lock. Epochs also invalidate a worker's already-dequeued frame. */
static void reset(void) {
    cancel_pending |= sequence_valid;
    sequence_valid = false;
    expires = 0;
    epoch++;
    k_msgq_purge(&frames);
}

void zmk_ptp_split_reset(void) {
    k_spinlock_key_t key = k_spin_lock(&lock);
    reset();
    k_spin_unlock(&lock, key);
    k_work_reschedule(&forward_work, K_NO_WAIT);
}

bool zmk_ptp_split_epoch_is_current(uint32_t expected) {
    k_spinlock_key_t key = k_spin_lock(&lock);
    bool current = epoch == expected;
    k_spin_unlock(&lock, key);
    return current;
}

int zmk_ptp_split_receive(uint8_t source, const struct zmk_ptp_split_frame *wire) {
    /* ponytail: one logical touchpad; separate HID collections if multiple pads are needed. */
    if (source != CONFIG_ZMK_TRACKPAD_SPLIT_SOURCE) {
        return -ENODEV;
    }
    struct queued_frame item = {0}, scratch;
    uint16_t next;
    int err = zmk_ptp_split_decode(wire, &next, &item.frame);
    if (err) {
        return err;
    }
    uint16_t state = zmk_ptp_frame_state(&item.frame);
    k_spinlock_key_t key = k_spin_lock(&lock);
    bool motion = sequence_valid && state == latest_state;
    if (sequence_valid) {
        int16_t delta = (int16_t)(next - sequence);
        if (!delta) {
            /* Coverage may differ between a coalesced packet and its heartbeat. */
            err = (wire->scan_time != latest.scan_time ||
                   wire->count_buttons != latest.count_buttons ||
                   memcmp(wire->contacts, latest.contacts, sizeof(latest.contacts)))
                      ? -EINVAL
                      : 0;
            if (!err) {
                expires = k_uptime_get() + LEASE_MS;
            }
            k_spin_unlock(&lock, key);
            return err; /* Heartbeat, not another host input report. */
        }
        if (delta < 0) {
            k_spin_unlock(&lock, key);
            return -ESTALE;
        }
        uint16_t covered = next - sys_le16_to_cpu(wire->first_sequence);
        if (delta != 1 && (!motion || (uint16_t)(next - (sequence + 1)) > covered)) {
            reset();
            motion = false;
        }
    }
    item.header =
        (struct zmk_ptp_queue_header){.ticket = ++ticket, .state = state, .motion = motion};
    item.epoch = epoch;
    if (zmk_ptp_queue_put(&frames, &item, &scratch) < 0) {
        /* Drop complete frames only. Cancel the old lifetime before recovery. */
        reset();
        item.epoch = epoch;
        item.header.motion = false;
        k_msgq_put(&frames, &item, K_NO_WAIT);
    }
    latest_state = state;
    sequence = next;
    sequence_valid = true;
    latest = *wire;
    expires = k_uptime_get() + LEASE_MS;
    k_spin_unlock(&lock, key);
    k_work_reschedule(&forward_work, K_NO_WAIT);
    return 0;
}

static void forward(struct k_work *work) {
    while (true) {
        k_spinlock_key_t key = k_spin_lock(&lock);
        if (expires && k_uptime_get() >= expires) {
            reset();
        }
        uint32_t current_epoch = epoch;
        bool cancel = cancel_pending;
        struct queued_frame item, scratch;
        bool ready = !cancel && k_msgq_peek(&frames, &item) == 0;
        int64_t deadline = expires;
        k_spin_unlock(&lock, key);
        if (cancel) {
            int err = zmk_ptp_cancel();
            if (err && err != -ENODEV) {
                k_work_schedule(&forward_work, K_MSEC(5));
                return;
            }
            key = k_spin_lock(&lock);
            if (epoch == current_epoch) {
                cancel_pending = false;
            }
            k_spin_unlock(&lock, key);
            continue;
        }
        if (!ready) {
            if (deadline) {
                k_work_schedule(&forward_work, K_MSEC(MAX(1, deadline - k_uptime_get())));
            }
            return;
        }
        int err = zmk_ptp_submit_split_frame(&item.frame, item.epoch);
        if (!err || err == -ESTALE) {
            key = k_spin_lock(&lock);
            zmk_ptp_queue_get(&frames, item.header.ticket, &scratch);
            k_spin_unlock(&lock, key);
            continue;
        }
        int delay = (err == -ENODEV || err == -EMSGSIZE) ? HEARTBEAT_MS : 5;
        k_work_schedule(&forward_work, K_MSEC(delay));
        return;
    }
}

#else

#include <zmk/split/peripheral.h>

static K_MUTEX_DEFINE(lock);
static struct zmk_ptp_frame latest;
static uint16_t sequence;
static bool seen;
static uint8_t idle_retries;
static void heartbeat(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(heartbeat_work, heartbeat);

static int send(void) {
    struct zmk_split_transport_peripheral_event event = {
        .type = ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_CONTACT_FRAME,
    };
    zmk_ptp_split_encode(&event.data.contact_frame, sequence, &latest);
    return zmk_split_peripheral_report_event(&event);
}

int zmk_ptp_submit_frame(const struct zmk_ptp_frame *frame) {
    int err = zmk_ptp_validate_frame(frame);
    if (err) {
        return err;
    }
    zmk_ptp_note_activity(frame);
    if (k_mutex_lock(&lock, K_NO_WAIT)) {
        return -EAGAIN;
    }
    /* Cache physical observations even offline: never replay a held finger
     * after the producer has observed its lift. Failed admissions create gaps. */
    /* Compare canonical wire bytes, not C structs with padding. A retry of
     * the same observation retains its identity even after failed admission. */
    struct zmk_ptp_split_frame previous, next;
    zmk_ptp_split_encode(&previous, sequence, &latest);
    zmk_ptp_split_encode(&next, sequence, frame);
    if (!seen || memcmp(&previous, &next, sizeof(next))) {
        sequence++;
    }
    latest = *frame;
    seen = true;
    idle_retries = 2;
    err = send();
    k_mutex_unlock(&lock);
    k_work_reschedule(&heartbeat_work, K_MSEC(HEARTBEAT_MS));
    return err;
}

int zmk_ptp_release(void) {
    struct zmk_ptp_frame empty = {.scan_time = zmk_ptp_scan_time()};
    return zmk_ptp_submit_frame(&empty);
}

void zmk_ptp_split_resume(void) { k_work_reschedule(&heartbeat_work, K_NO_WAIT); }

static void heartbeat(struct k_work *work) {
    if (k_mutex_lock(&lock, K_NO_WAIT)) {
        k_work_reschedule(&heartbeat_work, K_MSEC(5));
        return;
    }
    bool retry = false;
    if (seen) {
        int err = send();
        retry = latest.contact_count || latest.buttons;
        if (!retry && !err && idle_retries) {
            retry = --idle_retries > 0;
        }
        /* An offline idle source resumes on transport status/CCC changes. */
        if (!retry && err && err != -ENODEV && err != -EACCES) {
            retry = true;
        }
    }
    k_mutex_unlock(&lock);
    if (retry) {
        k_work_reschedule(&heartbeat_work, K_MSEC(HEARTBEAT_MS));
    }
}

#endif
