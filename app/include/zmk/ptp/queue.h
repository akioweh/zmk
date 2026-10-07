/*
 * Copyright (c) 2026 The ZMK Contributors
 * SPDX-License-Identifier: MIT
 */
#pragma once

#include <zephyr/kernel.h>

/* Every item starts with this header. Callers serialize all queue operations.
 * Transition entries are immutable; only adjacent motion in the same state
 * may replace the tail. Tickets protect a head copied by an unlocked sender. */
struct zmk_ptp_queue_header {
    uint32_t ticket;
    uint16_t state;
    uint16_t first_sequence;
    uint16_t sequence; /* Split sender only; zero for queues without wire coverage. */
    bool motion;
};

/* Reuse the bounded msgq rather than its private ring pointers. Rotation is
 * O(capacity), bounded by four entries in the default configuration. Scratch holds
 * the replaced entry on return 1, allowing its connection reference to be freed. */
static inline int zmk_ptp_queue_put(struct k_msgq *queue, void *item, void *scratch) {
    struct zmk_ptp_queue_header *next = item;
    unsigned int count = k_msgq_num_used_get(queue);
    if (next->motion) {
        for (unsigned int i = 0; i < count; i++) {
            k_msgq_get(queue, scratch, K_NO_WAIT);
            struct zmk_ptp_queue_header *old = scratch;
            if (i == count - 1 && old->motion && old->state == next->state &&
                (uint16_t)(next->sequence - old->sequence) <= 1) {
                next->first_sequence = old->first_sequence;
                k_msgq_put(queue, item, K_NO_WAIT);
                return 1;
            }
            k_msgq_put(queue, scratch, K_NO_WAIT);
        }
    }
    return k_msgq_put(queue, item, K_NO_WAIT);
}

/* A concurrently replaced head is still pending, even if the old copy was
 * admitted successfully. Never dequeue that newer observation by mistake. */
static inline bool zmk_ptp_queue_get(struct k_msgq *queue, uint32_t ticket, void *item) {
    if (k_msgq_peek(queue, item) || ((struct zmk_ptp_queue_header *)item)->ticket != ticket) {
        return false;
    }
    return k_msgq_get(queue, item, K_NO_WAIT) == 0;
}
