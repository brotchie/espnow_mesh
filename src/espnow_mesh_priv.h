#pragma once

/*
 * Internal leaf primitives shared across the espnow_mesh translation units.
 * These are header-inlined so no translation unit owns them and there is no
 * cross-module state: each is a thin wrapper over a system call.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "esp_now.h"
#include "esp_timer.h"

/* Monotonic milliseconds since boot (wraps every ~49 days; compare with
 * (int32_t)(a - b) for wrap-safe ordering). */
static inline uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

/* Monotonic microseconds since boot. */
static inline uint64_t now_us(void)
{
    return (uint64_t)esp_timer_get_time();
}

static inline bool mac_equal(const uint8_t *left, const uint8_t *right)
{
    return memcmp(left, right, ESP_NOW_ETH_ALEN) == 0;
}

/*
 * Serial-number (RFC 1982) duplicate test: true when `seq` is not newer than
 * `last`, evaluated wrap-safe over the 32-bit sequence space so it stays correct
 * across a counter wrap. `have_last` guards the first-ever packet (nothing seen
 * yet is never a duplicate). Correct while the true distance between seq and
 * last stays below 2^31.
 */
static inline bool seq_is_duplicate(bool have_last, uint32_t seq, uint32_t last)
{
    return have_last && (int32_t)(seq - last) <= 0;
}
