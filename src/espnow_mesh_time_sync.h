#pragma once

/*
 * Time-sync estimator: turns delay-corrected NTP-style samples into the local
 * estimate of controller monotonic time. Implements the clock read interface in
 * espnow_mesh_clock.h. Only the satellite role feeds samples in; the controller
 * clock is plain monotonic time.
 */

#include <stdbool.h>
#include <stdint.h>

#if !CONFIG_ESPNOW_MESH_ROLE_CONTROLLER

/* Whether at least one time-sync sample has been accepted (clock is usable). */
bool mesh_have_time(void);

#if CONFIG_ESPNOW_MESH_TIME_SYNC_PLL_ENABLE
/* Outcome of feeding one sample to the PLL, with fields for caller logging. */
typedef struct {
    bool accepted;
    bool reacquired;
    const char *reject_reason;
    int64_t residual_us;
    int64_t phase_step_us;
    int64_t offset_us;
    int64_t best_delay_us;
    int32_t freq_ppb;
    uint32_t accepted_samples;
    uint32_t consecutive_rejects;
} pll_update_result_t;

/* Feed one time-sync sample to the software PLL. sample_local_us/sample_mesh_us
 * are the midpoint local/controller timestamps; delay_us is the corrected
 * round-trip delay. */
pll_update_result_t pll_update_from_sample(int64_t sample_local_us, int64_t sample_mesh_us,
                                           int64_t sample_offset_us, int64_t delay_us);
#else
/* Blend an offset sample into the filtered clock offset; returns the resulting
 * filtered offset (for logging). */
int64_t time_sync_apply_offset(int64_t sample_offset_us);
#endif

#endif /* !CONFIG_ESPNOW_MESH_ROLE_CONTROLLER */
