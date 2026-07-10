#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

#include "espnow_mesh_clock.h"
#include "espnow_mesh_priv.h"
#include "espnow_mesh_time_sync.h"

#if CONFIG_ESPNOW_MESH_ROLE_CONTROLLER

bool shared_clock_get_time_us(int64_t *mesh_time_us)
{
    *mesh_time_us = (int64_t)now_us();
    return true;
}

#else /* satellite: estimate the controller clock from time-sync samples */

static portMUX_TYPE s_mesh_time_lock = portMUX_INITIALIZER_UNLOCKED;
static bool s_have_mesh_time;
static int64_t s_mesh_time_offset_us;
#if CONFIG_ESPNOW_MESH_TIME_SYNC_PLL_ENABLE
static int64_t s_pll_anchor_local_us;
static int64_t s_pll_anchor_mesh_us;
static int32_t s_pll_freq_ppb;
static int64_t s_pll_best_delay_us;
static uint32_t s_pll_accepted_samples;
static uint32_t s_pll_consecutive_rejects;
#endif

bool mesh_have_time(void)
{
    portENTER_CRITICAL(&s_mesh_time_lock);
    bool have = s_have_mesh_time;
    portEXIT_CRITICAL(&s_mesh_time_lock);
    return have;
}

#if CONFIG_ESPNOW_MESH_TIME_SYNC_PLL_ENABLE
static int64_t abs_i64(int64_t value)
{
    return value < 0 ? -value : value;
}

static int32_t clamp_i64_to_i32(int64_t value, int32_t min_value, int32_t max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return (int32_t)value;
}

static int64_t pll_predict_mesh_time_us(int64_t local_time_us, int64_t anchor_local_us,
                                        int64_t anchor_mesh_us, int32_t freq_ppb)
{
    int64_t local_delta_us = local_time_us - anchor_local_us;
    int64_t freq_adjust_us = (local_delta_us * (int64_t)freq_ppb) / 1000000000LL;
    return anchor_mesh_us + local_delta_us + freq_adjust_us;
}

static void pll_reset_locked(int64_t sample_local_us, int64_t sample_mesh_us,
                             int64_t sample_offset_us, int64_t delay_us)
{
    s_have_mesh_time = true;
    s_mesh_time_offset_us = sample_offset_us;
    s_pll_anchor_local_us = sample_local_us;
    s_pll_anchor_mesh_us = sample_mesh_us;
    s_pll_freq_ppb = 0;
    s_pll_best_delay_us = delay_us;
    s_pll_accepted_samples = 1;
    s_pll_consecutive_rejects = 0;
}

pll_update_result_t pll_update_from_sample(int64_t sample_local_us,
                                           int64_t sample_mesh_us,
                                           int64_t sample_offset_us,
                                           int64_t delay_us)
{
    pll_update_result_t result = { 0 };

    portENTER_CRITICAL(&s_mesh_time_lock);

    bool reacquire = !s_have_mesh_time ||
                     s_pll_consecutive_rejects >= CONFIG_ESPNOW_MESH_TIME_SYNC_PLL_RESET_AFTER_REJECTS;
    if (reacquire) {
        pll_reset_locked(sample_local_us, sample_mesh_us, sample_offset_us, delay_us);
        result.accepted = true;
        result.reacquired = true;
        result.offset_us = s_mesh_time_offset_us;
        result.best_delay_us = s_pll_best_delay_us;
        result.freq_ppb = s_pll_freq_ppb;
        result.accepted_samples = s_pll_accepted_samples;
        portEXIT_CRITICAL(&s_mesh_time_lock);
        return result;
    }

    if (delay_us < s_pll_best_delay_us) {
        s_pll_best_delay_us = delay_us;
    }

    int64_t predicted_mesh_us =
        pll_predict_mesh_time_us(sample_local_us, s_pll_anchor_local_us,
                                 s_pll_anchor_mesh_us, s_pll_freq_ppb);
    int64_t residual_us = sample_mesh_us - predicted_mesh_us;
    bool startup = s_pll_accepted_samples < CONFIG_ESPNOW_MESH_TIME_SYNC_PLL_LOCK_SAMPLES;
    bool delay_outlier =
        !startup &&
        delay_us > s_pll_best_delay_us + CONFIG_ESPNOW_MESH_TIME_SYNC_PLL_DELAY_MARGIN_US;
    bool residual_outlier =
        !startup && abs_i64(residual_us) > CONFIG_ESPNOW_MESH_TIME_SYNC_PLL_OUTLIER_US;

    result.residual_us = residual_us;
    result.best_delay_us = s_pll_best_delay_us;
    result.freq_ppb = s_pll_freq_ppb;
    result.accepted_samples = s_pll_accepted_samples;

    if (delay_outlier || residual_outlier) {
        s_pll_consecutive_rejects++;
        result.reject_reason = delay_outlier ? "delay" : "residual";
        result.consecutive_rejects = s_pll_consecutive_rejects;
        portEXIT_CRITICAL(&s_mesh_time_lock);
        return result;
    }

    int64_t phase_divisor = 1LL << CONFIG_ESPNOW_MESH_TIME_SYNC_PLL_PHASE_GAIN_SHIFT;
    int64_t phase_step_us = residual_us / phase_divisor;
    int64_t corrected_mesh_us = predicted_mesh_us + phase_step_us;

    int64_t local_delta_us = sample_local_us - s_pll_anchor_local_us;
    if (local_delta_us > 0) {
        int64_t freq_error_ppb = (residual_us * 1000000000LL) / local_delta_us;
        int64_t freq_divisor = 1LL << CONFIG_ESPNOW_MESH_TIME_SYNC_PLL_FREQ_GAIN_SHIFT;
        int64_t freq_adjust_ppb = freq_error_ppb / freq_divisor;
        int64_t next_freq_ppb = (int64_t)s_pll_freq_ppb + freq_adjust_ppb;
        s_pll_freq_ppb =
            clamp_i64_to_i32(next_freq_ppb, -CONFIG_ESPNOW_MESH_TIME_SYNC_PLL_MAX_FREQ_PPB,
                             CONFIG_ESPNOW_MESH_TIME_SYNC_PLL_MAX_FREQ_PPB);
    }

    s_pll_anchor_local_us = sample_local_us;
    s_pll_anchor_mesh_us = corrected_mesh_us;
    s_mesh_time_offset_us = corrected_mesh_us - sample_local_us;
    s_pll_accepted_samples++;
    s_pll_consecutive_rejects = 0;

    result.accepted = true;
    result.phase_step_us = phase_step_us;
    result.offset_us = s_mesh_time_offset_us;
    result.best_delay_us = s_pll_best_delay_us;
    result.freq_ppb = s_pll_freq_ppb;
    result.accepted_samples = s_pll_accepted_samples;
    portEXIT_CRITICAL(&s_mesh_time_lock);

    return result;
}

bool shared_clock_get_time_us(int64_t *mesh_time_us)
{
    int64_t local_time_us = (int64_t)now_us();
    bool have_mesh_time = false;
    int64_t anchor_local_us = 0;
    int64_t anchor_mesh_us = 0;
    int32_t freq_ppb = 0;

    portENTER_CRITICAL(&s_mesh_time_lock);
    have_mesh_time = s_have_mesh_time;
    anchor_local_us = s_pll_anchor_local_us;
    anchor_mesh_us = s_pll_anchor_mesh_us;
    freq_ppb = s_pll_freq_ppb;
    portEXIT_CRITICAL(&s_mesh_time_lock);

    *mesh_time_us = pll_predict_mesh_time_us(local_time_us, anchor_local_us, anchor_mesh_us,
                                             freq_ppb);
    return have_mesh_time;
}
#else /* satellite, PLL disabled: filtered offset */
int64_t time_sync_apply_offset(int64_t sample_offset_us)
{
    int64_t filtered_offset_us;
    portENTER_CRITICAL(&s_mesh_time_lock);
    if (!s_have_mesh_time || CONFIG_ESPNOW_MESH_TIME_SYNC_FILTER_SHIFT == 0) {
        s_mesh_time_offset_us = sample_offset_us;
    } else {
        int64_t divisor = 1LL << CONFIG_ESPNOW_MESH_TIME_SYNC_FILTER_SHIFT;
        s_mesh_time_offset_us =
            ((s_mesh_time_offset_us * (divisor - 1)) + sample_offset_us) / divisor;
    }
    s_have_mesh_time = true;
    filtered_offset_us = s_mesh_time_offset_us;
    portEXIT_CRITICAL(&s_mesh_time_lock);
    return filtered_offset_us;
}

bool shared_clock_get_time_us(int64_t *mesh_time_us)
{
    int64_t local_time_us = (int64_t)now_us();
    bool have_mesh_time = false;
    int64_t offset_us = 0;

    portENTER_CRITICAL(&s_mesh_time_lock);
    have_mesh_time = s_have_mesh_time;
    offset_us = s_mesh_time_offset_us;
    portEXIT_CRITICAL(&s_mesh_time_lock);

    *mesh_time_us = local_time_us + offset_us;
    return have_mesh_time;
}
#endif /* CONFIG_ESPNOW_MESH_TIME_SYNC_PLL_ENABLE */

#endif /* CONFIG_ESPNOW_MESH_ROLE_CONTROLLER */

int64_t mesh_time_us(void)
{
    int64_t shared_us = 0;
    (void)shared_clock_get_time_us(&shared_us);
    return shared_us;
}
