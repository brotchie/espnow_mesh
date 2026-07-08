#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "espnow_mesh_clock.h"
#include "espnow_mesh_sync_output.h"

#if CONFIG_ESPNOW_MESH_SYNC_OUTPUT_ENABLE || CONFIG_ESPNOW_MESH_HIL_TEST_ENABLE
static const char *TAG = "espnow_mesh";
#endif

#if CONFIG_ESPNOW_MESH_HIL_TEST_ENABLE
static esp_timer_handle_t s_hil_marker_off_timer;
static esp_timer_handle_t s_hil_apply_timer;

static void hil_mark_apply(void);

static void hil_marker_off_timer_cb(void *arg)
{
    (void)arg;
    (void)gpio_set_level((gpio_num_t)CONFIG_ESPNOW_MESH_SYNC_OUTPUT_GPIO, 0);
}

static void hil_apply_timer_cb(void *arg)
{
    (void)arg;
    hil_mark_apply();
}

void init_hil_test(void)
{
    const esp_timer_create_args_t marker_off_args = {
        .callback = hil_marker_off_timer_cb,
        .name = "hil_mark_off",
    };
    ESP_ERROR_CHECK(esp_timer_create(&marker_off_args, &s_hil_marker_off_timer));

    const esp_timer_create_args_t apply_args = {
        .callback = hil_apply_timer_cb,
        .name = "hil_apply",
    };
    ESP_ERROR_CHECK(esp_timer_create(&apply_args, &s_hil_apply_timer));
}

static void hil_mark_apply(void)
{
    (void)esp_timer_stop(s_hil_marker_off_timer);
    (void)gpio_set_level((gpio_num_t)CONFIG_ESPNOW_MESH_SYNC_OUTPUT_GPIO, 1);
    esp_err_t err =
        esp_timer_start_once(s_hil_marker_off_timer, CONFIG_ESPNOW_MESH_HIL_MARK_PULSE_US);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "HIL marker off schedule failed: %s", esp_err_to_name(err));
    }
}

void hil_schedule_apply_marker(int64_t apply_at_mesh_us)
{
    int64_t mesh_now_us = 0;
    bool ready = shared_clock_get_time_us(&mesh_now_us);
    int64_t delay_us = ready ? apply_at_mesh_us - mesh_now_us : 100000;
    if (delay_us < 100) {
        delay_us = 100;
    }

    (void)esp_timer_stop(s_hil_apply_timer);
    esp_err_t err = esp_timer_start_once(s_hil_apply_timer, (uint64_t)delay_us);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "HIL apply marker schedule failed delay=%" PRId64 "us: %s",
                 delay_us, esp_err_to_name(err));
    }
}
#else
void init_hil_test(void)
{
}
#endif

#if CONFIG_ESPNOW_MESH_SYNC_OUTPUT_ENABLE
static esp_timer_handle_t s_sync_output_timer;

static void sync_output_schedule_us(int64_t delay_us)
{
    if (delay_us < 100) {
        delay_us = 100;
    }

    esp_err_t err = esp_timer_start_once(s_sync_output_timer, (uint64_t)delay_us);
    if (err == ESP_ERR_INVALID_STATE) {
        (void)esp_timer_stop(s_sync_output_timer);
        err = esp_timer_start_once(s_sync_output_timer, (uint64_t)delay_us);
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "sync output timer schedule failed: %s", esp_err_to_name(err));
    }
}

static void sync_output_timer_cb(void *arg)
{
    (void)arg;

    int64_t mesh_now_us = 0;
    bool clock_ready = shared_clock_get_time_us(&mesh_now_us);
    if (!clock_ready && CONFIG_ESPNOW_MESH_SYNC_OUTPUT_REQUIRE_SYNC) {
        (void)gpio_set_level((gpio_num_t)CONFIG_ESPNOW_MESH_SYNC_OUTPUT_GPIO, 0);
        sync_output_schedule_us(10000);
        return;
    }

    const int64_t period_us = CONFIG_ESPNOW_MESH_SYNC_OUTPUT_PERIOD_US;
    const int64_t high_us = CONFIG_ESPNOW_MESH_SYNC_OUTPUT_HIGH_US;
    int64_t phase_us = mesh_now_us % period_us;
    if (phase_us < 0) {
        phase_us += period_us;
    }

    int level = phase_us < high_us ? 1 : 0;
    (void)gpio_set_level((gpio_num_t)CONFIG_ESPNOW_MESH_SYNC_OUTPUT_GPIO, level);

    int64_t delay_to_transition_us = level ? (high_us - phase_us) : (period_us - phase_us);
    sync_output_schedule_us(delay_to_transition_us);
}

void init_sync_output(void)
{
    if (CONFIG_ESPNOW_MESH_SYNC_OUTPUT_HIGH_US >= CONFIG_ESPNOW_MESH_SYNC_OUTPUT_PERIOD_US) {
        ESP_LOGE(TAG, "sync output high time must be less than the period");
        ESP_ERROR_CHECK(ESP_ERR_INVALID_ARG);
    }
    if (!GPIO_IS_VALID_OUTPUT_GPIO(CONFIG_ESPNOW_MESH_SYNC_OUTPUT_GPIO)) {
        ESP_LOGE(TAG, "sync output GPIO %d is not a valid output pin",
                 CONFIG_ESPNOW_MESH_SYNC_OUTPUT_GPIO);
        ESP_ERROR_CHECK(ESP_ERR_INVALID_ARG);
    }

    gpio_config_t io_config = {
        .pin_bit_mask = 1ULL << CONFIG_ESPNOW_MESH_SYNC_OUTPUT_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_config));
    ESP_ERROR_CHECK(gpio_set_level((gpio_num_t)CONFIG_ESPNOW_MESH_SYNC_OUTPUT_GPIO, 0));

    const esp_timer_create_args_t timer_args = {
        .callback = sync_output_timer_cb,
        .name = "sync_gpio",
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &s_sync_output_timer));

    ESP_LOGI(TAG,
             "device_id=%d sync output gpio=%d period=%dus high=%dus require_sync=%d",
             CONFIG_ESPNOW_MESH_DEVICE_ID, CONFIG_ESPNOW_MESH_SYNC_OUTPUT_GPIO,
             CONFIG_ESPNOW_MESH_SYNC_OUTPUT_PERIOD_US, CONFIG_ESPNOW_MESH_SYNC_OUTPUT_HIGH_US,
             CONFIG_ESPNOW_MESH_SYNC_OUTPUT_REQUIRE_SYNC ? 1 : 0);
#if CONFIG_ESPNOW_MESH_HIL_TEST_ENABLE
    ESP_LOGI(TAG, "HIL test mode owns sync output GPIO for apply markers");
#else
    sync_output_schedule_us(100);
#endif
}
#else
void init_sync_output(void)
{
}
#endif
