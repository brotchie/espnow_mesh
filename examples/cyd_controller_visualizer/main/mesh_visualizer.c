#include "mesh_visualizer.h"

#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "cyd_4in_lvgl.h"
#include "esp_log.h"
#include "esp_system.h"
#include "espnow_mesh.h"
#include "lvgl.h"

static const char *TAG = "mesh_visualizer";

#define VISUALIZER_MAX_ROWS 5

typedef struct {
    lv_obj_t *box;
    lv_obj_t *mac;
    lv_obj_t *state;
    lv_obj_t *stats;
} satellite_row_t;

typedef struct {
    lv_display_t *display;
    lv_obj_t *title;
    lv_obj_t *controller;
    lv_obj_t *sequence;
    lv_obj_t *clock;
    lv_obj_t *heap;
    lv_obj_t *empty;
    satellite_row_t rows[VISUALIZER_MAX_ROWS];
} mesh_visualizer_ui_t;

static mesh_visualizer_ui_t s_ui;

static void mac_to_string(const uint8_t mac[6], char *out, size_t out_len)
{
    snprintf(out, out_len, "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

static void age_to_string(uint32_t age_ms, char *out, size_t out_len)
{
    if (age_ms == UINT32_MAX) {
        snprintf(out, out_len, "never");
    } else if (age_ms < 1000) {
        snprintf(out, out_len, "%" PRIu32 "ms", age_ms);
    } else if (age_ms < 60000) {
        uint32_t tenths = age_ms / 100;
        snprintf(out, out_len, "%" PRIu32 ".%" PRIu32 "s", tenths / 10, tenths % 10);
    } else {
        snprintf(out, out_len, "%" PRIu32 "m", age_ms / 60000);
    }
}

static lv_obj_t *create_label(lv_obj_t *parent, int x, int y, uint32_t color)
{
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_pos(label, x, y);
    lv_obj_set_style_text_color(label, lv_color_hex(color), LV_PART_MAIN);
    lv_obj_set_style_text_letter_space(label, 0, LV_PART_MAIN);
    return label;
}

static void create_satellite_row(lv_obj_t *screen, satellite_row_t *row, int y)
{
    row->box = lv_obj_create(screen);
    lv_obj_set_pos(row->box, 12, y);
    lv_obj_set_size(row->box, 456, 47);
    lv_obj_set_style_radius(row->box, 6, LV_PART_MAIN);
    lv_obj_set_style_bg_color(row->box, lv_color_hex(0x18212b), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(row->box, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_color(row->box, lv_color_hex(0x2c3a45), LV_PART_MAIN);
    lv_obj_set_style_border_width(row->box, 1, LV_PART_MAIN);
    lv_obj_set_style_pad_all(row->box, 0, LV_PART_MAIN);
    lv_obj_clear_flag(row->box, LV_OBJ_FLAG_SCROLLABLE);

    row->mac = create_label(row->box, 10, 5, 0xe8eef4);
    row->state = create_label(row->box, 10, 25, 0x9ca8b3);
    row->stats = create_label(row->box, 286, 15, 0xd9e5ec);
}

static void update_satellite_row(const espnow_mesh_satellite_status_t *sat,
                                 satellite_row_t *row)
{
    char mac[18] = {0};
    char age[24] = {0};
    mac_to_string(sat->mac, mac, sizeof(mac));
    age_to_string(sat->last_seen_ms_ago, age, sizeof(age));

    const char *state = "quiet";
    uint32_t bg = 0x18212b;
    uint32_t border = 0x2c3a45;
    if (sat->last_seen_ms_ago < 3000) {
        state = sat->acked_current ? "acked" : "online";
        bg = sat->acked_current ? 0x143226 : 0x172d34;
        border = sat->acked_current ? 0x38c172 : 0x3f8fa3;
    } else if (sat->last_seen_ms_ago < 10000) {
        state = "late";
        bg = 0x332b16;
        border = 0xc9952f;
    } else {
        state = "stale";
        bg = 0x341b21;
        border = 0xb94a58;
    }

    lv_obj_clear_flag(row->box, LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_style_bg_color(row->box, lv_color_hex(bg), LV_PART_MAIN);
    lv_obj_set_style_border_color(row->box, lv_color_hex(border), LV_PART_MAIN);
    lv_label_set_text_fmt(row->mac, "%s", mac);
    lv_label_set_text_fmt(row->state, "%s  seen %s  rssi %d",
                          state, age, sat->last_rssi);
    lv_label_set_text_fmt(row->stats, "seq %" PRIu32 "\nacks %" PRIu32 "  tx %" PRIu32,
                          sat->last_sequence, sat->ack_count, sat->sends_current);
}

static void update_ui(lv_timer_t *timer)
{
    (void)timer;

    espnow_mesh_status_t status = {0};
    if (!espnow_mesh_get_status(&status)) {
        return;
    }

    char self_mac[18] = {0};
    mac_to_string(status.self_mac, self_mac, sizeof(self_mac));

    lv_label_set_text(s_ui.title, "ESP-NOW Mesh Controller");
    lv_label_set_text_fmt(s_ui.controller,
                          "%s  ch %u  boot 0x%08" PRIx32,
                          status.initialized ? self_mac : "starting",
                          status.channel,
                          status.boot_id);

    lv_label_set_text_fmt(s_ui.sequence,
                          "active seq %" PRIu32 "  current %" PRIu32 "/%" PRIu32
                          "  last %" PRIu32 "/%" PRIu32 " %s",
                          status.active_sequence,
                          status.acked_current,
                          status.expected_current,
                          status.last_acked,
                          status.last_expected,
                          status.last_sequence_complete ? "ok" : "wait");

    lv_label_set_text_fmt(s_ui.clock,
                          "known %" PRIu32 "  radio %s  clock %s  uptime %" PRIu32 "s",
                          status.known_satellites,
                          status.espnow_ready ? "ready" : "init",
                          status.time_synced ? "synced" : "waiting",
                          status.uptime_ms / 1000);

    lv_label_set_text_fmt(s_ui.heap, "free heap %lu",
                          (unsigned long)esp_get_free_heap_size());

    if (status.satellite_count == 0) {
        lv_obj_clear_flag(s_ui.empty, LV_OBJ_FLAG_HIDDEN);
        lv_label_set_text(s_ui.empty, "Waiting for satellites to register or ACK");
    } else {
        lv_obj_add_flag(s_ui.empty, LV_OBJ_FLAG_HIDDEN);
    }

    for (size_t i = 0; i < VISUALIZER_MAX_ROWS; ++i) {
        if (i < status.satellite_count) {
            update_satellite_row(&status.satellites[i], &s_ui.rows[i]);
        } else {
            lv_obj_add_flag(s_ui.rows[i].box, LV_OBJ_FLAG_HIDDEN);
        }
    }
}

static void create_ui(lv_display_t *display)
{
    memset(&s_ui, 0, sizeof(s_ui));
    s_ui.display = display;

    lv_obj_t *screen = lv_display_get_screen_active(display);
    lv_obj_clear_flag(screen, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(screen, lv_color_hex(0x0f1318), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, LV_PART_MAIN);

    s_ui.title = create_label(screen, 12, 8, 0xf7fbff);
    s_ui.controller = create_label(screen, 12, 34, 0xb9c6d1);
    s_ui.sequence = create_label(screen, 12, 58, 0xd5e1ea);
    s_ui.clock = create_label(screen, 12, 82, 0x98a6b2);
    s_ui.heap = create_label(screen, 350, 8, 0x7f8c99);
    s_ui.empty = create_label(screen, 12, 128, 0x7f8c99);

    lv_obj_t *header = create_label(screen, 12, 107, 0xf7fbff);
    lv_label_set_text(header, "Satellites");

    for (size_t i = 0; i < VISUALIZER_MAX_ROWS; ++i) {
        create_satellite_row(screen, &s_ui.rows[i], 129 + (int)i * 52);
        lv_obj_add_flag(s_ui.rows[i].box, LV_OBJ_FLAG_HIDDEN);
    }

    lv_timer_create(update_ui, 500, NULL);
    update_ui(NULL);
}

esp_err_t mesh_visualizer_start(void)
{
    lv_display_t *display = NULL;
    cyd_4in_lvgl_config_t config = CYD_4IN_LVGL_DEFAULT_CONFIG();
    config.backlight_idle_timeout_ms = 0;

    esp_err_t err = cyd_4in_lvgl_init(&config, &display);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "CYD LVGL init failed: %s", esp_err_to_name(err));
        return err;
    }

    cyd_4in_lvgl_lock();
    create_ui(display);
    cyd_4in_lvgl_unlock();

    ESP_LOGI(TAG, "mesh visualizer ready");
    return ESP_OK;
}
