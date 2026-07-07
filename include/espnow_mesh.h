#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ESPNOW_MESH_TASK_NO_AFFINITY (-1)
#define ESPNOW_MESH_STATUS_MAX_SATELLITES 19

typedef enum {
    ESPNOW_MESH_ROLE_CONTROLLER_RUNTIME = 1,
    ESPNOW_MESH_ROLE_SATELLITE_RUNTIME = 2,
} espnow_mesh_role_runtime_t;

typedef struct {
    const char *task_name;
    uint32_t task_stack_bytes;
    uint32_t task_priority;
    int task_core_id;
} espnow_mesh_config_t;

typedef struct {
    bool used;
    bool expected_current;
    bool acked_current;
    uint8_t mac[6];
    uint32_t last_seen_ms_ago;
    uint32_t last_sequence;
    uint32_t ack_count;
    uint32_t sends_current;
    int8_t last_rssi;
} espnow_mesh_satellite_status_t;

typedef struct {
    espnow_mesh_role_runtime_t role;
    bool initialized;
    bool espnow_ready;
    bool time_synced;
    uint8_t self_mac[6];
    uint8_t channel;
    uint32_t boot_id;
    uint32_t uptime_ms;

    uint32_t active_sequence;
    uint32_t last_sequence;
    uint32_t known_satellites;
    uint32_t expected_current;
    uint32_t acked_current;
    uint32_t last_expected;
    uint32_t last_acked;
    bool last_sequence_complete;

    size_t satellite_count;
    espnow_mesh_satellite_status_t satellites[ESPNOW_MESH_STATUS_MAX_SATELLITES];
} espnow_mesh_status_t;

#define ESPNOW_MESH_DEFAULT_CONFIG()                   \
    {                                                  \
        .task_name = "espnow_mesh",                    \
        .task_stack_bytes = 8192,                      \
        .task_priority = 5,                            \
        .task_core_id = ESPNOW_MESH_TASK_NO_AFFINITY,  \
    }

espnow_mesh_role_runtime_t espnow_mesh_role(void);

esp_err_t espnow_mesh_start(const espnow_mesh_config_t *config);
esp_err_t espnow_mesh_run(void);

bool espnow_mesh_get_time_us(int64_t *mesh_time_us);
bool espnow_mesh_is_time_synced(void);

/*
 * Snapshot mesh state for dashboards and visualizers. Safe to call from any
 * task. The snapshot is taken without stopping the mesh task, so it is
 * eventually consistent: individual counters may lag the mesh task by one
 * update, but satellite entries are always seen fully initialized.
 */
bool espnow_mesh_get_status(espnow_mesh_status_t *status);

#ifdef __cplusplus
}
#endif
