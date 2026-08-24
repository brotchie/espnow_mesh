#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Pass as espnow_mesh_config_t.task_core_id to leave the task unpinned. */
#define ESPNOW_MESH_TASK_NO_AFFINITY (-1)

/* Capacity of espnow_mesh_status_t.satellites; the controller may know more
 * satellites than this (see known_satellites vs satellite_count below). */
#define ESPNOW_MESH_STATUS_MAX_SATELLITES 19

/* Role this image was compiled for (set at build time via Kconfig). */
typedef enum {
    ESPNOW_MESH_ROLE_CONTROLLER_RUNTIME = 1,
    ESPNOW_MESH_ROLE_SATELLITE_RUNTIME = 2,
} espnow_mesh_role_runtime_t;

/* FreeRTOS task parameters for espnow_mesh_start(). Use ESPNOW_MESH_DEFAULT_CONFIG()
 * for sensible defaults and override individual fields as needed. */
typedef struct {
    const char *task_name;      /* task name (must be non-NULL) */
    uint32_t task_stack_bytes;  /* stack depth in bytes (must be non-zero) */
    uint32_t task_priority;     /* FreeRTOS priority (must be non-zero) */
    int task_core_id;           /* core to pin to, or ESPNOW_MESH_TASK_NO_AFFINITY */
    bool use_existing_wifi;     /* application already initialized and started Wi-Fi */
    bool adopt_current_wifi_channel; /* use the active Wi-Fi channel for ESP-NOW */
} espnow_mesh_config_t;

/* Per-satellite view within espnow_mesh_status_t (controller role). */
typedef struct {
    bool used;              /* slot occupied */
    bool expected_current;  /* counted toward the active sequence's ACK quorum */
    bool acked_current;     /* has ACKed the active sequence */
    uint8_t mac[6];         /* satellite station MAC */
    uint32_t last_seen_ms_ago;  /* ms since last frame from it; UINT32_MAX = never */
    uint32_t last_sequence;     /* last sequence it ACKed */
    uint32_t ack_count;         /* total ACKs received from it since boot */
    uint32_t sends_current;     /* send attempts to it during the active sequence */
    int8_t last_rssi;           /* RSSI reported in its last ACK */
} espnow_mesh_satellite_status_t;

/*
 * Snapshot of mesh state for dashboards/visualizers (see espnow_mesh_get_status()).
 * Fields marked (controller) are only populated in a controller build.
 */
typedef struct {
    espnow_mesh_role_runtime_t role;  /* compiled role */
    bool initialized;                 /* espnow_mesh_run() has completed init */
    bool espnow_ready;                /* ESP-NOW stack is up */
    bool time_synced;                 /* shared mesh clock is usable (see get_time_us) */
    uint8_t self_mac[6];              /* this node's station MAC */
    uint8_t channel;                  /* current Wi-Fi channel */
    uint32_t boot_id;                 /* random per-boot session id used for dedup */
    uint32_t uptime_ms;               /* ms since boot */
    uint32_t event_drops;             /* internal events dropped on a full queue since boot */

    /* Reliable-delivery counters. "current" = the in-flight sequence; "last" =
     * the most recently completed one. */
    uint32_t active_sequence;         /* (controller) sequence currently being delivered */
    uint32_t last_sequence;           /* last sequence completed (controller) / seen (satellite) */
    uint32_t known_satellites;        /* satellites in the table (may exceed satellite_count) */
    uint32_t expected_current;        /* (controller) satellites expected to ACK active_sequence */
    uint32_t acked_current;           /* (controller) how many have ACKed it so far */
    uint32_t last_expected;           /* (controller) expected count for the last sequence */
    uint32_t last_acked;              /* (controller) ACK count for the last sequence */
    bool last_sequence_complete;      /* (controller) last sequence fully ACKed */

    size_t satellite_count;           /* valid entries in satellites[] (<= array capacity) */
    espnow_mesh_satellite_status_t satellites[ESPNOW_MESH_STATUS_MAX_SATELLITES];
} espnow_mesh_status_t;

#define ESPNOW_MESH_DEFAULT_CONFIG()                   \
    {                                                  \
        .task_name = "espnow_mesh",                    \
        .task_stack_bytes = 8192,                      \
        .task_priority = 5,                            \
        .task_core_id = ESPNOW_MESH_TASK_NO_AFFINITY,  \
        .use_existing_wifi = false,                    \
        .adopt_current_wifi_channel = false,           \
    }

/*
 * Satellite receive callback. Invoked on the mesh task for the first accepted
 * copy of each controller sequence (duplicates are suppressed). Do not block;
 * copy what you need. `payload` points to `len` bytes (len may be 0) and is
 * valid only for the duration of the call.
 */
typedef void (*espnow_mesh_rx_cb_t)(const uint8_t *payload, size_t len, void *user_ctx);

/* The role this firmware was compiled for. */
espnow_mesh_role_runtime_t espnow_mesh_role(void);

/*
 * Start the mesh engine in its own FreeRTOS task and return immediately. Pass
 * NULL for ESPNOW_MESH_DEFAULT_CONFIG(). Returns ESP_ERR_INVALID_STATE if
 * already started, ESP_ERR_INVALID_ARG for a bad config, or ESP_ERR_NO_MEM if
 * the task could not be created. This is the usual entry point.
 */
esp_err_t espnow_mesh_start(const espnow_mesh_config_t *config);

/*
 * Run the mesh engine on the calling task. This performs one-time init and then
 * blocks forever (controller/satellite loop), so it never returns on success;
 * use it instead of espnow_mesh_start() when you want to own the task yourself.
 */
esp_err_t espnow_mesh_run(void);

/*
 * Read the shared mesh time base into *mesh_time_us (microseconds). This is a
 * monotonic time base shared across the mesh, NOT UTC/wall-clock. On the
 * controller it is local monotonic time and this always returns true. On a
 * satellite it is the synchronized estimate of controller time; returns false
 * until the first time-sync sample is accepted, though *mesh_time_us is still
 * written with the best available estimate. Safe to call from any task.
 */
bool espnow_mesh_get_time_us(int64_t *mesh_time_us);

/* Whether the shared mesh clock is usable (always true on the controller;
 * true on a satellite once time sync has locked). Safe to call from any task. */
bool espnow_mesh_is_time_synced(void);

/*
 * Snapshot mesh state for dashboards and visualizers. Safe to call from any
 * task. The snapshot is taken without stopping the mesh task, so it is
 * eventually consistent: individual counters may lag the mesh task by one
 * update, but satellite entries are always seen fully initialized.
 */
bool espnow_mesh_get_status(espnow_mesh_status_t *status);

/*
 * Controller: queue a payload for reliable fan-out to all known satellites
 * (broadcast plus per-satellite ACK tracking and unicast retries). Fire and
 * forget: returns ESP_OK once queued, ESP_ERR_NO_MEM if a previously queued
 * payload has not been sent yet (apply backpressure and retry),
 * ESP_ERR_INVALID_SIZE if len exceeds CONFIG_ESPNOW_MESH_PAYLOAD_BYTES, or
 * ESP_ERR_NOT_SUPPORTED on satellite or HIL-test builds. len may be 0. Safe to
 * call from any task. Delivery outcome is observable via espnow_mesh_get_status()
 * (last_sequence / last_sequence_complete).
 */
esp_err_t espnow_mesh_send(const void *payload, size_t len);

/*
 * Satellite: register the receive callback (or clear it with NULL). Returns
 * ESP_ERR_NOT_SUPPORTED on controller builds. Set it before espnow_mesh_start()
 * to avoid missing early frames.
 */
esp_err_t espnow_mesh_set_rx_callback(espnow_mesh_rx_cb_t cb, void *user_ctx);

#ifdef __cplusplus
}
#endif
