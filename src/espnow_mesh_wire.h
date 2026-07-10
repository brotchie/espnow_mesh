#pragma once

/*
 * On-air wire format for the ESP-NOW mesh: protocol constants, message type
 * identifiers, and the packed frame structs exchanged between controller and
 * satellites. Every struct begins with ESPNOW_MESH_HEADER_FIELDS so the header
 * (magic, version, type, mesh id, auth tag) can be parsed uniformly. This
 * header is pure declarations shared by every module.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_now.h"

#define ESPNOW_MESH_MAGIC 0x574f4e45u
#define ESPNOW_MESH_VERSION 1u
#define ESPNOW_MESH_AUTH_TAG_BYTES 8u

#define ESPNOW_MESH_HEADER_FIELDS \
    uint32_t magic;              \
    uint8_t version;             \
    uint8_t type;                \
    uint32_t mesh_id;            \
    uint64_t auth_tag

#if CONFIG_ESPNOW_MESH_HIL_TEST_ENABLE
#ifndef CONFIG_ESPNOW_MESH_HIL_RANDOM_START_CHANNEL_ENABLE
#define CONFIG_ESPNOW_MESH_HIL_RANDOM_START_CHANNEL_ENABLE 0
#endif
#ifndef CONFIG_ESPNOW_MESH_HIL_RANDOM_CHANNEL_MIN
#define CONFIG_ESPNOW_MESH_HIL_RANDOM_CHANNEL_MIN 1
#endif
#ifndef CONFIG_ESPNOW_MESH_HIL_RANDOM_CHANNEL_MAX
#define CONFIG_ESPNOW_MESH_HIL_RANDOM_CHANNEL_MAX 11
#endif
#ifndef CONFIG_ESPNOW_MESH_HIL_REQUIRE_SOFTAP_DISCOVERY
#define CONFIG_ESPNOW_MESH_HIL_REQUIRE_SOFTAP_DISCOVERY 0
#endif
#ifndef CONFIG_ESPNOW_MESH_HIL_PATCH_BYTES
#define CONFIG_ESPNOW_MESH_HIL_PATCH_BYTES 512
#endif
#ifndef CONFIG_ESPNOW_MESH_HIL_PATCH_BLOCK_BYTES
#define CONFIG_ESPNOW_MESH_HIL_PATCH_BLOCK_BYTES 64
#endif
#ifndef CONFIG_ESPNOW_MESH_HIL_PATCH_OFFER_INTERVAL_MS
#define CONFIG_ESPNOW_MESH_HIL_PATCH_OFFER_INTERVAL_MS 120
#endif
#ifndef CONFIG_ESPNOW_MESH_HIL_PATCH_DEADLINE_MS
#define CONFIG_ESPNOW_MESH_HIL_PATCH_DEADLINE_MS 10000
#endif
#endif

/*
 * Protocol overview
 * -----------------
 * All frames share the header (magic, version, type, mesh id, auth tag). The
 * controller stamps a random per-boot `controller_boot_id` and a monotonically
 * increasing `sequence`; satellites dedup on (boot_id, sequence) and reset that
 * state when they see a new boot_id (a controller restart).
 *
 * Reliable fanout (DATA / ACK):
 *   The controller broadcasts DATA for a sequence up to SEND_RETRIES times, then
 *   unicasts to each satellite that has not ACKed, with exponential backoff and
 *   jitter, until every expected satellite ACKs or the deadline passes. Each
 *   satellite ACKs every valid copy it receives (accepted vs duplicate flagged).
 *
 * Time sync (TIME_REQ / TIME_RESP):
 *   A satellite sends TIME_REQ stamped with satellite_tx_us. The controller
 *   replies with controller_rx_us and controller_tx_us. From the four
 *   timestamps the satellite computes round-trip delay and a clock offset,
 *   rejects high-delay samples, and feeds the rest to its PLL/filter to estimate
 *   controller monotonic time. See espnow_mesh_time_sync.c.
 *
 * Patch distribution (HIL only, pull model):
 *   PATCH_OFFER (metadata + hashes) -> satellite requests each missing block via
 *   PATCH_BLOCK_REQ -> controller answers with PATCH_BLOCK -> satellite verifies
 *   the per-block hash, and once all blocks are in, the full-patch hash, then
 *   sends PATCH_READY. After all expected satellites are ready the controller
 *   broadcasts PATCH_APPLY with a mesh timestamp to apply in unison.
 *
 * HIL command path (HIL only): HIL_CMD / HIL_ACK mirror DATA / ACK but carry
 * test-case parameters and deterministic fault-injection knobs.
 */
typedef enum {
    ESPNOW_MESH_MSG_DATA = 1,
    ESPNOW_MESH_MSG_ACK = 2,
    ESPNOW_MESH_MSG_TIME_REQ = 3,
    ESPNOW_MESH_MSG_TIME_RESP = 4,
    ESPNOW_MESH_MSG_HIL_CMD = 5,
    ESPNOW_MESH_MSG_HIL_ACK = 6,
    ESPNOW_MESH_MSG_PATCH_OFFER = 7,
    ESPNOW_MESH_MSG_PATCH_BLOCK_REQ = 8,
    ESPNOW_MESH_MSG_PATCH_BLOCK = 9,
    ESPNOW_MESH_MSG_PATCH_READY = 10,
    ESPNOW_MESH_MSG_PATCH_APPLY = 11,
} espnow_mesh_msg_type_t;

typedef enum {
    ESPNOW_MESH_HIL_CMD_PATCH_SELECT = 1,
    ESPNOW_MESH_HIL_CMD_PARAM_SET = 2,
    ESPNOW_MESH_HIL_CMD_CHANNEL_DISCOVERY = 3,
} espnow_mesh_hil_cmd_kind_t;

typedef struct __attribute__((packed)) {
    ESPNOW_MESH_HEADER_FIELDS;
} espnow_mesh_header_t;

typedef struct __attribute__((packed)) {
    ESPNOW_MESH_HEADER_FIELDS;
    uint16_t payload_len;
    uint32_t sequence;
    uint8_t controller_mac[ESP_NOW_ETH_ALEN];
    uint32_t controller_boot_id;
    uint32_t retry_index;
    uint32_t sent_ms;
    uint8_t payload[CONFIG_ESPNOW_MESH_PAYLOAD_BYTES];
} espnow_mesh_data_msg_t;

typedef struct __attribute__((packed)) {
    ESPNOW_MESH_HEADER_FIELDS;
    uint16_t reserved;
    uint32_t sequence;
    uint32_t controller_boot_id;
    uint8_t node_mac[ESP_NOW_ETH_ALEN];
    uint8_t accepted;
    uint8_t duplicate;
    int8_t last_rssi;
    uint8_t channel;
    uint32_t uptime_ms;
} espnow_mesh_ack_msg_t;

typedef struct __attribute__((packed)) {
    ESPNOW_MESH_HEADER_FIELDS;
    uint8_t reserved;
    uint32_t sequence;
    uint8_t node_mac[ESP_NOW_ETH_ALEN];
    uint64_t satellite_tx_us;
} espnow_mesh_time_req_msg_t;

typedef struct __attribute__((packed)) {
    ESPNOW_MESH_HEADER_FIELDS;
    uint8_t reserved;
    uint32_t sequence;
    uint8_t controller_mac[ESP_NOW_ETH_ALEN];
    uint64_t satellite_tx_us;
    uint64_t controller_rx_us;
    uint64_t controller_tx_us;
} espnow_mesh_time_resp_msg_t;

typedef struct __attribute__((packed)) {
    ESPNOW_MESH_HEADER_FIELDS;
    uint8_t kind;
    uint32_t sequence;
    uint32_t controller_boot_id;
    uint32_t retry_index;
    uint32_t test_case;
    uint32_t command_id;
    uint64_t apply_at_mesh_us;
    uint32_t patch_id;
    uint32_t param_id;
    int32_t param_value;
    uint8_t rx_drop_pct;
    uint8_t ack_drop_pct;
    uint8_t duplicate_send_count;
    uint8_t reserved;
    uint16_t time_resp_drop_pct;
    uint16_t flags;
    uint32_t fault_seed;
} espnow_mesh_hil_cmd_msg_t;

typedef struct __attribute__((packed)) {
    ESPNOW_MESH_HEADER_FIELDS;
    uint8_t kind;
    uint32_t sequence;
    uint32_t controller_boot_id;
    uint32_t test_case;
    uint32_t command_id;
    uint8_t node_mac[ESP_NOW_ETH_ALEN];
    uint8_t accepted;
    uint8_t duplicate;
    int8_t last_rssi;
    uint8_t channel;
    uint32_t uptime_ms;
    uint32_t patch_id;
    uint32_t param_id;
    int32_t param_value;
} espnow_mesh_hil_ack_msg_t;

#if CONFIG_ESPNOW_MESH_HIL_TEST_ENABLE
#define ESPNOW_MESH_HIL_PATCH_MAX_BLOCKS                                             \
    ((CONFIG_ESPNOW_MESH_HIL_PATCH_BYTES + CONFIG_ESPNOW_MESH_HIL_PATCH_BLOCK_BYTES - 1) / \
     CONFIG_ESPNOW_MESH_HIL_PATCH_BLOCK_BYTES)

#if ESPNOW_MESH_HIL_PATCH_MAX_BLOCKS > 32
#error "CONFIG_ESPNOW_MESH_HIL_PATCH_BYTES / CONFIG_ESPNOW_MESH_HIL_PATCH_BLOCK_BYTES must fit in 32 blocks"
#endif

typedef struct __attribute__((packed)) {
    ESPNOW_MESH_HEADER_FIELDS;
    uint16_t reserved;
    uint32_t sequence;
    uint32_t controller_boot_id;
    uint32_t patch_id;
    uint32_t total_len;
    uint16_t block_size;
    uint16_t block_count;
    uint32_t patch_hash;
    uint8_t block_req_drop_pct;
    uint8_t block_drop_pct;
    uint8_t ready_drop_pct;
    uint8_t reserved2;
    uint32_t fault_seed;
} espnow_mesh_patch_offer_msg_t;

typedef struct __attribute__((packed)) {
    ESPNOW_MESH_HEADER_FIELDS;
    uint16_t reserved;
    uint32_t sequence;
    uint32_t controller_boot_id;
    uint32_t patch_id;
    uint16_t block_index;
    uint16_t block_count;
    uint32_t patch_hash;
    uint32_t request_sequence;
    uint8_t block_drop_pct;
    uint8_t reserved2[3];
    uint32_t fault_seed;
} espnow_mesh_patch_block_req_msg_t;

typedef struct __attribute__((packed)) {
    ESPNOW_MESH_HEADER_FIELDS;
    uint16_t payload_len;
    uint32_t sequence;
    uint32_t controller_boot_id;
    uint32_t patch_id;
    uint16_t block_index;
    uint16_t block_count;
    uint32_t block_offset;
    uint32_t total_len;
    uint32_t patch_hash;
    uint32_t block_hash;
    uint32_t request_sequence;
    uint8_t block_drop_pct;
    uint8_t reserved[3];
    uint32_t fault_seed;
    uint8_t payload[CONFIG_ESPNOW_MESH_HIL_PATCH_BLOCK_BYTES];
} espnow_mesh_patch_block_msg_t;

typedef struct __attribute__((packed)) {
    ESPNOW_MESH_HEADER_FIELDS;
    uint16_t reserved;
    uint32_t sequence;
    uint32_t controller_boot_id;
    uint32_t patch_id;
    uint32_t patch_hash;
    uint32_t request_sequence;
    uint8_t node_mac[ESP_NOW_ETH_ALEN];
    uint16_t blocks_received;
    uint32_t total_len;
    uint32_t uptime_ms;
} espnow_mesh_patch_ready_msg_t;

typedef struct __attribute__((packed)) {
    ESPNOW_MESH_HEADER_FIELDS;
    uint16_t reserved;
    uint32_t sequence;
    uint32_t controller_boot_id;
    uint32_t patch_id;
    uint32_t patch_hash;
    uint64_t apply_at_mesh_us;
} espnow_mesh_patch_apply_msg_t;
#endif

_Static_assert(sizeof(espnow_mesh_data_msg_t) <= ESP_NOW_MAX_DATA_LEN,
               "data packets must fit in one ESP-NOW v1 frame");
#if CONFIG_ESPNOW_MESH_HIL_TEST_ENABLE
_Static_assert(sizeof(espnow_mesh_patch_block_msg_t) <= ESP_NOW_MAX_DATA_LEN,
               "patch block packets must fit in one ESP-NOW v1 frame");
#endif

typedef enum {
    ESPNOW_MESH_EVENT_RECV,
    ESPNOW_MESH_EVENT_SEND,
    ESPNOW_MESH_EVENT_REGISTRATION,
} espnow_mesh_event_type_t;

#if CONFIG_ESPNOW_MESH_HIL_TEST_ENABLE
#define ESPNOW_MESH_EVENT_DATA_BYTES                                       \
    ((sizeof(espnow_mesh_patch_block_msg_t) > sizeof(espnow_mesh_data_msg_t)) \
         ? sizeof(espnow_mesh_patch_block_msg_t)                            \
         : sizeof(espnow_mesh_data_msg_t))
#else
#define ESPNOW_MESH_EVENT_DATA_BYTES sizeof(espnow_mesh_data_msg_t)
#endif

/*
 * Internal work item passed from the ESP-NOW/Wi-Fi callbacks to the mesh task
 * through the event queue. For RECV events, data[0..len) holds the raw frame.
 */
typedef struct {
    espnow_mesh_event_type_t type;
    uint8_t mac[ESP_NOW_ETH_ALEN];
    esp_now_send_status_t send_status;
    int8_t rssi;
    uint16_t len;
    uint64_t rx_us;
    uint8_t data[ESPNOW_MESH_EVENT_DATA_BYTES];
} espnow_mesh_event_t;
