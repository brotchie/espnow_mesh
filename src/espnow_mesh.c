/*
 * espnow_mesh core translation unit: one-time init (NVS, Wi-Fi, ESP-NOW, boot
 * id, security), public API, the FreeRTOS mesh task, and the role logic - the
 * controller reliable-fanout loop, the satellite receive/registration loop, and
 * (when enabled) the HIL test driver. The wire format, packet auth/send,
 * time-sync estimator, and sync-output actuator live in their own modules.
 *
 * Threading model (why most state here needs no locking):
 *   - A single mesh task owns nearly all mutable state and runs the role loops.
 *     It is the only writer of the satellite table, sequence counters, and
 *     satellite time-sync bookkeeping.
 *   - The ESP-NOW receive/send callbacks and the Wi-Fi event handler run on
 *     other tasks but only enqueue events (recv, send status, registration)
 *     onto s_event_queue; the mesh task drains and acts on them.
 *   - esp_timer callbacks (sync output, HIL markers) run on the esp_timer task
 *     and touch only their own module state plus the lock-protected clock.
 *   - The two locks guard the only genuinely cross-task sharing: the satellite
 *     table (s_satellite_table_lock, for the status snapshot) and the mesh
 *     clock (owned by espnow_mesh_time_sync.c).
 */

#include <inttypes.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#include "espnow_mesh.h"

#include "espnow_mesh_clock.h"
#include "espnow_mesh_packet.h"
#include "espnow_mesh_priv.h"
#include "espnow_mesh_sync_output.h"
#include "espnow_mesh_time_sync.h"

typedef struct {
    bool used;
    bool expected_current;
    bool acked_current;
    uint8_t mac[ESP_NOW_ETH_ALEN];
    uint32_t last_seen_ms;
    uint32_t last_sequence;
    uint32_t ack_count;
    uint32_t sends_current;
    bool retry_scheduled;
    uint32_t next_retry_ms;
    uint32_t retry_backoff_ms;
    int8_t last_rssi;
#if CONFIG_ESPNOW_MESH_HIL_TEST_ENABLE
    bool hil_acked_current;
    bool hil_patch_ready_current;
    uint32_t hil_sends_current;
    uint32_t hil_last_sequence;
    uint32_t hil_patch_request_count;
    uint32_t hil_patch_block_count;
    uint32_t hil_patch_ready_sequence;
#endif
} satellite_node_t;

static const char *TAG = "espnow_mesh";
static const uint8_t BROADCAST_MAC[ESP_NOW_ETH_ALEN] = {
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
};

static QueueHandle_t s_event_queue;
static TaskHandle_t s_mesh_task_handle;
static uint8_t s_self_mac[ESP_NOW_ETH_ALEN];
static uint32_t s_boot_id;
static uint8_t s_mesh_channel = CONFIG_ESPNOW_MESH_CHANNEL;
static bool s_espnow_ready;
static bool s_mesh_initialized;
static espnow_mesh_config_t s_start_config = ESPNOW_MESH_DEFAULT_CONFIG();

#if !CONFIG_ESPNOW_MESH_ROLE_CONTROLLER
#if CONFIG_ESPNOW_MESH_REGISTRATION_AP_ENABLE
static volatile bool s_registration_sta_connected;
static uint32_t s_next_registration_ms;
#endif
#if CONFIG_ESPNOW_MESH_HIL_TEST_ENABLE
static bool s_hil_registration_discovered;
#endif
#endif

#if CONFIG_ESPNOW_MESH_HIL_TEST_ENABLE
static uint8_t hil_random_channel(void)
{
    uint8_t min_channel = clamp_wifi_channel(CONFIG_ESPNOW_MESH_HIL_RANDOM_CHANNEL_MIN);
    uint8_t max_channel = clamp_wifi_channel(CONFIG_ESPNOW_MESH_HIL_RANDOM_CHANNEL_MAX);
    if (max_channel < min_channel) {
        uint8_t tmp = min_channel;
        min_channel = max_channel;
        max_channel = tmp;
    }

    uint32_t span = (uint32_t)max_channel - min_channel + 1u;
    uint32_t entropy = esp_random() ^ s_boot_id ^ ((uint32_t)now_us() * 0x9e3779b9u) ^
                       ((uint32_t)CONFIG_ESPNOW_MESH_DEVICE_ID * 0x85ebca6bu);
    return (uint8_t)(min_channel + (entropy % span));
}
#endif

static void init_mesh_channel(void)
{
    s_mesh_channel = clamp_wifi_channel(CONFIG_ESPNOW_MESH_CHANNEL);
#if CONFIG_ESPNOW_MESH_HIL_TEST_ENABLE && CONFIG_ESPNOW_MESH_HIL_RANDOM_START_CHANNEL_ENABLE
    s_mesh_channel = hil_random_channel();
    ESP_LOGI(TAG, "HIL random startup channel=%u configured_default=%d device_id=%d",
             s_mesh_channel, CONFIG_ESPNOW_MESH_CHANNEL, CONFIG_ESPNOW_MESH_DEVICE_ID);
#else
    ESP_LOGI(TAG, "startup channel=%u", s_mesh_channel);
#endif
}

static esp_err_t add_peer_if_needed(const uint8_t *mac)
{
    if (!s_espnow_ready) {
        return ESP_ERR_ESPNOW_NOT_INIT;
    }

    if (esp_now_is_peer_exist(mac)) {
        return ESP_OK;
    }

    esp_now_peer_info_t peer = { 0 };
    memcpy(peer.peer_addr, mac, ESP_NOW_ETH_ALEN);
    peer.channel = 0;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;

    esp_err_t err = esp_now_add_peer(&peer);
    if (err == ESP_ERR_ESPNOW_EXIST) {
        return ESP_OK;
    }
    return err;
}

/*
 * Total events lost to a full queue. Incremented from the Wi-Fi and event-loop
 * task callbacks, read from the mesh task and the status API; concurrent
 * increments may occasionally undercount, which is fine for diagnostics.
 */
static uint32_t s_event_queue_drops;

static void queue_event_or_count_drop(const espnow_mesh_event_t *event)
{
    if (xQueueSend(s_event_queue, event, 0) != pdTRUE) {
        s_event_queue_drops++;
    }
}

/* Called from the mesh task loops so overflow shows up in the log. */
static void log_event_queue_drops_if_any(void)
{
    static uint32_t s_logged_drops;
    static uint32_t s_last_log_ms;

    uint32_t drops = s_event_queue_drops;
    if (drops == s_logged_drops) {
        return;
    }

    uint32_t current_ms = now_ms();
    if (s_last_log_ms != 0 && (int32_t)(current_ms - s_last_log_ms) < 1000) {
        return;
    }

    ESP_LOGW(TAG, "event queue overflow: %" PRIu32 " events dropped since boot", drops);
    s_logged_drops = drops;
    s_last_log_ms = current_ms;
}

static void queue_recv_event(const uint8_t *src_mac, const uint8_t *data, int data_len, int8_t rssi)
{
    if (s_event_queue == NULL || src_mac == NULL || data == NULL || data_len <= 0) {
        return;
    }
    if ((size_t)data_len > sizeof(((espnow_mesh_event_t *)0)->data)) {
        return;
    }

    espnow_mesh_event_t event = {
        .type = ESPNOW_MESH_EVENT_RECV,
        .rssi = rssi,
        .len = (uint16_t)data_len,
        .rx_us = now_us(),
    };
    memcpy(event.mac, src_mac, ESP_NOW_ETH_ALEN);
    memcpy(event.data, data, (size_t)data_len);
    queue_event_or_count_drop(&event);
}

static void queue_send_event(const uint8_t *dest_mac, esp_now_send_status_t status)
{
    if (s_event_queue == NULL || dest_mac == NULL) {
        return;
    }

    espnow_mesh_event_t event = {
        .type = ESPNOW_MESH_EVENT_SEND,
        .send_status = status,
    };
    memcpy(event.mac, dest_mac, ESP_NOW_ETH_ALEN);
    queue_event_or_count_drop(&event);
}

#if CONFIG_ESPNOW_MESH_REGISTRATION_AP_ENABLE && CONFIG_ESPNOW_MESH_ROLE_CONTROLLER
/*
 * Called from the default event-loop task. The satellite table is owned by the
 * mesh task, so registration only enqueues the station MAC and the mesh task
 * performs the table update when it drains the event.
 */
static void queue_registration_event(const uint8_t *sta_mac)
{
    if (s_event_queue == NULL || sta_mac == NULL) {
        return;
    }

    espnow_mesh_event_t event = {
        .type = ESPNOW_MESH_EVENT_REGISTRATION,
    };
    memcpy(event.mac, sta_mac, ESP_NOW_ETH_ALEN);
    queue_event_or_count_drop(&event);
}
#endif

static void espnow_recv_cb(const esp_now_recv_info_t *info, const uint8_t *data, int data_len)
{
    int8_t rssi = 0;
    if (info != NULL && info->rx_ctrl != NULL) {
        rssi = info->rx_ctrl->rssi;
    }
    queue_recv_event(info == NULL ? NULL : info->src_addr, data, data_len, rssi);
}

static void espnow_send_cb(const esp_now_send_info_t *tx_info, esp_now_send_status_t status)
{
    queue_send_event(tx_info == NULL ? NULL : tx_info->des_addr, status);
}

static void init_nvs(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
}

#if CONFIG_ESPNOW_MESH_REGISTRATION_AP_ENABLE && CONFIG_ESPNOW_MESH_ROLE_CONTROLLER
static void configure_registration_ap(void)
{
    const char *ssid = CONFIG_ESPNOW_MESH_REGISTRATION_SSID;
    const char *password = CONFIG_ESPNOW_MESH_REGISTRATION_PASSWORD;
    size_t ssid_len = strlen(ssid);
    size_t password_len = strlen(password);

    if (ssid_len == 0 || ssid_len > sizeof(((wifi_config_t *)0)->ap.ssid)) {
        ESP_LOGE(TAG, "registration SSID must be 1-%zu characters",
                 sizeof(((wifi_config_t *)0)->ap.ssid));
        ESP_ERROR_CHECK(ESP_ERR_INVALID_ARG);
    }
    if (password_len > sizeof(((wifi_config_t *)0)->ap.password) - 1 ||
        (password_len > 0 && password_len < 8)) {
        ESP_LOGE(TAG, "registration password must be empty or 8-%zu characters",
                 sizeof(((wifi_config_t *)0)->ap.password) - 1);
        ESP_ERROR_CHECK(ESP_ERR_INVALID_ARG);
    }

    wifi_config_t ap_config = { 0 };
    memcpy(ap_config.ap.ssid, ssid, ssid_len);
    ap_config.ap.ssid_len = (uint8_t)ssid_len;
    memcpy(ap_config.ap.password, password, password_len);
    ap_config.ap.channel = s_mesh_channel;
    ap_config.ap.max_connection = CONFIG_ESPNOW_MESH_REGISTRATION_MAX_CONNECTIONS;
    ap_config.ap.authmode = password_len == 0 ? WIFI_AUTH_OPEN : WIFI_AUTH_WPA2_PSK;
    ap_config.ap.beacon_interval = 100;

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
    ESP_LOGI(TAG, "registration AP SSID=\"%s\" channel=%u auth=%s", ssid,
             s_mesh_channel, password_len == 0 ? "open" : "wpa2");
}
#endif

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id,
                               void *event_data)
{
    (void)arg;
    if (event_base != WIFI_EVENT) {
        return;
    }

#if CONFIG_ESPNOW_MESH_REGISTRATION_AP_ENABLE && CONFIG_ESPNOW_MESH_ROLE_CONTROLLER
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        const wifi_event_ap_staconnected_t *event = (const wifi_event_ap_staconnected_t *)event_data;
        queue_registration_event(event->mac);
    }
#endif

#if CONFIG_ESPNOW_MESH_REGISTRATION_AP_ENABLE && !CONFIG_ESPNOW_MESH_ROLE_CONTROLLER
    if (event_id == WIFI_EVENT_STA_CONNECTED) {
        const wifi_event_sta_connected_t *event = (const wifi_event_sta_connected_t *)event_data;
        s_registration_sta_connected = true;
        ESP_LOGI(TAG, "connected to registration AP " MACSTR " channel=%u",
                 MAC2STR(event->bssid), event->channel);
    } else if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
        s_registration_sta_connected = false;
    }
#endif
}

static void init_wifi(void)
{
    if (s_start_config.use_existing_wifi) {
#if CONFIG_ESPNOW_MESH_REGISTRATION_AP_ENABLE
        ESP_LOGE(TAG, "external Wi-Fi mode is incompatible with registration SoftAP support");
        ESP_ERROR_CHECK(ESP_ERR_INVALID_STATE);
#endif

        ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                            wifi_event_handler, NULL, NULL));
        ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
        if (s_start_config.adopt_current_wifi_channel) {
            uint8_t primary = 0;
            wifi_second_chan_t secondary = WIFI_SECOND_CHAN_NONE;
            ESP_ERROR_CHECK(esp_wifi_get_channel(&primary, &secondary));
            s_mesh_channel = clamp_wifi_channel(primary);
        }
        ESP_ERROR_CHECK(esp_wifi_get_mac(WIFI_IF_STA, s_self_mac));
        ESP_LOGI(TAG, "using application-managed Wi-Fi station " MACSTR ", channel %u",
                 MAC2STR(s_self_mac), s_mesh_channel);
        return;
    }

    esp_err_t err = esp_netif_init();
    if (err != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(err);
    }

    err = esp_event_loop_create_default();
    if (err != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(err);
    }

    (void)esp_netif_create_default_wifi_sta();
#if CONFIG_ESPNOW_MESH_REGISTRATION_AP_ENABLE && CONFIG_ESPNOW_MESH_ROLE_CONTROLLER
    (void)esp_netif_create_default_wifi_ap();
#endif

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                        wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
#if CONFIG_ESPNOW_MESH_REGISTRATION_AP_ENABLE && CONFIG_ESPNOW_MESH_ROLE_CONTROLLER
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    configure_registration_ap();
#else
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
#endif
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_channel(s_mesh_channel, WIFI_SECOND_CHAN_NONE));
    ESP_ERROR_CHECK(esp_wifi_get_mac(WIFI_IF_STA, s_self_mac));

    ESP_LOGI(TAG, "station MAC " MACSTR ", channel %u", MAC2STR(s_self_mac),
             s_mesh_channel);
}

static void init_espnow(void)
{
    s_event_queue = xQueueCreate(CONFIG_ESPNOW_MESH_EVENT_QUEUE_LEN, sizeof(espnow_mesh_event_t));
    if (s_event_queue == NULL) {
        ESP_ERROR_CHECK(ESP_ERR_NO_MEM);
    }

    ESP_ERROR_CHECK(esp_now_init());
    s_espnow_ready = true;
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));
    ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb));
    ESP_ERROR_CHECK(add_peer_if_needed(BROADCAST_MAC));
}

static void init_boot_id(void)
{
    s_boot_id = esp_random();
    if (s_boot_id == 0) {
        s_boot_id = (uint32_t)now_us();
    }
    ESP_LOGI(TAG, "boot/session id 0x%08" PRIx32, s_boot_id);
}

static void init_mesh_security(void)
{
#if CONFIG_ESPNOW_MESH_AUTH_ENABLE
    if (strlen(CONFIG_ESPNOW_MESH_AUTH_KEY) == 0) {
        ESP_LOGE(TAG, "CONFIG_ESPNOW_MESH_AUTH_KEY must not be empty while auth is enabled");
        ESP_ERROR_CHECK(ESP_ERR_INVALID_ARG);
    }
    ESP_LOGI(TAG, "mesh id=0x%08" PRIx32 " auth=hmac-sha256-64",
             (uint32_t)CONFIG_ESPNOW_MESH_ID);
#else
    ESP_LOGW(TAG, "mesh id=0x%08" PRIx32
                  " auth disabled; ESP-NOW packets are filtered by mesh id only",
             (uint32_t)CONFIG_ESPNOW_MESH_ID);
#endif
}

#if CONFIG_ESPNOW_MESH_HIL_TEST_ENABLE
typedef enum {
    ESPNOW_MESH_HIL_FAULT_RX_CMD = 1,
    ESPNOW_MESH_HIL_FAULT_TX_ACK = 2,
    ESPNOW_MESH_HIL_FAULT_TX_TIME_RESP = 3,
    ESPNOW_MESH_HIL_FAULT_TX_PATCH_BLOCK_REQ = 4,
    ESPNOW_MESH_HIL_FAULT_RX_PATCH_BLOCK = 5,
    ESPNOW_MESH_HIL_FAULT_TX_PATCH_READY = 6,
} espnow_mesh_hil_fault_direction_t;

static uint32_t hil_mix32(uint32_t value)
{
    value ^= value >> 16;
    value *= 0x7feb352du;
    value ^= value >> 15;
    value *= 0x846ca68bu;
    value ^= value >> 16;
    return value;
}

static bool hil_fault_drop(uint8_t drop_pct, uint32_t seed, uint32_t sequence,
                           uint32_t attempt, espnow_mesh_hil_fault_direction_t direction)
{
    if (drop_pct == 0) {
        return false;
    }
    if (drop_pct >= 100) {
        return true;
    }

    uint32_t value = seed ^ (sequence * 0x9e3779b9u) ^ (attempt * 0x85ebca6bu) ^
                     ((uint32_t)CONFIG_ESPNOW_MESH_DEVICE_ID * 0xc2b2ae35u) ^
                     ((uint32_t)direction * 0x27d4eb2du);
    return (hil_mix32(value) % 100u) < drop_pct;
}

/*
 * Field validators below run after mesh_rx_packet_type() has already checked the
 * header and HMAC, so they only enforce message-specific semantics. The offer and
 * block messages are received by satellites; the request and ready messages by
 * the controller.
 */
#if !CONFIG_ESPNOW_MESH_ROLE_CONTROLLER
static bool patch_offer_fields_valid(const espnow_mesh_patch_offer_msg_t *msg)
{
    return msg->total_len > 0 && msg->total_len <= CONFIG_ESPNOW_MESH_HIL_PATCH_BYTES &&
           msg->block_size > 0 && msg->block_size <= CONFIG_ESPNOW_MESH_HIL_PATCH_BLOCK_BYTES &&
           msg->block_count > 0 && msg->block_count <= ESPNOW_MESH_HIL_PATCH_MAX_BLOCKS;
}

static bool patch_block_fields_valid(const espnow_mesh_patch_block_msg_t *msg, uint16_t len)
{
    if (msg->payload_len == 0 || msg->payload_len > sizeof(msg->payload)) {
        return false;
    }
    if (msg->block_count == 0 || msg->block_count > ESPNOW_MESH_HIL_PATCH_MAX_BLOCKS ||
        msg->block_index >= msg->block_count) {
        return false;
    }
    if (msg->total_len == 0 || msg->total_len > CONFIG_ESPNOW_MESH_HIL_PATCH_BYTES ||
        msg->block_offset >= msg->total_len ||
        msg->payload_len > msg->total_len - msg->block_offset) {
        return false;
    }
    return len >= offsetof(espnow_mesh_patch_block_msg_t, payload) + msg->payload_len;
}
#else
static bool patch_block_req_fields_valid(const espnow_mesh_patch_block_req_msg_t *msg)
{
    return msg->block_count > 0 &&
           msg->block_count <= ESPNOW_MESH_HIL_PATCH_MAX_BLOCKS &&
           msg->block_index < msg->block_count;
}

static bool patch_ready_fields_valid(const espnow_mesh_patch_ready_msg_t *msg)
{
    return msg->total_len > 0 && msg->total_len <= CONFIG_ESPNOW_MESH_HIL_PATCH_BYTES &&
           msg->blocks_received <= ESPNOW_MESH_HIL_PATCH_MAX_BLOCKS;
}
#endif

static uint8_t hil_patch_byte(uint32_t patch_id, uint32_t offset)
{
    return (uint8_t)((patch_id * 31u + offset * 17u + (offset >> 3) + 0x5au) & 0xffu);
}

static uint32_t hil_hash_update(uint32_t hash, uint8_t byte)
{
    hash ^= byte;
    hash *= 16777619u;
    return hash;
}

static uint32_t hil_hash_bytes(const uint8_t *data, size_t len)
{
    uint32_t hash = 2166136261u;
    for (size_t i = 0; i < len; ++i) {
        hash = hil_hash_update(hash, data[i]);
    }
    return hash;
}

static uint32_t hil_patch_hash(uint32_t patch_id, uint32_t total_len)
{
    uint32_t hash = 2166136261u;
    for (uint32_t i = 0; i < total_len; ++i) {
        hash = hil_hash_update(hash, hil_patch_byte(patch_id, i));
    }
    return hash;
}

static uint16_t hil_patch_block_count_for_len(uint32_t total_len)
{
    return (uint16_t)((total_len + CONFIG_ESPNOW_MESH_HIL_PATCH_BLOCK_BYTES - 1u) /
                      CONFIG_ESPNOW_MESH_HIL_PATCH_BLOCK_BYTES);
}

#if CONFIG_ESPNOW_MESH_ROLE_CONTROLLER
static void hil_fill_patch_block(uint32_t patch_id, uint32_t offset, uint8_t *payload,
                                 uint16_t payload_len)
{
    for (uint16_t i = 0; i < payload_len; ++i) {
        payload[i] = hil_patch_byte(patch_id, offset + i);
    }
}

static uint16_t hil_patch_payload_len_for_block(uint16_t block_index, uint32_t total_len)
{
    uint32_t offset = (uint32_t)block_index * CONFIG_ESPNOW_MESH_HIL_PATCH_BLOCK_BYTES;
    if (offset >= total_len) {
        return 0;
    }
    uint32_t remaining = total_len - offset;
    return (uint16_t)(remaining > CONFIG_ESPNOW_MESH_HIL_PATCH_BLOCK_BYTES
                          ? CONFIG_ESPNOW_MESH_HIL_PATCH_BLOCK_BYTES
                          : remaining);
}
#endif
#endif

#if CONFIG_ESPNOW_MESH_ROLE_CONTROLLER

/*
 * The satellite table is owned by the mesh task. The lock only orders slot
 * claiming against espnow_mesh_get_status(), which snapshots the table from
 * other tasks; per-field counter updates are read without it and are
 * eventually consistent.
 */
static satellite_node_t s_satellites[CONFIG_ESPNOW_MESH_MAX_SATELLITES];
static portMUX_TYPE s_satellite_table_lock = portMUX_INITIALIZER_UNLOCKED;
static uint32_t s_controller_active_sequence;
static uint32_t s_controller_last_sequence;
static uint32_t s_controller_last_expected;
static uint32_t s_controller_last_acked;
static bool s_controller_last_sequence_complete;

/* Payload carried by the active reliable sequence, read by both the broadcast
 * and unicast senders. Filled from the application buffer when a sequence
 * begins; stays within CONFIG_ESPNOW_MESH_PAYLOAD_BYTES. */
static uint8_t s_tx_payload[CONFIG_ESPNOW_MESH_PAYLOAD_BYTES];
static uint16_t s_tx_payload_len;

#if !CONFIG_ESPNOW_MESH_HIL_TEST_ENABLE
/* Single-slot application send buffer, written by espnow_mesh_send() from any
 * task and drained by the controller loop. */
static portMUX_TYPE s_pending_payload_lock = portMUX_INITIALIZER_UNLOCKED;
static uint8_t s_pending_payload[CONFIG_ESPNOW_MESH_PAYLOAD_BYTES];
static uint16_t s_pending_payload_len;
static bool s_have_pending_payload;
#endif

#if CONFIG_ESPNOW_MESH_HIL_TEST_ENABLE
static uint16_t s_hil_time_resp_drop_pct;
static uint32_t s_hil_time_resp_drop_seed;
static uint32_t s_hil_time_resp_drop_until_ms;
#endif

static int find_satellite(const uint8_t *mac)
{
    for (int i = 0; i < CONFIG_ESPNOW_MESH_MAX_SATELLITES; ++i) {
        if (s_satellites[i].used && mac_equal(s_satellites[i].mac, mac)) {
            return i;
        }
    }
    return -1;
}

#if CONFIG_ESPNOW_MESH_HIL_TEST_ENABLE
static int known_satellite_count(void)
{
    int known = 0;
    for (int i = 0; i < CONFIG_ESPNOW_MESH_MAX_SATELLITES; ++i) {
        if (s_satellites[i].used) {
            known++;
        }
    }
    return known;
}
#endif

static int find_or_add_satellite(const uint8_t *mac)
{
    int index = find_satellite(mac);
    if (index >= 0) {
        return index;
    }

    for (int i = 0; i < CONFIG_ESPNOW_MESH_MAX_SATELLITES; ++i) {
        if (!s_satellites[i].used) {
            portENTER_CRITICAL(&s_satellite_table_lock);
            memcpy(s_satellites[i].mac, mac, ESP_NOW_ETH_ALEN);
            s_satellites[i].used = true;
            portEXIT_CRITICAL(&s_satellite_table_lock);
            /* Seed the freshness timestamp so the new entry is not immediately
             * eligible for expiry before its first ACK/time-request lands. */
            s_satellites[i].last_seen_ms = now_ms();
            ESP_LOGI(TAG, "discovered satellite " MACSTR, MAC2STR(mac));
            if (s_espnow_ready) {
                esp_err_t err = add_peer_if_needed(mac);
                if (err != ESP_OK) {
                    ESP_LOGW(TAG, "could not add satellite peer " MACSTR ": %s", MAC2STR(mac),
                             esp_err_to_name(err));
                }
            }
            return i;
        }
    }

    ESP_LOGW(TAG, "satellite table full; dropping ACK from " MACSTR, MAC2STR(mac));
    return -1;
}

#if CONFIG_ESPNOW_MESH_REGISTRATION_AP_ENABLE
static void controller_note_registered_satellite(const uint8_t *mac)
{
    int index = find_or_add_satellite(mac);
    if (index < 0) {
        return;
    }

    s_satellites[index].last_seen_ms = now_ms();
    ESP_LOGI(TAG, "registration AP learned satellite " MACSTR, MAC2STR(mac));
}
#endif

static uint32_t retry_jitter_ms(uint32_t sequence, int satellite_index, uint32_t attempt)
{
    if (CONFIG_ESPNOW_MESH_RELIABLE_RETRY_JITTER_MS == 0) {
        return 0;
    }

    uint32_t value = sequence ^ ((uint32_t)satellite_index * 0x9e3779b9u) ^
                     (attempt * 0x85ebca6bu);
    value ^= value >> 16;
    value *= 0x7feb352du;
    value ^= value >> 15;
    return value % (CONFIG_ESPNOW_MESH_RELIABLE_RETRY_JITTER_MS + 1);
}

static uint32_t clamp_backoff_ms(uint32_t backoff_ms)
{
    if (backoff_ms > CONFIG_ESPNOW_MESH_RELIABLE_MAX_BACKOFF_MS) {
        return CONFIG_ESPNOW_MESH_RELIABLE_MAX_BACKOFF_MS;
    }
    return backoff_ms;
}

static void schedule_next_retry(satellite_node_t *node, int satellite_index, uint32_t sequence,
                                uint32_t current_ms)
{
    uint32_t jitter_ms = retry_jitter_ms(sequence, satellite_index, node->sends_current);
    node->retry_scheduled = true;
    node->next_retry_ms = current_ms + node->retry_backoff_ms + jitter_ms;

    uint32_t next_backoff_ms = node->retry_backoff_ms * 2;
    if (next_backoff_ms < node->retry_backoff_ms) {
        next_backoff_ms = CONFIG_ESPNOW_MESH_RELIABLE_MAX_BACKOFF_MS;
    }
    node->retry_backoff_ms = clamp_backoff_ms(next_backoff_ms);
}

static void init_satellite_delivery_state(satellite_node_t *node, bool expected)
{
    node->expected_current = expected;
    node->acked_current = false;
    node->sends_current = 0;
    node->retry_scheduled = false;
    node->next_retry_ms = 0;
    node->retry_backoff_ms = clamp_backoff_ms(CONFIG_ESPNOW_MESH_RELIABLE_INITIAL_BACKOFF_MS);
}

static void begin_reliable_sequence(void)
{
    for (int i = 0; i < CONFIG_ESPNOW_MESH_MAX_SATELLITES; ++i) {
        init_satellite_delivery_state(&s_satellites[i], s_satellites[i].used);
    }
}

static int current_expected_count(void)
{
    int expected = 0;
    for (int i = 0; i < CONFIG_ESPNOW_MESH_MAX_SATELLITES; ++i) {
        if (s_satellites[i].used && s_satellites[i].expected_current) {
            expected++;
        }
    }
    return expected;
}

static int current_acked_count(void)
{
    int acked = 0;
    for (int i = 0; i < CONFIG_ESPNOW_MESH_MAX_SATELLITES; ++i) {
        if (s_satellites[i].used && s_satellites[i].expected_current &&
            s_satellites[i].acked_current) {
            acked++;
        }
    }
    return acked;
}

static bool reliable_sequence_complete(void)
{
    int expected = current_expected_count();
    return expected > 0 && current_acked_count() == expected;
}

static bool retry_time_due(uint32_t current_ms, uint32_t retry_ms)
{
    return (int32_t)(current_ms - retry_ms) >= 0;
}

static void note_ack(const uint8_t *src_mac, const espnow_mesh_ack_msg_t *ack, uint32_t active_sequence)
{
    int index = find_or_add_satellite(src_mac);
    if (index < 0) {
        return;
    }

    satellite_node_t *node = &s_satellites[index];
    node->last_seen_ms = now_ms();
    node->last_sequence = ack->sequence;
    node->last_rssi = ack->last_rssi;

    if (ack->controller_boot_id != s_boot_id) {
        ESP_LOGD(TAG, "stale ACK seq=%" PRIu32 " boot=0x%08" PRIx32 " active=0x%08" PRIx32
                      " from " MACSTR,
                 ack->sequence, ack->controller_boot_id, s_boot_id, MAC2STR(src_mac));
        return;
    }

    if (ack->sequence != active_sequence) {
        ESP_LOGD(TAG, "stale ACK seq=%" PRIu32 " active=%" PRIu32 " from " MACSTR,
                 ack->sequence, active_sequence, MAC2STR(src_mac));
        return;
    }

    if (!node->expected_current) {
        init_satellite_delivery_state(node, true);
        ESP_LOGI(TAG, "late-discovered satellite " MACSTR " joined seq=%" PRIu32,
                 MAC2STR(src_mac), active_sequence);
    }

    node->ack_count++;
    if (!node->acked_current) {
        node->acked_current = true;
        ESP_LOGI(TAG, "ACK seq=%" PRIu32 " from " MACSTR " rssi=%d duplicate=%u",
                 ack->sequence, MAC2STR(src_mac), ack->last_rssi, ack->duplicate);
    }
}

static void log_sequence_summary(uint32_t sequence)
{
    int expected = 0;
    int acked = 0;

    for (int i = 0; i < CONFIG_ESPNOW_MESH_MAX_SATELLITES; ++i) {
        if (!s_satellites[i].used || !s_satellites[i].expected_current) {
            continue;
        }
        expected++;
        if (s_satellites[i].acked_current) {
            acked++;
        } else {
            ESP_LOGW(TAG, "missing ACK seq=%" PRIu32 " from " MACSTR
                          " attempts=%" PRIu32 " last_seq=%" PRIu32
                          " last_seen=%" PRIu32 "ms",
                     sequence, MAC2STR(s_satellites[i].mac), s_satellites[i].sends_current,
                     s_satellites[i].last_sequence, s_satellites[i].last_seen_ms);
        }
    }

    if (expected == 0) {
        ESP_LOGW(TAG, "seq=%" PRIu32 " complete: no satellites known for ACK quorum", sequence);
    } else if (acked == expected) {
        ESP_LOGI(TAG, "seq=%" PRIu32 " delivered: %d/%d satellites ACKed", sequence, acked,
                 expected);
    } else {
        ESP_LOGW(TAG, "seq=%" PRIu32 " failed/degraded: %d/%d satellites ACKed", sequence,
                 acked, expected);
    }
}

static void handle_time_request(const espnow_mesh_event_t *event)
{
    if (event->len < sizeof(espnow_mesh_time_req_msg_t)) {
        return;
    }
    espnow_mesh_time_req_msg_t req = { 0 };
    memcpy(&req, event->data, sizeof(req));

    int index = find_or_add_satellite(event->mac);
    if (index < 0) {
        return;
    }
    s_satellites[index].last_seen_ms = now_ms();

#if CONFIG_ESPNOW_MESH_HIL_TEST_ENABLE
    if (s_hil_time_resp_drop_pct > 0 &&
        (int32_t)(s_hil_time_resp_drop_until_ms - now_ms()) > 0 &&
        hil_fault_drop((uint8_t)s_hil_time_resp_drop_pct, s_hil_time_resp_drop_seed,
                       req.sequence, req.sequence, ESPNOW_MESH_HIL_FAULT_TX_TIME_RESP)) {
        ESP_LOGW(TAG, "HIL injected time response drop seq=%" PRIu32 " to " MACSTR,
                 req.sequence, MAC2STR(event->mac));
        return;
    }
#endif

    espnow_mesh_time_resp_msg_t resp = {
        .magic = ESPNOW_MESH_MAGIC,
        .version = ESPNOW_MESH_VERSION,
        .type = ESPNOW_MESH_MSG_TIME_RESP,
        .sequence = req.sequence,
        .satellite_tx_us = req.satellite_tx_us,
        .controller_rx_us = event->rx_us,
        .controller_tx_us = now_us(),
    };
    memcpy(resp.controller_mac, s_self_mac, ESP_NOW_ETH_ALEN);

    esp_err_t err = mesh_send(event->mac, &resp, sizeof(resp), ESPNOW_MESH_MSG_TIME_RESP);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "time response seq=%" PRIu32 " to " MACSTR " failed: %s",
                 req.sequence, MAC2STR(event->mac), esp_err_to_name(err));
    }
}

static void handle_controller_event(const espnow_mesh_event_t *event, uint32_t active_sequence)
{
    if (event->type == ESPNOW_MESH_EVENT_SEND) {
        if (event->send_status != ESP_NOW_SEND_SUCCESS) {
            ESP_LOGW(TAG, "ESP-NOW send callback failed for " MACSTR, MAC2STR(event->mac));
        }
        return;
    }

#if CONFIG_ESPNOW_MESH_REGISTRATION_AP_ENABLE
    if (event->type == ESPNOW_MESH_EVENT_REGISTRATION) {
        controller_note_registered_satellite(event->mac);
        return;
    }
#endif

    uint8_t type = 0;
    if (!mesh_rx_packet_type(event, &type)) {
        return;
    }

    switch (type) {
    case ESPNOW_MESH_MSG_TIME_REQ:
        handle_time_request(event);
        break;
    case ESPNOW_MESH_MSG_ACK: {
        if (event->len < sizeof(espnow_mesh_ack_msg_t)) {
            break;
        }
        espnow_mesh_ack_msg_t ack = { 0 };
        memcpy(&ack, event->data, sizeof(ack));
        note_ack(event->mac, &ack, active_sequence);
        break;
    }
    default:
        break;
    }
}

/*
 * Drain and dispatch queued events until the deadline, waking at least every
 * 100 ms so time-based retry scheduling stays responsive. The event handler
 * differs between the normal controller loop and the HIL loop, so it is passed
 * in; sequence is forwarded to the handler as its active-sequence argument.
 */
static void drain_events_until(uint32_t deadline_ms, uint32_t sequence,
                               void (*handler)(const espnow_mesh_event_t *, uint32_t))
{
    while (true) {
        uint32_t current_ms = now_ms();
        if ((int32_t)(deadline_ms - current_ms) <= 0) {
            return;
        }

        uint32_t wait_ms = deadline_ms - current_ms;
        if (wait_ms > 100) {
            wait_ms = 100;
        }

        espnow_mesh_event_t event = { 0 };
        if (xQueueReceive(s_event_queue, &event, pdMS_TO_TICKS(wait_ms)) == pdTRUE) {
            handler(&event, sequence);
        }
    }
}

static void drain_controller_events_until(uint32_t deadline_ms, uint32_t active_sequence)
{
    drain_events_until(deadline_ms, active_sequence, handle_controller_event);
}

static esp_err_t send_controller_packet(const uint8_t *dest_mac, uint32_t sequence,
                                        uint32_t attempt_index)
{
    espnow_mesh_data_msg_t msg = {
        .magic = ESPNOW_MESH_MAGIC,
        .version = ESPNOW_MESH_VERSION,
        .type = ESPNOW_MESH_MSG_DATA,
        .sequence = sequence,
        .controller_boot_id = s_boot_id,
        .retry_index = attempt_index,
        .sent_ms = now_ms(),
    };
    memcpy(msg.controller_mac, s_self_mac, ESP_NOW_ETH_ALEN);

    msg.payload_len = s_tx_payload_len;
    memcpy(msg.payload, s_tx_payload, s_tx_payload_len);

    size_t send_len = offsetof(espnow_mesh_data_msg_t, payload) + msg.payload_len;
    return mesh_send(dest_mac, &msg, send_len, ESPNOW_MESH_MSG_DATA);
}

static void record_broadcast_attempt_for_expected(void)
{
    for (int i = 0; i < CONFIG_ESPNOW_MESH_MAX_SATELLITES; ++i) {
        satellite_node_t *node = &s_satellites[i];
        if (node->used && node->expected_current && !node->acked_current) {
            node->sends_current++;
        }
    }
}

static void schedule_missing_after_broadcast(uint32_t sequence)
{
    uint32_t current_ms = now_ms();
    for (int i = 0; i < CONFIG_ESPNOW_MESH_MAX_SATELLITES; ++i) {
        satellite_node_t *node = &s_satellites[i];
        if (!node->used || !node->expected_current || node->acked_current ||
            node->sends_current >= CONFIG_ESPNOW_MESH_RELIABLE_MAX_ATTEMPTS) {
            continue;
        }
        schedule_next_retry(node, i, sequence, current_ms);
    }
}

static bool send_due_unicast_retries(uint32_t sequence, uint32_t *next_retry_ms)
{
    bool retryable_missing = false;
    uint32_t current_ms = now_ms();
    *next_retry_ms = UINT32_MAX;

    for (int i = 0; i < CONFIG_ESPNOW_MESH_MAX_SATELLITES; ++i) {
        satellite_node_t *node = &s_satellites[i];
        if (!node->used || !node->expected_current || node->acked_current) {
            continue;
        }
        if (node->sends_current >= CONFIG_ESPNOW_MESH_RELIABLE_MAX_ATTEMPTS) {
            continue;
        }

        retryable_missing = true;
        if (!node->retry_scheduled || retry_time_due(current_ms, node->next_retry_ms)) {
            uint32_t attempt = node->sends_current + 1;
            esp_err_t err = add_peer_if_needed(node->mac);
            if (err == ESP_OK) {
                err = send_controller_packet(node->mac, sequence, attempt);
            }
            node->sends_current++;
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "unicast retry seq=%" PRIu32 " attempt=%" PRIu32
                              " to " MACSTR " failed: %s",
                         sequence, attempt, MAC2STR(node->mac), esp_err_to_name(err));
            } else {
                ESP_LOGI(TAG, "unicast retry seq=%" PRIu32 " attempt=%" PRIu32
                              " to " MACSTR,
                         sequence, attempt, MAC2STR(node->mac));
            }
            schedule_next_retry(node, i, sequence, current_ms);
        }

        if (node->retry_scheduled && node->next_retry_ms < *next_retry_ms) {
            *next_retry_ms = node->next_retry_ms;
        }
    }

    return retryable_missing;
}

static void reliable_retry_until_done_or_deadline(uint32_t sequence, uint32_t deadline_ms)
{
    while (!reliable_sequence_complete()) {
        uint32_t current_ms = now_ms();
        if ((int32_t)(deadline_ms - current_ms) <= 0) {
            return;
        }

        uint32_t next_retry_ms = UINT32_MAX;
        bool retryable_missing = send_due_unicast_retries(sequence, &next_retry_ms);
        if (!retryable_missing) {
            return;
        }

        current_ms = now_ms();
        uint32_t wait_until_ms = deadline_ms;
        if (next_retry_ms != UINT32_MAX && (int32_t)(next_retry_ms - wait_until_ms) < 0) {
            wait_until_ms = next_retry_ms;
        }
        if ((int32_t)(wait_until_ms - current_ms) <= 0) {
            wait_until_ms = current_ms + 1;
        }
        drain_controller_events_until(wait_until_ms, sequence);
    }
}

#if CONFIG_ESPNOW_MESH_HIL_TEST_ENABLE
typedef struct {
    const char *name;
    uint8_t kind;
    uint8_t rx_drop_pct;
    uint8_t ack_drop_pct;
    uint8_t duplicate_send_count;
    uint16_t time_resp_drop_pct;
    uint32_t patch_id;
    uint32_t param_id;
    int32_t param_value;
} hil_case_t;

static const hil_case_t HIL_CASES[] = {
    {
        .name = "softap_channel_discovery",
        .kind = ESPNOW_MESH_HIL_CMD_CHANNEL_DISCOVERY,
    },
    {
        .name = "patch_select_no_loss",
        .kind = ESPNOW_MESH_HIL_CMD_PATCH_SELECT,
        .patch_id = 1,
    },
    {
        .name = "param_set_no_loss",
        .kind = ESPNOW_MESH_HIL_CMD_PARAM_SET,
        .patch_id = 1,
        .param_id = 7,
        .param_value = 110,
    },
    {
        .name = "ack_loss_recovery",
        .kind = ESPNOW_MESH_HIL_CMD_PATCH_SELECT,
        .ack_drop_pct = 50,
        .patch_id = 2,
    },
    {
        .name = "data_loss_recovery",
        .kind = ESPNOW_MESH_HIL_CMD_PARAM_SET,
        .rx_drop_pct = 55,
        .patch_id = 2,
        .param_id = 7,
        .param_value = 220,
    },
    {
        .name = "combined_loss_recovery",
        .kind = ESPNOW_MESH_HIL_CMD_PATCH_SELECT,
        .rx_drop_pct = 35,
        .ack_drop_pct = 35,
        .duplicate_send_count = 1,
        .patch_id = 3,
    },
    {
        .name = "time_sync_degraded",
        .kind = ESPNOW_MESH_HIL_CMD_PARAM_SET,
        .time_resp_drop_pct = 80,
        .patch_id = 3,
        .param_id = 9,
        .param_value = 330,
    },
};

typedef struct {
    const char *name;
    uint8_t block_req_drop_pct;
    uint8_t block_drop_pct;
    uint8_t ready_drop_pct;
    uint32_t patch_id;
} hil_patch_case_t;

static const hil_patch_case_t PATCH_HIL_CASES[] = {
    {
        .name = "patch_distribution_no_loss",
        .patch_id = 101,
    },
    {
        .name = "patch_distribution_block_loss",
        .block_drop_pct = 35,
        .patch_id = 102,
    },
    {
        .name = "patch_distribution_request_ready_loss",
        .block_req_drop_pct = 30,
        .ready_drop_pct = 50,
        .patch_id = 103,
    },
};

static void hil_reset_ack_state(void)
{
    for (int i = 0; i < CONFIG_ESPNOW_MESH_MAX_SATELLITES; ++i) {
        satellite_node_t *node = &s_satellites[i];
        node->expected_current = node->used;
        node->hil_acked_current = false;
        node->hil_patch_ready_current = false;
        node->hil_sends_current = 0;
        node->retry_scheduled = false;
        node->next_retry_ms = 0;
        node->retry_backoff_ms = clamp_backoff_ms(CONFIG_ESPNOW_MESH_RELIABLE_INITIAL_BACKOFF_MS);
    }
}

static void hil_reset_patch_state(void)
{
    for (int i = 0; i < CONFIG_ESPNOW_MESH_MAX_SATELLITES; ++i) {
        satellite_node_t *node = &s_satellites[i];
        node->expected_current = node->used;
        node->hil_patch_ready_current = false;
        node->hil_patch_request_count = 0;
        node->hil_patch_block_count = 0;
        node->hil_patch_ready_sequence = 0;
    }
}

static int hil_expected_count(void)
{
    int expected = 0;
    for (int i = 0; i < CONFIG_ESPNOW_MESH_MAX_SATELLITES; ++i) {
        if (s_satellites[i].used && s_satellites[i].expected_current) {
            expected++;
        }
    }
    return expected;
}

static int hil_acked_count(void)
{
    int acked = 0;
    for (int i = 0; i < CONFIG_ESPNOW_MESH_MAX_SATELLITES; ++i) {
        if (s_satellites[i].used && s_satellites[i].expected_current &&
            s_satellites[i].hil_acked_current) {
            acked++;
        }
    }
    return acked;
}

static bool hil_complete(void)
{
    int expected = hil_expected_count();
    return expected > 0 && hil_acked_count() == expected;
}

static int hil_patch_ready_count(void)
{
    int ready = 0;
    for (int i = 0; i < CONFIG_ESPNOW_MESH_MAX_SATELLITES; ++i) {
        if (s_satellites[i].used && s_satellites[i].expected_current &&
            s_satellites[i].hil_patch_ready_current) {
            ready++;
        }
    }
    return ready;
}

static bool hil_patch_complete(void)
{
    int expected = hil_expected_count();
    return expected > 0 && hil_patch_ready_count() == expected;
}

static void hil_note_ack(const uint8_t *src_mac, const espnow_mesh_hil_ack_msg_t *ack,
                         uint32_t sequence)
{
    int index = find_or_add_satellite(src_mac);
    if (index < 0) {
        return;
    }

    satellite_node_t *node = &s_satellites[index];
    node->last_seen_ms = now_ms();
    node->last_rssi = ack->last_rssi;
    node->hil_last_sequence = ack->sequence;

    if (ack->controller_boot_id != s_boot_id || ack->sequence != sequence) {
        ESP_LOGD(TAG, "HIL stale ACK seq=%" PRIu32 " active=%" PRIu32 " from " MACSTR,
                 ack->sequence, sequence, MAC2STR(src_mac));
        return;
    }

    if (!node->expected_current) {
        node->expected_current = true;
        ESP_LOGI(TAG, "HIL late satellite " MACSTR " joined seq=%" PRIu32,
                 MAC2STR(src_mac), sequence);
    }

    if (!node->hil_acked_current) {
        node->hil_acked_current = true;
        ESP_LOGI(TAG,
                 "HIL ACK case=%" PRIu32 " seq=%" PRIu32 " cmd=%" PRIu32
                 " from " MACSTR " duplicate=%u patch=%" PRIu32
                 " param=%" PRIu32 " value=%" PRId32,
                 ack->test_case, ack->sequence, ack->command_id, MAC2STR(src_mac),
                 ack->duplicate, ack->patch_id, ack->param_id, ack->param_value);
    }
}

static esp_err_t hil_send_patch_offer(const hil_patch_case_t *hil_case, uint32_t sequence)
{
    uint32_t total_len = CONFIG_ESPNOW_MESH_HIL_PATCH_BYTES;
    espnow_mesh_patch_offer_msg_t offer = {
        .magic = ESPNOW_MESH_MAGIC,
        .version = ESPNOW_MESH_VERSION,
        .type = ESPNOW_MESH_MSG_PATCH_OFFER,
        .sequence = sequence,
        .controller_boot_id = s_boot_id,
        .patch_id = hil_case->patch_id,
        .total_len = total_len,
        .block_size = CONFIG_ESPNOW_MESH_HIL_PATCH_BLOCK_BYTES,
        .block_count = hil_patch_block_count_for_len(total_len),
        .patch_hash = hil_patch_hash(hil_case->patch_id, total_len),
        .block_req_drop_pct = hil_case->block_req_drop_pct,
        .block_drop_pct = hil_case->block_drop_pct,
        .ready_drop_pct = hil_case->ready_drop_pct,
        .fault_seed = 0x50415400u ^ sequence,
    };

    return mesh_send(BROADCAST_MAC, &offer, sizeof(offer), ESPNOW_MESH_MSG_PATCH_OFFER);
}

static esp_err_t hil_send_patch_block(const uint8_t *dest_mac,
                                      const espnow_mesh_patch_block_req_msg_t *req)
{
    uint32_t total_len = CONFIG_ESPNOW_MESH_HIL_PATCH_BYTES;
    uint16_t payload_len = hil_patch_payload_len_for_block(req->block_index, total_len);
    if (payload_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    uint32_t block_offset =
        (uint32_t)req->block_index * CONFIG_ESPNOW_MESH_HIL_PATCH_BLOCK_BYTES;
    espnow_mesh_patch_block_msg_t block = {
        .magic = ESPNOW_MESH_MAGIC,
        .version = ESPNOW_MESH_VERSION,
        .type = ESPNOW_MESH_MSG_PATCH_BLOCK,
        .payload_len = payload_len,
        .sequence = req->sequence,
        .controller_boot_id = s_boot_id,
        .patch_id = req->patch_id,
        .block_index = req->block_index,
        .block_count = hil_patch_block_count_for_len(total_len),
        .block_offset = block_offset,
        .total_len = total_len,
        .patch_hash = req->patch_hash,
        .request_sequence = req->request_sequence,
        .block_drop_pct = req->block_drop_pct,
        .fault_seed = req->fault_seed,
    };
    hil_fill_patch_block(req->patch_id, block_offset, block.payload, payload_len);
    block.block_hash = hil_hash_bytes(block.payload, payload_len);

    return mesh_send(dest_mac, &block,
                     offsetof(espnow_mesh_patch_block_msg_t, payload) + payload_len,
                     ESPNOW_MESH_MSG_PATCH_BLOCK);
}

static void hil_handle_patch_block_request(const espnow_mesh_event_t *event, uint32_t sequence)
{
    if (event->len < sizeof(espnow_mesh_patch_block_req_msg_t)) {
        return;
    }
    espnow_mesh_patch_block_req_msg_t req = { 0 };
    memcpy(&req, event->data, sizeof(req));
    if (!patch_block_req_fields_valid(&req)) {
        return;
    }
    if (req.sequence != sequence || req.controller_boot_id != s_boot_id) {
        return;
    }
    if (req.patch_hash != hil_patch_hash(req.patch_id, CONFIG_ESPNOW_MESH_HIL_PATCH_BYTES) ||
        req.block_count != hil_patch_block_count_for_len(CONFIG_ESPNOW_MESH_HIL_PATCH_BYTES)) {
        ESP_LOGW(TAG, "HIL patch request mismatch seq=%" PRIu32 " from " MACSTR,
                 req.sequence, MAC2STR(event->mac));
        return;
    }

    int index = find_or_add_satellite(event->mac);
    if (index < 0) {
        return;
    }
    satellite_node_t *node = &s_satellites[index];
    node->last_seen_ms = now_ms();
    node->hil_patch_request_count++;

    esp_err_t err = add_peer_if_needed(event->mac);
    if (err == ESP_OK) {
        err = hil_send_patch_block(event->mac, &req);
    }
    if (err == ESP_OK) {
        node->hil_patch_block_count++;
        ESP_LOGI(TAG,
                 "HIL patch block seq=%" PRIu32 " patch=%" PRIu32
                 " block=%u/%u req=%" PRIu32 " to " MACSTR,
                 req.sequence, req.patch_id, req.block_index + 1, req.block_count,
                 req.request_sequence, MAC2STR(event->mac));
    } else {
        ESP_LOGW(TAG,
                 "HIL patch block send failed seq=%" PRIu32 " block=%u to " MACSTR ": %s",
                 req.sequence, req.block_index, MAC2STR(event->mac), esp_err_to_name(err));
    }
}

static void hil_note_patch_ready(const espnow_mesh_event_t *event, uint32_t sequence)
{
    if (event->len < sizeof(espnow_mesh_patch_ready_msg_t)) {
        return;
    }
    espnow_mesh_patch_ready_msg_t ready = { 0 };
    memcpy(&ready, event->data, sizeof(ready));
    if (!patch_ready_fields_valid(&ready)) {
        return;
    }
    if (ready.sequence != sequence || ready.controller_boot_id != s_boot_id ||
        ready.patch_hash != hil_patch_hash(ready.patch_id, ready.total_len)) {
        return;
    }

    int index = find_or_add_satellite(event->mac);
    if (index < 0) {
        return;
    }

    satellite_node_t *node = &s_satellites[index];
    node->last_seen_ms = now_ms();
    node->hil_patch_ready_sequence = ready.request_sequence;
    if (!node->expected_current) {
        node->expected_current = true;
    }
    if (!node->hil_patch_ready_current) {
        node->hil_patch_ready_current = true;
        ESP_LOGI(TAG,
                 "HIL patch ready seq=%" PRIu32 " patch=%" PRIu32
                 " from " MACSTR " blocks=%u req=%" PRIu32,
                 ready.sequence, ready.patch_id, MAC2STR(event->mac), ready.blocks_received,
                 ready.request_sequence);
    }
}

static esp_err_t hil_send_patch_apply(const hil_patch_case_t *hil_case, uint32_t sequence,
                                      uint64_t apply_at_mesh_us)
{
    espnow_mesh_patch_apply_msg_t apply = {
        .magic = ESPNOW_MESH_MAGIC,
        .version = ESPNOW_MESH_VERSION,
        .type = ESPNOW_MESH_MSG_PATCH_APPLY,
        .sequence = sequence,
        .controller_boot_id = s_boot_id,
        .patch_id = hil_case->patch_id,
        .patch_hash = hil_patch_hash(hil_case->patch_id, CONFIG_ESPNOW_MESH_HIL_PATCH_BYTES),
        .apply_at_mesh_us = apply_at_mesh_us,
    };

    return mesh_send(BROADCAST_MAC, &apply, sizeof(apply), ESPNOW_MESH_MSG_PATCH_APPLY);
}

static void hil_handle_controller_event(const espnow_mesh_event_t *event, uint32_t sequence)
{
    if (event->type == ESPNOW_MESH_EVENT_SEND) {
        if (event->send_status != ESP_NOW_SEND_SUCCESS) {
            ESP_LOGW(TAG, "ESP-NOW send callback failed for " MACSTR, MAC2STR(event->mac));
        }
        return;
    }

#if CONFIG_ESPNOW_MESH_REGISTRATION_AP_ENABLE
    if (event->type == ESPNOW_MESH_EVENT_REGISTRATION) {
        controller_note_registered_satellite(event->mac);
        return;
    }
#endif

    uint8_t type = 0;
    if (!mesh_rx_packet_type(event, &type)) {
        return;
    }

    switch (type) {
    case ESPNOW_MESH_MSG_TIME_REQ:
        handle_time_request(event);
        break;
    case ESPNOW_MESH_MSG_PATCH_BLOCK_REQ:
        hil_handle_patch_block_request(event, sequence);
        break;
    case ESPNOW_MESH_MSG_PATCH_READY:
        hil_note_patch_ready(event, sequence);
        break;
    case ESPNOW_MESH_MSG_HIL_ACK: {
        if (event->len < sizeof(espnow_mesh_hil_ack_msg_t)) {
            break;
        }
        espnow_mesh_hil_ack_msg_t ack = { 0 };
        memcpy(&ack, event->data, sizeof(ack));
        hil_note_ack(event->mac, &ack, sequence);
        break;
    }
    case ESPNOW_MESH_MSG_ACK: {
        if (event->len < sizeof(espnow_mesh_ack_msg_t)) {
            break;
        }
        int index = find_or_add_satellite(event->mac);
        if (index >= 0) {
            s_satellites[index].last_seen_ms = now_ms();
        }
        break;
    }
    default:
        break;
    }
}

static void hil_drain_until(uint32_t deadline_ms, uint32_t sequence)
{
    drain_events_until(deadline_ms, sequence, hil_handle_controller_event);
}

static esp_err_t hil_send_cmd(const uint8_t *dest_mac, const hil_case_t *hil_case,
                              uint32_t sequence, uint32_t attempt, uint64_t apply_at_mesh_us)
{
    espnow_mesh_hil_cmd_msg_t msg = {
        .magic = ESPNOW_MESH_MAGIC,
        .version = ESPNOW_MESH_VERSION,
        .type = ESPNOW_MESH_MSG_HIL_CMD,
        .kind = hil_case->kind,
        .sequence = sequence,
        .controller_boot_id = s_boot_id,
        .retry_index = attempt,
        .test_case = sequence,
        .command_id = sequence,
        .apply_at_mesh_us = apply_at_mesh_us,
        .patch_id = hil_case->patch_id,
        .param_id = hil_case->param_id,
        .param_value = hil_case->param_value,
        .rx_drop_pct = hil_case->rx_drop_pct,
        .ack_drop_pct = hil_case->ack_drop_pct,
        .duplicate_send_count = hil_case->duplicate_send_count,
        .time_resp_drop_pct = hil_case->time_resp_drop_pct,
        .fault_seed = 0x48494c00u ^ sequence,
    };

    return mesh_send(dest_mac, &msg, sizeof(msg), ESPNOW_MESH_MSG_HIL_CMD);
}

static void hil_record_broadcast_attempt(void)
{
    for (int i = 0; i < CONFIG_ESPNOW_MESH_MAX_SATELLITES; ++i) {
        satellite_node_t *node = &s_satellites[i];
        if (node->used && node->expected_current && !node->hil_acked_current) {
            node->hil_sends_current++;
        }
    }
}

static void hil_schedule_missing(uint32_t sequence)
{
    uint32_t current_ms = now_ms();
    for (int i = 0; i < CONFIG_ESPNOW_MESH_MAX_SATELLITES; ++i) {
        satellite_node_t *node = &s_satellites[i];
        if (node->used && node->expected_current && !node->hil_acked_current &&
            node->hil_sends_current < CONFIG_ESPNOW_MESH_RELIABLE_MAX_ATTEMPTS) {
            schedule_next_retry(node, i, sequence, current_ms);
        }
    }
}

static void hil_send_retries_until_done(const hil_case_t *hil_case, uint32_t sequence,
                                        uint64_t apply_at_mesh_us, uint32_t deadline_ms)
{
    while (!hil_complete() && (int32_t)(deadline_ms - now_ms()) > 0) {
        uint32_t next_retry_ms = deadline_ms;
        uint32_t current_ms = now_ms();

        for (int i = 0; i < CONFIG_ESPNOW_MESH_MAX_SATELLITES; ++i) {
            satellite_node_t *node = &s_satellites[i];
            if (!node->used || !node->expected_current || node->hil_acked_current ||
                node->hil_sends_current >= CONFIG_ESPNOW_MESH_RELIABLE_MAX_ATTEMPTS) {
                continue;
            }

            if (!node->retry_scheduled || retry_time_due(current_ms, node->next_retry_ms)) {
                uint32_t attempt = node->hil_sends_current + 1;
                esp_err_t err = add_peer_if_needed(node->mac);
                if (err == ESP_OK) {
                    err = hil_send_cmd(node->mac, hil_case, sequence, attempt, apply_at_mesh_us);
                }
                node->hil_sends_current++;
                ESP_LOGI(TAG, "HIL unicast retry case=%" PRIu32 " attempt=%" PRIu32
                              " to " MACSTR " status=%s",
                         sequence, attempt, MAC2STR(node->mac), esp_err_to_name(err));
                schedule_next_retry(node, i, sequence, current_ms);
            }
            if (node->retry_scheduled && (int32_t)(node->next_retry_ms - next_retry_ms) < 0) {
                next_retry_ms = node->next_retry_ms;
            }
        }

        if ((int32_t)(next_retry_ms - now_ms()) <= 0) {
            next_retry_ms = now_ms() + 1;
        }
        hil_drain_until(next_retry_ms, sequence);
    }
}

static void hil_run_case(const hil_case_t *hil_case, uint32_t sequence)
{
    hil_reset_ack_state();

    uint64_t apply_at_mesh_us = now_us() + CONFIG_ESPNOW_MESH_HIL_APPLY_DELAY_US;
    hil_schedule_apply_marker((int64_t)apply_at_mesh_us);

    if (hil_case->time_resp_drop_pct > 0) {
        s_hil_time_resp_drop_pct = hil_case->time_resp_drop_pct;
        s_hil_time_resp_drop_seed = 0x54535900u ^ sequence;
        s_hil_time_resp_drop_until_ms =
            now_ms() + (CONFIG_ESPNOW_MESH_HIL_APPLY_DELAY_US / 1000u) +
            CONFIG_ESPNOW_MESH_HIL_CASE_GAP_MS;
    } else {
        s_hil_time_resp_drop_pct = 0;
        s_hil_time_resp_drop_seed = 0;
        s_hil_time_resp_drop_until_ms = 0;
    }

    uint32_t deadline_ms = now_ms() + CONFIG_ESPNOW_MESH_RELIABLE_DEADLINE_MS;
    ESP_LOGI(TAG,
             "HIL CASE START id=%" PRIu32 " name=%s expected=%d apply_at=%" PRIu64
             " rx_drop=%u ack_drop=%u time_resp_drop=%u",
             sequence, hil_case->name, hil_expected_count(), apply_at_mesh_us,
             hil_case->rx_drop_pct, hil_case->ack_drop_pct, hil_case->time_resp_drop_pct);

    for (uint32_t attempt = 1; attempt <= CONFIG_ESPNOW_MESH_SEND_RETRIES; ++attempt) {
        esp_err_t err = hil_send_cmd(BROADCAST_MAC, hil_case, sequence, attempt,
                                     apply_at_mesh_us);
        hil_record_broadcast_attempt();
        ESP_LOGI(TAG, "HIL broadcast case=%" PRIu32 " attempt=%" PRIu32 " status=%s",
                 sequence, attempt, esp_err_to_name(err));

        for (uint32_t duplicate = 0; duplicate < hil_case->duplicate_send_count; ++duplicate) {
            err = hil_send_cmd(BROADCAST_MAC, hil_case, sequence, attempt,
                               apply_at_mesh_us);
            ESP_LOGI(TAG, "HIL duplicate broadcast case=%" PRIu32 " attempt=%" PRIu32
                          " duplicate=%" PRIu32 " status=%s",
                     sequence, attempt, duplicate + 1, esp_err_to_name(err));
        }

        hil_drain_until(now_ms() + CONFIG_ESPNOW_MESH_RETRY_INTERVAL_MS, sequence);
        if (hil_complete()) {
            break;
        }
    }

    if (!hil_complete()) {
        hil_schedule_missing(sequence);
        hil_send_retries_until_done(hil_case, sequence, apply_at_mesh_us, deadline_ms);
    }

    int expected = hil_expected_count();
    int acked = hil_acked_count();
    ESP_LOGI(TAG, "HIL CASE RESULT id=%" PRIu32 " name=%s acked=%d/%d status=%s",
             sequence, hil_case->name, acked, expected,
             (expected > 0 && acked == expected) ? "PASS" : "DEGRADED");

    hil_drain_until(now_ms() + CONFIG_ESPNOW_MESH_HIL_CASE_GAP_MS, sequence);
}

static void hil_run_patch_case(const hil_patch_case_t *hil_case, uint32_t sequence)
{
    hil_reset_patch_state();

    uint32_t deadline_ms = now_ms() + CONFIG_ESPNOW_MESH_HIL_PATCH_DEADLINE_MS;
    uint32_t offer_count = 0;
    ESP_LOGI(TAG,
             "HIL PATCH CASE START id=%" PRIu32 " name=%s expected=%d patch=%" PRIu32
             " bytes=%d block=%d req_drop=%u block_drop=%u ready_drop=%u",
             sequence, hil_case->name, hil_expected_count(), hil_case->patch_id,
             CONFIG_ESPNOW_MESH_HIL_PATCH_BYTES, CONFIG_ESPNOW_MESH_HIL_PATCH_BLOCK_BYTES,
             hil_case->block_req_drop_pct, hil_case->block_drop_pct,
             hil_case->ready_drop_pct);

    while (!hil_patch_complete() && (int32_t)(deadline_ms - now_ms()) > 0) {
        esp_err_t err = hil_send_patch_offer(hil_case, sequence);
        offer_count++;
        ESP_LOGI(TAG, "HIL patch offer case=%" PRIu32 " offer=%" PRIu32 " status=%s",
                 sequence, offer_count, esp_err_to_name(err));
        hil_drain_until(now_ms() + CONFIG_ESPNOW_MESH_HIL_PATCH_OFFER_INTERVAL_MS,
                        sequence);
    }

    int expected = hil_expected_count();
    int ready = hil_patch_ready_count();
    bool complete = expected > 0 && ready == expected;
    ESP_LOGI(TAG,
             "HIL PATCH STAGE RESULT id=%" PRIu32 " name=%s ready=%d/%d offers=%" PRIu32
             " status=%s",
             sequence, hil_case->name, ready, expected, offer_count,
             complete ? "PASS" : "DEGRADED");

    if (complete) {
        uint64_t apply_at_mesh_us = now_us() + CONFIG_ESPNOW_MESH_HIL_APPLY_DELAY_US;
        hil_schedule_apply_marker((int64_t)apply_at_mesh_us);

        for (uint32_t attempt = 1; attempt <= CONFIG_ESPNOW_MESH_SEND_RETRIES; ++attempt) {
            esp_err_t err = hil_send_patch_apply(hil_case, sequence, apply_at_mesh_us);
            ESP_LOGI(TAG,
                     "HIL patch apply case=%" PRIu32 " attempt=%" PRIu32
                     " apply_at=%" PRIu64 " status=%s",
                     sequence, attempt, apply_at_mesh_us, esp_err_to_name(err));
            hil_drain_until(now_ms() + CONFIG_ESPNOW_MESH_RETRY_INTERVAL_MS, sequence);
        }
    }

    hil_drain_until(now_ms() + CONFIG_ESPNOW_MESH_HIL_CASE_GAP_MS, sequence);
}

static void hil_controller_run(void)
{
    ESP_LOGI(TAG, "running HIL controller test mode; start_delay=%dms expected_satellites=%d",
             CONFIG_ESPNOW_MESH_HIL_CONTROLLER_START_DELAY_MS,
             CONFIG_ESPNOW_MESH_HIL_EXPECTED_SATELLITES);
    vTaskDelay(pdMS_TO_TICKS(CONFIG_ESPNOW_MESH_HIL_CONTROLLER_START_DELAY_MS));

    uint32_t settle_deadline = now_ms() + CONFIG_ESPNOW_MESH_HIL_DISCOVERY_MS;
    while ((int32_t)(settle_deadline - now_ms()) > 0) {
        (void)send_controller_packet(BROADCAST_MAC, 1, 1);
        hil_drain_until(now_ms() + 300, 1);
        if (known_satellite_count() >= CONFIG_ESPNOW_MESH_HIL_EXPECTED_SATELLITES) {
            break;
        }
    }
    /* Even with all expected satellites discovered, give them one settle window
     * for time-sync exchanges before the first case fires. */
    hil_drain_until(now_ms() + CONFIG_ESPNOW_MESH_HIL_DISCOVERY_MS, 1);

    ESP_LOGI(TAG, "HIL discovery complete known_satellites=%d expected=%d",
             known_satellite_count(), CONFIG_ESPNOW_MESH_HIL_EXPECTED_SATELLITES);

    uint32_t sequence = 1;
    for (uint32_t i = 0; i < sizeof(HIL_CASES) / sizeof(HIL_CASES[0]); ++i) {
        hil_run_case(&HIL_CASES[i], sequence++);
    }

    for (uint32_t i = 0; i < sizeof(PATCH_HIL_CASES) / sizeof(PATCH_HIL_CASES[0]); ++i) {
        hil_run_patch_case(&PATCH_HIL_CASES[i], sequence++);
    }

    ESP_LOGI(TAG, "HIL TEST PLAN COMPLETE cases=%u",
             (unsigned)((sizeof(HIL_CASES) / sizeof(HIL_CASES[0])) +
                        (sizeof(PATCH_HIL_CASES) / sizeof(PATCH_HIL_CASES[0]))));
    while (true) {
        log_event_queue_drops_if_any();
        hil_drain_until(now_ms() + 1000, 0);
    }
}
#endif

#if !CONFIG_ESPNOW_MESH_HIL_TEST_ENABLE
/*
 * Free satellite slots that have gone silent past the configured window. This
 * stops the controller from retrying (and holding the ACK deadline for) nodes
 * that have permanently left, and returns their peer and table slots. Runs on
 * the mesh task at the top of each sequence.
 */
static void expire_stale_satellites(void)
{
#if CONFIG_ESPNOW_MESH_SATELLITE_EXPIRE_MS > 0
    uint32_t current_ms = now_ms();
    for (int i = 0; i < CONFIG_ESPNOW_MESH_MAX_SATELLITES; ++i) {
        satellite_node_t *node = &s_satellites[i];
        if (!node->used ||
            (int32_t)(current_ms - node->last_seen_ms) < CONFIG_ESPNOW_MESH_SATELLITE_EXPIRE_MS) {
            continue;
        }

        ESP_LOGW(TAG, "evicting stale satellite " MACSTR " last_seen=%" PRIu32 "ms ago",
                 MAC2STR(node->mac), current_ms - node->last_seen_ms);
        if (s_espnow_ready) {
            (void)esp_now_del_peer(node->mac);
        }
        portENTER_CRITICAL(&s_satellite_table_lock);
        memset(node, 0, sizeof(*node));
        portEXIT_CRITICAL(&s_satellite_table_lock);
    }
#endif
}
#endif

#if !CONFIG_ESPNOW_MESH_HIL_TEST_ENABLE
/* Move a queued application payload into the active-sequence buffer. Returns
 * true if there was data to send. */
static bool controller_take_pending_payload(void)
{
    bool have = false;
    portENTER_CRITICAL(&s_pending_payload_lock);
    if (s_have_pending_payload) {
        s_tx_payload_len = s_pending_payload_len;
        memcpy(s_tx_payload, s_pending_payload, s_pending_payload_len);
        s_have_pending_payload = false;
        have = true;
    }
    portEXIT_CRITICAL(&s_pending_payload_lock);
    return have;
}
#endif

static void controller_run(void)
{
#if CONFIG_ESPNOW_MESH_HIL_TEST_ENABLE
    hil_controller_run();
#else
    ESP_LOGI(TAG, "running as controller");
    uint32_t sequence = 0;

    while (true) {
        log_event_queue_drops_if_any();
        expire_stale_satellites();

        /* Wait for the application to queue a payload. While idle, keep
         * answering ACKs and time-sync requests; short slices bound send
         * latency to ~100 ms. */
        if (!controller_take_pending_payload()) {
            drain_controller_events_until(now_ms() + 100, s_controller_active_sequence);
            continue;
        }

        sequence++;
        if (sequence == 0) {
            sequence = 1;
        }
        s_controller_active_sequence = sequence;

        begin_reliable_sequence();
        uint32_t round_start_ms = now_ms();
        uint32_t deadline_ms = round_start_ms + CONFIG_ESPNOW_MESH_RELIABLE_DEADLINE_MS;
        ESP_LOGI(TAG, "critical broadcast seq=%" PRIu32 " known_expected=%d", sequence,
                 current_expected_count());

        for (uint32_t retry = 1; retry <= CONFIG_ESPNOW_MESH_SEND_RETRIES; ++retry) {
            esp_err_t err = send_controller_packet(BROADCAST_MAC, sequence, retry);
            record_broadcast_attempt_for_expected();
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "broadcast seq=%" PRIu32 " attempt=%" PRIu32 " failed: %s", sequence,
                         retry, esp_err_to_name(err));
            }
            drain_controller_events_until(now_ms() + CONFIG_ESPNOW_MESH_RETRY_INTERVAL_MS,
                                          sequence);
            if (reliable_sequence_complete()) {
                break;
            }
        }

        if (!reliable_sequence_complete()) {
            schedule_missing_after_broadcast(sequence);
            reliable_retry_until_done_or_deadline(sequence, deadline_ms);
        }

        log_sequence_summary(sequence);
        s_controller_last_sequence = sequence;
        s_controller_last_expected = (uint32_t)current_expected_count();
        s_controller_last_acked = (uint32_t)current_acked_count();
        s_controller_last_sequence_complete =
            s_controller_last_expected > 0 && s_controller_last_acked == s_controller_last_expected;

        drain_controller_events_until(now_ms() + CONFIG_ESPNOW_MESH_DATA_INTERVAL_MS, sequence);
    }
#endif
}

#else

static bool s_have_controller;
static uint8_t s_controller_mac[ESP_NOW_ETH_ALEN];
static bool s_have_controller_boot_id;
static uint32_t s_controller_boot_id;
static bool s_have_last_sequence;
static uint32_t s_last_sequence;
static uint32_t s_next_time_sync_ms;
static espnow_mesh_rx_cb_t s_rx_cb;
static void *s_rx_cb_ctx;
#if CONFIG_ESPNOW_MESH_CONTROLLER_TIMEOUT_MS > 0
static uint32_t s_last_controller_rx_ms;
#endif
static uint32_t s_time_sync_sequence;
static uint32_t s_pending_time_sync_sequence;
static uint64_t s_pending_time_sync_tx_us;
static bool s_have_pending_time_sync;
#if CONFIG_ESPNOW_MESH_HIL_TEST_ENABLE
static bool s_have_last_hil_sequence;
static uint32_t s_last_hil_sequence;
static uint32_t s_current_patch_id;
static uint32_t s_current_param_id;
static int32_t s_current_param_value;
static uint8_t s_patch_staging[CONFIG_ESPNOW_MESH_HIL_PATCH_BYTES];
static uint32_t s_patch_received_mask;
static uint32_t s_patch_sequence;
static uint32_t s_patch_controller_boot_id;
static uint32_t s_patch_id;
static uint32_t s_patch_total_len;
static uint16_t s_patch_block_size;
static uint16_t s_patch_block_count;
static uint32_t s_patch_hash;
static uint32_t s_patch_fault_seed;
static uint32_t s_patch_request_sequence;
static uint8_t s_patch_block_req_drop_pct;
static uint8_t s_patch_block_drop_pct;
static uint8_t s_patch_ready_drop_pct;
static bool s_patch_offer_active;
static bool s_patch_ready;
#endif


#if CONFIG_ESPNOW_MESH_HIL_TEST_ENABLE
static void hil_patch_reset_staging(void)
{
    s_patch_received_mask = 0;
    s_patch_sequence = 0;
    s_patch_controller_boot_id = 0;
    s_patch_id = 0;
    s_patch_total_len = 0;
    s_patch_block_size = 0;
    s_patch_block_count = 0;
    s_patch_hash = 0;
    s_patch_fault_seed = 0;
    s_patch_request_sequence = 0;
    s_patch_block_req_drop_pct = 0;
    s_patch_block_drop_pct = 0;
    s_patch_ready_drop_pct = 0;
    s_patch_offer_active = false;
    s_patch_ready = false;
}

static bool hil_softap_discovery_required(void)
{
#if CONFIG_ESPNOW_MESH_REGISTRATION_AP_ENABLE
    return CONFIG_ESPNOW_MESH_HIL_REQUIRE_SOFTAP_DISCOVERY;
#else
    return false;
#endif
}

static bool hil_softap_discovery_ready(const char *packet_name)
{
    if (!hil_softap_discovery_required() || s_hil_registration_discovered) {
        return true;
    }

    ESP_LOGW(TAG, "HIL ignoring %s until registration SoftAP discovery succeeds",
             packet_name);
    return false;
}

static bool hil_accept_controller_from_event(const espnow_mesh_event_t *event,
                                             uint32_t controller_boot_id,
                                             const char *packet_name)
{
    if (!hil_softap_discovery_ready(packet_name)) {
        return false;
    }

    if (!s_have_controller) {
        memcpy(s_controller_mac, event->mac, ESP_NOW_ETH_ALEN);
        s_have_controller = true;
        s_next_time_sync_ms = 0;
        ESP_LOGI(TAG, "locked to HIL controller " MACSTR " via %s",
                 MAC2STR(s_controller_mac), packet_name);
    } else if (!mac_equal(event->mac, s_controller_mac)) {
        ESP_LOGW(TAG, "ignoring %s from extra controller " MACSTR, packet_name,
                 MAC2STR(event->mac));
        return false;
    }

    if (!s_have_controller_boot_id || controller_boot_id != s_controller_boot_id) {
        s_controller_boot_id = controller_boot_id;
        s_have_controller_boot_id = true;
        s_have_last_sequence = false;
        s_have_last_hil_sequence = false;
        hil_patch_reset_staging();
        ESP_LOGI(TAG, "HIL controller boot/session changed to 0x%08" PRIx32,
                 s_controller_boot_id);
    }

    esp_err_t err = add_peer_if_needed(event->mac);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "could not add HIL controller peer " MACSTR ": %s",
                 MAC2STR(event->mac), esp_err_to_name(err));
        return false;
    }
    return true;
}

static int hil_patch_next_missing_block(void)
{
    for (uint16_t block_index = 0; block_index < s_patch_block_count; ++block_index) {
        if ((s_patch_received_mask & (1u << block_index)) == 0) {
            return block_index;
        }
    }
    return -1;
}

static uint16_t hil_patch_received_count(void)
{
    uint16_t count = 0;
    for (uint16_t block_index = 0; block_index < s_patch_block_count; ++block_index) {
        if ((s_patch_received_mask & (1u << block_index)) != 0) {
            count++;
        }
    }
    return count;
}

static esp_err_t hil_patch_send_ready(const uint8_t *controller_mac)
{
    s_patch_request_sequence++;
    if (hil_fault_drop(s_patch_ready_drop_pct, s_patch_fault_seed, s_patch_sequence,
                       s_patch_request_sequence, ESPNOW_MESH_HIL_FAULT_TX_PATCH_READY)) {
        ESP_LOGW(TAG,
                 "HIL injected patch ready drop seq=%" PRIu32 " patch=%" PRIu32
                 " req=%" PRIu32,
                 s_patch_sequence, s_patch_id, s_patch_request_sequence);
        return ESP_OK;
    }

    espnow_mesh_patch_ready_msg_t ready = {
        .magic = ESPNOW_MESH_MAGIC,
        .version = ESPNOW_MESH_VERSION,
        .type = ESPNOW_MESH_MSG_PATCH_READY,
        .sequence = s_patch_sequence,
        .controller_boot_id = s_patch_controller_boot_id,
        .patch_id = s_patch_id,
        .patch_hash = s_patch_hash,
        .request_sequence = s_patch_request_sequence,
        .blocks_received = hil_patch_received_count(),
        .total_len = s_patch_total_len,
        .uptime_ms = now_ms(),
    };
    memcpy(ready.node_mac, s_self_mac, ESP_NOW_ETH_ALEN);
    return mesh_send(controller_mac, &ready, sizeof(ready), ESPNOW_MESH_MSG_PATCH_READY);
}

static void hil_patch_request_next_missing(const uint8_t *controller_mac)
{
    if (!s_patch_offer_active) {
        return;
    }
    if (s_patch_ready) {
        esp_err_t err = hil_patch_send_ready(controller_mac);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "HIL patch ready resend failed: %s", esp_err_to_name(err));
        }
        return;
    }

    int missing_block = hil_patch_next_missing_block();
    if (missing_block < 0) {
        return;
    }

    s_patch_request_sequence++;
    if (hil_fault_drop(s_patch_block_req_drop_pct, s_patch_fault_seed, s_patch_sequence,
                       s_patch_request_sequence, ESPNOW_MESH_HIL_FAULT_TX_PATCH_BLOCK_REQ)) {
        ESP_LOGW(TAG,
                 "HIL injected patch request drop seq=%" PRIu32 " patch=%" PRIu32
                 " block=%d req=%" PRIu32,
                 s_patch_sequence, s_patch_id, missing_block, s_patch_request_sequence);
        return;
    }

    espnow_mesh_patch_block_req_msg_t req = {
        .magic = ESPNOW_MESH_MAGIC,
        .version = ESPNOW_MESH_VERSION,
        .type = ESPNOW_MESH_MSG_PATCH_BLOCK_REQ,
        .sequence = s_patch_sequence,
        .controller_boot_id = s_patch_controller_boot_id,
        .patch_id = s_patch_id,
        .block_index = (uint16_t)missing_block,
        .block_count = s_patch_block_count,
        .patch_hash = s_patch_hash,
        .request_sequence = s_patch_request_sequence,
        .block_drop_pct = s_patch_block_drop_pct,
        .fault_seed = s_patch_fault_seed,
    };

    esp_err_t err = mesh_send(controller_mac, &req, sizeof(req),
                              ESPNOW_MESH_MSG_PATCH_BLOCK_REQ);
    if (err != ESP_OK) {
        ESP_LOGW(TAG,
                 "HIL patch block request failed seq=%" PRIu32 " patch=%" PRIu32
                 " block=%d: %s",
                 s_patch_sequence, s_patch_id, missing_block, esp_err_to_name(err));
    }
}

static void handle_satellite_patch_offer(const espnow_mesh_event_t *event)
{
    if (event->len < sizeof(espnow_mesh_patch_offer_msg_t)) {
        return;
    }
    espnow_mesh_patch_offer_msg_t offer = { 0 };
    memcpy(&offer, event->data, sizeof(offer));
    if (!patch_offer_fields_valid(&offer)) {
        return;
    }
    if (offer.block_count != hil_patch_block_count_for_len(offer.total_len) ||
        offer.patch_hash != hil_patch_hash(offer.patch_id, offer.total_len)) {
        ESP_LOGW(TAG, "reject patch offer seq=%" PRIu32 " patch=%" PRIu32 " hash mismatch",
                 offer.sequence, offer.patch_id);
        return;
    }
    if (!hil_accept_controller_from_event(event, offer.controller_boot_id, "patch offer")) {
        return;
    }

    bool same_offer = s_patch_offer_active && s_patch_sequence == offer.sequence &&
                      s_patch_controller_boot_id == offer.controller_boot_id &&
                      s_patch_id == offer.patch_id && s_patch_hash == offer.patch_hash &&
                      s_patch_total_len == offer.total_len;
    if (!same_offer) {
        s_patch_received_mask = 0;
        s_patch_sequence = offer.sequence;
        s_patch_controller_boot_id = offer.controller_boot_id;
        s_patch_id = offer.patch_id;
        s_patch_total_len = offer.total_len;
        s_patch_block_size = offer.block_size;
        s_patch_block_count = offer.block_count;
        s_patch_hash = offer.patch_hash;
        s_patch_fault_seed = offer.fault_seed;
        s_patch_request_sequence = 0;
        s_patch_block_req_drop_pct = offer.block_req_drop_pct;
        s_patch_block_drop_pct = offer.block_drop_pct;
        s_patch_ready_drop_pct = offer.ready_drop_pct;
        s_patch_offer_active = true;
        s_patch_ready = false;
        ESP_LOGI(TAG,
                 "HIL patch offer accepted seq=%" PRIu32 " patch=%" PRIu32
                 " bytes=%" PRIu32 " blocks=%u hash=0x%08" PRIx32,
                 s_patch_sequence, s_patch_id, s_patch_total_len, s_patch_block_count,
                 s_patch_hash);
    }

    hil_patch_request_next_missing(event->mac);
}

static void handle_satellite_patch_block(const espnow_mesh_event_t *event)
{
    if (event->len < offsetof(espnow_mesh_patch_block_msg_t, payload)) {
        return;
    }
    espnow_mesh_patch_block_msg_t block = { 0 };
    memcpy(&block, event->data, event->len < sizeof(block) ? event->len : sizeof(block));
    if (!patch_block_fields_valid(&block, event->len)) {
        return;
    }
    if (!s_patch_offer_active || !s_have_controller || !mac_equal(event->mac, s_controller_mac)) {
        return;
    }
    if (block.sequence != s_patch_sequence ||
        block.controller_boot_id != s_patch_controller_boot_id ||
        block.patch_id != s_patch_id || block.patch_hash != s_patch_hash ||
        block.block_count != s_patch_block_count || block.total_len != s_patch_total_len ||
        block.block_offset != (uint32_t)block.block_index * s_patch_block_size) {
        return;
    }
    if (hil_hash_bytes(block.payload, block.payload_len) != block.block_hash) {
        ESP_LOGW(TAG,
                 "HIL patch block hash mismatch seq=%" PRIu32 " patch=%" PRIu32
                 " block=%u",
                 block.sequence, block.patch_id, block.block_index);
        return;
    }
    if (hil_fault_drop(block.block_drop_pct, block.fault_seed, block.sequence,
                       block.request_sequence, ESPNOW_MESH_HIL_FAULT_RX_PATCH_BLOCK)) {
        ESP_LOGW(TAG,
                 "HIL injected patch block drop seq=%" PRIu32 " patch=%" PRIu32
                 " block=%u req=%" PRIu32,
                 block.sequence, block.patch_id, block.block_index, block.request_sequence);
        return;
    }

    memcpy(&s_patch_staging[block.block_offset], block.payload, block.payload_len);
    s_patch_received_mask |= 1u << block.block_index;
    ESP_LOGI(TAG,
             "HIL patch block stored seq=%" PRIu32 " patch=%" PRIu32
             " block=%u/%u req=%" PRIu32 " received=%u",
             block.sequence, block.patch_id, block.block_index + 1, block.block_count,
             block.request_sequence, hil_patch_received_count());

    if (hil_patch_next_missing_block() < 0) {
        uint32_t staged_hash = hil_hash_bytes(s_patch_staging, s_patch_total_len);
        if (staged_hash != s_patch_hash) {
            ESP_LOGW(TAG,
                     "HIL patch full hash mismatch seq=%" PRIu32
                     " patch=%" PRIu32 " got=0x%08" PRIx32 " want=0x%08" PRIx32,
                     s_patch_sequence, s_patch_id, staged_hash, s_patch_hash);
            s_patch_received_mask = 0;
            hil_patch_request_next_missing(event->mac);
            return;
        }
        s_patch_ready = true;
        s_current_patch_id = s_patch_id;
        ESP_LOGI(TAG, "HIL patch staged seq=%" PRIu32 " patch=%" PRIu32 " hash=0x%08" PRIx32,
                 s_patch_sequence, s_patch_id, s_patch_hash);
        esp_err_t err = hil_patch_send_ready(event->mac);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "HIL patch ready send failed: %s", esp_err_to_name(err));
        }
        return;
    }

    hil_patch_request_next_missing(event->mac);
}

static void handle_satellite_patch_apply(const espnow_mesh_event_t *event)
{
    if (event->len < sizeof(espnow_mesh_patch_apply_msg_t)) {
        return;
    }
    espnow_mesh_patch_apply_msg_t apply = { 0 };
    memcpy(&apply, event->data, sizeof(apply));
    if (!hil_accept_controller_from_event(event, apply.controller_boot_id, "patch apply")) {
        return;
    }
    if (!s_patch_ready || !s_patch_offer_active || apply.sequence != s_patch_sequence ||
        apply.controller_boot_id != s_patch_controller_boot_id || apply.patch_id != s_patch_id ||
        apply.patch_hash != s_patch_hash) {
        ESP_LOGW(TAG,
                 "HIL ignoring patch apply seq=%" PRIu32 " patch=%" PRIu32
                 " ready=%u active=%u",
                 apply.sequence, apply.patch_id, s_patch_ready ? 1 : 0,
                 s_patch_offer_active ? 1 : 0);
        return;
    }

    s_current_patch_id = apply.patch_id;
    hil_schedule_apply_marker((int64_t)apply.apply_at_mesh_us);
    ESP_LOGI(TAG,
             "HIL patch apply scheduled seq=%" PRIu32 " patch=%" PRIu32
             " apply_at=%" PRIu64 " mesh_now=%" PRId64,
             apply.sequence, apply.patch_id, apply.apply_at_mesh_us, mesh_time_us());
}
#endif

static bool data_msg_fields_valid(const espnow_mesh_data_msg_t *msg, uint16_t len)
{
    if (msg->payload_len > sizeof(msg->payload)) {
        return false;
    }
    return len >= offsetof(espnow_mesh_data_msg_t, payload) + msg->payload_len;
}

static esp_err_t send_satellite_ack(const uint8_t *controller_mac, uint32_t sequence,
                                    uint32_t controller_boot_id, bool accepted, bool duplicate,
                                    int8_t rssi)
{
    espnow_mesh_ack_msg_t ack = {
        .magic = ESPNOW_MESH_MAGIC,
        .version = ESPNOW_MESH_VERSION,
        .type = ESPNOW_MESH_MSG_ACK,
        .sequence = sequence,
        .controller_boot_id = controller_boot_id,
        .accepted = accepted ? 1 : 0,
        .duplicate = duplicate ? 1 : 0,
        .last_rssi = rssi,
        .channel = s_mesh_channel,
        .uptime_ms = now_ms(),
    };
    memcpy(ack.node_mac, s_self_mac, ESP_NOW_ETH_ALEN);

    return mesh_send(controller_mac, &ack, sizeof(ack), ESPNOW_MESH_MSG_ACK);
}

#if CONFIG_ESPNOW_MESH_HIL_TEST_ENABLE
static esp_err_t send_satellite_hil_ack(const uint8_t *controller_mac,
                                        const espnow_mesh_hil_cmd_msg_t *cmd,
                                        bool accepted, bool duplicate, int8_t rssi)
{
    espnow_mesh_hil_ack_msg_t ack = {
        .magic = ESPNOW_MESH_MAGIC,
        .version = ESPNOW_MESH_VERSION,
        .type = ESPNOW_MESH_MSG_HIL_ACK,
        .kind = cmd->kind,
        .sequence = cmd->sequence,
        .controller_boot_id = cmd->controller_boot_id,
        .test_case = cmd->test_case,
        .command_id = cmd->command_id,
        .accepted = accepted ? 1 : 0,
        .duplicate = duplicate ? 1 : 0,
        .last_rssi = rssi,
        .channel = s_mesh_channel,
        .uptime_ms = now_ms(),
        .patch_id = s_current_patch_id,
        .param_id = s_current_param_id,
        .param_value = s_current_param_value,
    };
    memcpy(ack.node_mac, s_self_mac, ESP_NOW_ETH_ALEN);
    return mesh_send(controller_mac, &ack, sizeof(ack), ESPNOW_MESH_MSG_HIL_ACK);
}
#endif

static void request_time_sync_if_due(void)
{
    if (!s_have_controller) {
        return;
    }

    uint32_t current_ms = now_ms();
    if ((int32_t)(s_next_time_sync_ms - current_ms) > 0) {
        return;
    }

    s_time_sync_sequence++;
    if (s_time_sync_sequence == 0) {
        s_time_sync_sequence = 1;
    }

    uint64_t satellite_tx_us = now_us();
    espnow_mesh_time_req_msg_t req = {
        .magic = ESPNOW_MESH_MAGIC,
        .version = ESPNOW_MESH_VERSION,
        .type = ESPNOW_MESH_MSG_TIME_REQ,
        .sequence = s_time_sync_sequence,
        .satellite_tx_us = satellite_tx_us,
    };
    memcpy(req.node_mac, s_self_mac, ESP_NOW_ETH_ALEN);

    esp_err_t err = mesh_send(s_controller_mac, &req, sizeof(req), ESPNOW_MESH_MSG_TIME_REQ);
    if (err == ESP_OK) {
        s_have_pending_time_sync = true;
        s_pending_time_sync_sequence = s_time_sync_sequence;
        s_pending_time_sync_tx_us = satellite_tx_us;
    } else {
        ESP_LOGW(TAG, "time request seq=%" PRIu32 " failed: %s", s_time_sync_sequence,
                 esp_err_to_name(err));
    }

    s_next_time_sync_ms = current_ms + CONFIG_ESPNOW_MESH_TIME_SYNC_INTERVAL_MS;
}

static TickType_t time_sync_wait_ticks(void)
{
    if (!s_have_controller) {
        return portMAX_DELAY;
    }

    uint32_t current_ms = now_ms();
    int32_t wait_ms = (int32_t)(s_next_time_sync_ms - current_ms);
    if (wait_ms <= 0) {
        return 0;
    }
    if (wait_ms > 1000) {
        wait_ms = 1000;
    }
    return pdMS_TO_TICKS(wait_ms);
}

#if CONFIG_ESPNOW_MESH_REGISTRATION_AP_ENABLE
static void restore_espnow_channel(void)
{
    esp_err_t err = esp_wifi_set_channel(s_mesh_channel, WIFI_SECOND_CHAN_NONE);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "could not restore ESP-NOW channel %u: %s", s_mesh_channel,
                 esp_err_to_name(err));
    }
}

static bool registration_config_valid(size_t *ssid_len, size_t *password_len)
{
    const char *ssid = CONFIG_ESPNOW_MESH_REGISTRATION_SSID;
    const char *password = CONFIG_ESPNOW_MESH_REGISTRATION_PASSWORD;
    *ssid_len = strlen(ssid);
    *password_len = strlen(password);

    if (*ssid_len == 0 || *ssid_len >= sizeof(((wifi_config_t *)0)->sta.ssid)) {
        ESP_LOGE(TAG, "registration SSID must be 1-%zu characters for satellite scan",
                 sizeof(((wifi_config_t *)0)->sta.ssid) - 1);
        return false;
    }
    if (*password_len > sizeof(((wifi_config_t *)0)->sta.password) - 1 ||
        (*password_len > 0 && *password_len < 8)) {
        ESP_LOGE(TAG, "registration password must be empty or 8-%zu characters",
                 sizeof(((wifi_config_t *)0)->sta.password) - 1);
        return false;
    }

    return true;
}

static bool scan_for_registration_ap(wifi_ap_record_t *ap)
{
    size_t ssid_len = 0;
    size_t password_len = 0;
    if (!registration_config_valid(&ssid_len, &password_len)) {
        return false;
    }
    (void)password_len;

    uint8_t scan_ssid[sizeof(((wifi_config_t *)0)->sta.ssid)] = { 0 };
    memcpy(scan_ssid, CONFIG_ESPNOW_MESH_REGISTRATION_SSID, ssid_len);

    wifi_scan_config_t scan_config = {
        .ssid = scan_ssid,
        .channel = 0,
        .show_hidden = true,
        .scan_type = WIFI_SCAN_TYPE_ACTIVE,
    };
    scan_config.scan_time.active.min = CONFIG_ESPNOW_MESH_REGISTRATION_SCAN_MS;
    scan_config.scan_time.active.max = CONFIG_ESPNOW_MESH_REGISTRATION_SCAN_MS;

    esp_err_t err = esp_wifi_scan_start(&scan_config, true);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "registration AP scan failed: %s", esp_err_to_name(err));
        restore_espnow_channel();
        return false;
    }

    uint16_t ap_count = 1;
    memset(ap, 0, sizeof(*ap));
    err = esp_wifi_scan_get_ap_records(&ap_count, ap);
    (void)esp_wifi_clear_ap_list();
    restore_espnow_channel();

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "registration AP scan results failed: %s", esp_err_to_name(err));
        return false;
    }
    if (ap_count == 0) {
        ESP_LOGW(TAG, "registration AP SSID=\"%s\" not found on any channel",
                 CONFIG_ESPNOW_MESH_REGISTRATION_SSID);
        return false;
    }

    ESP_LOGI(TAG, "found registration AP " MACSTR " rssi=%d channel=%u",
             MAC2STR(ap->bssid), ap->rssi, ap->primary);
    return true;
}

static void connect_to_registration_ap(const wifi_ap_record_t *ap)
{
    size_t ssid_len = 0;
    size_t password_len = 0;
    if (!registration_config_valid(&ssid_len, &password_len)) {
        return;
    }

    wifi_config_t sta_config = { 0 };
    memcpy(sta_config.sta.ssid, CONFIG_ESPNOW_MESH_REGISTRATION_SSID, ssid_len);
    memcpy(sta_config.sta.password, CONFIG_ESPNOW_MESH_REGISTRATION_PASSWORD, password_len);
    sta_config.sta.scan_method = WIFI_FAST_SCAN;
    sta_config.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;
    sta_config.sta.channel = ap->primary;
    sta_config.sta.bssid_set = true;
    memcpy(sta_config.sta.bssid, ap->bssid, ESP_NOW_ETH_ALEN);
    sta_config.sta.threshold.authmode = password_len == 0 ? WIFI_AUTH_OPEN : WIFI_AUTH_WPA2_PSK;
    sta_config.sta.failure_retry_cnt = 1;

    uint8_t previous_channel = s_mesh_channel;
    s_mesh_channel = clamp_wifi_channel(ap->primary);
    ESP_LOGI(TAG, "adopting registration AP channel %u (was %u)", s_mesh_channel,
             previous_channel);
    s_registration_sta_connected = false;
    (void)esp_wifi_disconnect();

    esp_err_t err = esp_wifi_set_config(WIFI_IF_STA, &sta_config);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "registration AP station config failed: %s", esp_err_to_name(err));
        restore_espnow_channel();
        return;
    }

    err = esp_wifi_connect();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "registration AP connect start failed: %s", esp_err_to_name(err));
        restore_espnow_channel();
        return;
    }

    uint32_t deadline_ms = now_ms() + CONFIG_ESPNOW_MESH_REGISTRATION_CONNECT_TIMEOUT_MS;
    while (!s_registration_sta_connected && (int32_t)(deadline_ms - now_ms()) > 0) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    if (s_registration_sta_connected) {
#if CONFIG_ESPNOW_MESH_HIL_TEST_ENABLE
        s_hil_registration_discovered = true;
#endif
        ESP_LOGI(TAG, "holding registration AP association for %dms",
                 CONFIG_ESPNOW_MESH_REGISTRATION_HOLD_MS);
        vTaskDelay(pdMS_TO_TICKS(CONFIG_ESPNOW_MESH_REGISTRATION_HOLD_MS));
    } else {
        ESP_LOGW(TAG, "registration AP connect timed out");
    }

    (void)esp_wifi_disconnect();
    s_registration_sta_connected = false;
    restore_espnow_channel();
}

static void attempt_registration_if_due(void)
{
    if (s_have_controller) {
        return;
    }

    uint32_t current_ms = now_ms();
    if ((int32_t)(s_next_registration_ms - current_ms) > 0) {
        return;
    }
    s_next_registration_ms = current_ms + CONFIG_ESPNOW_MESH_REGISTRATION_RETRY_INTERVAL_MS;

    ESP_LOGI(TAG, "no ESP-NOW controller heard on channel %u; scanning for registration AP SSID=\"%s\"",
             s_mesh_channel, CONFIG_ESPNOW_MESH_REGISTRATION_SSID);

    wifi_ap_record_t ap = { 0 };
    if (scan_for_registration_ap(&ap)) {
        connect_to_registration_ap(&ap);
    }
}

static TickType_t satellite_wait_ticks(void)
{
    if (s_have_controller) {
        return time_sync_wait_ticks();
    }

    uint32_t current_ms = now_ms();
    int32_t wait_ms = (int32_t)(s_next_registration_ms - current_ms);
    if (wait_ms <= 0) {
        return 0;
    }
    if (wait_ms > 1000) {
        wait_ms = 1000;
    }
    return pdMS_TO_TICKS(wait_ms);
}
#else
static void attempt_registration_if_due(void)
{
}

static TickType_t satellite_wait_ticks(void)
{
    return time_sync_wait_ticks();
}
#endif

static void handle_time_response(const espnow_mesh_event_t *event)
{
    if (event->len < sizeof(espnow_mesh_time_resp_msg_t)) {
        return;
    }
    espnow_mesh_time_resp_msg_t resp = { 0 };
    memcpy(&resp, event->data, sizeof(resp));

    if (!s_have_controller || !mac_equal(event->mac, s_controller_mac) ||
        !mac_equal(resp.controller_mac, s_controller_mac)) {
        return;
    }

    if (!s_have_pending_time_sync || resp.sequence != s_pending_time_sync_sequence ||
        resp.satellite_tx_us != s_pending_time_sync_tx_us) {
        ESP_LOGD(TAG, "ignoring stale time response seq=%" PRIu32, resp.sequence);
        return;
    }

    s_have_pending_time_sync = false;

    int64_t satellite_tx_us = (int64_t)resp.satellite_tx_us;
    int64_t satellite_rx_us = (int64_t)event->rx_us;
    int64_t controller_rx_us = (int64_t)resp.controller_rx_us;
    int64_t controller_tx_us = (int64_t)resp.controller_tx_us;

    int64_t delay_us = (satellite_rx_us - satellite_tx_us) -
                       (controller_tx_us - controller_rx_us);
    if (delay_us < 0 || delay_us > CONFIG_ESPNOW_MESH_TIME_SYNC_MAX_DELAY_US) {
        ESP_LOGW(TAG, "reject time sync seq=%" PRIu32 " delay=%" PRId64 "us", resp.sequence,
                 delay_us);
        return;
    }

    int64_t sample_offset_us =
        ((controller_rx_us - satellite_tx_us) + (controller_tx_us - satellite_rx_us)) / 2;

#if CONFIG_ESPNOW_MESH_TIME_SYNC_PLL_ENABLE
    int64_t sample_local_us = satellite_tx_us + ((satellite_rx_us - satellite_tx_us) / 2);
    int64_t sample_mesh_us = controller_rx_us + ((controller_tx_us - controller_rx_us) / 2);
    pll_update_result_t pll = pll_update_from_sample(sample_local_us, sample_mesh_us,
                                                     sample_offset_us, delay_us);
    if (!pll.accepted) {
        ESP_LOGW(TAG,
                 "reject time sync pll seq=%" PRIu32
                 " reason=%s residual=%" PRId64 "us delay=%" PRId64
                 "us best_delay=%" PRId64 "us rejects=%" PRIu32,
                 resp.sequence, pll.reject_reason, pll.residual_us, delay_us,
                 pll.best_delay_us, pll.consecutive_rejects);
        return;
    }

    ESP_LOGI(TAG,
             "time sync pll seq=%" PRIu32
             " reacquire=%u offset_sample=%" PRId64 "us residual=%" PRId64
             "us phase_step=%" PRId64 "us offset=%" PRId64
             "us freq=%" PRId32 "ppb delay=%" PRId64
             "us best_delay=%" PRId64 "us samples=%" PRIu32
             " mesh_time=%" PRId64 "us",
             resp.sequence, pll.reacquired ? 1 : 0, sample_offset_us, pll.residual_us,
             pll.phase_step_us, pll.offset_us, pll.freq_ppb, delay_us, pll.best_delay_us,
             pll.accepted_samples, mesh_time_us());
#else
    int64_t filtered_offset_us = time_sync_apply_offset(sample_offset_us);

    ESP_LOGI(TAG,
             "time sync seq=%" PRIu32 " offset_sample=%" PRId64
             "us offset=%" PRId64 "us delay=%" PRId64 "us mesh_time=%" PRId64 "us",
             resp.sequence, sample_offset_us, filtered_offset_us, delay_us, mesh_time_us());
#endif
}

#if CONFIG_ESPNOW_MESH_HIL_TEST_ENABLE
static void handle_satellite_hil_command(const espnow_mesh_event_t *event)
{
    if (event->len < sizeof(espnow_mesh_hil_cmd_msg_t)) {
        return;
    }
    espnow_mesh_hil_cmd_msg_t cmd = { 0 };
    memcpy(&cmd, event->data, sizeof(cmd));
    if (!hil_softap_discovery_ready("HIL command")) {
        return;
    }

    if (!s_have_controller) {
        memcpy(s_controller_mac, event->mac, ESP_NOW_ETH_ALEN);
        s_have_controller = true;
        s_next_time_sync_ms = 0;
        ESP_LOGI(TAG, "locked to HIL controller " MACSTR, MAC2STR(s_controller_mac));
    } else if (!mac_equal(event->mac, s_controller_mac)) {
        ESP_LOGW(TAG, "ignoring HIL command from extra controller " MACSTR, MAC2STR(event->mac));
        return;
    }

    if (!s_have_controller_boot_id || cmd.controller_boot_id != s_controller_boot_id) {
        s_controller_boot_id = cmd.controller_boot_id;
        s_have_controller_boot_id = true;
        s_have_last_sequence = false;
        s_have_last_hil_sequence = false;
        ESP_LOGI(TAG, "HIL controller boot/session changed to 0x%08" PRIx32,
                 s_controller_boot_id);
    }

    esp_err_t err = add_peer_if_needed(event->mac);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "could not add HIL controller peer " MACSTR ": %s",
                 MAC2STR(event->mac), esp_err_to_name(err));
        return;
    }

    if (hil_fault_drop(cmd.rx_drop_pct, cmd.fault_seed, cmd.sequence, cmd.retry_index,
                       ESPNOW_MESH_HIL_FAULT_RX_CMD)) {
        ESP_LOGW(TAG, "HIL injected RX drop case=%" PRIu32 " seq=%" PRIu32
                      " attempt=%" PRIu32 " rx_drop=%u",
                 cmd.test_case, cmd.sequence, cmd.retry_index, cmd.rx_drop_pct);
        return;
    }

    bool duplicate =
        seq_is_duplicate(s_have_last_hil_sequence, cmd.sequence, s_last_hil_sequence);
    bool accepted = !duplicate;
    if (accepted) {
        s_have_last_hil_sequence = true;
        s_last_hil_sequence = cmd.sequence;
        if (cmd.kind == ESPNOW_MESH_HIL_CMD_PATCH_SELECT) {
            s_current_patch_id = cmd.patch_id;
        } else if (cmd.kind == ESPNOW_MESH_HIL_CMD_PARAM_SET) {
            s_current_patch_id = cmd.patch_id;
            s_current_param_id = cmd.param_id;
            s_current_param_value = cmd.param_value;
        }

        hil_schedule_apply_marker((int64_t)cmd.apply_at_mesh_us);
        ESP_LOGI(TAG,
                 "HIL scheduled case=%" PRIu32 " seq=%" PRIu32
                 " kind=%u apply_at=%" PRIu64 " patch=%" PRIu32
                 " param=%" PRIu32 " value=%" PRId32 " mesh_now=%" PRId64,
                 cmd.test_case, cmd.sequence, cmd.kind, cmd.apply_at_mesh_us, cmd.patch_id,
                 cmd.param_id, cmd.param_value, mesh_time_us());
    } else {
        ESP_LOGD(TAG, "HIL duplicate case=%" PRIu32 " seq=%" PRIu32
                      " attempt=%" PRIu32,
                 cmd.test_case, cmd.sequence, cmd.retry_index);
    }

    if (hil_fault_drop(cmd.ack_drop_pct, cmd.fault_seed, cmd.sequence, cmd.retry_index,
                       ESPNOW_MESH_HIL_FAULT_TX_ACK)) {
        ESP_LOGW(TAG, "HIL injected ACK drop case=%" PRIu32 " seq=%" PRIu32
                      " attempt=%" PRIu32 " ack_drop=%u",
                 cmd.test_case, cmd.sequence, cmd.retry_index, cmd.ack_drop_pct);
        return;
    }

    err = send_satellite_hil_ack(event->mac, &cmd, accepted, duplicate, event->rssi);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "HIL ACK case=%" PRIu32 " seq=%" PRIu32 " send failed: %s",
                 cmd.test_case, cmd.sequence, esp_err_to_name(err));
    }
}
#endif

static void handle_satellite_data(const espnow_mesh_event_t *event)
{
    if (event->len < offsetof(espnow_mesh_data_msg_t, payload)) {
        return;
    }
    espnow_mesh_data_msg_t msg = { 0 };
    memcpy(&msg, event->data, event->len < sizeof(msg) ? event->len : sizeof(msg));
    if (!data_msg_fields_valid(&msg, event->len)) {
        return;
    }
#if CONFIG_ESPNOW_MESH_HIL_TEST_ENABLE
    if (!hil_softap_discovery_ready("data")) {
        return;
    }
#endif

    if (!s_have_controller) {
        memcpy(s_controller_mac, event->mac, ESP_NOW_ETH_ALEN);
        s_have_controller = true;
        s_next_time_sync_ms = 0;
        ESP_LOGI(TAG, "locked to controller " MACSTR, MAC2STR(s_controller_mac));
    } else if (!mac_equal(event->mac, s_controller_mac)) {
        ESP_LOGW(TAG, "ignoring packet from extra controller " MACSTR, MAC2STR(event->mac));
        return;
    }

    if (!s_have_controller_boot_id || msg.controller_boot_id != s_controller_boot_id) {
        s_controller_boot_id = msg.controller_boot_id;
        s_have_controller_boot_id = true;
        s_have_last_sequence = false;
        ESP_LOGI(TAG, "controller boot/session changed to 0x%08" PRIx32, s_controller_boot_id);
    }

    esp_err_t err = add_peer_if_needed(event->mac);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "could not add controller peer " MACSTR ": %s", MAC2STR(event->mac),
                 esp_err_to_name(err));
        return;
    }

    bool duplicate = seq_is_duplicate(s_have_last_sequence, msg.sequence, s_last_sequence);
    bool accepted = !duplicate;
    if (accepted) {
        s_have_last_sequence = true;
        s_last_sequence = msg.sequence;

        if (mesh_have_time()) {
            ESP_LOGI(TAG,
                     "rx seq=%" PRIu32 " attempt=%" PRIu32
                     " rssi=%d mesh_time=%" PRId64 "us bytes=%u",
                     msg.sequence, msg.retry_index, event->rssi, mesh_time_us(),
                     msg.payload_len);
        } else {
            ESP_LOGI(TAG, "rx seq=%" PRIu32 " attempt=%" PRIu32 " rssi=%d bytes=%u",
                     msg.sequence, msg.retry_index, event->rssi, msg.payload_len);
        }

        /* payload_len is bounded by data_msg_fields_valid(). */
        if (s_rx_cb != NULL) {
            s_rx_cb(msg.payload, msg.payload_len, s_rx_cb_ctx);
        }
    } else {
        ESP_LOGD(TAG, "duplicate seq=%" PRIu32 " attempt=%" PRIu32, msg.sequence,
                 msg.retry_index);
    }

    err = send_satellite_ack(event->mac, msg.sequence, msg.controller_boot_id, accepted,
                             duplicate, event->rssi);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "ACK seq=%" PRIu32 " send failed: %s", msg.sequence,
                 esp_err_to_name(err));
    }
}

#if CONFIG_ESPNOW_MESH_CONTROLLER_TIMEOUT_MS > 0
/* Record that a valid frame was received from the locked controller. */
static void note_controller_activity(void)
{
    s_last_controller_rx_ms = now_ms();
}

/*
 * Drop the controller lock if nothing has been heard from it for the configured
 * window. The boot-id and sequence dedup state are kept so a controller that is
 * still alive on a new channel is re-adopted cleanly, while registration
 * scanning resumes immediately to rediscover a controller that moved channels.
 */
static void check_controller_timeout(void)
{
    if (!s_have_controller) {
        return;
    }
    if ((int32_t)(now_ms() - s_last_controller_rx_ms) <
        CONFIG_ESPNOW_MESH_CONTROLLER_TIMEOUT_MS) {
        return;
    }

    ESP_LOGW(TAG, "no controller traffic for %dms; dropping lock on " MACSTR
                  " and resuming discovery",
             CONFIG_ESPNOW_MESH_CONTROLLER_TIMEOUT_MS, MAC2STR(s_controller_mac));
    s_have_controller = false;
    s_have_pending_time_sync = false;
#if CONFIG_ESPNOW_MESH_REGISTRATION_AP_ENABLE
    s_next_registration_ms = now_ms();
#endif
}
#else
static void note_controller_activity(void)
{
}

static void check_controller_timeout(void)
{
}
#endif

static void satellite_run(void)
{
    ESP_LOGI(TAG, "running as satellite");
#if CONFIG_ESPNOW_MESH_REGISTRATION_AP_ENABLE
    s_next_registration_ms = now_ms() + CONFIG_ESPNOW_MESH_REGISTRATION_LISTEN_MS;
    ESP_LOGI(TAG, "listening %dms for ESP-NOW before registration AP fallback",
             CONFIG_ESPNOW_MESH_REGISTRATION_LISTEN_MS);
#endif

    while (true) {
        log_event_queue_drops_if_any();
        check_controller_timeout();
        request_time_sync_if_due();
        attempt_registration_if_due();

        espnow_mesh_event_t event = { 0 };
        if (xQueueReceive(s_event_queue, &event, satellite_wait_ticks()) != pdTRUE) {
            continue;
        }

        if (event.type == ESPNOW_MESH_EVENT_SEND) {
            if (event.send_status != ESP_NOW_SEND_SUCCESS) {
                ESP_LOGW(TAG, "ESP-NOW send callback failed for " MACSTR, MAC2STR(event.mac));
            }
            continue;
        }

        uint8_t type = 0;
        if (!mesh_rx_packet_type(&event, &type)) {
            continue;
        }

        switch (type) {
        case ESPNOW_MESH_MSG_TIME_RESP:
            handle_time_response(&event);
            break;
#if CONFIG_ESPNOW_MESH_HIL_TEST_ENABLE
        case ESPNOW_MESH_MSG_PATCH_OFFER:
            handle_satellite_patch_offer(&event);
            break;
        case ESPNOW_MESH_MSG_PATCH_BLOCK:
            handle_satellite_patch_block(&event);
            break;
        case ESPNOW_MESH_MSG_PATCH_APPLY:
            handle_satellite_patch_apply(&event);
            break;
        case ESPNOW_MESH_MSG_HIL_CMD:
            handle_satellite_hil_command(&event);
            break;
#endif
        case ESPNOW_MESH_MSG_DATA:
            handle_satellite_data(&event);
            break;
        default:
            break;
        }

        if (s_have_controller && mac_equal(event.mac, s_controller_mac)) {
            note_controller_activity();
        }
    }
}

#endif

espnow_mesh_role_runtime_t espnow_mesh_role(void)
{
#if CONFIG_ESPNOW_MESH_ROLE_CONTROLLER
    return ESPNOW_MESH_ROLE_CONTROLLER_RUNTIME;
#else
    return ESPNOW_MESH_ROLE_SATELLITE_RUNTIME;
#endif
}

esp_err_t espnow_mesh_send(const void *payload, size_t len)
{
#if CONFIG_ESPNOW_MESH_ROLE_CONTROLLER && !CONFIG_ESPNOW_MESH_HIL_TEST_ENABLE
    if (len > sizeof(s_pending_payload)) {
        return ESP_ERR_INVALID_SIZE;
    }
    if (len > 0 && payload == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err;
    portENTER_CRITICAL(&s_pending_payload_lock);
    if (s_have_pending_payload) {
        err = ESP_ERR_NO_MEM;
    } else {
        if (len > 0) {
            memcpy(s_pending_payload, payload, len);
        }
        s_pending_payload_len = (uint16_t)len;
        s_have_pending_payload = true;
        err = ESP_OK;
    }
    portEXIT_CRITICAL(&s_pending_payload_lock);
    return err;
#else
    (void)payload;
    (void)len;
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

esp_err_t espnow_mesh_set_rx_callback(espnow_mesh_rx_cb_t cb, void *user_ctx)
{
#if CONFIG_ESPNOW_MESH_ROLE_CONTROLLER
    (void)cb;
    (void)user_ctx;
    return ESP_ERR_NOT_SUPPORTED;
#else
    s_rx_cb_ctx = user_ctx;
    s_rx_cb = cb;
    return ESP_OK;
#endif
}

bool espnow_mesh_get_time_us(int64_t *mesh_time_out_us)
{
    if (mesh_time_out_us == NULL) {
        return false;
    }
    return shared_clock_get_time_us(mesh_time_out_us);
}

bool espnow_mesh_is_time_synced(void)
{
    int64_t mesh_time_us = 0;
    return espnow_mesh_get_time_us(&mesh_time_us);
}

bool espnow_mesh_get_status(espnow_mesh_status_t *status)
{
    if (status == NULL) {
        return false;
    }

    memset(status, 0, sizeof(*status));
    status->role = espnow_mesh_role();
    status->initialized = s_mesh_initialized;
    status->espnow_ready = s_espnow_ready;
    status->time_synced = espnow_mesh_is_time_synced();
    memcpy(status->self_mac, s_self_mac, sizeof(status->self_mac));
    status->channel = s_mesh_channel;
    status->boot_id = s_boot_id;
    status->uptime_ms = now_ms();
    status->event_drops = s_event_queue_drops;

#if CONFIG_ESPNOW_MESH_ROLE_CONTROLLER
    status->active_sequence = s_controller_active_sequence;
    status->last_sequence = s_controller_last_sequence;
    status->last_expected = s_controller_last_expected;
    status->last_acked = s_controller_last_acked;
    status->last_sequence_complete = s_controller_last_sequence_complete;
    status->expected_current = (uint32_t)current_expected_count();
    status->acked_current = (uint32_t)current_acked_count();

    uint32_t current_ms = now_ms();
    size_t out_index = 0;
    portENTER_CRITICAL(&s_satellite_table_lock);
    for (int i = 0; i < CONFIG_ESPNOW_MESH_MAX_SATELLITES; ++i) {
        const satellite_node_t *node = &s_satellites[i];
        if (!node->used) {
            continue;
        }

        status->known_satellites++;
        if (out_index >= ESPNOW_MESH_STATUS_MAX_SATELLITES) {
            continue;
        }

        espnow_mesh_satellite_status_t *sat = &status->satellites[out_index++];
        sat->used = true;
        sat->expected_current = node->expected_current;
        sat->acked_current = node->acked_current;
        memcpy(sat->mac, node->mac, sizeof(sat->mac));
        sat->last_seen_ms_ago =
            node->last_seen_ms == 0 ? UINT32_MAX : current_ms - node->last_seen_ms;
        sat->last_sequence = node->last_sequence;
        sat->ack_count = node->ack_count;
        sat->sends_current = node->sends_current;
        sat->last_rssi = node->last_rssi;
    }
    portEXIT_CRITICAL(&s_satellite_table_lock);
    status->satellite_count = out_index;
#else
    status->known_satellites = s_have_controller ? 1 : 0;
    status->active_sequence = s_have_last_sequence ? s_last_sequence : 0;
    status->last_sequence = s_have_last_sequence ? s_last_sequence : 0;
#endif

    return true;
}

static esp_err_t espnow_mesh_init_once(void)
{
    if (s_mesh_initialized) {
        return ESP_OK;
    }

    init_nvs();
    init_boot_id();
    init_mesh_channel();
    init_mesh_security();
    init_wifi();
    init_espnow();
    init_sync_output();
    init_hil_test();

    s_mesh_initialized = true;
    return ESP_OK;
}

esp_err_t espnow_mesh_run(void)
{
    esp_err_t err = espnow_mesh_init_once();
    if (err != ESP_OK) {
        return err;
    }

#if CONFIG_ESPNOW_MESH_ROLE_CONTROLLER
    controller_run();
#else
    satellite_run();
#endif
    return ESP_OK;
}

static void espnow_mesh_task(void *arg)
{
    (void)arg;
    esp_err_t err = espnow_mesh_run();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "mesh task stopped: %s", esp_err_to_name(err));
    }
    s_mesh_task_handle = NULL;
    vTaskDelete(NULL);
}

esp_err_t espnow_mesh_start(const espnow_mesh_config_t *config)
{
    if (s_mesh_task_handle != NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    espnow_mesh_config_t cfg = ESPNOW_MESH_DEFAULT_CONFIG();
    if (config != NULL) {
        cfg = *config;
    }
    if (cfg.task_name == NULL || cfg.task_stack_bytes == 0 || cfg.task_priority == 0) {
        return ESP_ERR_INVALID_ARG;
    }
#if CONFIG_ESPNOW_MESH_REGISTRATION_AP_ENABLE
    if (cfg.use_existing_wifi) {
        return ESP_ERR_INVALID_ARG;
    }
#endif
    if (cfg.adopt_current_wifi_channel && !cfg.use_existing_wifi) {
        return ESP_ERR_INVALID_ARG;
    }

    s_start_config = cfg;

    BaseType_t ok = pdFALSE;
    if (cfg.task_core_id == ESPNOW_MESH_TASK_NO_AFFINITY) {
        ok = xTaskCreate(espnow_mesh_task, cfg.task_name, cfg.task_stack_bytes, NULL,
                         (UBaseType_t)cfg.task_priority, &s_mesh_task_handle);
    } else {
        ok = xTaskCreatePinnedToCore(espnow_mesh_task, cfg.task_name, cfg.task_stack_bytes,
                                     NULL, (UBaseType_t)cfg.task_priority,
                                     &s_mesh_task_handle, (BaseType_t)cfg.task_core_id);
    }
    if (ok != pdPASS) {
        s_mesh_task_handle = NULL;
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}
