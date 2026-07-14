#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "esp_event.h"

extern esp_event_base_t WIFI_EVENT;

typedef enum {
    WIFI_EVENT_STA_CONNECTED = 1,
    WIFI_EVENT_STA_DISCONNECTED,
    WIFI_EVENT_AP_STACONNECTED,
    WIFI_EVENT_AP_STADISCONNECTED,
} wifi_event_t;

typedef enum { WIFI_MODE_NULL = 0, WIFI_MODE_STA, WIFI_MODE_AP, WIFI_MODE_APSTA } wifi_mode_t;
typedef enum { WIFI_IF_STA = 0, WIFI_IF_AP } wifi_interface_t;
typedef enum { WIFI_STORAGE_FLASH = 0, WIFI_STORAGE_RAM } wifi_storage_t;
typedef enum { WIFI_PS_NONE = 0, WIFI_PS_MIN_MODEM, WIFI_PS_MAX_MODEM } wifi_ps_type_t;
typedef enum { WIFI_SECOND_CHAN_NONE = 0, WIFI_SECOND_CHAN_ABOVE, WIFI_SECOND_CHAN_BELOW } wifi_second_chan_t;
typedef enum { WIFI_AUTH_OPEN = 0, WIFI_AUTH_WEP, WIFI_AUTH_WPA_PSK, WIFI_AUTH_WPA2_PSK } wifi_auth_mode_t;
typedef enum { WIFI_FAST_SCAN = 0, WIFI_ALL_CHANNEL_SCAN } wifi_scan_method_t;
typedef enum { WIFI_CONNECT_AP_BY_SIGNAL = 0, WIFI_CONNECT_AP_BY_SECURITY } wifi_sort_method_t;
typedef enum { WIFI_SCAN_TYPE_ACTIVE = 0, WIFI_SCAN_TYPE_PASSIVE } wifi_scan_type_t;

typedef struct {
    wifi_auth_mode_t authmode;
    int8_t rssi;
} wifi_scan_threshold_t;

typedef struct {
    uint8_t ssid[32];
    uint8_t password[64];
    wifi_scan_method_t scan_method;
    bool bssid_set;
    uint8_t bssid[6];
    uint8_t channel;
    wifi_sort_method_t sort_method;
    wifi_scan_threshold_t threshold;
    uint8_t failure_retry_cnt;
} wifi_sta_config_t;

typedef struct {
    uint8_t ssid[32];
    uint8_t password[64];
    uint8_t ssid_len;
    uint8_t channel;
    wifi_auth_mode_t authmode;
    uint8_t ssid_hidden;
    uint8_t max_connection;
    uint16_t beacon_interval;
} wifi_ap_config_t;

typedef union {
    wifi_ap_config_t ap;
    wifi_sta_config_t sta;
} wifi_config_t;

typedef struct {
    uint8_t bssid[6];
    uint8_t ssid[33];
    uint8_t primary;
    int8_t rssi;
    wifi_auth_mode_t authmode;
} wifi_ap_record_t;

typedef struct {
    uint32_t min;
    uint32_t max;
} wifi_active_scan_time_t;

typedef struct {
    wifi_active_scan_time_t active;
    uint32_t passive;
} wifi_scan_time_t;

typedef struct {
    uint8_t *ssid;
    uint8_t *bssid;
    uint8_t channel;
    bool show_hidden;
    wifi_scan_type_t scan_type;
    wifi_scan_time_t scan_time;
} wifi_scan_config_t;

typedef struct {
    uint8_t mac[6];
    uint8_t aid;
} wifi_event_ap_staconnected_t;

typedef struct {
    uint8_t ssid[32];
    uint8_t ssid_len;
    uint8_t bssid[6];
    uint8_t channel;
    wifi_auth_mode_t authmode;
} wifi_event_sta_connected_t;

typedef struct {
    int dummy;
} wifi_init_config_t;
#define WIFI_INIT_CONFIG_DEFAULT() { .dummy = 0 }

esp_err_t esp_netif_init(void);
void *esp_netif_create_default_wifi_sta(void);
void *esp_netif_create_default_wifi_ap(void);

esp_err_t esp_wifi_init(const wifi_init_config_t *config);
esp_err_t esp_wifi_set_storage(wifi_storage_t storage);
esp_err_t esp_wifi_set_mode(wifi_mode_t mode);
esp_err_t esp_wifi_start(void);
esp_err_t esp_wifi_set_ps(wifi_ps_type_t type);
esp_err_t esp_wifi_set_channel(uint8_t primary, wifi_second_chan_t second);
esp_err_t esp_wifi_get_mac(wifi_interface_t ifx, uint8_t mac[6]);
esp_err_t esp_wifi_set_config(wifi_interface_t interface, wifi_config_t *conf);
esp_err_t esp_wifi_scan_start(const wifi_scan_config_t *config, bool block);
esp_err_t esp_wifi_scan_get_ap_records(uint16_t *number, wifi_ap_record_t *ap_records);
esp_err_t esp_wifi_clear_ap_list(void);
esp_err_t esp_wifi_connect(void);
esp_err_t esp_wifi_disconnect(void);
