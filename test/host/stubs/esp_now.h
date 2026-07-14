#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#include "esp_wifi.h"

#define ESP_NOW_ETH_ALEN 6
#define ESP_NOW_MAX_DATA_LEN 250

typedef enum { ESP_NOW_SEND_SUCCESS = 0, ESP_NOW_SEND_FAIL } esp_now_send_status_t;

typedef struct {
    uint8_t peer_addr[ESP_NOW_ETH_ALEN];
    uint8_t lmk[16];
    uint8_t channel;
    wifi_interface_t ifidx;
    bool encrypt;
    void *priv;
} esp_now_peer_info_t;

typedef struct {
    int8_t rssi;
    unsigned channel;
} wifi_pkt_rx_ctrl_t;

typedef struct {
    uint8_t *src_addr;
    uint8_t *des_addr;
    wifi_pkt_rx_ctrl_t *rx_ctrl;
} esp_now_recv_info_t;

typedef struct {
    uint8_t *src_addr;
    uint8_t *des_addr;
} esp_now_send_info_t;

typedef void (*esp_now_recv_cb_t)(const esp_now_recv_info_t *esp_now_info, const uint8_t *data, int data_len);
typedef void (*esp_now_send_cb_t)(const esp_now_send_info_t *tx_info, esp_now_send_status_t status);

esp_err_t esp_now_init(void);
esp_err_t esp_now_register_recv_cb(esp_now_recv_cb_t cb);
esp_err_t esp_now_register_send_cb(esp_now_send_cb_t cb);
bool esp_now_is_peer_exist(const uint8_t *peer_addr);
esp_err_t esp_now_add_peer(const esp_now_peer_info_t *peer);
esp_err_t esp_now_del_peer(const uint8_t *peer_addr);
esp_err_t esp_now_send(const uint8_t *peer_addr, const uint8_t *data, size_t len);
