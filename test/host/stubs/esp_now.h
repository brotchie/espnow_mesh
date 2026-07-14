#pragma once
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#define ESP_NOW_ETH_ALEN 6
#define ESP_NOW_MAX_DATA_LEN 250
typedef enum { ESP_NOW_SEND_SUCCESS = 0, ESP_NOW_SEND_FAIL } esp_now_send_status_t;
esp_err_t esp_now_send(const uint8_t *peer_addr, const uint8_t *data, size_t len);
