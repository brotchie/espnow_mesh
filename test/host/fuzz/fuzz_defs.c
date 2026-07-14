/* Benign host definitions for the ESP-IDF symbols the component references, so
 * a fuzz executable links. The fuzz harness calls the target handler directly
 * and never runs init, so none of these run on the hot path. */
#include <stddef.h>
#include <stdint.h>
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

int64_t esp_timer_get_time(void) { static int64_t t; return t += 1000; }
uint32_t esp_random(void) { return 0x9e3779b9u; }

esp_err_t esp_now_send(const uint8_t *m, const uint8_t *d, size_t n) { (void)m;(void)d;(void)n; return ESP_OK; }
esp_err_t esp_now_init(void) { return ESP_OK; }
esp_err_t esp_now_register_recv_cb(esp_now_recv_cb_t cb) { (void)cb; return ESP_OK; }
esp_err_t esp_now_register_send_cb(esp_now_send_cb_t cb) { (void)cb; return ESP_OK; }
bool esp_now_is_peer_exist(const uint8_t *m) { (void)m; return true; }
esp_err_t esp_now_add_peer(const esp_now_peer_info_t *p) { (void)p; return ESP_OK; }
esp_err_t esp_now_del_peer(const uint8_t *m) { (void)m; return ESP_OK; }

esp_err_t esp_netif_init(void) { return ESP_OK; }
void *esp_netif_create_default_wifi_sta(void) { return (void *)0; }
void *esp_netif_create_default_wifi_ap(void) { return (void *)0; }
esp_err_t esp_event_loop_create_default(void) { return ESP_OK; }
esp_err_t esp_event_handler_instance_register(esp_event_base_t b, int32_t id, esp_event_handler_t h, void *a, esp_event_handler_instance_t *i) { (void)b;(void)id;(void)h;(void)a;(void)i; return ESP_OK; }
esp_event_base_t WIFI_EVENT = "WIFI_EVENT";

esp_err_t esp_wifi_init(const wifi_init_config_t *c) { (void)c; return ESP_OK; }
esp_err_t esp_wifi_set_storage(wifi_storage_t s) { (void)s; return ESP_OK; }
esp_err_t esp_wifi_set_mode(wifi_mode_t m) { (void)m; return ESP_OK; }
esp_err_t esp_wifi_start(void) { return ESP_OK; }
esp_err_t esp_wifi_set_ps(wifi_ps_type_t t) { (void)t; return ESP_OK; }
esp_err_t esp_wifi_set_channel(uint8_t p, wifi_second_chan_t s) { (void)p;(void)s; return ESP_OK; }
esp_err_t esp_wifi_get_mac(wifi_interface_t i, uint8_t m[6]) { (void)i; for (int k=0;k<6;k++) m[k]=(uint8_t)k; return ESP_OK; }
esp_err_t esp_wifi_set_config(wifi_interface_t i, wifi_config_t *c) { (void)i;(void)c; return ESP_OK; }
esp_err_t esp_wifi_scan_start(const wifi_scan_config_t *c, bool b) { (void)c;(void)b; return ESP_OK; }
esp_err_t esp_wifi_scan_get_ap_records(uint16_t *n, wifi_ap_record_t *r) { (void)r; if (n) *n = 0; return ESP_OK; }
esp_err_t esp_wifi_clear_ap_list(void) { return ESP_OK; }
esp_err_t esp_wifi_connect(void) { return ESP_OK; }
esp_err_t esp_wifi_disconnect(void) { return ESP_OK; }

esp_err_t nvs_flash_init(void) { return ESP_OK; }
esp_err_t nvs_flash_erase(void) { return ESP_OK; }

esp_err_t esp_timer_create(const esp_timer_create_args_t *a, esp_timer_handle_t *h) { (void)a; if (h) *h = (esp_timer_handle_t)0; return ESP_OK; }
esp_err_t esp_timer_start_once(esp_timer_handle_t h, uint64_t t) { (void)h;(void)t; return ESP_OK; }
esp_err_t esp_timer_stop(esp_timer_handle_t h) { (void)h; return ESP_OK; }

esp_err_t gpio_config(const gpio_config_t *c) { (void)c; return ESP_OK; }
esp_err_t gpio_set_level(gpio_num_t g, uint32_t l) { (void)g;(void)l; return ESP_OK; }

const char *esp_err_to_name(esp_err_t e) { (void)e; return "ESP_OK"; }

QueueHandle_t xQueueCreate(UBaseType_t n, UBaseType_t s) { (void)n;(void)s; return (QueueHandle_t)1; }
BaseType_t xQueueSend(QueueHandle_t q, const void *i, TickType_t w) { (void)q;(void)i;(void)w; return pdTRUE; }
BaseType_t xQueueReceive(QueueHandle_t q, void *b, TickType_t w) { (void)q;(void)b;(void)w; return pdFALSE; }
BaseType_t xTaskCreate(TaskFunction_t f, const char *n, uint32_t s, void *p, UBaseType_t pr, TaskHandle_t *h) { (void)f;(void)n;(void)s;(void)p;(void)pr; if (h) *h=(TaskHandle_t)1; return pdPASS; }
BaseType_t xTaskCreatePinnedToCore(TaskFunction_t f, const char *n, uint32_t s, void *p, UBaseType_t pr, TaskHandle_t *h, BaseType_t c) { (void)f;(void)n;(void)s;(void)p;(void)pr;(void)c; if (h) *h=(TaskHandle_t)1; return pdPASS; }
void vTaskDelay(TickType_t t) { (void)t; }
void vTaskDelete(TaskHandle_t h) { (void)h; }
