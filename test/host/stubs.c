/* Host-side definitions for the ESP-IDF symbols the tested modules reference. */
#include <stddef.h>
#include <stdint.h>
#include "esp_now.h"

int64_t esp_timer_get_time(void)
{
    static int64_t t;
    return t += 1000; /* monotonically advances; tests that care pass explicit times */
}

esp_err_t esp_now_send(const uint8_t *peer_addr, const uint8_t *data, size_t len)
{
    (void)peer_addr;
    (void)data;
    (void)len;
    return ESP_OK;
}
