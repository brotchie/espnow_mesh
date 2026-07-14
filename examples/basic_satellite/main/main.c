#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_log.h"

#include "espnow_mesh.h"

static const char *TAG = "satellite_example";

/* Runs on the mesh task once per accepted controller sequence. Keep it short and
 * do not block; the payload is only valid for the duration of the call. */
static void on_data(const uint8_t *payload, size_t len, void *user_ctx)
{
    (void)user_ctx;
    ESP_LOGI(TAG, "received %u bytes: \"%.*s\"", (unsigned)len, (int)len,
             (const char *)payload);
}

void app_main(void)
{
    ESP_ERROR_CHECK(espnow_mesh_set_rx_callback(on_data, NULL));
    ESP_ERROR_CHECK(espnow_mesh_start(NULL));
}
