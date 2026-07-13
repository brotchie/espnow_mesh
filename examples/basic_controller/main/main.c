#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "espnow_mesh.h"

static const char *TAG = "controller_example";

void app_main(void)
{
    ESP_ERROR_CHECK(espnow_mesh_start(NULL));

    /* Periodically fan a message out to all satellites over the reliable
     * channel. espnow_mesh_send() is fire-and-forget; ESP_ERR_NO_MEM means the
     * previous payload is still being delivered, so back off and retry. */
    uint32_t counter = 0;
    while (true) {
        char msg[48];
        (void)snprintf(msg, sizeof(msg), "hello #%" PRIu32, counter);

        esp_err_t err = espnow_mesh_send(msg, strlen(msg));
        if (err == ESP_ERR_NO_MEM) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "send failed: %s", esp_err_to_name(err));
        } else {
            ESP_LOGI(TAG, "queued \"%s\"", msg);
        }

        counter++;
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
