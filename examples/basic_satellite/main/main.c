#include "esp_err.h"
#include "espnow_mesh.h"

void app_main(void)
{
    ESP_ERROR_CHECK(espnow_mesh_start(NULL));
}
