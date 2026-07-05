#include "esp_err.h"
#include "espnow_mesh.h"
#include "mesh_visualizer.h"

void app_main(void)
{
    ESP_ERROR_CHECK(mesh_visualizer_start());
    ESP_ERROR_CHECK(espnow_mesh_start(NULL));
}
