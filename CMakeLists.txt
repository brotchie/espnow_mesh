idf_component_register(
    SRCS "src/espnow_mesh.c" "src/espnow_mesh_packet.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_common
    PRIV_REQUIRES esp_driver_gpio esp_event esp_hw_support esp_netif esp_timer esp_wifi mbedtls nvs_flash
)
