idf_component_register(
    SRCS "src/manejo_wifi.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_netif esp_wifi esp_event nvs_flash
)
