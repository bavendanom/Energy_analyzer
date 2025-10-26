idf_component_register(
    SRCS "src/ntp_sync.c"
    INCLUDE_DIRS "include"
    REQUIRES lwip esp_netif esp_wifi
)
