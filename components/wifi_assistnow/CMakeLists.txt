idf_component_register(
    SRCS "wifi_assistnow.c"
    INCLUDE_DIRS "."
    REQUIRES esp_wifi esp_netif esp_event
)
