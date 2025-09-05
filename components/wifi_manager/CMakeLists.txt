idf_component_register(
    SRCS "wifi_manager.c"
    INCLUDE_DIRS "include"
    REQUIRES
        esp_wifi
        esp_wifi_remote
        esp_event
        esp_netif
        freertos
        esp_timer
)

# Add compile definitions for ESP-Hosted WiFi
target_compile_definitions(${COMPONENT_LIB} PRIVATE
    CONFIG_ESP_WIFI_ENABLED=1
)
