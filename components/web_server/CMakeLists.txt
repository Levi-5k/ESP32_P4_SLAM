idf_component_register(
    SRCS "web_server.c"
    INCLUDE_DIRS "include"
    REQUIRES "common" "slam_core" "esp_wifi" "esp_netif" "esp_event" "esp_http_server" "esp_timer" "nvs_flash" "json" "freertos" "heap" "vfs"
)
