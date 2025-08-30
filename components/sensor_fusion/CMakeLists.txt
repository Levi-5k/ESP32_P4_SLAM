idf_component_register(
    SRCS "sensor_fusion.c"
    INCLUDE_DIRS "include"
    REQUIRES "esp_timer" "freertos" "heap" "esp_common" "lwip" "common"
)
