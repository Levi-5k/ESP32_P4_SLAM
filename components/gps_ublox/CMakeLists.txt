idf_component_register(
    SRCS "gps_ublox.c"
    INCLUDE_DIRS "include"
    REQUIRES "esp_timer" "freertos" "driver" "esp_common" "common"
)
