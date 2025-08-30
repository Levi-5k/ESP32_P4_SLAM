idf_component_register(
    SRCS "msp_protocol.c"
    INCLUDE_DIRS "include"
    REQUIRES "esp_timer" "freertos" "driver" "esp_common"
)
