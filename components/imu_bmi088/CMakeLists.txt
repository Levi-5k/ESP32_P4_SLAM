idf_component_register(
    SRCS "imu_bmi088.c"
    INCLUDE_DIRS "include"
    REQUIRES "esp_timer" "freertos" "driver" "esp_common" "common"
)
