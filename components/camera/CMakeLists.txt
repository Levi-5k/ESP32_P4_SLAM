idf_component_register(
    SRCS "camera_init.c"
    INCLUDE_DIRS "."
    REQUIRES "esp_driver_cam" "esp_driver_i2c" "driver" "esp_common" "sensor_init" "common" "esp_hw_support"
)
