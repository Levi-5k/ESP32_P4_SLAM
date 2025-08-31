idf_component_register(
    SRCS "slam_core.c"
    INCLUDE_DIRS "include"
    REQUIRES "esp_timer" "freertos" "heap" "esp_common" "common" "orb_features" "camera"
)
