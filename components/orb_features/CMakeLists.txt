idf_component_register(
    SRCS "orb_features.c"
    INCLUDE_DIRS "include"
    REQUIRES "esp_timer" "freertos" "heap" "esp_common" "common"
)
