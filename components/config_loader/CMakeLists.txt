idf_component_register(
    SRCS "config_loader.c"
    INCLUDE_DIRS "include"
    REQUIRES
        json
        spiffs
        esp_timer
        common
        sd_storage
)
