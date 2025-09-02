idf_component_register(SRCS "assistnow.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_http_client json nvs_flash esp_timer sd_storage wifi_assistnow)
