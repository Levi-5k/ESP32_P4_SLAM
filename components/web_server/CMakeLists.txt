idf_component_register(SRCS "web_server.c"
                       INCLUDE_DIRS "include"
                       PRIV_REQUIRES esp_http_server json
                       REQUIRES common slam_core sensor_fusion config_loader sd_storage wifi_positioning)
