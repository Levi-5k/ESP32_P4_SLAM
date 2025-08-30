idf_component_register(SRCS "visual_slam_main.c"
                       INCLUDE_DIRS "."
                       REQUIRES common slam_core orb_features sensor_fusion web_server esp_wifi nvs_flash esp_http_server esp_mm esp_driver_cam esp_driver_i2c driver sensor_init esp_netif esp_driver_jpeg)
