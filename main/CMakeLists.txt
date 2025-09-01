idf_component_register(SRCS "visual_slam_main.c"
                       INCLUDE_DIRS "."
                       REQUIRES common slam_core orb_features sensor_fusion sd_storage gps_ublox msp_protocol imu_bmi088 config_loader nvs_flash esp_mm esp_driver_cam esp_driver_i2c driver sensor_init esp_driver_jpeg esp_event camera)
