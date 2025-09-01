idf_component_register(SRCS "sd_storage_sdmmc.c" "map_manager.c"
                    INCLUDE_DIRS "include"
                    REQUIRES fatfs sdmmc json slam_core esp_driver_gpio main)
# Following ESP-IDF SDMMC example pattern with power control support

# idf_component_register(SRCS "sd_storage_spi.c" "map_manager.c"
#                     INCLUDE_DIRS "include"
#                     REQUIRES fatfs esp_driver_gpio driver spi_flash json slam_core)
