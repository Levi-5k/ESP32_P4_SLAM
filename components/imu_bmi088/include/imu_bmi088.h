#pragma once

#include <esp_err.h>
#include <driver/spi_master.h>
#include <driver/gpio.h>
#include "visual_slam_common_types.h"

#ifdef __cplusplus
extern "C" {
#endif

// IMU initialization function  
esp_err_t sensor_fusion_init_imu(const imu_config_t *config);

// Get current IMU data
esp_err_t imu_get_data(imu_data_t *data);

#ifdef __cplusplus
}
#endif
