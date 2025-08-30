#pragma once

#include <esp_err.h>
#include <driver/uart.h>
#include <driver/gpio.h>
#include "visual_slam_common_types.h"

#ifdef __cplusplus
extern "C" {
#endif

// GPS initialization function
esp_err_t sensor_fusion_init_gps(const gps_config_t *config);

// Get current GPS data
esp_err_t gps_get_data(gps_data_t *data);

#ifdef __cplusplus
}
#endif
