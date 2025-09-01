/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// Camera sensor configuration
#define EXAMPLE_CAM_SENSOR_I2C_NUM      I2C_NUM_0
#define EXAMPLE_CAM_SENSOR_I2C_SDA_PIN  GPIO_NUM_8
#define EXAMPLE_CAM_SENSOR_I2C_SCL_PIN  GPIO_NUM_9
#define EXAMPLE_CAM_SCCB_FREQ           100000  // 100kHz I2C frequency

// OV5647 Camera configuration
#define EXAMPLE_OV5647_CAM_XCLK_PIN     GPIO_NUM_45
#define EXAMPLE_OV5647_CAM_RST_PIN      GPIO_NUM_46
#define EXAMPLE_OV5647_CAM_PWDN_PIN     GPIO_NUM_NC

// MIPI CSI configuration
#define EXAMPLE_MIPI_CSI_LANE_NUM       2
#define EXAMPLE_MIPI_CSI_DATA_LANES     {GPIO_NUM_0, GPIO_NUM_1}
#define EXAMPLE_MIPI_CSI_CLK_LANES      {GPIO_NUM_2, GPIO_NUM_3}

#ifdef __cplusplus
}
#endif
