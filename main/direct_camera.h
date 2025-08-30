/*
 * Direct OV5647 MIPI-CSI Camera Driver Header
 * Bypasses ESP-IDF detection system for hardwired camera
 */

#pragma once

#include "esp_err.h"
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the OV5647 camera directly (bypass detection)
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t direct_camera_init(void);

/**
 * @brief Capture a frame from the camera
 * 
 * @param buffer Pointer to store the image buffer
 * @param len Pointer to store the buffer length
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t direct_camera_capture(uint8_t **buffer, size_t *len);

/**
 * @brief Deinitialize the camera
 */
void direct_camera_deinit(void);

#ifdef __cplusplus
}
#endif
