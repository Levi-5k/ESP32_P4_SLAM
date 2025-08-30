#pragma once

#include "esp_err.h"
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the simple camera system
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t simple_camera_init(void);

/**
 * @brief Capture a frame from the camera
 * 
 * @param frame_buffer Pointer to receive frame buffer
 * @param frame_len Pointer to receive frame length
 * @return esp_err_t ESP_OK on success
 */
esp_err_t simple_camera_capture(uint8_t **frame_buffer, size_t *frame_len);

/**
 * @brief Deinitialize the camera system
 */
void simple_camera_deinit(void);

#ifdef __cplusplus
}
#endif
