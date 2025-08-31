/*
 * Camera initialization header for ESP32-P4-Pico with OV5647 MIPI-CSI
 */

#pragma once

#include "esp_err.h"
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the OV5647 MIPI-CSI camera
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t camera_init(void);

/**
 * @brief Start the camera capture
 * 
 * @return ESP_OK on success, error code otherwise  
 */
esp_err_t camera_start(void);

/**
 * @brief Capture an image from the camera
 * 
 * @param buffer Pointer to store the image buffer
 * @param len Pointer to store the buffer length
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t camera_capture(uint8_t **buffer, size_t *len);

/**
 * @brief Get the size of the current captured frame
 * 
 * @return Size of the current frame in bytes
 */
size_t camera_get_frame_size(void);

/**
 * @brief Deinitialize the camera
 */
void camera_deinit(void);

/**
 * @brief Set camera brightness
 * 
 * @param brightness Brightness value (-100 to 100)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t camera_set_brightness(int brightness);

/**
 * @brief Set camera contrast
 * 
 * @param contrast Contrast value (-100 to 100)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t camera_set_contrast(int contrast);

/**
 * @brief Set camera saturation
 * 
 * @param saturation Saturation value (-100 to 100)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t camera_set_saturation(int saturation);

/**
 * @brief Set camera exposure mode
 * 
 * @param mode Exposure mode string ("Auto", "Manual", "Night")
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t camera_set_exposure_mode(const char* mode);

/**
 * @brief Set camera white balance
 * 
 * @param wb White balance string ("Auto", "Daylight", "Cloudy")
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t camera_set_white_balance(const char* wb);

/**
 * @brief Automatically adjust camera settings for optimal image quality
 * 
 * @param frame_buffer Pointer to the current frame buffer
 * @param frame_size Size of the frame buffer
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t camera_auto_adjust(const uint8_t* frame_buffer, size_t frame_size);

#ifdef __cplusplus
}
#endif
