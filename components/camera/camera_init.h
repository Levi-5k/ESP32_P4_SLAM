/*
 * Camera initialization header for ESP32-P4 with OV5647 MIPI-CSI
 * Clean implementation using ESP-IDF v5.5 API
 */

#ifndef CAMERA_INIT_H
#define CAMERA_INIT_H

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"
#include "visual_slam_common_types.h"  // Use existing camera_config_t definition

// Function declarations

/**
 * @brief Initialize the camera system
 *
 * This function initializes the MIPI-CSI camera with OV5647 sensor
 * including I2C communication, LDO power, and CSI controller setup.
 *
 * @return ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t camera_init(void);

/**
 * @brief Start the camera capture
 *
 * Enables the camera sensor and CSI controller for frame capture.
 *
 * @return ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t camera_start(void);

/**
 * @brief Capture a single frame from the camera
 *
 * Captures a frame and returns the buffer pointer and size.
 *
 * @param[out] buffer Pointer to the captured frame buffer
 * @param[out] len Size of the captured frame in bytes
 * @return ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t camera_capture(uint8_t **buffer, size_t *len);

/**
 * @brief Get the expected frame buffer size
 *
 * @return Frame buffer size in bytes
 */
size_t camera_get_frame_size(void);

/**
 * @brief Deinitialize the camera system
 *
 * Cleans up all camera resources and disables power.
 */
void camera_deinit(void);

// Auto-adjustment functions (stubs for future implementation)

/**
 * @brief Set camera brightness
 *
 * @param brightness Brightness value (0-255)
 * @return ESP_OK on success, ESP_ERR_NOT_SUPPORTED if not implemented
 */
esp_err_t camera_set_brightness(int brightness);

/**
 * @brief Set camera contrast
 *
 * @param contrast Contrast value (0-255)
 * @return ESP_OK on success, ESP_ERR_NOT_SUPPORTED if not implemented
 */
esp_err_t camera_set_contrast(int contrast);

/**
 * @brief Set camera saturation
 *
 * @param saturation Saturation value (0-255)
 * @return ESP_OK on success, ESP_ERR_NOT_SUPPORTED if not implemented
 */
esp_err_t camera_set_saturation(int saturation);

/**
 * @brief Set camera exposure mode
 *
 * @param mode Exposure mode string ("auto", "manual", etc.)
 * @return ESP_OK on success, ESP_ERR_NOT_SUPPORTED if not implemented
 */
esp_err_t camera_set_exposure_mode(const char* mode);

/**
 * @brief Set camera white balance
 *
 * @param wb White balance mode string ("auto", "daylight", "cloudy", etc.)
 * @return ESP_OK on success, ESP_ERR_NOT_SUPPORTED if not implemented
 */
esp_err_t camera_set_white_balance(const char* wb);

/**
 * @brief Perform automatic camera adjustment based on frame analysis
 *
 * @param frame_buffer Pointer to the frame buffer
 * @param frame_size Size of the frame buffer
 * @return ESP_OK on success, ESP_ERR_NOT_SUPPORTED if not implemented
 */
esp_err_t camera_auto_adjust(const uint8_t* frame_buffer, size_t frame_size);

#endif // CAMERA_INIT_H
