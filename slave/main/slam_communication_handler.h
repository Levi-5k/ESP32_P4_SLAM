/**
 * @file slam_communication_handler.h
 * @brief Communication handler header for ESP32-C6 slave device
 */

#ifndef SLAM_COMMUNICATION_HANDLER_H
#define SLAM_COMMUNICATION_HANDLER_H

#include <esp_err.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize SLAM communication handler
 * @return ESP_OK on success
 */
esp_err_t slam_communication_init(void);

/**
 * @brief Send command to P4 master
 * @param msg_type Message type
 * @param payload Command payload
 * @param payload_size Payload size
 * @return ESP_OK on success
 */
esp_err_t slam_communication_send_command(uint8_t msg_type, const void* payload, uint16_t payload_size);

/**
 * @brief Simulate P4 data for testing (remove in production)
 * @return ESP_OK on success
 */
esp_err_t slam_communication_simulate_p4_data(void);

#ifdef __cplusplus
}
#endif

#endif // SLAM_COMMUNICATION_HANDLER_H
