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

/**
 * @brief Start WiFi positioning data transmission to P4
 * @param scan_interval_ms Interval between WiFi scans in milliseconds
 * @return ESP_OK on success
 */
esp_err_t slam_comm_start_wifi_positioning(uint32_t scan_interval_ms);

/**
 * @brief Stop WiFi positioning data transmission
 * @return ESP_OK on success
 */
esp_err_t slam_comm_stop_wifi_positioning(void);

/**
 * @brief Initiate handshake with P4
 * @return ESP_OK on success
 */
esp_err_t slam_comm_initiate_handshake(void);

/**
 * @brief Request WiFi credentials from P4
 * @return ESP_OK on success
 */
esp_err_t slam_comm_request_wifi_credentials(void);

#ifdef __cplusplus
}
#endif

#endif // SLAM_COMMUNICATION_HANDLER_H
