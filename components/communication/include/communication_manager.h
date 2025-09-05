/**
 * @file communication_manager.h
 * @brief Communication manager header for ESP32-P4 master
 */

#ifndef COMMUNICATION_MANAGER_H
#define COMMUNICATION_MANAGER_H

#include "communication_protocol.h"
#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize communication manager
 * @return ESP_OK on success
 */
esp_err_t comm_manager_init(void);

/**
 * @brief Send heartbeat with current system status
 * @return ESP_OK on success
 */
esp_err_t comm_send_heartbeat_auto(void);

/**
 * @brief Queue a message for sending to C6
 * @param msg_type Message type
 * @param payload Payload data
 * @param payload_size Size of payload
 * @return ESP_OK on success
 */
esp_err_t comm_queue_message(uint8_t msg_type, const void* payload, uint16_t payload_size);

/**
 * @brief Send a raw message to C6
 * @param message Complete message to send
 * @return ESP_OK on success
 */
esp_err_t comm_send_message_raw(const comm_message_t* message);

/**
 * @brief Process a received message from C6
 * @param message Received message to process
 * @return ESP_OK on success
 */
esp_err_t comm_process_received_message(const comm_message_t* message);

/**
 * @brief Handle configuration update from web interface
 */
esp_err_t comm_handle_config_update(const config_update_msg_t* config);

/**
 * @brief Handle WiFi scan request from web interface
 */
esp_err_t comm_handle_wifi_scan_request(void);

/**
 * @brief Handle WiFi connection request from web interface
 */
esp_err_t comm_handle_wifi_connect(const wifi_connect_msg_t* wifi_config);

/**
 * @brief Handle WiFi control command from web interface
 */
esp_err_t comm_handle_wifi_control(const wifi_control_msg_t* wifi_control);

/**
 * @brief Handle system command from web interface
 */
esp_err_t comm_handle_system_command(const system_cmd_msg_t* cmd);

/**
 * @brief Handle map command from web interface
 */
esp_err_t comm_handle_map_command(const map_cmd_msg_t* cmd);

/**
 * @brief Check if communication with C6 is active
 * @return true if connected, false otherwise
 */
bool comm_is_connected(void);

/**
 * @brief Get timestamp of last successful heartbeat
 * @return Timestamp in milliseconds
 */
uint32_t comm_get_last_heartbeat_time(void);

#ifdef __cplusplus
}
#endif

#endif // COMMUNICATION_MANAGER_H
