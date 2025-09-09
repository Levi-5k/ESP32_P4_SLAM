/**
 * @file slam_web_server.h
 * @brief Web server for ESP32-C6 slave device
 * 
 * Provides web interface for SLAM system running on ESP32-P4 master.
 * Communicates via ESP-Hosted protocol to exchange data and commands.
 */

#ifndef SLAM_WEB_SERVER_H
#define SLAM_WEB_SERVER_H

#include <esp_err.h>
#include <esp_http_server.h>
#include <stdbool.h>
#include "communication_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Web server configuration
 */
typedef struct {
    uint16_t port;              /**< HTTP server port (default: 80) */
    uint8_t max_clients;        /**< Maximum concurrent clients */
    bool enable_cors;           /**< Enable CORS headers */
    uint32_t stack_size;        /**< Task stack size */
    bool enable_captive_portal; /**< Enable captive portal functionality */
    bool captive_portal_redirect_all; /**< Redirect all requests to main page */
} web_server_config_t;

/**
 * @brief SLAM system data cache for web interface
 */
typedef struct {
    // System status
    heartbeat_msg_t heartbeat;
    slam_status_msg_t slam_status;
    position_msg_t position;
    telemetry_msg_t telemetry;
    
    // Timestamps for data freshness
    uint32_t heartbeat_timestamp;
    uint32_t slam_status_timestamp;
    uint32_t position_timestamp;
    uint32_t telemetry_timestamp;
    
    // Connection status
    bool p4_connected;
    uint32_t last_p4_message_time;
    
    // WiFi status and scan results
    wifi_status_msg_t wifi_status;
    wifi_scan_response_t wifi_scan_results;
    uint32_t wifi_status_timestamp;
    uint32_t wifi_scan_timestamp;
    
    // Camera preview data
    bool has_camera_frame;
    uint8_t* camera_frame_data;
    uint32_t camera_frame_size;
    uint32_t camera_frame_timestamp;
    uint16_t camera_frame_width;
    uint16_t camera_frame_height;
    uint32_t camera_frame_number;
    
    // Cached configuration (for settings validation)
    struct {
        uint32_t max_features;
        uint32_t fast_threshold;
        float scale_factor;
        uint32_t min_tracked_features;
        bool enable_loop_closure;
    } slam_config;
    
    // Camera settings cache
    uint32_t camera_exposure;
    uint32_t camera_brightness;
    uint32_t camera_fps;
    
} slam_data_cache_t;

/**
 * @brief Initialize SLAM web server
 * @param config Web server configuration
 * @return ESP_OK on success
 */
esp_err_t slam_web_server_init(const web_server_config_t* config);

/**
 * @brief Start web server
 * @return ESP_OK on success
 */
esp_err_t slam_web_server_start(void);

/**
 * @brief Stop web server
 * @return ESP_OK on success
 */
esp_err_t slam_web_server_stop(void);

/**
 * @brief Update system data from P4 master
 * @param message Received message from P4
 * @return ESP_OK on success
 */
esp_err_t slam_web_server_update_data(const comm_message_t* message);

/**
 * @brief Send command to P4 master
 * @param msg_type Message type
 * @param payload Command payload
 * @param payload_size Payload size
 * @return ESP_OK on success
 */
esp_err_t slam_web_server_send_command(uint8_t msg_type, const void* payload, uint16_t payload_size);

/**
 * @brief Get current SLAM data cache
 * @return Pointer to data cache
 */
const slam_data_cache_t* slam_web_server_get_data_cache(void);

/**
 * @brief Check if P4 master is connected
 * @return true if connected, false otherwise
 */
bool slam_web_server_is_p4_connected(void);

/**
 * @brief WebSocket broadcast SLAM data to connected clients
 * @return ESP_OK on success
 */
esp_err_t slam_web_server_broadcast_data(void);

#ifdef __cplusplus
}
#endif

#endif // SLAM_WEB_SERVER_H
