/**
 * @file web_server.h
 * @brief Comprehensive web server for ESP32-P4 Visual SLAM system
 *
 * Provides full-featured web interface with live data display,
 * settings management, and real-time telemetry broadcasting.
 */

#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include <esp_err.h>
#include <stdbool.h>
#include "visual_slam_common_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Web server configuration structure
 */
typedef struct {
    uint16_t port;              /**< HTTP server port (default: 80) */
    uint8_t max_clients;        /**< Maximum number of concurrent clients */
    bool enable_cors;           /**< Enable CORS headers for web interface */
    bool enable_auth;           /**< Enable basic authentication */
    bool enable_captive_portal; /**< Enable captive portal mode */
    bool websocket_enabled;     /**< Enable WebSocket for real-time data */
    const char* ssid;           /**< WIFI AP SSID */
    const char* password;       /**< WIFI AP password */
    uint8_t channel;            /**< WIFI channel */
    uint8_t max_connection;     /**< Maximum WIFI connections */
} web_server_config_t;

/**
 * @brief Initialize web server
 *
 * @param config Web server configuration
 * @return ESP_OK on success, error code on failure
 */
esp_err_t web_server_init(const web_server_config_t* config);

/**
 * @brief Start WIFI access point
 *
 * @return ESP_OK on success, error code on failure
 */
esp_err_t web_server_wifi_start(void);

/**
 * @brief Stop WIFI access point
 *
 * @return ESP_OK on success, error code on failure
 */
esp_err_t web_server_wifi_stop(void);

/**
 * @brief Start web server
 *
 * @return ESP_OK on success, error code on failure
 */
esp_err_t web_server_start(void);

/**
 * @brief Stop web server
 *
 * @return ESP_OK on success, error code on failure
 */
esp_err_t web_server_stop(void);

/**
 * @brief Deinitialize web server
 *
 * @return ESP_OK on success, error code on failure
 */
esp_err_t web_server_deinit(void);

/**
 * @brief Check if web server is running
 *
 * @return true if running, false otherwise
 */
bool web_server_is_running(void);

/**
 * @brief Broadcast telemetry data to connected clients
 *
 * @param system_status Current system status structure
 * @return ESP_OK on success, error code on failure
 */
esp_err_t web_server_broadcast_telemetry(const system_status_t* system_status);

/**
 * @brief Send notification to connected clients
 *
 * @param message Notification message
 * @param type Message type (info, warning, error)
 * @return ESP_OK on success, error code on failure
 */
esp_err_t web_server_send_notification(const char* message, const char* type);

#ifdef __cplusplus
}
#endif

#endif /* WEB_SERVER_H */
