/**
 * @file web_server.h
 * @brief WIFI remote web server for ESP32-P4 Visual SLAM system
 *
 * Provides WIFI access point and web interface for remote monitoring
 * and control of the Visual SLAM navigation system.
 */

#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include <esp_err.h>
#include <stdbool.h>

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

#ifdef __cplusplus
}
#endif

#endif /* WEB_SERVER_H */
