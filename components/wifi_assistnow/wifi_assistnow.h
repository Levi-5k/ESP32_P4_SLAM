/**
 * @file wifi_assistnow.h
 * @brief WiFi component for AssistNow Offline downloads
 * @note Minimal WiFi implementation for downloading GPS assistance data
 */

#ifndef WIFI_ASSISTNOW_H
#define WIFI_ASSISTNOW_H

#include <esp_err.h>
#include <stdbool.h>

// WiFi configuration structure
typedef struct {
    char ssid[32];              // WiFi SSID
    char password[64];          // WiFi password
    uint32_t connect_timeout_ms; // Connection timeout
    bool auto_reconnect;        // Auto reconnect on disconnect
} wifi_assistnow_config_t;

// WiFi status
typedef enum {
    WIFI_ASSISTNOW_DISCONNECTED = 0,
    WIFI_ASSISTNOW_CONNECTING,
    WIFI_ASSISTNOW_CONNECTED,
    WIFI_ASSISTNOW_FAILED
} wifi_assistnow_status_t;

/**
 * @brief Initialize WiFi for AssistNow downloads
 * @param config WiFi configuration
 * @return ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t wifi_assistnow_init(const wifi_assistnow_config_t *config);

/**
 * @brief Connect to WiFi network
 * @return ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t wifi_assistnow_connect(void);

/**
 * @brief Disconnect from WiFi network
 * @return ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t wifi_assistnow_disconnect(void);

/**
 * @brief Get current WiFi connection status
 * @return Current WiFi status
 */
wifi_assistnow_status_t wifi_assistnow_get_status(void);

/**
 * @brief Check if WiFi is currently connected
 * @return true if connected, false otherwise
 */
bool wifi_assistnow_is_connected(void);

/**
 * @brief Deinitialize WiFi component
 */
void wifi_assistnow_deinit(void);

#endif // WIFI_ASSISTNOW_H
