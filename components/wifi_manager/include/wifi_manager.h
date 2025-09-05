/**
 * @file wifi_manager.h
 * @brief WiFi Manager Component for ESP32-P4 with ESP-Hosted
 * @note Smart WiFi management with station/AP fallback and ESP-Hosted support
 */

#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <esp_err.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief WiFi manager configuration
 */
typedef struct {
    // Station mode configuration
    const char* sta_ssid;              /**< SSID to connect to in station mode */
    const char* sta_password;          /**< Password for station mode */
    uint32_t sta_connect_timeout_ms;   /**< Connection timeout for station mode */

    // AP mode configuration (fallback)
    const char* ap_ssid;               /**< SSID for AP mode */
    const char* ap_password;           /**< Password for AP mode */
    uint8_t ap_channel;                /**< AP channel */
    uint8_t ap_max_connections;        /**< Maximum AP connections */

    // General configuration
    bool auto_fallback;                /**< Auto fallback to AP if station fails */
    bool scan_only_mode;               /**< Enable scan-only mode (no connections) */
    uint32_t scan_interval_ms;         /**< Network scan interval */
    uint8_t max_retry_count;           /**< Maximum connection retry count */
} wifi_manager_config_t;

/**
 * @brief WiFi manager status
 */
typedef enum {
    WIFI_MGR_STATUS_DISABLED = 0,      /**< WiFi manager disabled */
    WIFI_MGR_STATUS_INITIALIZING,      /**< Initializing */
    WIFI_MGR_STATUS_SCANNING,          /**< Scanning for networks */
    WIFI_MGR_STATUS_SCAN_ONLY,         /**< Scan-only mode (no connections) */
    WIFI_MGR_STATUS_CONNECTING,        /**< Connecting to network */
    WIFI_MGR_STATUS_CONNECTED,         /**< Connected to network */
    WIFI_MGR_STATUS_AP_MODE,           /**< Running in AP mode */
    WIFI_MGR_STATUS_ERROR              /**< Error state */
} wifi_manager_status_t;

/**
 * @brief WiFi connection information
 */
typedef struct {
    char ssid[32];                     /**< Connected SSID */
    char ip_addr[16];                  /**< IP address */
    char mac_addr[18];                 /**< MAC address */
    int8_t rssi;                       /**< Signal strength */
    uint8_t channel;                   /**< WiFi channel */
    bool is_ap_mode;                   /**< True if in AP mode */
    uint8_t client_count;              /**< Number of connected clients (AP mode) */
} wifi_connection_info_t;

/**
 * @brief Initialize WiFi manager
 *
 * @param config WiFi manager configuration
 * @return ESP_OK on success, error code on failure
 */
esp_err_t wifi_manager_init(const wifi_manager_config_t* config);

/**
 * @brief Start WiFi manager
 *
 * @return ESP_OK on success, error code on failure
 */
esp_err_t wifi_manager_start(void);

/**
 * @brief Stop WiFi manager
 *
 * @return ESP_OK on success, error code on failure
 */
esp_err_t wifi_manager_stop(void);

/**
 * @brief Deinitialize WiFi manager
 *
 * @return ESP_OK on success, error code on failure
 */
esp_err_t wifi_manager_deinit(void);

/**
 * @brief Get current WiFi manager status
 *
 * @return Current status
 */
wifi_manager_status_t wifi_manager_get_status(void);

/**
 * @brief Get current connection information
 *
 * @param info Pointer to connection info structure
 * @return ESP_OK on success, error code on failure
 */
esp_err_t wifi_manager_get_connection_info(wifi_connection_info_t* info);

/**
 * @brief Force switch to AP mode
 *
 * @return ESP_OK on success, error code on failure
 */
esp_err_t wifi_manager_force_ap_mode(void);

/**
 * @brief Force switch to station mode
 *
 * @return ESP_OK on success, error code on failure
 */
esp_err_t wifi_manager_force_station_mode(void);

/**
 * @brief Check if WiFi manager is running
 *
 * @return true if running, false otherwise
 */
bool wifi_manager_is_running(void);

/**
 * @brief Get WiFi manager diagnostics
 *
 * @param buffer Buffer to store diagnostics
 * @param buffer_size Size of buffer
 * @return ESP_OK on success, error code on failure
 */
esp_err_t wifi_manager_get_diagnostics(char* buffer, size_t buffer_size);

/**
 * @brief Enable WiFi with optional scan-only mode
 *
 * @param scan_only true for scan-only mode (no connections), false for full mode
 * @return ESP_OK on success, error code on failure
 */
esp_err_t wifi_manager_enable(bool scan_only);

/**
 * @brief Disable WiFi completely
 *
 * @return ESP_OK on success, error code on failure
 */
esp_err_t wifi_manager_disable(void);

/**
 * @brief Enable scan-only mode (scan networks but don't connect)
 *
 * @return ESP_OK on success, error code on failure
 */
esp_err_t wifi_manager_enable_scan_only(void);

/**
 * @brief Start periodic network scanning
 *
 * @return ESP_OK on success, error code on failure
 */
esp_err_t wifi_manager_start_scanning(void);

/**
 * @brief Stop periodic network scanning
 *
 * @return ESP_OK on success, error code on failure
 */
esp_err_t wifi_manager_stop_scanning(void);

/**
 * @brief Check if WiFi is in scan-only mode
 *
 * @return true if in scan-only mode, false otherwise
 */
bool wifi_manager_is_scan_only(void);

#ifdef __cplusplus
}
#endif

#endif /* WIFI_MANAGER_H */
