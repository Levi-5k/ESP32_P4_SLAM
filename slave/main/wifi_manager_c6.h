/**
 * @file wifi_manager_c6.h
 * @brief WiFi manager for ESP32-C6 with AP mode fallback and comprehensive logging
 */

#ifndef WIFI_MANAGER_C6_H
#define WIFI_MANAGER_C6_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "esp_wifi.h"
#include "esp_event.h"

#ifdef __cplusplus
extern "C" {
#endif

// WiFi manager configuration
typedef struct {
    char default_ssid[32];          // Default SSID to connect to
    char default_password[64];      // Default password
    char ap_ssid[32];               // AP mode SSID
    char ap_password[64];           // AP mode password
    uint32_t connection_timeout_ms; // Timeout for STA connection attempts
    uint8_t max_retry_attempts;     // Max retry attempts before fallback
    bool enable_ap_fallback;        // Enable AP mode fallback
    uint8_t ap_channel;             // AP mode channel
    uint8_t ap_max_connections;     // Max AP connections
    bool enable_captive_portal;     // Enable captive portal for AP mode
    char captive_portal_url[128];   // Captive portal redirect URL
} wifi_manager_config_t;

// WiFi manager status
typedef enum {
    WIFI_MGR_STATUS_IDLE = 0,           // Not initialized
    WIFI_MGR_STATUS_CONNECTING,         // Attempting STA connection
    WIFI_MGR_STATUS_CONNECTED,          // Connected as STA
    WIFI_MGR_STATUS_DISCONNECTED,       // STA disconnected
    WIFI_MGR_STATUS_AP_STARTING,        // Starting AP mode
    WIFI_MGR_STATUS_AP_RUNNING,         // Running in AP mode
    WIFI_MGR_STATUS_ERROR               // Error state
} wifi_manager_status_t;

// WiFi manager events
typedef enum {
    WIFI_MGR_EVENT_STA_CONNECTED = 0,   // STA connected to AP
    WIFI_MGR_EVENT_STA_DISCONNECTED,    // STA disconnected
    WIFI_MGR_EVENT_STA_GOT_IP,          // STA got IP address
    WIFI_MGR_EVENT_AP_STARTED,          // AP mode started
    WIFI_MGR_EVENT_AP_STOPPED,          // AP mode stopped
    WIFI_MGR_EVENT_AP_CLIENT_CONNECTED, // Client connected to our AP
    WIFI_MGR_EVENT_AP_CLIENT_DISCONNECTED, // Client disconnected from our AP
    WIFI_MGR_EVENT_CONNECTION_FAILED,   // Connection attempt failed
    WIFI_MGR_EVENT_FALLBACK_TO_AP       // Fallback to AP mode triggered
} wifi_manager_event_t;

// WiFi connection info
typedef struct {
    char ssid[33];                      // Current SSID
    char ip_address[16];                // Current IP address
    int8_t rssi;                        // Signal strength
    uint8_t channel;                    // WiFi channel
    wifi_auth_mode_t auth_mode;         // Authentication mode
    bool is_connected;                  // Connection status
    bool is_ap_mode;                    // AP mode status
    uint32_t uptime_seconds;            // Connection uptime
    uint8_t connected_clients;          // Number of connected clients (AP mode)
} wifi_connection_info_t;

// WiFi manager statistics
typedef struct {
    uint32_t sta_connection_attempts;   // Total STA connection attempts
    uint32_t sta_successful_connections; // Successful STA connections
    uint32_t sta_disconnections;        // STA disconnections
    uint32_t ap_mode_activations;       // AP mode activations
    uint32_t ap_client_connections;     // Total AP client connections
    uint32_t last_connection_time;      // Last successful connection timestamp
    uint32_t total_uptime_seconds;      // Total connection uptime
} wifi_manager_stats_t;

// Callback function type for WiFi events
typedef void (*wifi_manager_event_callback_t)(wifi_manager_event_t event, void* event_data);

/**
 * @brief Initialize WiFi manager with default configuration
 * @param config WiFi manager configuration
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_manager_c6_init(const wifi_manager_config_t* config);

/**
 * @brief Start WiFi manager (attempt STA connection first)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_manager_c6_start(void);

/**
 * @brief Stop WiFi manager
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_manager_c6_stop(void);

/**
 * @brief Connect to specific WiFi network
 * @param ssid Network SSID
 * @param password Network password
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_manager_c6_connect(const char* ssid, const char* password);

/**
 * @brief Force switch to AP mode
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_manager_c6_start_ap_mode(void);

/**
 * @brief Get current WiFi status
 * @return Current WiFi manager status
 */
wifi_manager_status_t wifi_manager_c6_get_status(void);

/**
 * @brief Get current connection information
 * @param info Pointer to store connection info
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_manager_c6_get_connection_info(wifi_connection_info_t* info);

/**
 * @brief Get WiFi manager statistics
 * @param stats Pointer to store statistics
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_manager_c6_get_stats(wifi_manager_stats_t* stats);

/**
 * @brief Register event callback
 * @param callback Callback function for WiFi events
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_manager_c6_register_callback(wifi_manager_event_callback_t callback);

/**
 * @brief Scan for available networks
 * @param ap_records Array to store scan results
 * @param max_records Maximum number of records to store
 * @param actual_records Actual number of records found
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_manager_c6_scan_networks(wifi_ap_record_t* ap_records, 
                                        uint16_t max_records, 
                                        uint16_t* actual_records);

/**
 * @brief Get WiFi diagnostics string
 * @param buffer Buffer to store diagnostics
 * @param buffer_size Size of buffer
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_manager_c6_get_diagnostics(char* buffer, size_t buffer_size);

/**
 * @brief Print comprehensive WiFi status to console
 */
void wifi_manager_c6_print_status(void);

/**
 * @brief Check if WiFi manager is initialized
 * @return true if initialized, false otherwise
 */
bool wifi_manager_c6_is_initialized(void);

/**
 * @brief Get status string representation
 * @param status WiFi manager status
 * @return Status string
 */
const char* wifi_manager_c6_status_to_string(wifi_manager_status_t status);

/**
 * @brief Start continuous WiFi scanning for positioning
 * @param scan_interval_ms Interval between scans in milliseconds
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_manager_c6_start_continuous_scan(uint32_t scan_interval_ms);

/**
 * @brief Stop continuous WiFi scanning
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_manager_c6_stop_continuous_scan(void);

/**
 * @brief Set callback for new WiFi scan results (for positioning)
 * @param callback Callback function to receive scan results
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_manager_c6_set_scan_callback(void (*callback)(const wifi_ap_record_t* records, uint16_t count));

/**
 * @brief Get the latest WiFi scan results for positioning
 * @param ap_records Buffer to store access point records
 * @param max_records Maximum number of records to return
 * @param actual_records Pointer to store actual number of records returned
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_manager_c6_get_latest_scan(wifi_ap_record_t* ap_records, 
                                          uint16_t max_records, 
                                          uint16_t* actual_records);

/**
 * @brief Check if continuous scanning is active
 * @return true if scanning, false otherwise
 */
bool wifi_manager_c6_is_scanning(void);

#ifdef __cplusplus
}
#endif

#endif // WIFI_MANAGER_C6_H
