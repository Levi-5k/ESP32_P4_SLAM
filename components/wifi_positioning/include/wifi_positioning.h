/*
 * WiFi Positioning Component
 * Uses local WiGLE database for offline position estimation
 * Supports ESP32-P4 WiFi remote over SDIO interface
 * Features:
 * - Local WiGLE database lookup from SD card
 * - Automatic SD card directory creation for WiFi map data
 * - RSSI-based distance estimation
 * - Triangulation algorithms
 * - No internet connectivity required
 */

#pragma once

#include <esp_err.h>
#include <stdbool.h>
#include <stdint.h>
#include "sensor_fusion.h"  // For wifi_position_t type

#ifdef __cplusplus
extern "C" {
#endif

// WiFi access point information
typedef struct {
    char ssid[33];             // SSID (32 chars + null terminator)
    char bssid[18];            // BSSID in format "XX:XX:XX:XX:XX:XX"
    int8_t rssi;               // Signal strength in dBm
    uint8_t channel;           // WiFi channel
    uint8_t auth_mode;         // Authentication mode
    bool is_hidden;            // Hidden network flag
} wifi_ap_info_t;

// WiGLE database entry
typedef struct {
    char bssid[18];            // BSSID in format "XX:XX:XX:XX:XX:XX"
    double latitude;           // WGS84 latitude in degrees
    double longitude;          // WGS84 longitude in degrees
    float altitude;            // Altitude in meters (if available)
    uint32_t last_update;      // Last update timestamp (Unix epoch)
    uint8_t signal_samples;    // Number of signal strength samples
    int8_t avg_rssi;           // Average RSSI at this location
    uint16_t accuracy;         // Position accuracy in meters
} wigle_entry_t;

// WiFi positioning configuration
typedef struct {
    uint32_t scan_interval_ms;     // How often to scan for WiFi networks
    uint8_t min_ap_count;          // Minimum APs needed for positioning
    uint8_t max_ap_count;          // Maximum APs to consider
    int8_t min_rssi_threshold;     // Minimum RSSI to consider AP
    uint32_t wigle_db_size;        // Expected size of WiGLE database
    char wigle_db_path[256];       // Path to WiGLE database file
    bool enable_auto_scan;         // Enable automatic scanning
    uint32_t position_timeout_ms;  // Timeout for position calculation
} wifi_positioning_config_t;

// WiFi positioning result (defined in sensor_fusion.h)
// typedef struct wifi_position wifi_position_t;  // Removed - conflicts with sensor_fusion.h

// WiFi positioning status
typedef enum {
    WIFI_POS_STATUS_DISABLED = 0,
    WIFI_POS_STATUS_INITIALIZING,
    WIFI_POS_STATUS_SCANNING,
    WIFI_POS_STATUS_CALCULATING,
    WIFI_POS_STATUS_READY,
    WIFI_POS_STATUS_ERROR
} wifi_positioning_status_t;

// WiFi positioning statistics
typedef struct {
    uint32_t total_scans;
    uint32_t successful_positions;
    uint32_t failed_positions;
    uint32_t database_queries;
    uint32_t database_hits;
    float avg_calculation_time_ms;
    float avg_accuracy_m;
    uint32_t last_scan_ap_count;
} wifi_positioning_stats_t;

/**
 * @brief Initialize WiFi positioning system
 * @param config Configuration parameters
 * @return ESP_OK on success
 */
esp_err_t wifi_positioning_init(const wifi_positioning_config_t* config);

/**
 * @brief Deinitialize WiFi positioning system
 * @return ESP_OK on success
 */
esp_err_t wifi_positioning_deinit(void);

/**
 * @brief Start WiFi scanning and positioning
 * @return ESP_OK on success
 */
esp_err_t wifi_positioning_start(void);

/**
 * @brief Stop WiFi scanning and positioning
 * @return ESP_OK on success
 */
esp_err_t wifi_positioning_stop(void);

/**
 * @brief Perform single WiFi scan and position calculation
 * @param position Pointer to store calculated position
 * @return ESP_OK on success
 */
esp_err_t wifi_positioning_scan_and_calculate(wifi_position_t* position);

/**
 * @brief Get current WiFi position estimate
 * @param position Pointer to store position data
 * @return ESP_OK on success
 */
esp_err_t wifi_positioning_get_position(wifi_position_t* position);

/**
 * @brief Get WiFi positioning status
 * @return Current status
 */
wifi_positioning_status_t wifi_positioning_get_status(void);

/**
 * @brief Get WiFi positioning statistics
 * @param stats Pointer to store statistics
 * @return ESP_OK on success
 */
esp_err_t wifi_positioning_get_stats(wifi_positioning_stats_t* stats);

/**
 * @brief Check if WiFi positioning is available
 * @return true if available and ready
 */
bool wifi_positioning_is_available(void);

/**
 * @brief Set WiFi positioning configuration
 * @param config New configuration
 * @return ESP_OK on success
 */
esp_err_t wifi_positioning_set_config(const wifi_positioning_config_t* config);

/**
 * @brief Load WiGLE database from SD card
 * @param db_path Path to WiGLE database file
 * @return ESP_OK on success
 */
esp_err_t wifi_positioning_load_wigle_db(const char* db_path);

/**
 * @brief Get number of WiFi access points currently visible
 * @return Number of APs found
 */
uint8_t wifi_positioning_get_visible_ap_count(void);

/**
 * @brief Get list of visible WiFi access points
 * @param aps Array to store AP information
 * @param max_count Maximum number of APs to return
 * @return Number of APs stored in array
 */
uint8_t wifi_positioning_get_visible_aps(wifi_ap_info_t* aps, uint8_t max_count);

/**
 * @brief Process external WiFi access point data for positioning
 * @param aps Array of access point information from external source
 * @param ap_count Number of access points in array
 * @param position Pointer to store calculated position
 * @return ESP_OK on success
 */
esp_err_t wifi_positioning_process_external_aps(const wifi_ap_info_t* aps, uint8_t ap_count, wifi_position_t* position);

#ifdef __cplusplus
}
#endif
