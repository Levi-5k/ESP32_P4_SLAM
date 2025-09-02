/*
 * WiGLE Database Management Component
 * Manages WiGLE WiFi access point database for offline positioning
 */

#pragma once

#include <esp_err.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// WiGLE database entry (same as wifi_positioning.h for compatibility)
typedef struct {
    char bssid[18];            // BSSID in format "XX:XX:XX:XX:XX:XX"
    double latitude;           // WGS84 latitude in degrees
    double longitude;          // WGS84 longitude in degrees
    float altitude;            // Altitude in meters (if available)
    uint32_t last_update;      // Last update timestamp (Unix epoch)
    uint8_t signal_samples;    // Number of signal strength samples
    int8_t avg_rssi;           // Average RSSI at this location
    uint16_t accuracy;         // Position accuracy in meters
} wigle_db_entry_t;

// WiGLE database statistics
typedef struct {
    uint32_t total_entries;
    uint32_t valid_entries;
    uint32_t duplicate_entries;
    uint64_t last_update_timestamp;
    uint32_t database_version;
    char source_file[256];
} wigle_db_stats_t;

// WiGLE database configuration
typedef struct {
    char database_path[256];   // Path to database file
    uint32_t max_entries;      // Maximum entries to store
    bool enable_compression;   // Enable database compression
    uint32_t cache_size;       // Cache size for frequently accessed entries
} wigle_db_config_t;

/**
 * @brief Initialize WiGLE database system
 * @param config Database configuration
 * @return ESP_OK on success
 */
esp_err_t wigle_db_init(const wigle_db_config_t* config);

/**
 * @brief Deinitialize WiGLE database system
 * @return ESP_OK on success
 */
esp_err_t wigle_db_deinit(void);

/**
 * @brief Load WiGLE database from file
 * @param file_path Path to WiGLE CSV or database file
 * @return ESP_OK on success
 */
esp_err_t wigle_db_load_from_file(const char* file_path);

/**
 * @brief Save WiGLE database to file
 * @param file_path Path to save database file
 * @return ESP_OK on success
 */
esp_err_t wigle_db_save_to_file(const char* file_path);

/**
 * @brief Add entry to WiGLE database
 * @param entry Entry to add
 * @return ESP_OK on success
 */
esp_err_t wigle_db_add_entry(const wigle_db_entry_t* entry);

/**
 * @brief Find entry by BSSID
 * @param bssid BSSID to search for
 * @param entry Pointer to store found entry
 * @return ESP_OK if found, ESP_ERR_NOT_FOUND if not found
 */
esp_err_t wigle_db_find_by_bssid(const char* bssid, wigle_db_entry_t* entry);

/**
 * @brief Find entries within radius of a point
 * @param latitude Center latitude
 * @param longitude Center longitude
 * @param radius_m Radius in meters
 * @param entries Array to store found entries
 * @param max_entries Maximum entries to return
 * @param found_count Pointer to store number of entries found
 * @return ESP_OK on success
 */
esp_err_t wigle_db_find_nearby(double latitude, double longitude, float radius_m,
                              wigle_db_entry_t* entries, uint32_t max_entries,
                              uint32_t* found_count);

/**
 * @brief Get database statistics
 * @param stats Pointer to store statistics
 * @return ESP_OK on success
 */
esp_err_t wigle_db_get_stats(wigle_db_stats_t* stats);

/**
 * @brief Clear all entries from database
 * @return ESP_OK on success
 */
esp_err_t wigle_db_clear(void);

/**
 * @brief Optimize database (remove duplicates, sort by usage, etc.)
 * @return ESP_OK on success
 */
esp_err_t wigle_db_optimize(void);

/**
 * @brief Import from WiGLE CSV format
 * @param csv_path Path to WiGLE CSV file
 * @return ESP_OK on success
 */
esp_err_t wigle_db_import_csv(const char* csv_path);

/**
 * @brief Export to WiGLE CSV format
 * @param csv_path Path to export CSV file
 * @return ESP_OK on success
 */
esp_err_t wigle_db_export_csv(const char* csv_path);

/**
 * @brief Check if database is loaded and ready
 * @return true if ready
 */
bool wigle_db_is_ready(void);

/**
 * @brief Get database memory usage
 * @return Memory usage in bytes
 */
uint32_t wigle_db_get_memory_usage(void);

#ifdef __cplusplus
}
#endif
