#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// SD card mount point
#define SD_MOUNT_POINT "/sdcard"

// Map file directories
#define SD_MAPS_DIR SD_MOUNT_POINT "/maps"
#define SD_SESSIONS_DIR SD_MOUNT_POINT "/sessions"
#define SD_CONFIG_DIR SD_MOUNT_POINT "/config"

// File paths for different data types
#define SD_SLAM_MAP_FILE SD_MAPS_DIR "/slam_map.bin"
#define SD_KEYFRAMES_FILE SD_MAPS_DIR "/keyframes.bin"
#define SD_MAP_POINTS_FILE SD_MAPS_DIR "/map_points.bin"
#define SD_CONFIG_FILE SD_CONFIG_DIR "/slam_config.json"

// SD card configuration
typedef struct {
    int clk_pin;
    int cmd_pin;
    int d0_pin;
    int d1_pin;
    int d2_pin;
    int d3_pin;
    uint32_t max_freq_khz;
    bool format_if_mount_failed;
} sd_card_config_t;

// SD card status
typedef struct {
    bool mounted;
    uint64_t total_size_mb;
    uint64_t free_size_mb;
    char card_name[32];
    uint32_t speed_class;
} sd_card_info_t;

// Map file header structure
typedef struct {
    uint32_t magic;                 // Magic number for validation
    uint32_t version;               // File format version
    uint64_t timestamp;             // Creation timestamp
    uint32_t keyframe_count;        // Number of keyframes
    uint32_t map_point_count;       // Number of map points
    uint32_t checksum;              // Data checksum
    char mission_name[64];          // Mission identifier
    double origin_lat;              // GPS origin latitude
    double origin_lon;              // GPS origin longitude
    float origin_alt;               // GPS origin altitude
} map_file_header_t;

// Session data for logging
typedef struct {
    uint64_t session_start;
    uint64_t session_duration;
    uint32_t total_frames;
    uint32_t tracked_frames;
    float max_tracking_confidence;
    float average_processing_time;
    char session_name[32];
} session_metadata_t;

/**
 * Initialize SD card storage
 * @param config SD card pin configuration (NULL for default)
 * @return ESP_OK on success
 */
esp_err_t sd_storage_init(const sd_card_config_t *config);

/**
 * Deinitialize SD card storage
 * @return ESP_OK on success
 */
esp_err_t sd_storage_deinit(void);

/**
 * Get SD card information
 * @param info Pointer to store card information
 * @return ESP_OK on success
 */
esp_err_t sd_storage_get_info(sd_card_info_t *info);

/**
 * Format SD card (WARNING: Erases all data!)
 * @return ESP_OK on success
 */
esp_err_t sd_storage_format_card(void);

// =============================================================================
// MAP DATA MANAGEMENT
// =============================================================================

/**
 * Save complete SLAM map to SD card
 * @param map_name Mission/map identifier
 * @param origin_lat GPS origin latitude
 * @param origin_lon GPS origin longitude  
 * @param origin_alt GPS origin altitude
 * @return ESP_OK on success
 */
esp_err_t sd_storage_save_slam_map(const char *map_name, 
                                   double origin_lat, 
                                   double origin_lon, 
                                   float origin_alt);

/**
 * Load complete SLAM map from SD card
 * @param map_name Mission/map identifier (NULL for latest)
 * @param origin_lat Pointer to store GPS origin latitude
 * @param origin_lon Pointer to store GPS origin longitude
 * @param origin_alt Pointer to store GPS origin altitude
 * @return ESP_OK on success
 */
esp_err_t sd_storage_load_slam_map(const char *map_name,
                                   double *origin_lat,
                                   double *origin_lon,
                                   float *origin_alt);

/**
 * List available maps on SD card
 * @param map_names Array to store map names
 * @param max_maps Maximum number of maps to list
 * @param count Pointer to store actual number of maps found
 * @return ESP_OK on success
 */
esp_err_t sd_storage_list_maps(char map_names[][64], uint32_t max_maps, uint32_t *count);

/**
 * Delete map from SD card
 * @param map_name Map identifier to delete
 * @return ESP_OK on success
 */
esp_err_t sd_storage_delete_map(const char *map_name);

// =============================================================================
// SESSION LOGGING
// =============================================================================

/**
 * Start logging session data
 * @param session_name Session identifier
 * @return ESP_OK on success
 */
esp_err_t sd_storage_start_session_log(const char *session_name);

/**
 * Log SLAM pose data during session
 * @param timestamp Pose timestamp
 * @param pose SLAM pose data
 * @return ESP_OK on success
 */
esp_err_t sd_storage_log_slam_pose(uint64_t timestamp, const void *pose);

/**
 * Stop session logging and save metadata
 * @param metadata Session summary data
 * @return ESP_OK on success
 */
esp_err_t sd_storage_stop_session_log(const session_metadata_t *metadata);

// =============================================================================
// CONFIGURATION MANAGEMENT
// =============================================================================

/**
 * Save SLAM configuration to SD card
 * @param config SLAM configuration structure
 * @return ESP_OK on success
 */
esp_err_t sd_storage_save_config(const void *config);

/**
 * Load SLAM configuration from SD card
 * @param config Pointer to store loaded configuration
 * @return ESP_OK on success
 */
esp_err_t sd_storage_load_config(void *config);

// =============================================================================
// UTILITY FUNCTIONS
// =============================================================================

/**
 * Check if file exists on SD card
 * @param filepath Full path to file
 * @return true if file exists
 */
bool sd_storage_file_exists(const char *filepath);

/**
 * Get file size
 * @param filepath Full path to file
 * @param size Pointer to store file size
 * @return ESP_OK on success
 */
esp_err_t sd_storage_get_file_size(const char *filepath, size_t *size);

/**
 * Create directory on SD card
 * @param dirpath Directory path
 * @return ESP_OK on success
 */
esp_err_t sd_storage_create_directory(const char *dirpath);

/**
 * Clean up old session logs (keep last N sessions)
 * @param keep_count Number of recent sessions to keep
 * @return ESP_OK on success
 */
esp_err_t sd_storage_cleanup_old_sessions(uint32_t keep_count);

// =============================================================================
// MAP MANAGER FUNCTIONS (High-level interface)
// =============================================================================

/**
 * Initialize map management system (includes SD card init)
 */
esp_err_t map_manager_init(void);

/**
 * Save current SLAM map with automatic GPS origin
 */
esp_err_t save_current_slam_map(const char *map_name);

/**
 * Load SLAM map and set GPS origin
 */
esp_err_t load_slam_map(const char *map_name);

/**
 * Auto-save map if conditions are met (call periodically)
 */
esp_err_t auto_save_map_if_needed(void);

/**
 * List available maps on SD card
 */
esp_err_t list_available_maps(void);

/**
 * Start mission data logging
 */
esp_err_t start_mission_logging(const char *mission_name);

/**
 * Log current SLAM pose (call regularly during missions)
 */
esp_err_t log_current_slam_pose(void);

/**
 * Stop mission logging with metadata
 */
esp_err_t stop_mission_logging(const char *mission_name);

#ifdef __cplusplus
}
#endif
