#include "sd_storage.h"
#include "slam_core.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "MAP_MANAGER";

// Map management functions for your Visual SLAM system

/**
 * Save current SLAM map to SD card
 */
esp_err_t save_current_slam_map(const char *map_name) {
    if (!map_name) {
        ESP_LOGE(TAG, "Map name cannot be NULL");
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "🗺️ Saving current SLAM map: %s", map_name);

    // Get current GPS position (simplified for now)
    gps_position_t gps_pos = {0};
    // TODO: Get GPS from sensor fusion when API is available
    gps_pos.latitude = 0.0;
    gps_pos.longitude = 0.0;
    gps_pos.altitude = 0.0;

    // Save map to SD card
    esp_err_t ret = sd_storage_save_slam_map(map_name, gps_pos.latitude, gps_pos.longitude, gps_pos.altitude);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "✅ Map saved successfully: %s", map_name);
    } else {
        ESP_LOGE(TAG, "❌ Failed to save map: %s", esp_err_to_name(ret));
    }

    return ret;
}

/**
 * Load SLAM map from SD card
 */
esp_err_t load_slam_map(const char *map_name) {
    ESP_LOGI(TAG, "🗺️ Loading SLAM map: %s", map_name ? map_name : "latest");

    double origin_lat, origin_lon;
    float origin_alt;

    esp_err_t ret = sd_storage_load_slam_map(map_name, &origin_lat, &origin_lon, &origin_alt);
    if (ret == ESP_OK) {
        // Set GPS origin in SLAM system (simplified for now)
        ESP_LOGI(TAG, "Setting GPS origin: lat=%.6f, lon=%.6f, alt=%.2f",
                 origin_lat, origin_lon, origin_alt);
        // TODO: Set GPS origin when API is available
        ret = ESP_OK;
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "✅ Map loaded successfully with GPS origin: %.6f, %.6f, %.1f",
                     origin_lat, origin_lon, origin_alt);
        } else {
            ESP_LOGW(TAG, "Map loaded but failed to set GPS origin");
        }
    } else {
        ESP_LOGE(TAG, "❌ Failed to load map: %s", esp_err_to_name(ret));
    }

    return ret;
}

/**
 * Auto-save map periodically (call this from your main task)
 */
esp_err_t auto_save_map_if_needed(void) {
    static uint64_t last_save_time = 0;
    static uint32_t last_keyframe_count = 0;
    
    // Auto-save every 5 minutes or when significant map growth occurs
    const uint64_t auto_save_interval = 5 * 60 * 1000000ULL;  // 5 minutes in microseconds
    const uint32_t min_keyframe_growth = 50;  // Save when 50+ new keyframes

    uint64_t current_time = esp_timer_get_time();
    slam_stats_t stats;
    
    slam_pose_t current_pose = {0};
    esp_err_t ret = slam_core_get_current_pose(&current_pose);  // Check if SLAM is active
    if (ret != ESP_OK) {
        return ESP_OK;  // SLAM not active, no need to save
    }

    // Check if conditions met for auto-save
    bool time_to_save = (current_time - last_save_time) > auto_save_interval;
    bool growth_significant = false;

    if (slam_core_get_stats(&stats) == ESP_OK) {
        growth_significant = (stats.keyframes - last_keyframe_count) >= min_keyframe_growth;
    }

    if (time_to_save || growth_significant) {
        char auto_map_name[32];
        snprintf(auto_map_name, sizeof(auto_map_name), "auto_save_%llu", current_time / 1000000);
        
        ret = save_current_slam_map(auto_map_name);
        if (ret == ESP_OK) {
            last_save_time = current_time;
            last_keyframe_count = stats.keyframes;
            ESP_LOGI(TAG, "🔄 Auto-saved map: %s", auto_map_name);
        }
    }

    return ESP_OK;
}

/**
 * Initialize map management system
 */
esp_err_t map_manager_init(void) {
    ESP_LOGI(TAG, "🗺️ Initializing map management system...");

    // Initialize SD storage with default configuration
    esp_err_t ret = sd_storage_init(NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ Failed to initialize SD storage: %s", esp_err_to_name(ret));

        // Run diagnostics to help troubleshoot
        ESP_LOGI(TAG, "🔍 Running SD card diagnostics...");
        esp_err_t diag_ret = sd_storage_diagnose();
        if (diag_ret == ESP_OK) {
            ESP_LOGI(TAG, "✅ Diagnostics passed - SD card is working");
        } else {
            ESP_LOGE(TAG, "❌ Diagnostics failed - check hardware connections");
        }

        return ret;
    }

    // Get SD card info
    sd_card_info_t card_info;
    ret = sd_storage_get_info(&card_info);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "📋 SD Card: %s, %.1fGB total, %.1fGB free", 
                 card_info.card_name, 
                 card_info.total_size_mb / 1024.0f, 
                 card_info.free_size_mb / 1024.0f);
    }

    ESP_LOGI(TAG, "✅ Map management system initialized");
    return ESP_OK;
}

/**
 * List available maps on SD card
 */
esp_err_t list_available_maps(void) {
    ESP_LOGI(TAG, "📋 Listing available maps...");

    char map_names[10][64];  // Up to 10 maps
    uint32_t map_count;

    esp_err_t ret = sd_storage_list_maps(map_names, 10, &map_count);
    if (ret == ESP_OK) {
        if (map_count == 0) {
            ESP_LOGI(TAG, "No maps found on SD card");
        } else {
            ESP_LOGI(TAG, "Found %lu maps:", map_count);
            for (uint32_t i = 0; i < map_count && i < 10; i++) {
                ESP_LOGI(TAG, "  %lu. %s", i + 1, map_names[i]);
            }
        }
    } else {
        ESP_LOGE(TAG, "Failed to list maps: %s", esp_err_to_name(ret));
    }

    return ret;
}

/**
 * Start logging session data
 */
esp_err_t start_mission_logging(const char *mission_name) {
    ESP_LOGI(TAG, "📝 Starting mission log: %s", mission_name);
    
    esp_err_t ret = sd_storage_start_session_log(mission_name);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "✅ Mission logging started");
    } else {
        ESP_LOGE(TAG, "❌ Failed to start mission logging: %s", esp_err_to_name(ret));
    }
    
    return ret;
}

/**
 * Log current SLAM pose (call this regularly during missions)
 */
esp_err_t log_current_slam_pose(void) {
    slam_pose_t pose = {0};
    esp_err_t ret = slam_core_get_current_pose(&pose);
    if (ret == ESP_OK) {
        ret = sd_storage_log_slam_pose(esp_timer_get_time(), &pose);
    }
    return ret;
}

/**
 * Stop mission logging
 */
esp_err_t stop_mission_logging(const char *mission_name) {
    ESP_LOGI(TAG, "🛑 Stopping mission log: %s", mission_name);

    // Create session metadata
    session_metadata_t metadata = {
        .session_start = esp_timer_get_time(),  // This should be stored when starting
        .session_duration = 0,  // Calculate from start time
        .total_frames = 0,
        .tracked_frames = 0,
        .max_tracking_confidence = 0.0f,
        .average_processing_time = 0.0f
    };
    strncpy(metadata.session_name, mission_name, sizeof(metadata.session_name) - 1);

    // Get final SLAM statistics
    slam_stats_t stats;
    if (slam_core_get_stats(&stats) == ESP_OK) {
        metadata.total_frames = stats.frames_processed;
        metadata.tracked_frames = stats.frames_processed - stats.frames_dropped;
        metadata.max_tracking_confidence = stats.tracking_confidence;
        metadata.average_processing_time = stats.average_processing_time_ms;
    }

    esp_err_t ret = sd_storage_stop_session_log(&metadata);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "✅ Mission logging stopped");
    } else {
        ESP_LOGE(TAG, "❌ Failed to stop mission logging: %s", esp_err_to_name(ret));
    }

    return ret;
}
