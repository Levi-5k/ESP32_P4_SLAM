/*
 * SLAM Core Component
 * Main visual SLAM processing and coordinate system management
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <esp_err.h>
#include <esp_timer.h>
#include "visual_slam_common_types.h"
// #include "esp_camera.h"  // Commented out - not available in ESP-IDF v5.5
// #include "../main/visual_slam_inav.h"  // Will create this header separately

#ifdef __cplusplus
extern "C" {
#endif

// Keyframe structure
typedef struct {
    uint32_t id;                        // Unique keyframe ID
    slam_pose_t pose;                   // Keyframe pose
    uint32_t num_features;              // Number of ORB features
    void* feature_descriptors;          // ORB descriptors (internal format)
    void* map_points;                   // Associated 3D map points
    uint64_t timestamp_us;              // Creation timestamp
} keyframe_t;

// Map point structure
typedef struct {
    uint32_t id;                        // Unique map point ID
    float x, y, z;                      // 3D position in world coordinates
    uint32_t observations;              // Number of keyframes observing this point
    float descriptor[32];               // ORB descriptor
    bool is_outlier;                    // True if point should be culled
} map_point_t;

// SLAM system state
typedef enum {
    SLAM_STATE_NOT_INITIALIZED,
    SLAM_STATE_INITIALIZING,
    SLAM_STATE_TRACKING,
    SLAM_STATE_LOST,
    SLAM_STATE_RELOCALIZATION
} slam_state_t;

// SLAM statistics
typedef struct {
    slam_state_t state;
    uint32_t current_features;
    uint32_t tracked_features;
    uint32_t map_points;
    uint32_t keyframes;
    float tracking_confidence;
    uint32_t frames_processed;
    uint32_t frames_dropped;
    float average_processing_time_ms;
    uint64_t last_update_us;
} slam_stats_t;

// Initialize SLAM core
esp_err_t slam_core_init(const slam_config_t* config);
esp_err_t slam_core_deinit(void);

// Camera initialization
esp_err_t slam_core_init_camera(const camera_config_t* config);

// Frame management  
esp_err_t slam_core_get_frame(camera_frame_t* frame);
esp_err_t slam_core_release_frame(camera_frame_t* frame);

// Main processing function - call for each camera frame
esp_err_t slam_core_process_frame(const camera_frame_t* frame, slam_pose_t* output_pose);

// Map management
esp_err_t slam_core_reset_map(void);
esp_err_t slam_core_save_map_to_file(const char* filename);
esp_err_t slam_core_load_map_from_file(const char* filename);

// Coordinate system management
esp_err_t slam_core_set_gps_origin(double lat, double lon, float alt);
esp_err_t slam_core_gps_to_local(double lat, double lon, float alt, float* local_x, float* local_y, float* local_z);
esp_err_t slam_core_local_to_gps(float local_x, float local_y, float local_z, double* lat, double* lon, float* alt);

// State and statistics
esp_err_t slam_core_get_stats(slam_stats_t* stats);
esp_err_t slam_core_get_current_pose(slam_pose_t* pose);

// Configuration
esp_err_t slam_core_set_config(const slam_config_t* config);
esp_err_t slam_core_get_config(slam_config_t* config);

// Camera configuration
esp_err_t slam_core_set_camera_config(const camera_config_t* config);
esp_err_t slam_core_get_camera_config(camera_config_t* config);

// Debugging and visualization
esp_err_t slam_core_get_keyframes(keyframe_t** keyframes, uint32_t* count);
esp_err_t slam_core_get_map_points(map_point_t** points, uint32_t* count);

#ifdef __cplusplus
}
#endif
