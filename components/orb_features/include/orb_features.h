/*
 * ORB Features Detection and Description
 * Fast corner detection with rotation invariant descriptors
 */

#pragma once

#include <stdbool.h>
#include "esp_err.h"
#include "visual_slam_common_types.h"
// #include "esp_camera.h"  // Commented out - not available in ESP-IDF v5.5

#ifdef __cplusplus
extern "C" {
#endif

// Maximum number of features that can be detected
#define MAX_ORB_FEATURES 1000

// ORB feature point
typedef struct {
    uint16_t x, y;                      // Pixel coordinates
    float angle;                        // Orientation angle in radians
    uint8_t response;                   // Corner response strength
    uint8_t octave;                     // Pyramid level where detected
    uint8_t descriptor[32];             // 256-bit ORB descriptor
} orb_feature_point_t;

// Collection of ORB features from one frame
typedef struct {
    orb_feature_point_t* features;      // Array of feature points
    uint32_t num_features;              // Number of detected features
    uint32_t count;                     // Alias for num_features for compatibility
    uint32_t max_features;              // Maximum capacity
    uint64_t timestamp_us;              // Frame timestamp
    uint16_t frame_width, frame_height; // Frame dimensions
    
    // Feature matching results (with previous frame)
    uint32_t num_matches;               // Total number of matches
    uint32_t num_good_matches;          // Number of good quality matches
} orb_features_t;

// Feature matching
typedef struct {
    uint16_t query_idx;                 // Index in query features
    uint16_t train_idx;                 // Index in train features
    float distance;                     // Hamming distance (0-256)
    bool is_good_match;                 // True if distance < threshold
} orb_match_t;

typedef struct {
    orb_match_t* matches;               // Array of matches
    uint32_t num_matches;               // Number of matches found
    uint32_t num_good_matches;          // Number of good matches
    float average_distance;             // Average matching distance
} orb_matches_t;

// Initialize ORB feature detector
esp_err_t orb_features_init(uint32_t max_features);
esp_err_t orb_features_deinit(void);

// Set detector configuration
esp_err_t orb_set_config(const orb_config_t* config);
esp_err_t orb_get_config(orb_config_t* config);

// Feature detection and description
esp_err_t orb_extract_features(const camera_frame_t* frame, orb_features_t* features);
esp_err_t orb_features_extract(const camera_frame_t* frame, orb_features_t* features);  // Alias for compatibility

// Feature matching
esp_err_t orb_match_features(const orb_features_t* query_features, 
                            const orb_features_t* train_features,
                            orb_matches_t* matches,
                            float distance_threshold);

// Memory management for features
esp_err_t orb_alloc_features(orb_features_t* features, uint32_t max_features);
void orb_free_features(orb_features_t* features);

// Utility functions
uint32_t orb_hamming_distance(const uint8_t* desc1, const uint8_t* desc2);
esp_err_t orb_draw_features(camera_frame_t* frame, const orb_features_t* features);
esp_err_t orb_draw_matches(camera_frame_t* frame1, const orb_features_t* features1,
                          camera_frame_t* frame2, const orb_features_t* features2,
                          const orb_matches_t* matches);

// Performance monitoring
typedef struct {
    uint32_t frames_processed;
    float average_detection_time_ms;
    float average_description_time_ms;
    uint32_t total_features_detected;
    uint32_t average_features_per_frame;
} orb_performance_stats_t;

esp_err_t orb_get_performance_stats(orb_performance_stats_t* stats);
esp_err_t orb_reset_performance_stats(void);

#ifdef __cplusplus
}
#endif
