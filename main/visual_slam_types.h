/**
 * Visual SLAM Navigation System - Common Types
 * Shared type definitions for the complete navigation system
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

// =============================================================================
// CAMERA SYSTEM TYPES
// =============================================================================

// Camera resolution options
typedef enum {
    CAMERA_RES_QVGA = 0,    // 320x240
    CAMERA_RES_VGA,         // 640x480  
    CAMERA_RES_SVGA,        // 800x600
    CAMERA_RES_XGA,         // 1024x768
    CAMERA_RES_640x480 = CAMERA_RES_VGA
} camera_resolution_t;

// Camera format options
typedef enum {
    CAMERA_FORMAT_RGB565 = 0,
    CAMERA_FORMAT_RGB888,
    CAMERA_FORMAT_YUV422,
    CAMERA_FORMAT_GRAYSCALE
} camera_format_t;

// Camera configuration structure
typedef struct {
    camera_resolution_t resolution;
    camera_format_t format;
    uint8_t fps;
    bool auto_exposure;
    bool auto_white_balance;
} camera_config_t;

// Camera frame buffer structure
typedef struct {
    uint8_t *data;      // Pointer to image data
    size_t size;        // Size of image data in bytes
    int width;          // Image width in pixels
    int height;         // Image height in pixels
    int format;         // Image format (camera_format_t)
    uint64_t timestamp; // Capture timestamp in microseconds
} camera_frame_t;

// =============================================================================
// SENSOR FUSION TYPES  
// =============================================================================

// 3D vector structure
typedef struct {
    float x, y, z;
} vector3_t;

// Quaternion structure for attitude representation
typedef struct {
    float w, x, y, z;
} quaternion_t;

// GPS configuration
typedef struct {
    int uart_port;
    int baud_rate;
    int tx_pin;
    int rx_pin;
} gps_config_t;

// GPS data structure
typedef struct {
    double latitude;        // Degrees
    double longitude;       // Degrees  
    float altitude;         // Meters above sea level
    float speed;           // Ground speed m/s
    float course;          // Course over ground degrees
    float hdop;            // Horizontal dilution of precision
    uint8_t satellites;    // Number of satellites in use
    bool fix_valid;        // GPS fix validity
    uint64_t timestamp;    // GPS timestamp
} gps_data_t;

// IMU data structure  
typedef struct {
    vector3_t acceleration; // m/s^2 in body frame
    vector3_t angular_rate; // rad/s in body frame
    vector3_t magnetic;     // Magnetic field in body frame
    float temperature;      // Sensor temperature °C
    uint64_t timestamp;     // IMU timestamp
} imu_data_t;

// Combined sensor fusion output
typedef struct {
    vector3_t position;     // Position in world frame (m)
    vector3_t velocity;     // Velocity in world frame (m/s)
    vector3_t attitude;     // Roll, Pitch, Yaw in radians
    quaternion_t quaternion; // Attitude as quaternion
    vector3_t angular_velocity; // Angular velocity (rad/s)
    float covariance[15][15];   // State covariance matrix
    bool slam_initialized;  // SLAM system status
    uint64_t timestamp;     // Fusion timestamp
} sensor_fusion_data_t;

// =============================================================================
// SLAM SYSTEM TYPES
// =============================================================================

// SLAM configuration
typedef struct {
    // ORB Feature Detection Parameters
    uint16_t max_features;              // Maximum number of features to extract (500)
    float fast_threshold;               // FAST corner detection threshold (20.0)
    uint8_t levels;                     // Number of pyramid levels (8)
    float scale_factor;                 // Scale factor between levels (1.2)
    uint16_t max_keypoints_per_level;   // Max keypoints per pyramid level (100)
    bool use_harris_detector;           // Use Harris corner detector (false)
    float harris_k;                     // Harris detector k parameter (0.04)
    uint16_t descriptor_distance_threshold; // Descriptor matching distance threshold (50)
    
    // SLAM Algorithm Parameters
    float match_threshold;              // Feature matching threshold (0.7)
    uint16_t min_tracked_features;      // Minimum features for tracking (30)
    float keyframe_distance_threshold;  // Keyframe insertion distance threshold (1.0)
    float keyframe_angle_threshold;     // Keyframe insertion angle threshold (0.2)
    uint16_t max_keyframes;             // Maximum number of keyframes (50)
    
    // Loop Closure and Optimization
    bool enable_loop_closure;           // Enable loop closure detection (true)
    float loop_closure_threshold;       // Loop closure detection threshold (0.8)
    
    // Legacy compatibility fields (maintained for backward compatibility)
    uint16_t orb_features;              // Number of ORB features to extract
    uint8_t keyframe_threshold;         // Frames between keyframes
    bool loop_closure_enabled;         // Enable loop closure detection
    bool map_optimization_enabled;     // Enable map optimization
} slam_config_t;

// SLAM pose structure
typedef struct {
    float x, y, z;          // Position components
    float qw, qx, qy, qz;   // Orientation quaternion
    
    // Legacy array formats for compatibility
    float position[3];      // Position array {x, y, z}
    float orientation[4];   // Quaternion array {w, x, y, z}
    
    uint64_t timestamp_us;  // Timestamp
    float confidence;       // Confidence [0-1]
    bool is_lost;          // Tracking lost flag
    uint32_t tracked_features;  // Number of tracked features
} slam_pose_t;

// SLAM processing result
typedef struct {
    vector3_t camera_position;      // Camera position in world frame
    quaternion_t camera_orientation; // Camera orientation  
    uint16_t tracked_features;      // Number of tracked features
    uint16_t map_points;           // Total map points
    bool tracking_lost;            // Tracking status
    float reprojection_error;      // Average reprojection error
    uint64_t timestamp;            // Processing timestamp
} slam_result_t;

// =============================================================================
// SYSTEM STATUS TYPES
// =============================================================================

// Overall system status
typedef struct {
    bool camera_initialized;
    bool gps_connected;
    bool imu_calibrated;
    bool slam_active;
    bool web_server_running;
    uint32_t frame_count;
    uint32_t feature_count;
    float position_x, position_y, position_z;
    float attitude_roll, attitude_pitch, attitude_yaw;
    uint32_t uptime_seconds;
} system_status_t;

#ifdef __cplusplus
}
#endif
