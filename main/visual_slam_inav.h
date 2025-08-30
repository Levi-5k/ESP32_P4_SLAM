/*
 * Visual SLAM Navigation Module for INAV
 * ESP32-P4-WIFI6 based navigation system
 * 
 * Features:
 * - ORB-SLAM visual navigation
 * - GPS/IMU sensor fusion
 * - MSP communication with INAV
 * - SD card map storage
 * - Real-time position correction
 */

#pragma once

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#ifdef __cplusplus
extern "C" {
#endif

// System configuration
#define VISUAL_SLAM_VERSION "1.0.0"
#define MAX_ORB_FEATURES 500
#define MAP_UPDATE_RATE_HZ 10
#define MSP_UPDATE_RATE_HZ 50
#define GPS_UPDATE_RATE_HZ 5

// Position data structure
typedef struct {
    double latitude;          // Degrees
    double longitude;         // Degrees
    float altitude;           // Meters above sea level
    float relative_altitude;  // Meters above takeoff point
    uint64_t timestamp_us;    // Microseconds since boot
    uint8_t gps_fix_type;     // GPS fix quality (0=no fix, 3=3D fix, etc.)
    uint8_t satellites;       // Number of satellites
    float hdop;               // Horizontal dilution of precision
    float accuracy;           // Position accuracy in meters
} gps_position_t;

// IMU data structure
typedef struct {
    float accel_x, accel_y, accel_z;    // m/s²
    float gyro_x, gyro_y, gyro_z;       // rad/s
    float temp_c;                       // Temperature in Celsius
    uint64_t timestamp_us;              // Microseconds since boot
} imu_data_t;

// Visual SLAM pose
typedef struct {
    float x, y, z;                      // Position in meters (local frame)
    float qw, qx, qy, qz;              // Quaternion rotation
    float confidence;                   // Pose confidence (0.0-1.0)
    uint32_t tracked_features;          // Number of tracked ORB features
    uint64_t timestamp_us;              // Microseconds since boot
    bool is_lost;                       // True if tracking is lost
} slam_pose_t;

// Fused navigation state
typedef struct {
    // Position (WGS84)
    double latitude;
    double longitude;
    float altitude;
    
    // Local position (meters from origin)
    float local_x, local_y, local_z;
    
    // Velocity (m/s)
    float vel_x, vel_y, vel_z;
    
    // Attitude (quaternion)
    float qw, qx, qy, qz;
    
    // Quality indicators
    float position_accuracy;            // Meters
    float heading_accuracy;             // Degrees
    uint8_t navigation_status;          // 0=invalid, 1=GPS only, 2=SLAM only, 3=fused
    
    uint64_t timestamp_us;
} navigation_state_t;

// System events
typedef enum {
    VISUAL_SLAM_EVENT_GPS_FIX_ACQUIRED,
    VISUAL_SLAM_EVENT_GPS_FIX_LOST,
    VISUAL_SLAM_EVENT_SLAM_TRACKING_STARTED,
    VISUAL_SLAM_EVENT_SLAM_TRACKING_LOST,
    VISUAL_SLAM_EVENT_MAP_LOADED,
    VISUAL_SLAM_EVENT_MAP_SAVED,
    VISUAL_SLAM_EVENT_MSP_CONNECTED,
    VISUAL_SLAM_EVENT_MSP_DISCONNECTED,
    VISUAL_SLAM_EVENT_CALIBRATION_REQUIRED,
} visual_slam_event_t;

// Event callback function type
typedef void (*visual_slam_event_callback_t)(visual_slam_event_t event, void* data);

// System initialization
esp_err_t visual_slam_init(void);
esp_err_t visual_slam_start(void);
esp_err_t visual_slam_stop(void);
esp_err_t visual_slam_deinit(void);

// Event handling
esp_err_t visual_slam_register_event_callback(visual_slam_event_callback_t callback);

// Navigation data access
esp_err_t visual_slam_get_navigation_state(navigation_state_t* state);
esp_err_t visual_slam_get_gps_position(gps_position_t* position);
esp_err_t visual_slam_get_imu_data(imu_data_t* imu);
esp_err_t visual_slam_get_slam_pose(slam_pose_t* pose);

// Map management
esp_err_t visual_slam_save_map(const char* filename);
esp_err_t visual_slam_load_map(const char* filename);
esp_err_t visual_slam_clear_map(void);

// Calibration
esp_err_t visual_slam_calibrate_camera(void);
esp_err_t visual_slam_calibrate_imu(void);
esp_err_t visual_slam_set_gps_origin(double lat, double lon, float alt);

// Status and diagnostics
typedef struct {
    bool gps_active;
    bool imu_active;
    bool camera_active;
    bool slam_tracking;
    bool msp_connected;
    uint32_t orb_features_tracked;
    uint32_t map_points;
    float cpu_usage_percent;
    size_t free_heap_bytes;
    uint32_t frame_rate_fps;
} system_status_t;

esp_err_t visual_slam_get_system_status(system_status_t* status);

#ifdef __cplusplus
}
#endif
