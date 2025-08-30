/*
 * Sensor Fusion Component
 * Combines GPS, IMU, and Visual SLAM data using Extended Kalman Filter
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <esp_err.h>
#include <esp_timer.h>
#include "visual_slam_common_types.h"
// #include "../main/visual_slam_inav.h"  // Will create this header separately

#ifdef __cplusplus
extern "C" {
#endif

// Extended Kalman Filter state vector (15 states)
typedef struct {
    // Position (WGS84)
    double latitude;          // degrees
    double longitude;         // degrees
    float altitude;           // meters
    
    // Velocity (local NED frame)
    float vel_north;          // m/s
    float vel_east;           // m/s
    float vel_down;           // m/s
    
    // Attitude (quaternion)
    float qw, qx, qy, qz;     // unit quaternion
    
    // IMU biases
    float accel_bias_x;       // m/s²
    float accel_bias_y;       // m/s²
    float accel_bias_z;       // m/s²
    float gyro_bias_x;        // rad/s
    float gyro_bias_y;        // rad/s
    float gyro_bias_z;        // rad/s
} ekf_state_t;

// Covariance matrix for state uncertainty
typedef struct {
    float P[15][15];          // 15x15 covariance matrix
} ekf_covariance_t;

// Sensor availability status
typedef struct {
    bool gps_available;
    bool imu_available;
    bool slam_available;
    uint64_t last_gps_update_us;
    uint64_t last_imu_update_us;
    uint64_t last_slam_update_us;
} sensor_status_t;

// Fusion system statistics
typedef struct {
    uint32_t imu_updates;
    uint32_t gps_updates;
    uint32_t slam_updates;
    uint32_t prediction_steps;
    uint32_t correction_steps;
    float average_update_time_ms;
    float position_accuracy_estimate;
    float attitude_accuracy_estimate;
    navigation_state_t last_fused_state;
} fusion_stats_t;

// Initialize sensor fusion system
esp_err_t sensor_fusion_init(const fusion_config_t* config);
esp_err_t sensor_fusion_deinit(void);

// Initialize individual sensors
esp_err_t sensor_fusion_init_gps(const gps_config_t* config);
esp_err_t sensor_fusion_init_imu(const imu_config_t* config);

// Get current sensor fusion data
esp_err_t sensor_fusion_get_data(sensor_fusion_data_t* data);

// Set GPS reference origin
esp_err_t sensor_fusion_set_gps_origin(double lat, double lon, float alt);

// Update functions for each sensor
esp_err_t sensor_fusion_update_imu(const imu_data_t* imu_data);
esp_err_t sensor_fusion_update_gps(const gps_position_t* gps_data);
esp_err_t sensor_fusion_update_slam(const slam_pose_t* slam_pose);

// Get fused navigation state
esp_err_t sensor_fusion_get_state(navigation_state_t* nav_state);
esp_err_t sensor_fusion_get_ekf_state(ekf_state_t* ekf_state);
esp_err_t sensor_fusion_get_covariance(ekf_covariance_t* covariance);

// System status and diagnostics
esp_err_t sensor_fusion_get_sensor_status(sensor_status_t* status);
esp_err_t sensor_fusion_get_stats(fusion_stats_t* stats);

// Configuration management
esp_err_t sensor_fusion_set_config(const fusion_config_t* config);
esp_err_t sensor_fusion_get_config(fusion_config_t* config);

// Calibration and initialization
esp_err_t sensor_fusion_reset_state(void);
esp_err_t sensor_fusion_initialize_attitude(const imu_data_t* imu_data);
esp_err_t sensor_fusion_set_initial_position(double lat, double lon, float alt);

// Coordinate transformations
esp_err_t sensor_fusion_ned_to_gps(float north, float east, float down, 
                                 double* lat, double* lon, float* alt);
esp_err_t sensor_fusion_gps_to_ned(double lat, double lon, float alt,
                                 float* north, float* east, float* down);

// Advanced features
esp_err_t sensor_fusion_enable_adaptive_tuning(bool enable);
esp_err_t sensor_fusion_set_outlier_rejection_threshold(float threshold);

#ifdef __cplusplus
}
#endif
