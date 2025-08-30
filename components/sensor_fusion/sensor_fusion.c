/*
 * Sensor Fusion Implementation
 * Extended Kalman Filter for GPS/IMU/SLAM fusion
 */

#include "sensor_fusion.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <math.h>
#include <string.h>

static const char* TAG = "sensor_fusion";

// Earth parameters
#define EARTH_RADIUS_M 6378137.0
#define GRAVITY_MS2 9.80665f

// Global fusion state
static struct {
    fusion_config_t config;
    ekf_state_t state;
    ekf_covariance_t covariance;
    sensor_status_t sensor_status;
    fusion_stats_t stats;
    navigation_state_t nav_state;  // Navigation state for output
    
    // GPS origin for NED coordinate system
    double origin_lat, origin_lon;
    float origin_alt;
    bool origin_set;
    
    // Synchronization
    SemaphoreHandle_t fusion_mutex;
    bool initialized;
    
    // Timing
    uint64_t last_prediction_time_us;
    
} g_fusion_state = {0};

// Default configuration
static const fusion_config_t default_config = {
    .position_noise = 0.1f,
    .velocity_noise = 0.01f,
    .attitude_noise = 0.001f,
    .accel_bias_noise = 0.0001f,
    .gyro_bias_noise = 0.00001f,
    .gps_position_noise = 1.0f,
    .gps_velocity_noise = 0.1f,
    .slam_position_noise = 0.05f,
    .slam_attitude_noise = 0.01f,
    .gps_weight = 0.7f,
    .slam_weight = 0.3f,
    .imu_weight = 1.0f,
    .imu_dt = 0.01f,  // 100Hz IMU
    .max_prediction_time = 0.1f
};

// Forward declarations
static void initialize_state_and_covariance(void);
static esp_err_t predict_state(float dt, const imu_data_t* imu_data);
static esp_err_t correct_with_gps(const gps_position_t* gps_data);
static esp_err_t correct_with_slam(const slam_pose_t* slam_pose);
static void quaternion_multiply(const float* q1, const float* q2, float* result);
static void quaternion_to_rotation_matrix(const float* q, float R[3][3]);
static void propagate_covariance(float dt, const imu_data_t* imu_data);

esp_err_t sensor_fusion_init(const fusion_config_t* config) {
    if (g_fusion_state.initialized) {
        ESP_LOGW(TAG, "Sensor fusion already initialized");
        return ESP_OK;
    }
    
    // Set configuration
    if (config) {
        g_fusion_state.config = *config;
    } else {
        g_fusion_state.config = default_config;
    }
    
    // Create mutex
    g_fusion_state.fusion_mutex = xSemaphoreCreateMutex();
    if (!g_fusion_state.fusion_mutex) {
        ESP_LOGE(TAG, "Failed to create fusion mutex");
        return ESP_ERR_NO_MEM;
    }
    
    // Initialize state and covariance
    initialize_state_and_covariance();
    
    // Initialize sensor status
    memset(&g_fusion_state.sensor_status, 0, sizeof(sensor_status_t));
    
    // Initialize statistics
    memset(&g_fusion_state.stats, 0, sizeof(fusion_stats_t));
    
    g_fusion_state.origin_set = false;
    g_fusion_state.last_prediction_time_us = esp_timer_get_time();
    g_fusion_state.initialized = true;
    
    ESP_LOGI(TAG, "Sensor fusion initialized");
    return ESP_OK;
}

esp_err_t sensor_fusion_deinit(void) {
    if (!g_fusion_state.initialized) {
        return ESP_OK;
    }
    
    xSemaphoreTake(g_fusion_state.fusion_mutex, portMAX_DELAY);
    g_fusion_state.initialized = false;
    xSemaphoreGive(g_fusion_state.fusion_mutex);
    vSemaphoreDelete(g_fusion_state.fusion_mutex);
    
    ESP_LOGI(TAG, "Sensor fusion deinitialized");
    return ESP_OK;
}

esp_err_t sensor_fusion_set_gps_origin(double lat, double lon, float alt) {
    if (!g_fusion_state.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    xSemaphoreTake(g_fusion_state.fusion_mutex, portMAX_DELAY);
    
    g_fusion_state.origin_lat = lat;
    g_fusion_state.origin_lon = lon;
    g_fusion_state.origin_alt = alt;
    g_fusion_state.origin_set = true;
    
    // Initialize state position to origin
    g_fusion_state.state.latitude = lat;
    g_fusion_state.state.longitude = lon;
    g_fusion_state.state.altitude = alt;
    
    xSemaphoreGive(g_fusion_state.fusion_mutex);
    
    ESP_LOGI(TAG, "GPS origin set to %.8f, %.8f, %.2fm", lat, lon, alt);
    return ESP_OK;
}

esp_err_t sensor_fusion_update_imu(const imu_data_t* imu_data) {
    if (!g_fusion_state.initialized || !imu_data) {
        return ESP_ERR_INVALID_ARG;
    }
    
    xSemaphoreTake(g_fusion_state.fusion_mutex, portMAX_DELAY);
    
    uint64_t current_time = esp_timer_get_time();
    float dt = (current_time - g_fusion_state.last_prediction_time_us) / 1000000.0f;
    
    // Limit prediction time step
    if (dt > g_fusion_state.config.max_prediction_time) {
        dt = g_fusion_state.config.max_prediction_time;
    }
    
    if (dt > 0.001f) {  // Only predict if enough time has passed
        esp_err_t ret = predict_state(dt, imu_data);
        if (ret == ESP_OK) {
            g_fusion_state.last_prediction_time_us = current_time;
            g_fusion_state.stats.prediction_steps++;
            g_fusion_state.stats.imu_updates++;
            g_fusion_state.sensor_status.last_imu_update_us = current_time;
            g_fusion_state.sensor_status.imu_available = true;
        }
    }
    
    xSemaphoreGive(g_fusion_state.fusion_mutex);
    return ESP_OK;
}

esp_err_t sensor_fusion_update_gps(const gps_position_t* gps_data) {
    if (!g_fusion_state.initialized || !gps_data) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Only process GPS data if we have a good fix
    if (gps_data->gps_fix_type < 3 || gps_data->satellites < 6) {
        return ESP_OK;
    }
    
    xSemaphoreTake(g_fusion_state.fusion_mutex, portMAX_DELAY);
    
    esp_err_t ret = correct_with_gps(gps_data);
    if (ret == ESP_OK) {
        uint64_t current_time = esp_timer_get_time();
        g_fusion_state.stats.correction_steps++;
        g_fusion_state.stats.gps_updates++;
        g_fusion_state.sensor_status.last_gps_update_us = current_time;
        g_fusion_state.sensor_status.gps_available = true;
    }
    
    xSemaphoreGive(g_fusion_state.fusion_mutex);
    return ret;
}

esp_err_t sensor_fusion_update_slam(const slam_pose_t* slam_pose) {
    if (!g_fusion_state.initialized || !slam_pose) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Only process SLAM data if tracking is good
    if (slam_pose->is_lost || slam_pose->confidence < 0.5f) {
        return ESP_OK;
    }
    
    xSemaphoreTake(g_fusion_state.fusion_mutex, portMAX_DELAY);
    
    esp_err_t ret = correct_with_slam(slam_pose);
    if (ret == ESP_OK) {
        uint64_t current_time = esp_timer_get_time();
        g_fusion_state.stats.correction_steps++;
        g_fusion_state.stats.slam_updates++;
        g_fusion_state.sensor_status.last_slam_update_us = current_time;
        g_fusion_state.sensor_status.slam_available = true;
    }
    
    xSemaphoreGive(g_fusion_state.fusion_mutex);
    return ret;
}

esp_err_t sensor_fusion_get_state(navigation_state_t* nav_state) {
    if (!g_fusion_state.initialized || !nav_state) {
        return ESP_ERR_INVALID_ARG;
    }
    
    xSemaphoreTake(g_fusion_state.fusion_mutex, portMAX_DELAY);
    
    // Convert EKF state to navigation state
    nav_state->latitude = g_fusion_state.state.latitude;
    nav_state->longitude = g_fusion_state.state.longitude;
    nav_state->altitude = g_fusion_state.state.altitude;
    
    // Convert velocity from NED to local frame
    nav_state->vel_x = g_fusion_state.state.vel_east;   // East = X
    nav_state->vel_y = g_fusion_state.state.vel_north;  // North = Y
    nav_state->vel_z = -g_fusion_state.state.vel_down;  // Up = -Down
    
    // Copy attitude quaternion
    nav_state->qw = g_fusion_state.state.qw;
    nav_state->qx = g_fusion_state.state.qx;
    nav_state->qy = g_fusion_state.state.qy;
    nav_state->qz = g_fusion_state.state.qz;
    
    // Convert to local coordinates if origin is set
    if (g_fusion_state.origin_set) {
        sensor_fusion_gps_to_ned(nav_state->latitude, nav_state->longitude, nav_state->altitude,
                               &nav_state->local_y, &nav_state->local_x, &nav_state->local_z);
        nav_state->local_z = -nav_state->local_z;  // Convert down to up
    }
    
    // Set quality indicators
    nav_state->position_accuracy = sqrtf(g_fusion_state.covariance.P[0][0] + 
                                       g_fusion_state.covariance.P[1][1] + 
                                       g_fusion_state.covariance.P[2][2]);
    
    nav_state->heading_accuracy = sqrtf(g_fusion_state.covariance.P[6][6]) * 180.0f / M_PI;
    
    // Determine navigation status
    bool gps_recent = (esp_timer_get_time() - g_fusion_state.sensor_status.last_gps_update_us) < 2000000;  // 2 seconds
    bool slam_recent = (esp_timer_get_time() - g_fusion_state.sensor_status.last_slam_update_us) < 1000000;  // 1 second
    
    if (gps_recent && slam_recent) {
        nav_state->navigation_status = 3;  // Fused
    } else if (gps_recent) {
        nav_state->navigation_status = 1;  // GPS only
    } else if (slam_recent) {
        nav_state->navigation_status = 2;  // SLAM only
    } else {
        nav_state->navigation_status = 0;  // Invalid
    }
    
    nav_state->timestamp_us = esp_timer_get_time();
    
    xSemaphoreGive(g_fusion_state.fusion_mutex);
    return ESP_OK;
}

esp_err_t sensor_fusion_gps_to_ned(double lat, double lon, float alt,
                                 float* north, float* east, float* down) {
    if (!g_fusion_state.initialized || !g_fusion_state.origin_set) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Convert GPS to NED using equirectangular projection
    double lat_rad = lat * M_PI / 180.0;
    double lon_rad = lon * M_PI / 180.0;
    double origin_lat_rad = g_fusion_state.origin_lat * M_PI / 180.0;
    double origin_lon_rad = g_fusion_state.origin_lon * M_PI / 180.0;
    
    double cos_lat = cos((lat_rad + origin_lat_rad) / 2.0);
    
    *north = (float)((lat_rad - origin_lat_rad) * EARTH_RADIUS_M);
    *east = (float)((lon_rad - origin_lon_rad) * EARTH_RADIUS_M * cos_lat);
    *down = g_fusion_state.origin_alt - alt;
    
    return ESP_OK;
}

esp_err_t sensor_fusion_ned_to_gps(float north, float east, float down,
                                 double* lat, double* lon, float* alt) {
    if (!g_fusion_state.initialized || !g_fusion_state.origin_set) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Convert NED to GPS
    double origin_lat_rad = g_fusion_state.origin_lat * M_PI / 180.0;
    double origin_lon_rad = g_fusion_state.origin_lon * M_PI / 180.0;
    
    double lat_rad = origin_lat_rad + (north / EARTH_RADIUS_M);
    double cos_lat = cos((lat_rad + origin_lat_rad) / 2.0);
    double lon_rad = origin_lon_rad + (east / (EARTH_RADIUS_M * cos_lat));
    
    *lat = lat_rad * 180.0 / M_PI;
    *lon = lon_rad * 180.0 / M_PI;
    *alt = g_fusion_state.origin_alt - down;
    
    return ESP_OK;
}

// Private helper functions

static void initialize_state_and_covariance(void) {
    // Initialize state to zero (will be set when first GPS data arrives)
    memset(&g_fusion_state.state, 0, sizeof(ekf_state_t));
    g_fusion_state.state.qw = 1.0f;  // Identity quaternion
    
    // Initialize covariance matrix
    memset(&g_fusion_state.covariance, 0, sizeof(ekf_covariance_t));
    
    // Set initial uncertainties (diagonal elements)
    g_fusion_state.covariance.P[0][0] = 100.0f;   // Latitude variance
    g_fusion_state.covariance.P[1][1] = 100.0f;   // Longitude variance
    g_fusion_state.covariance.P[2][2] = 25.0f;    // Altitude variance
    g_fusion_state.covariance.P[3][3] = 1.0f;     // North velocity variance
    g_fusion_state.covariance.P[4][4] = 1.0f;     // East velocity variance
    g_fusion_state.covariance.P[5][5] = 1.0f;     // Down velocity variance
    g_fusion_state.covariance.P[6][6] = 0.1f;     // Quaternion w variance
    g_fusion_state.covariance.P[7][7] = 0.1f;     // Quaternion x variance
    g_fusion_state.covariance.P[8][8] = 0.1f;     // Quaternion y variance
    g_fusion_state.covariance.P[9][9] = 0.1f;     // Quaternion z variance
    g_fusion_state.covariance.P[10][10] = 0.01f;  // Accel bias x variance
    g_fusion_state.covariance.P[11][11] = 0.01f;  // Accel bias y variance
    g_fusion_state.covariance.P[12][12] = 0.01f;  // Accel bias z variance
    g_fusion_state.covariance.P[13][13] = 0.001f; // Gyro bias x variance
    g_fusion_state.covariance.P[14][14] = 0.001f; // Gyro bias y variance
}

static esp_err_t predict_state(float dt, const imu_data_t* imu_data) {
    // Remove biases from IMU measurements
    float accel_x = imu_data->accel_x - g_fusion_state.state.accel_bias_x;
    float accel_y = imu_data->accel_y - g_fusion_state.state.accel_bias_y;
    float accel_z = imu_data->accel_z - g_fusion_state.state.accel_bias_z;
    
    float gyro_x = imu_data->gyro_x - g_fusion_state.state.gyro_bias_x;
    float gyro_y = imu_data->gyro_y - g_fusion_state.state.gyro_bias_y;
    float gyro_z = imu_data->gyro_z - g_fusion_state.state.gyro_bias_z;
    
    // Convert body accelerations to NED frame
    float R[3][3];
    float q[4] = {g_fusion_state.state.qw, g_fusion_state.state.qx, 
                  g_fusion_state.state.qy, g_fusion_state.state.qz};
    quaternion_to_rotation_matrix(q, R);
    
    float accel_ned[3];
    accel_ned[0] = R[0][0] * accel_x + R[0][1] * accel_y + R[0][2] * accel_z;
    accel_ned[1] = R[1][0] * accel_x + R[1][1] * accel_y + R[1][2] * accel_z;
    accel_ned[2] = R[2][0] * accel_x + R[2][1] * accel_y + R[2][2] * accel_z + GRAVITY_MS2;
    
    // Predict position and velocity
    float dt2 = dt * dt / 2.0f;
    
    // Update position using kinematic equations: pos = pos + vel*dt + 0.5*accel*dt^2
    g_fusion_state.state.altitude += g_fusion_state.state.vel_down * dt + 0.5f * accel_ned[2] * dt2;
    
    // Convert velocity to lat/lon rates
    double lat_rad = g_fusion_state.state.latitude * M_PI / 180.0;
    double cos_lat = cos(lat_rad);
    
    double dlat_dt = g_fusion_state.state.vel_north / EARTH_RADIUS_M;
    double dlon_dt = g_fusion_state.state.vel_east / (EARTH_RADIUS_M * cos_lat);
    
    g_fusion_state.state.latitude += dlat_dt * dt * 180.0 / M_PI;
    g_fusion_state.state.longitude += dlon_dt * dt * 180.0 / M_PI;
    g_fusion_state.state.altitude -= g_fusion_state.state.vel_down * dt;  // Down is positive
    
    // Update velocities
    g_fusion_state.state.vel_north += accel_ned[0] * dt;
    g_fusion_state.state.vel_east += accel_ned[1] * dt;
    g_fusion_state.state.vel_down += accel_ned[2] * dt;
    
    // Update attitude using quaternion kinematics
    float omega_norm = sqrtf(gyro_x*gyro_x + gyro_y*gyro_y + gyro_z*gyro_z);
    if (omega_norm > 1e-6f) {
        float sin_half = sinf(omega_norm * dt / 2.0f);
        float cos_half = cosf(omega_norm * dt / 2.0f);
        
        float dq[4] = {
            cos_half,
            sin_half * gyro_x / omega_norm,
            sin_half * gyro_y / omega_norm,
            sin_half * gyro_z / omega_norm
        };
        
        float q_new[4];
        quaternion_multiply(q, dq, q_new);
        
        g_fusion_state.state.qw = q_new[0];
        g_fusion_state.state.qx = q_new[1];
        g_fusion_state.state.qy = q_new[2];
        g_fusion_state.state.qz = q_new[3];
        
        // Normalize quaternion
        float q_norm = sqrtf(q_new[0]*q_new[0] + q_new[1]*q_new[1] + 
                           q_new[2]*q_new[2] + q_new[3]*q_new[3]);
        g_fusion_state.state.qw /= q_norm;
        g_fusion_state.state.qx /= q_norm;
        g_fusion_state.state.qy /= q_norm;
        g_fusion_state.state.qz /= q_norm;
    }
    
    // Biases remain constant (random walk model)
    // Covariance propagation would be implemented here
    propagate_covariance(dt, imu_data);
    
    return ESP_OK;
}

static esp_err_t correct_with_gps(const gps_position_t* gps_data) {
    // Simple correction for demonstration - in practice this would be full EKF update
    float gps_weight = g_fusion_state.config.gps_weight;
    
    // Update position with GPS
    g_fusion_state.state.latitude = (1.0f - gps_weight) * g_fusion_state.state.latitude + 
                                   gps_weight * gps_data->latitude;
    g_fusion_state.state.longitude = (1.0f - gps_weight) * g_fusion_state.state.longitude + 
                                    gps_weight * gps_data->longitude;
    g_fusion_state.state.altitude = (1.0f - gps_weight) * g_fusion_state.state.altitude + 
                                   gps_weight * gps_data->altitude;
    
    // Reduce position uncertainty in covariance matrix
    g_fusion_state.covariance.P[0][0] *= (1.0f - gps_weight);
    g_fusion_state.covariance.P[1][1] *= (1.0f - gps_weight);
    g_fusion_state.covariance.P[2][2] *= (1.0f - gps_weight);
    
    return ESP_OK;
}

static esp_err_t correct_with_slam(const slam_pose_t* slam_pose) {
    // Simple correction for demonstration
    float slam_weight = g_fusion_state.config.slam_weight * slam_pose->confidence;
    
    // Convert SLAM pose to GPS coordinates if origin is set
    if (g_fusion_state.origin_set) {
        double slam_lat, slam_lon;
        float slam_alt;
        
        esp_err_t ret = sensor_fusion_ned_to_gps(slam_pose->y, slam_pose->x, -slam_pose->z,
                                               &slam_lat, &slam_lon, &slam_alt);
        if (ret == ESP_OK) {
            // Blend SLAM position with current estimate
            g_fusion_state.state.latitude = (1.0f - slam_weight) * g_fusion_state.state.latitude + 
                                          slam_weight * slam_lat;
            g_fusion_state.state.longitude = (1.0f - slam_weight) * g_fusion_state.state.longitude + 
                                           slam_weight * slam_lon;
        }
    }
    
    // Blend SLAM attitude with current estimate
    g_fusion_state.state.qw = (1.0f - slam_weight) * g_fusion_state.state.qw + 
                             slam_weight * slam_pose->qw;
    g_fusion_state.state.qx = (1.0f - slam_weight) * g_fusion_state.state.qx + 
                             slam_weight * slam_pose->qx;
    g_fusion_state.state.qy = (1.0f - slam_weight) * g_fusion_state.state.qy + 
                             slam_weight * slam_pose->qy;
    g_fusion_state.state.qz = (1.0f - slam_weight) * g_fusion_state.state.qz + 
                             slam_weight * slam_pose->qz;
    
    // Normalize quaternion
    float q_norm = sqrtf(g_fusion_state.state.qw * g_fusion_state.state.qw + 
                        g_fusion_state.state.qx * g_fusion_state.state.qx + 
                        g_fusion_state.state.qy * g_fusion_state.state.qy + 
                        g_fusion_state.state.qz * g_fusion_state.state.qz);
    g_fusion_state.state.qw /= q_norm;
    g_fusion_state.state.qx /= q_norm;
    g_fusion_state.state.qy /= q_norm;
    g_fusion_state.state.qz /= q_norm;
    
    // Reduce attitude uncertainty
    float attitude_reduction = slam_weight * 0.5f;
    g_fusion_state.covariance.P[6][6] *= (1.0f - attitude_reduction);
    g_fusion_state.covariance.P[7][7] *= (1.0f - attitude_reduction);
    g_fusion_state.covariance.P[8][8] *= (1.0f - attitude_reduction);
    g_fusion_state.covariance.P[9][9] *= (1.0f - attitude_reduction);
    
    return ESP_OK;
}

static void quaternion_multiply(const float* q1, const float* q2, float* result) {
    result[0] = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3];
    result[1] = q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2];
    result[2] = q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1];
    result[3] = q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0];
}

static void quaternion_to_rotation_matrix(const float* q, float R[3][3]) {
    float qw = q[0], qx = q[1], qy = q[2], qz = q[3];
    
    R[0][0] = 1 - 2*(qy*qy + qz*qz);
    R[0][1] = 2*(qx*qy - qw*qz);
    R[0][2] = 2*(qx*qz + qw*qy);
    
    R[1][0] = 2*(qx*qy + qw*qz);
    R[1][1] = 1 - 2*(qx*qx + qz*qz);
    R[1][2] = 2*(qy*qz - qw*qx);
    
    R[2][0] = 2*(qx*qz - qw*qy);
    R[2][1] = 2*(qy*qz + qw*qx);
    R[2][2] = 1 - 2*(qx*qx + qy*qy);
}

static void propagate_covariance(float dt, const imu_data_t* imu_data) {
    // Simplified covariance propagation
    // In a full implementation, this would compute the Jacobian matrices
    // and propagate the full 15x15 covariance matrix
    
    float dt2 = dt * dt;
    
    // Add process noise to diagonal elements
    g_fusion_state.covariance.P[0][0] += g_fusion_state.config.position_noise * dt2;
    g_fusion_state.covariance.P[1][1] += g_fusion_state.config.position_noise * dt2;
    g_fusion_state.covariance.P[2][2] += g_fusion_state.config.position_noise * dt2;
    
    g_fusion_state.covariance.P[3][3] += g_fusion_state.config.velocity_noise * dt;
    g_fusion_state.covariance.P[4][4] += g_fusion_state.config.velocity_noise * dt;
    g_fusion_state.covariance.P[5][5] += g_fusion_state.config.velocity_noise * dt;
    
    g_fusion_state.covariance.P[6][6] += g_fusion_state.config.attitude_noise * dt;
    g_fusion_state.covariance.P[7][7] += g_fusion_state.config.attitude_noise * dt;
    g_fusion_state.covariance.P[8][8] += g_fusion_state.config.attitude_noise * dt;
    g_fusion_state.covariance.P[9][9] += g_fusion_state.config.attitude_noise * dt;
    
    g_fusion_state.covariance.P[10][10] += g_fusion_state.config.accel_bias_noise * dt;
    g_fusion_state.covariance.P[11][11] += g_fusion_state.config.accel_bias_noise * dt;
    g_fusion_state.covariance.P[12][12] += g_fusion_state.config.accel_bias_noise * dt;
    
    g_fusion_state.covariance.P[13][13] += g_fusion_state.config.gyro_bias_noise * dt;
    g_fusion_state.covariance.P[14][14] += g_fusion_state.config.gyro_bias_noise * dt;
}

esp_err_t sensor_fusion_get_data(sensor_fusion_data_t* data) {
    if (!data) return ESP_ERR_INVALID_ARG;
    
    if (!g_fusion_state.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Fill compatibility fields from navigation state
    data->position.x = g_fusion_state.nav_state.latitude;
    data->position.y = g_fusion_state.nav_state.longitude;
    data->position.z = g_fusion_state.nav_state.altitude;
    
    data->attitude.roll = g_fusion_state.nav_state.attitude.x;
    data->attitude.pitch = g_fusion_state.nav_state.attitude.y;
    data->attitude.yaw = g_fusion_state.nav_state.attitude.z;
    
    return ESP_OK;
}
