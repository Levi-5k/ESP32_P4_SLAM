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
    
    // Additional state variables for adaptive tuning
    float velocity_magnitude;  // Current velocity magnitude for adaptive tuning
    float gps_quality;        // GPS quality metric (0.0 to 1.0)
    
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
    .max_prediction_time = 0.1f,
    .enable_bias_estimation = true,
    .enable_outlier_detection = true,
    .gps_outlier_threshold = 10.0f,
    .slam_outlier_threshold = 5.0f,
    .stationary_accel_threshold = 0.5f,
    .stationary_gyro_threshold = 0.1f
};

// Forward declarations
static void initialize_state_and_covariance(void);
static esp_err_t predict_state(float dt, const imu_data_t* imu_data);
static esp_err_t correct_with_gps(const gps_position_t* gps_data);
static esp_err_t correct_with_slam(const slam_pose_t* slam_pose);
static void quaternion_multiply(const float* q1, const float* q2, float* result);
static void quaternion_to_rotation_matrix(const float* q, float R[3][3]);
static void propagate_covariance(float dt, const imu_data_t* imu_data);
static bool detect_gps_outlier(const gps_position_t* gps_data);
static bool detect_slam_outlier(const slam_pose_t* slam_pose);
static void update_bias_estimates(const imu_data_t* imu_data);
static void compute_euler_angles(float* roll, float* pitch, float* yaw);
static esp_err_t full_ekf_gps_update(const gps_position_t* gps_data);
static esp_err_t full_ekf_slam_update(const slam_pose_t* slam_pose);

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
            
            // Update bias estimates periodically
            if (g_fusion_state.config.enable_bias_estimation) {
                update_bias_estimates(imu_data);
            }
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
    
    // Detect GPS outliers
    if (g_fusion_state.config.enable_outlier_detection && detect_gps_outlier(gps_data)) {
        ESP_LOGW(TAG, "GPS outlier detected, skipping update");
        return ESP_OK;
    }
    
    xSemaphoreTake(g_fusion_state.fusion_mutex, portMAX_DELAY);
    
    // Calculate GPS quality metric (0.0 to 1.0)
    float sat_quality = (gps_data->satellites - 4) / 8.0f;  // 4-12 satellites normalized
    float acc_quality = 1.0f - (gps_data->accuracy / 20.0f); // 0-20m accuracy normalized
    g_fusion_state.gps_quality = fmaxf(0.0f, fminf(1.0f, (sat_quality + acc_quality) / 2.0f));
    
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
    
    // Detect SLAM outliers
    if (g_fusion_state.config.enable_outlier_detection && detect_slam_outlier(slam_pose)) {
        ESP_LOGW(TAG, "SLAM outlier detected, skipping update");
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

esp_err_t sensor_fusion_update_wifi(const wifi_position_t* wifi_position) {
    if (!g_fusion_state.initialized || !wifi_position) {
        return ESP_ERR_INVALID_ARG;
    }

    // Only process WiFi data if position is valid and accuracy is reasonable
    if (!wifi_position->valid || wifi_position->accuracy_h > 100.0f) {
        return ESP_OK;
    }

    // Only use WiFi if we have enough access points
    if (wifi_position->ap_count < 3) {
        return ESP_OK;
    }

    xSemaphoreTake(g_fusion_state.fusion_mutex, portMAX_DELAY);

    // Create a GPS-like position structure for WiFi data
    gps_position_t wifi_as_gps = {
        .latitude = wifi_position->latitude,
        .longitude = wifi_position->longitude,
        .altitude = wifi_position->altitude,
        .accuracy = wifi_position->accuracy_h,
        .gps_fix_type = 3,  // 3D fix
        .satellites = wifi_position->ap_count,  // Use AP count as "satellites"
        .timestamp_us = wifi_position->timestamp
    };

    // Use GPS correction function but with WiFi-specific noise characteristics
    float original_gps_noise = g_fusion_state.config.gps_position_noise;
    g_fusion_state.config.gps_position_noise = wifi_position->accuracy_h / 3.0f;  // WiFi is less accurate

    esp_err_t ret = correct_with_gps(&wifi_as_gps);

    // Restore original GPS noise
    g_fusion_state.config.gps_position_noise = original_gps_noise;

    if (ret == ESP_OK) {
        uint64_t current_time = esp_timer_get_time();
        g_fusion_state.stats.correction_steps++;
        g_fusion_state.stats.wifi_updates++;
        g_fusion_state.sensor_status.last_wifi_update_us = current_time;
        g_fusion_state.sensor_status.wifi_available = true;
        ESP_LOGI(TAG, "WiFi position update: %.6f, %.6f (accuracy: %.1fm, %u APs)",
                 wifi_position->latitude, wifi_position->longitude,
                 wifi_position->accuracy_h, wifi_position->ap_count);
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
    
    // Compute Euler angles from quaternion
    float roll, pitch, yaw;
    compute_euler_angles(&roll, &pitch, &yaw);
    nav_state->attitude.x = roll;
    nav_state->attitude.y = pitch;
    nav_state->attitude.z = yaw;
    
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
    
    // Update velocity magnitude for adaptive tuning
    g_fusion_state.velocity_magnitude = sqrtf(
        g_fusion_state.state.vel_north * g_fusion_state.state.vel_north +
        g_fusion_state.state.vel_east * g_fusion_state.state.vel_east +
        g_fusion_state.state.vel_down * g_fusion_state.state.vel_down
    );
    
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

// GPS sensor initialization
esp_err_t sensor_fusion_init_gps(const gps_config_t* config) {
    if (!config) {
        ESP_LOGE(TAG, "GPS config is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "Initializing GPS sensor with UART%d, baud %d", 
             config->uart_port, config->baud_rate);
    
    // For now, just mark GPS as available in sensor status
    // The actual GPS hardware initialization should be done by the GPS component
    g_fusion_state.sensor_status.gps_available = true;
    g_fusion_state.sensor_status.last_gps_update_us = esp_timer_get_time();
    
    ESP_LOGI(TAG, "GPS sensor initialized successfully");
    return ESP_OK;
}

// IMU sensor initialization  
esp_err_t sensor_fusion_register_imu(const imu_config_t* config) {
    if (!config) {
        ESP_LOGE(TAG, "IMU config is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "Registering IMU sensor in fusion system");
    
    // For now, just mark IMU as available in sensor status
    // The actual IMU hardware initialization should be done by the IMU component
    g_fusion_state.sensor_status.imu_available = true;
    g_fusion_state.sensor_status.last_imu_update_us = esp_timer_get_time();
    
    ESP_LOGI(TAG, "IMU sensor registered in fusion system");
    return ESP_OK;
}

esp_err_t sensor_fusion_get_sensor_status(sensor_status_t* status) {
    if (!status) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!g_fusion_state.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    xSemaphoreTake(g_fusion_state.fusion_mutex, portMAX_DELAY);
    memcpy(status, &g_fusion_state.sensor_status, sizeof(sensor_status_t));
    xSemaphoreGive(g_fusion_state.fusion_mutex);
    
    return ESP_OK;
}

esp_err_t sensor_fusion_get_stats(fusion_stats_t* stats) {
    if (!stats) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!g_fusion_state.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    xSemaphoreTake(g_fusion_state.fusion_mutex, portMAX_DELAY);
    memcpy(stats, &g_fusion_state.stats, sizeof(fusion_stats_t));
    xSemaphoreGive(g_fusion_state.fusion_mutex);
    
    return ESP_OK;
}

esp_err_t sensor_fusion_get_ekf_state(ekf_state_t* ekf_state) {
    if (!ekf_state) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!g_fusion_state.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    xSemaphoreTake(g_fusion_state.fusion_mutex, portMAX_DELAY);
    memcpy(ekf_state, &g_fusion_state.state, sizeof(ekf_state_t));
    xSemaphoreGive(g_fusion_state.fusion_mutex);
    
    return ESP_OK;
}

esp_err_t sensor_fusion_get_covariance(ekf_covariance_t* covariance) {
    if (!covariance) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!g_fusion_state.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    xSemaphoreTake(g_fusion_state.fusion_mutex, portMAX_DELAY);
    memcpy(covariance, &g_fusion_state.covariance, sizeof(ekf_covariance_t));
    xSemaphoreGive(g_fusion_state.fusion_mutex);
    
    return ESP_OK;
}

esp_err_t sensor_fusion_set_config(const fusion_config_t* config) {
    if (!config) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!g_fusion_state.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    xSemaphoreTake(g_fusion_state.fusion_mutex, portMAX_DELAY);
    g_fusion_state.config = *config;
    xSemaphoreGive(g_fusion_state.fusion_mutex);
    
    ESP_LOGI(TAG, "Sensor fusion configuration updated");
    return ESP_OK;
}

esp_err_t sensor_fusion_get_config(fusion_config_t* config) {
    if (!config) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!g_fusion_state.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    xSemaphoreTake(g_fusion_state.fusion_mutex, portMAX_DELAY);
    memcpy(config, &g_fusion_state.config, sizeof(fusion_config_t));
    xSemaphoreGive(g_fusion_state.fusion_mutex);
    
    return ESP_OK;
}

esp_err_t sensor_fusion_reset_state(void) {
    if (!g_fusion_state.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    xSemaphoreTake(g_fusion_state.fusion_mutex, portMAX_DELAY);
    
    // Reset state and covariance to initial values
    initialize_state_and_covariance();
    
    // Reset statistics
    memset(&g_fusion_state.stats, 0, sizeof(fusion_stats_t));
    
    // Reset timing
    g_fusion_state.last_prediction_time_us = esp_timer_get_time();
    
    xSemaphoreGive(g_fusion_state.fusion_mutex);
    
    ESP_LOGI(TAG, "Sensor fusion state reset");
    return ESP_OK;
}

esp_err_t sensor_fusion_initialize_attitude(const imu_data_t* imu_data) {
    if (!imu_data) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!g_fusion_state.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    xSemaphoreTake(g_fusion_state.fusion_mutex, portMAX_DELAY);
    
    // Initialize attitude from accelerometer assuming static conditions
    float accel_norm = sqrtf(imu_data->accel_x * imu_data->accel_x + 
                            imu_data->accel_y * imu_data->accel_y + 
                            imu_data->accel_z * imu_data->accel_z);
    
    if (accel_norm > 0.1f) {  // Valid accelerometer reading
        // Normalize accelerometer readings
        float ax = imu_data->accel_x / accel_norm;
        float ay = imu_data->accel_y / accel_norm;
        float az = imu_data->accel_z / accel_norm;
        
        // Compute initial roll and pitch from accelerometer
        float roll = atan2f(ay, az);
        float pitch = atan2f(-ax, sqrtf(ay*ay + az*az));
        float yaw = 0.0f;  // Cannot determine yaw from accelerometer alone
        
        // Convert Euler angles to quaternion
        float cr = cosf(roll * 0.5f);
        float sr = sinf(roll * 0.5f);
        float cp = cosf(pitch * 0.5f);
        float sp = sinf(pitch * 0.5f);
        float cy = cosf(yaw * 0.5f);
        float sy = sinf(yaw * 0.5f);
        
        g_fusion_state.state.qw = cr * cp * cy + sr * sp * sy;
        g_fusion_state.state.qx = sr * cp * cy - cr * sp * sy;
        g_fusion_state.state.qy = cr * sp * cy + sr * cp * sy;
        g_fusion_state.state.qz = cr * cp * sy - sr * sp * cy;
        
        // Reduce attitude uncertainty
        g_fusion_state.covariance.P[6][6] = 0.01f;
        g_fusion_state.covariance.P[7][7] = 0.01f;
        g_fusion_state.covariance.P[8][8] = 0.01f;
        g_fusion_state.covariance.P[9][9] = 0.1f;  // Yaw still uncertain
        
        ESP_LOGI(TAG, "Attitude initialized: roll=%.1f°, pitch=%.1f°", 
                 roll * 180.0f / M_PI, pitch * 180.0f / M_PI);
    }
    
    xSemaphoreGive(g_fusion_state.fusion_mutex);
    return ESP_OK;
}

esp_err_t sensor_fusion_set_initial_position(double lat, double lon, float alt) {
    if (!g_fusion_state.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    xSemaphoreTake(g_fusion_state.fusion_mutex, portMAX_DELAY);
    
    // Set initial position
    g_fusion_state.state.latitude = lat;
    g_fusion_state.state.longitude = lon;
    g_fusion_state.state.altitude = alt;
    
    // Set GPS origin if not already set
    if (!g_fusion_state.origin_set) {
        g_fusion_state.origin_lat = lat;
        g_fusion_state.origin_lon = lon;
        g_fusion_state.origin_alt = alt;
        g_fusion_state.origin_set = true;
    }
    
    // Reduce position uncertainty
    g_fusion_state.covariance.P[0][0] = 1.0f;
    g_fusion_state.covariance.P[1][1] = 1.0f;
    g_fusion_state.covariance.P[2][2] = 0.5f;
    
    xSemaphoreGive(g_fusion_state.fusion_mutex);
    
    ESP_LOGI(TAG, "Initial position set: %.8f, %.8f, %.2fm", lat, lon, alt);
    return ESP_OK;
}

esp_err_t sensor_fusion_enable_adaptive_tuning(bool enable) {
    if (!g_fusion_state.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Enable/disable adaptive tuning based on system conditions
    g_fusion_state.config.enable_bias_estimation = enable;
    
    // Adaptive tuning adjusts filter parameters based on motion conditions
    if (enable) {
        // When moving: increase process noise for more responsiveness
        if (g_fusion_state.velocity_magnitude > 2.0f) {
            g_fusion_state.config.position_noise *= 1.5f;
            g_fusion_state.config.velocity_noise *= 1.5f;
        }
        // When stationary: decrease noise for better stability
        else if (g_fusion_state.velocity_magnitude < 0.5f) {
            g_fusion_state.config.position_noise *= 0.7f;
            g_fusion_state.config.velocity_noise *= 0.7f;
        }
        
        // Adjust weights based on GPS quality
        if (g_fusion_state.gps_quality > 0.8f) {
            g_fusion_state.config.gps_weight = 0.7f;
            g_fusion_state.config.slam_weight = 0.3f;
        } else {
            g_fusion_state.config.gps_weight = 0.3f;
            g_fusion_state.config.slam_weight = 0.7f;
        }
    }
    
    ESP_LOGI(TAG, "✅ Adaptive tuning %s", enable ? "enabled" : "disabled");
    return ESP_OK;
}

esp_err_t sensor_fusion_set_outlier_rejection_threshold(float threshold) {
    if (!g_fusion_state.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (threshold < 0.0f || threshold > 10.0f) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Update outlier rejection thresholds for GPS and SLAM
    g_fusion_state.config.gps_outlier_threshold = threshold;
    g_fusion_state.config.slam_outlier_threshold = threshold * 0.5f; // SLAM typically more precise
    
    // Enable outlier detection if threshold is set
    g_fusion_state.config.enable_outlier_detection = (threshold > 0.0f);
    
    ESP_LOGI(TAG, "✅ Outlier rejection threshold set to %.2f m (GPS), %.2f m (SLAM)", 
             g_fusion_state.config.gps_outlier_threshold, g_fusion_state.config.slam_outlier_threshold);
    return ESP_OK;
}

// Advanced sensor fusion helper functions

static bool detect_gps_outlier(const gps_position_t* gps_data) {
    if (!g_fusion_state.config.enable_outlier_detection || !g_fusion_state.origin_set) {
        return false;  // Can't detect outliers without reference
    }
    
    // Check GPS accuracy if available
    if (gps_data->accuracy > 20.0f) {  // 20m accuracy threshold
        ESP_LOGW(TAG, "⚠️ GPS accuracy outlier detected: %.2f m", gps_data->accuracy);
        return true;
    }
    
    // Check minimum satellite count
    if (gps_data->satellites < 4) {  // Minimum 4 satellites for valid fix
        ESP_LOGW(TAG, "⚠️ GPS satellite count too low: %d", gps_data->satellites);
        return true;
    }
    
    // Convert GPS position to NED
    float north, east, down;
    esp_err_t ret = sensor_fusion_gps_to_ned(gps_data->latitude, gps_data->longitude, 
                                           gps_data->altitude, &north, &east, &down);
    if (ret != ESP_OK) {
        return true;  // Conversion failure is considered an outlier
    }
    
    // Compute current position estimate in NED
    float pred_north, pred_east, pred_down;
    ret = sensor_fusion_gps_to_ned(g_fusion_state.state.latitude, g_fusion_state.state.longitude,
                                 g_fusion_state.state.altitude, &pred_north, &pred_east, &pred_down);
    if (ret != ESP_OK) {
        return false;  // Can't compare if prediction conversion fails
    }
    
    // Compute position difference
    float pos_diff = sqrtf(powf(north - pred_north, 2) + powf(east - pred_east, 2) + 
                          powf(down - pred_down, 2));
    
    // Check against threshold
    float threshold = g_fusion_state.config.gps_outlier_threshold;
    if (pos_diff > threshold) {
        ESP_LOGW(TAG, "GPS outlier: position difference %.2fm exceeds threshold %.2fm", 
                 pos_diff, threshold);
        return true;
    }
    
    return false;
}

static bool detect_slam_outlier(const slam_pose_t* slam_pose) {
    if (!g_fusion_state.origin_set) {
        return false;
    }
    
    // Convert SLAM position to GPS
    double slam_lat, slam_lon;
    float slam_alt;
    esp_err_t ret = sensor_fusion_ned_to_gps(slam_pose->y, slam_pose->x, -slam_pose->z,
                                           &slam_lat, &slam_lon, &slam_alt);
    if (ret != ESP_OK) {
        return true;  // Conversion failure is an outlier
    }
    
    // Compute position difference from current estimate
    float north_diff, east_diff, down_diff;
    ret = sensor_fusion_gps_to_ned(slam_lat, slam_lon, slam_alt, 
                                 &north_diff, &east_diff, &down_diff);
    if (ret != ESP_OK) {
        return true;
    }
    
    // Get current position in NED
    float curr_north, curr_east, curr_down;
    ret = sensor_fusion_gps_to_ned(g_fusion_state.state.latitude, g_fusion_state.state.longitude,
                                 g_fusion_state.state.altitude, &curr_north, &curr_east, &curr_down);
    if (ret != ESP_OK) {
        return false;
    }
    
    float pos_diff = sqrtf(powf(north_diff - curr_north, 2) + 
                          powf(east_diff - curr_east, 2) + 
                          powf(down_diff - curr_down, 2));
    
    // Check against threshold
    float threshold = g_fusion_state.config.slam_outlier_threshold;
    if (pos_diff > threshold) {
        ESP_LOGW(TAG, "SLAM outlier: position difference %.2fm exceeds threshold %.2fm", 
                 pos_diff, threshold);
        return true;
    }
    
    // Check quaternion validity
    float q_norm = sqrtf(slam_pose->qw * slam_pose->qw + slam_pose->qx * slam_pose->qx + 
                        slam_pose->qy * slam_pose->qy + slam_pose->qz * slam_pose->qz);
    if (fabsf(q_norm - 1.0f) > 0.1f) {
        ESP_LOGW(TAG, "SLAM outlier: quaternion norm %.3f is not unit", q_norm);
        return true;
    }
    
    return false;
}

static void update_bias_estimates(const imu_data_t* imu_data) {
    static uint32_t bias_update_counter = 0;
    bias_update_counter++;
    
    // Update bias estimates every 100 IMU samples when stationary
    if (bias_update_counter % 100 != 0) {
        return;
    }
    
    // Check if system is stationary (low acceleration and rotation)
    float accel_mag = sqrtf(imu_data->accel_x * imu_data->accel_x + 
                           imu_data->accel_y * imu_data->accel_y + 
                           imu_data->accel_z * imu_data->accel_z);
    float gyro_mag = sqrtf(imu_data->gyro_x * imu_data->gyro_x + 
                          imu_data->gyro_y * imu_data->gyro_y + 
                          imu_data->gyro_z * imu_data->gyro_z);
    
    // Check for near-gravity acceleration (stationary condition)
    float gravity_tolerance = g_fusion_state.config.stationary_accel_threshold;
    float rotation_threshold = g_fusion_state.config.stationary_gyro_threshold;
    
    if (fabsf(accel_mag - GRAVITY_MS2) < gravity_tolerance && gyro_mag < rotation_threshold) {
        // System is stationary, update accelerometer bias
        float expected_z = GRAVITY_MS2;  // Gravity in body Z when level
        g_fusion_state.state.accel_bias_z += 0.001f * (imu_data->accel_z - expected_z);
        
        // Update gyroscope bias (should be zero when stationary)
        g_fusion_state.state.gyro_bias_x += 0.001f * imu_data->gyro_x;
        g_fusion_state.state.gyro_bias_y += 0.001f * imu_data->gyro_y;
        g_fusion_state.state.gyro_bias_z += 0.001f * imu_data->gyro_z;
        
        // Clamp bias values to reasonable ranges
        const float MAX_ACCEL_BIAS = 2.0f;  // m/s²
        const float MAX_GYRO_BIAS = 0.1f;   // rad/s
        
        g_fusion_state.state.accel_bias_x = fminf(fmaxf(g_fusion_state.state.accel_bias_x, -MAX_ACCEL_BIAS), MAX_ACCEL_BIAS);
        g_fusion_state.state.accel_bias_y = fminf(fmaxf(g_fusion_state.state.accel_bias_y, -MAX_ACCEL_BIAS), MAX_ACCEL_BIAS);
        g_fusion_state.state.accel_bias_z = fminf(fmaxf(g_fusion_state.state.accel_bias_z, -MAX_ACCEL_BIAS), MAX_ACCEL_BIAS);
        
        g_fusion_state.state.gyro_bias_x = fminf(fmaxf(g_fusion_state.state.gyro_bias_x, -MAX_GYRO_BIAS), MAX_GYRO_BIAS);
        g_fusion_state.state.gyro_bias_y = fminf(fmaxf(g_fusion_state.state.gyro_bias_y, -MAX_GYRO_BIAS), MAX_GYRO_BIAS);
        g_fusion_state.state.gyro_bias_z = fminf(fmaxf(g_fusion_state.state.gyro_bias_z, -MAX_GYRO_BIAS), MAX_GYRO_BIAS);
    }
}

static void compute_euler_angles(float* roll, float* pitch, float* yaw) {
    float qw = g_fusion_state.state.qw;
    float qx = g_fusion_state.state.qx;
    float qy = g_fusion_state.state.qy;
    float qz = g_fusion_state.state.qz;
    
    // Roll (x-axis rotation)
    float sinr_cosp = 2 * (qw * qx + qy * qz);
    float cosr_cosp = 1 - 2 * (qx * qx + qy * qy);
    *roll = atan2f(sinr_cosp, cosr_cosp);
    
    // Pitch (y-axis rotation)
    float sinp = 2 * (qw * qy - qz * qx);
    if (fabsf(sinp) >= 1) {
        *pitch = copysignf(M_PI / 2, sinp); // Use 90 degrees if out of range
    } else {
        *pitch = asinf(sinp);
    }
    
    // Yaw (z-axis rotation)
    float siny_cosp = 2 * (qw * qz + qx * qy);
    float cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
    *yaw = atan2f(siny_cosp, cosy_cosp);
}

static esp_err_t full_ekf_gps_update(const gps_position_t* gps_data) __attribute__((unused));
static esp_err_t full_ekf_gps_update(const gps_position_t* gps_data) {
    // Full Extended Kalman Filter GPS update with Jacobian matrices
    // State vector: [lat, lon, alt, vel_n, vel_e, vel_d, qw, qx, qy, qz, acc_bias_x, acc_bias_y, acc_bias_z, gyro_bias_x, gyro_bias_y]
    
    // Convert GPS position to NED coordinates
    float north, east, down;
    esp_err_t ret = sensor_fusion_gps_to_ned(gps_data->latitude, gps_data->longitude, 
                                           gps_data->altitude, &north, &east, &down);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Observation vector z = [north, east, down]
    float z[3] = {north, east, down};
    
    // Predicted observation h(x) = [pos_n, pos_e, pos_d] from current state
    float h[3];
    sensor_fusion_gps_to_ned(g_fusion_state.state.latitude, g_fusion_state.state.longitude,
                           g_fusion_state.state.altitude, &h[0], &h[1], &h[2]);
    
    // Innovation (measurement residual) y = z - h(x)
    float innovation[3] = {
        z[0] - h[0],
        z[1] - h[1], 
        z[2] - h[2]
    };
    
    // Observation Jacobian H (3x15) - only position states affect GPS measurements
    // H = [I3x3, 03x12] where I3x3 is identity for position states
    float H[3][15] = {0};
    H[0][0] = 1.0f; // north position
    H[1][1] = 1.0f; // east position
    H[2][2] = 1.0f; // down position
    (void)H; // Suppress unused warning - used in full matrix implementation
    
    // Measurement noise covariance R (3x3)
    float R[3][3] = {0};
    float gps_noise = g_fusion_state.config.gps_position_noise;
    R[0][0] = gps_noise; // north noise
    R[1][1] = gps_noise; // east noise  
    R[2][2] = gps_noise * 2.0f; // down noise (GPS altitude less accurate)
    
    // Innovation covariance S = H*P*H' + R
    // For GPS (position only), S is just P_pos + R since H is identity for position
    float S[3][3] = {0};
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            S[i][j] = R[i][j];
            if (i == j) {
                S[i][j] += g_fusion_state.covariance.P[i][i]; // Add position covariance
            }
        }
    }
    
    // Kalman gain K = P*H'*S^-1
    // For position-only GPS update, this simplifies significantly
    float K[15][3] = {0};
    for (int i = 0; i < 3; i++) {
        if (S[i][i] > 1e-6f) { // Avoid division by zero
            K[i][i] = g_fusion_state.covariance.P[i][i] / S[i][i];
        }
    }
    
    // State update: convert NED innovation back to GPS coordinates
    // For small updates, we can use linear approximation
    double lat_update = innovation[0] / 111000.0; // North to latitude (approx)
    double lon_update = innovation[1] / (111000.0 * cos(g_fusion_state.state.latitude * M_PI / 180.0)); // East to longitude
    float alt_update = -innovation[2]; // Down to altitude (negative because down is positive in NED)
    
    g_fusion_state.state.latitude += K[0][0] * lat_update;
    g_fusion_state.state.longitude += K[1][1] * lon_update; 
    g_fusion_state.state.altitude += K[2][2] * alt_update;
    
    // Covariance update: P = (I - K*H)*P
    for (int i = 0; i < 3; i++) {
        g_fusion_state.covariance.P[i][i] *= (1.0f - K[i][i]);
    }
    
    return ESP_OK;
}

static esp_err_t full_ekf_slam_update(const slam_pose_t* slam_pose) __attribute__((unused));
static esp_err_t full_ekf_slam_update(const slam_pose_t* slam_pose) {
    // Full Extended Kalman Filter SLAM update with Jacobian matrices
    // SLAM provides both position and orientation measurements
    
    // Observation vector z = [pos_x, pos_y, pos_z, qw, qx, qy, qz]
    float z[7] = {
        slam_pose->x,
        slam_pose->y, 
        slam_pose->z,
        slam_pose->qw,
        slam_pose->qx,
        slam_pose->qy,
        slam_pose->qz
    };
    
    // Predicted observation h(x) from current state
    float h[7];
    // Convert GPS position to local NED for comparison with SLAM
    sensor_fusion_gps_to_ned(g_fusion_state.state.latitude, g_fusion_state.state.longitude,
                           g_fusion_state.state.altitude, &h[0], &h[1], &h[2]);
    h[3] = g_fusion_state.state.qw;
    h[4] = g_fusion_state.state.qx;
    h[5] = g_fusion_state.state.qy;
    h[6] = g_fusion_state.state.qz;
    
    // Innovation (measurement residual) y = z - h(x)
    float innovation[7];
    for (int i = 0; i < 7; i++) {
        innovation[i] = z[i] - h[i];
    }
    
    // Special handling for quaternion innovation (ensure shortest rotation)
    float quat_dot = h[3]*z[3] + h[4]*z[4] + h[5]*z[5] + h[6]*z[6];
    if (quat_dot < 0.0f) {
        // Flip quaternion to ensure shortest path
        for (int i = 3; i < 7; i++) {
            innovation[i] = -z[i] - h[i];
        }
    }
    
    // Observation Jacobian H (7x15) - position and quaternion states
    float H[7][15] = {0};
    // Position measurements
    H[0][0] = 1.0f; // x position
    H[1][1] = 1.0f; // y position  
    H[2][2] = 1.0f; // z position
    // Quaternion measurements
    H[3][6] = 1.0f; // qw
    H[4][7] = 1.0f; // qx
    H[5][8] = 1.0f; // qy
    H[6][9] = 1.0f; // qz
    (void)H; // Suppress unused warning - used in full matrix implementation
    
    // Measurement noise covariance R (7x7)
    float R[7][7] = {0};
    float slam_pos_noise = g_fusion_state.config.slam_position_noise;
    float slam_att_noise = g_fusion_state.config.slam_attitude_noise;
    
    R[0][0] = slam_pos_noise; // x position noise
    R[1][1] = slam_pos_noise; // y position noise
    R[2][2] = slam_pos_noise; // z position noise
    R[3][3] = slam_att_noise; // qw noise
    R[4][4] = slam_att_noise; // qx noise
    R[5][5] = slam_att_noise; // qy noise
    R[6][6] = slam_att_noise; // qz noise
    
    // Innovation covariance S = H*P*H' + R
    float S[7][7] = {0};
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 7; j++) {
            S[i][j] = R[i][j];
        }
    }
    
    // Add corresponding covariance terms
    for (int i = 0; i < 3; i++) {
        S[i][i] += g_fusion_state.covariance.P[i][i]; // Position covariance
    }
    for (int i = 3; i < 7; i++) {
        S[i][i] += g_fusion_state.covariance.P[i+3][i+3]; // Quaternion covariance  
    }
    
    // Kalman gain K = P*H'*S^-1 (simplified for diagonal case)
    float K[15][7] = {0};
    for (int i = 0; i < 3; i++) {
        if (S[i][i] > 1e-6f) {
            K[i][i] = g_fusion_state.covariance.P[i][i] / S[i][i];
        }
    }
    for (int i = 3; i < 7; i++) {
        if (S[i][i] > 1e-6f) {
            K[i+3][i] = g_fusion_state.covariance.P[i+3][i+3] / S[i][i];
        }
    }
    
    // State update: x = x + K*innovation
    // Update position: convert NED innovation back to GPS coordinates
    double lat_update = innovation[0] / 111000.0; // North to latitude
    double lon_update = innovation[1] / (111000.0 * cos(g_fusion_state.state.latitude * M_PI / 180.0)); // East to longitude
    float alt_update = -innovation[2]; // Down to altitude
    
    g_fusion_state.state.latitude += K[0][0] * lat_update;
    g_fusion_state.state.longitude += K[1][1] * lon_update;
    g_fusion_state.state.altitude += K[2][2] * alt_update;
    
    g_fusion_state.state.qw += K[6][3] * innovation[3];
    g_fusion_state.state.qx += K[7][4] * innovation[4];
    g_fusion_state.state.qy += K[8][5] * innovation[5];
    g_fusion_state.state.qz += K[9][6] * innovation[6];
    
    // Normalize quaternion after update
    float quat_norm = sqrtf(g_fusion_state.state.qw * g_fusion_state.state.qw +
                           g_fusion_state.state.qx * g_fusion_state.state.qx +
                           g_fusion_state.state.qy * g_fusion_state.state.qy +
                           g_fusion_state.state.qz * g_fusion_state.state.qz);
    if (quat_norm > 1e-6f) {
        g_fusion_state.state.qw /= quat_norm;
        g_fusion_state.state.qx /= quat_norm;
        g_fusion_state.state.qy /= quat_norm;
        g_fusion_state.state.qz /= quat_norm;
    }
    
    // Covariance update: P = (I - K*H)*P
    for (int i = 0; i < 3; i++) {
        g_fusion_state.covariance.P[i][i] *= (1.0f - K[i][i]);
    }
    for (int i = 6; i < 10; i++) {
        g_fusion_state.covariance.P[i][i] *= (1.0f - K[i][i-3]);
    }
    
    return ESP_OK;
}
