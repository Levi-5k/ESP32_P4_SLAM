/*
 * Configuration Loader Component
 * Loads JSON configuration files and populates system configuration structures
 */

#pragma once

#include <esp_err.h>
#include <stdbool.h>
#include "visual_slam_common_types.h"
#include "msp_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

// System configuration structure
typedef struct {
    char name[64];
    char version[16];
    char description[128];
    bool debug_mode;
    char log_level[8];
    bool enable_watchdog;
    uint32_t watchdog_timeout_ms;
    bool enable_statistics;
    uint32_t statistics_interval_ms;
    // Component enable/disable flags
    bool camera_enabled;
    bool gps_enabled;
    bool imu_enabled;
    bool slam_enabled;
    bool sensor_fusion_enabled;
    bool msp_enabled;
    bool sd_storage_enabled;
    bool web_server_enabled;
    // Performance settings
    uint32_t target_fps;
    uint32_t max_processing_time_ms;
    bool memory_monitoring;
    bool cpu_monitoring;
    bool enable_performance_logging;
    // Safety settings
    bool emergency_stop_enabled;
    uint32_t failsafe_timeout_ms;
    float maximum_altitude_m;
    float minimum_altitude_m;
    float maximum_velocity_ms;
    bool geofence_enabled;
    // Communication settings
    uint32_t uart_buffer_size;
    uint32_t i2c_clock_speed;
    uint32_t spi_clock_speed;
    uint32_t communication_timeout_ms;
    // WiFi settings
    char wifi_ssid[32];
    char wifi_password[64];
    uint32_t wifi_connect_timeout_ms;
    bool wifi_auto_reconnect;
} system_config_t;

// Extended configuration structures with additional fields
typedef struct {
    gps_config_t base_config;
    bool enable_sbas;
    bool enable_differential;
    uint32_t timeout_ms;
    bool auto_reconnect;
    uint32_t reconnect_attempts;
    uint32_t reconnect_delay_ms;
    uint8_t minimum_satellites;
    float maximum_hdop;
    float maximum_speed_ms;
    bool speed_filter_enabled;
    bool altitude_filter_enabled;
    float minimum_altitude_m;
    float maximum_altitude_m;
    uint8_t dynamic_model;
    uint8_t fix_mode;
    bool enable_gps;
    bool enable_glonass;
    bool enable_galileo;
    bool enable_beidou;
    bool time_pulse_enabled;
    bool antenna_detection;
    // AssistNow Offline configuration
    bool assistnow_enabled;
    char assistnow_token[64];
    uint32_t assistnow_update_interval_hours;
    uint32_t assistnow_validity_hours;
    bool assistnow_auto_download;
    char assistnow_server_url[128];
    // Position Aiding configuration
    bool position_aiding_enabled;
    bool use_takeoff_position;
    double takeoff_latitude;
    double takeoff_longitude;
    float takeoff_altitude_m;
    float position_accuracy_m;
    float altitude_accuracy_m;
    bool auto_update_from_gps;
} extended_gps_config_t;

typedef struct {
    msp_config_t base_config;
    uint32_t status_request_interval_ms;
    uint32_t navigation_status_interval_ms;
    uint32_t attitude_request_interval_ms;
    uint32_t altitude_request_interval_ms;
    uint32_t gps_request_interval_ms;
    uint32_t analog_request_interval_ms;
    uint8_t default_mode;
    float waypoint_radius_m;
    uint32_t waypoint_hold_time_ms;
    float cruise_speed_ms;
    float max_climb_rate_ms;
    float max_descent_rate_ms;
    float rtl_altitude_m;
    float land_speed_ms;
    bool emergency_stop_enabled;
    bool failsafe_enabled;
    float low_battery_voltage;
    float critical_battery_voltage;
    uint8_t maximum_roll_angle_deg;
    uint8_t maximum_pitch_angle_deg;
} extended_msp_config_t;

typedef struct {
    slam_config_t base_config;
    float tracking_confidence_threshold;
    bool relocalization_enabled;
    bool outlier_rejection_enabled;
    bool bundle_adjustment_enabled;
    char feature_tracking_method[32];
    char motion_model[32];
    bool map_save_enabled;
    bool map_load_enabled;
    bool map_compression_enabled;
    uint32_t max_map_points;
    float map_point_culling_threshold;
    float keyframe_culling_threshold;
    uint8_t processing_threads;
    uint8_t feature_extraction_threads;
    uint8_t optimization_threads;
    uint32_t memory_pool_size_kb;
    uint32_t feature_cache_size;
} extended_slam_config_t;

typedef struct {
    fusion_config_t base_config;
    float gps_rejection_threshold;
    float slam_rejection_threshold;
    float imu_rejection_threshold;
    bool outlier_detection_enabled;
    float gps_outlier_threshold;
    float slam_outlier_threshold;
    float imu_outlier_threshold;
    bool filter_reset_enabled;
    float filter_reset_threshold;
    uint32_t imu_update_rate_hz;
    uint32_t gps_update_rate_hz;
    uint32_t slam_update_rate_hz;
    uint32_t prediction_rate_hz;
    uint32_t correction_timeout_ms;
} extended_fusion_config_t;

typedef struct {
    camera_config_t base_config;
    bool aec2;
    uint32_t aec_value;
    int ae_level;
    bool agc;
    uint32_t agc_gain;
    uint32_t gain_ceiling;
    bool bpc;
    bool wpc;
    bool raw_gma;
    bool lenc;
    bool hmirror;
    bool vflip;
    bool dcw;
    bool colorbar;
    uint8_t quality;
    int sharpness;
    uint8_t denoise;
    uint8_t special_effect;
    uint8_t wb_mode;
    bool awb;
    bool awb_gain;
    bool aec;
    uint8_t aec_level;
} extended_camera_config_t;

typedef struct {
    imu_config_t base_config;
    bool calibration_enabled;
    bool auto_calibration;
    uint32_t calibration_samples;
    float accel_bias_x;
    float accel_bias_y;
    float accel_bias_z;
    float gyro_bias_x;
    float gyro_bias_y;
    float gyro_bias_z;
    float accel_scale_x;
    float accel_scale_y;
    float accel_scale_z;
    float gyro_scale_x;
    float gyro_scale_y;
    float gyro_scale_z;
    bool low_pass_filter_enabled;
    float low_pass_cutoff_hz;
    bool high_pass_filter_enabled;
    float high_pass_cutoff_hz;
    bool notch_filter_enabled;
    float notch_frequency_hz;
    float notch_bandwidth_hz;
    bool temperature_compensation;
    bool self_test_enabled;
    bool interrupt_enabled;
    bool fifo_enabled;
    uint32_t fifo_watermark;
    char power_mode[16];
} extended_imu_config_t;

// Master configuration structure
typedef struct {
    system_config_t system;
    extended_gps_config_t gps;
    extended_msp_config_t msp;
    extended_slam_config_t slam;
    extended_fusion_config_t fusion;
    extended_camera_config_t camera;
    extended_imu_config_t imu;
    bool loaded;
    uint64_t load_timestamp;
} master_config_t;

// Function declarations
esp_err_t config_loader_init(void);
esp_err_t config_loader_load_all(master_config_t* config);
esp_err_t config_loader_load_system_config(system_config_t* config);
esp_err_t config_loader_load_gps_config(extended_gps_config_t* config);
esp_err_t config_loader_load_msp_config(extended_msp_config_t* config);
esp_err_t config_loader_load_slam_config(extended_slam_config_t* config);
esp_err_t config_loader_load_fusion_config(extended_fusion_config_t* config);
esp_err_t config_loader_load_camera_config(extended_camera_config_t* config);
esp_err_t config_loader_load_imu_config(extended_imu_config_t* config);
esp_err_t config_loader_save_config(const master_config_t* config);
esp_err_t config_loader_validate_config(master_config_t* config);
void config_loader_print_config(const master_config_t* config);

// Default Configuration Creation
esp_err_t config_loader_create_defaults(void);

// SPIFFS Configuration Management
esp_err_t config_loader_save_system_to_spiffs(const system_config_t* config);
esp_err_t config_loader_save_gps_to_spiffs(const extended_gps_config_t* config);
esp_err_t config_loader_save_msp_to_spiffs(const extended_msp_config_t* config);
esp_err_t config_loader_save_slam_to_spiffs(const extended_slam_config_t* config);
esp_err_t config_loader_save_fusion_to_spiffs(const extended_fusion_config_t* config);
esp_err_t config_loader_save_camera_to_spiffs(const extended_camera_config_t* config);
esp_err_t config_loader_save_imu_to_spiffs(const extended_imu_config_t* config);

// SD Card Configuration Management
bool config_loader_is_sd_available(void);
esp_err_t config_loader_save_to_sd(const master_config_t* config);
esp_err_t config_loader_copy_defaults_to_sd(void);
esp_err_t config_loader_get_storage_info(bool* spiffs_available, bool* sd_available, uint64_t* sd_free_space_mb);

#ifdef __cplusplus
}
#endif
