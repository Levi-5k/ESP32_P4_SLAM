/*
 * Configuration Loader Implementation
 * Loads JSON configuration files using cJSON library
 * Supports both SPIFFS (defaults) and SD card (user configs)
 */

#include "config_loader.h"
#include <stdio.h>
#include <string.h>
#include <esp_log.h>
#include <esp_system.h>
#include <cJSON.h>
#include <esp_spiffs.h>
#include <esp_timer.h>
#include <driver/gpio.h>
#include <driver/spi_master.h>
#include "sd_storage.h"
#include "esp32p4_pin_config.h"

static const char* TAG = "CONFIG_LOADER";

// Forward declarations for SD save functions
static esp_err_t save_system_config_to_sd(const system_config_t* config);
static esp_err_t save_gps_config_to_sd(const extended_gps_config_t* config);
static esp_err_t save_msp_config_to_sd(const extended_msp_config_t* config);
static esp_err_t save_slam_config_to_sd(const extended_slam_config_t* config);
static esp_err_t save_fusion_config_to_sd(const extended_fusion_config_t* config);
static esp_err_t save_camera_config_to_sd(const extended_camera_config_t* config);
static esp_err_t save_imu_config_to_sd(const extended_imu_config_t* config);
static esp_err_t save_master_config_to_sd(const master_config_t* config);
static esp_err_t save_master_config_to_spiffs(const master_config_t* config);

// Embedded default configuration JSON strings
static const char* DEFAULT_SYSTEM_CONFIG_JSON = "{"
  "\"system\": {"
    "\"name\": \"ESP32-P4 Visual SLAM Navigation System\","
    "\"version\": \"1.0.0\","
    "\"description\": \"Real-time Visual SLAM navigation system for autonomous drone\","
    "\"debug_mode\": false,"
    "\"log_level\": \"INFO\","
    "\"enable_watchdog\": true,"
    "\"watchdog_timeout_ms\": 5000,"
    "\"enable_statistics\": true,"
    "\"statistics_interval_ms\": 1000"
  "},"
  "\"components\": {"
    "\"camera_enabled\": true,"
    "\"gps_enabled\": true,"
    "\"imu_enabled\": true,"
    "\"slam_enabled\": true,"
    "\"sensor_fusion_enabled\": true,"
    "\"msp_enabled\": true,"
    "\"sd_storage_enabled\": true,"
    "\"web_server_enabled\": false"
  "},"
  "\"performance\": {"
    "\"target_fps\": 30,"
    "\"max_processing_time_ms\": 33,"
    "\"memory_monitoring\": true,"
    "\"cpu_monitoring\": true,"
    "\"enable_performance_logging\": false"
  "},"
  "\"safety\": {"
    "\"emergency_stop_enabled\": true,"
    "\"failsafe_timeout_ms\": 10000,"
    "\"maximum_altitude_m\": 100.0,"
    "\"minimum_altitude_m\": 0.5,"
    "\"maximum_velocity_ms\": 15.0,"
    "\"geofence_enabled\": false"
  "},"
  "\"communication\": {"
    "\"uart_buffer_size\": 2048,"
    "\"i2c_clock_speed\": 400000,"
    "\"spi_clock_speed\": 10000000,"
    "\"communication_timeout_ms\": 5000"
  "}"
"}";

static const char* DEFAULT_GPS_CONFIG_JSON = "{"
  "\"gps\": {"
    "\"uart_port\": 1,"
    "\"baud_rate\": 115600,"
    "\"tx_pin\": 15,"
    "\"rx_pin\": 16,"
    "\"update_rate_hz\": 10,"
    "\"enable_sbas\": true,"
    "\"enable_differential\": false,"
    "\"timeout_ms\": 5000,"
    "\"auto_reconnect\": true,"
    "\"reconnect_attempts\": 3,"
    "\"reconnect_delay_ms\": 1000"
  "},"
  "\"gps_filters\": {"
    "\"minimum_satellites\": 7,"
    "\"maximum_hdop\": 5.0,"
    "\"maximum_speed_ms\": 50.0,"
    "\"speed_filter_enabled\": true,"
    "\"altitude_filter_enabled\": true,"
    "\"minimum_altitude_m\": -100.0,"
    "\"maximum_altitude_m\": 10000.0"
  "},"
  "\"gps_advanced\": {"
    "\"dynamic_model\": 0,"
    "\"fix_mode\": 6,"
    "\"enable_gps\": true,"
    "\"enable_glonass\": true,"
    "\"enable_galileo\": true,"
    "\"enable_beidou\": true,"
    "\"time_pulse_enabled\": false,"
    "\"antenna_detection\": false"
  "}"
"}";

static const char* DEFAULT_MSP_CONFIG_JSON = "{"
  "\"msp\": {"
    "\"uart_port\": 2,"
    "\"baud_rate\": 115200,"
    "\"tx_pin\": 17,"
    "\"rx_pin\": 18,"
    "\"timeout_ms\": 1000,"
    "\"auto_reconnect\": true,"
    "\"status_request_interval_ms\": 1000,"
    "\"navigation_status_interval_ms\": 500,"
    "\"attitude_request_interval_ms\": 100,"
    "\"altitude_request_interval_ms\": 200,"
    "\"gps_request_interval_ms\": 1000,"
    "\"analog_request_interval_ms\": 5000,"
    "\"default_mode\": 0,"
    "\"waypoint_radius_m\": 2.0,"
    "\"waypoint_hold_time_ms\": 5000,"
    "\"cruise_speed_ms\": 5.0,"
    "\"max_climb_rate_ms\": 2.0,"
    "\"max_descent_rate_ms\": 1.5,"
    "\"rtl_altitude_m\": 10.0,"
    "\"land_speed_ms\": 1.0,"
    "\"emergency_stop_enabled\": true,"
    "\"failsafe_enabled\": true,"
    "\"low_battery_voltage\": 3.3,"
    "\"critical_battery_voltage\": 3.0,"
    "\"maximum_roll_angle_deg\": 45,"
    "\"maximum_pitch_angle_deg\": 45"
  "}"
"}";

static const char* DEFAULT_SLAM_CONFIG_JSON = "{"
  "\"slam\": {"
    "\"max_features\": 500,"
    "\"fast_threshold\": 20.0,"
    "\"levels\": 8,"
    "\"scale_factor\": 1.2,"
    "\"max_keypoints_per_level\": 100,"
    "\"use_harris_detector\": false,"
    "\"harris_k\": 0.04,"
    "\"descriptor_distance_threshold\": 50,"
    "\"match_threshold\": 0.7,"
    "\"min_tracked_features\": 30,"
    "\"keyframe_distance_threshold\": 1.0,"
    "\"keyframe_angle_threshold\": 0.2,"
    "\"max_keyframes\": 50,"
    "\"enable_loop_closure\": true,"
    "\"loop_closure_threshold\": 0.8,"
    "\"map_optimization_enabled\": true,"
    "\"tracking_confidence_threshold\": 0.6,"
    "\"relocalization_enabled\": true,"
    "\"outlier_rejection_enabled\": true,"
    "\"bundle_adjustment_enabled\": true,"
    "\"feature_tracking_method\": \"ORB\","
    "\"motion_model\": \"CONSTANT_VELOCITY\","
    "\"map_save_enabled\": true,"
    "\"map_load_enabled\": true,"
    "\"map_compression_enabled\": true,"
    "\"max_map_points\": 10000,"
    "\"map_point_culling_threshold\": 0.1,"
    "\"keyframe_culling_threshold\": 0.5,"
    "\"processing_threads\": 2,"
    "\"feature_extraction_threads\": 1,"
    "\"optimization_threads\": 1,"
    "\"memory_pool_size_kb\": 1024,"
    "\"feature_cache_size\": 1000"
  "}"
"}";

static const char* DEFAULT_FUSION_CONFIG_JSON = "{"
  "\"fusion\": {"
    "\"description\": \"Sensor fusion EKF configuration\","
    "\"position_noise\": 0.1,"
    "\"velocity_noise\": 0.01,"
    "\"attitude_noise\": 0.001,"
    "\"slam_attitude_noise\": 0.01,"
    "\"gps_position_noise\": 2.5,"
    "\"gps_velocity_noise\": 0.5,"
    "\"slam_position_noise\": 0.1,"
    "\"accel_bias_noise\": 0.000001,"
    "\"gyro_bias_noise\": 0.00000001,"
    "\"imu_dt\": 0.01,"
    "\"max_prediction_time\": 1.0"
  "},"
  "\"fusion_weights\": {"
    "\"gps_weight\": 0.8,"
    "\"slam_weight\": 0.9,"
    "\"imu_weight\": 1.0,"
    "\"gps_rejection_threshold\": 3.0,"
    "\"slam_rejection_threshold\": 2.0,"
    "\"imu_rejection_threshold\": 5.0"
  "},"
  "\"fusion_filters\": {"
    "\"outlier_detection_enabled\": true,"
    "\"gps_outlier_threshold\": 10.0,"
    "\"slam_outlier_threshold\": 5.0,"
    "\"imu_outlier_threshold\": 20.0,"
    "\"filter_reset_enabled\": true,"
    "\"filter_reset_threshold\": 100.0"
  "},"
  "\"fusion_timing\": {"
    "\"imu_update_rate_hz\": 100,"
    "\"gps_update_rate_hz\": 5,"
    "\"slam_update_rate_hz\": 30,"
    "\"prediction_rate_hz\": 100,"
    "\"correction_timeout_ms\": 100"
  "}"
"}";

static const char* DEFAULT_CAMERA_CONFIG_JSON = "{"
  "\"camera\": {"
    "\"resolution\": \"1920x1080\","
    "\"format\": \"RGB565\","
    "\"fps\": 30,"
    "\"auto_exposure\": true,"
    "\"auto_white_balance\": true,"
    "\"brightness\": 0,"
    "\"contrast\": 0,"
    "\"saturation\": 0,"
    "\"exposure_value\": 100,"
    "\"aec2\": true,"
    "\"aec_value\": 300,"
    "\"ae_level\": 0,"
    "\"agc\": true,"
    "\"agc_gain\": 0,"
    "\"gain_ceiling\": 0,"
    "\"bpc\": false,"
    "\"wpc\": false,"
    "\"raw_gma\": true,"
    "\"lenc\": true,"
    "\"hmirror\": false,"
    "\"vflip\": false,"
    "\"dcw\": true,"
    "\"colorbar\": false,"
    "\"quality\": 12,"
    "\"sharpness\": 0,"
    "\"denoise\": 0,"
    "\"special_effect\": 0,"
    "\"wb_mode\": 0,"
    "\"awb\": true,"
    "\"awb_gain\": true,"
    "\"aec\": true,"
    "\"aec_level\": 0"
  "}"
"}";

static const char* DEFAULT_IMU_CONFIG_JSON = "{"
  "\"imu\": {"
    "\"spi_host\": 2,"
    "\"miso_pin\": 20,"
    "\"mosi_pin\": 21,"
    "\"sclk_pin\": 22,"
    "\"acc_cs_pin\": 23,"
    "\"gyro_cs_pin\": 24,"
    "\"accel_range\": 6,"
    "\"gyro_range\": 500,"
    "\"sample_rate\": 400,"
    "\"calibration_enabled\": true,"
    "\"auto_calibration\": true,"
    "\"calibration_samples\": 1000,"
    "\"accel_bias_x\": 0.0,"
    "\"accel_bias_y\": 0.0,"
    "\"accel_bias_z\": 0.0,"
    "\"gyro_bias_x\": 0.0,"
    "\"gyro_bias_y\": 0.0,"
    "\"gyro_bias_z\": 0.0,"
    "\"accel_scale_x\": 1.0,"
    "\"accel_scale_y\": 1.0,"
    "\"accel_scale_z\": 1.0,"
    "\"gyro_scale_x\": 1.0,"
    "\"gyro_scale_y\": 1.0,"
    "\"gyro_scale_z\": 1.0,"
    "\"low_pass_filter_enabled\": true,"
    "\"low_pass_cutoff_hz\": 100.0,"
    "\"high_pass_filter_enabled\": false,"
    "\"high_pass_cutoff_hz\": 0.1,"
    "\"notch_filter_enabled\": false,"
    "\"notch_frequency_hz\": 50.0,"
    "\"notch_bandwidth_hz\": 5.0,"
    "\"temperature_compensation\": true,"
    "\"self_test_enabled\": false,"
    "\"interrupt_enabled\": true,"
    "\"fifo_enabled\": true,"
    "\"fifo_watermark\": 100,"
    "\"power_mode\": \"NORMAL\""
  "}"
"}";















// Configuration file paths - SPIFFS (defaults)
#define SPIFFS_BASE_PATH "/config"
#define SYSTEM_CONFIG_FILE_SPIFFS SPIFFS_BASE_PATH "/system_config.json"
#define GPS_CONFIG_FILE_SPIFFS SPIFFS_BASE_PATH "/gps_config.json"
#define MSP_CONFIG_FILE_SPIFFS SPIFFS_BASE_PATH "/msp_config.json"
#define SLAM_CONFIG_FILE_SPIFFS SPIFFS_BASE_PATH "/slam_config.json"
#define FUSION_CONFIG_FILE_SPIFFS SPIFFS_BASE_PATH "/fusion_config.json"
#define CAMERA_CONFIG_FILE_SPIFFS SPIFFS_BASE_PATH "/camera_config.json"
#define IMU_CONFIG_FILE_SPIFFS SPIFFS_BASE_PATH "/imu_config.json"

// Configuration file paths - SD card (user configs)
#define SYSTEM_CONFIG_FILE_SD SD_CONFIG_DIR "/system_config.json"
#define GPS_CONFIG_FILE_SD SD_CONFIG_DIR "/gps_config.json"
#define MSP_CONFIG_FILE_SD SD_CONFIG_DIR "/msp_config.json"
#define SLAM_CONFIG_FILE_SD SD_CONFIG_DIR "/slam_config.json"
#define FUSION_CONFIG_FILE_SD SD_CONFIG_DIR "/fusion_config.json"
#define CAMERA_CONFIG_FILE_SD SD_CONFIG_DIR "/camera_config.json"
#define IMU_CONFIG_FILE_SD SD_CONFIG_DIR "/imu_config.json"

// SPIFFS mount point
#define SPIFFS_MOUNT_POINT "/config"

// Global state
static struct {
    bool spiffs_mounted;
    bool sd_available;
} g_config_state = {0};

// Helper function to read file from SPIFFS
static esp_err_t read_config_file_spiffs(const char* filename, char** buffer, size_t* size) {
    FILE* f = fopen(filename, "r");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open SPIFFS file %s", filename);
        return ESP_ERR_NOT_FOUND;
    }

    fseek(f, 0, SEEK_END);
    *size = ftell(f);
    fseek(f, 0, SEEK_SET);

    *buffer = malloc(*size + 1);
    if (*buffer == NULL) {
        fclose(f);
        return ESP_ERR_NO_MEM;
    }

    size_t read_size = fread(*buffer, 1, *size, f);
    (*buffer)[read_size] = '\0';
    fclose(f);

    if (read_size != *size) {
        free(*buffer);
        return ESP_ERR_INVALID_SIZE;
    }

    return ESP_OK;
}

// Helper function to read file from SD card
static esp_err_t read_config_file_sd(const char* filename, char** buffer, size_t* size) {
    if (!g_config_state.sd_available) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    FILE* f = fopen(filename, "r");
    if (f == NULL) {
        ESP_LOGD(TAG, "SD card file %s not found, will try SPIFFS", filename);
        return ESP_ERR_NOT_FOUND;
    }

    fseek(f, 0, SEEK_END);
    *size = ftell(f);
    fseek(f, 0, SEEK_SET);

    *buffer = malloc(*size + 1);
    if (*buffer == NULL) {
        fclose(f);
        return ESP_ERR_NO_MEM;
    }

    size_t read_size = fread(*buffer, 1, *size, f);
    (*buffer)[read_size] = '\0';
    fclose(f);

    if (read_size != *size) {
        free(*buffer);
        return ESP_ERR_INVALID_SIZE;
    }

    return ESP_OK;
}

// Helper function to copy embedded default JSON to SPIFFS
static esp_err_t copy_default_to_spiffs(const char* default_json, const char* spiffs_path) {
    if (default_json == NULL || spiffs_path == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    size_t size = strlen(default_json);

    // Write to SPIFFS
    FILE* dst_f = fopen(spiffs_path, "w");
    if (dst_f == NULL) {
        ESP_LOGE(TAG, "Failed to open SPIFFS file for writing: %s", spiffs_path);
        return ESP_ERR_INVALID_ARG;
    }

    size_t written = fwrite(default_json, 1, size, dst_f);
    fclose(dst_f);

    if (written != size) {
        ESP_LOGE(TAG, "Failed to write complete file to SPIFFS: %s", spiffs_path);
        return ESP_ERR_INVALID_SIZE;
    }

    ESP_LOGI(TAG, "Copied embedded default config to SPIFFS: %s", spiffs_path);
    return ESP_OK;
}

// Helper function to read config file with fallback (SD card first, then SPIFFS)
static esp_err_t read_config_file_with_fallback(const char* sd_filename, const char* spiffs_filename, char** buffer, size_t* size) {
    esp_err_t ret;

    // Try SD card first (user configurations)
    if (g_config_state.sd_available) {
        ret = read_config_file_sd(sd_filename, buffer, size);
        if (ret == ESP_OK) {
            ESP_LOGD(TAG, "Loaded config from SD card: %s", sd_filename);
            return ESP_OK;
        } else if (ret != ESP_ERR_NOT_FOUND) {
            ESP_LOGW(TAG, "Error reading SD card config %s: %s", sd_filename, esp_err_to_name(ret));
        }
    }

    // Fall back to SPIFFS (default configurations)
    if (g_config_state.spiffs_mounted) {
        ret = read_config_file_spiffs(spiffs_filename, buffer, size);
        if (ret == ESP_OK) {
            ESP_LOGD(TAG, "Loaded config from SPIFFS: %s", spiffs_filename);
            return ESP_OK;
        } else {
            ESP_LOGE(TAG, "Failed to read SPIFFS config %s: %s", spiffs_filename, esp_err_to_name(ret));
        }
    }

    ESP_LOGE(TAG, "No config file found: SD=%s, SPIFFS=%s", sd_filename, spiffs_filename);
    return ESP_ERR_NOT_FOUND;
}

// Initialize SPIFFS for configuration files
esp_err_t config_loader_init(void) {
    esp_vfs_spiffs_conf_t conf = {
        .base_path = SPIFFS_MOUNT_POINT,
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true
    };

    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount or format SPIFFS filesystem");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "Failed to find SPIFFS partition");
        } else {
            ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
        g_config_state.spiffs_mounted = false;
        return ret;
    }

    size_t total = 0, used = 0;
    ret = esp_spiffs_info(NULL, &total, &used);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "SPIFFS partition size: total: %d, used: %d", total, used);
    }

    g_config_state.spiffs_mounted = true;

    // Check if SD card is available
    sd_card_info_t sd_info;
    ret = sd_storage_get_info(&sd_info);
    if (ret == ESP_OK && sd_info.mounted) {
        g_config_state.sd_available = true;
        ESP_LOGI(TAG, "SD card available for configuration storage");
    } else {
        g_config_state.sd_available = false;
        ESP_LOGI(TAG, "SD card not available, using SPIFFS only");
    }

    ESP_LOGI(TAG, "Configuration loader initialized (SPIFFS: %s, SD: %s)",
             g_config_state.spiffs_mounted ? "YES" : "NO",
             g_config_state.sd_available ? "YES" : "NO");

    return ESP_OK;
}

esp_err_t config_loader_load_system_config(system_config_t* config) {
    // Initialize with default values first
    system_config_t default_config = {
        .name = "ESP32-P4 Visual SLAM Navigation System",
        .version = "1.0.0",
        .description = "Real-time Visual SLAM navigation system for autonomous drone",
        .debug_mode = false,
        .log_level = "INFO",
        .enable_watchdog = true,
        .watchdog_timeout_ms = 5000,
        .enable_statistics = true,
        .statistics_interval_ms = 1000,
        // Component defaults
        .camera_enabled = true,
        .gps_enabled = true,
        .imu_enabled = true,
        .slam_enabled = true,
        .sensor_fusion_enabled = true,
        .msp_enabled = true,
        .sd_storage_enabled = true,
        .web_server_enabled = false,
        // Performance defaults
        .target_fps = 30,
        .max_processing_time_ms = 33,
        .memory_monitoring = true,
        .cpu_monitoring = true,
        .enable_performance_logging = false,
        // Safety defaults
        .emergency_stop_enabled = true,
        .failsafe_timeout_ms = 10000,
        .maximum_altitude_m = 100.0,
        .minimum_altitude_m = 0.5,
        .maximum_velocity_ms = 15.0,
        .geofence_enabled = false,
        // Communication defaults
        .uart_buffer_size = 2048,
        .i2c_clock_speed = 400000,
        .spi_clock_speed = 10000000,
        .communication_timeout_ms = 5000
    };

    // Copy defaults to output config
    memcpy(config, &default_config, sizeof(system_config_t));

    char* buffer = NULL;
    size_t size = 0;
    bool sd_file_exists = false;

    // Try to read from SD card first
    if (g_config_state.sd_available) {
        esp_err_t sd_ret = read_config_file_sd(SYSTEM_CONFIG_FILE_SD, &buffer, &size);
        if (sd_ret == ESP_OK) {
            sd_file_exists = true;
            ESP_LOGD(TAG, "Loaded system config from SD card");
        } else if (sd_ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGI(TAG, "System config not found on SD card, will save defaults");
        } else {
            ESP_LOGW(TAG, "Error reading SD card system config: %s", esp_err_to_name(sd_ret));
        }
    }

    // If SD card read failed, try SPIFFS
    if (!sd_file_exists && buffer == NULL) {
        esp_err_t spiffs_ret = read_config_file_spiffs(SYSTEM_CONFIG_FILE_SPIFFS, &buffer, &size);
        if (spiffs_ret == ESP_OK) {
            ESP_LOGD(TAG, "Loaded system config from SPIFFS");
        } else {
            ESP_LOGW(TAG, "Failed to read system config from SPIFFS: %s", esp_err_to_name(spiffs_ret));
            // No config file found anywhere, save defaults to SD card if available
            if (g_config_state.sd_available) {
                esp_err_t save_ret = save_system_config_to_sd(&default_config);
                if (save_ret == ESP_OK) {
                    ESP_LOGI(TAG, "✅ Saved default system config to SD card");
                } else {
                    ESP_LOGW(TAG, "Failed to save default system config to SD card: %s", esp_err_to_name(save_ret));
                }
            }
            return ESP_OK; // Return OK since we have defaults
        }
    }

    // If we got here, we have a buffer with config data
    if (buffer == NULL) {
        ESP_LOGE(TAG, "No config buffer available, using defaults");
        // Save defaults to SD card if available
        if (g_config_state.sd_available) {
            esp_err_t save_ret = save_system_config_to_sd(&default_config);
            if (save_ret == ESP_OK) {
                ESP_LOGI(TAG, "✅ Saved default system config to SD card");
            } else {
                ESP_LOGW(TAG, "Failed to save default system config to SD card: %s", esp_err_to_name(save_ret));
            }
        }
        return ESP_OK;
    }

    cJSON* root = cJSON_Parse(buffer);
    free(buffer);

    if (root == NULL) {
        ESP_LOGE(TAG, "Failed to parse system config JSON, using defaults");
        // Save defaults to SD card if available and file didn't exist
        if (g_config_state.sd_available && !sd_file_exists) {
            esp_err_t save_ret = save_system_config_to_sd(&default_config);
            if (save_ret == ESP_OK) {
                ESP_LOGI(TAG, "✅ Saved default system config to SD card");
            } else {
                ESP_LOGW(TAG, "Failed to save default system config to SD card: %s", esp_err_to_name(save_ret));
            }
        }
        return ESP_OK; // Return OK since we have defaults
    }

    cJSON* system = cJSON_GetObjectItem(root, "system");
    if (system == NULL) {
        ESP_LOGE(TAG, "No 'system' object in config JSON, using defaults");
        cJSON_Delete(root);
        // Save defaults to SD card if available and file didn't exist
        if (g_config_state.sd_available && !sd_file_exists) {
            esp_err_t save_ret = save_system_config_to_sd(&default_config);
            if (save_ret == ESP_OK) {
                ESP_LOGI(TAG, "✅ Saved default system config to SD card");
            } else {
                ESP_LOGW(TAG, "Failed to save default system config to SD card: %s", esp_err_to_name(save_ret));
            }
        }
        return ESP_OK; // Return OK since we have defaults
    }

    // Load system configuration
    cJSON* item = cJSON_GetObjectItem(system, "name");
    if (item && cJSON_IsString(item)) {
        strncpy(config->name, item->valuestring, sizeof(config->name) - 1);
    }

    item = cJSON_GetObjectItem(system, "version");
    if (item && cJSON_IsString(item)) {
        strncpy(config->version, item->valuestring, sizeof(config->version) - 1);
    }

    item = cJSON_GetObjectItem(system, "description");
    if (item && cJSON_IsString(item)) {
        strncpy(config->description, item->valuestring, sizeof(config->description) - 1);
    }

    item = cJSON_GetObjectItem(system, "debug_mode");
    if (item && cJSON_IsBool(item)) {
        config->debug_mode = cJSON_IsTrue(item);
    }

    item = cJSON_GetObjectItem(system, "log_level");
    if (item && cJSON_IsString(item)) {
        strncpy(config->log_level, item->valuestring, sizeof(config->log_level) - 1);
    }

    item = cJSON_GetObjectItem(system, "enable_watchdog");
    if (item && cJSON_IsBool(item)) {
        config->enable_watchdog = cJSON_IsTrue(item);
    }

    item = cJSON_GetObjectItem(system, "watchdog_timeout_ms");
    if (item && cJSON_IsNumber(item)) {
        config->watchdog_timeout_ms = (uint32_t)item->valuedouble;
    }

    item = cJSON_GetObjectItem(system, "enable_statistics");
    if (item && cJSON_IsBool(item)) {
        config->enable_statistics = cJSON_IsTrue(item);
    }

    item = cJSON_GetObjectItem(system, "statistics_interval_ms");
    if (item && cJSON_IsNumber(item)) {
        config->statistics_interval_ms = (uint32_t)item->valuedouble;
    }

    // Load components configuration
    cJSON* components = cJSON_GetObjectItem(root, "components");
    if (components) {
        cJSON* comp_item = cJSON_GetObjectItem(components, "camera_enabled");
        if (comp_item && cJSON_IsBool(comp_item)) {
            config->camera_enabled = cJSON_IsTrue(comp_item);
        }

        comp_item = cJSON_GetObjectItem(components, "gps_enabled");
        if (comp_item && cJSON_IsBool(comp_item)) {
            config->gps_enabled = cJSON_IsTrue(comp_item);
        }

        comp_item = cJSON_GetObjectItem(components, "imu_enabled");
        if (comp_item && cJSON_IsBool(comp_item)) {
            config->imu_enabled = cJSON_IsTrue(comp_item);
        }

        comp_item = cJSON_GetObjectItem(components, "slam_enabled");
        if (comp_item && cJSON_IsBool(comp_item)) {
            config->slam_enabled = cJSON_IsTrue(comp_item);
        }

        comp_item = cJSON_GetObjectItem(components, "sensor_fusion_enabled");
        if (comp_item && cJSON_IsBool(comp_item)) {
            config->sensor_fusion_enabled = cJSON_IsTrue(comp_item);
        }

        comp_item = cJSON_GetObjectItem(components, "msp_enabled");
        if (comp_item && cJSON_IsBool(comp_item)) {
            config->msp_enabled = cJSON_IsTrue(comp_item);
        }

        comp_item = cJSON_GetObjectItem(components, "sd_storage_enabled");
        if (comp_item && cJSON_IsBool(comp_item)) {
            config->sd_storage_enabled = cJSON_IsTrue(comp_item);
        }

        comp_item = cJSON_GetObjectItem(components, "web_server_enabled");
        if (comp_item && cJSON_IsBool(comp_item)) {
            config->web_server_enabled = cJSON_IsTrue(comp_item);
        }
    }

    // Load performance configuration
    cJSON* performance = cJSON_GetObjectItem(root, "performance");
    if (performance) {
        cJSON* perf_item = cJSON_GetObjectItem(performance, "target_fps");
        if (perf_item && cJSON_IsNumber(perf_item)) {
            config->target_fps = (uint32_t)perf_item->valuedouble;
        }

        perf_item = cJSON_GetObjectItem(performance, "max_processing_time_ms");
        if (perf_item && cJSON_IsNumber(perf_item)) {
            config->max_processing_time_ms = (uint32_t)perf_item->valuedouble;
        }

        perf_item = cJSON_GetObjectItem(performance, "memory_monitoring");
        if (perf_item && cJSON_IsBool(perf_item)) {
            config->memory_monitoring = cJSON_IsTrue(perf_item);
        }

        perf_item = cJSON_GetObjectItem(performance, "cpu_monitoring");
        if (perf_item && cJSON_IsBool(perf_item)) {
            config->cpu_monitoring = cJSON_IsTrue(perf_item);
        }

        perf_item = cJSON_GetObjectItem(performance, "enable_performance_logging");
        if (perf_item && cJSON_IsBool(perf_item)) {
            config->enable_performance_logging = cJSON_IsTrue(perf_item);
        }
    }

    // Load safety configuration
    cJSON* safety = cJSON_GetObjectItem(root, "safety");
    if (safety) {
        cJSON* safe_item = cJSON_GetObjectItem(safety, "emergency_stop_enabled");
        if (safe_item && cJSON_IsBool(safe_item)) {
            config->emergency_stop_enabled = cJSON_IsTrue(safe_item);
        }

        safe_item = cJSON_GetObjectItem(safety, "failsafe_timeout_ms");
        if (safe_item && cJSON_IsNumber(safe_item)) {
            config->failsafe_timeout_ms = (uint32_t)safe_item->valuedouble;
        }

        safe_item = cJSON_GetObjectItem(safety, "maximum_altitude_m");
        if (safe_item && cJSON_IsNumber(safe_item)) {
            config->maximum_altitude_m = (float)safe_item->valuedouble;
        }

        safe_item = cJSON_GetObjectItem(safety, "minimum_altitude_m");
        if (safe_item && cJSON_IsNumber(safe_item)) {
            config->minimum_altitude_m = (float)safe_item->valuedouble;
        }

        safe_item = cJSON_GetObjectItem(safety, "maximum_velocity_ms");
        if (safe_item && cJSON_IsNumber(safe_item)) {
            config->maximum_velocity_ms = (float)safe_item->valuedouble;
        }

        safe_item = cJSON_GetObjectItem(safety, "geofence_enabled");
        if (safe_item && cJSON_IsBool(safe_item)) {
            config->geofence_enabled = cJSON_IsTrue(safe_item);
        }
    }

    // Load communication configuration
    cJSON* communication = cJSON_GetObjectItem(root, "communication");
    if (communication) {
        cJSON* comm_item = cJSON_GetObjectItem(communication, "uart_buffer_size");
        if (comm_item && cJSON_IsNumber(comm_item)) {
            config->uart_buffer_size = (uint32_t)comm_item->valuedouble;
        }

        comm_item = cJSON_GetObjectItem(communication, "i2c_clock_speed");
        if (comm_item && cJSON_IsNumber(comm_item)) {
            config->i2c_clock_speed = (uint32_t)comm_item->valuedouble;
        }

        comm_item = cJSON_GetObjectItem(communication, "spi_clock_speed");
        if (comm_item && cJSON_IsNumber(comm_item)) {
            config->spi_clock_speed = (uint32_t)comm_item->valuedouble;
        }

        comm_item = cJSON_GetObjectItem(communication, "communication_timeout_ms");
        if (comm_item && cJSON_IsNumber(comm_item)) {
            config->communication_timeout_ms = (uint32_t)comm_item->valuedouble;
        }
    }

    cJSON_Delete(root);

    // If we loaded from SPIFFS but SD card is available and file didn't exist, save to SD card
    if (g_config_state.sd_available && !sd_file_exists) {
        esp_err_t save_ret = save_system_config_to_sd(config);
        if (save_ret == ESP_OK) {
            ESP_LOGI(TAG, "✅ Saved loaded system config to SD card");
        } else {
            ESP_LOGW(TAG, "Failed to save system config to SD card: %s", esp_err_to_name(save_ret));
        }
    }

    return ESP_OK;
}

esp_err_t config_loader_load_gps_config(extended_gps_config_t* config) {
    // Initialize with default values first
    extended_gps_config_t default_config = {
        .base_config = {
            .uart_port = UART_NUM_1,
            .baud_rate = 115200,
            .tx_pin = 15,
            .rx_pin = 16,
            .update_rate_hz = 10
        },
        .enable_sbas = true,
        .enable_differential = false,
        .timeout_ms = 5000,
        .auto_reconnect = true,
        .reconnect_attempts = 3,
        .reconnect_delay_ms = 1000,
        .minimum_satellites = 4,
        .maximum_hdop = 5.0f,
        .maximum_speed_ms = 50.0f,
        .speed_filter_enabled = true,
        .altitude_filter_enabled = true,
        .minimum_altitude_m = -100.0f,
        .maximum_altitude_m = 10000.0f,
        .dynamic_model = 0,
        .fix_mode = 0,
        .enable_gps = true,
        .enable_glonass = true,
        .enable_galileo = false,
        .enable_beidou = false,
        .time_pulse_enabled = false,
        .antenna_detection = false
    };

    // Copy defaults to output config
    memcpy(config, &default_config, sizeof(extended_gps_config_t));

    char* buffer = NULL;
    size_t size = 0;

    esp_err_t ret = read_config_file_with_fallback(GPS_CONFIG_FILE_SD, GPS_CONFIG_FILE_SPIFFS, &buffer, &size);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to read GPS config file, using defaults: %s", esp_err_to_name(ret));
        return ESP_OK; // Return OK since we have defaults
    }

    cJSON* root = cJSON_Parse(buffer);
    free(buffer);

    if (root == NULL) {
        ESP_LOGE(TAG, "Failed to parse GPS config JSON, using defaults");
        return ESP_OK; // Return OK since we have defaults
    }

    // Load GPS configuration
    cJSON* gps = cJSON_GetObjectItem(root, "gps");
    if (gps) {
        cJSON* item = cJSON_GetObjectItem(gps, "uart_port");
        if (item && cJSON_IsNumber(item)) {
            config->base_config.uart_port = (uint32_t)item->valuedouble;
        }

        item = cJSON_GetObjectItem(gps, "baud_rate");
        if (item && cJSON_IsNumber(item)) {
            config->base_config.baud_rate = (uint32_t)item->valuedouble;
        }

        item = cJSON_GetObjectItem(gps, "tx_pin");
        if (item && cJSON_IsNumber(item)) {
            config->base_config.tx_pin = (uint32_t)item->valuedouble;
        }

        item = cJSON_GetObjectItem(gps, "rx_pin");
        if (item && cJSON_IsNumber(item)) {
            config->base_config.rx_pin = (uint32_t)item->valuedouble;
        }

        item = cJSON_GetObjectItem(gps, "update_rate_hz");
        if (item && cJSON_IsNumber(item)) {
            config->base_config.update_rate_hz = (uint32_t)item->valuedouble;
        }

        item = cJSON_GetObjectItem(gps, "enable_sbas");
        if (item && cJSON_IsBool(item)) {
            config->enable_sbas = cJSON_IsTrue(item);
        }

        item = cJSON_GetObjectItem(gps, "enable_differential");
        if (item && cJSON_IsBool(item)) {
            config->enable_differential = cJSON_IsTrue(item);
        }

        item = cJSON_GetObjectItem(gps, "timeout_ms");
        if (item && cJSON_IsNumber(item)) {
            config->timeout_ms = (uint32_t)item->valuedouble;
        }

        item = cJSON_GetObjectItem(gps, "auto_reconnect");
        if (item && cJSON_IsBool(item)) {
            config->auto_reconnect = cJSON_IsTrue(item);
        }

        item = cJSON_GetObjectItem(gps, "reconnect_attempts");
        if (item && cJSON_IsNumber(item)) {
            config->reconnect_attempts = (uint32_t)item->valuedouble;
        }

        item = cJSON_GetObjectItem(gps, "reconnect_delay_ms");
        if (item && cJSON_IsNumber(item)) {
            config->reconnect_delay_ms = (uint32_t)item->valuedouble;
        }
    }

    // Load GPS filters
    cJSON* filters = cJSON_GetObjectItem(root, "gps_filters");
    if (filters) {
        cJSON* item = cJSON_GetObjectItem(filters, "minimum_satellites");
        if (item && cJSON_IsNumber(item)) {
            config->minimum_satellites = (uint8_t)item->valuedouble;
        }

        item = cJSON_GetObjectItem(filters, "maximum_hdop");
        if (item && cJSON_IsNumber(item)) {
            config->maximum_hdop = (float)item->valuedouble;
        }

        item = cJSON_GetObjectItem(filters, "maximum_speed_ms");
        if (item && cJSON_IsNumber(item)) {
            config->maximum_speed_ms = (float)item->valuedouble;
        }

        item = cJSON_GetObjectItem(filters, "speed_filter_enabled");
        if (item && cJSON_IsBool(item)) {
            config->speed_filter_enabled = cJSON_IsTrue(item);
        }

        item = cJSON_GetObjectItem(filters, "altitude_filter_enabled");
        if (item && cJSON_IsBool(item)) {
            config->altitude_filter_enabled = cJSON_IsTrue(item);
        }

        item = cJSON_GetObjectItem(filters, "minimum_altitude_m");
        if (item && cJSON_IsNumber(item)) {
            config->minimum_altitude_m = (float)item->valuedouble;
        }

        item = cJSON_GetObjectItem(filters, "maximum_altitude_m");
        if (item && cJSON_IsNumber(item)) {
            config->maximum_altitude_m = (float)item->valuedouble;
        }
    }

    // Load GPS advanced settings
    cJSON* advanced = cJSON_GetObjectItem(root, "gps_advanced");
    if (advanced) {
        cJSON* item = cJSON_GetObjectItem(advanced, "dynamic_model");
        if (item && cJSON_IsNumber(item)) {
            config->dynamic_model = (uint8_t)item->valuedouble;
        }

        item = cJSON_GetObjectItem(advanced, "fix_mode");
        if (item && cJSON_IsNumber(item)) {
            config->fix_mode = (uint8_t)item->valuedouble;
        }

        item = cJSON_GetObjectItem(advanced, "enable_gps");
        if (item && cJSON_IsBool(item)) {
            config->enable_gps = cJSON_IsTrue(item);
        }

        item = cJSON_GetObjectItem(advanced, "enable_glonass");
        if (item && cJSON_IsBool(item)) {
            config->enable_glonass = cJSON_IsTrue(item);
        }

        item = cJSON_GetObjectItem(advanced, "enable_galileo");
        if (item && cJSON_IsBool(item)) {
            config->enable_galileo = cJSON_IsTrue(item);
        }

        item = cJSON_GetObjectItem(advanced, "enable_beidou");
        if (item && cJSON_IsBool(item)) {
            config->enable_beidou = cJSON_IsTrue(item);
        }

        item = cJSON_GetObjectItem(advanced, "time_pulse_enabled");
        if (item && cJSON_IsBool(item)) {
            config->time_pulse_enabled = cJSON_IsTrue(item);
        }

        item = cJSON_GetObjectItem(advanced, "antenna_detection");
        if (item && cJSON_IsBool(item)) {
            config->antenna_detection = cJSON_IsTrue(item);
        }
    }

    cJSON_Delete(root);
    return ESP_OK;
}

esp_err_t config_loader_load_msp_config(extended_msp_config_t* config) {
    // Initialize with default values first
    extended_msp_config_t default_config = {
        .base_config = {
            .uart_port = UART_NUM_2,
            .baud_rate = 115200,
            .tx_pin = 17,
            .rx_pin = 18,
            .timeout_ms = 1000,
            .auto_reconnect = true
        },
        .status_request_interval_ms = 1000,
        .navigation_status_interval_ms = 500,
        .attitude_request_interval_ms = 100,
        .altitude_request_interval_ms = 200,
        .gps_request_interval_ms = 1000,
        .analog_request_interval_ms = 5000,
        .default_mode = 0,
        .waypoint_radius_m = 2.0f,
        .waypoint_hold_time_ms = 5000,
        .cruise_speed_ms = 5.0f,
        .max_climb_rate_ms = 2.0f,
        .max_descent_rate_ms = 1.5f,
        .rtl_altitude_m = 10.0f,
        .land_speed_ms = 1.0f,
        .emergency_stop_enabled = true,
        .failsafe_enabled = true,
        .low_battery_voltage = 3.3f,
        .critical_battery_voltage = 3.0f,
        .maximum_roll_angle_deg = 45,
        .maximum_pitch_angle_deg = 45
    };

    // Copy defaults to output config
    memcpy(config, &default_config, sizeof(extended_msp_config_t));

    char* buffer = NULL;
    size_t size = 0;

    esp_err_t ret = read_config_file_with_fallback(MSP_CONFIG_FILE_SD, MSP_CONFIG_FILE_SPIFFS, &buffer, &size);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to read MSP config file, using defaults: %s", esp_err_to_name(ret));
        return ESP_OK; // Return OK since we have defaults
    }

    cJSON* root = cJSON_Parse(buffer);
    free(buffer);

    if (root == NULL) {
        ESP_LOGE(TAG, "Failed to parse MSP config JSON, using defaults");
        return ESP_OK; // Return OK since we have defaults
    }

    // Load MSP configuration
    cJSON* msp = cJSON_GetObjectItem(root, "msp");
    if (msp) {
        cJSON* item = cJSON_GetObjectItem(msp, "uart_port");
        if (item && cJSON_IsNumber(item)) {
            config->base_config.uart_port = (uart_port_t)item->valuedouble;
        }

        item = cJSON_GetObjectItem(msp, "baud_rate");
        if (item && cJSON_IsNumber(item)) {
            config->base_config.baud_rate = (int)item->valuedouble;
        }

        item = cJSON_GetObjectItem(msp, "tx_pin");
        if (item && cJSON_IsNumber(item)) {
            config->base_config.tx_pin = (int)item->valuedouble;
        }

        item = cJSON_GetObjectItem(msp, "rx_pin");
        if (item && cJSON_IsNumber(item)) {
            config->base_config.rx_pin = (int)item->valuedouble;
        }

        item = cJSON_GetObjectItem(msp, "timeout_ms");
        if (item && cJSON_IsNumber(item)) {
            config->base_config.timeout_ms = (uint32_t)item->valuedouble;
        }

        item = cJSON_GetObjectItem(msp, "auto_reconnect");
        if (item && cJSON_IsBool(item)) {
            config->base_config.auto_reconnect = cJSON_IsTrue(item);
        }
    }

    // Load MSP commands configuration
    cJSON* commands = cJSON_GetObjectItem(root, "msp_commands");
    if (commands) {
        cJSON* item = cJSON_GetObjectItem(commands, "status_request_interval_ms");
        if (item && cJSON_IsNumber(item)) {
            config->status_request_interval_ms = (uint32_t)item->valuedouble;
        }

        item = cJSON_GetObjectItem(commands, "navigation_status_interval_ms");
        if (item && cJSON_IsNumber(item)) {
            config->navigation_status_interval_ms = (uint32_t)item->valuedouble;
        }

        item = cJSON_GetObjectItem(commands, "attitude_request_interval_ms");
        if (item && cJSON_IsNumber(item)) {
            config->attitude_request_interval_ms = (uint32_t)item->valuedouble;
        }

        item = cJSON_GetObjectItem(commands, "altitude_request_interval_ms");
        if (item && cJSON_IsNumber(item)) {
            config->altitude_request_interval_ms = (uint32_t)item->valuedouble;
        }

        item = cJSON_GetObjectItem(commands, "gps_request_interval_ms");
        if (item && cJSON_IsNumber(item)) {
            config->gps_request_interval_ms = (uint32_t)item->valuedouble;
        }

        item = cJSON_GetObjectItem(commands, "analog_request_interval_ms");
        if (item && cJSON_IsNumber(item)) {
            config->analog_request_interval_ms = (uint32_t)item->valuedouble;
        }
    }

    cJSON_Delete(root);
    return ESP_OK;
}

esp_err_t config_loader_load_slam_config(extended_slam_config_t* config) {
    // Initialize with default values first
    extended_slam_config_t default_config = {
        .base_config = {
            .max_features = 500,
            .fast_threshold = 20.0f,
            .levels = 8,
            .scale_factor = 1.2f,
            .max_keypoints_per_level = 100,
            .use_harris_detector = false,
            .harris_k = 0.04f,
            .descriptor_distance_threshold = 50,
            .match_threshold = 0.7f,
            .min_tracked_features = 30,
            .keyframe_distance_threshold = 1.0f,
            .keyframe_angle_threshold = 0.2f,
            .max_keyframes = 50,
            .enable_loop_closure = true,
            .loop_closure_threshold = 0.8f,
            .map_optimization_enabled = true
        },
        .tracking_confidence_threshold = 0.6f,
        .relocalization_enabled = true,
        .outlier_rejection_enabled = true,
        .bundle_adjustment_enabled = true,
        .feature_tracking_method = "ORB",
        .motion_model = "CONSTANT_VELOCITY",
        .map_save_enabled = true,
        .map_load_enabled = true,
        .map_compression_enabled = true,
        .max_map_points = 10000,
        .map_point_culling_threshold = 0.1f,
        .keyframe_culling_threshold = 0.5f,
        .processing_threads = 2,
        .feature_extraction_threads = 1,
        .optimization_threads = 1,
        .memory_pool_size_kb = 1024,
        .feature_cache_size = 1000
    };

    // Copy defaults to output config
    memcpy(config, &default_config, sizeof(extended_slam_config_t));

    char* buffer = NULL;
    size_t size = 0;

    esp_err_t ret = read_config_file_with_fallback(SLAM_CONFIG_FILE_SD, SLAM_CONFIG_FILE_SPIFFS, &buffer, &size);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to read SLAM config file, using defaults: %s", esp_err_to_name(ret));
        return ESP_OK; // Return OK since we have defaults
    }

    cJSON* root = cJSON_Parse(buffer);
    free(buffer);

    if (root == NULL) {
        ESP_LOGE(TAG, "Failed to parse SLAM config JSON, using defaults");
        return ESP_OK; // Return OK since we have defaults
    }

    // Load SLAM configuration
    cJSON* slam = cJSON_GetObjectItem(root, "slam");
    if (slam) {
        cJSON* item = cJSON_GetObjectItem(slam, "max_features");
        if (item && cJSON_IsNumber(item)) {
            config->base_config.max_features = (uint32_t)item->valuedouble;
        }

        item = cJSON_GetObjectItem(slam, "fast_threshold");
        if (item && cJSON_IsNumber(item)) {
            config->base_config.fast_threshold = (float)item->valuedouble;
        }

        item = cJSON_GetObjectItem(slam, "levels");
        if (item && cJSON_IsNumber(item)) {
            config->base_config.levels = (uint32_t)item->valuedouble;
        }

        item = cJSON_GetObjectItem(slam, "scale_factor");
        if (item && cJSON_IsNumber(item)) {
            config->base_config.scale_factor = (float)item->valuedouble;
        }

        item = cJSON_GetObjectItem(slam, "max_keypoints_per_level");
        if (item && cJSON_IsNumber(item)) {
            config->base_config.max_keypoints_per_level = (uint32_t)item->valuedouble;
        }

        item = cJSON_GetObjectItem(slam, "use_harris_detector");
        if (item && cJSON_IsBool(item)) {
            config->base_config.use_harris_detector = cJSON_IsTrue(item);
        }

        item = cJSON_GetObjectItem(slam, "harris_k");
        if (item && cJSON_IsNumber(item)) {
            config->base_config.harris_k = (float)item->valuedouble;
        }

        item = cJSON_GetObjectItem(slam, "descriptor_distance_threshold");
        if (item && cJSON_IsNumber(item)) {
            config->base_config.descriptor_distance_threshold = (uint32_t)item->valuedouble;
        }

        item = cJSON_GetObjectItem(slam, "match_threshold");
        if (item && cJSON_IsNumber(item)) {
            config->base_config.match_threshold = (float)item->valuedouble;
        }

        item = cJSON_GetObjectItem(slam, "min_tracked_features");
        if (item && cJSON_IsNumber(item)) {
            config->base_config.min_tracked_features = (uint32_t)item->valuedouble;
        }

        item = cJSON_GetObjectItem(slam, "keyframe_distance_threshold");
        if (item && cJSON_IsNumber(item)) {
            config->base_config.keyframe_distance_threshold = (float)item->valuedouble;
        }

        item = cJSON_GetObjectItem(slam, "keyframe_angle_threshold");
        if (item && cJSON_IsNumber(item)) {
            config->base_config.keyframe_angle_threshold = (float)item->valuedouble;
        }

        item = cJSON_GetObjectItem(slam, "max_keyframes");
        if (item && cJSON_IsNumber(item)) {
            config->base_config.max_keyframes = (uint32_t)item->valuedouble;
        }

        item = cJSON_GetObjectItem(slam, "enable_loop_closure");
        if (item && cJSON_IsBool(item)) {
            config->base_config.enable_loop_closure = cJSON_IsTrue(item);
        }

        item = cJSON_GetObjectItem(slam, "loop_closure_threshold");
        if (item && cJSON_IsNumber(item)) {
            config->base_config.loop_closure_threshold = (float)item->valuedouble;
        }

        item = cJSON_GetObjectItem(slam, "map_optimization_enabled");
        if (item && cJSON_IsBool(item)) {
            config->base_config.map_optimization_enabled = cJSON_IsTrue(item);
        }
    }

    // Load extended SLAM parameters
    cJSON* slam_extended = cJSON_GetObjectItem(root, "slam_extended");
    if (slam_extended) {
        cJSON* item = cJSON_GetObjectItem(slam_extended, "tracking_confidence_threshold");
        if (item && cJSON_IsNumber(item)) {
            config->tracking_confidence_threshold = (float)item->valuedouble;
        }

        item = cJSON_GetObjectItem(slam_extended, "relocalization_enabled");
        if (item && cJSON_IsBool(item)) {
            config->relocalization_enabled = cJSON_IsTrue(item);
        }

        item = cJSON_GetObjectItem(slam_extended, "outlier_rejection_enabled");
        if (item && cJSON_IsBool(item)) {
            config->outlier_rejection_enabled = cJSON_IsTrue(item);
        }

        item = cJSON_GetObjectItem(slam_extended, "bundle_adjustment_enabled");
        if (item && cJSON_IsBool(item)) {
            config->bundle_adjustment_enabled = cJSON_IsTrue(item);
        }

        item = cJSON_GetObjectItem(slam_extended, "feature_tracking_method");
        if (item && cJSON_IsString(item)) {
            strncpy(config->feature_tracking_method, item->valuestring, sizeof(config->feature_tracking_method) - 1);
        }

        item = cJSON_GetObjectItem(slam_extended, "motion_model");
        if (item && cJSON_IsString(item)) {
            strncpy(config->motion_model, item->valuestring, sizeof(config->motion_model) - 1);
        }

        item = cJSON_GetObjectItem(slam_extended, "map_save_enabled");
        if (item && cJSON_IsBool(item)) {
            config->map_save_enabled = cJSON_IsTrue(item);
        }

        item = cJSON_GetObjectItem(slam_extended, "map_load_enabled");
        if (item && cJSON_IsBool(item)) {
            config->map_load_enabled = cJSON_IsTrue(item);
        }

        item = cJSON_GetObjectItem(slam_extended, "map_compression_enabled");
        if (item && cJSON_IsBool(item)) {
            config->map_compression_enabled = cJSON_IsTrue(item);
        }

        item = cJSON_GetObjectItem(slam_extended, "max_map_points");
        if (item && cJSON_IsNumber(item)) {
            config->max_map_points = (uint32_t)item->valuedouble;
        }

        item = cJSON_GetObjectItem(slam_extended, "map_point_culling_threshold");
        if (item && cJSON_IsNumber(item)) {
            config->map_point_culling_threshold = (float)item->valuedouble;
        }

        item = cJSON_GetObjectItem(slam_extended, "keyframe_culling_threshold");
        if (item && cJSON_IsNumber(item)) {
            config->keyframe_culling_threshold = (float)item->valuedouble;
        }

        item = cJSON_GetObjectItem(slam_extended, "processing_threads");
        if (item && cJSON_IsNumber(item)) {
            config->processing_threads = (uint8_t)item->valuedouble;
        }

        item = cJSON_GetObjectItem(slam_extended, "feature_extraction_threads");
        if (item && cJSON_IsNumber(item)) {
            config->feature_extraction_threads = (uint8_t)item->valuedouble;
        }

        item = cJSON_GetObjectItem(slam_extended, "optimization_threads");
        if (item && cJSON_IsNumber(item)) {
            config->optimization_threads = (uint8_t)item->valuedouble;
        }

        item = cJSON_GetObjectItem(slam_extended, "memory_pool_size_kb");
        if (item && cJSON_IsNumber(item)) {
            config->memory_pool_size_kb = (uint32_t)item->valuedouble;
        }

        item = cJSON_GetObjectItem(slam_extended, "feature_cache_size");
        if (item && cJSON_IsNumber(item)) {
            config->feature_cache_size = (uint32_t)item->valuedouble;
        }
    }

    cJSON_Delete(root);
    return ESP_OK;
}

esp_err_t config_loader_load_fusion_config(extended_fusion_config_t* config) {
    // Initialize with default values first
    extended_fusion_config_t default_config = {
        .base_config = {
            .position_noise = 0.1f,
            .velocity_noise = 0.1f,
            .attitude_noise = 0.01f,
            .slam_attitude_noise = 0.1f,
            .gps_position_noise = 1.0f,
            .gps_velocity_noise = 0.1f,
            .slam_position_noise = 0.5f,
            .accel_bias_noise = 0.001f,
            .gyro_bias_noise = 0.001f,
            .imu_dt = 0.01f,
            .max_prediction_time = 1.0f,
            .gps_weight = 0.8f,
            .slam_weight = 0.7f,
            .imu_weight = 0.9f
        },
        .gps_rejection_threshold = 3.0f,
        .slam_rejection_threshold = 2.0f,
        .imu_rejection_threshold = 2.0f,
        .outlier_detection_enabled = true,
        .gps_outlier_threshold = 5.0f,
        .slam_outlier_threshold = 3.0f,
        .imu_outlier_threshold = 2.0f,
        .filter_reset_enabled = true,
        .filter_reset_threshold = 10.0f,
        .imu_update_rate_hz = 400,
        .gps_update_rate_hz = 5,
        .slam_update_rate_hz = 30,
        .prediction_rate_hz = 100,
        .correction_timeout_ms = 1000
    };

    // Copy defaults to output config
    memcpy(config, &default_config, sizeof(extended_fusion_config_t));

    char* buffer = NULL;
    size_t size = 0;

    esp_err_t ret = read_config_file_with_fallback(FUSION_CONFIG_FILE_SD, FUSION_CONFIG_FILE_SPIFFS, &buffer, &size);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to read fusion config file, using defaults: %s", esp_err_to_name(ret));
        return ESP_OK; // Return OK since we have defaults
    }

    cJSON* root = cJSON_Parse(buffer);
    free(buffer);

    if (root == NULL) {
        ESP_LOGE(TAG, "Failed to parse fusion config JSON, using defaults");
        return ESP_OK; // Return OK since we have defaults
    }

    // Load fusion configuration
    cJSON* fusion = cJSON_GetObjectItem(root, "fusion");
    if (fusion) {
        cJSON* item = cJSON_GetObjectItem(fusion, "position_noise");
        if (item && cJSON_IsNumber(item)) {
            config->base_config.position_noise = (float)item->valuedouble;
        }

        item = cJSON_GetObjectItem(fusion, "velocity_noise");
        if (item && cJSON_IsNumber(item)) {
            config->base_config.velocity_noise = (float)item->valuedouble;
        }

        item = cJSON_GetObjectItem(fusion, "attitude_noise");
        if (item && cJSON_IsNumber(item)) {
            config->base_config.attitude_noise = (float)item->valuedouble;
        }

        item = cJSON_GetObjectItem(fusion, "slam_attitude_noise");
        if (item && cJSON_IsNumber(item)) {
            config->base_config.slam_attitude_noise = (float)item->valuedouble;
        }

        item = cJSON_GetObjectItem(fusion, "gps_position_noise");
        if (item && cJSON_IsNumber(item)) {
            config->base_config.gps_position_noise = (float)item->valuedouble;
        }

        item = cJSON_GetObjectItem(fusion, "gps_velocity_noise");
        if (item && cJSON_IsNumber(item)) {
            config->base_config.gps_velocity_noise = (float)item->valuedouble;
        }

        item = cJSON_GetObjectItem(fusion, "slam_position_noise");
        if (item && cJSON_IsNumber(item)) {
            config->base_config.slam_position_noise = (float)item->valuedouble;
        }

        item = cJSON_GetObjectItem(fusion, "accel_bias_noise");
        if (item && cJSON_IsNumber(item)) {
            config->base_config.accel_bias_noise = (float)item->valuedouble;
        }

        item = cJSON_GetObjectItem(fusion, "gyro_bias_noise");
        if (item && cJSON_IsNumber(item)) {
            config->base_config.gyro_bias_noise = (float)item->valuedouble;
        }

        item = cJSON_GetObjectItem(fusion, "imu_dt");
        if (item && cJSON_IsNumber(item)) {
            config->base_config.imu_dt = (float)item->valuedouble;
        }

        item = cJSON_GetObjectItem(fusion, "max_prediction_time");
        if (item && cJSON_IsNumber(item)) {
            config->base_config.max_prediction_time = (float)item->valuedouble;
        }
    }

    // Load fusion weights
    cJSON* weights = cJSON_GetObjectItem(root, "fusion_weights");
    if (weights) {
        cJSON* item = cJSON_GetObjectItem(weights, "gps_weight");
        if (item && cJSON_IsNumber(item)) {
            config->base_config.gps_weight = (float)item->valuedouble;
        }

        item = cJSON_GetObjectItem(weights, "slam_weight");
        if (item && cJSON_IsNumber(item)) {
            config->base_config.slam_weight = (float)item->valuedouble;
        }

        item = cJSON_GetObjectItem(weights, "imu_weight");
        if (item && cJSON_IsNumber(item)) {
            config->base_config.imu_weight = (float)item->valuedouble;
        }

        item = cJSON_GetObjectItem(weights, "gps_rejection_threshold");
        if (item && cJSON_IsNumber(item)) {
            config->gps_rejection_threshold = (float)item->valuedouble;
        }

        item = cJSON_GetObjectItem(weights, "slam_rejection_threshold");
        if (item && cJSON_IsNumber(item)) {
            config->slam_rejection_threshold = (float)item->valuedouble;
        }

        item = cJSON_GetObjectItem(weights, "imu_rejection_threshold");
        if (item && cJSON_IsNumber(item)) {
            config->imu_rejection_threshold = (float)item->valuedouble;
        }
    }

    // Load fusion filters
    cJSON* filters = cJSON_GetObjectItem(root, "fusion_filters");
    if (filters) {
        cJSON* item = cJSON_GetObjectItem(filters, "outlier_detection_enabled");
        if (item && cJSON_IsBool(item)) {
            config->outlier_detection_enabled = cJSON_IsTrue(item);
        }

        item = cJSON_GetObjectItem(filters, "gps_outlier_threshold");
        if (item && cJSON_IsNumber(item)) {
            config->gps_outlier_threshold = (float)item->valuedouble;
        }

        item = cJSON_GetObjectItem(filters, "slam_outlier_threshold");
        if (item && cJSON_IsNumber(item)) {
            config->slam_outlier_threshold = (float)item->valuedouble;
        }

        item = cJSON_GetObjectItem(filters, "imu_outlier_threshold");
        if (item && cJSON_IsNumber(item)) {
            config->imu_outlier_threshold = (float)item->valuedouble;
        }

        item = cJSON_GetObjectItem(filters, "filter_reset_enabled");
        if (item && cJSON_IsBool(item)) {
            config->filter_reset_enabled = cJSON_IsTrue(item);
        }

        item = cJSON_GetObjectItem(filters, "filter_reset_threshold");
        if (item && cJSON_IsNumber(item)) {
            config->filter_reset_threshold = (float)item->valuedouble;
        }
    }

    // Load fusion timing
    cJSON* timing = cJSON_GetObjectItem(root, "fusion_timing");
    if (timing) {
        cJSON* item = cJSON_GetObjectItem(timing, "imu_update_rate_hz");
        if (item && cJSON_IsNumber(item)) {
            config->imu_update_rate_hz = (uint32_t)item->valueint;
        }

        item = cJSON_GetObjectItem(timing, "gps_update_rate_hz");
        if (item && cJSON_IsNumber(item)) {
            config->gps_update_rate_hz = (uint32_t)item->valueint;
        }

        item = cJSON_GetObjectItem(timing, "slam_update_rate_hz");
        if (item && cJSON_IsNumber(item)) {
            config->slam_update_rate_hz = (uint32_t)item->valueint;
        }

        item = cJSON_GetObjectItem(timing, "prediction_rate_hz");
        if (item && cJSON_IsNumber(item)) {
            config->prediction_rate_hz = (uint32_t)item->valueint;
        }

        item = cJSON_GetObjectItem(timing, "correction_timeout_ms");
        if (item && cJSON_IsNumber(item)) {
            config->correction_timeout_ms = (uint32_t)item->valueint;
        }
    }

    cJSON_Delete(root);
    return ESP_OK;
}

esp_err_t config_loader_load_camera_config(extended_camera_config_t* config) {
    // Initialize with default values first
    extended_camera_config_t default_config = {
        .base_config = {
            .resolution = 0, // CAMERA_RES_640x480
            .format = 1,     // CAMERA_FORMAT_RGB565
            .fps = 30,
            .auto_exposure = true,
            .auto_white_balance = true,
            .brightness = 0,
            .contrast = 0,
            .saturation = 0,
            .exposure_value = 100,
            .auto_adjustment_enabled = true,
            .target_brightness = 128,
            .adjustment_speed = 0.3f,
            .brightness_threshold = 20.0f
        },
        .aec2 = true,
        .aec_value = 300,
        .ae_level = 0,
        .agc = true,
        .agc_gain = 0,
        .gain_ceiling = 0,
        .bpc = false,
        .wpc = false,
        .raw_gma = true,
        .lenc = true,
        .hmirror = false,
        .vflip = false,
        .dcw = true,
        .colorbar = false,
        .quality = 12,
        .sharpness = 0,
        .denoise = 0,
        .special_effect = 0,
        .wb_mode = 0,
        .awb = true,
        .awb_gain = true,
        .aec = true,
        .aec_level = 0
    };

    // Copy defaults to output config
    memcpy(config, &default_config, sizeof(extended_camera_config_t));

    char* buffer = NULL;
    size_t size = 0;

    esp_err_t ret = read_config_file_with_fallback(CAMERA_CONFIG_FILE_SD, CAMERA_CONFIG_FILE_SPIFFS, &buffer, &size);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to read camera config file, using defaults: %s", esp_err_to_name(ret));
        return ESP_OK; // Return OK since we have defaults
    }

    cJSON* root = cJSON_Parse(buffer);
    free(buffer);

    if (root == NULL) {
        ESP_LOGE(TAG, "Failed to parse camera config JSON, using defaults");
        return ESP_OK; // Return OK since we have defaults
    }

    // Load camera configuration
    cJSON* camera = cJSON_GetObjectItem(root, "camera");
    if (camera) {
        cJSON* item = cJSON_GetObjectItem(camera, "resolution");
        if (item) {
            if (cJSON_IsString(item)) {
                // Handle string resolution values
                if (strcmp(item->valuestring, "1920x1080") == 0 || strcmp(item->valuestring, "FULL_HD") == 0) {
                    config->base_config.resolution = 1920 * 1080; // Use pixel count as numeric value
                } else if (strcmp(item->valuestring, "1280x960") == 0) {
                    config->base_config.resolution = 1280 * 960;
                } else if (strcmp(item->valuestring, "800x600") == 0) {
                    config->base_config.resolution = 800 * 600;
                } else if (strcmp(item->valuestring, "640x480") == 0) {
                    config->base_config.resolution = 640 * 480;
                } else {
                    ESP_LOGW(TAG, "Unknown resolution string: %s, using default", item->valuestring);
                    config->base_config.resolution = 1920 * 1080; // Default to 1920x1080
                }
            } else if (cJSON_IsNumber(item)) {
                // Handle legacy numeric values
                config->base_config.resolution = (uint32_t)item->valuedouble;
            }
        }

        item = cJSON_GetObjectItem(camera, "format");
        if (item) {
            if (cJSON_IsString(item)) {
                // Handle string format values
                if (strcmp(item->valuestring, "RGB565") == 0) {
                    config->base_config.format = 1;
                } else if (strcmp(item->valuestring, "RGB888") == 0) {
                    config->base_config.format = 2;
                } else if (strcmp(item->valuestring, "YUV422") == 0) {
                    config->base_config.format = 3;
                } else {
                    ESP_LOGW(TAG, "Unknown format string: %s, using RGB565", item->valuestring);
                    config->base_config.format = 1; // Default to RGB565
                }
            } else if (cJSON_IsNumber(item)) {
                // Handle legacy numeric values
                config->base_config.format = (uint32_t)item->valuedouble;
            }
        }

        item = cJSON_GetObjectItem(camera, "fps");
        if (item && cJSON_IsNumber(item)) {
            config->base_config.fps = (uint32_t)item->valuedouble;
        }

        item = cJSON_GetObjectItem(camera, "auto_exposure");
        if (item && cJSON_IsBool(item)) {
            config->base_config.auto_exposure = cJSON_IsTrue(item);
        }

        item = cJSON_GetObjectItem(camera, "auto_white_balance");
        if (item && cJSON_IsBool(item)) {
            config->base_config.auto_white_balance = cJSON_IsTrue(item);
        }

        item = cJSON_GetObjectItem(camera, "brightness");
        if (item && cJSON_IsNumber(item)) {
            config->base_config.brightness = (int)item->valuedouble;
        }

        item = cJSON_GetObjectItem(camera, "contrast");
        if (item && cJSON_IsNumber(item)) {
            config->base_config.contrast = (int)item->valuedouble;
        }

        item = cJSON_GetObjectItem(camera, "saturation");
        if (item && cJSON_IsNumber(item)) {
            config->base_config.saturation = (int)item->valuedouble;
        }

        item = cJSON_GetObjectItem(camera, "exposure_value");
        if (item && cJSON_IsNumber(item)) {
            config->base_config.exposure_value = (int)item->valuedouble;
        }
    }

    cJSON_Delete(root);
    return ESP_OK;
}

esp_err_t config_loader_load_imu_config(extended_imu_config_t* config) {
    // Initialize with default values first
    extended_imu_config_t default_config = {
        .base_config = {
            .spi_host = 2, // SPI3_HOST
            .miso_pin = IMU_SPI_MISO_PIN,
            .mosi_pin = IMU_SPI_MOSI_PIN,
            .sclk_pin = IMU_SPI_CLK_PIN,
            .acc_cs_pin = IMU_ACC_CS_PIN,
            .gyro_cs_pin = IMU_GYRO_CS_PIN,
            .accel_range = 6,    // BMI088_ACCEL_RANGE_6G
            .gyro_range = 500,   // BMI088_GYRO_RANGE_500DPS
            .sample_rate = 400
        },
        .calibration_enabled = true,
        .auto_calibration = true,
        .calibration_samples = 1000,
        .accel_bias_x = 0.0f,
        .accel_bias_y = 0.0f,
        .accel_bias_z = 0.0f,
        .gyro_bias_x = 0.0f,
        .gyro_bias_y = 0.0f,
        .gyro_bias_z = 0.0f,
        .accel_scale_x = 1.0f,
        .accel_scale_y = 1.0f,
        .accel_scale_z = 1.0f,
        .gyro_scale_x = 1.0f,
        .gyro_scale_y = 1.0f,
        .gyro_scale_z = 1.0f,
        .low_pass_filter_enabled = true,
        .low_pass_cutoff_hz = 100.0f,
        .high_pass_filter_enabled = false,
        .high_pass_cutoff_hz = 0.1f,
        .notch_filter_enabled = false,
        .notch_frequency_hz = 50.0f,
        .notch_bandwidth_hz = 5.0f,
        .temperature_compensation = true,
        .self_test_enabled = false,
        .interrupt_enabled = true,
        .fifo_enabled = true,
        .fifo_watermark = 100,
        .power_mode = "NORMAL"
    };

    // Copy defaults to output config
    memcpy(config, &default_config, sizeof(extended_imu_config_t));

    char* buffer = NULL;
    size_t size = 0;

    esp_err_t ret = read_config_file_with_fallback(IMU_CONFIG_FILE_SD, IMU_CONFIG_FILE_SPIFFS, &buffer, &size);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to read IMU config file, using defaults: %s", esp_err_to_name(ret));
        return ESP_OK; // Return OK since we have defaults
    }

    cJSON* root = cJSON_Parse(buffer);
    free(buffer);

    if (root == NULL) {
        ESP_LOGE(TAG, "Failed to parse IMU config JSON, using defaults");
        return ESP_OK; // Return OK since we have defaults
    }

    // Load IMU configuration
    cJSON* imu = cJSON_GetObjectItem(root, "imu");
    if (imu) {
        cJSON* item = cJSON_GetObjectItem(imu, "spi_host");
        if (item && cJSON_IsNumber(item)) {
            config->base_config.spi_host = (uint32_t)item->valuedouble;
        }

        item = cJSON_GetObjectItem(imu, "miso_pin");
        if (item && cJSON_IsNumber(item)) {
            config->base_config.miso_pin = (uint32_t)item->valuedouble;
        }

        item = cJSON_GetObjectItem(imu, "mosi_pin");
        if (item && cJSON_IsNumber(item)) {
            config->base_config.mosi_pin = (uint32_t)item->valuedouble;
        }

        item = cJSON_GetObjectItem(imu, "sclk_pin");
        if (item && cJSON_IsNumber(item)) {
            config->base_config.sclk_pin = (uint32_t)item->valuedouble;
        }

        item = cJSON_GetObjectItem(imu, "acc_cs_pin");
        if (item && cJSON_IsNumber(item)) {
            config->base_config.acc_cs_pin = (uint32_t)item->valuedouble;
        }

        item = cJSON_GetObjectItem(imu, "gyro_cs_pin");
        if (item && cJSON_IsNumber(item)) {
            config->base_config.gyro_cs_pin = (uint32_t)item->valuedouble;
        }

        item = cJSON_GetObjectItem(imu, "accel_range");
        if (item && cJSON_IsNumber(item)) {
            config->base_config.accel_range = (uint32_t)item->valuedouble;
        }

        item = cJSON_GetObjectItem(imu, "gyro_range");
        if (item && cJSON_IsNumber(item)) {
            config->base_config.gyro_range = (uint32_t)item->valuedouble;
        }

        item = cJSON_GetObjectItem(imu, "sample_rate");
        if (item && cJSON_IsNumber(item)) {
            config->base_config.sample_rate = (uint32_t)item->valuedouble;
        }
    }

    cJSON_Delete(root);
    return ESP_OK;
}

esp_err_t config_loader_load_all(master_config_t* config) {
    esp_err_t ret;

    // Load all configuration files
    ret = config_loader_load_system_config(&config->system);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to load system config: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = config_loader_load_gps_config(&config->gps);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to load GPS config: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = config_loader_load_msp_config(&config->msp);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to load MSP config: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = config_loader_load_slam_config(&config->slam);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to load SLAM config: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = config_loader_load_fusion_config(&config->fusion);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to load fusion config: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = config_loader_load_camera_config(&config->camera);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to load camera config: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = config_loader_load_imu_config(&config->imu);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to load IMU config: %s", esp_err_to_name(ret));
        return ret;
    }

    config->loaded = true;
    config->load_timestamp = esp_timer_get_time();

    // Validate and apply defaults to invalid configuration values
    ret = config_loader_validate_config(config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Configuration validation failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "All configuration files loaded and validated successfully");
    return ESP_OK;
}

esp_err_t config_loader_validate_config(master_config_t* config) {
    // Basic validation - apply defaults for invalid values
    if (!config->loaded) {
        ESP_LOGE(TAG, "Configuration not loaded");
        return ESP_ERR_INVALID_STATE;
    }

    bool applied_defaults = false;

    // Validate and fix GPS configuration
    if (config->gps.base_config.uart_port > UART_NUM_MAX - 1) {
        ESP_LOGW(TAG, "Invalid GPS UART port %d, setting to default UART1", config->gps.base_config.uart_port);
        config->gps.base_config.uart_port = UART_NUM_1;
        applied_defaults = true;
    }

    // Validate and fix MSP configuration
    if (config->msp.base_config.uart_port > UART_NUM_MAX - 1) {
        ESP_LOGW(TAG, "Invalid MSP UART port %d, setting to default UART2", config->msp.base_config.uart_port);
        config->msp.base_config.uart_port = UART_NUM_2;
        applied_defaults = true;
    }

    // Validate and fix SLAM configuration - apply defaults for zero/invalid values
    if (config->slam.base_config.max_features == 0) {
        ESP_LOGW(TAG, "Invalid SLAM max_features %d, applying default value 500", config->slam.base_config.max_features);
        config->slam.base_config.max_features = 500;
        applied_defaults = true;
    }
    if (config->slam.base_config.fast_threshold <= 0.0f) {
        ESP_LOGW(TAG, "Invalid SLAM fast_threshold %.1f, applying default value 20.0", config->slam.base_config.fast_threshold);
        config->slam.base_config.fast_threshold = 20.0f;
        applied_defaults = true;
    }
    if (config->slam.base_config.levels == 0) {
        ESP_LOGW(TAG, "Invalid SLAM levels %d, applying default value 8", config->slam.base_config.levels);
        config->slam.base_config.levels = 8;
        applied_defaults = true;
    }
    if (config->slam.base_config.scale_factor <= 1.0f) {
        ESP_LOGW(TAG, "Invalid SLAM scale_factor %.2f, applying default value 1.2", config->slam.base_config.scale_factor);
        config->slam.base_config.scale_factor = 1.2f;
        applied_defaults = true;
    }
    if (config->slam.base_config.max_keypoints_per_level == 0) {
        ESP_LOGW(TAG, "Invalid SLAM max_keypoints_per_level %d, applying default value 100", config->slam.base_config.max_keypoints_per_level);
        config->slam.base_config.max_keypoints_per_level = 100;
        applied_defaults = true;
    }
    if (config->slam.base_config.descriptor_distance_threshold == 0) {
        ESP_LOGW(TAG, "Invalid SLAM descriptor_distance_threshold %d, applying default value 50", config->slam.base_config.descriptor_distance_threshold);
        config->slam.base_config.descriptor_distance_threshold = 50;
        applied_defaults = true;
    }
    if (config->slam.base_config.match_threshold <= 0.0f) {
        ESP_LOGW(TAG, "Invalid SLAM match_threshold %.2f, applying default value 0.7", config->slam.base_config.match_threshold);
        config->slam.base_config.match_threshold = 0.7f;
        applied_defaults = true;
    }
    if (config->slam.base_config.min_tracked_features == 0) {
        ESP_LOGW(TAG, "Invalid SLAM min_tracked_features %d, applying default value 30", config->slam.base_config.min_tracked_features);
        config->slam.base_config.min_tracked_features = 30;
        applied_defaults = true;
    }
    if (config->slam.base_config.keyframe_distance_threshold <= 0.0f) {
        ESP_LOGW(TAG, "Invalid SLAM keyframe_distance_threshold %.2f, applying default value 1.0", config->slam.base_config.keyframe_distance_threshold);
        config->slam.base_config.keyframe_distance_threshold = 1.0f;
        applied_defaults = true;
    }
    if (config->slam.base_config.keyframe_angle_threshold <= 0.0f) {
        ESP_LOGW(TAG, "Invalid SLAM keyframe_angle_threshold %.2f, applying default value 0.2", config->slam.base_config.keyframe_angle_threshold);
        config->slam.base_config.keyframe_angle_threshold = 0.2f;
        applied_defaults = true;
    }
    if (config->slam.base_config.max_keyframes == 0) {
        ESP_LOGW(TAG, "Invalid SLAM max_keyframes %d, applying default value 50", config->slam.base_config.max_keyframes);
        config->slam.base_config.max_keyframes = 50;
        applied_defaults = true;
    }
    if (config->slam.base_config.loop_closure_threshold <= 0.0f) {
        ESP_LOGW(TAG, "Invalid SLAM loop_closure_threshold %.2f, applying default value 0.8", config->slam.base_config.loop_closure_threshold);
        config->slam.base_config.loop_closure_threshold = 0.8f;
        applied_defaults = true;
    }

    if (applied_defaults) {
        ESP_LOGI(TAG, "Applied default values to invalid configuration parameters");
    } else {
        ESP_LOGI(TAG, "Configuration validation passed - no defaults applied");
    }

    return ESP_OK;
}

void config_loader_print_config(const master_config_t* config) {
    if (!config->loaded) {
        ESP_LOGW(TAG, "Configuration not loaded");
        return;
    }

    ESP_LOGI(TAG, "=== System Configuration ===");
    ESP_LOGI(TAG, "Name: %s", config->system.name);
    ESP_LOGI(TAG, "Version: %s", config->system.version);
    ESP_LOGI(TAG, "Debug Mode: %s", config->system.debug_mode ? "true" : "false");

    ESP_LOGI(TAG, "=== GPS Configuration ===");
    ESP_LOGI(TAG, "UART Port: %d, Baud: %d", config->gps.base_config.uart_port, config->gps.base_config.baud_rate);
    ESP_LOGI(TAG, "TX Pin: %d, RX Pin: %d", config->gps.base_config.tx_pin, config->gps.base_config.rx_pin);

    ESP_LOGI(TAG, "=== MSP Configuration ===");
    ESP_LOGI(TAG, "UART Port: %d, Baud: %d", config->msp.base_config.uart_port, config->msp.base_config.baud_rate);
    ESP_LOGI(TAG, "TX Pin: %d, RX Pin: %d", config->msp.base_config.tx_pin, config->msp.base_config.rx_pin);

    ESP_LOGI(TAG, "=== SLAM Configuration ===");
    ESP_LOGI(TAG, "Max Features: %d, FAST Threshold: %.1f", config->slam.base_config.max_features, config->slam.base_config.fast_threshold);
    ESP_LOGI(TAG, "Levels: %d, Scale Factor: %.2f", config->slam.base_config.levels, config->slam.base_config.scale_factor);
}

esp_err_t config_loader_save_config(const master_config_t* config) {
    if (!config) {
        ESP_LOGE(TAG, "Configuration cannot be NULL");
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "💾 Saving configuration to SD card");

    // Save to SD card if available
    if (g_config_state.sd_available) {
        esp_err_t ret = save_master_config_to_sd(config);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "✅ Configuration saved to SD card");
            return ESP_OK;
        } else {
            ESP_LOGW(TAG, "Failed to save to SD card, trying SPIFFS");
        }
    }

    // Fallback to SPIFFS
    esp_err_t ret = save_master_config_to_spiffs(config);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "✅ Configuration saved to SPIFFS");
    } else {
        ESP_LOGE(TAG, "❌ Failed to save configuration");
    }

    return ret;
}

// =============================================================================
// SD CARD CONFIGURATION MANAGEMENT
// =============================================================================

/**
 * Save system configuration to SD card
 */
static esp_err_t save_system_config_to_sd(const system_config_t* config) {
    if (!g_config_state.sd_available) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    cJSON* root = cJSON_CreateObject();
    if (!root) return ESP_ERR_NO_MEM;

    // Create system object
    cJSON* system = cJSON_AddObjectToObject(root, "system");
    if (!system) {
        cJSON_Delete(root);
        return ESP_ERR_NO_MEM;
    }

    cJSON_AddStringToObject(system, "name", config->name);
    cJSON_AddStringToObject(system, "version", config->version);
    cJSON_AddStringToObject(system, "description", config->description);
    cJSON_AddBoolToObject(system, "debug_mode", config->debug_mode);
    cJSON_AddStringToObject(system, "log_level", config->log_level);
    cJSON_AddBoolToObject(system, "enable_watchdog", config->enable_watchdog);
    cJSON_AddNumberToObject(system, "watchdog_timeout_ms", config->watchdog_timeout_ms);
    cJSON_AddBoolToObject(system, "enable_statistics", config->enable_statistics);
    cJSON_AddNumberToObject(system, "statistics_interval_ms", config->statistics_interval_ms);

    // Create components object
    cJSON* components = cJSON_AddObjectToObject(root, "components");
    if (!components) {
        cJSON_Delete(root);
        return ESP_ERR_NO_MEM;
    }

    cJSON_AddBoolToObject(components, "camera_enabled", config->camera_enabled);
    cJSON_AddBoolToObject(components, "gps_enabled", config->gps_enabled);
    cJSON_AddBoolToObject(components, "imu_enabled", config->imu_enabled);
    cJSON_AddBoolToObject(components, "slam_enabled", config->slam_enabled);
    cJSON_AddBoolToObject(components, "sensor_fusion_enabled", config->sensor_fusion_enabled);
    cJSON_AddBoolToObject(components, "msp_enabled", config->msp_enabled);
    cJSON_AddBoolToObject(components, "sd_storage_enabled", config->sd_storage_enabled);
    cJSON_AddBoolToObject(components, "web_server_enabled", config->web_server_enabled);

    // Create performance object
    cJSON* performance = cJSON_AddObjectToObject(root, "performance");
    if (!performance) {
        cJSON_Delete(root);
        return ESP_ERR_NO_MEM;
    }

    cJSON_AddNumberToObject(performance, "target_fps", config->target_fps);
    cJSON_AddNumberToObject(performance, "max_processing_time_ms", config->max_processing_time_ms);
    cJSON_AddBoolToObject(performance, "memory_monitoring", config->memory_monitoring);
    cJSON_AddBoolToObject(performance, "cpu_monitoring", config->cpu_monitoring);
    cJSON_AddBoolToObject(performance, "enable_performance_logging", config->enable_performance_logging);

    // Create safety object
    cJSON* safety = cJSON_AddObjectToObject(root, "safety");
    if (!safety) {
        cJSON_Delete(root);
        return ESP_ERR_NO_MEM;
    }

    cJSON_AddBoolToObject(safety, "emergency_stop_enabled", config->emergency_stop_enabled);
    cJSON_AddNumberToObject(safety, "failsafe_timeout_ms", config->failsafe_timeout_ms);
    cJSON_AddNumberToObject(safety, "maximum_altitude_m", config->maximum_altitude_m);
    cJSON_AddNumberToObject(safety, "minimum_altitude_m", config->minimum_altitude_m);
    cJSON_AddNumberToObject(safety, "maximum_velocity_ms", config->maximum_velocity_ms);
    cJSON_AddBoolToObject(safety, "geofence_enabled", config->geofence_enabled);

    // Create communication object
    cJSON* communication = cJSON_AddObjectToObject(root, "communication");
    if (!communication) {
        cJSON_Delete(root);
        return ESP_ERR_NO_MEM;
    }

    cJSON_AddNumberToObject(communication, "uart_buffer_size", config->uart_buffer_size);
    cJSON_AddNumberToObject(communication, "i2c_clock_speed", config->i2c_clock_speed);
    cJSON_AddNumberToObject(communication, "spi_clock_speed", config->spi_clock_speed);
    cJSON_AddNumberToObject(communication, "communication_timeout_ms", config->communication_timeout_ms);

    char* json_str = cJSON_Print(root);
    cJSON_Delete(root);

    if (!json_str) return ESP_ERR_NO_MEM;

    size_t json_len = strlen(json_str);
    FILE* f = fopen(SYSTEM_CONFIG_FILE_SD, "w");
    if (!f) {
        ESP_LOGE(TAG, "❌ Failed to open system config file on SD: %s", SYSTEM_CONFIG_FILE_SD);
        free(json_str);
        return ESP_ERR_INVALID_ARG;
    }

    size_t written = fwrite(json_str, 1, json_len, f);
    if (written != json_len) {
        ESP_LOGE(TAG, "❌ Failed to write system config file to SD: wrote %zu/%zu bytes", written, json_len);
    }
    fclose(f);
    free(json_str);

    ESP_LOGI(TAG, "✅ Saved system config to SD card: %s", SYSTEM_CONFIG_FILE_SD);
    return (written == json_len) ? ESP_OK : ESP_ERR_INVALID_SIZE;
}

/**
 * Save GPS configuration to SD card
 */
static esp_err_t save_gps_config_to_sd(const extended_gps_config_t* config) {
    if (!g_config_state.sd_available) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    cJSON* root = cJSON_CreateObject();
    if (!root) return ESP_ERR_NO_MEM;

    cJSON* gps = cJSON_AddObjectToObject(root, "gps");
    if (!gps) {
        cJSON_Delete(root);
        return ESP_ERR_NO_MEM;
    }

    cJSON_AddNumberToObject(gps, "uart_port", config->base_config.uart_port);
    cJSON_AddNumberToObject(gps, "baud_rate", config->base_config.baud_rate);
    cJSON_AddNumberToObject(gps, "tx_pin", config->base_config.tx_pin);
    cJSON_AddNumberToObject(gps, "rx_pin", config->base_config.rx_pin);
    cJSON_AddNumberToObject(gps, "update_rate_hz", config->base_config.update_rate_hz);
    cJSON_AddBoolToObject(gps, "enable_sbas", config->enable_sbas);
    cJSON_AddBoolToObject(gps, "enable_differential", config->enable_differential);
    cJSON_AddNumberToObject(gps, "timeout_ms", config->timeout_ms);
    cJSON_AddBoolToObject(gps, "auto_reconnect", config->auto_reconnect);
    cJSON_AddNumberToObject(gps, "reconnect_attempts", config->reconnect_attempts);
    cJSON_AddNumberToObject(gps, "reconnect_delay_ms", config->reconnect_delay_ms);

    cJSON* filters = cJSON_AddObjectToObject(root, "gps_filters");
    if (filters) {
        cJSON_AddNumberToObject(filters, "minimum_satellites", config->minimum_satellites);
        cJSON_AddNumberToObject(filters, "maximum_hdop", config->maximum_hdop);
        cJSON_AddNumberToObject(filters, "maximum_speed_ms", config->maximum_speed_ms);
        cJSON_AddBoolToObject(filters, "speed_filter_enabled", config->speed_filter_enabled);
        cJSON_AddBoolToObject(filters, "altitude_filter_enabled", config->altitude_filter_enabled);
        cJSON_AddNumberToObject(filters, "minimum_altitude_m", config->minimum_altitude_m);
        cJSON_AddNumberToObject(filters, "maximum_altitude_m", config->maximum_altitude_m);
    }

    cJSON* advanced = cJSON_AddObjectToObject(root, "gps_advanced");
    if (advanced) {
        cJSON_AddNumberToObject(advanced, "dynamic_model", config->dynamic_model);
        cJSON_AddNumberToObject(advanced, "fix_mode", config->fix_mode);
        cJSON_AddBoolToObject(advanced, "enable_gps", config->enable_gps);
        cJSON_AddBoolToObject(advanced, "enable_glonass", config->enable_glonass);
        cJSON_AddBoolToObject(advanced, "enable_galileo", config->enable_galileo);
        cJSON_AddBoolToObject(advanced, "enable_beidou", config->enable_beidou);
        cJSON_AddBoolToObject(advanced, "time_pulse_enabled", config->time_pulse_enabled);
        cJSON_AddBoolToObject(advanced, "antenna_detection", config->antenna_detection);
    }

    char* json_str = cJSON_Print(root);
    cJSON_Delete(root);

    if (!json_str) return ESP_ERR_NO_MEM;

    size_t json_len = strlen(json_str);
    FILE* f = fopen(GPS_CONFIG_FILE_SD, "w");
    if (!f) {
        ESP_LOGE(TAG, "❌ Failed to open GPS config file on SD: %s", GPS_CONFIG_FILE_SD);
        free(json_str);
        return ESP_ERR_INVALID_ARG;
    }

    size_t written = fwrite(json_str, 1, json_len, f);
    if (written != json_len) {
        ESP_LOGE(TAG, "❌ Failed to write GPS config file to SD: wrote %zu/%zu bytes", written, json_len);
    }
    fclose(f);
    free(json_str);

    ESP_LOGI(TAG, "✅ Saved GPS config to SD card: %s", GPS_CONFIG_FILE_SD);
    return (written == json_len) ? ESP_OK : ESP_ERR_INVALID_SIZE;
}

/**
 * Save MSP configuration to SD card
 */
static esp_err_t save_msp_config_to_sd(const extended_msp_config_t* config) {
    if (!g_config_state.sd_available) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    cJSON* root = cJSON_CreateObject();
    if (!root) return ESP_ERR_NO_MEM;

    cJSON* msp = cJSON_AddObjectToObject(root, "msp");
    if (!msp) {
        cJSON_Delete(root);
        return ESP_ERR_NO_MEM;
    }

    cJSON_AddNumberToObject(msp, "uart_port", config->base_config.uart_port);
    cJSON_AddNumberToObject(msp, "baud_rate", config->base_config.baud_rate);
    cJSON_AddNumberToObject(msp, "tx_pin", config->base_config.tx_pin);
    cJSON_AddNumberToObject(msp, "rx_pin", config->base_config.rx_pin);
    cJSON_AddNumberToObject(msp, "timeout_ms", config->base_config.timeout_ms);
    cJSON_AddBoolToObject(msp, "auto_reconnect", config->base_config.auto_reconnect);
    cJSON_AddNumberToObject(msp, "status_request_interval_ms", config->status_request_interval_ms);
    cJSON_AddNumberToObject(msp, "navigation_status_interval_ms", config->navigation_status_interval_ms);
    cJSON_AddNumberToObject(msp, "attitude_request_interval_ms", config->attitude_request_interval_ms);
    cJSON_AddNumberToObject(msp, "altitude_request_interval_ms", config->altitude_request_interval_ms);
    cJSON_AddNumberToObject(msp, "gps_request_interval_ms", config->gps_request_interval_ms);
    cJSON_AddNumberToObject(msp, "analog_request_interval_ms", config->analog_request_interval_ms);
    cJSON_AddNumberToObject(msp, "default_mode", config->default_mode);
    cJSON_AddNumberToObject(msp, "waypoint_radius_m", config->waypoint_radius_m);
    cJSON_AddNumberToObject(msp, "waypoint_hold_time_ms", config->waypoint_hold_time_ms);
    cJSON_AddNumberToObject(msp, "cruise_speed_ms", config->cruise_speed_ms);
    cJSON_AddNumberToObject(msp, "max_climb_rate_ms", config->max_climb_rate_ms);
    cJSON_AddNumberToObject(msp, "max_descent_rate_ms", config->max_descent_rate_ms);
    cJSON_AddNumberToObject(msp, "rtl_altitude_m", config->rtl_altitude_m);
    cJSON_AddNumberToObject(msp, "land_speed_ms", config->land_speed_ms);
    cJSON_AddBoolToObject(msp, "emergency_stop_enabled", config->emergency_stop_enabled);
    cJSON_AddBoolToObject(msp, "failsafe_enabled", config->failsafe_enabled);
    cJSON_AddNumberToObject(msp, "low_battery_voltage", config->low_battery_voltage);
    cJSON_AddNumberToObject(msp, "critical_battery_voltage", config->critical_battery_voltage);
    cJSON_AddNumberToObject(msp, "maximum_roll_angle_deg", config->maximum_roll_angle_deg);
    cJSON_AddNumberToObject(msp, "maximum_pitch_angle_deg", config->maximum_pitch_angle_deg);

    char* json_str = cJSON_Print(root);
    cJSON_Delete(root);

    if (!json_str) return ESP_ERR_NO_MEM;

    size_t json_len = strlen(json_str);
    FILE* f = fopen(MSP_CONFIG_FILE_SD, "w");
    if (!f) {
        ESP_LOGE(TAG, "❌ Failed to open MSP config file on SD: %s", MSP_CONFIG_FILE_SD);
        free(json_str);
        return ESP_ERR_INVALID_ARG;
    }

    size_t written = fwrite(json_str, 1, json_len, f);
    if (written != json_len) {
        ESP_LOGE(TAG, "❌ Failed to write MSP config file to SD: wrote %zu/%zu bytes", written, json_len);
    }
    fclose(f);
    free(json_str);

    ESP_LOGI(TAG, "✅ Saved MSP config to SD card: %s", MSP_CONFIG_FILE_SD);
    return (written == json_len) ? ESP_OK : ESP_ERR_INVALID_SIZE;
}

/**
 * Save SLAM configuration to SD card
 */
static esp_err_t save_slam_config_to_sd(const extended_slam_config_t* config) {
    if (!g_config_state.sd_available) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    cJSON* root = cJSON_CreateObject();
    if (!root) return ESP_ERR_NO_MEM;

    cJSON* slam = cJSON_AddObjectToObject(root, "slam");
    if (!slam) {
        cJSON_Delete(root);
        return ESP_ERR_NO_MEM;
    }

    cJSON_AddNumberToObject(slam, "max_features", config->base_config.max_features);
    cJSON_AddNumberToObject(slam, "fast_threshold", config->base_config.fast_threshold);
    cJSON_AddNumberToObject(slam, "levels", config->base_config.levels);
    cJSON_AddNumberToObject(slam, "scale_factor", config->base_config.scale_factor);
    cJSON_AddNumberToObject(slam, "max_keypoints_per_level", config->base_config.max_keypoints_per_level);
    cJSON_AddBoolToObject(slam, "use_harris_detector", config->base_config.use_harris_detector);
    cJSON_AddNumberToObject(slam, "harris_k", config->base_config.harris_k);
    cJSON_AddNumberToObject(slam, "descriptor_distance_threshold", config->base_config.descriptor_distance_threshold);
    cJSON_AddNumberToObject(slam, "match_threshold", config->base_config.match_threshold);
    cJSON_AddNumberToObject(slam, "min_tracked_features", config->base_config.min_tracked_features);
    cJSON_AddNumberToObject(slam, "keyframe_distance_threshold", config->base_config.keyframe_distance_threshold);
    cJSON_AddNumberToObject(slam, "keyframe_angle_threshold", config->base_config.keyframe_angle_threshold);
    cJSON_AddNumberToObject(slam, "max_keyframes", config->base_config.max_keyframes);
    cJSON_AddBoolToObject(slam, "enable_loop_closure", config->base_config.enable_loop_closure);
    cJSON_AddNumberToObject(slam, "loop_closure_threshold", config->base_config.loop_closure_threshold);
    cJSON_AddBoolToObject(slam, "map_optimization_enabled", config->base_config.map_optimization_enabled);

    // Extended SLAM parameters
    cJSON_AddNumberToObject(slam, "tracking_confidence_threshold", config->tracking_confidence_threshold);
    cJSON_AddBoolToObject(slam, "relocalization_enabled", config->relocalization_enabled);
    cJSON_AddBoolToObject(slam, "outlier_rejection_enabled", config->outlier_rejection_enabled);
    cJSON_AddBoolToObject(slam, "bundle_adjustment_enabled", config->bundle_adjustment_enabled);
    cJSON_AddStringToObject(slam, "feature_tracking_method", config->feature_tracking_method);
    cJSON_AddStringToObject(slam, "motion_model", config->motion_model);
    cJSON_AddBoolToObject(slam, "map_save_enabled", config->map_save_enabled);
    cJSON_AddBoolToObject(slam, "map_load_enabled", config->map_load_enabled);
    cJSON_AddBoolToObject(slam, "map_compression_enabled", config->map_compression_enabled);
    cJSON_AddNumberToObject(slam, "max_map_points", config->max_map_points);
    cJSON_AddNumberToObject(slam, "map_point_culling_threshold", config->map_point_culling_threshold);
    cJSON_AddNumberToObject(slam, "keyframe_culling_threshold", config->keyframe_culling_threshold);
    cJSON_AddNumberToObject(slam, "processing_threads", config->processing_threads);
    cJSON_AddNumberToObject(slam, "feature_extraction_threads", config->feature_extraction_threads);
    cJSON_AddNumberToObject(slam, "optimization_threads", config->optimization_threads);
    cJSON_AddNumberToObject(slam, "memory_pool_size_kb", config->memory_pool_size_kb);
    cJSON_AddNumberToObject(slam, "feature_cache_size", config->feature_cache_size);

    char* json_str = cJSON_Print(root);
    cJSON_Delete(root);

    if (!json_str) return ESP_ERR_NO_MEM;

    size_t json_len = strlen(json_str);
    FILE* f = fopen(SLAM_CONFIG_FILE_SD, "w");
    if (!f) {
        ESP_LOGE(TAG, "❌ Failed to open SLAM config file: %s", SLAM_CONFIG_FILE_SD);
        free(json_str);
        return ESP_ERR_INVALID_ARG;
    }

    size_t written = fwrite(json_str, 1, json_len, f);
    if (written != json_len) {
        ESP_LOGE(TAG, "❌ Failed to write SLAM config file: wrote %zu/%zu bytes", written, json_len);
    }
    fclose(f);
    free(json_str);

    return (written == json_len) ? ESP_OK : ESP_ERR_INVALID_SIZE;
}

/**
 * Save fusion configuration to SD card
 */
static esp_err_t save_fusion_config_to_sd(const extended_fusion_config_t* config) {
    if (!g_config_state.sd_available) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    cJSON* root = cJSON_CreateObject();
    if (!root) return ESP_ERR_NO_MEM;

    // Create fusion object for base config parameters
    cJSON* fusion = cJSON_AddObjectToObject(root, "fusion");
    if (!fusion) {
        cJSON_Delete(root);
        return ESP_ERR_NO_MEM;
    }

    cJSON_AddStringToObject(fusion, "description", "Sensor fusion EKF configuration");
    cJSON_AddNumberToObject(fusion, "position_noise", config->base_config.position_noise);
    cJSON_AddNumberToObject(fusion, "velocity_noise", config->base_config.velocity_noise);
    cJSON_AddNumberToObject(fusion, "attitude_noise", config->base_config.attitude_noise);
    cJSON_AddNumberToObject(fusion, "slam_attitude_noise", config->base_config.slam_attitude_noise);
    cJSON_AddNumberToObject(fusion, "gps_position_noise", config->base_config.gps_position_noise);
    cJSON_AddNumberToObject(fusion, "gps_velocity_noise", config->base_config.gps_velocity_noise);
    cJSON_AddNumberToObject(fusion, "slam_position_noise", config->base_config.slam_position_noise);
    cJSON_AddNumberToObject(fusion, "accel_bias_noise", config->base_config.accel_bias_noise);
    cJSON_AddNumberToObject(fusion, "gyro_bias_noise", config->base_config.gyro_bias_noise);
    cJSON_AddNumberToObject(fusion, "imu_dt", config->base_config.imu_dt);
    cJSON_AddNumberToObject(fusion, "max_prediction_time", config->base_config.max_prediction_time);

    // Create fusion_weights object
    cJSON* weights = cJSON_AddObjectToObject(root, "fusion_weights");
    if (!weights) {
        cJSON_Delete(root);
        return ESP_ERR_NO_MEM;
    }

    cJSON_AddNumberToObject(weights, "gps_weight", config->base_config.gps_weight);
    cJSON_AddNumberToObject(weights, "slam_weight", config->base_config.slam_weight);
    cJSON_AddNumberToObject(weights, "imu_weight", config->base_config.imu_weight);
    cJSON_AddNumberToObject(weights, "gps_rejection_threshold", config->gps_rejection_threshold);
    cJSON_AddNumberToObject(weights, "slam_rejection_threshold", config->slam_rejection_threshold);
    cJSON_AddNumberToObject(weights, "imu_rejection_threshold", config->imu_rejection_threshold);

    // Create fusion_filters object
    cJSON* filters = cJSON_AddObjectToObject(root, "fusion_filters");
    if (!filters) {
        cJSON_Delete(root);
        return ESP_ERR_NO_MEM;
    }

    cJSON_AddBoolToObject(filters, "outlier_detection_enabled", config->outlier_detection_enabled);
    cJSON_AddNumberToObject(filters, "gps_outlier_threshold", config->gps_outlier_threshold);
    cJSON_AddNumberToObject(filters, "slam_outlier_threshold", config->slam_outlier_threshold);
    cJSON_AddNumberToObject(filters, "imu_outlier_threshold", config->imu_outlier_threshold);
    cJSON_AddBoolToObject(filters, "filter_reset_enabled", config->filter_reset_enabled);
    cJSON_AddNumberToObject(filters, "filter_reset_threshold", config->filter_reset_threshold);

    // Create fusion_timing object
    cJSON* timing = cJSON_AddObjectToObject(root, "fusion_timing");
    if (!timing) {
        cJSON_Delete(root);
        return ESP_ERR_NO_MEM;
    }

    cJSON_AddNumberToObject(timing, "imu_update_rate_hz", config->imu_update_rate_hz);
    cJSON_AddNumberToObject(timing, "gps_update_rate_hz", config->gps_update_rate_hz);
    cJSON_AddNumberToObject(timing, "slam_update_rate_hz", config->slam_update_rate_hz);
    cJSON_AddNumberToObject(timing, "prediction_rate_hz", config->prediction_rate_hz);
    cJSON_AddNumberToObject(timing, "correction_timeout_ms", config->correction_timeout_ms);

    char* json_str = cJSON_Print(root);
    cJSON_Delete(root);

    if (!json_str) return ESP_ERR_NO_MEM;

    size_t json_len = strlen(json_str);
    FILE* f = fopen(FUSION_CONFIG_FILE_SD, "w");
    if (!f) {
        ESP_LOGE(TAG, "❌ Failed to open fusion config file: %s", FUSION_CONFIG_FILE_SD);
        free(json_str);
        return ESP_ERR_INVALID_ARG;
    }

    size_t written = fwrite(json_str, 1, json_len, f);
    if (written != json_len) {
        ESP_LOGE(TAG, "❌ Failed to write fusion config file: wrote %zu/%zu bytes", written, json_len);
    }
    fclose(f);
    free(json_str);

    return (written == json_len) ? ESP_OK : ESP_ERR_INVALID_SIZE;
}

/**
 * Save camera configuration to SD card
 */
static esp_err_t save_camera_config_to_sd(const extended_camera_config_t* config) {
    if (!g_config_state.sd_available) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    cJSON* root = cJSON_CreateObject();
    if (!root) return ESP_ERR_NO_MEM;

    cJSON* camera = cJSON_AddObjectToObject(root, "camera");
    if (!camera) {
        cJSON_Delete(root);
        return ESP_ERR_NO_MEM;
    }

    // Convert numeric resolution back to string for readability
    const char* resolution_str = "1920x1080"; // Default
    if (config->base_config.resolution == 640 * 480) {
        resolution_str = "640x480";
    } else if (config->base_config.resolution == 800 * 600) {
        resolution_str = "800x600";
    } else if (config->base_config.resolution == 1280 * 960) {
        resolution_str = "1280x960";
    } else if (config->base_config.resolution == 1920 * 1080) {
        resolution_str = "1920x1080";
    }

    cJSON_AddStringToObject(camera, "resolution", resolution_str);

    // Convert numeric format back to string
    const char* format_str = "RGB565"; // Default
    if (config->base_config.format == 1) {
        format_str = "RGB565";
    } else if (config->base_config.format == 2) {
        format_str = "RGB888";
    } else if (config->base_config.format == 3) {
        format_str = "YUV422";
    }

    cJSON_AddStringToObject(camera, "format", format_str);
    cJSON_AddNumberToObject(camera, "fps", config->base_config.fps);
    cJSON_AddBoolToObject(camera, "auto_exposure", config->base_config.auto_exposure);
    cJSON_AddBoolToObject(camera, "auto_white_balance", config->base_config.auto_white_balance);
    cJSON_AddNumberToObject(camera, "brightness", config->base_config.brightness);
    cJSON_AddNumberToObject(camera, "contrast", config->base_config.contrast);
    cJSON_AddNumberToObject(camera, "saturation", config->base_config.saturation);
    cJSON_AddNumberToObject(camera, "exposure_value", config->base_config.exposure_value);

    // Extended camera parameters
    cJSON_AddBoolToObject(camera, "aec2", config->aec2);
    cJSON_AddNumberToObject(camera, "aec_value", config->aec_value);
    cJSON_AddNumberToObject(camera, "ae_level", config->ae_level);
    cJSON_AddBoolToObject(camera, "agc", config->agc);
    cJSON_AddNumberToObject(camera, "agc_gain", config->agc_gain);
    cJSON_AddNumberToObject(camera, "gain_ceiling", config->gain_ceiling);
    cJSON_AddBoolToObject(camera, "bpc", config->bpc);
    cJSON_AddBoolToObject(camera, "wpc", config->wpc);
    cJSON_AddBoolToObject(camera, "raw_gma", config->raw_gma);
    cJSON_AddBoolToObject(camera, "lenc", config->lenc);
    cJSON_AddBoolToObject(camera, "hmirror", config->hmirror);
    cJSON_AddBoolToObject(camera, "vflip", config->vflip);
    cJSON_AddBoolToObject(camera, "dcw", config->dcw);
    cJSON_AddBoolToObject(camera, "colorbar", config->colorbar);
    cJSON_AddNumberToObject(camera, "quality", config->quality);
    cJSON_AddNumberToObject(camera, "sharpness", config->sharpness);
    cJSON_AddNumberToObject(camera, "denoise", config->denoise);
    cJSON_AddNumberToObject(camera, "special_effect", config->special_effect);
    cJSON_AddNumberToObject(camera, "wb_mode", config->wb_mode);
    cJSON_AddBoolToObject(camera, "awb", config->awb);
    cJSON_AddBoolToObject(camera, "awb_gain", config->awb_gain);
    cJSON_AddBoolToObject(camera, "aec", config->aec);
    cJSON_AddNumberToObject(camera, "aec_level", config->aec_level);

    char* json_str = cJSON_Print(root);
    cJSON_Delete(root);

    if (!json_str) return ESP_ERR_NO_MEM;

    size_t json_len = strlen(json_str);
    FILE* f = fopen(CAMERA_CONFIG_FILE_SD, "w");
    if (!f) {
        ESP_LOGE(TAG, "❌ Failed to open camera config file: %s", CAMERA_CONFIG_FILE_SD);
        free(json_str);
        return ESP_ERR_INVALID_ARG;
    }

    size_t written = fwrite(json_str, 1, json_len, f);
    if (written != json_len) {
        ESP_LOGE(TAG, "❌ Failed to write camera config file: wrote %zu/%zu bytes", written, json_len);
    }
    fclose(f);
    free(json_str);

    return (written == json_len) ? ESP_OK : ESP_ERR_INVALID_SIZE;
}

/**
 * Save IMU configuration to SD card
 */
static esp_err_t save_imu_config_to_sd(const extended_imu_config_t* config) {
    if (!g_config_state.sd_available) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    cJSON* root = cJSON_CreateObject();
    if (!root) return ESP_ERR_NO_MEM;

    cJSON* imu = cJSON_AddObjectToObject(root, "imu");
    if (!imu) {
        cJSON_Delete(root);
        return ESP_ERR_NO_MEM;
    }

    cJSON_AddNumberToObject(imu, "spi_host", config->base_config.spi_host);
    cJSON_AddNumberToObject(imu, "miso_pin", config->base_config.miso_pin);
    cJSON_AddNumberToObject(imu, "mosi_pin", config->base_config.mosi_pin);
    cJSON_AddNumberToObject(imu, "sclk_pin", config->base_config.sclk_pin);
    cJSON_AddNumberToObject(imu, "acc_cs_pin", config->base_config.acc_cs_pin);
    cJSON_AddNumberToObject(imu, "gyro_cs_pin", config->base_config.gyro_cs_pin);
    cJSON_AddNumberToObject(imu, "accel_range", config->base_config.accel_range);
    cJSON_AddNumberToObject(imu, "gyro_range", config->base_config.gyro_range);
    cJSON_AddNumberToObject(imu, "sample_rate", config->base_config.sample_rate);

    // Extended IMU parameters
    cJSON_AddBoolToObject(imu, "calibration_enabled", config->calibration_enabled);
    cJSON_AddBoolToObject(imu, "auto_calibration", config->auto_calibration);
    cJSON_AddNumberToObject(imu, "calibration_samples", config->calibration_samples);
    cJSON_AddNumberToObject(imu, "accel_bias_x", config->accel_bias_x);
    cJSON_AddNumberToObject(imu, "accel_bias_y", config->accel_bias_y);
    cJSON_AddNumberToObject(imu, "accel_bias_z", config->accel_bias_z);
    cJSON_AddNumberToObject(imu, "gyro_bias_x", config->gyro_bias_x);
    cJSON_AddNumberToObject(imu, "gyro_bias_y", config->gyro_bias_y);
    cJSON_AddNumberToObject(imu, "gyro_bias_z", config->gyro_bias_z);
    cJSON_AddNumberToObject(imu, "accel_scale_x", config->accel_scale_x);
    cJSON_AddNumberToObject(imu, "accel_scale_y", config->accel_scale_y);
    cJSON_AddNumberToObject(imu, "accel_scale_z", config->accel_scale_z);
    cJSON_AddNumberToObject(imu, "gyro_scale_x", config->gyro_scale_x);
    cJSON_AddNumberToObject(imu, "gyro_scale_y", config->gyro_scale_y);
    cJSON_AddNumberToObject(imu, "gyro_scale_z", config->gyro_scale_z);
    cJSON_AddBoolToObject(imu, "low_pass_filter_enabled", config->low_pass_filter_enabled);
    cJSON_AddNumberToObject(imu, "low_pass_cutoff_hz", config->low_pass_cutoff_hz);
    cJSON_AddBoolToObject(imu, "high_pass_filter_enabled", config->high_pass_filter_enabled);
    cJSON_AddNumberToObject(imu, "high_pass_cutoff_hz", config->high_pass_cutoff_hz);
    cJSON_AddBoolToObject(imu, "notch_filter_enabled", config->notch_filter_enabled);
    cJSON_AddNumberToObject(imu, "notch_frequency_hz", config->notch_frequency_hz);
    cJSON_AddNumberToObject(imu, "notch_bandwidth_hz", config->notch_bandwidth_hz);
    cJSON_AddBoolToObject(imu, "temperature_compensation", config->temperature_compensation);
    cJSON_AddBoolToObject(imu, "self_test_enabled", config->self_test_enabled);
    cJSON_AddBoolToObject(imu, "interrupt_enabled", config->interrupt_enabled);
    cJSON_AddBoolToObject(imu, "fifo_enabled", config->fifo_enabled);
    cJSON_AddNumberToObject(imu, "fifo_watermark", config->fifo_watermark);
    cJSON_AddStringToObject(imu, "power_mode", config->power_mode);

    char* json_str = cJSON_Print(root);
    cJSON_Delete(root);

    if (!json_str) return ESP_ERR_NO_MEM;

    size_t json_len = strlen(json_str);
    FILE* f = fopen(IMU_CONFIG_FILE_SD, "w");
    if (!f) {
        ESP_LOGE(TAG, "❌ Failed to open IMU config file: %s", IMU_CONFIG_FILE_SD);
        free(json_str);
        return ESP_ERR_INVALID_ARG;
    }

    size_t written = fwrite(json_str, 1, json_len, f);
    if (written != json_len) {
        ESP_LOGE(TAG, "❌ Failed to write IMU config file: wrote %zu/%zu bytes", written, json_len);
    }
    fclose(f);
    free(json_str);

    return (written == json_len) ? ESP_OK : ESP_ERR_INVALID_SIZE;
}

/**
 * Check if SD card is available for configuration storage
 */
bool config_loader_is_sd_available(void) {
    return g_config_state.sd_available;
}

/**
 * Save configuration to SD card
 */
esp_err_t config_loader_save_to_sd(const master_config_t* config) {
    if (!g_config_state.sd_available) {
        ESP_LOGE(TAG, "SD card not available for saving configuration");
        return ESP_ERR_NOT_SUPPORTED;
    }

    if (!config->loaded) {
        ESP_LOGE(TAG, "Configuration not loaded, cannot save to SD card");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Saving configuration to SD card...");

    // Create config directory if it doesn't exist
    esp_err_t ret = sd_storage_create_directory(SD_CONFIG_DIR);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to create config directory: %s", esp_err_to_name(ret));
        return ret;
    }

    // Save each configuration file
    int success_count = 0;
    int total_count = 0;

    // Save system configuration
    ret = save_system_config_to_sd(&config->system);
    if (ret == ESP_OK) {
        success_count++;
        ESP_LOGI(TAG, "✅ System config saved");
    } else {
        ESP_LOGE(TAG, "❌ Failed to save system config: %s", esp_err_to_name(ret));
    }
    total_count++;

    // Save GPS configuration
    ret = save_gps_config_to_sd(&config->gps);
    if (ret == ESP_OK) {
        success_count++;
        ESP_LOGI(TAG, "✅ GPS config saved");
    } else {
        ESP_LOGE(TAG, "❌ Failed to save GPS config: %s", esp_err_to_name(ret));
    }
    total_count++;

    // Save MSP configuration
    ret = save_msp_config_to_sd(&config->msp);
    if (ret == ESP_OK) {
        success_count++;
        ESP_LOGI(TAG, "✅ MSP config saved");
    } else {
        ESP_LOGE(TAG, "❌ Failed to save MSP config: %s", esp_err_to_name(ret));
    }
    total_count++;

    // Save SLAM configuration
    ret = save_slam_config_to_sd(&config->slam);
    if (ret == ESP_OK) {
        success_count++;
        ESP_LOGI(TAG, "✅ SLAM config saved");
    } else {
        ESP_LOGE(TAG, "❌ Failed to save SLAM config: %s", esp_err_to_name(ret));
    }
    total_count++;

    // Save fusion configuration
    ret = save_fusion_config_to_sd(&config->fusion);
    if (ret == ESP_OK) {
        success_count++;
        ESP_LOGI(TAG, "✅ Fusion config saved");
    } else {
        ESP_LOGE(TAG, "❌ Failed to save fusion config: %s", esp_err_to_name(ret));
    }
    total_count++;

    // Save camera configuration
    ret = save_camera_config_to_sd(&config->camera);
    if (ret == ESP_OK) {
        success_count++;
        ESP_LOGI(TAG, "✅ Camera config saved");
    } else {
        ESP_LOGE(TAG, "❌ Failed to save camera config: %s", esp_err_to_name(ret));
    }
    total_count++;

    // Save IMU configuration
    ret = save_imu_config_to_sd(&config->imu);
    if (ret == ESP_OK) {
        success_count++;
        ESP_LOGI(TAG, "✅ IMU config saved");
    } else {
        ESP_LOGE(TAG, "❌ Failed to save IMU config: %s", esp_err_to_name(ret));
    }
    total_count++;

    ESP_LOGI(TAG, "Configuration save complete: %d/%d files saved successfully", success_count, total_count);

    if (success_count == total_count) {
        ESP_LOGI(TAG, "✅ All configuration files saved to SD card successfully");
        return ESP_OK;
    } else if (success_count > 0) {
        ESP_LOGW(TAG, "⚠️ Partial save: %d/%d configuration files saved", success_count, total_count);
        return ESP_ERR_INVALID_SIZE;
    } else {
        ESP_LOGE(TAG, "❌ Failed to save any configuration files");
        return ESP_FAIL;
    }
}

/**
 * Copy default configurations from SPIFFS to SD card
 */
esp_err_t config_loader_copy_defaults_to_sd(void) {
    if (!g_config_state.sd_available) {
        ESP_LOGE(TAG, "SD card not available for copying defaults");
        return ESP_ERR_NOT_SUPPORTED;
    }

    if (!g_config_state.spiffs_mounted) {
        ESP_LOGE(TAG, "SPIFFS not available for copying defaults");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Copying default configurations from SPIFFS to SD card...");

    // Create config directory
    esp_err_t ret = sd_storage_create_directory(SD_CONFIG_DIR);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to create config directory: %s", esp_err_to_name(ret));
        return ret;
    }

    // List of config files to copy
    const char* config_files[][2] = {
        {SYSTEM_CONFIG_FILE_SPIFFS, SYSTEM_CONFIG_FILE_SD},
        {GPS_CONFIG_FILE_SPIFFS, GPS_CONFIG_FILE_SD},
        {MSP_CONFIG_FILE_SPIFFS, MSP_CONFIG_FILE_SD},
        {SLAM_CONFIG_FILE_SPIFFS, SLAM_CONFIG_FILE_SD},
        {FUSION_CONFIG_FILE_SPIFFS, FUSION_CONFIG_FILE_SD},
        {CAMERA_CONFIG_FILE_SPIFFS, CAMERA_CONFIG_FILE_SD},
        {IMU_CONFIG_FILE_SPIFFS, IMU_CONFIG_FILE_SD}
    };

    int success_count = 0;
    int total_count = sizeof(config_files) / sizeof(config_files[0]);

    for (int i = 0; i < total_count; i++) {
        const char* src_file = config_files[i][0];
        const char* dst_file = config_files[i][1];

        // Read from SPIFFS
        char* buffer = NULL;
        size_t size = 0;
        ret = read_config_file_spiffs(src_file, &buffer, &size);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to read %s: %s", src_file, esp_err_to_name(ret));
            continue;
        }

        // Write to SD card
        FILE* dst_f = fopen(dst_file, "w");
        if (dst_f == NULL) {
            ESP_LOGW(TAG, "Failed to open %s for writing", dst_file);
            free(buffer);
            continue;
        }

        size_t written = fwrite(buffer, 1, size, dst_f);
        fclose(dst_f);
        free(buffer);

        if (written == size) {
            ESP_LOGI(TAG, "Copied %s -> %s", src_file, dst_file);
            success_count++;
        } else {
            ESP_LOGW(TAG, "Failed to write complete file %s", dst_file);
        }
    }

    ESP_LOGI(TAG, "Configuration copy complete: %d/%d files copied successfully", success_count, total_count);

    if (success_count == total_count) {
        return ESP_OK;
    } else if (success_count > 0) {
        return ESP_ERR_INVALID_SIZE; // Partial success
    } else {
        return ESP_FAIL;
    }
}

/**
 * Save system configuration to SPIFFS
 */
static esp_err_t save_system_config_to_spiffs(const system_config_t* config) {
    cJSON* root = cJSON_CreateObject();
    if (!root) return ESP_ERR_NO_MEM;

    cJSON* system = cJSON_AddObjectToObject(root, "system");
    if (!system) {
        cJSON_Delete(root);
        return ESP_ERR_NO_MEM;
    }

    cJSON_AddStringToObject(system, "name", config->name);
    cJSON_AddStringToObject(system, "version", config->version);
    cJSON_AddStringToObject(system, "description", config->description);
    cJSON_AddBoolToObject(system, "debug_mode", config->debug_mode);
    cJSON_AddStringToObject(system, "log_level", config->log_level);
    cJSON_AddBoolToObject(system, "enable_watchdog", config->enable_watchdog);
    cJSON_AddNumberToObject(system, "watchdog_timeout_ms", config->watchdog_timeout_ms);
    cJSON_AddBoolToObject(system, "enable_statistics", config->enable_statistics);
    cJSON_AddNumberToObject(system, "statistics_interval_ms", config->statistics_interval_ms);

    char* json_str = cJSON_Print(root);
    cJSON_Delete(root);

    if (!json_str) return ESP_ERR_NO_MEM;

    size_t json_len = strlen(json_str);
    FILE* f = fopen(SYSTEM_CONFIG_FILE_SPIFFS, "w");
    if (!f) {
        ESP_LOGE(TAG, "❌ Failed to open system config file: %s", SYSTEM_CONFIG_FILE_SPIFFS);
        free(json_str);
        return ESP_ERR_INVALID_ARG;
    }

    size_t written = fwrite(json_str, 1, json_len, f);
    if (written != json_len) {
        ESP_LOGE(TAG, "❌ Failed to write system config file: wrote %zu/%zu bytes", written, json_len);
    }
    fclose(f);
    free(json_str);

    return (written == json_len) ? ESP_OK : ESP_ERR_INVALID_SIZE;
}

/**
 * Save GPS configuration to SPIFFS
 */
static esp_err_t save_gps_config_to_spiffs(const extended_gps_config_t* config) {
    cJSON* root = cJSON_CreateObject();
    if (!root) return ESP_ERR_NO_MEM;

    cJSON* gps = cJSON_AddObjectToObject(root, "gps");
    if (!gps) {
        cJSON_Delete(root);
        return ESP_ERR_NO_MEM;
    }

    cJSON_AddNumberToObject(gps, "uart_port", config->base_config.uart_port);
    cJSON_AddNumberToObject(gps, "baud_rate", config->base_config.baud_rate);
    cJSON_AddNumberToObject(gps, "tx_pin", config->base_config.tx_pin);
    cJSON_AddNumberToObject(gps, "rx_pin", config->base_config.rx_pin);
    cJSON_AddBoolToObject(gps, "enable_sbas", config->enable_sbas);
    cJSON_AddBoolToObject(gps, "enable_differential", config->enable_differential);
    cJSON_AddNumberToObject(gps, "timeout_ms", config->timeout_ms);
    cJSON_AddBoolToObject(gps, "auto_reconnect", config->auto_reconnect);
    cJSON_AddNumberToObject(gps, "reconnect_attempts", config->reconnect_attempts);
    cJSON_AddNumberToObject(gps, "reconnect_delay_ms", config->reconnect_delay_ms);

    cJSON* filters = cJSON_AddObjectToObject(root, "gps_filters");
    if (filters) {
        cJSON_AddNumberToObject(filters, "minimum_satellites", config->minimum_satellites);
        cJSON_AddNumberToObject(filters, "maximum_hdop", config->maximum_hdop);
        cJSON_AddNumberToObject(filters, "maximum_speed_ms", config->maximum_speed_ms);
        cJSON_AddBoolToObject(filters, "speed_filter_enabled", config->speed_filter_enabled);
        cJSON_AddBoolToObject(filters, "altitude_filter_enabled", config->altitude_filter_enabled);
        cJSON_AddNumberToObject(filters, "minimum_altitude_m", config->minimum_altitude_m);
        cJSON_AddNumberToObject(filters, "maximum_altitude_m", config->maximum_altitude_m);
    }

    char* json_str = cJSON_Print(root);
    cJSON_Delete(root);

    if (!json_str) return ESP_ERR_NO_MEM;

    size_t json_len = strlen(json_str);
    FILE* f = fopen(GPS_CONFIG_FILE_SPIFFS, "w");
    if (!f) {
        ESP_LOGE(TAG, "❌ Failed to open GPS config file: %s", GPS_CONFIG_FILE_SPIFFS);
        free(json_str);
        return ESP_ERR_INVALID_ARG;
    }

    size_t written = fwrite(json_str, 1, json_len, f);
    if (written != json_len) {
        ESP_LOGE(TAG, "❌ Failed to write GPS config file: wrote %zu/%zu bytes", written, json_len);
    }
    fclose(f);
    free(json_str);

    return (written == json_len) ? ESP_OK : ESP_ERR_INVALID_SIZE;
}

/**
 * Save MSP configuration to SPIFFS
 */
static esp_err_t save_msp_config_to_spiffs(const extended_msp_config_t* config) {
    cJSON* root = cJSON_CreateObject();
    if (!root) return ESP_ERR_NO_MEM;

    cJSON* msp = cJSON_AddObjectToObject(root, "msp");
    if (!msp) {
        cJSON_Delete(root);
        return ESP_ERR_NO_MEM;
    }

    cJSON_AddNumberToObject(msp, "uart_port", config->base_config.uart_port);
    cJSON_AddNumberToObject(msp, "baud_rate", config->base_config.baud_rate);
    cJSON_AddNumberToObject(msp, "tx_pin", config->base_config.tx_pin);
    cJSON_AddNumberToObject(msp, "rx_pin", config->base_config.rx_pin);
    cJSON_AddNumberToObject(msp, "timeout_ms", config->base_config.timeout_ms);
    cJSON_AddBoolToObject(msp, "auto_reconnect", config->base_config.auto_reconnect);

    cJSON* commands = cJSON_AddObjectToObject(root, "msp_commands");
    if (commands) {
        cJSON_AddNumberToObject(commands, "status_request_interval_ms", config->status_request_interval_ms);
        cJSON_AddNumberToObject(commands, "navigation_status_interval_ms", config->navigation_status_interval_ms);
        cJSON_AddNumberToObject(commands, "attitude_request_interval_ms", config->attitude_request_interval_ms);
        cJSON_AddNumberToObject(commands, "altitude_request_interval_ms", config->altitude_request_interval_ms);
        cJSON_AddNumberToObject(commands, "gps_request_interval_ms", config->gps_request_interval_ms);
        cJSON_AddNumberToObject(commands, "analog_request_interval_ms", config->analog_request_interval_ms);
    }

    char* json_str = cJSON_Print(root);
    cJSON_Delete(root);

    if (!json_str) return ESP_ERR_NO_MEM;

    size_t json_len = strlen(json_str);
    FILE* f = fopen(MSP_CONFIG_FILE_SPIFFS, "w");
    if (!f) {
        ESP_LOGE(TAG, "❌ Failed to open MSP config file: %s", MSP_CONFIG_FILE_SPIFFS);
        free(json_str);
        return ESP_ERR_INVALID_ARG;
    }

    size_t written = fwrite(json_str, 1, json_len, f);
    if (written != json_len) {
        ESP_LOGE(TAG, "❌ Failed to write MSP config file: wrote %zu/%zu bytes", written, json_len);
    }
    fclose(f);
    free(json_str);

    return (written == json_len) ? ESP_OK : ESP_ERR_INVALID_SIZE;
}

/**
 * Save SLAM configuration to SPIFFS
 */
static esp_err_t save_slam_config_to_spiffs(const extended_slam_config_t* config) {
    cJSON* root = cJSON_CreateObject();
    if (!root) return ESP_ERR_NO_MEM;

    cJSON* slam = cJSON_AddObjectToObject(root, "slam");
    if (!slam) {
        cJSON_Delete(root);
        return ESP_ERR_NO_MEM;
    }

    cJSON_AddNumberToObject(slam, "max_features", config->base_config.max_features);
    cJSON_AddNumberToObject(slam, "fast_threshold", config->base_config.fast_threshold);
    cJSON_AddNumberToObject(slam, "levels", config->base_config.levels);
    cJSON_AddNumberToObject(slam, "scale_factor", config->base_config.scale_factor);
    cJSON_AddNumberToObject(slam, "max_keypoints_per_level", config->base_config.max_keypoints_per_level);
    cJSON_AddBoolToObject(slam, "use_harris_detector", config->base_config.use_harris_detector);
    cJSON_AddNumberToObject(slam, "harris_k", config->base_config.harris_k);
    cJSON_AddNumberToObject(slam, "descriptor_distance_threshold", config->base_config.descriptor_distance_threshold);
    cJSON_AddNumberToObject(slam, "match_threshold", config->base_config.match_threshold);
    cJSON_AddNumberToObject(slam, "min_tracked_features", config->base_config.min_tracked_features);
    cJSON_AddNumberToObject(slam, "keyframe_distance_threshold", config->base_config.keyframe_distance_threshold);
    cJSON_AddNumberToObject(slam, "keyframe_angle_threshold", config->base_config.keyframe_angle_threshold);
    cJSON_AddNumberToObject(slam, "max_keyframes", config->base_config.max_keyframes);
    cJSON_AddBoolToObject(slam, "enable_loop_closure", config->base_config.enable_loop_closure);
    cJSON_AddNumberToObject(slam, "loop_closure_threshold", config->base_config.loop_closure_threshold);
    cJSON_AddBoolToObject(slam, "map_optimization_enabled", config->base_config.map_optimization_enabled);

    char* json_str = cJSON_Print(root);
    cJSON_Delete(root);

    if (!json_str) return ESP_ERR_NO_MEM;

    size_t json_len = strlen(json_str);
    FILE* f = fopen(SLAM_CONFIG_FILE_SPIFFS, "w");
    if (!f) {
        ESP_LOGE(TAG, "❌ Failed to open SLAM config file: %s", SLAM_CONFIG_FILE_SPIFFS);
        free(json_str);
        return ESP_ERR_INVALID_ARG;
    }

    size_t written = fwrite(json_str, 1, json_len, f);
    if (written != json_len) {
        ESP_LOGE(TAG, "❌ Failed to write SLAM config file: wrote %zu/%zu bytes", written, json_len);
    }
    fclose(f);
    free(json_str);

    return (written == json_len) ? ESP_OK : ESP_ERR_INVALID_SIZE;
}

/**
 * Save fusion configuration to SPIFFS
 */
static esp_err_t save_fusion_config_to_spiffs(const extended_fusion_config_t* config) {
    cJSON* root = cJSON_CreateObject();
    if (!root) return ESP_ERR_NO_MEM;

    cJSON* fusion = cJSON_AddObjectToObject(root, "fusion");
    if (!fusion) {
        cJSON_Delete(root);
        return ESP_ERR_NO_MEM;
    }

    cJSON_AddNumberToObject(fusion, "accel_bias_noise", config->base_config.accel_bias_noise);
    cJSON_AddNumberToObject(fusion, "gyro_bias_noise", config->base_config.gyro_bias_noise);
    cJSON_AddNumberToObject(fusion, "slam_attitude_noise", config->base_config.slam_attitude_noise);
    cJSON_AddNumberToObject(fusion, "gps_position_noise", config->base_config.gps_position_noise);
    cJSON_AddNumberToObject(fusion, "gps_velocity_noise", config->base_config.gps_velocity_noise);
    cJSON_AddNumberToObject(fusion, "slam_position_noise", config->base_config.slam_position_noise);
    cJSON_AddNumberToObject(fusion, "imu_dt", config->base_config.imu_dt);
    cJSON_AddNumberToObject(fusion, "max_prediction_time", config->base_config.max_prediction_time);

    cJSON* weights = cJSON_AddObjectToObject(root, "fusion_weights");
    if (weights) {
        cJSON_AddNumberToObject(weights, "gps_weight", config->base_config.gps_weight);
        cJSON_AddNumberToObject(weights, "slam_weight", config->base_config.slam_weight);
        cJSON_AddNumberToObject(weights, "imu_weight", config->base_config.imu_weight);
        cJSON_AddNumberToObject(weights, "gps_rejection_threshold", config->gps_rejection_threshold);
        cJSON_AddNumberToObject(weights, "slam_rejection_threshold", config->slam_rejection_threshold);
        cJSON_AddNumberToObject(weights, "imu_rejection_threshold", config->imu_rejection_threshold);
    }

    char* json_str = cJSON_Print(root);
    cJSON_Delete(root);

    if (!json_str) return ESP_ERR_NO_MEM;

    size_t json_len = strlen(json_str);
    FILE* f = fopen(FUSION_CONFIG_FILE_SPIFFS, "w");
    if (!f) {
        ESP_LOGE(TAG, "❌ Failed to open fusion config file: %s", FUSION_CONFIG_FILE_SPIFFS);
        free(json_str);
        return ESP_ERR_INVALID_ARG;
    }

    size_t written = fwrite(json_str, 1, json_len, f);
    if (written != json_len) {
        ESP_LOGE(TAG, "❌ Failed to write fusion config file: wrote %zu/%zu bytes", written, json_len);
    }
    fclose(f);
    free(json_str);

    return (written == json_len) ? ESP_OK : ESP_ERR_INVALID_SIZE;
}

/**
 * Save camera configuration to SPIFFS
 */
static esp_err_t save_camera_config_to_spiffs(const extended_camera_config_t* config) {
    cJSON* root = cJSON_CreateObject();
    if (!root) return ESP_ERR_NO_MEM;

    cJSON* camera = cJSON_AddObjectToObject(root, "camera");
    if (!camera) {
        cJSON_Delete(root);
        return ESP_ERR_NO_MEM;
    }

    cJSON_AddNumberToObject(camera, "resolution", config->base_config.resolution);
    cJSON_AddNumberToObject(camera, "format", config->base_config.format);
    cJSON_AddNumberToObject(camera, "fps", config->base_config.fps);
    cJSON_AddBoolToObject(camera, "auto_exposure", config->base_config.auto_exposure);
    cJSON_AddBoolToObject(camera, "auto_white_balance", config->base_config.auto_white_balance);
    cJSON_AddNumberToObject(camera, "brightness", config->base_config.brightness);
    cJSON_AddNumberToObject(camera, "contrast", config->base_config.contrast);
    cJSON_AddNumberToObject(camera, "saturation", config->base_config.saturation);
    cJSON_AddNumberToObject(camera, "exposure_value", config->base_config.exposure_value);
    cJSON_AddBoolToObject(camera, "auto_adjustment_enabled", config->base_config.auto_adjustment_enabled);
    cJSON_AddNumberToObject(camera, "target_brightness", config->base_config.target_brightness);
    cJSON_AddNumberToObject(camera, "adjustment_speed", config->base_config.adjustment_speed);
    cJSON_AddNumberToObject(camera, "brightness_threshold", config->base_config.brightness_threshold);

    char* json_str = cJSON_Print(root);
    cJSON_Delete(root);

    if (!json_str) return ESP_ERR_NO_MEM;

    size_t json_len = strlen(json_str);
    FILE* f = fopen(CAMERA_CONFIG_FILE_SPIFFS, "w");
    if (!f) {
        ESP_LOGE(TAG, "❌ Failed to open camera config file: %s", CAMERA_CONFIG_FILE_SPIFFS);
        free(json_str);
        return ESP_ERR_INVALID_ARG;
    }

    size_t written = fwrite(json_str, 1, json_len, f);
    if (written != json_len) {
        ESP_LOGE(TAG, "❌ Failed to write camera config file: wrote %zu/%zu bytes", written, json_len);
    }
    fclose(f);
    free(json_str);

    return (written == json_len) ? ESP_OK : ESP_ERR_INVALID_SIZE;
}

/**
 * Save IMU configuration to SPIFFS
 */
static esp_err_t save_imu_config_to_spiffs(const extended_imu_config_t* config) {
    cJSON* root = cJSON_CreateObject();
    if (!root) return ESP_ERR_NO_MEM;

    cJSON* imu = cJSON_AddObjectToObject(root, "imu");
    if (!imu) {
        cJSON_Delete(root);
        return ESP_ERR_NO_MEM;
    }

    cJSON_AddNumberToObject(imu, "spi_host", config->base_config.spi_host);
    cJSON_AddNumberToObject(imu, "miso_pin", config->base_config.miso_pin);
    cJSON_AddNumberToObject(imu, "mosi_pin", config->base_config.mosi_pin);
    cJSON_AddNumberToObject(imu, "sclk_pin", config->base_config.sclk_pin);
    cJSON_AddNumberToObject(imu, "acc_cs_pin", config->base_config.acc_cs_pin);
    cJSON_AddNumberToObject(imu, "gyro_cs_pin", config->base_config.gyro_cs_pin);
    cJSON_AddNumberToObject(imu, "accel_range", config->base_config.accel_range);
    cJSON_AddNumberToObject(imu, "gyro_range", config->base_config.gyro_range);
    cJSON_AddNumberToObject(imu, "sample_rate", config->base_config.sample_rate);

    char* json_str = cJSON_Print(root);
    cJSON_Delete(root);

    if (!json_str) return ESP_ERR_NO_MEM;

    size_t json_len = strlen(json_str);
    FILE* f = fopen(IMU_CONFIG_FILE_SPIFFS, "w");
    if (!f) {
        ESP_LOGE(TAG, "❌ Failed to open IMU config file: %s", IMU_CONFIG_FILE_SPIFFS);
        free(json_str);
        return ESP_ERR_INVALID_ARG;
    }

    size_t written = fwrite(json_str, 1, json_len, f);
    if (written != json_len) {
        ESP_LOGE(TAG, "❌ Failed to write IMU config file: wrote %zu/%zu bytes", written, json_len);
    }
    fclose(f);
    free(json_str);

    return (written == json_len) ? ESP_OK : ESP_ERR_INVALID_SIZE;
}

/**
 * Create default configuration files in SPIFFS
 * Now reads from config_defaults/ directory instead of embedded strings
 */
esp_err_t config_loader_create_defaults(void) {
    ESP_LOGI(TAG, "Creating default configuration files...");

    // List of config files to create
    const struct {
        const char* default_json;
        const char* spiffs_path;
        const char* sd_path;
        const char* name;
    } config_files[] = {
        {DEFAULT_SYSTEM_CONFIG_JSON, SYSTEM_CONFIG_FILE_SPIFFS, SYSTEM_CONFIG_FILE_SD, "system"},
        {DEFAULT_GPS_CONFIG_JSON, GPS_CONFIG_FILE_SPIFFS, GPS_CONFIG_FILE_SD, "GPS"},
        {DEFAULT_MSP_CONFIG_JSON, MSP_CONFIG_FILE_SPIFFS, MSP_CONFIG_FILE_SD, "MSP"},
        {DEFAULT_SLAM_CONFIG_JSON, SLAM_CONFIG_FILE_SPIFFS, SLAM_CONFIG_FILE_SD, "SLAM"},
        {DEFAULT_FUSION_CONFIG_JSON, FUSION_CONFIG_FILE_SPIFFS, FUSION_CONFIG_FILE_SD, "fusion"},
        {DEFAULT_CAMERA_CONFIG_JSON, CAMERA_CONFIG_FILE_SPIFFS, CAMERA_CONFIG_FILE_SD, "camera"},
        {DEFAULT_IMU_CONFIG_JSON, IMU_CONFIG_FILE_SPIFFS, IMU_CONFIG_FILE_SD, "IMU"}
    };

    int spiffs_success_count = 0;
    int sd_success_count = 0;
    int total_count = sizeof(config_files) / sizeof(config_files[0]);

    // Create defaults based on storage availability priority
    if (g_config_state.sd_available) {
        // SD card is primary storage - create defaults there
        ESP_LOGI(TAG, "Creating default configuration files on SD card (primary storage)...");
        for (int i = 0; i < total_count; i++) {
            size_t size = strlen(config_files[i].default_json);
            FILE* dst_f = fopen(config_files[i].sd_path, "w");
            if (dst_f == NULL) {
                ESP_LOGW(TAG, "Failed to open SD card file for writing: %s", config_files[i].sd_path);
                continue;
            }

            size_t written = fwrite(config_files[i].default_json, 1, size, dst_f);
            fclose(dst_f);

            if (written != size) {
                ESP_LOGW(TAG, "Failed to write complete %s config file to SD card", config_files[i].name);
            } else {
                sd_success_count++;
                ESP_LOGI(TAG, "✅ Created %s config on SD card", config_files[i].name);
            }
        }

        // Only create SPIFFS backup if SD card creation was incomplete
        if (sd_success_count < total_count && g_config_state.spiffs_mounted) {
            ESP_LOGW(TAG, "SD card creation incomplete (%d/%d), creating SPIFFS backup...", sd_success_count, total_count);
            for (int i = 0; i < total_count; i++) {
                // Check if this config already exists on SD card
                FILE* check_f = fopen(config_files[i].sd_path, "r");
                if (check_f != NULL) {
                    fclose(check_f);
                    continue; // Already exists on SD card
                }

                esp_err_t ret = copy_default_to_spiffs(config_files[i].default_json, config_files[i].spiffs_path);
                if (ret == ESP_OK) {
                    spiffs_success_count++;
                    ESP_LOGI(TAG, "✅ Created %s config backup in SPIFFS", config_files[i].name);
                }
            }
        }
    } else if (g_config_state.spiffs_mounted) {
        // No SD card - use SPIFFS as primary storage
        ESP_LOGI(TAG, "No SD card available, creating default configuration files in SPIFFS...");
        for (int i = 0; i < total_count; i++) {
            esp_err_t ret = copy_default_to_spiffs(config_files[i].default_json, config_files[i].spiffs_path);
            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "Failed to create %s config in SPIFFS: %s", config_files[i].name, esp_err_to_name(ret));
            } else {
                spiffs_success_count++;
            }
        }
    } else {
        ESP_LOGE(TAG, "❌ Neither SPIFFS nor SD card available for configuration storage");
        return ESP_ERR_NOT_SUPPORTED;
    }

    ESP_LOGI(TAG, "Configuration creation complete:");
    ESP_LOGI(TAG, "  SPIFFS: %d/%d files created successfully", spiffs_success_count, total_count);
    ESP_LOGI(TAG, "  SD card: %d/%d files created successfully", sd_success_count, total_count);

    // Success criteria: All configs created in primary storage (SD card if available, otherwise SPIFFS)
    bool primary_storage_success = (g_config_state.sd_available && sd_success_count == total_count) ||
                                  (!g_config_state.sd_available && spiffs_success_count == total_count);

    if (primary_storage_success) {
        ESP_LOGI(TAG, "✅ All default configuration files created successfully");
        return ESP_OK;
    } else if (spiffs_success_count > 0 || sd_success_count > 0) {
        ESP_LOGW(TAG, "⚠️ Some configuration files failed to create, but system can continue with available configs");
        return ESP_ERR_INVALID_SIZE; // Partial success
    } else {
        ESP_LOGE(TAG, "❌ Failed to create any configuration files");
        return ESP_FAIL;
    }
}

/**
 * Get configuration storage information
 */
esp_err_t config_loader_get_storage_info(bool* spiffs_available, bool* sd_available, uint64_t* sd_free_space_mb) {
    *spiffs_available = g_config_state.spiffs_mounted;
    *sd_available = g_config_state.sd_available;

    if (sd_available && *sd_available) {
        sd_card_info_t sd_info;
        esp_err_t ret = sd_storage_get_info(&sd_info);
        if (ret == ESP_OK) {
            *sd_free_space_mb = sd_info.free_size_mb;
        } else {
            *sd_free_space_mb = 0;
            return ret;
        }
    } else {
        *sd_free_space_mb = 0;
    }

    return ESP_OK;
}

/**
 * Save master configuration to SD card
 */
static esp_err_t save_master_config_to_sd(const master_config_t* config) {
    if (!g_config_state.sd_available) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    // Save individual config files
    esp_err_t ret;

    ret = save_system_config_to_sd(&config->system);
    if (ret != ESP_OK) return ret;

    ret = save_gps_config_to_sd(&config->gps);
    if (ret != ESP_OK) return ret;

    ret = save_msp_config_to_sd(&config->msp);
    if (ret != ESP_OK) return ret;

    ret = save_slam_config_to_sd(&config->slam);
    if (ret != ESP_OK) return ret;

    ret = save_fusion_config_to_sd(&config->fusion);
    if (ret != ESP_OK) return ret;

    ret = save_camera_config_to_sd(&config->camera);
    if (ret != ESP_OK) return ret;

    ret = save_imu_config_to_sd(&config->imu);
    if (ret != ESP_OK) return ret;

    ESP_LOGI(TAG, "✅ Saved master config to SD card");
    return ESP_OK;
}

/**
 * Save master configuration to SPIFFS
 */
static esp_err_t save_master_config_to_spiffs(const master_config_t* config) {
    // Save individual config files to SPIFFS
    esp_err_t ret;

    ret = save_system_config_to_spiffs(&config->system);
    if (ret != ESP_OK) return ret;

    ret = save_gps_config_to_spiffs(&config->gps);
    if (ret != ESP_OK) return ret;

    ret = save_msp_config_to_spiffs(&config->msp);
    if (ret != ESP_OK) return ret;

    ret = save_slam_config_to_spiffs(&config->slam);
    if (ret != ESP_OK) return ret;

    ret = save_fusion_config_to_spiffs(&config->fusion);
    if (ret != ESP_OK) return ret;

    ret = save_camera_config_to_spiffs(&config->camera);
    if (ret != ESP_OK) return ret;

    ret = save_imu_config_to_spiffs(&config->imu);
    if (ret != ESP_OK) return ret;

    ESP_LOGI(TAG, "✅ Saved master config to SPIFFS");
    return ESP_OK;
}
