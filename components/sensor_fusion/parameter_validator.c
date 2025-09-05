/**
 * @file parameter_validator.c
 * @brief Safe parameter validation implementation
 */

#include "parameter_validator.h"
#include "esp_log.h"
#include <math.h>

static const char* TAG = "PARAM_VALIDATOR";

// Parameter ranges for safety validation
#define MIN_POSITION_NOISE    0.001f
#define MAX_POSITION_NOISE    10.0f
#define MIN_VELOCITY_NOISE    0.0001f
#define MAX_VELOCITY_NOISE    5.0f
#define MIN_ATTITUDE_NOISE    0.0001f
#define MAX_ATTITUDE_NOISE    1.0f
#define MIN_WEIGHT            0.0f
#define MAX_WEIGHT            1.0f
#define MIN_OUTLIER_THRESHOLD 0.1f
#define MAX_OUTLIER_THRESHOLD 50.0f

param_validation_result_t validate_fusion_config(const fusion_config_t* config, bool is_flying) {
    if (!config) {
        return PARAM_INVALID_COMBINATION;
    }
    
    // Validate noise parameters
    if (config->position_noise < MIN_POSITION_NOISE || config->position_noise > MAX_POSITION_NOISE) {
        ESP_LOGW(TAG, "Position noise out of range: %.6f", config->position_noise);
        return PARAM_OUT_OF_RANGE;
    }
    
    if (config->velocity_noise < MIN_VELOCITY_NOISE || config->velocity_noise > MAX_VELOCITY_NOISE) {
        ESP_LOGW(TAG, "Velocity noise out of range: %.6f", config->velocity_noise);
        return PARAM_OUT_OF_RANGE;
    }
    
    if (config->attitude_noise < MIN_ATTITUDE_NOISE || config->attitude_noise > MAX_ATTITUDE_NOISE) {
        ESP_LOGW(TAG, "Attitude noise out of range: %.6f", config->attitude_noise);
        return PARAM_OUT_OF_RANGE;
    }
    
    // Validate fusion weights
    if (config->gps_weight < MIN_WEIGHT || config->gps_weight > MAX_WEIGHT ||
        config->slam_weight < MIN_WEIGHT || config->slam_weight > MAX_WEIGHT ||
        config->imu_weight < MIN_WEIGHT || config->imu_weight > MAX_WEIGHT) {
        ESP_LOGW(TAG, "Fusion weights out of range");
        return PARAM_OUT_OF_RANGE;
    }
    
    // Validate outlier thresholds
    if (config->gps_outlier_threshold < MIN_OUTLIER_THRESHOLD || 
        config->gps_outlier_threshold > MAX_OUTLIER_THRESHOLD) {
        ESP_LOGW(TAG, "GPS outlier threshold out of range: %.2f", config->gps_outlier_threshold);
        return PARAM_OUT_OF_RANGE;
    }
    
    // Check for potentially unsafe combinations during flight
    if (is_flying) {
        // Don't allow extremely high noise values during flight (could destabilize filter)
        if (config->position_noise > 1.0f || config->velocity_noise > 0.5f) {
            ESP_LOGW(TAG, "High noise values during flight may be unsafe");
            return PARAM_UNSAFE_DURING_FLIGHT;
        }
        
        // Don't allow disabling all data sources during flight
        if (config->gps_weight < 0.1f && config->slam_weight < 0.1f) {
            ESP_LOGW(TAG, "Cannot disable both GPS and SLAM during flight");
            return PARAM_UNSAFE_DURING_FLIGHT;
        }
    }
    
    ESP_LOGI(TAG, "✅ Parameter validation passed");
    return PARAM_VALID;
}

esp_err_t apply_parameters_safely(const fusion_config_t* new_config, bool preserve_state) {
    param_validation_result_t validation = validate_fusion_config(new_config, preserve_state);
    
    if (validation != PARAM_VALID) {
        ESP_LOGE(TAG, "❌ Parameter validation failed: %d", validation);
        return ESP_ERR_INVALID_ARG;
    }
    
    // Apply parameters using thread-safe sensor fusion function
    esp_err_t ret = sensor_fusion_set_config(new_config);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "✅ Parameters updated safely during operation");
    } else {
        ESP_LOGE(TAG, "❌ Failed to apply parameters: %s", esp_err_to_name(ret));
    }
    
    return ret;
}

esp_err_t get_recommended_config(uint8_t flight_mode, fusion_config_t* config_out) {
    if (!config_out) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Get current configuration as baseline
    esp_err_t ret = sensor_fusion_get_config(config_out);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Adjust parameters based on flight mode
    switch (flight_mode) {
        case 0: // MANUAL/STABILIZE - More responsive
            config_out->position_noise = 0.15f;
            config_out->velocity_noise = 0.02f;
            config_out->gps_weight = 0.7f;
            config_out->slam_weight = 0.8f;
            break;
            
        case 1: // ALTITUDE HOLD - Balanced
            config_out->position_noise = 0.1f;
            config_out->velocity_noise = 0.01f;
            config_out->gps_weight = 0.8f;
            config_out->slam_weight = 0.9f;
            break;
            
        case 2: // POSITION HOLD - More stable
            config_out->position_noise = 0.05f;
            config_out->velocity_noise = 0.005f;
            config_out->gps_weight = 0.9f;
            config_out->slam_weight = 0.7f;
            break;
            
        case 3: // AUTONOMOUS - Precision mode
            config_out->position_noise = 0.02f;
            config_out->velocity_noise = 0.002f;
            config_out->gps_weight = 0.6f;
            config_out->slam_weight = 0.95f;
            config_out->enable_outlier_detection = true;
            break;
            
        default:
            ESP_LOGW(TAG, "Unknown flight mode: %d, using defaults", flight_mode);
            break;
    }
    
    ESP_LOGI(TAG, "✅ Generated recommended config for flight mode %d", flight_mode);
    return ESP_OK;
}
