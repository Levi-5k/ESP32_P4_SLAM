/**
 * @file parameter_validator.h
 * @brief Safe parameter validation for real-time updates
 */

#pragma once

#include "sensor_fusion.h"
#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Parameter validation result
 */
typedef enum {
    PARAM_VALID = 0,
    PARAM_OUT_OF_RANGE,
    PARAM_INVALID_COMBINATION,
    PARAM_UNSAFE_DURING_FLIGHT
} param_validation_result_t;

/**
 * @brief Validate fusion configuration parameters
 * @param config Configuration to validate
 * @param is_flying Whether system is currently in flight
 * @return Validation result
 */
param_validation_result_t validate_fusion_config(const fusion_config_t* config, bool is_flying);

/**
 * @brief Apply parameter changes safely during operation
 * @param new_config New configuration parameters
 * @param current_state Current filter state
 * @return ESP_OK on success
 */
esp_err_t apply_parameters_safely(const fusion_config_t* new_config, bool preserve_state);

/**
 * @brief Get parameter change recommendations based on flight conditions
 * @param flight_mode Current flight mode
 * @param config_out Recommended configuration
 * @return ESP_OK on success
 */
esp_err_t get_recommended_config(uint8_t flight_mode, fusion_config_t* config_out);

#ifdef __cplusplus
}
#endif
