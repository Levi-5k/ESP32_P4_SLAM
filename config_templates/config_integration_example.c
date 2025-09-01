/*
 * Configuration Integration Example
 * Shows how to integrate the config loader into the main application
 */

#include <esp_log.h>
#include <esp_err.h>
#include "config_loader.h"
#include "gps_ublox.h"
#include "msp_protocol.h"
#include "slam_core.h"
#include "sensor_fusion.h"

static const char* TAG = "CONFIG_INTEGRATION";

esp_err_t initialize_system_with_config(void) {
    esp_err_t ret;

    ESP_LOGI(TAG, "Initializing system with configuration files...");

    // Initialize SPIFFS for configuration files
    ret = config_loader_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize config loader: %s", esp_err_to_name(ret));
        return ret;
    }

    // Check storage availability
    bool spiffs_ok, sd_ok;
    uint64_t sd_free_mb;
    config_loader_get_storage_info(&spiffs_ok, &sd_ok, &sd_free_mb);
    ESP_LOGI(TAG, "Storage Status - SPIFFS: %s, SD Card: %s (%llu MB free)",
             spiffs_ok ? "Available" : "Unavailable",
             sd_ok ? "Available" : "Unavailable", sd_free_mb);

    // Copy defaults to SD card if SD is available but configs don't exist
    if (sd_ok && !config_loader_is_sd_available()) {
        ESP_LOGI(TAG, "Copying default configurations to SD card...");
        ret = config_loader_copy_defaults_to_sd();
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to copy defaults to SD card: %s", esp_err_to_name(ret));
            // Continue anyway - will use SPIFFS defaults
        }
    }

    // Load master configuration
    master_config_t master_config = {0};
    ret = config_loader_load_all(&master_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to load master configuration: %s", esp_err_to_name(ret));
        return ret;
    }

    // Validate configuration
    ret = config_loader_validate_config(&master_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Configuration validation failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Print configuration summary
    config_loader_print_config(&master_config);

    // Initialize GPS with loaded configuration
    ret = gps_ublox_init(&master_config.gps.base_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize GPS: %s", esp_err_to_name(ret));
        return ret;
    }

    // Initialize MSP protocol with loaded configuration
    ret = msp_protocol_init_config(&master_config.msp.base_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize MSP: %s", esp_err_to_name(ret));
        return ret;
    }

    // Initialize SLAM core with loaded configuration
    ret = slam_core_init(&master_config.slam.base_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SLAM: %s", esp_err_to_name(ret));
        return ret;
    }

    // Initialize sensor fusion with loaded configuration
    ret = sensor_fusion_init(&master_config.fusion.base_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize sensor fusion: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "✅ System initialized successfully with configuration");
    return ESP_OK;
}

esp_err_t reload_configuration(void) {
    ESP_LOGI(TAG, "Reloading configuration from files...");

    // Load updated configuration
    master_config_t new_config = {0};
    esp_err_t ret = config_loader_load_all(&new_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to reload configuration: %s", esp_err_to_name(ret));
        return ret;
    }

    // Validate new configuration
    ret = config_loader_validate_config(&new_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "New configuration validation failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Apply new configuration to running components
    // Note: This would require component-specific update functions
    // For now, just log the successful reload
    ESP_LOGI(TAG, "✅ Configuration reloaded successfully");
    config_loader_print_config(&new_config);

    return ESP_OK;
}

/*
 * Usage in main application:
 *
 * void app_main(void) {
 *     // Initialize system with configuration
 *     esp_err_t ret = initialize_system_with_config();
 *     if (ret != ESP_OK) {
 *         ESP_LOGE(TAG, "System initialization failed");
 *         return;
 *     }
 *
 *     // Main application loop
 *     while (1) {
 *         // Your main application logic here
 *         vTaskDelay(pdMS_TO_TICKS(100));
 *     }
 * }
 */
