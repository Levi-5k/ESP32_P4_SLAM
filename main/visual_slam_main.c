/**
 * Visual SLAM Navigation Module for INAV
 * ESP32-P4-WIFI6 with OV5647 MIPI-CSI Camera
 * Complete navigation system with real-time web interface
 */

#include <stdio.h>
#include <string.h>
#include <time.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <freertos/event_groups.h>
#include <esp_log.h>
#include <esp_system.h>
// #include <esp_wifi.h>  // WIFI remote disabled temporarily
#include <driver/spi_master.h>
#include <esp_event.h>
#include <nvs_flash.h>
#include <driver/gpio.h>
#include <esp_timer.h>
#include <esp_heap_caps.h>

// Visual SLAM Components
#include "visual_slam_common_types.h"
#include "slam_core.h"
#include "orb_features.h"
#include "sensor_fusion.h"
#include "web_server.h"  // WIFI remote enabled
#include "sd_storage.h"
#include "gps_ublox.h"    // GPS uBlox module
#include "imu_bmi088.h"   // BMI088 IMU sensor
#include "msp_protocol.h" // MSP protocol for INAV
#include "config_loader.h" // Configuration management
#include "assistnow.h"    // AssistNow Offline GPS enhancement
#include "wifi_assistnow.h" // WiFi for AssistNow downloads
#include "wifi_positioning.h" // WiFi positioning using WiGLE database

// Centralized pin configuration
#include "esp32p4_pin_config.h"

static const char *TAG = "VisualSLAM_INAV";

// System synchronization
static SemaphoreHandle_t system_mutex;
static EventGroupHandle_t system_event_group;

// System state flags
#define CAMERA_READY_BIT        BIT0
#define GPS_READY_BIT          BIT1
#define IMU_READY_BIT          BIT2
#define SLAM_READY_BIT         BIT3
#define WEB_SERVER_READY_BIT   BIT4  // WIFI remote enabled
#define ALL_SYSTEMS_READY      (CAMERA_READY_BIT | GPS_READY_BIT | IMU_READY_BIT | SLAM_READY_BIT)

// Task handles
static TaskHandle_t main_processing_task_handle = NULL;
static TaskHandle_t web_interface_task_handle = NULL;
static TaskHandle_t telemetry_broadcast_task_handle = NULL;
static TaskHandle_t sensor_monitoring_task_handle = NULL;

// System status instance
static system_status_t system_status = {0};

// WIFI remote functionality
// static bool wifi_enabled = false;
// static bool wifi_connected = false;
// static SemaphoreHandle_t wifi_mutex = NULL;

// Forward declarations for WIFI functions
// static void IRAM_ATTR wifi_toggle_button_isr(void* arg);
// static void wifi_toggle_task(void* arg);
// static esp_err_t wifi_enable(void);
// static esp_err_t wifi_disable(void);

// Master configuration instance
static master_config_t master_config = {0};

/**
 * Initialize NVS (Non-Volatile Storage)
 */
static void initialize_nvs(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "✅ NVS initialized");
}

/**
 * Initialize system GPIO pins
 */
static void initialize_gpio(void)
{
    // LED indicators for system status
    gpio_config_t gpio_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << SYSTEM_STATUS_LED_PIN) |
                       (1ULL << SLAM_ACTIVITY_LED_PIN) |
                       (1ULL << COMMUNICATION_LED_PIN),
        .pull_down_en = 0,
        .pull_up_en = 0,
    };
    gpio_config(&gpio_conf);

    // WIFI toggle button input
    // gpio_config_t button_conf = {
    //     .intr_type = GPIO_INTR_NEGEDGE,  // Trigger on falling edge (button press)
    //     .mode = GPIO_MODE_INPUT,
    //     .pin_bit_mask = (1ULL << WIFI_TOGGLE_BUTTON_PIN),
    //     .pull_down_en = 0,
    //     .pull_up_en = 1,  // Enable internal pull-up resistor
    // };
    // gpio_config(&button_conf);

    // Install GPIO ISR service for button interrupt
    // gpio_install_isr_service(0);
    // gpio_isr_handler_add(WIFI_TOGGLE_BUTTON_PIN, wifi_toggle_button_isr, NULL);

    // Set initial LED states
    gpio_set_level(SYSTEM_STATUS_LED_PIN, 0);   // System status LED
    gpio_set_level(SLAM_ACTIVITY_LED_PIN, 0);   // SLAM activity LED
    gpio_set_level(COMMUNICATION_LED_PIN, 0);   // Communication LED

    ESP_LOGI(TAG, "✅ GPIO initialized - Status LEDs: GPIO%d, SLAM: GPIO%d, Comm: GPIO%d",
             SYSTEM_STATUS_LED_PIN, SLAM_ACTIVITY_LED_PIN, COMMUNICATION_LED_PIN);
}

/**
 * Save current configuration to SD card
 */
static esp_err_t save_configuration_to_sd(void)
{
    if (!master_config.loaded) {
        ESP_LOGW(TAG, "⚠️ No configuration loaded, cannot save to SD card");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "💾 Saving configuration to SD card...");
    esp_err_t ret = config_loader_save_to_sd(&master_config);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "✅ Configuration saved to SD card successfully");
    } else {
        ESP_LOGE(TAG, "❌ Failed to save configuration to SD card: %s", esp_err_to_name(ret));
    }

    return ret;
}

/**
 * Initialize camera system (OV5647 MIPI-CSI)
 */
static esp_err_t initialize_camera(void)
{
    ESP_LOGI(TAG, "🎥 Initializing OV5647 MIPI-CSI camera...");
    
    // Initialize camera configuration - use loaded config if available
    camera_config_t camera_config;
    if (master_config.loaded) {
        ESP_LOGI(TAG, "📷 Using loaded camera configuration");
        camera_config.resolution = master_config.camera.base_config.resolution;
        camera_config.format = master_config.camera.base_config.format;
        camera_config.fps = master_config.camera.base_config.fps;
        camera_config.auto_exposure = master_config.camera.base_config.auto_exposure;
        camera_config.auto_white_balance = master_config.camera.base_config.auto_white_balance;
        camera_config.brightness = master_config.camera.base_config.brightness;
        camera_config.contrast = master_config.camera.base_config.contrast;
        camera_config.saturation = master_config.camera.base_config.saturation;
        camera_config.exposure_value = master_config.camera.base_config.exposure_value;
        camera_config.auto_adjustment_enabled = master_config.camera.base_config.auto_adjustment_enabled;
        camera_config.target_brightness = master_config.camera.base_config.target_brightness;
        camera_config.adjustment_speed = master_config.camera.base_config.adjustment_speed;
        camera_config.brightness_threshold = master_config.camera.base_config.brightness_threshold;
    } else {
        ESP_LOGI(TAG, "📷 Using default camera configuration");
        // Use hardcoded defaults
        camera_config.resolution = CAMERA_RES_640x480;
        camera_config.format = CAMERA_FORMAT_RGB565;
        camera_config.fps = 30;
        camera_config.auto_exposure = true;
        camera_config.auto_white_balance = true;
        camera_config.brightness = 0;
        camera_config.contrast = 0;
        camera_config.saturation = 0;
        camera_config.exposure_value = 100;
        camera_config.auto_adjustment_enabled = true;
        camera_config.target_brightness = 128;
        camera_config.adjustment_speed = 0.3f;
        camera_config.brightness_threshold = 20.0f;
    }
    
    esp_err_t ret = slam_core_init_camera(&camera_config);
    if (ret == ESP_OK) {
        system_status.camera_initialized = true;
        xEventGroupSetBits(system_event_group, CAMERA_READY_BIT);
        ESP_LOGI(TAG, "✅ Camera initialized successfully");
    } else {
        ESP_LOGE(TAG, "❌ Camera initialization failed: %s", esp_err_to_name(ret));
    }
    
    return ret;
}

/**
 * Initialize GPS module (uBlox)
 */
static esp_err_t initialize_gps(void)
{
    ESP_LOGI(TAG, "🛰️ Initializing uBlox GPS module...");
    
    // Initialize GPS configuration - use loaded config if available
    gps_ublox_config_t gps_config;
    if (master_config.loaded) {
        ESP_LOGI(TAG, "🛰️ Using loaded GPS configuration");
        gps_config.uart_port = master_config.gps.base_config.uart_port;
        gps_config.baud_rate = master_config.gps.base_config.baud_rate;
        gps_config.tx_pin = master_config.gps.base_config.tx_pin;
        gps_config.rx_pin = master_config.gps.base_config.rx_pin;
        gps_config.update_rate_hz = 5;  // Default 5Hz
        gps_config.enable_sbas = false;
        gps_config.enable_differential = false;
        gps_config.timeout_ms = 10000;
    } else {
        ESP_LOGI(TAG, "🛰️ Using default GPS configuration");
        // Use hardcoded defaults
        gps_config.uart_port = UART_NUM_1;
        gps_config.baud_rate = 9600;
        gps_config.tx_pin = GPS_UART_TX_PIN;
        gps_config.rx_pin = GPS_UART_RX_PIN;
        gps_config.update_rate_hz = 5;
        gps_config.enable_sbas = false;
        gps_config.enable_differential = false;
        gps_config.timeout_ms = 10000;
    }
    
    esp_err_t ret = gps_ublox_init(&gps_config);
    if (ret == ESP_OK) {
        // Test hardware connectivity by checking if GPS is responding
        vTaskDelay(pdMS_TO_TICKS(2000)); // Wait 2 seconds for GPS to initialize
        
        if (gps_ublox_is_connected()) {
            system_status.gps_connected = true;
            xEventGroupSetBits(system_event_group, GPS_READY_BIT);
            ESP_LOGI(TAG, "✅ GPS initialized successfully - hardware connected");
        } else {
            system_status.gps_connected = false;
            ESP_LOGW(TAG, "⚠️ GPS initialized but hardware not detected - no GPS data received");
            ESP_LOGI(TAG, "ℹ️ GPS will continue monitoring for hardware connection");
        }
    } else {
        ESP_LOGE(TAG, "❌ GPS initialization failed: %s", esp_err_to_name(ret));
    }

    // Initialize AssistNow Offline if enabled and token is configured
    if (master_config.gps.assistnow_enabled &&
        strlen(master_config.gps.assistnow_token) > 0) {

        assistnow_config_t assistnow_config = {
            .enabled = master_config.gps.assistnow_enabled,
            .update_interval_hours = master_config.gps.assistnow_update_interval_hours,
            .validity_hours = master_config.gps.assistnow_validity_hours,
            .auto_download = master_config.gps.assistnow_auto_download,
            .period_days = 4,           // Match u-blox software default
            .resolution_hours = 1,      // Match u-blox software default
        };

        strcpy(assistnow_config.token, master_config.gps.assistnow_token);
        strcpy(assistnow_config.server_url, master_config.gps.assistnow_server_url);
        strcpy(assistnow_config.data_file_path, "/sdcard/assistnow/assistnow.ubx");
        strcpy(assistnow_config.gnss_constellations, "gps,glo,bds,gal");  // Include BeiDou

        ret = assistnow_init(&assistnow_config);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "✅ AssistNow Offline initialized");

            // Set WiFi configuration for AssistNow downloads
            wifi_assistnow_config_t wifi_config = {
                .connect_timeout_ms = master_config.system.wifi_connect_timeout_ms > 0 ?
                                    master_config.system.wifi_connect_timeout_ms : 30000,
                .auto_reconnect = master_config.system.wifi_auto_reconnect
            };
            strcpy(wifi_config.ssid, master_config.system.wifi_ssid);
            strcpy(wifi_config.password, master_config.system.wifi_password);

            ret = assistnow_set_wifi_config(&wifi_config);
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "✅ WiFi configuration set for AssistNow downloads");
            } else {
                ESP_LOGW(TAG, "⚠️ Failed to set WiFi config for AssistNow: %s", esp_err_to_name(ret));
            }

            // Configure WiFi for AssistNow downloads if credentials are provided
            if (strlen(master_config.system.wifi_ssid) > 0 &&
                strlen(master_config.system.wifi_password) > 0) {

                ret = wifi_assistnow_init(&wifi_config);
                if (ret == ESP_OK) {
                    ESP_LOGI(TAG, "✅ WiFi configured for AssistNow downloads");
                } else {
                    ESP_LOGW(TAG, "⚠️ WiFi configuration failed: %s", esp_err_to_name(ret));
                }
            } else {
                ESP_LOGI(TAG, "ℹ️ WiFi credentials not configured for AssistNow");
            }

            // Load existing AssistNow data to GPS if available
            if (assistnow_is_data_valid()) {
                assistnow_upload_to_gps();
            }

            // Start auto-update if enabled
            if (master_config.gps.assistnow_auto_download) {
                assistnow_start_auto_update();
            }
        } else {
            ESP_LOGW(TAG, "⚠️ AssistNow Offline initialization failed: %s", esp_err_to_name(ret));
        }
    } else {
        ESP_LOGI(TAG, "ℹ️ AssistNow Offline disabled or no token configured");
    }

    // Initialize Position Aiding if enabled
    if (master_config.gps.position_aiding_enabled) {
        ESP_LOGI(TAG, "🎯 Initializing Position Aiding...");

        if (master_config.gps.use_takeoff_position &&
            master_config.gps.takeoff_latitude != 0.0 &&
            master_config.gps.takeoff_longitude != 0.0) {

            // Use takeoff position for position aiding
            ret = gps_ublox_set_position_aiding(
                master_config.gps.takeoff_latitude,
                master_config.gps.takeoff_longitude,
                master_config.gps.takeoff_altitude_m,
                master_config.gps.position_accuracy_m,
                master_config.gps.altitude_accuracy_m
            );

            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "✅ Position aiding set to takeoff coordinates: %.6f, %.6f, %.1fm",
                         master_config.gps.takeoff_latitude,
                         master_config.gps.takeoff_longitude,
                         master_config.gps.takeoff_altitude_m);
            } else {
                ESP_LOGW(TAG, "⚠️ Failed to set position aiding: %s", esp_err_to_name(ret));
            }
        } else {
            ESP_LOGI(TAG, "ℹ️ Position aiding enabled but no takeoff coordinates configured");
            ESP_LOGI(TAG, "💡 Configure takeoff coordinates in gps_config.json for better GPS performance");
        }
    } else {
        ESP_LOGI(TAG, "ℹ️ Position aiding disabled");
    }

    return ret;
}

/**
 * Initialize MSP Protocol for INAV communication
 */
static esp_err_t initialize_msp(void)
{
    ESP_LOGI(TAG, "� Initializing MSP Protocol for INAV...");
    
    // Initialize MSP configuration - use loaded config if available
    msp_config_t msp_config;
    if (master_config.loaded) {
        ESP_LOGI(TAG, "📡 Using loaded MSP configuration");
        msp_config.uart_port = master_config.msp.base_config.uart_port;
        msp_config.baud_rate = master_config.msp.base_config.baud_rate;
        msp_config.tx_pin = master_config.msp.base_config.tx_pin;
        msp_config.rx_pin = master_config.msp.base_config.rx_pin;
        msp_config.timeout_ms = master_config.msp.base_config.timeout_ms;
        msp_config.auto_reconnect = master_config.msp.base_config.auto_reconnect;
    } else {
        ESP_LOGI(TAG, "📡 Using default MSP configuration");
        // Use hardcoded defaults
        msp_config.uart_port = UART_NUM_2;
        msp_config.baud_rate = 115200;
        msp_config.tx_pin = MSP_UART_TX_PIN;
        msp_config.rx_pin = MSP_UART_RX_PIN;
        msp_config.timeout_ms = 5000;
        msp_config.auto_reconnect = true;
    }
    
    esp_err_t ret = msp_protocol_init(&msp_config);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "✅ MSP Protocol initialized successfully");
        
        // Test MSP connection by requesting status
        ret = msp_request_status();
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "� MSP connection test successful");
        } else {
            ESP_LOGW(TAG, "⚠️ MSP connection test failed, but protocol initialized");
        }
    } else {
        ESP_LOGE(TAG, "❌ MSP Protocol initialization failed: %s", esp_err_to_name(ret));
    }
    
    return ret;
}

/**
 * Initialize Visual SLAM system
 */
static esp_err_t initialize_slam(void)
{
    ESP_LOGI(TAG, "🧠 Initializing Visual SLAM system...");
    
    // Log memory status before SLAM initialization
    ESP_LOGI(TAG, "Memory status before SLAM init:");
    ESP_LOGI(TAG, "  Internal RAM free: %zu bytes", heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    ESP_LOGI(TAG, "  SPIRAM free: %zu bytes", heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    ESP_LOGI(TAG, "  Total free: %zu bytes", heap_caps_get_free_size(MALLOC_CAP_8BIT));
    
    // Initialize SLAM configuration - use loaded config if available
    slam_config_t slam_config;
    if (master_config.loaded) {
        ESP_LOGI(TAG, "🧠 Using loaded SLAM configuration");
        slam_config.max_features = master_config.slam.base_config.max_features;
        slam_config.fast_threshold = master_config.slam.base_config.fast_threshold;
        slam_config.levels = master_config.slam.base_config.levels;
        slam_config.scale_factor = master_config.slam.base_config.scale_factor;
        slam_config.max_keypoints_per_level = master_config.slam.base_config.max_keypoints_per_level;
        slam_config.use_harris_detector = master_config.slam.base_config.use_harris_detector;
        slam_config.harris_k = master_config.slam.base_config.harris_k;
        slam_config.descriptor_distance_threshold = master_config.slam.base_config.descriptor_distance_threshold;
        slam_config.match_threshold = master_config.slam.base_config.match_threshold;
        slam_config.min_tracked_features = master_config.slam.base_config.min_tracked_features;
        slam_config.keyframe_distance_threshold = master_config.slam.base_config.keyframe_distance_threshold;
        slam_config.keyframe_angle_threshold = master_config.slam.base_config.keyframe_angle_threshold;
        slam_config.max_keyframes = master_config.slam.base_config.max_keyframes;
        slam_config.enable_loop_closure = master_config.slam.base_config.enable_loop_closure;
        slam_config.loop_closure_threshold = master_config.slam.base_config.loop_closure_threshold;
    } else {
        ESP_LOGI(TAG, "🧠 Using default SLAM configuration");
        // Use hardcoded defaults
        slam_config.max_features = 500;
        slam_config.fast_threshold = 20.0f;
        slam_config.levels = 8;
        slam_config.scale_factor = 1.2f;
        slam_config.max_keypoints_per_level = 100;
        slam_config.use_harris_detector = false;
        slam_config.harris_k = 0.04f;
        slam_config.descriptor_distance_threshold = 50;
        slam_config.match_threshold = 0.7f;
        slam_config.min_tracked_features = 30;
        slam_config.keyframe_distance_threshold = 1.0f;
        slam_config.keyframe_angle_threshold = 0.2f;
        slam_config.max_keyframes = 50;
        slam_config.enable_loop_closure = true;
        slam_config.loop_closure_threshold = 0.8f;
    }
    
    esp_err_t ret = slam_core_init(&slam_config);
    if (ret == ESP_OK) {
        system_status.slam_active = true;
        xEventGroupSetBits(system_event_group, SLAM_READY_BIT);
        ESP_LOGI(TAG, "✅ SLAM system initialized successfully");
        
        // Log memory status after SLAM initialization
        ESP_LOGI(TAG, "Memory status after SLAM init:");
        ESP_LOGI(TAG, "  Internal RAM free: %zu bytes", heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
        ESP_LOGI(TAG, "  SPIRAM free: %zu bytes", heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
        ESP_LOGI(TAG, "  Total free: %zu bytes", heap_caps_get_free_size(MALLOC_CAP_8BIT));
    } else {
        ESP_LOGE(TAG, "❌ SLAM initialization failed: %s", esp_err_to_name(ret));
    }
    
    return ret;
}

/**
 * Main processing task - handles camera frames and SLAM processing
 */
static void main_processing_task(void *pvParameters)
{
    ESP_LOGI(TAG, "🏃 Main processing task started");
    
    // Wait for all systems to be ready
    EventBits_t bits = xEventGroupWaitBits(system_event_group, ALL_SYSTEMS_READY, 
                                          pdFALSE, pdTRUE, portMAX_DELAY);
    
    if ((bits & ALL_SYSTEMS_READY) == ALL_SYSTEMS_READY) {
        ESP_LOGI(TAG, "🚀 All systems ready - starting main processing loop");
        gpio_set_level(GPIO_NUM_15, 1); // System ready LED
        
        // Start comprehensive logging session
        char session_name[64];
        time_t now = time(NULL);
        struct tm timeinfo;
        localtime_r(&now, &timeinfo);
        strftime(session_name, sizeof(session_name), "slam_mission_%Y%m%d_%H%M%S", &timeinfo);
        
        esp_err_t log_ret = sd_storage_start_comprehensive_logging(session_name);
        if (log_ret == ESP_OK) {
            ESP_LOGI(TAG, "📊 Comprehensive logging started: %s", session_name);
        } else {
            ESP_LOGW(TAG, "⚠️ Failed to start comprehensive logging: %s", esp_err_to_name(log_ret));
        }
    }
    
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t frequency = pdMS_TO_TICKS(33); // ~30 FPS
    uint32_t loop_counter = 0;
    
    while (1) {
        uint64_t loop_start_time = esp_timer_get_time();
        loop_counter++;
        
        // Get camera frame
        camera_frame_t frame;
        esp_err_t ret = slam_core_get_frame(&frame);
        
        if (ret == ESP_OK && frame.data != NULL) {
            system_status.frame_count++;
            
            // Toggle SLAM activity LED
            static bool slam_led_state = false;
            slam_led_state = !slam_led_state;
            gpio_set_level(GPIO_NUM_16, slam_led_state);
            
            // Process frame through ORB feature detection
            uint64_t feature_start_time = esp_timer_get_time();
            orb_features_t features;
            ret = orb_features_extract(&frame, &features);
            uint64_t feature_end_time = esp_timer_get_time();
            float feature_processing_time = (feature_end_time - feature_start_time) / 1000.0f;
            
            if (ret == ESP_OK) {
                system_status.feature_count = features.count;
                
                // Log feature extraction data
                sd_storage_log_feature_data(loop_start_time, features.count, 
                                          features.num_good_matches, feature_processing_time);
                
                // Get sensor fusion data
                sensor_fusion_data_t sensor_data;
                sensor_fusion_get_data(&sensor_data);
                
                // Update WiFi positioning if available
                if (wifi_positioning_is_available()) {
                    wifi_position_t wifi_pos;
                    esp_err_t wifi_ret = wifi_positioning_get_position(&wifi_pos);
                    if (wifi_ret == ESP_OK && wifi_pos.valid) {
                        // Update sensor fusion with WiFi position
                        sensor_fusion_update_wifi(&wifi_pos);
                        
                        // Log WiFi position data
                        ESP_LOGD(TAG, "WiFi position: %.6f, %.6f (accuracy: %.1fm)",
                                wifi_pos.latitude, wifi_pos.longitude, wifi_pos.accuracy_h);
                    }
                }
                
                // Log sensor data (GPS, IMU)
                sd_storage_log_sensor_data(loop_start_time,
                                          sensor_data.gps_data.latitude, sensor_data.gps_data.longitude, sensor_data.gps_data.altitude,
                                          sensor_data.imu_data.accel_x, sensor_data.imu_data.accel_y, sensor_data.imu_data.accel_z,
                                          sensor_data.imu_data.gyro_x, sensor_data.imu_data.gyro_y, sensor_data.imu_data.gyro_z);
                
                // Update system status
                system_status.position_x = sensor_data.position.x;
                system_status.position_y = sensor_data.position.y;
                system_status.position_z = sensor_data.position.z;
                system_status.attitude_roll = sensor_data.attitude.roll;
                system_status.attitude_pitch = sensor_data.attitude.pitch;
                system_status.attitude_yaw = sensor_data.attitude.yaw;
                
                // Process through SLAM core  
                slam_pose_t slam_pose;
                ret = slam_core_process_frame(&frame, &slam_pose);
                
                if (ret == ESP_OK) {
                    // Log SLAM pose for mission recording (if active)
                    log_current_slam_pose();
                    
                    // Get SLAM system statistics and log them
                    slam_stats_t slam_stats;
                    esp_err_t stats_ret = slam_core_get_stats(&slam_stats);
                    if (stats_ret == ESP_OK) {
                        sd_storage_log_system_status(loop_start_time, &slam_stats);
                    }
                } else {
                    // Log SLAM processing error
                    sd_storage_log_error_event(loop_start_time, "SLAM_CORE", ret, 
                                              "Failed to process camera frame");
                }
            } else {
                // Log feature extraction error
                sd_storage_log_error_event(loop_start_time, "ORB_FEATURES", ret, 
                                          "Failed to extract ORB features");
            }
            
            // Release frame buffer
            slam_core_release_frame(&frame);
        } else {
            // Log camera frame error
            if (ret != ESP_OK) {
                sd_storage_log_error_event(loop_start_time, "CAMERA", ret, 
                                          "Failed to get camera frame");
            }
        }
        
        // Log memory usage every 30 seconds (at ~30 FPS)
        if (loop_counter % 900 == 0) {
            sd_storage_log_memory_usage(loop_start_time);
        }
        
        // Check for auto-save conditions (every few iterations)
        static uint32_t auto_save_counter = 0;
        if (++auto_save_counter >= 1000) {  // Check every ~33 seconds at 30Hz
            auto_save_map_if_needed();
            auto_save_counter = 0;
            
            // Log system health status
            ESP_LOGI(TAG, "🧠 SLAM Status: frames=%lu, features=%lu, mem_free=%zu KB", 
                     system_status.frame_count, system_status.feature_count,
                     heap_caps_get_free_size(MALLOC_CAP_8BIT) / 1024);
        }
        
        // Maintain consistent timing
        vTaskDelayUntil(&last_wake_time, frequency);
    }
}

/**
 * Web interface task - handles HTTP server and WebSocket communications
 */
static void web_interface_task(void *pvParameters)
{
    ESP_LOGI(TAG, "🌐 Web interface task started (WiFi disabled)");
    
    // Web server initialization disabled - WiFi not available
    /*
    web_server_config_t config = {
        .enable_captive_portal = true,
        .max_clients = 4,
        .websocket_enabled = true
    };
    
    // Web server and WiFi completely disabled to avoid SDMMC conflicts
    esp_err_t ret = web_server_init(&config);
    if (ret == ESP_OK) {
        // Try to start WiFi in AP mode for captive portal
        ret = web_server_wifi_start();
        if (ret == ESP_OK) {
            ret = web_server_start();
            if (ret == ESP_OK) {
                // system_status.web_server_running = true;  // Disabled - WiFi not available
                // xEventGroupSetBits(system_event_group, WEB_SERVER_READY_BIT);
                ESP_LOGI(TAG, "✅ Web server started successfully");
                gpio_set_level(GPIO_NUM_17, 1); // Communication LED
            } else {
                ESP_LOGW(TAG, "⚠️  HTTP server failed to start: %s", esp_err_to_name(ret));
                ESP_LOGI(TAG, "📡 Web server will retry in background");
            }
        } else {
            ESP_LOGW(TAG, "⚠️  WiFi AP failed to start: %s", esp_err_to_name(ret));
            ESP_LOGI(TAG, "📡 System continuing without WiFi interface");
        }
    } else {
        ESP_LOGW(TAG, "⚠️  Web server init failed: %s", esp_err_to_name(ret));
        ESP_LOGI(TAG, "📡 System continuing without web interface - proceeding with SLAM");
    }
    */
    
    ESP_LOGI(TAG, "📡 Web server and WiFi disabled - SDMMC available for SD storage");
    
    // Set the bit to allow system to continue (WiFi disabled)
    ESP_LOGI(TAG, "📡 System continuing without web interface - WiFi disabled");
    // xEventGroupSetBits(system_event_group, WEB_SERVER_READY_BIT);  // Not needed anymore
    
    // Web server runs indefinitely
    while (1) {
        // Web server handles its own event loop
        vTaskDelay(pdMS_TO_TICKS(1000));
        
        // Update uptime
        system_status.uptime_seconds = esp_timer_get_time() / 1000000;
    }
}

/**
 * Telemetry broadcast task - sends real-time data to connected clients
 */
static void telemetry_broadcast_task(void *pvParameters)
{
    ESP_LOGI(TAG, "📡 Telemetry broadcast task started");
    
    // Wait for web server to be ready (disabled - WiFi not available)
    // xEventGroupWaitBits(system_event_group, WEB_SERVER_READY_BIT, 
    //                    pdFALSE, pdTRUE, portMAX_DELAY);
    
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t frequency = pdMS_TO_TICKS(100); // 10 Hz telemetry
    
    while (1) {
        // Update system status and broadcast to web clients
        system_status.uptime_seconds = esp_timer_get_time() / 1000000;
        
        // Send telemetry data via WebSocket (disabled - WiFi not available)
        // web_server_broadcast_telemetry(&system_status);
        
        vTaskDelayUntil(&last_wake_time, frequency);
    }
}

/**
 * Sensor monitoring task - monitors GPS/IMU/MSP connections and attempts reconnection
 */
static void sensor_monitoring_task(void *pvParameters)
{
    ESP_LOGI(TAG, "🔍 Sensor monitoring task started");
    
    // Initial delay to let system stabilize
    vTaskDelay(pdMS_TO_TICKS(10000)); // Wait 10 seconds after boot
    
    TickType_t last_gps_check = 0;
    TickType_t last_imu_check = 0;
    TickType_t last_msp_check = 0;
    
    while (1) {
        TickType_t current_time = xTaskGetTickCount();
        
        // Check GPS connection every 30 seconds
        if ((current_time - last_gps_check) >= pdMS_TO_TICKS(30000)) {
            last_gps_check = current_time;
            
            // Check GPS hardware connectivity by attempting to read data
            bool gps_hardware_connected = false;
            
            // Try to read from GPS UART to check if hardware is responding
            uart_config_t uart_config = {
                .baud_rate = master_config.loaded ? master_config.gps.base_config.baud_rate : 9600,
                .data_bits = UART_DATA_8_BITS,
                .parity = UART_PARITY_DISABLE,
                .stop_bits = UART_STOP_BITS_1,
                .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
                .source_clk = UART_SCLK_DEFAULT,
            };
            
            uart_port_t gps_port = master_config.loaded ? master_config.gps.base_config.uart_port : UART_NUM_1;
            
            // Configure UART for GPS
            if (uart_param_config(gps_port, &uart_config) == ESP_OK) {
                if (uart_set_pin(gps_port, 
                    master_config.loaded ? master_config.gps.base_config.tx_pin : GPS_UART_TX_PIN,
                    master_config.loaded ? master_config.gps.base_config.rx_pin : GPS_UART_RX_PIN,
                    UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) == ESP_OK) {
                    
                    if (uart_driver_install(gps_port, 256, 0, 0, NULL, 0) == ESP_OK) {
                        // Try to read some data to check if GPS is actually connected
                        uint8_t gps_buffer[64];
                        int len = uart_read_bytes(gps_port, gps_buffer, sizeof(gps_buffer), pdMS_TO_TICKS(100));
                        
                        if (len > 0) {
                            // Check if we received valid NMEA data (starts with $)
                            for (int i = 0; i < len; i++) {
                                if (gps_buffer[i] == '$') {
                                    gps_hardware_connected = true;
                                    break;
                                }
                            }
                        }
                        
                        uart_driver_delete(gps_port);
                    }
                }
            }
            
            if (!gps_hardware_connected) {
                ESP_LOGW(TAG, "🛰️ GPS hardware not detected - no valid data received");
                system_status.gps_connected = false;
                xEventGroupClearBits(system_event_group, GPS_READY_BIT);
                
                // Mark GPS as unavailable in sensor fusion
                sensor_status_t sensor_status;
                if (sensor_fusion_get_sensor_status(&sensor_status) == ESP_OK) {
                    sensor_status.gps_available = false;
                    // Note: We can't directly set this back, but we can reinitialize sensor fusion
                }
            } else {
                ESP_LOGD(TAG, "🛰️ GPS hardware detected - receiving valid data");
                system_status.gps_connected = true;
                xEventGroupSetBits(system_event_group, GPS_READY_BIT);
            }
        }
        
        // Check IMU connection every 15 seconds
        if ((current_time - last_imu_check) >= pdMS_TO_TICKS(15000)) {
            last_imu_check = current_time;
            
            // Check IMU hardware connectivity by attempting to read sensor data
            bool imu_hardware_connected = false;
            
            // Try to read IMU data to check if hardware is responding
            imu_data_t imu_data;
            if (imu_get_data(&imu_data) == ESP_OK) {
                // Check if the data looks reasonable (not all zeros or invalid values)
                if (imu_data.accel_x != 0.0f || imu_data.accel_y != 0.0f || imu_data.accel_z != 0.0f ||
                    imu_data.gyro_x != 0.0f || imu_data.gyro_y != 0.0f || imu_data.gyro_z != 0.0f) {
                    imu_hardware_connected = true;
                }
            }
            
            if (!imu_hardware_connected) {
                ESP_LOGW(TAG, "🔄 IMU hardware not detected - no valid sensor data");
                system_status.imu_connected = false;
                
                // Mark IMU as unavailable in sensor fusion
                sensor_status_t sensor_status;
                if (sensor_fusion_get_sensor_status(&sensor_status) == ESP_OK) {
                    sensor_status.imu_available = false;
                    // Note: We can't directly set this back, but we can reinitialize sensor fusion
                }
            } else {
                ESP_LOGD(TAG, "🔄 IMU hardware detected - receiving valid sensor data");
                system_status.imu_connected = true;
            }
        }
        
        // Check MSP connection every 20 seconds
        if ((current_time - last_msp_check) >= pdMS_TO_TICKS(20000)) {
            last_msp_check = current_time;
            
            // Check MSP connection using the built-in connection check
            if (!msp_is_connected()) {
                ESP_LOGW(TAG, "📡 MSP flight controller not detected - no recent messages");
                system_status.msp_connected = false;
            } else {
                ESP_LOGD(TAG, "📡 MSP flight controller connected - receiving messages");
                system_status.msp_connected = true;
            }
        }
        
        // Update system status with current sensor states
        // Note: GPS and IMU status are updated in their respective check sections above
        // MSP status is updated in its check section above
        
        vTaskDelay(pdMS_TO_TICKS(5000)); // Check every 5 seconds
    }
}

/**
 * WIFI toggle button interrupt service routine
 */
/*
static void IRAM_ATTR wifi_toggle_button_isr(void* arg)
{
    // Create a task to handle WIFI toggle to avoid ISR complexity
    xTaskCreate(wifi_toggle_task, "wifi_toggle", 2048, NULL, 5, NULL);
}
*/

/**
 * WIFI toggle task - handles WIFI on/off logic
 */
/*
void wifi_toggle_task(void* arg)
{
    // Debounce delay
    vTaskDelay(pdMS_TO_TICKS(50));

    // Check if button is still pressed (debounce)
    if (gpio_get_level(WIFI_TOGGLE_BUTTON_PIN) == 0) {
        // Toggle WIFI state
        if (wifi_enabled) {
            wifi_disable();
        } else {
            wifi_enable();
        }
    }

    vTaskDelete(NULL);
}
*/

/**
 * Enable WIFI remote functionality
 */
/*
esp_err_t wifi_enable(void)
{
    if (wifi_enabled) {
        ESP_LOGW(TAG, "⚠️ WIFI is already enabled");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "📡 Enabling WIFI remote...");

    // Take WIFI mutex
    if (xSemaphoreTake(wifi_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "❌ Failed to acquire WIFI mutex");
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t ret = ESP_OK;

    // Initialize WIFI if not already done
    if (!wifi_connected) {
        // Configure web server
        web_server_config_t config = {
            .port = 80,
            .max_clients = 4,
            .enable_cors = true,
            .enable_auth = false,
            .ssid = "DroneCam-SLAM",
            .password = "dronecam123",
            .channel = 6,
            .max_connection = 4
        };

        ret = web_server_init(&config);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "❌ Web server initialization failed: %s", esp_err_to_name(ret));
            xSemaphoreGive(wifi_mutex);
            return ret;
        }

        // Start WIFI in AP mode
        ret = web_server_wifi_start();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "❌ WIFI AP start failed: %s", esp_err_to_name(ret));
            xSemaphoreGive(wifi_mutex);
            return ret;
        }

        // Start web server
        ret = web_server_start();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "❌ Web server start failed: %s", esp_err_to_name(ret));
            xSemaphoreGive(wifi_mutex);
            return ret;
        }

        wifi_connected = true;
        system_status.web_server_running = true;
        xEventGroupSetBits(system_event_group, WEB_SERVER_READY_BIT);
    }

    wifi_enabled = true;
    xSemaphoreGive(wifi_mutex);

    // Download AssistNow data if needed and configured
    if (master_config.gps.assistnow_enabled &&
        strlen(master_config.gps.assistnow_token) > 0 &&
        assistnow_is_download_needed()) {

        ESP_LOGI(TAG, "📡 WiFi available - checking for AssistNow data update...");
        esp_err_t assistnow_ret = assistnow_download_data();
        if (assistnow_ret == ESP_OK) {
            ESP_LOGI(TAG, "✅ AssistNow data downloaded successfully");

            // Load the new data to GPS
            assistnow_upload_to_gps();
        } else {
            ESP_LOGW(TAG, "⚠️ AssistNow data download failed: %s", esp_err_to_name(assistnow_ret));
        }
    }

    ESP_LOGI(TAG, "🌐 Web interface: Connect to WiFi or use captive portal");
    ESP_LOGI(TAG, "📱 Access dashboard at: http://dronecam.local or device IP");

    return ESP_OK;
}
*/

/**
 * Disable WIFI remote functionality
 */
/*
esp_err_t wifi_disable(void)
{
    if (!wifi_enabled) {
        ESP_LOGW(TAG, "⚠️ WIFI is already disabled");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "📡 Disabling WIFI remote...");

    // Take WIFI mutex
    if (xSemaphoreTake(wifi_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "❌ Failed to acquire WIFI mutex");
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t ret = ESP_OK;

    // Stop web server and WIFI
    if (wifi_connected) {
        ret = web_server_stop();
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "⚠️ Web server stop warning: %s", esp_err_to_name(ret));
        }

        ret = web_server_wifi_stop();
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "⚠️ WIFI stop warning: %s", esp_err_to_name(ret));
        }

        wifi_connected = false;
        system_status.web_server_running = false;
        xEventGroupClearBits(system_event_group, WEB_SERVER_READY_BIT);
    }

    wifi_enabled = false;
    xSemaphoreGive(wifi_mutex);

    ESP_LOGI(TAG, "✅ WIFI remote disabled");

    return ESP_OK;
}
*/

/**
 * Initialize WIFI system
 */
/*
static esp_err_t initialize_wifi(void)
{
    ESP_LOGI(TAG, "📡 Initializing WIFI system...");

    // Create WIFI mutex
    wifi_mutex = xSemaphoreCreateMutex();
    if (wifi_mutex == NULL) {
        ESP_LOGE(TAG, "❌ Failed to create WIFI mutex");
        return ESP_ERR_NO_MEM;
    }

    // Initialize WIFI state
    wifi_enabled = false;
    wifi_connected = false;

    ESP_LOGI(TAG, "✅ WIFI system initialized (disabled by default)");
    ESP_LOGI(TAG, "🔘 Press GPIO%d button to toggle WIFI on/off", WIFI_TOGGLE_BUTTON_PIN);

    return ESP_OK;
}
*/

/**
 * Application main entry point
 */
void app_main(void)
{
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=================================================");
    ESP_LOGI(TAG, "🚁 Visual SLAM Navigation Module for INAV");
    ESP_LOGI(TAG, "📡 ESP32-P4-WIFI6 with OV5647 MIPI-CSI Camera");
    ESP_LOGI(TAG, "🧠 Real-time ORB-SLAM with Web Interface");
    ESP_LOGI(TAG, "=================================================");
    ESP_LOGI(TAG, "");

    // Validate pin assignments at compile time
    VALIDATE_ALL_PIN_ASSIGNMENTS;

    ESP_LOGI(TAG, "✅ Pin configuration validated - no conflicts detected");

    // Initialize system components
    initialize_nvs();
    initialize_gpio();

    // Initialize WIFI system (disabled by default)
    // initialize_wifi();

    // Create system synchronization objects
    system_mutex = xSemaphoreCreateMutex();
    system_event_group = xEventGroupCreate();
    
    if (system_mutex == NULL || system_event_group == NULL) {
        ESP_LOGE(TAG, "❌ Failed to create synchronization objects");
        return;
    }
    
    // Initialize TCP/IP stack (disabled - WiFi not available)
    // ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    // Initialize hardware subsystems
    esp_err_t ret;
    
    // Initialize SD card storage FIRST (before components that need config)
    ESP_LOGI(TAG, "💾 Initializing SD card storage... (early init for config loading)");
    ret = map_manager_init();
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "✅ SD card storage initialized successfully");

        // Initialize configuration loader
        ESP_LOGI(TAG, "⚙️ Initializing configuration loader... (early init for component config)");
        ret = config_loader_init();
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "✅ Configuration loader initialized successfully");

            // Load master configuration BEFORE initializing components
            ESP_LOGI(TAG, "📖 Loading master configuration... (before component init)");
            ret = config_loader_load_all(&master_config);
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "✅ Master configuration loaded successfully");
                master_config.loaded = true;
            } else {
                ESP_LOGI(TAG, "ℹ️ No existing configuration found, creating defaults...");

                // Create default configuration files if they don't exist
                ESP_LOGI(TAG, "📝 Creating default configuration files...");
                ret = config_loader_create_defaults();
                if (ret == ESP_OK) {
                    ESP_LOGI(TAG, "✅ Default configuration files created successfully");

                    // Now try to load the newly created defaults
                    ESP_LOGI(TAG, "📖 Loading newly created configuration...");
                    ret = config_loader_load_all(&master_config);
                    if (ret == ESP_OK) {
                        ESP_LOGI(TAG, "✅ Default configuration loaded successfully");
                        master_config.loaded = true;
                    } else {
                        ESP_LOGE(TAG, "❌ Failed to load default configuration: %s", esp_err_to_name(ret));
                        ESP_LOGI(TAG, "⚠️ Continuing with built-in defaults");
                    }
                } else {
                    ESP_LOGE(TAG, "❌ Failed to create default configuration files: %s", esp_err_to_name(ret));
                    ESP_LOGI(TAG, "⚠️ Continuing with built-in defaults");
                }
            }
        } else {
            ESP_LOGE(TAG, "❌ Configuration loader initialization failed: %s", esp_err_to_name(ret));
            ESP_LOGI(TAG, "⚠️ Continuing with built-in defaults");
        }
    } else {
        ESP_LOGE(TAG, "❌ SD card storage initialization failed: %s", esp_err_to_name(ret));
        ESP_LOGI(TAG, "⚠️ Continuing with built-in defaults (no SD card available)");
    }
    
    // Now initialize components with loaded configuration
    ret = initialize_camera();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Camera initialization failed, continuing anyway...");
    }
    
    ret = initialize_gps();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GPS initialization failed, continuing anyway...");
    }
    
    ret = initialize_msp();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "MSP initialization failed, continuing anyway...");
    }
    
    // Initialize IMU
    ret = sensor_fusion_register_imu(&master_config.imu.base_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "IMU registration failed, continuing anyway...");
    }
    
    // List available maps (if SD card was initialized earlier)
    if (config_loader_is_sd_available()) {
        list_available_maps();

        // Try to load a default map if available
        ESP_LOGI(TAG, "🗺️ Checking for existing maps to load...");
        ret = load_slam_map(NULL);  // Load latest map
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "✅ Existing map loaded successfully");
        } else {
            ESP_LOGI(TAG, "ℹ️ No existing map found, starting fresh");
        }
    }

            // First try to load existing configuration
            ESP_LOGI(TAG, "� Loading master configuration...");
            ret = config_loader_load_all(&master_config);
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "✅ Master configuration loaded successfully");
                master_config.loaded = true;
            } else {
                ESP_LOGI(TAG, "ℹ️ No existing configuration found, creating defaults...");
                
                // Create default configuration files if they don't exist
                ESP_LOGI(TAG, "� Creating default configuration files...");
                ret = config_loader_create_defaults();
                if (ret == ESP_OK) {
                    ESP_LOGI(TAG, "✅ Default configuration files created successfully");
                    
                    // Now try to load the newly created defaults
                    ESP_LOGI(TAG, "📖 Loading newly created configuration...");
                    ret = config_loader_load_all(&master_config);
                    if (ret == ESP_OK) {
                        ESP_LOGI(TAG, "✅ Default configuration loaded successfully");
                        master_config.loaded = true;
                    } else {
                        ESP_LOGE(TAG, "❌ Failed to load default configuration: %s", esp_err_to_name(ret));
                        ESP_LOGI(TAG, "⚠️ Continuing with built-in defaults");
                    }
                } else {
                    ESP_LOGE(TAG, "❌ Failed to create default configuration files: %s", esp_err_to_name(ret));
                    ESP_LOGI(TAG, "⚠️ Continuing with built-in defaults");
                }
            }
    // Initialize SLAM system
    ret = initialize_slam();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SLAM initialization failed, continuing anyway...");
    }

    // Initialize WiFi positioning system
    ESP_LOGI(TAG, "📍 Initializing WiFi positioning system...");
    wifi_positioning_config_t wifi_pos_config = {
        .scan_interval_ms = 30000,        // Scan every 30 seconds
        .min_ap_count = 3,                // Need at least 3 APs
        .max_ap_count = 10,               // Use up to 10 APs
        .min_rssi_threshold = -85,        // Minimum RSSI -85dBm
        .wigle_db_size = 10000,           // Support up to 10k entries
        .wigle_db_path = "/sdcard/wigle/wigle.db",
        .enable_auto_scan = true,         // Enable automatic scanning
        .position_timeout_ms = 5000       // 5 second timeout
    };

    ret = wifi_positioning_init(&wifi_pos_config);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "✅ WiFi positioning initialized successfully");

        // Start WiFi positioning
        ret = wifi_positioning_start();
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "✅ WiFi positioning started successfully");
        } else {
            ESP_LOGW(TAG, "⚠️ Failed to start WiFi positioning: %s", esp_err_to_name(ret));
        }
    } else {
        ESP_LOGW(TAG, "⚠️ WiFi positioning initialization failed: %s", esp_err_to_name(ret));
        ESP_LOGW(TAG, "ℹ️ Continuing without WiFi positioning - GPS/SLAM will still work");
    }

    // Save current configuration to SD card (if SD card is available)
    if (config_loader_is_sd_available()) {
        ESP_LOGI(TAG, "💾 Saving initial configuration to SD card...");
        save_configuration_to_sd();
    }

    // Create main processing tasks
    xTaskCreatePinnedToCore(
        main_processing_task,
        "main_processing",
        8192,
        NULL,
        5,
        &main_processing_task_handle,
        1  // Pin to core 1
    );
    
    xTaskCreatePinnedToCore(
        web_interface_task,
        "web_interface",
        8192,
        NULL,
        4,
        &web_interface_task_handle,
        0  // Pin to core 0
    );
    
    xTaskCreatePinnedToCore(
        telemetry_broadcast_task,
        "telemetry_broadcast",
        4096,
        NULL,
        3,
        &telemetry_broadcast_task_handle,
        0  // Pin to core 0
    );
    
    // Create sensor monitoring task for GPS/IMU/MSP reconnection
    xTaskCreatePinnedToCore(
        sensor_monitoring_task,
        "sensor_monitor",
        4096,
        NULL,
        2,
        &sensor_monitoring_task_handle,
        0  // Pin to core 0
    );
    
    ESP_LOGI(TAG, "🚀 Visual SLAM Navigation Module started successfully!");
    ESP_LOGI(TAG, "🌐 Web interface: Connect to WiFi or use captive portal");
    ESP_LOGI(TAG, "📱 Access dashboard at: http://dronecam.local or device IP");
    
    // Main loop - monitor system health
    while (1) {
        // Check task health
        if (main_processing_task_handle != NULL && 
            eTaskGetState(main_processing_task_handle) == eDeleted) {
            ESP_LOGE(TAG, "❌ Main processing task died - stopping logging and restarting system");
            sd_storage_stop_comprehensive_logging();
            esp_restart();
        }
        
        if (web_interface_task_handle != NULL && 
            eTaskGetState(web_interface_task_handle) == eDeleted) {
            ESP_LOGE(TAG, "❌ Web interface task died - stopping logging and restarting system");
            sd_storage_stop_comprehensive_logging();
            esp_restart();
        }
        
        vTaskDelay(pdMS_TO_TICKS(5000)); // Check every 5 seconds
    }
    
    // Cleanup (should never reach here, but good practice)
    sd_storage_stop_comprehensive_logging();
}
