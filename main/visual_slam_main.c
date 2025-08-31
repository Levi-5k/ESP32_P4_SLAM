/**
 * Visual SLAM Navigation Module for INAV
 * ESP32-P4-WIFI6 with OV5647 MIPI-CSI Camera
 * Complete navigation system with real-time web interface
 */

#include <stdio.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <freertos/event_groups.h>
#include <esp_log.h>
#include <esp_system.h>
// #include <esp_wifi.h>  // Disabled - WiFi not available
#include <driver/spi_master.h>
#include <esp_event.h>
#include <nvs_flash.h>
#include <driver/gpio.h>
#include <esp_timer.h>

// Visual SLAM Components
#include "visual_slam_common_types.h"
#include "slam_core.h"
#include "orb_features.h"
#include "sensor_fusion.h"
// #include "web_server.h"  // Disabled - WiFi not available
#include "sd_storage.h"
// #include "gps_ublox.h"    // Will be implemented later
// #include "imu_bmi088.h"   // Will be implemented later
// #include "msp_protocol.h" // Will be implemented later

static const char *TAG = "VisualSLAM_INAV";

// System synchronization
static SemaphoreHandle_t system_mutex;
static EventGroupHandle_t system_event_group;

// System state flags
#define CAMERA_READY_BIT        BIT0
#define GPS_READY_BIT          BIT1
#define IMU_READY_BIT          BIT2
#define SLAM_READY_BIT         BIT3
// #define WEB_SERVER_READY_BIT   BIT4  // Disabled - WiFi not available
#define ALL_SYSTEMS_READY      (CAMERA_READY_BIT | GPS_READY_BIT | IMU_READY_BIT | SLAM_READY_BIT)

// Task handles
static TaskHandle_t main_processing_task_handle = NULL;
static TaskHandle_t web_interface_task_handle = NULL;
static TaskHandle_t telemetry_broadcast_task_handle = NULL;

// System status instance
static system_status_t system_status = {0};

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
        .pin_bit_mask = (1ULL << GPIO_NUM_15) | (1ULL << GPIO_NUM_16) | (1ULL << GPIO_NUM_17),
        .pull_down_en = 0,
        .pull_up_en = 0,
    };
    gpio_config(&gpio_conf);
    
    // Set initial LED states
    gpio_set_level(GPIO_NUM_15, 0); // System status LED
    gpio_set_level(GPIO_NUM_16, 0); // SLAM activity LED
    gpio_set_level(GPIO_NUM_17, 0); // Communication LED
    
    ESP_LOGI(TAG, "✅ GPIO initialized");
}

/**
 * Initialize camera system (OV5647 MIPI-CSI)
 */
static esp_err_t initialize_camera(void)
{
    ESP_LOGI(TAG, "🎥 Initializing OV5647 MIPI-CSI camera...");
    
    // Initialize camera configuration
    camera_config_t camera_config = {
        .resolution = CAMERA_RES_640x480,
        .format = CAMERA_FORMAT_RGB565,
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
    };
    
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
    
    gps_config_t gps_config = {
        .uart_port = UART_NUM_1,
        .baud_rate = 115200,
        .tx_pin = GPIO_NUM_15,  // Changed from 43 (SDIO CLK conflict)
        .rx_pin = GPIO_NUM_16   // Changed from 44 (SDIO CMD conflict)
    };
    
    esp_err_t ret = sensor_fusion_init_gps(&gps_config);
    if (ret == ESP_OK) {
        system_status.gps_connected = true;
        xEventGroupSetBits(system_event_group, GPS_READY_BIT);
        ESP_LOGI(TAG, "✅ GPS initialized successfully");
    } else {
        ESP_LOGE(TAG, "❌ GPS initialization failed: %s", esp_err_to_name(ret));
    }
    
    return ret;
}

/**
 * Initialize IMU (BMI088)
 */
static esp_err_t initialize_imu(void)
{
    ESP_LOGI(TAG, "📐 Initializing BMI088 IMU...");
    
    imu_config_t imu_config = {
        .spi_host = SPI3_HOST,
        .miso_pin = GPIO_NUM_8,
        .mosi_pin = GPIO_NUM_9,
        .sclk_pin = GPIO_NUM_10,
        .acc_cs_pin = GPIO_NUM_11,
        .gyro_cs_pin = GPIO_NUM_12,
        .accel_range = BMI088_ACCEL_RANGE_6G,
        .gyro_range = BMI088_GYRO_RANGE_500DPS,
        .sample_rate = 400
    };
    
    esp_err_t ret = sensor_fusion_init_imu(&imu_config);
    if (ret == ESP_OK) {
        system_status.imu_calibrated = true;
        xEventGroupSetBits(system_event_group, IMU_READY_BIT);
        ESP_LOGI(TAG, "✅ IMU initialized successfully");
    } else {
        ESP_LOGE(TAG, "❌ IMU initialization failed: %s", esp_err_to_name(ret));
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
    
    slam_config_t slam_config = {
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
        .loop_closure_threshold = 0.8f
    };
    
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
    }
    
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t frequency = pdMS_TO_TICKS(33); // ~30 FPS
    
    while (1) {
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
            orb_features_t features;
            ret = orb_features_extract(&frame, &features);
            
            if (ret == ESP_OK) {
                system_status.feature_count = features.count;
                
                // Get sensor fusion data
                sensor_fusion_data_t sensor_data;
                sensor_fusion_get_data(&sensor_data);
                
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
                    // Create a slam_result from slam_pose for web server (disabled - WiFi not available)
                    /*
                    slam_result_t slam_result = {
                        .pose = slam_pose,
                        .num_features = features.num_features,
                        .tracking_quality = 0.8f,  // Default value
                        .is_lost = false
                    };
                    
                    // Update web server with latest data (disabled - WiFi not available)
                    // web_server_update_slam_data(&slam_result);
                    // web_server_update_system_status(&system_status);
                    */
                    
                    // Log SLAM pose for mission recording (if active)
                    log_current_slam_pose();
                }
            }
            
            // Release frame buffer
            slam_core_release_frame(&frame);
        }
        
        // Check for auto-save conditions (every few iterations)
        static uint32_t auto_save_counter = 0;
        if (++auto_save_counter >= 1000) {  // Check every ~20 seconds at 50Hz
            auto_save_map_if_needed();
            auto_save_counter = 0;
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
    
    // Initialize system components
    initialize_nvs();
    initialize_gpio();
    
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
    
    ret = initialize_camera();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Camera initialization failed, continuing anyway...");
    }
    
    ret = initialize_gps();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GPS initialization failed, continuing anyway...");
    }
    
    ret = initialize_imu();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "IMU initialization failed, continuing anyway...");
    }
    
    // Initialize SD card storage for map data
    ESP_LOGI(TAG, "💾 Initializing SD card storage...");
    ret = map_manager_init();
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "✅ SD card storage initialized successfully");
        
        // List available maps
        list_available_maps();
        
        // Try to load a default map if available
        ESP_LOGI(TAG, "🗺️ Checking for existing maps to load...");
        ret = load_slam_map(NULL);  // Load latest map
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "✅ Existing map loaded successfully");
        } else {
            ESP_LOGI(TAG, "ℹ️ No existing map found, starting fresh");
        }
    } else {
        ESP_LOGE(TAG, "❌ SD card initialization failed: %s", esp_err_to_name(ret));
        ESP_LOGI(TAG, "⚠️ Continuing without map storage capability");
    }
    
    ret = initialize_slam();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SLAM initialization failed, continuing anyway...");
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
    
    ESP_LOGI(TAG, "🚀 Visual SLAM Navigation Module started successfully!");
    ESP_LOGI(TAG, "🌐 Web interface: Connect to WiFi or use captive portal");
    ESP_LOGI(TAG, "📱 Access dashboard at: http://dronecam.local or device IP");
    
    // Main loop - monitor system health
    while (1) {
        // Check task health
        if (main_processing_task_handle != NULL && 
            eTaskGetState(main_processing_task_handle) == eDeleted) {
            ESP_LOGE(TAG, "❌ Main processing task died - restarting system");
            esp_restart();
        }
        
        if (web_interface_task_handle != NULL && 
            eTaskGetState(web_interface_task_handle) == eDeleted) {
            ESP_LOGE(TAG, "❌ Web interface task died - restarting system");
            esp_restart();
        }
        
        vTaskDelay(pdMS_TO_TICKS(5000)); // Check every 5 seconds
    }
}
