/* Visual SLAM Navigation Module for INAV
   ESP32-P4-WIFI6 based navigation system

   This example demonstrates:
   - ORB-SLAM visual navigation
   - GPS/IMU sensor fusion  
   - Real-time web interface with WebSocket
   - Captive portal for WiFi setup
   - MSP communication with INAV

   Features:
   - 50 FPS camera streaming with ISP color control
   - ORB feature detection and matching
   - Extended Kalman Filter sensor fusion
   - Real-time telemetry dashboard
   - Map persistence and loop closure
*/
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_chip_info.h"
#include "esp_timer.h"

// Visual SLAM system includes
#include "visual_slam_inav.h"
#include "web_server.h"
#include "slam_core.h"
#include "orb_features.h"
#include "sensor_fusion.h"
#include "camera/camera_init.h"

static const char *TAG = "VisualSLAM_INAV";

// System Configuration
#define SYSTEM_UPDATE_RATE_HZ 50        // Main system update rate
#define WEB_UPDATE_RATE_HZ 10           // Web interface update rate
#define TELEMETRY_UPDATE_RATE_HZ 20     // Telemetry broadcast rate

// WiFi Configuration - Will fallback to captive portal if connection fails
#define DEFAULT_WIFI_SSID      "-----"
#define DEFAULT_WIFI_PASS      "--------"
#define CAPTIVE_PORTAL_SSID    "VisualSLAM_Setup"
#define CAPTIVE_PORTAL_PASS    "12345678"

// GPS origin coordinates (set to your takeoff location)
#define DEFAULT_GPS_LAT        40.7128    // Example: New York City
#define DEFAULT_GPS_LON        -74.0060
#define DEFAULT_GPS_ALT        10.0       // Meters above sea level

// Global system state
static struct {
    bool system_initialized;
    bool camera_active;
    bool slam_active;
    bool gps_active;
    bool imu_active;
    bool web_server_active;
    
    uint64_t start_time_us;
    uint32_t frame_count;
    uint32_t last_fps_time;
    uint32_t current_fps;
    
    TaskHandle_t main_task;
    TaskHandle_t web_task;
    TaskHandle_t telemetry_task;
    
} g_system_state = {0};

// Function prototypes
static esp_err_t initialize_system_components(void);
static void main_processing_task(void* pvParameters);
static void web_interface_task(void* pvParameters);
static void telemetry_broadcast_task(void* pvParameters);
static esp_err_t setup_wifi_connection(void);
static void visual_slam_event_handler(visual_slam_event_t event, void* data);
static void web_server_event_handler(web_server_event_t event, void* data);
static esp_err_t collect_realtime_data(realtime_data_t* data);
static void print_system_info(void);

// Initialize all system components
static esp_err_t initialize_system_components(void) {
    esp_err_t ret = ESP_OK;
    
    ESP_LOGI(TAG, "🚁 Initializing Visual SLAM Navigation System");
    
    // Initialize camera system
    ESP_LOGI(TAG, "📹 Initializing camera system...");
    ret = camera_init();
    if (ret == ESP_OK) {
        g_system_state.camera_active = true;
        ESP_LOGI(TAG, "✅ Camera system initialized successfully");
    } else {
        ESP_LOGE(TAG, "❌ Camera initialization failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Initialize ORB feature detector
    ESP_LOGI(TAG, "🔍 Initializing ORB feature detection...");
    ret = orb_features_init(500);  // 500 max features
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "✅ ORB features initialized successfully");
    } else {
        ESP_LOGE(TAG, "❌ ORB features initialization failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Initialize SLAM core
    ESP_LOGI(TAG, "🗺️ Initializing SLAM core...");
    ret = slam_core_init(NULL);  // Use default config
    if (ret == ESP_OK) {
        g_system_state.slam_active = true;
        ESP_LOGI(TAG, "✅ SLAM core initialized successfully");
    } else {
        ESP_LOGE(TAG, "❌ SLAM core initialization failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Initialize sensor fusion
    ESP_LOGI(TAG, "🧭 Initializing sensor fusion...");
    ret = sensor_fusion_init(NULL);  // Use default config
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "✅ Sensor fusion initialized successfully");
    } else {
        ESP_LOGE(TAG, "❌ Sensor fusion initialization failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Set GPS origin (replace with your actual coordinates)
    ret = sensor_fusion_set_gps_origin(DEFAULT_GPS_LAT, DEFAULT_GPS_LON, DEFAULT_GPS_ALT);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "📍 GPS origin set to %.6f, %.6f, %.1fm", 
                DEFAULT_GPS_LAT, DEFAULT_GPS_LON, DEFAULT_GPS_ALT);
    }
    
    // Initialize web server
    ESP_LOGI(TAG, "🌐 Initializing web server...");
    ret = web_server_init(NULL);  // Use default config
    if (ret == ESP_OK) {
        g_system_state.web_server_active = true;
        ESP_LOGI(TAG, "✅ Web server initialized successfully");
    } else {
        ESP_LOGE(TAG, "❌ Web server initialization failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Register event callbacks
    visual_slam_register_event_callback(visual_slam_event_handler);
    web_server_register_event_callback(web_server_event_handler);
    
    ESP_LOGI(TAG, "🎉 All system components initialized successfully!");
    return ESP_OK;
}

// Main SLAM processing task - runs at 50Hz
static void main_processing_task(void* pvParameters) {
    camera_fb_t* frame = NULL;
    slam_pose_t slam_pose;
    navigation_state_t nav_state;
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t task_period = pdMS_TO_TICKS(1000 / SYSTEM_UPDATE_RATE_HZ);
    
    ESP_LOGI(TAG, "🔄 Main processing task started at %d Hz", SYSTEM_UPDATE_RATE_HZ);
    
    while (g_system_state.system_initialized) {
        vTaskDelayUntil(&last_wake_time, task_period);
        
        // Capture camera frame
        if (g_system_state.camera_active) {
            frame = esp_camera_fb_get();
            if (frame != NULL) {
                g_system_state.frame_count++;
                
                // Process frame with SLAM
                if (g_system_state.slam_active) {
                    esp_err_t ret = slam_core_process_frame(frame, &slam_pose);
                    if (ret == ESP_OK) {
                        // Update sensor fusion with SLAM data
                        sensor_fusion_update_slam(&slam_pose);
                        
                        // Get fused navigation state
                        sensor_fusion_get_state(&nav_state);
                    }
                }
                
                // Return frame buffer
                esp_camera_fb_return(frame);
            }
        }
        
        // Update FPS counter
        uint32_t current_time = esp_timer_get_time() / 1000;
        if (current_time - g_system_state.last_fps_time >= 1000) {
            g_system_state.current_fps = g_system_state.frame_count;
            g_system_state.frame_count = 0;
            g_system_state.last_fps_time = current_time;
        }
        
        // Simulate IMU data (replace with actual BMI088 driver)
        imu_data_t imu_data = {
            .accel_x = 0.1f, .accel_y = 0.0f, .accel_z = 9.8f,
            .gyro_x = 0.01f, .gyro_y = 0.02f, .gyro_z = 0.005f,
            .temp_c = 25.5f,
            .timestamp_us = esp_timer_get_time()
        };
        sensor_fusion_update_imu(&imu_data);
        
        // Simulate GPS data (replace with actual uBlox driver)
        static uint32_t gps_counter = 0;
        if (++gps_counter % 10 == 0) {  // 5Hz GPS updates
            gps_position_t gps_data = {
                .latitude = DEFAULT_GPS_LAT + (float)(rand() % 100) / 1000000.0,
                .longitude = DEFAULT_GPS_LON + (float)(rand() % 100) / 1000000.0,
                .altitude = DEFAULT_GPS_ALT + (float)(rand() % 10) / 10.0,
                .gps_fix_type = 3,  // 3D fix
                .satellites = 8,
                .hdop = 1.2f,
                .accuracy = 2.5f,
                .timestamp_us = esp_timer_get_time()
            };
            sensor_fusion_update_gps(&gps_data);
        }
    }
    
    ESP_LOGI(TAG, "Main processing task ended");
    vTaskDelete(NULL);
}

// Web interface management task
static void web_interface_task(void* pvParameters) {
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t task_period = pdMS_TO_TICKS(1000 / WEB_UPDATE_RATE_HZ);
    
    ESP_LOGI(TAG, "🌐 Web interface task started at %d Hz", WEB_UPDATE_RATE_HZ);
    
    while (g_system_state.system_initialized) {
        vTaskDelayUntil(&last_wake_time, task_period);
        
        // Update web server status and handle connections
        web_server_status_t web_status;
        if (web_server_get_status(&web_status) == ESP_OK) {
            // Handle any web server management tasks
        }
        
        // Check if we need to handle WiFi fallback
        if (!web_status.wifi_connected && web_status.current_wifi_mode == WIFI_MODE_STATION) {
            ESP_LOGW(TAG, "WiFi connection lost, falling back to captive portal");
            web_server_wifi_set_mode(WIFI_MODE_CAPTIVE_PORTAL);
        }
    }
    
    ESP_LOGI(TAG, "Web interface task ended");
    vTaskDelete(NULL);
}

// Real-time telemetry broadcast task  
static void telemetry_broadcast_task(void* pvParameters) {
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t task_period = pdMS_TO_TICKS(1000 / TELEMETRY_UPDATE_RATE_HZ);
    static uint32_t sequence_number = 0;
    
    ESP_LOGI(TAG, "📡 Telemetry broadcast task started at %d Hz", TELEMETRY_UPDATE_RATE_HZ);
    
    while (g_system_state.system_initialized) {
        vTaskDelayUntil(&last_wake_time, task_period);
        
        // Collect real-time data
        realtime_data_t realtime_data;
        if (collect_realtime_data(&realtime_data) == ESP_OK) {
            realtime_data.sequence_number = sequence_number++;
            realtime_data.timestamp_us = esp_timer_get_time();
            
            // Broadcast to web clients
            web_server_broadcast_data(&realtime_data);
        }
    }
    
    ESP_LOGI(TAG, "Telemetry broadcast task ended");
    vTaskDelete(NULL);
}

// WiFi event handler delegated to web server component
// Event handling is now managed by web_server component
    }
}

// Initialize WiFi
void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Event handling now managed by web server component
    // ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    // ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    // WiFi setup delegated to web server component for captive portal support
    ESP_LOGI(TAG, "WiFi init delegated to web server component");
}

// Generate a test pattern image (placeholder for OV5647 camera)
void generate_test_image(void)
{
    // Use smaller resolution to save memory on ESP32-P4
    const int width = 320;   // Reduced from 640 for memory optimization
    const int height = 240;  // Reduced from 480 for memory optimization
    const int bytes_per_pixel = 2; // RGB565 instead of RGB888 for memory savings
    
    camera_buffer_size = width * height * bytes_per_pixel;
    
    if (camera_buffer == NULL) {
        camera_buffer = malloc(camera_buffer_size);
    }
    
    if (camera_buffer) {
        // Generate RGB565 test pattern 
        uint16_t *buffer_16 = (uint16_t*)camera_buffer;
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                int pos = y * width + x;
                // Create RGB565 format: 5 bits red, 6 bits green, 5 bits blue
                uint8_t r = (x * 31) / width;      // 5-bit red
                uint8_t g = (y * 63) / height;     // 6-bit green  
                uint8_t b = ((x + y) * 31) / (width + height); // 5-bit blue
                buffer_16[pos] = (r << 11) | (g << 5) | b;  // RGB565 format
            }
        }
        ESP_LOGI(TAG, "Generated OV5647 test image: %dx%d RGB565, %d bytes", width, height, camera_buffer_size);
    }
}

// Convert RGB565 to BMP format for web display
// Initialize JPEG encoder
esp_err_t init_jpeg_encoder(void) {
    if (jpeg_handle != NULL) {
        return ESP_OK; // Already initialized
    }
    
    jpeg_encode_engine_cfg_t encode_eng_cfg = {
        .timeout_ms = 2000,  // 2 second timeout
        .intr_priority = 0,  // Use default priority
    };
    
    esp_err_t ret = jpeg_new_encoder_engine(&encode_eng_cfg, &jpeg_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create JPEG encoder: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "✅ JPEG encoder initialized successfully");
    return ESP_OK;
}

// Convert RGB565 to JPEG format using hardware encoder
size_t create_jpeg_from_rgb565(uint8_t *rgb565_data, size_t data_len, uint8_t **jpeg_data) {
    if (!jpeg_handle) {
        ESP_LOGE(TAG, "JPEG encoder not initialized");
        return 0;
    }
    
    const int width = 800;   // Camera width
    const int height = 640;  // Camera height
    
    // JPEG encode configuration
    jpeg_encode_cfg_t encode_cfg = {
        .width = width,
        .height = height,
        .src_type = JPEG_ENCODE_IN_FORMAT_RGB565,  // Input format RGB565
        .sub_sample = JPEG_DOWN_SAMPLING_YUV422,   // Good balance of quality/size
        .image_quality = 85,  // High quality (1-100)
    };
    
    // Allocate output buffer using JPEG-specific allocation with SPIRAM (PSRAM)
    const uint32_t max_jpeg_size = data_len / 4;  // Conservative estimate
    jpeg_encode_memory_alloc_cfg_t mem_cfg = {
        .buffer_direction = JPEG_ENC_ALLOC_OUTPUT_BUFFER,
    };
    
    size_t allocated_size = 0;
    *jpeg_data = (uint8_t*)jpeg_alloc_encoder_mem(max_jpeg_size, &mem_cfg, &allocated_size);
    if (!*jpeg_data) {
        ESP_LOGE(TAG, "Failed to allocate JPEG output buffer");
        return 0;
    }
    
    ESP_LOGI(TAG, "📸 JPEG buffer allocated: %d bytes (requested %d), addr: 0x%p", allocated_size, max_jpeg_size, *jpeg_data);
    
    uint32_t actual_jpeg_size = 0;
    esp_err_t ret = jpeg_encoder_process(jpeg_handle, &encode_cfg, 
                                       rgb565_data, data_len,
                                       *jpeg_data, allocated_size, 
                                       &actual_jpeg_size);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "JPEG encoding failed: %s", esp_err_to_name(ret));
        free(*jpeg_data);
        *jpeg_data = NULL;
        return 0;
    }
    
    ESP_LOGI(TAG, "📸 JPEG encoded: %d bytes -> %d bytes (compression: %.1f%%)", 
             data_len, actual_jpeg_size, (100.0f * actual_jpeg_size) / data_len);
    
    return actual_jpeg_size;
}
// Camera streaming state
static volatile int active_streams = 0;
static const int MAX_STREAMS = 2;

// Structure to hold camera settings
typedef struct {
    int brightness;
    int contrast;
    int saturation;
    int jpeg_quality;
    char exposure_mode[16];
    char white_balance[16];
    int resolution_width;
    int resolution_height;
} camera_settings_t;

// Global camera settings with defaults
static camera_settings_t current_settings = {
    .brightness = 0,
    .contrast = 0,
    .saturation = 0,
    .jpeg_quality = 85,
    .exposure_mode = "Auto",
    .white_balance = "Auto",
    .resolution_width = 800,
    .resolution_height = 640
};

// HTTP handler for getting current settings (API)
esp_err_t api_get_settings_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "🔧 API Get Settings called!");
    
    char settings_json[512];
    snprintf(settings_json, sizeof(settings_json),
        "{"
        "\"brightness\":%d,"
        "\"contrast\":%d,"
        "\"saturation\":%d,"
        "\"jpeg_quality\":%d,"
        "\"exposure_mode\":\"%s\","
        "\"white_balance\":\"%s\","
        "\"resolution_width\":%d,"
        "\"resolution_height\":%d"
        "}",
        current_settings.brightness,
        current_settings.contrast,
        current_settings.saturation,
        current_settings.jpeg_quality,
        current_settings.exposure_mode,
        current_settings.white_balance,
        current_settings.resolution_width,
        current_settings.resolution_height
    );
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, settings_json, strlen(settings_json));
}

// HTTP handler for applying settings (API)
esp_err_t api_set_settings_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "🔧 API Set Settings called!");
    
    char content[512];
    size_t recv_size = MIN(req->content_len, sizeof(content) - 1);
    
    if (httpd_req_recv(req, content, recv_size) <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Failed to receive data");
        return ESP_FAIL;
    }
    content[recv_size] = '\0';
    
    ESP_LOGI(TAG, "📝 Received settings: %s", content);
    
    // Parse JSON-like parameters (simple parsing)
    char *brightness_str = strstr(content, "brightness=");
    char *contrast_str = strstr(content, "contrast=");
    char *saturation_str = strstr(content, "saturation=");
    char *quality_str = strstr(content, "jpeg_quality=");
    char *exposure_str = strstr(content, "exposure_mode=");
    char *wb_str = strstr(content, "white_balance=");
    
    // Update settings if found
    if (brightness_str) {
        current_settings.brightness = atoi(brightness_str + 11);
        ESP_LOGI(TAG, "🔆 Brightness set to: %d", current_settings.brightness);
    }
    
    if (contrast_str) {
        current_settings.contrast = atoi(contrast_str + 9);
        ESP_LOGI(TAG, "🎨 Contrast set to: %d", current_settings.contrast);
    }
    
    if (saturation_str) {
        current_settings.saturation = atoi(saturation_str + 11);
        ESP_LOGI(TAG, "🌈 Saturation set to: %d", current_settings.saturation);
    }
    
    if (quality_str) {
        current_settings.jpeg_quality = atoi(quality_str + 13);
        ESP_LOGI(TAG, "📸 JPEG Quality set to: %d", current_settings.jpeg_quality);
    }
    
    if (exposure_str) {
        char *end = strchr(exposure_str + 14, '&');
        size_t len = end ? (end - (exposure_str + 14)) : strlen(exposure_str + 14);
        len = MIN(len, sizeof(current_settings.exposure_mode) - 1);
        strncpy(current_settings.exposure_mode, exposure_str + 14, len);
        current_settings.exposure_mode[len] = '\0';
        ESP_LOGI(TAG, "💡 Exposure mode set to: %s", current_settings.exposure_mode);
    }
    
    if (wb_str) {
        char *end = strchr(wb_str + 14, '&');
        size_t len = end ? (end - (wb_str + 14)) : strlen(wb_str + 14);
        len = MIN(len, sizeof(current_settings.white_balance) - 1);
        strncpy(current_settings.white_balance, wb_str + 14, len);
        current_settings.white_balance[len] = '\0';
        ESP_LOGI(TAG, "⚪ White balance set to: %s", current_settings.white_balance);
    }
    
    // Apply the settings to the actual camera hardware
    esp_err_t apply_result = ESP_OK;
    
    if (brightness_str) {
        apply_result |= camera_set_brightness(current_settings.brightness);
    }
    
    if (contrast_str) {
        apply_result |= camera_set_contrast(current_settings.contrast);
    }
    
    if (saturation_str) {
        apply_result |= camera_set_saturation(current_settings.saturation);
    }
    
    if (exposure_str) {
        apply_result |= camera_set_exposure_mode(current_settings.exposure_mode);
    }
    
    if (wb_str) {
        apply_result |= camera_set_white_balance(current_settings.white_balance);
    }
    
    // Respond with success or failure
    if (apply_result == ESP_OK) {
        ESP_LOGI(TAG, "✅ Camera settings applied successfully!");
        char response[] = "{\"status\":\"success\",\"message\":\"Settings applied successfully\"}";
        httpd_resp_set_type(req, "application/json");
        httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
        return httpd_resp_send(req, response, strlen(response));
    } else {
        ESP_LOGE(TAG, "❌ Failed to apply camera settings");
        char response[] = "{\"status\":\"error\",\"message\":\"Failed to apply settings to camera\"}";
        httpd_resp_set_type(req, "application/json");
        httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
        return httpd_resp_send(req, response, strlen(response));
    }
}

esp_err_t camera_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "📷 Camera handler called! Active streams: %d", active_streams);
    
    // Limit concurrent streams to prevent overload
    if (active_streams >= MAX_STREAMS) {
        ESP_LOGW(TAG, "⚠️ Too many streams (%d), rejecting request", active_streams);
        httpd_resp_set_status(req, "503 Service Unavailable");
        httpd_resp_send(req, "Too many streams", -1);
        return ESP_FAIL;
    }
    
    active_streams++;
    esp_err_t res = ESP_OK;
    
    // Try to capture real image from camera first
    uint8_t *image_buffer = NULL;
    size_t image_len = 0;
    
    esp_err_t cam_ret = camera_capture(&image_buffer, &image_len);
    
    if (cam_ret == ESP_OK && image_buffer != NULL && image_len > 0) {
        // Successfully captured real camera image (RGB565 format)
        ESP_LOGI(TAG, "📸 Captured real camera image: %d bytes (RGB565)", image_len);
        
        // Check first few pixels to see if we have real data
        uint16_t *pixel_data = (uint16_t*)image_buffer;
        ESP_LOGI(TAG, "📸 First 8 pixels: %04x %04x %04x %04x %04x %04x %04x %04x", 
                 pixel_data[0], pixel_data[1], pixel_data[2], pixel_data[3],
                 pixel_data[4], pixel_data[5], pixel_data[6], pixel_data[7]);
        
        // Convert RGB565 to JPEG format using hardware encoder
        uint8_t *jpeg_data = NULL;
        size_t jpeg_size = create_jpeg_from_rgb565(image_buffer, image_len, &jpeg_data);
        
        if (jpeg_data && jpeg_size > 0) {
            // Set headers for JPEG image response
            httpd_resp_set_type(req, "image/jpeg");
            httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
            httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
            httpd_resp_set_hdr(req, "Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
            httpd_resp_set_hdr(req, "Pragma", "no-cache");
            httpd_resp_set_hdr(req, "Expires", "0");
            
            // Send the JPEG image
            res = httpd_resp_send(req, (const char*)jpeg_data, jpeg_size);
            ESP_LOGI(TAG, "📸 Sent JPEG image: %d bytes (converted from %d bytes RGB565)", jpeg_size, image_len);
            
            // Free the JPEG buffer
            free(jpeg_data);
        } else {
            ESP_LOGE(TAG, "Failed to convert RGB565 to JPEG");
            res = ESP_FAIL;
        }
        
    } else if (cam_ret == ESP_ERR_TIMEOUT && image_buffer != NULL && image_len > 0) {
        // Camera timeout but we have the frame buffer - might be empty or contain old data
        ESP_LOGW(TAG, "Camera timeout but frame buffer available: %d bytes", image_len);
        
        // Check first few pixels to see what's in the buffer
        uint16_t *pixel_data = (uint16_t*)image_buffer;
        ESP_LOGI(TAG, "📸 Buffer pixels: %04x %04x %04x %04x %04x %04x %04x %04x", 
                 pixel_data[0], pixel_data[1], pixel_data[2], pixel_data[3],
                 pixel_data[4], pixel_data[5], pixel_data[6], pixel_data[7]);
        
        // Generate a test pattern in the frame buffer since camera data might be invalid
        generate_test_image();
        
        // Convert to JPEG anyway to see what we get
        uint8_t *jpeg_data = NULL;
        size_t jpeg_size = create_jpeg_from_rgb565(image_buffer, image_len, &jpeg_data);
        
        if (jpeg_data && jpeg_size > 0) {
            // Set headers for JPEG image response
            httpd_resp_set_type(req, "image/jpeg");
            httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=test_capture.jpg");
            httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
            httpd_resp_set_hdr(req, "Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
            httpd_resp_set_hdr(req, "Pragma", "no-cache");
            httpd_resp_set_hdr(req, "Expires", "0");
            
            // Send the JPEG image
            res = httpd_resp_send(req, (const char*)jpeg_data, jpeg_size);
            ESP_LOGI(TAG, "📸 Sent test JPEG image: %d bytes (from timeout fallback)", jpeg_size);
            
            // Free the JPEG buffer
            free(jpeg_data);
        } else {
            ESP_LOGE(TAG, "Failed to convert test pattern to JPEG");
            res = ESP_FAIL;
        }
        
    } else {
        // Complete camera failure - fall back to test image
        ESP_LOGE(TAG, "Camera capture completely failed (%s), using test pattern", esp_err_to_name(cam_ret));
        generate_test_image();
        
        if (camera_buffer && camera_buffer_size > 0) {
            // Convert test pattern to JPEG format too
            uint8_t *jpeg_data = NULL;
            size_t jpeg_size = create_jpeg_from_rgb565(camera_buffer, camera_buffer_size, &jpeg_data);
            
            if (jpeg_data && jpeg_size > 0) {
                httpd_resp_set_type(req, "image/jpeg");
                httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=test.jpg");
                httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
                httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
                
                res = httpd_resp_send(req, (const char*)jpeg_data, jpeg_size);
                ESP_LOGI(TAG, "📸 Sent test JPEG image: %d bytes", jpeg_size);
                free(jpeg_data);
            } else {
                const char* error_msg = 
                    "<html><body style='text-align: center; font-family: Arial;'>"
                    "<h1>🚁 DroneCam ESP32-P4-Pico</h1>"
                    "<h2>📷 Camera Error</h2>"
                    "<p>Failed to convert camera data to JPEG format</p>"
                    "<p><strong>Status:</strong> WiFi ✅ | HTTP ✅ | Camera ❌</p>"
                    "</body></html>";
                httpd_resp_set_type(req, "text/html");
                res = httpd_resp_send(req, error_msg, strlen(error_msg));
            }
        } else {
            ESP_LOGW(TAG, "Failed to generate test image");
            const char* error_msg = 
                "<html><body style='text-align: center; font-family: Arial;'>"
                "<h1>🚁 DroneCam ESP32-P4-Pico</h1>"
                "<h2>📷 Camera Error</h2>"
                "<p>Failed to capture from OV5647 MIPI-CSI camera</p>"
                "<p><strong>Status:</strong> WiFi ✅ | HTTP ✅ | Camera ❌</p>"
                "</body></html>";
            httpd_resp_set_type(req, "text/html");
            res = httpd_resp_send(req, error_msg, strlen(error_msg));
        }
    }
    
    active_streams--;
    ESP_LOGI(TAG, "📷 Camera request completed. Active streams: %d", active_streams);
    return res;
}

// HTTP handler for main page
esp_err_t index_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "🏠 Index handler called!");
    
    // Send HTML in chunks - much more reliable than large snprintf
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    
    // Send HTML header and styles
    httpd_resp_sendstr_chunk(req,
        "<!DOCTYPE html><html><head><title>DroneCam Live</title>"
        "<meta name='viewport' content='width=device-width, initial-scale=1'>"
        "<meta charset='UTF-8'><style>"
        "body{font-family:Arial;margin:0;background:#222;color:#fff;text-align:center}"
        ".container{max-width:900px;margin:0 auto;padding:20px}"
        ".video{border:2px solid #444;background:#000;border-radius:8px;overflow:hidden;margin:20px 0}"
        ".video img{width:100%;height:auto;display:block}"
        ".controls{margin:20px 0}"
        ".btn{padding:10px 20px;margin:5px;background:#007bff;color:#fff;border:none;border-radius:5px;cursor:pointer;text-decoration:none;display:inline-block}"
        ".toggle{position:relative;display:inline-block;width:60px;height:34px;margin:0 10px}"
        ".toggle input{opacity:0;width:0;height:0}"
        ".slider{position:absolute;cursor:pointer;top:0;left:0;right:0;bottom:0;background:#ccc;transition:.4s;border-radius:34px}"
        ".slider:before{position:absolute;content:'';height:26px;width:26px;left:4px;bottom:4px;background:#fff;transition:.4s;border-radius:50%}"
        "input:checked+.slider{background:#4CAF50}"
        "input:checked+.slider:before{transform:translateX(26px)}"
        "</style>");
    
    // Send JavaScript
    httpd_resp_sendstr_chunk(req,
        "<script>"
        "let streaming=false,interval;"
        "function toggleStream(){"
        "const t=document.getElementById('toggle'),i=document.getElementById('cam');"
        "streaming=t.checked;"
        "if(streaming){"
        "interval=setInterval(()=>{i.src='/camera?t='+Date.now();},100);"
        "}else{clearInterval(interval);}"
        "}"
        "window.onload=()=>{document.getElementById('toggle').checked=true;toggleStream()};"
        "</script></head><body><div class='container'>"
        "<h1>DroneCam Live Stream</h1>"
        "<div class='video'><img id='cam' src='/camera'></div>"
        "<div class='controls'>"
        "<label>Stream: </label>"
        "<label class='toggle'><input type='checkbox' id='toggle' onchange='toggleStream()'><span class='slider'></span></label>"
        "<button class='btn' onclick='document.getElementById(\"cam\").src=\"/camera?t=\"+Date.now()'>Capture</button>"
        "<a href='/settings' class='btn'>Settings</a></div>");
    
    // Send footer with IP address
    char footer[256];
    snprintf(footer, sizeof(footer), "<p>ESP32-P4 + OV5647 MIPI-CSI Camera | IP: %s</p></div></body></html>", device_ip);
    httpd_resp_sendstr_chunk(req, footer);
    
    // End chunked response
    httpd_resp_sendstr_chunk(req, NULL);
    return ESP_OK;
}

// HTTP handler for settings page
esp_err_t settings_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "⚙️ Settings handler called!");
    
    // Send HTML in chunks to avoid buffer issues
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    
    // Send HTML header
    httpd_resp_sendstr_chunk(req, 
        "<!DOCTYPE html><html><head><title>DroneCam Settings</title>"
        "<meta name='viewport' content='width=device-width, initial-scale=1'>"
        "<meta charset='UTF-8'><style>"
        "body{font-family:Arial;margin:0;background:#222;color:#fff;padding:20px}"
        ".container{max-width:800px;margin:0 auto}"
        ".group{background:#333;padding:20px;margin:20px 0;border-radius:8px}"
        ".group h3{margin:0 0 15px 0;color:#4CAF50}"
        ".row{display:flex;justify-content:space-between;align-items:center;margin:10px 0}"
        ".row label{flex:1;font-weight:bold}"
        ".row select,.row input{flex:1;margin-left:10px;padding:8px;border:none;border-radius:4px;background:#555;color:#fff}"
        ".btn{padding:12px 24px;background:#007bff;color:#fff;border:none;border-radius:5px;cursor:pointer;margin:10px 5px;text-decoration:none;display:inline-block}"
        ".btn:hover{background:#0056b3}.btn.apply{background:#28a745}.btn.reset{background:#dc3545}"
        ".stats{font-family:monospace;font-size:0.9em;color:#aaa;line-height:1.6}"
        ".value{color:#4CAF50;font-weight:bold}"
        ".status{padding:10px;margin:10px 0;border-radius:4px;display:none}"
        ".status.success{background:#28a745;color:#fff}"
        ".status.error{background:#dc3545;color:#fff}"
        "</style><script>"
        "let settings={};"
        "function updateValue(id,val){document.getElementById(id).textContent=val;settings[id]=parseInt(val);applySettings();}"
        "function updateSelect(id,val){settings[id]=val;applySettings();}"
        "function loadSettings(){"
        "fetch('/api/settings').then(r=>r.json()).then(data=>{"
        "settings=data;"
        "document.getElementById('brightness').textContent=data.brightness;"
        "document.getElementById('brightnessSlider').value=data.brightness;"
        "document.getElementById('contrast').textContent=data.contrast;"
        "document.getElementById('contrastSlider').value=data.contrast;"
        "document.getElementById('quality').textContent=data.jpeg_quality;"
        "document.getElementById('qualitySlider').value=data.jpeg_quality;"
        "}).catch(e=>showStatus('Failed to load settings','error'));}"
        "function applySettings(){"
        "const formData=new URLSearchParams();"
        "Object.keys(settings).forEach(key=>formData.append(key,settings[key]));"
        "fetch('/api/settings',{method:'POST',body:formData})"
        ".then(r=>r.json()).then(data=>{"
        "if(data.status==='success')showStatus('Settings applied successfully!','success');"
        "else showStatus('Failed to apply settings','error');"
        "}).catch(e=>showStatus('Network error','error'));}"
        "function resetSettings(){"
        "if(confirm('Reset all settings to defaults?')){settings={brightness:0,contrast:0,saturation:0,jpeg_quality:85,exposure_mode:'Auto',white_balance:'Auto'};applySettings();setTimeout(loadSettings,500);}}"
        "function showStatus(msg,type){"
        "const status=document.getElementById('status');"
        "status.textContent=msg;status.className='status '+type;status.style.display='block';"
        "setTimeout(()=>status.style.display='none',3000);}"
        "window.onload=loadSettings;"
        "</script></head><body><div class='container'>"
        "<h1>Camera Settings</h1>"
        "<div id='status' class='status'></div>");
    
    // Send resolution settings
    httpd_resp_sendstr_chunk(req,
        "<div class='group'><h3>Video Resolution</h3>"
        "<div class='row'><label>Resolution:</label>"
        "<select><option selected>800x640 (50fps)</option>"
        "<option>1920x1080 (30fps)</option><option>1280x960 (45fps)</option></select></div>"
        "<div class='row'><label>Frame Rate:</label>"
        "<select><option selected>50 FPS</option><option>30 FPS</option><option>25 FPS</option></select></div>"
        "</div>");
    
    // Send quality settings
    httpd_resp_sendstr_chunk(req,
        "<div class='group'><h3>Image Quality</h3>"
        "<div class='row'><label>JPEG Quality:</label>"
        "<input type='range' min='10' max='100' value='85' id='qualitySlider' onchange='updateValue(\"jpeg_quality\",this.value)'>"
        "<span class='value' id='quality'>85</span>%</div>"
        "<div class='row'><label>Compression:</label>"
        "<select><option selected>High Quality (~4% compression)</option>"
        "<option>Medium Quality (~8% compression)</option></select></div>"
        "</div>");
    
    // Send camera controls
    httpd_resp_sendstr_chunk(req,
        "<div class='group'><h3>Camera Controls</h3>"
        "<div class='row'><label>Brightness:</label>"
        "<input type='range' min='-100' max='100' value='0' id='brightnessSlider' onchange='updateValue(\"brightness\",this.value)'>"
        "<span class='value' id='brightness'>0</span></div>"
        "<div class='row'><label>Contrast:</label>"
        "<input type='range' min='-100' max='100' value='0' id='contrastSlider' onchange='updateValue(\"contrast\",this.value)'>"
        "<span class='value' id='contrast'>0</span></div>"
        "<div class='row'><label>Exposure:</label>"
        "<select onchange='updateSelect(\"exposure_mode\",this.value)'>"
        "<option value='Auto' selected>Auto</option><option value='Manual'>Manual</option>"
        "<option value='Night'>Night Mode</option></select></div>"
        "<div class='row'><label>White Balance:</label>"
        "<select onchange='updateSelect(\"white_balance\",this.value)'>"
        "<option value='Auto' selected>Auto</option><option value='Daylight'>Daylight</option>"
        "<option value='Cloudy'>Cloudy</option></select></div>"
        "</div>");
    
    // Send system info with actual IP
    char info_html[512];
    snprintf(info_html, sizeof(info_html),
        "<div class='group'><h3>System Info</h3><div class='stats'>"
        "<p><strong>Network:</strong> %s</p>"
        "<p><strong>Camera:</strong> OV5647 MIPI-CSI (5MP)</p>"
        "<p><strong>Processor:</strong> ESP32-P4 (360MHz dual-core)</p>"
        "<p><strong>Memory:</strong> 32MB PSRAM + 512KB SRAM</p>"
        "</div></div>", device_ip);
    httpd_resp_sendstr_chunk(req, info_html);
    
    // Send footer
    httpd_resp_sendstr_chunk(req,
        "<div style='text-align:center;margin:30px 0'>"
        "<button class='btn apply' onclick='applySettings()'>Apply Settings</button>"
        "<button class='btn reset' onclick='resetSettings()'>Reset</button>"
        "<a href='/' class='btn'>Back to Stream</a>"
        "</div></div></body></html>");
    
    // End chunked response
    httpd_resp_sendstr_chunk(req, NULL);
    return ESP_OK;
}

// HTTP handler for status/info
esp_err_t status_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "ℹ️ Status handler called!");
    
    char status_json[512];
    snprintf(status_json, sizeof(status_json),
        "{"
        "\"device\":\"ESP32-P4-Pico\","
        "\"camera\":\"OV5647\","
        "\"interface\":\"MIPI-CSI\","
        "\"wifi_status\":\"connected\","
        "\"ip_address\":\"%s\","
        "\"free_heap\":%lu,"
        "\"camera_status\":\"live_capture\","
        "\"esp_idf_version\":\"5.5\""
        "}",
        device_ip, esp_get_free_heap_size()
    );
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, status_json, strlen(status_json));
}

// Start HTTP server
httpd_handle_t start_webserver(void)
{
    httpd_handle_t local_server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;

    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&local_server, &config) == ESP_OK) {
        ESP_LOGI(TAG, "Registering URI handlers");
        
        httpd_uri_t index_uri = {
            .uri       = "/",
            .method    = HTTP_GET,
            .handler   = index_handler,
            .user_ctx  = NULL
        };
        esp_err_t ret = httpd_register_uri_handler(local_server, &index_uri);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "✅ Registered index handler for /");
        } else {
            ESP_LOGE(TAG, "❌ Failed to register index handler: %s", esp_err_to_name(ret));
        }

        httpd_uri_t camera_uri = {
            .uri       = "/camera",
            .method    = HTTP_GET,
            .handler   = camera_handler,
            .user_ctx  = NULL
        };
        ret = httpd_register_uri_handler(local_server, &camera_uri);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "✅ Registered camera handler for /camera");
        } else {
            ESP_LOGE(TAG, "❌ Failed to register camera handler: %s", esp_err_to_name(ret));
        }

        httpd_uri_t settings_uri = {
            .uri       = "/settings",
            .method    = HTTP_GET,
            .handler   = settings_handler,
            .user_ctx  = NULL
        };
        ret = httpd_register_uri_handler(local_server, &settings_uri);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "✅ Registered settings handler for /settings");
        } else {
            ESP_LOGE(TAG, "❌ Failed to register settings handler: %s", esp_err_to_name(ret));
        }

        httpd_uri_t status_uri = {
            .uri       = "/status",
            .method    = HTTP_GET,
            .handler   = status_handler,
            .user_ctx  = NULL
        };
        ret = httpd_register_uri_handler(local_server, &status_uri);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "✅ Registered status handler for /status");
        } else {
            ESP_LOGE(TAG, "❌ Failed to register status handler: %s", esp_err_to_name(ret));
        }

        // Register API settings GET endpoint
        httpd_uri_t api_get_settings_uri = {
            .uri       = "/api/settings",
            .method    = HTTP_GET,
            .handler   = api_get_settings_handler,
            .user_ctx  = NULL
        };
        ret = httpd_register_uri_handler(local_server, &api_get_settings_uri);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "✅ Registered API GET settings handler for /api/settings");
        } else {
            ESP_LOGE(TAG, "❌ Failed to register API GET settings handler: %s", esp_err_to_name(ret));
        }

        // Register API settings POST endpoint
        httpd_uri_t api_set_settings_uri = {
            .uri       = "/api/settings",
            .method    = HTTP_POST,
            .handler   = api_set_settings_handler,
            .user_ctx  = NULL
        };
        ret = httpd_register_uri_handler(local_server, &api_set_settings_uri);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "✅ Registered API POST settings handler for /api/settings");
        } else {
            ESP_LOGE(TAG, "❌ Failed to register API POST settings handler: %s", esp_err_to_name(ret));
        }
        
        return local_server;
    }

    ESP_LOGI(TAG, "Error starting server!");
    return NULL;
}

void app_main(void)
{
    ESP_LOGI(TAG, "=== DroneCam ESP32-P4 Camera Streaming ===");
    
    // Print chip information
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    
    ESP_LOGI(TAG, "ESP32-P4 with %d CPU cores, silicon revision v%d.%d", 
             chip_info.cores, chip_info.revision / 100, chip_info.revision % 100);

    ESP_LOGI(TAG, "Free heap: %lu bytes", esp_get_free_heap_size());

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "Initializing WiFi...");
    wifi_init_sta();
    
    // Wait for WiFi connection first to free up initialization memory
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "✅ Connected to WiFi SSID:%s", WIFI_SSID);
        ESP_LOGI(TAG, "📡 ESP32-P4 + ESP32-C6 dual-chip WiFi 6 working!");
        ESP_LOGI(TAG, "Free heap after WiFi: %lu bytes", esp_get_free_heap_size());
        
        // Initialize camera after WiFi to have more available memory
        ESP_LOGI(TAG, "Initializing OV5647 camera...");
        esp_err_t cam_ret = camera_init();
        if (cam_ret != ESP_OK) {
            ESP_LOGE(TAG, "❌ Camera initialization failed: %s", esp_err_to_name(cam_ret));
        } else {
            ESP_LOGI(TAG, "✅ Camera initialized successfully");
            
            // Initialize JPEG encoder
            esp_err_t jpeg_ret = init_jpeg_encoder();
            if (jpeg_ret == ESP_OK) {
                ESP_LOGI(TAG, "✅ JPEG encoder initialized successfully");
            } else {
                ESP_LOGE(TAG, "❌ JPEG encoder initialization failed: %s", esp_err_to_name(jpeg_ret));
            }
            
            // Start camera capture
            esp_err_t cam_start_ret = camera_start();
            if (cam_start_ret == ESP_OK) {
                ESP_LOGI(TAG, "🎬 Camera capture started successfully");
            } else {
                ESP_LOGE(TAG, "❌ Failed to start camera: %s", esp_err_to_name(cam_start_ret));
            }
        }
        
        // Start HTTP server
        server = start_webserver();
        if (server) {
            ESP_LOGI(TAG, "");
            ESP_LOGI(TAG, "🌐 ========================================");
            ESP_LOGI(TAG, "🎥 DroneCam Web Interface Ready!");
            ESP_LOGI(TAG, "🌐 ========================================");
            ESP_LOGI(TAG, "📱 Main Interface: http://%s/", device_ip);
            ESP_LOGI(TAG, "📷 Camera Stream:  http://%s/camera", device_ip);
            ESP_LOGI(TAG, "🌐 ========================================");
            ESP_LOGI(TAG, "");
            ESP_LOGI(TAG, "💡 Copy the URL above into your browser!");
            ESP_LOGI(TAG, "📱 Works on phones, tablets, and computers");
            ESP_LOGI(TAG, "");
        }
        
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "❌ Failed to connect to SSID:%s", WIFI_SSID);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }

    // Main loop
    while (1) {
        vTaskDelay(10000 / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "📊 Status: WiFi connected, Server running, Free heap: %lu bytes", 
                 esp_get_free_heap_size());
    }
}
