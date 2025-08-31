/*
 * SLAM Core Implementation
 * Main visual SLAM processing engine
 */

#include "slam_core.h"
#include "orb_features.h"
#include "camera_init.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <math.h>
#include <string.h>

static const char* TAG = "slam_core";

// Camera constants (matching camera_init.c)
#define CAMERA_WIDTH               800
#define CAMERA_HEIGHT              640
#define CAMERA_FORMAT_RGB565       1

// Global SLAM state
static struct {
    slam_config_t config;
    slam_stats_t stats;
    slam_pose_t current_pose;
    
    // Map data
    keyframe_t* keyframes;
    uint32_t num_keyframes;
    map_point_t* map_points;
    uint32_t num_map_points;
    
    // GPS origin for coordinate conversion
    double origin_lat, origin_lon;
    float origin_alt;
    bool origin_set;
    
    // Synchronization
    SemaphoreHandle_t slam_mutex;
    
    // Memory management
    bool initialized;
} g_slam_state = {0};

// Global camera configuration
static camera_config_t g_camera_config = {
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
    .brightness_threshold = 10.0f
};

// Earth radius for GPS calculations (meters)
#define EARTH_RADIUS_M 6378137.0

// Default SLAM configuration
static const slam_config_t default_config = {
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
    .keyframe_angle_threshold = 0.2f,  // ~11 degrees
    .max_keyframes = 50,
    .enable_loop_closure = true,
    .loop_closure_threshold = 0.8f
};

// Forward declarations
static esp_err_t initialize_map_memory(void);
static void cleanup_map_memory(void);
static esp_err_t track_features(const camera_frame_t* frame, slam_pose_t* pose);
static bool should_create_keyframe(const slam_pose_t* current_pose);
static esp_err_t create_keyframe(const camera_frame_t* frame, const slam_pose_t* pose);
static esp_err_t triangulate_new_points(void);
static esp_err_t optimize_local_map(void);
static void adjust_camera_settings(const camera_frame_t* frame);

esp_err_t slam_core_init(const slam_config_t* config) {
    if (g_slam_state.initialized) {
        ESP_LOGW(TAG, "SLAM core already initialized");
        return ESP_OK;
    }
    
    // Use provided config or default
    if (config) {
        g_slam_state.config = *config;
    } else {
        g_slam_state.config = default_config;
    }
    
    // Create mutex for thread safety
    g_slam_state.slam_mutex = xSemaphoreCreateMutex();
    if (!g_slam_state.slam_mutex) {
        ESP_LOGE(TAG, "Failed to create SLAM mutex");
        return ESP_ERR_NO_MEM;
    }
    
    // Initialize ORB feature detector
    esp_err_t ret = orb_features_init(g_slam_state.config.max_features);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize ORB features: %s", esp_err_to_name(ret));
        vSemaphoreDelete(g_slam_state.slam_mutex);
        return ret;
    }
    
    // Allocate memory for map data
    ret = initialize_map_memory();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize map memory: %s", esp_err_to_name(ret));
        orb_features_deinit();
        vSemaphoreDelete(g_slam_state.slam_mutex);
        return ret;
    }
    
    // Initialize state
    g_slam_state.stats.state = SLAM_STATE_NOT_INITIALIZED;
    g_slam_state.current_pose = (slam_pose_t){
        .position = {0, 0, 0},
        .orientation = {1, 0, 0, 0},
        .x = 0, .y = 0, .z = 0,
        .qw = 1, .qx = 0, .qy = 0, .qz = 0,
        .confidence = 0,
        .is_lost = true,
        .timestamp_us = 0
    };
    
    g_slam_state.origin_set = false;
    g_slam_state.initialized = true;
    
    ESP_LOGI(TAG, "SLAM core initialized with %d max features", g_slam_state.config.max_features);
    return ESP_OK;
}

esp_err_t slam_core_deinit(void) {
    if (!g_slam_state.initialized) {
        return ESP_OK;
    }
    
    xSemaphoreTake(g_slam_state.slam_mutex, portMAX_DELAY);
    
    // Cleanup components
    orb_features_deinit();
    cleanup_map_memory();
    
    g_slam_state.initialized = false;
    
    xSemaphoreGive(g_slam_state.slam_mutex);
    vSemaphoreDelete(g_slam_state.slam_mutex);
    
    ESP_LOGI(TAG, "SLAM core deinitialized");
    return ESP_OK;
}

esp_err_t slam_core_process_frame(const camera_frame_t* frame, slam_pose_t* output_pose) {
    if (!g_slam_state.initialized || !frame || !output_pose) {
        return ESP_ERR_INVALID_ARG;
    }
    
    xSemaphoreTake(g_slam_state.slam_mutex, portMAX_DELAY);
    
    uint64_t start_time = esp_timer_get_time();
    esp_err_t ret = ESP_OK;
    
    // Process frame based on current state
    switch (g_slam_state.stats.state) {
        case SLAM_STATE_NOT_INITIALIZED:
        case SLAM_STATE_INITIALIZING:
            // Try to initialize SLAM system
            ret = track_features(frame, &g_slam_state.current_pose);
            if (ret == ESP_OK && g_slam_state.current_pose.tracked_features >= g_slam_state.config.min_tracked_features) {
                g_slam_state.stats.state = SLAM_STATE_TRACKING;
                ESP_LOGI(TAG, "SLAM tracking initialized with %d features", g_slam_state.current_pose.tracked_features);
            }
            break;
            
        case SLAM_STATE_TRACKING:
            // Normal tracking
            ret = track_features(frame, &g_slam_state.current_pose);
            if (ret == ESP_OK) {
                if (g_slam_state.current_pose.tracked_features < g_slam_state.config.min_tracked_features) {
                    g_slam_state.stats.state = SLAM_STATE_LOST;
                    g_slam_state.current_pose.is_lost = true;
                    ESP_LOGW(TAG, "SLAM tracking lost - insufficient features");
                } else {
                    // Check if we need a new keyframe
                    if (should_create_keyframe(&g_slam_state.current_pose)) {
                        create_keyframe(frame, &g_slam_state.current_pose);
                        triangulate_new_points();
                        optimize_local_map();
                    }
                }
            }
            break;
            
        case SLAM_STATE_LOST:
        case SLAM_STATE_RELOCALIZATION:
            // Try to relocalize
            ret = track_features(frame, &g_slam_state.current_pose);
            if (ret == ESP_OK && g_slam_state.current_pose.tracked_features >= g_slam_state.config.min_tracked_features) {
                g_slam_state.stats.state = SLAM_STATE_TRACKING;
                g_slam_state.current_pose.is_lost = false;
                ESP_LOGI(TAG, "SLAM tracking recovered");
            }
            break;
    }
    
    // Auto-adjust camera settings if enabled
    if (g_camera_config.auto_adjustment_enabled) {
        adjust_camera_settings(frame);
    }
    
    // Update statistics
    uint64_t processing_time = esp_timer_get_time() - start_time;
    g_slam_state.stats.frames_processed++;
    g_slam_state.stats.current_features = g_slam_state.current_pose.tracked_features;
    g_slam_state.stats.tracking_confidence = g_slam_state.current_pose.confidence;
    g_slam_state.stats.last_update_us = esp_timer_get_time();
    
    // Update average processing time
    float alpha = 0.1f;  // Exponential moving average factor
    g_slam_state.stats.average_processing_time_ms = 
        (1.0f - alpha) * g_slam_state.stats.average_processing_time_ms + 
        alpha * (processing_time / 1000.0f);
    
    // Copy current pose to output
    *output_pose = g_slam_state.current_pose;
    
    xSemaphoreGive(g_slam_state.slam_mutex);
    return ret;
}

esp_err_t slam_core_set_gps_origin(double lat, double lon, float alt) {
    if (!g_slam_state.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    xSemaphoreTake(g_slam_state.slam_mutex, portMAX_DELAY);
    
    g_slam_state.origin_lat = lat;
    g_slam_state.origin_lon = lon;
    g_slam_state.origin_alt = alt;
    g_slam_state.origin_set = true;
    
    xSemaphoreGive(g_slam_state.slam_mutex);
    
    ESP_LOGI(TAG, "GPS origin set to %.8f, %.8f, %.2f", lat, lon, alt);
    return ESP_OK;
}

esp_err_t slam_core_gps_to_local(double lat, double lon, float alt, float* local_x, float* local_y, float* local_z) {
    if (!g_slam_state.initialized || !g_slam_state.origin_set || !local_x || !local_y || !local_z) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Convert GPS coordinates to local meters using equirectangular projection
    double lat_rad = lat * M_PI / 180.0;
    double lon_rad = lon * M_PI / 180.0;
    double origin_lat_rad = g_slam_state.origin_lat * M_PI / 180.0;
    double origin_lon_rad = g_slam_state.origin_lon * M_PI / 180.0;
    
    double cos_lat = cos((lat_rad + origin_lat_rad) / 2.0);
    
    *local_x = (float)((lon_rad - origin_lon_rad) * EARTH_RADIUS_M * cos_lat);
    *local_y = (float)((lat_rad - origin_lat_rad) * EARTH_RADIUS_M);
    *local_z = alt - g_slam_state.origin_alt;
    
    return ESP_OK;
}

esp_err_t slam_core_local_to_gps(float local_x, float local_y, float local_z, double* lat, double* lon, float* alt) {
    if (!g_slam_state.initialized || !g_slam_state.origin_set || !lat || !lon || !alt) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Convert local meters to GPS coordinates
    double origin_lat_rad = g_slam_state.origin_lat * M_PI / 180.0;
    double origin_lon_rad = g_slam_state.origin_lon * M_PI / 180.0;
    
    double lat_rad = origin_lat_rad + (local_y / EARTH_RADIUS_M);
    double cos_lat = cos((lat_rad + origin_lat_rad) / 2.0);
    double lon_rad = origin_lon_rad + (local_x / (EARTH_RADIUS_M * cos_lat));
    
    *lat = lat_rad * 180.0 / M_PI;
    *lon = lon_rad * 180.0 / M_PI;
    *alt = g_slam_state.origin_alt + local_z;
    
    return ESP_OK;
}

esp_err_t slam_core_get_stats(slam_stats_t* stats) {
    if (!g_slam_state.initialized || !stats) {
        return ESP_ERR_INVALID_ARG;
    }
    
    xSemaphoreTake(g_slam_state.slam_mutex, portMAX_DELAY);
    *stats = g_slam_state.stats;
    xSemaphoreGive(g_slam_state.slam_mutex);
    
    return ESP_OK;
}

esp_err_t slam_core_get_current_pose(slam_pose_t* pose) {
    if (!g_slam_state.initialized || !pose) {
        return ESP_ERR_INVALID_ARG;
    }
    
    xSemaphoreTake(g_slam_state.slam_mutex, portMAX_DELAY);
    *pose = g_slam_state.current_pose;
    xSemaphoreGive(g_slam_state.slam_mutex);
    
    return ESP_OK;
}

// Private helper functions

static esp_err_t initialize_map_memory(void) {
    // Calculate memory requirements
    size_t keyframes_size = g_slam_state.config.max_keyframes * sizeof(keyframe_t);
    size_t map_points_size = g_slam_state.config.max_features * 10 * sizeof(map_point_t);
    
    ESP_LOGI(TAG, "Allocating SLAM memory: %.1f KB keyframes + %.1f KB map points = %.1f KB total", 
             keyframes_size / 1024.0f, map_points_size / 1024.0f, 
             (keyframes_size + map_points_size) / 1024.0f);
    
    // Try SPIRAM first (preferred for large allocations)
    g_slam_state.keyframes = heap_caps_calloc(1, keyframes_size, MALLOC_CAP_SPIRAM);
    if (!g_slam_state.keyframes) {
        ESP_LOGW(TAG, "SPIRAM allocation failed for keyframes, trying internal RAM");
        g_slam_state.keyframes = heap_caps_calloc(1, keyframes_size, MALLOC_CAP_8BIT);
        if (!g_slam_state.keyframes) {
            ESP_LOGE(TAG, "Failed to allocate keyframes memory (%zu bytes)", keyframes_size);
            return ESP_ERR_NO_MEM;
        }
    }
    
    // Try SPIRAM first for map points
    g_slam_state.map_points = heap_caps_calloc(1, map_points_size, MALLOC_CAP_SPIRAM);
    if (!g_slam_state.map_points) {
        ESP_LOGW(TAG, "SPIRAM allocation failed for map points, trying internal RAM");
        g_slam_state.map_points = heap_caps_calloc(1, map_points_size, MALLOC_CAP_8BIT);
        if (!g_slam_state.map_points) {
            ESP_LOGE(TAG, "Failed to allocate map points memory (%zu bytes)", map_points_size);
            free(g_slam_state.keyframes);
            g_slam_state.keyframes = NULL;
            return ESP_ERR_NO_MEM;
        }
    }
    
    g_slam_state.num_keyframes = 0;
    g_slam_state.num_map_points = 0;
    
    // Log memory usage info
    multi_heap_info_t heap_info;
    heap_caps_get_info(&heap_info, MALLOC_CAP_SPIRAM);
    ESP_LOGI(TAG, "SPIRAM available: %zu KB, used SLAM: %.1f KB", 
             heap_info.total_free_bytes / 1024, (keyframes_size + map_points_size) / 1024.0f);
    
    ESP_LOGI(TAG, "SLAM memory allocation successful");
    return ESP_OK;
}

static void cleanup_map_memory(void) {
    if (g_slam_state.keyframes) {
        free(g_slam_state.keyframes);
        g_slam_state.keyframes = NULL;
    }
    
    if (g_slam_state.map_points) {
        free(g_slam_state.map_points);
        g_slam_state.map_points = NULL;
    }
    
    g_slam_state.num_keyframes = 0;
    g_slam_state.num_map_points = 0;
}

static esp_err_t track_features(const camera_frame_t* frame, slam_pose_t* pose) {
    // Extract ORB features from current frame
    orb_features_t features;
    esp_err_t ret = orb_extract_features(frame, &features);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to extract ORB features");
        return ret;
    }
    
    // For now, implement basic feature tracking
    // In a full implementation, this would:
    // 1. Match features with previous frame
    // 2. Estimate motion using matched features
    // 3. Update pose estimate
    // 4. Track map points in current frame
    
    pose->tracked_features = features.num_features;
    pose->confidence = (features.num_features >= g_slam_state.config.min_tracked_features) ? 0.8f : 0.3f;
    pose->timestamp_us = esp_timer_get_time();
    pose->is_lost = (features.num_features < g_slam_state.config.min_tracked_features);
    
    // Simulate some pose update for demonstration
    static float sim_x = 0, sim_y = 0;
    sim_x += 0.01f;  // Move 1cm per frame
    sim_y += 0.005f; // Move 0.5cm per frame
    
    pose->x = sim_x;
    pose->y = sim_y;
    pose->z = 0;
    
    return ESP_OK;
}

static bool should_create_keyframe(const slam_pose_t* current_pose) {
    if (g_slam_state.num_keyframes == 0) {
        return true;  // First keyframe
    }
    
    // Get last keyframe
    keyframe_t* last_kf = &g_slam_state.keyframes[g_slam_state.num_keyframes - 1];
    
    // Check distance threshold
    float dx = current_pose->x - last_kf->pose.x;
    float dy = current_pose->y - last_kf->pose.y;
    float dz = current_pose->z - last_kf->pose.z;
    float distance = sqrtf(dx*dx + dy*dy + dz*dz);
    
    if (distance > g_slam_state.config.keyframe_distance_threshold) {
        return true;
    }
    
    // Check angle threshold (simplified - should use proper quaternion math)
    // For now, just use a time-based approach
    uint64_t time_diff = current_pose->timestamp_us - last_kf->timestamp_us;
    if (time_diff > 1000000) {  // 1 second
        return true;
    }
    
    return false;
}

static esp_err_t create_keyframe(const camera_frame_t* frame, const slam_pose_t* pose) {
    if (g_slam_state.num_keyframes >= g_slam_state.config.max_keyframes) {
        ESP_LOGW(TAG, "Maximum keyframes reached, removing oldest");
        // In a full implementation, we would implement keyframe culling
        return ESP_ERR_NO_MEM;
    }
    
    keyframe_t* new_kf = &g_slam_state.keyframes[g_slam_state.num_keyframes];
    new_kf->id = g_slam_state.num_keyframes;
    new_kf->pose = *pose;
    new_kf->timestamp_us = pose->timestamp_us;
    
    // Extract features for this keyframe
    orb_features_t features;
    esp_err_t ret = orb_extract_features(frame, &features);
    if (ret == ESP_OK) {
        new_kf->num_features = features.num_features;
        // In a full implementation, we would store the descriptors
    }
    
    g_slam_state.num_keyframes++;
    g_slam_state.stats.keyframes = g_slam_state.num_keyframes;
    
    ESP_LOGI(TAG, "Created keyframe %d with %d features", new_kf->id, new_kf->num_features);
    return ESP_OK;
}

static esp_err_t triangulate_new_points(void) {
    // Placeholder for triangulation logic
    // In a full implementation, this would create new 3D map points
    // from stereo feature matches between keyframes
    return ESP_OK;
}

static esp_err_t optimize_local_map(void) {
    // Placeholder for bundle adjustment
    // In a full implementation, this would optimize poses and map points
    // using techniques like Levenberg-Marquardt
    return ESP_OK;
}

// Missing function implementations
esp_err_t slam_core_init_camera(const camera_config_t* config) {
    if (!config) return ESP_ERR_INVALID_ARG;
    
    ESP_LOGI(TAG, "Initializing camera for SLAM");
    // Camera initialization would be handled by the main app
    // This is just a compatibility function
    return ESP_OK;
}

esp_err_t slam_core_get_frame(camera_frame_t* frame) {
    if (!frame) return ESP_ERR_INVALID_ARG;
    
    // Get frame from camera system
    uint8_t* buffer = NULL;
    size_t len = 0;
    esp_err_t ret = camera_capture(&buffer, &len);
    
    if (ret == ESP_OK && buffer && len > 0) {
        frame->data = buffer;
        frame->data_size = len;
        frame->width = CAMERA_WIDTH;  // From camera_init.c
        frame->height = CAMERA_HEIGHT;
        frame->format = CAMERA_FORMAT_RGB565;
        frame->timestamp_us = esp_timer_get_time();
        
        // Run auto-adjustment on the captured frame
        if (g_camera_config.auto_adjustment_enabled) {
            camera_auto_adjust(buffer, len);
        }
        
        return ESP_OK;
    }
    
    return ESP_ERR_NOT_FOUND;
}

esp_err_t slam_core_release_frame(camera_frame_t* frame) {
    if (!frame) return ESP_ERR_INVALID_ARG;
    
    // Camera system manages its own frame buffers
    // Just clear the frame structure
    frame->data = NULL;
    frame->data_size = 0;
    frame->width = 0;
    frame->height = 0;
    frame->timestamp_us = 0;
    
    return ESP_OK;
}

static void adjust_camera_settings(const camera_frame_t* frame) {
    // Skip if auto-adjustment is disabled
    if (!g_camera_config.auto_adjustment_enabled) {
        return;
    }
    
    // Calculate average brightness of the frame
    uint32_t total_brightness = 0;
    uint32_t pixel_count = frame->width * frame->height;
    
    if (frame->format == CAMERA_FORMAT_RGB565) {
        // For RGB565, sample pixels and calculate average brightness
        uint16_t* pixels = (uint16_t*)frame->data;
        uint32_t sample_step = pixel_count / 1000; // Sample every Nth pixel for performance
        if (sample_step == 0) sample_step = 1;
        
        for (uint32_t i = 0; i < pixel_count; i += sample_step) {
            uint16_t pixel = pixels[i];
            // Extract RGB components from RGB565
            uint8_t r = (pixel >> 11) & 0x1F;
            uint8_t g = (pixel >> 5) & 0x3F;
            uint8_t b = pixel & 0x1F;
            
            // Scale to 8-bit and calculate brightness (weighted average)
            uint8_t brightness = (uint8_t)(((r << 3) * 0.299f + (g << 2) * 0.587f + (b << 3) * 0.114f));
            total_brightness += brightness;
        }
        total_brightness /= (pixel_count / sample_step);
    } else {
        // For other formats, assume grayscale or use simplified calculation
        uint8_t* pixels = (uint8_t*)frame->data;
        for (uint32_t i = 0; i < pixel_count; i++) {
            total_brightness += pixels[i];
        }
        total_brightness /= pixel_count;
    }
    
    float current_brightness = (float)total_brightness;
    float target_brightness = g_camera_config.target_brightness;
    float brightness_diff = target_brightness - current_brightness;
    
    // Only adjust if difference is significant
    if (fabsf(brightness_diff) > g_camera_config.brightness_threshold) {
        // Calculate adjustment amount
        float adjustment = brightness_diff * g_camera_config.adjustment_speed;
        
        // Update brightness setting (clamp to valid range)
        int new_brightness = g_camera_config.brightness + (int)adjustment;
        if (new_brightness < -100) new_brightness = -100;
        if (new_brightness > 100) new_brightness = 100;
        
        // Update camera config if change is significant
        if (abs(new_brightness - g_camera_config.brightness) >= 5) {
            g_camera_config.brightness = new_brightness;
            ESP_LOGD(TAG, "Auto-adjusted brightness: %d (current: %.1f, target: %.1f)", 
                     new_brightness, current_brightness, target_brightness);
            
            // Apply brightness adjustment to camera hardware
            camera_set_brightness(new_brightness);
        }
        
        // Also adjust exposure if auto-exposure is enabled
        if (g_camera_config.auto_exposure) {
            int exposure_adjustment = (int)(adjustment * 0.5f); // Smaller exposure changes
            int new_exposure = g_camera_config.exposure_value + exposure_adjustment;
            if (new_exposure < 0) new_exposure = 0;
            if (new_exposure > 1000) new_exposure = 1000;
            
            if (abs(new_exposure - g_camera_config.exposure_value) >= 10) {
                g_camera_config.exposure_value = new_exposure;
                ESP_LOGD(TAG, "Auto-adjusted exposure: %d", new_exposure);
                
                // Apply exposure adjustment to camera hardware
                camera_set_exposure_mode("Manual"); // Switch to manual for custom exposure
                // Note: camera_set_exposure_mode doesn't handle custom values, 
                // this would need sensor-specific IOCTL calls
            }
        }
    }
}

esp_err_t slam_core_set_camera_config(const camera_config_t* config) {
    if (!config) {
        return ESP_ERR_INVALID_ARG;
    }
    
    xSemaphoreTake(g_slam_state.slam_mutex, portMAX_DELAY);
    g_camera_config = *config;
    xSemaphoreGive(g_slam_state.slam_mutex);
    
    ESP_LOGI(TAG, "Camera config updated - Brightness: %d, Contrast: %d, Auto-adjust: %s", 
             config->brightness, config->contrast, 
             config->auto_adjustment_enabled ? "enabled" : "disabled");
    
    return ESP_OK;
}

esp_err_t slam_core_get_camera_config(camera_config_t* config) {
    if (!config) {
        return ESP_ERR_INVALID_ARG;
    }
    
    xSemaphoreTake(g_slam_state.slam_mutex, portMAX_DELAY);
    *config = g_camera_config;
    xSemaphoreGive(g_slam_state.slam_mutex);
    
    return ESP_OK;
}
