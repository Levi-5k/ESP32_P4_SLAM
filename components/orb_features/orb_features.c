/*
 * ORB Features Implementation
 * FAST corner detection with BRIEF descriptors
 */

#include "orb_features.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <string.h>
#include <math.h>

static const char* TAG = "orb_features";

// Global ORB state
static struct {
    orb_config_t config;
    orb_performance_stats_t stats;
    SemaphoreHandle_t orb_mutex;
    bool initialized;
    
    // Temporary buffers for processing
    uint8_t* gray_buffer;               // Grayscale conversion buffer
    uint8_t* pyramid_buffers[8];        // Image pyramid levels
    uint16_t pyramid_widths[8];         // Width of each pyramid level
    uint16_t pyramid_heights[8];        // Height of each pyramid level
    
    // FAST corner detection thresholds for each level
    uint8_t fast_thresholds[8];
    
    // Pre-computed rotation patterns for descriptor
    int16_t* rotation_patterns;         // 256 rotation patterns
    
    // Previous frame features for matching
    orb_features_t* previous_features;  // Features from previous frame
    orb_matches_t* current_matches;     // Current frame matches
    bool has_previous_frame;            // Flag if previous frame exists
    
} g_orb_state = {0};

// Default ORB configuration
static const orb_config_t default_config = {
    .max_features = 500,
    .scale_factor = 1.2f,
    .num_levels = 8,
    .edge_threshold = 31,
    .first_level = 0,
    .wta_k = 2,
    .patch_size = 31
};

// FAST corner detection circle offsets (16 points)
static const int16_t fast_circle_16[][2] = {
    {0, 3}, {1, 3}, {2, 2}, {3, 1}, {3, 0}, {3, -1}, {2, -2}, {1, -3},
    {0, -3}, {-1, -3}, {-2, -2}, {-3, -1}, {-3, 0}, {-3, 1}, {-2, 2}, {-1, 3}
};

// BRIEF descriptor pattern (256 pairs of points)
static const int8_t brief_pattern_256[][4] = {
    // Simplified pattern - in practice this would be 256 carefully chosen pairs
    {-8, -3, 6, 2}, {4, -1, -2, 5}, {-3, 7, 1, -4}, {2, -8, 5, 3},
    {-6, 1, 8, -2}, {3, 4, -7, 0}, {1, -5, -1, 6}, {-4, 8, 2, -3},
    // ... (would continue for 256 pairs)
};

// Forward declarations
static esp_err_t initialize_buffers(uint16_t max_width, uint16_t max_height);
static void cleanup_buffers(void);
static esp_err_t rgb565_to_grayscale(const camera_frame_t* frame, uint8_t* gray_buffer);
static esp_err_t build_image_pyramid(const uint8_t* image, uint16_t width, uint16_t height);
static esp_err_t detect_fast_corners(const uint8_t* image, uint16_t width, uint16_t height, 
                                   uint8_t threshold, orb_feature_point_t* corners, uint32_t* num_corners);
static esp_err_t compute_orb_descriptors(const uint8_t* image, uint16_t width, uint16_t height,
                                       orb_feature_point_t* features, uint32_t num_features);
static float compute_orientation(const uint8_t* image, uint16_t width, uint16_t height, 
                               uint16_t x, uint16_t y, uint8_t patch_size);

esp_err_t orb_features_init(uint32_t max_features) {
    if (g_orb_state.initialized) {
        ESP_LOGW(TAG, "ORB features already initialized");
        return ESP_OK;
    }
    
    // Set configuration
    g_orb_state.config = default_config;
    g_orb_state.config.max_features = max_features;
    
    // Create mutex
    g_orb_state.orb_mutex = xSemaphoreCreateMutex();
    if (!g_orb_state.orb_mutex) {
        ESP_LOGE(TAG, "Failed to create ORB mutex");
        return ESP_ERR_NO_MEM;
    }
    
    // Initialize buffers for typical camera resolution
    esp_err_t ret = initialize_buffers(800, 640);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize buffers: %s", esp_err_to_name(ret));
        vSemaphoreDelete(g_orb_state.orb_mutex);
        return ret;
    }
    
    // Initialize previous frame features buffer
    g_orb_state.previous_features = heap_caps_malloc(sizeof(orb_features_t), MALLOC_CAP_8BIT);
    if (!g_orb_state.previous_features) {
        ESP_LOGE(TAG, "Failed to allocate previous features buffer");
        cleanup_buffers();
        vSemaphoreDelete(g_orb_state.orb_mutex);
        return ESP_ERR_NO_MEM;
    }
    memset(g_orb_state.previous_features, 0, sizeof(orb_features_t));
    
    // Allocate features array for previous frame
    ret = orb_alloc_features(g_orb_state.previous_features, max_features);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to allocate previous features array");
        heap_caps_free(g_orb_state.previous_features);
        cleanup_buffers();
        vSemaphoreDelete(g_orb_state.orb_mutex);
        return ret;
    }
    
    // Initialize matches buffer
    g_orb_state.current_matches = heap_caps_malloc(sizeof(orb_matches_t), MALLOC_CAP_8BIT);
    if (!g_orb_state.current_matches) {
        ESP_LOGE(TAG, "Failed to allocate matches buffer");
        orb_free_features(g_orb_state.previous_features);
        heap_caps_free(g_orb_state.previous_features);
        cleanup_buffers();
        vSemaphoreDelete(g_orb_state.orb_mutex);
        return ESP_ERR_NO_MEM;
    }
    memset(g_orb_state.current_matches, 0, sizeof(orb_matches_t));
    
    g_orb_state.has_previous_frame = false;
    
    // Initialize performance stats
    memset(&g_orb_state.stats, 0, sizeof(orb_performance_stats_t));
    
    g_orb_state.initialized = true;
    ESP_LOGI(TAG, "ORB features initialized with %d max features", max_features);
    
    return ESP_OK;
}

esp_err_t orb_features_deinit(void) {
    if (!g_orb_state.initialized) {
        return ESP_OK;
    }
    
    xSemaphoreTake(g_orb_state.orb_mutex, portMAX_DELAY);
    
    // Cleanup matching buffers
    if (g_orb_state.current_matches) {
        if (g_orb_state.current_matches->matches) {
            heap_caps_free(g_orb_state.current_matches->matches);
        }
        heap_caps_free(g_orb_state.current_matches);
        g_orb_state.current_matches = NULL;
    }
    
    if (g_orb_state.previous_features) {
        orb_free_features(g_orb_state.previous_features);
        heap_caps_free(g_orb_state.previous_features);
        g_orb_state.previous_features = NULL;
    }
    
    cleanup_buffers();
    g_orb_state.has_previous_frame = false;
    g_orb_state.initialized = false;
    
    xSemaphoreGive(g_orb_state.orb_mutex);
    vSemaphoreDelete(g_orb_state.orb_mutex);
    
    ESP_LOGI(TAG, "ORB features deinitialized");
    return ESP_OK;
}

esp_err_t orb_extract_features(const camera_frame_t* frame, orb_features_t* features) {
    if (!g_orb_state.initialized || !frame || !features) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Check format using macro constant
    if (frame->format != CAMERA_FORMAT_RGB565) {
        ESP_LOGE(TAG, "Only RGB565 format supported for ORB extraction");
        return ESP_ERR_NOT_SUPPORTED;
    }
    
    xSemaphoreTake(g_orb_state.orb_mutex, portMAX_DELAY);
    
    uint64_t start_time = esp_timer_get_time();
    esp_err_t ret = ESP_OK;
    
    // Convert to grayscale
    ret = rgb565_to_grayscale(frame, g_orb_state.gray_buffer);
    if (ret != ESP_OK) {
        goto cleanup;
    }
    
    // Build image pyramid
    ret = build_image_pyramid(g_orb_state.gray_buffer, frame->width, frame->height);
    if (ret != ESP_OK) {
        goto cleanup;
    }
    
    uint64_t detection_start = esp_timer_get_time();
    
    // Detect features on each pyramid level
    uint32_t total_features = 0;
    uint32_t features_per_level = g_orb_state.config.max_features / g_orb_state.config.num_levels;
    
    for (uint8_t level = 0; level < g_orb_state.config.num_levels && total_features < features->max_features; level++) {
        uint32_t level_features = 0;
        uint32_t max_level_features = MIN(features_per_level, features->max_features - total_features);
        
        ret = detect_fast_corners(g_orb_state.pyramid_buffers[level],
                                g_orb_state.pyramid_widths[level],
                                g_orb_state.pyramid_heights[level],
                                g_orb_state.fast_thresholds[level],
                                &features->features[total_features],
                                &level_features);
        
        if (ret != ESP_OK) {
            goto cleanup;
        }
        
        // Limit features to max_level_features to prevent overflow
        if (level_features > max_level_features) {
            level_features = max_level_features;
        }
        
        // Scale coordinates back to original image size
        float scale = powf(g_orb_state.config.scale_factor, level);
        for (uint32_t i = 0; i < level_features; i++) {
            features->features[total_features + i].x = (uint16_t)(features->features[total_features + i].x * scale);
            features->features[total_features + i].y = (uint16_t)(features->features[total_features + i].y * scale);
            features->features[total_features + i].octave = level;
        }
        
        total_features += level_features;
    }
    
    uint64_t detection_time = esp_timer_get_time() - detection_start;
    uint64_t description_start = esp_timer_get_time();
    
    // Compute descriptors for all detected features
    if (total_features > 0) {
        ret = compute_orb_descriptors(g_orb_state.gray_buffer, frame->width, frame->height,
                                    features->features, total_features);
        if (ret != ESP_OK) {
            goto cleanup;
        }
    }
    
    uint64_t description_time = esp_timer_get_time() - description_start;
    uint64_t total_time = esp_timer_get_time() - start_time;
    
    // Update feature structure
    features->num_features = total_features;
    features->count = total_features;  // Alias for compatibility
    features->timestamp_us = esp_timer_get_time();
    features->frame_width = frame->width;
    features->frame_height = frame->height;
    
    // Perform feature matching with previous frame if available
    features->num_matches = 0;
    features->num_good_matches = 0;
    
    if (g_orb_state.has_previous_frame && g_orb_state.previous_features && 
        g_orb_state.current_matches && total_features > 0) {
        
        uint64_t matching_start = esp_timer_get_time();
        esp_err_t match_ret = orb_match_features(g_orb_state.previous_features, features, 
                                                g_orb_state.current_matches, 0.7f);
        
        if (match_ret == ESP_OK) {
            features->num_matches = g_orb_state.current_matches->num_matches;
            features->num_good_matches = g_orb_state.current_matches->num_good_matches;
            
            uint64_t matching_time = esp_timer_get_time() - matching_start;
            ESP_LOGD(TAG, "Feature matching: %lu matches (%lu good) in %.1fms", 
                     features->num_matches, features->num_good_matches, 
                     matching_time / 1000.0f);
        }
    }
    
    // Store current features as previous for next frame
    if (g_orb_state.previous_features) {
        // Copy current features to previous features buffer
        memcpy(g_orb_state.previous_features, features, sizeof(orb_features_t));
        if (g_orb_state.previous_features->features && features->features && total_features > 0) {
            memcpy(g_orb_state.previous_features->features, features->features, 
                   total_features * sizeof(orb_feature_point_t));
        }
        g_orb_state.previous_features->num_features = total_features;
        g_orb_state.previous_features->count = total_features;
    }
    g_orb_state.has_previous_frame = true;
    
    // Update performance statistics
    g_orb_state.stats.frames_processed++;
    g_orb_state.stats.total_features_detected += total_features;
    
    float alpha = 0.1f;  // Exponential moving average
    g_orb_state.stats.average_detection_time_ms = 
        (1.0f - alpha) * g_orb_state.stats.average_detection_time_ms + 
        alpha * (detection_time / 1000.0f);
    
    g_orb_state.stats.average_description_time_ms = 
        (1.0f - alpha) * g_orb_state.stats.average_description_time_ms + 
        alpha * (description_time / 1000.0f);
    
    g_orb_state.stats.average_features_per_frame = 
        g_orb_state.stats.total_features_detected / g_orb_state.stats.frames_processed;
    
    ESP_LOGD(TAG, "Extracted %d features in %.1fms (det: %.1fms, desc: %.1fms)", 
             total_features, total_time / 1000.0f, detection_time / 1000.0f, description_time / 1000.0f);

cleanup:
    xSemaphoreGive(g_orb_state.orb_mutex);
    return ret;
}

esp_err_t orb_alloc_features(orb_features_t* features, uint32_t max_features) {
    if (!features) {
        return ESP_ERR_INVALID_ARG;
    }
    
    features->features = heap_caps_calloc(max_features, sizeof(orb_feature_point_t), MALLOC_CAP_SPIRAM);
    if (!features->features) {
        ESP_LOGE(TAG, "Failed to allocate memory for %d features", max_features);
        return ESP_ERR_NO_MEM;
    }
    
    features->max_features = max_features;
    features->num_features = 0;
    
    return ESP_OK;
}

void orb_free_features(orb_features_t* features) {
    if (features && features->features) {
        free(features->features);
        features->features = NULL;
        features->max_features = 0;
        features->num_features = 0;
    }
}

uint32_t orb_hamming_distance(const uint8_t* desc1, const uint8_t* desc2) {
    uint32_t distance = 0;
    
    // Count differing bits in 32-byte descriptors
    for (int i = 0; i < 32; i++) {
        uint8_t xor_result = desc1[i] ^ desc2[i];
        // Count set bits using Brian Kernighan's algorithm
        while (xor_result) {
            distance++;
            xor_result &= xor_result - 1;
        }
    }
    
    return distance;
}

esp_err_t orb_get_performance_stats(orb_performance_stats_t* stats) {
    if (!g_orb_state.initialized || !stats) {
        return ESP_ERR_INVALID_ARG;
    }
    
    xSemaphoreTake(g_orb_state.orb_mutex, portMAX_DELAY);
    *stats = g_orb_state.stats;
    xSemaphoreGive(g_orb_state.orb_mutex);
    
    return ESP_OK;
}

// Private helper functions

static esp_err_t initialize_buffers(uint16_t max_width, uint16_t max_height) {
    // Allocate grayscale buffer
    g_orb_state.gray_buffer = heap_caps_malloc(max_width * max_height, MALLOC_CAP_SPIRAM);
    if (!g_orb_state.gray_buffer) {
        ESP_LOGE(TAG, "Failed to allocate grayscale buffer");
        return ESP_ERR_NO_MEM;
    }
    
    // Allocate pyramid buffers
    uint16_t current_width = max_width;
    uint16_t current_height = max_height;
    
    for (int level = 0; level < g_orb_state.config.num_levels; level++) {
        g_orb_state.pyramid_widths[level] = current_width;
        g_orb_state.pyramid_heights[level] = current_height;
        
        size_t buffer_size = current_width * current_height;
        g_orb_state.pyramid_buffers[level] = heap_caps_malloc(buffer_size, MALLOC_CAP_SPIRAM);
        if (!g_orb_state.pyramid_buffers[level]) {
            ESP_LOGE(TAG, "Failed to allocate pyramid buffer for level %d", level);
            cleanup_buffers();
            return ESP_ERR_NO_MEM;
        }
        
        // Set FAST threshold for this level
        g_orb_state.fast_thresholds[level] = g_orb_state.config.edge_threshold;
        
        // Scale down for next level
        current_width = (uint16_t)(current_width / g_orb_state.config.scale_factor);
        current_height = (uint16_t)(current_height / g_orb_state.config.scale_factor);
        
        if (current_width < 32 || current_height < 32) {
            // Stop if image gets too small
            g_orb_state.config.num_levels = level + 1;
            break;
        }
    }
    
    ESP_LOGI(TAG, "Initialized %d pyramid levels, max size: %dx%d", 
             g_orb_state.config.num_levels, max_width, max_height);
    
    return ESP_OK;
}

static void cleanup_buffers(void) {
    if (g_orb_state.gray_buffer) {
        free(g_orb_state.gray_buffer);
        g_orb_state.gray_buffer = NULL;
    }
    
    for (int level = 0; level < 8; level++) {
        if (g_orb_state.pyramid_buffers[level]) {
            free(g_orb_state.pyramid_buffers[level]);
            g_orb_state.pyramid_buffers[level] = NULL;
        }
    }
}

static esp_err_t rgb565_to_grayscale(const camera_frame_t* frame, uint8_t* gray_buffer) {
    uint16_t* rgb565_data = (uint16_t*)frame->data;
    uint32_t num_pixels = frame->width * frame->height;
    
    for (uint32_t i = 0; i < num_pixels; i++) {
        uint16_t pixel = rgb565_data[i];
        
        // Extract RGB components from RGB565
        uint8_t r = (pixel >> 11) & 0x1F;
        uint8_t g = (pixel >> 5) & 0x3F;
        uint8_t b = pixel & 0x1F;
        
        // Scale to 8-bit
        r = (r * 255) / 31;
        g = (g * 255) / 63;
        b = (b * 255) / 31;
        
        // Convert to grayscale using ITU-R BT.709 coefficients
        gray_buffer[i] = (uint8_t)(0.299f * r + 0.587f * g + 0.114f * b);
    }
    
    return ESP_OK;
}

static esp_err_t build_image_pyramid(const uint8_t* image, uint16_t width, uint16_t height) {
    // Copy original image to level 0
    memcpy(g_orb_state.pyramid_buffers[0], image, width * height);
    
    // Build subsequent levels by downsampling
    for (int level = 1; level < g_orb_state.config.num_levels; level++) {
        uint16_t src_width = g_orb_state.pyramid_widths[level - 1];
        uint16_t src_height = g_orb_state.pyramid_heights[level - 1];
        uint16_t dst_width = g_orb_state.pyramid_widths[level];
        uint16_t dst_height = g_orb_state.pyramid_heights[level];
        
        uint8_t* src = g_orb_state.pyramid_buffers[level - 1];
        uint8_t* dst = g_orb_state.pyramid_buffers[level];
        
        // Simple downsampling by averaging 2x2 blocks
        for (uint16_t y = 0; y < dst_height; y++) {
            for (uint16_t x = 0; x < dst_width; x++) {
                uint16_t src_x = x * 2;
                uint16_t src_y = y * 2;
                
                // Average 2x2 block (with bounds checking)
                uint32_t sum = 0;
                uint32_t count = 0;
                
                for (int dy = 0; dy < 2 && src_y + dy < src_height; dy++) {
                    for (int dx = 0; dx < 2 && src_x + dx < src_width; dx++) {
                        sum += src[(src_y + dy) * src_width + (src_x + dx)];
                        count++;
                    }
                }
                
                dst[y * dst_width + x] = (uint8_t)(sum / count);
            }
        }
    }
    
    return ESP_OK;
}

static esp_err_t detect_fast_corners(const uint8_t* image, uint16_t width, uint16_t height, 
                                   uint8_t threshold, orb_feature_point_t* corners, uint32_t* num_corners) {
    *num_corners = 0;
    uint32_t max_corners = g_orb_state.config.max_features;
    
    // Simple FAST-like corner detection
    // In a full implementation, this would be optimized FAST-9 or FAST-12
    for (uint16_t y = 3; y < height - 3 && *num_corners < max_corners; y++) {
        for (uint16_t x = 3; x < width - 3 && *num_corners < max_corners; x++) {
            uint8_t center = image[y * width + x];
            
            // Check circle of 16 pixels
            int brighter = 0, darker = 0;
            int consecutive_brighter = 0, consecutive_darker = 0;
            int max_consecutive_brighter = 0, max_consecutive_darker = 0;
            
            for (int i = 0; i < 16; i++) {
                int px = x + fast_circle_16[i][0];
                int py = y + fast_circle_16[i][1];
                uint8_t pixel = image[py * width + px];
                
                if (pixel > center + threshold) {
                    brighter++;
                    consecutive_brighter++;
                    consecutive_darker = 0;
                    max_consecutive_brighter = MAX(max_consecutive_brighter, consecutive_brighter);
                } else if (pixel < center - threshold) {
                    darker++;
                    consecutive_darker++;
                    consecutive_brighter = 0;
                    max_consecutive_darker = MAX(max_consecutive_darker, consecutive_darker);
                } else {
                    consecutive_brighter = 0;
                    consecutive_darker = 0;
                }
            }
            
            // Check for wraparound consecutive pixels
            for (int i = 0; i < 9; i++) {
                int px = x + fast_circle_16[i][0];
                int py = y + fast_circle_16[i][1];
                uint8_t pixel = image[py * width + px];
                
                if (pixel > center + threshold) {
                    consecutive_brighter++;
                    max_consecutive_brighter = MAX(max_consecutive_brighter, consecutive_brighter);
                } else if (pixel < center - threshold) {
                    consecutive_darker++;
                    max_consecutive_darker = MAX(max_consecutive_darker, consecutive_darker);
                } else {
                    break;
                }
            }
            
            // Corner detected if 9 consecutive pixels are brighter or darker
            if (max_consecutive_brighter >= 9 || max_consecutive_darker >= 9) {
                corners[*num_corners].x = x;
                corners[*num_corners].y = y;
                corners[*num_corners].response = MAX(brighter, darker);
                corners[*num_corners].angle = 0;  // Will be computed later
                (*num_corners)++;
            }
        }
    }
    
    return ESP_OK;
}

static esp_err_t compute_orb_descriptors(const uint8_t* image, uint16_t width, uint16_t height,
                                       orb_feature_point_t* features, uint32_t num_features) {
    for (uint32_t i = 0; i < num_features; i++) {
        orb_feature_point_t* feature = &features[i];
        
        // Compute orientation
        feature->angle = compute_orientation(image, width, height, feature->x, feature->y, 
                                           g_orb_state.config.patch_size);
        
        // Compute BRIEF descriptor with rotation
        memset(feature->descriptor, 0, 32);
        
        // Simplified BRIEF computation (32 bytes = 256 bits)
        for (int byte_idx = 0; byte_idx < 32; byte_idx++) {
            uint8_t descriptor_byte = 0;
            
            for (int bit_idx = 0; bit_idx < 8; bit_idx++) {
                int pattern_idx = byte_idx * 8 + bit_idx;
                if (pattern_idx >= sizeof(brief_pattern_256) / sizeof(brief_pattern_256[0])) {
                    break;
                }
                
                // Get pattern points and rotate them
                float cos_angle = cosf(feature->angle);
                float sin_angle = sinf(feature->angle);
                
                int x1 = brief_pattern_256[pattern_idx][0];
                int y1 = brief_pattern_256[pattern_idx][1];
                int x2 = brief_pattern_256[pattern_idx][2];
                int y2 = brief_pattern_256[pattern_idx][3];
                
                // Rotate points
                int rx1 = (int)(x1 * cos_angle - y1 * sin_angle);
                int ry1 = (int)(x1 * sin_angle + y1 * cos_angle);
                int rx2 = (int)(x2 * cos_angle - y2 * sin_angle);
                int ry2 = (int)(x2 * sin_angle + y2 * cos_angle);
                
                // Sample pixels
                int px1 = feature->x + rx1;
                int py1 = feature->y + ry1;
                int px2 = feature->x + rx2;
                int py2 = feature->y + ry2;
                
                // Bounds check
                if (px1 >= 0 && px1 < width && py1 >= 0 && py1 < height &&
                    px2 >= 0 && px2 < width && py2 >= 0 && py2 < height) {
                    
                    uint8_t pixel1 = image[py1 * width + px1];
                    uint8_t pixel2 = image[py2 * width + px2];
                    
                    if (pixel1 < pixel2) {
                        descriptor_byte |= (1 << bit_idx);
                    }
                }
            }
            
            feature->descriptor[byte_idx] = descriptor_byte;
        }
    }
    
    return ESP_OK;
}

static float compute_orientation(const uint8_t* image, uint16_t width, uint16_t height, 
                               uint16_t x, uint16_t y, uint8_t patch_size) {
    int half_patch = patch_size / 2;
    float m01 = 0, m10 = 0;
    
    // Compute intensity centroid
    for (int v = -half_patch; v <= half_patch; v++) {
        for (int u = -half_patch; u <= half_patch; u++) {
            int px = x + u;
            int py = y + v;
            
            if (px >= 0 && px < width && py >= 0 && py < height) {
                uint8_t intensity = image[py * width + px];
                m01 += v * intensity;
                m10 += u * intensity;
            }
        }
    }
    
    return atan2f(m01, m10);
}

// Compatibility function - alias for orb_extract_features
esp_err_t orb_features_extract(const camera_frame_t* frame, orb_features_t* features) {
    return orb_extract_features(frame, features);
}

// Match features between two frames
esp_err_t orb_match_features(const orb_features_t* query_features, 
                            const orb_features_t* train_features,
                            orb_matches_t* matches,
                            float distance_threshold) {
    if (!query_features || !train_features || !matches) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (query_features->num_features == 0 || train_features->num_features == 0) {
        matches->num_matches = 0;
        matches->num_good_matches = 0;
        matches->average_distance = 0.0f;
        return ESP_OK;
    }
    
    uint32_t max_possible_matches = MIN(query_features->num_features, train_features->num_features);
    
    // Allocate match buffer if needed
    if (!matches->matches) {
        matches->matches = heap_caps_malloc(max_possible_matches * sizeof(orb_match_t), 
                                           MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!matches->matches) {
            return ESP_ERR_NO_MEM;
        }
    }
    
    uint32_t match_count = 0;
    uint32_t good_match_count = 0;
    float total_distance = 0.0f;
    
    // For each query feature, find best match in train features
    for (uint32_t q = 0; q < query_features->num_features; q++) {
        uint32_t best_distance = 256;  // Maximum Hamming distance
        uint16_t best_train_idx = 0;
        
        // Search all train features for best match
        for (uint32_t t = 0; t < train_features->num_features; t++) {
            uint32_t distance = orb_hamming_distance(
                query_features->features[q].descriptor,
                train_features->features[t].descriptor
            );
            
            if (distance < best_distance) {
                best_distance = distance;
                best_train_idx = t;
            }
        }
        
        // Apply distance threshold and ratio test
        bool is_good_match = (best_distance <= (uint32_t)(distance_threshold * 256.0f));
        
        // Store the match
        if (match_count < max_possible_matches) {
            matches->matches[match_count].query_idx = q;
            matches->matches[match_count].train_idx = best_train_idx;
            matches->matches[match_count].distance = best_distance / 256.0f;
            matches->matches[match_count].is_good_match = is_good_match;
            
            total_distance += matches->matches[match_count].distance;
            match_count++;
            
            if (is_good_match) {
                good_match_count++;
            }
        }
    }
    
    // Update match statistics
    matches->num_matches = match_count;
    matches->num_good_matches = good_match_count;
    matches->average_distance = (match_count > 0) ? (total_distance / match_count) : 0.0f;
    
    ESP_LOGD(TAG, "Feature matching: %lu total matches, %lu good matches (%.1f%%), avg distance: %.3f",
             match_count, good_match_count, 
             (match_count > 0) ? (100.0f * good_match_count / match_count) : 0.0f,
             matches->average_distance);
    
    return ESP_OK;
}
