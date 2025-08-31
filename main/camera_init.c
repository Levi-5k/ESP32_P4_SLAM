/*
 * Camera initialization for ESP32-P4-WIFI6 wi// Camera controller callback functions for continuous capture
static bool IRAM_ATTR s_camera_get_new_vb(esp_cam_ctlr_handle_t handle, esp_cam_ctlr_trans_t *trans, void *user_data)
{
    esp_cam_ctlr_trans_t *new_trans = (esp_cam_ctlr_trans_t *)user_data;
    trans->buffer = new_trans->buffer;
    trans->buflen = new_trans->buflen;
    
    ESP_DRAM_LOGI(TAG, "s_camera_get_new_vb called - providing buffer for continuous capture");
    return false;  // Return false to continue using the same buffer for continuous capture
}

static bool IRAM_ATTR s_camera_get_finished_trans(esp_cam_ctlr_handle_t handle, esp_cam_ctlr_trans_t *trans, void *user_data)
{
    s_finished_trans_counter++;
    ESP_DRAM_LOGI(TAG, "s_camera_get_finished_trans: frame %lu captured", s_finished_trans_counter);
    return false;  // Return false to automatically start the next capture (continuous mode)
}I
 * Based on official ESP-IDF mipi_isp_dsi example and GitHub issue #15409
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_cam_ctlr_csi.h"
#include "esp_cam_ctlr.h"
#include "driver/i2c_master.h"
#include "driver/isp.h"
#include "esp_ldo_regulator.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "esp_cache.h"
#include "example_sensor_init.h"
#include "camera_init.h"

static const char *TAG = "camera_init";

// OV5647 MIPI-CSI Configuration for ESP32-P4-WIFI6 (based on working examples)
#define CAMERA_I2C_SCL_IO           8      // I2C SCL pin for OV5647
#define CAMERA_I2C_SDA_IO           7      // I2C SDA pin for OV5647  
#define CAMERA_LANE_BITRATE_MBPS    200    // MIPI lane bitrate
#define CAMERA_DATA_LANES           2      // OV5647 uses 2 data lanes

// Camera resolution settings (using standard OV5647 format)
#define CAMERA_WIDTH               800     // OV5647 standard width
#define CAMERA_HEIGHT              640     // OV5647 standard height

// MIPI LDO configuration for ESP32-P4 camera power
#define MIPI_LDO_CHANNEL_ID         3      // LDO channel 3 for MIPI PHY
#define MIPI_LDO_VOLTAGE_MV         2500   // 2.5V for MIPI DPHY

// RGB565 frame configuration
#define RGB565_BITS_PER_PIXEL       16

static esp_cam_ctlr_handle_t s_cam_handle = NULL;
static example_sensor_handle_t s_sensor_handle = {0};
static esp_ldo_channel_handle_t s_ldo_mipi_phy = NULL;
static isp_proc_handle_t s_isp_proc = NULL;
static void *s_frame_buffer = NULL;
static size_t s_frame_buffer_size = 0;
static size_t s_frame_size = 0;  // Size of the current captured frame
static uint32_t s_finished_trans_counter = 0;

// Continuous capture management
static void *s_frame_buffer_1 = NULL;
static void *s_frame_buffer_2 = NULL;
static esp_cam_ctlr_trans_t s_new_trans_1;
static esp_cam_ctlr_trans_t s_new_trans_2;
static volatile bool s_continuous_capture_running = false;
static volatile void *s_latest_frame_buffer = NULL;  // Pointer to latest captured frame
static volatile size_t s_latest_frame_size = 0;     // Size of latest captured frame

// Global ISP color state to preserve all settings
static esp_isp_color_config_t s_current_color_config = {
    .color_contrast = {
        .integer = 1,
        .decimal = 0,
    },
    .color_saturation = {
        .integer = 1,
        .decimal = 0,
    },
    .color_hue = 0,
    .color_brightness = 0,
};

// Camera controller callback functions (based on ESP-IDF examples)
static bool IRAM_ATTR s_camera_get_new_vb(esp_cam_ctlr_handle_t handle, esp_cam_ctlr_trans_t *trans, void *user_data)
{
    static bool use_buffer_1 = true;
    
    if (s_continuous_capture_running) {
        if (use_buffer_1 && s_frame_buffer_1) {
            trans->buffer = s_frame_buffer_1;
            trans->buflen = s_frame_buffer_size;
            use_buffer_1 = false;
        } else if (s_frame_buffer_2) {
            trans->buffer = s_frame_buffer_2;
            trans->buflen = s_frame_buffer_size;
            use_buffer_1 = true;
        } else {
            trans->buffer = s_frame_buffer;
            trans->buflen = s_frame_buffer_size;
        }
        // Disable logging completely to prevent watchdog timeout
        // if (s_finished_trans_counter % 1000 == 0) {
        //     ESP_DRAM_LOGI(TAG, "s_camera_get_new_vb: assigned buffer %p (frame %lu)", trans->buffer, s_finished_trans_counter);
        // }
        return true;
    }
    
    // Fallback to main buffer
    trans->buffer = s_frame_buffer;
    trans->buflen = s_frame_buffer_size;
    return true;
}

static bool IRAM_ATTR s_camera_get_finished_trans(esp_cam_ctlr_handle_t handle, esp_cam_ctlr_trans_t *trans, void *user_data)
{
    s_finished_trans_counter++;
    
    // Instead of copying the entire frame, just update the pointer and size
    // This is much faster and won't cause watchdog timeouts
    s_latest_frame_buffer = trans->buffer;
    s_latest_frame_size = trans->buflen;
    s_frame_size = trans->buflen;
    
    // Return false to recycle buffer for continuous capture
    return false;
}

esp_err_t camera_init(void)
{
    esp_err_t ret = ESP_OK;
    
    ESP_LOGI(TAG, "=== ESP32-P4 OV5647 MIPI-CSI Camera Initialization ===");
    ESP_LOGI(TAG, "Free heap before camera init: %lu bytes", esp_get_free_heap_size());
    
    // Step 1: Configure MIPI LDO power for ESP32-P4 camera
    ESP_LOGI(TAG, "Configuring MIPI LDO power...");
    esp_ldo_channel_config_t ldo_mipi_phy_config = {
        .chan_id = MIPI_LDO_CHANNEL_ID,
        .voltage_mv = MIPI_LDO_VOLTAGE_MV,
    };
    ret = esp_ldo_acquire_channel(&ldo_mipi_phy_config, &s_ldo_mipi_phy);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to acquire MIPI LDO channel: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "✅ MIPI LDO configured: Channel %d at %dmV", MIPI_LDO_CHANNEL_ID, MIPI_LDO_VOLTAGE_MV);
    
    // Step 2: Calculate frame buffer size and allocate memory
    s_frame_buffer_size = CAMERA_WIDTH * CAMERA_HEIGHT * RGB565_BITS_PER_PIXEL / 8;
    ESP_LOGI(TAG, "Frame buffer size needed: %zu bytes (%.1f KB)", s_frame_buffer_size, s_frame_buffer_size / 1024.0);
    ESP_LOGI(TAG, "Resolution: %dx%d RGB565", CAMERA_WIDTH, CAMERA_HEIGHT);
    
    // Memory allocation with proper cache alignment (based on ESP-IDF examples)
    size_t frame_buffer_alignment = 64;  // Use standard cache line size for ESP32-P4
    ESP_LOGI(TAG, "Cache alignment: %zu bytes", frame_buffer_alignment);
    
    // Try PSRAM first, then internal RAM (following ESP-IDF test examples)
    s_frame_buffer = heap_caps_aligned_calloc(frame_buffer_alignment, 1, s_frame_buffer_size,
                                              MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (s_frame_buffer) {
        ESP_LOGI(TAG, "✅ Frame buffer allocated in PSRAM: %zu bytes at %p", s_frame_buffer_size, s_frame_buffer);
    } else {
        ESP_LOGE(TAG, "❌ Frame buffer allocation failed in PSRAM");
        esp_ldo_release_channel(s_ldo_mipi_phy);
        return ESP_ERR_NO_MEM;
    }
    
    // Allocate additional buffers for continuous capture ping-pong mechanism
    s_frame_buffer_1 = heap_caps_aligned_calloc(frame_buffer_alignment, 1, s_frame_buffer_size,
                                                MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    s_frame_buffer_2 = heap_caps_aligned_calloc(frame_buffer_alignment, 1, s_frame_buffer_size,
                                                MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    
    if (s_frame_buffer_1 && s_frame_buffer_2) {
        ESP_LOGI(TAG, "✅ Continuous capture buffers allocated: %p, %p", s_frame_buffer_1, s_frame_buffer_2);
    } else {
        ESP_LOGW(TAG, "⚠️ Continuous capture buffers allocation failed, using single buffer mode");
        if (s_frame_buffer_1) heap_caps_free(s_frame_buffer_1);
        if (s_frame_buffer_2) heap_caps_free(s_frame_buffer_2);
        s_frame_buffer_1 = NULL;
        s_frame_buffer_2 = NULL;
    }
    
    // Prepare transaction data structure
    esp_cam_ctlr_trans_t new_trans = {
        .buffer = s_frame_buffer,
        .buflen = s_frame_buffer_size,
    };
    
    // Step 3: Initialize camera sensor using ESP-IDF example_sensor_init
    ESP_LOGI(TAG, "Initializing OV5647 camera sensor...");
    example_sensor_config_t cam_sensor_config = {
        .i2c_port_num = I2C_NUM_0,
        .i2c_sda_io_num = CAMERA_I2C_SDA_IO,
        .i2c_scl_io_num = CAMERA_I2C_SCL_IO,
        .port = ESP_CAM_SENSOR_MIPI_CSI,
        .format_name = "MIPI_2lane_24Minput_RAW8_800x640_50fps",
    };
    
    example_sensor_init(&cam_sensor_config, &s_sensor_handle);
    if (s_sensor_handle.sccb_handle == NULL || s_sensor_handle.i2c_bus_handle == NULL) {
        ESP_LOGE(TAG, "❌ Camera sensor initialization failed");
        heap_caps_free(s_frame_buffer);
        esp_ldo_release_channel(s_ldo_mipi_phy);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "✅ OV5647 sensor initialized successfully");
    
    // Step 4: Initialize ISP (Image Signal Processor) BEFORE camera controller
    ESP_LOGI(TAG, "Initializing ISP processor...");
    esp_isp_processor_cfg_t isp_config = {
        .clk_hz = 80 * 1000 * 1000,
        .input_data_source = ISP_INPUT_DATA_SOURCE_CSI,
        .input_data_color_type = ISP_COLOR_RAW8,
        .output_data_color_type = ISP_COLOR_RGB565,
        .has_line_start_packet = false,
        .has_line_end_packet = false,
        .h_res = CAMERA_WIDTH,
        .v_res = CAMERA_HEIGHT,
    };
    ret = esp_isp_new_processor(&isp_config, &s_isp_proc);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create ISP processor: %s", esp_err_to_name(ret));
        example_sensor_deinit(s_sensor_handle);
        heap_caps_free(s_frame_buffer);
        esp_ldo_release_channel(s_ldo_mipi_phy);
        return ret;
    }
    
    ret = esp_isp_enable(s_isp_proc);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable ISP processor: %s", esp_err_to_name(ret));
        esp_isp_del_processor(s_isp_proc);
        example_sensor_deinit(s_sensor_handle);
        heap_caps_free(s_frame_buffer);
        esp_ldo_release_channel(s_ldo_mipi_phy);
        return ret;
    }
    ESP_LOGI(TAG, "✅ ISP processor initialized and enabled");
    
    // Step 4a: Initialize and enable ISP color controller
    ESP_LOGI(TAG, "Initializing ISP color controller...");
    ret = esp_isp_color_configure(s_isp_proc, &s_current_color_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure ISP color: %s", esp_err_to_name(ret));
        esp_isp_del_processor(s_isp_proc);
        example_sensor_deinit(s_sensor_handle);
        heap_caps_free(s_frame_buffer);
        esp_ldo_release_channel(s_ldo_mipi_phy);
        return ret;
    }
    
    ret = esp_isp_color_enable(s_isp_proc);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable ISP color: %s", esp_err_to_name(ret));
        esp_isp_del_processor(s_isp_proc);
        example_sensor_deinit(s_sensor_handle);
        heap_caps_free(s_frame_buffer);
        esp_ldo_release_channel(s_ldo_mipi_phy);
        return ret;
    }
    ESP_LOGI(TAG, "✅ ISP color controller enabled");
    
    // Step 5: Configure MIPI-CSI camera controller (connecting to ISP)
    ESP_LOGI(TAG, "Configuring CSI controller to connect with ISP...");
    esp_cam_ctlr_csi_config_t csi_config = {
        .ctlr_id = 0,
        .h_res = CAMERA_WIDTH,
        .v_res = CAMERA_HEIGHT,
        .lane_bit_rate_mbps = CAMERA_LANE_BITRATE_MBPS,
        .input_data_color_type = CAM_CTLR_COLOR_RAW8,
        .output_data_color_type = CAM_CTLR_COLOR_RAW8,  // Output RAW8 to ISP
        .data_lane_num = CAMERA_DATA_LANES,
        .byte_swap_en = false,
        .queue_items = 4,  // Increase queue size for continuous capture
    };
    
    ret = esp_cam_new_csi_ctlr(&csi_config, &s_cam_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create CSI controller: %s", esp_err_to_name(ret));
        esp_isp_del_processor(s_isp_proc);
        example_sensor_deinit(s_sensor_handle);
        heap_caps_free(s_frame_buffer);
        esp_ldo_release_channel(s_ldo_mipi_phy);
        return ret;
    }
    
    // Connect camera controller to ISP processor
    ret = esp_cam_ctlr_isp_bind(s_cam_handle, s_isp_proc);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to bind camera controller to ISP: %s", esp_err_to_name(ret));
        esp_cam_ctlr_del(s_cam_handle);
        esp_isp_del_processor(s_isp_proc);
        example_sensor_deinit(s_sensor_handle);
        heap_caps_free(s_frame_buffer);
        esp_ldo_release_channel(s_ldo_mipi_phy);
        return ret;
    }
    ESP_LOGI(TAG, "✅ Camera controller created and bound to ISP");
    
    // Step 6: Register camera event callbacks (critical for frame capture)
    esp_cam_ctlr_evt_cbs_t cbs = {
        .on_get_new_trans = s_camera_get_new_vb,
        .on_trans_finished = s_camera_get_finished_trans,
    };
    ret = esp_cam_ctlr_register_event_callbacks(s_cam_handle, &cbs, &new_trans);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register camera callbacks: %s", esp_err_to_name(ret));
        esp_cam_ctlr_del(s_cam_handle);
        example_sensor_deinit(s_sensor_handle);
        heap_caps_free(s_frame_buffer);
        esp_ldo_release_channel(s_ldo_mipi_phy);
        return ret;
    }
    ESP_LOGI(TAG, "✅ Camera callbacks registered");
    
    // Step 7: Enable camera controller
    ret = esp_cam_ctlr_enable(s_cam_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable camera controller: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "✅ Camera controller enabled");
    
    // Don't initialize frame buffer to white - let real camera data fill it
    // memset(s_frame_buffer, 0xFF, s_frame_buffer_size);
    
    // Cache sync for DMA (critical for ESP32-P4)
    esp_cache_msync(s_frame_buffer, s_frame_buffer_size, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
    
    ESP_LOGI(TAG, "🎉 Camera initialization completed successfully!");
    ESP_LOGI(TAG, "📷 OV5647 MIPI-CSI: %dx%d, %d lanes, %d Mbps", 
             CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_DATA_LANES, CAMERA_LANE_BITRATE_MBPS);
    ESP_LOGI(TAG, "💾 Frame buffer: %zu bytes @ %p", s_frame_buffer_size, s_frame_buffer);
    ESP_LOGI(TAG, "Free heap after init: %lu bytes", esp_get_free_heap_size());
    
    return ESP_OK;
}

esp_err_t camera_start(void)
{
    if (s_cam_handle == NULL) {
        ESP_LOGE(TAG, "Camera not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Starting camera capture...");
    
    // Start camera controller (based on working examples)
    esp_err_t ret = esp_cam_ctlr_start(s_cam_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start camera: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Start sensor streaming now that everything is initialized
    ESP_LOGI(TAG, "Starting sensor streaming...");
    if (s_sensor_handle.sensor_device) {
        int enable_stream = 1;
        esp_err_t stream_ret = esp_cam_sensor_ioctl(s_sensor_handle.sensor_device, ESP_CAM_SENSOR_IOC_S_STREAM, &enable_stream);
        if (stream_ret == ESP_OK) {
            ESP_LOGI(TAG, "✅ Sensor streaming enabled successfully");
            
            // Give sensor time to start producing data (critical for OV5647)
            vTaskDelay(100 / portTICK_PERIOD_MS);
            ESP_LOGI(TAG, "Sensor stabilization delay completed");
        } else {
            ESP_LOGE(TAG, "❌ Failed to enable sensor streaming: %s", esp_err_to_name(stream_ret));
            return stream_ret;
        }
    } else {
        ESP_LOGE(TAG, "❌ Sensor device handle is NULL");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Enable continuous capture mode
    s_continuous_capture_running = true;
    
    // Start with a single blocking receive to test if sensor is actually streaming
    esp_cam_ctlr_trans_t test_trans = {
        .buffer = s_frame_buffer,
        .buflen = s_frame_buffer_size,
    };
    
    ESP_LOGI(TAG, "Testing sensor data flow with blocking receive...");
    ret = esp_cam_ctlr_receive(s_cam_handle, &test_trans, 500 / portTICK_PERIOD_MS);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "✅ Sensor is streaming data - first frame received!");
        
        // Check if we got real data or just zeros
        uint16_t *pixel_data = (uint16_t *)test_trans.buffer;
        bool has_data = false;
        for (int i = 0; i < 100; i++) {
            if (pixel_data[i] != 0) {
                has_data = true;
                break;
            }
        }
        
        if (has_data) {
            ESP_LOGI(TAG, "✅ Frame contains real image data");
        } else {
            ESP_LOGW(TAG, "⚠️ Frame contains only zeros - sensor may need more time");
        }
        
        // Now start continuous capture with callback-driven ping-pong buffers
        if (s_frame_buffer_1 && s_frame_buffer_2) {
            s_new_trans_1.buffer = s_frame_buffer_1;
            s_new_trans_1.buflen = s_frame_buffer_size;
            s_new_trans_2.buffer = s_frame_buffer_2;
            s_new_trans_2.buflen = s_frame_buffer_size;
            
            // Queue transactions with no timeout (non-blocking)
            ret = esp_cam_ctlr_receive(s_cam_handle, &s_new_trans_1, 0);
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "✅ Continuous capture transaction 1 queued");
            } else {
                ESP_LOGW(TAG, "⚠️ Failed to queue transaction 1: %s", esp_err_to_name(ret));
            }
            
            ret = esp_cam_ctlr_receive(s_cam_handle, &s_new_trans_2, 0);
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "✅ Continuous capture transaction 2 queued");
            } else {
                ESP_LOGW(TAG, "⚠️ Failed to queue transaction 2: %s", esp_err_to_name(ret));
            }
        }
    } else {
        ESP_LOGE(TAG, "❌ Sensor is NOT streaming data: %s", esp_err_to_name(ret));
        ESP_LOGE(TAG, "This means the sensor is not actually transmitting data over MIPI-CSI");
        return ret;
    }
    
    ESP_LOGI(TAG, "📸 Camera started successfully - continuous capture mode enabled");
    return ESP_OK;
}

esp_err_t camera_capture(uint8_t **buffer, size_t *len)
{
    if (s_cam_handle == NULL || s_frame_buffer == NULL) {
        ESP_LOGE(TAG, "Camera not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (!s_continuous_capture_running) {
        ESP_LOGW(TAG, "Continuous capture not running");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (s_latest_frame_buffer == NULL || s_latest_frame_size == 0) {
        ESP_LOGW(TAG, "No frame captured yet");
        return ESP_ERR_NOT_FOUND;
    }
    
    // Return pointer to the latest captured frame (no copy needed!)
    *buffer = (uint8_t *)s_latest_frame_buffer;
    *len = s_latest_frame_size;
    
    ESP_LOGI(TAG, "📸 Frame from continuous capture: %zu bytes (frame count: %lu)", *len, s_finished_trans_counter);
    
    return ESP_OK;
}

void camera_deinit(void)
{
    ESP_LOGI(TAG, "Deinitializing camera...");
    
    // Stop continuous capture
    s_continuous_capture_running = false;
    
    if (s_cam_handle) {
        esp_cam_ctlr_stop(s_cam_handle);
        esp_cam_ctlr_disable(s_cam_handle);
        esp_cam_ctlr_del(s_cam_handle);
        s_cam_handle = NULL;
        ESP_LOGI(TAG, "Camera controller stopped and released");
    }
    
    if (s_isp_proc) {
        esp_isp_disable(s_isp_proc);
        esp_isp_del_processor(s_isp_proc);
        s_isp_proc = NULL;
        ESP_LOGI(TAG, "ISP processor stopped and released");
    }
    
    example_sensor_deinit(s_sensor_handle);
    ESP_LOGI(TAG, "Camera sensor deinitialized");
    
    // Release frame buffers
    if (s_frame_buffer) {
        heap_caps_free(s_frame_buffer);
        s_frame_buffer = NULL;
        ESP_LOGI(TAG, "Main frame buffer released");
    }
    
    if (s_frame_buffer_1) {
        heap_caps_free(s_frame_buffer_1);
        s_frame_buffer_1 = NULL;
        ESP_LOGI(TAG, "Continuous capture buffer 1 released");
    }
    
    if (s_frame_buffer_2) {
        heap_caps_free(s_frame_buffer_2);
        s_frame_buffer_2 = NULL;
        ESP_LOGI(TAG, "Continuous capture buffer 2 released");
    }
    
    // Release MIPI LDO channel
    if (s_ldo_mipi_phy) {
        esp_ldo_release_channel(s_ldo_mipi_phy);
        s_ldo_mipi_phy = NULL;
        ESP_LOGI(TAG, "MIPI LDO released");
    }
    
    ESP_LOGI(TAG, "Camera deinitialization completed");
}

size_t camera_get_frame_size(void)
{
    return s_frame_size;
}

// Camera control functions for adjusting image parameters using ESP32-P4 ISP
esp_err_t camera_set_brightness(int brightness)
{
    ESP_LOGI(TAG, "🔆 Setting camera brightness to: %d", brightness);
    
    if (!s_isp_proc) {
        ESP_LOGE(TAG, "❌ ISP processor not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    // ESP32-P4 ISP brightness range: -128 to 127
    int isp_brightness = (brightness - 50) * 255 / 100;  // Convert 0-100 to -127 to 128
    if (isp_brightness < -128) isp_brightness = -128;
    if (isp_brightness > 127) isp_brightness = 127;
    
    // Update global state
    s_current_color_config.color_brightness = isp_brightness;
    
    esp_err_t ret = esp_isp_color_configure(s_isp_proc, &s_current_color_config);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "✅ Camera brightness set successfully (ISP value: %d)", isp_brightness);
    } else {
        ESP_LOGE(TAG, "❌ Failed to set camera brightness: %s", esp_err_to_name(ret));
    }
    
    return ret;
}

esp_err_t camera_set_contrast(int contrast)
{
    ESP_LOGI(TAG, "🎨 Setting camera contrast to: %d", contrast);
    
    if (!s_isp_proc) {
        ESP_LOGE(TAG, "❌ ISP processor not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    // ESP32-P4 ISP contrast range: 0.0 to 1.0, convert 0-100 to 0.0-2.0
    float contrast_float = (float)contrast / 50.0f;  // 50% = 1.0 (normal)
    if (contrast_float > 2.0f) contrast_float = 2.0f;
    if (contrast_float < 0.0f) contrast_float = 0.0f;
    
    // Update global state
    s_current_color_config.color_contrast.integer = (uint32_t)contrast_float;
    s_current_color_config.color_contrast.decimal = (uint32_t)((contrast_float - s_current_color_config.color_contrast.integer) * 127);
    
    esp_err_t ret = esp_isp_color_configure(s_isp_proc, &s_current_color_config);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "✅ Camera contrast set successfully (ISP value: %.2f)", contrast_float);
    } else {
        ESP_LOGE(TAG, "❌ Failed to set camera contrast: %s", esp_err_to_name(ret));
    }
    
    return ret;
}

esp_err_t camera_set_saturation(int saturation)
{
    ESP_LOGI(TAG, "🌈 Setting camera saturation to: %d", saturation);
    
    if (!s_isp_proc) {
        ESP_LOGE(TAG, "❌ ISP processor not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    // ESP32-P4 ISP saturation range: 0.0 to 1.0, convert 0-100 to 0.0-2.0
    float saturation_float = (float)saturation / 50.0f;  // 50% = 1.0 (normal)
    if (saturation_float > 2.0f) saturation_float = 2.0f;
    if (saturation_float < 0.0f) saturation_float = 0.0f;
    
    // Update global state
    s_current_color_config.color_saturation.integer = (uint32_t)saturation_float;
    s_current_color_config.color_saturation.decimal = (uint32_t)((saturation_float - s_current_color_config.color_saturation.integer) * 127);
    
    esp_err_t ret = esp_isp_color_configure(s_isp_proc, &s_current_color_config);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "✅ Camera saturation set successfully (ISP value: %.2f)", saturation_float);
    } else {
        ESP_LOGE(TAG, "❌ Failed to set camera saturation: %s", esp_err_to_name(ret));
    }
    
    return ret;
}

esp_err_t camera_set_exposure_mode(const char* mode)
{
    if (!mode) {
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "💡 Setting camera exposure mode to: %s", mode);
    
    // For ESP32-P4, exposure can be controlled through sensor IOCTL
    if (s_sensor_handle.sensor_device) {
        if (strcmp(mode, "Auto") == 0) {
            // Enable auto exposure
            uint32_t auto_exp = 1;
            esp_err_t ret = esp_cam_sensor_ioctl(s_sensor_handle.sensor_device, 
                                               ESP_CAM_SENSOR_IOC_S_AE, &auto_exp);
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "✅ Auto exposure enabled");
            } else {
                ESP_LOGW(TAG, "⚠️ Failed to enable auto exposure: %s", esp_err_to_name(ret));
            }
            return ret;
        } else if (strcmp(mode, "Manual") == 0) {
            // Disable auto exposure
            uint32_t auto_exp = 0;
            esp_err_t ret = esp_cam_sensor_ioctl(s_sensor_handle.sensor_device, 
                                               ESP_CAM_SENSOR_IOC_S_AE, &auto_exp);
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "✅ Manual exposure enabled");
            } else {
                ESP_LOGW(TAG, "⚠️ Failed to enable manual exposure: %s", esp_err_to_name(ret));
            }
            return ret;
        } else if (strcmp(mode, "Night") == 0) {
            // Set night mode (longer exposure)
            uint32_t night_exp = 1;
            esp_err_t ret = esp_cam_sensor_ioctl(s_sensor_handle.sensor_device, 
                                               ESP_CAM_SENSOR_IOC_S_NIGHT_MODE, &night_exp);
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "✅ Night exposure mode set");
            } else {
                ESP_LOGW(TAG, "⚠️ Failed to set night mode: %s", esp_err_to_name(ret));
            }
            return ret;
        }
    } else {
        ESP_LOGW(TAG, "⚠️ Sensor device not available for exposure control");
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGW(TAG, "⚠️ Unknown exposure mode: %s", mode);
    return ESP_ERR_INVALID_ARG;
}

esp_err_t camera_set_white_balance(const char* wb)
{
    if (!wb) {
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "⚪ Setting camera white balance to: %s", wb);
    
    // For ESP32-P4, white balance can be controlled through sensor IOCTL
    if (s_sensor_handle.sensor_device) {
        if (strcmp(wb, "Auto") == 0) {
            // Enable auto white balance
            uint32_t awb = 1;
            esp_err_t ret = esp_cam_sensor_ioctl(s_sensor_handle.sensor_device, 
                                               ESP_CAM_SENSOR_IOC_S_AWB, &awb);
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "✅ Auto white balance enabled");
            } else {
                ESP_LOGW(TAG, "⚠️ Failed to enable auto white balance: %s", esp_err_to_name(ret));
            }
            return ret;
        } else if (strcmp(wb, "Daylight") == 0) {
            // Set daylight white balance
            uint32_t wb_mode = ESP_CAM_SENSOR_WB_DAYLIGHT;
            esp_err_t ret = esp_cam_sensor_ioctl(s_sensor_handle.sensor_device, 
                                               ESP_CAM_SENSOR_IOC_S_WB_MODE, &wb_mode);
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "✅ Daylight white balance set");
            } else {
                ESP_LOGW(TAG, "⚠️ Failed to set daylight white balance: %s", esp_err_to_name(ret));
            }
            return ret;
        } else if (strcmp(wb, "Cloudy") == 0) {
            // Set cloudy white balance
            uint32_t wb_mode = ESP_CAM_SENSOR_WB_CLOUDY;
            esp_err_t ret = esp_cam_sensor_ioctl(s_sensor_handle.sensor_device, 
                                               ESP_CAM_SENSOR_IOC_S_WB_MODE, &wb_mode);
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "✅ Cloudy white balance set");
            } else {
                ESP_LOGW(TAG, "⚠️ Failed to set cloudy white balance: %s", esp_err_to_name(ret));
            }
            return ret;
        }
    } else {
        ESP_LOGW(TAG, "⚠️ Sensor device not available for white balance control");
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGW(TAG, "⚠️ Unknown white balance mode: %s", wb);
    return ESP_ERR_INVALID_ARG;
}

esp_err_t camera_auto_adjust(const uint8_t* frame_buffer, size_t frame_size)
{
    if (!frame_buffer || frame_size == 0) {
        ESP_LOGE(TAG, "Invalid frame buffer for auto-adjustment");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!s_isp_proc) {
        ESP_LOGE(TAG, "ISP processor not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGD(TAG, "🔧 Running automatic camera adjustment analysis...");
    
    // Analyze frame statistics for auto-adjustment
    uint32_t total_pixels = frame_size / 2; // RGB565 = 2 bytes per pixel
    uint32_t bright_pixels = 0;
    uint32_t dark_pixels = 0;
    uint32_t total_brightness = 0;
    uint32_t sample_count = 0;
    
    // Sample pixels for analysis (sample every 10th pixel for performance)
    const uint16_t* pixels = (const uint16_t*)frame_buffer;
    uint32_t sample_step = total_pixels / 1000; // Sample ~1000 pixels
    if (sample_step == 0) sample_step = 1;
    
    for (uint32_t i = 0; i < total_pixels; i += sample_step) {
        uint16_t pixel = pixels[i];
        
        // Extract RGB components from RGB565
        uint8_t r = (pixel >> 11) & 0x1F;
        uint8_t g = (pixel >> 5) & 0x3F;
        uint8_t b = pixel & 0x1F;
        
        // Convert to 8-bit and calculate brightness
        uint8_t r8 = (r << 3) | (r >> 2);
        uint8_t g8 = (g << 2) | (g >> 4);
        uint8_t b8 = (b << 3) | (b >> 2);
        
        uint8_t brightness = (uint8_t)((r8 * 0.299f + g8 * 0.587f + b8 * 0.114f));
        total_brightness += brightness;
        sample_count++;
        
        // Count bright/dark pixels
        if (brightness > 200) bright_pixels++;
        if (brightness < 50) dark_pixels++;
    }
    
    if (sample_count == 0) {
        ESP_LOGW(TAG, "⚠️ No pixels sampled for analysis");
        return ESP_ERR_INVALID_STATE;
    }
    
    float avg_brightness = (float)total_brightness / sample_count;
    float bright_ratio = (float)bright_pixels / sample_count;
    float dark_ratio = (float)dark_pixels / sample_count;
    
    ESP_LOGD(TAG, "📊 Frame analysis - Avg brightness: %.1f, Bright pixels: %.1f%%, Dark pixels: %.1f%%", 
             avg_brightness, bright_ratio * 100.0f, dark_ratio * 100.0f);
    
    // Auto-adjustment logic
    esp_err_t ret = ESP_OK;
    bool adjustment_made = false;
    
    // Brightness adjustment
    if (avg_brightness < 100.0f || avg_brightness > 150.0f) {
        int brightness_adjustment = (int)((128.0f - avg_brightness) * 0.5f);
        brightness_adjustment = (brightness_adjustment > 20) ? 20 : 
                               (brightness_adjustment < -20) ? -20 : brightness_adjustment;
        
        if (abs(brightness_adjustment) >= 5) {
            ret |= camera_set_brightness(brightness_adjustment);
            adjustment_made = true;
            ESP_LOGI(TAG, "🔆 Auto-adjusted brightness by %d (avg: %.1f)", 
                     brightness_adjustment, avg_brightness);
        }
    }
    
    // Contrast adjustment based on bright/dark pixel ratios
    if (bright_ratio > 0.3f || dark_ratio > 0.3f) {
        int contrast_adjustment = (bright_ratio > dark_ratio) ? -10 : 10;
        ret |= camera_set_contrast(contrast_adjustment);
        adjustment_made = true;
        ESP_LOGI(TAG, "🎨 Auto-adjusted contrast by %d (bright: %.1f%%, dark: %.1f%%)", 
                 contrast_adjustment, bright_ratio * 100.0f, dark_ratio * 100.0f);
    }
    
    // Saturation adjustment (slight boost for better color)
    if (bright_ratio < 0.1f && dark_ratio < 0.1f) {
        ret |= camera_set_saturation(10);
        adjustment_made = true;
        ESP_LOGI(TAG, "🌈 Auto-adjusted saturation +10 for better color");
    }
    
    if (!adjustment_made) {
        ESP_LOGD(TAG, "✅ No camera adjustments needed");
    }
    
    return ret;
}
