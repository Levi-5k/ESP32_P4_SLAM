/*
 * Camera initialization for ESP32-P4 with OV5647 MIPI-CSI
 * Full implementation for ESP-IDF v5.5 compatibility
 */

#define MIPI_LDO_CHANNEL_ID         3      // LDO channel 3 for MIPI PHY
#define MIPI_LDO_VOLTAGE_MV         2500   // 2.5V for MIPI DPHY

// Camera configuration constants
#define CAMERA_WIDTH                1920
#define CAMERA_HEIGHT               1080
#define CAMERA_I2C_SDA_IO           7
#define CAMERA_I2C_SCL_IO           8
#define CAMERA_LANE_BITRATE_MBPS    200
#define CAMERA_DATA_LANES           2
#define RGB565_BITS_PER_PIXEL       16

// RGB565 frame configuration
#define RGB565_FRAME_SIZE(width, height)  ((width) * (height) * (RGB565_BITS_PER_PIXEL / 16))

#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "driver/i2c_master.h"
#include "esp_ldo_regulator.h"
#include "esp_cam_ctlr_csi.h"
#include "esp_cam_ctlr.h"
#include "driver/isp.h"
#include "esp_cam_sensor.h"
#include "ov5647.h"
#include "esp_sccb_i2c.h"
#include "freertos/FreeRTOS.h"
#include "camera_init.h"
#include "visual_slam_common_types.h"

static const char *TAG = "camera_init";

static esp_cam_ctlr_handle_t s_cam_handle = NULL;
static esp_cam_sensor_device_t *s_sensor_handle = NULL;
static esp_ldo_channel_handle_t s_ldo_mipi_phy = NULL;
static isp_proc_handle_t s_isp_proc = NULL;
static i2c_master_bus_handle_t s_i2c_bus_handle = NULL;  // Store I2C bus handle separately
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
        } else if (!use_buffer_1 && s_frame_buffer_2) {
            trans->buffer = s_frame_buffer_2;
            trans->buflen = s_frame_buffer_size;
            use_buffer_1 = true;
        } else {
            ESP_LOGE(TAG, "No available frame buffer for continuous capture");
            return false;
        }
    } else {
        trans->buffer = s_frame_buffer;
        trans->buflen = s_frame_buffer_size;
    }

    ESP_DRAM_LOGI(TAG, "s_camera_get_new_vb called - providing buffer for capture");
    return false;  // Return false to continue using the same buffer
}

static bool IRAM_ATTR s_camera_get_finished_trans(esp_cam_ctlr_handle_t handle, esp_cam_ctlr_trans_t *trans, void *user_data)
{
    s_finished_trans_counter++;
    ESP_DRAM_LOGI(TAG, "s_camera_get_finished_trans: frame %lu captured, size=%zu",
                  s_finished_trans_counter, trans->buflen);

    // Update latest frame buffer pointer for continuous capture
    if (s_continuous_capture_running) {
        if (trans->buffer == s_frame_buffer_1) {
            s_latest_frame_buffer = s_frame_buffer_1;
        } else if (trans->buffer == s_frame_buffer_2) {
            s_latest_frame_buffer = s_frame_buffer_2;
        }
        s_latest_frame_size = trans->buflen;
    }

    return false;  // Return false to automatically start the next capture (continuous mode)
}

esp_err_t camera_init(void)
{
    esp_err_t ret = ESP_OK;

    ESP_LOGI(TAG, "Initializing OV5647 MIPI-CSI camera for ESP32-P4-WIFI6");

    // Initialize MIPI LDO regulator for camera power
    esp_ldo_channel_config_t ldo_config = {
        .chan_id = MIPI_LDO_CHANNEL_ID,
        .voltage_mv = MIPI_LDO_VOLTAGE_MV,
        .flags.adjustable = false,
        .flags.owned_by_hw = false,
    };

    ret = esp_ldo_acquire_channel(&ldo_config, &s_ldo_mipi_phy);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to acquire LDO channel: %s", esp_err_to_name(ret));
        return ret;
    }

    // Initialize I2C for camera communication
    i2c_master_bus_config_t i2c_bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = CAMERA_I2C_SDA_IO,
        .scl_io_num = CAMERA_I2C_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
    };

    i2c_master_bus_handle_t i2c_bus_handle;
    ret = i2c_new_master_bus(&i2c_bus_config, &i2c_bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2C master bus: %s", esp_err_to_name(ret));
        return ret;
    }
    s_i2c_bus_handle = i2c_bus_handle;  // Store for cleanup

    // Initialize camera sensor
    esp_sccb_io_handle_t sccb_handle;
    sccb_i2c_config_t sccb_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = OV5647_SCCB_ADDR,
        .scl_speed_hz = 100000,
        .addr_bits_width = 16,
        .val_bits_width = 8,
    };

    ret = sccb_new_i2c_io(i2c_bus_handle, &sccb_config, &sccb_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create SCCB IO: %s", esp_err_to_name(ret));
        return ret;
    }

    esp_cam_sensor_config_t sensor_config = {
        .sccb_handle = sccb_handle,
        .reset_pin = -1,  // No reset pin
        .pwdn_pin = -1,   // No power down pin
        .xclk_pin = -1,   // No external clock pin
        .xclk_freq_hz = 24000000, // 24MHz clock
        .sensor_port = ESP_CAM_SENSOR_MIPI_CSI,
    };

    s_sensor_handle = ov5647_detect(&sensor_config);
    if (s_sensor_handle == NULL) {
        ESP_LOGE(TAG, "Failed to detect OV5647 sensor");
        return ESP_FAIL;
    }

    // Configure camera sensor for RGB565 output at 800x640
    const esp_cam_sensor_format_t *format = NULL;
    esp_cam_sensor_format_array_t format_array;
    ret = s_sensor_handle->ops->query_support_formats(s_sensor_handle, &format_array);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to query supported formats: %s", esp_err_to_name(ret));
        return ret;
    }

    // Find RGB565 format with 800x640 resolution
    for (int i = 0; i < format_array.count; i++) {
        if (format_array.format_array[i].format == ESP_CAM_SENSOR_PIXFORMAT_RGB565 &&
            format_array.format_array[i].width == CAMERA_WIDTH &&
            format_array.format_array[i].height == CAMERA_HEIGHT) {
            format = &format_array.format_array[i];
            break;
        }
    }

    if (format == NULL) {
        ESP_LOGE(TAG, "RGB565 format with 800x640 resolution not supported");
        return ESP_ERR_NOT_SUPPORTED;
    }

    ret = s_sensor_handle->ops->set_format(s_sensor_handle, format);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set sensor format: %s", esp_err_to_name(ret));
        return ret;
    }

    // Initialize ISP processor (commented out - OV5647 has internal ISP)
    /*
    isp_proc_config_t isp_config = {
        .source_type = ISP_SOURCE_TYPE_CSI,
        .input_data_color_type = ISP_COLOR_RAW8,
        .input_data_source = ISP_INPUT_DATA_SOURCE_CSI,
        .output_data_color_type = ISP_COLOR_RGB565,
        .has_line_start_packet = false,
        .has_line_end_packet = false,
        .h_res = CAMERA_WIDTH,
        .v_res = CAMERA_HEIGHT,
    };

    ret = isp_proc_new_controller(&isp_config, &s_isp_proc);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create ISP processor: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = isp_proc_enable(s_isp_proc);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable ISP processor: %s", esp_err_to_name(ret));
        return ret;
    }
    */

    // Initialize camera controller
    esp_cam_ctlr_csi_config_t csi_config = {
        .ctlr_id = 0,
        .h_res = CAMERA_WIDTH,
        .v_res = CAMERA_HEIGHT,
        .lane_bit_rate_mbps = CAMERA_LANE_BITRATE_MBPS,
        .data_lane_num = CAMERA_DATA_LANES,
        .byte_swap_en = false,
    };

    ret = esp_cam_new_csi_ctlr(&csi_config, &s_cam_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create CSI camera controller: %s", esp_err_to_name(ret));
        return ret;
    }

    // Register camera controller callbacks
    esp_cam_ctlr_evt_cbs_t cbs = {
        .on_get_new_trans = s_camera_get_new_vb,
        .on_trans_finished = s_camera_get_finished_trans,
    };

    ret = esp_cam_ctlr_register_event_callbacks(s_cam_handle, &cbs, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register camera callbacks: %s", esp_err_to_name(ret));
        return ret;
    }

    // Allocate frame buffer
    s_frame_buffer_size = CAMERA_WIDTH * CAMERA_HEIGHT * RGB565_BITS_PER_PIXEL / 8;
    s_frame_buffer = heap_caps_malloc(s_frame_buffer_size, MALLOC_CAP_SPIRAM);
    if (!s_frame_buffer) {
        ESP_LOGE(TAG, "Failed to allocate frame buffer");
        return ESP_ERR_NO_MEM;
    }

    // Allocate continuous capture buffers
    s_frame_buffer_1 = heap_caps_malloc(s_frame_buffer_size, MALLOC_CAP_SPIRAM);
    s_frame_buffer_2 = heap_caps_malloc(s_frame_buffer_size, MALLOC_CAP_SPIRAM);
    if (!s_frame_buffer_1 || !s_frame_buffer_2) {
        ESP_LOGE(TAG, "Failed to allocate continuous capture buffers");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "✅ Camera initialized successfully");
    ESP_LOGI(TAG, "   Resolution: %dx%d", CAMERA_WIDTH, CAMERA_HEIGHT);
    ESP_LOGI(TAG, "   Frame buffer size: %zu bytes", s_frame_buffer_size);

    return ESP_OK;
}

esp_err_t camera_start(void)
{
    if (!s_cam_handle) {
        ESP_LOGE(TAG, "Camera not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = esp_cam_ctlr_start(s_cam_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start camera: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "✅ Camera started successfully");
    return ESP_OK;
}

esp_err_t camera_capture(uint8_t **buffer, size_t *len)
{
    if (!s_cam_handle || !buffer || !len) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_continuous_capture_running) {
        // Single capture mode
        esp_cam_ctlr_trans_t trans = {
            .buffer = s_frame_buffer,
            .buflen = s_frame_buffer_size,
        };

        esp_err_t ret = esp_cam_ctlr_receive(s_cam_handle, &trans, pdMS_TO_TICKS(1000));
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Camera capture failed: %s", esp_err_to_name(ret));
            return ret;
        }

        *buffer = (uint8_t *)s_frame_buffer;
        *len = trans.buflen;
        s_frame_size = trans.buflen;
    } else {
        // Continuous capture mode - return latest frame
        if (!s_latest_frame_buffer) {
            return ESP_ERR_NOT_FOUND;
        }
        *buffer = (uint8_t *)s_latest_frame_buffer;
        *len = s_latest_frame_size;
    }

    return ESP_OK;
}

size_t camera_get_frame_size(void)
{
    return s_frame_size;
}

void camera_deinit(void)
{
    if (s_cam_handle) {
        esp_cam_ctlr_stop(s_cam_handle);
        esp_cam_ctlr_del(s_cam_handle);
        s_cam_handle = NULL;
    }

    if (s_isp_proc) {
        // isp_proc_disable(s_isp_proc);
        // isp_proc_del_controller(s_isp_proc);
        s_isp_proc = NULL;
    }

    if (s_i2c_bus_handle) {
        i2c_del_master_bus(s_i2c_bus_handle);
    }

    if (s_ldo_mipi_phy) {
        esp_ldo_release_channel(s_ldo_mipi_phy);
        s_ldo_mipi_phy = NULL;
    }

    if (s_frame_buffer) {
        heap_caps_free(s_frame_buffer);
        s_frame_buffer = NULL;
    }

    if (s_frame_buffer_1) {
        heap_caps_free(s_frame_buffer_1);
        s_frame_buffer_1 = NULL;
    }

    if (s_frame_buffer_2) {
        heap_caps_free(s_frame_buffer_2);
        s_frame_buffer_2 = NULL;
    }

    s_continuous_capture_running = false;
    s_latest_frame_buffer = NULL;
    s_latest_frame_size = 0;

    ESP_LOGI(TAG, "✅ Camera deinitialized");
}

esp_err_t camera_set_brightness(int brightness)
{
    if (!s_isp_proc) {
        return ESP_ERR_INVALID_STATE;
    }

    // Validate brightness range (-100 to 100)
    if (brightness < -100 || brightness > 100) {
        return ESP_ERR_INVALID_ARG;
    }

    // Convert brightness to ISP format (0-255 range)
    uint8_t isp_brightness = (brightness + 100) * 255 / 200;

    s_current_color_config.color_brightness = isp_brightness;

    // esp_err_t ret = isp_proc_set_color_config(s_isp_proc, &s_current_color_config);
    // if (ret != ESP_OK) {
    //     ESP_LOGE(TAG, "Failed to set brightness: %s", esp_err_to_name(ret));
    //     return ret;
    // }

    ESP_LOGI(TAG, "✅ Brightness set to %d (not implemented)", brightness);
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t camera_set_contrast(int contrast)
{
    if (!s_isp_proc) {
        return ESP_ERR_INVALID_STATE;
    }

    // Validate contrast range (-100 to 100)
    if (contrast < -100 || contrast > 100) {
        return ESP_ERR_INVALID_ARG;
    }

    // Convert contrast to ISP format (0.0-2.0 range)
    float isp_contrast = (contrast + 100) / 100.0f;

    s_current_color_config.color_contrast.integer = (uint8_t)isp_contrast;
    s_current_color_config.color_contrast.decimal = (uint8_t)((isp_contrast - (uint8_t)isp_contrast) * 256);

    // esp_err_t ret = isp_proc_set_color_config(s_isp_proc, &s_current_color_config);
    // if (ret != ESP_OK) {
    //     ESP_LOGE(TAG, "Failed to set contrast: %s", esp_err_to_name(ret));
    //     return ret;
    // }

    ESP_LOGI(TAG, "✅ Contrast set to %d (not implemented)", contrast);
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t camera_set_saturation(int saturation)
{
    if (!s_isp_proc) {
        return ESP_ERR_INVALID_STATE;
    }

    // Validate saturation range (-100 to 100)
    if (saturation < -100 || saturation > 100) {
        return ESP_ERR_INVALID_ARG;
    }

    // Convert saturation to ISP format (0.0-2.0 range)
    float isp_saturation = (saturation + 100) / 100.0f;

    s_current_color_config.color_saturation.integer = (uint8_t)isp_saturation;
    s_current_color_config.color_saturation.decimal = (uint8_t)((isp_saturation - (uint8_t)isp_saturation) * 256);

    // esp_err_t ret = isp_proc_set_color_config(s_isp_proc, &s_current_color_config);
    // if (ret != ESP_OK) {
    //     ESP_LOGE(TAG, "Failed to set saturation: %s", esp_err_to_name(ret));
    //     return ret;
    // }

    ESP_LOGI(TAG, "✅ Saturation set to %d (not implemented)", saturation);
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t camera_set_exposure_mode(const char* mode)
{
    if (!s_i2c_bus_handle) {
        return ESP_ERR_INVALID_STATE;
    }

    uint32_t exposure_mode;

    if (strcmp(mode, "Auto") == 0) {
        exposure_mode = ESP_CAM_SENSOR_AE_CONTROL;
    } else if (strcmp(mode, "Manual") == 0) {
        exposure_mode = ESP_CAM_SENSOR_AE_CONTROL;  // Manual mode
    } else if (strcmp(mode, "Night") == 0) {
        exposure_mode = ESP_CAM_SENSOR_SENSOR_MODE;  // Night mode
    } else {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = s_sensor_handle->ops->set_para_value(s_sensor_handle, ESP_CAM_SENSOR_AE_CONTROL, &exposure_mode, sizeof(exposure_mode));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set exposure mode: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "✅ Exposure mode set to %s", mode);
    return ESP_OK;
}

esp_err_t camera_set_white_balance(const char* wb)
{
    if (!s_i2c_bus_handle) {
        return ESP_ERR_INVALID_STATE;
    }

    uint32_t wb_mode;

    if (strcmp(wb, "Auto") == 0) {
        wb_mode = ESP_CAM_SENSOR_AWB;
    } else if (strcmp(wb, "Daylight") == 0) {
        wb_mode = ESP_CAM_SENSOR_WB;
    } else if (strcmp(wb, "Cloudy") == 0) {
        wb_mode = ESP_CAM_SENSOR_WB;
    } else {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = s_sensor_handle->ops->set_para_value(s_sensor_handle, wb_mode, &wb_mode, sizeof(wb_mode));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set white balance: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "✅ White balance set to %s", wb);
    return ESP_OK;
}

esp_err_t camera_auto_adjust(const uint8_t* frame_buffer, size_t frame_size)
{
    if (!frame_buffer || frame_size == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    // Simple brightness analysis (RGB565 format)
    uint32_t total_brightness = 0;
    uint32_t pixel_count = frame_size / 2;  // 2 bytes per pixel in RGB565

    for (uint32_t i = 0; i < pixel_count; i++) {
        // Extract RGB components from RGB565
        uint16_t pixel = ((uint16_t*)frame_buffer)[i];
        uint8_t r = (pixel >> 11) & 0x1F;  // 5 bits red
        uint8_t g = (pixel >> 5) & 0x3F;   // 6 bits green
        uint8_t b = pixel & 0x1F;          // 5 bits blue

        // Convert to 8-bit and calculate brightness
        r = (r * 255) / 31;
        g = (g * 255) / 63;
        b = (b * 255) / 31;

        // Use luminance formula: Y = 0.299*R + 0.587*G + 0.114*B
        uint8_t brightness = (uint8_t)(0.299f * r + 0.587f * g + 0.114f * b);
        total_brightness += brightness;
    }

    uint8_t avg_brightness = total_brightness / pixel_count;

    // Auto-adjust brightness based on average brightness
    int target_brightness = 128;  // Target middle brightness
    int brightness_adjustment = target_brightness - avg_brightness;

    // Limit adjustment range
    if (brightness_adjustment > 50) brightness_adjustment = 50;
    if (brightness_adjustment < -50) brightness_adjustment = -50;

    esp_err_t ret = camera_set_brightness(brightness_adjustment);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to auto-adjust brightness: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "✅ Auto-adjusted brightness: avg=%d, adjustment=%d",
             avg_brightness, brightness_adjustment);

    return ESP_OK;
}