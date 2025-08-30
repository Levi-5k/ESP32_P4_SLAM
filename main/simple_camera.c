/*
 * Simple Camera Implementation for ESP32-P4-WIFI6
 * Using basic MIPI-CSI setup without complex ISP
 */

#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_err.h"
#include "esp_cam_ctlr_csi.h"
#include "driver/i2c_master.h"
#include "esp_ldo_regulator.h"
#include "simple_camera.h"

static const char *TAG = "simple_camera";

// Configuration
#define I2C_MASTER_SCL_IO           8
#define I2C_MASTER_SDA_IO           7  
#define I2C_MASTER_FREQ_HZ          400000
#define OV5647_SENSOR_ADDR          0x36

// Global handles
static esp_cam_ctlr_handle_t cam_handle = NULL;
static i2c_master_bus_handle_t i2c_bus = NULL;
static i2c_master_dev_handle_t ov5647_dev = NULL;
static esp_ldo_channel_handle_t ldo_chan = NULL;

esp_err_t simple_camera_init(void)
{
    esp_err_t ret = ESP_OK;
    
    ESP_LOGI(TAG, "Initializing simple camera system");
    
    // 1. Initialize MIPI LDO power
    esp_ldo_channel_config_t ldo_config = {
        .chan_id = 3,
        .voltage_mv = 2500,
    };
    ret = esp_ldo_acquire_channel(&ldo_config, &ldo_chan);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to acquire LDO channel: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = esp_ldo_channel_enable(ldo_chan);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable LDO: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "✅ MIPI LDO enabled at 2.5V");
    
    // 2. Initialize I2C for camera control
    i2c_master_bus_config_t i2c_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
    };
    ret = i2c_new_master_bus(&i2c_config, &i2c_bus);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2C bus: %s", esp_err_to_name(ret));
        return ret;
    }
    
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = OV5647_SENSOR_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ret = i2c_master_bus_add_device(i2c_bus, &dev_cfg, &ov5647_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add I2C device: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "✅ I2C bus initialized");
    
    // 3. Test camera detection
    uint8_t chip_id_h, chip_id_l;
    uint8_t reg_addr_h = 0x30, reg_addr_l = 0x0A;
    
    ret = i2c_master_transmit_receive(ov5647_dev, &reg_addr_h, 1, &chip_id_h, 1, 1000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read chip ID high: %s", esp_err_to_name(ret));
        return ret;
    }
    
    reg_addr_l = 0x0B;
    ret = i2c_master_transmit_receive(ov5647_dev, &reg_addr_l, 1, &chip_id_l, 1, 1000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read chip ID low: %s", esp_err_to_name(ret));
        return ret;
    }
    
    uint16_t chip_id = (chip_id_h << 8) | chip_id_l;
    ESP_LOGI(TAG, "📷 OV5647 Chip ID: 0x%04X", chip_id);
    
    if (chip_id != 0x5647) {
        ESP_LOGW(TAG, "⚠️  Unexpected chip ID, but continuing...");
    }
    
    // 4. Basic MIPI-CSI setup
    esp_cam_ctlr_csi_config_t csi_config = {
        .ctlr_id = 0,
        .h_res = 640,
        .v_res = 480,
        .lane_bit_rate_mbps = 200,
        .input_data_color_type = MIPI_CSI_COLOR_RAW8,
        .output_data_color_type = MIPI_CSI_COLOR_RGB565,
        .data_lane_num = 2,
        .byte_swap_en = false,
        .queue_items = 1,
    };
    
    ret = esp_cam_new_csi_ctlr(&csi_config, &cam_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create CSI controller: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "✅ MIPI-CSI controller created");
    
    ret = esp_cam_ctlr_enable(cam_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable camera controller: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "✅ Camera controller enabled");
    
    ESP_LOGI(TAG, "🎥 Simple camera system initialized successfully");
    return ESP_OK;
}

esp_err_t simple_camera_capture(uint8_t **frame_buffer, size_t *frame_len)
{
    esp_err_t ret = ESP_OK;
    
    if (!cam_handle) {
        ESP_LOGE(TAG, "Camera not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "📸 Attempting to capture frame...");
    
    esp_cam_ctlr_trans_t trans = {0};
    ret = esp_cam_ctlr_receive(cam_handle, &trans, ESP_CAM_CTLR_MAX_DELAY);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to receive frame: %s", esp_err_to_name(ret));
        return ret;
    }
    
    *frame_buffer = trans.buffer;
    *frame_len = trans.received_size;
    
    ESP_LOGI(TAG, "✅ Frame captured: %d bytes", trans.received_size);
    return ESP_OK;
}

void simple_camera_deinit(void)
{
    if (cam_handle) {
        esp_cam_ctlr_disable(cam_handle);
        esp_cam_del_ctlr(cam_handle);
        cam_handle = NULL;
    }
    
    if (ov5647_dev) {
        i2c_master_bus_rm_device(ov5647_dev);
        ov5647_dev = NULL;
    }
    
    if (i2c_bus) {
        i2c_del_master_bus(i2c_bus);
        i2c_bus = NULL;
    }
    
    if (ldo_chan) {
        esp_ldo_channel_disable(ldo_chan);
        esp_ldo_release_channel(ldo_chan);
        ldo_chan = NULL;
    }
    
    ESP_LOGI(TAG, "Camera system deinitialized");
}
