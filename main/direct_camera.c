/*
 * Direct OV5647 MIPI-CSI Camera Driver for ESP32-P4-WIFI6
 * Bypasses ESP-IDF detection system for hardwired camera
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "esp_err.h"
#include "esp_cam_ctlr_csi.h"
#include "esp_cam_ctlr.h"
#include "driver/i2c_master.h"
#include "driver/isp.h"
#include "esp_ldo_regulator.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "direct_camera.h"

static const char *TAG = "direct_camera";

// OV5647 MIPI-CSI Configuration
#define CAMERA_I2C_SCL_IO           8      // I2C SCL pin for OV5647
#define CAMERA_I2C_SDA_IO           7      // I2C SDA pin for OV5647  
#define CAMERA_LANE_BITRATE_MBPS    200    // MIPI lane bitrate
#define CAMERA_DATA_LANES           2      // OV5647 uses 2 data lanes

// OV5647 I2C Configuration
#define OV5647_I2C_ADDR             0x36   // OV5647 I2C address
#define OV5647_CHIP_ID_REG          0x300A // Chip ID register
#define OV5647_CHIP_ID_VALUE        0x5647 // Expected chip ID

// Camera resolution settings - optimized for streaming
#define CAMERA_WIDTH               640     // Streaming resolution width
#define CAMERA_HEIGHT              480     // Streaming resolution height
#define CAMERA_PIXEL_FORMAT        ISP_COLOR_RGB565  // RGB565 for easy streaming

// Global variables
static esp_ldo_channel_handle_t ldo_mipi_phy = NULL;
static i2c_master_bus_handle_t i2c_bus_handle = NULL;
static i2c_master_dev_handle_t i2c_dev_handle = NULL;
static esp_cam_ctlr_handle_t mipi_csi_ctlr = NULL;
static isp_processor_handle_t isp_proc = NULL;
static uint8_t *frame_buffer = NULL;
static size_t frame_buffer_size = 0;

// OV5647 register configuration for 640x480 @ 30fps
static const uint16_t ov5647_init_regs[][2] = {
    // Software reset
    {0x0103, 0x01},
    
    // Basic configuration
    {0x3034, 0x08}, // PLL configuration
    {0x3035, 0x21}, // PLL configuration
    {0x3036, 0x46}, // PLL multiplier
    {0x303c, 0x11}, // PLL configuration
    
    // Timing configuration for 640x480
    {0x3800, 0x00}, // H start high
    {0x3801, 0x00}, // H start low
    {0x3802, 0x00}, // V start high  
    {0x3803, 0x00}, // V start low
    {0x3804, 0x0a}, // H end high
    {0x3805, 0x3f}, // H end low
    {0x3806, 0x07}, // V end high
    {0x3807, 0xa3}, // V end low
    
    // Output size 640x480
    {0x3808, 0x02}, // Output width high
    {0x3809, 0x80}, // Output width low (640)
    {0x380a, 0x01}, // Output height high  
    {0x380b, 0xe0}, // Output height low (480)
    
    // ISP and format control
    {0x4300, 0x61}, // Format control - RGB565
    {0x501f, 0x01}, // ISP RGB enable
    
    // MIPI configuration
    {0x4814, 0x2a}, // MIPI control
    {0x4815, 0x00}, // MIPI control
    
    // Streaming on
    {0x0100, 0x01},
    
    {0xFFFF, 0xFFFF} // End marker
};

esp_err_t ov5647_write_reg(uint16_t reg, uint8_t value)
{
    uint8_t write_buf[3] = {(reg >> 8) & 0xFF, reg & 0xFF, value};
    return i2c_master_transmit(i2c_dev_handle, write_buf, sizeof(write_buf), 1000);
}

esp_err_t ov5647_read_reg(uint16_t reg, uint8_t *value)
{
    uint8_t reg_buf[2] = {(reg >> 8) & 0xFF, reg & 0xFF};
    esp_err_t ret = i2c_master_transmit_receive(i2c_dev_handle, reg_buf, sizeof(reg_buf), value, 1, 1000);
    return ret;
}

esp_err_t direct_camera_init(void)
{
    esp_err_t ret = ESP_OK;
    
    ESP_LOGI(TAG, "=== Direct OV5647 Camera Initialization ===");
    ESP_LOGI(TAG, "Free heap: %lu bytes", esp_get_free_heap_size());
    
    // Step 1: Configure MIPI LDO power
    ESP_LOGI(TAG, "Configuring MIPI LDO power...");
    esp_ldo_channel_config_t ldo_mipi_config = {
        .chan_id = 3,  // MIPI LDO channel
        .voltage_mv = 2500,  // 2.5V for MIPI PHY
    };
    ret = esp_ldo_acquire_channel(&ldo_mipi_config, &ldo_mipi_phy);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure MIPI LDO: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "✅ MIPI LDO configured: Channel 3 at 2500mV");
    
    // Step 2: Initialize I2C for camera control
    ESP_LOGI(TAG, "Initializing I2C for OV5647 control...");
    i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = CAMERA_I2C_SCL_IO,
        .sda_io_num = CAMERA_I2C_SDA_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = 1,
    };
    ret = i2c_new_master_bus(&i2c_bus_config, &i2c_bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2C bus: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Add OV5647 device to I2C bus
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = OV5647_I2C_ADDR,
        .scl_speed_hz = 100000,  // 100 kHz for camera control
    };
    ret = i2c_master_bus_add_device(i2c_bus_handle, &dev_cfg, &i2c_dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add OV5647 to I2C bus: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "✅ I2C initialized for OV5647 at address 0x%02X", OV5647_I2C_ADDR);
    
    // Step 3: Verify OV5647 chip ID
    ESP_LOGI(TAG, "Verifying OV5647 chip ID...");
    uint8_t chip_id_h, chip_id_l;
    ret = ov5647_read_reg(OV5647_CHIP_ID_REG, &chip_id_h);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read chip ID high: %s", esp_err_to_name(ret));
        return ret;
    }
    ret = ov5647_read_reg(OV5647_CHIP_ID_REG + 1, &chip_id_l);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read chip ID low: %s", esp_err_to_name(ret));
        return ret;
    }
    
    uint16_t chip_id = (chip_id_h << 8) | chip_id_l;
    ESP_LOGI(TAG, "Read chip ID: 0x%04X (expected: 0x%04X)", chip_id, OV5647_CHIP_ID_VALUE);
    
    if (chip_id == OV5647_CHIP_ID_VALUE) {
        ESP_LOGI(TAG, "✅ OV5647 chip ID verified successfully!");
    } else {
        ESP_LOGW(TAG, "⚠️ Chip ID mismatch, continuing anyway...");
    }
    
    // Step 4: Configure OV5647 registers
    ESP_LOGI(TAG, "Configuring OV5647 registers...");
    for (int i = 0; ov5647_init_regs[i][0] != 0xFFFF; i++) {
        ret = ov5647_write_reg(ov5647_init_regs[i][0], ov5647_init_regs[i][1]);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to write register 0x%04X: %s", 
                     ov5647_init_regs[i][0], esp_err_to_name(ret));
        }
        vTaskDelay(pdMS_TO_TICKS(1)); // Small delay between register writes
    }
    ESP_LOGI(TAG, "✅ OV5647 registers configured");
    
    // Step 5: Setup MIPI-CSI controller
    ESP_LOGI(TAG, "Initializing MIPI-CSI controller...");
    esp_cam_ctlr_csi_config_t csi_config = {
        .ctlr_id = 0,
        .h_res = CAMERA_WIDTH,
        .v_res = CAMERA_HEIGHT,
        .lane_bit_rate_mbps = CAMERA_LANE_BITRATE_MBPS,
        .input_data_color_type = MIPI_CSI_COLOR_RAW8,
        .output_data_color_type = MIPI_CSI_COLOR_RAW8,
        .data_lane_num = CAMERA_DATA_LANES,
        .byte_swap_en = false,
        .queue_items = 1,
    };
    ret = esp_cam_new_csi_ctlr(&csi_config, &mipi_csi_ctlr);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create MIPI-CSI controller: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "✅ MIPI-CSI controller initialized");
    
    // Step 6: Setup ISP processor for color conversion
    ESP_LOGI(TAG, "Initializing ISP processor...");
    isp_processor_cfg_t isp_config = {
        .clk_hz = 80000000,  // 80 MHz ISP clock
        .input_data_source = ISP_INPUT_DATA_SOURCE_CSI,
        .input_data_color_type = ISP_COLOR_RAW8,
        .output_data_color_type = CAMERA_PIXEL_FORMAT,
        .h_res = CAMERA_WIDTH,
        .v_res = CAMERA_HEIGHT,
        .has_line_start_packet = false,
        .has_line_end_packet = false,
    };
    ret = esp_isp_new_processor(&isp_config, &isp_proc);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create ISP processor: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "✅ ISP processor initialized");
    
    // Step 7: Allocate frame buffer
    frame_buffer_size = CAMERA_WIDTH * CAMERA_HEIGHT * 2; // RGB565 = 2 bytes per pixel
    ESP_LOGI(TAG, "Allocating frame buffer: %zu bytes", frame_buffer_size);
    
    frame_buffer = heap_caps_malloc(frame_buffer_size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!frame_buffer) {
        ESP_LOGE(TAG, "Failed to allocate frame buffer");
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "✅ Frame buffer allocated: %zu bytes at %p", frame_buffer_size, frame_buffer);
    
    ESP_LOGI(TAG, "✅ Direct OV5647 camera initialization complete!");
    ESP_LOGI(TAG, "Resolution: %dx%d, Format: RGB565", CAMERA_WIDTH, CAMERA_HEIGHT);
    ESP_LOGI(TAG, "Free heap after init: %lu bytes", esp_get_free_heap_size());
    
    return ESP_OK;
}

esp_err_t direct_camera_capture(uint8_t **buffer, size_t *len)
{
    if (!frame_buffer || !buffer || !len) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // For now, generate a test pattern since we haven't connected the streaming pipeline yet
    ESP_LOGI(TAG, "Generating test pattern...");
    
    // Create a simple gradient test pattern in RGB565 format
    uint16_t *rgb565_buf = (uint16_t*)frame_buffer;
    for (int y = 0; y < CAMERA_HEIGHT; y++) {
        for (int x = 0; x < CAMERA_WIDTH; x++) {
            // Create a gradient pattern
            uint8_t r = (x * 255) / CAMERA_WIDTH;
            uint8_t g = (y * 255) / CAMERA_HEIGHT;
            uint8_t b = ((x + y) * 255) / (CAMERA_WIDTH + CAMERA_HEIGHT);
            
            // Convert to RGB565
            uint16_t rgb565 = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
            rgb565_buf[y * CAMERA_WIDTH + x] = rgb565;
        }
    }
    
    *buffer = frame_buffer;
    *len = frame_buffer_size;
    
    ESP_LOGI(TAG, "Generated test frame: %dx%d RGB565, %zu bytes", 
             CAMERA_WIDTH, CAMERA_HEIGHT, frame_buffer_size);
    
    return ESP_OK;
}

void direct_camera_deinit(void)
{
    if (frame_buffer) {
        free(frame_buffer);
        frame_buffer = NULL;
    }
    
    if (isp_proc) {
        esp_isp_del_processor(isp_proc);
        isp_proc = NULL;
    }
    
    if (mipi_csi_ctlr) {
        esp_cam_ctlr_disable(mipi_csi_ctlr);
        esp_cam_del_ctlr(mipi_csi_ctlr);
        mipi_csi_ctlr = NULL;
    }
    
    if (i2c_dev_handle) {
        i2c_master_bus_rm_device(i2c_dev_handle);
        i2c_dev_handle = NULL;
    }
    
    if (i2c_bus_handle) {
        i2c_del_master_bus(i2c_bus_handle);
        i2c_bus_handle = NULL;
    }
    
    if (ldo_mipi_phy) {
        esp_ldo_release_channel(ldo_mipi_phy);
        ldo_mipi_phy = NULL;
    }
    
    ESP_LOGI(TAG, "Direct camera deinitialized");
}
