#include "imu_bmi088.h"
#include "visual_slam_common_types.h"
#include <string.h>
#include <esp_log.h>
#include <driver/spi_master.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <esp_timer.h>
#include <math.h>

static const char *TAG = "IMU_BMI088";

// BMI088 Register addresses
#define BMI088_ACC_CHIP_ID          0x00
#define BMI088_ACC_DATA_X_LSB       0x12
#define BMI088_ACC_DATA_X_MSB       0x13
#define BMI088_ACC_DATA_Y_LSB       0x14
#define BMI088_ACC_DATA_Y_MSB       0x15
#define BMI088_ACC_DATA_Z_LSB       0x16
#define BMI088_ACC_DATA_Z_MSB       0x17
#define BMI088_ACC_TEMP_MSB         0x22
#define BMI088_ACC_TEMP_LSB         0x23
#define BMI088_ACC_CONF             0x40
#define BMI088_ACC_RANGE            0x41
#define BMI088_ACC_PWR_CONF         0x7C
#define BMI088_ACC_PWR_CTRL         0x7D
#define BMI088_ACC_SOFTRESET        0x7E

#define BMI088_GYRO_CHIP_ID         0x00
#define BMI088_GYRO_RATE_X_LSB      0x02
#define BMI088_GYRO_RATE_X_MSB      0x03
#define BMI088_GYRO_RATE_Y_LSB      0x04
#define BMI088_GYRO_RATE_Y_MSB      0x05
#define BMI088_GYRO_RATE_Z_LSB      0x06
#define BMI088_GYRO_RATE_Z_MSB      0x07
#define BMI088_GYRO_RANGE           0x0F
#define BMI088_GYRO_BANDWIDTH       0x10
#define BMI088_GYRO_LPM1            0x11
#define BMI088_GYRO_SOFTRESET       0x14

// Chip IDs
#define BMI088_ACC_CHIP_ID_VALUE    0x1E
#define BMI088_GYRO_CHIP_ID_VALUE   0x0F

// IMU state structure
static struct {
    spi_device_handle_t acc_spi;
    spi_device_handle_t gyro_spi;
    int acc_cs_pin;
    int gyro_cs_pin;
    bool initialized;
    imu_data_t current_data;
    SemaphoreHandle_t data_mutex;
    TaskHandle_t read_task;
    volatile bool task_running;
    
    // Calibration data
    float acc_offset[3];
    float gyro_offset[3];
    float acc_scale[3];
    float gyro_scale[3];
    bool calibrated;
} imu_state = {0};

// Forward declarations
static void imu_read_task(void *param);
static esp_err_t bmi088_acc_init(void);
static esp_err_t bmi088_gyro_init(void);
static esp_err_t bmi088_read_acc_data(float *acc_x, float *acc_y, float *acc_z, float *temp);
static esp_err_t bmi088_read_gyro_data(float *gyro_x, float *gyro_y, float *gyro_z);
static esp_err_t bmi088_write_acc_reg(uint8_t reg, uint8_t value);
static esp_err_t bmi088_write_gyro_reg(uint8_t reg, uint8_t value);
static esp_err_t bmi088_read_acc_reg(uint8_t reg, uint8_t *value);
static esp_err_t bmi088_read_gyro_reg(uint8_t reg, uint8_t *value);
static esp_err_t calibrate_imu(void);

esp_err_t sensor_fusion_init_imu(const imu_config_t *config) {
    if (!config) {
        ESP_LOGE(TAG, "IMU config is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (imu_state.initialized) {
        ESP_LOGW(TAG, "IMU already initialized");
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Initializing BMI088 IMU on SPI");
    
    // Store configuration
    imu_state.acc_cs_pin = config->acc_cs_pin;
    imu_state.gyro_cs_pin = config->gyro_cs_pin;
    
    // Configure SPI bus
    spi_bus_config_t buscfg = {
        .miso_io_num = config->miso_pin,
        .mosi_io_num = config->mosi_pin,
        .sclk_io_num = config->sclk_pin,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32,
    };
    
    esp_err_t ret = spi_bus_initialize(config->spi_host, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus");
        return ret;
    }
    
    // Configure accelerometer SPI device
    spi_device_interface_config_t acc_devcfg = {
        .clock_speed_hz = 10 * 1000 * 1000,  // 10 MHz
        .mode = 0,                            // SPI mode 0
        .spics_io_num = config->acc_cs_pin,
        .queue_size = 7,
        .pre_cb = NULL,
        .post_cb = NULL,
    };
    
    ret = spi_bus_add_device(config->spi_host, &acc_devcfg, &imu_state.acc_spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add accelerometer SPI device");
        return ret;
    }
    
    // Configure gyroscope SPI device
    spi_device_interface_config_t gyro_devcfg = {
        .clock_speed_hz = 10 * 1000 * 1000,  // 10 MHz
        .mode = 0,                            // SPI mode 0
        .spics_io_num = config->gyro_cs_pin,
        .queue_size = 7,
        .pre_cb = NULL,
        .post_cb = NULL,
    };
    
    ret = spi_bus_add_device(config->spi_host, &gyro_devcfg, &imu_state.gyro_spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add gyroscope SPI device");
        return ret;
    }
    
    // Initialize accelerometer
    ret = bmi088_acc_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize accelerometer");
        return ret;
    }
    
    // Initialize gyroscope
    ret = bmi088_gyro_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize gyroscope");
        return ret;
    }
    
    // Create mutex
    imu_state.data_mutex = xSemaphoreCreateMutex();
    if (!imu_state.data_mutex) {
        ESP_LOGE(TAG, "Failed to create IMU data mutex");
        return ESP_ERR_NO_MEM;
    }
    
    // Initialize IMU data structure
    memset(&imu_state.current_data, 0, sizeof(imu_data_t));
    
    // Initialize calibration parameters
    for (int i = 0; i < 3; i++) {
        imu_state.acc_offset[i] = 0.0f;
        imu_state.gyro_offset[i] = 0.0f;
        imu_state.acc_scale[i] = 1.0f;
        imu_state.gyro_scale[i] = 1.0f;
    }
    
    // Calibrate IMU
    ret = calibrate_imu();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "IMU calibration failed, using default values");
    }
    
    // Create reading task
    imu_state.task_running = true;
    BaseType_t task_ret = xTaskCreate(imu_read_task, "imu_read", 4096, NULL, 5, &imu_state.read_task);
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create IMU reading task");
        imu_state.task_running = false;
        return ESP_ERR_NO_MEM;
    }
    
    imu_state.initialized = true;
    ESP_LOGI(TAG, "BMI088 IMU initialized successfully");
    
    return ESP_OK;
}

esp_err_t imu_get_data(imu_data_t *data) {
    if (!data) return ESP_ERR_INVALID_ARG;
    if (!imu_state.initialized) return ESP_ERR_INVALID_STATE;
    
    xSemaphoreTake(imu_state.data_mutex, portMAX_DELAY);
    memcpy(data, &imu_state.current_data, sizeof(imu_data_t));
    xSemaphoreGive(imu_state.data_mutex);
    
    return ESP_OK;
}

static void imu_read_task(void *param) {
    float acc_x, acc_y, acc_z, temp;
    float gyro_x, gyro_y, gyro_z;
    
    ESP_LOGI(TAG, "IMU reading task started");
    
    while (imu_state.task_running) {
        // Read accelerometer data
        if (bmi088_read_acc_data(&acc_x, &acc_y, &acc_z, &temp) == ESP_OK) {
            // Read gyroscope data
            if (bmi088_read_gyro_data(&gyro_x, &gyro_y, &gyro_z) == ESP_OK) {
                xSemaphoreTake(imu_state.data_mutex, portMAX_DELAY);
                
                // Apply calibration
                imu_state.current_data.accel_x = (acc_x - imu_state.acc_offset[0]) * imu_state.acc_scale[0];
                imu_state.current_data.accel_y = (acc_y - imu_state.acc_offset[1]) * imu_state.acc_scale[1];
                imu_state.current_data.accel_z = (acc_z - imu_state.acc_offset[2]) * imu_state.acc_scale[2];
                
                imu_state.current_data.gyro_x = (gyro_x - imu_state.gyro_offset[0]) * imu_state.gyro_scale[0];
                imu_state.current_data.gyro_y = (gyro_y - imu_state.gyro_offset[1]) * imu_state.gyro_scale[1];
                imu_state.current_data.gyro_z = (gyro_z - imu_state.gyro_offset[2]) * imu_state.gyro_scale[2];
                
                imu_state.current_data.temp = temp;
                imu_state.current_data.temp_c = temp; // Alias
                imu_state.current_data.timestamp_us = esp_timer_get_time();
                
                xSemaphoreGive(imu_state.data_mutex);
                
                ESP_LOGD(TAG, "IMU: acc=[%.3f,%.3f,%.3f] gyro=[%.3f,%.3f,%.3f] temp=%.1f°C",
                         imu_state.current_data.accel_x, imu_state.current_data.accel_y, imu_state.current_data.accel_z,
                         imu_state.current_data.gyro_x, imu_state.current_data.gyro_y, imu_state.current_data.gyro_z,
                         imu_state.current_data.temp);
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10)); // 100Hz update rate
    }
    
    ESP_LOGI(TAG, "IMU reading task ended");
    vTaskDelete(NULL);
}

static esp_err_t bmi088_acc_init(void) {
    uint8_t chip_id;
    
    // Check chip ID
    esp_err_t ret = bmi088_read_acc_reg(BMI088_ACC_CHIP_ID, &chip_id);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read accelerometer chip ID");
        return ret;
    }
    
    if (chip_id != BMI088_ACC_CHIP_ID_VALUE) {
        ESP_LOGE(TAG, "Invalid accelerometer chip ID: 0x%02X (expected 0x%02X)", chip_id, BMI088_ACC_CHIP_ID_VALUE);
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Accelerometer chip ID: 0x%02X", chip_id);
    
    // Soft reset
    ret = bmi088_write_acc_reg(BMI088_ACC_SOFTRESET, 0xB6);
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(50));
    
    // Power on
    ret = bmi088_write_acc_reg(BMI088_ACC_PWR_CONF, 0x00);
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(5));
    
    ret = bmi088_write_acc_reg(BMI088_ACC_PWR_CTRL, 0x04);
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(5));
    
    // Configure range (±6g)
    ret = bmi088_write_acc_reg(BMI088_ACC_RANGE, 0x01);
    if (ret != ESP_OK) return ret;
    
    // Configure output data rate and bandwidth (400Hz ODR, OSR4)
    ret = bmi088_write_acc_reg(BMI088_ACC_CONF, 0x2C);
    if (ret != ESP_OK) return ret;
    
    ESP_LOGI(TAG, "Accelerometer initialized");
    return ESP_OK;
}

static esp_err_t bmi088_gyro_init(void) {
    uint8_t chip_id;
    
    // Check chip ID
    esp_err_t ret = bmi088_read_gyro_reg(BMI088_GYRO_CHIP_ID, &chip_id);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read gyroscope chip ID");
        return ret;
    }
    
    if (chip_id != BMI088_GYRO_CHIP_ID_VALUE) {
        ESP_LOGE(TAG, "Invalid gyroscope chip ID: 0x%02X (expected 0x%02X)", chip_id, BMI088_GYRO_CHIP_ID_VALUE);
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Gyroscope chip ID: 0x%02X", chip_id);
    
    // Soft reset
    ret = bmi088_write_gyro_reg(BMI088_GYRO_SOFTRESET, 0xB6);
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(50));
    
    // Configure range (±500°/s)
    ret = bmi088_write_gyro_reg(BMI088_GYRO_RANGE, 0x01);
    if (ret != ESP_OK) return ret;
    
    // Configure bandwidth (400Hz ODR, 47Hz filter)
    ret = bmi088_write_gyro_reg(BMI088_GYRO_BANDWIDTH, 0x83);
    if (ret != ESP_OK) return ret;
    
    // Normal power mode
    ret = bmi088_write_gyro_reg(BMI088_GYRO_LPM1, 0x00);
    if (ret != ESP_OK) return ret;
    
    ESP_LOGI(TAG, "Gyroscope initialized");
    return ESP_OK;
}

static esp_err_t bmi088_read_acc_data(float *acc_x, float *acc_y, float *acc_z, float *temp) {
    uint8_t data[6];
    uint8_t temp_data[2];
    
    // Read accelerometer data registers
    for (int i = 0; i < 6; i++) {
        esp_err_t ret = bmi088_read_acc_reg(BMI088_ACC_DATA_X_LSB + i, &data[i]);
        if (ret != ESP_OK) return ret;
    }
    
    // Read temperature data
    esp_err_t ret = bmi088_read_acc_reg(BMI088_ACC_TEMP_LSB, &temp_data[0]);
    if (ret != ESP_OK) return ret;
    ret = bmi088_read_acc_reg(BMI088_ACC_TEMP_MSB, &temp_data[1]);
    if (ret != ESP_OK) return ret;
    
    // Convert raw data to g (±6g range, 16-bit)
    int16_t raw_x = (int16_t)((data[1] << 8) | data[0]);
    int16_t raw_y = (int16_t)((data[3] << 8) | data[2]);
    int16_t raw_z = (int16_t)((data[5] << 8) | data[4]);
    
    *acc_x = (float)raw_x * 6.0f / 32768.0f;
    *acc_y = (float)raw_y * 6.0f / 32768.0f;
    *acc_z = (float)raw_z * 6.0f / 32768.0f;
    
    // Convert temperature (11-bit resolution, offset)
    int16_t raw_temp = (int16_t)((temp_data[1] << 8) | temp_data[0]);
    if (raw_temp > 1023) raw_temp -= 2048;
    *temp = (float)raw_temp * 0.125f + 23.0f;
    
    return ESP_OK;
}

static esp_err_t bmi088_read_gyro_data(float *gyro_x, float *gyro_y, float *gyro_z) {
    uint8_t data[6];
    
    // Read gyroscope data registers
    for (int i = 0; i < 6; i++) {
        esp_err_t ret = bmi088_read_gyro_reg(BMI088_GYRO_RATE_X_LSB + i, &data[i]);
        if (ret != ESP_OK) return ret;
    }
    
    // Convert raw data to rad/s (±500°/s range, 16-bit)
    int16_t raw_x = (int16_t)((data[1] << 8) | data[0]);
    int16_t raw_y = (int16_t)((data[3] << 8) | data[2]);
    int16_t raw_z = (int16_t)((data[5] << 8) | data[4]);
    
    float deg_to_rad = M_PI / 180.0f;
    *gyro_x = (float)raw_x * 500.0f / 32768.0f * deg_to_rad;
    *gyro_y = (float)raw_y * 500.0f / 32768.0f * deg_to_rad;
    *gyro_z = (float)raw_z * 500.0f / 32768.0f * deg_to_rad;
    
    return ESP_OK;
}

static esp_err_t calibrate_imu(void) {
    const int calibration_samples = 1000;
    float acc_sum[3] = {0, 0, 0};
    float gyro_sum[3] = {0, 0, 0};
    
    ESP_LOGI(TAG, "Starting IMU calibration (keep device stationary)...");
    
    // Collect samples for calibration
    for (int i = 0; i < calibration_samples; i++) {
        float acc_x, acc_y, acc_z, temp;
        float gyro_x, gyro_y, gyro_z;
        
        if (bmi088_read_acc_data(&acc_x, &acc_y, &acc_z, &temp) == ESP_OK &&
            bmi088_read_gyro_data(&gyro_x, &gyro_y, &gyro_z) == ESP_OK) {
            
            acc_sum[0] += acc_x;
            acc_sum[1] += acc_y;
            acc_sum[2] += acc_z;
            
            gyro_sum[0] += gyro_x;
            gyro_sum[1] += gyro_y;
            gyro_sum[2] += gyro_z;
        }
        
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    
    // Calculate offsets
    imu_state.gyro_offset[0] = gyro_sum[0] / calibration_samples;
    imu_state.gyro_offset[1] = gyro_sum[1] / calibration_samples;
    imu_state.gyro_offset[2] = gyro_sum[2] / calibration_samples;
    
    // For accelerometer, only X and Y should be zero, Z should be ±9.81
    imu_state.acc_offset[0] = acc_sum[0] / calibration_samples;
    imu_state.acc_offset[1] = acc_sum[1] / calibration_samples;
    float avg_z = acc_sum[2] / calibration_samples;
    imu_state.acc_offset[2] = avg_z - (avg_z > 0 ? 9.81f : -9.81f);
    
    imu_state.calibrated = true;
    
    ESP_LOGI(TAG, "IMU calibration complete:");
    ESP_LOGI(TAG, "Gyro offsets: [%.4f, %.4f, %.4f] rad/s", 
             imu_state.gyro_offset[0], imu_state.gyro_offset[1], imu_state.gyro_offset[2]);
    ESP_LOGI(TAG, "Acc offsets: [%.4f, %.4f, %.4f] m/s²", 
             imu_state.acc_offset[0], imu_state.acc_offset[1], imu_state.acc_offset[2]);
    
    return ESP_OK;
}

static esp_err_t bmi088_write_acc_reg(uint8_t reg, uint8_t value) {
    spi_transaction_t t = {
        .length = 16,
        .tx_buffer = (uint8_t[]){reg, value},
        .rx_buffer = NULL,
    };
    
    return spi_device_transmit(imu_state.acc_spi, &t);
}

static esp_err_t bmi088_write_gyro_reg(uint8_t reg, uint8_t value) {
    spi_transaction_t t = {
        .length = 16,
        .tx_buffer = (uint8_t[]){reg, value},
        .rx_buffer = NULL,
    };
    
    return spi_device_transmit(imu_state.gyro_spi, &t);
}

static esp_err_t bmi088_read_acc_reg(uint8_t reg, uint8_t *value) {
    uint8_t tx_data[2] = {reg | 0x80, 0x00}; // Set read bit
    uint8_t rx_data[2];
    
    spi_transaction_t t = {
        .length = 16,
        .tx_buffer = tx_data,
        .rx_buffer = rx_data,
    };
    
    esp_err_t ret = spi_device_transmit(imu_state.acc_spi, &t);
    if (ret == ESP_OK) {
        *value = rx_data[1];
    }
    
    return ret;
}

static esp_err_t bmi088_read_gyro_reg(uint8_t reg, uint8_t *value) {
    uint8_t tx_data[2] = {reg | 0x80, 0x00}; // Set read bit
    uint8_t rx_data[2];
    
    spi_transaction_t t = {
        .length = 16,
        .tx_buffer = tx_data,
        .rx_buffer = rx_data,
    };
    
    esp_err_t ret = spi_device_transmit(imu_state.gyro_spi, &t);
    if (ret == ESP_OK) {
        *value = rx_data[1];
    }
    
    return ret;
}
