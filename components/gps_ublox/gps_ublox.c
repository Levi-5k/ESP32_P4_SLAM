#include "gps_ublox.h"
#include "visual_slam_common_types.h"
#include <string.h>
#include <esp_log.h>
#include <driver/uart.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <esp_timer.h>

static const char *TAG = "GPS_uBlox";

// GPS state structure
static struct {
    uart_port_t uart_port;
    int tx_pin;
    int rx_pin;
    bool initialized;
    gps_data_t current_data;
    SemaphoreHandle_t data_mutex;
    TaskHandle_t parse_task;
    volatile bool task_running;
} gps_state = {0};

// UBX message parsing
#define UBX_SYNC_CHAR_1 0xB5
#define UBX_SYNC_CHAR_2 0x62
#define UBX_NAV_PVT 0x07
#define UBX_NAV_CLASS 0x01

typedef struct {
    uint8_t sync1;
    uint8_t sync2;
    uint8_t class;
    uint8_t id;
    uint16_t length;
} ubx_header_t;

typedef struct {
    uint32_t iTOW;          // GPS time of week
    uint16_t year;          // Year (UTC)
    uint8_t month;          // Month (UTC)
    uint8_t day;            // Day (UTC)
    uint8_t hour;           // Hour (UTC)
    uint8_t min;            // Minute (UTC)
    uint8_t sec;            // Seconds (UTC)
    uint8_t valid;          // Validity flags
    uint32_t tAcc;          // Time accuracy estimate
    int32_t nano;           // Fraction of second
    uint8_t fixType;        // GNSS fix type
    uint8_t flags;          // Fix status flags
    uint8_t flags2;         // Additional flags
    uint8_t numSV;          // Number of satellites
    int32_t lon;            // Longitude (1e-7 deg)
    int32_t lat;            // Latitude (1e-7 deg)
    int32_t height;         // Height above ellipsoid (mm)
    int32_t hMSL;           // Height above MSL (mm)
    uint32_t hAcc;          // Horizontal accuracy estimate (mm)
    uint32_t vAcc;          // Vertical accuracy estimate (mm)
    int32_t velN;           // NED north velocity (mm/s)
    int32_t velE;           // NED east velocity (mm/s)
    int32_t velD;           // NED down velocity (mm/s)
    int32_t gSpeed;         // Ground speed (mm/s)
    int32_t headMot;        // Heading of motion (1e-5 deg)
    uint32_t sAcc;          // Speed accuracy estimate (mm/s)
    uint32_t headAcc;       // Heading accuracy estimate (1e-5 deg)
    uint16_t pDOP;          // Position DOP (0.01)
    uint8_t flags3;         // Additional flags
    uint8_t reserved1[5];   // Reserved
    int32_t headVeh;        // Heading of vehicle (1e-5 deg)
    int16_t magDec;         // Magnetic declination (1e-2 deg)
    uint16_t magAcc;        // Magnetic declination accuracy (1e-2 deg)
} ubx_nav_pvt_t;

// Forward declarations
static void gps_parse_task(void *param);
static esp_err_t parse_ubx_message(const uint8_t *data, size_t len);
static esp_err_t configure_ubx_nav_pvt(void);
static uint16_t calculate_checksum(const uint8_t *data, size_t len);

esp_err_t sensor_fusion_init_gps(const gps_config_t *config) {
    if (!config) {
        ESP_LOGE(TAG, "GPS config is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (gps_state.initialized) {
        ESP_LOGW(TAG, "GPS already initialized");
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Initializing uBlox GPS on UART%d", config->uart_port);
    
    // Store configuration
    gps_state.uart_port = config->uart_port;
    gps_state.tx_pin = config->tx_pin;
    gps_state.rx_pin = config->rx_pin;
    
    // Configure UART
    uart_config_t uart_config = {
        .baud_rate = config->baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    ESP_ERROR_CHECK(uart_driver_install(gps_state.uart_port, 2048, 2048, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(gps_state.uart_port, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(gps_state.uart_port, gps_state.tx_pin, gps_state.rx_pin, 
                                  UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    
    // Create mutex
    gps_state.data_mutex = xSemaphoreCreateMutex();
    if (!gps_state.data_mutex) {
        ESP_LOGE(TAG, "Failed to create GPS data mutex");
        return ESP_ERR_NO_MEM;
    }
    
    // Initialize GPS data structure
    memset(&gps_state.current_data, 0, sizeof(gps_data_t));
    
    // Configure UBX messages
    esp_err_t ret = configure_ubx_nav_pvt();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure UBX messages");
        return ret;
    }
    
    // Create parsing task
    gps_state.task_running = true;
    BaseType_t task_ret = xTaskCreate(gps_parse_task, "gps_parse", 4096, NULL, 5, &gps_state.parse_task);
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create GPS parsing task");
        gps_state.task_running = false;
        return ESP_ERR_NO_MEM;
    }
    
    gps_state.initialized = true;
    ESP_LOGI(TAG, "GPS uBlox initialized successfully");
    
    return ESP_OK;
}

esp_err_t gps_get_data(gps_data_t *data) {
    if (!data) return ESP_ERR_INVALID_ARG;
    if (!gps_state.initialized) return ESP_ERR_INVALID_STATE;
    
    xSemaphoreTake(gps_state.data_mutex, portMAX_DELAY);
    memcpy(data, &gps_state.current_data, sizeof(gps_data_t));
    xSemaphoreGive(gps_state.data_mutex);
    
    return ESP_OK;
}

static void gps_parse_task(void *param) {
    uint8_t buffer[512];
    uint8_t ubx_buffer[512];
    size_t ubx_pos = 0;
    bool in_ubx_message = false;
    uint16_t expected_length = 0;
    
    ESP_LOGI(TAG, "GPS parsing task started");
    
    while (gps_state.task_running) {
        int len = uart_read_bytes(gps_state.uart_port, buffer, sizeof(buffer), 100 / portTICK_PERIOD_MS);
        
        if (len > 0) {
            for (int i = 0; i < len; i++) {
                uint8_t byte = buffer[i];
                
                if (!in_ubx_message) {
                    // Look for UBX sync characters
                    if (ubx_pos == 0 && byte == UBX_SYNC_CHAR_1) {
                        ubx_buffer[ubx_pos++] = byte;
                    } else if (ubx_pos == 1 && byte == UBX_SYNC_CHAR_2) {
                        ubx_buffer[ubx_pos++] = byte;
                        in_ubx_message = true;
                    } else {
                        ubx_pos = 0;
                    }
                } else {
                    ubx_buffer[ubx_pos++] = byte;
                    
                    // Check if we have enough bytes for header
                    if (ubx_pos == 6) {
                        ubx_header_t *header = (ubx_header_t *)ubx_buffer;
                        expected_length = header->length + 8; // header + payload + checksum
                    }
                    
                    // Check if message is complete
                    if (ubx_pos >= 6 && ubx_pos == expected_length) {
                        parse_ubx_message(ubx_buffer, ubx_pos);
                        ubx_pos = 0;
                        in_ubx_message = false;
                    }
                    
                    // Prevent buffer overflow
                    if (ubx_pos >= sizeof(ubx_buffer)) {
                        ubx_pos = 0;
                        in_ubx_message = false;
                    }
                }
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    ESP_LOGI(TAG, "GPS parsing task ended");
    vTaskDelete(NULL);
}

static esp_err_t parse_ubx_message(const uint8_t *data, size_t len) {
    if (len < 8) return ESP_ERR_INVALID_ARG;
    
    ubx_header_t *header = (ubx_header_t *)data;
    
    // Verify checksum
    uint16_t calc_checksum = calculate_checksum(data + 2, len - 4);
    uint16_t msg_checksum = (data[len-1] << 8) | data[len-2];
    
    if (calc_checksum != msg_checksum) {
        ESP_LOGW(TAG, "UBX checksum mismatch");
        return ESP_ERR_INVALID_CRC;
    }
    
    // Parse NAV-PVT message
    if (header->class == UBX_NAV_CLASS && header->id == UBX_NAV_PVT) {
        if (header->length >= sizeof(ubx_nav_pvt_t)) {
            ubx_nav_pvt_t *pvt = (ubx_nav_pvt_t *)(data + 6);
            
            xSemaphoreTake(gps_state.data_mutex, portMAX_DELAY);
            
            // Convert to standard GPS data format
            gps_state.current_data.latitude = pvt->lat * 1e-7;
            gps_state.current_data.longitude = pvt->lon * 1e-7;
            gps_state.current_data.altitude = pvt->hMSL * 1e-3f;
            gps_state.current_data.accuracy = pvt->hAcc * 1e-3f;
            gps_state.current_data.gps_fix_type = pvt->fixType;
            gps_state.current_data.satellites = pvt->numSV;
            gps_state.current_data.timestamp_us = esp_timer_get_time();
            
            xSemaphoreGive(gps_state.data_mutex);
            
            ESP_LOGD(TAG, "GPS: %.7f, %.7f, %.2fm, %d sats, fix:%d", 
                     gps_state.current_data.latitude,
                     gps_state.current_data.longitude,
                     gps_state.current_data.altitude,
                     gps_state.current_data.satellites,
                     gps_state.current_data.gps_fix_type);
        }
    }
    
    return ESP_OK;
}

static esp_err_t configure_ubx_nav_pvt(void) {
    // UBX-CFG-MSG to enable NAV-PVT messages
    uint8_t cfg_msg[] = {
        0xB5, 0x62,           // sync chars
        0x06, 0x01,           // CFG-MSG
        0x03, 0x00,           // length
        0x01, 0x07,           // NAV-PVT
        0x01,                 // rate
        0x13, 0x51            // checksum
    };
    
    int written = uart_write_bytes(gps_state.uart_port, cfg_msg, sizeof(cfg_msg));
    if (written != sizeof(cfg_msg)) {
        ESP_LOGE(TAG, "Failed to write UBX configuration");
        return ESP_FAIL;
    }
    
    // Set update rate to 5Hz
    uint8_t cfg_rate[] = {
        0xB5, 0x62,           // sync chars
        0x06, 0x08,           // CFG-RATE
        0x06, 0x00,           // length
        0xC8, 0x00,           // measRate = 200ms (5Hz)
        0x01, 0x00,           // navRate = 1
        0x01, 0x00,           // timeRef = GPS time
        0xDE, 0x6A            // checksum
    };
    
    written = uart_write_bytes(gps_state.uart_port, cfg_rate, sizeof(cfg_rate));
    if (written != sizeof(cfg_rate)) {
        ESP_LOGE(TAG, "Failed to write rate configuration");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "UBX configuration sent");
    return ESP_OK;
}

static uint16_t calculate_checksum(const uint8_t *data, size_t len) {
    uint8_t ck_a = 0, ck_b = 0;
    
    for (size_t i = 0; i < len; i++) {
        ck_a += data[i];
        ck_b += ck_a;
    }
    
    return (ck_b << 8) | ck_a;
}
