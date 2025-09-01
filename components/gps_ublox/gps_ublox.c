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

// Legacy compatibility function
esp_err_t gps_ublox_init_legacy(const gps_ublox_config_t *config) {
    return gps_ublox_init(config);
}

static struct {
    uart_port_t uart_port;
    int tx_pin;
    int rx_pin;
    uint32_t update_rate_hz;
    bool initialized;
    gps_ublox_data_t current_data;
    gps_stats_t stats;
    SemaphoreHandle_t data_mutex;
    TaskHandle_t parse_task;
    volatile bool task_running;
    uint64_t last_message_time;
    uint32_t timeout_ms;
} gps_state = {0};

// UBX message parsing constants
#define UBX_SYNC_CHAR_1 0xB5
#define UBX_SYNC_CHAR_2 0x62
#define UBX_NAV_CLASS 0x01
#define UBX_NAV_PVT 0x07
#define UBX_ACK_CLASS 0x05
#define UBX_ACK_ACK 0x01
#define UBX_ACK_NAK 0x00
#define UBX_CFG_CLASS 0x06
#define UBX_CFG_MSG 0x01
#define UBX_CFG_RATE 0x08
#define UBX_CFG_RST 0x04

// UBX message header
typedef struct {
    uint8_t sync1;
    uint8_t sync2;
    uint8_t class;
    uint8_t id;
    uint16_t length;
} ubx_header_t;

// UBX NAV-PVT message structure
typedef struct {
    uint32_t iTOW;          // GPS time of week (ms)
    uint16_t year;          // Year (UTC)
    uint8_t month;          // Month (UTC)
    uint8_t day;            // Day (UTC)
    uint8_t hour;           // Hour (UTC)
    uint8_t min;            // Minute (UTC)
    uint8_t sec;            // Seconds (UTC)
    uint8_t valid;          // Validity flags
    uint32_t tAcc;          // Time accuracy estimate (ns)
    int32_t nano;           // Fraction of second (ns)
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
    uint16_t flags3;        // Additional flags
    uint8_t reserved1[5];   // Reserved
    int32_t headVeh;        // Heading of vehicle (1e-5 deg)
    int16_t magDec;         // Magnetic declination (1e-2 deg)
    uint16_t magAcc;        // Magnetic declination accuracy (1e-2 deg)
} ubx_nav_pvt_t;

// Forward declarations
static void gps_parse_task(void *param);
static esp_err_t parse_ubx_message(const uint8_t *data, size_t len);
static esp_err_t configure_ubx_nav_pvt(void);
static esp_err_t send_ubx_message(uint8_t class, uint8_t id, const uint8_t *payload, uint16_t payload_len);
static uint16_t calculate_checksum(const uint8_t *data, size_t len);
static void update_gps_stats(const ubx_nav_pvt_t *pvt);
static gps_fix_quality_t convert_ubx_fix_type(uint8_t fix_type);
static gps_status_t convert_ubx_flags(uint8_t flags);

esp_err_t gps_ublox_init(const gps_ublox_config_t *config) {
    if (!config) {
        ESP_LOGE(TAG, "GPS config is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (gps_state.initialized) {
        ESP_LOGW(TAG, "GPS already initialized");
        return ESP_OK;
    }
    
    // Validate configuration
    if (config->uart_port >= UART_NUM_MAX || config->baud_rate < 9600 || 
        config->update_rate_hz < 1 || config->update_rate_hz > 10) {
        ESP_LOGE(TAG, "Invalid GPS configuration parameters");
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "Initializing uBlox GPS on UART%d at %d baud, %d Hz update rate", 
             config->uart_port, config->baud_rate, config->update_rate_hz);
    
    // Store configuration
    gps_state.uart_port = config->uart_port;
    gps_state.tx_pin = config->tx_pin;
    gps_state.rx_pin = config->rx_pin;
    gps_state.update_rate_hz = config->update_rate_hz;
    gps_state.timeout_ms = config->timeout_ms > 0 ? config->timeout_ms : 30000; // 30s default
    
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
    memset(&gps_state.stats, 0, sizeof(gps_stats_t));
    gps_state.current_data.status = GPS_STATUS_DISCONNECTED;
    gps_state.current_data.fix_quality = GPS_FIX_NONE;
    gps_state.stats.hdop_min = 99.9f;
    
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
    // Don't set last_message_time here - it should only be set when actual messages are received
    // gps_state.last_message_time = esp_timer_get_time();
    
    ESP_LOGI(TAG, "GPS uBlox initialized successfully");
    
    return ESP_OK;
}

esp_err_t gps_ublox_deinit(void) {
    if (!gps_state.initialized) {
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Deinitializing GPS...");
    
    // Stop parsing task
    gps_state.task_running = false;
    if (gps_state.parse_task) {
        vTaskDelete(gps_state.parse_task);
        gps_state.parse_task = NULL;
    }
    
    // Clean up UART
    uart_driver_delete(gps_state.uart_port);
    
    // Clean up mutex
    if (gps_state.data_mutex) {
        vSemaphoreDelete(gps_state.data_mutex);
        gps_state.data_mutex = NULL;
    }
    
    gps_state.initialized = false;
    ESP_LOGI(TAG, "GPS deinitialized");
    
    return ESP_OK;
}

esp_err_t gps_ublox_get_data(gps_ublox_data_t *data) {
    if (!data) return ESP_ERR_INVALID_ARG;
    if (!gps_state.initialized) return ESP_ERR_INVALID_STATE;
    
    xSemaphoreTake(gps_state.data_mutex, portMAX_DELAY);
    memcpy(data, &gps_state.current_data, sizeof(gps_data_t));
    xSemaphoreGive(gps_state.data_mutex);
    
    return ESP_OK;
}

esp_err_t gps_ublox_get_stats(gps_stats_t *stats) {
    if (!stats) return ESP_ERR_INVALID_ARG;
    if (!gps_state.initialized) return ESP_ERR_INVALID_STATE;
    
    xSemaphoreTake(gps_state.data_mutex, portMAX_DELAY);
    memcpy(stats, &gps_state.stats, sizeof(gps_stats_t));
    xSemaphoreGive(gps_state.data_mutex);
    
    return ESP_OK;
}

esp_err_t gps_ublox_reset(void) {
    if (!gps_state.initialized) return ESP_ERR_INVALID_STATE;
    
    ESP_LOGI(TAG, "Resetting GPS module...");
    
    // Send cold start command
    uint8_t cold_start_payload[] = {0xFF, 0xFF, 0x00, 0x00};
    esp_err_t ret = send_ubx_message(UBX_CFG_CLASS, UBX_CFG_RST, cold_start_payload, sizeof(cold_start_payload));
    
    if (ret == ESP_OK) {
        // Reset local state
        xSemaphoreTake(gps_state.data_mutex, portMAX_DELAY);
        memset(&gps_state.current_data, 0, sizeof(gps_data_t));
        memset(&gps_state.stats, 0, sizeof(gps_stats_t));
        gps_state.current_data.status = GPS_STATUS_DISCONNECTED;
        gps_state.current_data.fix_quality = GPS_FIX_NONE;
        gps_state.stats.hdop_min = 99.9f;
        xSemaphoreGive(gps_state.data_mutex);
        
        ESP_LOGI(TAG, "GPS reset command sent");
    }
    
    return ret;
}

esp_err_t gps_ublox_set_update_rate(uint32_t rate_hz) {
    if (!gps_state.initialized) return ESP_ERR_INVALID_STATE;
    if (rate_hz < 1 || rate_hz > 10) return ESP_ERR_INVALID_ARG;
    
    uint16_t meas_rate = 1000 / rate_hz; // Convert Hz to milliseconds
    
    uint8_t rate_payload[] = {
        (uint8_t)(meas_rate & 0xFF),        // measRate LSB
        (uint8_t)(meas_rate >> 8),          // measRate MSB
        0x01, 0x00,                         // navRate = 1
        0x01, 0x00                          // timeRef = GPS time
    };
    
    esp_err_t ret = send_ubx_message(UBX_CFG_CLASS, UBX_CFG_RATE, rate_payload, sizeof(rate_payload));
    if (ret == ESP_OK) {
        gps_state.update_rate_hz = rate_hz;
        ESP_LOGI(TAG, "GPS update rate set to %d Hz", rate_hz);
    }
    
    return ret;
}

esp_err_t gps_ublox_force_cold_start(void) {
    return gps_ublox_reset();
}

esp_err_t gps_ublox_save_configuration(void) {
    if (!gps_state.initialized) return ESP_ERR_INVALID_STATE;
    
    // Send save configuration command
    uint8_t save_payload[] = {
        0x00, 0x00, 0x00, 0x00,   // clearMask
        0xFF, 0xFF, 0x00, 0x00,   // saveMask (save all)
        0x00, 0x00, 0x00, 0x00,   // loadMask
        0x01                        // deviceMask (BBR)
    };
    
    esp_err_t ret = send_ubx_message(UBX_CFG_CLASS, 0x09, save_payload, sizeof(save_payload));
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "GPS configuration saved to flash");
    }
    
    return ret;
}

bool gps_ublox_is_connected(void) {
    if (!gps_state.initialized) return false;
    
    uint64_t current_time = esp_timer_get_time();
    uint64_t time_since_last_msg = current_time - gps_state.last_message_time;
    
    // Consider connected if we received a message within the last 2 seconds
    return (time_since_last_msg < 2000000ULL);
}

bool gps_ublox_has_fix(void) {
    if (!gps_state.initialized) return false;
    
    xSemaphoreTake(gps_state.data_mutex, portMAX_DELAY);
    bool has_fix = (gps_state.current_data.fix_valid && 
                   gps_state.current_data.fix_quality >= GPS_FIX_2D);
    xSemaphoreGive(gps_state.data_mutex);
    
    return has_fix;
}

// Legacy compatibility function
esp_err_t gps_ublox_init_compat(const gps_config_t *config) {
    // Convert legacy config to new format
    gps_ublox_config_t ublox_config = {
        .uart_port = config->uart_port,
        .baud_rate = config->baud_rate,
        .tx_pin = config->tx_pin,
        .rx_pin = config->rx_pin,
        .update_rate_hz = 5,           // Default 5Hz
        .enable_sbas = true,           // Enable SBAS by default
        .enable_differential = false,  // No differential by default
        .timeout_ms = 30000            // 30 second timeout
    };
    
    return gps_ublox_init(&ublox_config);
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
            gps_state.last_message_time = esp_timer_get_time();
            
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
                        ESP_LOGW(TAG, "UBX buffer overflow, resetting parser");
                        ubx_pos = 0;
                        in_ubx_message = false;
                    }
                }
            }
        } else {
            // Check for GPS timeout
            uint64_t current_time = esp_timer_get_time();
            if (current_time - gps_state.last_message_time > gps_state.timeout_ms * 1000ULL) {
                xSemaphoreTake(gps_state.data_mutex, portMAX_DELAY);
                gps_state.current_data.status = GPS_STATUS_DISCONNECTED;
                gps_state.current_data.fix_valid = false;
                xSemaphoreGive(gps_state.data_mutex);
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
    
    // Update statistics
    gps_state.stats.messages_received++;
    
    // Verify checksum
    uint16_t calc_checksum = calculate_checksum(data + 2, len - 4);
    uint16_t msg_checksum = (data[len-1] << 8) | data[len-2];
    
    if (calc_checksum != msg_checksum) {
        gps_state.stats.checksum_errors++;
        ESP_LOGW(TAG, "UBX checksum mismatch (calc: 0x%04X, msg: 0x%04X)", calc_checksum, msg_checksum);
        return ESP_ERR_INVALID_CRC;
    }
    
    gps_state.stats.messages_parsed++;
    
    // Parse NAV-PVT message
    if (header->class == UBX_NAV_CLASS && header->id == UBX_NAV_PVT) {
        if (header->length >= sizeof(ubx_nav_pvt_t)) {
            ubx_nav_pvt_t *pvt = (ubx_nav_pvt_t *)(data + 6);
            
            xSemaphoreTake(gps_state.data_mutex, portMAX_DELAY);
            
            // Convert and store GPS data
            gps_state.current_data.latitude = pvt->lat * 1e-7;
            gps_state.current_data.longitude = pvt->lon * 1e-7;
            gps_state.current_data.altitude = pvt->height * 1e-3f;
            gps_state.current_data.geoid_separation = (pvt->height - pvt->hMSL) * 1e-3f;
            gps_state.current_data.speed = pvt->gSpeed * 1e-3f;
            gps_state.current_data.course = pvt->headMot * 1e-5f;
            gps_state.current_data.hdop = (pvt->hAcc * 1e-3f) / 5.0f; // Approximate HDOP
            gps_state.current_data.vdop = (pvt->vAcc * 1e-3f) / 5.0f; // Approximate VDOP
            gps_state.current_data.pdop = pvt->pDOP * 1e-2f;
            gps_state.current_data.satellites = pvt->numSV;
            gps_state.current_data.fix_quality = convert_ubx_fix_type(pvt->fixType);
            gps_state.current_data.status = convert_ubx_flags(pvt->flags);
            gps_state.current_data.uptime = pvt->iTOW / 1000;
            gps_state.current_data.timestamp = esp_timer_get_time();
            gps_state.current_data.fix_valid = (pvt->fixType >= 2); // 2D or 3D fix
            gps_state.current_data.differential = (pvt->flags & 0x02) != 0;
            
            update_gps_stats(pvt);
            
            xSemaphoreGive(gps_state.data_mutex);
            
            ESP_LOGD(TAG, "GPS: %.7f, %.7f, %.2fm, %d sats, fix:%d, hdop:%.1f", 
                     gps_state.current_data.latitude,
                     gps_state.current_data.longitude,
                     gps_state.current_data.altitude,
                     gps_state.current_data.satellites,
                     gps_state.current_data.fix_quality,
                     gps_state.current_data.hdop);
        }
    } else if (header->class == UBX_ACK_CLASS) {
        if (header->id == UBX_ACK_ACK) {
            ESP_LOGD(TAG, "UBX command acknowledged");
        } else if (header->id == UBX_ACK_NAK) {
            ESP_LOGW(TAG, "UBX command not acknowledged");
        }
    }
    
    return ESP_OK;
}

static esp_err_t configure_ubx_nav_pvt(void) {
    // Disable NMEA messages first
    uint8_t disable_nmea[] = {
        0xB5, 0x62,           // sync chars
        0x06, 0x00,           // CFG-MSG
        0x08, 0x00,           // length
        0xF0, 0x00,           // NMEA GGA
        0x00,                 // rate
        0xF0, 0x01,           // NMEA GLL
        0x00,                 // rate
        0xF0, 0x02,           // NMEA GSA
        0x00,                 // rate
        0xF0, 0x03,           // NMEA GSV
        0x00,                 // rate
        0xF0, 0x04,           // NMEA RMC
        0x00,                 // rate
        0xF0, 0x05,           // NMEA VTG
        0x00,                 // rate
        0x00, 0x00            // checksum
    };
    
    // Calculate checksum for disable NMEA
    uint16_t checksum = calculate_checksum(disable_nmea + 2, sizeof(disable_nmea) - 4);
    disable_nmea[sizeof(disable_nmea) - 2] = checksum & 0xFF;
    disable_nmea[sizeof(disable_nmea) - 1] = checksum >> 8;
    
    int written = uart_write_bytes(gps_state.uart_port, disable_nmea, sizeof(disable_nmea));
    if (written != sizeof(disable_nmea)) {
        ESP_LOGE(TAG, "Failed to disable NMEA messages");
        return ESP_FAIL;
    }
    
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Enable UBX NAV-PVT messages
    uint8_t enable_ubx[] = {
        0xB5, 0x62,           // sync chars
        0x06, 0x01,           // CFG-MSG
        0x03, 0x00,           // length
        0x01, 0x07,           // NAV-PVT
        0x01,                 // rate
        0x00, 0x00            // checksum
    };
    
    checksum = calculate_checksum(enable_ubx + 2, sizeof(enable_ubx) - 4);
    enable_ubx[sizeof(enable_ubx) - 2] = checksum & 0xFF;
    enable_ubx[sizeof(enable_ubx) - 1] = checksum >> 8;
    
    written = uart_write_bytes(gps_state.uart_port, enable_ubx, sizeof(enable_ubx));
    if (written != sizeof(enable_ubx)) {
        ESP_LOGE(TAG, "Failed to enable UBX messages");
        return ESP_FAIL;
    }
    
    // Set update rate
    uint16_t meas_rate = 1000 / gps_state.update_rate_hz;
    uint8_t rate_msg[] = {
        0xB5, 0x62,           // sync chars
        0x06, 0x08,           // CFG-RATE
        0x06, 0x00,           // length
        (uint8_t)(meas_rate & 0xFF),        // measRate LSB
        (uint8_t)(meas_rate >> 8),          // measRate MSB
        0x01, 0x00,           // navRate = 1
        0x01, 0x00,           // timeRef = GPS time
        0x00, 0x00            // checksum
    };
    
    checksum = calculate_checksum(rate_msg + 2, sizeof(rate_msg) - 4);
    rate_msg[sizeof(rate_msg) - 2] = checksum & 0xFF;
    rate_msg[sizeof(rate_msg) - 1] = checksum >> 8;
    
    written = uart_write_bytes(gps_state.uart_port, rate_msg, sizeof(rate_msg));
    if (written != sizeof(rate_msg)) {
        ESP_LOGE(TAG, "Failed to set update rate");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "UBX configuration sent (rate: %d Hz)", gps_state.update_rate_hz);
    return ESP_OK;
}

static esp_err_t send_ubx_message(uint8_t class, uint8_t id, const uint8_t *payload, uint16_t payload_len) {
    if (!gps_state.initialized) return ESP_ERR_INVALID_STATE;
    
    uint16_t total_len = 8 + payload_len; // header + payload + checksum
    uint8_t *message = malloc(total_len);
    if (!message) return ESP_ERR_NO_MEM;
    
    // Build message
    message[0] = UBX_SYNC_CHAR_1;
    message[1] = UBX_SYNC_CHAR_2;
    message[2] = class;
    message[3] = id;
    message[4] = payload_len & 0xFF;
    message[5] = payload_len >> 8;
    
    if (payload && payload_len > 0) {
        memcpy(message + 6, payload, payload_len);
    }
    
    // Calculate checksum
    uint16_t checksum = calculate_checksum(message + 2, payload_len + 4);
    message[total_len - 2] = checksum & 0xFF;
    message[total_len - 1] = checksum >> 8;
    
    // Send message
    int written = uart_write_bytes(gps_state.uart_port, message, total_len);
    free(message);
    
    if (written != total_len) {
        ESP_LOGE(TAG, "Failed to send UBX message");
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

static void update_gps_stats(const ubx_nav_pvt_t *pvt) {
    if (pvt->fixType >= 2) { // Valid fix
        gps_state.stats.fix_count++;
        gps_state.stats.last_fix_time = esp_timer_get_time();
        
        if (pvt->numSV > gps_state.stats.satellites_max) {
            gps_state.stats.satellites_max = pvt->numSV;
        }
        
        float hdop = (pvt->hAcc * 1e-3f) / 5.0f;
        if (hdop > 0 && hdop < gps_state.stats.hdop_min) {
            gps_state.stats.hdop_min = hdop;
        }
        
        float accuracy = pvt->hAcc * 1e-3f;
        if (accuracy > 0 && (gps_state.stats.accuracy_best == 0 || accuracy < gps_state.stats.accuracy_best)) {
            gps_state.stats.accuracy_best = accuracy;
        }
    }
    
    gps_state.stats.uptime_seconds = esp_timer_get_time() / 1000000ULL;
}

static gps_fix_quality_t convert_ubx_fix_type(uint8_t fix_type) {
    switch (fix_type) {
        case 0: return GPS_FIX_NONE;
        case 1: return GPS_FIX_DEAD_RECKONING;
        case 2: return GPS_FIX_2D;
        case 3: return GPS_FIX_3D;
        case 4: return GPS_FIX_GNSS_DEAD_RECKONING;
        case 5: return GPS_FIX_TIME_ONLY;
        default: return GPS_FIX_NONE;
    }
}

static gps_status_t convert_ubx_flags(uint8_t flags) {
    if (flags & 0x01) { // GPS fix available
        if (flags & 0x02) { // Differential correction available
            return GPS_STATUS_DGPS;
        } else {
            return GPS_STATUS_FIX_3D;
        }
    } else {
        return GPS_STATUS_CONNECTED;
    }
}

static uint16_t calculate_checksum(const uint8_t *data, size_t len) {
    uint16_t checksum = 0;
    for (size_t i = 0; i < len; i++) {
        checksum = (checksum * 256) ^ data[i];
        for (int j = 0; j < 8; j++) {
            if (checksum & 0x8000) {
                checksum = (checksum << 1) ^ 0x1021;
            } else {
                checksum <<= 1;
            }
        }
    }
    return checksum;
}
