#include "msp_protocol.h"
#include <string.h>
#include <esp_log.h>
#include <driver/uart.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>
#include <esp_timer.h>

static const char *TAG = "MSP_Protocol";

// MSP Protocol State
static struct {
    uart_port_t uart_port;
    int tx_pin;
    int rx_pin;
    bool initialized;
    msp_stats_t stats;
    SemaphoreHandle_t tx_mutex;
    SemaphoreHandle_t rx_mutex;
    TaskHandle_t rx_task;
    volatile bool task_running;
    uint64_t last_message_time;
    uint32_t timeout_ms;
    QueueHandle_t message_queue;
} msp_state = {0};

// MSP Parser State
static struct {
    msp_parser_state_t state;
    uint8_t direction;
    uint8_t size;
    uint8_t command;
    uint8_t payload[256];
    uint8_t payload_index;
    uint8_t checksum;
    uint8_t calculated_checksum;
} parser_state = {0};

// Forward declarations
static void msp_rx_task(void *param);
static esp_err_t msp_send_raw_message(const msp_message_t *message);
static uint8_t msp_calculate_checksum(const msp_message_t *message);
static void msp_reset_parser(void);
static esp_err_t msp_parse_byte(uint8_t byte, msp_message_t *message);

esp_err_t msp_protocol_init_config(const msp_config_t *config) {
    if (!config) {
        ESP_LOGE(TAG, "MSP config is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (msp_state.initialized) {
        ESP_LOGW(TAG, "MSP protocol already initialized");
        return ESP_OK;
    }
    
    // Validate configuration
    if (config->uart_port >= UART_NUM_MAX || config->baud_rate < 9600) {
        ESP_LOGE(TAG, "Invalid MSP configuration parameters");
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "Initializing MSP protocol on UART%d at %d baud", 
             config->uart_port, config->baud_rate);
    
    // Store configuration
    msp_state.uart_port = config->uart_port;
    msp_state.tx_pin = config->tx_pin;
    msp_state.rx_pin = config->rx_pin;
    msp_state.timeout_ms = config->timeout_ms > 0 ? config->timeout_ms : 5000; // 5s default
    
    // Configure UART
    uart_config_t uart_config = {
        .baud_rate = config->baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    ESP_ERROR_CHECK(uart_driver_install(msp_state.uart_port, 2048, 2048, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(msp_state.uart_port, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(msp_state.uart_port, msp_state.tx_pin, msp_state.rx_pin, 
                                  UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    
    // Create synchronization primitives
    msp_state.tx_mutex = xSemaphoreCreateMutex();
    msp_state.rx_mutex = xSemaphoreCreateMutex();
    if (!msp_state.tx_mutex || !msp_state.rx_mutex) {
        ESP_LOGE(TAG, "Failed to create MSP mutexes");
        return ESP_ERR_NO_MEM;
    }
    
    // Create message queue
    msp_state.message_queue = xQueueCreate(10, sizeof(msp_message_t));
    if (!msp_state.message_queue) {
        ESP_LOGE(TAG, "Failed to create MSP message queue");
        return ESP_ERR_NO_MEM;
    }
    
    // Initialize parser
    msp_reset_parser();
    
    // Create RX task
    msp_state.task_running = true;
    BaseType_t task_ret = xTaskCreate(msp_rx_task, "msp_rx", 4096, NULL, 5, &msp_state.rx_task);
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create MSP RX task");
        msp_state.task_running = false;
        return ESP_ERR_NO_MEM;
    }
    
    msp_state.initialized = true;
    // Don't set last_message_time here - it should only be set when actual messages are received
    // msp_state.last_message_time = esp_timer_get_time();
    
    ESP_LOGI(TAG, "MSP protocol initialized successfully");
    
    return ESP_OK;
}

esp_err_t msp_protocol_deinit(void) {
    if (!msp_state.initialized) {
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Deinitializing MSP protocol...");
    
    // Stop RX task
    msp_state.task_running = false;
    if (msp_state.rx_task) {
        vTaskDelete(msp_state.rx_task);
        msp_state.rx_task = NULL;
    }
    
    // Clean up UART
    uart_driver_delete(msp_state.uart_port);
    
    // Clean up synchronization primitives
    if (msp_state.tx_mutex) {
        vSemaphoreDelete(msp_state.tx_mutex);
        msp_state.tx_mutex = NULL;
    }
    if (msp_state.rx_mutex) {
        vSemaphoreDelete(msp_state.rx_mutex);
        msp_state.rx_mutex = NULL;
    }
    if (msp_state.message_queue) {
        vQueueDelete(msp_state.message_queue);
        msp_state.message_queue = NULL;
    }
    
    msp_state.initialized = false;
    ESP_LOGI(TAG, "MSP protocol deinitialized");
    
    return ESP_OK;
}

esp_err_t msp_send_raw_rc(const uint16_t *channels, uint8_t num_channels) {
    if (!msp_state.initialized) return ESP_ERR_INVALID_STATE;
    if (!channels || num_channels == 0 || num_channels > 16) return ESP_ERR_INVALID_ARG;
    
    uint8_t payload[32]; // Max 16 channels * 2 bytes
    size_t payload_size = num_channels * 2;
    
    for (uint8_t i = 0; i < num_channels; i++) {
        payload[i * 2] = channels[i] & 0xFF;
        payload[i * 2 + 1] = channels[i] >> 8;
    }
    
    return msp_send_message(MSP_SET_RAW_RC, payload, payload_size);
}

esp_err_t msp_send_set_wp(const msp_waypoint_t *waypoint, uint8_t wp_number) {
    if (!msp_state.initialized) return ESP_ERR_INVALID_STATE;
    if (!waypoint) return ESP_ERR_INVALID_ARG;
    
    uint8_t payload[21]; // waypoint structure size
    payload[0] = wp_number;
    payload[1] = waypoint->action;
    payload[2] = waypoint->flag;
    
    // Latitude (little endian)
    payload[3] = waypoint->lat & 0xFF;
    payload[4] = (waypoint->lat >> 8) & 0xFF;
    payload[5] = (waypoint->lat >> 16) & 0xFF;
    payload[6] = (waypoint->lat >> 24) & 0xFF;
    
    // Longitude (little endian)
    payload[7] = waypoint->lon & 0xFF;
    payload[8] = (waypoint->lon >> 8) & 0xFF;
    payload[9] = (waypoint->lon >> 16) & 0xFF;
    payload[10] = (waypoint->lon >> 24) & 0xFF;
    
    // Altitude (little endian)
    payload[11] = waypoint->alt & 0xFF;
    payload[12] = (waypoint->alt >> 8) & 0xFF;
    payload[13] = (waypoint->alt >> 16) & 0xFF;
    payload[14] = (waypoint->alt >> 24) & 0xFF;
    
    // Parameters
    payload[15] = waypoint->p1 & 0xFF;
    payload[16] = waypoint->p1 >> 8;
    payload[17] = waypoint->p2 & 0xFF;
    payload[18] = waypoint->p2 >> 8;
    payload[19] = waypoint->p3 & 0xFF;
    payload[20] = waypoint->p3 >> 8;
    
    return msp_send_message(MSP_SET_WP, payload, sizeof(payload));
}

esp_err_t msp_send_set_nav_config(uint8_t mode) {
    if (!msp_state.initialized) return ESP_ERR_INVALID_STATE;
    
    uint8_t payload[1] = {mode};
    return msp_send_message(MSP_SET_NAV_CONFIG, payload, sizeof(payload));
}

esp_err_t msp_send_set_pid(const uint8_t *pid_data, size_t data_size) {
    if (!msp_state.initialized) return ESP_ERR_INVALID_STATE;
    if (!pid_data || data_size == 0 || data_size > 255) return ESP_ERR_INVALID_ARG;
    
    return msp_send_message(MSP_SET_PID, pid_data, data_size);
}

esp_err_t msp_send_set_rc_tuning(const uint8_t *tuning_data, size_t data_size) {
    if (!msp_state.initialized) return ESP_ERR_INVALID_STATE;
    if (!tuning_data || data_size == 0 || data_size > 255) return ESP_ERR_INVALID_ARG;
    
    return msp_send_message(MSP_SET_RC_TUNING, tuning_data, data_size);
}

esp_err_t msp_send_set_misc(const uint8_t *misc_data, size_t data_size) {
    if (!msp_state.initialized) return ESP_ERR_INVALID_STATE;
    if (!misc_data || data_size == 0 || data_size > 255) return ESP_ERR_INVALID_ARG;
    
    return msp_send_message(MSP_SET_MISC, misc_data, data_size);
}

esp_err_t msp_send_set_gps_rescue(const uint8_t *rescue_data, size_t data_size) {
    if (!msp_state.initialized) return ESP_ERR_INVALID_STATE;
    if (!rescue_data || data_size == 0 || data_size > 255) return ESP_ERR_INVALID_ARG;
    
    return msp_send_message(MSP_SET_GPS_RESCUE, rescue_data, data_size);
}

esp_err_t msp_send_emergency_stop(void) {
    if (!msp_state.initialized) return ESP_ERR_INVALID_STATE;
    
    // Send zero throttle and set emergency stop flag
    uint16_t channels[8] = {1500, 1500, 900, 1500, 1500, 1500, 1500, 2000}; // Throttle low, emergency flag
    return msp_send_raw_rc(channels, 8);
}

esp_err_t msp_request_status(void) {
    if (!msp_state.initialized) return ESP_ERR_INVALID_STATE;
    
    // Clear any pending messages before sending status request
    xQueueReset(msp_state.message_queue);
    
    // Send status request
    esp_err_t ret = msp_send_message(MSP_STATUS, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send MSP status request: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Wait for response with timeout
    msp_message_t response;
    ret = msp_receive_message(&response, 2000); // 2 second timeout
    if (ret == ESP_OK) {
        // Check if we received a valid status response
        if (response.command == MSP_STATUS && response.direction == MSP_DIRECTION_FROM_FC) {
            ESP_LOGD(TAG, "MSP status response received - flight controller connected");
            return ESP_OK;
        } else {
            ESP_LOGW(TAG, "MSP received unexpected response: cmd=%d, dir=%d", 
                     response.command, response.direction);
            return ESP_ERR_INVALID_RESPONSE;
        }
    } else if (ret == ESP_ERR_TIMEOUT) {
        ESP_LOGW(TAG, "MSP status request timeout - flight controller not responding");
        return ESP_ERR_TIMEOUT;
    } else {
        ESP_LOGE(TAG, "MSP receive error: %s", esp_err_to_name(ret));
        return ret;
    }
}

esp_err_t msp_request_nav_status(void) {
    return msp_send_message(MSP_NAV_STATUS, NULL, 0);
}

esp_err_t msp_request_attitude(void) {
    return msp_send_message(MSP_ATTITUDE, NULL, 0);
}

esp_err_t msp_request_altitude(void) {
    return msp_send_message(MSP_ALTITUDE, NULL, 0);
}

esp_err_t msp_request_raw_gps(void) {
    return msp_send_message(MSP_RAW_GPS, NULL, 0);
}

esp_err_t msp_request_comp_gps(void) {
    return msp_send_message(MSP_COMP_GPS, NULL, 0);
}

esp_err_t msp_request_analog(void) {
    return msp_send_message(MSP_ANALOG, NULL, 0);
}

esp_err_t msp_request_wp(uint8_t wp_number) {
    uint8_t payload[1] = {wp_number};
    return msp_send_message(MSP_WP, payload, sizeof(payload));
}

esp_err_t msp_get_stats(msp_stats_t *stats) {
    if (!stats) return ESP_ERR_INVALID_ARG;
    if (!msp_state.initialized) return ESP_ERR_INVALID_STATE;
    
    xSemaphoreTake(msp_state.rx_mutex, portMAX_DELAY);
    memcpy(stats, &msp_state.stats, sizeof(msp_stats_t));
    xSemaphoreGive(msp_state.rx_mutex);
    
    return ESP_OK;
}

bool msp_is_connected(void) {
    if (!msp_state.initialized) return false;
    
    uint64_t current_time = esp_timer_get_time();
    uint64_t time_since_last_msg = current_time - msp_state.last_message_time;
    
    // Consider connected if we received a message within the last 2 seconds
    return (time_since_last_msg < 2000000ULL);
}

esp_err_t msp_reset_connection(void) {
    if (!msp_state.initialized) return ESP_ERR_INVALID_STATE;
    
    ESP_LOGI(TAG, "Resetting MSP connection...");
    
    // Reset parser state
    msp_reset_parser();
    
    // Clear message queue
    xQueueReset(msp_state.message_queue);
    
    // Reset statistics
    memset(&msp_state.stats, 0, sizeof(msp_stats_t));
    
    return ESP_OK;
}

esp_err_t msp_nav_set_mode(uint8_t mode) {
    return msp_send_set_nav_config(mode);
}

esp_err_t msp_nav_set_home_position(double lat, double lon, float alt) {
    if (!msp_state.initialized) return ESP_ERR_INVALID_STATE;
    
    // Convert to waypoint format
    msp_waypoint_t home_wp = {
        .action = 1, // HOME waypoint
        .flag = 0,
        .lat = (int32_t)(lat * 1e7),
        .lon = (int32_t)(lon * 1e7),
        .alt = (int32_t)(alt * 100), // Convert to cm
        .p1 = 0,
        .p2 = 0,
        .p3 = 0
    };
    
    return msp_send_set_wp(&home_wp, 0); // Home is waypoint 0
}

esp_err_t msp_nav_goto_position(double lat, double lon, float alt, uint16_t hold_time) {
    if (!msp_state.initialized) return ESP_ERR_INVALID_STATE;
    
    // Convert to waypoint format
    msp_waypoint_t goto_wp = {
        .action = 2, // GOTO waypoint
        .flag = 0,
        .lat = (int32_t)(lat * 1e7),
        .lon = (int32_t)(lon * 1e7),
        .alt = (int32_t)(alt * 100), // Convert to cm
        .p1 = hold_time,
        .p2 = 0,
        .p3 = 0
    };
    
    return msp_send_set_wp(&goto_wp, 1); // Use waypoint 1 for goto
}

esp_err_t msp_nav_return_to_home(void) {
    return msp_send_set_nav_config(NAV_MODE_RTH);
}

esp_err_t msp_nav_land_here(void) {
    if (!msp_state.initialized) return ESP_ERR_INVALID_STATE;
    
    // Send land command - set throttle low and switch to land mode
    uint16_t channels[8] = {1500, 1500, 900, 1500, 1500, 1500, 1500, 1500};
    return msp_send_raw_rc(channels, 8);
}

esp_err_t msp_nav_takeoff(float altitude) {
    if (!msp_state.initialized) return ESP_ERR_INVALID_STATE;
    
    // Send takeoff command - set throttle high and switch to takeoff mode
    uint16_t channels[8] = {1500, 1500, 1800, 1500, 1500, 1500, 1500, 1500};
    return msp_send_raw_rc(channels, 8);
}

esp_err_t msp_send_message(uint8_t command, const uint8_t *payload, uint16_t payload_size) {
    if (!msp_state.initialized) return ESP_ERR_INVALID_STATE;
    if (payload_size > 255) return ESP_ERR_INVALID_ARG;
    
    msp_message_t message = {
        .direction = MSP_DIRECTION_TO_FC,
        .size = (uint8_t)payload_size,
        .command = command,
        .checksum = 0
    };
    
    if (payload && payload_size > 0) {
        memcpy(message.payload, payload, payload_size);
    }
    
    message.checksum = msp_calculate_checksum(&message);
    
    return msp_send_raw_message(&message);
}

esp_err_t msp_receive_message(msp_message_t *message, uint32_t timeout_ms) {
    if (!message) return ESP_ERR_INVALID_ARG;
    if (!msp_state.initialized) return ESP_ERR_INVALID_STATE;
    
    if (xQueueReceive(msp_state.message_queue, message, pdMS_TO_TICKS(timeout_ms)) == pdTRUE) {
        return ESP_OK;
    }
    
    return ESP_ERR_TIMEOUT;
}

// Legacy compatibility function
esp_err_t msp_protocol_init(const msp_config_t *config) {
    return msp_protocol_init_config(config);
}

static void msp_rx_task(void *param) {
    uint8_t buffer[256];
    msp_message_t message;
    
    ESP_LOGI(TAG, "MSP RX task started");
    
    while (msp_state.task_running) {
        int len = uart_read_bytes(msp_state.uart_port, buffer, sizeof(buffer), 100 / portTICK_PERIOD_MS);
        
        if (len > 0) {
            msp_state.last_message_time = esp_timer_get_time();
            
            for (int i = 0; i < len; i++) {
                if (msp_parse_byte(buffer[i], &message) == ESP_OK) {
                    // Successfully parsed a complete message
                    msp_state.stats.messages_received++;
                    
                    // Add to message queue (don't block if queue is full)
                    if (xQueueSend(msp_state.message_queue, &message, 0) != pdTRUE) {
                        ESP_LOGW(TAG, "MSP message queue full, dropping message");
                    }
                    
                    ESP_LOGD(TAG, "MSP: Received command %d, size %d", message.command, message.size);
                }
            }
        } else {
            // Check for timeout
            uint64_t current_time = esp_timer_get_time();
            if (current_time - msp_state.last_message_time > msp_state.timeout_ms * 1000ULL) {
                msp_state.stats.timeouts++;
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    ESP_LOGI(TAG, "MSP RX task ended");
    vTaskDelete(NULL);
}

static esp_err_t msp_send_raw_message(const msp_message_t *message) {
    if (!message) return ESP_ERR_INVALID_ARG;
    
    xSemaphoreTake(msp_state.tx_mutex, portMAX_DELAY);
    
    // Calculate total message size
    size_t total_size = 6 + message->size; // header + size + command + payload + checksum
    uint8_t *buffer = malloc(total_size);
    if (!buffer) {
        xSemaphoreGive(msp_state.tx_mutex);
        return ESP_ERR_NO_MEM;
    }
    
    // Build message
    buffer[0] = message->direction;
    buffer[1] = message->size;
    buffer[2] = message->command;
    
    if (message->size > 0) {
        memcpy(&buffer[3], message->payload, message->size);
    }
    
    buffer[3 + message->size] = message->checksum;
    
    // Send message
    int written = uart_write_bytes(msp_state.uart_port, buffer, total_size);
    free(buffer);
    
    xSemaphoreGive(msp_state.tx_mutex);
    
    if (written != total_size) {
        ESP_LOGE(TAG, "Failed to send MSP message");
        return ESP_FAIL;
    }
    
    msp_state.stats.messages_sent++;
    ESP_LOGD(TAG, "MSP: Sent command %d, size %d", message->command, message->size);
    
    return ESP_OK;
}

static uint8_t msp_calculate_checksum(const msp_message_t *message) {
    uint8_t checksum = 0;
    
    checksum ^= message->size;
    checksum ^= message->command;
    
    for (uint8_t i = 0; i < message->size; i++) {
        checksum ^= message->payload[i];
    }
    
    return checksum;
}

static void msp_reset_parser(void) {
    parser_state.state = MSP_STATE_IDLE;
    parser_state.payload_index = 0;
    parser_state.calculated_checksum = 0;
}

static esp_err_t msp_parse_byte(uint8_t byte, msp_message_t *message) {
    switch (parser_state.state) {
        case MSP_STATE_IDLE:
            if (byte == MSP_DIRECTION_FROM_FC) {
                parser_state.direction = byte;
                parser_state.state = MSP_STATE_SIZE;
                parser_state.calculated_checksum = 0;
            }
            break;
            
        case MSP_STATE_DIRECTION:
            // This state is not used in current implementation
            // Direction is handled in IDLE state
            parser_state.state = MSP_STATE_IDLE;
            break;
            
        case MSP_STATE_SIZE:
            parser_state.size = byte;
            parser_state.calculated_checksum ^= byte;
            parser_state.state = MSP_STATE_COMMAND;
            break;
            
        case MSP_STATE_COMMAND:
            parser_state.command = byte;
            parser_state.calculated_checksum ^= byte;
            if (parser_state.size > 0) {
                parser_state.state = MSP_STATE_PAYLOAD;
                parser_state.payload_index = 0;
            } else {
                parser_state.state = MSP_STATE_CHECKSUM;
            }
            break;
            
        case MSP_STATE_PAYLOAD:
            parser_state.payload[parser_state.payload_index++] = byte;
            parser_state.calculated_checksum ^= byte;
            if (parser_state.payload_index >= parser_state.size) {
                parser_state.state = MSP_STATE_CHECKSUM;
            }
            break;
            
        case MSP_STATE_CHECKSUM:
            parser_state.checksum = byte;
            if (parser_state.calculated_checksum == parser_state.checksum) {
                // Valid message received
                message->direction = parser_state.direction;
                message->size = parser_state.size;
                message->command = parser_state.command;
                memcpy(message->payload, parser_state.payload, parser_state.size);
                message->checksum = parser_state.checksum;
                
                msp_reset_parser();
                return ESP_OK;
            } else {
                // Checksum error
                msp_state.stats.checksum_errors++;
                ESP_LOGW(TAG, "MSP checksum error (calc: 0x%02X, recv: 0x%02X)", 
                        parser_state.calculated_checksum, parser_state.checksum);
                msp_reset_parser();
                return ESP_ERR_INVALID_CRC;
            }
            break;
    }
    
    return ESP_ERR_NOT_FINISHED;
}
