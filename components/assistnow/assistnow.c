/*
 * AssistNow Offline Implementation
 * Downloads satellite ephemeris data from u-blox AssistNow servers
 */

#include "assistnow.h"
#include <esp_log.h>
#include <esp_http_client.h>
#include <esp_timer.h>
#include <nvs.h>
#include <nvs_flash.h>
#include <string.h>
#include <stdlib.h>
#include <sys/param.h>
#include <cJSON.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <driver/uart.h>
#include "sd_storage.h"
#include "wifi_assistnow.h"

static const char *TAG = "AssistNow";

// UBX Protocol Constants
#define UBX_SYNC_CHAR_1 0xB5
#define UBX_SYNC_CHAR_2 0x62
#define UBX_CLASS_MGA 0x13  // Message Geodetic Assistance
#define UBX_CLASS_ACK 0x05  // Acknowledgment
#define UBX_ACK_ACK 0x01    // Positive acknowledgment
#define UBX_ACK_NAK 0x00    // Negative acknowledgment

// UBX Message Structure
typedef struct {
    uint8_t sync1;      // 0xB5
    uint8_t sync2;      // 0x62
    uint8_t class_id;   // Message class
    uint8_t msg_id;     // Message ID
    uint16_t length;    // Payload length (little-endian)
    uint8_t payload[];  // Variable length payload
} __attribute__((packed)) ubx_message_t;

// UBX ACK/NAK Message Structure
typedef struct {
    uint8_t class_id;
    uint8_t msg_id;
} __attribute__((packed)) ubx_ack_t;

// Component state
static struct {
    assistnow_config_t config;
    assistnow_info_t info;
    bool initialized;
    TaskHandle_t auto_update_task;
    SemaphoreHandle_t data_mutex;
    nvs_handle_t nvs_handle;
    // WiFi configuration for downloads
    wifi_assistnow_config_t wifi_config;
    bool wifi_configured;
} assistnow_state = {0};

// Default AssistNow server URL
#define ASSISTNOW_DEFAULT_URL "https://online-live1.services.u-blox.com/GetOfflineData.ashx"

// Default GNSS constellations (matching u-blox software)
#define ASSISTNOW_DEFAULT_GNSS "gps,glo,bds,gal"

// Default parameters matching u-blox software
#define ASSISTNOW_DEFAULT_PERIOD_DAYS 4
#define ASSISTNOW_DEFAULT_RESOLUTION_HOURS 1

// NVS keys for persistent storage
#define NVS_NAMESPACE "assistnow"
#define NVS_KEY_DATA_SIZE "data_size"
#define NVS_KEY_TIMESTAMP "timestamp"
#define NVS_KEY_EXPIRY "expiry"
#define NVS_KEY_VERSION "version"

// HTTP event handler for downloading data
static esp_err_t http_event_handler(esp_http_client_event_t *evt)
{
    static int output_len = 0;
    static uint8_t *output_buffer = NULL;

    switch(evt->event_id) {
        case HTTP_EVENT_ERROR:
            ESP_LOGD(TAG, "HTTP_EVENT_ERROR");
            break;
        case HTTP_EVENT_ON_CONNECTED:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_CONNECTED");
            output_len = 0;
            if (output_buffer) {
                free(output_buffer);
                output_buffer = NULL;
            }
            break;
        case HTTP_EVENT_HEADER_SENT:
            ESP_LOGD(TAG, "HTTP_EVENT_HEADER_SENT");
            break;
        case HTTP_EVENT_ON_HEADER:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
            break;
        case HTTP_EVENT_ON_DATA:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
            if (!esp_http_client_is_chunked_response(evt->client)) {
                if (output_buffer == NULL) {
                    output_buffer = (uint8_t *)malloc(esp_http_client_get_content_length(evt->client));
                    if (output_buffer == NULL) {
                        ESP_LOGE(TAG, "Failed to allocate memory for output buffer");
                        return ESP_FAIL;
                    }
                }
                memcpy(output_buffer + output_len, evt->data, evt->data_len);
                output_len += evt->data_len;
            }
            break;
        case HTTP_EVENT_ON_FINISH:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_FINISH");
            if (output_buffer) {
                // Save the downloaded data
                assistnow_save_to_storage(output_buffer, output_len);
                free(output_buffer);
                output_buffer = NULL;
            }
            output_len = 0;
            break;
        case HTTP_EVENT_DISCONNECTED:
            ESP_LOGD(TAG, "HTTP_EVENT_DISCONNECTED");
            if (output_buffer) {
                free(output_buffer);
                output_buffer = NULL;
            }
            output_len = 0;
            break;
        case HTTP_EVENT_REDIRECT:
            ESP_LOGD(TAG, "HTTP_EVENT_REDIRECT");
            break;
    }
    return ESP_OK;
}

// Initialize NVS for AssistNow data storage
static esp_err_t init_nvs(void)
{
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &assistnow_state.nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS namespace: %s", esp_err_to_name(ret));
        return ret;
    }
    return ESP_OK;
}

// Save AssistNow data to storage
esp_err_t assistnow_save_to_storage(const uint8_t* data, size_t size)
{
    if (!assistnow_state.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Saving AssistNow data (%d bytes) to storage", size);

    // Save to NVS for metadata
    uint64_t timestamp = esp_timer_get_time() / 1000000; // Convert to seconds
    uint64_t expiry = timestamp + (assistnow_state.config.validity_hours * 3600);

    esp_err_t ret = nvs_set_u32(assistnow_state.nvs_handle, NVS_KEY_DATA_SIZE, size);
    if (ret != ESP_OK) goto cleanup;

    ret = nvs_set_u64(assistnow_state.nvs_handle, NVS_KEY_TIMESTAMP, timestamp);
    if (ret != ESP_OK) goto cleanup;

    ret = nvs_set_u64(assistnow_state.nvs_handle, NVS_KEY_EXPIRY, expiry);
    if (ret != ESP_OK) goto cleanup;

    ret = nvs_set_str(assistnow_state.nvs_handle, NVS_KEY_VERSION, "1.0");
    if (ret != ESP_OK) goto cleanup;

    ret = nvs_commit(assistnow_state.nvs_handle);
    if (ret != ESP_OK) goto cleanup;

    // Save binary data to file
    FILE* file = fopen(assistnow_state.config.data_file_path, "wb");
    if (!file) {
        ESP_LOGE(TAG, "Failed to open file for writing: %s", assistnow_state.config.data_file_path);
        return ESP_ERR_NOT_FOUND;
    }

    size_t written = fwrite(data, 1, size, file);
    fclose(file);

    if (written != size) {
        ESP_LOGE(TAG, "Failed to write complete data to file");
        return ESP_ERR_INVALID_SIZE;
    }

    // Update state
    assistnow_state.info.status = ASSISTNOW_STATUS_DATA_VALID;
    assistnow_state.info.download_timestamp = timestamp;
    assistnow_state.info.expiry_timestamp = expiry;
    assistnow_state.info.data_size_bytes = size;

    ESP_LOGI(TAG, "✅ AssistNow data saved successfully");
    return ESP_OK;

cleanup:
    ESP_LOGE(TAG, "Failed to save metadata to NVS: %s", esp_err_to_name(ret));
    return ret;
}

// Load AssistNow data from storage
esp_err_t assistnow_load_from_storage(uint8_t* data, size_t* size)
{
    if (!assistnow_state.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    FILE* file = fopen(assistnow_state.config.data_file_path, "rb");
    if (!file) {
        ESP_LOGW(TAG, "No AssistNow data file found");
        return ESP_ERR_NOT_FOUND;
    }

    fseek(file, 0, SEEK_END);
    size_t file_size = ftell(file);
    fseek(file, 0, SEEK_SET);

    if (file_size > *size) {
        ESP_LOGE(TAG, "Buffer too small for AssistNow data (%d > %d)", file_size, *size);
        fclose(file);
        return ESP_ERR_INVALID_SIZE;
    }

    size_t read = fread(data, 1, file_size, file);
    fclose(file);

    if (read != file_size) {
        ESP_LOGE(TAG, "Failed to read complete data from file");
        return ESP_ERR_INVALID_SIZE;
    }

    *size = file_size;
    return ESP_OK;
}

// Delete AssistNow data from storage
esp_err_t assistnow_delete_from_storage(void)
{
    if (!assistnow_state.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    // Delete file
    if (remove(assistnow_state.config.data_file_path) != 0) {
        ESP_LOGW(TAG, "Failed to delete AssistNow data file");
    }

    // Clear NVS data
    nvs_erase_key(assistnow_state.nvs_handle, NVS_KEY_DATA_SIZE);
    nvs_erase_key(assistnow_state.nvs_handle, NVS_KEY_TIMESTAMP);
    nvs_erase_key(assistnow_state.nvs_handle, NVS_KEY_EXPIRY);
    nvs_erase_key(assistnow_state.nvs_handle, NVS_KEY_VERSION);
    nvs_commit(assistnow_state.nvs_handle);

    // Update state
    assistnow_state.info.status = ASSISTNOW_STATUS_NO_DATA;
    assistnow_state.info.data_size_bytes = 0;

    ESP_LOGI(TAG, "AssistNow data deleted from storage");
    return ESP_OK;
}

// Download AssistNow data from server
esp_err_t assistnow_download_data(void)
{
    if (!assistnow_state.initialized || !assistnow_state.config.enabled) {
        return ESP_ERR_INVALID_STATE;
    }

    if (strlen(assistnow_state.config.token) == 0) {
        ESP_LOGW(TAG, "No AssistNow token configured");
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "📡 Downloading AssistNow data from server...");

    assistnow_state.info.status = ASSISTNOW_STATUS_DOWNLOADING;

    // Initialize WiFi if not already done
    static bool wifi_initialized = false;
    if (!wifi_initialized) {
        // Check if WiFi is already initialized (from main application)
        if (wifi_assistnow_get_status() != WIFI_ASSISTNOW_DISCONNECTED) {
            ESP_LOGI(TAG, "WiFi already configured for AssistNow downloads");
            wifi_initialized = true;
        } else if (assistnow_state.wifi_configured) {
            // Use the configured WiFi settings
            ESP_LOGI(TAG, "Using configured WiFi settings for AssistNow downloads");
            esp_err_t wifi_ret = wifi_assistnow_init(&assistnow_state.wifi_config);
            if (wifi_ret != ESP_OK) {
                if (wifi_ret == ESP_ERR_NOT_SUPPORTED) {
                    ESP_LOGW(TAG, "⚠️ WiFi not available - AssistNow downloads disabled");
                    ESP_LOGI(TAG, "ℹ️ To enable AssistNow, either:");
                    ESP_LOGI(TAG, "   1. Enable WiFi in sdkconfig (CONFIG_ESP_WIFI_ENABLED=y)");
                    ESP_LOGI(TAG, "   2. Or disable SDMMC to allow WiFi usage");
                    assistnow_state.info.status = ASSISTNOW_STATUS_ERROR;
                    return ESP_ERR_NOT_SUPPORTED;
                } else {
                    ESP_LOGE(TAG, "Failed to initialize WiFi for AssistNow: %s", esp_err_to_name(wifi_ret));
                    assistnow_state.info.status = ASSISTNOW_STATUS_ERROR;
                    return wifi_ret;
                }
            }
            wifi_initialized = true;
        } else {
            // No WiFi configuration available
            ESP_LOGW(TAG, "No WiFi configuration available for AssistNow downloads");
            ESP_LOGI(TAG, "ℹ️ Call assistnow_set_wifi_config() before attempting downloads");
            ESP_LOGI(TAG, "ℹ️ Or ensure WiFi is pre-configured in the main application");
            assistnow_state.info.status = ASSISTNOW_STATUS_ERROR;
            return ESP_ERR_INVALID_STATE;
        }
    }

    // Connect to WiFi
    ESP_LOGI(TAG, "🔗 Connecting to WiFi for AssistNow download...");
    esp_err_t wifi_ret = wifi_assistnow_connect();
    if (wifi_ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to connect to WiFi: %s", esp_err_to_name(wifi_ret));

        // Check if this is due to missing credentials
        if (wifi_ret == ESP_ERR_INVALID_ARG || wifi_ret == ESP_FAIL) {
            ESP_LOGW(TAG, "⚠️ WiFi connection failed - likely due to missing or invalid credentials");
            ESP_LOGI(TAG, "ℹ️ Ensure WiFi SSID and password are configured in system config");
            ESP_LOGI(TAG, "ℹ️ WiFi credentials should be set in the main application before AssistNow downloads");
        }

        assistnow_state.info.status = ASSISTNOW_STATUS_ERROR;
        return wifi_ret;
    }

    // Construct URL with token and enhanced parameters
    char url[512];
    const char* gnss = strlen(assistnow_state.config.gnss_constellations) > 0 ?
                      assistnow_state.config.gnss_constellations : "gps,glo,gal";
    uint32_t period = assistnow_state.config.period_days > 0 ?
                     assistnow_state.config.period_days : 4;
    uint32_t resolution = assistnow_state.config.resolution_hours > 0 ?
                         assistnow_state.config.resolution_hours : 1;

    // Use semicolon separators like u-blox software for compatibility
    // Note: format parameter removed as server defaults to MGA format
    snprintf(url, sizeof(url), "%s?token=%s;gnss=%s;period=%lu;resolution=%lu",
             assistnow_state.config.server_url,
             assistnow_state.config.token,
             gnss,
             (unsigned long)period,
             (unsigned long)resolution);

    ESP_LOGI(TAG, "📡 AssistNow download URL: %s", url);
    ESP_LOGI(TAG, "📡 Parameters: gnss=%s, period=%lu days, resolution=%lu hours", 
             gnss, (unsigned long)period, (unsigned long)resolution);

    esp_http_client_config_t config = {
        .url = url,
        .event_handler = http_event_handler,
        .timeout_ms = 30000,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) {
        ESP_LOGE(TAG, "Failed to initialize HTTP client");
        wifi_assistnow_disconnect();
        assistnow_state.info.status = ASSISTNOW_STATUS_ERROR;
        return ESP_ERR_NO_MEM;
    }

    esp_err_t err = esp_http_client_perform(client);

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "✅ AssistNow data downloaded successfully");
    } else {
        ESP_LOGE(TAG, "❌ Failed to download AssistNow data: %s", esp_err_to_name(err));
        assistnow_state.info.status = ASSISTNOW_STATUS_ERROR;
    }

    esp_http_client_cleanup(client);

    // Disconnect from WiFi to avoid SDMMC conflicts
    ESP_LOGI(TAG, "🔌 Disconnecting WiFi after download...");
    wifi_assistnow_disconnect();

    return err;
}

// Check if AssistNow data is valid
bool assistnow_is_data_valid(void)
{
    if (!assistnow_state.initialized) {
        return false;
    }

    uint64_t current_time = esp_timer_get_time() / 1000000; // Convert to seconds
    return (assistnow_state.info.status == ASSISTNOW_STATUS_DATA_VALID &&
            assistnow_state.info.expiry_timestamp > current_time);
}

// Check if download is needed
bool assistnow_is_download_needed(void)
{
    if (!assistnow_state.initialized || !assistnow_state.config.enabled) {
        return false;
    }

    if (assistnow_state.info.status == ASSISTNOW_STATUS_NO_DATA) {
        return true;
    }

    uint64_t current_time = esp_timer_get_time() / 1000000;
    uint64_t time_since_download = current_time - assistnow_state.info.download_timestamp;
    uint64_t update_interval_seconds = assistnow_state.config.update_interval_hours * 3600;

    return (time_since_download >= update_interval_seconds);
}

// Set WiFi configuration for AssistNow downloads
// This function should be called after assistnow_init() to provide WiFi credentials
// for downloading AssistNow data from u-blox servers
esp_err_t assistnow_set_wifi_config(const wifi_assistnow_config_t* wifi_config)
{
    if (!wifi_config) {
        return ESP_ERR_INVALID_ARG;
    }

    memcpy(&assistnow_state.wifi_config, wifi_config, sizeof(wifi_assistnow_config_t));
    assistnow_state.wifi_configured = true;

    ESP_LOGI(TAG, "✅ WiFi configuration set for AssistNow downloads (SSID: %s)", wifi_config->ssid);
    return ESP_OK;
}

// Initialize AssistNow component
esp_err_t assistnow_init(const assistnow_config_t* config)
{
    if (!config) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Initializing AssistNow Offline component");

    // Copy configuration
    memcpy(&assistnow_state.config, config, sizeof(assistnow_config_t));

    // Set default server URL if not provided
    if (strlen(assistnow_state.config.server_url) == 0) {
        strcpy(assistnow_state.config.server_url, ASSISTNOW_DEFAULT_URL);
    }

    // Set default GNSS constellations if not provided
    if (strlen(assistnow_state.config.gnss_constellations) == 0) {
        strcpy(assistnow_state.config.gnss_constellations, ASSISTNOW_DEFAULT_GNSS);
    }

    // Set default period and resolution if not provided
    if (assistnow_state.config.period_days == 0) {
        assistnow_state.config.period_days = ASSISTNOW_DEFAULT_PERIOD_DAYS;
    }
    if (assistnow_state.config.resolution_hours == 0) {
        assistnow_state.config.resolution_hours = ASSISTNOW_DEFAULT_RESOLUTION_HOURS;
    }

    // Set default data file path
    if (strlen(assistnow_state.config.data_file_path) == 0) {
        strcpy(assistnow_state.config.data_file_path, "/sdcard/assistnow/assistnow.ubx");
    }

    // Initialize NVS
    esp_err_t ret = init_nvs();
    if (ret != ESP_OK) {
        return ret;
    }

    // Create mutex
    assistnow_state.data_mutex = xSemaphoreCreateMutex();
    if (!assistnow_state.data_mutex) {
        ESP_LOGE(TAG, "Failed to create data mutex");
        return ESP_ERR_NO_MEM;
    }

    // Load existing data info from NVS
    uint32_t data_size = 0;
    uint64_t timestamp = 0;
    uint64_t expiry = 0;

    nvs_get_u32(assistnow_state.nvs_handle, NVS_KEY_DATA_SIZE, &data_size);
    nvs_get_u64(assistnow_state.nvs_handle, NVS_KEY_TIMESTAMP, &timestamp);
    nvs_get_u64(assistnow_state.nvs_handle, NVS_KEY_EXPIRY, &expiry);

    assistnow_state.info.data_size_bytes = data_size;
    assistnow_state.info.download_timestamp = timestamp;
    assistnow_state.info.expiry_timestamp = expiry;

    if (data_size > 0 && assistnow_is_data_valid()) {
        assistnow_state.info.status = ASSISTNOW_STATUS_DATA_VALID;
        ESP_LOGI(TAG, "✅ Loaded valid AssistNow data from storage");
    } else if (data_size > 0) {
        assistnow_state.info.status = ASSISTNOW_STATUS_DATA_EXPIRED;
        ESP_LOGW(TAG, "⚠️ AssistNow data expired, will download new data when WiFi available");
    } else {
        assistnow_state.info.status = ASSISTNOW_STATUS_NO_DATA;
        ESP_LOGI(TAG, "📭 No AssistNow data found, will download when WiFi available");
    }

    assistnow_state.initialized = true;
    ESP_LOGI(TAG, "✅ AssistNow component initialized");
    return ESP_OK;
}

// Deinitialize AssistNow component
esp_err_t assistnow_deinit(void)
{
    if (!assistnow_state.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Deinitializing AssistNow component");

    // Stop auto update task
    assistnow_stop_auto_update();

    // Close NVS
    if (assistnow_state.nvs_handle) {
        nvs_close(assistnow_state.nvs_handle);
    }

    // Delete mutex
    if (assistnow_state.data_mutex) {
        vSemaphoreDelete(assistnow_state.data_mutex);
    }

    assistnow_state.initialized = false;
    ESP_LOGI(TAG, "✅ AssistNow component deinitialized");
    return ESP_OK;
}

// Get AssistNow information
esp_err_t assistnow_get_info(assistnow_info_t* info)
{
    if (!assistnow_state.initialized || !info) {
        return ESP_ERR_INVALID_STATE;
    }

    memcpy(info, &assistnow_state.info, sizeof(assistnow_info_t));
    return ESP_OK;
}

// Force download of AssistNow data
esp_err_t assistnow_force_download(void)
{
    if (!assistnow_state.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "🔄 Forcing AssistNow data download...");
    return assistnow_download_data();
}

// Clear AssistNow data
esp_err_t assistnow_clear_data(void)
{
    if (!assistnow_state.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "🗑️ Clearing AssistNow data...");
    return assistnow_delete_from_storage();
}

// Set new configuration
esp_err_t assistnow_set_config(const assistnow_config_t* config)
{
    if (!assistnow_state.initialized || !config) {
        return ESP_ERR_INVALID_STATE;
    }

    memcpy(&assistnow_state.config, config, sizeof(assistnow_config_t));
    ESP_LOGI(TAG, "✅ AssistNow configuration updated");
    return ESP_OK;
}

// Auto update task
static void auto_update_task(void* arg)
{
    ESP_LOGI(TAG, "AssistNow auto-update task started");

    while (assistnow_state.initialized) {
        // Check if download is needed
        if (assistnow_is_download_needed()) {
            ESP_LOGI(TAG, "📡 Auto-downloading AssistNow data...");
            assistnow_download_data();
        }

        // Wait for next check interval (1 hour)
        vTaskDelay(pdMS_TO_TICKS(3600000)); // 1 hour
    }

    ESP_LOGI(TAG, "AssistNow auto-update task stopped");
    vTaskDelete(NULL);
}

// Start auto update task
esp_err_t assistnow_start_auto_update(void)
{
    if (!assistnow_state.initialized || !assistnow_state.config.auto_download) {
        return ESP_ERR_INVALID_STATE;
    }

    if (assistnow_state.auto_update_task) {
        ESP_LOGW(TAG, "Auto-update task already running");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Starting AssistNow auto-update task");

    xTaskCreate(auto_update_task, "assistnow_auto", 4096, NULL, 5, &assistnow_state.auto_update_task);
    return ESP_OK;
}

// Stop auto update task
esp_err_t assistnow_stop_auto_update(void)
{
    if (assistnow_state.auto_update_task) {
        ESP_LOGI(TAG, "Stopping AssistNow auto-update task");
        vTaskDelete(assistnow_state.auto_update_task);
        assistnow_state.auto_update_task = NULL;
    }
    return ESP_OK;
}

// Calculate UBX checksum (Fletcher-16)
static void ubx_calculate_checksum(const uint8_t* data, uint16_t length, uint8_t* ck_a, uint8_t* ck_b) {
    *ck_a = 0;
    *ck_b = 0;
    
    for (uint16_t i = 0; i < length; i++) {
        *ck_a += data[i];
        *ck_b += *ck_a;
    }
}

// Send UBX message to GPS and wait for acknowledgment
static esp_err_t send_ubx_message(uart_port_t uart_port, const uint8_t* message, uint16_t message_length, uint32_t timeout_ms) {
    // Send the message
    int written = uart_write_bytes(uart_port, (const char*)message, message_length);
    if (written != message_length) {
        ESP_LOGE(TAG, "Failed to write UBX message (%d/%d bytes)", written, message_length);
        return ESP_ERR_INVALID_RESPONSE;
    }
    
    // For ACK messages, wait for response
    if (message[2] == UBX_CLASS_ACK) {
        return ESP_OK; // Don't wait for ACK of ACK messages
    }
    
    // Wait for ACK/NAK response
    uint8_t response[10];
    uint32_t start_time = esp_timer_get_time() / 1000; // microseconds to milliseconds
    
    while ((esp_timer_get_time() / 1000 - start_time) < timeout_ms) {
        int read_bytes = uart_read_bytes(uart_port, response, sizeof(response), pdMS_TO_TICKS(10));
        
        if (read_bytes > 0) {
            // Check if this is a UBX ACK/NAK message
            if (read_bytes >= 10 && response[0] == UBX_SYNC_CHAR_1 && response[1] == UBX_SYNC_CHAR_2) {
                if (response[2] == UBX_CLASS_ACK) {
                    if (response[3] == UBX_ACK_ACK) {
                        ESP_LOGD(TAG, "✅ UBX message acknowledged");
                        return ESP_OK;
                    } else if (response[3] == UBX_ACK_NAK) {
                        ESP_LOGW(TAG, "❌ UBX message not acknowledged (NAK)");
                        return ESP_ERR_INVALID_RESPONSE;
                    }
                }
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10)); // Small delay to avoid busy waiting
    }
    
    ESP_LOGW(TAG, "Timeout waiting for UBX acknowledgment");
    return ESP_ERR_TIMEOUT;
}

// Parse and send individual UBX messages from AssistNow data
static esp_err_t parse_and_send_ubx_messages(uart_port_t uart_port, const uint8_t* data, size_t data_size) {
    size_t offset = 0;
    uint32_t messages_sent = 0;
    uint32_t messages_failed = 0;
    
    ESP_LOGI(TAG, "📤 Parsing and sending UBX messages (%d bytes)", data_size);
    
    while (offset < data_size - 8) { // Minimum UBX message size
        // Look for UBX sync characters
        if (data[offset] == UBX_SYNC_CHAR_1 && data[offset + 1] == UBX_SYNC_CHAR_2) {
            // Found potential UBX message
            uint16_t payload_length = data[offset + 4] | (data[offset + 5] << 8);
            uint16_t total_length = 8 + payload_length; // Header + payload + checksum
            
            // Check if we have enough data for this message
            if (offset + total_length > data_size) {
                ESP_LOGW(TAG, "Incomplete UBX message at offset %d", offset);
                break;
            }
            
            // Validate checksum
            uint8_t ck_a, ck_b;
            ubx_calculate_checksum(&data[offset + 2], payload_length + 4, &ck_a, &ck_b);
            
            uint8_t expected_ck_a = data[offset + total_length - 2];
            uint8_t expected_ck_b = data[offset + total_length - 1];
            
            if (ck_a != expected_ck_a || ck_b != expected_ck_b) {
                ESP_LOGW(TAG, "UBX checksum mismatch at offset %d", offset);
                offset += 2; // Skip sync chars and try next
                continue;
            }
            
            // Send the message
            esp_err_t ret = send_ubx_message(uart_port, &data[offset], total_length, 1000); // 1 second timeout
            if (ret == ESP_OK) {
                messages_sent++;
                ESP_LOGD(TAG, "📤 Sent UBX message %d (class: 0x%02X, id: 0x%02X, len: %d)", 
                        messages_sent, data[offset + 2], data[offset + 3], payload_length);
            } else {
                messages_failed++;
                ESP_LOGW(TAG, "❌ Failed to send UBX message %d: %s", messages_sent + messages_failed, esp_err_to_name(ret));
            }
            
            // Add small delay between messages to avoid overwhelming GPS
            vTaskDelay(pdMS_TO_TICKS(50));
            
            offset += total_length;
        } else {
            offset++; // Not a sync char, continue searching
        }
    }
    
    ESP_LOGI(TAG, "✅ UBX message injection complete: %d sent, %d failed", messages_sent, messages_failed);
    
    if (messages_failed > 0) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    
    return ESP_OK;
}

// Upload AssistNow data to u-blox GPS
// Upload AssistNow data to u-blox GPS
esp_err_t assistnow_upload_to_ublox(uart_port_t uart_port)
{
    if (!assistnow_state.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!assistnow_is_data_valid()) {
        ESP_LOGW(TAG, "⚠️ AssistNow data is not valid or expired");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "🔄 Injecting AssistNow data to u-blox GPS...");

    // Allocate buffer for AssistNow data
    size_t data_size = assistnow_state.info.data_size_bytes;
    if (data_size == 0) {
        ESP_LOGE(TAG, "No AssistNow data available");
        return ESP_ERR_NOT_FOUND;
    }

    uint8_t* assistnow_data = malloc(data_size);
    if (!assistnow_data) {
        ESP_LOGE(TAG, "Failed to allocate memory for AssistNow data (%d bytes)", data_size);
        return ESP_ERR_NO_MEM;
    }

    // Load AssistNow data from storage
    esp_err_t ret = assistnow_load_from_storage(assistnow_data, &data_size);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to load AssistNow data from storage: %s", esp_err_to_name(ret));
        free(assistnow_data);
        return ret;
    }

    ESP_LOGI(TAG, "📖 Loaded %d bytes of AssistNow data from storage", data_size);

    // Parse and send UBX messages
    ret = parse_and_send_ubx_messages(uart_port, assistnow_data, data_size);

    // Clean up
    free(assistnow_data);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "✅ AssistNow data successfully injected to GPS");
        assistnow_state.info.loaded_in_gps = true;
    } else {
        ESP_LOGE(TAG, "❌ Failed to inject AssistNow data to GPS: %s", esp_err_to_name(ret));
        assistnow_state.info.loaded_in_gps = false;
    }

    return ret;
}

// Wrapper function for backward compatibility - uses default GPS UART port
esp_err_t assistnow_upload_to_gps(void)
{
    // Use default GPS UART port (UART_NUM_1 as defined in visual_slam_main.c)
    uart_port_t gps_port = UART_NUM_1;
    return assistnow_upload_to_ublox(gps_port);
}
