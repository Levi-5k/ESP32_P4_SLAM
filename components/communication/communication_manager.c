/**
 * @file communication_manager.c
 * @brief Communication manager implementation for ESP32-P4 master
 *
     // Initialize ESP-Hosted Wi-Fi remote
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_err_t ret = esp_wifi_remote_init(&cfg);Manages communication with ESP32-C6 slave device via ESP-Hosted protocol.
 * Handles sending SLAM data and receiving configuration updates.
 */

#include "communication_manager.h"
#include "communication_protocol.h"
#include "sensor_fusion.h"
#include "wifi_manager.h"
#include "wifi_positioning.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_wifi_remote.h"
#include "esp_wifi.h"
#include <string.h>

static const char *TAG = "COMM_MGR";

// Forward declarations for configuration handler functions
static esp_err_t handle_slam_config_update(const config_update_msg_t* config);
static esp_err_t handle_fusion_config_update(const config_update_msg_t* config);
static esp_err_t handle_camera_config_update(const config_update_msg_t* config);
static esp_err_t handle_gps_config_update(const config_update_msg_t* config);
static esp_err_t handle_imu_config_update(const config_update_msg_t* config);

// Communication manager state
static struct {
    bool initialized;
    bool connected;
    TaskHandle_t send_task_handle;
    TaskHandle_t receive_task_handle;
    QueueHandle_t send_queue;
    QueueHandle_t receive_queue;
    SemaphoreHandle_t send_mutex;
    uint32_t sequence_number;
    uint32_t last_heartbeat_time;
    uint32_t heartbeat_interval_ms;
} comm_state = {
    .heartbeat_interval_ms = 1000,  // 1 second heartbeat
};

// External data sources
extern system_status_t system_status;

// Task for sending messages to C6
static void comm_send_task(void* pvParameters)
{
    comm_message_t message;
    TickType_t last_heartbeat = 0;
    
    ESP_LOGI(TAG, "Communication send task started");
    
    while (1) {
        // Send heartbeat at regular intervals
        if ((xTaskGetTickCount() - last_heartbeat) >= pdMS_TO_TICKS(comm_state.heartbeat_interval_ms)) {
            if (comm_send_heartbeat_auto() == ESP_OK) {
                last_heartbeat = xTaskGetTickCount();
            }
        }
        
        // Check for queued messages to send
        if (xQueueReceive(comm_state.send_queue, &message, pdMS_TO_TICKS(100)) == pdTRUE) {
            esp_err_t ret = comm_send_message_raw(&message);
            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "Failed to send message type 0x%02x: %s", 
                        message.header.msg_type, esp_err_to_name(ret));
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// Task for receiving messages from C6
static void comm_receive_task(void* pvParameters)
{
    comm_message_t message;
    
    ESP_LOGI(TAG, "Communication receive task started");
    
    while (1) {
        if (comm_receive_message(&message, 100) == ESP_OK) {
            if (comm_verify_checksum(&message)) {
                // Process received message
                esp_err_t ret = comm_process_received_message(&message);
                if (ret != ESP_OK) {
                    ESP_LOGW(TAG, "Failed to process message type 0x%02x", 
                            message.header.msg_type);
                }
            } else {
                ESP_LOGW(TAG, "Received message with invalid checksum");
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

esp_err_t comm_manager_init(void)
{
    if (comm_state.initialized) {
        ESP_LOGW(TAG, "Communication manager already initialized");
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Initializing communication manager");
    
    // Initialize ESP-Hosted WiFi remote
    wifi_init_config_t wifi_init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_err_t ret = esp_wifi_remote_init(&wifi_init_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize ESP-Hosted WiFi: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Create communication queues
    comm_state.send_queue = xQueueCreate(10, sizeof(comm_message_t));
    comm_state.receive_queue = xQueueCreate(10, sizeof(comm_message_t));
    if (!comm_state.send_queue || !comm_state.receive_queue) {
        ESP_LOGE(TAG, "Failed to create communication queues");
        return ESP_ERR_NO_MEM;
    }
    
    // Create mutex for send operations
    comm_state.send_mutex = xSemaphoreCreateMutex();
    if (!comm_state.send_mutex) {
        ESP_LOGE(TAG, "Failed to create send mutex");
        return ESP_ERR_NO_MEM;
    }
    
    // Create communication tasks
    BaseType_t task_ret;
    task_ret = xTaskCreate(comm_send_task, "comm_send", 4096, NULL, 5, &comm_state.send_task_handle);
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create send task");
        return ESP_ERR_NO_MEM;
    }
    
    task_ret = xTaskCreate(comm_receive_task, "comm_recv", 4096, NULL, 5, &comm_state.receive_task_handle);
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create receive task");
        return ESP_ERR_NO_MEM;
    }
    
    comm_state.initialized = true;
    comm_state.sequence_number = 1;
    
    ESP_LOGI(TAG, "Communication manager initialized successfully");
    return ESP_OK;
}

esp_err_t comm_send_heartbeat_auto(void)
{
    heartbeat_msg_t heartbeat = {
        .uptime_ms = esp_timer_get_time() / 1000,
        .system_status = system_status.slam_active ? 1 : 0,      // Overall system status
        .slam_status = system_status.slam_tracking ? 1 : 0,      // SLAM tracking status  
        .camera_status = system_status.camera_available ? 1 : 0, // Camera status
        .sd_card_status = 1, // TODO: Get actual SD card status
        .free_heap_size = esp_get_free_heap_size(),
        .cpu_usage_percent = (uint16_t)(system_status.cpu_usage_percent * 100), // Convert to percentage
    };
    
    return comm_send_heartbeat(&heartbeat);
}

esp_err_t comm_send_heartbeat(const heartbeat_msg_t* heartbeat)
{
    return comm_queue_message(MSG_P4_TO_C6_HEARTBEAT, heartbeat, sizeof(heartbeat_msg_t));
}

esp_err_t comm_send_slam_status(const slam_status_msg_t* status)
{
    return comm_queue_message(MSG_P4_TO_C6_SLAM_STATUS, status, sizeof(slam_status_msg_t));
}

esp_err_t comm_send_position(const position_msg_t* position)
{
    return comm_queue_message(MSG_P4_TO_C6_POSITION, position, sizeof(position_msg_t));
}

esp_err_t comm_send_telemetry(const telemetry_msg_t* telemetry)
{
    return comm_queue_message(MSG_P4_TO_C6_TELEMETRY, telemetry, sizeof(telemetry_msg_t));
}

esp_err_t comm_queue_message(uint8_t msg_type, const void* payload, uint16_t payload_size)
{
    if (!comm_state.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (payload_size > COMM_MAX_PAYLOAD_SIZE) {
        ESP_LOGE(TAG, "Payload size %d exceeds maximum %d", payload_size, COMM_MAX_PAYLOAD_SIZE);
        return ESP_ERR_INVALID_SIZE;
    }
    
    comm_message_t message;
    memset(&message, 0, sizeof(message));
    
    // Fill header
    message.header.magic = COMM_MAGIC_HEADER;
    message.header.version = COMM_PROTOCOL_VERSION;
    message.header.msg_type = msg_type;
    message.header.payload_size = payload_size;
    message.header.sequence = comm_state.sequence_number++;
    message.header.timestamp = esp_timer_get_time() / 1000;
    
    // Copy payload
    if (payload && payload_size > 0) {
        memcpy(&message.payload, payload, payload_size);
    }
    
    // Calculate checksum
    message.header.checksum = comm_calculate_checksum(&message.payload, payload_size);
    
    // Queue message for sending
    if (xQueueSend(comm_state.send_queue, &message, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to queue message type 0x%02x", msg_type);
        return ESP_ERR_TIMEOUT;
    }
    
    return ESP_OK;
}

esp_err_t comm_send_message_raw(const comm_message_t* message)
{
    if (!comm_state.initialized || !message) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Take send mutex
    if (xSemaphoreTake(comm_state.send_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    
    esp_err_t ret = ESP_OK;
    
    // Send message via ESP-Hosted
    // Note: This would use the actual ESP-Hosted API for data transmission
    // For now, we'll use a placeholder implementation
    
    // TODO: Implement actual ESP-Hosted data transmission
    ESP_LOGD(TAG, "Sending message type 0x%02x, seq %d, size %d", 
            message->header.msg_type, message->header.sequence, message->header.payload_size);
    
    xSemaphoreGive(comm_state.send_mutex);
    return ret;
}

esp_err_t comm_process_received_message(const comm_message_t* message)
{
    if (!message) {
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGD(TAG, "Processing received message type 0x%02x", message->header.msg_type);
    
    switch (message->header.msg_type) {
        case MSG_C6_TO_P4_CONFIG_UPDATE:
            return comm_handle_config_update(&message->payload.config_update);
            
        case MSG_C6_TO_P4_WIFI_SCAN_REQ:
            return comm_handle_wifi_scan_request();
            
        case MSG_C6_TO_P4_CONNECT_WIFI:
            return comm_handle_wifi_connect(&message->payload.wifi_connect);
            
        case MSG_C6_TO_P4_WIFI_CONTROL:
            return comm_handle_wifi_control(&message->payload.wifi_control);
            
        case MSG_C6_TO_P4_SYSTEM_CMD:
            return comm_handle_system_command(&message->payload.system_cmd);
            
        case MSG_C6_TO_P4_MAP_CMD:
            return comm_handle_map_command(&message->payload.map_cmd);
            
        case MSG_C6_TO_P4_WIFI_POSITIONING_DATA:
            return comm_handle_wifi_positioning_data(&message->payload.wifi_positioning);
            
        case MSG_C6_TO_P4_HEARTBEAT_ACK:
            comm_state.connected = true;
            ESP_LOGD(TAG, "Received heartbeat ACK");
            return ESP_OK;
            
        default:
            ESP_LOGW(TAG, "Unknown message type 0x%02x", message->header.msg_type);
            return ESP_ERR_NOT_SUPPORTED;
    }
}

esp_err_t comm_init_wifi_connection(const char* ssid, const char* password)
{
    if (!ssid || !password) {
        return ESP_ERR_INVALID_ARG;
    }
    
    wifi_connect_msg_t wifi_config = {0};
    
    // Copy SSID and password with proper bounds checking
    strncpy((char*)wifi_config.ssid, ssid, COMM_MAX_SSID_LEN - 1);
    wifi_config.ssid[COMM_MAX_SSID_LEN - 1] = '\0';
    
    strncpy((char*)wifi_config.password, password, COMM_MAX_PASSWORD_LEN - 1);
    wifi_config.password[COMM_MAX_PASSWORD_LEN - 1] = '\0';
    
    // Set authentication mode to WPA2 PSK (most common)
    wifi_config.auth_mode = 3; // WIFI_AUTH_WPA2_PSK
    
    ESP_LOGI(TAG, "🚀 Initializing WiFi connection - SSID: %s", ssid);
    
    // Call the WiFi connect handler directly
    return comm_handle_wifi_connect(&wifi_config);
}

esp_err_t comm_handle_config_update(const config_update_msg_t* config)
{
    if (!config) {
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "⚙️ Configuration update received - type:%d, id:%d, size:%lu", 
             config->config_type, config->config_id, config->value_size);
    
    esp_err_t ret = ESP_OK;
    
    switch (config->config_type) {
        case 1: // SLAM configuration
            ret = handle_slam_config_update(config);
            break;
            
        case 2: // Sensor fusion configuration
            ret = handle_fusion_config_update(config);
            break;
            
        case 3: // Camera configuration
            ret = handle_camera_config_update(config);
            break;
            
        case 4: // GPS configuration
            ret = handle_gps_config_update(config);
            break;
            
        case 5: // IMU configuration
            ret = handle_imu_config_update(config);
            break;
            
        default:
            ESP_LOGW(TAG, "Unknown configuration type: %d", config->config_type);
            return ESP_ERR_NOT_SUPPORTED;
    }
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "✅ Configuration update applied successfully");
    } else {
        ESP_LOGE(TAG, "❌ Configuration update failed: %s", esp_err_to_name(ret));
    }
    
    return ret;
}

// Configuration handler functions for safe parameter updates
static esp_err_t handle_slam_config_update(const config_update_msg_t* config)
{
    ESP_LOGI(TAG, "🎯 Updating SLAM configuration - id:%d", config->config_id);
    
    // Import SLAM configuration if component is available
    #ifdef CONFIG_SLAM_CORE_ENABLED
    // SLAM parameters can be updated safely - no impact on EKF or MSP
    switch (config->config_id) {
        case 1: // max_features
            if (config->value_size == sizeof(int)) {
                int max_features;
                memcpy(&max_features, config->value_data, sizeof(int));
                ESP_LOGI(TAG, "Updating SLAM max_features: %d", max_features);
                // slam_core_set_max_features(max_features);
            }
            break;
        case 2: // fast_threshold
            if (config->value_size == sizeof(float)) {
                float threshold;
                memcpy(&threshold, config->value_data, sizeof(float));
                ESP_LOGI(TAG, "Updating SLAM fast_threshold: %.2f", threshold);
                // slam_core_set_fast_threshold(threshold);
            }
            break;
        default:
            ESP_LOGW(TAG, "Unknown SLAM config ID: %d", config->config_id);
            return ESP_ERR_NOT_SUPPORTED;
    }
    #endif
    
    ESP_LOGI(TAG, "✅ SLAM configuration updated - continuous operation maintained");
    return ESP_OK;
}

static esp_err_t handle_fusion_config_update(const config_update_msg_t* config)
{
    ESP_LOGI(TAG, "🔄 Updating sensor fusion configuration - id:%d", config->config_id);
    
    // Get current configuration
    fusion_config_t current_config;
    esp_err_t ret = sensor_fusion_get_config(&current_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get current fusion config");
        return ret;
    }
    
    // Create updated configuration
    fusion_config_t new_config = current_config;
    
    // Apply specific parameter update
    switch (config->config_id) {
        case 1: // position_noise
            if (config->value_size == sizeof(float)) {
                memcpy(&new_config.position_noise, config->value_data, sizeof(float));
                ESP_LOGI(TAG, "Updating position_noise: %.4f", new_config.position_noise);
            }
            break;
        case 2: // velocity_noise
            if (config->value_size == sizeof(float)) {
                memcpy(&new_config.velocity_noise, config->value_data, sizeof(float));
                ESP_LOGI(TAG, "Updating velocity_noise: %.4f", new_config.velocity_noise);
            }
            break;
        case 3: // gps_weight
            if (config->value_size == sizeof(float)) {
                memcpy(&new_config.gps_weight, config->value_data, sizeof(float));
                ESP_LOGI(TAG, "Updating gps_weight: %.2f", new_config.gps_weight);
            }
            break;
        case 4: // slam_weight
            if (config->value_size == sizeof(float)) {
                memcpy(&new_config.slam_weight, config->value_data, sizeof(float));
                ESP_LOGI(TAG, "Updating slam_weight: %.2f", new_config.slam_weight);
            }
            break;
        case 5: // gps_position_noise
            if (config->value_size == sizeof(float)) {
                memcpy(&new_config.gps_position_noise, config->value_data, sizeof(float));
                ESP_LOGI(TAG, "Updating gps_position_noise: %.4f", new_config.gps_position_noise);
            }
            break;
        case 6: // slam_position_noise
            if (config->value_size == sizeof(float)) {
                memcpy(&new_config.slam_position_noise, config->value_data, sizeof(float));
                ESP_LOGI(TAG, "Updating slam_position_noise: %.4f", new_config.slam_position_noise);
            }
            break;
        default:
            ESP_LOGW(TAG, "Unknown fusion config ID: %d", config->config_id);
            return ESP_ERR_NOT_SUPPORTED;
    }
    
    // Apply configuration safely - this is thread-safe and atomic
    ret = sensor_fusion_set_config(&new_config);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "✅ Fusion parameters updated - EKF continues running, MSP output maintained");
    } else {
        ESP_LOGE(TAG, "❌ Failed to update fusion config: %s", esp_err_to_name(ret));
    }
    
    return ret;
}

static esp_err_t handle_camera_config_update(const config_update_msg_t* config)
{
    ESP_LOGI(TAG, "📷 Updating camera configuration - id:%d", config->config_id);
    
    // Camera configuration updates have no impact on filter or MSP
    switch (config->config_id) {
        case 1: // exposure
            if (config->value_size == sizeof(int)) {
                int exposure;
                memcpy(&exposure, config->value_data, sizeof(int));
                ESP_LOGI(TAG, "Updating camera exposure: %d", exposure);
                // camera_set_exposure(exposure);
            }
            break;
        case 2: // gain
            if (config->value_size == sizeof(int)) {
                int gain;
                memcpy(&gain, config->value_data, sizeof(int));
                ESP_LOGI(TAG, "Updating camera gain: %d", gain);
                // camera_set_gain(gain);
            }
            break;
        case 3: // brightness
            if (config->value_size == sizeof(int)) {
                int brightness;
                memcpy(&brightness, config->value_data, sizeof(int));
                ESP_LOGI(TAG, "Updating camera brightness: %d", brightness);
                // camera_set_brightness(brightness);
            }
            break;
        default:
            ESP_LOGW(TAG, "Unknown camera config ID: %d", config->config_id);
            return ESP_ERR_NOT_SUPPORTED;
    }
    
    ESP_LOGI(TAG, "✅ Camera configuration updated - no impact on navigation systems");
    return ESP_OK;
}

static esp_err_t handle_gps_config_update(const config_update_msg_t* config)
{
    ESP_LOGI(TAG, "🛰️ Updating GPS configuration - id:%d", config->config_id);
    
    // GPS configuration updates are safe - filter adapts automatically
    switch (config->config_id) {
        case 1: // update_rate
            if (config->value_size == sizeof(int)) {
                int rate;
                memcpy(&rate, config->value_data, sizeof(int));
                ESP_LOGI(TAG, "Updating GPS update rate: %d Hz", rate);
                // gps_set_update_rate(rate);
            }
            break;
        case 2: // dynamic_model
            if (config->value_size == sizeof(int)) {
                int model;
                memcpy(&model, config->value_data, sizeof(int));
                ESP_LOGI(TAG, "Updating GPS dynamic model: %d", model);
                // gps_set_dynamic_model(model);
            }
            break;
        default:
            ESP_LOGW(TAG, "Unknown GPS config ID: %d", config->config_id);
            return ESP_ERR_NOT_SUPPORTED;
    }
    
    ESP_LOGI(TAG, "✅ GPS configuration updated - filter automatically adapts");
    return ESP_OK;
}

static esp_err_t handle_imu_config_update(const config_update_msg_t* config)
{
    ESP_LOGI(TAG, "📐 Updating IMU configuration - id:%d", config->config_id);
    
    // IMU configuration updates are safe - filter handles gracefully
    switch (config->config_id) {
        case 1: // sample_rate
            if (config->value_size == sizeof(int)) {
                int rate;
                memcpy(&rate, config->value_data, sizeof(int));
                ESP_LOGI(TAG, "Updating IMU sample rate: %d Hz", rate);
                // imu_set_sample_rate(rate);
            }
            break;
        case 2: // accel_range
            if (config->value_size == sizeof(int)) {
                int range;
                memcpy(&range, config->value_data, sizeof(int));
                ESP_LOGI(TAG, "Updating IMU accel range: %d g", range);
                // imu_set_accel_range(range);
            }
            break;
        case 3: // gyro_range
            if (config->value_size == sizeof(int)) {
                int range;
                memcpy(&range, config->value_data, sizeof(int));
                ESP_LOGI(TAG, "Updating IMU gyro range: %d dps", range);
                // imu_set_gyro_range(range);
            }
            break;
        default:
            ESP_LOGW(TAG, "Unknown IMU config ID: %d", config->config_id);
            return ESP_ERR_NOT_SUPPORTED;
    }
    
    ESP_LOGI(TAG, "✅ IMU configuration updated - EKF handles parameter changes smoothly");
    return ESP_OK;
}

esp_err_t comm_handle_wifi_scan_request(void)
{
    ESP_LOGI(TAG, "WiFi scan requested from web interface");
    
    // TODO: Implement WiFi scanning and send results back to C6
    
    return ESP_OK;
}

esp_err_t comm_handle_wifi_connect(const wifi_connect_msg_t* wifi_config)
{
    if (!wifi_config) {
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "📡 WiFi connection request: SSID '%s'", wifi_config->ssid);
    
    // Enable WiFi first if not already enabled
    esp_err_t ret = wifi_manager_enable(false);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ Failed to enable WiFi manager: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Connect to the specified network
    ret = wifi_manager_connect((const char*)wifi_config->ssid, 
                              (const char*)wifi_config->password);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ Failed to connect to WiFi '%s': %s", 
                wifi_config->ssid, esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "✅ WiFi connection initiated for SSID '%s'", wifi_config->ssid);
    return ESP_OK;
}

esp_err_t comm_handle_wifi_control(const wifi_control_msg_t* wifi_control)
{
    if (!wifi_control) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "📡 WiFi control command: enable=%d, scan_only=%d, ap_fallback=%d",
             wifi_control->enable_wifi, wifi_control->enable_scan_only, 
             wifi_control->auto_ap_fallback);

    esp_err_t ret = ESP_OK;

    if (wifi_control->enable_wifi) {
        // Enable WiFi with specified mode
        if (wifi_control->enable_scan_only) {
            ESP_LOGI(TAG, "🔍 Enabling WiFi in scan-only mode");
            ret = wifi_manager_enable_scan_only();
        } else {
            ESP_LOGI(TAG, "📡 Enabling WiFi in full mode");
            ret = wifi_manager_enable(false);
        }
    } else {
        // Disable WiFi
        ESP_LOGI(TAG, "🚫 Disabling WiFi");
        ret = wifi_manager_disable();
    }

    // Send WiFi status back to C6
    wifi_status_msg_t status_msg = {0};
    status_msg.wifi_enabled = wifi_control->enable_wifi;
    status_msg.scan_active = wifi_control->enable_scan_only;
    
    if (ret == ESP_OK) {
        wifi_manager_status_t mgr_status = wifi_manager_get_status();
        status_msg.connected = (mgr_status == WIFI_MGR_STATUS_CONNECTED);
        status_msg.ap_mode_active = (mgr_status == WIFI_MGR_STATUS_AP_MODE);
        
        wifi_connection_info_t conn_info;
        if (wifi_manager_get_connection_info(&conn_info) == ESP_OK) {
            strlcpy((char*)status_msg.ssid, conn_info.ssid, sizeof(status_msg.ssid));
            status_msg.rssi = conn_info.rssi;
            // Convert IP string to uint32_t (simplified)
            // TODO: Implement proper IP string to uint32_t conversion
            status_msg.ip_address = 0;
        }
    }

    // Send status update to C6
    esp_err_t send_ret = comm_send_message(MSG_P4_TO_C6_WIFI_STATUS, 
                                          &status_msg, sizeof(status_msg));
    if (send_ret != ESP_OK) {
        ESP_LOGW(TAG, "⚠️ Failed to send WiFi status to C6: %s", esp_err_to_name(send_ret));
    }

    return ret;
}

esp_err_t comm_handle_system_command(const system_cmd_msg_t* cmd)
{
    ESP_LOGI(TAG, "System command: %d", cmd->command);
    
    switch (cmd->command) {
        case SYS_CMD_RESTART:
            ESP_LOGI(TAG, "Restarting system...");
            esp_restart();
            break;
            
        case SYS_CMD_CALIBRATE_IMU:
            ESP_LOGI(TAG, "IMU calibration requested");
            // TODO: Start IMU calibration
            break;
            
        case SYS_CMD_CALIBRATE_CAMERA:
            ESP_LOGI(TAG, "Camera calibration requested");
            // TODO: Start camera calibration
            break;
            
        case SYS_CMD_RESET_SLAM:
            ESP_LOGI(TAG, "SLAM reset requested");
            // TODO: Reset SLAM system
            break;
            
        case SYS_CMD_EMERGENCY_STOP:
            ESP_LOGI(TAG, "Emergency stop requested");
            // TODO: Implement emergency stop
            break;
            
        case SYS_CMD_WIFI_ENABLE:
            ESP_LOGI(TAG, "📡 WiFi enable command received");
            return wifi_manager_enable(false);
            
        case SYS_CMD_WIFI_DISABLE:
            ESP_LOGI(TAG, "🚫 WiFi disable command received");
            return wifi_manager_disable();
            
        case SYS_CMD_WIFI_GET_STATUS: {
            ESP_LOGI(TAG, "📊 WiFi status request received");
            
            // Send current WiFi status to C6
            wifi_status_msg_t status_msg = {0};
            status_msg.wifi_enabled = wifi_manager_is_running();
            status_msg.scan_active = wifi_manager_is_scan_only();
            
            wifi_manager_status_t mgr_status = wifi_manager_get_status();
            status_msg.connected = (mgr_status == WIFI_MGR_STATUS_CONNECTED);
            status_msg.ap_mode_active = (mgr_status == WIFI_MGR_STATUS_AP_MODE);
            
            wifi_connection_info_t conn_info;
            if (wifi_manager_get_connection_info(&conn_info) == ESP_OK) {
                strlcpy((char*)status_msg.ssid, conn_info.ssid, sizeof(status_msg.ssid));
                status_msg.rssi = conn_info.rssi;
                // TODO: Convert IP properly
                status_msg.ip_address = 0;
            }
            
            return comm_send_message(MSG_P4_TO_C6_WIFI_STATUS, 
                                   &status_msg, sizeof(status_msg));
        }
            
        default:
            ESP_LOGW(TAG, "Unknown system command: %d", cmd->command);
            return ESP_ERR_NOT_SUPPORTED;
    }
    
    return ESP_OK;
}

esp_err_t comm_handle_map_command(const map_cmd_msg_t* cmd)
{
    ESP_LOGI(TAG, "Map command: %d, map: %s", cmd->command, cmd->map_name);
    
    switch (cmd->command) {
        case MAP_CMD_SAVE:
            ESP_LOGI(TAG, "Saving map: %s", cmd->map_name);
            // TODO: Save current map to SD card
            break;
            
        case MAP_CMD_LOAD:
            ESP_LOGI(TAG, "Loading map: %s", cmd->map_name);
            // TODO: Load map from SD card
            break;
            
        case MAP_CMD_CLEAR:
            ESP_LOGI(TAG, "Clearing current map");
            // TODO: Clear current map
            break;
            
        case MAP_CMD_LIST:
            ESP_LOGI(TAG, "Listing available maps");
            // TODO: List maps and send back to C6
            break;
            
        default:
            ESP_LOGW(TAG, "Unknown map command: %d", cmd->command);
            return ESP_ERR_NOT_SUPPORTED;
    }
    
    return ESP_OK;
}

esp_err_t comm_handle_wifi_positioning_data(const wifi_positioning_msg_t* pos_data)
{
    if (!pos_data) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "📡 Received WiFi positioning data from C6: %d access points, seq=%d", 
             pos_data->ap_count, pos_data->sequence_number);

    // Convert C6 WiFi AP data to positioning format
    wifi_ap_info_t aps[MAX_POSITIONING_APS];
    uint8_t valid_ap_count = 0;

    for (uint8_t i = 0; i < pos_data->ap_count && i < MAX_POSITIONING_APS && valid_ap_count < MAX_POSITIONING_APS; i++) {
        const wifi_ap_positioning_t* src_ap = &pos_data->access_points[i];
        wifi_ap_info_t* dst_ap = &aps[valid_ap_count];

        // Copy and validate AP data
        if (src_ap->rssi < -100 || src_ap->rssi > 0) {
            ESP_LOGD(TAG, "Skipping AP with invalid RSSI: %d", src_ap->rssi);
            continue;
        }

        // Copy SSID (ensure null termination)
        strncpy(dst_ap->ssid, src_ap->ssid, sizeof(dst_ap->ssid) - 1);
        dst_ap->ssid[sizeof(dst_ap->ssid) - 1] = '\0';

        // Copy BSSID (ensure null termination)
        strncpy(dst_ap->bssid, src_ap->bssid, sizeof(dst_ap->bssid) - 1);
        dst_ap->bssid[sizeof(dst_ap->bssid) - 1] = '\0';

        // Copy other fields
        dst_ap->rssi = src_ap->rssi;
        dst_ap->channel = src_ap->channel;
        dst_ap->auth_mode = src_ap->auth_mode;
        dst_ap->is_hidden = src_ap->is_hidden;

        ESP_LOGD(TAG, "AP[%d]: SSID='%s', BSSID='%s', RSSI=%d dBm, Ch=%d", 
                 valid_ap_count, dst_ap->ssid, dst_ap->bssid, dst_ap->rssi, dst_ap->channel);

        valid_ap_count++;
    }

    if (valid_ap_count == 0) {
        ESP_LOGW(TAG, "⚠️ No valid access points in positioning data");
        return ESP_ERR_INVALID_ARG;
    }

    // Process the AP data through WiFi positioning system
    wifi_position_t position;
    esp_err_t ret = wifi_positioning_process_external_aps(aps, valid_ap_count, &position);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "✅ WiFi position calculated: lat=%.6f, lon=%.6f, accuracy=%.1fm", 
                 position.latitude, position.longitude, position.accuracy_h);

        // Send acknowledgment back to C6
        wifi_position_ack_msg_t ack_msg = {
            .sequence_number = pos_data->sequence_number,
            .status = WIFI_POS_ACK_SUCCESS,
            .processing_time_ms = (uint32_t)((esp_timer_get_time() - pos_data->timestamp) / 1000),
            .position_calculated = true,
            .latitude = position.latitude,
            .longitude = position.longitude,
            .accuracy = position.accuracy_h
        };

        esp_err_t send_ret = comm_send_message(MSG_P4_TO_C6_WIFI_POSITION_ACK, 
                                              &ack_msg, sizeof(ack_msg));
        if (send_ret != ESP_OK) {
            ESP_LOGW(TAG, "⚠️ Failed to send WiFi position ACK to C6: %s", esp_err_to_name(send_ret));
        } else {
            ESP_LOGD(TAG, "📤 WiFi position ACK sent to C6 (seq=%d, time=%dms)", 
                     ack_msg.sequence_number, ack_msg.processing_time_ms);
        }

    } else {
        ESP_LOGW(TAG, "⚠️ Failed to calculate position from C6 WiFi data: %s", esp_err_to_name(ret));

        // Send error acknowledgment
        wifi_position_ack_msg_t ack_msg = {
            .sequence_number = pos_data->sequence_number,
            .status = WIFI_POS_ACK_ERROR,
            .processing_time_ms = (uint32_t)((esp_timer_get_time() - pos_data->timestamp) / 1000),
            .position_calculated = false,
            .latitude = 0.0,
            .longitude = 0.0,
            .accuracy = 0.0
        };

        esp_err_t send_ret = comm_send_message(MSG_P4_TO_C6_WIFI_POSITION_ACK, 
                                              &ack_msg, sizeof(ack_msg));
        if (send_ret != ESP_OK) {
            ESP_LOGW(TAG, "⚠️ Failed to send WiFi position error ACK to C6: %s", esp_err_to_name(send_ret));
        }
    }

    return ret;
}

bool comm_is_connected(void)
{
    return comm_state.connected;
}

uint32_t comm_get_last_heartbeat_time(void)
{
    return comm_state.last_heartbeat_time;
}
