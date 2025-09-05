/**
 * @file slam_communication_handler.c
 * @brief Communication handler for ESP32-C6 slave device
 * 
 * Handles communication with ESP32-P4 master via ESP-Hosted protocol.
 * Receives SLAM data and sends web interface commands.
 */

#include "slam_web_server.h"
#include "communication_protocol.h"
#include "interface.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_timer.h"
#include <string.h>

static const char *TAG = "SLAM_COMM";

// Communication state
static QueueHandle_t incoming_message_queue = NULL;
static QueueHandle_t outgoing_message_queue = NULL;
static TaskHandle_t comm_task_handle = NULL;
static bool comm_initialized = false;

// Forward declarations
static void slam_comm_task(void *pvParameters);
static esp_err_t process_incoming_message(const comm_message_t* message);
static esp_err_t send_message_to_p4(const comm_message_t* message);

// External function to register our communication callback
extern esp_err_t register_slam_comm_callback(void (*callback)(uint8_t* data, uint16_t len));

// Callback function called by ESP-Hosted when data is received from P4
static void slam_data_received_callback(uint8_t* data, uint16_t len)
{
    if (!data || len < sizeof(comm_msg_header_t)) {
        ESP_LOGW(TAG, "Invalid data received: len=%d", len);
        return;
    }
    
    comm_message_t* message = (comm_message_t*)data;
    
    // Verify magic header
    if (message->header.magic != COMM_MAGIC_HEADER) {
        ESP_LOGW(TAG, "Invalid magic header: 0x%08x", message->header.magic);
        return;
    }
    
    // Verify message size
    if (len != sizeof(comm_msg_header_t) + message->header.payload_size) {
        ESP_LOGW(TAG, "Message size mismatch: expected %d, got %d", 
                sizeof(comm_msg_header_t) + message->header.payload_size, len);
        return;
    }
    
    // Verify checksum
    if (!comm_verify_checksum(message)) {
        ESP_LOGW(TAG, "Invalid checksum for message type 0x%02x", message->header.msg_type);
        return;
    }
    
    // Queue message for processing
    if (incoming_message_queue) {
        if (xQueueSend(incoming_message_queue, message, 0) != pdTRUE) {
            ESP_LOGW(TAG, "Failed to queue incoming message (queue full)");
        }
    }
}

// Main communication task
static void slam_comm_task(void *pvParameters)
{
    comm_message_t incoming_message;
    comm_message_t outgoing_message;
    
    ESP_LOGI(TAG, "SLAM communication task started");
    
    while (1) {
        // Process incoming messages from P4
        if (xQueueReceive(incoming_message_queue, &incoming_message, pdMS_TO_TICKS(10)) == pdTRUE) {
            esp_err_t ret = process_incoming_message(&incoming_message);
            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "Failed to process incoming message: %s", esp_err_to_name(ret));
            }
        }
        
        // Process outgoing messages to P4
        if (xQueueReceive(outgoing_message_queue, &outgoing_message, pdMS_TO_TICKS(10)) == pdTRUE) {
            esp_err_t ret = send_message_to_p4(&outgoing_message);
            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "Failed to send message to P4: %s", esp_err_to_name(ret));
            }
        }
        
        // Small delay to prevent task from consuming too much CPU
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

static esp_err_t process_incoming_message(const comm_message_t* message)
{
    if (!message) {
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGD(TAG, "Processing message type 0x%02x from P4", message->header.msg_type);
    
    // Update web server data cache
    esp_err_t ret = slam_web_server_update_data(message);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to update web server data: %s", esp_err_to_name(ret));
    }
    
    // Broadcast to connected web clients
    ret = slam_web_server_broadcast_data();
    if (ret != ESP_OK) {
        ESP_LOGD(TAG, "Failed to broadcast data (no clients connected)");
    }
    
    // Send heartbeat acknowledgment for heartbeat messages
    if (message->header.msg_type == MSG_P4_TO_C6_HEARTBEAT) {
        comm_message_t ack_message = {0};
        ack_message.header.magic = COMM_MAGIC_HEADER;
        ack_message.header.version = COMM_PROTOCOL_VERSION;
        ack_message.header.msg_type = MSG_C6_TO_P4_HEARTBEAT_ACK;
        ack_message.header.payload_size = 0;
        ack_message.header.timestamp = esp_timer_get_time() / 1000;
        ack_message.header.checksum = 0; // No payload
        
        // Queue acknowledgment for sending
        if (xQueueSend(outgoing_message_queue, &ack_message, 0) != pdTRUE) {
            ESP_LOGW(TAG, "Failed to queue heartbeat acknowledgment");
        }
    }
    
    return ESP_OK;
}

static esp_err_t send_message_to_p4(const comm_message_t* message)
{
    if (!message) {
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGD(TAG, "Sending message type 0x%02x to P4", message->header.msg_type);
    
    // Calculate total message size
    uint16_t total_size = sizeof(comm_msg_header_t) + message->header.payload_size;
    
    // Send via ESP-Hosted interface
    // Note: This is a placeholder - actual implementation would use ESP-Hosted API
    // You would need to implement this based on your ESP-Hosted configuration
    
    // For now, just log the message
    ESP_LOGI(TAG, "Would send %d bytes to P4 (type 0x%02x)", total_size, message->header.msg_type);
    
    return ESP_OK;
}

esp_err_t slam_communication_init(void)
{
    if (comm_initialized) {
        ESP_LOGW(TAG, "SLAM communication already initialized");
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Initializing SLAM communication handler");
    
    // Create message queues
    incoming_message_queue = xQueueCreate(10, sizeof(comm_message_t));
    outgoing_message_queue = xQueueCreate(10, sizeof(comm_message_t));
    
    if (!incoming_message_queue || !outgoing_message_queue) {
        ESP_LOGE(TAG, "Failed to create message queues");
        return ESP_ERR_NO_MEM;
    }
    
    // Register callback with ESP-Hosted system
    // Note: This would need to be implemented based on your ESP-Hosted setup
    // register_slam_comm_callback(slam_data_received_callback);
    
    // Create communication task
    BaseType_t ret = xTaskCreate(slam_comm_task, "slam_comm", 4096, NULL, 5, &comm_task_handle);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create communication task");
        return ESP_ERR_NO_MEM;
    }
    
    comm_initialized = true;
    ESP_LOGI(TAG, "SLAM communication handler initialized");
    return ESP_OK;
}

esp_err_t slam_communication_send_command(uint8_t msg_type, const void* payload, uint16_t payload_size)
{
    if (!comm_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (payload_size > COMM_MAX_PAYLOAD_SIZE) {
        ESP_LOGE(TAG, "Payload size %d exceeds maximum %d", payload_size, COMM_MAX_PAYLOAD_SIZE);
        return ESP_ERR_INVALID_SIZE;
    }
    
    comm_message_t message = {0};
    
    // Fill header
    message.header.magic = COMM_MAGIC_HEADER;
    message.header.version = COMM_PROTOCOL_VERSION;
    message.header.msg_type = msg_type;
    message.header.payload_size = payload_size;
    message.header.timestamp = esp_timer_get_time() / 1000;
    
    // Copy payload if provided
    if (payload && payload_size > 0) {
        memcpy(&message.payload, payload, payload_size);
    }
    
    // Calculate checksum
    message.header.checksum = comm_calculate_checksum(&message.payload, payload_size);
    
    // Queue message for sending
    if (xQueueSend(outgoing_message_queue, &message, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to queue outgoing message");
        return ESP_ERR_TIMEOUT;
    }
    
    return ESP_OK;
}

// Simulate receiving data from P4 (for testing)
esp_err_t slam_communication_simulate_p4_data(void)
{
    static uint32_t sequence = 1;
    static float pos_x = 0.0f, pos_y = 0.0f;
    
    // Simulate heartbeat
    comm_message_t heartbeat_msg = {0};
    heartbeat_msg.header.magic = COMM_MAGIC_HEADER;
    heartbeat_msg.header.version = COMM_PROTOCOL_VERSION;
    heartbeat_msg.header.msg_type = MSG_P4_TO_C6_HEARTBEAT;
    heartbeat_msg.header.payload_size = sizeof(heartbeat_msg_t);
    heartbeat_msg.header.sequence = sequence++;
    heartbeat_msg.header.timestamp = esp_timer_get_time() / 1000;
    
    heartbeat_msg.payload.heartbeat.uptime_ms = esp_timer_get_time() / 1000;
    heartbeat_msg.payload.heartbeat.system_status = 1;
    heartbeat_msg.payload.heartbeat.slam_status = 2;
    heartbeat_msg.payload.heartbeat.camera_status = 1;
    heartbeat_msg.payload.heartbeat.sd_card_status = 1;
    heartbeat_msg.payload.heartbeat.free_heap_size = 100000;
    heartbeat_msg.payload.heartbeat.cpu_usage_percent = 45;
    
    heartbeat_msg.header.checksum = comm_calculate_checksum(&heartbeat_msg.payload, sizeof(heartbeat_msg_t));
    
    // Simulate position update
    pos_x += 0.1f;
    pos_y += 0.05f;
    
    comm_message_t position_msg = {0};
    position_msg.header.magic = COMM_MAGIC_HEADER;
    position_msg.header.version = COMM_PROTOCOL_VERSION;
    position_msg.header.msg_type = MSG_P4_TO_C6_POSITION;
    position_msg.header.payload_size = sizeof(position_msg_t);
    position_msg.header.sequence = sequence++;
    position_msg.header.timestamp = esp_timer_get_time() / 1000;
    
    position_msg.payload.position.position_x = pos_x;
    position_msg.payload.position.position_y = pos_y;
    position_msg.payload.position.position_z = 1.5f;
    position_msg.payload.position.orientation_roll = 0.1f;
    position_msg.payload.position.orientation_pitch = 0.05f;
    position_msg.payload.position.orientation_yaw = pos_x * 0.1f;
    
    position_msg.header.checksum = comm_calculate_checksum(&position_msg.payload, sizeof(position_msg_t));
    
    // Process simulated messages
    process_incoming_message(&heartbeat_msg);
    process_incoming_message(&position_msg);
    
    return ESP_OK;
}
