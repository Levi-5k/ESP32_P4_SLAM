/**
 * @file communication_monitor_c6.c
 * @brief Communication and status monitoring for ESP32-C6
 */

#include "communication_monitor_c6.h"
#include "wifi_manager_c6.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "COMM_MON_C6";

// Monitor state
static struct {
    comm_monitor_config_t config;
    comm_monitor_stats_t stats;
    TaskHandle_t monitor_task_handle;
    TimerHandle_t status_timer;
    bool initialized;
    bool running;
    uint32_t system_start_time;
} monitor_state = {0};

// Forward declarations
static void monitor_task(void *pvParameters);
static void status_timer_callback(TimerHandle_t xTimer);
static void update_system_stats(void);
static void log_comprehensive_status(void);

static void update_system_stats(void)
{
    uint32_t current_time = esp_timer_get_time() / 1000000; // Convert to seconds
    monitor_state.stats.uptime_seconds = current_time - monitor_state.system_start_time;
    
    monitor_state.stats.free_heap_size = esp_get_free_heap_size();
    uint32_t min_heap = esp_get_minimum_free_heap_size();
    if (min_heap < monitor_state.stats.min_free_heap_size || monitor_state.stats.min_free_heap_size == 0) {
        monitor_state.stats.min_free_heap_size = min_heap;
    }
    
    // Check P4 communication health
    uint32_t time_since_last_p4_msg = current_time - monitor_state.stats.last_p4_message_time;
    monitor_state.stats.p4_communication_active = (time_since_last_p4_msg < (monitor_state.config.p4_comm_timeout_ms / 1000));
}

static void log_comprehensive_status(void)
{
    update_system_stats();
    
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "════════════════════════════════════════════════════════════════");
    ESP_LOGI(TAG, "                    ESP32-C6 SYSTEM STATUS                     ");
    ESP_LOGI(TAG, "════════════════════════════════════════════════════════════════");
    
    // System information
    ESP_LOGI(TAG, "🖥️  SYSTEM INFO:");
    ESP_LOGI(TAG, "   ⏰ Uptime: %lu seconds (%lu minutes)", 
             monitor_state.stats.uptime_seconds, 
             monitor_state.stats.uptime_seconds / 60);
    ESP_LOGI(TAG, "   💾 Free Heap: %lu bytes (%lu KB)", 
             monitor_state.stats.free_heap_size,
             monitor_state.stats.free_heap_size / 1024);
    ESP_LOGI(TAG, "   📉 Min Heap: %lu bytes (%lu KB)", 
             monitor_state.stats.min_free_heap_size,
             monitor_state.stats.min_free_heap_size / 1024);
    
    // WiFi status
    wifi_connection_info_t wifi_info;
    wifi_manager_stats_t wifi_stats;
    wifi_manager_status_t wifi_status = wifi_manager_c6_get_status();
    
    ESP_LOGI(TAG, "📶 WIFI STATUS:");
    ESP_LOGI(TAG, "   🔧 Status: %s", wifi_manager_c6_status_to_string(wifi_status));
    
    if (wifi_manager_c6_get_connection_info(&wifi_info) == ESP_OK) {
        if (wifi_info.is_connected) {
            ESP_LOGI(TAG, "   🔗 Connected to: %s", wifi_info.ssid);
            ESP_LOGI(TAG, "   🌐 IP Address: %s", wifi_info.ip_address);
            ESP_LOGI(TAG, "   📡 RSSI: %d dBm", wifi_info.rssi);
            ESP_LOGI(TAG, "   📻 Channel: %d", wifi_info.channel);
            ESP_LOGI(TAG, "   ⏰ Connection uptime: %lu seconds", wifi_info.uptime_seconds);
        } else if (wifi_info.is_ap_mode) {
            ESP_LOGI(TAG, "   📡 AP Mode: %s", wifi_info.ssid);
            ESP_LOGI(TAG, "   🌐 AP IP: %s", wifi_info.ip_address);
            ESP_LOGI(TAG, "   📻 AP Channel: %d", wifi_info.channel);
            ESP_LOGI(TAG, "   👥 Connected clients: %d", wifi_info.connected_clients);
        } else {
            ESP_LOGI(TAG, "   ❌ Not connected");
        }
    }
    
    if (wifi_manager_c6_get_stats(&wifi_stats) == ESP_OK) {
        ESP_LOGI(TAG, "   📊 WiFi Stats:");
        ESP_LOGI(TAG, "      🔄 Connection attempts: %lu", wifi_stats.sta_connection_attempts);
        ESP_LOGI(TAG, "      ✅ Successful connections: %lu", wifi_stats.sta_successful_connections);
        ESP_LOGI(TAG, "      ❌ Disconnections: %lu", wifi_stats.sta_disconnections);
        ESP_LOGI(TAG, "      📡 AP activations: %lu", wifi_stats.ap_mode_activations);
        ESP_LOGI(TAG, "      👥 AP client connections: %lu", wifi_stats.ap_client_connections);
    }
    
    // P4 Communication status
    ESP_LOGI(TAG, "🔗 P4 COMMUNICATION:");
    ESP_LOGI(TAG, "   📤 Messages sent to P4: %lu", monitor_state.stats.messages_sent_to_p4);
    ESP_LOGI(TAG, "   📥 Messages received from P4: %lu", monitor_state.stats.messages_received_from_p4);
    ESP_LOGI(TAG, "   ❌ Connection failures: %lu", monitor_state.stats.p4_connection_failures);
    ESP_LOGI(TAG, "   🟢 Communication active: %s", 
             monitor_state.stats.p4_communication_active ? "Yes" : "No");
    
    if (monitor_state.stats.last_p4_message_time > 0) {
        uint32_t time_since_last = monitor_state.stats.uptime_seconds - 
                                  (monitor_state.stats.last_p4_message_time - monitor_state.system_start_time);
        ESP_LOGI(TAG, "   ⏰ Last P4 message: %lu seconds ago", time_since_last);
    } else {
        ESP_LOGI(TAG, "   ⏰ Last P4 message: Never");
    }
    
    // Web server status
    ESP_LOGI(TAG, "🌐 WEB SERVER:");
    ESP_LOGI(TAG, "   📄 Requests handled: %lu", monitor_state.stats.web_requests_handled);
    
    // Performance metrics
    if (monitor_state.config.enable_performance_monitoring) {
        ESP_LOGI(TAG, "⚡ PERFORMANCE:");
        ESP_LOGI(TAG, "   💾 Heap efficiency: %.1f%%", 
                 (float)(monitor_state.stats.min_free_heap_size * 100) / monitor_state.stats.free_heap_size);
        
        if (monitor_state.stats.messages_received_from_p4 > 0) {
            float msg_rate = (float)monitor_state.stats.messages_received_from_p4 / 
                           (monitor_state.stats.uptime_seconds > 0 ? monitor_state.stats.uptime_seconds : 1);
            ESP_LOGI(TAG, "   📊 P4 message rate: %.2f msg/sec", msg_rate);
        }
        
        if (monitor_state.stats.web_requests_handled > 0) {
            float web_rate = (float)monitor_state.stats.web_requests_handled / 
                           (monitor_state.stats.uptime_seconds > 0 ? monitor_state.stats.uptime_seconds : 1);
            ESP_LOGI(TAG, "   🌐 Web request rate: %.2f req/sec", web_rate);
        }
    }
    
    ESP_LOGI(TAG, "════════════════════════════════════════════════════════════════");
    ESP_LOGI(TAG, "");
}

static void status_timer_callback(TimerHandle_t xTimer)
{
    if (monitor_state.config.enable_detailed_logging) {
        log_comprehensive_status();
    } else {
        // Brief status update
        update_system_stats();
        
        wifi_manager_status_t wifi_status = wifi_manager_c6_get_status();
        ESP_LOGI(TAG, "🔄 Status: WiFi=%s, P4_Comm=%s, Uptime=%lus, Heap=%luKB", 
                 wifi_manager_c6_status_to_string(wifi_status),
                 monitor_state.stats.p4_communication_active ? "OK" : "FAIL",
                 monitor_state.stats.uptime_seconds,
                 monitor_state.stats.free_heap_size / 1024);
    }
}

static void monitor_task(void *pvParameters)
{
    ESP_LOGI(TAG, "🚀 Communication monitor task started");
    
    while (monitor_state.running) {
        update_system_stats();
        
        // Check for low memory
        if (monitor_state.stats.free_heap_size < 10240) { // Less than 10KB
            ESP_LOGW(TAG, "⚠️ Low memory warning: %lu bytes free", monitor_state.stats.free_heap_size);
        }
        
        // Check P4 communication health
        if (!monitor_state.stats.p4_communication_active && monitor_state.stats.last_p4_message_time > 0) {
            ESP_LOGW(TAG, "⚠️ P4 communication timeout - no messages for %lu seconds", 
                     monitor_state.config.p4_comm_timeout_ms / 1000);
        }
        
        // Sleep for a short interval
        vTaskDelay(pdMS_TO_TICKS(5000)); // Check every 5 seconds
    }
    
    ESP_LOGI(TAG, "🛑 Communication monitor task stopped");
    vTaskDelete(NULL);
}

esp_err_t comm_monitor_c6_init(const comm_monitor_config_t* config)
{
    if (monitor_state.initialized) {
        ESP_LOGW(TAG, "⚠️ Communication monitor already initialized");
        return ESP_OK;
    }
    
    if (!config) {
        ESP_LOGE(TAG, "❌ Invalid configuration");
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "🚀 Initializing Communication Monitor C6");
    
    // Copy configuration
    memcpy(&monitor_state.config, config, sizeof(comm_monitor_config_t));
    
    // Initialize statistics
    memset(&monitor_state.stats, 0, sizeof(comm_monitor_stats_t));
    monitor_state.system_start_time = esp_timer_get_time() / 1000000;
    
    // Create status timer
    monitor_state.status_timer = xTimerCreate(
        "status_timer",
        pdMS_TO_TICKS(monitor_state.config.status_log_interval_ms),
        pdTRUE,  // Auto-reload timer
        NULL,
        status_timer_callback
    );
    
    if (!monitor_state.status_timer) {
        ESP_LOGE(TAG, "❌ Failed to create status timer");
        return ESP_ERR_NO_MEM;
    }
    
    monitor_state.initialized = true;
    
    ESP_LOGI(TAG, "✅ Communication Monitor C6 initialized");
    ESP_LOGI(TAG, "📋 Config: LogInterval=%lums, P4Timeout=%lums, DetailedLog=%s, PerfMon=%s",
             monitor_state.config.status_log_interval_ms,
             monitor_state.config.p4_comm_timeout_ms,
             monitor_state.config.enable_detailed_logging ? "Yes" : "No",
             monitor_state.config.enable_performance_monitoring ? "Yes" : "No");
    
    return ESP_OK;
}

esp_err_t comm_monitor_c6_start(void)
{
    if (!monitor_state.initialized) {
        ESP_LOGE(TAG, "❌ Communication monitor not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (monitor_state.running) {
        ESP_LOGW(TAG, "⚠️ Communication monitor already running");
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "🚀 Starting Communication Monitor");
    
    monitor_state.running = true;
    
    // Create monitor task
    BaseType_t task_created = xTaskCreate(
        monitor_task,
        "comm_monitor",
        4096,  // Stack size
        NULL,
        tskIDLE_PRIORITY + 2,  // Priority
        &monitor_state.monitor_task_handle
    );
    
    if (task_created != pdTRUE) {
        ESP_LOGE(TAG, "❌ Failed to create monitor task");
        monitor_state.running = false;
        return ESP_ERR_NO_MEM;
    }
    
    // Start status timer
    if (xTimerStart(monitor_state.status_timer, 0) != pdPASS) {
        ESP_LOGE(TAG, "❌ Failed to start status timer");
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "✅ Communication Monitor started successfully");
    
    // Log initial status
    log_comprehensive_status();
    
    return ESP_OK;
}

esp_err_t comm_monitor_c6_stop(void)
{
    if (!monitor_state.running) {
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "🛑 Stopping Communication Monitor");
    
    monitor_state.running = false;
    
    // Stop timer
    if (monitor_state.status_timer) {
        xTimerStop(monitor_state.status_timer, 0);
    }
    
    // Wait for task to finish
    if (monitor_state.monitor_task_handle) {
        vTaskDelay(pdMS_TO_TICKS(100)); // Give task time to finish
        monitor_state.monitor_task_handle = NULL;
    }
    
    ESP_LOGI(TAG, "✅ Communication Monitor stopped");
    return ESP_OK;
}

void comm_monitor_c6_update_p4_activity(uint8_t message_type, bool is_incoming)
{
    if (!monitor_state.initialized) {
        return;
    }
    
    uint32_t current_time = esp_timer_get_time() / 1000000;
    monitor_state.stats.last_p4_message_time = current_time;
    
    if (is_incoming) {
        monitor_state.stats.messages_received_from_p4++;
        if (monitor_state.config.enable_detailed_logging) {
            ESP_LOGD(TAG, "📥 P4 Message received: type=0x%02X", message_type);
        }
    } else {
        monitor_state.stats.messages_sent_to_p4++;
        if (monitor_state.config.enable_detailed_logging) {
            ESP_LOGD(TAG, "📤 P4 Message sent: type=0x%02X", message_type);
        }
    }
}

void comm_monitor_c6_update_web_activity(void)
{
    if (!monitor_state.initialized) {
        return;
    }
    
    monitor_state.stats.web_requests_handled++;
    
    if (monitor_state.config.enable_detailed_logging) {
        ESP_LOGD(TAG, "🌐 Web request handled (total: %lu)", monitor_state.stats.web_requests_handled);
    }
}

esp_err_t comm_monitor_c6_get_stats(comm_monitor_stats_t* stats)
{
    if (!stats) {
        return ESP_ERR_INVALID_ARG;
    }
    
    update_system_stats();
    memcpy(stats, &monitor_state.stats, sizeof(comm_monitor_stats_t));
    return ESP_OK;
}

void comm_monitor_c6_print_comprehensive_status(void)
{
    log_comprehensive_status();
}

bool comm_monitor_c6_is_p4_communication_healthy(void)
{
    if (!monitor_state.initialized) {
        return false;
    }
    
    update_system_stats();
    return monitor_state.stats.p4_communication_active;
}
