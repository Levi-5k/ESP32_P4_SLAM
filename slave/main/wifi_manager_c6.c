/**
 * @file wifi_manager_c6.c
 * @brief WiFi manager for ESP32-C6 with AP mode fallback and comprehensive logging
 */

#include "wifi_manager_c6.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/timers.h"
#include "nvs_flash.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "WIFI_MGR_C6";

// Event group bits
#define WIFI_CONNECTED_BIT     BIT0
#define WIFI_FAIL_BIT          BIT1
#define WIFI_AP_STARTED_BIT    BIT2

// WiFi manager state
static struct {
    wifi_manager_config_t config;
    wifi_manager_status_t status;
    wifi_connection_info_t connection_info;
    wifi_manager_stats_t stats;
    wifi_manager_event_callback_t event_callback;
    
    esp_netif_t* sta_netif;
    esp_netif_t* ap_netif;
    EventGroupHandle_t wifi_event_group;
    TimerHandle_t connection_timer;
    
    uint8_t retry_count;
    bool initialized;
    bool is_connecting;
    uint32_t connection_start_time;
    uint32_t last_status_log_time;
    
    // Continuous scanning for positioning
    bool continuous_scan_active;
    uint32_t scan_interval_ms;
    TimerHandle_t scan_timer;
    void (*scan_callback)(const wifi_ap_record_t* records, uint16_t count);
    wifi_ap_record_t latest_scan_results[16];
    uint16_t latest_scan_count;
    uint32_t last_scan_time;
} wifi_mgr_state = {0};

// Forward declarations
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
static void connection_timeout_callback(TimerHandle_t xTimer);
static void scan_timer_callback(TimerHandle_t xTimer);
static esp_err_t perform_wifi_scan_internal(void);
static esp_err_t start_sta_mode(void);
static esp_err_t start_ap_mode_internal(void);
static void update_connection_info(void);
static void log_wifi_status(bool force_log);
static void call_event_callback(wifi_manager_event_t event, void* event_data);

const char* wifi_manager_c6_status_to_string(wifi_manager_status_t status)
{
    switch (status) {
        case WIFI_MGR_STATUS_IDLE: return "IDLE";
        case WIFI_MGR_STATUS_CONNECTING: return "CONNECTING";
        case WIFI_MGR_STATUS_CONNECTED: return "CONNECTED";
        case WIFI_MGR_STATUS_DISCONNECTED: return "DISCONNECTED";
        case WIFI_MGR_STATUS_AP_STARTING: return "AP_STARTING";
        case WIFI_MGR_STATUS_AP_RUNNING: return "AP_RUNNING";
        case WIFI_MGR_STATUS_ERROR: return "ERROR";
        default: return "UNKNOWN";
    }
}

static void log_wifi_status(bool force_log)
{
    uint32_t current_time = esp_timer_get_time() / 1000000; // Convert to seconds
    
    // Log status every 30 seconds or when forced
    if (!force_log && (current_time - wifi_mgr_state.last_status_log_time) < 30) {
        return;
    }
    
    wifi_mgr_state.last_status_log_time = current_time;
    
    ESP_LOGI(TAG, "📶 WiFi Status: %s", wifi_manager_c6_status_to_string(wifi_mgr_state.status));
    
    if (wifi_mgr_state.connection_info.is_connected) {
        ESP_LOGI(TAG, "🔗 Connected to: %s (IP: %s, RSSI: %d dBm, Ch: %d)", 
                 wifi_mgr_state.connection_info.ssid,
                 wifi_mgr_state.connection_info.ip_address,
                 wifi_mgr_state.connection_info.rssi,
                 wifi_mgr_state.connection_info.channel);
        ESP_LOGI(TAG, "⏰ Connection uptime: %lu seconds", wifi_mgr_state.connection_info.uptime_seconds);
    } else if (wifi_mgr_state.connection_info.is_ap_mode) {
        ESP_LOGI(TAG, "📡 AP Mode: %s (IP: %s, Ch: %d, Clients: %d)", 
                 wifi_mgr_state.connection_info.ssid,
                 wifi_mgr_state.connection_info.ip_address,
                 wifi_mgr_state.connection_info.channel,
                 wifi_mgr_state.connection_info.connected_clients);
    }
    
    // Log statistics
    ESP_LOGI(TAG, "📊 Stats: STA attempts=%lu, successes=%lu, disconnects=%lu, AP activations=%lu",
             wifi_mgr_state.stats.sta_connection_attempts,
             wifi_mgr_state.stats.sta_successful_connections,
             wifi_mgr_state.stats.sta_disconnections,
             wifi_mgr_state.stats.ap_mode_activations);
}

static void call_event_callback(wifi_manager_event_t event, void* event_data)
{
    if (wifi_mgr_state.event_callback) {
        wifi_mgr_state.event_callback(event, event_data);
    }
}

static void update_connection_info(void)
{
    memset(&wifi_mgr_state.connection_info, 0, sizeof(wifi_connection_info_t));
    
    if (wifi_mgr_state.status == WIFI_MGR_STATUS_CONNECTED) {
        // STA mode connection info
        wifi_config_t wifi_config;
        if (esp_wifi_get_config(WIFI_IF_STA, &wifi_config) == ESP_OK) {
            strncpy(wifi_mgr_state.connection_info.ssid, (char*)wifi_config.sta.ssid, sizeof(wifi_mgr_state.connection_info.ssid) - 1);
        }
        
        esp_netif_ip_info_t ip_info;
        if (esp_netif_get_ip_info(wifi_mgr_state.sta_netif, &ip_info) == ESP_OK) {
            esp_ip4addr_ntoa(&ip_info.ip, wifi_mgr_state.connection_info.ip_address, sizeof(wifi_mgr_state.connection_info.ip_address));
        }
        
        wifi_ap_record_t ap_info;
        if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
            wifi_mgr_state.connection_info.rssi = ap_info.rssi;
            wifi_mgr_state.connection_info.channel = ap_info.primary;
            wifi_mgr_state.connection_info.auth_mode = ap_info.authmode;
        }
        
        wifi_mgr_state.connection_info.is_connected = true;
        wifi_mgr_state.connection_info.is_ap_mode = false;
        
        // Calculate uptime
        if (wifi_mgr_state.stats.last_connection_time > 0) {
            wifi_mgr_state.connection_info.uptime_seconds = 
                (esp_timer_get_time() / 1000000) - wifi_mgr_state.stats.last_connection_time;
        }
        
    } else if (wifi_mgr_state.status == WIFI_MGR_STATUS_AP_RUNNING) {
        // AP mode connection info
        strncpy(wifi_mgr_state.connection_info.ssid, wifi_mgr_state.config.ap_ssid, sizeof(wifi_mgr_state.connection_info.ssid) - 1);
        wifi_mgr_state.connection_info.channel = wifi_mgr_state.config.ap_channel;
        wifi_mgr_state.connection_info.is_ap_mode = true;
        wifi_mgr_state.connection_info.is_connected = false;
        
        esp_netif_ip_info_t ip_info;
        if (esp_netif_get_ip_info(wifi_mgr_state.ap_netif, &ip_info) == ESP_OK) {
            esp_ip4addr_ntoa(&ip_info.ip, wifi_mgr_state.connection_info.ip_address, sizeof(wifi_mgr_state.connection_info.ip_address));
        }
        
        // Get connected clients count
        wifi_sta_list_t sta_list;
        if (esp_wifi_ap_get_sta_list(&sta_list) == ESP_OK) {
            wifi_mgr_state.connection_info.connected_clients = sta_list.num;
        }
    }
}

static void connection_timeout_callback(TimerHandle_t xTimer)
{
    ESP_LOGW(TAG, "⏰ Connection timeout after %lu seconds", wifi_mgr_state.config.connection_timeout_ms / 1000);
    
    if (wifi_mgr_state.retry_count >= wifi_mgr_state.config.max_retry_attempts) {
        ESP_LOGW(TAG, "🔄 Max retry attempts (%d) reached", wifi_mgr_state.config.max_retry_attempts);
        
        if (wifi_mgr_state.config.enable_ap_fallback) {
            ESP_LOGI(TAG, "📡 Falling back to AP mode");
            call_event_callback(WIFI_MGR_EVENT_FALLBACK_TO_AP, NULL);
            start_ap_mode_internal();
        } else {
            wifi_mgr_state.status = WIFI_MGR_STATUS_ERROR;
            call_event_callback(WIFI_MGR_EVENT_CONNECTION_FAILED, NULL);
        }
    } else {
        ESP_LOGI(TAG, "🔄 Retrying connection (attempt %d/%d)", 
                 wifi_mgr_state.retry_count + 1, wifi_mgr_state.config.max_retry_attempts);
        wifi_mgr_state.retry_count++;
        esp_wifi_connect();
    }
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    static const char* event_names[] = {
        "WIFI_READY", "SCAN_DONE", "STA_START", "STA_STOP", "STA_CONNECTED", 
        "STA_DISCONNECTED", "STA_AUTHMODE_CHANGE", "STA_WPS_ER_SUCCESS", 
        "STA_WPS_ER_FAILED", "STA_WPS_ER_TIMEOUT", "STA_WPS_ER_PIN", 
        "STA_WPS_ER_PBC_OVERLAP", "AP_START", "AP_STOP", "AP_STACONNECTED", 
        "AP_STADISCONNECTED", "AP_PROBEREQRECVED"
    };
    
    const char* event_name = (event_id < sizeof(event_names)/sizeof(event_names[0])) ? 
                            event_names[event_id] : "UNKNOWN";
    
    ESP_LOGI(TAG, "🎯 WiFi Event: %s (ID: %ld)", event_name, event_id);
    
    if (event_base == WIFI_EVENT) {
        switch (event_id) {
            case WIFI_EVENT_STA_START:
                ESP_LOGI(TAG, "🚀 STA mode started - attempting connection");
                wifi_mgr_state.stats.sta_connection_attempts++;
                wifi_mgr_state.connection_start_time = esp_timer_get_time() / 1000000;
                esp_wifi_connect();
                
                // Start connection timeout timer
                if (wifi_mgr_state.connection_timer) {
                    xTimerStart(wifi_mgr_state.connection_timer, 0);
                }
                break;
                
            case WIFI_EVENT_STA_CONNECTED: {
                wifi_event_sta_connected_t* event = (wifi_event_sta_connected_t*) event_data;
                ESP_LOGI(TAG, "✅ Connected to AP: %s (Channel: %d)", event->ssid, event->channel);
                
                wifi_mgr_state.status = WIFI_MGR_STATUS_CONNECTED;
                wifi_mgr_state.is_connecting = false;
                wifi_mgr_state.retry_count = 0;
                wifi_mgr_state.stats.sta_successful_connections++;
                wifi_mgr_state.stats.last_connection_time = esp_timer_get_time() / 1000000;
                
                // Stop connection timeout timer
                if (wifi_mgr_state.connection_timer) {
                    xTimerStop(wifi_mgr_state.connection_timer, 0);
                }
                
                call_event_callback(WIFI_MGR_EVENT_STA_CONNECTED, event_data);
                break;
            }
            
            case WIFI_EVENT_STA_DISCONNECTED: {
                wifi_event_sta_disconnected_t* event = (wifi_event_sta_disconnected_t*) event_data;
                ESP_LOGW(TAG, "❌ Disconnected from AP (reason: %d)", event->reason);
                
                wifi_mgr_state.status = WIFI_MGR_STATUS_DISCONNECTED;
                wifi_mgr_state.stats.sta_disconnections++;
                wifi_mgr_state.connection_info.is_connected = false;
                
                call_event_callback(WIFI_MGR_EVENT_STA_DISCONNECTED, event_data);
                
                // Auto-reconnect logic
                if (wifi_mgr_state.retry_count < wifi_mgr_state.config.max_retry_attempts) {
                    ESP_LOGI(TAG, "🔄 Auto-reconnecting (attempt %d/%d)", 
                             wifi_mgr_state.retry_count + 1, wifi_mgr_state.config.max_retry_attempts);
                    wifi_mgr_state.retry_count++;
                    wifi_mgr_state.stats.sta_connection_attempts++;
                    esp_wifi_connect();
                    
                    // Restart timeout timer
                    if (wifi_mgr_state.connection_timer) {
                        xTimerStart(wifi_mgr_state.connection_timer, 0);
                    }
                } else {
                    ESP_LOGW(TAG, "🔄 Max retry attempts reached");
                    if (wifi_mgr_state.config.enable_ap_fallback) {
                        ESP_LOGI(TAG, "📡 Falling back to AP mode");
                        call_event_callback(WIFI_MGR_EVENT_FALLBACK_TO_AP, NULL);
                        start_ap_mode_internal();
                    }
                }
                break;
            }
            
            case WIFI_EVENT_AP_START:
                ESP_LOGI(TAG, "📡 AP mode started successfully");
                wifi_mgr_state.status = WIFI_MGR_STATUS_AP_RUNNING;
                wifi_mgr_state.stats.ap_mode_activations++;
                xEventGroupSetBits(wifi_mgr_state.wifi_event_group, WIFI_AP_STARTED_BIT);
                call_event_callback(WIFI_MGR_EVENT_AP_STARTED, NULL);
                break;
                
            case WIFI_EVENT_AP_STOP:
                ESP_LOGI(TAG, "📡 AP mode stopped");
                call_event_callback(WIFI_MGR_EVENT_AP_STOPPED, NULL);
                break;
                
            case WIFI_EVENT_AP_STACONNECTED: {
                wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
                ESP_LOGI(TAG, "👤 Client connected to AP: " MACSTR, MAC2STR(event->mac));
                wifi_mgr_state.stats.ap_client_connections++;
                call_event_callback(WIFI_MGR_EVENT_AP_CLIENT_CONNECTED, event_data);
                break;
            }
            
            case WIFI_EVENT_AP_STADISCONNECTED: {
                wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
                ESP_LOGI(TAG, "👤 Client disconnected from AP: " MACSTR, MAC2STR(event->mac));
                call_event_callback(WIFI_MGR_EVENT_AP_CLIENT_DISCONNECTED, event_data);
                break;
            }
            
            case WIFI_EVENT_SCAN_DONE:
                ESP_LOGD(TAG, "🔍 WiFi scan completed");
                if (wifi_mgr_state.continuous_scan_active) {
                    // Get scan results for positioning
                    uint16_t ap_count = 16; // Maximum we can store
                    esp_err_t ret = esp_wifi_scan_get_ap_records(&ap_count, wifi_mgr_state.latest_scan_results);
                    if (ret == ESP_OK) {
                        wifi_mgr_state.latest_scan_count = ap_count;
                        wifi_mgr_state.last_scan_time = esp_timer_get_time() / 1000;
                        
                        ESP_LOGD(TAG, "📡 Scan found %d access points for positioning", ap_count);
                        
                        // Call callback if registered
                        if (wifi_mgr_state.scan_callback) {
                            wifi_mgr_state.scan_callback(wifi_mgr_state.latest_scan_results, ap_count);
                        }
                    } else {
                        ESP_LOGW(TAG, "⚠️ Failed to get scan results: %s", esp_err_to_name(ret));
                    }
                }
                break;
        }
    } else if (event_base == IP_EVENT) {
        switch (event_id) {
            case IP_EVENT_STA_GOT_IP: {
                ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
                ESP_LOGI(TAG, "🌐 Got IP address: " IPSTR, IP2STR(&event->ip_info.ip));
                
                xEventGroupSetBits(wifi_mgr_state.wifi_event_group, WIFI_CONNECTED_BIT);
                call_event_callback(WIFI_MGR_EVENT_STA_GOT_IP, event_data);
                break;
            }
        }
    }
    
    // Update connection info and log status
    update_connection_info();
    log_wifi_status(true); // Force log on events
}

static esp_err_t start_sta_mode(void)
{
    ESP_LOGI(TAG, "🔧 Starting STA mode with SSID: %s", wifi_mgr_state.config.default_ssid);
    
    wifi_mgr_state.status = WIFI_MGR_STATUS_CONNECTING;
    wifi_mgr_state.is_connecting = true;
    wifi_mgr_state.retry_count = 0;
    
    wifi_config_t wifi_config = {
        .sta = {
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    
    memset(&wifi_config.sta.ssid, 0, sizeof(wifi_config.sta.ssid));
    memset(&wifi_config.sta.password, 0, sizeof(wifi_config.sta.password));
    strlcpy((char*)wifi_config.sta.ssid, wifi_mgr_state.config.default_ssid, sizeof(wifi_config.sta.ssid));
    strlcpy((char*)wifi_config.sta.password, wifi_mgr_state.config.default_password, sizeof(wifi_config.sta.password));
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    return ESP_OK;
}

static esp_err_t start_ap_mode_internal(void)
{
    ESP_LOGI(TAG, "🔧 Starting AP mode with SSID: %s", wifi_mgr_state.config.ap_ssid);
    
    wifi_mgr_state.status = WIFI_MGR_STATUS_AP_STARTING;
    
    // Stop STA mode if running
    esp_wifi_stop();
    
    wifi_config_t wifi_config = {
        .ap = {
            .channel = wifi_mgr_state.config.ap_channel,
            .max_connection = wifi_mgr_state.config.ap_max_connections,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };
    
    memset(&wifi_config.ap.ssid, 0, sizeof(wifi_config.ap.ssid));
    memset(&wifi_config.ap.password, 0, sizeof(wifi_config.ap.password));
    strlcpy((char*)wifi_config.ap.ssid, wifi_mgr_state.config.ap_ssid, sizeof(wifi_config.ap.ssid));
    wifi_config.ap.ssid_len = strlen(wifi_mgr_state.config.ap_ssid);
    strlcpy((char*)wifi_config.ap.password, wifi_mgr_state.config.ap_password, sizeof(wifi_config.ap.password));
    
    if (strlen(wifi_mgr_state.config.ap_password) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    return ESP_OK;
}

esp_err_t wifi_manager_c6_init(const wifi_manager_config_t* config)
{
    if (wifi_mgr_state.initialized) {
        ESP_LOGW(TAG, "⚠️ WiFi manager already initialized");
        return ESP_OK;
    }
    
    if (!config) {
        ESP_LOGE(TAG, "❌ Invalid configuration");
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "🚀 Initializing WiFi Manager C6");
    
    // Copy configuration
    memcpy(&wifi_mgr_state.config, config, sizeof(wifi_manager_config_t));
    
    // Initialize WiFi event group
    wifi_mgr_state.wifi_event_group = xEventGroupCreate();
    if (!wifi_mgr_state.wifi_event_group) {
        ESP_LOGE(TAG, "❌ Failed to create event group");
        return ESP_ERR_NO_MEM;
    }
    
    // Create connection timeout timer
    wifi_mgr_state.connection_timer = xTimerCreate(
        "wifi_timeout",
        pdMS_TO_TICKS(wifi_mgr_state.config.connection_timeout_ms),
        pdFALSE,  // One-shot timer
        NULL,
        connection_timeout_callback
    );
    
    if (!wifi_mgr_state.connection_timer) {
        ESP_LOGE(TAG, "❌ Failed to create connection timer");
        return ESP_ERR_NO_MEM;
    }
    
    // Initialize network interfaces (if not already done)
    static bool netif_initialized = false;
    if (!netif_initialized) {
        ESP_ERROR_CHECK(esp_netif_init());
        netif_initialized = true;
    }
    
    // Create default event loop (if not already created)
    esp_err_t loop_ret = esp_event_loop_create_default();
    if (loop_ret != ESP_OK && loop_ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "❌ Failed to create event loop: %s", esp_err_to_name(loop_ret));
        return loop_ret;
    }
    
    wifi_mgr_state.sta_netif = esp_netif_create_default_wifi_sta();
    wifi_mgr_state.ap_netif = esp_netif_create_default_wifi_ap();
    
    if (!wifi_mgr_state.sta_netif || !wifi_mgr_state.ap_netif) {
        ESP_LOGE(TAG, "❌ Failed to create network interfaces");
        return ESP_ERR_NO_MEM;
    }
    
    // Initialize WiFi
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    
    // Register event handlers
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));
    
    wifi_mgr_state.status = WIFI_MGR_STATUS_IDLE;
    wifi_mgr_state.initialized = true;
    wifi_mgr_state.last_status_log_time = 0; // Force initial log
    
    ESP_LOGI(TAG, "✅ WiFi Manager C6 initialized successfully");
    ESP_LOGI(TAG, "📋 Config: STA_SSID=%s, AP_SSID=%s, Timeout=%lums, MaxRetry=%d, AP_Fallback=%s",
             wifi_mgr_state.config.default_ssid,
             wifi_mgr_state.config.ap_ssid,
             wifi_mgr_state.config.connection_timeout_ms,
             wifi_mgr_state.config.max_retry_attempts,
             wifi_mgr_state.config.enable_ap_fallback ? "Yes" : "No");
    
    return ESP_OK;
}

esp_err_t wifi_manager_c6_start(void)
{
    if (!wifi_mgr_state.initialized) {
        ESP_LOGE(TAG, "❌ WiFi manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "🚀 Starting WiFi Manager - attempting STA connection first");
    return start_sta_mode();
}

esp_err_t wifi_manager_c6_start_ap_mode(void)
{
    if (!wifi_mgr_state.initialized) {
        ESP_LOGE(TAG, "❌ WiFi manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "📡 Force starting AP mode");
    return start_ap_mode_internal();
}

esp_err_t wifi_manager_c6_connect(const char* ssid, const char* password)
{
    if (!wifi_mgr_state.initialized) {
        ESP_LOGE(TAG, "❌ WiFi manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (!ssid) {
        ESP_LOGE(TAG, "❌ Invalid SSID");
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "🔄 Connecting to new network: %s", ssid);
    
    // Update configuration
    strncpy(wifi_mgr_state.config.default_ssid, ssid, sizeof(wifi_mgr_state.config.default_ssid) - 1);
    if (password) {
        strncpy(wifi_mgr_state.config.default_password, password, sizeof(wifi_mgr_state.config.default_password) - 1);
    } else {
        wifi_mgr_state.config.default_password[0] = '\0';
    }
    
    // Reset retry count and start connection
    wifi_mgr_state.retry_count = 0;
    return start_sta_mode();
}

wifi_manager_status_t wifi_manager_c6_get_status(void)
{
    return wifi_mgr_state.status;
}

esp_err_t wifi_manager_c6_get_connection_info(wifi_connection_info_t* info)
{
    if (!info) {
        return ESP_ERR_INVALID_ARG;
    }
    
    update_connection_info();
    memcpy(info, &wifi_mgr_state.connection_info, sizeof(wifi_connection_info_t));
    return ESP_OK;
}

esp_err_t wifi_manager_c6_get_stats(wifi_manager_stats_t* stats)
{
    if (!stats) {
        return ESP_ERR_INVALID_ARG;
    }
    
    memcpy(stats, &wifi_mgr_state.stats, sizeof(wifi_manager_stats_t));
    return ESP_OK;
}

esp_err_t wifi_manager_c6_register_callback(wifi_manager_event_callback_t callback)
{
    wifi_mgr_state.event_callback = callback;
    return ESP_OK;
}

esp_err_t wifi_manager_c6_scan_networks(wifi_ap_record_t* ap_records, uint16_t max_records, uint16_t* actual_records)
{
    if (!ap_records || !actual_records) {
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "🔍 Scanning for WiFi networks...");
    
    wifi_scan_config_t scan_config = {
        .ssid = NULL,
        .bssid = NULL,
        .channel = 0,
        .show_hidden = false
    };
    
    ESP_ERROR_CHECK(esp_wifi_scan_start(&scan_config, true));
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&max_records, ap_records));
    *actual_records = max_records;
    
    ESP_LOGI(TAG, "🔍 Found %d networks", *actual_records);
    return ESP_OK;
}

void wifi_manager_c6_print_status(void)
{
    log_wifi_status(true);
}

bool wifi_manager_c6_is_initialized(void)
{
    return wifi_mgr_state.initialized;
}

esp_err_t wifi_manager_c6_get_diagnostics(char* buffer, size_t buffer_size)
{
    if (!buffer || buffer_size == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    update_connection_info();
    
    snprintf(buffer, buffer_size,
             "WiFi Manager C6 Diagnostics:\n"
             "Status: %s\n"
             "Initialized: %s\n"
             "STA Connected: %s\n"
             "AP Mode: %s\n"
             "Current SSID: %s\n"
             "IP Address: %s\n"
             "RSSI: %d dBm\n"
             "Channel: %d\n"
             "Connected Clients: %d\n"
             "Connection Attempts: %lu\n"
             "Successful Connections: %lu\n"
             "Disconnections: %lu\n"
             "AP Activations: %lu\n"
             "Retry Count: %d/%d\n",
             wifi_manager_c6_status_to_string(wifi_mgr_state.status),
             wifi_mgr_state.initialized ? "Yes" : "No",
             wifi_mgr_state.connection_info.is_connected ? "Yes" : "No",
             wifi_mgr_state.connection_info.is_ap_mode ? "Yes" : "No",
             wifi_mgr_state.connection_info.ssid,
             wifi_mgr_state.connection_info.ip_address,
             wifi_mgr_state.connection_info.rssi,
             wifi_mgr_state.connection_info.channel,
             wifi_mgr_state.connection_info.connected_clients,
             wifi_mgr_state.stats.sta_connection_attempts,
             wifi_mgr_state.stats.sta_successful_connections,
             wifi_mgr_state.stats.sta_disconnections,
             wifi_mgr_state.stats.ap_mode_activations,
             wifi_mgr_state.retry_count,
             wifi_mgr_state.config.max_retry_attempts);
    
    return ESP_OK;
}

esp_err_t wifi_manager_c6_stop(void)
{
    if (!wifi_mgr_state.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "🛑 Stopping WiFi Manager");
    
    if (wifi_mgr_state.connection_timer) {
        xTimerStop(wifi_mgr_state.connection_timer, 0);
        xTimerDelete(wifi_mgr_state.connection_timer, 0);
        wifi_mgr_state.connection_timer = NULL;
    }
    
    esp_wifi_stop();
    esp_wifi_deinit();
    
    if (wifi_mgr_state.wifi_event_group) {
        vEventGroupDelete(wifi_mgr_state.wifi_event_group);
        wifi_mgr_state.wifi_event_group = NULL;
    }
    
    wifi_mgr_state.initialized = false;
    wifi_mgr_state.status = WIFI_MGR_STATUS_IDLE;
    
    ESP_LOGI(TAG, "✅ WiFi Manager stopped");
    return ESP_OK;
}

// WiFi scan timer callback for continuous scanning
static void scan_timer_callback(TimerHandle_t xTimer)
{
    if (!wifi_mgr_state.continuous_scan_active) {
        return;
    }
    
    esp_err_t ret = perform_wifi_scan_internal();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "⚠️ WiFi scan failed in timer callback: %s", esp_err_to_name(ret));
    }
}

// Internal WiFi scan function
static esp_err_t perform_wifi_scan_internal(void)
{
    if (!wifi_mgr_state.initialized) {
        ESP_LOGW(TAG, "⚠️ WiFi manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Check if WiFi is started
    wifi_mode_t mode;
    esp_err_t wifi_ret = esp_wifi_get_mode(&mode);
    if (wifi_ret != ESP_OK) {
        ESP_LOGW(TAG, "⚠️ WiFi not started, cannot perform scan: %s", esp_err_to_name(wifi_ret));
        return wifi_ret;
    }
    
    ESP_LOGD(TAG, "🔍 Performing WiFi scan for positioning...");
    
    // Configure scan to find all networks
    wifi_scan_config_t scan_config = {
        .ssid = NULL,
        .bssid = NULL,
        .channel = 0,          // Scan all channels
        .show_hidden = true,   // Include hidden networks
        .scan_type = WIFI_SCAN_TYPE_ACTIVE,
        .scan_time = {
            .active = {
                .min = 100,    // Minimum time per channel (ms)
                .max = 300     // Maximum time per channel (ms)
            }
        }
    };
    
    esp_err_t ret = esp_wifi_scan_start(&scan_config, false); // Non-blocking scan
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "⚠️ Failed to start WiFi scan: %s", esp_err_to_name(ret));
        if (ret == ESP_ERR_WIFI_STATE) {
            ESP_LOGW(TAG, "   💡 WiFi may be in wrong state for scanning");
        }
        return ret;
    }
    
    // Note: Results will be handled in the WIFI_EVENT_SCAN_DONE event
    return ESP_OK;
}

esp_err_t wifi_manager_c6_start_continuous_scan(uint32_t scan_interval_ms)
{
    if (!wifi_mgr_state.initialized) {
        ESP_LOGE(TAG, "❌ WiFi manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (wifi_mgr_state.continuous_scan_active) {
        ESP_LOGW(TAG, "⚠️ Continuous scan already active");
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "🔍 Starting continuous WiFi scan for positioning (interval: %lu ms)", scan_interval_ms);
    
    wifi_mgr_state.scan_interval_ms = scan_interval_ms;
    wifi_mgr_state.continuous_scan_active = true;
    
    // Create scan timer
    wifi_mgr_state.scan_timer = xTimerCreate(
        "wifi_scan",
        pdMS_TO_TICKS(scan_interval_ms),
        pdTRUE,  // Auto-reload timer
        NULL,
        scan_timer_callback
    );
    
    if (!wifi_mgr_state.scan_timer) {
        ESP_LOGE(TAG, "❌ Failed to create scan timer");
        wifi_mgr_state.continuous_scan_active = false;
        return ESP_ERR_NO_MEM;
    }
    
    // Perform initial scan
    esp_err_t ret = perform_wifi_scan_internal();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "⚠️ Initial WiFi scan failed: %s", esp_err_to_name(ret));
    }
    
    // Start timer for future scans
    xTimerStart(wifi_mgr_state.scan_timer, 0);
    
    ESP_LOGI(TAG, "✅ Continuous WiFi scanning started");
    return ESP_OK;
}

esp_err_t wifi_manager_c6_stop_continuous_scan(void)
{
    if (!wifi_mgr_state.continuous_scan_active) {
        ESP_LOGW(TAG, "⚠️ Continuous scan not active");
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "🛑 Stopping continuous WiFi scan");
    
    wifi_mgr_state.continuous_scan_active = false;
    
    if (wifi_mgr_state.scan_timer) {
        xTimerStop(wifi_mgr_state.scan_timer, 0);
        xTimerDelete(wifi_mgr_state.scan_timer, 0);
        wifi_mgr_state.scan_timer = NULL;
    }
    
    ESP_LOGI(TAG, "✅ Continuous WiFi scanning stopped");
    return ESP_OK;
}

esp_err_t wifi_manager_c6_set_scan_callback(void (*callback)(const wifi_ap_record_t* records, uint16_t count))
{
    wifi_mgr_state.scan_callback = callback;
    ESP_LOGI(TAG, "📡 WiFi scan callback %s", callback ? "registered" : "cleared");
    return ESP_OK;
}

esp_err_t wifi_manager_c6_get_latest_scan(wifi_ap_record_t* ap_records, 
                                          uint16_t max_records, 
                                          uint16_t* actual_records)
{
    if (!ap_records || !actual_records) {
        return ESP_ERR_INVALID_ARG;
    }
    
    uint16_t copy_count = (wifi_mgr_state.latest_scan_count < max_records) ? 
                          wifi_mgr_state.latest_scan_count : max_records;
    
    memcpy(ap_records, wifi_mgr_state.latest_scan_results, 
           copy_count * sizeof(wifi_ap_record_t));
    
    *actual_records = copy_count;
    
    ESP_LOGD(TAG, "📊 Returning %d WiFi scan results (age: %lu ms)", 
             copy_count, 
             (esp_timer_get_time() / 1000) - wifi_mgr_state.last_scan_time);
    
    return ESP_OK;
}

bool wifi_manager_c6_is_scanning(void)
{
    return wifi_mgr_state.continuous_scan_active;
}
