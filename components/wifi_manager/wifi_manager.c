/**
 * @file wifi_manager.c
 * @brief WiFi Manager Component Implementation for ESP32-P4 with ESP-Hosted
 * @note Smart WiFi management with station/AP fallback using ESP-Hosted APIs
 */

#include "wifi_manager.h"
#include <esp_log.h>
#include <esp_wifi_remote.h>
#include <esp_event.h>
#include <esp_netif.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <string.h>
#include <esp_mac.h>

static const char *TAG = "WIFI_MANAGER";

// Component state
static bool initialized = false;
static bool running = false;
static bool wifi_enabled = false;
static bool scan_only_mode = false;
static bool scanning_active = false;
static wifi_manager_config_t config;
static wifi_manager_status_t status = WIFI_MGR_STATUS_DISABLED;
static wifi_connection_info_t connection_info = {0};

// Task and synchronization
static TaskHandle_t wifi_task = NULL;
static TaskHandle_t scan_task = NULL;
static EventGroupHandle_t event_group = NULL;
static SemaphoreHandle_t mutex = NULL;

// Event group bits
#define WIFI_SCAN_DONE_BIT       BIT0
#define WIFI_CONNECTED_BIT       BIT1
#define WIFI_DISCONNECTED_BIT    BIT2
#define WIFI_AP_STARTED_BIT      BIT3
#define WIFI_RETRY_TIMEOUT_BIT   BIT4

// Network interfaces
static esp_netif_t *sta_netif = NULL;
static esp_netif_t *ap_netif = NULL;

// Retry counters
static uint8_t station_retry_count = 0;
static uint8_t ap_retry_count = 0;

/**
 * @brief WiFi event handler
 */
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                              int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT) {
        switch (event_id) {
            case WIFI_EVENT_STA_START:
                ESP_LOGI(TAG, "📡 WiFi station started");
                status = WIFI_MGR_STATUS_CONNECTING;
                break;

            case WIFI_EVENT_STA_DISCONNECTED:
                ESP_LOGW(TAG, "📡 WiFi station disconnected");
                status = WIFI_MGR_STATUS_ERROR;
                xEventGroupSetBits(event_group, WIFI_DISCONNECTED_BIT);
                break;

            case WIFI_EVENT_AP_START:
                ESP_LOGI(TAG, "📡 WiFi AP started");
                status = WIFI_MGR_STATUS_AP_MODE;
                xEventGroupSetBits(event_group, WIFI_AP_STARTED_BIT);
                break;

            case WIFI_EVENT_AP_STACONNECTED: {
                wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
                ESP_LOGI(TAG, "📡 Station connected to AP, MAC: " MACSTR, MAC2STR(event->mac));
                connection_info.client_count++;
                break;
            }

            case WIFI_EVENT_AP_STADISCONNECTED: {
                wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
                ESP_LOGI(TAG, "📡 Station disconnected from AP, MAC: " MACSTR, MAC2STR(event->mac));
                if (connection_info.client_count > 0) {
                    connection_info.client_count--;
                }
                break;
            }

            case WIFI_EVENT_SCAN_DONE:
                ESP_LOGI(TAG, "📡 WiFi scan completed");
                xEventGroupSetBits(event_group, WIFI_SCAN_DONE_BIT);
                break;

            default:
                break;
        }
    } else if (event_base == IP_EVENT) {
        switch (event_id) {
            case IP_EVENT_STA_GOT_IP: {
                ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
                ESP_LOGI(TAG, "📡 Got IP: " IPSTR, IP2STR(&event->ip_info.ip));

                // Update connection info
                sprintf(connection_info.ip_addr, IPSTR, IP2STR(&event->ip_info.ip));
                connection_info.is_ap_mode = false;
                status = WIFI_MGR_STATUS_CONNECTED;

                xEventGroupSetBits(event_group, WIFI_CONNECTED_BIT);
                break;
            }

            case IP_EVENT_AP_STAIPASSIGNED: {
                ESP_LOGI(TAG, "📡 AP assigned IP to station");
                break;
            }

            default:
                break;
        }
    }
}

/**
 * @brief Initialize WiFi with ESP-Hosted
 */
static esp_err_t wifi_init_remote(void)
{
    ESP_LOGI(TAG, "🔧 Initializing ESP-Hosted WiFi...");

    // Initialize TCP/IP stack (only if not already initialized)
    esp_err_t ret = esp_netif_init();
    if (ret == ESP_ERR_INVALID_STATE) {
        ESP_LOGI(TAG, "🔄 TCP/IP stack already initialized");
    } else if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ TCP/IP stack initialization failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Note: Event loop is created by main application, don't create it here

    // Create network interfaces
    sta_netif = esp_netif_create_default_wifi_sta();
    ap_netif = esp_netif_create_default_wifi_ap();

    if (!sta_netif || !ap_netif) {
        ESP_LOGE(TAG, "❌ Failed to create network interfaces");
        return ESP_FAIL;
    }

    // Initialize WiFi with ESP-Hosted
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ret = esp_wifi_remote_init(&cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ ESP-Hosted WiFi initialization failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Register event handlers
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                      ESP_EVENT_ANY_ID,
                                                      &wifi_event_handler,
                                                      NULL,
                                                      NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                      ESP_EVENT_ANY_ID,
                                                      &wifi_event_handler,
                                                      NULL,
                                                      NULL));

    ESP_LOGI(TAG, "✅ ESP-Hosted WiFi initialized successfully");
    return ESP_OK;
}

/**
 * @brief Configure WiFi for station mode
 */
static esp_err_t wifi_configure_station(void)
{
    ESP_LOGI(TAG, "🔧 Configuring station mode...");

    // Set WiFi mode to station
    esp_err_t ret = esp_wifi_remote_set_mode(WIFI_MODE_STA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ Failed to set station mode: %s", esp_err_to_name(ret));
        return ret;
    }

    // Configure station settings
    wifi_config_t wifi_config = {0};
    strlcpy((char*)wifi_config.sta.ssid, config.sta_ssid, sizeof(wifi_config.sta.ssid));
    strlcpy((char*)wifi_config.sta.password, config.sta_password, sizeof(wifi_config.sta.password));

    ret = esp_wifi_remote_set_config(WIFI_IF_STA, &wifi_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ Failed to set station config: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "✅ Station mode configured for SSID: %s", config.sta_ssid);
    return ESP_OK;
}

/**
 * @brief Configure WiFi for AP mode
 */
static esp_err_t wifi_configure_ap(void)
{
    ESP_LOGI(TAG, "🔧 Configuring AP mode...");

    // Set WiFi mode to AP
    esp_err_t ret = esp_wifi_remote_set_mode(WIFI_MODE_AP);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ Failed to set AP mode: %s", esp_err_to_name(ret));
        return ret;
    }

    // Configure AP settings
    wifi_config_t wifi_config = {0};
    strlcpy((char*)wifi_config.ap.ssid, config.ap_ssid, sizeof(wifi_config.ap.ssid));
    strlcpy((char*)wifi_config.ap.password, config.ap_password, sizeof(wifi_config.ap.password));
    wifi_config.ap.ssid_len = strlen(config.ap_ssid);
    wifi_config.ap.channel = config.ap_channel;
    wifi_config.ap.max_connection = config.ap_max_connections;
    wifi_config.ap.authmode = WIFI_AUTH_WPA2_PSK;

    ret = esp_wifi_remote_set_config(WIFI_IF_AP, &wifi_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ Failed to set AP config: %s", esp_err_to_name(ret));
        return ret;
    }

    // Update connection info
    strlcpy(connection_info.ssid, config.ap_ssid, sizeof(connection_info.ssid));
    connection_info.is_ap_mode = true;
    connection_info.channel = config.ap_channel;

    ESP_LOGI(TAG, "✅ AP mode configured - SSID: %s, Channel: %d", config.ap_ssid, config.ap_channel);
    return ESP_OK;
}

/**
 * @brief Attempt to connect to configured network
 */
static esp_err_t wifi_try_connect_station(void)
{
    ESP_LOGI(TAG, "🔗 Attempting to connect to network: %s", config.sta_ssid);

    status = WIFI_MGR_STATUS_CONNECTING;
    station_retry_count = 0;

    // Start WiFi
    esp_err_t ret = esp_wifi_remote_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ Failed to start WiFi: %s", esp_err_to_name(ret));
        return ret;
    }

    // Connect to network
    ret = esp_wifi_remote_connect();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ Failed to connect to network: %s", esp_err_to_name(ret));
        return ret;
    }

    // Wait for connection or timeout
    EventBits_t bits = xEventGroupWaitBits(event_group,
                                          WIFI_CONNECTED_BIT | WIFI_DISCONNECTED_BIT,
                                          pdTRUE, pdFALSE,
                                          pdMS_TO_TICKS(config.sta_connect_timeout_ms));

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "✅ Successfully connected to network: %s", config.sta_ssid);

        // Get connection info
        wifi_ap_record_t ap_info;
        ret = esp_wifi_remote_sta_get_ap_info(&ap_info);
        if (ret == ESP_OK) {
            strlcpy(connection_info.ssid, (char*)ap_info.ssid, sizeof(connection_info.ssid));
            connection_info.rssi = ap_info.rssi;
            connection_info.channel = ap_info.primary;
        }

        return ESP_OK;
    } else {
        ESP_LOGW(TAG, "⚠️ Failed to connect to network: %s", config.sta_ssid);
        return ESP_FAIL;
    }
}

/**
 * @brief Start AP mode as fallback
 */
static esp_err_t wifi_start_ap_fallback(void)
{
    ESP_LOGI(TAG, "🔄 Starting AP mode as fallback...");

    // Stop current WiFi operations
    esp_wifi_remote_stop();

    // Configure for AP mode
    esp_err_t ret = wifi_configure_ap();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ Failed to configure AP mode: %s", esp_err_to_name(ret));
        return ret;
    }

    // Start WiFi in AP mode
    ret = esp_wifi_remote_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ Failed to start AP: %s", esp_err_to_name(ret));
        return ret;
    }

    // Wait for AP to start
    EventBits_t bits = xEventGroupWaitBits(event_group,
                                          WIFI_AP_STARTED_BIT,
                                          pdTRUE, pdFALSE,
                                          pdMS_TO_TICKS(5000));

    if (bits & WIFI_AP_STARTED_BIT) {
        ESP_LOGI(TAG, "✅ AP mode started successfully");
        ESP_LOGI(TAG, "🌐 AP SSID: %s", config.ap_ssid);
        ESP_LOGI(TAG, "🔐 AP Password: %s", config.ap_password);
        ESP_LOGI(TAG, "📡 AP Channel: %d", config.ap_channel);
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "❌ Failed to start AP mode");
        return ESP_FAIL;
    }
}

/**
 * @brief WiFi manager task
 */
static void wifi_manager_task(void *pvParameters)
{
    ESP_LOGI(TAG, "🏃 WiFi manager task started");

    while (running) {
        // Check if WiFi is disabled
        if (!wifi_enabled) {
            ESP_LOGD(TAG, "💤 WiFi disabled, task sleeping");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        switch (status) {
            case WIFI_MGR_STATUS_INITIALIZING:
                // Skip connection logic if in scan-only mode
                if (scan_only_mode) {
                    ESP_LOGI(TAG, "🔍 WiFi enabled in scan-only mode");
                    status = WIFI_MGR_STATUS_SCAN_ONLY;
                    break;
                }

                // Try station mode first
                if (config.sta_ssid && strlen(config.sta_ssid) > 0) {
                    ESP_LOGI(TAG, "🔍 Attempting station mode connection...");

                    if (wifi_try_connect_station() == ESP_OK) {
                        ESP_LOGI(TAG, "✅ Station mode successful - connected to network");
                        break;
                    } else {
                        ESP_LOGW(TAG, "⚠️ Station mode failed - checking AP fallback");
                    }
                }

                // Enhanced AP fallback logic
                if (config.auto_fallback) {
                    ESP_LOGI(TAG, "🔄 No networks found or connection failed, starting AP mode");
                    if (wifi_start_ap_fallback() == ESP_OK) {
                        ESP_LOGI(TAG, "✅ AP mode fallback successful");
                        ESP_LOGI(TAG, "🌐 Clients can connect to: %s", config.ap_ssid);
                    } else {
                        ESP_LOGE(TAG, "❌ Both station and AP mode failed");
                        status = WIFI_MGR_STATUS_ERROR;
                    }
                } else {
                    ESP_LOGW(TAG, "⚠️ Station mode failed and auto-fallback disabled");
                    status = WIFI_MGR_STATUS_ERROR;
                }
                break;

            case WIFI_MGR_STATUS_SCAN_ONLY:
                // In scan-only mode, just monitor and maintain scanning
                if (!scanning_active && wifi_enabled) {
                    ESP_LOGI(TAG, "🔍 Starting background scanning in scan-only mode");
                    wifi_manager_start_scanning();
                }
                vTaskDelay(pdMS_TO_TICKS(5000));
                break;

            case WIFI_MGR_STATUS_CONNECTED:
                // Monitor connection status - switch to AP if connection lost and enabled
                if (wifi_enabled && !scan_only_mode) {
                    wifi_ap_record_t ap_info;
                    esp_err_t ret = esp_wifi_remote_sta_get_ap_info(&ap_info);
                    if (ret != ESP_OK) {
                        ESP_LOGW(TAG, "⚠️ Lost connection, checking AP fallback");
                        if (config.auto_fallback) {
                            status = WIFI_MGR_STATUS_INITIALIZING;
                        }
                    }
                }
                vTaskDelay(pdMS_TO_TICKS(5000));
                break;

            case WIFI_MGR_STATUS_AP_MODE:
                // Monitor AP mode - stay in AP mode until manual change
                ESP_LOGD(TAG, "📡 AP mode active, %d clients connected", connection_info.client_count);
                vTaskDelay(pdMS_TO_TICKS(10000));
                break;

            case WIFI_MGR_STATUS_ERROR:
                // Enhanced retry logic with AP fallback
                if (station_retry_count < config.max_retry_count) {
                    station_retry_count++;
                    ESP_LOGI(TAG, "🔄 Retrying WiFi connection (attempt %d/%d)...",
                            station_retry_count, config.max_retry_count);
                    status = WIFI_MGR_STATUS_INITIALIZING;
                    vTaskDelay(pdMS_TO_TICKS(2000));
                } else if (config.auto_fallback && status != WIFI_MGR_STATUS_AP_MODE) {
                    ESP_LOGI(TAG, "🔄 Max retries reached, falling back to AP mode");
                    if (wifi_start_ap_fallback() == ESP_OK) {
                        ESP_LOGI(TAG, "✅ Emergency AP mode started");
                        station_retry_count = 0; // Reset counter for future attempts
                    } else {
                        ESP_LOGE(TAG, "❌ Emergency AP fallback failed, stopping");
                        running = false;
                    }
                } else {
                    ESP_LOGE(TAG, "❌ Maximum retry count reached, WiFi manager stopping");
                    running = false;
                }
                break;

            default:
                vTaskDelay(pdMS_TO_TICKS(1000));
                break;
        }
    }

    ESP_LOGI(TAG, "🛑 WiFi manager task stopped");
    wifi_task = NULL;
    vTaskDelete(NULL);
}

esp_err_t wifi_manager_init(const wifi_manager_config_t* mgr_config)
{
    if (initialized) {
        ESP_LOGW(TAG, "⚠️ WiFi manager already initialized");
        return ESP_OK;
    }

    if (!mgr_config) {
        ESP_LOGE(TAG, "❌ Invalid configuration");
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "🔧 Initializing WiFi manager...");

    // Copy configuration
    memcpy(&config, mgr_config, sizeof(wifi_manager_config_t));

    // Create synchronization objects
    event_group = xEventGroupCreate();
    mutex = xSemaphoreCreateMutex();

    if (!event_group || !mutex) {
        ESP_LOGE(TAG, "❌ Failed to create synchronization objects");
        return ESP_ERR_NO_MEM;
    }

    // Initialize ESP-Hosted WiFi
    esp_err_t ret = wifi_init_remote();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ WiFi initialization failed: %s", esp_err_to_name(ret));
        return ret;
    }

    initialized = true;
    status = WIFI_MGR_STATUS_DISABLED;

    ESP_LOGI(TAG, "✅ WiFi manager initialized successfully");
    return ESP_OK;
}

esp_err_t wifi_manager_start(void)
{
    if (!initialized) {
        ESP_LOGE(TAG, "❌ WiFi manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (running) {
        ESP_LOGW(TAG, "⚠️ WiFi manager already running");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "🚀 Starting WiFi manager...");

    running = true;
    status = WIFI_MGR_STATUS_INITIALIZING;

    // Create WiFi manager task
    xTaskCreate(wifi_manager_task, "wifi_mgr", 4096, NULL, 5, &wifi_task);

    ESP_LOGI(TAG, "✅ WiFi manager started");
    return ESP_OK;
}

esp_err_t wifi_manager_stop(void)
{
    if (!running) {
        ESP_LOGW(TAG, "⚠️ WiFi manager not running");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "🛑 Stopping WiFi manager...");

    running = false;

    // Stop WiFi
    esp_wifi_remote_stop();

    // Wait for task to stop
    if (wifi_task) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    status = WIFI_MGR_STATUS_DISABLED;

    ESP_LOGI(TAG, "✅ WiFi manager stopped");
    return ESP_OK;
}

esp_err_t wifi_manager_deinit(void)
{
    if (!initialized) {
        ESP_LOGW(TAG, "⚠️ WiFi manager not initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "🔧 Deinitializing WiFi manager...");

    // Stop if running
    wifi_manager_stop();

    // Deinitialize WiFi
    esp_wifi_remote_deinit();

    // Clean up network interfaces
    if (sta_netif) {
        esp_netif_destroy(sta_netif);
        sta_netif = NULL;
    }
    if (ap_netif) {
        esp_netif_destroy(ap_netif);
        ap_netif = NULL;
    }

    // Clean up synchronization objects
    if (event_group) {
        vEventGroupDelete(event_group);
        event_group = NULL;
    }
    if (mutex) {
        vSemaphoreDelete(mutex);
        mutex = NULL;
    }

    initialized = false;
    status = WIFI_MGR_STATUS_DISABLED;

    ESP_LOGI(TAG, "✅ WiFi manager deinitialized");
    return ESP_OK;
}

wifi_manager_status_t wifi_manager_get_status(void)
{
    return status;
}

esp_err_t wifi_manager_get_connection_info(wifi_connection_info_t* info)
{
    if (!info) {
        return ESP_ERR_INVALID_ARG;
    }

    memcpy(info, &connection_info, sizeof(wifi_connection_info_t));
    return ESP_OK;
}

esp_err_t wifi_manager_force_ap_mode(void)
{
    if (!initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "🔄 Forcing AP mode...");

    // Stop current operations
    esp_wifi_remote_stop();

    // Start AP mode
    return wifi_start_ap_fallback();
}

esp_err_t wifi_manager_force_station_mode(void)
{
    if (!initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "🔄 Forcing station mode...");

    // Stop current operations
    esp_wifi_remote_stop();

    // Configure and try station mode
    esp_err_t ret = wifi_configure_station();
    if (ret != ESP_OK) {
        return ret;
    }

    return wifi_try_connect_station();
}

bool wifi_manager_is_running(void)
{
    return running;
}

esp_err_t wifi_manager_get_diagnostics(char* buffer, size_t buffer_size)
{
    if (!buffer || buffer_size == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    wifi_ap_record_t ap_info;
    esp_err_t ret = esp_wifi_remote_sta_get_ap_info(&ap_info);
    
    // Use connection_info for diagnostics, ap_info is supplementary
    (void)ret;  // Suppress unused variable warning

    snprintf(buffer, buffer_size,
             "WiFi Manager Diagnostics:\n"
             "Status: %d\n"
             "Running: %s\n"
             "WiFi Enabled: %s\n"
             "Scan Only: %s\n"
             "Scanning Active: %s\n"
             "Connected SSID: %s\n"
             "IP Address: %s\n"
             "AP Mode: %s\n"
             "RSSI: %d\n"
             "Channel: %d\n"
             "Client Count: %d\n"
             "Station Retry Count: %d\n"
             "AP Retry Count: %d\n",
             status,
             running ? "Yes" : "No",
             wifi_enabled ? "Yes" : "No",
             scan_only_mode ? "Yes" : "No",
             scanning_active ? "Yes" : "No",
             connection_info.ssid,
             connection_info.ip_addr,
             connection_info.is_ap_mode ? "Yes" : "No",
             connection_info.rssi,
             connection_info.channel,
             connection_info.client_count,
             station_retry_count,
             ap_retry_count);

    return ESP_OK;
}

/**
 * @brief WiFi scan task for scan-only mode
 */
static void wifi_scan_task(void *pvParameters)
{
    ESP_LOGI(TAG, "🔍 WiFi scan task started");

    while (scanning_active) {
        if (wifi_enabled && scan_only_mode) {
            ESP_LOGD(TAG, "🔍 Starting WiFi scan...");
            
            esp_err_t ret = esp_wifi_remote_scan_start(NULL, false);
            if (ret == ESP_OK) {
                // Wait for scan completion
                EventBits_t bits = xEventGroupWaitBits(event_group,
                                                      WIFI_SCAN_DONE_BIT,
                                                      pdTRUE, pdFALSE,
                                                      pdMS_TO_TICKS(10000));
                
                if (bits & WIFI_SCAN_DONE_BIT) {
                    ESP_LOGD(TAG, "✅ WiFi scan completed");
                }
            } else {
                ESP_LOGW(TAG, "⚠️ Failed to start WiFi scan: %s", esp_err_to_name(ret));
            }
        }
        
        // Wait for next scan interval
        vTaskDelay(pdMS_TO_TICKS(config.scan_interval_ms));
    }

    ESP_LOGI(TAG, "🛑 WiFi scan task stopped");
    scan_task = NULL;
    vTaskDelete(NULL);
}

esp_err_t wifi_manager_enable(bool scan_only)
{
    if (!initialized) {
        ESP_LOGE(TAG, "❌ WiFi manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "📡 Enabling WiFi (scan_only: %s)", scan_only ? "true" : "false");

    xSemaphoreTake(mutex, portMAX_DELAY);

    wifi_enabled = true;
    scan_only_mode = scan_only;

    if (scan_only) {
        status = WIFI_MGR_STATUS_SCAN_ONLY;
        
        // Start WiFi for scanning only
        esp_err_t ret = esp_wifi_remote_set_mode(WIFI_MODE_STA);
        if (ret == ESP_OK) {
            ret = esp_wifi_remote_start();
        }
        
        if (ret == ESP_OK) {
            ret = wifi_manager_start_scanning();
        }
        
        xSemaphoreGive(mutex);
        return ret;
    } else {
        // Start full WiFi manager
        xSemaphoreGive(mutex);
        return wifi_manager_start();
    }
}

esp_err_t wifi_manager_disable(void)
{
    if (!initialized) {
        ESP_LOGE(TAG, "❌ WiFi manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "📡 Disabling WiFi");

    xSemaphoreTake(mutex, portMAX_DELAY);

    wifi_enabled = false;
    scan_only_mode = false;
    
    // Stop scanning if active
    if (scanning_active) {
        wifi_manager_stop_scanning();
    }
    
    // Stop WiFi manager if running
    if (running) {
        wifi_manager_stop();
    }
    
    // Stop WiFi hardware
    esp_wifi_remote_stop();
    
    status = WIFI_MGR_STATUS_DISABLED;
    memset(&connection_info, 0, sizeof(connection_info));
    
    xSemaphoreGive(mutex);
    
    ESP_LOGI(TAG, "✅ WiFi disabled successfully");
    return ESP_OK;
}

esp_err_t wifi_manager_enable_scan_only(void)
{
    return wifi_manager_enable(true);
}

esp_err_t wifi_manager_start_scanning(void)
{
    if (!wifi_enabled) {
        ESP_LOGE(TAG, "❌ WiFi not enabled");
        return ESP_ERR_INVALID_STATE;
    }

    if (scanning_active) {
        ESP_LOGW(TAG, "⚠️ Scanning already active");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "🔍 Starting WiFi scanning");

    scanning_active = true;

    // Create scan task
    BaseType_t ret = xTaskCreate(wifi_scan_task,
                                "wifi_scan",
                                4096,
                                NULL,
                                5,
                                &scan_task);

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "❌ Failed to create scan task");
        scanning_active = false;
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "✅ WiFi scanning started");
    return ESP_OK;
}

esp_err_t wifi_manager_stop_scanning(void)
{
    if (!scanning_active) {
        ESP_LOGW(TAG, "⚠️ Scanning not active");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "🛑 Stopping WiFi scanning");

    scanning_active = false;

    // Wait for scan task to finish
    if (scan_task != NULL) {
        vTaskDelay(pdMS_TO_TICKS(100)); // Give task time to exit
    }

    ESP_LOGI(TAG, "✅ WiFi scanning stopped");
    return ESP_OK;
}

bool wifi_manager_is_scan_only(void)
{
    return scan_only_mode;
}

esp_err_t wifi_manager_connect(const char* ssid, const char* password)
{
    if (!ssid || !password) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!initialized) {
        ESP_LOGE(TAG, "❌ WiFi manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "🔄 Connecting to WiFi network: %s", ssid);

    // Update configuration with new credentials
    config.sta_ssid = ssid;
    config.sta_password = password;

    // Stop any current WiFi operations
    esp_err_t ret = esp_wifi_remote_stop();
    if (ret != ESP_OK && ret != ESP_ERR_WIFI_NOT_STARTED) {
        ESP_LOGW(TAG, "⚠️ Failed to stop WiFi: %s", esp_err_to_name(ret));
    }

    // Configure station mode with new credentials
    ret = wifi_configure_station();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ Failed to configure station: %s", esp_err_to_name(ret));
        return ret;
    }

    // Start WiFi
    ret = esp_wifi_remote_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ Failed to start WiFi: %s", esp_err_to_name(ret));
        return ret;
    }

    // Initiate connection
    ret = esp_wifi_remote_connect();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ Failed to connect to WiFi: %s", esp_err_to_name(ret));
        return ret;
    }

    // Update status
    status = WIFI_MGR_STATUS_CONNECTING;
    
    ESP_LOGI(TAG, "✅ WiFi connection initiated for SSID: %s", ssid);
    return ESP_OK;
}

/**
 * @brief Print comprehensive WiFi status for P4
 */
void wifi_manager_print_comprehensive_status(void)
{
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "════════════════════════════════════════════════════════════════");
    ESP_LOGI(TAG, "                   ESP32-P4 WIFI STATUS                        ");
    ESP_LOGI(TAG, "════════════════════════════════════════════════════════════════");
    
    // WiFi Manager Status
    const char* status_names[] = {
        "DISABLED", "INITIALIZING", "SCAN_ONLY", "CONNECTING", 
        "CONNECTED", "DISCONNECTED", "AP_MODE", "ERROR"
    };
    
    const char* status_name = (status < sizeof(status_names)/sizeof(status_names[0])) ? 
                             status_names[status] : "UNKNOWN";
    
    ESP_LOGI(TAG, "🔧 WIFI MANAGER:");
    ESP_LOGI(TAG, "   📊 Status: %s", status_name);
    ESP_LOGI(TAG, "   🏃 Running: %s", running ? "Yes" : "No");
    ESP_LOGI(TAG, "   📡 WiFi Enabled: %s", wifi_enabled ? "Yes" : "No");
    ESP_LOGI(TAG, "   🔍 Scan Only: %s", scan_only_mode ? "Yes" : "No");
    ESP_LOGI(TAG, "   🔄 Scanning Active: %s", scanning_active ? "Yes" : "No");
    
    // Connection Information
    ESP_LOGI(TAG, "🔗 CONNECTION INFO:");
    if (connection_info.ssid[0] != '\0') {
        ESP_LOGI(TAG, "   📶 SSID: %s", connection_info.ssid);
        ESP_LOGI(TAG, "   🌐 IP Address: %s", connection_info.ip_addr);
        ESP_LOGI(TAG, "   📡 RSSI: %d dBm", connection_info.rssi);
        ESP_LOGI(TAG, "   📻 Channel: %d", connection_info.channel);
        ESP_LOGI(TAG, "   🏠 AP Mode: %s", connection_info.is_ap_mode ? "Yes" : "No");
        if (connection_info.is_ap_mode) {
            ESP_LOGI(TAG, "   👥 Connected Clients: %d", connection_info.client_count);
        }
    } else {
        ESP_LOGI(TAG, "   ❌ No connection information available");
    }
    
    // Configuration Details
    ESP_LOGI(TAG, "⚙️ CONFIGURATION:");
    ESP_LOGI(TAG, "   📋 Station SSID: %s", config.sta_ssid);
    ESP_LOGI(TAG, "   🏠 AP SSID: %s", config.ap_ssid);
    ESP_LOGI(TAG, "   📻 AP Channel: %d", config.ap_channel);
    ESP_LOGI(TAG, "   👥 Max AP Connections: %d", config.ap_max_connections);
    ESP_LOGI(TAG, "   🔄 Auto Fallback: %s", config.auto_fallback ? "Yes" : "No");
    ESP_LOGI(TAG, "   ⏰ Connection Timeout: %lu ms", config.sta_connect_timeout_ms);
    ESP_LOGI(TAG, "   🔄 Max Retries: %d", config.max_retry_count);
    ESP_LOGI(TAG, "   🔍 Scan Interval: %lu ms", config.scan_interval_ms);
    
    // Retry Status
    ESP_LOGI(TAG, "🔄 RETRY STATUS:");
    ESP_LOGI(TAG, "   📶 Station Retries: %d/%d", station_retry_count, config.max_retry_count);
    ESP_LOGI(TAG, "   🏠 AP Retries: %d", ap_retry_count);
    
    // ESP-Hosted Communication
    ESP_LOGI(TAG, "🔗 ESP-HOSTED:");
    ESP_LOGI(TAG, "   🔧 Initialized: %s", initialized ? "Yes" : "No");
    ESP_LOGI(TAG, "   📡 STA Interface: %s", sta_netif ? "Ready" : "Not Ready");
    ESP_LOGI(TAG, "   🏠 AP Interface: %s", ap_netif ? "Ready" : "Not Ready");
    
    ESP_LOGI(TAG, "════════════════════════════════════════════════════════════════");
    ESP_LOGI(TAG, "");
}
