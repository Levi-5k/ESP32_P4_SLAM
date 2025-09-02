/**
 * @file wifi_assistnow.c
 * @brief WiFi component implementation for AssistNow Offline downloads
 * @note Minimal WiFi implementation that connects, downloads data, then disconnects
 */

#include "wifi_assistnow.h"
#include <esp_log.h>
#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_netif.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <string.h>

static const char *TAG = "WIFI_ASSISTNOW";

// Component state
static bool initialized = false;
static wifi_assistnow_config_t current_config;
static wifi_assistnow_status_t current_status = WIFI_ASSISTNOW_DISCONNECTED;
static EventGroupHandle_t wifi_event_group = NULL;

// Event group bits
#define WIFI_CONNECTED_BIT    BIT0
#define WIFI_FAIL_BIT         BIT1

// Network interface
static esp_netif_t *netif = NULL;

/**
 * @brief WiFi event handler
 */
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                              int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "WiFi station started");
        current_status = WIFI_ASSISTNOW_CONNECTING;
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "WiFi disconnected");
        current_status = WIFI_ASSISTNOW_DISCONNECTED;
        xEventGroupSetBits(wifi_event_group, WIFI_FAIL_BIT);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        current_status = WIFI_ASSISTNOW_CONNECTED;
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

esp_err_t wifi_assistnow_init(const wifi_assistnow_config_t *config)
{
    if (initialized) {
        ESP_LOGW(TAG, "WiFi AssistNow already initialized");
        return ESP_OK;
    }

    if (!config) {
        ESP_LOGE(TAG, "Invalid configuration");
        return ESP_ERR_INVALID_ARG;
    }

    // Check if WiFi is available in the build
    #ifndef CONFIG_ESP_WIFI_ENABLED
    ESP_LOGE(TAG, "WiFi is not enabled in sdkconfig - AssistNow downloads will not work");
    ESP_LOGE(TAG, "To enable WiFi, set CONFIG_ESP_WIFI_ENABLED=y in sdkconfig");
    ESP_LOGE(TAG, "Note: WiFi and SDMMC cannot be used simultaneously on ESP32-P4");
    return ESP_ERR_NOT_SUPPORTED;
    #endif

    // Copy configuration
    memcpy(&current_config, config, sizeof(wifi_assistnow_config_t));

    ESP_LOGI(TAG, "Initializing WiFi for AssistNow downloads");

    #ifdef CONFIG_ESP_WIFI_ENABLED
    // Initialize TCP/IP stack
    ESP_ERROR_CHECK(esp_netif_init());

    // Create default event loop
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Create network interface
    netif = esp_netif_create_default_wifi_sta();
    if (!netif) {
        ESP_LOGE(TAG, "Failed to create network interface");
        return ESP_FAIL;
    }

    // Initialize WiFi with default configuration
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Register event handlers
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                      ESP_EVENT_ANY_ID,
                                                      &wifi_event_handler,
                                                      NULL,
                                                      NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                      IP_EVENT_STA_GOT_IP,
                                                      &wifi_event_handler,
                                                      NULL,
                                                      NULL));

    // Create event group
    wifi_event_group = xEventGroupCreate();
    if (!wifi_event_group) {
        ESP_LOGE(TAG, "Failed to create event group");
        return ESP_FAIL;
    }

    // Set WiFi mode to station
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    initialized = true;
    ESP_LOGI(TAG, "✅ WiFi AssistNow initialized");
    return ESP_OK;
    #else
    // WiFi not enabled - return error
    ESP_LOGE(TAG, "WiFi is not enabled in sdkconfig - AssistNow downloads will not work");
    ESP_LOGE(TAG, "To enable WiFi, set CONFIG_ESP_WIFI_ENABLED=y in sdkconfig");
    ESP_LOGE(TAG, "Note: WiFi and SDMMC cannot be used simultaneously on ESP32-P4");
    return ESP_ERR_NOT_SUPPORTED;
    #endif
}

esp_err_t wifi_assistnow_connect(void)
{
    if (!initialized) {
        ESP_LOGE(TAG, "WiFi AssistNow not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    #ifndef CONFIG_ESP_WIFI_ENABLED
    ESP_LOGE(TAG, "WiFi is not enabled in sdkconfig");
    return ESP_ERR_NOT_SUPPORTED;
    #endif

    if (current_status == WIFI_ASSISTNOW_CONNECTED) {
        ESP_LOGI(TAG, "WiFi already connected");
        return ESP_OK;
    }

    #ifdef CONFIG_ESP_WIFI_ENABLED
    ESP_LOGI(TAG, "Connecting to WiFi: %s", current_config.ssid);

    // Configure WiFi connection
    wifi_config_t wifi_config = {
        .sta = {
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };

    // Copy SSID and password
    strcpy((char*)wifi_config.sta.ssid, current_config.ssid);
    strcpy((char*)wifi_config.sta.password, current_config.password);

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    // Wait for connection or timeout
    EventBits_t bits = xEventGroupWaitBits(wifi_event_group,
                                          WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                          pdFALSE,
                                          pdFALSE,
                                          pdMS_TO_TICKS(current_config.connect_timeout_ms));

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "✅ WiFi connected successfully");
        return ESP_OK;
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGE(TAG, "❌ WiFi connection failed");
        current_status = WIFI_ASSISTNOW_FAILED;
        return ESP_FAIL;
    } else {
        ESP_LOGE(TAG, "❌ WiFi connection timeout");
        current_status = WIFI_ASSISTNOW_FAILED;
        return ESP_ERR_TIMEOUT;
    }
    #else
    return ESP_ERR_NOT_SUPPORTED;
    #endif
}

esp_err_t wifi_assistnow_disconnect(void)
{
    if (!initialized) {
        ESP_LOGE(TAG, "WiFi AssistNow not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    #ifndef CONFIG_ESP_WIFI_ENABLED
    ESP_LOGW(TAG, "WiFi is not enabled in sdkconfig");
    return ESP_OK; // Not an error, just not supported
    #endif

    if (current_status != WIFI_ASSISTNOW_CONNECTED) {
        ESP_LOGI(TAG, "WiFi not connected");
        return ESP_OK;
    }

    #ifdef CONFIG_ESP_WIFI_ENABLED
    ESP_LOGI(TAG, "Disconnecting from WiFi");

    // Stop WiFi
    ESP_ERROR_CHECK(esp_wifi_stop());

    current_status = WIFI_ASSISTNOW_DISCONNECTED;
    ESP_LOGI(TAG, "✅ WiFi disconnected");
    return ESP_OK;
    #else
    current_status = WIFI_ASSISTNOW_DISCONNECTED;
    return ESP_OK;
    #endif
}

wifi_assistnow_status_t wifi_assistnow_get_status(void)
{
    return current_status;
}

bool wifi_assistnow_is_connected(void)
{
    return (current_status == WIFI_ASSISTNOW_CONNECTED);
}

void wifi_assistnow_deinit(void)
{
    if (!initialized) {
        return;
    }

    ESP_LOGI(TAG, "Deinitializing WiFi AssistNow");

    // Disconnect if connected
    wifi_assistnow_disconnect();

    // Clean up event group
    if (wifi_event_group) {
        vEventGroupDelete(wifi_event_group);
        wifi_event_group = NULL;
    }

    // Clean up network interface
    if (netif) {
        esp_netif_destroy(netif);
        netif = NULL;
    }

    // Deinitialize WiFi
    esp_wifi_deinit();

    initialized = false;
    current_status = WIFI_ASSISTNOW_DISCONNECTED;

    ESP_LOGI(TAG, "✅ WiFi AssistNow deinitialized");
}
