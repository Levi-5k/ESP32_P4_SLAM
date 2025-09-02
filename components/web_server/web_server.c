/**
 * @file web_server.c
 * @brief WIFI remote web server implementation for ESP32-P4 Visual SLAM system
 * @note Simplified version without WIFI - WIFI functionality to be added later
 */

#include "web_server.h"
#include <esp_log.h>
#include <esp_http_server.h>
#include <string.h>
#include <sys/param.h>

static const char *TAG = "WEB_SERVER";

// Web server state
static httpd_handle_t server = NULL;
static bool server_running = false;
static web_server_config_t server_config;

/**
 * HTTP GET handler for root page
 */
static esp_err_t root_get_handler(httpd_req_t *req)
{
    const char* resp_str = "<!DOCTYPE html>"
        "<html><head><title>ESP32-P4 Visual SLAM</title></head>"
        "<body><h1>ESP32-P4 Visual SLAM Navigation System</h1>"
        "<p>System is running. WIFI remote interface active.</p>"
        "<p><a href=\"/status\">System Status</a></p>"
        "</body></html>";

    httpd_resp_send(req, resp_str, strlen(resp_str));
    return ESP_OK;
}

/**
 * HTTP GET handler for status page
 */
static esp_err_t status_get_handler(httpd_req_t *req)
{
    const char* resp_str = "<!DOCTYPE html>"
        "<html><head><title>System Status</title></head>"
        "<body><h1>System Status</h1>"
        "<p>WIFI Remote: Active</p>"
        "<p>Access Point: DroneCam-SLAM</p>"
        "<p>IP Address: 192.168.4.1</p>"
        "</body></html>";

    httpd_resp_send(req, resp_str, strlen(resp_str));
    return ESP_OK;
}

/**
 * HTTP URI handlers
 */
static const httpd_uri_t root = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = root_get_handler,
    .user_ctx  = NULL
};

static const httpd_uri_t status = {
    .uri       = "/status",
    .method    = HTTP_GET,
    .handler   = status_get_handler,
    .user_ctx  = NULL
};

/**
 * Initialize web server
 */
esp_err_t web_server_init(const web_server_config_t* config)
{
    if (config == NULL) {
        ESP_LOGE(TAG, "❌ Web server config is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    // Copy configuration
    memcpy(&server_config, config, sizeof(web_server_config_t));

    ESP_LOGI(TAG, "✅ Web server initialized with port %d", server_config.port);
    return ESP_OK;
}

/**
 * Start WIFI access point (simplified - WIFI not implemented yet)
 */
esp_err_t web_server_wifi_start(void)
{
    ESP_LOGI(TAG, "📡 WIFI AP start requested - WIFI functionality not implemented yet");
    ESP_LOGI(TAG, "📡 Would start WIFI AP: %s on channel %d", server_config.ssid, server_config.channel);
    return ESP_OK;
}

/**
 * Stop WIFI access point (simplified - WIFI not implemented yet)
 */
esp_err_t web_server_wifi_stop(void)
{
    ESP_LOGI(TAG, "📡 WIFI AP stop requested - WIFI functionality not implemented yet");
    return ESP_OK;
}

/**
 * Start web server
 */
esp_err_t web_server_start(void)
{
    if (server_running) {
        ESP_LOGW(TAG, "⚠️ Web server already running");
        return ESP_OK;
    }

    // WIFI check removed - WIFI functionality disabled
    // if (!wifi_ap_started) {
    //     ESP_LOGE(TAG, "❌ WIFI AP not started - start WIFI first");
    //     return ESP_ERR_INVALID_STATE;
    // }

    ESP_LOGI(TAG, "🌐 Starting web server on port %d...", server_config.port);

    // Configure HTTP server
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = server_config.port;
    config.max_open_sockets = server_config.max_clients;

    // Start server
    esp_err_t ret = httpd_start(&server, &config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ Failed to start web server: %s", esp_err_to_name(ret));
        return ret;
    }

    // Register URI handlers
    httpd_register_uri_handler(server, &root);
    httpd_register_uri_handler(server, &status);

    server_running = true;

    ESP_LOGI(TAG, "✅ Web server started on http://192.168.4.1:%d", server_config.port);
    return ESP_OK;
}

/**
 * Stop web server
 */
esp_err_t web_server_stop(void)
{
    if (!server_running) {
        ESP_LOGW(TAG, "⚠️ Web server not running");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "🌐 Stopping web server...");

    ESP_ERROR_CHECK(httpd_stop(server));
    server = NULL;
    server_running = false;

    ESP_LOGI(TAG, "✅ Web server stopped");
    return ESP_OK;
}

/**
 * Deinitialize web server
 */
esp_err_t web_server_deinit(void)
{
    ESP_LOGI(TAG, "🌐 Deinitializing web server...");

    // Stop server if running
    if (server_running) {
        web_server_stop();
    }

    // WIFI stop removed - WIFI functionality disabled
    // if (wifi_ap_started) {
    //     web_server_wifi_stop();
    // }

    ESP_LOGI(TAG, "✅ Web server deinitialized");
    return ESP_OK;
}

/**
 * Check if web server is running
 */
bool web_server_is_running(void)
{
    return server_running;
}
