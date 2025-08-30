/*
 * Web Server Implementation
 * Real-time web interface with WebSocket support and captive portal
 */

#include "web_server.h"
#include "slam_core.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
// #include "esp_spiffs.h"  // Not available in ESP-IDF v5.5
#include "esp_vfs.h"
#include "nvs_flash.h"
#include "lwip/inet.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"
#include "cJSON.h"
#include <string.h>

static const char* TAG = "web_server";

// HTML pages embedded as strings
static const char* index_html = 
"<!DOCTYPE html><html><head><title>Visual SLAM Navigation</title>"
"<style>"
"body{font-family:Arial,sans-serif;margin:20px;background:#f0f0f0}"
".container{max-width:1200px;margin:0 auto;background:white;padding:20px;border-radius:10px;box-shadow:0 2px 10px rgba(0,0,0,0.1)}"
".header{text-align:center;color:#333;border-bottom:2px solid #007bff;padding-bottom:20px;margin-bottom:20px}"
".section{margin:20px 0;padding:15px;border:1px solid #ddd;border-radius:5px}"
".controls{display:grid;grid-template-columns:repeat(auto-fit,minmax(250px,1fr));gap:20px}"
".control-group{display:flex;flex-direction:column;gap:10px}"
"label{font-weight:bold;color:#555}"
"input,select,button{padding:8px;border:1px solid #ccc;border-radius:4px;font-size:14px}"
"button{background:#007bff;color:white;cursor:pointer;transition:background 0.3s}"
"button:hover{background:#0056b3}"
".status{display:inline-block;padding:4px 8px;border-radius:4px;font-size:12px;font-weight:bold}"
".status.ok{background:#d4edda;color:#155724}"
".status.error{background:#f8d7da;color:#721c24}"
"#frame-display{text-align:center;margin:20px 0}"
"#frame-display img{max-width:100%;height:auto;border:2px solid #ddd;border-radius:5px}"
"</style></head>"
"<body><div class='container'>"
"<div class='header'><h1>🚁 Visual SLAM Navigation System</h1>"
"<p>ESP32-P4-WIFI6 with OV5647 MIPI-CSI Camera</p></div>"

"<div class='section'><h2>📷 Camera Controls</h2>"
"<div class='controls'>"
"<div class='control-group'>"
"<label>Camera Frame:</label>"
"<button onclick='captureFrame()'>📸 Capture Frame</button>"
"<div id='frame-display'><p>Click capture to see camera frame</p></div>"
"</div>"
"<div class='control-group'>"
"<label>Brightness: <span id='brightness-value'>0</span></label>"
"<input type='range' id='brightness' min='-100' max='100' value='0' oninput='updateCameraSettings()'>"
"<label>Contrast: <span id='contrast-value'>0</span></label>"
"<input type='range' id='contrast' min='-100' max='100' value='0' oninput='updateCameraSettings()'>"
"<label>Saturation: <span id='saturation-value'>0</span></label>"
"<input type='range' id='saturation' min='-100' max='100' value='0' oninput='updateCameraSettings()'>"
"</div>"
"<div class='control-group'>"
"<label><input type='checkbox' id='auto-adjustment'> Auto-Adjustment</label>"
"<label>Target Brightness: <span id='target-brightness-value'>128</span></label>"
"<input type='range' id='target-brightness' min='50' max='200' value='128' oninput='updateCameraSettings()'>"
"<label>Adjustment Speed: <span id='adjustment-speed-value'>0.3</span></label>"
"<input type='range' id='adjustment-speed' min='0.1' max='1.0' step='0.1' value='0.3' oninput='updateCameraSettings()'>"
"</div></div></div>"

"<div class='section'><h2>🧠 SLAM Status</h2>"
"<p>System Status: <span id='slam-status' class='status ok'>Loading...</span></p>"
"<p>Features Tracked: <span id='features-tracked'>-</span></p>"
"<p>Keyframes: <span id='keyframes'>-</span></p>"
"<p>Processing Time: <span id='processing-time'>-</span> ms</p></div>"

"<div class='section'><h2>🌐 Network Info</h2>"
"<p>WiFi Mode: <span id='wifi-mode'>AP Mode</span></p>"
"<p>IP Address: <span id='ip-address'>192.168.4.1</span></p>"
"<p>Connected Clients: <span id='connected-clients'>-</span></p></div>"

"<script>"
"function updateSliderValue(sliderId, valueId) {"
"  const slider = document.getElementById(sliderId);"
"  const valueSpan = document.getElementById(valueId);"
"  valueSpan.textContent = slider.value;"
"}"

"function updateCameraSettings() {"
"  updateSliderValue('brightness', 'brightness-value');"
"  updateSliderValue('contrast', 'contrast-value');"
"  updateSliderValue('saturation', 'saturation-value');"
"  updateSliderValue('target-brightness', 'target-brightness-value');"
"  updateSliderValue('adjustment-speed', 'adjustment-speed-value');"
"  "
"  const settings = {"
"    brightness: parseInt(document.getElementById('brightness').value),"
"    contrast: parseInt(document.getElementById('contrast').value),"
"    saturation: parseInt(document.getElementById('saturation').value),"
"    auto_adjustment_enabled: document.getElementById('auto-adjustment').checked,"
"    target_brightness: parseInt(document.getElementById('target-brightness').value),"
"    adjustment_speed: parseFloat(document.getElementById('adjustment-speed').value)"
"  };"
"  "
"  fetch('/api/camera', {"
"    method: 'POST',"
"    headers: {'Content-Type': 'application/json'},"
"    body: JSON.stringify(settings)"
"  }).catch(err => console.error('Camera settings update failed:', err));"
"}"

"function captureFrame() {"
"  fetch('/camera?action=capture')"
"    .then(response => response.blob())"
"    .then(blob => {"
"      const url = URL.createObjectURL(blob);"
"      const img = document.createElement('img');"
"      img.src = url;"
"      document.getElementById('frame-display').innerHTML = '';"
"      document.getElementById('frame-display').appendChild(img);"
"    })"
"    .catch(err => {"
"      console.error('Frame capture failed:', err);"
"      document.getElementById('frame-display').innerHTML = '<p style=\"color:red;\">Frame capture failed</p>';"
"    });"
"}"

"function updateStatus() {"
"  fetch('/api/status')"
"    .then(r => r.json())"
"    .then(d => {"
"      document.getElementById('slam-status').textContent = d.slam_state || 'Unknown';"
"      document.getElementById('features-tracked').textContent = d.tracked_features || '-';"
"      document.getElementById('keyframes').textContent = d.keyframes || '-';"
"      document.getElementById('processing-time').textContent = d.processing_time || '-';"
"    })"
"    .catch(err => console.error('Status update failed:', err));"
"}"

"// Load current camera settings"
"fetch('/api/camera')"
"  .then(r => r.json())"
"  .then(d => {"
"    if (!d.error) {"
"      document.getElementById('brightness').value = d.brightness || 0;"
"      document.getElementById('contrast').value = d.contrast || 0;"
"      document.getElementById('saturation').value = d.saturation || 0;"
"      document.getElementById('auto-adjustment').checked = d.auto_adjustment_enabled || false;"
"      document.getElementById('target-brightness').value = d.target_brightness || 128;"
"      document.getElementById('adjustment-speed').value = d.adjustment_speed || 0.3;"
"      updateCameraSettings();"
"    }"
"  });"

"// Update status every 2 seconds"
"setInterval(updateStatus, 2000);"
"updateStatus();"
"</script>"
"</div></body></html>";

// Captive portal redirect page
static const char* captive_portal_html = 
"<!DOCTYPE html><html><head><title>WiFi Setup</title></head>"
"<body><h1>Visual SLAM WiFi Setup</h1>"
"<p>Please configure WiFi to continue.</p>"
"<button onclick=\"window.location.href='/setup'\">Setup WiFi</button>"
"</body></html>";

// Global web server state
static struct {
    web_server_config_t config;
    web_wifi_config_t wifi_config;
    httpd_handle_t server;
    web_server_status_t status;
    web_settings_t settings;
    web_server_event_callback_t event_callback;
    
    // WebSocket connections
    int websocket_fds[10];
    size_t websocket_count;
    
    // Real-time data
    realtime_data_t current_data;
    uint32_t sequence_number;
    
    bool initialized;
    SemaphoreHandle_t server_mutex;
    
} g_web_server = {0};

// Default configurations
static const web_server_config_t default_server_config = {
    .server_port = 80,
    .max_open_sockets = 10,
    .enable_cors = true,
    .websocket_timeout_ms = 30000,
    .max_resp_headers = 8,
    .max_resp_size = 1024
};

static const web_wifi_config_t default_wifi_config = {
    .ssid = "",
    .password = "",
    .ap_ssid = "VisualSLAM_Setup",
    .ap_password = "",  // No password for open AP
    .max_connections = 4,
    .mode = WEB_WIFI_MODE_CAPTIVE_PORTAL,
    .auto_fallback = true,
    .connection_timeout_ms = 15000
};

// Forward declarations
static esp_err_t setup_wifi_station(void);
static esp_err_t setup_wifi_ap(void);
static esp_err_t start_http_server(void);
static esp_err_t stop_http_server(void);
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);

// HTTP handlers
static esp_err_t index_handler(httpd_req_t *req);
static esp_err_t dashboard_handler(httpd_req_t *req);
static esp_err_t websocket_handler(httpd_req_t *req);
static esp_err_t camera_handler(httpd_req_t *req);
static esp_err_t camera_capture_frame(httpd_req_t *req);
static esp_err_t api_camera_handler(httpd_req_t *req);
static esp_err_t api_slam_handler(httpd_req_t *req);
static esp_err_t api_wifi_scan_handler(httpd_req_t *req);
static esp_err_t api_wifi_connect_handler(httpd_req_t *req);
static esp_err_t api_status_handler(httpd_req_t *req);
static esp_err_t captive_portal_handler(httpd_req_t *req);

esp_err_t web_server_init(const web_server_config_t* config) {
    if (g_web_server.initialized) {
        ESP_LOGW(TAG, "Web server already initialized");
        return ESP_OK;
    }
    
    // Set configuration
    if (config) {
        g_web_server.config = *config;
    } else {
        g_web_server.config = default_server_config;
    }
    
    g_web_server.wifi_config = default_wifi_config;
    
    // Create mutex
    g_web_server.server_mutex = xSemaphoreCreateMutex();
    if (!g_web_server.server_mutex) {
        ESP_LOGE(TAG, "Failed to create server mutex");
        return ESP_ERR_NO_MEM;
    }
    
    // Note: Network interface and event loop already initialized in main app
    
    // Initialize WiFi (non-blocking)
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_err_t wifi_init_ret = esp_wifi_init(&cfg);
    if (wifi_init_ret != ESP_OK) {
        ESP_LOGW(TAG, "WiFi initialization failed: %s", esp_err_to_name(wifi_init_ret));
        g_web_server.status.wifi_connected = false;
        goto skip_wifi_init;
    }
    
    esp_err_t event_ret = esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL);
    if (event_ret != ESP_OK) {
        ESP_LOGW(TAG, "WiFi event handler registration failed: %s", esp_err_to_name(event_ret));
    }
    
    event_ret = esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL);
    if (event_ret != ESP_OK) {
        ESP_LOGW(TAG, "IP event handler registration failed: %s", esp_err_to_name(event_ret));
    }
    
skip_wifi_init:
    
    // Initialize status
    memset(&g_web_server.status, 0, sizeof(web_server_status_t));
    strcpy(g_web_server.status.ip_address, "0.0.0.0");
    
    g_web_server.initialized = true;
    ESP_LOGI(TAG, "Web server initialized");
    
    return ESP_OK;
}

esp_err_t web_server_deinit(void) {
    if (!g_web_server.initialized) {
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Deinitializing web server...");
    
    // Stop HTTP server if running
    if (g_web_server.server != NULL) {
        stop_http_server();
    }
    
    // Stop WiFi
    esp_wifi_stop();
    esp_wifi_deinit();
    
    // Unregister event handlers
    esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler);
    esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler);
    
    // Clean up mutex
    if (g_web_server.server_mutex != NULL) {
        vSemaphoreDelete(g_web_server.server_mutex);
        g_web_server.server_mutex = NULL;
    }
    
    g_web_server.initialized = false;
    ESP_LOGI(TAG, "Web server deinitialized");
    
    return ESP_OK;
}

esp_err_t web_server_wifi_start(void) {
    if (!g_web_server.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    esp_err_t ret;
    
    if (g_web_server.wifi_config.mode == WEB_WIFI_MODE_STATION) {
        ret = setup_wifi_station();
    } else {
        ret = setup_wifi_ap();
    }
    
    if (ret == ESP_OK) {
        esp_err_t wifi_start_ret = esp_wifi_start();
        if (wifi_start_ret != ESP_OK) {
            ESP_LOGW(TAG, "WiFi start failed: %s", esp_err_to_name(wifi_start_ret));
            return wifi_start_ret;
        }
    } else {
        ESP_LOGW(TAG, "WiFi setup failed: %s", esp_err_to_name(ret));
    }
    
    return ret;
}

esp_err_t web_server_start(void) {
    if (!g_web_server.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    return start_http_server();
}

static esp_err_t setup_wifi_station(void) {
    esp_netif_create_default_wifi_sta();
    
    wifi_config_t wifi_config = {
        .sta = {
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    
    strncpy((char*)wifi_config.sta.ssid, g_web_server.wifi_config.ssid, sizeof(wifi_config.sta.ssid));
    strncpy((char*)wifi_config.sta.password, g_web_server.wifi_config.password, sizeof(wifi_config.sta.password));
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    
    g_web_server.status.current_wifi_mode = WIFI_MODE_STA;
    ESP_LOGI(TAG, "WiFi station mode configured for SSID: %s", g_web_server.wifi_config.ssid);
    
    return ESP_OK;
}

static esp_err_t setup_wifi_ap(void) {
    esp_netif_t* ap_netif = esp_netif_create_default_wifi_ap();
    
    // Configure AP IP address
    esp_netif_ip_info_t ip_info;
    IP4_ADDR(&ip_info.ip, 192, 168, 4, 1);
    IP4_ADDR(&ip_info.gw, 192, 168, 4, 1);
    IP4_ADDR(&ip_info.netmask, 255, 255, 255, 0);
    esp_netif_dhcps_stop(ap_netif);
    esp_netif_set_ip_info(ap_netif, &ip_info);
    esp_netif_dhcps_start(ap_netif);
    
    wifi_config_t wifi_config = {
        .ap = {
            .channel = 6,  // Use channel 6 for better compatibility
            .max_connection = g_web_server.wifi_config.max_connections,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK,
            .ssid_hidden = 0,  // Ensure SSID is visible
            .beacon_interval = 100  // Standard beacon interval
        },
    };
    
    strncpy((char*)wifi_config.ap.ssid, g_web_server.wifi_config.ap_ssid, sizeof(wifi_config.ap.ssid));
    wifi_config.ap.ssid_len = strlen(g_web_server.wifi_config.ap_ssid);
    strncpy((char*)wifi_config.ap.password, g_web_server.wifi_config.ap_password, sizeof(wifi_config.ap.password));
    
    if (strlen(g_web_server.wifi_config.ap_password) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    
    g_web_server.status.current_wifi_mode = WEB_WIFI_MODE_CAPTIVE_PORTAL;
    strcpy(g_web_server.status.ip_address, "192.168.4.1");
    
    ESP_LOGI(TAG, "WiFi AP mode configured: %s", g_web_server.wifi_config.ap_ssid);
    
    return ESP_OK;
}

static esp_err_t start_http_server(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = g_web_server.config.server_port;
    
    // Reduce memory usage by limiting connections and handlers
    config.max_open_sockets = 4;  // Reduced from default 7
    config.max_resp_headers = 8;   // Reduced from default 12
    config.max_uri_handlers = 15;  // Reduced from 20
    config.stack_size = 4096;      // Reduced stack size
    config.recv_wait_timeout = 5;  // Shorter timeout
    config.send_wait_timeout = 5;  // Shorter timeout
    
    // Start server
    esp_err_t ret = httpd_start(&g_web_server.server, &config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Register URI handlers
    httpd_uri_t handlers[] = {
        { .uri = "/", .method = HTTP_GET, .handler = index_handler },
        { .uri = "/dashboard", .method = HTTP_GET, .handler = dashboard_handler },
        { .uri = "/ws", .method = HTTP_GET, .handler = websocket_handler, .is_websocket = true },
        { .uri = "/camera", .method = HTTP_GET, .handler = camera_handler },
        { .uri = "/api/camera", .method = HTTP_POST, .handler = api_camera_handler },
        { .uri = "/api/slam", .method = HTTP_POST, .handler = api_slam_handler },
        { .uri = "/api/slam/reset", .method = HTTP_POST, .handler = api_slam_handler },
        { .uri = "/api/slam/save", .method = HTTP_POST, .handler = api_slam_handler },
        { .uri = "/api/slam/load", .method = HTTP_POST, .handler = api_slam_handler },
        { .uri = "/api/wifi/scan", .method = HTTP_GET, .handler = api_wifi_scan_handler },
        { .uri = "/api/wifi/connect", .method = HTTP_POST, .handler = api_wifi_connect_handler },
        { .uri = "/api/status", .method = HTTP_GET, .handler = api_status_handler },
    };
    
    for (size_t i = 0; i < sizeof(handlers) / sizeof(handlers[0]); i++) {
        ESP_ERROR_CHECK(httpd_register_uri_handler(g_web_server.server, &handlers[i]));
    }
    
    // Register catch-all handler for captive portal
    httpd_uri_t captive_uri = {
        .uri = "/*",
        .method = HTTP_GET,
        .handler = captive_portal_handler
    };
    httpd_register_uri_handler(g_web_server.server, &captive_uri);
    
    g_web_server.status.server_running = true;
    ESP_LOGI(TAG, "HTTP server started on port %d", config.server_port);
    
    return ESP_OK;
}

// HTTP Handlers
static esp_err_t index_handler(httpd_req_t *req) {
    if (g_web_server.status.current_wifi_mode == WEB_WIFI_MODE_CAPTIVE_PORTAL) {
        return captive_portal_handler(req);
    } else {
        return dashboard_handler(req);
    }
}

static esp_err_t dashboard_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    if (g_web_server.config.enable_cors) {
        httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    }
    return httpd_resp_send(req, index_html, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t captive_portal_handler(httpd_req_t *req) {
    // Check if this is a request for the main interface
    if (strstr(req->uri, "/dashboard") != NULL || strstr(req->uri, "/api/") != NULL) {
        return HTTPD_404_NOT_FOUND;
    }
    
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, captive_portal_html, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t websocket_handler(httpd_req_t *req) {
    if (req->method == HTTP_GET) {
        ESP_LOGI(TAG, "WebSocket connection established");
        
        // Store WebSocket file descriptor
        if (g_web_server.websocket_count < sizeof(g_web_server.websocket_fds) / sizeof(int)) {
            g_web_server.websocket_fds[g_web_server.websocket_count++] = httpd_req_to_sockfd(req);
            g_web_server.status.active_websockets = g_web_server.websocket_count;
        }
        
        return ESP_OK;
    }
    
    // Handle WebSocket frames
    httpd_ws_frame_t ws_pkt;
    uint8_t *buf = NULL;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    
    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "httpd_ws_recv_frame failed with %d", ret);
        return ret;
    }
    
    if (ws_pkt.len) {
        buf = calloc(1, ws_pkt.len + 1);
        if (buf == NULL) {
            ESP_LOGE(TAG, "Failed to calloc memory for buf");
            return ESP_ERR_NO_MEM;
        }
        ws_pkt.payload = buf;
        ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "httpd_ws_recv_frame failed with %d", ret);
            free(buf);
            return ret;
        }
    }
    
    // Handle different frame types
    if (ws_pkt.type == HTTPD_WS_TYPE_TEXT) {
        ESP_LOGI(TAG, "Received WebSocket message: %s", (char*)ws_pkt.payload);
        // Echo back for now
        httpd_ws_frame_t ws_response = {
            .type = HTTPD_WS_TYPE_TEXT,
            .payload = ws_pkt.payload,
            .len = ws_pkt.len
        };
        ret = httpd_ws_send_frame(req, &ws_response);
    }
    
    if (buf) {
        free(buf);
    }
    
    return ret;
}

static esp_err_t camera_handler(httpd_req_t *req) {
    // Check if this is a frame capture request
    size_t query_len = httpd_req_get_url_query_len(req);
    if (query_len > 0) {
        char* query_buf = malloc(query_len + 1);
        if (httpd_req_get_url_query_str(req, query_buf, query_len + 1) == ESP_OK) {
            char param[32];
            if (httpd_query_key_value(query_buf, "action", param, sizeof(param)) == ESP_OK) {
                if (strcmp(param, "capture") == 0) {
                    free(query_buf);
                    return camera_capture_frame(req);
                }
            }
        }
        free(query_buf);
    }
    
    // Default: return camera stream placeholder
    httpd_resp_set_type(req, "text/html");
    const char* html = 
        "<!DOCTYPE html><html><head><title>Camera Stream</title></head>"
        "<body><h1>Camera Stream</h1>"
        "<p>Live camera streaming not yet implemented</p>"
        "<button onclick=\"captureFrame()\">Capture Frame</button>"
        "<div id=\"frame-display\"></div>"
        "<script>"
        "function captureFrame() {"
        "  fetch('/camera?action=capture')"
        "    .then(response => response.blob())"
        "    .then(blob => {"
        "      const url = URL.createObjectURL(blob);"
        "      const img = document.createElement('img');"
        "      img.src = url;"
        "      img.style.maxWidth = '100%';"
        "      document.getElementById('frame-display').innerHTML = '';"
        "      document.getElementById('frame-display').appendChild(img);"
        "    })"
        "    .catch(err => console.error('Frame capture failed:', err));"
        "}"
        "</script></body></html>";
    return httpd_resp_send(req, html, strlen(html));
}

static esp_err_t camera_capture_frame(httpd_req_t *req) {
    // Capture a single frame from the camera
    camera_frame_t frame;
    esp_err_t ret = slam_core_get_frame(&frame);
    
    if (ret != ESP_OK) {
        // Generate a placeholder image (simple gradient)
        uint32_t width = 320, height = 240;
        uint32_t data_size = width * height * 3; // RGB24
        uint8_t* image_data = malloc(data_size);
        
        if (!image_data) {
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Memory allocation failed");
            return ESP_ERR_NO_MEM;
        }
        
        // Create a simple test pattern
        for (uint32_t y = 0; y < height; y++) {
            for (uint32_t x = 0; x < width; x++) {
                uint32_t idx = (y * width + x) * 3;
                image_data[idx + 0] = (x * 255) / width;     // Red gradient
                image_data[idx + 1] = (y * 255) / height;    // Green gradient
                image_data[idx + 2] = 128;                   // Blue constant
            }
        }
        
        // Convert to JPEG (simplified - just return as binary data)
        httpd_resp_set_type(req, "image/jpeg");
        httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=\"capture.jpg\"");
        
        // For now, return raw RGB data (browsers won't display this correctly, but it's a placeholder)
        ret = httpd_resp_send(req, (const char*)image_data, data_size);
        free(image_data);
        return ret;
    }
    
    // Send the actual camera frame
    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=\"capture.jpg\"");
    esp_err_t send_ret = httpd_resp_send(req, (const char*)frame.data, frame.data_size);
    
    // Release the frame
    slam_core_release_frame(&frame);
    
    return send_ret;
}

static esp_err_t api_camera_handler(httpd_req_t *req) {
    if (req->method == HTTP_GET) {
        // Return current camera settings
        httpd_resp_set_type(req, "application/json");
        cJSON *json = cJSON_CreateObject();
        
        // Get current camera configuration
        camera_config_t camera_config;
        esp_err_t ret = slam_core_get_camera_config(&camera_config);
        if (ret == ESP_OK) {
            cJSON_AddNumberToObject(json, "brightness", camera_config.brightness);
            cJSON_AddNumberToObject(json, "contrast", camera_config.contrast);
            cJSON_AddNumberToObject(json, "saturation", camera_config.saturation);
            cJSON_AddNumberToObject(json, "auto_exposure", camera_config.auto_exposure);
            cJSON_AddNumberToObject(json, "exposure_value", camera_config.exposure_value);
            cJSON_AddBoolToObject(json, "auto_adjustment_enabled", camera_config.auto_adjustment_enabled);
            cJSON_AddNumberToObject(json, "target_brightness", camera_config.target_brightness);
            cJSON_AddNumberToObject(json, "adjustment_speed", camera_config.adjustment_speed);
        } else {
            cJSON_AddStringToObject(json, "error", "Failed to get camera config");
        }
        
        char *json_string = cJSON_Print(json);
        httpd_resp_send(req, json_string, strlen(json_string));
        free(json_string);
        cJSON_Delete(json);
        return ESP_OK;
    }
    
    // Parse JSON request for camera settings
    char buf[512];
    int ret = httpd_req_recv(req, buf, sizeof(buf));
    if (ret <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid request");
        return ESP_FAIL;
    }
    
    buf[ret] = '\0';
    cJSON *json = cJSON_Parse(buf);
    if (json == NULL) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }
    
    // Process camera settings with auto-adjustment logic
    camera_config_t camera_config;
    esp_err_t config_ret = slam_core_get_camera_config(&camera_config);
    if (config_ret != ESP_OK) {
        cJSON_Delete(json);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to get camera config");
        return ESP_FAIL;
    }
    
    // Update settings from JSON
    cJSON *brightness = cJSON_GetObjectItem(json, "brightness");
    cJSON *contrast = cJSON_GetObjectItem(json, "contrast");
    cJSON *saturation = cJSON_GetObjectItem(json, "saturation");
    cJSON *auto_exposure = cJSON_GetObjectItem(json, "auto_exposure");
    cJSON *auto_adjustment = cJSON_GetObjectItem(json, "auto_adjustment_enabled");
    cJSON *target_brightness = cJSON_GetObjectItem(json, "target_brightness");
    cJSON *adjustment_speed = cJSON_GetObjectItem(json, "adjustment_speed");
    
    if (brightness && cJSON_IsNumber(brightness)) {
        camera_config.brightness = (int)cJSON_GetNumberValue(brightness);
    }
    if (contrast && cJSON_IsNumber(contrast)) {
        camera_config.contrast = (int)cJSON_GetNumberValue(contrast);
    }
    if (saturation && cJSON_IsNumber(saturation)) {
        camera_config.saturation = (int)cJSON_GetNumberValue(saturation);
    }
    if (auto_exposure && cJSON_IsBool(auto_exposure)) {
        camera_config.auto_exposure = cJSON_IsTrue(auto_exposure);
    }
    if (auto_adjustment && cJSON_IsBool(auto_adjustment)) {
        camera_config.auto_adjustment_enabled = cJSON_IsTrue(auto_adjustment);
    }
    if (target_brightness && cJSON_IsNumber(target_brightness)) {
        camera_config.target_brightness = (int)cJSON_GetNumberValue(target_brightness);
    }
    if (adjustment_speed && cJSON_IsNumber(adjustment_speed)) {
        camera_config.adjustment_speed = (float)cJSON_GetNumberValue(adjustment_speed);
    }
    
    // Apply settings to camera
    esp_err_t apply_ret = slam_core_set_camera_config(&camera_config);
    
    cJSON_Delete(json);
    
    httpd_resp_set_type(req, "application/json");
    if (apply_ret == ESP_OK) {
        const char* response = "{\"success\": true, \"message\": \"Camera settings updated\"}";
        return httpd_resp_send(req, response, strlen(response));
    } else {
        const char* response = "{\"success\": false, \"message\": \"Failed to apply camera settings\"}";
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, response);
        return ESP_FAIL;
    }
}

static esp_err_t api_slam_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "application/json");
    
    if (strstr(req->uri, "/reset") != NULL) {
        // TODO: Call slam_core_reset_map()
        const char* response = "{\"success\": true, \"message\": \"SLAM map reset\"}";
        return httpd_resp_send(req, response, strlen(response));
    } else if (strstr(req->uri, "/save") != NULL) {
        // TODO: Call slam_core_save_map_to_file()
        const char* response = "{\"success\": true, \"message\": \"SLAM map saved\"}";
        return httpd_resp_send(req, response, strlen(response));
    } else if (strstr(req->uri, "/load") != NULL) {
        // TODO: Call slam_core_load_map_from_file()
        const char* response = "{\"success\": true, \"message\": \"SLAM map loaded\"}";
        return httpd_resp_send(req, response, strlen(response));
    } else {
        // Handle SLAM configuration updates
        char buf[200];
        int ret = httpd_req_recv(req, buf, sizeof(buf));
        if (ret > 0) {
            buf[ret] = '\0';
            // TODO: Parse and apply SLAM settings
        }
        const char* response = "{\"success\": true}";
        return httpd_resp_send(req, response, strlen(response));
    }
}

static esp_err_t api_wifi_scan_handler(httpd_req_t *req) {
    // TODO: Implement WiFi scanning
    httpd_resp_set_type(req, "application/json");
    const char* response = "{\"networks\": [{\"ssid\": \"TestNetwork\", \"rssi\": -45, \"auth\": true}]}";
    return httpd_resp_send(req, response, strlen(response));
}

static esp_err_t api_wifi_connect_handler(httpd_req_t *req) {
    char buf[200];
    int ret = httpd_req_recv(req, buf, sizeof(buf));
    if (ret <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid request");
        return ESP_FAIL;
    }
    
    buf[ret] = '\0';
    cJSON *json = cJSON_Parse(buf);
    if (json == NULL) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }
    
    cJSON *ssid = cJSON_GetObjectItem(json, "ssid");
    cJSON *password = cJSON_GetObjectItem(json, "password");
    
    if (ssid && cJSON_IsString(ssid)) {
        strncpy(g_web_server.wifi_config.ssid, ssid->valuestring, sizeof(g_web_server.wifi_config.ssid));
        if (password && cJSON_IsString(password)) {
            strncpy(g_web_server.wifi_config.password, password->valuestring, sizeof(g_web_server.wifi_config.password));
        }
        
        // TODO: Attempt WiFi connection
        ESP_LOGI(TAG, "Attempting to connect to WiFi: %s", g_web_server.wifi_config.ssid);
    }
    
    cJSON_Delete(json);
    
    httpd_resp_set_type(req, "application/json");
    const char* response = "{\"success\": true, \"ip\": \"192.168.1.100\"}";
    return httpd_resp_send(req, response, strlen(response));
}

static esp_err_t api_status_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "application/json");
    
    cJSON *json = cJSON_CreateObject();
    cJSON *status = cJSON_CreateString("Running");
    cJSON *uptime = cJSON_CreateNumber(esp_timer_get_time() / 1000000);
    
    cJSON_AddItemToObject(json, "status", status);
    cJSON_AddItemToObject(json, "uptime", uptime);
    
    char *response = cJSON_Print(json);
    esp_err_t ret = httpd_resp_send(req, response, strlen(response));
    
    free(response);
    cJSON_Delete(json);
    
    return ret;
}

esp_err_t web_server_broadcast_data(const realtime_data_t* data) {
    if (!g_web_server.initialized || !data || g_web_server.websocket_count == 0) {
        return ESP_OK;
    }
    
    // Create JSON from real-time data
    cJSON *json = cJSON_CreateObject();
    
    // Add navigation state
    cJSON *nav_state = cJSON_CreateObject();
    cJSON_AddNumberToObject(nav_state, "latitude", data->nav_state.latitude);
    cJSON_AddNumberToObject(nav_state, "longitude", data->nav_state.longitude);
    cJSON_AddNumberToObject(nav_state, "altitude", data->nav_state.altitude);
    cJSON_AddItemToObject(json, "nav_state", nav_state);
    
    // Add GPS data
    cJSON *gps_data = cJSON_CreateObject();
    cJSON_AddNumberToObject(gps_data, "gps_fix_type", data->gps_data.gps_fix_type);
    cJSON_AddNumberToObject(gps_data, "satellites", data->gps_data.satellites);
    cJSON_AddItemToObject(json, "gps_data", gps_data);
    
    // Add SLAM pose
    cJSON *slam_pose = cJSON_CreateObject();
    cJSON_AddNumberToObject(slam_pose, "tracked_features", data->slam_pose.tracked_features);
    cJSON_AddBoolToObject(slam_pose, "is_lost", data->slam_pose.is_lost);
    cJSON_AddNumberToObject(slam_pose, "confidence", data->slam_pose.confidence);
    cJSON_AddItemToObject(json, "slam_pose", slam_pose);
    
    // Add system status
    cJSON *system_status = cJSON_CreateObject();
    cJSON_AddNumberToObject(system_status, "frame_rate_fps", data->system_status.frame_rate_fps);
    cJSON_AddNumberToObject(system_status, "cpu_usage_percent", data->system_status.cpu_usage_percent);
    cJSON_AddNumberToObject(system_status, "free_heap_bytes", data->system_status.free_heap_bytes);
    cJSON_AddItemToObject(json, "system_status", system_status);
    
    // Add IMU data
    cJSON *imu_data = cJSON_CreateObject();
    cJSON_AddNumberToObject(imu_data, "temp_c", data->imu_data.temp_c);
    cJSON_AddItemToObject(json, "imu_data", imu_data);
    
    // Add timestamp
    cJSON_AddNumberToObject(json, "timestamp_us", data->timestamp_us);
    cJSON_AddNumberToObject(json, "sequence_number", data->sequence_number);
    
    char *json_string = cJSON_Print(json);
    
    // Broadcast to all connected WebSocket clients
    httpd_ws_frame_t ws_pkt;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.payload = (uint8_t*)json_string;
    ws_pkt.len = strlen(json_string);
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;
    
    for (size_t i = 0; i < g_web_server.websocket_count; i++) {
        int fd = g_web_server.websocket_fds[i];
        if (fd > 0) {
            httpd_ws_send_frame_async(g_web_server.server, fd, &ws_pkt);
        }
    }
    
    free(json_string);
    cJSON_Delete(json);
    
    g_web_server.status.websocket_messages_sent++;
    
    return ESP_OK;
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT) {
        switch (event_id) {
            case WIFI_EVENT_STA_START:
                esp_wifi_connect();
                break;
            case WIFI_EVENT_STA_CONNECTED:
                ESP_LOGI(TAG, "Connected to WiFi");
                g_web_server.status.wifi_connected = true;
                break;
            case WIFI_EVENT_STA_DISCONNECTED:
                ESP_LOGI(TAG, "Disconnected from WiFi");
                g_web_server.status.wifi_connected = false;
                if (g_web_server.wifi_config.auto_fallback) {
                    ESP_LOGI(TAG, "Falling back to AP mode");
                    web_server_wifi_set_mode(WEB_WIFI_MODE_CAPTIVE_PORTAL);
                }
                break;
            case WIFI_EVENT_AP_START:
                ESP_LOGI(TAG, "WiFi AP started");
                break;
        }
    } else if (event_base == IP_EVENT) {
        if (event_id == IP_EVENT_STA_GOT_IP) {
            ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
            ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
            snprintf(g_web_server.status.ip_address, sizeof(g_web_server.status.ip_address), 
                    IPSTR, IP2STR(&event->ip_info.ip));
            g_web_server.status.wifi_connected = true;
        }
    }
}

esp_err_t web_server_wifi_set_mode(web_wifi_mode_t mode) {
    g_web_server.wifi_config.mode = mode;
    
    esp_wifi_stop();
    
    if (mode == WEB_WIFI_MODE_STATION) {
        return setup_wifi_station();
    } else {
        return setup_wifi_ap();
    }
}

// Compatibility function implementations
esp_err_t web_server_update_slam_data(const slam_result_t* data) {
    if (!data) return ESP_ERR_INVALID_ARG;
    // Convert slam_result_t to realtime_data_t and broadcast
    realtime_data_t realtime_data = {0};
    realtime_data.slam_pose = data->pose;
    realtime_data.sequence_number = g_web_server.sequence_number++;
    return web_server_broadcast_data(&realtime_data);
}

esp_err_t web_server_update_system_status(const system_status_t* status) {
    if (!status) return ESP_ERR_INVALID_ARG;
    // Convert system_status_t to realtime_data_t and broadcast
    realtime_data_t realtime_data = {0};
    realtime_data.system_status = *status;
    realtime_data.sequence_number = g_web_server.sequence_number++;
    return web_server_broadcast_data(&realtime_data);
}

esp_err_t web_server_broadcast_telemetry(const system_status_t* status) {
    return web_server_update_system_status(status);
}

static esp_err_t stop_http_server(void) {
    if (g_web_server.server) {
        esp_err_t ret = httpd_stop(g_web_server.server);
        if (ret == ESP_OK) {
            g_web_server.server = NULL;
            ESP_LOGI(TAG, "HTTP server stopped");
        } else {
            ESP_LOGE(TAG, "Failed to stop HTTP server: %s", esp_err_to_name(ret));
        }
        return ret;
    }
    return ESP_OK;
}
