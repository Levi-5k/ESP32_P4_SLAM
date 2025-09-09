/**
 * @file slam_web_server.c
 * @brief Web server implementation for ESP32-C6 slave device
 */

#include "slam_web_server.h"
#include "communication_protocol.h"
#include <esp_log.h>
#include <esp_http_server.h>
#include <esp_wifi.h>
#include <esp_timer.h>
#include <cJSON.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

static const char *TAG = "SLAM_WEB";

// Web server state
static httpd_handle_t server = NULL;
static bool server_running = false;
static web_server_config_t server_config;
static slam_data_cache_t data_cache = {0};
static SemaphoreHandle_t data_cache_mutex = NULL;

// WebSocket file descriptor for broadcasting
static int websocket_fd = -1;

/**
 * HTML Dashboard - Compact version optimized for C6 memory
 */
static const char* get_dashboard_html(void) 
{
    return "<!DOCTYPE html>"
        "<html><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width,initial-scale=1.0'>"
        "<title>ESP32-P4 SLAM</title><style>"
        "*{margin:0;padding:0;box-sizing:border-box}"
        "body{font-family:Arial,sans-serif;background:#1a1a1a;color:#fff;font-size:14px}"
        ".header{background:linear-gradient(135deg,#667eea,#764ba2);padding:15px;text-align:center}"
        ".container{max-width:1000px;margin:0 auto;padding:15px}"
        ".dashboard{display:grid;grid-template-columns:repeat(auto-fit,minmax(250px,1fr));gap:15px;margin-top:15px}"
        ".card{background:#2a2a2a;border-radius:8px;padding:15px;border:1px solid #444}"
        ".card h3{color:#667eea;margin-bottom:10px;font-size:16px}"
        ".status-grid{display:grid;grid-template-columns:1fr 1fr;gap:8px}"
        ".status-item{background:#333;padding:8px;border-radius:4px;font-size:12px}"
        ".status-value{font-weight:bold;color:#4ade80}"
        ".nav-tabs{display:flex;background:#333;border-radius:8px;overflow:hidden;margin:15px 0}"
        ".nav-tab{flex:1;padding:12px;text-align:center;cursor:pointer;background:#333;color:#ccc;border:none}"
        ".nav-tab.active{background:#667eea;color:white}"
        ".tab-content{display:none}"
        ".tab-content.active{display:block}"
        ".btn{padding:8px 16px;background:#667eea;color:white;border:none;border-radius:4px;cursor:pointer;margin:5px}"
        ".btn:hover{background:#5a6fd8}"
        ".btn-danger{background:#ef4444}"
        ".form-group{margin:10px 0}"
        ".form-group label{display:block;margin-bottom:5px;color:#ccc}"
        ".form-group input,.form-group select{width:100%;padding:8px;border:1px solid #555;background:#333;color:#fff;border-radius:4px}"
        ".live{color:#4ade80}"
        "#position-viz{width:100%;height:200px;background:#333;border-radius:4px;position:relative;overflow:hidden}"
        ".position-dot{width:6px;height:6px;background:#4ade80;border-radius:50%;position:absolute;transform:translate(-50%,-50%)}"
        ".trail{width:2px;height:2px;background:rgba(74,222,128,0.3);border-radius:50%;position:absolute;transform:translate(-50%,-50%)}"
        "</style></head><body>"
        "<div class='header'><h1>🎯 ESP32-P4 SLAM</h1><p>Real-time Navigation System</p></div>"
        "<div class='container'>"
        "<div class='nav-tabs'>"
        "<button class='nav-tab active' onclick='showTab(\"dashboard\")'>📊 Dashboard</button>"
        "<button class='nav-tab' onclick='showTab(\"settings\")'>⚙️ Settings</button>"
        "<button class='nav-tab' onclick='showTab(\"wifi\")'>📶 WiFi</button>"
        "<button class='nav-tab' onclick='showTab(\"control\")'>🎮 Control</button>"
        "</div>"
        
        "<div id='dashboard' class='tab-content active'>"
        "<div class='dashboard'>"
        "<div class='card'><h3>System Status</h3><div class='status-grid'>"
        "<div class='status-item'>P4 Connection: <span class='status-value' id='p4-conn'>--</span></div>"
        "<div class='status-item'>Uptime: <span class='status-value' id='uptime'>--</span></div>"
        "<div class='status-item'>Memory: <span class='status-value' id='memory'>--</span></div>"
        "<div class='status-item'>SLAM: <span class='status-value' id='slam-status'>--</span></div>"
        "</div></div>"
        
        "<div class='card'><h3>Position</h3>"
        "<div id='position-viz'></div>"
        "<div style='margin-top:10px;font-size:12px'>"
        "X: <span id='pos-x'>0.0</span>m | Y: <span id='pos-y'>0.0</span>m | Z: <span id='pos-z'>0.0</span>m"
        "</div></div>"
        
        "<div class='card'><h3>SLAM Info</h3><div class='status-grid'>"
        "<div class='status-item'>Features: <span class='status-value' id='features'>--</span></div>"
        "<div class='status-item'>Map Points: <span class='status-value' id='map-points'>--</span></div>"
        "<div class='status-item'>Keyframes: <span class='status-value' id='keyframes'>--</span></div>"
        "<div class='status-item'>Quality: <span class='status-value' id='quality'>--</span></div>"
        "</div></div>"
        
        "<div class='card'><h3>Sensors</h3><div class='status-grid'>"
        "<div class='status-item'>Camera: <span class='status-value' id='camera'>--</span></div>"
        "<div class='status-item'>IMU: <span class='status-value' id='imu'>--</span></div>"
        "<div class='status-item'>GPS: <span class='status-value' id='gps'>--</span></div>"
        "<div class='status-item'>SD Card: <span class='status-value' id='sd-card'>--</span></div>"
        "</div></div>"
        "</div></div>"
        
        "<div id='settings' class='tab-content'>"
        "<div class='card'><h3>SLAM Settings</h3>"
        "<div class='form-group'><label>Tracking Quality Threshold:</label><input type='range' min='0.1' max='1.0' step='0.1' value='0.5' id='quality-threshold'></div>"
        "<div class='form-group'><label>Max Features:</label><input type='number' min='100' max='2000' value='1000' id='max-features'></div>"
        "<button class='btn' onclick='updateSettings()'>Update Settings</button>"
        "</div>"
        "<div class='card'><h3>Camera Settings</h3>"
        "<div class='form-group'><label>Exposure:</label><input type='range' min='0' max='100' value='50' id='exposure'></div>"
        "<div class='form-group'><label>Brightness:</label><input type='range' min='-50' max='50' value='0' id='brightness'></div>"
        "<button class='btn' onclick='updateCameraSettings()'>Update Camera</button>"
        "</div></div>"
        
        "<div id='wifi' class='tab-content'>"
        "<div class='card'><h3>WiFi Networks</h3>"
        "<button class='btn' onclick='scanWiFi()'>🔍 Scan Networks</button>"
        "<div id='wifi-list' style='margin-top:15px'></div>"
        "</div>"
        "<div class='card'><h3>Connect to WiFi</h3>"
        "<div class='form-group'><label>SSID:</label><input type='text' id='wifi-ssid' placeholder='Network name'></div>"
        "<div class='form-group'><label>Password:</label><input type='password' id='wifi-password' placeholder='Password'></div>"
        "<button class='btn' onclick='connectWiFi()'>📶 Connect</button>"
        "</div></div>"
        
        "<div id='control' class='tab-content'>"
        "<div class='card'><h3>System Control</h3>"
        "<button class='btn' onclick='sendCommand(\"restart\")'>🔄 Restart P4</button>"
        "<button class='btn' onclick='sendCommand(\"calibrate_imu\")'>📐 Calibrate IMU</button>"
        "<button class='btn' onclick='sendCommand(\"calibrate_camera\")'>📷 Calibrate Camera</button>"
        "<button class='btn btn-danger' onclick='sendCommand(\"reset_slam\")'>🗑️ Reset SLAM</button>"
        "</div>"
        "<div class='card'><h3>Map Control</h3>"
        "<div class='form-group'><label>Map Name:</label><input type='text' id='map-name' placeholder='map_001'></div>"
        "<button class='btn' onclick='sendMapCommand(\"save\")'>💾 Save Map</button>"
        "<button class='btn' onclick='sendMapCommand(\"load\")'>📂 Load Map</button>"
        "<button class='btn btn-danger' onclick='sendMapCommand(\"clear\")'>🗑️ Clear Map</button>"
        "<button class='btn' onclick='sendMapCommand(\"list\")'>📋 List Maps</button>"
        "</div></div>"
        
        "</div>"
        "<script>"
        "let ws,positionTrail=[];"
        "function showTab(tab){document.querySelectorAll('.tab-content').forEach(t=>t.classList.remove('active'));document.querySelectorAll('.nav-tab').forEach(t=>t.classList.remove('active'));document.getElementById(tab).classList.add('active');event.target.classList.add('active')}"
        "function connectWebSocket(){ws=new WebSocket('ws://'+location.host+'/ws');ws.onmessage=e=>{const data=JSON.parse(e.data);updateDashboard(data)};ws.onclose=()=>setTimeout(connectWebSocket,2000)}"
        "function updateDashboard(data){"
        "if(data.heartbeat){const h=data.heartbeat;document.getElementById('p4-conn').textContent=data.connected?'Connected':'Disconnected';document.getElementById('uptime').textContent=Math.floor(h.uptime_ms/1000)+'s';document.getElementById('memory').textContent=Math.floor(h.free_heap_size/1024)+'KB';document.getElementById('slam-status').textContent=h.slam_status||'Unknown'}"
        "if(data.slam){const s=data.slam;document.getElementById('features').textContent=s.tracked_features||0;document.getElementById('map-points').textContent=s.map_points||0;document.getElementById('keyframes').textContent=s.keyframes||0;document.getElementById('quality').textContent=(s.tracking_quality||0).toFixed(2)}"
        "if(data.position){const p=data.position;document.getElementById('pos-x').textContent=p.position_x.toFixed(2);document.getElementById('pos-y').textContent=p.position_y.toFixed(2);document.getElementById('pos-z').textContent=p.position_z.toFixed(2);updatePositionViz(p.position_x,p.position_y)}"
        "if(data.telemetry){const t=data.telemetry;document.getElementById('camera').textContent=t.camera_fps+'fps';document.getElementById('imu').textContent='OK';document.getElementById('gps').textContent=t.gps_satellites+' sats'}}"
        "function updatePositionViz(x,y){const viz=document.getElementById('position-viz');const rect=viz.getBoundingClientRect();const centerX=rect.width/2;const centerY=rect.height/2;const scale=20;const pixelX=centerX+x*scale;const pixelY=centerY-y*scale;positionTrail.push({x:pixelX,y:pixelY});if(positionTrail.length>50)positionTrail.shift();viz.innerHTML='';positionTrail.forEach((pos,i)=>{const dot=document.createElement('div');dot.className=i===positionTrail.length-1?'position-dot':'trail';dot.style.left=pos.x+'px';dot.style.top=pos.y+'px';viz.appendChild(dot)})}"
        "function sendCommand(cmd){fetch('/api/command',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({command:cmd})})}"
        "function sendMapCommand(cmd){const mapName=document.getElementById('map-name').value||'default_map';fetch('/api/map',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({command:cmd,map_name:mapName})})}"
        "function updateSettings(){fetch('/api/settings',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({quality_threshold:document.getElementById('quality-threshold').value,max_features:document.getElementById('max-features').value})})}"
        "function updateCameraSettings(){fetch('/api/camera',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({exposure:document.getElementById('exposure').value,brightness:document.getElementById('brightness').value})})}"
        "function scanWiFi(){fetch('/api/wifi/scan',{method:'POST'}).then(r=>r.json()).then(data=>{const list=document.getElementById('wifi-list');list.innerHTML='';data.networks.forEach(net=>{const div=document.createElement('div');div.style.cssText='background:#333;padding:10px;margin:5px 0;border-radius:4px;cursor:pointer';div.innerHTML=`<strong>${net.ssid}</strong><br>Signal: ${net.rssi}dBm | Channel: ${net.channel}`;div.onclick=()=>{document.getElementById('wifi-ssid').value=net.ssid};list.appendChild(div)})})}"
        "function connectWiFi(){const ssid=document.getElementById('wifi-ssid').value;const password=document.getElementById('wifi-password').value;if(!ssid)return alert('Please enter SSID');fetch('/api/wifi/connect',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({ssid,password})})}"
        "connectWebSocket();"
        "setInterval(()=>{if(ws&&ws.readyState===WebSocket.OPEN)ws.send('ping')},1000);"
        "</script></body></html>";
}

/**
 * HTTP Handlers
 */
static esp_err_t root_handler(httpd_req_t *req)
{
    const char* html = get_dashboard_html();
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, html, strlen(html));
    return ESP_OK;
}

static esp_err_t websocket_handler(httpd_req_t *req)
{
    if (req->method == HTTP_GET) {
        ESP_LOGI(TAG, "WebSocket connection established");
        websocket_fd = httpd_req_to_sockfd(req);
        return ESP_OK;
    }
    
    httpd_ws_frame_t ws_pkt;
    uint8_t *buf = NULL;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;
    
    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "httpd_ws_recv_frame failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    if (ws_pkt.len) {
        buf = calloc(1, ws_pkt.len + 1);
        ws_pkt.payload = buf;
        ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
        if (ret == ESP_OK) {
            // Handle WebSocket message (ping/pong)
            if (strcmp((char*)ws_pkt.payload, "ping") == 0) {
                // Send current data
                slam_web_server_broadcast_data();
            }
        }
        free(buf);
    }
    
    return ret;
}

static esp_err_t api_command_handler(httpd_req_t *req)
{
    size_t recv_size = MIN(req->content_len, 512);
    char *content = malloc(recv_size + 1);
    int ret = httpd_req_recv(req, content, recv_size);
    if (ret <= 0) {
        free(content);
        return ESP_FAIL;
    }
    content[ret] = '\0';
    
    cJSON *json = cJSON_Parse(content);
    free(content);
    
    if (!json) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    
    cJSON *cmd = cJSON_GetObjectItem(json, "command");
    if (!cmd || !cJSON_IsString(cmd)) {
        cJSON_Delete(json);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid command");
        return ESP_FAIL;
    }
    
    // Send system command to P4
    system_cmd_msg_t sys_cmd = {0};
    if (strcmp(cmd->valuestring, "restart") == 0) {
        sys_cmd.command = SYS_CMD_RESTART;
    } else if (strcmp(cmd->valuestring, "calibrate_imu") == 0) {
        sys_cmd.command = SYS_CMD_CALIBRATE_IMU;
    } else if (strcmp(cmd->valuestring, "calibrate_camera") == 0) {
        sys_cmd.command = SYS_CMD_CALIBRATE_CAMERA;
    } else if (strcmp(cmd->valuestring, "reset_slam") == 0) {
        sys_cmd.command = SYS_CMD_RESET_SLAM;
    }
    
    slam_web_server_send_command(MSG_C6_TO_P4_SYSTEM_CMD, &sys_cmd, sizeof(sys_cmd));
    
    cJSON_Delete(json);
    httpd_resp_send(req, "OK", 2);
    return ESP_OK;
}

static esp_err_t api_wifi_scan_handler(httpd_req_t *req)
{
    // TODO: Trigger WiFi scan via P4
    slam_web_server_send_command(MSG_C6_TO_P4_WIFI_SCAN_REQ, NULL, 0);
    
    // Return cached results for now
    cJSON *json = cJSON_CreateObject();
    cJSON *networks = cJSON_CreateArray();
    
    if (xSemaphoreTake(data_cache_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        for (int i = 0; i < data_cache.wifi_scan_results.network_count; i++) {
            cJSON *network = cJSON_CreateObject();
            cJSON_AddStringToObject(network, "ssid", (char*)data_cache.wifi_scan_results.networks[i].ssid);
            cJSON_AddNumberToObject(network, "rssi", data_cache.wifi_scan_results.networks[i].rssi);
            cJSON_AddNumberToObject(network, "channel", data_cache.wifi_scan_results.networks[i].channel);
            cJSON_AddItemToArray(networks, network);
        }
        xSemaphoreGive(data_cache_mutex);
    }
    
    cJSON_AddItemToObject(json, "networks", networks);
    
    const char *json_string = cJSON_Print(json);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_string, strlen(json_string));
    
    free((void*)json_string);
    cJSON_Delete(json);
    return ESP_OK;
}

esp_err_t slam_web_server_init(const web_server_config_t* config)
{
    if (server_running) {
        ESP_LOGW(TAG, "Web server already running");
        return ESP_OK;
    }
    
    if (config) {
        server_config = *config;
    } else {
        server_config.port = 80;
        server_config.max_clients = 4;
        server_config.enable_cors = true;
        server_config.stack_size = 8192;
    }
    
    // Create data cache mutex
    data_cache_mutex = xSemaphoreCreateMutex();
    if (!data_cache_mutex) {
        ESP_LOGE(TAG, "Failed to create data cache mutex");
        return ESP_ERR_NO_MEM;
    }
    
    ESP_LOGI(TAG, "SLAM web server initialized");
    return ESP_OK;
}

esp_err_t slam_web_server_start(void)
{
    if (server_running) {
        return ESP_OK;
    }
    
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = server_config.port;
    config.max_open_sockets = server_config.max_clients;
    config.stack_size = server_config.stack_size;
    
    if (httpd_start(&server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server");
        return ESP_FAIL;
    }
    
    // Register handlers
    httpd_uri_t root_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = root_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &root_uri);
    
    httpd_uri_t ws_uri = {
        .uri = "/ws",
        .method = HTTP_GET,
        .handler = websocket_handler,
        .user_ctx = NULL,
        .is_websocket = true
    };
    httpd_register_uri_handler(server, &ws_uri);
    
    httpd_uri_t api_command_uri = {
        .uri = "/api/command",
        .method = HTTP_POST,
        .handler = api_command_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &api_command_uri);
    
    httpd_uri_t api_wifi_scan_uri = {
        .uri = "/api/wifi/scan",
        .method = HTTP_POST,
        .handler = api_wifi_scan_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &api_wifi_scan_uri);
    
    server_running = true;
    ESP_LOGI(TAG, "Web server started on port %d", server_config.port);
    return ESP_OK;
}

esp_err_t slam_web_server_update_data(const comm_message_t* message)
{
    if (!message || !data_cache_mutex) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (xSemaphoreTake(data_cache_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    
    uint32_t now = esp_timer_get_time() / 1000;
    data_cache.p4_connected = true;
    data_cache.last_p4_message_time = now;
    
    switch (message->header.msg_type) {
        case MSG_P4_TO_C6_HEARTBEAT:
            data_cache.heartbeat = message->payload.heartbeat;
            data_cache.heartbeat_timestamp = now;
            break;
            
        case MSG_P4_TO_C6_SLAM_STATUS:
            data_cache.slam_status = message->payload.slam_status;
            data_cache.slam_status_timestamp = now;
            break;
            
        case MSG_P4_TO_C6_POSITION:
            data_cache.position = message->payload.position;
            data_cache.position_timestamp = now;
            break;
            
        case MSG_P4_TO_C6_TELEMETRY:
            data_cache.telemetry = message->payload.telemetry;
            data_cache.telemetry_timestamp = now;
            break;
    }
    
    xSemaphoreGive(data_cache_mutex);
    return ESP_OK;
}

esp_err_t slam_web_server_broadcast_data(void)
{
    if (!server_running || websocket_fd < 0) {
        return ESP_ERR_INVALID_STATE;
    }
    
    cJSON *json = cJSON_CreateObject();
    
    if (xSemaphoreTake(data_cache_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        cJSON_AddBoolToObject(json, "connected", data_cache.p4_connected);
        
        // Add heartbeat data
        cJSON *heartbeat = cJSON_CreateObject();
        cJSON_AddNumberToObject(heartbeat, "uptime_ms", data_cache.heartbeat.uptime_ms);
        cJSON_AddNumberToObject(heartbeat, "free_heap_size", data_cache.heartbeat.free_heap_size);
        cJSON_AddNumberToObject(heartbeat, "slam_status", data_cache.heartbeat.slam_status);
        cJSON_AddItemToObject(json, "heartbeat", heartbeat);
        
        // Add SLAM status
        cJSON *slam = cJSON_CreateObject();
        cJSON_AddNumberToObject(slam, "tracked_features", data_cache.slam_status.tracked_features);
        cJSON_AddNumberToObject(slam, "map_points", data_cache.slam_status.map_points);
        cJSON_AddNumberToObject(slam, "keyframes", data_cache.slam_status.keyframes);
        cJSON_AddNumberToObject(slam, "tracking_quality", data_cache.slam_status.tracking_quality);
        cJSON_AddItemToObject(json, "slam", slam);
        
        // Add position
        cJSON *position = cJSON_CreateObject();
        cJSON_AddNumberToObject(position, "position_x", data_cache.position.position_x);
        cJSON_AddNumberToObject(position, "position_y", data_cache.position.position_y);
        cJSON_AddNumberToObject(position, "position_z", data_cache.position.position_z);
        cJSON_AddItemToObject(json, "position", position);
        
        // Add telemetry
        cJSON *telemetry = cJSON_CreateObject();
        cJSON_AddNumberToObject(telemetry, "camera_fps", data_cache.telemetry.camera_fps);
        cJSON_AddNumberToObject(telemetry, "gps_satellites", data_cache.telemetry.gps_satellites);
        cJSON_AddItemToObject(json, "telemetry", telemetry);
        
        xSemaphoreGive(data_cache_mutex);
    }
    
    const char *json_string = cJSON_Print(json);
    
    httpd_ws_frame_t ws_pkt;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.payload = (uint8_t*)json_string;
    ws_pkt.len = strlen(json_string);
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;
    
    esp_err_t ret = httpd_ws_send_frame_async(server, websocket_fd, &ws_pkt);
    
    free((void*)json_string);
    cJSON_Delete(json);
    
    return ret;
}

esp_err_t slam_web_server_send_command(uint8_t msg_type, const void* payload, uint16_t payload_size)
{
    // TODO: Implement actual command sending to P4 via ESP-Hosted
    ESP_LOGI(TAG, "Sending command to P4: type 0x%02x, size %d", msg_type, payload_size);
    return ESP_OK;
}

const slam_data_cache_t* slam_web_server_get_data_cache(void)
{
    return &data_cache;
}

bool slam_web_server_is_p4_connected(void)
{
    if (xSemaphoreTake(data_cache_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        bool connected = data_cache.p4_connected;
        xSemaphoreGive(data_cache_mutex);
        return connected;
    }
    return false;
}
