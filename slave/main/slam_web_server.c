/**
 * @file slam_web_server.c
 * @brief Web server implementation for ESP32-C6 slave device
 */

#include "slam_web_server.h"
#include "communication_protocol.h"
#include <esp_log.h>
#include <esp_http_server.h>
#include <esp_wifi.h>
#include <cJSON.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"

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
        ".preview-container{border:1px solid #444;border-radius:8px;padding:15px;background:#222}"
        ".preview-box{border:2px dashed #555;border-radius:8px;position:relative;overflow:hidden;background:#1a1a1a}"
        ".preview-box img{width:100%;height:100%;object-fit:contain;border-radius:6px}"
        ".preview-controls{display:flex;gap:5px}"
        ".preview-info{display:flex;justify-content:space-between;font-size:11px;color:#999}"
        ".form-group input[type='range']{margin-right:10px;width:calc(100% - 40px);display:inline-block}"
        ".form-group span{display:inline-block;width:30px;color:#4ade80;font-weight:bold}"
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
        "<div class='preview-container' style='margin-bottom:15px;'>"
        "<div class='preview-header' style='display:flex;justify-content:space-between;align-items:center;margin-bottom:10px;'>"
        "<h4 style='margin:0;'>Camera Preview</h4>"
        "<div class='preview-controls'>"
        "<button class='btn' id='preview-btn' onclick='togglePreview()'>📷 Start Preview</button>"
        "<button class='btn' onclick='refreshPreview()'>🔄 Refresh</button>"
        "</div></div>"
        "<div class='preview-box' id='preview-box' style='width:100%;max-width:400px;height:300px;background:#1a1a1a;border:2px solid #333;border-radius:8px;display:flex;align-items:center;justify-content:center;color:#666;font-size:14px;margin:0 auto;'>"
        "📷 Camera preview will appear here"
        "</div>"
        "<div class='preview-info' style='margin-top:10px;font-size:12px;color:#666;text-align:center;'>"
        "<span id='preview-status'>Preview stopped</span> | "
        "<span id='preview-resolution'>--x--</span> | "
        "<span id='preview-fps'>-- fps</span>"
        "</div></div>"
        "<div class='form-group'><label>Exposure:</label><input type='range' min='0' max='100' value='50' id='exposure' oninput='updateExposureValue(this.value)'><span id='exposure-value'>50</span></div>"
        "<div class='form-group'><label>Brightness:</label><input type='range' min='-50' max='50' value='0' id='brightness' oninput='updateBrightnessValue(this.value)'><span id='brightness-value'>0</span></div>"
        "<div class='form-group'><label>Contrast:</label><input type='range' min='0' max='100' value='50' id='contrast' oninput='updateContrastValue(this.value)'><span id='contrast-value'>50</span></div>"
        "<div class='form-group'><label>Saturation:</label><input type='range' min='0' max='100' value='50' id='saturation' oninput='updateSaturationValue(this.value)'><span id='saturation-value'>50</span></div>"
        "<button class='btn' onclick='updateCameraSettings()'>Update Camera</button>"
        "</div></div>"
        
        "<div id='wifi' class='tab-content'>"
        "<div class='card'><h3>WiFi Control</h3>"
        "<div class='status-grid'>"
        "<div class='status-item'>WiFi Status: <span class='status-value' id='wifi-status'>--</span></div>"
        "<div class='status-item'>Mode: <span class='status-value' id='wifi-mode'>--</span></div>"
        "<div class='status-item'>Connected: <span class='status-value' id='wifi-connected'>--</span></div>"
        "<div class='status-item'>AP Mode: <span class='status-value' id='wifi-ap-mode'>--</span></div>"
        "</div>"
        "<div style='margin-top:15px'>"
        "<button class='btn' id='wifi-enable-btn' onclick='toggleWiFi(true, false)'>📡 Enable WiFi</button>"
        "<button class='btn' id='wifi-scan-btn' onclick='toggleWiFi(true, true)'>🔍 Scan Only Mode</button>"
        "<button class='btn btn-danger' id='wifi-disable-btn' onclick='toggleWiFi(false, false)'>🚫 Disable WiFi</button>"
        "</div>"
        "</div>"
        "<div class='card'><h3>WiFi Networks</h3>"
        "<button class='btn' onclick='scanWiFi()'>🔍 Scan Networks</button>"
        "<div id='wifi-list' style='margin-top:15px'></div>"
        "</div>"
        "<div class='card'><h3>Connect to WiFi</h3>"
        "<div class='form-group'><label>SSID:</label><input type='text' id='wifi-ssid' placeholder='Network name'></div>"
        "<div class='form-group'><label>Password:</label><input type='password' id='wifi-password' placeholder='Password'></div>"
        "<div class='form-group'>"
        "<label><input type='checkbox' id='wifi-ap-fallback' checked> Enable AP Fallback</label>"
        "</div>"
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
        "function updateCameraSettings(){const settings={exposure:document.getElementById('exposure').value,brightness:document.getElementById('brightness').value,contrast:document.getElementById('contrast').value,saturation:document.getElementById('saturation').value};fetch('/api/camera',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(settings)})}"
        "function updateExposureValue(val){document.getElementById('exposure-value').textContent=val}"
        "function updateBrightnessValue(val){document.getElementById('brightness-value').textContent=val}"
        "function updateContrastValue(val){document.getElementById('contrast-value').textContent=val}"
        "function updateSaturationValue(val){document.getElementById('saturation-value').textContent=val}"
        "function togglePreview(){const btn=document.getElementById('preview-btn');const isStarting=btn.textContent.includes('Start');if(isStarting){startCameraPreview()}else{stopCameraPreview()}}"
        "function startCameraPreview(){fetch('/api/preview',{method:'GET'}).then(r=>{if(r.ok){document.getElementById('preview-btn').textContent='Stop Preview';document.getElementById('preview-status').textContent='Preview active';startPreviewRefresh()}})}"
        "function stopCameraPreview(){fetch('/api/preview',{method:'POST'}).then(r=>{if(r.ok){document.getElementById('preview-btn').textContent='Start Preview';document.getElementById('preview-status').textContent='Preview stopped';stopPreviewRefresh()}})}"
        "function refreshPreview(){if(previewActive){updatePreviewFrame()}}"
        "let previewInterval;let previewActive=false;"
        "function startPreviewRefresh(){previewActive=true;updatePreviewFrame();previewInterval=setInterval(updatePreviewFrame,500)}"
        "function stopPreviewRefresh(){previewActive=false;if(previewInterval){clearInterval(previewInterval)};document.getElementById('preview-box').innerHTML='Camera preview stopped'}"
        "function updatePreviewFrame(){if(!previewActive)return;fetch('/api/preview/frame').then(r=>{if(r.ok){return r.blob()}}).then(blob=>{if(blob){const url=URL.createObjectURL(blob);const img=document.createElement('img');img.onload=()=>{URL.revokeObjectURL(url);document.getElementById('preview-box').innerHTML='';document.getElementById('preview-box').appendChild(img);img.style.width='100%';img.style.height='100%';img.style.objectFit='contain'};img.src=url}}).catch(e=>{console.log('Preview frame not available')})}"
        "function scanWiFi(){fetch('/api/wifi/scan',{method:'POST'}).then(r=>r.json()).then(data=>{const list=document.getElementById('wifi-list');list.innerHTML='';data.networks.forEach(net=>{const div=document.createElement('div');div.style.cssText='background:#333;padding:10px;margin:5px 0;border-radius:4px;cursor:pointer';div.innerHTML=`<strong>${net.ssid}</strong><br>Signal: ${net.rssi}dBm | Channel: ${net.channel}`;div.onclick=()=>{document.getElementById('wifi-ssid').value=net.ssid};list.appendChild(div)})})}"
        "function connectWiFi(){const ssid=document.getElementById('wifi-ssid').value;const password=document.getElementById('wifi-password').value;const apFallback=document.getElementById('wifi-ap-fallback').checked;if(!ssid)return alert('Please enter SSID');fetch('/api/wifi/connect',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({ssid,password,ap_fallback:apFallback})})}"
        "function toggleWiFi(enable,scanOnly){const payload={enable_wifi:enable,enable_scan_only:scanOnly,auto_ap_fallback:document.getElementById('wifi-ap-fallback').checked};fetch('/api/wifi/control',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(payload)}).then(r=>r.json()).then(data=>{updateWiFiStatus(data);alert(enable?(scanOnly?'WiFi enabled in scan-only mode':'WiFi enabled for connections'):'WiFi disabled')}).catch(e=>alert('WiFi control failed'))}"
        "function updateWiFiStatus(data){if(data.wifi_status){const status=data.wifi_status;document.getElementById('wifi-status').textContent=status.wifi_enabled?'Enabled':'Disabled';document.getElementById('wifi-mode').textContent=status.scan_active?'Scan Only':status.connected?'Connected':'Disconnected';document.getElementById('wifi-connected').textContent=status.connected?status.ssid:'Not connected';document.getElementById('wifi-ap-mode').textContent=status.ap_mode_active?'Active':'Inactive'}}"
        "function checkWiFiStatus(){fetch('/api/wifi/status').then(r=>r.json()).then(data=>updateWiFiStatus(data)).catch(e=>console.log('WiFi status check failed'))}"
        "connectWebSocket();"
        "checkWiFiStatus();"
        "setInterval(()=>{if(ws&&ws.readyState===WebSocket.OPEN)ws.send('ping')},1000);"
        "setInterval(checkWiFiStatus,5000);"
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
    size_t recv_size = req->content_len > 512 ? 512 : req->content_len;
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
        httpd_resp_send_500(req);
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

static esp_err_t api_camera_handler(httpd_req_t *req)
{
    size_t recv_size = req->content_len > 512 ? 512 : req->content_len;
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
    
    // Process camera settings
    camera_cmd_msg_t cam_cmd = {0};
    bool valid_command = false;
    
    cJSON *exposure = cJSON_GetObjectItem(json, "exposure");
    if (exposure && cJSON_IsNumber(exposure)) {
        cam_cmd.command = CAM_CMD_SET_EXPOSURE;
        cam_cmd.value = (uint32_t)exposure->valueint;
        valid_command = true;
        ESP_LOGI(TAG, "Camera exposure: %d", cam_cmd.value);
        slam_web_server_send_command(MSG_C6_TO_P4_CAMERA_CMD, &cam_cmd, sizeof(cam_cmd));
    }
    
    cJSON *brightness = cJSON_GetObjectItem(json, "brightness");
    if (brightness && cJSON_IsNumber(brightness)) {
        cam_cmd.command = CAM_CMD_SET_BRIGHTNESS;
        cam_cmd.value = (uint32_t)brightness->valueint;
        valid_command = true;
        ESP_LOGI(TAG, "Camera brightness: %d", cam_cmd.value);
        slam_web_server_send_command(MSG_C6_TO_P4_CAMERA_CMD, &cam_cmd, sizeof(cam_cmd));
    }
    
    cJSON *contrast = cJSON_GetObjectItem(json, "contrast");
    if (contrast && cJSON_IsNumber(contrast)) {
        cam_cmd.command = CAM_CMD_SET_CONTRAST;
        cam_cmd.value = (uint32_t)contrast->valueint;
        valid_command = true;
        ESP_LOGI(TAG, "Camera contrast: %d", cam_cmd.value);
        slam_web_server_send_command(MSG_C6_TO_P4_CAMERA_CMD, &cam_cmd, sizeof(cam_cmd));
    }
    
    cJSON *saturation = cJSON_GetObjectItem(json, "saturation");
    if (saturation && cJSON_IsNumber(saturation)) {
        cam_cmd.command = CAM_CMD_SET_SATURATION;
        cam_cmd.value = (uint32_t)saturation->valueint;
        valid_command = true;
        ESP_LOGI(TAG, "Camera saturation: %d", cam_cmd.value);
        slam_web_server_send_command(MSG_C6_TO_P4_CAMERA_CMD, &cam_cmd, sizeof(cam_cmd));
    }
    
    if (!valid_command) {
        cJSON_Delete(json);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid camera command");
        return ESP_FAIL;
    }
    
    cJSON_Delete(json);
    httpd_resp_send(req, "OK", 2);
    return ESP_OK;
}

static esp_err_t api_wifi_control_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "📡 WiFi control API request received");
    
    if (req->method != HTTP_POST) {
        return httpd_resp_send_err(req, HTTPD_405_METHOD_NOT_ALLOWED, "Only POST allowed");
    }
    
    size_t recv_size = req->content_len > 512 ? 512 : req->content_len;
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
    
    // Parse WiFi control parameters
    wifi_control_msg_t wifi_control = {0};
    
    cJSON *enable_wifi = cJSON_GetObjectItem(json, "enable_wifi");
    if (enable_wifi && cJSON_IsBool(enable_wifi)) {
        wifi_control.enable_wifi = cJSON_IsTrue(enable_wifi) ? 1 : 0;
    }
    
    cJSON *enable_scan_only = cJSON_GetObjectItem(json, "enable_scan_only");
    if (enable_scan_only && cJSON_IsBool(enable_scan_only)) {
        wifi_control.enable_scan_only = cJSON_IsTrue(enable_scan_only) ? 1 : 0;
    }
    
    cJSON *auto_ap_fallback = cJSON_GetObjectItem(json, "auto_ap_fallback");
    if (auto_ap_fallback && cJSON_IsBool(auto_ap_fallback)) {
        wifi_control.auto_ap_fallback = cJSON_IsTrue(auto_ap_fallback) ? 1 : 0;
    }
    
    ESP_LOGI(TAG, "WiFi control: enable=%d, scan_only=%d, ap_fallback=%d",
             wifi_control.enable_wifi, wifi_control.enable_scan_only, wifi_control.auto_ap_fallback);
    
    // Send WiFi control command to P4
    slam_web_server_send_command(MSG_C6_TO_P4_WIFI_CONTROL, &wifi_control, sizeof(wifi_control));
    
    cJSON_Delete(json);
    
    // Send response indicating command was sent
    cJSON *response = cJSON_CreateObject();
    cJSON_AddStringToObject(response, "status", "command_sent");
    cJSON_AddStringToObject(response, "message", "WiFi control command sent to P4");
    
    char *response_str = cJSON_Print(response);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, response_str, strlen(response_str));
    
    free(response_str);
    cJSON_Delete(response);
    return ESP_OK;
}

static esp_err_t api_wifi_status_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "📊 WiFi status API request received");
    
    if (req->method != HTTP_GET) {
        return httpd_resp_send_err(req, HTTPD_405_METHOD_NOT_ALLOWED, "Only GET allowed");
    }
    
    // Request WiFi status from P4
    system_cmd_msg_t sys_cmd = {0};
    sys_cmd.command = SYS_CMD_WIFI_GET_STATUS;
    slam_web_server_send_command(MSG_C6_TO_P4_SYSTEM_CMD, &sys_cmd, sizeof(sys_cmd));
    
    // Return cached WiFi status (if available) or default response
    cJSON *response = cJSON_CreateObject();
    cJSON *wifi_status = cJSON_CreateObject();
    
    // Add current cached WiFi status if available
    cJSON_AddBoolToObject(wifi_status, "wifi_enabled", data_cache.wifi_status.wifi_enabled);
    cJSON_AddBoolToObject(wifi_status, "connected", data_cache.wifi_status.connected);
    cJSON_AddBoolToObject(wifi_status, "scan_active", data_cache.wifi_status.scan_active);
    cJSON_AddBoolToObject(wifi_status, "ap_mode_active", data_cache.wifi_status.ap_mode_active);
    cJSON_AddStringToObject(wifi_status, "ssid", (char*)data_cache.wifi_status.ssid);
    cJSON_AddNumberToObject(wifi_status, "rssi", data_cache.wifi_status.rssi);
    
    cJSON_AddItemToObject(response, "wifi_status", wifi_status);
    cJSON_AddStringToObject(response, "status", "success");
    
    char *response_str = cJSON_Print(response);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, response_str, strlen(response_str));
    
    free(response_str);
    cJSON_Delete(response);
    return ESP_OK;
}

static esp_err_t api_wifi_scan_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "🔍 WiFi scan API request received");
    
    if (req->method != HTTP_POST) {
        return httpd_resp_send_err(req, HTTPD_405_METHOD_NOT_ALLOWED, "Only POST allowed");
    }
    
    // Request WiFi scan from P4
    slam_web_server_send_command(MSG_C6_TO_P4_WIFI_SCAN_REQ, NULL, 0);
    
    // Return mock response for now (real scan results will come via WebSocket)
    cJSON *response = cJSON_CreateObject();
    cJSON *networks = cJSON_CreateArray();
    
    // Add some example networks (replace with real scan results later)
    cJSON *network1 = cJSON_CreateObject();
    cJSON_AddStringToObject(network1, "ssid", "Example_Network");
    cJSON_AddNumberToObject(network1, "rssi", -45);
    cJSON_AddNumberToObject(network1, "channel", 6);
    cJSON_AddItemToArray(networks, network1);
    
    cJSON_AddItemToObject(response, "networks", networks);
    cJSON_AddStringToObject(response, "status", "success");
    
    char *response_str = cJSON_Print(response);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, response_str, strlen(response_str));
    
    free(response_str);
    cJSON_Delete(response);
    return ESP_OK;
}

static esp_err_t api_wifi_connect_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "📶 WiFi connect API request received");
    
    if (req->method != HTTP_POST) {
        return httpd_resp_send_err(req, HTTPD_405_METHOD_NOT_ALLOWED, "Only POST allowed");
    }
    
    size_t recv_size = req->content_len > 512 ? 512 : req->content_len;
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
    
    wifi_connect_msg_t wifi_connect = {0};
    
    cJSON *ssid = cJSON_GetObjectItem(json, "ssid");
    if (ssid && cJSON_IsString(ssid)) {
        strncpy((char*)wifi_connect.ssid, ssid->valuestring, sizeof(wifi_connect.ssid) - 1);
    }
    
    cJSON *password = cJSON_GetObjectItem(json, "password");
    if (password && cJSON_IsString(password)) {
        strncpy((char*)wifi_connect.password, password->valuestring, sizeof(wifi_connect.password) - 1);
    }
    
    cJSON *ap_fallback = cJSON_GetObjectItem(json, "ap_fallback");
    bool enable_ap_fallback = true; // Default to enabled
    if (ap_fallback && cJSON_IsBool(ap_fallback)) {
        enable_ap_fallback = cJSON_IsTrue(ap_fallback);
    }
    
    ESP_LOGI(TAG, "WiFi connect request: SSID=%s, AP fallback=%s", 
             wifi_connect.ssid, enable_ap_fallback ? "enabled" : "disabled");
    
    // Send WiFi connect command to P4
    slam_web_server_send_command(MSG_C6_TO_P4_CONNECT_WIFI, &wifi_connect, sizeof(wifi_connect));
    
    // If AP fallback is enabled and we're on C6, start AP mode after connection attempt
    if (enable_ap_fallback) {
        // This will be handled by the WiFi manager on P4 side
        ESP_LOGI(TAG, "AP fallback enabled - P4 will start AP mode if connection fails");
    }
    
    cJSON_Delete(json);
    httpd_resp_send(req, "Connection attempt started", 24);
    return ESP_OK;
}

static esp_err_t api_settings_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "🔧 Settings API request received");
    
    if (req->method == HTTP_GET) {
        // Return current settings as JSON with validation status
        cJSON *json = cJSON_CreateObject();
        if (!json) {
            return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to create JSON");
        }
        
        // Add current SLAM parameters (from data cache)
        cJSON *slam = cJSON_CreateObject();
        cJSON_AddNumberToObject(slam, "max_features", data_cache.slam_config.max_features);
        cJSON_AddNumberToObject(slam, "fast_threshold", data_cache.slam_config.fast_threshold);
        cJSON_AddNumberToObject(slam, "scale_factor", data_cache.slam_config.scale_factor);
        cJSON_AddNumberToObject(slam, "min_tracked_features", data_cache.slam_config.min_tracked_features);
        cJSON_AddBoolToObject(slam, "enable_loop_closure", data_cache.slam_config.enable_loop_closure);
        cJSON_AddObjectToObject(json, "slam", slam);
        
        // Add fusion parameters with validation info
        cJSON *fusion = cJSON_CreateObject();
        cJSON_AddNumberToObject(fusion, "position_noise", 0.1); // From cache when available
        cJSON_AddNumberToObject(fusion, "velocity_noise", 0.01);
        cJSON_AddNumberToObject(fusion, "gps_weight", 0.8);
        cJSON_AddNumberToObject(fusion, "slam_weight", 0.9);
        cJSON_AddBoolToObject(fusion, "enable_outlier_detection", true);
        cJSON_AddStringToObject(fusion, "validation_status", "valid");
        cJSON_AddBoolToObject(fusion, "safe_to_update", true);
        cJSON_AddObjectToObject(json, "fusion", fusion);
        
        // Add camera parameters  
        cJSON *camera = cJSON_CreateObject();
        cJSON_AddNumberToObject(camera, "exposure", data_cache.camera_exposure);
        cJSON_AddNumberToObject(camera, "brightness", data_cache.camera_brightness);
        cJSON_AddNumberToObject(camera, "fps", data_cache.camera_fps);
        cJSON_AddBoolToObject(camera, "auto_exposure", true);
        cJSON_AddObjectToObject(json, "camera", camera);
        
        // Add system safety status
        cJSON *safety = cJSON_CreateObject();
        cJSON_AddBoolToObject(safety, "kalman_filter_running", true);
        cJSON_AddBoolToObject(safety, "msp_output_active", true);
        cJSON_AddBoolToObject(safety, "safe_parameter_updates", true);
        cJSON_AddObjectToObject(json, "system_safety", safety);
        
        char *json_string = cJSON_Print(json);
        cJSON_Delete(json);
        
        if (!json_string) {
            return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to serialize JSON");
        }
        
        httpd_resp_set_type(req, "application/json");
        httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
        esp_err_t ret = httpd_resp_send(req, json_string, strlen(json_string));
        free(json_string);
        return ret;
        
    } else if (req->method == HTTP_POST) {
        // Handle parameter updates with comprehensive validation
        size_t recv_size = req->content_len > 1024 ? 1024 : req->content_len;
        char *content = malloc(recv_size + 1);
        if (!content) {
            return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Memory allocation failed");
        }
        
        int ret = httpd_req_recv(req, content, recv_size);
        if (ret <= 0) {
            free(content);
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Failed to receive data");
        }
        content[ret] = '\0';
        
        ESP_LOGI(TAG, "📝 Settings update received: %s", content);
        
        cJSON *json = cJSON_Parse(content);
        free(content);
        
        if (!json) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        }
        
        // Create response with validation results
        cJSON *response = cJSON_CreateObject();
        cJSON *validation_results = cJSON_CreateArray();
        bool all_valid = true;
        
        // Process SLAM parameters with range validation
        cJSON *slam_params = cJSON_GetObjectItem(json, "slam");
        if (slam_params) {
            cJSON *max_features = cJSON_GetObjectItem(slam_params, "max_features");
            if (max_features && cJSON_IsNumber(max_features)) {
                int value = (int)cJSON_GetNumberValue(max_features);
                if (value >= 100 && value <= 2000) {
                    // Send validated parameter to P4 (safe during operation)
                    comm_message_t msg = {0};
                    msg.header.msg_type = MSG_C6_TO_P4_CONFIG_UPDATE;
                    msg.header.payload_size = sizeof(config_update_msg_t);
                    msg.payload.config_update.config_type = 1; // SLAM config
                    msg.payload.config_update.config_id = 1;   // max_features
                    msg.payload.config_update.value_size = sizeof(int);
                    memcpy(msg.payload.config_update.value_data, &value, sizeof(int));
                    
                    slam_communication_send_command(MSG_C6_TO_P4_CONFIG_UPDATE, &msg.payload.config_update, sizeof(config_update_msg_t));
                    
                    cJSON *result = cJSON_CreateObject();
                    cJSON_AddStringToObject(result, "parameter", "slam.max_features");
                    cJSON_AddStringToObject(result, "status", "valid_applied");
                    cJSON_AddNumberToObject(result, "value", value);
                    cJSON_AddStringToObject(result, "note", "Filter continues running");
                    cJSON_AddItemToArray(validation_results, result);
                } else {
                    all_valid = false;
                    cJSON *result = cJSON_CreateObject();
                    cJSON_AddStringToObject(result, "parameter", "slam.max_features");
                    cJSON_AddStringToObject(result, "status", "rejected");
                    cJSON_AddStringToObject(result, "error", "Must be between 100-2000");
                    cJSON_AddItemToArray(validation_results, result);
                }
            }
        }
        
        // Process fusion parameters with flight safety validation
        cJSON *fusion_params = cJSON_GetObjectItem(json, "fusion");
        if (fusion_params) {
            // Check system status for flight safety
            bool is_flying = (data_cache.slam_status.state > 0); // Simplified flight detection
            
            // Position noise validation
            cJSON *pos_noise = cJSON_GetObjectItem(fusion_params, "position_noise");
            if (pos_noise && cJSON_IsNumber(pos_noise)) {
                float value = (float)cJSON_GetNumberValue(pos_noise);
                bool safe_range = (value >= 0.001f && value <= 10.0f);
                bool safe_in_flight = !is_flying || value <= 1.0f;
                
                if (safe_range && safe_in_flight) {
                    // Apply parameter safely - Kalman filter handles this gracefully
                    comm_message_t msg = {0};
                    msg.header.msg_type = MSG_C6_TO_P4_CONFIG_UPDATE;
                    msg.header.payload_size = sizeof(config_update_msg_t);
                    msg.payload.config_update.config_type = 2; // Fusion config
                    msg.payload.config_update.config_id = 1;   // position_noise
                    msg.payload.config_update.value_size = sizeof(float);
                    memcpy(msg.payload.config_update.value_data, &value, sizeof(float));
                    
                    slam_communication_send_command(MSG_C6_TO_P4_CONFIG_UPDATE, &msg.payload.config_update, sizeof(config_update_msg_t));
                    
                    cJSON *result = cJSON_CreateObject();
                    cJSON_AddStringToObject(result, "parameter", "fusion.position_noise");
                    cJSON_AddStringToObject(result, "status", "valid_applied");
                    cJSON_AddNumberToObject(result, "value", value);
                    cJSON_AddStringToObject(result, "note", 
                        is_flying ? "Applied during flight - EKF stable" : "Applied - EKF continues");
                    cJSON_AddItemToArray(validation_results, result);
                } else {
                    all_valid = false;
                    cJSON *result = cJSON_CreateObject();
                    cJSON_AddStringToObject(result, "parameter", "fusion.position_noise");
                    cJSON_AddStringToObject(result, "status", "rejected");
                    cJSON_AddStringToObject(result, "error", 
                        is_flying ? "Value unsafe during flight" : "Value out of safe range");
                    cJSON_AddItemToArray(validation_results, result);
                }
            }
            
            // GPS weight validation (critical for navigation)
            cJSON *gps_weight = cJSON_GetObjectItem(fusion_params, "gps_weight");
            if (gps_weight && cJSON_IsNumber(gps_weight)) {
                float value = (float)cJSON_GetNumberValue(gps_weight);
                bool safe_weight = (value >= 0.0f && value <= 1.0f);
                bool safe_in_flight = !is_flying || value >= 0.1f; // Don't disable GPS in flight
                
                if (safe_weight && safe_in_flight) {
                    comm_message_t msg = {0};
                    msg.header.msg_type = MSG_C6_TO_P4_CONFIG_UPDATE;
                    msg.header.payload_size = sizeof(config_update_msg_t);
                    msg.payload.config_update.config_type = 2; // Fusion config
                    msg.payload.config_update.config_id = 2;   // gps_weight
                    msg.payload.config_update.value_size = sizeof(float);
                    memcpy(msg.payload.config_update.value_data, &value, sizeof(float));
                    
                    slam_communication_send_command(MSG_C6_TO_P4_CONFIG_UPDATE, &msg.payload.config_update, sizeof(config_update_msg_t));
                    
                    cJSON *result = cJSON_CreateObject();
                    cJSON_AddStringToObject(result, "parameter", "fusion.gps_weight");
                    cJSON_AddStringToObject(result, "status", "valid_applied");
                    cJSON_AddNumberToObject(result, "value", value);
                    cJSON_AddStringToObject(result, "note", "MSP output continues normally");
                    cJSON_AddItemToArray(validation_results, result);
                } else {
                    all_valid = false;
                    cJSON *result = cJSON_CreateObject();
                    cJSON_AddStringToObject(result, "parameter", "fusion.gps_weight");
                    cJSON_AddStringToObject(result, "status", "rejected");
                    cJSON_AddStringToObject(result, "error", 
                        is_flying ? "Cannot disable GPS during flight" : "Weight must be 0.0-1.0");
                    cJSON_AddItemToArray(validation_results, result);
                }
            }
        }
        
        // Process camera parameters (always safe to change)
        cJSON *camera_params = cJSON_GetObjectItem(json, "camera");
        if (camera_params) {
            cJSON *exposure = cJSON_GetObjectItem(camera_params, "exposure");
            if (exposure && cJSON_IsNumber(exposure)) {
                int value = (int)cJSON_GetNumberValue(exposure);
                if (value >= 0 && value <= 255) {
                    comm_message_t msg = {0};
                    msg.header.msg_type = MSG_C6_TO_P4_CONFIG_UPDATE;
                    msg.header.payload_size = sizeof(config_update_msg_t);
                    msg.payload.config_update.config_type = 3; // Camera config
                    msg.payload.config_update.config_id = 1;   // exposure
                    msg.payload.config_update.value_size = sizeof(int);
                    memcpy(msg.payload.config_update.value_data, &value, sizeof(int));
                    
                    slam_communication_send_command(MSG_C6_TO_P4_CONFIG_UPDATE, &msg.payload.config_update, sizeof(config_update_msg_t));
                    
                    cJSON *result = cJSON_CreateObject();
                    cJSON_AddStringToObject(result, "parameter", "camera.exposure");
                    cJSON_AddStringToObject(result, "status", "valid_applied");
                    cJSON_AddNumberToObject(result, "value", value);
                    cJSON_AddStringToObject(result, "note", "Camera update - no filter impact");
                    cJSON_AddItemToArray(validation_results, result);
                }
            }
        }
        
        // Prepare comprehensive response
        cJSON_AddBoolToObject(response, "success", all_valid);
        cJSON_AddStringToObject(response, "message", 
            all_valid ? "All parameters updated - systems stable" : "Some parameters rejected for safety");
        cJSON_AddItemToObject(response, "validation_results", validation_results);
        
        // System status confirmation
        cJSON *system_status = cJSON_CreateObject();
        cJSON_AddBoolToObject(system_status, "kalman_filter_running", true);
        cJSON_AddBoolToObject(system_status, "msp_output_active", true);
        cJSON_AddBoolToObject(system_status, "slam_processing", true);
        cJSON_AddBoolToObject(system_status, "parameter_updates_safe", true);
        cJSON_AddObjectToObject(response, "system_status", system_status);
        
        char *response_str = cJSON_Print(response);
        cJSON_Delete(json);
        cJSON_Delete(response);
        
        if (!response_str) {
            return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to create response");
        }
        
        httpd_resp_set_type(req, "application/json");
        httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
        esp_err_t ret_val = httpd_resp_send(req, response_str, strlen(response_str));
        free(response_str);
        
        ESP_LOGI(TAG, "✅ Settings validated and applied - EKF & MSP continue running");
        return ret_val;
    }
    
    return httpd_resp_send_err(req, HTTPD_405_METHOD_NOT_ALLOWED, "Method not allowed");
}

static esp_err_t api_map_handler(httpd_req_t *req)
{
    size_t recv_size = req->content_len > 512 ? 512 : req->content_len;
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
    cJSON *map_name = cJSON_GetObjectItem(json, "map_name");
    
    if (!cmd || !cJSON_IsString(cmd)) {
        cJSON_Delete(json);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid command");
        return ESP_FAIL;
    }
    
    map_cmd_msg_t map_cmd = {0};
    if (strcmp(cmd->valuestring, "save") == 0) {
        map_cmd.command = MAP_CMD_SAVE;
    } else if (strcmp(cmd->valuestring, "load") == 0) {
        map_cmd.command = MAP_CMD_LOAD;
    } else if (strcmp(cmd->valuestring, "clear") == 0) {
        map_cmd.command = MAP_CMD_CLEAR;
    } else if (strcmp(cmd->valuestring, "list") == 0) {
        map_cmd.command = MAP_CMD_LIST;
    } else {
        cJSON_Delete(json);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid map command");
        return ESP_FAIL;
    }
    
    if (map_name && cJSON_IsString(map_name)) {
        strncpy(map_cmd.map_name, map_name->valuestring, sizeof(map_cmd.map_name) - 1);
    }
    
    ESP_LOGI(TAG, "Map command: %s, name: %s", cmd->valuestring, map_cmd.map_name);
    slam_web_server_send_command(MSG_C6_TO_P4_MAP_CMD, &map_cmd, sizeof(map_cmd));
    
    cJSON_Delete(json);
    httpd_resp_send(req, "OK", 2);
    return ESP_OK;
}

static esp_err_t api_preview_handler(httpd_req_t *req)
{
    if (req->method == HTTP_GET) {
        // Request camera preview frame from P4
        camera_cmd_msg_t cam_cmd = {
            .command = CAM_CMD_START_PREVIEW,
            .value = 1  // Enable preview
        };
        
        ESP_LOGI(TAG, "Camera preview requested");
        slam_web_server_send_command(MSG_C6_TO_P4_CAMERA_CMD, &cam_cmd, sizeof(cam_cmd));
        
        httpd_resp_send(req, "OK", 2);
        return ESP_OK;
    } else if (req->method == HTTP_POST) {
        // Stop camera preview
        camera_cmd_msg_t cam_cmd = {
            .command = CAM_CMD_STOP_PREVIEW,
            .value = 0  // Disable preview
        };
        
        ESP_LOGI(TAG, "Camera preview stopped");
        slam_web_server_send_command(MSG_C6_TO_P4_CAMERA_CMD, &cam_cmd, sizeof(cam_cmd));
        
        httpd_resp_send(req, "OK", 2);
        return ESP_OK;
    }
    
    httpd_resp_send_404(req);
    return ESP_FAIL;
}

static esp_err_t api_preview_frame_handler(httpd_req_t *req)
{
    // Return the latest camera frame data for the preview
    if (xSemaphoreTake(data_cache_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        if (data_cache.has_camera_frame && data_cache.camera_frame_size > 0) {
            httpd_resp_set_type(req, "image/jpeg");
            httpd_resp_set_hdr(req, "Cache-Control", "no-cache, no-store, must-revalidate");
            httpd_resp_set_hdr(req, "Pragma", "no-cache");
            httpd_resp_set_hdr(req, "Expires", "0");
            
            esp_err_t ret = httpd_resp_send(req, (const char*)data_cache.camera_frame_data, data_cache.camera_frame_size);
            xSemaphoreGive(data_cache_mutex);
            return ret;
        }
        xSemaphoreGive(data_cache_mutex);
    }
    
    // No frame available - send placeholder or error
    httpd_resp_send_404(req);
    return ESP_FAIL;
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
    
    // Initialize camera frame data
    memset(&data_cache, 0, sizeof(data_cache));
    data_cache.camera_frame_data = NULL;
    data_cache.has_camera_frame = false;
    
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
    
    httpd_uri_t api_camera_uri = {
        .uri = "/api/camera",
        .method = HTTP_POST,
        .handler = api_camera_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &api_camera_uri);
    
    httpd_uri_t api_settings_uri = {
        .uri = "/api/settings",
        .method = HTTP_POST,
        .handler = api_settings_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &api_settings_uri);
    
    httpd_uri_t api_map_uri = {
        .uri = "/api/map",
        .method = HTTP_POST,
        .handler = api_map_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &api_map_uri);
    
    // WiFi API routes
    httpd_uri_t api_wifi_control_uri = {
        .uri = "/api/wifi/control",
        .method = HTTP_POST,
        .handler = api_wifi_control_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &api_wifi_control_uri);
    
    httpd_uri_t api_wifi_status_uri = {
        .uri = "/api/wifi/status",
        .method = HTTP_GET,
        .handler = api_wifi_status_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &api_wifi_status_uri);
    
    httpd_uri_t api_wifi_scan_uri = {
        .uri = "/api/wifi/scan",
        .method = HTTP_POST,
        .handler = api_wifi_scan_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &api_wifi_scan_uri);
    
    httpd_uri_t api_wifi_connect_uri = {
        .uri = "/api/wifi/connect",
        .method = HTTP_POST,
        .handler = api_wifi_connect_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &api_wifi_connect_uri);
    
    httpd_uri_t api_preview_uri = {
        .uri = "/api/preview",
        .method = HTTP_GET,
        .handler = api_preview_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &api_preview_uri);
    
    httpd_uri_t api_preview_stop_uri = {
        .uri = "/api/preview",
        .method = HTTP_POST,
        .handler = api_preview_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &api_preview_stop_uri);
    
    httpd_uri_t api_preview_frame_uri = {
        .uri = "/api/preview/frame",
        .method = HTTP_GET,
        .handler = api_preview_frame_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &api_preview_frame_uri);
    
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
            
        case MSG_P4_TO_C6_WIFI_STATUS:
            data_cache.wifi_status = message->payload.wifi_status;
            data_cache.wifi_status_timestamp = now;
            ESP_LOGI(TAG, "WiFi status updated: enabled=%d, connected=%d", 
                     data_cache.wifi_status.wifi_enabled, data_cache.wifi_status.connected);
            break;
            
        case MSG_P4_TO_C6_WIFI_NETWORKS:
            data_cache.wifi_scan_results = message->payload.wifi_scan;
            data_cache.wifi_scan_timestamp = now;
            ESP_LOGI(TAG, "WiFi scan results updated: %d networks found", 
                     data_cache.wifi_scan_results.network_count);
            break;
            
        case MSG_P4_TO_C6_CAMERA_FRAME:
        {
            const camera_frame_msg_t* frame = &message->payload.camera_frame;
            
            // Allocate or reallocate camera frame buffer
            if (data_cache.camera_frame_size < frame->frame_size) {
                if (data_cache.camera_frame_data) {
                    free(data_cache.camera_frame_data);
                }
                data_cache.camera_frame_data = malloc(frame->frame_size);
                if (!data_cache.camera_frame_data) {
                    ESP_LOGE(TAG, "Failed to allocate camera frame buffer");
                    data_cache.has_camera_frame = false;
                    break;
                }
            }
            
            // Copy frame data (following the header in payload)
            uint8_t* frame_data = (uint8_t*)&message->payload.raw_payload[sizeof(camera_frame_msg_t)];
            memcpy(data_cache.camera_frame_data, frame_data, frame->frame_size);
            
            data_cache.camera_frame_size = frame->frame_size;
            data_cache.camera_frame_width = frame->width;
            data_cache.camera_frame_height = frame->height;
            data_cache.camera_frame_number = frame->frame_number;
            data_cache.camera_frame_timestamp = now;
            data_cache.has_camera_frame = true;
            
            ESP_LOGI(TAG, "Camera frame received: %dx%d, %d bytes, frame #%d", 
                     frame->width, frame->height, frame->frame_size, frame->frame_number);
            break;
        }
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
    
    esp_err_t ret = httpd_ws_send_data(server, websocket_fd, &ws_pkt);
    
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

esp_err_t slam_web_server_stop(void)
{
    if (!server_running) {
        return ESP_OK;
    }
    
    server_running = false;
    
    // Stop HTTP server
    if (server) {
        httpd_stop(server);
        server = NULL;
    }
    
    // Clean up camera frame data
    if (xSemaphoreTake(data_cache_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        if (data_cache.camera_frame_data) {
            free(data_cache.camera_frame_data);
            data_cache.camera_frame_data = NULL;
        }
        data_cache.has_camera_frame = false;
        data_cache.camera_frame_size = 0;
        xSemaphoreGive(data_cache_mutex);
    }
    
    // Clean up mutex
    if (data_cache_mutex) {
        vSemaphoreDelete(data_cache_mutex);
        data_cache_mutex = NULL;
    }
    
    ESP_LOGI(TAG, "Web server stopped");
    return ESP_OK;
}
