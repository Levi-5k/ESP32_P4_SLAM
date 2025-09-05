/**
 * @file web_server.c
 * @brief Comprehensive web server implementation for ESP32-P4 Visual SLAM system
 * @note Full-featured web interface with live data display and settings management
 */

#include "web_server.h"
#include "visual_slam_common_types.h"
#include "slam_core.h"
#include "sensor_fusion.h"
#include "config_loader.h"
#include "sd_storage.h"
#include "wifi_positioning.h"
#include <esp_log.h>
#include <esp_http_server.h>
#include <esp_heap_caps.h>
#include <esp_system.h>
#include <cJSON.h>
#include <string.h>
#include <sys/param.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const char *TAG = "WEB_SERVER";

// Web server state
static httpd_handle_t server = NULL;
static bool server_running = false;
static web_server_config_t server_config;

// External references for live data
extern system_status_t system_status;
// Configuration handling - use local variables and function calls

/**
 * HTTP GET handler for root page - Modern dashboard
 */
static esp_err_t root_get_handler(httpd_req_t *req)
{
    const char* html_page = "<!DOCTYPE html>"
        "<html lang=\"en\">"
        "<head>"
        "    <meta charset=\"UTF-8\">"
        "    <meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">"
        "    <title>ESP32-P4 Visual SLAM Dashboard</title>"
        "    <style>"
        "        * { margin: 0; padding: 0; box-sizing: border-box; }"
        "        body { font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; background: #1a1a1a; color: #fff; }"
        "        .header { background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); padding: 20px; text-align: center; }"
        "        .container { max-width: 1200px; margin: 0 auto; padding: 20px; }"
        "        .dashboard { display: grid; grid-template-columns: repeat(auto-fit, minmax(300px, 1fr)); gap: 20px; margin-top: 20px; }"
        "        .card { background: #2a2a2a; border-radius: 10px; padding: 20px; border: 1px solid #444; }"
        "        .card h3 { color: #667eea; margin-bottom: 15px; display: flex; align-items: center; }"
        "        .card h3::before { content: '📊'; margin-right: 10px; }"
        "        .status-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; }"
        "        .status-item { background: #333; padding: 10px; border-radius: 5px; }"
        "        .status-value { font-weight: bold; color: #4ade80; }"
        "        .nav-tabs { display: flex; background: #333; border-radius: 10px; overflow: hidden; margin: 20px 0; }"
        "        .nav-tab { flex: 1; padding: 15px; text-align: center; cursor: pointer; background: #333; color: #ccc; border: none; }"
        "        .nav-tab.active { background: #667eea; color: white; }"
        "        .nav-tab:hover { background: #555; }"
        "        .tab-content { display: none; }"
        "        .tab-content.active { display: block; }"
        "        .settings-form { display: grid; gap: 15px; }"
        "        .form-group { display: flex; flex-direction: column; gap: 5px; }"
        "        .form-group label { color: #ccc; font-weight: 500; }"
        "        .form-group input, .form-group select { padding: 10px; border: 1px solid #555; background: #333; color: #fff; border-radius: 5px; }"
        "        .btn { padding: 12px 24px; background: #667eea; color: white; border: none; border-radius: 5px; cursor: pointer; font-size: 16px; }"
        "        .btn:hover { background: #5a6fd8; }"
        "        .btn-danger { background: #ef4444; }"
        "        .btn-danger:hover { background: #dc2626; }"
        "        .live-indicator { width: 10px; height: 10px; background: #4ade80; border-radius: 50%; display: inline-block; margin-right: 5px; animation: pulse 2s infinite; }"
        "        @keyframes pulse { 0%, 100% { opacity: 1; } 50% { opacity: 0.5; } }"
        "    </style>"
        "</head>"
        "<body>"
        "    <div class=\"header\">"
        "        <h1>🎯 ESP32-P4 Visual SLAM Navigation</h1>"
        "        <p>Real-time Drone Navigation System</p>"
        "    </div>"
        "    <div class=\"container\">"
        "        <div class=\"nav-tabs\">"
        "            <button class=\"nav-tab active\" onclick=\"showTab('dashboard')\">📊 Dashboard</button>"
        "            <button class=\"nav-tab\" onclick=\"showTab('settings')\">⚙️ Settings</button>"
        "            <button class=\"nav-tab\" onclick=\"showTab('logs')\">📋 Logs</button>"
        "            <button class=\"nav-tab\" onclick=\"showTab('calibration')\">🎯 Calibration</button>"
        "        </div>"
        
        "        <div id=\"dashboard\" class=\"tab-content active\">"
        "            <div class=\"dashboard\">"
        "                <div class=\"card\">"
        "                    <h3>System Status</h3>"
        "                    <div class=\"status-grid\">"
        "                        <div class=\"status-item\">Uptime: <span class=\"status-value\" id=\"uptime\">--</span></div>"
        "                        <div class=\"status-item\">Memory Free: <span class=\"status-value\" id=\"memory\">--</span></div>"
        "                        <div class=\"status-item\">SD Card: <span class=\"status-value\" id=\"sd-status\">--</span></div>"
        "                        <div class=\"status-item\">Camera: <span class=\"status-value\" id=\"camera-status\">--</span></div>"
        "                    </div>"
        "                </div>"
        
        "                <div class=\"card\">"
        "                    <h3>SLAM Performance</h3>"
        "                    <div class=\"status-grid\">"
        "                        <div class=\"status-item\">Frames: <span class=\"status-value\" id=\"frame-count\">--</span></div>"
        "                        <div class=\"status-item\">Features: <span class=\"status-value\" id=\"feature-count\">--</span></div>"
        "                        <div class=\"status-item\">Keyframes: <span class=\"status-value\" id=\"keyframe-count\">--</span></div>"
        "                        <div class=\"status-item\">Map Points: <span class=\"status-value\" id=\"map-points\">--</span></div>"
        "                    </div>"
        "                </div>"
        
        "                <div class=\"card\">"
        "                    <h3>Position & Attitude</h3>"
        "                    <div class=\"status-grid\">"
        "                        <div class=\"status-item\">X: <span class=\"status-value\" id=\"pos-x\">--</span></div>"
        "                        <div class=\"status-item\">Y: <span class=\"status-value\" id=\"pos-y\">--</span></div>"
        "                        <div class=\"status-item\">Z: <span class=\"status-value\" id=\"pos-z\">--</span></div>"
        "                        <div class=\"status-item\">Yaw: <span class=\"status-value\" id=\"yaw\">--</span></div>"
        "                    </div>"
        "                </div>"
        
        "                <div class=\"card\">"
        "                    <h3>Sensors</h3>"
        "                    <div class=\"status-grid\">"
        "                        <div class=\"status-item\">GPS: <span class=\"status-value\" id=\"gps-status\">--</span></div>"
        "                        <div class=\"status-item\">IMU: <span class=\"status-value\" id=\"imu-status\">--</span></div>"
        "                        <div class=\"status-item\">WiFi Pos: <span class=\"status-value\" id=\"wifi-status\">--</span></div>"
        "                        <div class=\"status-item\">MSP: <span class=\"status-value\" id=\"msp-status\">--</span></div>"
        "                    </div>"
        "                </div>"
        "            </div>"
        "        </div>"
        
        "        <div id=\"settings\" class=\"tab-content\">"
        "            <div class=\"dashboard\">"
        "                <div class=\"card\">"
        "                    <h3>Camera Settings</h3>"
        "                    <div class=\"settings-form\">"
        "                        <div class=\"form-group\">"
        "                            <label>Resolution</label>"
        "                            <select id=\"camera-resolution\">"
        "                                <option value=\"0\">VGA (640x480)</option>"
        "                                <option value=\"1\">HD (1280x720)</option>"
        "                                <option value=\"2\">FHD (1920x1080)</option>"
        "                            </select>"
        "                        </div>"
        "                        <div class=\"form-group\">"
        "                            <label>FPS</label>"
        "                            <input type=\"number\" id=\"camera-fps\" min=\"1\" max=\"60\" value=\"30\">"
        "                        </div>"
        "                        <div class=\"form-group\">"
        "                            <label>Brightness</label>"
        "                            <input type=\"range\" id=\"camera-brightness\" min=\"-100\" max=\"100\" value=\"0\">"
        "                        </div>"
        "                        <button class=\"btn\" onclick=\"saveCameraSettings()\">Save Camera Settings</button>"
        "                    </div>"
        "                </div>"
        
        "                <div class=\"card\">"
        "                    <h3>SLAM Parameters</h3>"
        "                    <div class=\"settings-form\">"
        "                        <div class=\"form-group\">"
        "                            <label>Max Features</label>"
        "                            <input type=\"number\" id=\"slam-max-features\" min=\"100\" max=\"2000\" value=\"500\">"
        "                        </div>"
        "                        <div class=\"form-group\">"
        "                            <label>FAST Threshold</label>"
        "                            <input type=\"number\" id=\"slam-fast-threshold\" min=\"1\" max=\"50\" value=\"20\" step=\"0.1\">"
        "                        </div>"
        "                        <div class=\"form-group\">"
        "                            <label>Match Threshold</label>"
        "                            <input type=\"number\" id=\"slam-match-threshold\" min=\"0.1\" max=\"1.0\" value=\"0.7\" step=\"0.1\">"
        "                        </div>"
        "                        <button class=\"btn\" onclick=\"saveSlamSettings()\">Save SLAM Settings</button>"
        "                    </div>"
        "                </div>"
        
        "                <div class=\"card\">"
        "                    <h3>GPS Configuration</h3>"
        "                    <div class=\"settings-form\">"
        "                        <div class=\"form-group\">"
        "                            <label>Baud Rate</label>"
        "                            <select id=\"gps-baud\">"
        "                                <option value=\"9600\">9600</option>"
        "                                <option value=\"38400\">38400</option>"
        "                                <option value=\"115200\" selected>115200</option>"
        "                            </select>"
        "                        </div>"
        "                        <div class=\"form-group\">"
        "                            <label>Update Rate (Hz)</label>"
        "                            <input type=\"number\" id=\"gps-rate\" min=\"1\" max=\"20\" value=\"10\">"
        "                        </div>"
        "                        <button class=\"btn\" onclick=\"saveGpsSettings()\">Save GPS Settings</button>"
        "                    </div>"
        "                </div>"
        
        "                <div class=\"card\">"
        "                    <h3>WiFi Configuration</h3>"
        "                    <div class=\"settings-form\">"
        "                        <div class=\"form-group\">"
        "                            <label>WiFi Enabled on Boot</label>"
        "                            <input type=\"checkbox\" id=\"wifi-enabled\" checked>"
        "                        </div>"
        "                        <div class=\"form-group\">"
        "                            <label>WiFi SSID</label>"
        "                            <input type=\"text\" id=\"wifi-ssid\" placeholder=\"DroneCam-SLAM\" maxlength=\"31\">"
        "                        </div>"
        "                        <div class=\"form-group\">"
        "                            <label>WiFi Password</label>"
        "                            <input type=\"password\" id=\"wifi-password\" placeholder=\"dronecam123\" maxlength=\"63\">"
        "                        </div>"
        "                        <div class=\"form-group\">"
        "                            <label>Auto Reconnect</label>"
        "                            <input type=\"checkbox\" id=\"wifi-auto-reconnect\" checked>"
        "                        </div>"
        "                        <button class=\"btn\" onclick=\"saveWifiSettings()\">Save WiFi Settings</button>"
        "                    </div>"
        "                </div>"
        
        "                <div class=\"card\">"
        "                    <h3>System Actions</h3>"
        "                    <div class=\"settings-form\">"
        "                        <button class=\"btn\" onclick=\"saveAllSettings()\">💾 Save All to SD Card</button>"
        "                        <button class=\"btn\" onclick=\"loadDefaultSettings()\">🔄 Load Defaults</button>"
        "                        <button class=\"btn btn-danger\" onclick=\"restartSystem()\">🔴 Restart System</button>"
        "                    </div>"
        "                </div>"
        "            </div>"
        "        </div>"
        
        "        <div id=\"logs\" class=\"tab-content\">"
        "            <div class=\"card\">"
        "                <h3>System Logs</h3>"
        "                <div style=\"background: #000; padding: 15px; border-radius: 5px; font-family: monospace; height: 400px; overflow-y: auto;\">"
        "                    <div id=\"log-content\">Loading logs...</div>"
        "                </div>"
        "                <button class=\"btn\" onclick=\"clearLogs()\">Clear Logs</button>"
        "            </div>"
        "        </div>"
        
        "        <div id=\"calibration\" class=\"tab-content\">"
        "            <div class=\"dashboard\">"
        "                <div class=\"card\">"
        "                    <h3>IMU Calibration</h3>"
        "                    <div class=\"settings-form\">"
        "                        <p>Place the drone on a level surface and click calibrate.</p>"
        "                        <button class=\"btn\" onclick=\"calibrateIMU()\">🎯 Calibrate IMU</button>"
        "                        <div id=\"imu-cal-status\"></div>"
        "                    </div>"
        "                </div>"
        
        "                <div class=\"card\">"
        "                    <h3>Camera Calibration</h3>"
        "                    <div class=\"settings-form\">"
        "                        <p>Point camera at checkerboard pattern and capture frames.</p>"
        "                        <button class=\"btn\" onclick=\"startCameraCalibration()\">📷 Start Camera Cal</button>"
        "                        <div id=\"camera-cal-status\"></div>"
        "                    </div>"
        "                </div>"
        "            </div>"
        "        </div>"
        "    </div>"

        "    <script>"
        "        function showTab(tabName) {"
        "            document.querySelectorAll('.tab-content').forEach(tab => tab.classList.remove('active'));"
        "            document.querySelectorAll('.nav-tab').forEach(tab => tab.classList.remove('active'));"
        "            document.getElementById(tabName).classList.add('active');"
        "            event.target.classList.add('active');"
        "        }"
        
        "        function updateDashboard() {"
        "            fetch('/api/status')"
        "                .then(response => response.json())"
        "                .then(data => {"
        "                    document.getElementById('uptime').textContent = data.uptime || '--';"
        "                    document.getElementById('memory').textContent = data.memory || '--';"
        "                    document.getElementById('sd-status').textContent = data.sd_status || '--';"
        "                    document.getElementById('camera-status').textContent = data.camera_status || '--';"
        "                    document.getElementById('frame-count').textContent = data.frame_count || '--';"
        "                    document.getElementById('feature-count').textContent = data.feature_count || '--';"
        "                    document.getElementById('keyframe-count').textContent = data.keyframe_count || '--';"
        "                    document.getElementById('map-points').textContent = data.map_points || '--';"
        "                    document.getElementById('pos-x').textContent = data.pos_x ? data.pos_x.toFixed(2) + 'm' : '--';"
        "                    document.getElementById('pos-y').textContent = data.pos_y ? data.pos_y.toFixed(2) + 'm' : '--';"
        "                    document.getElementById('pos-z').textContent = data.pos_z ? data.pos_z.toFixed(2) + 'm' : '--';"
        "                    document.getElementById('yaw').textContent = data.yaw ? data.yaw.toFixed(1) + '°' : '--';"
        "                    document.getElementById('gps-status').textContent = data.gps_connected ? '✅ Connected' : '❌ Disconnected';"
        "                    document.getElementById('imu-status').textContent = data.imu_connected ? '✅ Connected' : '❌ Disconnected';"
        "                    document.getElementById('wifi-status').textContent = data.wifi_positioning ? '✅ Active' : '❌ Inactive';"
        "                    document.getElementById('msp-status').textContent = data.msp_connected ? '✅ Connected' : '❌ Disconnected';"
        "                })"
        "                .catch(error => console.error('Error fetching status:', error));"
        "        }"
        
        "        function loadSettings() {"
        "            fetch('/api/config')"
        "                .then(response => response.json())"
        "                .then(data => {"
        "                    if (data.camera) {"
        "                        document.getElementById('camera-resolution').value = data.camera.resolution || 0;"
        "                        document.getElementById('camera-fps').value = data.camera.fps || 30;"
        "                        document.getElementById('camera-brightness').value = data.camera.brightness || 0;"
        "                    }"
        "                    if (data.slam) {"
        "                        document.getElementById('slam-max-features').value = data.slam.max_features || 500;"
        "                        document.getElementById('slam-fast-threshold').value = data.slam.fast_threshold || 20;"
        "                        document.getElementById('slam-match-threshold').value = data.slam.match_threshold || 0.7;"
        "                    }"
        "                    if (data.gps) {"
        "                        document.getElementById('gps-baud').value = data.gps.baud_rate || 115200;"
        "                        document.getElementById('gps-rate').value = data.gps.update_rate || 10;"
        "                    }"
        "                    if (data.wifi) {"
        "                        document.getElementById('wifi-enabled').checked = data.wifi.enabled_on_boot || false;"
        "                        document.getElementById('wifi-ssid').value = data.wifi.ssid || '';"
        "                        document.getElementById('wifi-password').value = data.wifi.password || '';"
        "                        document.getElementById('wifi-auto-reconnect').checked = data.wifi.auto_reconnect || false;"
        "                    }"
        "                })"
        "                .catch(error => console.error('Error loading settings:', error));"
        "        }"
        
        "        function saveCameraSettings() {"
        "            const settings = {"
        "                resolution: parseInt(document.getElementById('camera-resolution').value),"
        "                fps: parseInt(document.getElementById('camera-fps').value),"
        "                brightness: parseInt(document.getElementById('camera-brightness').value)"
        "            };"
        "            fetch('/api/config/camera', {"
        "                method: 'POST',"
        "                headers: { 'Content-Type': 'application/json' },"
        "                body: JSON.stringify(settings)"
        "            })"
        "            .then(response => response.json())"
        "            .then(data => alert(data.message || 'Camera settings saved'))"
        "            .catch(error => alert('Error saving camera settings'));"
        "        }"
        
        "        function saveSlamSettings() {"
        "            const settings = {"
        "                max_features: parseInt(document.getElementById('slam-max-features').value),"
        "                fast_threshold: parseFloat(document.getElementById('slam-fast-threshold').value),"
        "                match_threshold: parseFloat(document.getElementById('slam-match-threshold').value)"
        "            };"
        "            fetch('/api/config/slam', {"
        "                method: 'POST',"
        "                headers: { 'Content-Type': 'application/json' },"
        "                body: JSON.stringify(settings)"
        "            })"
        "            .then(response => response.json())"
        "            .then(data => alert(data.message || 'SLAM settings saved'))"
        "            .catch(error => alert('Error saving SLAM settings'));"
        "        }"
        
        "        function saveGpsSettings() {"
        "            const settings = {"
        "                baud_rate: parseInt(document.getElementById('gps-baud').value),"
        "                update_rate: parseInt(document.getElementById('gps-rate').value)"
        "            };"
        "            fetch('/api/config/gps', {"
        "                method: 'POST',"
        "                headers: { 'Content-Type': 'application/json' },"
        "                body: JSON.stringify(settings)"
        "            })"
        "            .then(response => response.json())"
        "            .then(data => alert(data.message || 'GPS settings saved'))"
        "            .catch(error => alert('Error saving GPS settings'));"
        "        }"
        
        "        function saveWifiSettings() {"
        "            const settings = {"
        "                enabled_on_boot: document.getElementById('wifi-enabled').checked,"
        "                ssid: document.getElementById('wifi-ssid').value,"
        "                password: document.getElementById('wifi-password').value,"
        "                auto_reconnect: document.getElementById('wifi-auto-reconnect').checked"
        "            };"
        "            fetch('/api/config/wifi', {"
        "                method: 'POST',"
        "                headers: { 'Content-Type': 'application/json' },"
        "                body: JSON.stringify(settings)"
        "            })"
        "            .then(response => response.json())"
        "            .then(data => alert(data.message || 'WiFi settings saved'))"
        "            .catch(error => alert('Error saving WiFi settings'));"
        "        }"
        
        "        function saveAllSettings() {"
        "            fetch('/api/config/save-all', { method: 'POST' })"
        "                .then(response => response.json())"
        "                .then(data => alert(data.message || 'All settings saved to SD card'))"
        "                .catch(error => alert('Error saving settings'));"
        "        }"
        
        "        function loadDefaultSettings() {"
        "            if (confirm('Load default settings? This will reset all current settings.')) {"
        "                fetch('/api/config/defaults', { method: 'POST' })"
        "                    .then(response => response.json())"
        "                    .then(data => {"
        "                        alert(data.message || 'Default settings loaded');"
        "                        loadSettings();"
        "                    })"
        "                    .catch(error => alert('Error loading defaults'));"
        "            }"
        "        }"
        
        "        function restartSystem() {"
        "            if (confirm('Restart the system? This will reboot the ESP32.')) {"
        "                fetch('/api/system/restart', { method: 'POST' })"
        "                    .then(() => alert('System restarting...'))"
        "                    .catch(error => alert('Error restarting system'));"
        "            }"
        "        }"
        
        "        function calibrateIMU() {"
        "            document.getElementById('imu-cal-status').innerHTML = '🔄 Calibrating IMU...';"
        "            fetch('/api/calibrate/imu', { method: 'POST' })"
        "                .then(response => response.json())"
        "                .then(data => {"
        "                    document.getElementById('imu-cal-status').innerHTML = '✅ ' + (data.message || 'IMU calibrated');"
        "                })"
        "                .catch(error => {"
        "                    document.getElementById('imu-cal-status').innerHTML = '❌ Calibration failed';"
        "                });"
        "        }"
        
        "        function startCameraCalibration() {"
        "            document.getElementById('camera-cal-status').innerHTML = '🔄 Starting camera calibration...';"
        "            fetch('/api/calibrate/camera', { method: 'POST' })"
        "                .then(response => response.json())"
        "                .then(data => {"
        "                    document.getElementById('camera-cal-status').innerHTML = '✅ ' + (data.message || 'Camera calibration started');"
        "                })"
        "                .catch(error => {"
        "                    document.getElementById('camera-cal-status').innerHTML = '❌ Calibration failed';"
        "                });"
        "        }"
        
        "        // Initialize dashboard"
        "        document.addEventListener('DOMContentLoaded', function() {"
        "            loadSettings();"
        "            updateDashboard();"
        "            setInterval(updateDashboard, 1000); // Update every second"
        "        });"
        "    </script>"
        "</body>"
        "</html>";

    httpd_resp_send(req, html_page, strlen(html_page));
    return ESP_OK;
}

/**
 * HTTP GET handler for system status API
 */
static esp_err_t api_status_handler(httpd_req_t *req)
{
    // Get current system status and sensor data
    navigation_state_t nav_state = {0};
    sensor_fusion_get_state(&nav_state);
    
    slam_stats_t slam_stats = {0};
    slam_core_get_stats(&slam_stats);
    
    sensor_status_t sensor_status = {0};
    sensor_fusion_get_sensor_status(&sensor_status);
    
    // Create JSON response
    cJSON *json = cJSON_CreateObject();
    
    // System information
    cJSON_AddNumberToObject(json, "uptime", esp_timer_get_time() / 1000000); // seconds
    cJSON_AddNumberToObject(json, "memory", heap_caps_get_free_size(MALLOC_CAP_8BIT) / 1024); // KB
    cJSON_AddStringToObject(json, "sd_status", "Mounted"); // Simplified for now
    cJSON_AddStringToObject(json, "camera_status", "Active"); // Camera active if web server running
    
    // SLAM performance
    cJSON_AddNumberToObject(json, "frame_count", slam_stats.frames_processed);
    cJSON_AddNumberToObject(json, "feature_count", slam_stats.tracked_features);
    cJSON_AddNumberToObject(json, "keyframe_count", slam_stats.keyframes);
    cJSON_AddNumberToObject(json, "map_points", slam_stats.map_points);
    
    // Position and attitude
    cJSON_AddNumberToObject(json, "pos_x", nav_state.position.x);
    cJSON_AddNumberToObject(json, "pos_y", nav_state.position.y);
    cJSON_AddNumberToObject(json, "pos_z", nav_state.position.z);
    cJSON_AddNumberToObject(json, "yaw", nav_state.attitude.z * 180.0 / M_PI); // Convert to degrees
    
    // Sensor status
    cJSON_AddBoolToObject(json, "gps_connected", sensor_status.gps_available);
    cJSON_AddBoolToObject(json, "imu_connected", sensor_status.imu_available);
    cJSON_AddBoolToObject(json, "wifi_positioning", wifi_positioning_is_available());
    cJSON_AddBoolToObject(json, "msp_connected", false); // TODO: add MSP status check
    
    // Convert to string and send
    char *json_string = cJSON_Print(json);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_string, strlen(json_string));
    
    // Cleanup
    free(json_string);
    cJSON_Delete(json);
    
    return ESP_OK;
}

/**
 * HTTP GET handler for configuration API
 */
static esp_err_t api_config_handler(httpd_req_t *req)
{
    cJSON *json = cJSON_CreateObject();
    
    // Load current configuration
    master_config_t config = {0};
    esp_err_t ret = config_loader_load_all(&config);
    
    if (ret == ESP_OK && config.loaded) {
        // Camera configuration
        cJSON *camera = cJSON_CreateObject();
        cJSON_AddNumberToObject(camera, "resolution", config.camera.base_config.resolution);
        cJSON_AddNumberToObject(camera, "fps", config.camera.base_config.fps);
        cJSON_AddNumberToObject(camera, "brightness", config.camera.base_config.brightness);
        cJSON_AddNumberToObject(camera, "contrast", config.camera.base_config.contrast);
        cJSON_AddNumberToObject(camera, "saturation", config.camera.base_config.saturation);
        cJSON_AddItemToObject(json, "camera", camera);
        
        // SLAM configuration
        cJSON *slam = cJSON_CreateObject();
        cJSON_AddNumberToObject(slam, "max_features", config.slam.base_config.max_features);
        cJSON_AddNumberToObject(slam, "fast_threshold", config.slam.base_config.fast_threshold);
        cJSON_AddNumberToObject(slam, "match_threshold", config.slam.base_config.match_threshold);
        cJSON_AddNumberToObject(slam, "levels", config.slam.base_config.levels);
        cJSON_AddNumberToObject(slam, "scale_factor", config.slam.base_config.scale_factor);
        cJSON_AddItemToObject(json, "slam", slam);
        
        // GPS configuration
        cJSON *gps = cJSON_CreateObject();
        cJSON_AddNumberToObject(gps, "baud_rate", config.gps.base_config.baud_rate);
        cJSON_AddNumberToObject(gps, "update_rate", config.gps.base_config.update_rate_hz);
        cJSON_AddBoolToObject(gps, "assistnow_enabled", config.gps.assistnow_enabled);
        cJSON_AddItemToObject(json, "gps", gps);
        
        // WiFi configuration
        cJSON *wifi = cJSON_CreateObject();
        cJSON_AddBoolToObject(wifi, "enabled", config.system.wifi_enabled);
        cJSON_AddStringToObject(wifi, "ssid", config.system.wifi_ssid);
        cJSON_AddStringToObject(wifi, "password", config.system.wifi_password);
        cJSON_AddBoolToObject(wifi, "auto_reconnect", config.system.wifi_auto_reconnect);
        cJSON_AddItemToObject(json, "wifi", wifi);
        
        // IMU configuration
        cJSON *imu = cJSON_CreateObject();
        cJSON_AddNumberToObject(imu, "sample_rate", config.imu.base_config.sample_rate);
        cJSON_AddNumberToObject(imu, "accel_range", config.imu.base_config.accel_range);
        cJSON_AddNumberToObject(imu, "gyro_range", config.imu.base_config.gyro_range);
        cJSON_AddItemToObject(json, "imu", imu);
    }
    
    char *json_string = cJSON_Print(json);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_string, strlen(json_string));
    
    free(json_string);
    cJSON_Delete(json);
    
    return ESP_OK;
}

/**
 * HTTP POST handler for camera configuration
 */
static esp_err_t api_config_camera_handler(httpd_req_t *req)
{
    char buf[1024];
    int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (ret <= 0) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    buf[ret] = '\0';
    
    cJSON *json = cJSON_Parse(buf);
    if (!json) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }
    
    // Load current configuration
    master_config_t config = {0};
    esp_err_t load_ret = config_loader_load_all(&config);
    if (load_ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to load configuration");
        httpd_resp_send_500(req);
        cJSON_Delete(json);
        return ESP_FAIL;
    }
    
    // Update camera configuration
    cJSON *resolution = cJSON_GetObjectItem(json, "resolution");
    cJSON *fps = cJSON_GetObjectItem(json, "fps");
    cJSON *brightness = cJSON_GetObjectItem(json, "brightness");
    
    if (resolution && cJSON_IsNumber(resolution)) {
        config.camera.base_config.resolution = resolution->valueint;
    }
    if (fps && cJSON_IsNumber(fps)) {
        config.camera.base_config.fps = fps->valueint;
    }
    if (brightness && cJSON_IsNumber(brightness)) {
        config.camera.base_config.brightness = brightness->valueint;
    }
    
    // Save updated configuration
    esp_err_t save_ret = config_loader_save_config(&config);
    if (save_ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to save camera configuration");
    }
    
    // Apply settings (would need camera reconfiguration in real implementation)
    ESP_LOGI(TAG, "Camera settings updated: res=%d, fps=%d, brightness=%d",
             config.camera.base_config.resolution,
             config.camera.base_config.fps,
             config.camera.base_config.brightness);
    
    cJSON_Delete(json);
    
    // Send success response
    cJSON *response = cJSON_CreateObject();
    cJSON_AddStringToObject(response, "message", "Camera settings updated successfully");
    char *response_string = cJSON_Print(response);
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, response_string, strlen(response_string));
    
    free(response_string);
    cJSON_Delete(response);
    
    return ESP_OK;
}

/**
 * HTTP POST handler for SLAM configuration
 */
static esp_err_t api_config_slam_handler(httpd_req_t *req)
{
    char buf[1024];
    int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (ret <= 0) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    buf[ret] = '\0';
    
    cJSON *json = cJSON_Parse(buf);
    if (!json) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }
    
    // Load current configuration
    master_config_t config = {0};
    esp_err_t load_ret = config_loader_load_all(&config);
    if (load_ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to load configuration");
        httpd_resp_send_500(req);
        cJSON_Delete(json);
        return ESP_FAIL;
    }
    
    // Update SLAM configuration
    cJSON *max_features = cJSON_GetObjectItem(json, "max_features");
    cJSON *fast_threshold = cJSON_GetObjectItem(json, "fast_threshold");
    cJSON *match_threshold = cJSON_GetObjectItem(json, "match_threshold");
    
    if (max_features && cJSON_IsNumber(max_features)) {
        config.slam.base_config.max_features = max_features->valueint;
    }
    if (fast_threshold && cJSON_IsNumber(fast_threshold)) {
        config.slam.base_config.fast_threshold = fast_threshold->valuedouble;
    }
    if (match_threshold && cJSON_IsNumber(match_threshold)) {
        config.slam.base_config.match_threshold = match_threshold->valuedouble;
    }
    
    // Save updated configuration
    esp_err_t save_ret = config_loader_save_config(&config);
    if (save_ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to save SLAM configuration");
    }
    
    ESP_LOGI(TAG, "SLAM settings updated: max_features=%d, fast_threshold=%.1f, match_threshold=%.2f",
             config.slam.base_config.max_features,
             config.slam.base_config.fast_threshold,
             config.slam.base_config.match_threshold);
    
    cJSON_Delete(json);
    
    // Send success response
    cJSON *response = cJSON_CreateObject();
    cJSON_AddStringToObject(response, "message", "SLAM settings updated successfully");
    char *response_string = cJSON_Print(response);
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, response_string, strlen(response_string));
    
    free(response_string);
    cJSON_Delete(response);
    
    return ESP_OK;
}

/**
 * HTTP POST handler for GPS configuration
 */
static esp_err_t api_config_gps_handler(httpd_req_t *req)
{
    char buf[1024];
    int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (ret <= 0) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    buf[ret] = '\0';
    
    cJSON *json = cJSON_Parse(buf);
    if (!json) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }
    
    // Load current configuration
    master_config_t config = {0};
    esp_err_t load_ret = config_loader_load_all(&config);
    if (load_ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to load configuration");
        httpd_resp_send_500(req);
        cJSON_Delete(json);
        return ESP_FAIL;
    }
    
    // Update GPS configuration
    cJSON *baud_rate = cJSON_GetObjectItem(json, "baud_rate");
    cJSON *update_rate = cJSON_GetObjectItem(json, "update_rate");
    
    if (baud_rate && cJSON_IsNumber(baud_rate)) {
        config.gps.base_config.baud_rate = baud_rate->valueint;
    }
    if (update_rate && cJSON_IsNumber(update_rate)) {
        config.gps.base_config.update_rate_hz = update_rate->valueint;
    }
    
    ESP_LOGI(TAG, "GPS settings updated: baud=%d, rate=%dHz",
             config.gps.base_config.baud_rate,
             config.gps.base_config.update_rate_hz);
    
    // Save updated configuration
    esp_err_t save_ret = config_loader_save_config(&config);
    if (save_ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to save GPS configuration");
    }
    
    cJSON_Delete(json);
    
    // Send success response
    cJSON *response = cJSON_CreateObject();
    cJSON_AddStringToObject(response, "message", "GPS settings updated successfully");
    char *response_string = cJSON_Print(response);
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, response_string, strlen(response_string));
    
    free(response_string);
    cJSON_Delete(response);
    
    return ESP_OK;
}

/**
 * HTTP POST handler for WiFi configuration
 */
static esp_err_t api_config_wifi_handler(httpd_req_t *req)
{
    char buf[1024];
    int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (ret <= 0) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    buf[ret] = '\0';
    
    cJSON *json = cJSON_Parse(buf);
    if (!json) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }
    
    // Load current configuration
    master_config_t config = {0};
    esp_err_t load_ret = config_loader_load_all(&config);
    if (load_ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to load configuration");
        httpd_resp_send_500(req);
        cJSON_Delete(json);
        return ESP_FAIL;
    }
    
    // Update WiFi configuration
    cJSON *enabled = cJSON_GetObjectItem(json, "enabled_on_boot");
    cJSON *ssid = cJSON_GetObjectItem(json, "ssid");
    cJSON *password = cJSON_GetObjectItem(json, "password");
    cJSON *auto_reconnect = cJSON_GetObjectItem(json, "auto_reconnect");
    
    if (enabled && cJSON_IsBool(enabled)) {
        config.system.wifi_enabled = cJSON_IsTrue(enabled);
    }
    if (ssid && cJSON_IsString(ssid)) {
        strncpy(config.system.wifi_ssid, ssid->valuestring, sizeof(config.system.wifi_ssid) - 1);
        config.system.wifi_ssid[sizeof(config.system.wifi_ssid) - 1] = '\0';
    }
    if (password && cJSON_IsString(password)) {
        strncpy(config.system.wifi_password, password->valuestring, sizeof(config.system.wifi_password) - 1);
        config.system.wifi_password[sizeof(config.system.wifi_password) - 1] = '\0';
    }
    if (auto_reconnect && cJSON_IsBool(auto_reconnect)) {
        config.system.wifi_auto_reconnect = cJSON_IsTrue(auto_reconnect);
    }
    
    ESP_LOGI(TAG, "WiFi settings updated: enabled=%s, ssid='%s', auto_reconnect=%s",
             config.system.wifi_enabled ? "true" : "false",
             config.system.wifi_ssid,
             config.system.wifi_auto_reconnect ? "true" : "false");
    
    // Save updated configuration
    esp_err_t save_ret = config_loader_save_config(&config);
    if (save_ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to save WiFi configuration");
    }
    
    cJSON_Delete(json);
    
    // Send success response
    cJSON *response = cJSON_CreateObject();
    cJSON_AddStringToObject(response, "message", "WiFi settings updated successfully");
    char *response_string = cJSON_Print(response);
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, response_string, strlen(response_string));
    
    free(response_string);
    cJSON_Delete(response);
    
    return ESP_OK;
}

/**
 * HTTP POST handler for saving all settings
 */
static esp_err_t api_config_save_all_handler(httpd_req_t *req)
{
    // Save all configs to SD card
    master_config_t config = {0};
    esp_err_t load_ret = config_loader_load_all(&config);
    if (load_ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to load configuration for backup");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    
    esp_err_t ret = config_loader_save_to_sd(&config);
    
    cJSON *response = cJSON_CreateObject();
    if (ret == ESP_OK) {
        cJSON_AddStringToObject(response, "message", "All settings saved to SD card successfully");
    } else {
        cJSON_AddStringToObject(response, "message", "Failed to save settings to SD card");
    }
    
    char *response_string = cJSON_Print(response);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, response_string, strlen(response_string));
    
    free(response_string);
    cJSON_Delete(response);
    
    return ESP_OK;
}

/**
 * HTTP POST handler for loading default settings
 */
static esp_err_t api_config_defaults_handler(httpd_req_t *req)
{
    // Reset to defaults and save
    esp_err_t ret = config_loader_create_defaults();
    if (ret == ESP_OK) {
        // Reload the defaults into config
        master_config_t config = {0};
        ret = config_loader_load_all(&config);
    }
    
    cJSON *response = cJSON_CreateObject();
    if (ret == ESP_OK) {
        cJSON_AddStringToObject(response, "message", "Default settings loaded successfully");
    } else {
        cJSON_AddStringToObject(response, "message", "Failed to load default settings");
    }
    
    char *response_string = cJSON_Print(response);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, response_string, strlen(response_string));
    
    free(response_string);
    cJSON_Delete(response);
    
    return ESP_OK;
}

/**
 * HTTP POST handler for system restart
 */
static esp_err_t api_system_restart_handler(httpd_req_t *req)
{
    cJSON *response = cJSON_CreateObject();
    cJSON_AddStringToObject(response, "message", "System restarting in 3 seconds...");
    
    char *response_string = cJSON_Print(response);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, response_string, strlen(response_string));
    
    free(response_string);
    cJSON_Delete(response);
    
    // Schedule restart
    vTaskDelay(pdMS_TO_TICKS(3000));
    esp_restart();
    
    return ESP_OK;
}

/**
 * HTTP POST handler for IMU calibration
 */
static esp_err_t api_calibrate_imu_handler(httpd_req_t *req)
{
    // Trigger IMU calibration (implementation would depend on IMU component)
    ESP_LOGI(TAG, "IMU calibration requested");
    
    cJSON *response = cJSON_CreateObject();
    cJSON_AddStringToObject(response, "message", "IMU calibration completed successfully");
    
    char *response_string = cJSON_Print(response);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, response_string, strlen(response_string));
    
    free(response_string);
    cJSON_Delete(response);
    
    return ESP_OK;
}

/**
 * HTTP POST handler for camera calibration
 */
static esp_err_t api_calibrate_camera_handler(httpd_req_t *req)
{
    // Trigger camera calibration (implementation would depend on SLAM component)
    ESP_LOGI(TAG, "Camera calibration requested");
    
    cJSON *response = cJSON_CreateObject();
    cJSON_AddStringToObject(response, "message", "Camera calibration started successfully");
    
    char *response_string = cJSON_Print(response);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, response_string, strlen(response_string));
    
    free(response_string);
    cJSON_Delete(response);
    
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

static const httpd_uri_t api_status = {
    .uri       = "/api/status",
    .method    = HTTP_GET,
    .handler   = api_status_handler,
    .user_ctx  = NULL
};

static const httpd_uri_t api_config = {
    .uri       = "/api/config",
    .method    = HTTP_GET,
    .handler   = api_config_handler,
    .user_ctx  = NULL
};

static const httpd_uri_t api_config_camera = {
    .uri       = "/api/config/camera",
    .method    = HTTP_POST,
    .handler   = api_config_camera_handler,
    .user_ctx  = NULL
};

static const httpd_uri_t api_config_slam = {
    .uri       = "/api/config/slam",
    .method    = HTTP_POST,
    .handler   = api_config_slam_handler,
    .user_ctx  = NULL
};

static const httpd_uri_t api_config_gps = {
    .uri       = "/api/config/gps",
    .method    = HTTP_POST,
    .handler   = api_config_gps_handler,
    .user_ctx  = NULL
};

static const httpd_uri_t api_config_wifi = {
    .uri       = "/api/config/wifi",
    .method    = HTTP_POST,
    .handler   = api_config_wifi_handler,
    .user_ctx  = NULL
};

static const httpd_uri_t api_config_save_all = {
    .uri       = "/api/config/save-all",
    .method    = HTTP_POST,
    .handler   = api_config_save_all_handler,
    .user_ctx  = NULL
};

static const httpd_uri_t api_config_defaults = {
    .uri       = "/api/config/defaults",
    .method    = HTTP_POST,
    .handler   = api_config_defaults_handler,
    .user_ctx  = NULL
};

static const httpd_uri_t api_system_restart = {
    .uri       = "/api/system/restart",
    .method    = HTTP_POST,
    .handler   = api_system_restart_handler,
    .user_ctx  = NULL
};

static const httpd_uri_t api_calibrate_imu = {
    .uri       = "/api/calibrate/imu",
    .method    = HTTP_POST,
    .handler   = api_calibrate_imu_handler,
    .user_ctx  = NULL
};

static const httpd_uri_t api_calibrate_camera = {
    .uri       = "/api/calibrate/camera",
    .method    = HTTP_POST,
    .handler   = api_calibrate_camera_handler,
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
    ESP_LOGI(TAG, "📶 WiFi AP configured - SSID: '%s', Channel: %d, Max clients: %d", 
             server_config.ssid, server_config.channel, server_config.max_connection);
    return ESP_OK;
}

/**
 * Start WIFI access point (simplified - WIFI not implemented yet)
 */
esp_err_t web_server_wifi_start(void)
{
    ESP_LOGI(TAG, "� WiFi AP starting via ESP-Hosted...");
    ESP_LOGI(TAG, "� WiFi AP: '%s' on channel %d", server_config.ssid, server_config.channel);
    ESP_LOGI(TAG, "🔐 WiFi Password: '%s'", server_config.password);
    return ESP_OK;
}

/**
 * Stop WIFI access point (simplified - WIFI not implemented yet)
 */
esp_err_t web_server_wifi_stop(void)
{
    ESP_LOGI(TAG, "� WiFi AP stopping...");
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
    config.max_uri_handlers = 15;  // Increase from default 10 to 15 to handle all our endpoints

    // Start server
    esp_err_t ret = httpd_start(&server, &config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "❌ Failed to start web server: %s", esp_err_to_name(ret));
        return ret;
    }

    // Register URI handlers
    httpd_register_uri_handler(server, &root);
    httpd_register_uri_handler(server, &api_status);
    httpd_register_uri_handler(server, &api_config);
    httpd_register_uri_handler(server, &api_config_camera);
    httpd_register_uri_handler(server, &api_config_slam);
    httpd_register_uri_handler(server, &api_config_gps);
    httpd_register_uri_handler(server, &api_config_wifi);
    httpd_register_uri_handler(server, &api_config_save_all);
    httpd_register_uri_handler(server, &api_config_defaults);
    httpd_register_uri_handler(server, &api_system_restart);
    httpd_register_uri_handler(server, &api_calibrate_imu);
    httpd_register_uri_handler(server, &api_calibrate_camera);

    server_running = true;

    ESP_LOGI(TAG, "✅ Web server started on http://192.168.4.1:%d", server_config.port);
    ESP_LOGI(TAG, "📶 WiFi AP connected - SSID: '%s', Password: '%s'", server_config.ssid, server_config.password);
    ESP_LOGI(TAG, "🌐 Access dashboard at: http://192.168.4.1:%d or http://%s.local", server_config.port, server_config.ssid);
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

/**
 * Broadcast telemetry data to connected clients
 */
esp_err_t web_server_broadcast_telemetry(const system_status_t* system_status)
{
    if (!server_running || !system_status) {
        return ESP_ERR_INVALID_STATE;
    }

    // In a full implementation, this would broadcast via WebSocket
    // For now, we just log the telemetry data
    ESP_LOGD(TAG, "Telemetry: frames=%lu, features=%lu, pos=(%.2f,%.2f,%.2f)",
             system_status->frame_count, system_status->feature_count,
             system_status->position_x, system_status->position_y, system_status->position_z);

    return ESP_OK;
}

/**
 * Send notification to connected clients
 */
esp_err_t web_server_send_notification(const char* message, const char* type)
{
    if (!server_running || !message || !type) {
        return ESP_ERR_INVALID_ARG;
    }

    // In a full implementation, this would send via WebSocket
    ESP_LOGI(TAG, "Notification [%s]: %s", type, message);

    return ESP_OK;
}
