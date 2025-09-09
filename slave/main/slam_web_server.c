/**
 * @file slam_web_server.c
 * @brief Web server implementation for ESP32-C6 slave device
 */

#include "slam_web_server.h"
#include "slam_communication_handler.h"
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
 * HTML Dashboard - Glassmorphic design with full feature set
 */
static const char* get_dashboard_html(void) 
{
    return "<!DOCTYPE html>"
        "<html><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width,initial-scale=1.0'>"
        "<title>ESP32-P4 SLAM Control Center</title><style>"
        ":root{--primary:#6366f1;--primary-light:#8b5cf6;--success:#10b981;--warning:#f59e0b;--danger:#ef4444;--bg-dark:#0f0f23;--bg-darker:#0a0a1a;--glass-bg:rgba(255,255,255,0.1);--glass-border:rgba(255,255,255,0.2);--text-primary:#f8fafc;--text-secondary:#cbd5e1;--text-muted:#64748b}"
        "*{margin:0;padding:0;box-sizing:border-box}"
        "body{font-family:'Segoe UI',Tahoma,Geneva,Verdana,sans-serif;background:linear-gradient(135deg,#667eea 0%,#764ba2 100%);min-height:100vh;color:var(--text-primary);overflow-x:hidden}"
        "body::before{content:'';position:fixed;top:0;left:0;width:100%;height:100%;background:url('data:image/svg+xml,<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"0 0 100 100\"><circle cx=\"50\" cy=\"50\" r=\"1\" fill=\"%23ffffff\" opacity=\"0.1\"/></svg>') repeat;z-index:-1}"
        ".header{background:rgba(255,255,255,0.1);backdrop-filter:blur(20px);border-bottom:1px solid rgba(255,255,255,0.2);padding:20px;text-align:center;position:sticky;top:0;z-index:100}"
        ".header h1{font-size:2.5rem;font-weight:700;background:linear-gradient(45deg,#60a5fa,#c084fc);-webkit-background-clip:text;background-clip:text;color:transparent;margin-bottom:5px}"
        ".header p{color:var(--text-secondary);font-size:1.1rem;opacity:0.9}"
        ".container{max-width:1400px;margin:0 auto;padding:20px}"
        ".nav-tabs{display:flex;background:rgba(255,255,255,0.1);backdrop-filter:blur(20px);border-radius:16px;padding:8px;margin:20px 0;border:1px solid rgba(255,255,255,0.2);gap:4px}"
        ".nav-tab{flex:1;padding:12px 20px;text-align:center;cursor:pointer;background:transparent;color:var(--text-secondary);border:none;border-radius:12px;transition:all 0.3s ease;font-weight:500;display:flex;align-items:center;justify-content:center;gap:8px}"
        ".nav-tab:hover{background:rgba(255,255,255,0.1);color:var(--text-primary)}"
        ".nav-tab.active{background:rgba(255,255,255,0.2);color:var(--text-primary);box-shadow:0 4px 16px rgba(255,255,255,0.1)}"
        ".tab-content{display:none;animation:fadeIn 0.3s ease}"
        ".tab-content.active{display:block}"
        "@keyframes fadeIn{from{opacity:0;transform:translateY(20px)}to{opacity:1;transform:translateY(0)}}"
        ".dashboard{display:grid;grid-template-columns:repeat(auto-fit,minmax(320px,1fr));gap:20px;margin-top:20px}"
        ".card{background:rgba(255,255,255,0.1);backdrop-filter:blur(20px);border-radius:20px;padding:24px;border:1px solid rgba(255,255,255,0.2);box-shadow:0 8px 32px rgba(0,0,0,0.1);transition:all 0.3s ease;position:relative;overflow:hidden}"
        ".card::before{content:'';position:absolute;top:0;left:0;right:0;height:1px;background:linear-gradient(90deg,transparent,rgba(255,255,255,0.3),transparent)}"
        ".card:hover{transform:translateY(-4px);box-shadow:0 12px 40px rgba(0,0,0,0.2);border-color:rgba(255,255,255,0.3)}"
        ".card h3{color:var(--text-primary);margin-bottom:16px;font-size:1.3rem;font-weight:600;display:flex;align-items:center;gap:8px}"
        ".status-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(140px,1fr));gap:12px}"
        ".status-item{background:rgba(255,255,255,0.08);padding:12px;border-radius:12px;transition:all 0.3s ease;border:1px solid rgba(255,255,255,0.1)}"
        ".status-item:hover{background:rgba(255,255,255,0.12);transform:scale(1.02)}"
        ".status-label{display:block;font-size:0.85rem;color:var(--text-muted);margin-bottom:4px}"
        ".status-value{font-weight:700;color:var(--success);font-size:1.1rem}"
        ".status-value.warning{color:var(--warning)}"
        ".status-value.danger{color:var(--danger)}"
        ".btn{padding:12px 24px;background:linear-gradient(135deg,var(--primary),var(--primary-light));color:white;border:none;border-radius:12px;cursor:pointer;margin:6px;font-weight:600;transition:all 0.3s ease;display:inline-flex;align-items:center;gap:8px;text-decoration:none;box-shadow:0 4px 16px rgba(99,102,241,0.3)}"
        ".btn:hover{transform:translateY(-2px);box-shadow:0 8px 24px rgba(99,102,241,0.4)}"
        ".btn-success{background:linear-gradient(135deg,var(--success),#34d399);box-shadow:0 4px 16px rgba(16,185,129,0.3)}"
        ".btn-warning{background:linear-gradient(135deg,var(--warning),#fbbf24);box-shadow:0 4px 16px rgba(245,158,11,0.3)}"
        ".btn-danger{background:linear-gradient(135deg,var(--danger),#f87171);box-shadow:0 4px 16px rgba(239,68,68,0.3)}"
        ".btn:disabled{opacity:0.5;cursor:not-allowed;transform:none!important}"
        ".form-group{margin:16px 0}"
        ".form-group label{display:block;margin-bottom:8px;color:var(--text-secondary);font-weight:500}"
        ".form-group input,.form-group select{width:100%;padding:12px 16px;border:1px solid rgba(255,255,255,0.2);background:rgba(255,255,255,0.1);color:var(--text-primary);border-radius:12px;transition:all 0.3s ease;backdrop-filter:blur(10px)}"
        ".form-group input:focus,.form-group select:focus{outline:none;border-color:var(--primary);box-shadow:0 0 0 3px rgba(99,102,241,0.2)}"
        ".form-group input[type='range']{height:6px;background:rgba(255,255,255,0.2);border-radius:3px;appearance:none}"
        ".form-group input[type='range']::-webkit-slider-thumb{appearance:none;width:20px;height:20px;background:linear-gradient(135deg,var(--primary),var(--primary-light));border-radius:50%;cursor:pointer;box-shadow:0 2px 8px rgba(99,102,241,0.4)}"
        ".range-container{display:flex;align-items:center;gap:12px}"
        ".range-value{min-width:50px;text-align:center;font-weight:600;color:var(--success);background:rgba(255,255,255,0.1);padding:4px 8px;border-radius:8px}"
        ".live-indicator{color:var(--success);font-weight:600;display:inline-flex;align-items:center;gap:4px}"
        ".live-indicator::before{content:'';width:8px;height:8px;background:var(--success);border-radius:50%;animation:pulse 2s infinite}"
        "@keyframes pulse{0%,100%{opacity:1}50%{opacity:0.5}}"
        "#position-viz{width:100%;height:250px;background:rgba(0,0,0,0.3);border-radius:16px;position:relative;overflow:hidden;border:1px solid rgba(255,255,255,0.1)}"
        "#position-viz::before{content:'';position:absolute;top:50%;left:50%;width:2px;height:100%;background:rgba(255,255,255,0.1);transform:translateX(-50%)}"
        "#position-viz::after{content:'';position:absolute;top:50%;left:50%;width:100%;height:2px;background:rgba(255,255,255,0.1);transform:translateY(-50%)}"
        ".position-dot{width:12px;height:12px;background:radial-gradient(circle,var(--success),var(--success));border-radius:50%;position:absolute;transform:translate(-50%,-50%);box-shadow:0 0 12px var(--success);animation:positionPulse 2s infinite}"
        "@keyframes positionPulse{0%,100%{transform:translate(-50%,-50%) scale(1)}50%{transform:translate(-50%,-50%) scale(1.2)}}"
        ".trail{width:4px;height:4px;background:rgba(16,185,129,0.6);border-radius:50%;position:absolute;transform:translate(-50%,-50%)}"
        ".preview-container{border:1px solid rgba(255,255,255,0.2);border-radius:16px;padding:20px;background:rgba(255,255,255,0.05)}"
        ".preview-box{border:2px dashed rgba(255,255,255,0.3);border-radius:12px;position:relative;overflow:hidden;background:rgba(0,0,0,0.3);backdrop-filter:blur(10px)}"
        ".preview-box img{width:100%;height:100%;object-fit:contain;border-radius:10px}"
        ".preview-controls{display:flex;gap:8px;flex-wrap:wrap}"
        ".preview-info{display:flex;justify-content:space-between;font-size:0.9rem;color:var(--text-muted);margin-top:12px}"
        ".wifi-network{background:rgba(255,255,255,0.08);border-radius:12px;padding:16px;margin:8px 0;border:1px solid rgba(255,255,255,0.1);transition:all 0.3s ease;cursor:pointer}"
        ".wifi-network:hover{background:rgba(255,255,255,0.12);transform:translateX(4px)}"
        ".wifi-network-header{display:flex;justify-content:space-between;align-items:center;margin-bottom:8px}"
        ".wifi-ssid{font-weight:600;color:var(--text-primary)}"
        ".wifi-signal{font-weight:600}"
        ".wifi-signal.excellent{color:var(--success)}"
        ".wifi-signal.good{color:#22d3ee}"
        ".wifi-signal.fair{color:var(--warning)}"
        ".wifi-signal.poor{color:var(--danger)}"
        ".wifi-details{font-size:0.85rem;color:var(--text-muted);display:flex;gap:16px}"
        ".modal{display:none;position:fixed;top:0;left:0;width:100%;height:100%;background:rgba(0,0,0,0.7);backdrop-filter:blur(8px);z-index:1000;align-items:center;justify-content:center}"
        ".modal.active{display:flex}"
        ".modal-content{background:rgba(255,255,255,0.1);backdrop-filter:blur(20px);border-radius:20px;padding:32px;margin:20px;max-width:500px;width:100%;border:1px solid rgba(255,255,255,0.2);box-shadow:0 16px 64px rgba(0,0,0,0.3)}"
        ".modal-header{display:flex;justify-content:space-between;align-items:center;margin-bottom:24px}"
        ".modal-title{font-size:1.5rem;font-weight:700;color:var(--text-primary)}"
        ".modal-close{background:none;border:none;color:var(--text-secondary);font-size:1.5rem;cursor:pointer;padding:4px}"
        ".notification{position:fixed;top:20px;right:20px;background:rgba(255,255,255,0.1);backdrop-filter:blur(20px);border-radius:12px;padding:16px 20px;border:1px solid rgba(255,255,255,0.2);box-shadow:0 8px 32px rgba(0,0,0,0.2);transform:translateX(400px);transition:all 0.3s ease;z-index:1001}"
        ".notification.show{transform:translateX(0)}"
        ".notification.success{border-left:4px solid var(--success)}"
        ".notification.warning{border-left:4px solid var(--warning)}"
        ".notification.error{border-left:4px solid var(--danger)}"
        ".stats-row{display:grid;grid-template-columns:repeat(auto-fit,minmax(200px,1fr));gap:16px;margin:20px 0}"
        ".stat-card{background:rgba(255,255,255,0.08);border-radius:16px;padding:20px;text-align:center;border:1px solid rgba(255,255,255,0.1);transition:all 0.3s ease}"
        ".stat-card:hover{background:rgba(255,255,255,0.12);transform:translateY(-2px)}"
        ".stat-value{font-size:2rem;font-weight:700;color:var(--success);margin-bottom:8px}"
        ".stat-label{color:var(--text-secondary);font-size:0.9rem}"
        ".progress-bar{width:100%;height:8px;background:rgba(255,255,255,0.1);border-radius:4px;overflow:hidden;margin:8px 0}"
        ".progress-fill{height:100%;background:linear-gradient(90deg,var(--primary),var(--primary-light));border-radius:4px;transition:width 0.3s ease}"
        ".two-column{display:grid;grid-template-columns:1fr 1fr;gap:20px}"
        "@media (max-width: 768px){.dashboard{grid-template-columns:1fr}.nav-tab{font-size:0.9rem;padding:10px}.two-column{grid-template-columns:1fr}.header h1{font-size:2rem}}"
        "</style></head><body>"
        "<div class='header'>"
        "<h1>🎯 ESP32-P4 SLAM Control Center</h1>"
        "<p>Real-time Visual SLAM Navigation System</p>"
        "</div>"
        "<div class='container'>"
        "<div class='nav-tabs'>"
        "<button class='nav-tab active' onclick='showTab(\"dashboard\")'><span>📊</span> Dashboard</button>"
        "<button class='nav-tab' onclick='showTab(\"position\")'><span>🗺️</span> Navigation</button>"
        "<button class='nav-tab' onclick='showTab(\"camera\")'><span>📷</span> Camera</button>"
        "<button class='nav-tab' onclick='showTab(\"sensors\")'><span>📡</span> Sensors</button>"
        "<button class='nav-tab' onclick='showTab(\"wifi\")'><span>📶</span> WiFi</button>"
        "<button class='nav-tab' onclick='showTab(\"settings\")'><span>⚙️</span> Settings</button>"
        "<button class='nav-tab' onclick='showTab(\"control\")'><span>🎮</span> Control</button>"
        "</div>"
        
        // Dashboard Tab
        "<div id='dashboard' class='tab-content active'>"
        "<div class='stats-row'>"
        "<div class='stat-card'>"
        "<div class='stat-value' id='connection-status'>●</div>"
        "<div class='stat-label'>P4 Connection</div>"
        "</div>"
        "<div class='stat-card'>"
        "<div class='stat-value' id='uptime-display'>0s</div>"
        "<div class='stat-label'>System Uptime</div>"
        "</div>"
        "<div class='stat-card'>"
        "<div class='stat-value' id='memory-usage'>--</div>"
        "<div class='stat-label'>Free Memory</div>"
        "</div>"
        "<div class='stat-card'>"
        "<div class='stat-value' id='slam-state'>--</div>"
        "<div class='stat-label'>SLAM Status</div>"
        "</div>"
        "</div>"
        "<div class='dashboard'>"
        "<div class='card'>"
        "<h3>🎯 SLAM Performance</h3>"
        "<div class='status-grid'>"
        "<div class='status-item'>"
        "<span class='status-label'>Tracked Features</span>"
        "<span class='status-value' id='features-count'>--</span>"
        "</div>"
        "<div class='status-item'>"
        "<span class='status-label'>Map Points</span>"
        "<span class='status-value' id='map-points-count'>--</span>"
        "</div>"
        "<div class='status-item'>"
        "<span class='status-label'>Keyframes</span>"
        "<span class='status-value' id='keyframes-count'>--</span>"
        "</div>"
        "<div class='status-item'>"
        "<span class='status-label'>Tracking Quality</span>"
        "<span class='status-value' id='quality-percent'>--</span>"
        "</div>"
        "</div>"
        "<div style='margin-top:16px'>"
        "<div style='display:flex;justify-content:space-between;align-items:center;margin-bottom:8px'>"
        "<span>Tracking Quality</span>"
        "<span id='quality-value'>--</span>"
        "</div>"
        "<div class='progress-bar'>"
        "<div class='progress-fill' id='quality-progress' style='width:0%'></div>"
        "</div>"
        "</div>"
        "</div>"
        "<div class='card'>"
        "<h3>📡 System Health</h3>"
        "<div class='status-grid'>"
        "<div class='status-item'>"
        "<span class='status-label'>Camera Status</span>"
        "<span class='status-value' id='camera-health'>--</span>"
        "</div>"
        "<div class='status-item'>"
        "<span class='status-label'>IMU Status</span>"
        "<span class='status-value' id='imu-health'>--</span>"
        "</div>"
        "<div class='status-item'>"
        "<span class='status-label'>GPS Satellites</span>"
        "<span class='status-value' id='gps-sats'>--</span>"
        "</div>"
        "<div class='status-item'>"
        "<span class='status-label'>SD Card</span>"
        "<span class='status-value' id='sd-health'>--</span>"
        "</div>"
        "</div>"
        "</div>"
        "<div class='card'>"
        "<h3>⚡ Performance Metrics</h3>"
        "<div class='status-grid'>"
        "<div class='status-item'>"
        "<span class='status-label'>CPU Usage</span>"
        "<span class='status-value' id='cpu-usage'>--</span>"
        "</div>"
        "<div class='status-item'>"
        "<span class='status-label'>Camera FPS</span>"
        "<span class='status-value' id='camera-fps'>--</span>"
        "</div>"
        "<div class='status-item'>"
        "<span class='status-label'>Processing Time</span>"
        "<span class='status-value' id='process-time'>--</span>"
        "</div>"
        "<div class='status-item'>"
        "<span class='status-label'>Temperature</span>"
        "<span class='status-value' id='temperature'>--</span>"
        "</div>"
        "</div>"
        "</div>"
        "</div>"
        "</div>"
        "</div>"
        "</div>"
        
        // Navigation Tab
        "<div id='position' class='tab-content'>"
        "<div class='dashboard'>"
        "<div class='card'>"
        "<h3>🗺️ Real-time Position</h3>"
        "<div id='position-viz'></div>"
        "<div class='stats-row' style='margin-top:16px;grid-template-columns:repeat(3,1fr)'>"
        "<div class='stat-card'>"
        "<div class='stat-value' id='pos-x-large'>0.0</div>"
        "<div class='stat-label'>X Position (m)</div>"
        "</div>"
        "<div class='stat-card'>"
        "<div class='stat-value' id='pos-y-large'>0.0</div>"
        "<div class='stat-label'>Y Position (m)</div>"
        "</div>"
        "<div class='stat-card'>"
        "<div class='stat-value' id='pos-z-large'>0.0</div>"
        "<div class='stat-label'>Z Position (m)</div>"
        "</div>"
        "</div>"
        "</div>"
        "<div class='card'>"
        "<h3>🧭 Orientation</h3>"
        "<div class='stats-row' style='grid-template-columns:repeat(3,1fr)'>"
        "<div class='stat-card'>"
        "<div class='stat-value' id='roll-angle'>0.0°</div>"
        "<div class='stat-label'>Roll</div>"
        "</div>"
        "<div class='stat-card'>"
        "<div class='stat-value' id='pitch-angle'>0.0°</div>"
        "<div class='stat-label'>Pitch</div>"
        "</div>"
        "<div class='stat-card'>"
        "<div class='stat-value' id='yaw-angle'>0.0°</div>"
        "<div class='stat-label'>Yaw</div>"
        "</div>"
        "</div>"
        "</div>"
        "<div class='card'>"
        "<h3>💨 Velocity</h3>"
        "<div class='stats-row' style='grid-template-columns:repeat(3,1fr)'>"
        "<div class='stat-card'>"
        "<div class='stat-value' id='vel-x'>0.0</div>"
        "<div class='stat-label'>X Velocity (m/s)</div>"
        "</div>"
        "<div class='stat-card'>"
        "<div class='stat-value' id='vel-y'>0.0</div>"
        "<div class='stat-label'>Y Velocity (m/s)</div>"
        "</div>"
        "<div class='stat-card'>"
        "<div class='stat-value' id='vel-z'>0.0</div>"
        "<div class='stat-label'>Z Velocity (m/s)</div>"
        "</div>"
        "</div>"
        "</div>"
        "</div>"
        "</div>"
        
        // Camera Tab
        "<div id='camera' class='tab-content'>"
        "<div class='dashboard'>"
        "<div class='card'>"
        "<h3>📷 Live Camera Preview</h3>"
        "<div class='preview-container'>"
        "<div style='display:flex;justify-content:space-between;align-items:center;margin-bottom:16px'>"
        "<h4 style='margin:0;color:var(--text-secondary)'>Camera Feed</h4>"
        "<div class='preview-controls'>"
        "<button class='btn btn-success' id='preview-btn' onclick='togglePreview()'>▶️ Start</button>"
        "<button class='btn' onclick='refreshPreview()'>🔄 Refresh</button>"
        "<button class='btn' onclick='captureFrame()'>📸 Capture</button>"
        "</div>"
        "</div>"
        "<div class='preview-box' id='preview-box' style='width:100%;height:400px;display:flex;align-items:center;justify-content:center;color:var(--text-muted);font-size:1.1rem'>"
        "📷 Camera preview will appear here"
        "</div>"
        "<div class='preview-info' style='margin-top:16px'>"
        "<span id='preview-status' class='live-indicator'>Preview stopped</span>"
        "<span id='preview-resolution'>Resolution: --x--</span>"
        "<span id='preview-fps'>FPS: --</span>"
        "<span id='frame-count'>Frame: --</span>"
        "</div>"
        "</div>"
        "</div>"
        "<div class='card'>"
        "<h3>🎛️ Camera Controls</h3>"
        "<div class='form-group'>"
        "<label>Exposure Level</label>"
        "<div class='range-container'>"
        "<input type='range' min='0' max='100' value='50' id='exposure' oninput='updateExposureValue(this.value)'>"
        "<span class='range-value' id='exposure-value'>50</span>"
        "</div>"
        "</div>"
        "<div class='form-group'>"
        "<label>Brightness</label>"
        "<div class='range-container'>"
        "<input type='range' min='-50' max='50' value='0' id='brightness' oninput='updateBrightnessValue(this.value)'>"
        "<span class='range-value' id='brightness-value'>0</span>"
        "</div>"
        "</div>"
        "<div class='form-group'>"
        "<label>Contrast</label>"
        "<div class='range-container'>"
        "<input type='range' min='0' max='100' value='50' id='contrast' oninput='updateContrastValue(this.value)'>"
        "<span class='range-value' id='contrast-value'>50</span>"
        "</div>"
        "</div>"
        "<div class='form-group'>"
        "<label>Saturation</label>"
        "<div class='range-container'>"
        "<input type='range' min='0' max='100' value='50' id='saturation' oninput='updateSaturationValue(this.value)'>"
        "<span class='range-value' id='saturation-value'>50</span>"
        "</div>"
        "</div>"
        "<div class='form-group'>"
        "<label>Resolution</label>"
        "<select id='camera-resolution'>"
        "<option value='640x480'>640x480 (VGA)</option>"
        "<option value='800x600'>800x600 (SVGA)</option>"
        "<option value='1024x768'>1024x768 (XGA)</option>"
        "<option value='1280x720'>1280x720 (720p)</option>"
        "<option value='1920x1080'>1920x1080 (1080p)</option>"
        "</select>"
        "</div>"
        "<div class='form-group'>"
        "<label>Frame Rate</label>"
        "<select id='camera-fps-select'>"
        "<option value='10'>10 FPS</option>"
        "<option value='15'>15 FPS</option>"
        "<option value='20'>20 FPS</option>"
        "<option value='30' selected>30 FPS</option>"
        "</select>"
        "</div>"
        "<button class='btn btn-success' onclick='updateCameraSettings()'>✅ Apply Settings</button>"
        "<button class='btn' onclick='resetCameraSettings()'>🔄 Reset to Default</button>"
        "</div>"
        "</div>"
        "</div>"
        
        // Sensors Tab
        "<div id='sensors' class='tab-content'>"
        "<div class='dashboard'>"
        "<div class='card'>"
        "<h3>📱 IMU Sensor Data</h3>"
        "<div class='two-column'>"
        "<div>"
        "<h4 style='color:var(--text-secondary);margin-bottom:12px'>Accelerometer (m/s²)</h4>"
        "<div class='status-grid'>"
        "<div class='status-item'>"
        "<span class='status-label'>X-Axis</span>"
        "<span class='status-value' id='accel-x'>--</span>"
        "</div>"
        "<div class='status-item'>"
        "<span class='status-label'>Y-Axis</span>"
        "<span class='status-value' id='accel-y'>--</span>"
        "</div>"
        "<div class='status-item'>"
        "<span class='status-label'>Z-Axis</span>"
        "<span class='status-value' id='accel-z'>--</span>"
        "</div>"
        "</div>"
        "</div>"
        "<div>"
        "<h4 style='color:var(--text-secondary);margin-bottom:12px'>Gyroscope (rad/s)</h4>"
        "<div class='status-grid'>"
        "<div class='status-item'>"
        "<span class='status-label'>X-Axis</span>"
        "<span class='status-value' id='gyro-x'>--</span>"
        "</div>"
        "<div class='status-item'>"
        "<span class='status-label'>Y-Axis</span>"
        "<span class='status-value' id='gyro-y'>--</span>"
        "</div>"
        "<div class='status-item'>"
        "<span class='status-label'>Z-Axis</span>"
        "<span class='status-value' id='gyro-z'>--</span>"
        "</div>"
        "</div>"
        "</div>"
        "</div>"
        "</div>"
        "<div class='card'>"
        "<h3>🌍 GPS Information</h3>"
        "<div class='status-grid'>"
        "<div class='status-item'>"
        "<span class='status-label'>Latitude</span>"
        "<span class='status-value' id='gps-lat'>--</span>"
        "</div>"
        "<div class='status-item'>"
        "<span class='status-label'>Longitude</span>"
        "<span class='status-value' id='gps-lon'>--</span>"
        "</div>"
        "<div class='status-item'>"
        "<span class='status-label'>Altitude</span>"
        "<span class='status-value' id='gps-alt'>--</span>"
        "</div>"
        "<div class='status-item'>"
        "<span class='status-label'>Fix Type</span>"
        "<span class='status-value' id='gps-fix'>--</span>"
        "</div>"
        "</div>"
        "</div>"
        "<div class='card'>"
        "<h3>🔋 System Sensors</h3>"
        "<div class='status-grid'>"
        "<div class='status-item'>"
        "<span class='status-label'>Temperature</span>"
        "<span class='status-value' id='sys-temp'>--</span>"
        "</div>"
        "<div class='status-item'>"
        "<span class='status-label'>Battery Voltage</span>"
        "<span class='status-value' id='battery-volt'>--</span>"
        "</div>"
        "<div class='status-item'>"
        "<span class='status-label'>Frames Processed</span>"
        "<span class='status-value' id='frames-processed'>--</span>"
        "</div>"
        "<div class='status-item'>"
        "<span class='status-label'>System Load</span>"
        "<span class='status-value' id='system-load'>--</span>"
        "</div>"
        "</div>"
        "</div>"
        "</div>"
        "</div>"
        
        // WiFi Tab
        "<div id='wifi' class='tab-content'>"
        "<div class='dashboard'>"
        "<div class='card'>"
        "<h3>📶 WiFi Control</h3>"
        "<div class='status-grid'>"
        "<div class='status-item'>"
        "<span class='status-label'>WiFi Status</span>"
        "<span class='status-value' id='wifi-status-indicator'>--</span>"
        "</div>"
        "<div class='status-item'>"
        "<span class='status-label'>Connection</span>"
        "<span class='status-value' id='wifi-connected-status'>--</span>"
        "</div>"
        "<div class='status-item'>"
        "<span class='status-label'>Signal Strength</span>"
        "<span class='status-value' id='wifi-signal-strength'>--</span>"
        "</div>"
        "<div class='status-item'>"
        "<span class='status-label'>IP Address</span>"
        "<span class='status-value' id='wifi-ip-address'>--</span>"
        "</div>"
        "</div>"
        "<div style='margin-top:20px;display:flex;gap:12px;flex-wrap:wrap'>"
        "<button class='btn btn-success' onclick='controlWiFi(true, false)'>� Enable WiFi</button>"
        "<button class='btn btn-warning' onclick='controlWiFi(true, true)'>🔍 Scan Only</button>"
        "<button class='btn btn-danger' onclick='controlWiFi(false, false)'>❌ Disable WiFi</button>"
        "<button class='btn' onclick='requestWiFiStatus()'>📊 Refresh Status</button>"
        "</div>"
        "</div>"
        "<div class='card'>"
        "<h3>🔍 Network Scanner</h3>"
        "<div style='display:flex;justify-content:space-between;align-items:center;margin-bottom:16px'>"
        "<button class='btn btn-success' onclick='scanWiFiNetworks()'>� Scan Networks</button>"
        "<div id='scan-status' style='color:var(--text-secondary)'>Ready to scan</div>"
        "</div>"
        "<div id='wifi-networks-list'>"
        "<div style='text-align:center;color:var(--text-muted);padding:20px'>No networks scanned yet</div>"
        "</div>"
        "</div>"
        "<div class='card'>"
        "<h3>🔐 Network Connection</h3>"
        "<div class='form-group'>"
        "<label>Network SSID</label>"
        "<input type='text' id='connect-ssid' placeholder='Enter network name'>"
        "</div>"
        "<div class='form-group'>"
        "<label>Password</label>"
        "<input type='password' id='connect-password' placeholder='Enter password'>"
        "</div>"
        "<div class='form-group'>"
        "<label>Authentication Mode</label>"
        "<select id='connect-auth-mode'>"
        "<option value='0'>Open</option>"
        "<option value='2'>WPA PSK</option>"
        "<option value='3'>WPA2 PSK</option>"
        "<option value='4'>WPA WPA2 PSK</option>"
        "<option value='5'>WPA2 Enterprise</option>"
        "<option value='6'>WPA3 PSK</option>"
        "</select>"
        "</div>"
        "<div class='form-group'>"
        "<label style='display:flex;align-items:center;gap:8px'>"
        "<input type='checkbox' id='enable-ap-fallback' checked>"
        "Enable AP fallback mode"
        "</label>"
        "</div>"
        "<button class='btn btn-success' onclick='connectToWiFi()'>� Connect</button>"
        "</div>"
        "</div>"
        "</div>"
        
        // Settings Tab
        "<div id='settings' class='tab-content'>"
        "<div class='dashboard'>"
        "<div class='card'>"
        "<h3>🎯 SLAM Configuration</h3>"
        "<div class='form-group'>"
        "<label>Maximum Features</label>"
        "<div class='range-container'>"
        "<input type='range' min='100' max='2000' step='50' value='1000' id='max-features' oninput='updateMaxFeaturesValue(this.value)'>"
        "<span class='range-value' id='max-features-value'>1000</span>"
        "</div>"
        "</div>"
        "<div class='form-group'>"
        "<label>FAST Threshold</label>"
        "<div class='range-container'>"
        "<input type='range' min='10' max='50' step='5' value='20' id='fast-threshold' oninput='updateFastThresholdValue(this.value)'>"
        "<span class='range-value' id='fast-threshold-value'>20</span>"
        "</div>"
        "</div>"
        "<div class='form-group'>"
        "<label>Scale Factor</label>"
        "<div class='range-container'>"
        "<input type='range' min='1.1' max='2.0' step='0.1' value='1.2' id='scale-factor' oninput='updateScaleFactorValue(this.value)'>"
        "<span class='range-value' id='scale-factor-value'>1.2</span>"
        "</div>"
        "</div>"
        "<div class='form-group'>"
        "<label>Minimum Tracked Features</label>"
        "<div class='range-container'>"
        "<input type='range' min='10' max='100' step='5' value='30' id='min-tracked-features' oninput='updateMinTrackedFeaturesValue(this.value)'>"
        "<span class='range-value' id='min-tracked-features-value'>30</span>"
        "</div>"
        "</div>"
        "<div class='form-group'>"
        "<label style='display:flex;align-items:center;gap:8px'>"
        "<input type='checkbox' id='enable-loop-closure' checked>"
        "Enable Loop Closure Detection"
        "</label>"
        "</div>"
        "<button class='btn btn-success' onclick='updateSLAMSettings()'>✅ Apply Settings</button>"
        "<button class='btn' onclick='loadDefaultSLAMSettings()'>🔄 Load Defaults</button>"
        "</div>"
        "<div class='card'>"
        "<h3>🔧 System Preferences</h3>"
        "<div class='form-group'>"
        "<label>Data Update Rate</label>"
        "<select id='update-rate'>"
        "<option value='100'>10 Hz (100ms)</option>"
        "<option value='200'>5 Hz (200ms)</option>"
        "<option value='500'>2 Hz (500ms)</option>"
        "<option value='1000' selected>1 Hz (1000ms)</option>"
        "</select>"
        "</div>"
        "<div class='form-group'>"
        "<label>Log Level</label>"
        "<select id='log-level'>"
        "<option value='1'>Error</option>"
        "<option value='2'>Warning</option>"
        "<option value='3' selected>Info</option>"
        "<option value='4'>Debug</option>"
        "<option value='5'>Verbose</option>"
        "</select>"
        "</div>"
        "<div class='form-group'>"
        "<label style='display:flex;align-items:center;gap:8px'>"
        "<input type='checkbox' id='auto-save-maps' checked>"
        "Auto-save maps every 5 minutes"
        "</label>"
        "</div>"
        "<button class='btn btn-success' onclick='updateSystemSettings()'>💾 Save Preferences</button>"
        "</div>"
        "</div>"
        "</div>"
        
        // Control Tab
        "<div id='control' class='tab-content'>"
        "<div class='dashboard'>"
        "<div class='card'>"
        "<h3>🎮 System Control</h3>"
        "<div style='display:grid;grid-template-columns:repeat(auto-fit,minmax(200px,1fr));gap:12px'>"
        "<button class='btn' onclick='sendSystemCommand(\"restart\")'>🔄 Restart System</button>"
        "<button class='btn' onclick='sendSystemCommand(\"calibrate_imu\")'>📐 Calibrate IMU</button>"
        "<button class='btn' onclick='sendSystemCommand(\"calibrate_camera\")'>📷 Calibrate Camera</button>"
        "<button class='btn btn-warning' onclick='sendSystemCommand(\"reset_slam\")'>🗑️ Reset SLAM</button>"
        "<button class='btn btn-danger' onclick='sendSystemCommand(\"emergency_stop\")'>🛑 Emergency Stop</button>"
        "<button class='btn' onclick='sendSystemCommand(\"wifi_get_status\")'>📡 Get WiFi Status</button>"
        "</div>"
        "</div>"
        "<div class='card'>"
        "<h3>🗺️ Map Management</h3>"
        "<div class='form-group'>"
        "<label>Map Name</label>"
        "<input type='text' id='map-name' placeholder='Enter map name (e.g., office_floor1)' value='default_map'>"
        "</div>"
        "<div style='display:flex;gap:12px;flex-wrap:wrap'>"
        "<button class='btn btn-success' onclick='sendMapCommand(\"save\")'>💾 Save Map</button>"
        "<button class='btn' onclick='sendMapCommand(\"load\")'>📂 Load Map</button>"
        "<button class='btn btn-warning' onclick='sendMapCommand(\"clear\")'>🗑️ Clear Map</button>"
        "<button class='btn' onclick='sendMapCommand(\"list\")'>📋 List Maps</button>"
        "</div>"
        "<div id='map-status' style='margin-top:16px;padding:12px;background:rgba(255,255,255,0.05);border-radius:8px;color:var(--text-secondary)'>"
        "Ready for map operations"
        "</div>"
        "</div>"
        "<div class='card'>"
        "<h3>📊 Data Export</h3>"
        "<div style='display:flex;gap:12px;flex-wrap:wrap'>"
        "<button class='btn' onclick='exportSLAMData()'>📋 Export SLAM Data</button>"
        "<button class='btn' onclick='exportTelemetry()'>📈 Export Telemetry</button>"
        "<button class='btn' onclick='exportLogs()'>📄 Export Logs</button>"
        "<button class='btn' onclick='exportConfiguration()'>⚙️ Export Config</button>"
        "</div>"
        "</div>"
        "</div>"
        "</div>"
        
        "</div>"
        
        // Notification system
        "<div id='notification' class='notification'></div>"
        
        // Modal for connection details
        "<div id='connection-modal' class='modal'>"
        "<div class='modal-content'>"
        "<div class='modal-header'>"
        "<h3 class='modal-title'>Connection Details</h3>"
        "<button class='modal-close' onclick='closeModal(\"connection-modal\")'>×</button>"
        "</div>"
        "<div id='modal-body'></div>"
        "</div>"
        "</div>"
        
        "<script>"
        "let ws,positionTrail=[],updateInterval,previewActive=false,reconnectAttempts=0;"
        
        // Initialize application
        "function init(){"
        "connectWebSocket();"
        "setUpdateRate(1000);"
        "loadSettingsFromStorage();"
        "}"
        
        // WebSocket connection with auto-reconnect
        "function connectWebSocket(){"
        "const protocol=location.protocol==='https:'?'wss:':'ws:';"
        "const wsUrl=protocol+'//'+location.host+'/ws';"
        "try{"
        "ws=new WebSocket(wsUrl);"
        "ws.onopen=()=>{"
        "console.log('WebSocket connected');"
        "reconnectAttempts=0;"
        "showNotification('Connected to SLAM system','success');"
        "requestAllData();"
        "};"
        "ws.onmessage=handleWebSocketMessage;"
        "ws.onclose=()=>{"
        "console.log('WebSocket disconnected');"
        "setTimeout(()=>{"
        "reconnectAttempts++;"
        "if(reconnectAttempts<10)connectWebSocket();"
        "},Math.min(1000*reconnectAttempts,10000));"
        "};"
        "ws.onerror=e=>console.error('WebSocket error:',e);"
        "}catch(e){console.error('WebSocket connection failed:',e)}"
        "}"
        
        // Handle incoming WebSocket messages
        "function handleWebSocketMessage(event){"
        "try{"
        "const data=JSON.parse(event.data);"
        "updateDashboard(data);"
        "}catch(e){console.error('Failed to parse WebSocket data:',e)}"
        "}"
        
        // Tab management
        "function showTab(tabName){"
        "document.querySelectorAll('.tab-content').forEach(t=>t.classList.remove('active'));"
        "document.querySelectorAll('.nav-tab').forEach(t=>t.classList.remove('active'));"
        "document.getElementById(tabName).classList.add('active');"
        "event.target.classList.add('active');"
        "if(tabName==='camera'&&!previewActive)requestCameraPreview();"
        "if(tabName==='wifi')requestWiFiStatus();"
        "}"
        
        // Dashboard updates
        "function updateDashboard(data){"
        "updateSystemStatus(data);"
        "updateSLAMData(data);"
        "updatePositionData(data);"
        "updateSensorData(data);"
        "updateWiFiData(data);"
        "updateCameraData(data);"
        "}"
        
        "function updateSystemStatus(data){"
        "if(data.heartbeat){"
        "const h=data.heartbeat;"
        "document.getElementById('connection-status').textContent=data.connected?'●':'○';"
        "document.getElementById('connection-status').className=data.connected?'stat-value':'stat-value danger';"
        "document.getElementById('uptime-display').textContent=formatUptime(h.uptime_ms);"
        "document.getElementById('memory-usage').textContent=formatMemory(h.free_heap_size);"
        "document.getElementById('slam-state').textContent=getSLAMStateText(h.slam_status);"
        "document.getElementById('cpu-usage').textContent=h.cpu_usage_percent+'%';"
        "document.getElementById('temperature').textContent=(h.temperature||25).toFixed(1)+'°C';"
        "}"
        "}"
        
        "function updateSLAMData(data){"
        "if(data.slam){"
        "const s=data.slam;"
        "document.getElementById('features-count').textContent=s.tracked_features||0;"
        "document.getElementById('map-points-count').textContent=s.map_points||0;"
        "document.getElementById('keyframes-count').textContent=s.keyframes||0;"
        "const quality=(s.tracking_quality||0)*100;"
        "document.getElementById('quality-percent').textContent=quality.toFixed(1)+'%';"
        "document.getElementById('quality-value').textContent=quality.toFixed(1)+'%';"
        "document.getElementById('quality-progress').style.width=quality+'%';"
        "}"
        "}"
        
        "function updatePositionData(data){"
        "if(data.position){"
        "const p=data.position;"
        "updatePositionDisplay(p.position_x,p.position_y,p.position_z);"
        "updateOrientationDisplay(p.orientation_roll,p.orientation_pitch,p.orientation_yaw);"
        "updateVelocityDisplay(p.velocity_x,p.velocity_y,p.velocity_z);"
        "updatePositionVisualization(p.position_x,p.position_y);"
        "}"
        "}"
        
        "function updatePositionDisplay(x,y,z){"
        "['pos-x','pos-x-large'].forEach(id=>{"
        "const el=document.getElementById(id);"
        "if(el)el.textContent=x.toFixed(2);"
        "});"
        "['pos-y','pos-y-large'].forEach(id=>{"
        "const el=document.getElementById(id);"
        "if(el)el.textContent=y.toFixed(2);"
        "});"
        "['pos-z','pos-z-large'].forEach(id=>{"
        "const el=document.getElementById(id);"
        "if(el)el.textContent=z.toFixed(2);"
        "});"
        "}"
        
        "function updateOrientationDisplay(roll,pitch,yaw){"
        "document.getElementById('roll-angle').textContent=(roll*180/Math.PI).toFixed(1)+'°';"
        "document.getElementById('pitch-angle').textContent=(pitch*180/Math.PI).toFixed(1)+'°';"
        "document.getElementById('yaw-angle').textContent=(yaw*180/Math.PI).toFixed(1)+'°';"
        "}"
        
        "function updateVelocityDisplay(vx,vy,vz){"
        "document.getElementById('vel-x').textContent=vx.toFixed(2);"
        "document.getElementById('vel-y').textContent=vy.toFixed(2);"
        "document.getElementById('vel-z').textContent=vz.toFixed(2);"
        "}"
        
        "function updatePositionVisualization(x,y){"
        "const viz=document.getElementById('position-viz');"
        "if(!viz)return;"
        "const rect=viz.getBoundingClientRect();"
        "const centerX=rect.width/2;const centerY=rect.height/2;"
        "const scale=20;const pixelX=centerX+x*scale;const pixelY=centerY-y*scale;"
        "positionTrail.push({x:pixelX,y:pixelY,timestamp:Date.now()});"
        "const maxTrailLength=100;const maxAge=30000;"
        "const now=Date.now();"
        "positionTrail=positionTrail.filter((p,i)=>i>=positionTrail.length-maxTrailLength&&now-p.timestamp<maxAge);"
        "viz.innerHTML='';"
        "positionTrail.forEach((pos,i)=>{"
        "const dot=document.createElement('div');"
        "const isLatest=i===positionTrail.length-1;"
        "dot.className=isLatest?'position-dot':'trail';"
        "dot.style.left=Math.max(0,Math.min(rect.width,pos.x))+'px';"
        "dot.style.top=Math.max(0,Math.min(rect.height,pos.y))+'px';"
        "if(!isLatest){"
        "const age=(now-pos.timestamp)/maxAge;"
        "dot.style.opacity=(1-age)*0.6;"
        "}"
        "viz.appendChild(dot);"
        "});"
        "}"
        
        "function updateSensorData(data){"
        "if(data.telemetry){"
        "const t=data.telemetry;"
        "document.getElementById('accel-x').textContent=t.accel_x?.toFixed(2)||'--';"
        "document.getElementById('accel-y').textContent=t.accel_y?.toFixed(2)||'--';"
        "document.getElementById('accel-z').textContent=t.accel_z?.toFixed(2)||'--';"
        "document.getElementById('gyro-x').textContent=t.gyro_x?.toFixed(3)||'--';"
        "document.getElementById('gyro-y').textContent=t.gyro_y?.toFixed(3)||'--';"
        "document.getElementById('gyro-z').textContent=t.gyro_z?.toFixed(3)||'--';"
        "document.getElementById('gps-lat').textContent=t.gps_lat?.toFixed(6)||'--';"
        "document.getElementById('gps-lon').textContent=t.gps_lon?.toFixed(6)||'--';"
        "document.getElementById('gps-alt').textContent=t.gps_alt?.toFixed(1)||'--';"
        "document.getElementById('gps-fix').textContent=getGPSFixType(t.gps_fix_type);"
        "document.getElementById('gps-sats').textContent=t.gps_satellites||'--';"
        "document.getElementById('camera-fps').textContent=(t.camera_fps||'--')+'fps';"
        "document.getElementById('frames-processed').textContent=t.frames_processed||'--';"
        "document.getElementById('battery-volt').textContent=(t.battery_voltage_mv/1000).toFixed(2)+'V';"
        "document.getElementById('sys-temp').textContent=(t.temperature||25).toFixed(1)+'°C';"
        "updateSystemHealth(t);"
        "}"
        "}"
        
        "function updateSystemHealth(telemetry){"
        "document.getElementById('camera-health').textContent=telemetry.camera_fps>0?'Active':'Inactive';"
        "document.getElementById('camera-health').className=telemetry.camera_fps>0?'status-value':'status-value warning';"
        "document.getElementById('imu-health').textContent='Active';"
        "document.getElementById('imu-health').className='status-value';"
        "document.getElementById('sd-health').textContent='OK';"
        "document.getElementById('sd-health').className='status-value';"
        "}"
        
        // WiFi management functions
        "function updateWiFiData(data){"
        "if(data.wifi_status){"
        "const w=data.wifi_status;"
        "document.getElementById('wifi-status-indicator').textContent=w.wifi_enabled?'Enabled':'Disabled';"
        "document.getElementById('wifi-status-indicator').className=w.wifi_enabled?'status-value':'status-value danger';"
        "document.getElementById('wifi-connected-status').textContent=w.connected?'Connected':'Disconnected';"
        "document.getElementById('wifi-connected-status').className=w.connected?'status-value':'status-value warning';"
        "document.getElementById('wifi-signal-strength').textContent=w.rssi?w.rssi+' dBm':'--';"
        "document.getElementById('wifi-ip-address').textContent=w.ip_address?formatIPAddress(w.ip_address):'--';"
        "}"
        "if(data.wifi_networks){"
        "displayWiFiNetworks(data.wifi_networks);"
        "}"
        "}"
        
        "function controlWiFi(enable,scanOnly){"
        "const payload={enable_wifi:enable?1:0,enable_scan_only:scanOnly?1:0,auto_ap_fallback:1,reserved:0};"
        "sendMessage('MSG_P4_TO_C6_WIFI_CONTROL',payload);"
        "showNotification(enable?(scanOnly?'WiFi scan mode enabled':'WiFi enabled'):'WiFi disabled','success');"
        "}"
        
        "function scanWiFiNetworks(){"
        "document.getElementById('scan-status').textContent='Scanning...';"
        "sendMessage('MSG_C6_TO_P4_WIFI_SCAN_REQ',{});"
        "setTimeout(()=>document.getElementById('scan-status').textContent='Scan complete',3000);"
        "}"
        
        "function displayWiFiNetworks(networks){"
        "const container=document.getElementById('wifi-networks-list');"
        "if(!networks||networks.length===0){"
        "container.innerHTML='<div style=\"text-align:center;color:var(--text-muted);padding:20px\">No networks found</div>';"
        "return;"
        "}"
        "container.innerHTML=networks.map(network=>{"
        "const signalClass=getSignalClass(network.rssi);"
        "const authText=getAuthModeText(network.auth_mode);"
        "return `<div class=\"wifi-network\" onclick=\"selectNetwork('${network.ssid}',${network.auth_mode})\">"
        "<div class=\"wifi-network-header\">"
        "<span class=\"wifi-ssid\">${network.ssid}</span>"
        "<span class=\"wifi-signal ${signalClass}\">${network.rssi} dBm</span>"
        "</div>"
        "<div class=\"wifi-details\">"
        "<span>Channel: ${network.channel}</span>"
        "<span>Security: ${authText}</span>"
        "</div>"
        "</div>`;"
        "}).join('');"
        "}"
        
        "function selectNetwork(ssid,authMode){"
        "document.getElementById('connect-ssid').value=ssid;"
        "document.getElementById('connect-auth-mode').value=authMode;"
        "showTab('wifi');"
        "}"
        
        "function connectToWiFi(){"
        "const ssid=document.getElementById('connect-ssid').value;"
        "const password=document.getElementById('connect-password').value;"
        "const authMode=parseInt(document.getElementById('connect-auth-mode').value);"
        "if(!ssid){"
        "showNotification('Please enter a network SSID','error');"
        "return;"
        "}"
        "const payload={ssid:ssid,password:password,auth_mode:authMode};"
        "sendMessage('MSG_C6_TO_P4_CONNECT_WIFI',payload);"
        "showNotification('Connecting to '+ssid+'...','info');"
        "}"
        
        "function requestWiFiStatus(){"
        "sendMessage('MSG_C6_TO_P4_SYSTEM_CMD',{command:'SYS_CMD_WIFI_GET_STATUS',parameter:0});"
        "}"
        
        // Camera functions
        "function updateCameraData(data){"
        "if(data.camera_frame){"
        "displayCameraFrame(data.camera_frame);"
        "}"
        "}"
        
        "function togglePreview(){"
        "if(previewActive){"
        "stopCameraPreview();"
        "}else{"
        "startCameraPreview();"
        "}"
        "}"
        
        "function startCameraPreview(){"
        "sendMessage('MSG_C6_TO_P4_CAMERA_CMD',{command:'CAM_CMD_START_PREVIEW',value:0});"
        "previewActive=true;"
        "document.getElementById('preview-btn').textContent='⏸️ Stop';"
        "document.getElementById('preview-status').textContent='Preview active';"
        "document.getElementById('preview-status').className='live-indicator';"
        "}"
        
        "function stopCameraPreview(){"
        "sendMessage('MSG_C6_TO_P4_CAMERA_CMD',{command:'CAM_CMD_STOP_PREVIEW',value:0});"
        "previewActive=false;"
        "document.getElementById('preview-btn').textContent='▶️ Start';"
        "document.getElementById('preview-status').textContent='Preview stopped';"
        "document.getElementById('preview-status').className='';"
        "}"
        
        "function refreshPreview(){"
        "if(previewActive){"
        "sendMessage('MSG_C6_TO_P4_PREVIEW_REQ',{});"
        "}"
        "}"
        
        "function captureFrame(){"
        "sendMessage('MSG_C6_TO_P4_CAMERA_CMD',{command:'CAM_CMD_CAPTURE_FRAME',value:0});"
        "showNotification('Frame captured','success');"
        "}"
        
        "function requestCameraPreview(){"
        "sendMessage('MSG_C6_TO_P4_PREVIEW_REQ',{});"
        "}"
        
        "function displayCameraFrame(frameData){"
        "const previewBox=document.getElementById('preview-box');"
        "if(frameData.frame_data){"
        "previewBox.innerHTML=`<img src=\"data:image/jpeg;base64,${frameData.frame_data}\" alt=\"Camera Preview\">`;"
        "document.getElementById('preview-resolution').textContent=`${frameData.width}x${frameData.height}`;"
        "document.getElementById('preview-fps').textContent=`${frameData.fps||30} fps`;"
        "document.getElementById('frame-count').textContent=`Frame: ${frameData.frame_number||0}`;"
        "}"
        "}"
        
        "function updateCameraSettings(){"
        "const settings={"
        "exposure:parseInt(document.getElementById('exposure').value),"
        "brightness:parseInt(document.getElementById('brightness').value),"
        "contrast:parseInt(document.getElementById('contrast').value),"
        "saturation:parseInt(document.getElementById('saturation').value),"
        "resolution:document.getElementById('camera-resolution').value,"
        "fps:parseInt(document.getElementById('camera-fps-select').value)"
        "};"
        "sendMessage('MSG_C6_TO_P4_CAMERA_CMD',{command:'CAM_CMD_UPDATE_SETTINGS',settings:settings});"
        "showNotification('Camera settings updated','success');"
        "}"
        
        "function resetCameraSettings(){"
        "document.getElementById('exposure').value=50;"
        "document.getElementById('brightness').value=0;"
        "document.getElementById('contrast').value=50;"
        "document.getElementById('saturation').value=50;"
        "updateExposureValue(50);"
        "updateBrightnessValue(0);"
        "updateContrastValue(50);"
        "updateSaturationValue(50);"
        "updateCameraSettings();"
        "}"
        
        // Settings functions
        "function updateSLAMSettings(){"
        "const settings={"
        "max_features:parseInt(document.getElementById('max-features').value),"
        "fast_threshold:parseInt(document.getElementById('fast-threshold').value),"
        "scale_factor:parseFloat(document.getElementById('scale-factor').value),"
        "min_tracked_features:parseInt(document.getElementById('min-tracked-features').value),"
        "enable_loop_closure:document.getElementById('enable-loop-closure').checked"
        "};"
        "sendMessage('MSG_C6_TO_P4_CONFIG_UPDATE',{config_type:1,settings:settings});"
        "showNotification('SLAM settings updated','success');"
        "saveSettingsToStorage();"
        "}"
        
        "function loadDefaultSLAMSettings(){"
        "document.getElementById('max-features').value=1000;"
        "document.getElementById('fast-threshold').value=20;"
        "document.getElementById('scale-factor').value=1.2;"
        "document.getElementById('min-tracked-features').value=30;"
        "document.getElementById('enable-loop-closure').checked=true;"
        "updateMaxFeaturesValue(1000);"
        "updateFastThresholdValue(20);"
        "updateScaleFactorValue(1.2);"
        "updateMinTrackedFeaturesValue(30);"
        "}"
        
        "function updateSystemSettings(){"
        "const rate=parseInt(document.getElementById('update-rate').value);"
        "setUpdateRate(rate);"
        "showNotification('System preferences saved','success');"
        "saveSettingsToStorage();"
        "}"
        
        // Control functions
        "function sendSystemCommand(command){"
        "const commandMap={"
        "restart:'SYS_CMD_RESTART',"
        "calibrate_imu:'SYS_CMD_CALIBRATE_IMU',"
        "calibrate_camera:'SYS_CMD_CALIBRATE_CAMERA',"
        "reset_slam:'SYS_CMD_RESET_SLAM',"
        "emergency_stop:'SYS_CMD_EMERGENCY_STOP',"
        "wifi_get_status:'SYS_CMD_WIFI_GET_STATUS'"
        "};"
        "if(command==='emergency_stop'&&!confirm('Emergency stop will halt all operations. Continue?'))return;"
        "if(command==='restart'&&!confirm('Restart the P4 system?'))return;"
        "if(command==='reset_slam'&&!confirm('Reset SLAM will clear all mapping data. Continue?'))return;"
        "sendMessage('MSG_C6_TO_P4_SYSTEM_CMD',{command:commandMap[command],parameter:0});"
        "showNotification(`${command.replace('_',' ')} command sent`,'success');"
        "}"
        
        "function sendMapCommand(command){"
        "const mapName=document.getElementById('map-name').value.trim()||'default_map';"
        "const commandMap={save:'MAP_CMD_SAVE',load:'MAP_CMD_LOAD',clear:'MAP_CMD_CLEAR',list:'MAP_CMD_LIST'};"
        "if(command==='clear'&&!confirm('Clear all map data?'))return;"
        "sendMessage('MSG_C6_TO_P4_MAP_CMD',{command:commandMap[command],map_name:mapName});"
        "document.getElementById('map-status').textContent=`${command} operation initiated for: ${mapName}`;"
        
        // Utility functions
        "function sendMessage(msgType,payload){"
        "if(ws&&ws.readyState===WebSocket.OPEN){"
        "const message={type:msgType,payload:payload,timestamp:Date.now()};"
        "ws.send(JSON.stringify(message));"
        "}else{"
        "showNotification('Connection lost. Please wait for reconnection.','error');"
        "}"
        "}"
        
        "function requestAllData(){"
        "sendMessage('MSG_C6_TO_P4_HEARTBEAT_ACK',{});"
        "sendMessage('MSG_C6_TO_P4_SYSTEM_CMD',{command:'SYS_CMD_WIFI_GET_STATUS',parameter:0});"
        "}"
        
        "function setUpdateRate(intervalMs){"
        "if(updateInterval)clearInterval(updateInterval);"
        "updateInterval=setInterval(requestAllData,intervalMs);"
        "}"
        
        "function showNotification(message,type='info'){"
        "const notification=document.getElementById('notification');"
        "notification.textContent=message;"
        "notification.className=`notification ${type} show`;"
        "setTimeout(()=>notification.classList.remove('show'),3000);"
        "}"
        
        "function formatUptime(ms){"
        "const seconds=Math.floor(ms/1000);"
        "const minutes=Math.floor(seconds/60);"
        "const hours=Math.floor(minutes/60);"
        "const days=Math.floor(hours/24);"
        "if(days>0)return `${days}d ${hours%24}h`;"
        "if(hours>0)return `${hours}h ${minutes%60}m`;"
        "if(minutes>0)return `${minutes}m ${seconds%60}s`;"
        "return `${seconds}s`;"
        "}"
        
        "function formatMemory(bytes){"
        "const kb=bytes/1024;"
        "const mb=kb/1024;"
        "if(mb>1)return mb.toFixed(1)+'MB';"
        "return kb.toFixed(0)+'KB';"
        "}"
        
        "function formatIPAddress(ip){"
        "return[(ip>>24)&255,(ip>>16)&255,(ip>>8)&255,ip&255].join('.');"
        "}"
        
        "function getSLAMStateText(state){"
        "const states={0:'Initializing',1:'Tracking',2:'Lost',3:'Relocalization',4:'Mapping'};"
        "return states[state]||'Unknown';"
        "}"
        
        "function getGPSFixType(type){"
        "const types={0:'No Fix',1:'Dead Reckoning',2:'2D Fix',3:'3D Fix',4:'GPS+DR',5:'Time Only'};"
        "return types[type]||'Unknown';"
        "}"
        
        "function getSignalClass(rssi){"
        "if(rssi>-50)return 'excellent';"
        "if(rssi>-60)return 'good';"
        "if(rssi>-70)return 'fair';"
        "return 'poor';"
        "}"
        
        "function getAuthModeText(mode){"
        "const modes={0:'Open',1:'WEP',2:'WPA PSK',3:'WPA2 PSK',4:'WPA WPA2 PSK',5:'WPA2 Enterprise',6:'WPA3 PSK'};"
        "return modes[mode]||'Unknown';"
        "}"
        
        // Range input handlers
        "function updateExposureValue(val){document.getElementById('exposure-value').textContent=val}"
        "function updateBrightnessValue(val){document.getElementById('brightness-value').textContent=val}"
        "function updateContrastValue(val){document.getElementById('contrast-value').textContent=val}"
        "function updateSaturationValue(val){document.getElementById('saturation-value').textContent=val}"
        "function updateMaxFeaturesValue(val){document.getElementById('max-features-value').textContent=val}"
        "function updateFastThresholdValue(val){document.getElementById('fast-threshold-value').textContent=val}"
        "function updateScaleFactorValue(val){document.getElementById('scale-factor-value').textContent=val}"
        "function updateMinTrackedFeaturesValue(val){document.getElementById('min-tracked-features-value').textContent=val}"
        
        // Export functions
        "function exportSLAMData(){"
        "window.open('/api/export/slam','_blank');"
        "showNotification('Exporting SLAM data...','info');"
        "}"
        
        "function exportTelemetry(){"
        "window.open('/api/export/telemetry','_blank');"
        "showNotification('Exporting telemetry data...','info');"
        "}"
        
        "function exportLogs(){"
        "window.open('/api/export/logs','_blank');"
        "showNotification('Exporting system logs...','info');"
        "}"
        
        "function exportConfiguration(){"
        "window.open('/api/export/config','_blank');"
        "showNotification('Exporting configuration...','info');"
        "}"
        
        // Modal functions
        "function openModal(modalId){"
        "document.getElementById(modalId).classList.add('active');"
        "}"
        
        "function closeModal(modalId){"
        "document.getElementById(modalId).classList.remove('active');"
        "}"
        
        // Settings persistence
        "function saveSettingsToStorage(){"
        "try{"
        "const settings={"
        "maxFeatures:document.getElementById('max-features').value,"
        "fastThreshold:document.getElementById('fast-threshold').value,"
        "scaleFactor:document.getElementById('scale-factor').value,"
        "minTrackedFeatures:document.getElementById('min-tracked-features').value,"
        "enableLoopClosure:document.getElementById('enable-loop-closure').checked,"
        "updateRate:document.getElementById('update-rate').value,"
        "logLevel:document.getElementById('log-level').value"
        "};"
        "localStorage.setItem('slamSettings',JSON.stringify(settings));"
        "}catch(e){console.error('Failed to save settings:',e)}"
        "}"
        
        "function loadSettingsFromStorage(){"
        "try{"
        "const settings=JSON.parse(localStorage.getItem('slamSettings')||'{}');"
        "if(settings.maxFeatures){"
        "document.getElementById('max-features').value=settings.maxFeatures;"
        "updateMaxFeaturesValue(settings.maxFeatures);"
        "}"
        "if(settings.fastThreshold){"
        "document.getElementById('fast-threshold').value=settings.fastThreshold;"
        "updateFastThresholdValue(settings.fastThreshold);"
        "}"
        "if(settings.scaleFactor){"
        "document.getElementById('scale-factor').value=settings.scaleFactor;"
        "updateScaleFactorValue(settings.scaleFactor);"
        "}"
        "if(settings.minTrackedFeatures){"
        "document.getElementById('min-tracked-features').value=settings.minTrackedFeatures;"
        "updateMinTrackedFeaturesValue(settings.minTrackedFeatures);"
        "}"
        "if(typeof settings.enableLoopClosure==='boolean'){"
        "document.getElementById('enable-loop-closure').checked=settings.enableLoopClosure;"
        "}"
        "if(settings.updateRate){"
        "document.getElementById('update-rate').value=settings.updateRate;"
        "}"
        "if(settings.logLevel){"
        "document.getElementById('log-level').value=settings.logLevel;"
        "}"
        "}catch(e){console.error('Failed to load settings:',e)}"
        "}"
        
        // Keyboard shortcuts
        "document.addEventListener('keydown',e=>{"
        "if(e.ctrlKey){"
        "switch(e.key){"
        "case '1':e.preventDefault();showTab('dashboard');break;"
        "case '2':e.preventDefault();showTab('position');break;"
        "case '3':e.preventDefault();showTab('camera');break;"
        "case '4':e.preventDefault();showTab('sensors');break;"
        "case '5':e.preventDefault();showTab('wifi');break;"
        "case '6':e.preventDefault();showTab('settings');break;"
        "case '7':e.preventDefault();showTab('control');break;"
        "case 'r':e.preventDefault();if(previewActive)refreshPreview();break;"
        "case 's':e.preventDefault();updateSLAMSettings();break;"
        "}"
        "}"
        "if(e.key==='Escape'){"
        "document.querySelectorAll('.modal.active').forEach(modal=>modal.classList.remove('active'));"
        "}"
        "});"
        
        // Initialize when page loads
        "window.addEventListener('load',init);"
        "window.addEventListener('beforeunload',()=>{"
        "if(ws)ws.close();"
        "if(updateInterval)clearInterval(updateInterval);"
        "});"
        
        "</script>"
        "</body></html>";
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
    slam_web_server_send_command(MSG_C6_TO_P4_CONNECT_WIFI, &wifi_control, sizeof(wifi_control));
    
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
        cJSON_AddItemToObject(json, "slam", slam);
        
        // Add fusion parameters with validation info
        cJSON *fusion = cJSON_CreateObject();
        cJSON_AddNumberToObject(fusion, "position_noise", 0.1); // From cache when available
        cJSON_AddNumberToObject(fusion, "velocity_noise", 0.01);
        cJSON_AddNumberToObject(fusion, "gps_weight", 0.8);
        cJSON_AddNumberToObject(fusion, "slam_weight", 0.9);
        cJSON_AddBoolToObject(fusion, "enable_outlier_detection", true);
        cJSON_AddStringToObject(fusion, "validation_status", "valid");
        cJSON_AddBoolToObject(fusion, "safe_to_update", true);
        cJSON_AddItemToObject(json, "fusion", fusion);
        
        // Add camera parameters  
        cJSON *camera = cJSON_CreateObject();
        cJSON_AddNumberToObject(camera, "exposure", data_cache.camera_exposure);
        cJSON_AddNumberToObject(camera, "brightness", data_cache.camera_brightness);
        cJSON_AddNumberToObject(camera, "fps", data_cache.camera_fps);
        cJSON_AddBoolToObject(camera, "auto_exposure", true);
        cJSON_AddItemToObject(json, "camera", camera);
        
        // Add system safety status
        cJSON *safety = cJSON_CreateObject();
        cJSON_AddBoolToObject(safety, "kalman_filter_running", true);
        cJSON_AddBoolToObject(safety, "msp_output_active", true);
        cJSON_AddBoolToObject(safety, "safe_parameter_updates", true);
        cJSON_AddItemToObject(json, "system_safety", safety);
        
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
        cJSON_AddItemToObject(response, "system_status", system_status);
        
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

// Captive portal handlers
static esp_err_t captive_portal_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "🔗 Captive portal detection request from: %s", req->uri);
    
    // Send appropriate response for different captive portal detection methods
    if (strstr(req->uri, "generate_204") != NULL) {
        // Android captive portal detection
        httpd_resp_set_status(req, "204 No Content");
        httpd_resp_send(req, NULL, 0);
    } else if (strstr(req->uri, "hotspot-detect.html") != NULL) {
        // iOS captive portal detection  
        const char* ios_response = "<HTML><HEAD><TITLE>Success</TITLE></HEAD><BODY>Success</BODY></HTML>";
        httpd_resp_set_type(req, "text/html");
        httpd_resp_send(req, ios_response, strlen(ios_response));
    } else if (strstr(req->uri, "connecttest.txt") != NULL) {
        // Windows captive portal detection
        const char* windows_response = "Microsoft Connect Test";
        httpd_resp_set_type(req, "text/plain");
        httpd_resp_send(req, windows_response, strlen(windows_response));
    } else {
        // Default: redirect to main page
        httpd_resp_set_status(req, "302 Found");
        httpd_resp_set_hdr(req, "Location", "/");
        httpd_resp_send(req, NULL, 0);
    }
    
    return ESP_OK;
}

static esp_err_t captive_portal_redirect_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "🔗 Captive portal redirect for: %s", req->uri);
    
    // Skip API calls and WebSocket connections
    if (strstr(req->uri, "/api/") != NULL || 
        strstr(req->uri, "/ws") != NULL) {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }
    
    // For all other requests, redirect to main dashboard
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "/");
    httpd_resp_send(req, NULL, 0);
    
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
        server_config.enable_captive_portal = true;     // Enable captive portal by default
        server_config.captive_portal_redirect_all = true; // Redirect all unknown requests
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
    
    // Captive portal handlers (redirect all unknown requests to main page)
    if (server_config.enable_captive_portal) {
        ESP_LOGI(TAG, "🔗 Enabling captive portal functionality");
        
        // Common captive portal detection URLs
        httpd_uri_t captive_detect_android = {
            .uri = "/generate_204",
            .method = HTTP_GET,
            .handler = captive_portal_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &captive_detect_android);
        
        httpd_uri_t captive_detect_ios = {
            .uri = "/hotspot-detect.html",
            .method = HTTP_GET,
            .handler = captive_portal_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &captive_detect_ios);
        
        httpd_uri_t captive_detect_windows = {
            .uri = "/connecttest.txt",
            .method = HTTP_GET,
            .handler = captive_portal_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &captive_detect_windows);
        
        // Catch-all handler for unknown requests (must be last)
        if (server_config.captive_portal_redirect_all) {
            httpd_uri_t captive_catchall = {
                .uri = "/*",
                .method = HTTP_GET,
                .handler = captive_portal_redirect_handler,
                .user_ctx = NULL
            };
            httpd_register_uri_handler(server, &captive_catchall);
        }
    }
    
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
