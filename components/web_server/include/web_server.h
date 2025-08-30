/*
 * Web Server Component
 * HTTP server with WebSocket support for real-time data
 * Supports both Station mode and Captive Portal AP mode
 */

#pragma once

#include "esp_err.h"
#include "esp_http_server.h"
#include "visual_slam_common_types.h"
// #include "../main/visual_slam_inav.h"  // Will create this header separately

#ifdef __cplusplus
extern "C" {
#endif

// Web server configuration
typedef struct {
    uint16_t server_port;           // HTTP server port (default 80)
    uint16_t max_open_sockets;      // Maximum WebSocket connections
    bool enable_cors;               // Enable CORS headers
    uint32_t websocket_timeout_ms;  // WebSocket ping timeout
    size_t max_resp_headers;        // Maximum response headers
    size_t max_resp_size;          // Maximum response size
    bool enable_captive_portal;     // Enable captive portal mode
    uint8_t max_clients;            // Maximum HTTP clients
    bool websocket_enabled;         // Enable WebSocket support
} web_server_config_t;

// Web server WiFi modes (renamed to avoid conflict with ESP-IDF)
typedef enum {
    WEB_WIFI_MODE_STATION,              // Connect to existing WiFi
    WEB_WIFI_MODE_CAPTIVE_PORTAL        // Create AP with captive portal
} web_wifi_mode_t;

// Web server WiFi configuration (renamed to avoid conflict with ESP-IDF) 
typedef struct {
    char ssid[32];                  // WiFi SSID
    char password[64];              // WiFi password
    char ap_ssid[32];               // AP mode SSID
    char ap_password[64];           // AP mode password
    uint8_t max_connections;        // Max AP connections
    web_wifi_mode_t mode;           // Current WiFi mode
    bool auto_fallback;             // Fallback to AP if STA fails
    uint32_t connection_timeout_ms; // STA connection timeout
} web_wifi_config_t;

// Real-time data packet for WebSocket
typedef struct {
    // Navigation data
    navigation_state_t nav_state;
    
    // System status
    system_status_t system_status;
    
    // Sensor data
    gps_position_t gps_data;
    imu_data_t imu_data;
    slam_pose_t slam_pose;
    
    // Camera info
    uint32_t frame_count;
    float fps;
    
    // Timestamps
    uint64_t timestamp_us;
    uint32_t sequence_number;
} realtime_data_t;

// Web interface settings
typedef struct {
    // SLAM settings
    slam_config_t slam_config;
    
    // Sensor fusion settings
    fusion_config_t fusion_config;
    
    // ORB feature settings
    orb_config_t orb_config;
    
    // System settings
    bool enable_logging;
    uint8_t log_level;
    bool enable_telemetry;
    uint32_t update_rate_hz;
    
    // Calibration data
    bool camera_calibrated;
    bool imu_calibrated;
    bool gps_origin_set;
    
} web_settings_t;

// Initialize web server
esp_err_t web_server_init(const web_server_config_t* config);
esp_err_t web_server_deinit(void);

// WiFi management
esp_err_t web_server_wifi_init(const web_wifi_config_t* config);
esp_err_t web_server_wifi_start(void);
esp_err_t web_server_wifi_stop(void);
esp_err_t web_server_wifi_set_mode(web_wifi_mode_t mode);
web_wifi_mode_t web_server_wifi_get_mode(void);

// Server control
esp_err_t web_server_start(void);
esp_err_t web_server_stop(void);
bool web_server_is_running(void);

// Real-time data streaming
esp_err_t web_server_broadcast_data(const realtime_data_t* data);
esp_err_t web_server_set_camera_stream(bool enable);

// Data update functions (compatibility aliases)
esp_err_t web_server_update_slam_data(const slam_result_t* data);
esp_err_t web_server_update_system_status(const system_status_t* status);
esp_err_t web_server_broadcast_telemetry(const system_status_t* status);

// Settings management
esp_err_t web_server_get_settings(web_settings_t* settings);
esp_err_t web_server_set_settings(const web_settings_t* settings);
esp_err_t web_server_save_settings_to_nvs(void);
esp_err_t web_server_load_settings_from_nvs(void);

// Status and diagnostics
typedef struct {
    bool server_running;
    uint32_t active_websockets;
    uint32_t total_requests;
    uint32_t websocket_messages_sent;
    uint64_t bytes_sent;
    uint64_t bytes_received;
    web_wifi_mode_t current_wifi_mode;
    bool wifi_connected;
    char ip_address[16];
    int8_t rssi;
    uint32_t uptime_seconds;
} web_server_status_t;

esp_err_t web_server_get_status(web_server_status_t* status);

// Event callbacks
typedef enum {
    WEB_SERVER_EVENT_STARTED,
    WEB_SERVER_EVENT_STOPPED,
    WEB_SERVER_EVENT_CLIENT_CONNECTED,
    WEB_SERVER_EVENT_CLIENT_DISCONNECTED,
    WEB_SERVER_EVENT_SETTINGS_CHANGED,
    WEB_SERVER_EVENT_WIFI_CONNECTED,
    WEB_SERVER_EVENT_WIFI_DISCONNECTED,
    WEB_SERVER_EVENT_AP_MODE_STARTED,
    WEB_SERVER_EVENT_CAPTIVE_PORTAL_ACCESS
} web_server_event_t;

typedef void (*web_server_event_callback_t)(web_server_event_t event, void* data);
esp_err_t web_server_register_event_callback(web_server_event_callback_t callback);

// Captive portal specific functions
esp_err_t web_server_setup_captive_portal(void);
esp_err_t web_server_redirect_to_portal(httpd_req_t* req);

#ifdef __cplusplus
}
#endif
