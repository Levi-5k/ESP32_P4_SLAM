# ESP32-C6 WiFi Management & Web Interface Documentation

## Overview

The ESP32-C6 slave device serves as the WiFi management and web interface hub for the ESP32-P4 Visual SLAM drone camera system. It provides comprehensive WiFi control, real-time monitoring, and configuration management through a modern web interface.

## Architecture

### Hardware Configuration
- **Device**: ESP32-C6 (slave)
- **Communication**: ESP-Hosted protocol with ESP32-P4 (master)
- **WiFi Modes**: Station (STA), Access Point (AP), Scan-only
- **Web Server**: HTTP server with WebSocket real-time updates

### Software Components
- **Web Server**: Real-time SLAM monitoring dashboard
- **WiFi Manager**: Advanced WiFi control with automatic fallback
- **Communication Manager**: ESP-Hosted message handling
- **Data Cache**: Real-time system status and telemetry storage

## WiFi Management System

### Features

#### 1. WiFi Toggle Control
- **Enable WiFi**: Full connection mode with network association
- **Scan-Only Mode**: Network scanning without connections (maintains situational awareness)
- **Disable WiFi**: Complete WiFi shutdown for power saving or security
- **Real-time Status**: Live WiFi state monitoring

#### 2. Automatic AP Fallback
- **Smart Fallback**: Automatically starts AP mode when no networks found
- **Connection Monitoring**: Continuous connection health checking
- **Retry Logic**: Configurable connection retry attempts
- **Emergency Mode**: AP fallback after maximum connection retries

#### 3. Network Scanning
- **Background Scanning**: Periodic network discovery in scan-only mode
- **Real-time Results**: Live network list updates via WebSocket
- **Signal Strength**: RSSI monitoring and display
- **Channel Information**: WiFi channel and security details

### API Endpoints

#### WiFi Control
```
POST /api/wifi/control
Content-Type: application/json

{
  "enable_wifi": true,           // Enable/disable WiFi
  "enable_scan_only": false,     // Scan-only mode flag
  "auto_ap_fallback": true       // AP fallback on connection failure
}
```

#### WiFi Status
```
GET /api/wifi/status

Response:
{
  "wifi_status": {
    "wifi_enabled": true,
    "connected": true,
    "scan_active": false,
    "ap_mode_active": false,
    "ssid": "MyNetwork",
    "rssi": -45,
    "ip_address": "192.168.1.100"
  },
  "status": "success"
}
```

#### Network Scanning
```
POST /api/wifi/scan

Response:
{
  "networks": [
    {
      "ssid": "Network_Name",
      "rssi": -45,
      "channel": 6,
      "auth_mode": 3
    }
  ],
  "status": "success"
}
```

#### WiFi Connection
```
POST /api/wifi/connect
Content-Type: application/json

{
  "ssid": "NetworkName",
  "password": "password123",
  "ap_fallback": true
}
```

## Web Interface

### Dashboard Features

#### 1. System Status
- **P4 Connection**: Real-time connection status with ESP32-P4
- **System Uptime**: Device operation time
- **Memory Usage**: RAM utilization monitoring
- **SLAM Status**: Visual SLAM system state

#### 2. WiFi Management
- **WiFi Controls**: Enable/disable, scan-only mode toggle
- **Connection Status**: Current WiFi state and signal strength
- **Network List**: Available networks with signal strength
- **AP Mode Status**: Access point operation status

#### 3. SLAM Monitoring
- **Real-time Position**: 3D position visualization with trail
- **Feature Tracking**: Active features and map points
- **Quality Metrics**: Tracking quality and system health
- **Camera Preview**: Live camera feed (optional)

#### 4. Configuration Management
- **SLAM Parameters**: Feature detection and tracking settings
- **Camera Settings**: Exposure, brightness, contrast controls
- **Map Management**: Save, load, and clear maps
- **System Commands**: Calibration and system control

### WebSocket Real-time Updates

The web interface uses WebSocket connections for real-time data streaming:

```javascript
// WebSocket connection for live updates
const ws = new WebSocket('ws://192.168.4.1/ws');

ws.onmessage = function(event) {
    const data = JSON.parse(event.data);
    updateDashboard(data);
};

// Periodic WiFi status checks
setInterval(checkWiFiStatus, 5000);
```

## Communication Protocol

### Message Types (C6 → P4)
- `MSG_C6_TO_P4_WIFI_CONTROL` (0x18): WiFi enable/disable commands
- `MSG_C6_TO_P4_WIFI_SCAN_REQ` (0x11): Network scan requests
- `MSG_C6_TO_P4_CONNECT_WIFI` (0x12): Connection commands
- `MSG_C6_TO_P4_SYSTEM_CMD` (0x13): System control commands

### Message Types (P4 → C6)
- `MSG_P4_TO_C6_WIFI_STATUS` (0x09): WiFi status updates
- `MSG_P4_TO_C6_WIFI_NETWORKS` (0x0A): Scan results
- `MSG_P4_TO_C6_SLAM_STATUS` (0x02): SLAM system status
- `MSG_P4_TO_C6_TELEMETRY` (0x04): Sensor data

### WiFi Control Message Structure
```c
typedef struct __attribute__((packed)) {
    uint8_t enable_wifi;          // 1 = enable, 0 = disable
    uint8_t enable_scan_only;     // 1 = scan-only mode
    uint8_t auto_ap_fallback;     // 1 = enable AP fallback
    uint8_t reserved;             // Reserved for future use
} wifi_control_msg_t;
```

### WiFi Status Message Structure
```c
typedef struct __attribute__((packed)) {
    uint8_t wifi_enabled;         // WiFi module state
    uint8_t connected;            // Connection status
    uint8_t scan_active;          // Scanning state
    uint8_t ap_mode_active;       // AP mode status
    uint8_t ssid[32];            // Connected SSID
    int8_t rssi;                 // Signal strength
    uint32_t ip_address;         // IP address
    uint8_t mac_address[6];      // MAC address
} wifi_status_msg_t;
```

## Operational Modes

### 1. Station Mode (Default)
- Connects to existing WiFi networks
- Provides internet connectivity for drone operations
- Enables remote monitoring and control
- Automatic reconnection on connection loss

### 2. Access Point Mode (Fallback)
- **SSID**: `ESP32_P4_SLAM_AP` (configurable)
- **IP Range**: 192.168.4.1/24
- **Security**: WPA2-PSK
- **Max Clients**: 4 concurrent connections
- **Auto-activation**: When no networks available or connection fails

### 3. Scan-Only Mode
- Network discovery without connections
- Maintains situational awareness
- Lower power consumption
- Security scanning capability
- Background operation while P4 processes SLAM

## Configuration

### WiFi Manager Configuration
```c
wifi_manager_config_t config = {
    .sta_ssid = "YourNetwork",
    .sta_password = "YourPassword",
    .sta_connect_timeout_ms = 15000,
    .ap_ssid = "ESP32_P4_SLAM_AP",
    .ap_password = "slam123456",
    .ap_channel = 6,
    .ap_max_connections = 4,
    .auto_fallback = true,
    .scan_only_mode = false,
    .scan_interval_ms = 10000,
    .max_retry_count = 3
};
```

### Web Server Configuration
```c
web_server_config_t server_config = {
    .port = 80,
    .max_clients = 4,
    .enable_cors = true,
    .stack_size = 8192
};
```

## System Commands

### WiFi Commands
- `SYS_CMD_WIFI_ENABLE` (0x06): Enable WiFi
- `SYS_CMD_WIFI_DISABLE` (0x07): Disable WiFi
- `SYS_CMD_WIFI_GET_STATUS` (0x08): Request status

### System Control
- `SYS_CMD_RESTART` (0x01): System restart
- `SYS_CMD_CALIBRATE_IMU` (0x02): IMU calibration
- `SYS_CMD_CALIBRATE_CAMERA` (0x03): Camera calibration
- `SYS_CMD_RESET_SLAM` (0x04): SLAM system reset
- `SYS_CMD_EMERGENCY_STOP` (0x05): Emergency stop

## Security Features

### Network Security
- WPA2-PSK encryption for AP mode
- Secure password requirements
- Connection timeout limits
- Automatic disconnection detection

### Web Interface Security
- CORS headers for secure cross-origin requests
- Input validation for all API endpoints
- JSON parsing with error handling
- Resource limits to prevent DoS attacks

## Troubleshooting

### Common Issues

#### 1. WiFi Connection Failures
**Symptoms**: Cannot connect to networks, frequent disconnections
**Solutions**:
- Check network credentials
- Verify signal strength (RSSI > -70 dBm recommended)
- Enable AP fallback mode
- Restart WiFi manager

#### 2. Web Interface Not Accessible
**Symptoms**: Cannot access dashboard
**Solutions**:
- Check WiFi connection status
- Verify IP address (192.168.4.1 in AP mode)
- Clear browser cache
- Check server running status

#### 3. P4 Communication Issues
**Symptoms**: No SLAM data, outdated status
**Solutions**:
- Check ESP-Hosted connection
- Verify communication protocol versions
- Restart both devices
- Check physical connections

### Debug Commands

#### WiFi Status Check
```bash
# Via web interface
curl http://192.168.4.1/api/wifi/status

# Response indicates current state
```

#### Enable Debug Logging
```c
// In main configuration
esp_log_level_set("WIFI_MANAGER", ESP_LOG_DEBUG);
esp_log_level_set("SLAM_WEB", ESP_LOG_DEBUG);
```

### Performance Monitoring

#### Memory Usage
- **Heap Usage**: Monitor via dashboard
- **Stack Usage**: Check task watermarks
- **Buffer Usage**: WebSocket and HTTP buffers

#### Network Performance
- **Connection Time**: Monitor association delays
- **Scan Duration**: Track discovery performance
- **WebSocket Latency**: Real-time update delays

## Future Enhancements

### Planned Features
1. **Advanced Security**: WPA3 support, certificate management
2. **Mesh Networking**: ESP-MESH integration for extended range
3. **Cloud Integration**: IoT cloud connectivity options
4. **Mobile App**: Dedicated mobile application
5. **Advanced Analytics**: Historical data analysis

### API Extensions
- RESTful API for external integration
- MQTT support for IoT platforms
- Real-time video streaming protocols
- Advanced configuration management

## Support and Maintenance

### Log Analysis
- WiFi manager logs for connection debugging
- Web server access logs for usage analysis
- Communication protocol logs for message tracking

### Performance Tuning
- WiFi scan interval optimization
- WebSocket update frequency adjustment
- Memory usage optimization
- Power consumption management

### Version Information
- **Firmware Version**: Check via web interface
- **Protocol Version**: ESP-Hosted compatibility
- **API Version**: Web interface compatibility

---

*This documentation covers the comprehensive WiFi management and web interface capabilities of the ESP32-C6 device in the Visual SLAM drone camera system.*
