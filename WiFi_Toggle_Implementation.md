# ESP32-P4 WiFi Toggle Implementation Guide

## Quick Reference: WiFi Toggle Functionality

### Overview
The ESP32-P4 can now toggle WiFi on/off while maintaining network scan capability, with automatic AP fallback on the C6 when no networks are found.

## Key Features Implemented

### 1. WiFi Toggle Commands (P4)
```c
// New system commands added
SYS_CMD_WIFI_ENABLE = 0x06,     // Enable WiFi
SYS_CMD_WIFI_DISABLE = 0x07,    // Disable WiFi  
SYS_CMD_WIFI_GET_STATUS = 0x08  // Get WiFi status
```

### 2. WiFi Control Modes
- **Full Mode**: WiFi enabled with connections
- **Scan-Only Mode**: WiFi scans networks but doesn't connect
- **Disabled Mode**: WiFi completely off

### 3. Enhanced AP Fallback (C6)
- Automatically starts AP mode when no networks found
- Configurable fallback behavior
- Client connection monitoring
- Retry logic with fallback safety

## Implementation Details

### P4-Side WiFi Manager Enhancements

#### New Functions Added
```c
esp_err_t wifi_manager_enable(bool scan_only);
esp_err_t wifi_manager_disable(void);
esp_err_t wifi_manager_enable_scan_only(void);
bool wifi_manager_is_scan_only(void);
```

#### Configuration Structure Extended
```c
typedef struct {
    // ... existing fields ...
    bool scan_only_mode;               // Enable scan-only mode
    // ... other fields ...
} wifi_manager_config_t;
```

### Communication Protocol Updates

#### New Message Types
```c
// P4 to C6
MSG_P4_TO_C6_WIFI_STATUS = 0x08,      // WiFi status updates
MSG_P4_TO_C6_WIFI_NETWORKS = 0x09,    // Network scan results

// C6 to P4  
MSG_C6_TO_P4_WIFI_CONTROL = 0x16,     // WiFi control commands
```

#### WiFi Control Message Structure
```c
typedef struct __attribute__((packed)) {
    uint8_t enable_wifi;               // 1 = enable, 0 = disable
    uint8_t enable_scan_only;          // 1 = scan-only mode  
    uint8_t auto_ap_fallback;          // 1 = enable AP fallback
    uint8_t reserved;                  // Reserved
} wifi_control_msg_t;
```

#### WiFi Status Message Structure
```c
typedef struct __attribute__((packed)) {
    uint8_t wifi_enabled;              // WiFi module state
    uint8_t connected;                 // Connection status
    uint8_t scan_active;               // Scanning state
    uint8_t ap_mode_active;            // AP mode status
    uint8_t ssid[32];                  // Connected SSID
    int8_t rssi;                       // Signal strength
    uint32_t ip_address;               // IP address
    uint8_t mac_address[6];            // MAC address
} wifi_status_msg_t;
```

### C6 Web Interface Updates

#### New WiFi Control Panel
```html
<div class='card'><h3>WiFi Control</h3>
  <div class='status-grid'>
    <div class='status-item'>WiFi Status: <span id='wifi-status'>--</span></div>
    <div class='status-item'>Mode: <span id='wifi-mode'>--</span></div>
    <div class='status-item'>Connected: <span id='wifi-connected'>--</span></div>
    <div class='status-item'>AP Mode: <span id='wifi-ap-mode'>--</span></div>
  </div>
  <div>
    <button onclick='toggleWiFi(true, false)'>📡 Enable WiFi</button>
    <button onclick='toggleWiFi(true, true)'>🔍 Scan Only Mode</button>
    <button onclick='toggleWiFi(false, false)'>🚫 Disable WiFi</button>
  </div>
</div>
```

#### New API Endpoints
```javascript
// WiFi control
POST /api/wifi/control
{
  "enable_wifi": true,
  "enable_scan_only": false, 
  "auto_ap_fallback": true
}

// WiFi status
GET /api/wifi/status
```

## Usage Examples

### 1. Enable WiFi in Full Mode
```c
// From web interface
toggleWiFi(true, false);

// From P4 code  
wifi_manager_enable(false);

// Via system command
system_cmd_msg_t cmd = {.command = SYS_CMD_WIFI_ENABLE};
```

### 2. Enable Scan-Only Mode
```c
// From web interface
toggleWiFi(true, true);

// From P4 code
wifi_manager_enable_scan_only();
```

### 3. Disable WiFi Completely
```c
// From web interface  
toggleWiFi(false, false);

// From P4 code
wifi_manager_disable();

// Via system command
system_cmd_msg_t cmd = {.command = SYS_CMD_WIFI_DISABLE};
```

### 4. Check WiFi Status
```c
// From P4 code
wifi_manager_status_t status = wifi_manager_get_status();
bool scan_only = wifi_manager_is_scan_only();

// From web interface
fetch('/api/wifi/status').then(r => r.json()).then(updateWiFiStatus);
```

## AP Fallback Configuration

### Enhanced WiFi Manager Behavior
```c
// In main WiFi manager task
case WIFI_MGR_STATUS_INITIALIZING:
    if (scan_only_mode) {
        status = WIFI_MGR_STATUS_SCAN_ONLY;
        break;
    }
    
    // Try station mode first
    if (wifi_try_connect_station() != ESP_OK) {
        // Enhanced AP fallback logic
        if (config.auto_fallback) {
            ESP_LOGI(TAG, "🔄 No networks found, starting AP mode");
            wifi_start_ap_fallback();
        }
    }
    break;
```

### C6 AP Mode Settings
```c
wifi_manager_config_t config = {
    .ap_ssid = "ESP32_P4_SLAM_AP",
    .ap_password = "slam123456", 
    .ap_channel = 6,
    .ap_max_connections = 4,
    .auto_fallback = true  // Enable automatic AP fallback
};
```

## State Transitions

```
[WiFi Disabled] 
    ↓ enable_wifi=true, scan_only=false
[WiFi Connecting] 
    ↓ connection_success
[WiFi Connected]
    ↓ connection_failed & auto_fallback=true  
[AP Mode Active]

[WiFi Disabled]
    ↓ enable_wifi=true, scan_only=true
[Scan-Only Mode] ← scanning networks continuously

[Any State]
    ↓ enable_wifi=false
[WiFi Disabled]
```

## Testing Commands

### Web Interface Testing
```bash
# Enable WiFi full mode
curl -X POST http://192.168.4.1/api/wifi/control \
  -H "Content-Type: application/json" \
  -d '{"enable_wifi":true,"enable_scan_only":false,"auto_ap_fallback":true}'

# Enable scan-only mode  
curl -X POST http://192.168.4.1/api/wifi/control \
  -H "Content-Type: application/json" \
  -d '{"enable_wifi":true,"enable_scan_only":true,"auto_ap_fallback":true}'

# Disable WiFi
curl -X POST http://192.168.4.1/api/wifi/control \
  -H "Content-Type: application/json" \
  -d '{"enable_wifi":false,"enable_scan_only":false,"auto_ap_fallback":false}'

# Check status
curl http://192.168.4.1/api/wifi/status
```

## Configuration Files Updated

### Files Modified
1. `communication_protocol.h` - Added WiFi control messages
2. `wifi_manager.h` - Added new WiFi control functions  
3. `wifi_manager.c` - Implemented WiFi toggle and scan-only mode
4. `communication_manager.c` - Added WiFi control message handlers
5. `slam_web_server.c` - Added WiFi control web interface
6. `slam_web_server.h` - Extended data cache structure

### Build Dependencies
- ESP-IDF WiFi Remote components
- ESP-Hosted protocol support
- cJSON library for web API
- FreeRTOS task management

## Benefits

### Operational Advantages
1. **Power Management**: WiFi can be disabled when not needed
2. **Security**: Scan-only mode for reconnaissance without connections  
3. **Reliability**: Automatic AP fallback ensures connectivity
4. **Flexibility**: Real-time WiFi control via web interface
5. **Monitoring**: Continuous network awareness even when disconnected

### Use Cases
1. **Field Operations**: Scan for networks without connecting
2. **Power Saving**: Disable WiFi during autonomous flight
3. **Connectivity Assurance**: AP fallback for emergency access
4. **Network Assessment**: Monitor available networks continuously
5. **Remote Control**: Web-based WiFi management

---

*This implementation provides comprehensive WiFi control for the ESP32-P4 Visual SLAM system with enhanced reliability and user control.*
