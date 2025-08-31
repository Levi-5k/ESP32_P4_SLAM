# ESP32-P4 Visual SLAM Navigation System - AI Agent Instructions

## Project Overview
This is a real-time Visual SLAM (Simultaneous Localization and Mapping) navigation system running on ESP32-P4 microcontroller with OV5647 MIPI-CSI camera. The system integrates GPS, IMU, and visual odometry for autonomous drone navigation.

## Architecture Overview

### Core Components (`components/`)
- **`slam_core/`**: Main SLAM processing engine with pose estimation and map management
- **`orb_features/`**: ORB feature detection and descriptor extraction from camera frames
- **`sensor_fusion/`**: Extended Kalman Filter combining GPS, IMU, and visual data
- **`sd_storage/`**: SD card storage for map persistence and mission logging
- **`gps_ublox/`**: uBlox GPS module interface
- **`imu_bmi088/`**: BMI088 IMU sensor driver
- **`msp_protocol/`**: Multiwii Serial Protocol for flight controller communication (INAV integration)

### Data Flow
```
Camera Frame → ORB Features → SLAM Core → Pose Estimation
     ↓              ↓              ↓              ↓
   IMU Data → Sensor Fusion → State Estimation → Navigation Commands
     ↓              ↓              ↓              ↓
   GPS Data → Position Updates → Map Updates → MSP Protocol → INAV Flight Controller
     ↓              ↓              ↓              ↓
   SD Storage ← Mission Logging ← Telemetry ← Web Interface (WiFi disabled)
        ↑              ↑              ↑              ↑
   Map Loading ← Loop Closure ← Map Comparison ← Previous Missions
```

## Critical Patterns & Conventions

### 1. ESP32-P4 Hardware Configuration
```c
// SDMMC pins (ESP32-P4 specific)
static const sd_card_config_t default_sd_config = {
    .clk_pin = GPIO_NUM_43,     // SDMMC CLK
    .cmd_pin = GPIO_NUM_44,     // SDMMC CMD
    .d0_pin = GPIO_NUM_39,      // SDMMC D0
    .d1_pin = GPIO_NUM_40,      // SDMMC D1
    .d2_pin = GPIO_NUM_41,      // SDMMC D2
    .d3_pin = GPIO_NUM_42,      // SDMMC D3
    .max_freq_khz = 20000,      // 20MHz for 4-bit mode
    .format_if_mount_failed = false
};
```

### 2. Component Initialization Pattern
```c
// Always check return values and log appropriately
esp_err_t ret = component_init(&config);
if (ret == ESP_OK) {
    ESP_LOGI(TAG, "✅ Component initialized successfully");
    xEventGroupSetBits(system_event_group, COMPONENT_READY_BIT);
} else {
    ESP_LOGE(TAG, "❌ Component initialization failed: %s", esp_err_to_name(ret));
}
```

### 3. Memory Management
```c
// Use PSRAM for large allocations
#define MAX_ORB_FEATURES 1000
typedef struct {
    orb_feature_point_t* features;  // PSRAM allocation
    uint32_t num_features;
    uint32_t max_features;
} orb_features_t;
```

### 4. MSP Protocol Integration
```c
// MSP (Multiwii Serial Protocol) for INAV flight controller communication
// Currently placeholder - will send navigation commands to flight controller
esp_err_t msp_protocol_init(void);  // Initialize MSP communication
// TODO: Implement MSP message sending for:
// - Navigation waypoints
// - Position corrections
// - Altitude hold commands
// - Emergency stop signals
```

**Note**: MSP protocol is currently a placeholder component. The system architecture is designed to output navigation commands to INAV flight controller via MSP, but this integration is planned for future implementation.

## Development Workflow

### Build Commands
```bash
# Full build
idf.py build

# Clean build
idf.py clean && idf.py build

# Flash to device
idf.py flash -p COM9

# Monitor serial output
idf.py monitor -p COM9
```

### Configuration Files
- **`sdkconfig.defaults.esp32p4`**: ESP32-P4 specific settings (PSRAM, SD card)
- **`.vscode/settings.json`**: Development environment configuration
- **`CMakeLists.txt`**: Component dependencies and build configuration

### Testing Pattern
```c
// Use pytest for unit tests (limited on ESP32)
pytest_blink.py  # Basic GPIO test

// Integration testing via serial monitor
ESP_LOGI(TAG, "System status: frames=%lu, features=%lu",
         system_status.frame_count, system_status.feature_count);
```

## Key Integration Points

### 1. Component Communication
```c
// Use event groups for inter-task synchronization
EventGroupHandle_t system_event_group;
#define CAMERA_READY_BIT    BIT0
#define GPS_READY_BIT      BIT1
#define IMU_READY_BIT      BIT2
#define SLAM_READY_BIT     BIT3

// Wait for all systems ready
EventBits_t bits = xEventGroupWaitBits(system_event_group,
                                      ALL_SYSTEMS_READY,
                                      pdFALSE, pdTRUE,
                                      portMAX_DELAY);
```

### 2. Data Persistence
```c
// SD card file structure
#define SD_MOUNT_POINT "/sdcard"
#define SD_MAPS_DIR SD_MOUNT_POINT "/maps"
#define SD_SESSIONS_DIR SD_MOUNT_POINT "/sessions"
#define SD_CONFIG_DIR SD_MOUNT_POINT "/config"

// Always check SD card state before file operations
if (!g_sd_state.mounted) {
    return ESP_ERR_INVALID_STATE;
}
```

### 3. SD Card File Structure & Data Formats

#### Directory Structure
```
/sdcard/
├── maps/           # SLAM map files
│   ├── slam_map.bin
│   ├── keyframes.bin
│   └── map_points.bin
├── sessions/       # Mission logs
│   └── session_YYYYMMDD_HHMMSS.csv
├── config/         # Configuration files
│   └── slam_config.json
└── logs/           # System logs
```

#### Map File Format (slam_map.bin)
```c
typedef struct {
    uint32_t magic;                 // Magic number: 0x534C414D ('SLAM')
    uint32_t version;               // File format version
    uint64_t timestamp;             // Creation timestamp (microseconds)
    uint32_t keyframe_count;        // Number of keyframes
    uint32_t map_point_count;       // Number of map points
    uint32_t checksum;              // Data checksum
    char mission_name[64];          // Mission identifier
    double origin_lat;              // GPS origin latitude (degrees)
    double origin_lon;              // GPS origin longitude (degrees)
    float origin_alt;               // GPS origin altitude (meters)
} map_file_header_t;

// Followed by binary keyframe data and map point data
```

#### Session Log Format (CSV)
```csv
timestamp_us,frame_count,tracked_features,confidence,pos_x,pos_y,pos_z,quat_w,quat_x,quat_y,quat_z
1693526400000000,1,45,0.85,1.234,-0.567,2.890,0.707,0.0,0.0,0.707
1693526400333333,2,47,0.87,1.245,-0.569,2.895,0.706,0.001,0.002,0.708
```

#### Configuration Format (slam_config.json)
```json
{
  "max_features": 500,
  "fast_threshold": 20.0,
  "levels": 8,
  "scale_factor": 1.2,
  "max_keypoints_per_level": 100,
  "use_harris_detector": false,
  "harris_k": 0.04,
  "descriptor_distance_threshold": 50,
  "match_threshold": 0.7,
  "min_tracked_features": 30,
  "keyframe_distance_threshold": 1.0,
  "keyframe_angle_threshold": 0.2,
  "max_keyframes": 50,
  "enable_loop_closure": true,
  "loop_closure_threshold": 0.8
}
```

### 4. SD Card Data Loading Programs

Use the file format specifications above to create programs that prepare SD cards with:

**Map Data Preparation:**
- Create `slam_map.bin` files with proper headers and binary data
- Generate keyframe and map point data structures
- Calculate and include checksums for data integrity

**Configuration Setup:**
- Generate `slam_config.json` with optimized parameters
- Validate configuration against SLAM system requirements
- Include mission-specific parameter sets

**Session Data Templates:**
- Create CSV templates for session logging
- Pre-populate with expected data formats
- Include timestamp and telemetry field definitions

**Data Integrity Tools:**
- Implement checksum validation for loaded data
- Create verification programs for SD card contents
- Build tools to repair corrupted map files

### 3. Real-time Processing
```c
// Maintain 30 FPS processing loop
const TickType_t frequency = pdMS_TO_TICKS(33); // ~30 FPS
vTaskDelayUntil(&last_wake_time, frequency);

// Process camera frame → features → SLAM → pose estimation
esp_err_t ret = slam_core_process_frame(&frame, &slam_pose);
```

## Common Pitfalls

### 1. WiFi/SDMMC Conflict
```c
// WiFi and SDMMC cannot coexist on ESP32-P4
// Either use WiFi (web interface) OR SDMMC (data storage)
// Never enable both simultaneously
```

### 2. Memory Constraints
```c
// Monitor memory usage - ESP32-P4 has limited RAM
ESP_LOGI(TAG, "Free RAM: %zu bytes", heap_caps_get_free_size(MALLOC_CAP_8BIT));

// Use PSRAM for large allocations
CONFIG_SPIRAM_USE_MALLOC=y
```

### 3. GPIO Pin Conflicts
```c
// GPS UART conflicts with SDMMC pins
.gps_config = {
    .tx_pin = GPIO_NUM_15,  // Changed from 43 (SDIO CLK conflict)
    .rx_pin = GPIO_NUM_16   // Changed from 44 (SDIO CMD conflict)
}
```

## File Organization Patterns

### Component Structure
```
components/[component_name]/
├── CMakeLists.txt           # Component build configuration
├── include/
│   └── [component_name].h   # Public API
└── [component_name].c       # Implementation
```

### Header Includes
```c
// Component-specific headers
#include "[component_name].h"

// ESP-IDF headers
#include "esp_log.h"
#include "driver/gpio.h"

// Shared types (always in components/common/include/)
#include "visual_slam_common_types.h"
```

## Debugging Commands

### Serial Monitor
```bash
idf.py monitor -p COM9
# Look for log patterns:
# ✅ Component initialized successfully
# ❌ Component initialization failed: ESP_ERR_*
# 🧠 SLAM system status updates
```

### Memory Debugging
```c
// Check memory status before/after operations
ESP_LOGI(TAG, "Internal RAM free: %zu bytes",
         heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
ESP_LOGI(TAG, "SPIRAM free: %zu bytes",
         heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
```

## Code Style Guidelines

### Naming Conventions
- **Functions**: `snake_case` (e.g., `slam_core_init()`, `sd_storage_load_slam_map()`)
- **Types**: `snake_case_t` (e.g., `slam_pose_t`, `orb_features_t`)
- **Constants**: `UPPER_SNAKE_CASE` (e.g., `MAX_ORB_FEATURES`, `SD_MOUNT_POINT`)
- **Files**: `snake_case.c/.h` (e.g., `slam_core.c`, `sensor_fusion.h`)

### Logging Pattern
```c
// Use emoji prefixes for different log levels
ESP_LOGI(TAG, "✅ Operation successful");
ESP_LOGW(TAG, "⚠️ Warning condition");
ESP_LOGE(TAG, "❌ Error occurred: %s", esp_err_to_name(ret));
ESP_LOGD(TAG, "🔍 Debug information");
```

### Error Return Pattern
```c
// Always return ESP_ERR codes, never crash
if (!valid_condition) {
    ESP_LOGE(TAG, "Invalid condition detected");
    return ESP_ERR_INVALID_ARG;
}
```

## Performance Considerations

### Real-time Constraints
- **30 FPS camera processing** requires optimized algorithms
- **ORB feature extraction** must complete within 33ms frame time
- **Sensor fusion** runs at IMU sample rate (400Hz)
- **SD card operations** are asynchronous to avoid blocking

### Memory Optimization
- Use PSRAM for frame buffers and feature storage
- Implement circular buffers for recent data
- Free memory immediately after use
- Monitor heap usage in long-running tasks

## Testing Strategy

### Unit Testing (Limited on ESP32)
```python
# pytest_blink.py - Basic GPIO functionality test
def test_gpio_output():
    gpio_set_level(GPIO_NUM_15, 1)
    assert gpio_get_level(GPIO_NUM_15) == 1
```

### Integration Testing
- Monitor serial logs for system status
- Check event group bits for component readiness
- Verify SD card file operations
- Test sensor data flow through fusion algorithm

### Performance Testing
- Measure frame processing time
- Monitor memory usage over time
- Check SD card write speeds
- Validate real-time constraints are met
