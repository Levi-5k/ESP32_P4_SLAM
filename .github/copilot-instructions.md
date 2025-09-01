# ESP32-P4 Visual SLAM Navigation System - AI Agent Instructions

## Important Guideline
keep comments to a minimum

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
// Camera I2C pins (ESP32-P4 MIPI-CSI hardware fixed pins)
#define CAMERA_I2C_SDA_IO           7       // I2C SDA pin for OV5647 camera
#define CAMERA_I2C_SCL_IO           8       // I2C SCL pin for OV5647 camera

// IMU SPI pins (configurable, moved to avoid camera conflict)
#define IMU_SPI_MISO_PIN           GPIO_NUM_20   // SPI MISO for IMU
#define IMU_SPI_MOSI_PIN           GPIO_NUM_21   // SPI MOSI for IMU  
#define IMU_SPI_CLK_PIN            GPIO_NUM_22   // SPI Clock for IMU
#define IMU_ACC_CS_PIN             GPIO_NUM_23   // Accelerometer Chip Select
#define IMU_GYRO_CS_PIN            GPIO_NUM_24   // Gyroscope Chip Select

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

// GPS UART pins (moved to avoid SDMMC conflicts)
#define GPS_UART_TX_PIN            GPIO_NUM_4    // GPS UART TX pin
#define GPS_UART_RX_PIN            GPIO_NUM_5    // GPS UART RX pin

// MSP UART pins for flight controller communication
#define MSP_UART_TX_PIN            GPIO_NUM_18   // MSP UART TX
#define MSP_UART_RX_PIN            GPIO_NUM_19   // MSP UART RX
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
idf.py flash -p "COM port"

# Monitor serial output
idf.py monitor -p "COM port"
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
├── maps/           # SLAM map files (binary format)
│   ├── slam_map.bin
│   ├── keyframes.bin
│   └── map_points.bin
├── sessions/       # Mission logs (CSV format)
│   ├── slam_YYYYMMDD_HHMMSS.csv
│   ├── gps_YYYYMMDD_HHMMSS.csv
│   ├── imu_YYYYMMDD_HHMMSS.csv
│   ├── features_YYYYMMDD_HHMMSS.csv
│   └── errors_YYYYMMDD_HHMMSS.csv
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

#### Keyframe Binary Format (keyframes.bin)
```c
typedef struct {
    uint32_t id;                    // Keyframe identifier
    uint64_t timestamp;             // Frame timestamp (microseconds)
    slam_pose_t pose;               // Camera pose
    uint32_t num_features;          // Number of features in this keyframe
    // Followed by feature data: orb_feature_point_t[num_features]
} keyframe_data_t;
```

#### Map Point Binary Format (map_points.bin)
```c
typedef struct {
    uint32_t id;                    // Map point identifier
    float position[3];              // 3D world position [x, y, z]
    uint8_t descriptor[32];         // ORB descriptor (256 bits)
    uint32_t observations;          // Number of keyframes observing this point
    float confidence;               // Point quality/confidence metric
} map_point_data_t;
```

#### Comprehensive CSV Logging System

**SLAM Data Log (slam_YYYYMMDD_HHMMSS.csv)**
```csv
timestamp_us,frame_count,tracked_features,confidence,pos_x,pos_y,pos_z,quat_w,quat_x,quat_y,quat_z
1693526400000000,1,45,0.85,1.234,-0.567,2.890,0.707,0.0,0.0,0.707
1693526400333333,2,47,0.87,1.245,-0.569,2.895,0.706,0.001,0.002,0.708
```

**GPS Data Log (gps_YYYYMMDD_HHMMSS.csv)**
```csv
timestamp_us,latitude,longitude,altitude,speed,course,fix_quality,satellites
1693526400000000,40.123456,-74.654321,45.6,0.0,0.0,3,8
1693526401000000,40.123457,-74.654322,45.7,0.5,15.2,3,8
```

**IMU Data Log (imu_YYYYMMDD_HHMMSS.csv)**
```csv
timestamp_us,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z,temperature
1693526400000000,0.05,-0.12,9.81,0.001,-0.002,0.003,25.4
1693526400002500,0.06,-0.11,9.80,0.002,-0.001,0.002,25.5
```

**Feature Data Log (features_YYYYMMDD_HHMMSS.csv)**
```csv
timestamp_us,frame_count,total_features,matches,match_quality,processing_time_ms
1693526400000000,1,45,0,0.0,15.2
1693526400333333,2,47,32,0.87,16.8
```

**Error/Event Log (errors_YYYYMMDD_HHMMSS.csv)**
```csv
timestamp_us,severity,component,error_code,message
1693526400000000,INFO,SLAM,0,System initialized successfully
1693526405000000,WARNING,GPS,1001,Signal weak - only 4 satellites
1693526410000000,ERROR,SD_CARD,2005,Write failed - disk full
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
- Use `sd_storage_save_slam_map()` for binary keyframe/map point persistence

**Configuration Setup:**
- Generate `slam_config.json` with optimized parameters
- Validate configuration against SLAM system requirements
- Include mission-specific parameter sets

**Comprehensive Logging Implementation:**
- **5-File CSV System**: SLAM poses, GPS coordinates, IMU data, feature tracking, error events
- **Real-time Logging**: Integrated into 30 FPS processing loop
- **Session Management**: Automatic timestamped session creation
- **Error Recovery**: Graceful handling of SD card write failures
- **Format Specifications**: Standardized CSV headers for data analysis

**Key Implemented Functions:**
```c
// Binary map data persistence
esp_err_t sd_storage_save_slam_map(const slam_keyframe_t* keyframes, uint32_t num_keyframes,
                                   const slam_map_point_t* map_points, uint32_t num_map_points);

// Comprehensive logging functions
esp_err_t sd_storage_log_slam_data(uint64_t timestamp, uint32_t frame_count, 
                                   uint32_t tracked_features, float confidence, 
                                   const slam_pose_t* pose);
esp_err_t sd_storage_log_gps_data(uint64_t timestamp, double latitude, double longitude, 
                                  float altitude, float speed, float course, 
                                  uint8_t fix_quality, uint8_t satellites);
esp_err_t sd_storage_log_imu_data(uint64_t timestamp, float acc_x, float acc_y, float acc_z,
                                  float gyro_x, float gyro_y, float gyro_z, float temperature);
esp_err_t sd_storage_log_feature_data(uint64_t timestamp, uint32_t frame_count, 
                                      uint32_t total_features, uint32_t matches, 
                                      float match_quality, float processing_time_ms);
esp_err_t sd_storage_log_error_event(uint64_t timestamp, const char* severity, 
                                     const char* component, uint32_t error_code, 
                                     const char* message);
```

**Data Integrity Tools:**
- Implement checksum validation for loaded data
- Create verification programs for SD card contents
- Build tools to repair corrupted map files
- Real-time error event logging for system monitoring

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
// Camera I2C pins (FIXED - Hardware defined by ESP32-P4 MIPI-CSI interface)
// These pins CANNOT be changed - they are dedicated camera interface pins
.camera_config = {
    .i2c_sda_pin = 7,        // Hardware fixed
    .i2c_scl_pin = 8         // Hardware fixed
}

// IMU SPI pins (CONFIGURABLE - Updated to avoid camera conflict)
// Previously used GPIO 8-12, moved to GPIO 20-24 to avoid camera I2C SCL conflict
.imu_config = {
    .miso_pin = 20,          // Changed from 8 (conflicted with camera SCL)
    .mosi_pin = 21,          // Changed from 9
    .sclk_pin = 22,          // Changed from 10
    .acc_cs_pin = 23,        // Changed from 11
    .gyro_cs_pin = 24        // Changed from 12
}

// GPS UART pins (moved to avoid conflicts)
.gps_config = {
    .tx_pin = GPIO_NUM_4,    // Changed from 15 (LED conflict)
    .rx_pin = GPIO_NUM_5     // Changed from 16 (LED conflict)
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
