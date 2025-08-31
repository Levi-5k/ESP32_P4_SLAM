# ESP32-P4 Visual SLAM Navigation System

A real-time Visual SLAM (Simultaneous Localization and Mapping) navigation system running on ESP32-P4 microcontroller with OV5647 MIPI-CSI camera. The system integrates GPS, IMU, and visual odometry for autonomous drone navigation.

## Features

- **Real-time Visual SLAM**: ORB feature detection and tracking at 30 FPS
- **Multi-sensor Fusion**: Extended Kalman Filter combining GPS, IMU, and camera data
- **SD Card Storage**: Persistent map storage and mission logging
- **INAV Integration**: MSP protocol for flight controller communication
- **Web Interface**: Real-time telemetry and system monitoring (WiFi disabled for SDMMC compatibility)

## Architecture

### Core Components

- **`slam_core/`**: Main SLAM processing engine with pose estimation and map management
- **`orb_features/`**: ORB feature detection and descriptor extraction from camera frames
- **`sensor_fusion/`**: Extended Kalman Filter combining GPS, IMU, and visual data
- **`sd_storage/`**: SD card storage for map persistence and mission logging
- **`gps_ublox/`**: uBlox GPS module interface
- **`imu_bmi088/`**: BMI088 IMU sensor driver
- **`msp_protocol/`**: Multiwii Serial Protocol for flight controller communication

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

## Hardware Requirements

- **ESP32-P4-WIFI6** microcontroller
- **OV5647 MIPI-CSI** camera module
- **uBlox GPS** module
- **BMI088 IMU** sensor
- **SD Card** for data storage
- **INAV-compatible** flight controller

## SD Card Data Formats

### Directory Structure

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

### Map File Format (slam_map.bin)

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
```

### Session Log Format (CSV)

```csv
timestamp_us,frame_count,tracked_features,confidence,pos_x,pos_y,pos_z,quat_w,quat_x,quat_y,quat_z
1693526400000000,1,45,0.85,1.234,-0.567,2.890,0.707,0.0,0.0,0.707
1693526400333333,2,47,0.87,1.245,-0.569,2.895,0.706,0.001,0.002,0.708
```

### Configuration Format (slam_config.json)

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

## Setup and Build

### Prerequisites

- ESP-IDF v5.5
- Python 3.11+
- Git

### Build Commands

```bash
# Clone repository
git clone <repository-url>
cd DroneCam

# Set up ESP-IDF environment
export IDF_PATH=/path/to/esp-idf-v5.5

# Configure for ESP32-P4
idf.py set-target esp32p4

# Build project
idf.py build

# Flash to device
idf.py flash -p COM9

# Monitor serial output
idf.py monitor -p COM9
```

### Configuration

The project uses multiple `sdkconfig.defaults.*` files for different ESP32 variants:

- `sdkconfig.defaults.esp32p4` - ESP32-P4 specific configuration
- `sdkconfig.defaults.esp32s3` - ESP32-S3 configuration
- etc.

## ESP32-P4 Specific Configuration

### SDMMC Pin Configuration

```c
// ESP32-P4 SDMMC pins
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

### PSRAM Configuration

```cmake
# ESP32-P4 PSRAM for camera frames
CONFIG_SPIRAM=y
CONFIG_SPIRAM_MODE_OCT=y
CONFIG_SPIRAM_SPEED_200M=y
CONFIG_SPIRAM_USE_MALLOC=y
```

## Development

### Component Structure

```
components/[component_name]/
├── CMakeLists.txt           # Component build configuration
├── include/
│   └── [component_name].h   # Public API
└── [component_name].c       # Implementation
```

### Key Development Patterns

- **Event Groups**: Inter-task synchronization using FreeRTOS event groups
- **Error Handling**: Always check ESP_ERR codes and log appropriately
- **Memory Management**: Use PSRAM for large allocations (camera frames, feature data)
- **Real-time Processing**: Maintain 30 FPS processing loop with timing constraints

### Common Pitfalls

1. **WiFi/SDMMC Conflict**: Cannot use both simultaneously on ESP32-P4
2. **GPIO Pin Conflicts**: GPS UART conflicts with SDMMC pins
3. **Memory Constraints**: Monitor heap usage, use PSRAM for large buffers
4. **Real-time Constraints**: Processing must complete within 33ms frame time

## Testing

### Unit Testing

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

## Contributing

1. Follow the established code style and patterns
2. Add appropriate error handling and logging
3. Test on ESP32-P4 hardware
4. Update documentation for any new features

## License

[Add license information here]

## Contact

[Add contact information here]