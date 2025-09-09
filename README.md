# ESP32-P4 Visual SLAM Navigation System

A real-time visual navigation system for autonomous drones using the ESP32-P4 microcontroller. This project combines computer vision, GPS, and IMU sensors to create a robust SLAM (Simultaneous Localization and Mapping) solution for drone navigation.

## 🚀 Current System Status - **NOT YET OPERATIONAL** ✅

## 🚀 Quick Start

### WiFi Access
- **Network Name**: `ESP32-AP`
- **Password**: `slam123456`
- **Web Interface**: `http://192.168.4.1`

### Hardware Connections (P4 ↔ C6)
```
ESP32-C6 → ESP32-P4
GPIO 23  → GPIO 17 (MOSI)
GPIO 22  → GPIO 16 (MISO)
GPIO 21  → GPIO 18 (CLK)
GPIO 24  → GPIO 19 (CS)
```


### ✨ Features

- **Real-time SLAM Processing** - 30 FPS camera processing with ORB feature detection
- **Multi-sensor Fusion** - Combines GPS, IMU, and camera data using Extended Kalman Filter
- **Persistent Storage** - SD card storage for maps and mission logs
- **Flight Controller Integration** - MSP protocol support for INAV-compatible controllers
- **Automatic Configuration** - Smart defaults with SD card override capability
- **Robust Error Handling** - Automatic recovery and sensible fallbacks

## 🏗️ Architecture

This project implements a dual-chip architecture:
- **ESP32-P4**: Main processing (SLAM, sensors, navigation)
- **ESP32-C6**: WiFi management and web interface

The system processes camera frames through ORB feature detection, fuses sensor data using an Extended Kalman Filter, and communicates navigation commands to flight controllers via MSP protocol. The P4 and C6 communicate via ESP-Hosted SPI protocol with automatic handshake and WiFi credential exchange.

```
Camera → Feature Detection → SLAM Processing → Navigation
   ↓           ↓                    ↓              ↓
 IMU → Sensor Fusion → State Estimation → Flight Control
   ↓           ↓                    ↓              ↓
 GPS → Position Updates → Map Building → MSP Protocol
   ↓           ↓                    ↓              ↓
WiFi ← ESP32-C6 ← ESP-Hosted ← P4 Communication
```

## 📋 Hardware Requirements

- **ESP32-P4-WIFI6** microcontroller (32MB flash, 32MB PSRAM) - Main processor
- **ESP32-C6** microcontroller - WiFi coprocessor
- **OV5647 MIPI-CSI** camera module
- **uBlox GPS** module
- **BMI088 IMU** sensor
- **SD Card** (FAT32 formatted)
- **INAV-compatible** flight controller (optional)

## 📁 Project Structure

### ESP32-P4 Master (Main Directory)
```
components/
├── slam_core/          # Main SLAM processing engine
├── orb_features/       # ORB feature detection
├── sensor_fusion/      # Extended Kalman Filter
├── sd_storage/         # SD card data persistence
├── gps_ublox/          # GPS module interface
├── imu_bmi088/         # IMU sensor driver
├── msp_protocol/       # Flight controller communication
└── config_loader/      # Configuration management
```

### ESP32-C6 Slave (slave/ Directory)
```
slave/
├── main/               # C6 main application
├── components/         # WiFi and web components
└── managed_components/ # ESP-Hosted framework
```

## ⚙️ Configuration

The system uses a dual configuration system:
- **SPIFFS** - Internal flash for factory defaults
- **SD Card** - External storage for user configurations

Configuration files are automatically validated and corrected if invalid values are detected.
```
## 📊 Performance

- **Processing**: 30 FPS at 800x640 resolution
- **Features**: Up to 500 ORB features per frame
- **Memory**: ~748KB for SLAM data structures
- **Storage**: SD card support for persistent maps and logs

## 🔧 Development

### Building ESP32-P4 (Main)
```bash
cd DroneCam
idf.py build
```

### Building ESP32-C6 (WiFi)
```bash
cd DroneCam/slave
idf.py build
```

### Flashing Both Devices
```bash
# Flash P4 (main processor)
cd DroneCam
idf.py flash -p <P4_PORT>

# Flash C6 (WiFi processor)
cd DroneCam/slave
idf.py flash -p <C6_PORT>
```

### Monitoring
```bash
# Monitor P4 (SLAM logs)
cd DroneCam
idf.py monitor -p <P4_PORT>

# Monitor C6 (WiFi logs)
cd DroneCam/slave
idf.py monitor -p <C6_PORT>
```


### Pin Configuration

#### ESP32-P4 Pin Configuration

##### Camera (OV5647) - MIPI-CSI Interface
- **I2C SDA**: GPIO_NUM_7 (Hardware fixed)
- **I2C SCL**: GPIO_NUM_8 (Hardware fixed)
- **MIPI-CSI**: Dedicated MIPI-CSI pins
- **Resolution**: 1920x1080 at 30 FPS

##### GPS (uBlox) - UART Interface
- **TX Pin**: GPIO_NUM_4 (ESP32-P4 → GPS RX)
- **RX Pin**: GPIO_NUM_5 (GPS TX → ESP32-P4)
- **Baud Rate**: 115600
- **Update Rate**: 10Hz

##### IMU (BMI088) - SPI Interface
- **MISO Pin**: GPIO_NUM_20
- **MOSI Pin**: GPIO_NUM_21
- **CLK Pin**: GPIO_NUM_22
- **ACC CS Pin**: GPIO_NUM_23 (Accelerometer Chip Select)
- **GYRO CS Pin**: GPIO_NUM_24 (Gyroscope Chip Select)
- **Sample Rate**: 400Hz

##### SD Card - SDMMC Interface
- **CLK Pin**: GPIO_NUM_43
- **CMD Pin**: GPIO_NUM_44
- **D0 Pin**: GPIO_NUM_39
- **D1 Pin**: GPIO_NUM_40
- **D2 Pin**: GPIO_NUM_41
- **D3 Pin**: GPIO_NUM_42

##### ESP-Hosted Communication (P4 → C6)
- **MOSI Pin**: GPIO_NUM_17 (P4 → C6 GPIO 23)
- **MISO Pin**: GPIO_NUM_16 (P4 ← C6 GPIO 22)
- **CLK Pin**: GPIO_NUM_18 (P4 → C6 GPIO 21)
- **CS Pin**: GPIO_NUM_19 (P4 → C6 GPIO 24)

#### ESP32-C6 Pin Configuration

The ESP32-C6 operates as a WiFi coprocessor connected to the ESP32-P4 via ESP-Hosted SPI communication interface.

##### ESP-Hosted Communication Interface (C6 ← P4 Communication)
- **MOSI Pin**: GPIO_NUM_23 (C6 ← P4 GPIO 17)
- **MISO Pin**: GPIO_NUM_22 (C6 → P4 GPIO 16)
- **CLK Pin**: GPIO_NUM_21 (C6 ← P4 GPIO 18)
- **CS Pin**: GPIO_NUM_24 (C6 ← P4 GPIO 19)

##### WiFi Hardware Interface
- **Antenna**: Built-in PCB antenna or external connector
- **WiFi Standard**: 802.11ax (WiFi 6) - 2.4 GHz
- **Operating Modes**: AP mode, STA mode, concurrent AP+STA
- **Web Server**: HTTP server on port 80 for SLAM control
- **Communication**: WebSocket support for real-time streaming

##### Communication Protocol
- **Transport**: ESP-Hosted high-speed SPI communication protocol
- **Handshake**: Automatic P4↔C6 handshake on startup
- **WiFi Credentials**: P4 provides WiFi credentials to C6 via handshake
- **AP Fallback**: Automatic AP mode (`ESP32-AP`) if WiFi connection fails
- **Timeout**: 2-minute AP timeout switches to WiFi scanning mode
- **Data Exchange**: WiFi control commands, network scan results, web interface data


## 🐛 Troubleshooting

### Common Issues

1. **Build Errors**: Ensure ESP-IDF v5.5 is properly installed
2. **Flash Failures**: Check serial port and USB connections
3. **Sensor Issues**: Verify pin connections and sensor power
4. **Memory Errors**: Monitor heap usage and PSRAM allocation

### Debug Logging
Enable detailed logging for troubleshooting:
```c
esp_log_level_set("slam_core", ESP_LOG_DEBUG);
esp_log_level_set("sensor_fusion", ESP_LOG_DEBUG);
```

## 🤝 Contributing

We welcome contributions! Please:
- Keep code comments minimal and focused
- Add comprehensive error handling
- Follow established component patterns
- Test configuration changes thoroughly
- Update documentation for new features

## 📄 License

This project is open source. Please check the license file for details.

---

**Built with ESP-IDF v5.5** | **Last updated: September 2025**
