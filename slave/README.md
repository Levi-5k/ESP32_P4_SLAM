# ESP32-C6 WiFi Coprocessor

This document details the **ESP32-C6 WiFi Coprocessor Implementation** for the Visual SLAM Navigation System, providing Wi-Fi connectivity and web interface services to the ESP32-P4 main processor via ESP-Hosted communication protocol.

## Overview

The ESP32-C6 coprocessor enables the ESP32-P4 main processor to utilize advanced Wi-Fi capabilities through **ESP-Hosted communication interface**. This implementation provides comprehensive WiFi management, web-based SLAM system control, and real-time data streaming capabilities optimized for drone navigation applications.

## Supported Hardware Configuration

The Visual SLAM system utilizes a dual-chip architecture with ESP32-P4 as the main processor and ESP32-C6 as the WiFi coprocessor. This implementation specifically uses **ESP-Hosted communication interface** as the transport between the two processors.

| **Hardware Configuration** | **Status** |
|---|:---:|
| ESP32-P4 (Main Processor) | ✓ Active |
| ESP32-C6 (WiFi Coprocessor) | ✓ Active |
| ESP-Hosted Communication | ✓ Implemented |
| Visual SLAM Processing | ✓ P4 Implementation |
| WiFi 6 (802.11ax) | ✓ C6 Implementation |
| Web Interface | ✓ C6 Implementation |
| Real-time Communication | ✓ Bidirectional |


## Hardware Connections

The Visual SLAM system uses ESP-Hosted communication interface for high-speed data exchange between ESP32-P4 (main processor) and ESP32-C6 (WiFi coprocessor).

### ESP32-C6 Pin Configuration

#### Communication Interface (Default for ESP32-P4-Function-EV-Board)

The ESP32-C6 operates in slave mode, providing WiFi and communication services to the ESP32-P4 host:

| Signal | GPIO | Function | Connection |
|:-------|:-----|:---------|:-----------|
| CLK | 19 | Communication Clock | Connects to P4 host CLK |
| CMD | 18 | Command Interface | Connects to P4 host CMD |
| D0 | 20 | Data Line 0 | Connects to P4 host D0 |
| D1 | 21 | Data Line 1 | Connects to P4 host D1 |
| D2 | 22 | Data Line 2 | Connects to P4 host D2 |
| D3 | 23 | Data Line 3 | Connects to P4 host D3 |
| Reset | EN | Reset input | Controlled by P4 for reset management |

#### WiFi Hardware Configuration

**Antenna Interface**
- **Type**: Built-in PCB antenna or external antenna connector
- **Frequency**: 2.4 GHz band (WiFi 6 / 802.11ax)
- **Modes**: Station (STA), Access Point (AP), concurrent AP+STA
- **Range**: Up to 100 meters (depending on environment and antenna)

**WiFi Capabilities**
- **Standards**: 802.11 b/g/n/ax (WiFi 6)
- **Security**: WPA2, WPA3, WEP, Open
- **AP Mode**: Supports up to 10 concurrent connections
- **Power**: Configurable power management modes

#### Web Server & Communication

**HTTP Server Configuration**
- **Port**: 80 (HTTP)
- **Features**: SLAM system control interface
- **WebSocket**: Real-time data streaming support
- **API Endpoints**: WiFi control, system status, network management

**ESP-Hosted Communication Protocol**
- **Transport**: High-speed communication interface
- **Data Types**: WiFi control commands, network scan results, web data
- **Special Features**: WiFi toggle with scan capability retention
- **Fallback**: Automatic AP mode activation if no network found

#### Power and Reset Management

**Power Supply**
- **Voltage**: 3.3V (same as ESP32-P4)
- **Current**: Variable based on WiFi activity (50-200mA typical)
- **Shared Ground**: Connected to P4 ground plane

**Reset Control**
- **Reset Pin**: EN pin controlled by ESP32-P4
- **Boot Mode**: Automatic based on ESP-Hosted protocol
- **Recovery**: P4 can reset C6 if communication fails

For detailed hardware connection requirements, refer to the official ESP-Hosted documentation. The GPIO pins and communication interface are optimized for the Visual SLAM navigation system.

### System Integration Architecture

This project implements a sophisticated dual-chip architecture for real-time Visual SLAM navigation:

#### ESP32-P4 (Host/Main Processor)
- **Role**: Main SLAM processing, sensor fusion, data storage
- **Sensors**: OV5647 camera, BMI088 IMU, uBlox GPS
- **Storage**: SD card for map persistence and logging
- **Communication**: Communication host for C6 interface

#### ESP32-C6 (Slave/WiFi Coprocessor)
- **Role**: WiFi management, web server, network operations
- **Features**: WiFi 6 support, dual-mode operation (AP/STA)
- **Services**: HTTP server, WebSocket streaming, network scanning
- **Communication**: Communication slave responding to P4 commands

#### Inter-Chip Communication Protocol
- **Transport**: High-speed communication interface (up to 50 MHz)
- **Commands**: WiFi control, network management, web interface data
- **Special Features**: 
  - WiFi toggle while maintaining scan capability
  - Automatic AP fallback if no networks available
  - Real-time status synchronization
  - Web-based SLAM system control


## Deployment Guide

### 1. Firmware Setup

The C6 firmware is integrated into the main Visual SLAM project. Navigate to the project directory:

```bash
cd d:\ESP-IDF-projects\DroneCam\slave
```

### 2. Set Up ESP-IDF

It is presumed that ESP-IDF has already been set up. If not, please proceed with the setup using one of the following options:

#### Option 1: Installer Way

  * **Windows**

      * Install and set up ESP-IDF on Windows as documented in the [Standard Setup of Toolchain for Windows](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/windows-setup.html).
      * Use the ESP-IDF [Powershell Command Prompt](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/windows-setup.html#using-the-command-prompt) for subsequent commands.

  * **Linux or macOS**

      * For bash:
        ```bash
        bash docs/setup_esp_idf__latest_stable__linux_macos.sh
        ```
      * For fish:
        ```fish
        fish docs/setup_esp_idf__latest_stable__linux_macos.fish
        ```

#### Option 2: Manual Way

Please follow the [ESP-IDF Get Started Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html) for manual installation.

### 3. Configure Target Hardware

Set the target to ESP32-C6 for the WiFi coprocessor:

```bash
idf.py set-target esp32c6
```

> [!NOTE]
> The ESP32-C6 target is pre-configured for this Visual SLAM navigation system.

### 4. System Configuration

The Visual SLAM system configuration is pre-optimized. You can access advanced settings using:

```bash
idf.py menuconfig
```

The configuration structure for the Visual SLAM system:
```
Visual SLAM Configuration
└── ESP32-C6 WiFi Coprocessor
    └── ESP-Hosted Communication Interface
        └── WiFi Management & Web Interface
    └── <Additional system features>
```


> [!TIP]
> Advanced users can customize the communication interface and WiFi features within this menu.

### 5. Build and Deploy

Build and flash the C6 firmware to your device using the commands below, replacing `<SERIAL_PORT>` with your device's serial port:

```bash
idf.py build
idf.py -p <SERIAL_PORT> flash monitor
```

> [!TIP]
> Use the same serial port configuration as your ESP32-P4 main processor for consistency.


## References

- [Visual SLAM Main System Documentation](../README.md)
- [ESP32-P4-Function-EV-Board Setup](../../docs/esp32_p4_function_ev_board.md)
- [ESP-Hosted Communication Protocol](../../docs/)
- [WiFi Toggle Implementation Guide](../WiFi_Toggle_Implementation.md)
- [ESP32-C6 System Documentation](../ESP32-C6_Documentation.md)
- [Troubleshooting Guide](../../docs/troubleshooting.md)
