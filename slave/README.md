# ESP32-C6 WiFi Coprocessor

This document details the **ESP32-C6 WiFi Coprocessor Implementation** for the Visual SLAM Navigation System, providing Wi-Fi connectivity and web interface services to the ESP32-P4 main processor via ESP-Hosted communication protocol.

## Quick Start - WiFi Access

**AP Mode (Default Fallback):**
- **SSID**: `ESP32-AP`
- **Password**: `slam123456`
- **Web Interface**: `http://192.168.4.1`

**Handshake Protocol:**
The C6 automatically initiates handshake with P4 on startup to receive WiFi credentials. If connection fails, it falls back to AP mode with 2-minute timeout before switching to scanning mode.

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

The ESP32-C6 operates in slave mode, providing WiFi and communication services to the ESP32-P4 host via ESP-Hosted SPI protocol:

| Signal | GPIO | Function | Connection |
|:-------|:-----|:---------|:-----------|
| CLK | 21 | SPI Clock | Connects to P4 GPIO 18 (CLK) |
| MOSI | 23 | SPI Master Out | Connects to P4 GPIO 17 (MOSI) |
| MISO | 22 | SPI Master In | Connects to P4 GPIO 16 (MISO) |
| CS | 24 | Chip Select | Connects to P4 GPIO 19 (CS) |
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
- **Transport**: ESP-Hosted SPI interface at up to 10 MHz
- **Data Types**: WiFi control commands, network scan results, web data, handshake protocol
- **Message Types**: Handshake requests/acks, WiFi credential exchange, status updates
- **Special Features**: 
  - Automatic handshake on startup to receive WiFi credentials from P4
  - WiFi toggle with scan capability retention
  - AP fallback with 2-minute timeout
  - Real-time status synchronization

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
- **Transport**: ESP-Hosted SPI interface (up to 10 MHz)
- **Pin Configuration**: C6 GPIO 21-24 ↔ P4 GPIO 16-19 (CLK/MOSI/MISO/CS)
- **Commands**: WiFi control, network management, web interface data, handshake protocol
- **Message Flow**:
  1. C6 sends handshake request on startup
  2. P4 acknowledges handshake
  3. C6 requests WiFi credentials
  4. P4 sends WiFi SSID/password
  5. C6 attempts connection or falls back to AP mode
- **Special Features**: 
  - Automatic handshake-based WiFi credential exchange
  - AP fallback with `ESP32-AP` / `slam123456`
  - 2-minute AP timeout before switching to scanning mode
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
