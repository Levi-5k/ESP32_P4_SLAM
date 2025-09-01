#!/bin/bash

# Configuration File Deployment Script
# Copies JSON configuration files to SPIFFS and SD card directories

set -e

echo "ESP32-P4 Visual SLAM Configuration Deployment Script"
echo "=================================================="

# Configuration source directory
CONFIG_SRC_DIR="config"
SPIFFS_DIR="spiffs_image"
SD_DIR="sdcard_image"

# Create SPIFFS directory structure
echo "Creating SPIFFS directory structure..."
mkdir -p "$SPIFFS_DIR/config"

# Create SD card directory structure
echo "Creating SD card directory structure..."
mkdir -p "$SD_DIR/config"
mkdir -p "$SD_DIR/maps"
mkdir -p "$SD_DIR/sessions"
mkdir -p "$SD_DIR/logs"

# Copy configuration files to SPIFFS
echo "Copying configuration files to SPIFFS..."
cp "$CONFIG_SRC_DIR/system_config.json" "$SPIFFS_DIR/config/"
cp "$CONFIG_SRC_DIR/gps_config.json" "$SPIFFS_DIR/config/"
cp "$CONFIG_SRC_DIR/msp_config.json" "$SPIFFS_DIR/config/"
cp "$CONFIG_SRC_DIR/slam_config.json" "$SPIFFS_DIR/config/"
cp "$CONFIG_SRC_DIR/fusion_config.json" "$SPIFFS_DIR/config/"
cp "$CONFIG_SRC_DIR/camera_config.json" "$SPIFFS_DIR/config/"
cp "$CONFIG_SRC_DIR/imu_config.json" "$SPIFFS_DIR/config/"

# Copy configuration files to SD card
echo "Copying configuration files to SD card..."
cp "$CONFIG_SRC_DIR/system_config.json" "$SD_DIR/config/"
cp "$CONFIG_SRC_DIR/gps_config.json" "$SD_DIR/config/"
cp "$CONFIG_SRC_DIR/msp_config.json" "$SD_DIR/config/"
cp "$CONFIG_SRC_DIR/slam_config.json" "$SD_DIR/config/"
cp "$CONFIG_SRC_DIR/fusion_config.json" "$SD_DIR/config/"
cp "$CONFIG_SRC_DIR/camera_config.json" "$SD_DIR/config/"
cp "$CONFIG_SRC_DIR/imu_config.json" "$SD_DIR/config/"

echo "Configuration files copied successfully!"
echo ""
echo "SPIFFS Contents:"
ls -la "$SPIFFS_DIR/config/"
echo ""
echo "SD Card Contents:"
ls -la "$SD_DIR/config/"
echo ""
echo "Next steps:"
echo "1. Build your ESP-IDF project: idf.py build"
echo "2. Create SPIFFS image: mkspiffs -c $SPIFFS_DIR -b 4096 -p 256 -s 0x0F0000 spiffs.bin"
echo "3. Flash SPIFFS: esptool.py --chip esp32p4 write_flash 0x110000 spiffs.bin"
echo "4. Copy SD card contents to your SD card root directory"
echo "5. Or use: idf.py spiffs (if configured in partitions.csv)"
