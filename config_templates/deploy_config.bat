@echo off
REM Configuration File Deployment Script for Windows
REM Copies JSON configuration files to SPIFFS and SD card directories

echo ESP32-P4 Visual SLAM Configuration Deployment Script
echo ==================================================

REM Configuration source directory
set CONFIG_SRC_DIR=config
set SPIFFS_DIR=spiffs_image
set SD_DIR=sdcard_image

REM Create SPIFFS directory structure
echo Creating SPIFFS directory structure...
if not exist "%SPIFFS_DIR%" mkdir "%SPIFFS_DIR%"
if not exist "%SPIFFS_DIR%\config" mkdir "%SPIFFS_DIR%\config"

REM Create SD card directory structure
echo Creating SD card directory structure...
if not exist "%SD_DIR%" mkdir "%SD_DIR%"
if not exist "%SD_DIR%\config" mkdir "%SD_DIR%\config"
if not exist "%SD_DIR%\maps" mkdir "%SD_DIR%\maps"
if not exist "%SD_DIR%\sessions" mkdir "%SD_DIR%\sessions"
if not exist "%SD_DIR%\logs" mkdir "%SD_DIR%\logs"

REM Copy configuration files to SPIFFS
echo Copying configuration files to SPIFFS...
copy "%CONFIG_SRC_DIR%\system_config.json" "%SPIFFS_DIR%\config\"
copy "%CONFIG_SRC_DIR%\gps_config.json" "%SPIFFS_DIR%\config\"
copy "%CONFIG_SRC_DIR%\msp_config.json" "%SPIFFS_DIR%\config\"
copy "%CONFIG_SRC_DIR%\slam_config.json" "%SPIFFS_DIR%\config\"
copy "%CONFIG_SRC_DIR%\fusion_config.json" "%SPIFFS_DIR%\config\"
copy "%CONFIG_SRC_DIR%\camera_config.json" "%SPIFFS_DIR%\config\"
copy "%CONFIG_SRC_DIR%\imu_config.json" "%SPIFFS_DIR%\config\"

REM Copy configuration files to SD card
echo Copying configuration files to SD card...
copy "%CONFIG_SRC_DIR%\system_config.json" "%SD_DIR%\config\"
copy "%CONFIG_SRC_DIR%\gps_config.json" "%SD_DIR%\config\"
copy "%CONFIG_SRC_DIR%\msp_config.json" "%SD_DIR%\config\"
copy "%CONFIG_SRC_DIR%\slam_config.json" "%SD_DIR%\config\"
copy "%CONFIG_SRC_DIR%\fusion_config.json" "%SD_DIR%\config\"
copy "%CONFIG_SRC_DIR%\camera_config.json" "%SD_DIR%\config\"
copy "%CONFIG_SRC_DIR%\imu_config.json" "%SD_DIR%\config\"

echo Configuration files copied successfully!
echo.
echo SPIFFS Contents:
dir "%SPIFFS_DIR%\config\"
echo.
echo SD Card Contents:
dir "%SD_DIR%\config\"
echo.
echo Next steps:
echo 1. Build your ESP-IDF project: idf.py build
echo 2. Create SPIFFS image: mkspiffs -c %SPIFFS_DIR% -b 4096 -p 256 -s 0x0F0000 spiffs.bin
echo 3. Flash SPIFFS: esptool.py --chip esp32p4 write_flash 0x110000 spiffs.bin
echo 4. Copy SD card contents to your SD card root directory
echo 5. Or use: idf.py spiffs (if configured in partitions.csv)

pause
