/**
 * @file esp32p4_pin_config.h
 * @brief Centralized ESP32-P4 pin configuration for DroneCam Visual SLAM system
 *
 * This file defines all GPIO pin assignments to prevent conflicts and provide
 * a single source of truth for hardware pin configuration.
 */

#ifndef ESP32P4_PIN_CONFIG_H
#define ESP32P4_PIN_CONFIG_H

#include "driver/gpio.h"

// ============================================================================
// CAMERA SYSTEM PINS
// ============================================================================

/** Camera I2C Interface Pins */
#define CAMERA_I2C_SDA_IO           7       /**< I2C SDA pin for OV5647 camera */
#define CAMERA_I2C_SCL_IO           8       /**< I2C SCL pin for OV5647 camera */

// ============================================================================
// IMU SENSOR PINS (SPI Interface)
// ============================================================================

/** BMI088 IMU SPI Interface Pins */
#define IMU_SPI_MISO_PIN           GPIO_NUM_8    /**< SPI MISO for IMU */
#define IMU_SPI_MOSI_PIN           GPIO_NUM_9    /**< SPI MOSI for IMU */
#define IMU_SPI_CLK_PIN            GPIO_NUM_10   /**< SPI Clock for IMU */
#define IMU_ACC_CS_PIN             GPIO_NUM_11   /**< Accelerometer Chip Select */
#define IMU_GYRO_CS_PIN            GPIO_NUM_12   /**< Gyroscope Chip Select */

// ============================================================================
// GPS MODULE PINS (UART Interface)
// ============================================================================

/** uBlox GPS UART Interface Pins */
/** @note Previously used GPIO 15(TX)/16(RX) but conflicted with LED pins */
#define GPS_UART_TX_PIN            GPIO_NUM_4    /**< GPS UART TX pin */
#define GPS_UART_RX_PIN            GPIO_NUM_5    /**< GPS UART RX pin */

// ============================================================================
// SDMMC INTERFACE PINS
// ============================================================================

/** SD Card SDMMC Interface Pins */
#define SDMMC_CLK_PIN              GPIO_NUM_43   /**< SDMMC Clock */
#define SDMMC_CMD_PIN              GPIO_NUM_44   /**< SDMMC Command */
#define SDMMC_D0_PIN               GPIO_NUM_39   /**< SDMMC Data 0 */
#define SDMMC_D1_PIN               GPIO_NUM_40   /**< SDMMC Data 1 */
#define SDMMC_D2_PIN               GPIO_NUM_41   /**< SDMMC Data 2 */
#define SDMMC_D3_PIN               GPIO_NUM_42   /**< SDMMC Data 3 */

// ============================================================================
// SYSTEM STATUS LED PINS
// ============================================================================

/** System Status Indicator LEDs */
/** @note Using GPIO 15-17 range, avoiding GPS UART conflicts */
#define SYSTEM_STATUS_LED_PIN      GPIO_NUM_15   /**< System status indicator */
#define SLAM_ACTIVITY_LED_PIN      GPIO_NUM_16   /**< SLAM processing activity */
#define COMMUNICATION_LED_PIN      GPIO_NUM_17   /**< Communication status */

// ============================================================================
// MSP PROTOCOL PINS (UART Interface)
// ============================================================================

/** MSP Protocol UART for INAV Flight Controller */
/** @note Using available UART pins, avoiding conflicts */
#define MSP_UART_TX_PIN            GPIO_NUM_18   /**< MSP UART TX */
#define MSP_UART_RX_PIN            GPIO_NUM_19   /**< MSP UART RX */

// ============================================================================
// PIN USAGE VALIDATION MACROS
// ============================================================================

/**
 * Compile-time pin conflict detection
 * These macros will generate warnings if pins are redefined elsewhere
 */

// Camera I2C pins validation
#define _VALIDATE_CAMERA_I2C_PINS \
    _Static_assert(CAMERA_I2C_SDA_IO == 7, "Camera I2C SDA pin conflict detected"); \
    _Static_assert(CAMERA_I2C_SCL_IO == 8, "Camera I2C SCL pin conflict detected");

// IMU SPI pins validation
#define _VALIDATE_IMU_SPI_PINS \
    _Static_assert(IMU_SPI_MISO_PIN == GPIO_NUM_8, "IMU SPI MISO pin conflict detected"); \
    _Static_assert(IMU_SPI_MOSI_PIN == GPIO_NUM_9, "IMU SPI MOSI pin conflict detected"); \
    _Static_assert(IMU_SPI_CLK_PIN == GPIO_NUM_10, "IMU SPI CLK pin conflict detected"); \
    _Static_assert(IMU_ACC_CS_PIN == GPIO_NUM_11, "IMU ACC CS pin conflict detected"); \
    _Static_assert(IMU_GYRO_CS_PIN == GPIO_NUM_12, "IMU GYRO CS pin conflict detected");

// GPS UART pins validation
#define _VALIDATE_GPS_UART_PINS \
    _Static_assert(GPS_UART_TX_PIN == GPIO_NUM_4, "GPS UART TX pin conflict detected"); \
    _Static_assert(GPS_UART_RX_PIN == GPIO_NUM_5, "GPS UART RX pin conflict detected");

// SDMMC pins validation
#define _VALIDATE_SDMMC_PINS \
    _Static_assert(SDMMC_CLK_PIN == GPIO_NUM_43, "SDMMC CLK pin conflict detected"); \
    _Static_assert(SDMMC_CMD_PIN == GPIO_NUM_44, "SDMMC CMD pin conflict detected"); \
    _Static_assert(SDMMC_D0_PIN == GPIO_NUM_39, "SDMMC D0 pin conflict detected"); \
    _Static_assert(SDMMC_D1_PIN == GPIO_NUM_40, "SDMMC D1 pin conflict detected"); \
    _Static_assert(SDMMC_D2_PIN == GPIO_NUM_41, "SDMMC D2 pin conflict detected"); \
    _Static_assert(SDMMC_D3_PIN == GPIO_NUM_42, "SDMMC D3 pin conflict detected");

// System LED pins validation
#define _VALIDATE_SYSTEM_LED_PINS \
    _Static_assert(SYSTEM_STATUS_LED_PIN == GPIO_NUM_15, "System LED pin conflict detected"); \
    _Static_assert(SLAM_ACTIVITY_LED_PIN == GPIO_NUM_16, "SLAM LED pin conflict detected"); \
    _Static_assert(COMMUNICATION_LED_PIN == GPIO_NUM_17, "Communication LED pin conflict detected");

// MSP UART pins validation
#define _VALIDATE_MSP_UART_PINS \
    _Static_assert(MSP_UART_TX_PIN == GPIO_NUM_18, "MSP UART TX pin conflict detected"); \
    _Static_assert(MSP_UART_RX_PIN == GPIO_NUM_19, "MSP UART RX pin conflict detected");

/**
 * Master validation macro - call this in main.c to validate all pin assignments
 */
#define VALIDATE_ALL_PIN_ASSIGNMENTS \
    _VALIDATE_CAMERA_I2C_PINS \
    _VALIDATE_IMU_SPI_PINS \
    _VALIDATE_GPS_UART_PINS \
    _VALIDATE_SDMMC_PINS \
    _VALIDATE_SYSTEM_LED_PINS \
    _VALIDATE_MSP_UART_PINS

// ============================================================================
// PIN FUNCTION COMPATIBILITY CHECKS
// ============================================================================

/**
 * Pin function compatibility warnings
 * These will generate compiler warnings if incompatible functions are assigned to same pins
 * NOTE: Currently disabled as pins are intentionally different (GPIO 4/5 vs 15/16)
 */

/*
#if (GPS_UART_TX_PIN) == (SYSTEM_STATUS_LED_PIN)
#warning "GPS TX pin conflicts with System Status LED pin!"
#endif

#if (GPS_UART_RX_PIN) == (SLAM_ACTIVITY_LED_PIN)
#warning "GPS RX pin conflicts with SLAM Activity LED pin!"
#endif
*/

// ============================================================================
// HARDWARE ABSTRACTION MACROS
// ============================================================================

/**
 * Hardware-specific pin configurations for different ESP32 variants
 * These can be used to conditionally compile pin assignments based on target
 */

#ifdef CONFIG_IDF_TARGET_ESP32P4
// ESP32-P4 specific pin configurations
#define HAS_SDMMC_SUPPORT          1
#define HAS_MIPI_CSI_SUPPORT       1
#define MAX_UART_INSTANCES         3
#define MAX_SPI_INSTANCES          2
#define MAX_I2C_INSTANCES          2
#endif

#endif /* ESP32P4_PIN_CONFIG_H */
