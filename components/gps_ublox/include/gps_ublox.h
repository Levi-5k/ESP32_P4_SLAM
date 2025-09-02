#pragma once

#include <esp_err.h>
#include <driver/uart.h>
#include <driver/gpio.h>
#include "visual_slam_common_types.h"

#ifdef __cplusplus
extern "C" {
#endif

// GPS status enumeration
typedef enum {
    GPS_STATUS_DISCONNECTED = 0,
    GPS_STATUS_CONNECTED,
    GPS_STATUS_FIX_2D,
    GPS_STATUS_FIX_3D,
    GPS_STATUS_DGPS,
    GPS_STATUS_RTK_FLOAT,
    GPS_STATUS_RTK_FIXED
} gps_status_t;

// GPS fix quality enumeration
typedef enum {
    GPS_FIX_NONE = 0,
    GPS_FIX_DEAD_RECKONING,
    GPS_FIX_2D,
    GPS_FIX_3D,
    GPS_FIX_GNSS_DEAD_RECKONING,
    GPS_FIX_TIME_ONLY
} gps_fix_quality_t;

// Enhanced GPS data structure (extends the common gps_data_t)
typedef struct {
    double latitude;           // Degrees (-90 to 90)
    double longitude;          // Degrees (-180 to 180)
    float altitude;            // Meters above ellipsoid
    float geoid_separation;    // Geoid separation (m)
    float speed;               // Ground speed (m/s)
    float course;              // Course over ground (degrees)
    float hdop;                // Horizontal dilution of precision
    float vdop;                // Vertical dilution of precision
    float pdop;                // Position dilution of precision
    uint8_t satellites;        // Number of satellites used
    gps_fix_quality_t fix_quality; // GPS fix quality
    gps_status_t status;       // GPS module status
    uint32_t uptime;           // GPS uptime (seconds)
    uint64_t timestamp;        // Data timestamp (microseconds)
    bool fix_valid;            // True if GPS has valid fix
    bool differential;         // True if differential GPS is used
} gps_ublox_data_t;

// GPS configuration structure (extends the common gps_config_t)
typedef struct {
    uart_port_t uart_port;     // UART port number
    int baud_rate;             // UART baud rate
    int tx_pin;                // UART TX pin
    int rx_pin;                // UART RX pin
    uint32_t update_rate_hz;   // GPS update rate (1-10 Hz)
    bool enable_sbas;          // Enable SBAS corrections
    bool enable_differential;  // Enable differential GPS
    uint32_t timeout_ms;       // GPS fix timeout (milliseconds)
} gps_ublox_config_t;

// GPS statistics structure
typedef struct {
    uint32_t messages_received;
    uint32_t messages_parsed;
    uint32_t checksum_errors;
    uint32_t fix_count;
    uint32_t satellites_max;
    float hdop_min;
    float accuracy_best;
    uint64_t last_fix_time;
    uint64_t uptime_seconds;
} gps_stats_t;

// Function declarations
esp_err_t gps_ublox_init(const gps_ublox_config_t *config);
esp_err_t gps_ublox_deinit(void);
esp_err_t gps_ublox_get_data(gps_ublox_data_t *data);
esp_err_t gps_ublox_get_stats(gps_stats_t *stats);
esp_err_t gps_ublox_reset(void);
esp_err_t gps_ublox_set_update_rate(uint32_t rate_hz);
esp_err_t gps_ublox_force_cold_start(void);
esp_err_t gps_ublox_save_configuration(void);
bool gps_ublox_is_connected(void);
bool gps_ublox_has_fix(void);

// AssistNow Offline functions
esp_err_t gps_ublox_inject_assistnow_data(const uint8_t *data, size_t size);
esp_err_t gps_ublox_clear_assistnow_data(void);

// Position Aiding functions
esp_err_t gps_ublox_set_position_aiding(double latitude, double longitude, float altitude,
                                       float position_accuracy, float altitude_accuracy);
esp_err_t gps_ublox_clear_position_aiding(void);
esp_err_t gps_ublox_get_position_aiding_status(bool *enabled);

// Legacy compatibility functions
esp_err_t gps_ublox_init_legacy(const gps_ublox_config_t *config);

#ifdef __cplusplus
}
#endif
