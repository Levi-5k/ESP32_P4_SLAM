#pragma once

#include <esp_err.h>
#include <stdint.h>
#include <stdbool.h>
#include <driver/uart.h>
#include "visual_slam_common_types.h"

#ifdef __cplusplus
extern "C" {
#endif

// MSP Protocol Constants
#define MSP_PROTOCOL_VERSION 0
#define MSP_API_VERSION_1_42 0x5E42
#define MSP_API_VERSION_1_43 0x5E43
#define MSP_API_VERSION_1_44 0x5E44

// MSP Message Directions
#define MSP_DIRECTION_FROM_FC 0x3C  // '<' - From Flight Controller
#define MSP_DIRECTION_TO_FC   0x3E  // '>' - To Flight Controller

// MSP Command IDs (subset for navigation)
#define MSP_API_VERSION         1
#define MSP_FC_VARIANT          2
#define MSP_FC_VERSION          3
#define MSP_BOARD_INFO          4
#define MSP_BUILD_INFO          5
#define MSP_STATUS              101
#define MSP_RAW_IMU             102
#define MSP_SERVO               103
#define MSP_MOTOR               104
#define MSP_RC                  105
#define MSP_RAW_GPS             106
#define MSP_COMP_GPS            107
#define MSP_ATTITUDE            108
#define MSP_ALTITUDE            109
#define MSP_ANALOG              110
#define MSP_RC_TUNING           111
#define MSP_PID                 112
#define MSP_BOX                 113
#define MSP_MISC                114
#define MSP_MOTOR_PINS          115
#define MSP_BOXNAMES            116
#define MSP_PIDNAMES            117
#define MSP_WP                  118
#define MSP_BOXIDS              119
#define MSP_SERVO_CONF          120
#define MSP_NAV_STATUS          121
#define MSP_NAV_CONFIG          122
#define MSP_MOTOR_3D_CONFIG     124
#define MSP_RC_DEADBAND         125
#define MSP_SENSOR_ALIGNMENT    126
#define MSP_LED_STRIP_MODECOLOR 127
#define MSP_VOLTAGE_METER_CONFIG 128
#define MSP_CURRENT_METER_CONFIG 129
#define MSP_GPS_CONFIG          132
#define MSP_COMPASS_CONFIG      133
#define MSP_ESC_SENSOR_CONFIG   140
#define MSP_GPS_RESCUE          135
#define MSP_GPS_RESCUE_PIDS     136
#define MSP_VTX_CONFIG          137
#define MSP_VTXTABLE_BAND       138
#define MSP_VTXTABLE_CHANNEL    139
#define MSP_VTXTABLE_POWERLEVEL 140
#define MSP_RTC                 247
#define MSP_SET_RAW_RC          200
#define MSP_SET_RAW_GPS         201
#define MSP_SET_PID             202
#define MSP_SET_BOX             203
#define MSP_SET_RC_TUNING       204
#define MSP_ACC_CALIBRATION     205
#define MSP_MAG_CALIBRATION     206
#define MSP_SET_MISC            207
#define MSP_RESET_CONF          208
#define MSP_SET_WP              209
#define MSP_SELECT_SETTING      210
#define MSP_SET_HEAD            211
#define MSP_SET_SERVO_CONF      212
#define MSP_SET_MOTOR           214
#define MSP_SET_NAV_CONFIG      215
#define MSP_SET_MOTOR_3D_CONFIG 217
#define MSP_SET_RC_DEADBAND     218
#define MSP_SET_RESET_CURR_PID  219
#define MSP_SET_SENSOR_ALIGNMENT 220
#define MSP_SET_LED_STRIP_MODECOLOR 221
#define MSP_SET_VOLTAGE_METER_CONFIG 222
#define MSP_SET_CURRENT_METER_CONFIG 223
#define MSP_SET_GPS_CONFIG      225
#define MSP_SET_COMPASS_CONFIG  226
#define MSP_SET_GPS_RESCUE      227
#define MSP_SET_GPS_RESCUE_PIDS 228
#define MSP_SET_VTX_CONFIG      229
#define MSP_SET_VTXTABLE_BAND   230
#define MSP_SET_VTXTABLE_CHANNEL 231
#define MSP_SET_VTXTABLE_POWERLEVEL 232

// Navigation modes for MSP_SET_NAV_CONFIG
#define NAV_MODE_NONE           0
#define NAV_MODE_POSHOLD        1
#define NAV_MODE_WP             2
#define NAV_MODE_RTH            3
#define NAV_MODE_CRUISE         4

// MSP Message Structure
typedef struct {
    uint8_t direction;        // '<' or '>'
    uint16_t size;           // payload size
    uint8_t command;         // MSP command ID
    uint8_t payload[256];    // payload data (max 256 bytes)
    uint8_t checksum;        // checksum
} msp_message_t;

// MSP Protocol State
typedef enum {
    MSP_STATE_IDLE,
    MSP_STATE_DIRECTION,
    MSP_STATE_SIZE,
    MSP_STATE_COMMAND,
    MSP_STATE_PAYLOAD,
    MSP_STATE_CHECKSUM
} msp_parser_state_t;

// Navigation waypoint structure
typedef struct {
    uint8_t action;          // waypoint action
    uint8_t flag;            // waypoint flag
    int32_t lat;             // latitude * 1e7
    int32_t lon;             // longitude * 1e7
    int32_t alt;             // altitude in cm
    uint16_t p1;             // parameter 1
    uint16_t p2;             // parameter 2
    uint16_t p3;             // parameter 3
} msp_waypoint_t;

// Navigation status structure
typedef struct {
    uint8_t mode;            // navigation mode
    uint8_t state;           // navigation state
    uint8_t active_wp_action; // active waypoint action
    uint8_t active_wp_number; // active waypoint number
    uint8_t error;           // navigation error
    uint16_t target_bearing; // target bearing
} msp_nav_status_t;

// MSP Protocol Configuration
typedef struct {
    uart_port_t uart_port;
    int baud_rate;
    int tx_pin;
    int rx_pin;
    uint32_t timeout_ms;
    bool auto_reconnect;
} msp_config_t;

// MSP Statistics
typedef struct {
    uint32_t messages_sent;
    uint32_t messages_received;
    uint32_t checksum_errors;
    uint32_t timeouts;
    uint32_t parse_errors;
    uint64_t last_message_time;
} msp_stats_t;

// Function declarations
esp_err_t msp_protocol_init_config(const msp_config_t *config);
esp_err_t msp_protocol_deinit(void);

// Message sending functions
esp_err_t msp_send_raw_rc(const uint16_t *channels, uint8_t num_channels);
esp_err_t msp_send_set_wp(const msp_waypoint_t *waypoint, uint8_t wp_number);
esp_err_t msp_send_set_nav_config(uint8_t mode);
esp_err_t msp_send_set_pid(const uint8_t *pid_data, size_t data_size);
esp_err_t msp_send_set_rc_tuning(const uint8_t *tuning_data, size_t data_size);
esp_err_t msp_send_set_misc(const uint8_t *misc_data, size_t data_size);
esp_err_t msp_send_set_gps_rescue(const uint8_t *rescue_data, size_t data_size);
esp_err_t msp_send_emergency_stop(void);

// Message receiving functions
esp_err_t msp_request_status(void);
esp_err_t msp_request_nav_status(void);
esp_err_t msp_request_attitude(void);
esp_err_t msp_request_altitude(void);
esp_err_t msp_request_raw_gps(void);
esp_err_t msp_request_comp_gps(void);
esp_err_t msp_request_analog(void);
esp_err_t msp_request_wp(uint8_t wp_number);

// Status and diagnostics
esp_err_t msp_get_stats(msp_stats_t *stats);
bool msp_is_connected(void);
esp_err_t msp_reset_connection(void);

// Navigation commands
esp_err_t msp_nav_set_mode(uint8_t mode);
esp_err_t msp_nav_set_home_position(double lat, double lon, float alt);
esp_err_t msp_nav_goto_position(double lat, double lon, float alt, uint16_t hold_time);
esp_err_t msp_nav_return_to_home(void);
esp_err_t msp_nav_land_here(void);
esp_err_t msp_nav_takeoff(float altitude);

// Utility functions
esp_err_t msp_send_message(uint8_t command, const uint8_t *payload, uint16_t payload_size);
esp_err_t msp_receive_message(msp_message_t *message, uint32_t timeout_ms);

// Legacy compatibility
esp_err_t msp_protocol_init(const msp_config_t *config);

#ifdef __cplusplus
}
#endif
