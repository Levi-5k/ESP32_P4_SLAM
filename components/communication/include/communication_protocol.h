/**
 * @file communication_protocol.h
 * @brief Communication protocol between ESP32-P4 (master) and ESP32-C6 (slave)
 *
 * Defines the message structure and protocol for exchanging SLAM data,
 * configuration parameters, and web interface commands between devices.
 */

#ifndef COMMUNICATION_PROTOCOL_H
#define COMMUNICATION_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "visual_slam_common_types.h"

#ifdef __cplusplus
extern "C" {
#endif

// Protocol constants
#define COMM_PROTOCOL_VERSION       0x01
#define COMM_MAX_PAYLOAD_SIZE       1024
#define COMM_MAGIC_HEADER          0xDEADBEEF
#define COMM_MAX_SSID_LIST_SIZE    10
#define COMM_MAX_SSID_LEN          32
#define COMM_MAX_PASSWORD_LEN      64

// Message types from P4 to C6
typedef enum {
    MSG_P4_TO_C6_HEARTBEAT = 0x01,      // Regular heartbeat with basic status
    MSG_P4_TO_C6_SLAM_STATUS = 0x02,    // SLAM system status
    MSG_P4_TO_C6_POSITION = 0x03,       // Current position and orientation
    MSG_P4_TO_C6_TELEMETRY = 0x04,      // Sensor telemetry data
    MSG_P4_TO_C6_MAP_INFO = 0x05,       // Map information and statistics
    MSG_P4_TO_C6_ERROR = 0x06,          // Error conditions
    MSG_P4_TO_C6_CONFIG_ACK = 0x07,     // Acknowledge configuration changes
    MSG_P4_TO_C6_WIFI_STATUS = 0x08,    // WiFi status update from P4
    MSG_P4_TO_C6_WIFI_NETWORKS = 0x09,  // Network scan results
    MSG_P4_TO_C6_WIFI_POSITION_ACK = 0x0A,  // WiFi positioning acknowledgment
} msg_p4_to_c6_type_t;

// Message types from C6 to P4
typedef enum {
    MSG_C6_TO_P4_CONFIG_UPDATE = 0x10,  // Configuration parameter updates
    MSG_C6_TO_P4_WIFI_SCAN_REQ = 0x11,  // Request WiFi scan
    MSG_C6_TO_P4_CONNECT_WIFI = 0x12,   // Connect to WiFi network
    MSG_C6_TO_P4_SYSTEM_CMD = 0x13,     // System commands (restart, calibrate, etc.)
    MSG_C6_TO_P4_MAP_CMD = 0x14,        // Map commands (save, load, clear)
    MSG_C6_TO_P4_HEARTBEAT_ACK = 0x15,  // Heartbeat acknowledgment
    MSG_C6_TO_P4_WIFI_CONTROL = 0x16,   // WiFi enable/disable control
    MSG_C6_TO_P4_WIFI_POSITIONING_DATA = 0x19,  // WiFi access point data for positioning
} msg_c6_to_p4_type_t;

// System commands
typedef enum {
    SYS_CMD_RESTART = 0x01,
    SYS_CMD_CALIBRATE_IMU = 0x02,
    SYS_CMD_CALIBRATE_CAMERA = 0x03,
    SYS_CMD_RESET_SLAM = 0x04,
    SYS_CMD_EMERGENCY_STOP = 0x05,
    SYS_CMD_WIFI_ENABLE = 0x06,
    SYS_CMD_WIFI_DISABLE = 0x07,
    SYS_CMD_WIFI_GET_STATUS = 0x08,
} system_command_t;

// Map commands
typedef enum {
    MAP_CMD_SAVE = 0x01,
    MAP_CMD_LOAD = 0x02,
    MAP_CMD_CLEAR = 0x03,
    MAP_CMD_LIST = 0x04,
} map_command_t;

// Message header structure
typedef struct __attribute__((packed)) {
    uint32_t magic;                     // Magic header for synchronization
    uint8_t version;                    // Protocol version
    uint8_t msg_type;                   // Message type
    uint16_t payload_size;              // Size of payload data
    uint32_t sequence;                  // Message sequence number
    uint32_t timestamp;                 // Timestamp in milliseconds
    uint16_t checksum;                  // Simple checksum for data integrity
} comm_msg_header_t;

// Heartbeat message (P4 to C6)
typedef struct __attribute__((packed)) {
    uint32_t uptime_ms;                 // System uptime in milliseconds
    uint8_t system_status;              // Overall system status
    uint8_t slam_status;                // SLAM processing status
    uint8_t camera_status;              // Camera system status
    uint8_t sd_card_status;             // SD card status
    uint32_t free_heap_size;            // Available heap memory
    uint16_t cpu_usage_percent;         // CPU usage percentage
} heartbeat_msg_t;

// SLAM status message (P4 to C6)
typedef struct __attribute__((packed)) {
    uint8_t state;                      // SLAM state (tracking, lost, etc.)
    uint32_t tracked_features;          // Number of tracked features
    uint32_t map_points;                // Number of map points
    uint32_t keyframes;                 // Number of keyframes
    float tracking_quality;             // Tracking quality (0.0-1.0)
    uint8_t initialization_progress;    // Initialization progress (0-100%)
} slam_status_msg_t;

// Position message (P4 to C6)
typedef struct __attribute__((packed)) {
    float position_x;                   // X position in meters
    float position_y;                   // Y position in meters
    float position_z;                   // Z position in meters
    float orientation_roll;             // Roll angle in radians
    float orientation_pitch;            // Pitch angle in radians
    float orientation_yaw;              // Yaw angle in radians
    float velocity_x;                   // X velocity in m/s
    float velocity_y;                   // Y velocity in m/s
    float velocity_z;                   // Z velocity in m/s
    uint32_t timestamp_us;              // Position timestamp in microseconds
} position_msg_t;

// Telemetry message (P4 to C6)
typedef struct __attribute__((packed)) {
    // IMU data
    float accel_x, accel_y, accel_z;    // Accelerometer (m/s²)
    float gyro_x, gyro_y, gyro_z;       // Gyroscope (rad/s)
    float mag_x, mag_y, mag_z;          // Magnetometer (µT)
    
    // GPS data (if available)
    double gps_lat, gps_lon;            // GPS coordinates
    float gps_alt;                      // GPS altitude
    uint8_t gps_fix_type;               // GPS fix type
    uint8_t gps_satellites;             // Number of satellites
    
    // Camera data
    uint16_t camera_fps;                // Current camera FPS
    uint32_t frames_processed;          // Total frames processed
    uint8_t camera_exposure;            // Camera exposure level
    
    // System data
    float temperature;                  // System temperature
    uint16_t battery_voltage_mv;        // Battery voltage in millivolts
} telemetry_msg_t;

// Configuration update message (C6 to P4)
typedef struct __attribute__((packed)) {
    uint8_t config_type;                // Type of configuration to update
    uint8_t config_id;                  // Specific configuration parameter ID
    uint32_t value_size;                // Size of the value data
    uint8_t value_data[64];             // Configuration value data
} config_update_msg_t;

// WiFi scan request/response
typedef struct __attribute__((packed)) {
    uint8_t ssid[COMM_MAX_SSID_LEN];    // SSID name
    int8_t rssi;                        // Signal strength
    uint8_t auth_mode;                  // Authentication mode
    uint8_t channel;                    // WiFi channel
} wifi_network_info_t;

typedef struct __attribute__((packed)) {
    uint8_t network_count;              // Number of networks found
    wifi_network_info_t networks[COMM_MAX_SSID_LIST_SIZE];
} wifi_scan_response_t;

// WiFi connection request (C6 to P4)
typedef struct __attribute__((packed)) {
    uint8_t ssid[COMM_MAX_SSID_LEN];    // SSID to connect to
    uint8_t password[COMM_MAX_PASSWORD_LEN]; // WiFi password
    uint8_t auth_mode;                  // Authentication mode
} wifi_connect_msg_t;

// System command message (C6 to P4)
typedef struct __attribute__((packed)) {
    system_command_t command;           // System command to execute
    uint32_t parameter;                 // Command parameter (if needed)
} system_cmd_msg_t;

// Map command message (C6 to P4)
typedef struct __attribute__((packed)) {
    map_command_t command;              // Map command to execute
    char map_name[32];                  // Map name (for save/load operations)
} map_cmd_msg_t;

// WiFi control message (C6 to P4)
typedef struct __attribute__((packed)) {
    uint8_t enable_wifi;                // 1 = enable, 0 = disable
    uint8_t enable_scan_only;           // 1 = scan-only mode, 0 = full mode
    uint8_t auto_ap_fallback;           // 1 = enable AP fallback, 0 = disable
    uint8_t reserved;                   // Reserved for future use
} wifi_control_msg_t;

// WiFi status message (P4 to C6)
typedef struct __attribute__((packed)) {
    uint8_t wifi_enabled;               // WiFi module state
    uint8_t connected;                  // Connection status
    uint8_t scan_active;                // Scanning state
    uint8_t ap_mode_active;             // AP mode status
    uint8_t ssid[COMM_MAX_SSID_LEN];    // Connected SSID
    int8_t rssi;                        // Signal strength
    uint32_t ip_address;                // IP address (if connected)
    uint8_t mac_address[6];             // MAC address
} wifi_status_msg_t;

// WiFi positioning constants
#define MAX_POSITIONING_APS 10          // Maximum access points for positioning

// WiFi access point for positioning
typedef struct __attribute__((packed)) {
    char ssid[32];                      // SSID (31 chars + null terminator)
    char bssid[18];                     // BSSID in format "XX:XX:XX:XX:XX:XX"
    int8_t rssi;                        // Signal strength in dBm
    uint8_t channel;                    // WiFi channel
    uint8_t auth_mode;                  // Authentication mode
    uint8_t is_hidden;                  // Hidden network flag
} wifi_ap_positioning_t;

// WiFi positioning message (C6 to P4)
typedef struct __attribute__((packed)) {
    uint16_t sequence_number;           // Sequence number for tracking
    uint8_t ap_count;                   // Number of access points in this message
    uint8_t reserved;                   // Reserved for future use
    uint64_t timestamp;                 // Timestamp when scan was performed
    wifi_ap_positioning_t access_points[MAX_POSITIONING_APS];  // Access point data
} wifi_positioning_msg_t;

// WiFi positioning acknowledgment status
typedef enum {
    WIFI_POS_ACK_SUCCESS = 0x00,        // Position calculation successful
    WIFI_POS_ACK_ERROR = 0x01,          // Position calculation failed
    WIFI_POS_ACK_INSUFFICIENT_DATA = 0x02,  // Not enough access points
    WIFI_POS_ACK_TIMEOUT = 0x03,        // Processing timeout
} wifi_position_ack_status_t;

// WiFi positioning acknowledgment (P4 to C6)
typedef struct __attribute__((packed)) {
    uint16_t sequence_number;           // Sequence number from original message
    uint8_t status;                     // Status code (wifi_position_ack_status_t)
    uint8_t position_calculated;        // 1 if position was calculated, 0 otherwise
    uint32_t processing_time_ms;        // Time taken to process positioning data
    double latitude;                    // Calculated latitude (if successful)
    double longitude;                   // Calculated longitude (if successful)
    float accuracy;                     // Position accuracy in meters (if successful)
} wifi_position_ack_msg_t;

// Generic message structure
typedef struct __attribute__((packed)) {
    comm_msg_header_t header;
    union {
        heartbeat_msg_t heartbeat;
        slam_status_msg_t slam_status;
        position_msg_t position;
        telemetry_msg_t telemetry;
        config_update_msg_t config_update;
        wifi_scan_response_t wifi_scan;
        wifi_connect_msg_t wifi_connect;
        wifi_control_msg_t wifi_control;
        wifi_status_msg_t wifi_status;
        wifi_positioning_msg_t wifi_positioning;
        wifi_position_ack_msg_t wifi_position_ack;
        system_cmd_msg_t system_cmd;
        map_cmd_msg_t map_cmd;
        uint8_t raw_payload[COMM_MAX_PAYLOAD_SIZE];
    } payload;
} comm_message_t;

// Function prototypes
esp_err_t comm_protocol_init(void);
esp_err_t comm_send_heartbeat(const heartbeat_msg_t* heartbeat);
esp_err_t comm_send_slam_status(const slam_status_msg_t* status);
esp_err_t comm_send_position(const position_msg_t* position);
esp_err_t comm_send_telemetry(const telemetry_msg_t* telemetry);
esp_err_t comm_send_message(uint8_t msg_type, const void* payload, uint16_t payload_size);
esp_err_t comm_receive_message(comm_message_t* message, uint32_t timeout_ms);
uint16_t comm_calculate_checksum(const void* data, uint16_t size);
bool comm_verify_checksum(const comm_message_t* message);

#ifdef __cplusplus
}
#endif

#endif // COMMUNICATION_PROTOCOL_H
