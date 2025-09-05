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
    MSG_P4_TO_C6_CAMERA_FRAME = 0x08,   // Camera preview frame data
    MSG_P4_TO_C6_WIFI_STATUS = 0x09,    // WiFi status update from P4
    MSG_P4_TO_C6_WIFI_NETWORKS = 0x0A,  // Network scan results
} msg_p4_to_c6_type_t;

// Message types from C6 to P4
typedef enum {
    MSG_C6_TO_P4_CONFIG_UPDATE = 0x10,  // Configuration parameter updates
    MSG_C6_TO_P4_WIFI_SCAN_REQ = 0x11,  // Request WiFi scan
    MSG_C6_TO_P4_CONNECT_WIFI = 0x12,   // Connect to WiFi network
    MSG_C6_TO_P4_SYSTEM_CMD = 0x13,     // System commands (restart, calibrate, etc.)
    MSG_C6_TO_P4_MAP_CMD = 0x14,        // Map commands (save, load, clear)
    MSG_C6_TO_P4_HEARTBEAT_ACK = 0x15,  // Heartbeat acknowledgment
    MSG_C6_TO_P4_CAMERA_CMD = 0x16,     // Camera configuration commands
    MSG_C6_TO_P4_PREVIEW_REQ = 0x17,    // Request camera preview frame
    MSG_C6_TO_P4_WIFI_CONTROL = 0x18,   // WiFi enable/disable control
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

// Camera commands
typedef enum {
    CAM_CMD_SET_EXPOSURE = 0x01,
    CAM_CMD_SET_BRIGHTNESS = 0x02,
    CAM_CMD_SET_CONTRAST = 0x03,
    CAM_CMD_SET_SATURATION = 0x04,
    CAM_CMD_SET_RESOLUTION = 0x05,
    CAM_CMD_SET_FPS = 0x06,
    CAM_CMD_START_PREVIEW = 0x07,
    CAM_CMD_STOP_PREVIEW = 0x08,
    CAM_CMD_GET_SETTINGS = 0x09,
} camera_command_t;

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

// Camera command message (C6 to P4)
typedef struct __attribute__((packed)) {
    camera_command_t command;           // Camera command to execute
    uint32_t value;                     // Command value (exposure, brightness, etc.)
    uint8_t parameter_data[32];         // Additional parameter data if needed
} camera_cmd_msg_t;

// Camera frame message (P4 to C6) - for preview
typedef struct __attribute__((packed)) {
    uint16_t width;                     // Frame width
    uint16_t height;                    // Frame height
    uint8_t format;                     // Image format (JPEG=1, RGB=2, etc.)
    uint32_t frame_size;                // Size of frame data in bytes
    uint32_t frame_number;              // Frame sequence number
    uint8_t quality;                    // JPEG quality (0-100)
    // Frame data follows this header in the payload
} camera_frame_msg_t;

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
        system_cmd_msg_t system_cmd;
        map_cmd_msg_t map_cmd;
        camera_cmd_msg_t camera_cmd;
        camera_frame_msg_t camera_frame;
        uint8_t raw_payload[COMM_MAX_PAYLOAD_SIZE];
    } payload;
} comm_message_t;

// Function prototypes
uint16_t comm_calculate_checksum(const void* data, uint16_t size);
bool comm_verify_checksum(const comm_message_t* message);

#ifdef __cplusplus
}
#endif

#endif // COMMUNICATION_PROTOCOL_H
