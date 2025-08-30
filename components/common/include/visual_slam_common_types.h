#ifndef VISUAL_SLAM_COMMON_TYPES_H
#define VISUAL_SLAM_COMMON_TYPES_H

#include <stdint.h>
#include <stdbool.h>

// Hardware constants and definitions
#define CAMERA_RES_640x480        0
#define CAMERA_FORMAT_RGB565      1
#define UART_NUM_1               1
#define I2C_NUM_0                0
#define BMI088_ACCEL_RANGE_6G    6
#define BMI088_GYRO_RANGE_500DPS 500

#ifdef __cplusplus
extern "C" {
#endif

// Basic geometric types
typedef struct {
    float x, y, z;
} vector3_t;

typedef struct {
    float w, x, y, z;
} quaternion_t;

// Camera frame structure
typedef struct {
    uint8_t* data;
    uint32_t width;
    uint32_t height;
    uint32_t data_size;
    uint64_t timestamp_us;
    uint32_t format;
} camera_frame_t;

// SLAM pose structure
typedef struct {
    float x, y, z;          // Position components
    float qw, qx, qy, qz;   // Orientation quaternion
    
    // Legacy array formats for compatibility
    float position[3];      // Position array {x, y, z}
    float orientation[4];   // Quaternion array {w, x, y, z}
    
    uint64_t timestamp_us;  // Timestamp
    float confidence;       // Confidence [0-1]
    bool is_lost;          // Tracking lost flag
    uint32_t tracked_features;  // Number of tracked features
} slam_pose_t;

// IMU data structure
typedef struct {
    float accel_x, accel_y, accel_z;  // Acceleration m/s²
    float gyro_x, gyro_y, gyro_z;     // Angular velocity rad/s
    float temp;                       // Temperature °C
    float temp_c;                     // Temperature alias for compatibility
    uint64_t timestamp_us;            // Timestamp
} imu_data_t;

// GPS position structure
typedef struct {
    double latitude;        // Degrees
    double longitude;       // Degrees
    float altitude;         // Meters above sea level
    float accuracy;         // Horizontal accuracy estimate
    uint8_t gps_fix_type;   // GPS fix type
    uint8_t satellites;     // Number of satellites
    uint64_t timestamp_us;  // Timestamp
} gps_position_t;

// GPS data alias for compatibility
typedef gps_position_t gps_data_t;

// Navigation state structure (extended)
typedef struct {
    // GPS coordinates
    double latitude;
    double longitude;
    float altitude;
    
    // Local NED coordinates
    float local_x;
    float local_y;
    float local_z;
    
    // Velocity components
    float vel_x;
    float vel_y;
    float vel_z;
    
    // Quaternion orientation
    float qw, qx, qy, qz;
    
    // Legacy fields for compatibility
    vector3_t position;      // Position in world frame
    vector3_t velocity;      // Velocity in world frame  
    vector3_t attitude;      // Roll, pitch, yaw
    
    // Accuracy estimates
    float accuracy;          // Position accuracy estimate
    float position_accuracy;
    float heading_accuracy;
    
    // Status and metadata
    uint8_t navigation_status;
    uint64_t timestamp_us;   // Timestamp
} navigation_state_t;

// System status structure
typedef struct {
    bool slam_initialized;
    bool slam_tracking;
    bool gps_available;
    bool imu_available;
    bool camera_available;
    bool camera_initialized;
    bool gps_connected;
    bool imu_calibrated;
    bool slam_active;
    bool wifi_connected;
    bool web_server_running;
    uint32_t uptime_seconds;
    float cpu_usage;
    float cpu_usage_percent;
    float frame_rate_fps;
    uint32_t frame_count;
    uint32_t feature_count;
    float position_x, position_y, position_z;
    float attitude_roll, attitude_pitch, attitude_yaw;
    uint32_t free_heap;
    uint32_t free_heap_bytes;
    uint32_t min_free_heap;
    int8_t wifi_rssi;
    char ip_address[16];
    uint64_t timestamp_us;
} system_status_t;

// Configuration structures
typedef struct {
    uint32_t max_features;
    float fast_threshold;
    uint32_t levels;
    float scale_factor;
    uint32_t max_keypoints_per_level;
    bool use_harris_detector;
    float harris_k;
    uint32_t descriptor_distance_threshold;
    
    // Additional SLAM-specific config members 
    float match_threshold;              // Feature matching threshold
    uint32_t min_tracked_features;      // Minimum features for valid pose
    float keyframe_distance_threshold;  // Meters - when to create new keyframe
    float keyframe_angle_threshold;     // Radians - when to create new keyframe
    uint32_t max_keyframes;             // Maximum keyframes in local map
    bool enable_loop_closure;           // Enable loop closure detection
    float loop_closure_threshold;       // Similarity threshold for loop detection
    
    // Additional members for compatibility
    uint32_t orb_features;              // Alternative name for max_features
    uint32_t keyframe_threshold;        // Alternative name for min_tracked_features
    bool loop_closure_enabled;         // Alternative name
    bool map_optimization_enabled;      // Map optimization flag
} slam_config_t;

// Additional config types
typedef struct {
    float position_noise;         // Position process noise (m²/s³)
    float velocity_noise;         // Velocity process noise (m²/s³)
    float attitude_noise;         // Attitude process noise (rad²/s)
    float slam_attitude_noise;    // SLAM attitude noise
    float gps_position_noise;     // GPS position measurement noise (m²)
    float gps_velocity_noise;     // GPS velocity measurement noise (m²/s²)
    float slam_position_noise;    // SLAM position measurement noise (m²)
    float accel_bias_noise;       // Accelerometer bias noise
    float gyro_bias_noise;        // Gyroscope bias noise
    float gps_weight;             // GPS fusion weight (0.0-1.0)
    float slam_weight;            // SLAM fusion weight (0.0-1.0)
    float imu_weight;             // IMU fusion weight (0.0-1.0)
    float imu_dt;                 // IMU sample time
    float max_prediction_time;    // Maximum prediction time
} fusion_config_t;

typedef struct {
    uint32_t max_features;        // Maximum features per frame
    float scale_factor;           // Scale factor between levels (typically 1.2)
    uint8_t num_levels;           // Number of pyramid levels
    uint8_t edge_threshold;       // FAST edge threshold
    uint8_t first_level;          // First pyramid level
    uint8_t wta_k;                // Number of points for descriptor (2 or 4)
    uint8_t patch_size;           // Patch size for descriptor computation
} orb_config_t;

// Configuration structures for hardware components
typedef struct {
    uint32_t resolution;          // Camera resolution
    uint32_t format;             // Pixel format
    uint32_t fps;                // Frame rate
    bool auto_exposure;          // Auto exposure enable
    bool auto_white_balance;     // Auto white balance enable
    
    // Manual adjustment settings
    int brightness;              // Brightness (-100 to 100)
    int contrast;                // Contrast (-100 to 100)  
    int saturation;              // Saturation (-100 to 100)
    int exposure_value;          // Manual exposure value
    
    // Auto-adjustment settings
    bool auto_adjustment_enabled; // Enable automatic brightness/contrast adjustment
    int target_brightness;       // Target average brightness (0-255)
    float adjustment_speed;      // Adjustment speed factor (0.1-1.0)
    float brightness_threshold;  // Brightness change threshold to trigger adjustment
} camera_config_t;

typedef struct {
    uint32_t uart_port;          // UART port number
    uint32_t baud_rate;          // Baud rate
    uint32_t tx_pin;             // TX pin
    uint32_t rx_pin;             // RX pin
} gps_config_t;

typedef struct {
    uint32_t spi_host;           // SPI host
    uint32_t miso_pin;           // MISO pin
    uint32_t mosi_pin;           // MOSI pin
    uint32_t sclk_pin;           // SCLK pin
    uint32_t acc_cs_pin;         // Accelerometer CS pin
    uint32_t gyro_cs_pin;        // Gyroscope CS pin
    uint32_t accel_range;        // Accelerometer range
    uint32_t gyro_range;         // Gyroscope range  
    uint32_t sample_rate;        // Sample rate
} imu_config_t;

typedef struct {
    navigation_state_t navigation;
    system_status_t system_status;
    imu_data_t imu_data;
    gps_data_t gps_data;
    
    // Compatibility fields for main app
    struct {
        float x, y, z;
    } position;
    
    struct {
        float roll, pitch, yaw;
    } attitude;
} sensor_fusion_data_t;

typedef struct {
    slam_pose_t pose;
    uint32_t num_features;
    float tracking_quality;
    bool is_lost;
} slam_result_t;

#ifdef __cplusplus
}
#endif

#endif // VISUAL_SLAM_COMMON_TYPES_H
