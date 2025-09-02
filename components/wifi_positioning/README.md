# WiFi Positioning System for ESP32 Drone Camera

This system integrates WiFi-based positioning using WiGLE database data to complement GPS positioning for better navigation accuracy.

## Features

- **WiFi Scanning**: Automatically scans for nearby WiFi access points
- **WiGLE Database**: Uses offline WiGLE database for position estimation
- **Triangulation**: Calculates position using multiple access points
- **Sensor Fusion**: Integrates with existing GPS/IMU/SLAM sensor fusion
- **SD Card Storage**: Stores WiGLE database on SD card for offline operation

## Components

### 1. WiFi Positioning (`components/wifi_positioning/`)
- Core positioning algorithm using WiFi signals
- Distance-weighted triangulation
- Automatic scanning and position calculation

### 2. WiGLE Database (`components/wigle_db/`)
- Database management for WiGLE WiFi access point data
- CSV import/export functionality
- Efficient storage and lookup

### 3. Sensor Fusion Integration
- WiFi position updates integrated into EKF
- Automatic outlier detection and filtering
- Complementary to GPS positioning

## Setup Instructions

### 1. Obtain WiGLE Data

1. Visit [WiGLE.net](https://wigle.net/) and create an account
2. Export WiFi data for your area of interest (CSV format)
3. The CSV should contain columns: MAC,SSID,AuthMode,FirstSeen,Channel,RSSI,CurrentLatitude,CurrentLongitude,AltitudeMeters,AccuracyMeters,Type

### 2. Convert WiGLE CSV to Binary Database

**You need to create your own conversion program** that converts WiGLE CSV data to the binary format expected by the ESP32.

#### Binary Format Specification

The ESP32 expects WiGLE data in a custom binary format. Each database entry is a 40-byte structure:

```c
typedef struct {
    char bssid[18];            // BSSID "XX:XX:XX:XX:XX:XX" (null-terminated)
    double latitude;           // WGS84 latitude (-90.0 to 90.0)
    double longitude;          // WGS84 longitude (-180.0 to 180.0)
    float altitude;            // Altitude in meters
    uint32_t last_update;      // Unix timestamp (seconds since epoch)
    uint8_t signal_samples;    // Number of RSSI samples (1-255)
    int8_t avg_rssi;           // Average RSSI (-127 to 0)
    uint16_t accuracy;         // Position accuracy in meters (0-65535)
} wigle_db_entry_t;
```

**Binary file structure:**
- No header required
- Sequential `wigle_db_entry_t` records
- Little-endian byte order (ESP32 native)
- End of file = natural end when no more data

#### Conversion Requirements

Your conversion program should:

1. **Parse WiGLE CSV** with columns: `MAC,SSID,AuthMode,FirstSeen,Channel,RSSI,CurrentLatitude,CurrentLongitude,AltitudeMeters,AccuracyMeters,Type`

2. **Filter WiFi entries** where `Type == "WIFI"`

3. **Convert each entry**:
   - `bssid`: Use MAC column, format "XX:XX:XX:XX:XX:XX", pad to 18 bytes with null
   - `latitude/longitude`: Parse as doubles from CurrentLatitude/CurrentLongitude
   - `altitude`: Parse AltitudeMeters as float (0.0 if empty)
   - `last_update`: Convert FirstSeen to Unix timestamp
   - `signal_samples`: Set to 1 (or count multiple samples per BSSID)
   - `avg_rssi`: Convert RSSI to int8_t
   - `accuracy`: Parse AccuracyMeters as uint16_t

4. **Write binary records** sequentially using this struct format:
   ```c
   // C struct packing format: <18s d d f I B b H
   // Python: struct.pack('<18sddfIBbH', ...)
   ```

#### Example Python Conversion Code

```python
import struct
import csv
from datetime import datetime

def convert_wigle_csv(csv_path, binary_path):
    with open(csv_path, 'r') as csvfile, open(binary_path, 'wb') as binfile:
        reader = csv.DictReader(csvfile)
        
        for row in reader:
            if row['Type'] != 'WIFI':
                continue
                
            # Convert BSSID (pad to 18 bytes)
            bssid = row['MAC'].encode('ascii').ljust(18, b'\0')
            
            # Convert coordinates
            lat = float(row['CurrentLatitude'])
            lon = float(row['CurrentLongitude'])
            alt = float(row['AltitudeMeters'] or 0.0)
            
            # Convert timestamp
            dt = datetime.strptime(row['FirstSeen'], '%Y-%m-%d %H:%M:%S')
            timestamp = int(dt.timestamp())
            
            # Convert signal data
            signal_samples = 1
            rssi = int(row['RSSI'])
            accuracy = int(float(row['AccuracyMeters']))
            
            # Pack binary data (little-endian)
            data = struct.pack('<18s d d f I B b H',
                             bssid, lat, lon, alt, timestamp,
                             signal_samples, rssi, accuracy)
            
            binfile.write(data)
```

### 3. Copy Database to SD Card

1. Insert SD card into your development machine
2. Copy your converted binary file to `/sdcard/wigle_db.bin`
3. Insert SD card into ESP32

### 4. Configuration

The WiFi positioning system is automatically configured in `main/visual_slam_main.c`:

```c
wifi_positioning_config_t wifi_pos_config = {
    .scan_interval_ms = 30000,        // Scan every 30 seconds
    .min_ap_count = 3,                // Need at least 3 APs
    .max_ap_count = 10,               // Use up to 10 APs
    .min_rssi_threshold = -85,        // Minimum RSSI -85dBm
    .wigle_db_size = 10000,           // Support up to 10k entries
    .wigle_db_path = "/sdcard/wigle_db.bin",  // Binary database path
    .enable_auto_scan = true,         // Enable automatic scanning
    .position_timeout_ms = 5000       // 5 second timeout
};
```

## How It Works

### WiFi Scanning
1. ESP32 scans for nearby WiFi networks
2. Filters by RSSI threshold and minimum access point count
3. Extracts BSSID, SSID, RSSI, and channel information

### Position Calculation
1. Looks up each detected BSSID in WiGLE database
2. Uses distance-weighted triangulation algorithm
3. Considers signal strength and known AP locations
4. Estimates position accuracy based on AP distribution

### Sensor Fusion Integration
1. WiFi position fed into Extended Kalman Filter
2. Weighted with GPS and other sensors based on accuracy
3. Provides backup positioning when GPS is unavailable
4. Improves overall navigation accuracy

## Algorithm Details

### Distance Estimation
The system estimates distance to access points using RSSI:
```
distance = 10^((-50 - RSSI) / 20) * 1000 meters
```

### Position Calculation
Uses weighted average of known AP positions:
```
weight = 1 / (distance + 1) * (RSSI + 100) / 100
position = Σ(weight_i * position_i) / Σ(weight_i)
```

### Accuracy Estimation
```
accuracy = max_distance_between_APs * 1.5
```

## Performance Characteristics

- **Scan Time**: ~3-5 seconds for WiFi scanning
- **Position Calculation**: ~10-50ms
- **Memory Usage**: ~50KB for 1000 AP database
- **Accuracy**: 20-100 meters (depends on AP density)
- **Update Rate**: Configurable, default 30 seconds

## API Usage

### Initialize WiFi Positioning
```c
wifi_positioning_config_t config = {
    .scan_interval_ms = 30000,
    .min_ap_count = 3,
    .wigle_db_path = "/sdcard/wigle/wigle.db"
};

esp_err_t ret = wifi_positioning_init(&config);
wifi_positioning_start();
```

### Get WiFi Position
```c
wifi_position_t position;
if (wifi_positioning_get_position(&position) == ESP_OK && position.valid) {
    // Use position.latitude, position.longitude, position.accuracy_h
}
```

### Manual Scan and Calculate
```c
wifi_position_t position;
wifi_positioning_scan_and_calculate(&position);
```

## Troubleshooting

### No WiFi Networks Found
- Check WiFi is enabled in ESP32 configuration
- Verify scanning permissions
- Check antenna connection

### Poor Position Accuracy
- Ensure sufficient WiGLE data for your area
- Check database contains current AP locations
- Verify SD card is properly mounted

### Database Loading Issues
- Check SD card file system
- Verify database file exists at correct path
- Check file permissions

### Integration Issues
- Ensure sensor fusion is properly initialized
- Check WiFi position timestamps are reasonable
- Verify coordinate systems match (WGS84)

## Limitations

- **Accuracy**: 20-100 meters depending on AP density
- **Update Rate**: Limited by WiFi scanning time
- **Database Size**: Limited by ESP32 memory/SD card space
- **Coverage**: Only works in areas with WiGLE data
- **Privacy**: Requires WiFi scanning (may be restricted in some areas)

## Future Enhancements

- **Machine Learning**: Use ML for better position estimation
- **Crowdsourcing**: Allow users to contribute WiFi data
- **Indoor Positioning**: Enhanced algorithms for indoor environments
- **Real-time Updates**: Download fresh WiGLE data over WiFi
- **Multi-band Support**: Use 2.4GHz and 5GHz bands
- **BLE Integration**: Combine with Bluetooth Low Energy beacons

## Dependencies

- ESP-IDF WiFi stack
- SD card storage system
- Sensor fusion EKF
- FreeRTOS for task management

## License

This WiFi positioning system is part of the ESP32 Drone Camera project.
