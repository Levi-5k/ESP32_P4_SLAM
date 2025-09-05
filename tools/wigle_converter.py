#!/usr/bin/env python3
"""
WiGLE Database Converter
Converts WiGLE CSV files to binary database format for ESP32 WiFi positioning

Usage:
    python wigle_converter.py input.csv output.db

WiGLE CSV format expected:
MAC,SSID,AuthMode,FirstSeen,Channel,RSSI,CurrentLatitude,CurrentLongitude,AltitudeMeters,AccuracyMeters,Type

Output format:
Binary file with wigle_db_entry_t structures
"""

import struct
import sys
import csv
import os
from datetime import datetime

# WiGLE database entry structure (must match C struct)
WIGLE_ENTRY_FORMAT = '<18sddIfII'  # Little endian
WIGLE_ENTRY_SIZE = 18 + 8 + 8 + 4 + 4 + 4  # 46 bytes

class WigleEntry:
    def __init__(self):
        self.bssid = b'\x00' * 18
        self.latitude = 0.0
        self.longitude = 0.0
        self.altitude = 0.0
        self.last_update = 0
        self.signal_samples = 1
        self.avg_rssi = -50
        self.accuracy = 50

def parse_wigle_csv(csv_path, max_entries=10000):
    """Parse WiGLE CSV file and return list of WigleEntry objects"""
    entries = []

    print(f"Parsing WiGLE CSV: {csv_path}")

    with open(csv_path, 'r', encoding='utf-8') as csvfile:
        # Skip header
        next(csvfile, None)

        reader = csv.reader(csvfile)
        processed = 0
        valid_entries = 0

        for row in reader:
            processed += 1

            if len(row) < 11:
                continue

            try:
                entry = WigleEntry()

                # Parse BSSID (MAC address)
                mac = row[0].strip()
                if len(mac) != 17:  # XX:XX:XX:XX:XX:XX format
                    continue
                entry.bssid = mac.encode('ascii').ljust(18, b'\x00')

                # Parse coordinates
                entry.latitude = float(row[6])
                entry.longitude = float(row[7])

                # Validate coordinates
                if not (-90 <= entry.latitude <= 90) or not (-180 <= entry.longitude <= 180):
                    continue

                # Parse altitude (optional)
                try:
                    entry.altitude = float(row[8])
                except (ValueError, IndexError):
                    entry.altitude = 0.0

                # Parse RSSI
                try:
                    entry.avg_rssi = int(row[5])
                except (ValueError, IndexError):
                    entry.avg_rssi = -50

                # Parse accuracy
                try:
                    entry.accuracy = int(row[9])
                except (ValueError, IndexError):
                    entry.accuracy = 50

                # Set timestamp
                entry.last_update = int(datetime.now().timestamp())

                entries.append(entry)
                valid_entries += 1

                if valid_entries >= max_entries:
                    print(f"Reached maximum entries limit: {max_entries}")
                    break

            except (ValueError, IndexError) as e:
                continue

            # Progress reporting
            if processed % 1000 == 0:
                print(f"Processed {processed} lines, valid entries: {valid_entries}")

    print(f"Successfully parsed {valid_entries} valid entries from {processed} lines")
    return entries

def save_binary_database(entries, output_path):
    """Save entries to binary database file"""
    print(f"Saving binary database: {output_path}")

    with open(output_path, 'wb') as binfile:
        for entry in entries:
            # Pack the entry according to the C struct format
            data = struct.pack(WIGLE_ENTRY_FORMAT,
                             entry.bssid,
                             entry.latitude,
                             entry.longitude,
                             entry.altitude,
                             entry.last_update,
                             entry.signal_samples,
                             entry.avg_rssi,
                             entry.accuracy)
            binfile.write(data)

    print(f"Saved {len(entries)} entries to binary database")

def main():
    if len(sys.argv) != 3:
        print("Usage: python wigle_converter.py <input.csv> <output.db>")
        print("\nConverts WiGLE CSV export to binary database format")
        print("WiGLE CSV should be exported from: https://wigle.net/")
        sys.exit(1)

    csv_path = sys.argv[1]
    db_path = sys.argv[2]

    if not os.path.exists(csv_path):
        print(f"Error: Input file '{csv_path}' does not exist")
        sys.exit(1)

    # Parse CSV
    entries = parse_wigle_csv(csv_path)

    if len(entries) == 0:
        print("Error: No valid entries found in CSV file")
        sys.exit(1)

    # Save binary database
    save_binary_database(entries, db_path)

    # Print statistics
    print("
Database Statistics:")
    print(f"  Total entries: {len(entries)}")
    print(f"  Database size: {len(entries) * WIGLE_ENTRY_SIZE} bytes")
    print(f"  Average accuracy: {sum(e.accuracy for e in entries) / len(entries):.1f} meters")

    print(f"\n✅ Successfully converted WiGLE CSV to binary database")
    print(f"Copy '{os.path.basename(db_path)}' to your ESP32 SD card at '/sdcard/wigle/wigle.db'")

if __name__ == "__main__":
    main()
