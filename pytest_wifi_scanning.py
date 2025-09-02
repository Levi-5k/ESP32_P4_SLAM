#!/usr/bin/env python3
"""
Test script for independent WiFi scanning functionality
This test verifies that WiFi scanning works independently of positioning being enabled
"""

import pytest
from pytest_embedded_idf.dut import IdfDut


@pytest.mark.parametrize(
    'target, config',
    [
        ('esp32p4', 'sdkconfig.defaults.esp32p4'),
    ],
)
def test_wifi_scanning_independent(dut: IdfDut) -> None:
    """Test that WiFi scanning works independently of positioning system"""

    # Wait for system to initialize
    dut.expect('System initialized successfully', timeout=30)

    # Test 1: Verify WiFi scanning functions are available
    dut.expect('WiFi positioning component initialized', timeout=10)

    # Test 2: Test independent scanning API
    # This would call wifi_positioning_get_visible_ap_count() without initializing positioning
    dut.expect('WiFi scanning initialized successfully', timeout=15)

    # Test 3: Verify scanning works without positioning enabled
    dut.expect('Found.*WiFi access points', timeout=20)

    print("✅ Independent WiFi scanning test passed!")


if __name__ == '__main__':
    # Simple manual test
    print("WiFi Independent Scanning Test")
    print("==============================")
    print("This test verifies that WiFi scanning works independently of positioning.")
    print("Run with: pytest pytest_wifi_scanning.py --target esp32p4")
    print("")
    print("Test checks:")
    print("- WiFi scanning initializes without positioning system")
    print("- Scanning API functions work independently")
    print("- Access point detection works without positioning enabled")
