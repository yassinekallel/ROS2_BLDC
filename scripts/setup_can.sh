#!/bin/bash

# RefBoard CAN Interface Setup Script
# Sets up CAN interface for RefBoard communication

set -e

CAN_INTERFACE=${1:-can0}
BITRATE=${2:-500000}

echo "=== Setting up CAN interface: $CAN_INTERFACE ==="

# Check if interface exists
if ! ip link show $CAN_INTERFACE &> /dev/null; then
    echo "Error: CAN interface $CAN_INTERFACE not found!"
    echo "Available interfaces:"
    ip link show | grep can || echo "No CAN interfaces found"
    echo ""
    echo "If using CANable v2:"
    echo "1. Connect CANable v2 to USB"
    echo "2. Check if it appears as a USB device: lsusb | grep -i can"
    echo "3. The interface should appear automatically as can0"
    echo ""
    echo "For virtual CAN (testing without hardware):"
    echo "  sudo modprobe vcan"
    echo "  sudo ip link add dev vcan0 type vcan"
    echo "  sudo ip link set up vcan0"
    exit 1
fi

# Bring down interface if it's already up
echo "Bringing down interface if already up..."
sudo ip link set $CAN_INTERFACE down 2>/dev/null || true

# Configure and bring up the interface
echo "Configuring CAN interface..."
sudo ip link set $CAN_INTERFACE type can bitrate $BITRATE

echo "Bringing up CAN interface..."
sudo ip link set $CAN_INTERFACE up

# Verify interface is up
if ip link show $CAN_INTERFACE | grep -q "UP"; then
    echo "✓ CAN interface $CAN_INTERFACE is UP with bitrate $BITRATE"
else
    echo "✗ Failed to bring up CAN interface"
    exit 1
fi

# Show interface status
echo ""
echo "Interface status:"
ip link show $CAN_INTERFACE

echo ""
echo "CAN interface setup complete!"
echo ""
echo "Test commands:"
echo "  # Monitor CAN traffic:"
echo "  candump $CAN_INTERFACE"
echo ""
echo "  # Send test frame:"
echo "  cansend $CAN_INTERFACE 123#DEADBEEF"
echo ""
echo "  # Test with RefBoard:"
echo "  ros2 run refboard_base test_mks_can $CAN_INTERFACE"
