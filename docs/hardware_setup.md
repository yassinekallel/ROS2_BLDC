# Hardware Setup Guide

This guide covers the physical setup and wiring for the RefBoard ROS2 control system.

## Components Required

### Essential Hardware
- **Raspberry Pi 4** (or compatible computer with Ubuntu 22.04)
- **CANable v2 USB-CAN adapter**
- **Arduino/XMC board** with onboard CAN transceiver
- **RefBoard BLDC motors** (up to 6 motors)
- **CAN cables** (twisted pair, preferably with shielding)
- **120Ω termination resistors** (2 pieces)
- **Power supplies** (for Pi, Arduino, and motors)

### Optional Hardware
- **USB cables** for programming and debugging
- **Oscilloscope** for CAN signal debugging
- **Multimeter** for power verification

## Wiring Diagram

```
[Raspberry Pi] ─USB─ [CANable v2] ─CAN Bus─ [Arduino/XMC] ─Motor Drivers─ [BLDC Motors]
                              │                    │
                          CANH/CANL            CANH/CANL
                              │                    │
                         [120Ω Term]        [120Ω Term]
```

## Step-by-Step Setup

### 1. Raspberry Pi Setup

1. **Install Ubuntu 22.04** on Raspberry Pi
   - Download Ubuntu 22.04 LTS for Raspberry Pi
   - Flash to SD card using Raspberry Pi Imager
   - Boot and complete initial setup

2. **Enable SSH** (optional)
   ```bash
   sudo systemctl enable ssh
   sudo systemctl start ssh
   ```

3. **Update system**
   ```bash
   sudo apt update
   sudo apt upgrade -y
   ```

### 2. CANable v2 Setup

1. **Connect CANable v2** to Raspberry Pi USB port

2. **Verify detection**
   ```bash
   lsusb | grep -i can
   dmesg | grep -i can
   ```
   You should see the CANable device listed.

3. **Check CAN interface**
   ```bash
   ip link show
   ```
   You should see `can0` interface.

### 3. Arduino/XMC Setup

1. **Program Arduino/XMC**
   - Use Arduino IDE or platform-specific tools
   - Flash the RefBoard CAN controller firmware
   - Verify serial output shows "CAN Receiver" message

2. **Verify CAN transceiver**
   - Check that your board has onboard CAN transceiver
   - Common chips: MCP2515, MCP2551, SN65HVD230

### 4. CAN Bus Wiring

**Important**: Use proper CAN bus wiring practices!

1. **Cable Requirements**
   - Use twisted pair cable (preferably CAN-specific cable)
   - Maximum length: 40m at 1Mbps, 500m at 125kbps
   - For our 500kbps: maximum ~100m

2. **Wiring Connections**
   ```
   CANable v2          Arduino/XMC
   CANH (Pin 7) ────── CANH
   CANL (Pin 2) ────── CANL
   GND (Pin 3)  ────── GND
   ```

3. **Termination**
   - Install 120Ω resistor between CANH and CANL at BOTH ends
   - CANable v2 end: Connect 120Ω between pins 7 and 2
   - Arduino end: Connect 120Ω between CANH and CANL

### 5. Motor Connections

1. **Power Supply**
   - Connect appropriate power supply to motors
   - Ensure Arduino/XMC has adequate power
   - Common voltages: 12V, 24V, 48V (check motor specs)

2. **Motor Driver Setup**
   - Configure motor drivers for CAN communication
   - Set CAN IDs: 0x01, 0x02, 0x03, 0x04, 0x05, 0x06
   - Set CAN bitrate to 500kbps

3. **Safety Considerations**
   - Install emergency stop switches
   - Ensure proper grounding
   - Use appropriate fuses/circuit breakers

## Verification Steps

### 1. Power-On Sequence

1. **Power Raspberry Pi** first
2. **Connect CANable v2**
3. **Power Arduino/XMC**
4. **Power motor drivers** last

### 2. Basic Connectivity Test

1. **Check USB devices**
   ```bash
   lsusb
   ```

2. **Check CAN interface**
   ```bash
   ip link show can0
   ```

3. **Setup CAN interface**
   ```bash
   sudo ip link set can0 up type can bitrate 500000
   ```

4. **Monitor CAN traffic**
   ```bash
   candump can0
   ```

### 3. Arduino Communication Test

1. **Connect to Arduino serial**
   ```bash
   sudo screen /dev/ttyUSB0 115200
   ```

2. **Send test commands**
   - `$EX` - Enable joint X
   - `$MX` - Read position of joint X
   - `X1000` - Move joint X to position 1000

3. **Verify CAN frames**
   - Run `candump can0` in another terminal
   - Should see CAN frames when sending commands

## Troubleshooting

### CAN Interface Issues

**Problem**: `can0` interface not appearing
- **Check**: CANable v2 USB connection
- **Solution**: Try different USB port, check `dmesg` for errors

**Problem**: "Network is down" error
- **Check**: Interface configuration
- **Solution**: `sudo ip link set can0 up type can bitrate 500000`

### CAN Communication Issues

**Problem**: No CAN frames visible with `candump`
- **Check**: Termination resistors (120Ω at both ends)
- **Check**: Wiring (CANH/CANL not swapped)
- **Check**: Arduino firmware running

**Problem**: CAN errors or timeouts
- **Check**: Bitrate matches (500kbps)
- **Check**: Cable length and quality
- **Check**: Electrical interference

### Motor Issues

**Problem**: Motors not responding
- **Check**: Motor power supply
- **Check**: CAN ID configuration
- **Check**: Motor driver settings

**Problem**: Position feedback incorrect
- **Check**: Encoder connections
- **Check**: Motor direction configuration
- **Check**: Gear ratio settings

## Safety Warnings

⚠️ **ELECTRICAL SAFETY**
- Always disconnect power before making connections
- Use appropriate PPE (safety glasses, insulated tools)
- Verify voltages with multimeter before connecting

⚠️ **MECHANICAL SAFETY**
- Ensure motors are securely mounted
- Keep hands clear of moving parts
- Test with low speeds initially

⚠️ **CAN BUS SAFETY**
- Never hot-plug CAN connections under power
- Use proper termination to prevent signal reflections
- Shield cables in electrically noisy environments

## Additional Resources

- [CANable v2 Documentation](https://canable.io/)
- [CAN Bus Fundamentals](https://www.kvaser.com/about-can/)
- [Linux SocketCAN Documentation](https://www.kernel.org/doc/html/latest/networking/can.html)
