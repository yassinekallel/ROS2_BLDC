# RefBoard ROS2 Control

This package provides a ROS2 control hardware interface for controlling BLDC motors using the MKS protocol over CAN bus, designed to work with your custom RefBoard controllers and CANable v2 devices.

## Architecture

The package consists of two main components:

1. **refboard_base**: Low-level C++ library implementing the MKS protocol over CAN
2. **refboard_ros2_control**: ROS2 control hardware interface plugin

## Features

- **MKS Protocol Support**: Complete implementation of your Arduino MKS protocol
- **Position Control**: Primary control mode for precise motor positioning
- **Multi-Joint Support**: Controls up to 6 motors (joints X, Y, Z, A, B, C)
- **CAN Bus Communication**: Uses SocketCAN for reliable communication
- **ROS2 Control Integration**: Standard ros2_control interfaces for easy integration
- **Synchronous and Asynchronous Operations**: Blocking and non-blocking motor commands

## Hardware Setup

1. **Arduino/XMC Setup**:
   - Flash your Arduino/XMC board with the MKS protocol firmware
   - Connect CAN transceiver to your motors
   - Ensure motors are properly configured with CAN IDs 0x01-0x06

2. **Raspberry Pi Setup**:
   - Connect CANable v2 device to USB
   - Set up CAN interface:
     ```bash
     sudo ip link set can0 up type can bitrate 500000
     ```

3. **Physical Connections**:
   - Connect Arduino CAN bus to CANable v2 CAN bus (CANH/CANL)
   - Ensure proper termination (120Ω resistors at each end)
   - Power both devices appropriately

## Installation and Building

1. **Prerequisites**:
   ```bash
   # Install ROS2 (if not already installed)
   # Install ros2_control packages
   sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers
   sudo apt install ros-humble-joint-state-publisher-gui
   sudo apt install can-utils
   ```

2. **Build the workspace**:
   ```bash
   cd /path/to/your/workspace
   colcon build --packages-select refboard_base refboard_ros2_control
   source install/setup.bash
   ```

## Testing

### 1. Test CAN Communication

First, test the low-level CAN interface:

```bash
# Run the test program
ros2 run refboard_base test_mks_can can0

# Example commands in the test program:
# e 1        - Enable motor 1
# p 1        - Read position of motor 1  
# s 1 1000   - Set position of motor 1 to 1000 steps
# d 1        - Disable motor 1
```

### 2. Test with can-utils

Monitor CAN traffic:
```bash
# Listen to all CAN frames
candump can0

# Send test frame (enable motor 1)
cansend can0 01#F3010000000001F4
```

### 3. Test ROS2 Control Integration

Launch the complete system:
```bash
# Launch the RefBoard control system
ros2 launch refboard_ros2_control refboard_ros2_control.launch.py can_interface:=can0

# In another terminal, check joint states
ros2 topic echo /joint_states

# Send position commands
ros2 topic pub /refboard_joint_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory '{
  joint_names: ["joint_x"],
  points: [{
    positions: [1.0],
    time_from_start: {sec: 2}
  }]
}'
```

## Configuration

### Joint Mapping

The system maps ROS2 joint names to MKS CAN IDs:

| Joint Name | CAN ID | Description |
|------------|---------|-------------|
| joint_x    | 0x01   | X-axis motor |
| joint_y    | 0x02   | Y-axis motor |
| joint_z    | 0x03   | Z-axis motor |
| joint_a    | 0x04   | A-axis motor |
| joint_b    | 0x05   | B-axis motor |
| joint_c    | 0x06   | C-axis motor |

### Control Parameters

- **Position Units**: Radians (converted to/from steps internally)
- **Step Resolution**: 36,000 steps per revolution
- **Default Speed**: 1024 (MKS protocol units)
- **Update Rate**: 100 Hz (configurable)

### Hardware Parameters

In your URDF, specify the CAN interface:

```xml
<ros2_control name="RefBoardHardwareInterface" type="system">
  <hardware>
    <plugin>refboard_ros2_control/RefBoardHardwareInterface</plugin>
    <param name="can_interface">can0</param>
  </hardware>
  <!-- joints... -->
</ros2_control>
```

## MKS Protocol Details

The implementation supports these MKS commands:

- **0xF5**: Position Command (with speed and position data)
- **0xF3**: Enable/Disable Motor
- **0x31**: Read Position
- **0x32**: Read Velocity  
- **0xF7**: Emergency Stop
- **0xF8**: Calibration

### Message Format

All CAN frames are 8 bytes:
- Byte 0: Command ID
- Bytes 1-6: Command-specific data
- Byte 7: CRC (sum of bytes 0-6)

## Troubleshooting

### CAN Interface Issues

1. **Check CAN interface status**:
   ```bash
   ip link show can0
   ```

2. **Verify CAN traffic**:
   ```bash
   candump can0
   ```

3. **Check for errors**:
   ```bash
   cat /sys/class/net/can0/statistics/rx_errors
   ```

### Motor Communication Issues

1. **Enable debug logging**:
   ```bash
   ros2 launch refboard_ros2_control refboard_ros2_control.launch.py can_interface:=can0 --ros-args --log-level DEBUG
   ```

2. **Test individual motors**:
   ```bash
   ros2 run refboard_base test_mks_can can0
   ```

3. **Check motor responses**:
   - Motors should send ACK responses to commands
   - Position reads should return valid data
   - Enable commands should succeed

### ROS2 Control Issues

1. **Check controller status**:
   ```bash
   ros2 control list_controllers
   ```

2. **Monitor hardware interface**:
   ```bash
   ros2 control list_hardware_interfaces
   ```

3. **Check joint states**:
   ```bash
   ros2 topic echo /joint_states
   ```

## Advanced Usage

### Custom Controllers

You can create custom controllers using the standard ros2_control interfaces:

```cpp
// Example: Custom position controller
#include "controller_interface/controller_interface.hpp"

class MyCustomController : public controller_interface::ControllerInterface {
  // Implementation...
};
```

### Multiple Robots

To control multiple RefBoard systems:

1. Use different CAN interfaces (can0, can1, etc.)
2. Create separate hardware interface instances
3. Use namespaces for topic isolation

### Integration with MoveIt

The RefBoard interface is compatible with MoveIt for motion planning:

1. Generate MoveIt configuration for your robot
2. Use the joint_trajectory_controller
3. Configure planning groups for your joints

## API Reference

### MksCanInterface Class

Key methods:
- `bool init(const std::string& can_interface)`
- `bool setPosition(JointId joint_id, int32_t position, uint16_t speed)`
- `bool enableMotor(JointId joint_id)`
- `bool readPosition(JointId joint_id)`

### RefBoardHardwareInterface Class

Standard ros2_control SystemInterface implementation:
- Exports position, velocity, and effort interfaces
- Supports position command mode
- Handles CAN communication automatically

## Contributing

To extend functionality:

1. Add new MKS commands to `mks_protocol.hpp`
2. Implement command handlers in `mks_can_interface.cpp`
3. Update hardware interface if needed
4. Add tests for new features

## License

MIT License - see LICENSE file for details.
