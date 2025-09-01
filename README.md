# RefBoard ROS2 Control - Direct Motor Control

This package provides direct control of BLDC motors using the MKS protocol over CAN bus through CANable v2, **replacing the Arduino completely**.

## System Architecture

```
[ROS2 Node] → [MKS Protocol] → [CANable v2] → [CAN Bus] → [BLDC Motors with MKS Controllers]
```

**No Arduino needed!** The ROS2 system directly sends MKS protocol commands to your motor controllers.

## Hardware Setup

### Required Components
- **Raspberry Pi** (or PC with Ubuntu 22.04)
- **CANable v2** USB-CAN adapter  
- **BLDC Motors** with MKS protocol controllers
- **CAN Bus wiring** (CANH/CANL with 120Ω termination)

### Connections
```
[Raspberry Pi] ─USB─ [CANable v2] ─CAN Bus─ [Motor Controllers] ─ [BLDC Motors]
                              │                     │
                         (CANH/CANL)           (CAN IDs 0x01-0x06)
```

## Quick Start

### 1. Install Dependencies
```bash
# On Ubuntu/Raspberry Pi:
curl -sSL https://raw.githubusercontent.com/yourusername/refboard_ros2_control/main/scripts/install_dependencies.sh | bash
```

### 2. Clone and Build
```bash
mkdir -p ~/refboard_ws/src
cd ~/refboard_ws/src
git clone https://github.com/yourusername/refboard_ros2_control.git
cd ..
colcon build --packages-select refboard_base refboard_ros2_control
source install/setup.bash
```

### 3. Setup CAN Interface
```bash
sudo ip link set can0 up type can bitrate 500000
```

### 4. Test Direct Motor Control
```bash
# Test individual motor commands (replaces Arduino functionality)
ros2 run refboard_base test_mks_can can0

# Commands to try:
# e 1        - Enable motor 1
# s 1 1000   - Set motor 1 to position 1000 steps  
# p 1        - Read position from motor 1
```

### 5. Launch ROS2 Control
```bash
ros2 launch refboard_ros2_control refboard_ros2_control.launch.py can_interface:=can0
```

### 6. Send Commands
```bash
# Send joint trajectory
ros2 topic pub /refboard_joint_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory '{
  joint_names: ["joint_x", "joint_y"],
  points: [{
    positions: [1.0, 0.5],
    time_from_start: {sec: 3}
  }]
}'

# Monitor joint states
ros2 topic echo /joint_states
```

## Detailed Setup Guide

See [docs/software_setup.md](docs/software_setup.md) for complete instructions.

## Hardware Setup

See [docs/hardware_setup.md](docs/hardware_setup.md) for wiring and configuration.

## Testing

See [docs/troubleshooting.md](docs/troubleshooting.md) for step-by-step testing procedures.

## License

MIT License - see LICENSE file for details.
# ROS2_BLDC
