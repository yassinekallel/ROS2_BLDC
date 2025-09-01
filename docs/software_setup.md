# Software Setup Guide - Direct Motor Control

This guide covers installing and configuring ROS2 to directly control BLDC motors using CANable v2.

## Prerequisites

- **Ubuntu 22.04** (Raspberry Pi or PC)
- **CANable v2** USB-CAN adapter
- **BLDC Motors** with MKS protocol controllers (CAN IDs 0x01-0x06)
- **Internet connection** for downloading packages

## Step 1: Install ROS2 and Dependencies

### Automated Installation
```bash
curl -sSL https://raw.githubusercontent.com/yourusername/refboard_ros2_control/main/scripts/install_dependencies.sh | bash
```

### Manual Installation

1. **Install ROS2 Humble**
   ```bash
   # Add ROS2 repository
   sudo apt update
   sudo apt install software-properties-common
   sudo add-apt-repository universe
   
   # Install ROS2 key
   sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
   
   # Add repository
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
   
   # Install ROS2
   sudo apt update
   sudo apt install ros-humble-desktop
   sudo apt install ros-dev-tools
   ```

2. **Install ROS2 Control**
   ```bash
   sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers
   sudo apt install ros-humble-joint-state-publisher-gui
   sudo apt install ros-humble-robot-state-publisher
   sudo apt install ros-humble-xacro
   ```

3. **Install CAN utilities**
   ```bash
   sudo apt install can-utils net-tools
   ```

4. **Source ROS2**
   ```bash
   echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```

## Step 2: Create Workspace and Clone Repository

```bash
# Create workspace
mkdir -p ~/refboard_ws/src
cd ~/refboard_ws/src

# Clone repository (replace with your actual repo URL)
git clone https://github.com/yourusername/refboard_ros2_control.git

# Build workspace
cd ~/refboard_ws
colcon build --packages-select refboard_base refboard_ros2_control

# Source workspace
source install/setup.bash
echo "source ~/refboard_ws/install/setup.bash" >> ~/.bashrc
```

## Step 3: Hardware Connection

1. **Connect CANable v2** to USB port
2. **Verify detection**:
   ```bash
   lsusb | grep -i can
   dmesg | tail | grep -i can
   ```
3. **Check CAN interface**:
   ```bash
   ip link show | grep can
   ```
   You should see `can0` interface.

## Step 4: Configure CAN Interface

```bash
# Set up CAN interface (500kbps to match motor controllers)
sudo ip link set can0 up type can bitrate 500000

# Verify interface is up
ip link show can0
```

For automatic setup on boot, add to `/etc/systemd/system/can-setup.service`:
```ini
[Unit]
Description=Setup CAN interface
After=network.target

[Service]
Type=oneshot
ExecStart=/bin/ip link set can0 up type can bitrate 500000
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
```

Then enable it:
```bash
sudo systemctl enable can-setup.service
```

## Step 5: Test Direct Motor Communication

### Test 1: CAN Interface
```bash
# Monitor CAN traffic
candump can0

# In another terminal, send test frame
cansend can0 01#F3010000000001F4
```

### Test 2: Low-level Motor Control
```bash
# Start interactive test program
ros2 run refboard_base test_mks_can can0

# Try these commands:
# p 1        - Read position from motor 1
# e 1        - Enable motor 1  
# s 1 1000   - Set motor 1 to position 1000 steps
# d 1        - Disable motor 1
# quit       - Exit
```

### Test 3: ROS2 Control
```bash
# Launch the complete system
ros2 launch refboard_ros2_control refboard_ros2_control.launch.py can_interface:=can0

# In another terminal, check joint states
ros2 topic echo /joint_states

# Send position command
ros2 topic pub /refboard_joint_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory '{
  joint_names: ["joint_x"],
  points: [{
    positions: [1.0],
    time_from_start: {sec: 2}
  }]
}'
```

## Step 6: Configure Your Motor Setup

### Motor CAN ID Configuration
Ensure your motors are configured with these CAN IDs:
- Motor X: 0x01
- Motor Y: 0x02  
- Motor Z: 0x03
- Motor A: 0x04
- Motor B: 0x05
- Motor C: 0x06

### Motor Controller Settings
- **CAN Bitrate**: 500kbps
- **Protocol**: MKS (as implemented in your Arduino code)
- **Position Units**: Steps (36,000 per revolution)
- **Default Speed**: 1024

## Advanced Configuration

### Custom Joint Names
Edit `refboard_ros2_control/description/refboard_ros2_control.xacro` to change joint names:
```xml
<joint name="your_joint_name">
  <command_interface name="position"/>
  <!-- ... -->
</joint>
```

### Controller Parameters
Edit `refboard_ros2_control/config/refboard_controllers.yaml`:
```yaml
refboard_joint_controller:
  ros__parameters:
    joints:
      - your_joint_1
      - your_joint_2
    # ... other parameters
```

### Hardware Interface Parameters
In your URDF, specify CAN interface:
```xml
<param name="can_interface">can0</param>
```

## Troubleshooting

### CAN Interface Issues
```bash
# Check interface status
ip link show can0

# Check for errors
cat /sys/class/net/can0/statistics/rx_errors
cat /sys/class/net/can0/statistics/tx_errors

# Reset interface if needed
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 500000
```

### Motor Communication Issues
```bash
# Enable debug logging
export RCUTILS_LOGGING_SEVERITY=DEBUG
ros2 launch refboard_ros2_control refboard_ros2_control.launch.py

# Monitor all CAN traffic
candump can0

# Test specific motor manually
cansend can0 01#31000000000031  # Read position from motor 1
```

### ROS2 Control Issues
```bash
# Check controller status
ros2 control list_controllers

# Check hardware interface
ros2 control list_hardware_interfaces

# Load controller manually
ros2 control load_controller refboard_joint_controller
ros2 control switch_controller --start refboard_joint_controller
```

## Integration Examples

### With MoveIt
```bash
# Install MoveIt
sudo apt install ros-humble-moveit

# Generate MoveIt config (replace with your robot)
ros2 run moveit_setup_assistant moveit_setup_assistant
```

### With Navigation
The joint controller is compatible with standard ROS2 navigation stacks.

### Custom Applications
Create custom nodes that publish to `/refboard_joint_controller/joint_trajectory` for application-specific control.

## Performance Tuning

### Update Rates
- **Controller Manager**: 100Hz (configurable in YAML)
- **CAN Bus**: Up to 1000 frames/second
- **Position Updates**: Limited by motor controller response time

### Latency Optimization
- Use real-time kernel for better performance
- Adjust controller update rates based on application needs
- Monitor CAN bus utilization

## Next Steps

1. **Test with your specific motors**
2. **Integrate with your application**
3. **Configure safety limits and emergency stops**
4. **Add sensor feedback if needed**
5. **Create custom controllers for specific tasks**
