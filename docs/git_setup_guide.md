# Git Repository Setup and Testing Guide

## Files to Push to Git Repository

### Essential Files (must include)
```
refboard_ros2_control/
├── refboard_base/
│   ├── include/
│   │   ├── mks_protocol.hpp                 ✓ Push
│   │   └── mks_can_interface.hpp            ✓ Push  
│   ├── src/
│   │   ├── mks_protocol.cpp                 ✓ Push
│   │   └── mks_can_interface.cpp            ✓ Push
│   ├── test/
│   │   └── test_mks_can.cpp                 ✓ Push
│   ├── CMakeLists.txt                       ✓ Push
│   └── package.xml                          ✓ Push
├── refboard_ros2_control/
│   ├── include/
│   │   └── refboard_hardware_interface.hpp  ✓ Push
│   ├── src/
│   │   └── refboard_hardware_interface.cpp  ✓ Push
│   ├── config/
│   │   └── refboard_controllers.yaml        ✓ Push
│   ├── description/
│   │   ├── refboard_robot.urdf.xacro        ✓ Push
│   │   └── refboard_ros2_control.xacro      ✓ Push
│   ├── launch/
│   │   └── refboard_ros2_control.launch.py  ✓ Push
│   ├── CMakeLists.txt                       ✓ Push
│   ├── package.xml                          ✓ Push
│   ├── refboard_hardware_interface.xml      ✓ Push
│   └── README.md                            ✓ Push
├── scripts/
│   ├── install_dependencies.sh              ✓ Push
│   ├── setup_can.sh                         ✓ Push
│   └── test_system.sh                       ✓ Push
├── docs/
│   ├── hardware_setup.md                    ✓ Push
│   ├── software_setup.md                    ✓ Push
│   └── git_setup_guide.md                   ✓ Push (this file)
├── .gitignore                               ✓ Push
├── README.md                                ✓ Push
└── LICENSE                                  ✓ Push
```

### Files NOT to Push
```
❌ build/                    # Build artifacts
❌ install/                  # Install artifacts  
❌ log/                      # Log files
❌ *.bag, *.bag2/           # ROS2 bag files
❌ arduino_firmware/        # Not needed for direct control
❌ odrive_base/             # Original ODrive code (for reference only)
❌ odrive_ros2_control/     # Original ODrive code (for reference only)
❌ joint_sine_publisher/    # Original example code
❌ ros2_control_template/   # Original template code
```

## Step-by-Step Git Setup

### 1. Initialize Repository
```bash
cd /path/to/your/refboard_ws/src

# Initialize git repository
git init

# Add .gitignore
git add .gitignore
git commit -m "Initial commit: Add .gitignore"
```

### 2. Add Essential Files
```bash
# Add all RefBoard packages
git add refboard_base/ refboard_ros2_control/

# Add scripts and documentation
git add scripts/ docs/ README.md

# Create LICENSE file (choose your license)
echo "MIT License" > LICENSE
echo "Copyright (c) 2025 Your Name" >> LICENSE
echo "..." >> LICENSE  # Add full license text
git add LICENSE

# Commit everything
git commit -m "Add RefBoard ROS2 control packages

- refboard_base: Low-level MKS protocol implementation
- refboard_ros2_control: ROS2 control hardware interface
- Direct motor control via CANable v2 (no Arduino needed)
- Complete documentation and setup scripts"
```

### 3. Create GitHub Repository
```bash
# Create repository on GitHub first, then:
git remote add origin https://github.com/yourusername/refboard_ros2_control.git
git branch -M main
git push -u origin main
```

## Complete Testing Guide

### Prerequisites Test
```bash
# 1. Check hardware connections
lsusb | grep -i can                    # Should show CANable v2
ip link show | grep can                # Should show can0

# 2. Check ROS2 installation
ros2 --version                         # Should show ROS2 Humble

# 3. Check workspace build
cd ~/refboard_ws
colcon build --packages-select refboard_base refboard_ros2_control
source install/setup.bash
```

### Level 1: CAN Interface Test
```bash
# Setup CAN interface
sudo ip link set can0 up type can bitrate 500000

# Test 1: CAN loopback
candump can0 &                         # Start monitoring
cansend can0 123#DEADBEEF              # Send test frame
# You should see the frame in candump output
pkill candump
```

### Level 2: Direct Motor Communication Test
```bash
# Test 2: MKS protocol test program
ros2 run refboard_base test_mks_can can0

# Try these commands in order:
# p 1        # Read position from motor 1 (should get response)
# e 1        # Enable motor 1 (should get ACK)
# s 1 100    # Move motor 1 to position 100 (should move)
# p 1        # Read position again (should show new position)
# d 1        # Disable motor 1
# quit       # Exit program
```

**Expected Results:**
- Position read should return current motor position
- Enable command should get ACK response from motor
- Set position should cause motor to move
- No Arduino involved - direct CAN communication

### Level 3: ROS2 Control Integration Test
```bash
# Test 3: Full ROS2 control stack
ros2 launch refboard_ros2_control refboard_ros2_control.launch.py can_interface:=can0

# In another terminal:
# Check controllers loaded
ros2 control list_controllers

# Check joint states
ros2 topic echo /joint_states

# Send trajectory command
ros2 topic pub --once /refboard_joint_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory '{
  joint_names: ["joint_x"],
  points: [{
    positions: [0.5],
    velocities: [0.0],
    time_from_start: {sec: 2}
  }]
}'
```

**Expected Results:**
- Controllers should load successfully
- Joint states should show current motor positions
- Trajectory command should move motor to new position

### Level 4: Multi-Motor Test
```bash
# Test 4: Control multiple motors
ros2 topic pub --once /refboard_joint_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory '{
  joint_names: ["joint_x", "joint_y", "joint_z"],
  points: [{
    positions: [1.0, -0.5, 0.8],
    time_from_start: {sec: 3}
  }]
}'
```

### Level 5: Integration Test
```bash
# Test 5: Use with standard ROS2 tools
# Joint state publisher GUI
ros2 run joint_state_publisher_gui joint_state_publisher_gui

# RViz visualization
rviz2 -d $(ros2 pkg prefix refboard_ros2_control)/share/refboard_ros2_control/rviz/refboard.rviz
```

## Troubleshooting Guide

### Problem: CAN interface not found
```bash
# Check USB connection
lsusb | grep -i can
dmesg | tail | grep -i can

# Solution: Reconnect CANable v2, try different USB port
```

### Problem: No response from motors
```bash
# Check CAN traffic
candump can0

# Check termination resistors (120Ω at both ends)
# Verify motor CAN IDs (0x01-0x06)
# Check bitrate matches (500kbps)
```

### Problem: Motors move incorrectly
```bash
# Check position scaling
# Verify motor direction configuration
# Check gear ratios in URDF
```

### Problem: ROS2 controllers fail to load
```bash
# Check hardware interface
ros2 control list_hardware_interfaces

# Enable debug logging
export RCUTILS_LOGGING_SEVERITY=DEBUG
ros2 launch refboard_ros2_control refboard_ros2_control.launch.py
```

## Repository Maintenance

### Regular Updates
```bash
# Update documentation
git add docs/
git commit -m "Update documentation"

# Add new features
git add .
git commit -m "Add new feature: description"
git push
```

### Version Tagging
```bash
# Tag stable versions
git tag -a v1.0.0 -m "Release version 1.0.0 - Initial stable release"
git push origin v1.0.0
```

### Issue Tracking
Use GitHub Issues to track:
- Motor compatibility issues
- Performance improvements
- Feature requests
- Bug reports

This setup gives you a complete, Arduino-free system for controlling BLDC motors with ROS2!
