# Git Repository Setup and Deployment Guide

## What to Push to Git

Here's exactly what files you need to include in your Git repository:

### Core Packages
```
refboard_ros2_control/
├── refboard_base/
│   ├── include/
│   │   ├── mks_protocol.hpp
│   │   └── mks_can_interface.hpp
│   ├── src/
│   │   ├── mks_protocol.cpp
│   │   └── mks_can_interface.cpp
│   ├── test/
│   │   └── test_mks_can.cpp
│   ├── CMakeLists.txt
│   └── package.xml
├── refboard_ros2_control/
│   ├── include/
│   │   └── refboard_hardware_interface.hpp
│   ├── src/
│   │   └── refboard_hardware_interface.cpp
│   ├── config/
│   │   └── refboard_controllers.yaml
│   ├── description/
│   │   ├── refboard_robot.urdf.xacro
│   │   └── refboard_ros2_control.xacro
│   ├── launch/
│   │   └── refboard_ros2_control.launch.py
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── refboard_hardware_interface.xml
│   └── README.md
├── scripts/
│   ├── install_dependencies.sh
│   ├── setup_can.sh
│   └── test_system.sh
├── docs/
│   ├── hardware_setup.md
│   └── software_setup.md
├── .gitignore
├── README.md
└── LICENSE
```

### DO NOT PUSH (already in .gitignore)
- `build/` folders
- `install/` folders  
- `log/` folders
- Any temporary files
- IDE-specific files

## Step 1: Create Git Repository

### On Windows (your current setup)

1. **Initialize Git repository**:
   ```powershell
   cd C:\Users\Kallel\Pictures\ws_odrive\src
   git init
   git add .
   git commit -m "Initial commit: RefBoard ROS2 Control system"
   ```

2. **Create repository on GitHub** (or GitLab/Bitbucket):
   - Go to GitHub.com
   - Click "New repository"
   - Name: `refboard_ros2_control`
   - Description: "Direct BLDC motor control using MKS protocol via CANable v2"
   - Public/Private as needed
   - Don't initialize with README (you already have one)

3. **Push to remote**:
   ```powershell
   git remote add origin https://github.com/YOUR_USERNAME/refboard_ros2_control.git
   git branch -M main
   git push -u origin main
   ```

## Step 2: Deploy on Raspberry Pi (Ubuntu Jammy)

### Quick Setup (One-liner)
```bash
# On your Raspberry Pi
curl -sSL https://raw.githubusercontent.com/YOUR_USERNAME/refboard_ros2_control/main/scripts/install_dependencies.sh | bash
```

### Manual Setup

1. **Update system**:
   ```bash
   sudo apt update && sudo apt upgrade -y
   ```

2. **Clone repository**:
   ```bash
   mkdir -p ~/refboard_ws/src
   cd ~/refboard_ws/src
   git clone https://github.com/YOUR_USERNAME/refboard_ros2_control.git
   ```

3. **Install dependencies**:
   ```bash
   cd refboard_ros2_control
   chmod +x scripts/*.sh
   ./scripts/install_dependencies.sh
   ```

4. **Build workspace**:
   ```bash
   cd ~/refboard_ws
   colcon build --packages-select refboard_base refboard_ros2_control
   source install/setup.bash
   ```

5. **Setup CAN interface**:
   ```bash
   # Connect your CANable v2 to USB first
   sudo ip link set can0 up type can bitrate 500000
   ```

6. **Test the system**:
   ```bash
   ./src/refboard_ros2_control/scripts/test_system.sh
   ```

## Step 3: Hardware Connection on Raspberry Pi

### Physical Setup
```
[Raspberry Pi] ─USB─ [CANable v2] ─CAN Bus─ [BLDC Motor Controllers] ─ [Motors]
                              │                      │
                         (CANH/CANL)            (CAN IDs 0x01-0x06)
                              │                      │
                          [120Ω Term]           [120Ω Term]
```

### Wiring Checklist
- ✅ CANable v2 connected to Raspberry Pi USB
- ✅ CANH/CANL wires from CANable to motor controllers  
- ✅ 120Ω termination resistors at both ends of CAN bus
- ✅ Motor controllers configured with CAN IDs 0x01-0x06
- ✅ Motor controllers set to 500kbps CAN bitrate
- ✅ Motor power supplies connected

## Step 4: Testing and Validation

### Test 1: CAN Interface
```bash
# Check CANable v2 detection
lsusb | grep -i can

# Check CAN interface
ip link show can0

# Monitor CAN traffic
candump can0
```

### Test 2: Direct Motor Control
```bash
# Interactive test program
ros2 run refboard_base test_mks_can can0

# Commands to try:
# p 1        - Read position from motor 1
# e 1        - Enable motor 1  
# s 1 1000   - Set motor 1 to position 1000 steps
# d 1        - Disable motor 1
```

### Test 3: ROS2 Control
```bash
# Launch complete system
ros2 launch refboard_ros2_control refboard_ros2_control.launch.py can_interface:=can0

# Check joint states
ros2 topic echo /joint_states

# Send trajectory command
ros2 topic pub /refboard_joint_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory '{
  joint_names: ["joint_x"],
  points: [{positions: [1.0], time_from_start: {sec: 2}}]
}'
```

## Step 5: Usage Examples

### Basic Position Control
```bash
# Move single joint
ros2 topic pub --once /refboard_joint_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory '{
  joint_names: ["joint_x"],
  points: [{positions: [1.57], time_from_start: {sec: 3}}]
}'

# Move multiple joints
ros2 topic pub --once /refboard_joint_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory '{
  joint_names: ["joint_x", "joint_y", "joint_z"],
  points: [{positions: [1.0, 0.5, -0.5], time_from_start: {sec: 5}}]
}'
```

### Monitor System Status
```bash
# Joint states
ros2 topic echo /joint_states

# Controller status  
ros2 control list_controllers

# Hardware interfaces
ros2 control list_hardware_interfaces
```

### Emergency Operations
```bash
# Emergency stop (through test program)
ros2 run refboard_base test_mks_can can0
# Then type: stopall

# Or stop ROS2 control
ros2 lifecycle set /controller_manager shutdown
```

## Troubleshooting

### Common Issues

1. **CANable v2 not detected**:
   ```bash
   lsusb | grep -i can
   dmesg | grep -i can
   ```

2. **CAN interface won't come up**:
   ```bash
   sudo ip link set can0 down
   sudo ip link set can0 up type can bitrate 500000
   ```

3. **No response from motors**:
   - Check CAN wiring (CANH/CANL)
   - Verify termination resistors (120Ω)
   - Check motor CAN IDs (0x01-0x06)
   - Verify motor controller settings

4. **ROS2 build errors**:
   ```bash
   # Clean build
   cd ~/refboard_ws
   rm -rf build install log
   colcon build --packages-select refboard_base refboard_ros2_control
   ```

## Git Workflow for Updates

### Making Changes
```bash
# On Windows (development)
git add .
git commit -m "Description of changes"
git push origin main

# On Raspberry Pi (deployment)
cd ~/refboard_ws/src/refboard_ros2_control
git pull origin main
cd ~/refboard_ws
colcon build --packages-select refboard_base refboard_ros2_control
source install/setup.bash
```

## Auto-Start on Boot (Optional)

Create systemd service:
```bash
sudo nano /etc/systemd/system/refboard-control.service
```

Content:
```ini
[Unit]
Description=RefBoard ROS2 Control
After=network.target

[Service]
Type=forking
User=pi
ExecStartPre=/bin/ip link set can0 up type can bitrate 500000
ExecStart=/bin/bash -c 'cd /home/pi/refboard_ws && source install/setup.bash && ros2 launch refboard_ros2_control refboard_ros2_control.launch.py can_interface:=can0'
Restart=always

[Install]
WantedBy=multi-user.target
```

Enable:
```bash
sudo systemctl enable refboard-control.service
sudo systemctl start refboard-control.service
```

Your system will now start controlling motors automatically on boot!
