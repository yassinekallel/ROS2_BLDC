# 🚀 COMPLETE SETUP CHECKLIST

## Step 1: Files to Push to Git (From Windows)

### ✅ Files that SHOULD be in your Git repository:

**Core ROS2 Packages:**
```
refboard_base/
├── include/mks_protocol.hpp
├── include/mks_can_interface.hpp
├── src/mks_protocol.cpp
├── src/mks_can_interface.cpp
├── test/test_mks_can.cpp
├── CMakeLists.txt
└── package.xml

refboard_ros2_control/
├── include/refboard_hardware_interface.hpp
├── src/refboard_hardware_interface.cpp
├── config/refboard_controllers.yaml
├── description/refboard_robot.urdf.xacro
├── description/refboard_ros2_control.xacro
├── launch/refboard_ros2_control.launch.py
├── CMakeLists.txt
├── package.xml
├── refboard_hardware_interface.xml
└── README.md
```

**Scripts and Documentation:**
```
scripts/
├── install_dependencies.sh
├── setup_can.sh
└── test_system.sh

docs/
├── hardware_setup.md
└── software_setup.md

README.md
.gitignore
LICENSE
GIT_SETUP_GUIDE.md
```

### ❌ Files that should NOT be pushed (in .gitignore):
- `build/` folders
- `install/` folders
- `log/` folders
- `joint_sine_publisher/` (not needed)
- `odrive_*` folders (original examples)
- `ros2_control_template/` (not needed)

## Step 2: Git Commands (Run in PowerShell on Windows)

```powershell
cd C:\Users\Kallel\Pictures\ws_odrive\src

# Initialize Git (if not already done)
git init

# Add all files
git add .

# Commit
git commit -m "RefBoard ROS2 Control - Direct BLDC motor control via CANable v2

- Complete MKS protocol implementation  
- ROS2 control hardware interface
- Direct motor control (no Arduino needed)
- Support for 6 motors via CAN IDs 0x01-0x06
- Position control with 36000 steps per revolution
- Compatible with CANable v2 USB-CAN adapter"

# Add remote (replace YOUR_USERNAME with your GitHub username)
git remote add origin https://github.com/YOUR_USERNAME/refboard_ros2_control.git

# Push to GitHub
git branch -M main
git push -u origin main
```

## Step 3: Setup on Raspberry Pi (Ubuntu Jammy)

### 🔧 Quick One-Line Setup:
```bash
curl -sSL https://raw.githubusercontent.com/YOUR_USERNAME/refboard_ros2_control/main/scripts/install_dependencies.sh | bash
```

### 📋 Manual Setup Steps:
```bash
# 1. Clone repository
mkdir -p ~/refboard_ws/src
cd ~/refboard_ws/src
git clone https://github.com/YOUR_USERNAME/refboard_ros2_control.git

# 2. Install dependencies and build
cd refboard_ros2_control
chmod +x scripts/*.sh
./scripts/install_dependencies.sh

# 3. Build workspace
cd ~/refboard_ws
colcon build
source install/setup.bash

# 4. Connect CANable v2 to USB and setup CAN
sudo ip link set can0 up type can bitrate 500000

# 5. Test system
cd ~/refboard_ws/src/refboard_ros2_control
./scripts/test_system.sh
```

## Step 4: Hardware Connections

### 🔌 Physical Setup:
```
[Raspberry Pi] ──USB──> [CANable v2] ──CAN──> [Motor Controllers] ──> [BLDC Motors]
                                │                    │
                           CANH/CANL            CAN IDs 0x01-0x06
                                │                    │
                           [120Ω Term]         [120Ω Term]
```

### ⚙️ Motor Controller Settings:
- **CAN Bitrate**: 500kbps
- **CAN IDs**: 0x01, 0x02, 0x03, 0x04, 0x05, 0x06
- **Protocol**: MKS (same as your Arduino implementation)

## Step 5: Usage Examples

### 🎮 Basic Control:
```bash
# Launch the system
ros2 launch refboard_ros2_control refboard_ros2_control.launch.py can_interface:=can0

# Move motor to position (1 radian ≈ 57.3 degrees)
ros2 topic pub --once /refboard_joint_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory '{
  joint_names: ["joint_x"],
  points: [{positions: [1.57], time_from_start: {sec: 3}}]
}'

# Check joint states
ros2 topic echo /joint_states
```

### 🔧 Direct Motor Testing:
```bash
# Interactive control
ros2 run refboard_base test_mks_can can0

# Commands:
# e 1      - Enable motor 1
# s 1 1000 - Move motor 1 to position 1000 steps  
# p 1      - Read position from motor 1
# d 1      - Disable motor 1
# quit     - Exit
```

## Step 6: What Makes This Different

### 🔥 Key Advantages:
- **No Arduino needed** - ROS2 sends MKS commands directly
- **Standard ROS2 control** - Works with MoveIt, Navigation, etc.
- **Real-time capable** - Direct CAN communication
- **Scalable** - Easy to add more motors
- **Compatible** - Uses exact same MKS protocol as your Arduino

### 📊 Comparison:
| Before (Arduino) | After (ROS2 Direct) |
|-----------------|-------------------|
| PC → Arduino → CAN → Motors | PC → CANable → CAN → Motors |
| G-code commands | ROS2 trajectories |
| Serial communication | USB-CAN |
| Manual control | Automated/programmatic |
| Single-threaded | Multi-threaded |

## Step 7: Troubleshooting

### 🚨 Common Issues:
1. **CANable not detected**: `lsusb | grep -i can`
2. **CAN interface down**: `sudo ip link set can0 up type can bitrate 500000`
3. **No motor response**: Check CAN wiring and termination
4. **Build errors**: Clean and rebuild workspace

### 📞 Get Help:
- Check logs: `ros2 launch refboard_ros2_control refboard_ros2_control.launch.py --ros-args --log-level DEBUG`
- Monitor CAN: `candump can0`
- Test individual motors: `ros2 run refboard_base test_mks_can can0`

---

## 🎯 Summary

You now have a complete ROS2 system that:
- ✅ Replaces your Arduino completely
- ✅ Controls BLDC motors directly via CANable v2
- ✅ Uses the exact same MKS protocol
- ✅ Integrates with standard ROS2 ecosystem
- ✅ Provides both low-level and high-level control interfaces

**Next step**: Push to Git and deploy on your Raspberry Pi! 🚀
