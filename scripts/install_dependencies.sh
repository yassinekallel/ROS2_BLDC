#!/bin/bash

# RefBoard ROS2 Control - Dependency Installation Script
# This script installs all necessary dependencies for direct motor control via CANable v2

set -e

echo "=== RefBoard ROS2 Control - Installing Dependencies ==="
echo "This will set up ROS2 to directly control BLDC motors via CANable v2"
echo "(No Arduino required - ROS2 will send MKS protocol commands directly)"
echo ""

# Check if running on Ubuntu
if ! lsb_release -a 2>/dev/null | grep -q "Ubuntu"; then
    echo "Warning: This script is designed for Ubuntu. Proceeding anyway..."
fi

# Update package list
echo "Updating package list..."
sudo apt update

# Install basic dependencies
echo "Installing basic dependencies..."
sudo apt install -y \
    curl \
    git \
    build-essential \
    cmake \
    python3-pip \
    can-utils \
    net-tools

# Check if ROS2 is installed
if ! command -v ros2 &> /dev/null; then
    echo "ROS2 not found. Installing ROS2 Humble..."
    
    # Add ROS2 repository
    sudo apt install -y software-properties-common
    sudo add-apt-repository universe -y
    sudo apt update
    
    # Install ROS2 key
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    
    # Add ROS2 repository
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    
    # Update and install ROS2
    sudo apt update
    sudo apt install -y ros-humble-desktop
    
    # Install development tools
    sudo apt install -y ros-dev-tools
    
    # Source ROS2 in bashrc
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    
    echo "ROS2 Humble installed successfully!"
else
    echo "ROS2 already installed: $(ros2 --version)"
fi

# Install ROS2 control packages
echo "Installing ROS2 control packages..."
sudo apt install -y \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-joint-state-publisher-gui \
    ros-humble-robot-state-publisher \
    ros-humble-xacro \
    ros-humble-moveit \
    ros-humble-rviz2

# Install additional build dependencies
echo "Installing build dependencies..."
sudo apt install -y \
    libeigen3-dev \
    libboost-all-dev

# Create workspace if it doesn't exist
WORKSPACE_DIR="$HOME/refboard_ws"
if [ ! -d "$WORKSPACE_DIR" ]; then
    echo "Creating workspace directory: $WORKSPACE_DIR"
    mkdir -p "$WORKSPACE_DIR/src"
fi

# Source ROS2 for current session
source /opt/ros/humble/setup.bash

echo ""
echo "=== Installation Complete ==="
echo ""
echo "Next steps:"
echo "1. Clone the RefBoard repository:"
echo "   cd $WORKSPACE_DIR/src"
echo "   git clone <your-repository-url>"
echo ""
echo "2. Build the workspace:"
echo "   cd $WORKSPACE_DIR"
echo "   colcon build"
echo ""
echo "3. Source the workspace:"
echo "   source install/setup.bash"
echo ""
echo "4. Set up CAN interface:"
echo "   sudo ip link set can0 up type can bitrate 500000"
echo ""
echo "5. Test direct motor control:"
echo "   ros2 run refboard_base test_mks_can can0"
echo ""
echo "6. Launch ROS2 control:"
echo "   ros2 launch refboard_ros2_control refboard_ros2_control.launch.py"
echo ""

# Add workspace sourcing to bashrc
if ! grep -q "source $WORKSPACE_DIR/install/setup.bash" ~/.bashrc; then
    echo "# RefBoard workspace" >> ~/.bashrc
    echo "if [ -f $WORKSPACE_DIR/install/setup.bash ]; then" >> ~/.bashrc
    echo "    source $WORKSPACE_DIR/install/setup.bash" >> ~/.bashrc
    echo "fi" >> ~/.bashrc
    echo "Added workspace sourcing to ~/.bashrc"
fi

echo "Please restart your terminal or run 'source ~/.bashrc' to load the environment."
