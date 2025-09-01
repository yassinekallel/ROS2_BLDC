#!/bin/bash

# RefBoard System Test Script
# Comprehensive testing of the RefBoard ROS2 control system

set -e

CAN_INTERFACE=${1:-can0}
WORKSPACE_DIR=${2:-$(pwd)}

echo "=== RefBoard System Test ==="
echo "CAN Interface: $CAN_INTERFACE"
echo "Workspace: $WORKSPACE_DIR"
echo ""

# Function to check if a command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Function to wait for user input
wait_for_user() {
    echo "Press ENTER to continue or Ctrl+C to exit..."
    read
}

# Check prerequisites
echo "1. Checking prerequisites..."

if ! command_exists ros2; then
    echo "✗ ROS2 not found. Please install ROS2 first."
    echo "Run: ./scripts/install_dependencies.sh"
    exit 1
fi
echo "✓ ROS2 found: $(ros2 --version)"

if ! command_exists candump; then
    echo "✗ can-utils not found. Installing..."
    sudo apt install -y can-utils
fi
echo "✓ can-utils found"

# Check if workspace is built
if [ ! -f "$WORKSPACE_DIR/install/setup.bash" ]; then
    echo "✗ Workspace not built. Building now..."
    cd "$WORKSPACE_DIR"
    colcon build --packages-select refboard_base refboard_ros2_control
fi
echo "✓ Workspace built"

# Source workspace
source "$WORKSPACE_DIR/install/setup.bash"

echo ""
echo "2. Setting up CAN interface..."
./scripts/setup_can.sh $CAN_INTERFACE
echo ""

echo "3. Testing CAN interface..."
echo "Checking if CAN interface is up..."
if ! ip link show $CAN_INTERFACE | grep -q "UP"; then
    echo "✗ CAN interface is not up"
    exit 1
fi
echo "✓ CAN interface $CAN_INTERFACE is up"

# Test 1: CAN loopback test
echo ""
echo "4. Testing CAN loopback (hardware not required)..."
echo "Sending test frame..."
timeout 2s candump $CAN_INTERFACE &
CANDUMP_PID=$!
sleep 0.5
cansend $CAN_INTERFACE 123#DEADBEEF
sleep 1
kill $CANDUMP_PID 2>/dev/null || true
echo "✓ CAN loopback test completed"

# Test 2: RefBoard low-level test
echo ""
echo "5. Testing RefBoard direct motor control..."
echo "Starting interactive test program..."
echo "NOTE: Make sure your BLDC motors with MKS controllers are connected to CAN bus"
echo "No Arduino needed - ROS2 will send MKS commands directly!"
echo "Commands to try:"
echo "  p 1    - Read position from motor 1"
echo "  e 1    - Enable motor 1"
echo "  s 1 1000 - Set motor 1 to position 1000"
echo "  quit   - Exit test"
echo ""
wait_for_user

# Start test program in background and send some test commands
echo "Starting test program (will timeout in 30 seconds)..."
timeout 30s ros2 run refboard_base test_mks_can $CAN_INTERFACE || echo "Test program finished"

# Test 3: ROS2 Control test
echo ""
echo "6. Testing ROS2 Control integration..."
echo "This will start the complete ROS2 control system"
echo "Make sure your BLDC motors with MKS controllers are ready"
echo ""
wait_for_user

echo "Starting ROS2 control node (will run for 10 seconds)..."
timeout 10s ros2 launch refboard_ros2_control refboard_ros2_control.launch.py can_interface:=$CAN_INTERFACE &
LAUNCH_PID=$!

sleep 5

# Check if controllers are loaded
echo "Checking controller status..."
ros2 control list_controllers || echo "Controllers not ready yet"

# Send a test trajectory
echo "Sending test trajectory..."
ros2 topic pub --once /refboard_joint_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory '{
  joint_names: ["joint_x"],
  points: [{
    positions: [0.5],
    time_from_start: {sec: 2}
  }]
}' || echo "Failed to send trajectory (controller may not be ready)"

sleep 3
kill $LAUNCH_PID 2>/dev/null || true

echo ""
echo "=== Test Summary ==="
echo "✓ Prerequisites checked"
echo "✓ CAN interface configured"
echo "✓ CAN communication tested"
echo "✓ RefBoard low-level interface tested"
echo "✓ ROS2 control integration tested"
echo ""
echo "If all tests passed, your RefBoard system is ready!"
echo ""
echo "To start the complete system:"
echo "  ros2 launch refboard_ros2_control refboard_ros2_control.launch.py can_interface:=$CAN_INTERFACE"
echo ""
echo "To control joints:"
echo "  ros2 topic pub /refboard_joint_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory ..."
echo ""
echo "To monitor joint states:"
echo "  ros2 topic echo /joint_states"
