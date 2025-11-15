# Quick Gazebo UR30 Launch Script
# Run this inside the Docker container

echo "======================================================================"
echo "LAUNCHING UR30 GAZEBO SIMULATION"
echo "======================================================================"
echo ""

# Setup environment
cd /workspace
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "✅ Environment sourced"
echo ""

# Kill old processes
echo "Cleaning up old processes..."
pkill -9 gzserver 2>/dev/null
pkill -9 gzclient 2>/dev/null
pkill -9 -f "ros2 launch unified_vision_system" 2>/dev/null
sleep 3
echo "✅ Old processes cleaned"
echo ""

# Launch
echo "Starting Gazebo with UR30..."
echo "This will launch:"
echo "  - Gazebo simulator"
echo "  - UR30 robot with gripper"
echo "  - RealSense camera"
echo "  - Red cube target"
echo ""
echo "Press Ctrl+C to stop"
echo "======================================================================"
echo ""

ros2 launch unified_vision_system launch_gazebo_with_red_cube.py \
  use_rviz:=false \
  use_moveit:=false
