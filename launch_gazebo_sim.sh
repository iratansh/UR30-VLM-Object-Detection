#!/bin/bash
# Complete build and launch script for UR30 Gazebo Simulation
# This ensures all dependencies are met and environment is properly configured

set -e

echo "=================================================="
echo "  UR30 + Robotiq 2F-85 Gazebo Simulation Setup"
echo "=================================================="
echo ""

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Step 1: Clean environment
echo -e "${YELLOW}Step 1: Cleaning conda environment variables...${NC}"
unset CONDA_PREFIX
unset CONDA_DEFAULT_ENV
unset PYTHONPATH

# Step 2: Source ROS2
echo -e "${YELLOW}Step 2: Sourcing ROS2 Humble...${NC}"
source /opt/ros/humble/setup.bash

# Step 3: Build essential packages
echo -e "${YELLOW}Step 3: Building ROS2 workspace packages...${NC}"
cd /workspace

# Build only packages we need (skip problematic robotiq_driver and robotiq_controllers)
colcon build \
    --packages-select serial robotiq_description ur_description robotiq_2f_85_gripper_visualization unified_vision_system \
    --symlink-install \
    --cmake-args \
        -Dconsole_bridge_DIR=/usr/lib/x86_64-linux-gnu/console_bridge/cmake \
        -Dspdlog_DIR=/usr/lib/x86_64-linux-gnu/cmake/spdlog

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ Build successful${NC}"
else
    echo -e "${RED}✗ Build failed${NC}"
    exit 1
fi

# Step 4: Source workspace
echo -e "${YELLOW}Step 4: Sourcing workspace...${NC}"
source install/setup.bash

# Step 5: Set Gazebo environment variables
echo -e "${YELLOW}Step 5: Setting up Gazebo environment...${NC}"
export GAZEBO_MODEL_PATH=/workspace/install/robotiq_description/share:/workspace/install/ur_description/share:/opt/ros/humble/share:/usr/share/gazebo-11/models:$GAZEBO_MODEL_PATH
export GAZEBO_RESOURCE_PATH=/opt/ros/humble/share:/usr/share/gazebo-11:$GAZEBO_RESOURCE_PATH

echo -e "${GREEN}✓ Environment configured${NC}"
echo ""
echo "Gazebo Model Path:"
echo "  $GAZEBO_MODEL_PATH"
echo ""

# Step 6: Kill any existing Gazebo
echo -e "${YELLOW}Step 6: Cleaning up any existing Gazebo processes...${NC}"
pkill -9 gzserver 2>/dev/null || true
pkill -9 gzclient 2>/dev/null || true
sleep 2

# Step 7: Launch simulation
echo -e "${YELLOW}Step 7: Launching Gazebo simulation...${NC}"
echo -e "${GREEN}Press Ctrl+C to stop the simulation${NC}"
echo ""
sleep 1

ros2 launch unified_vision_system launch_gazebo_with_red_cube.py \
    use_rviz:=false \
    use_moveit:=false
