# Script to build the ROS2 workspace with proper environment setup
# This handles the conda vs system Python issue and sets CMake paths for system libraries

set -e  # Exit on error

echo "=== Building ROS2 Workspace ==="
echo "This script will build the workspace using system Python (not conda)"
echo ""

# Unset conda environment variables to avoid conflicts
unset CONDA_PREFIX
unset CONDA_DEFAULT_ENV
unset PYTHONPATH

# Source ROS2 Humble
source /opt/ros/humble/setup.bash

# Set CMake paths for system libraries that ROS needs
export CMAKE_PREFIX_PATH=/usr/lib/x86_64-linux-gnu/cmake:/usr/lib/x86_64-linux-gnu:$CMAKE_PREFIX_PATH

# Build packages that are working
echo "Building core packages: serial, ur_description, unified_vision_system"
colcon build \
    --packages-select serial ur_description unified_vision_system \
    --symlink-install \
    --cmake-args \
        -Dconsole_bridge_DIR=/usr/lib/x86_64-linux-gnu/console_bridge/cmake \
        -Dspdlog_DIR=/usr/lib/x86_64-linux-gnu/cmake/spdlog

echo ""
echo "=== Build Complete ==="
echo ""
echo "✅ Successfully built packages:"
echo "  - serial (ROS2 serial communication library)"
echo "  - ur_description (UR30 robot URDF/meshes)"
echo "  - unified_vision_system (Vision system with custom gripper)"
echo ""
echo "To use the built packages, source the workspace:"
echo "  source install/setup.bash"
echo ""
echo "Note: Using custom simple parallel gripper for stable Gazebo simulation."
echo "Robotiq packages removed - custom gripper provides same functionality."
