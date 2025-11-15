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
echo "Building core packages: serial, robotiq_description, ur_description, robotiq_2f_85_gripper_visualization"
colcon build \
    --packages-select serial robotiq_description ur_description robotiq_2f_85_gripper_visualization \
    --symlink-install \
    --cmake-args \
        -Dconsole_bridge_DIR=/usr/lib/x86_64-linux-gnu/console_bridge/cmake \
        -Dspdlog_DIR=/usr/lib/x86_64-linux-gnu/cmake/spdlog

echo ""
echo "=== Build Complete ==="
echo ""
echo "✅ Successfully built packages:"
echo "  - serial (ROS2 serial communication library)"
echo "  - robotiq_description (Gripper URDF/meshes)"
echo "  - ur_description (UR robot URDF/meshes)"
echo "  - robotiq_2f_85_gripper_visualization (Gripper visualization)"
echo ""
echo "⚠️  Skipped packages (dependency issues):"
echo "  - robotiq_driver (needs class_loader transitive dependency fix)"
echo "  - robotiq_controllers (needs class_loader transitive dependency fix)"
echo ""
echo "To use the built packages, source the workspace:"
echo "  source install/setup.bash"
echo ""
echo "Note: For actual robot control, robotiq_driver/controller need to be fixed."
echo "For visualization and simulation, the built packages should be sufficient."
