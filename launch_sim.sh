#!/bin/bash
# Convenient script to launch Gazebo simulation
# Usage: ./launch_sim.sh [use_rviz] [use_moveit]
# Example: ./launch_sim.sh true false  (with RViz, without MoveIt)

set -e

# Default arguments
USE_RVIZ=${1:-false}
USE_MOVEIT=${2:-false}

echo "========================================="
echo "   UR30 + Robotiq Gazebo Simulation"
echo "========================================="
echo "RViz: $USE_RVIZ"
echo "MoveIt: $USE_MOVEIT"
echo ""

# Run inside Docker container with proper environment
docker exec -it ur30-vlm-working bash -c "
    cd /workspace && \
    unset CONDA_PREFIX && \
    unset CONDA_DEFAULT_ENV && \
    source /opt/ros/humble/setup.bash && \
    source install/setup.bash && \
    ros2 launch unified_vision_system launch_gazebo_with_red_cube.py \
        use_rviz:=$USE_RVIZ \
        use_moveit:=$USE_MOVEIT
"
