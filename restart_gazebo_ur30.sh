# Script to restart Gazebo with UR30 world

echo "======================================================================"
echo "RESTARTING GAZEBO WITH UR30 WORLD"
echo "======================================================================"

# Kill all existing Gazebo and ROS2 launch processes
echo "Step 1: Stopping existing Gazebo processes..."
docker exec -it ur5e-vlm-working bash -c "
pkill -9 gzserver
pkill -9 gzclient
pkill -9 -f 'ros2 launch'
pkill -9 -f 'spawn_entity'
sleep 2
"

echo "✅ Existing processes stopped"
echo ""

# Clean up any stale locks
echo "Step 2: Cleaning up Gazebo locks..."
docker exec -it ur5e-vlm-working bash -c "
rm -f /tmp/.gazebo_lock* 2>/dev/null
rm -f /tmp/.X*-lock 2>/dev/null
"

echo "✅ Locks cleaned"
echo ""

# Launch UR30 world
echo "Step 3: Launching UR30 Gazebo world..."
echo "This will start:"
echo "  - Gazebo with ur30_vision_world.world"
echo "  - UR30 robot with Robotiq gripper"
echo "  - RealSense camera"
echo "  - Red cube target"
echo ""

docker exec -it ur5e-vlm-working bash -c "
cd /workspace && \
source /opt/ros/humble/setup.bash && \
source install/setup.bash && \
ros2 launch unified_vision_system launch_gazebo_with_red_cube.py \
  use_rviz:=false \
  use_moveit:=false \
  > /tmp/ur30_gazebo.log 2>&1 &
"

echo "✅ Launch command sent"
echo ""

# Wait for Gazebo to start
echo "Step 4: Waiting for Gazebo to initialize (15 seconds)..."
sleep 15

# Verify topics
echo ""
echo "======================================================================"
echo "VERIFICATION - Checking ROS2 Topics"
echo "======================================================================"
docker exec -it ur5e-vlm-working bash -c "
source /opt/ros/humble/setup.bash && \
ros2 topic list | grep -E '(camera|joint_states|trajectory)' | sort
"

echo ""
echo "======================================================================"
echo "Gazebo restart complete!"
echo ""
echo "Check logs: docker exec -it ur5e-vlm-working tail -f /tmp/ur30_gazebo.log"
echo "GUI: Set DISPLAY variable and use gzclient"
echo "======================================================================"
