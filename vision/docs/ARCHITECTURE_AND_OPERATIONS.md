# Unified Vision System: Architecture and Operations

This project simulates an eye-in-hand UR30 with a RealSense D435i for manipulation research. The stack runs in ROS 2 Humble with Gazebo, ros2_control, and custom perception/planning code under `vision/unified_vision_system`.

## Core Components
- Robot model: `vision/urdf/ur30_simple_gripper_realsense.urdf.xacro` mounts a simplified parallel gripper on `tool0` and attaches the RealSense to the gripper base.
- Launch: `vision/launch/launch_gazebo_with_red_cube.py` brings up Gazebo, spawns the robot, loads controllers, and moves the arm to an observation pose so the camera is not occluded.
- Controllers: `vision/scripts/spawn_controllers.sh` loads `joint_state_broadcaster`, `scaled_joint_trajectory_controller`, and `robotiq_gripper_controller`.
- Observation pose: `vision/scripts/set_observation_pose.sh` publishes a short trajectory to move the arm out of the all-zero home pose for a clear camera view.
- Vision system: nodes under `vision/unified_vision_system/system/` provide the main perception and control loop; perception, calibration, and HRI utilities live in sibling subpackages.

## Simulation Workflow
1) Build and source: `colcon build` then `source install/setup.bash` (after sourcing `/opt/ros/humble/setup.bash`).
2) Launch Gazebo: `ros2 launch unified_vision_system launch_gazebo_with_red_cube.py use_rviz:=false use_moveit:=false`. The launch file spawns the UR30, loads controllers, and automatically sends the observation pose trajectory.
3) Verify camera view: subscribe to `/camera/image_raw` to confirm the camera is clear of the robot body.
4) Run a regression test: `python3 vision/scripts/test_pick_green_book.py` exercises RGB + depth capture, color-based detection, IK, and gripper control.

## Eye-in-Hand Notes
- The camera is fixed to `gripper_base_link` with a downward pitch; at the all-zero joint pose it self-occludes, so the observation pose is sent immediately after controllers start.
- If the robot must start somewhere else, adjust the trajectory in `vision/scripts/set_observation_pose.sh` and keep the camera clear of collisions.
- Hand-eye and camera intrinsics files are parameterized in `vision/scripts/launch_eye_in_hand.py`.

## Calibration and Safety
- Eye-in-hand calibration helpers: `vision/scripts/calibrate_eye_in_hand.py` (simulation) and `vision/scripts/calibrate_hand_eye.py` (hardware).
- Safety checks for camera placement: `vision/unified_vision_system/calibration/EyeInHandSafetyChecker.py`.
- Transform sanity check: `vision/scripts/check_camera_tf.py` validates the camera pose and table height.

