#!/bin/bash
# Spawn ros2_control controllers for UR30 simulation

echo "Waiting for Gazebo to be ready..."
sleep 5

echo "Spawning joint_state_broadcaster..."
ros2 control load_controller --set-state active joint_state_broadcaster

echo "Spawning scaled_joint_trajectory_controller..."
ros2 control load_controller --set-state active scaled_joint_trajectory_controller

echo "Spawning robotiq_gripper_controller..."
ros2 control load_controller --set-state active robotiq_gripper_controller

echo "All controllers spawned"
ros2 control list_controllers
