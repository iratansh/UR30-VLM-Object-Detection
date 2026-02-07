#!/bin/bash
# Set UR30 to observation pose after spawn

echo "Waiting for robot to spawn..."
sleep 8

echo "Setting robot to observation pose..."
ros2 topic pub --once /scaled_joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'],
  points: [
    {
      positions: [0.0, -1.05, -1.57, -1.57, 1.57, 0.0],
      time_from_start: {sec: 2}
    }
  ]
}"

echo "Robot moved to observation pose"
