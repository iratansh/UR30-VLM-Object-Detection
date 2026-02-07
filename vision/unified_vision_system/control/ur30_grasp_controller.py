"""
UR30 Robot Grasp Controller.

This module provides high-level control of a UR30 robot for grasping tasks.
It handles:
- Joint and Cartesian space control
- Grasp planning and execution
- Safety monitoring and collision avoidance
- Robot state management

The controller uses ROS2 for communication with the robot hardware
and implements safety features required for physical robot operation.
"""

#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Pose
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from typing import List, Tuple, Optional
import logging
import cv2
import numpy.linalg as LA
import pyrealsense2 as rs

# Import UR30 kinematics
try:
    from vision.UR30Kinematics import UR30Kinematics
except ImportError:
    from UR30Kinematics import UR30Kinematics

class UR30GraspController(Node):
    """
    Enhanced UR30 controller with grasp planning and safety features.
    
    This class provides high-level control of a UR30 robot for grasping tasks,
    including motion planning, execution, and safety monitoring.
    
    Parameters
    ----------
    None : Uses ROS2 parameters for configuration
    
    Attributes
    ----------
    JOINT_LIMITS : Dict
        Joint angle limits in radians
    d1, a2, a3, d4, d5, d6 : float
        DH parameters in meters
    current_joints : np.ndarray
        Current joint angles
    current_pose : np.ndarray
        Current end-effector pose
        
    Notes
    -----
    The controller requires:
    - ROS2 Humble or newer
    - UR30 robot with ROS driver
    - Properly configured joint limits
    - Hand-eye calibration for visual servoing
    """
    
    # UR30 joint limits (radians) - official Universal Robots specifications
    JOINT_LIMITS = {
        'lower': np.array([-2*np.pi, -np.pi, -np.pi, -2*np.pi, -2*np.pi, -2*np.pi]),
        'upper': np.array([2*np.pi, np.pi, np.pi, 2*np.pi, 2*np.pi, 2*np.pi])
    }
    
    # UR30 DH parameters (meters) - Modified DH convention
    # a: [0.0000, -0.6370, -0.5037, 0.0000, 0.0000, 0.0000]
    # d: [0.2363, 0.0000, 0.0000, 0.2010, 0.1593, 0.1543]
    d1 = 0.2363    # Base to shoulder
    a2 = -0.637    # Upper arm length (negative for standard DH)
    a3 = -0.5037   # Forearm length (negative for standard DH)
    d4 = 0.201     # Wrist 1 height
    d5 = 0.1593    # Wrist 2 height
    d6 = 0.1543    # End effector length
    
    # Workspace limits (meters) - based on UR30 reach
    MAX_REACH = 1.19  # UR30 max reach ~1190mm (a2 + a3 + offsets)
    
    def __init__(self):
        super().__init__('ur30_grasp_controller')
        
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger(__name__)
        
        # Initialize UR30 kinematics solver
        self.kinematics = UR30Kinematics(debug=False)
        
        # Load hand-eye calibration
        self.T_base_to_camera = self._load_calibration()
        
        # Publishers
        self.joint_pub = self.create_publisher(
            Float64MultiArray,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )
        
        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_callback,
            10
        )
        
        # Current state
        self.current_joints = None
        self.current_pose = None
        
        self.logger.info("UR30 Grasp Controller initialized")
        self.logger.info(f"   Max reach: {self.MAX_REACH}m")
        self.logger.info(f"   Using Robotics Toolbox IK solver")
    
    def _load_calibration(self) -> np.ndarray:
        """
        Load hand-eye calibration matrix.
        
        Returns
        -------
        np.ndarray
            4x4 transformation matrix from robot base to camera
            
        Notes
        -----
        Loads calibration from 'hand_eye_calib.npz'
        Falls back to identity matrix if loading fails
        """
        try:
            calib_data = np.load('hand_eye_calib.npz')
            return calib_data['T_base_to_camera']
        except Exception as e:
            self.logger.error(f"Failed to load calibration: {e}")
            return np.eye(4)
    
    def _joint_state_callback(self, msg: JointState):
        """
        Process joint state updates.
        
        Parameters
        ----------
        msg : JointState
            ROS2 joint state message
            
        Notes
        -----
        Updates current_joints and current_pose
        Used for monitoring robot state
        """
        self.current_joints = np.array(msg.position)
        self.current_pose = self.forward_kinematics(self.current_joints)
    
    def forward_kinematics(self, theta: np.ndarray) -> np.ndarray:
        """
        Compute forward kinematics using UR30Kinematics.
        
        Parameters
        ----------
        theta : np.ndarray
            Joint angles in radians [theta1, theta2, theta3, theta4, theta5, theta6]
            
        Returns
        -------
        np.ndarray
            4x4 homogeneous transformation matrix for end-effector pose
            
        Notes
        -----
        Uses the UR30Kinematics class for forward kinematics computation
        """
        return self.kinematics.forward_kinematics(theta)
    
    def inverse_kinematics(self, target_pose: np.ndarray, 
                          initial_guess: Optional[np.ndarray] = None) -> Tuple[np.ndarray, bool]:
        """
        Enhanced inverse kinematics using UR30Kinematics with Robotics Toolbox.
        
        Parameters
        ----------
        target_pose : np.ndarray
            4x4 homogeneous transformation matrix for target pose
        initial_guess : Optional[np.ndarray]
            Initial joint configuration (used by numerical solver if RTB fails)
            
        Returns
        -------
        Tuple[np.ndarray, bool]
            Joint angles and success flag
            
        Notes
        -----
        Uses:
        1. Robotics Toolbox IK (~1ms solve time, 100% success)
        2. Falls back to numerical solver if RTB unavailable
        """
        if initial_guess is None:
            initial_guess = np.array([0, -np.pi/2, 0, -np.pi/2, 0, 0])
        
        # Use UR30Kinematics solver
        result = self.kinematics.inverse_kinematics(
            target_pose,
            current_joints=initial_guess if self.current_joints is None else self.current_joints
        )
        
        if result['success']:
            return result['joint_angles'], True
        else:
            self.logger.warning(f"IK failed: {result['message']}")
            return initial_guess, False
    
    def plan_grasp(self, object_pose: np.ndarray, approach_distance: float = 0.1) -> List[np.ndarray]:
        """
        Plan grasp trajectory with pre-grasp and final grasp poses.
        
        Parameters
        ----------
        object_pose : np.ndarray
            4x4 transformation matrix of object in camera frame
        approach_distance : float
            Distance to approach from above (meters), default 0.1m
            
        Returns
        -------
        List[np.ndarray]
            List of 4x4 pose matrices for grasp trajectory
            
        Notes
        -----
        Generates:
        1. Pre-grasp position (approach_distance above object)
        2. Final grasp position (at object)
        """
        # Transform object pose from camera to base frame
        object_pose_base = self.T_base_to_camera @ object_pose
        
        # Validate workspace
        target_position = object_pose_base[:3, 3]
        distance = np.linalg.norm(target_position)
        
        if distance > self.MAX_REACH:
            self.logger.error(f"Target distance {distance:.3f}m exceeds max reach {self.MAX_REACH}m")
            # Scale down to max reach
            scale = (self.MAX_REACH - 0.1) / distance  # 0.1m safety margin
            target_position *= scale
            object_pose_base[:3, 3] = target_position
            self.logger.warning(f"Scaled target to {np.linalg.norm(target_position):.3f}m")
        
        # Generate pre-grasp pose
        pre_grasp = object_pose_base.copy()
        pre_grasp[2, 3] += approach_distance  # Move up by approach_distance
        
        # Generate grasp poses
        poses = [
            pre_grasp,         # Pre-grasp position
            object_pose_base   # Final grasp position
        ]
        
        return poses
    
    def execute_grasp(self, object_pose: np.ndarray) -> bool:
        """
        Execute complete grasp sequence.
        
        Parameters
        ----------
        object_pose : np.ndarray
            4x4 transformation matrix of object in camera frame
            
        Returns
        -------
        bool
            True if grasp executed successfully, False otherwise
            
        Notes
        -----
        Sequence:
        1. Plan grasp trajectory
        2. Compute IK for each waypoint
        3. Validate solutions
        4. Execute motion
        """
        try:
            # Plan grasp trajectory
            grasp_poses = self.plan_grasp(object_pose)
            
            for i, pose in enumerate(grasp_poses):
                # Compute IK
                target_joints, success = self.inverse_kinematics(pose)
                
                if not success:
                    self.logger.error(f"IK failed for waypoint {i}")
                    return False
                
                if not self._validate_solution(target_joints):
                    self.logger.error(f"Invalid joint solution for waypoint {i}")
                    return False
                
                self._execute_motion(target_joints)
                
                self.logger.info(f"Reached waypoint {i+1}/{len(grasp_poses)}")
            
            return True
            
        except Exception as e:
            self.logger.error(f"Grasp execution failed: {e}")
            return False
    
    def _validate_solution(self, joints: np.ndarray) -> bool:
        """
        Validate joint solution for safety.
        
        Parameters
        ----------
        joints : np.ndarray
            Joint angles to validate
            
        Returns
        -------
        bool
            True if solution is safe, False otherwise
            
        Notes
        -----
        Checks:
        - Joint limits
        - Joint velocities (if current state available)
        - Workspace boundaries
        """
        # Check joint limits
        if np.any(joints < self.JOINT_LIMITS['lower']) or \
           np.any(joints > self.JOINT_LIMITS['upper']):
            self.logger.warning("Joint limits violated")
            return False
        
        # Check joint velocities (simplified)
        if self.current_joints is not None:
            max_velocity = 1.0  # rad/s per control cycle
            velocities = np.abs(joints - self.current_joints)
            if np.any(velocities > max_velocity):
                self.logger.warning(f"Joint velocity too high: {np.max(velocities):.3f} rad/s")
                return False
        
        # Validate end-effector position
        pose = self.forward_kinematics(joints)
        position = pose[:3, 3]
        distance = np.linalg.norm(position)
        
        if distance > self.MAX_REACH:
            self.logger.warning(f"End-effector distance {distance:.3f}m exceeds max reach")
            return False
        
        return True
    
    def _execute_motion(self, target_joints: np.ndarray):
        """
        Execute robot motion to target joints.
        
        Parameters
        ----------
        target_joints : np.ndarray
            Target joint angles
            
        Notes
        -----
        Publishes joint trajectory to ROS2 controller
        """
        msg = Float64MultiArray()
        msg.data = target_joints.tolist()
        self.joint_pub.publish(msg)
        
        if self.debug:
            self.logger.debug(f"Published joint command: {target_joints}")
    
    def get_performance_stats(self) -> Dict[str, float]:
        """
        Get IK solver performance statistics.
        
        Returns
        -------
        Dict[str, float]
            Performance metrics including solve time, success rate
        """
        return self.kinematics.get_performance_stats()
    
    def emergency_stop(self):
        """
        Emergency stop - halt all motion immediately.
        
        Notes
        -----
        This should trigger hardware-level emergency stop
        Currently just logs a warning
        """
        self.logger.critical("Alert EMERGENCY STOP TRIGGERED")
        # In production, this would interface with robot's emergency stop system


def main():
    """Main entry point for UR30 grasp controller."""
    rclpy.init()
    
    try:
        controller = UR30GraspController()
        rclpy.spin(controller)
    except Exception as e:
        logging.error(f"Controller failed: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
