"""
Gazebo Simulation Test Launch File for UR30 VLM System

This launch file starts:
1. Gazebo with UR30 robot and test objects
2. Vision system node
3. Test command publisher (simulates "pick up the red cube")

Test Flow:
- Gazebo launches with red cube on table
- Vision system initializes
- After 5 seconds, automated command "pick up the red cube" is published
- System should: detect cube -> compute grasp -> execute motion
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from pathlib import Path


def generate_launch_description():
    
    # Get paths
    vision_dir = Path(__file__).parent.parent
    world_file = str(vision_dir / "worlds" / "ur30_vision_world.world")
    
    # Declare arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    gui = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Start Gazebo with GUI'
    )
    
    verbose = DeclareLaunchArgument(
        'verbose',
        default_value='true',
        description='Verbose output'
    )
    
    # Start Gazebo server
    gazebo_server = ExecuteProcess(
        cmd=['gzserver',
             world_file,
             '--verbose' if LaunchConfiguration('verbose') else ''],
        output='screen',
        name='gazebo_server'
    )
    
    # Start Gazebo client (GUI)
    gazebo_client = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        name='gazebo_client',
        condition=LaunchConfiguration('gui')
    )
    
    # Robot state publisher (publishes robot description and joint states)
    # Note: You may need to adjust this based on your UR30 URDF setup
    robot_description_file = vision_dir.parent / "Universal_Robots_ROS2_Description" / "urdf" / "ur30.urdf"
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_description': open(str(robot_description_file)).read() if robot_description_file.exists() else ''
        }]
    ) if robot_description_file.exists() else None
    
    # Spawn UR30 robot in Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'ur30',
            '-topic', '/robot_description',
            '-x', '0.0',
            '-y', '0.0', 
            '-z', '0.0',
        ],
        output='screen'
    )
    
    # Vision system node (eye-in-hand configuration)
    vision_system_node = Node(
        package='unified_vision_system',
        executable='unified_vision_system',
        name='unified_vision_system',
        output='screen',
        parameters=[{
            # Simulation settings
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            
            # Robot configuration
            'robot_namespace': 'ur30',
            
            # Eye-in-hand configuration
            'eye_in_hand': True,
            'broadcast_camera_tf': True,
            
            # Calibration (use simulation defaults)
            'hand_eye_calibration_file': '',  # Will use identity if empty
            'camera_calibration_file': '',  # Will use defaults
            
            # Simulation camera (lower res for performance)
            'realsense_serial': '',
            'max_depth': 2.0,
            'min_depth': 0.1,
            'depth_fps': 15,
            'color_fps': 15,
            'depth_width': 640,
            'depth_height': 480,
            'color_width': 640,
            'color_height': 480,
            
            # VLM detection
            'vlm_confidence_threshold': 0.10,  # Lower for simulation
            
            # Hybrid IK
            'enable_hybrid_ik': True,
            'ik_timeout_ms': 100.0,
            
            # Safety
            'min_grasp_height': 0.02,
        }],
        remappings=[
            ('/ur30/joint_states', '/joint_states'),
            ('/ur30/scaled_joint_trajectory_controller/joint_trajectory',
             '/scaled_joint_trajectory_controller/joint_trajectory'),
        ]
    )
    
    # Test command publisher node (publishes hardcoded command after delay)
    test_command_node = Node(
        package='unified_vision_system',
        executable='test_command_publisher',
        name='test_command_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'delay_seconds': 5.0,  # Wait 5s for system to initialize
            'test_command': 'pick up the red cube',
        }]
    )
    
    # Joint state publisher (if no real controller, publish fake joint states)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )
    
    # RViz for visualization
    rviz_config = str(vision_dir / "config" / "ur30_vision.rviz") if (vision_dir / "config" / "ur30_vision.rviz").exists() else ''
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config] if rviz_config else [],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )
    
    # Build launch description
    ld = LaunchDescription()
    
    # Add arguments
    ld.add_action(use_sim_time)
    ld.add_action(gui)
    ld.add_action(verbose)
    
    # Start Gazebo
    ld.add_action(gazebo_server)
    ld.add_action(gazebo_client)
    
    # Add robot nodes
    if robot_state_publisher:
        ld.add_action(robot_state_publisher)
    # ld.add_action(spawn_robot)  # Uncomment when URDF is ready
    ld.add_action(joint_state_publisher)
    
    # Start vision system after a delay
    ld.add_action(
        TimerAction(
            period=3.0,
            actions=[vision_system_node]
        )
    )
    
    # Start test command publisher after another delay
    ld.add_action(
        TimerAction(
            period=8.0,  # 3s for Gazebo + 5s for vision system
            actions=[test_command_node]
        )
    )
    
    # Start RViz
    ld.add_action(rviz_node)
    
    return ld
