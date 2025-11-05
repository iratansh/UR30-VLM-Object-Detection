#!/usr/bin/env python3
"""
Test Command Publisher Node

Publishes a hardcoded voice command to the vision system after a delay.
This simulates a user saying "pick up the red cube" without needing
actual microphone input.

Usage:
    ros2 run unified_vision_system test_command_publisher

Or include in launch file with parameters:
    - delay_seconds: Time to wait before publishing command (default: 5.0)
    - test_command: The command text to publish (default: "pick up the red cube")
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time


class TestCommandPublisher(Node):
    """Publishes a test command after initialization delay"""
    
    def __init__(self):
        super().__init__('test_command_publisher')
        
        # Declare parameters
        self.declare_parameter('delay_seconds', 5.0)
        self.declare_parameter('test_command', 'pick up the red cube')
        
        # Get parameters
        self.delay = self.get_parameter('delay_seconds').value
        self.command = self.get_parameter('test_command').value
        
        # Create publisher
        self.publisher = self.create_publisher(
            String,
            '/voice_command',
            10
        )
        
        self.get_logger().info(f'Test Command Publisher initialized')
        self.get_logger().info(f'Will publish "{self.command}" after {self.delay}s delay')
        
        # Create timer to publish command after delay
        self.timer = self.create_timer(self.delay, self.publish_command)
        self.command_published = False
    
    def publish_command(self):
        """Publish the test command once"""
        if not self.command_published:
            msg = String()
            msg.data = self.command
            
            self.publisher.publish(msg)
            
            self.get_logger().info('=' * 70)
            self.get_logger().info(f'📢 PUBLISHED TEST COMMAND: "{self.command}"')
            self.get_logger().info('=' * 70)
            self.get_logger().info('Vision system should now:')
            self.get_logger().info('  1. Receive voice command')
            self.get_logger().info('  2. Detect "red cube" using OWL-ViT')
            self.get_logger().info('  3. Compute grasp points from depth')
            self.get_logger().info('  4. Plan trajectory to grasp pose')
            self.get_logger().info('  5. Execute pick-and-place motion')
            self.get_logger().info('=' * 70)
            
            self.command_published = True
            
            # Keep node alive for a bit to ensure message is received
            self.timer = self.create_timer(60.0, lambda: None)  # Keep alive


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    node = TestCommandPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
