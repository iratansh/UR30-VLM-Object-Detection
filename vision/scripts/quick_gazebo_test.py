"""
Quick Gazebo Test - Minimal version to verify everything works
"""

import sys
print("=" * 70)
print("QUICK GAZEBO VISION TEST")
print("=" * 70)
print()

# Test 1: ROS2 imports
print("Test 1: Importing ROS2...")
try:
    import rclpy
    from sensor_msgs.msg import Image
    from std_msgs.msg import String
    print("✅ ROS2 imports successful")
except ImportError as e:
    print(f"❌ ROS2 import failed: {e}")
    sys.exit(1)

# Test 2: Vision system imports  
print("\nTest 2: Importing vision system...")
sys.path.insert(0, '/workspace/vision')
try:
    from unified_vision_system.perception.OWLViTDetector import OWLViTDetector
    print("✅ OWL-ViT detector imported")
except ImportError as e:
    print(f"⚠️  OWL-ViT import failed: {e}")
    print("   This is expected if PyTorch not available in system Python")

# Test 3: Check ROS2 topics
print("\nTest 3: Checking ROS2 topics...")
rclpy.init()

from rclpy.node import Node

class QuickTest(Node):
    def __init__(self):
        super().__init__('quick_test')
        self.image_count = 0
        self.joint_count = 0
        
        self.image_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10
        )
        
        self.image_sub2 = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        self.get_logger().info("Subscribed to camera topics")
        self.get_logger().info("Listening for 5 seconds...")
        
        # Auto-shutdown after 5 seconds
        self.create_timer(5.0, self.shutdown)
    
    def image_callback(self, msg):
        self.image_count += 1
        if self.image_count == 1:
            self.get_logger().info(f"✅ Received first image! ({msg.width}x{msg.height})")
        
    def shutdown(self):
        self.get_logger().info("")
        self.get_logger().info("=" * 70)
        self.get_logger().info(f"TEST RESULTS:")
        self.get_logger().info(f"  Images received: {self.image_count}")
        if self.image_count > 0:
            self.get_logger().info(f"  ✅ Camera is publishing!")
        else:
            self.get_logger().warn(f"  ⚠️  No images received")
            self.get_logger().warn(f"     Check if Gazebo is running with camera")
        self.get_logger().info("=" * 70)
        raise KeyboardInterrupt()

try:
    node = QuickTest()
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
finally:
    rclpy.shutdown()

print("\n✅ Quick test complete!")
print()
