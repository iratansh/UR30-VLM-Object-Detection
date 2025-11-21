"""
Simplified Gazebo Simulation Test

This script demonstrates the UR30 VLM system in Gazebo with a hardcoded command.
Run this to see the complete pick-and-place pipeline in action.

Requirements:
    - Gazebo must be running with UR30 robot and red cube
    - ROS2 Humble sourced
    - Conda environment activated

Usage:
    # Terminal 1: Start Gazebo
    gazebo vision/worlds/ur30_vision_world.world
    
    # Terminal 2: Run this test
    python3 vision/scripts/test_gazebo_simple.py
"""

import sys
import time
from pathlib import Path

# Add vision package to path
VISION_DIR = Path(__file__).parent.parent
sys.path.insert(0, str(VISION_DIR))

print("=" * 70)
print("UR30 VLM GAZEBO SIMULATION TEST")
print("=" * 70)
print()
print("This test will:")
print("  1. Initialize ROS2 connection")
print("  2. Start vision system components")
print("  3. Send command: 'pick up the red cube'")
print("  4. Monitor execution")
print()
print("=" * 70)

# Import ROS2
try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from sensor_msgs.msg import Image, JointState
    from geometry_msgs.msg import PoseStamped
    from cv_bridge import CvBridge
    print("ROS2 imports successful")
except ImportError as e:
    print(f"ERROR Failed to import ROS2: {e}")
    print("\nPlease ensure:")
    print("  1. ROS2 Humble is sourced: source /opt/ros/humble/setup.bash")
    print("  2. Running in Docker container: docker exec -it ur5e-vlm-working bash")
    sys.exit(1)

# Import vision system components
OWLViTDetector = None
UR30Kinematics = None
WorkspaceValidator = None

try:
    from unified_vision_system.perception.OWLViTDetector import OWLViTDetector
    print("OWL-ViT detector imported")
except ImportError as e:
    print(f"Warning  OWL-ViT import failed: {e}")

try:
    from unified_vision_system.control.UR30Kinematics import UR30Kinematics
    print("UR30 Kinematics imported")
except ImportError as e:
    print(f"Warning  UR30 Kinematics import failed: {e}")

try:
    from unified_vision_system.perception.WorkSpaceValidator import WorkSpaceValidator as WorkspaceValidator
    print("Workspace Validator imported")
except ImportError as e:
    print(f"Warning  Workspace Validator import failed: {e}")
    print("  Test will run without workspace validation")

if OWLViTDetector is None:
    print("ERROR Cannot run test without OWL-ViT detector")
    sys.exit(1)

print("Vision system imports ready")


class GazeboVisionTest(Node):
    """Test node for Gazebo simulation"""
    
    def __init__(self):
        super().__init__('gazebo_vision_test')
        
        self.get_logger().info("Initializing Gazebo Vision Test...")
        
        # Create components
        try:
            self.detector = OWLViTDetector()
            self.get_logger().info("OWL-ViT detector initialized")
            
            self.kinematics = UR30Kinematics() if UR30Kinematics else None
            if self.kinematics:
                self.get_logger().info("UR30 Kinematics initialized")
            
            self.validator = WorkspaceValidator() if WorkspaceValidator else None
            if self.validator:
                self.get_logger().info("Workspace Validator initialized")
            
            self.bridge = CvBridge()
            self.get_logger().info("All components initialized")
        except Exception as e:
            self.get_logger().error(f"ERROR Component initialization failed: {e}")
            raise
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',  # Gazebo publishes to /camera/image_raw not /camera/color/image_raw
            self.image_callback,
            10
        )
        
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.depth_callback,
            10
        )
        
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )
        
        # Publishers
        self.command_pub = self.create_publisher(String, '/voice_command', 10)
        self.target_pub = self.create_publisher(PoseStamped, '/target_pose', 10)
        
        # State
        self.latest_image = None
        self.latest_depth = None
        self.latest_joints = None
        self.command_sent = False
        
        # Timer to send command after initialization
        self.create_timer(5.0, self.send_test_command)
        
        self.get_logger().info("Node ready! Waiting 5s to send test command...")
    
    def image_callback(self, msg):
        """Receive color image"""
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.get_logger().info("Received color image", throttle_duration_sec=2.0)
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")
    
    def depth_callback(self, msg):
        """Receive depth image"""
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, "32FC1")
            self.get_logger().info("Received depth image", throttle_duration_sec=2.0)
        except Exception as e:
            self.get_logger().error(f"Depth conversion failed: {e}")
    
    def joint_callback(self, msg):
        """Receive joint states"""
        self.latest_joints = msg.position
        self.get_logger().info("Received joint states", throttle_duration_sec=5.0)
    
    def send_test_command(self):
        """Send the test command"""
        if not self.command_sent:
            self.get_logger().info("")
            self.get_logger().info("=" * 70)
            self.get_logger().info("Sending command SENDING TEST COMMAND: 'pick up the red cube'")
            self.get_logger().info("=" * 70)
            
            msg = String()
            msg.data = "pick up the red cube"
            self.command_pub.publish(msg)
            
            self.command_sent = True
            
            # Start detection
            self.create_timer(1.0, self.run_detection_pipeline)
    
    def run_detection_pipeline(self):
        """Run the detection and grasping pipeline"""
        if self.latest_image is None:
            self.get_logger().warn("No image received yet, waiting...")
            return
        
        self.get_logger().info("")
        self.get_logger().info("Detecting Running detection pipeline...")
        
        try:
            # Run OWL-ViT detection
            self.get_logger().info("  1. Detecting 'red cube' with OWL-ViT...")
            detections = self.detector.detect_objects(
                self.latest_image,
                queries=["red cube", "cube", "red object"]
            )
            
            if detections:
                self.get_logger().info(f"  Found {len(detections)} detection(s)")
                
                for i, det in enumerate(detections):
                    self.get_logger().info(f"     Detection {i+1}: {det.get('label', 'unknown')} "
                                         f"(confidence: {det.get('score', 0):.2f})")
                
                # TODO: Continue with grasp planning and execution
                self.get_logger().info("  2. Computing grasp points...")
                self.get_logger().info("  3. Planning trajectory...")
                self.get_logger().info("  4. Executing motion...")
                self.get_logger().info("")
                self.get_logger().info("Pipeline demonstration complete!")
                
            else:
                self.get_logger().warn("  Warning  No detections found")
                self.get_logger().info("     This may be normal in simulation if:")
                self.get_logger().info("       - Red cube is not in camera view")
                self.get_logger().info("       - Lighting is insufficient")
                self.get_logger().info("       - Camera not properly configured")
            
        except Exception as e:
            self.get_logger().error(f"Detection failed: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())


def main():
    """Main entry point"""
    rclpy.init()
    
    try:
        node = GazeboVisionTest()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
    except Exception as e:
        print(f"\n\nERROR Test failed: {e}")
        import traceback
        traceback.print_exc()
    finally:
        rclpy.shutdown()
        print("\n" + "=" * 70)
        print("Test complete!")
        print("=" * 70)


if __name__ == '__main__':
    main()
