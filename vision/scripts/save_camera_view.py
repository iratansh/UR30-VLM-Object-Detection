"""
Camera View Saver - Save what the camera sees to debug cube position
"""

import sys
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

print("=" * 70)
print("CAMERA VIEW SAVER")
print("=" * 70)
print("\nThis will save what the camera sees to /tmp/camera_view.jpg")
print("Use this to check if the red cube is visible!\n")

rclpy.init()

class CameraViewSaver(Node):
    def __init__(self):
        super().__init__('camera_view_saver')
        self.bridge = CvBridge()
        self.saved = False
        
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        self.get_logger().info("Waiting for camera image...")
        self.create_timer(5.0, self.timeout)
    
    def image_callback(self, msg):
        if not self.saved:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                
                # Save image
                cv2.imwrite('/tmp/camera_view.jpg', cv_image)
                
                # Also save with annotations
                annotated = cv_image.copy()
                h, w = annotated.shape[:2]
                
                # Draw crosshair at center
                cv2.line(annotated, (w//2-20, h//2), (w//2+20, h//2), (0, 255, 0), 2)
                cv2.line(annotated, (w//2, h//2-20), (w//2, h//2+20), (0, 255, 0), 2)
                
                # Add text
                cv2.putText(annotated, f"Camera View {w}x{h}", 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                cv2.putText(annotated, "Center crosshair", 
                           (10, h-20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                cv2.imwrite('/tmp/camera_view_annotated.jpg', annotated)
                
                self.get_logger().info("")
                self.get_logger().info("=" * 70)
                self.get_logger().info("Camera images saved!")
                self.get_logger().info("   - /tmp/camera_view.jpg (raw)")
                self.get_logger().info("   - /tmp/camera_view_annotated.jpg (with crosshair)")
                self.get_logger().info("")
                self.get_logger().info("Check these files to see if red cube is visible!")
                self.get_logger().info("If not visible, we need to adjust cube position.")
                self.get_logger().info("=" * 70)
                
                self.saved = True
                raise KeyboardInterrupt()
                
            except Exception as e:
                self.get_logger().error(f"Failed to save image: {e}")
    
    def timeout(self):
        if not self.saved:
            self.get_logger().error("Warning  No image received after 5 seconds")
        raise KeyboardInterrupt()

try:
    node = CameraViewSaver()
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
finally:
    rclpy.shutdown()

print("\nOK Done! Check /tmp/camera_view.jpg")
print("   You can copy it out with:")
print("   docker cp ur5e-vlm-working:/tmp/camera_view.jpg ./camera_view.jpg")
print()
