"""
Camera TF Checker - Find out where the camera actually is
"""

import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
import time

print("=" * 70)
print("CAMERA TF (TRANSFORM) CHECKER")
print("=" * 70)

rclpy.init()

class TFChecker(Node):
    def __init__(self):
        super().__init__('tf_checker')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.get_logger().info("Waiting 2 seconds for TF tree...")
        time.sleep(2)
        
        self.check_transforms()
    
    def check_transforms(self):
        frames_to_check = [
            'camera_link',
            'camera_color_optical_frame',
            'camera_depth_optical_frame',
            'tool0',
            'base_link'
        ]
        
        self.get_logger().info("")
        self.get_logger().info("=" * 70)
        self.get_logger().info("CHECKING CAMERA POSITION")
        self.get_logger().info("=" * 70)
        
        for frame in frames_to_check:
            try:
                transform = self.tf_buffer.lookup_transform(
                    'world',
                    frame,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=1.0)
                )
                
                t = transform.transform.translation
                r = transform.transform.rotation
                
                self.get_logger().info(f"\n{frame}:")
                self.get_logger().info(f"  Position: x={t.x:.3f}, y={t.y:.3f}, z={t.z:.3f}")
                self.get_logger().info(f"  Rotation: x={r.x:.3f}, y={r.y:.3f}, z={r.z:.3f}, w={r.w:.3f}")
                
            except Exception as e:
                self.get_logger().warn(f"\n{frame}: NOT FOUND ({e})")
        
        self.get_logger().info("")
        self.get_logger().info("=" * 70)
        self.get_logger().info("DIAGNOSIS:")
        
        try:
            cam_tf = self.tf_buffer.lookup_transform(
                'world', 'camera_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            z = cam_tf.transform.translation.z
            
            if z < 0.3:
                self.get_logger().warn(f"  ⚠️  Camera is LOW (z={z:.2f}m) - might be in ground!")
            elif z > 2.0:
                self.get_logger().warn(f"  ⚠️  Camera is HIGH (z={z:.2f}m) - might be in sky!")
            else:
                self.get_logger().info(f"  ✅ Camera height seems OK (z={z:.2f}m)")
            
            # Check if camera orientation makes sense
            r = cam_tf.transform.rotation
            if abs(r.w) < 0.1:
                self.get_logger().warn(f"  ⚠️  Camera rotation seems wrong!")
            
        except:
            self.get_logger().error("  ❌ Cannot find camera_link transform!")
        
        self.get_logger().info("=" * 70)
        
        raise KeyboardInterrupt()

try:
    node = TFChecker()
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
finally:
    rclpy.shutdown()

print("\n✅ TF check complete!")
