"""
ROS2 node wrapper for UnifiedVisionSystem
"""

import rclpy
from unified_vision_system.system.UnifiedVisionSystemSim import UnifiedVisionSystemSim

def main(args=None):
    rclpy.init(args=args)
    
    try:
        vision_system = UnifiedVisionSystemSim()
        vision_system.run()
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
