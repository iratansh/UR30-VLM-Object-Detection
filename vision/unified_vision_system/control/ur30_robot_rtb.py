"""
UR30 Robot Model for Robotics Toolbox
Defines the UR30 robot using correct DH parameters for Robotics Toolbox
"""

import numpy as np
from roboticstoolbox import DHRobot, RevoluteDH
from spatialmath import SE3


class UR30(DHRobot):
    """
    UR30 robot model for Robotics Toolbox.
    
    This uses Modified DH parameters matching the official UR30 specifications.
    
    DH Parameters (Modified DH Convention):
    a:  [0.0000, -0.6370, -0.5037, 0.0000, 0.0000, 0.0000]
    d:  [0.2363, 0.0000, 0.0000, 0.2010, 0.1593, 0.1543]
    alpha: [0, pi/2, 0, 0, pi/2, -pi/2]
    """
    
    def __init__(self):
        # UR30 DH parameters (Modified DH)
        deg = np.pi / 180
        
        # Modified DH parameters: [d, a, alpha, offset]
        # Link 1
        L1 = RevoluteDH(
            d=0.2363,
            a=0.0,
            alpha=np.pi/2,
            qlim=[-2*np.pi, 2*np.pi]
        )
        
        # Link 2
        L2 = RevoluteDH(
            d=0.0,
            a=-0.637,
            alpha=0.0,
            qlim=[-np.pi, np.pi]
        )
        
        # Link 3
        L3 = RevoluteDH(
            d=0.0,
            a=-0.5037,
            alpha=0.0,
            qlim=[-np.pi, np.pi]
        )
        
        # Link 4
        L4 = RevoluteDH(
            d=0.201,
            a=0.0,
            alpha=np.pi/2,
            qlim=[-2*np.pi, 2*np.pi]
        )
        
        # Link 5
        L5 = RevoluteDH(
            d=0.1593,
            a=0.0,
            alpha=-np.pi/2,
            qlim=[-2*np.pi, 2*np.pi]
        )
        
        # Link 6
        L6 = RevoluteDH(
            d=0.1543,
            a=0.0,
            alpha=0.0,
            qlim=[-2*np.pi, 2*np.pi]
        )
        
        # Create robot with all links
        super().__init__(
            [L1, L2, L3, L4, L5, L6],
            name="UR30",
            manufacturer="Universal Robots"
        )
        
        # Define named configurations
        self.addconfiguration("qz", [0, 0, 0, 0, 0, 0])
        self.addconfiguration("qr", [0, -np.pi/2, 0, 0, 0, 0])  # Ready pose
        
        # Tool offset (if any) - adjust as needed
        # self.tool = SE3(0, 0, 0)  # No tool offset by default


def test_ur30_rtb():
    """Test the UR30 robot model"""
    print("=" * 60)
    print("TESTING UR30 ROBOTICS TOOLBOX MODEL")
    print("=" * 60)
    
    robot = UR30()
    print(f"\nRobot: {robot.name}")
    print(f"Manufacturer: {robot.manufacturer}")
    print(f"Number of joints: {robot.n}")
    print(f"\nDH Parameters (Modified DH):")
    print(robot)
    
    # Test forward kinematics at home position
    print("\n" + "-" * 60)
    print("Testing Forward Kinematics")
    print("-" * 60)
    
    test_configs = [
        ("Zero", [0, 0, 0, 0, 0, 0]),
        ("Ready", [0, -np.pi/2, 0, 0, 0, 0]),
        ("Test", [0, -np.pi/4, -np.pi/4, 0, 0, 0])
    ]
    
    for name, q in test_configs:
        T = robot.fkine(q)
        print(f"\n{name} config: {[round(np.degrees(a), 1) for a in q]}")
        print(f"Position: [{T.t[0]:.4f}, {T.t[1]:.4f}, {T.t[2]:.4f}]")
    
    print("\n" + "=" * 60)
    print("UR30 Model Ready")
    print("=" * 60)


if __name__ == "__main__":
    test_ur30_rtb()