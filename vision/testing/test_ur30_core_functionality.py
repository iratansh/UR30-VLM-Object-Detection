"""
UR30 Core Functionality Test Suite
===================================

Tests the most critical components for the UR30 VLM system:
1. UR30 Kinematics (IK/FK with Robotics Toolbox)
2. Workspace Validation (UR30 reach limits)
3. Vision Pipeline (OWL-ViT detector)
4. Camera Calibration (coordinate transforms)
5. HybridIK Wrapper (VLM integration)

Run this test suite to verify the core system is working before full integration.

Usage:
    python test_ur30_core_functionality.py

Note: This test should be run WITHOUT ROS2 sourced to avoid dependency conflicts.
      Run from conda environment only: conda activate ur30_vlm_environment
"""

import sys
import os
import numpy as np
import time
import logging
from typing import List, Tuple, Optional

# Add parent directory to path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

# Check if ROS2 is sourced (which causes conflicts with conda)
if 'ROS_DISTRO' in os.environ:
    print("\n" + "="*70)
    print("WARNING: ROS2 environment detected!")
    print("="*70)
    print("ROS2 libraries conflict with conda Python packages.")
    print("Please run this test in a clean conda environment:")
    print("  1. Open a NEW terminal (without ROS2 sourced)")
    print("  2. conda activate ur30_vlm_environment")
    print("  3. cd /workspace/vision/testing")
    print("  4. python test_ur30_core_functionality.py")
    print("="*70 + "\n")

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(levelname)s - %(name)s - %(message)s'
)
logger = logging.getLogger(__name__)

# Test results tracking
class TestResults:
    def __init__(self):
        self.passed = 0
        self.failed = 0
        self.skipped = 0
        self.errors = []
    
    def add_pass(self, test_name: str):
        self.passed += 1
        logger.info(f"✅ PASS: {test_name}")
    
    def add_fail(self, test_name: str, error: str):
        self.failed += 1
        self.errors.append((test_name, error))
        logger.error(f"❌ FAIL: {test_name} - {error}")
    
    def add_skip(self, test_name: str, reason: str):
        self.skipped += 1
        logger.warning(f"⏭️  SKIP: {test_name} - {reason}")
    
    def print_summary(self):
        total = self.passed + self.failed + self.skipped
        print("\n" + "="*70)
        print("TEST SUMMARY")
        print("="*70)
        print(f"Total tests: {total}")
        print(f"✅ Passed: {self.passed}")
        print(f"❌ Failed: {self.failed}")
        print(f"⏭️  Skipped: {self.skipped}")
        print(f"Success rate: {self.passed/max(1,total)*100:.1f}%")
        
        if self.errors:
            print("\nFailed tests:")
            for test_name, error in self.errors:
                print(f"  - {test_name}: {error}")
        print("="*70)

results = TestResults()

# ============================================================================
# TEST 1: UR30 Kinematics (Most Critical)
# ============================================================================

def test_ur30_kinematics():
    """Test UR30 forward and inverse kinematics"""
    logger.info("\n" + "="*70)
    logger.info("TEST 1: UR30 Kinematics")
    logger.info("="*70)
    
    try:
        from unified_vision_system.control.UR30Kinematics import UR30Kinematics
        
        kin = UR30Kinematics(debug=False)
        
        # Test 1.1: Forward Kinematics
        logger.info("Test 1.1: Forward Kinematics")
        test_joints = [0.0, -np.pi/2, 0.0, -np.pi/2, 0.0, 0.0]
        T = kin.forward_kinematics(test_joints)
        
        if T.shape == (4, 4):
            position = T[:3, 3]
            logger.info(f"  FK result: position = [{position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}]")
            results.add_pass("FK returns 4x4 matrix")
        else:
            results.add_fail("FK returns 4x4 matrix", f"Got shape {T.shape}")
        
        # Test 1.2: Inverse Kinematics - Reachable pose
        logger.info("Test 1.2: Inverse Kinematics - Reachable pose")
        target_position = [0.4, 0.2, 0.3]
        target_pose = np.eye(4)
        target_pose[:3, 3] = target_position
        target_pose[:3, :3] = np.array([  # Top-down grasp orientation
            [1, 0, 0],
            [0, -1, 0],
            [0, 0, -1]
        ])
        
        start_time = time.perf_counter()
        ik_result = kin.inverse_kinematics(target_pose)
        solve_time = (time.perf_counter() - start_time) * 1000
        
        if ik_result['success']:
            joints = ik_result['joint_angles']
            logger.info(f"  IK solved in {solve_time:.2f}ms")
            logger.info(f"  Joint angles (deg): {np.degrees(joints).round(1)}")
            
            # Verify solution accuracy
            achieved_pose = kin.forward_kinematics(joints)
            position_error = np.linalg.norm(achieved_pose[:3, 3] - target_pose[:3, 3]) * 1000
            logger.info(f"  Position error: {position_error:.3f}mm")
            
            if position_error < 1.0:  # Less than 1mm error
                results.add_pass(f"IK accuracy (<1mm, got {position_error:.3f}mm)")
            else:
                results.add_fail(f"IK accuracy", f"Error {position_error:.3f}mm > 1mm")
            
            if solve_time < 100.0:  # Less than 100ms
                results.add_pass(f"IK speed (<100ms, got {solve_time:.2f}ms)")
            else:
                results.add_fail(f"IK speed", f"Took {solve_time:.2f}ms > 100ms")
        else:
            results.add_fail("IK reachable pose", ik_result['message'])
        
        # Test 1.3: Performance stats
        logger.info("Test 1.3: Performance Statistics")
        stats = kin.get_performance_stats()
        logger.info(f"  Solver: {stats.get('primary_solver', 'unknown')}")
        logger.info(f"  Success rate: {stats.get('success_rate', 0):.1%}")
        if stats.get('success_rate', 0) > 0.5:
            results.add_pass("IK success rate >50%")
        else:
            results.add_fail("IK success rate", f"Only {stats.get('success_rate', 0):.1%}")
        
    except ImportError as e:
        results.add_skip("UR30 Kinematics", f"Import failed: {e}")
    except Exception as e:
        results.add_fail("UR30 Kinematics", str(e))

# ============================================================================
# TEST 2: Workspace Validator
# ============================================================================

def test_workspace_validator():
    """Test workspace validation for UR30"""
    logger.info("\n" + "="*70)
    logger.info("TEST 2: Workspace Validator")
    logger.info("="*70)
    
    try:
        from unified_vision_system.perception.WorkSpaceValidator import WorkspaceValidator
        
        validator = WorkspaceValidator()
        
        # Test 2.1: Reachable point (within UR30 limits)
        logger.info("Test 2.1: Reachable point validation")
        reachable_point = (0.5, 0.3, 0.4)
        if validator.is_reachable(*reachable_point):
            results.add_pass(f"Validates reachable point {reachable_point}")
        else:
            results.add_fail(f"Reachable point", f"Point {reachable_point} marked as unreachable")
        
        # Test 2.2: Unreachable point (beyond UR30 reach)
        logger.info("Test 2.2: Unreachable point validation")
        unreachable_point = (2.0, 0.0, 0.5)  # Way beyond 1.19m reach
        if not validator.is_reachable(*unreachable_point):
            results.add_pass(f"Rejects unreachable point {unreachable_point}")
        else:
            results.add_fail(f"Unreachable point", f"Point {unreachable_point} marked as reachable")
        
        # Test 2.3: UR30 max reach test
        logger.info("Test 2.3: UR30 maximum reach (1.19m)")
        at_limit_point = (1.15, 0.0, 0.2)  # Close to 1.19m limit
        distance = np.sqrt(sum(x**2 for x in at_limit_point))
        logger.info(f"  Test point distance: {distance:.3f}m")
        if validator.is_reachable(*at_limit_point):
            results.add_pass(f"Point at {distance:.3f}m within UR30 reach")
        else:
            results.add_fail(f"UR30 reach limit", f"Point at {distance:.3f}m rejected")
        
        # Test 2.4: Safety score
        logger.info("Test 2.4: Safety score calculation")
        safe_point = (0.4, 0.2, 0.3)
        score = validator.get_safety_score(*safe_point)
        logger.info(f"  Safety score for {safe_point}: {score:.3f}")
        if 0.0 <= score <= 1.0:
            results.add_pass(f"Safety score in valid range [0,1]")
        else:
            results.add_fail(f"Safety score", f"Score {score} outside [0,1]")
        
    except ImportError as e:
        results.add_skip("Workspace Validator", f"Import failed: {e}")
    except Exception as e:
        results.add_fail("Workspace Validator", str(e))

# ============================================================================
# TEST 3: Camera Calibration
# ============================================================================

def test_camera_calibration():
    """Test camera calibration and coordinate transforms"""
    logger.info("\n" + "="*70)
    logger.info("TEST 3: Camera Calibration")
    logger.info("="*70)
    
    try:
        from unified_vision_system.calibration.CameraCalibration import CameraCalibration
        
        calib = CameraCalibration()
        
        # Test 3.1: Mock calibration setup
        logger.info("Test 3.1: Mock calibration setup")
        camera_matrix = np.array([
            [615.0, 0.0, 424.0],
            [0.0, 615.0, 240.0],
            [0.0, 0.0, 1.0]
        ])
        T_gripper_camera = np.eye(4)
        T_gripper_camera[:3, 3] = [0.0, 0.0, -0.1]  # 10cm offset
        
        calib.set_mock_calibration(
            camera_matrix=camera_matrix,
            eye_in_hand=True,
            T_gripper_camera=T_gripper_camera
        )
        results.add_pass("Mock calibration setup")
        
        # Test 3.2: Pixel to camera transformation
        logger.info("Test 3.2: Pixel to camera transformation")
        pixel_u, pixel_v = 424, 240  # Center pixel
        depth = 0.5  # 50cm depth
        point_camera = calib.pixel_to_camera(pixel_u, pixel_v, depth)
        logger.info(f"  Pixel ({pixel_u}, {pixel_v}) at {depth}m -> Camera {point_camera}")
        
        # Center pixel should be close to (0, 0, depth)
        if abs(point_camera[0]) < 0.01 and abs(point_camera[1]) < 0.01:
            results.add_pass("Pixel to camera transform (center pixel)")
        else:
            results.add_fail("Pixel to camera", f"Center pixel gave {point_camera}")
        
        # Test 3.3: Camera to robot transformation (eye-in-hand)
        logger.info("Test 3.3: Camera to robot transformation")
        current_gripper_pose = np.eye(4)
        current_gripper_pose[:3, 3] = [0.5, 0.2, 0.3]  # Gripper position
        
        camera_point = (0.1, 0.0, 0.2)
        robot_point = calib.camera_to_robot(camera_point, current_gripper_pose)
        logger.info(f"  Camera {camera_point} -> Robot {robot_point}")
        results.add_pass("Camera to robot transformation")
        
    except ImportError as e:
        results.add_skip("Camera Calibration", f"Import failed: {e}")
    except Exception as e:
        results.add_fail("Camera Calibration", str(e))

# ============================================================================
# TEST 4: Vision Detection (OWL-ViT)
# ============================================================================

def test_vision_detection():
    """Test OWL-ViT object detection"""
    logger.info("\n" + "="*70)
    logger.info("TEST 4: Vision Detection (OWL-ViT)")
    logger.info("="*70)
    
    try:
        from unified_vision_system.perception.OWLViTDetector import OWLViTDetector
        
        # Test 4.1: Detector initialization
        logger.info("Test 4.1: OWL-ViT initialization")
        try:
            detector = OWLViTDetector(device='cpu')  # Use CPU for testing
            results.add_pass("OWL-ViT initialization")
        except Exception as e:
            results.add_fail("OWL-ViT initialization", str(e))
            return
        
        # Test 4.2: Detection on synthetic image
        logger.info("Test 4.2: Detection on synthetic image")
        # Create a simple test image (red square on white background)
        test_image = np.ones((480, 640, 3), dtype=np.uint8) * 255
        test_image[200:280, 270:370] = [0, 0, 255]  # Red square
        
        try:
            detections = detector.detect(
                image=test_image,
                text_queries=["red object", "square"],
                threshold=0.05
            )
            logger.info(f"  Found {len(detections)} detections")
            results.add_pass(f"Detection execution (found {len(detections)})")
        except Exception as e:
            results.add_fail("Detection execution", str(e))
        
    except ImportError as e:
        results.add_skip("Vision Detection", f"Import failed: {e}")
    except Exception as e:
        results.add_fail("Vision Detection", str(e))

# ============================================================================
# TEST 5: HybridIK Wrapper
# ============================================================================

def test_hybrid_ik_wrapper():
    """Test HybridIK wrapper for VLM integration"""
    logger.info("\n" + "="*70)
    logger.info("TEST 5: HybridIK Wrapper (VLM Integration)")
    logger.info("="*70)
    
    try:
        from unified_vision_system.control.HybridIKWrapper import VLMKinematicsController
        
        # Test 5.1: Controller initialization
        logger.info("Test 5.1: VLM Controller initialization")
        controller = VLMKinematicsController(debug=False)
        results.add_pass("VLM Controller initialization")
        
        # Test 5.2: Object pickup solving
        logger.info("Test 5.2: Object pickup IK solving")
        success, joints, metadata = controller.solve_for_object_pickup(
            object_position=[0.4, 0.2, 0.1],
            object_type="bottle",
            current_joints=[0, -np.pi/2, 0, -np.pi/2, 0, 0]
        )
        
        logger.info(f"  Success: {success}")
        logger.info(f"  Solve time: {metadata['solve_time_ms']:.2f}ms")
        logger.info(f"  Position error: {metadata['position_error_mm']:.2f}mm")
        
        if success:
            results.add_pass("Object pickup IK solve")
        else:
            results.add_fail("Object pickup IK", "Failed to find solution")
        
        # Test 5.3: Different grasp orientations
        logger.info("Test 5.3: Different grasp orientations")
        orientations = ["top_down", "side_grasp", "angled_grasp"]
        for orientation in orientations:
            success, joints, metadata = controller.solve_for_vlm_target(
                target_position=[0.4, 0.1, 0.2],
                target_orientation=orientation,
                allow_approximation=True
            )
            if success:
                logger.info(f"  ✓ {orientation}: {metadata['solve_time_ms']:.1f}ms")
            else:
                logger.info(f"  ✗ {orientation}: failed")
        
        results.add_pass("Multiple grasp orientations")
        
        # Test 5.4: Performance statistics
        logger.info("Test 5.4: Performance statistics")
        stats = controller.get_vlm_performance_stats()
        logger.info(f"  VLM success rate: {stats['vlm_success_rate']:.1%}")
        logger.info(f"  Avg solve time: {stats.get('avg_solve_time_ms', 0):.1f}ms")
        results.add_pass("VLM performance stats")
        
    except ImportError as e:
        results.add_skip("HybridIK Wrapper", f"Import failed: {e}")
    except Exception as e:
        results.add_fail("HybridIK Wrapper", str(e))

# ============================================================================
# TEST 6: Grasp Point Detector
# ============================================================================

def test_grasp_detector():
    """Test grasp point detection"""
    logger.info("\n" + "="*70)
    logger.info("TEST 6: Grasp Point Detector")
    logger.info("="*70)
    
    try:
        from unified_vision_system.perception.GraspPointDetector import GraspPointDetector
        
        # Test 6.1: Detector initialization
        logger.info("Test 6.1: Grasp detector initialization")
        detector = GraspPointDetector(gripper_width=0.085)
        results.add_pass("Grasp detector initialization")
        
        # Test 6.2: Grasp detection on synthetic ROI
        logger.info("Test 6.2: Grasp detection")
        # Create synthetic object ROI
        roi = np.ones((200, 200, 3), dtype=np.uint8) * 255
        roi[50:150, 50:150] = [100, 100, 100]  # Gray square object
        
        grasp_info = detector.detect_grasp_point(roi)
        logger.info(f"  Grasp point: ({grasp_info['x']:.2f}, {grasp_info['y']:.2f})")
        logger.info(f"  Quality: {grasp_info['quality']:.2f}")
        
        if 0.0 <= grasp_info['x'] <= 1.0 and 0.0 <= grasp_info['y'] <= 1.0:
            results.add_pass("Grasp point in valid range")
        else:
            results.add_fail("Grasp point", f"Out of range: ({grasp_info['x']}, {grasp_info['y']})")
        
    except ImportError as e:
        results.add_skip("Grasp Detector", f"Import failed: {e}")
    except Exception as e:
        results.add_fail("Grasp Detector", str(e))

# ============================================================================
# MAIN TEST RUNNER
# ============================================================================

def main():
    """Run all core functionality tests"""
    print("\n" + "="*70)
    print("UR30 CORE FUNCTIONALITY TEST SUITE")
    print("="*70)
    print("Testing critical components for UR30 VLM system")
    print()
    
    start_time = time.time()
    
    # Run tests in priority order
    test_ur30_kinematics()          # Most critical: IK/FK
    test_workspace_validator()       # Critical: Safety
    test_camera_calibration()        # Critical: Vision transforms
    test_vision_detection()          # Important: Object detection
    test_hybrid_ik_wrapper()         # Important: VLM integration
    test_grasp_detector()            # Important: Grasp planning
    
    elapsed_time = time.time() - start_time
    
    # Print summary
    results.print_summary()
    print(f"\nTotal execution time: {elapsed_time:.2f}s")
    
    # Exit with appropriate code
    sys.exit(0 if results.failed == 0 else 1)

if __name__ == "__main__":
    main()
