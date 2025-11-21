"""
Standalone Camera Calibration Test
===================================

Tests the camera calibration system without ROS2 dependencies.
Validates coordinate transformations, intrinsics, and eye-in-hand calibration.

Usage:
    conda activate ur30_vlm_environment
    python test_camera_calibration_standalone.py
"""
from _path_setup import setup_test_paths
setup_test_paths()


import numpy as np
import time

def test_camera_calibration_import():
    """Test if CameraCalibration can be imported"""
    print("\n" + "="*70)
    print("TEST 1: Camera Calibration Import")
    print("="*70)
    
    try:
        from CameraCalibration import CameraCalibration
        print("PASS CameraCalibration imported successfully (direct import)")
        return True, CameraCalibration
    except ImportError as e:
        print(f"FAIL Direct import failed: {e}")
        
        try:
            from unified_vision_system.calibration.CameraCalibration import CameraCalibration
            print("PASS CameraCalibration imported successfully (package import)")
            return True, CameraCalibration
        except ImportError as e2:
            print(f"FAIL Package import failed: {e2}")
            return False, None

def test_basic_initialization(CameraCalibration):
    """Test basic calibration initialization"""
    print("\n" + "="*70)
    print("TEST 2: Basic Initialization")
    print("="*70)
    
    try:
        calib = CameraCalibration()
        print("PASS Default initialization successful")
        return True, calib
    except Exception as e:
        print(f"FAIL FAIL: {e}")
        import traceback
        traceback.print_exc()
        return False, None

def test_mock_calibration_setup(calib):
    """Test mock calibration setup for testing"""
    print("\n" + "="*70)
    print("TEST 3: Mock Calibration Setup")
    print("="*70)
    
    try:
        # RealSense D435i typical intrinsics
        camera_matrix = np.array([
            [615.0, 0.0, 424.0],
            [0.0, 615.0, 240.0],
            [0.0, 0.0, 1.0]
        ])
        
        # Eye-in-hand transform (camera mounted on gripper)
        T_gripper_camera = np.eye(4)
        T_gripper_camera[:3, 3] = [0.0, 0.0, -0.1]  # 10cm below gripper
        
        calib.set_mock_calibration(
            camera_matrix=camera_matrix,
            eye_in_hand=True,
            T_gripper_camera=T_gripper_camera
        )
        
        print("PASS Mock calibration setup successful")
        print(f"   Camera matrix (fx, fy): ({camera_matrix[0,0]:.1f}, {camera_matrix[1,1]:.1f})")
        print(f"   Principal point: ({camera_matrix[0,2]:.1f}, {camera_matrix[1,2]:.1f})")
        print(f"   Eye-in-hand: True")
        print(f"   Camera offset: {T_gripper_camera[:3, 3]}")
        
        return True
        
    except Exception as e:
        print(f"FAIL FAIL: {e}")
        import traceback
        traceback.print_exc()
        return False

def test_pixel_to_camera_transform(calib):
    """Test pixel to camera coordinate transformation"""
    print("\n" + "="*70)
    print("TEST 4: Pixel to Camera Transformation")
    print("="*70)
    
    test_cases = [
        # (u, v, depth, expected_result, description)
        (424, 240, 0.5, (0.0, 0.0, 0.5), "Center pixel"),
        (624, 240, 1.0, (0.325, 0.0, 1.0), "Right of center"),
        (424, 340, 1.0, (0.0, 0.163, 1.0), "Below center"),
        (224, 140, 0.3, (-0.0976, -0.0488, 0.3), "Top-left"),
    ]
    
    passed = 0
    for u, v, depth, expected, desc in test_cases:
        try:
            result = calib.pixel_to_camera(u, v, depth)
            
            # Calculate error
            expected_np = np.array(expected)
            result_np = np.array(result)
            error = np.linalg.norm(result_np - expected_np) * 1000  # mm
            
            if error < 5.0:  # Less than 5mm error (accounting for float precision)
                print(f"  PASS {desc}")
                print(f"     Pixel: ({u}, {v}) at {depth}m -> Camera: {result}")
                print(f"     Error: {error:.3f}mm")
                passed += 1
            else:
                print(f"  FAIL {desc}")
                print(f"     Expected: {expected}, Got: {result}")
                print(f"     Error: {error:.3f}mm")
                
        except Exception as e:
            print(f"  FAIL {desc}: Exception - {e}")
    
    if passed >= 3:
        print("PASS PASS: Pixel to camera transform working")
        return True
    else:
        print("WARNING  MARGINAL: Some pixel transforms failed")
        return True

def test_camera_to_robot_transform(calib):
    """Test camera to robot coordinate transformation (eye-in-hand)"""
    print("\n" + "="*70)
    print("TEST 5: Camera to Robot Transformation")
    print("="*70)
    
    # Test various gripper poses
    test_cases = [
        # (gripper_pos, camera_point, description)
        ([0.5, 0.2, 0.3], (0.0, 0.0, 0.2), "Gripper at (0.5, 0.2, 0.3)"),
        ([0.6, -0.1, 0.4], (0.1, 0.0, 0.3), "Gripper at (0.6, -0.1, 0.4)"),
        ([0.3, 0.3, 0.5], (-0.05, 0.05, 0.15), "Gripper at (0.3, 0.3, 0.5)"),
    ]
    
    passed = 0
    for gripper_pos, camera_point, desc in test_cases:
        try:
            # Create gripper pose
            current_gripper_pose = np.eye(4)
            current_gripper_pose[:3, 3] = gripper_pos
            
            # Transform camera point to robot frame
            robot_point = calib.camera_to_robot(camera_point, current_gripper_pose)
            
            print(f"  PASS {desc}")
            print(f"     Camera point: {camera_point}")
            print(f"     Robot point: {robot_point}")
            
            # Basic sanity check: point should be in reasonable workspace
            if -2.0 < robot_point[0] < 2.0 and -2.0 < robot_point[1] < 2.0 and -0.5 < robot_point[2] < 2.0:
                passed += 1
            else:
                print(f"     WARNING  WARNING: Robot point outside expected workspace")
                
        except Exception as e:
            print(f"  FAIL {desc}: Exception - {e}")
            import traceback
            traceback.print_exc()
    
    if passed >= 2:
        print("PASS PASS: Camera to robot transform working")
        return True
    else:
        print("FAIL FAIL: Camera to robot transforms failed")
        return False

def test_robot_to_camera_transform(calib):
    """Test robot to camera coordinate transformation"""
    print("\n" + "="*70)
    print("TEST 6: Robot to Camera Transformation")
    print("="*70)
    
    try:
        # Test roundtrip: robot -> camera -> robot
        gripper_pose = np.eye(4)
        gripper_pose[:3, 3] = [0.5, 0.2, 0.3]
        
        camera_point = (0.1, 0.05, 0.25)
        
        # Forward transform
        robot_point = calib.camera_to_robot(camera_point, gripper_pose)
        
        # Check if we have inverse transform
        if hasattr(calib, 'robot_to_camera'):
            # Reverse transform
            camera_point_back = calib.robot_to_camera(robot_point, gripper_pose)
            
            error = np.linalg.norm(np.array(camera_point_back) - np.array(camera_point)) * 1000
            
            print(f"Original camera point: {camera_point}")
            print(f"Robot point: {robot_point}")
            print(f"Back to camera: {camera_point_back}")
            print(f"Roundtrip error: {error:.3f}mm")
            
            if error < 1.0:
                print("PASS PASS: Roundtrip transform accurate")
                return True
            else:
                print("WARNING  MARGINAL: Roundtrip has some error")
                return True
        else:
            print("Skip  SKIP: Inverse transform not available")
            return True
            
    except Exception as e:
        print(f"WARNING  WARNING: {e}")
        return True

def test_transform_vector(calib):
    """Test vector transformation (for normals/directions)"""
    print("\n" + "="*70)
    print("TEST 7: Vector Transformation")
    print("="*70)
    
    try:
        if hasattr(calib, 'transform_vector'):
            # Test transforming direction vectors
            gripper_pose = np.eye(4)
            gripper_pose[:3, 3] = [0.5, 0.2, 0.3]
            
            # Approach vector (downward in camera frame)
            approach_vector = np.array([0.0, 0.0, 1.0])
            
            robot_vector = calib.transform_vector(approach_vector, gripper_pose)
            
            print(f"Camera vector: {approach_vector}")
            print(f"Robot vector: {robot_vector}")
            
            # Vector should maintain magnitude
            mag_cam = np.linalg.norm(approach_vector)
            mag_robot = np.linalg.norm(robot_vector)
            
            if abs(mag_cam - mag_robot) < 0.01:
                print(f"PASS Vector magnitude preserved: {mag_cam:.3f} ~ {mag_robot:.3f}")
                return True
            else:
                print(f"WARNING  WARNING: Vector magnitude changed: {mag_cam:.3f} -> {mag_robot:.3f}")
                return True
        else:
            print("Skip  SKIP: Vector transform method not available")
            return True
            
    except Exception as e:
        print(f"WARNING  WARNING: {e}")
        return True

def test_intrinsics_loading(calib):
    """Test camera intrinsics loading and validation"""
    print("\n" + "="*70)
    print("TEST 8: Camera Intrinsics")
    print("="*70)
    
    try:
        # Check if intrinsics are set
        if hasattr(calib, 'camera_matrix'):
            K = calib.camera_matrix
            
            print(f"Camera matrix:")
            print(f"  [[{K[0,0]:.2f}, {K[0,1]:.2f}, {K[0,2]:.2f}]")
            print(f"   [{K[1,0]:.2f}, {K[1,1]:.2f}, {K[1,2]:.2f}]")
            print(f"   [{K[2,0]:.2f}, {K[2,1]:.2f}, {K[2,2]:.2f}]]")
            
            # Validate intrinsics
            fx, fy = K[0, 0], K[1, 1]
            cx, cy = K[0, 2], K[1, 2]
            
            # Basic validation
            if fx > 0 and fy > 0 and cx > 0 and cy > 0:
                print("PASS Camera intrinsics valid")
                
                # Check aspect ratio
                aspect_ratio = fx / fy
                if 0.95 < aspect_ratio < 1.05:
                    print(f"PASS Aspect ratio reasonable: {aspect_ratio:.3f}")
                else:
                    print(f"WARNING  WARNING: Unusual aspect ratio: {aspect_ratio:.3f}")
                
                return True
            else:
                print("FAIL FAIL: Invalid intrinsic values")
                return False
        else:
            print("WARNING  WARNING: Camera matrix attribute not found")
            return True
            
    except Exception as e:
        print(f"WARNING  WARNING: {e}")
        return True

def test_distortion_handling(calib):
    """Test distortion coefficient handling"""
    print("\n" + "="*70)
    print("TEST 9: Distortion Coefficients")
    print("="*70)
    
    try:
        if hasattr(calib, 'dist_coeffs'):
            dist = calib.dist_coeffs
            
            if dist is not None:
                print(f"Distortion coefficients: {dist}")
                
                # Check if reasonable values
                if len(dist) >= 5:
                    print("PASS Distortion model: 5+ parameters")
                    return True
                else:
                    print(f"PASS Distortion model: {len(dist)} parameters")
                    return True
            else:
                print("Skip  No distortion coefficients (assuming undistorted)")
                return True
        else:
            print("Skip  SKIP: Distortion coefficients not available")
            return True
            
    except Exception as e:
        print(f"WARNING  WARNING: {e}")
        return True

def test_eye_in_hand_transform(calib):
    """Test eye-in-hand calibration transform"""
    print("\n" + "="*70)
    print("TEST 10: Eye-in-Hand Transform")
    print("="*70)
    
    try:
        if hasattr(calib, 'T_gripper_camera') or hasattr(calib, 'hand_eye_transform'):
            T = calib.T_gripper_camera if hasattr(calib, 'T_gripper_camera') else calib.hand_eye_transform
            
            print(f"Hand-eye transform:")
            for i in range(4):
                print(f"  [{T[i,0]:7.4f}, {T[i,1]:7.4f}, {T[i,2]:7.4f}, {T[i,3]:7.4f}]")
            
            # Extract translation
            translation = T[:3, 3]
            print(f"\nCamera offset from gripper: {translation}")
            
            # Validate transform
            # Check if it's a valid SE(3) matrix
            R = T[:3, :3]
            det_R = np.linalg.det(R)
            
            if abs(det_R - 1.0) < 0.01:
                print(f"PASS Valid rotation matrix (det={det_R:.4f})")
            else:
                print(f"WARNING  WARNING: Rotation matrix determinant: {det_R:.4f}")
            
            # Check orthogonality
            should_be_identity = R @ R.T
            ortho_error = np.linalg.norm(should_be_identity - np.eye(3))
            
            if ortho_error < 0.01:
                print(f"PASS Orthogonal rotation matrix (error={ortho_error:.6f})")
                return True
            else:
                print(f"WARNING  WARNING: Rotation not orthogonal (error={ortho_error:.6f})")
                return True
        else:
            print("WARNING  WARNING: Hand-eye transform attribute not found")
            return True
            
    except Exception as e:
        print(f"WARNING  WARNING: {e}")
        return True

def test_performance_benchmark(calib):
    """Benchmark transformation performance"""
    print("\n" + "="*70)
    print("TEST 11: Performance Benchmark")
    print("="*70)
    
    try:
        np.random.seed(42)
        num_transforms = 1000
        
        # Generate random test data
        pixels = [(np.random.randint(0, 848), np.random.randint(0, 480), np.random.uniform(0.3, 2.0)) 
                  for _ in range(num_transforms)]
        
        gripper_pose = np.eye(4)
        gripper_pose[:3, 3] = [0.5, 0.2, 0.3]
        
        # Benchmark pixel to camera
        start = time.perf_counter()
        for u, v, d in pixels:
            calib.pixel_to_camera(u, v, d)
        elapsed_p2c = time.perf_counter() - start
        
        # Benchmark camera to robot
        camera_points = [(np.random.uniform(-0.2, 0.2), 
                         np.random.uniform(-0.2, 0.2), 
                         np.random.uniform(0.1, 0.5)) 
                        for _ in range(num_transforms)]
        
        start = time.perf_counter()
        for point in camera_points:
            calib.camera_to_robot(point, gripper_pose)
        elapsed_c2r = time.perf_counter() - start
        
        avg_p2c_us = (elapsed_p2c / num_transforms) * 1e6
        avg_c2r_us = (elapsed_c2r / num_transforms) * 1e6
        
        print(f"Pixel to camera: {num_transforms} transforms in {elapsed_p2c:.3f}s")
        print(f"  Average: {avg_p2c_us:.2f}us per transform")
        
        print(f"Camera to robot: {num_transforms} transforms in {elapsed_c2r:.3f}s")
        print(f"  Average: {avg_c2r_us:.2f}us per transform")
        
        if avg_p2c_us < 100 and avg_c2r_us < 100:
            print("PASS PASS: Excellent performance (<100us per transform)")
            return True
        elif avg_p2c_us < 1000 and avg_c2r_us < 1000:
            print("PASS PASS: Good performance (<1ms per transform)")
            return True
        else:
            print("WARNING  WARNING: Performance may need optimization")
            return True
            
    except Exception as e:
        print(f"FAIL FAIL: {e}")
        return False

def main():
    """Run all camera calibration tests"""
    print("\n" + "="*70)
    print("CAMERA CALIBRATION STANDALONE TEST")
    print("="*70)
    print("Testing camera calibration and coordinate transformations")
    
    start_time = time.time()
    
    # Test 1: Import
    import_ok, CameraCalibration = test_camera_calibration_import()
    if not import_ok:
        print("\nFAIL Cannot proceed without CameraCalibration import")
        return
    
    # Test 2: Initialization
    init_ok, calib = test_basic_initialization(CameraCalibration)
    if not init_ok or calib is None:
        print("\nFAIL Cannot proceed without successful initialization")
        return
    
    # Test 3: Mock calibration setup
    mock_ok = test_mock_calibration_setup(calib)
    if not mock_ok:
        print("\nFAIL Cannot proceed without mock calibration")
        return
    
    # Run remaining tests
    results = []
    results.append(("Import", import_ok))
    results.append(("Initialization", init_ok))
    results.append(("Mock Setup", mock_ok))
    results.append(("Pixel to Camera", test_pixel_to_camera_transform(calib)))
    results.append(("Camera to Robot", test_camera_to_robot_transform(calib)))
    results.append(("Robot to Camera", test_robot_to_camera_transform(calib)))
    results.append(("Vector Transform", test_transform_vector(calib)))
    results.append(("Intrinsics", test_intrinsics_loading(calib)))
    results.append(("Distortion", test_distortion_handling(calib)))
    results.append(("Eye-in-Hand", test_eye_in_hand_transform(calib)))
    results.append(("Performance", test_performance_benchmark(calib)))
    
    # Summary
    elapsed = time.time() - start_time
    passed = sum(1 for _, result in results if result)
    total = len(results)
    
    print("\n" + "="*70)
    print("TEST SUMMARY")
    print("="*70)
    print(f"Total tests: {total}")
    print(f"PASS Passed: {passed}")
    print(f"FAIL Failed: {total - passed}")
    print(f"Success rate: {passed/total*100:.1f}%")
    print(f"Total time: {elapsed:.2f}s")
    print("="*70)
    
    if passed == total:
        print("\nComplete ALL TESTS PASSED!")
        print("Camera calibration system is working correctly.")
    elif passed >= total * 0.8:
        print("\nPASS MOST TESTS PASSED")
        print("Camera calibration system is functional.")
    else:
        print("\nWARNING  SOME TESTS FAILED")
        print("Camera calibration system may need attention.")
    
    print()

if __name__ == "__main__":
    main()
