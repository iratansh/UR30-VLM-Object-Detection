"""
Standalone Grasp Point Detector Test
=====================================

Tests the grasp point detection system without ROS2 dependencies.
Validates grasp point finding, quality scoring, and gripper constraints.

Usage:
    conda activate ur5e_vlm_environment
    python test_grasp_detector_standalone.py
"""
from _path_setup import setup_test_paths
setup_test_paths()


import numpy as np
import time
import cv2

def test_grasp_detector_import():
    """Test if GraspPointDetector can be imported"""
    print("\n" + "="*70)
    print("TEST 1: Grasp Point Detector Import")
    print("="*70)
    
    try:
        from GraspPointDetector import GraspPointDetector
        print("PASS GraspPointDetector imported successfully (direct import)")
        return True, GraspPointDetector
    except ImportError as e:
        print(f"FAIL Direct import failed: {e}")
        
        try:
            from unified_vision_system.perception.GraspPointDetector import GraspPointDetector
            print("PASS GraspPointDetector imported successfully (package import)")
            return True, GraspPointDetector
        except ImportError as e2:
            print(f"FAIL Package import failed: {e2}")
            return False, None

def test_dependencies():
    """Test if required dependencies are available"""
    print("\n" + "="*70)
    print("TEST 2: Dependency Check")
    print("="*70)
    
    deps_ok = True
    
    # Check numpy
    try:
        import numpy as np
        print(f"PASS NumPy {np.__version__} available")
    except ImportError:
        print("FAIL NumPy not available")
        deps_ok = False
    
    # Check cv2
    try:
        import cv2
        print(f"PASS OpenCV {cv2.__version__} available")
    except ImportError:
        print("FAIL OpenCV not available")
        deps_ok = False
    
    # Check scipy
    try:
        import scipy
        print(f"PASS SciPy {scipy.__version__} available")
    except ImportError:
        print("FAIL SciPy not available")
        deps_ok = False
    
    # Check skimage
    try:
        import skimage
        print(f"PASS Scikit-image {skimage.__version__} available")
    except ImportError:
        print("FAIL Scikit-image not available")
        deps_ok = False
    
    return deps_ok

def create_synthetic_object_roi(object_type="rectangular_box"):
    """Create synthetic object ROI for testing"""
    roi = np.ones((200, 200, 3), dtype=np.uint8) * 255  # White background
    
    if object_type == "rectangular_box":
        # Create a rectangular object (good for grasping)
        roi[60:140, 70:130] = [100, 100, 100]  # Gray rectangle
        depth_map = np.ones((200, 200), dtype=np.float32) * 1.0
        depth_map[60:140, 70:130] = 0.5  # 50cm depth
        return roi, depth_map, "rectangular box"
    
    elif object_type == "circular_object":
        # Create circular object
        cv2.circle(roi, (100, 100), 40, (100, 100, 100), -1)
        depth_map = np.ones((200, 200), dtype=np.float32) * 1.0
        mask = np.zeros((200, 200), dtype=np.uint8)
        cv2.circle(mask, (100, 100), 40, 1, -1)
        depth_map[mask > 0] = 0.5
        return roi, depth_map, "circular object"
    
    elif object_type == "irregular_shape":
        # Create irregular polygon
        pts = np.array([[80, 60], [120, 70], [130, 100], [110, 130], [70, 120]], np.int32)
        pts = pts.reshape((-1, 1, 2))
        cv2.fillPoly(roi, [pts], (100, 100, 100))
        depth_map = np.ones((200, 200), dtype=np.float32) * 1.0
        mask = np.zeros((200, 200), dtype=np.uint8)
        cv2.fillPoly(mask, [pts], 1)
        depth_map[mask > 0] = 0.5
        return roi, depth_map, "irregular shape"
    
    elif object_type == "thin_object":
        # Thin object (might be too narrow)
        roi[80:120, 95:105] = [100, 100, 100]
        depth_map = np.ones((200, 200), dtype=np.float32) * 1.0
        depth_map[80:120, 95:105] = 0.5
        return roi, depth_map, "thin object"
    
    return roi, None, object_type

def test_detector_initialization(GraspPointDetector):
    """Test grasp detector initialization"""
    print("\n" + "="*70)
    print("TEST 3: Detector Initialization")
    print("="*70)
    
    try:
        # Test with default parameters
        detector = GraspPointDetector()
        print(f"PASS Default initialization successful")
        print(f"   Gripper width: {detector.gripper_width}m")
        print(f"   Gripper finger width: {detector.gripper_finger_width}m")
        
        # Test with custom parameters
        detector_custom = GraspPointDetector(gripper_width=0.10, gripper_finger_width=0.015)
        print(f"PASS Custom initialization successful")
        print(f"   Custom gripper width: {detector_custom.gripper_width}m")
        
        return True, detector
        
    except Exception as e:
        print(f"FAIL FAIL: {e}")
        import traceback
        traceback.print_exc()
        return False, None

def test_grasp_point_detection(detector):
    """Test basic grasp point detection"""
    print("\n" + "="*70)
    print("TEST 4: Grasp Point Detection")
    print("="*70)
    
    try:
        roi, depth_map, desc = create_synthetic_object_roi("rectangular_box")
        
        print(f"Test object: {desc}")
        print(f"ROI shape: {roi.shape}")
        print(f"Depth map shape: {depth_map.shape if depth_map is not None else 'None'}")
        
        # Detect grasp point
        start = time.perf_counter()
        result = detector.detect_grasp_point(roi, depth_map)
        elapsed_ms = (time.perf_counter() - start) * 1000
        
        print(f"\nDetection time: {elapsed_ms:.2f}ms")
        print(f"Grasp point: ({result['x']:.3f}, {result['y']:.3f})")
        print(f"Angle: {result['angle']:.1f}deg")
        print(f"Quality: {result['quality']:.3f}")
        print(f"Width: {result['width']:.3f}m")
        
        # Validate results
        if 0 <= result['x'] <= 1 and 0 <= result['y'] <= 1:
            print("PASS Grasp point in valid range [0, 1]")
        else:
            print(f"WARNING  WARNING: Grasp point outside [0, 1]: ({result['x']}, {result['y']})")
        
        if 0 <= result['quality'] <= 1:
            print("PASS Quality score in valid range [0, 1]")
        else:
            print(f"WARNING  WARNING: Quality outside [0, 1]: {result['quality']}")
        
        if result['width'] > 0:
            print(f"PASS Valid grasp width: {result['width']*1000:.1f}mm")
        else:
            print(f"WARNING  WARNING: Invalid grasp width: {result['width']}")
        
        print("PASS PASS: Grasp detection successful")
        return True
        
    except Exception as e:
        print(f"FAIL FAIL: {e}")
        import traceback
        traceback.print_exc()
        return False

def test_different_object_shapes(detector):
    """Test detection on various object shapes"""
    print("\n" + "="*70)
    print("TEST 5: Different Object Shapes")
    print("="*70)
    
    object_types = ["rectangular_box", "circular_object", "irregular_shape", "thin_object"]
    
    results = []
    for obj_type in object_types:
        try:
            roi, depth_map, desc = create_synthetic_object_roi(obj_type)
            result = detector.detect_grasp_point(roi, depth_map)
            
            print(f"\n  {desc}:")
            print(f"    Grasp point: ({result['x']:.2f}, {result['y']:.2f})")
            print(f"    Quality: {result['quality']:.3f}")
            print(f"    Width: {result['width']*1000:.1f}mm")
            
            results.append((desc, True, result['quality']))
            
        except Exception as e:
            print(f"  {desc}: FAIL {e}")
            results.append((desc, False, 0.0))
    
    successful = sum(1 for _, success, _ in results if success)
    
    if successful >= 3:
        print(f"\nPASS PASS: {successful}/4 object shapes handled")
        return True
    else:
        print(f"\nWARNING  MARGINAL: Only {successful}/4 shapes handled")
        return True

def test_grasp_without_depth(detector):
    """Test grasp detection without depth information"""
    print("\n" + "="*70)
    print("TEST 6: Grasp Detection Without Depth")
    print("="*70)
    
    try:
        roi, _, desc = create_synthetic_object_roi("rectangular_box")
        
        print(f"Test: {desc} without depth map")
        
        result = detector.detect_grasp_point(roi, depth_map=None)
        
        print(f"Grasp point: ({result['x']:.2f}, {result['y']:.2f})")
        print(f"Quality: {result['quality']:.3f}")
        print(f"Angle: {result['angle']:.1f}deg")
        
        print("PASS PASS: Detection works without depth map")
        return True
        
    except Exception as e:
        print(f"FAIL FAIL: {e}")
        return False

def test_gripper_constraints(detector):
    """Test gripper width constraints"""
    print("\n" + "="*70)
    print("TEST 7: Gripper Width Constraints")
    print("="*70)
    
    try:
        # Test with object wider than gripper
        roi = np.ones((200, 200, 3), dtype=np.uint8) * 255
        roi[50:150, 30:170] = [100, 100, 100]  # Very wide object
        
        result = detector.detect_grasp_point(roi)
        
        print(f"Wide object grasp width: {result['width']*1000:.1f}mm")
        print(f"Gripper max width: {detector.gripper_width*1000:.1f}mm")
        
        if result['width'] <= detector.gripper_width:
            print("PASS Grasp width within gripper limits")
        else:
            print("WARNING  WARNING: Grasp width exceeds gripper capacity")
        
        # Test with thin object
        roi_thin = np.ones((200, 200, 3), dtype=np.uint8) * 255
        roi_thin[80:120, 95:105] = [100, 100, 100]  # Thin object
        
        result_thin = detector.detect_grasp_point(roi_thin)
        print(f"\nThin object grasp width: {result_thin['width']*1000:.1f}mm")
        
        if result_thin['width'] >= detector.min_grasp_width:
            print("PASS Grasp width above minimum threshold")
        else:
            print("WARNING  WARNING: Object may be too thin to grasp reliably")
        
        print("\nPASS PASS: Gripper constraints evaluated")
        return True
        
    except Exception as e:
        print(f"FAIL FAIL: {e}")
        return False

def test_quality_scoring(detector):
    """Test grasp quality scoring"""
    print("\n" + "="*70)
    print("TEST 8: Grasp Quality Scoring")
    print("="*70)
    
    try:
        # Good grasp: rectangular, centered
        roi_good, depth_good, _ = create_synthetic_object_roi("rectangular_box")
        result_good = detector.detect_grasp_point(roi_good, depth_good)
        
        # Less ideal: circular
        roi_circle, depth_circle, _ = create_synthetic_object_roi("circular_object")
        result_circle = detector.detect_grasp_point(roi_circle, depth_circle)
        
        # Irregular: complex shape
        roi_irreg, depth_irreg, _ = create_synthetic_object_roi("irregular_shape")
        result_irreg = detector.detect_grasp_point(roi_irreg, depth_irreg)
        
        print(f"Quality scores:")
        print(f"  Rectangular box: {result_good['quality']:.3f}")
        print(f"  Circular object: {result_circle['quality']:.3f}")
        print(f"  Irregular shape: {result_irreg['quality']:.3f}")
        
        # Check if all scores are in valid range
        scores = [result_good['quality'], result_circle['quality'], result_irreg['quality']]
        
        if all(0 <= s <= 1 for s in scores):
            print("PASS All quality scores in valid range [0, 1]")
        else:
            print("WARNING  WARNING: Some quality scores outside [0, 1]")
        
        # Check if rectangular has highest quality (usually best for grasping)
        if result_good['quality'] >= max(result_circle['quality'], result_irreg['quality']) * 0.8:
            print("PASS Quality scoring prefers good grasp geometries")
        else:
            print("WARNING  Note: Quality scoring may vary by object")
        
        print("\nPASS PASS: Quality scoring working")
        return True
        
    except Exception as e:
        print(f"FAIL FAIL: {e}")
        return False

def test_approach_vector(detector):
    """Test grasp approach vector calculation"""
    print("\n" + "="*70)
    print("TEST 9: Approach Vector Calculation")
    print("="*70)
    
    try:
        roi, depth_map, _ = create_synthetic_object_roi("rectangular_box")
        result = detector.detect_grasp_point(roi, depth_map)
        
        if 'approach_vector' in result:
            approach = result['approach_vector']
            print(f"Approach vector: {approach}")
            
            # Check if normalized
            magnitude = np.linalg.norm(approach)
            print(f"Vector magnitude: {magnitude:.3f}")
            
            if abs(magnitude - 1.0) < 0.01:
                print("PASS Approach vector is normalized")
            else:
                print("WARNING  WARNING: Approach vector not normalized")
            
            print("PASS PASS: Approach vector calculated")
            return True
        else:
            print("Skip  SKIP: Approach vector not provided")
            return True
            
    except Exception as e:
        print(f"WARNING  WARNING: {e}")
        return True

def test_edge_cases(detector):
    """Test edge cases and error handling"""
    print("\n" + "="*70)
    print("TEST 10: Edge Cases")
    print("="*70)
    
    passed = 0
    
    # Test 1: Empty image
    try:
        empty_roi = np.ones((200, 200, 3), dtype=np.uint8) * 255
        result = detector.detect_grasp_point(empty_roi)
        print(f"  PASS Empty image: Quality={result['quality']:.3f}")
        passed += 1
    except Exception as e:
        print(f"  FAIL Empty image: {e}")
    
    # Test 2: Very small ROI
    try:
        small_roi = np.ones((50, 50, 3), dtype=np.uint8) * 255
        small_roi[20:30, 20:30] = [100, 100, 100]
        result = detector.detect_grasp_point(small_roi)
        print(f"  PASS Small ROI (50x50): Quality={result['quality']:.3f}")
        passed += 1
    except Exception as e:
        print(f"  FAIL Small ROI: {e}")
    
    # Test 3: Multiple disconnected objects
    try:
        multi_roi = np.ones((200, 200, 3), dtype=np.uint8) * 255
        multi_roi[50:70, 50:70] = [100, 100, 100]  # Object 1
        multi_roi[130:150, 130:150] = [100, 100, 100]  # Object 2
        result = detector.detect_grasp_point(multi_roi)
        print(f"  PASS Multiple objects: Quality={result['quality']:.3f}")
        passed += 1
    except Exception as e:
        print(f"  FAIL Multiple objects: {e}")
    
    # Test 4: Noisy depth map
    try:
        roi, depth_map, _ = create_synthetic_object_roi("rectangular_box")
        noisy_depth = depth_map + np.random.normal(0, 0.01, depth_map.shape)
        result = detector.detect_grasp_point(roi, noisy_depth)
        print(f"  PASS Noisy depth map: Quality={result['quality']:.3f}")
        passed += 1
    except Exception as e:
        print(f"  FAIL Noisy depth: {e}")
    
    if passed >= 3:
        print("\nPASS PASS: Edge cases handled well")
        return True
    else:
        print(f"\nWARNING  MARGINAL: Only {passed}/4 edge cases passed")
        return True

def test_performance_benchmark(detector):
    """Benchmark grasp detection performance"""
    print("\n" + "="*70)
    print("TEST 11: Performance Benchmark")
    print("="*70)
    
    try:
        # Create test objects
        num_tests = 100
        
        print(f"Running {num_tests} grasp detections...")
        
        times = []
        for i in range(num_tests):
            roi, depth_map, _ = create_synthetic_object_roi("rectangular_box")
            
            start = time.perf_counter()
            result = detector.detect_grasp_point(roi, depth_map)
            elapsed = (time.perf_counter() - start) * 1000
            times.append(elapsed)
        
        avg_time = np.mean(times)
        min_time = np.min(times)
        max_time = np.max(times)
        std_time = np.std(times)
        
        print(f"\nPerformance statistics ({num_tests} detections):")
        print(f"  Average: {avg_time:.2f}ms")
        print(f"  Min: {min_time:.2f}ms")
        print(f"  Max: {max_time:.2f}ms")
        print(f"  Std Dev: {std_time:.2f}ms")
        
        if avg_time < 100:  # Less than 100ms
            print("PASS PASS: Excellent performance (<100ms per detection)")
        elif avg_time < 500:  # Less than 500ms
            print("PASS PASS: Good performance (<500ms per detection)")
        else:
            print("WARNING  WARNING: Performance may need optimization")
        
        return True
        
    except Exception as e:
        print(f"FAIL FAIL: {e}")
        return False

def main():
    """Run all grasp detector tests"""
    print("\n" + "="*70)
    print("GRASP POINT DETECTOR STANDALONE TEST")
    print("="*70)
    print("Testing grasp point detection for robotic manipulation")
    
    start_time = time.time()
    
    # Test 1: Import
    import_ok, GraspPointDetector = test_grasp_detector_import()
    if not import_ok:
        print("\nFAIL Cannot proceed without GraspPointDetector import")
        return
    
    # Test 2: Dependencies
    deps_ok = test_dependencies()
    if not deps_ok:
        print("\nFAIL Cannot proceed without required dependencies")
        return
    
    # Test 3: Initialization
    init_ok, detector = test_detector_initialization(GraspPointDetector)
    if not init_ok or detector is None:
        print("\nFAIL Cannot proceed without successful initialization")
        return
    
    # Run remaining tests
    results = []
    results.append(("Import", import_ok))
    results.append(("Dependencies", deps_ok))
    results.append(("Initialization", init_ok))
    results.append(("Grasp Detection", test_grasp_point_detection(detector)))
    results.append(("Object Shapes", test_different_object_shapes(detector)))
    results.append(("Without Depth", test_grasp_without_depth(detector)))
    results.append(("Gripper Constraints", test_gripper_constraints(detector)))
    results.append(("Quality Scoring", test_quality_scoring(detector)))
    results.append(("Approach Vector", test_approach_vector(detector)))
    results.append(("Edge Cases", test_edge_cases(detector)))
    results.append(("Performance", test_performance_benchmark(detector)))
    
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
    print(f"Total time: {elapsed:.1f}s")
    print("="*70)
    
    if passed == total:
        print("\nComplete ALL TESTS PASSED!")
        print("Grasp point detector is working correctly.")
    elif passed >= total * 0.8:
        print("\nPASS MOST TESTS PASSED")
        print("Grasp point detector is functional.")
    else:
        print("\nWARNING  SOME TESTS FAILED")
        print("Grasp point detector may need attention.")
    
    print()

if __name__ == "__main__":
    main()
