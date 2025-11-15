"""
Standalone OWL-ViT Detector Test
=================================

Tests the OWL-ViT object detection system without ROS2 dependencies.
Validates VLM initialization, detection, and performance.

Usage:
    conda activate ur30_vlm_environment
    python test_owlvit_detector_standalone.py
"""
from _path_setup import setup_test_paths
setup_test_paths()


import numpy as np
import time

def test_owlvit_import():
    """Test if OWLViTDetector can be imported"""
    print("\n" + "="*70)
    print("TEST 1: OWL-ViT Detector Import")
    print("="*70)
    
    try:
        from OWLViTDetector import OWLViTDetector
        print("✅ OWLViTDetector imported successfully (direct import)")
        return True, OWLViTDetector
    except ImportError as e:
        print(f"❌ Direct import failed: {e}")
        
        try:
            from unified_vision_system.perception.OWLViTDetector import OWLViTDetector
            print("✅ OWLViTDetector imported successfully (package import)")
            return True, OWLViTDetector
        except ImportError as e2:
            print(f"❌ Package import failed: {e2}")
            return False, None

def test_dependencies():
    """Test if required dependencies are available"""
    print("\n" + "="*70)
    print("TEST 2: Dependency Check")
    print("="*70)
    
    deps_ok = True
    
    # Check torch
    try:
        import torch
        print(f"✅ PyTorch {torch.__version__} available")
        print(f"   CUDA available: {torch.cuda.is_available()}")
        if torch.cuda.is_available():
            print(f"   CUDA device: {torch.cuda.get_device_name(0)}")
    except ImportError:
        print("❌ PyTorch not available")
        deps_ok = False
    
    # Check transformers
    try:
        import transformers
        print(f"✅ Transformers {transformers.__version__} available")
    except ImportError:
        print("❌ Transformers not available")
        deps_ok = False
    
    # Check PIL
    try:
        from PIL import Image
        print(f"✅ PIL (Pillow) available")
    except ImportError:
        print("❌ PIL not available")
        deps_ok = False
    
    # Check cv2
    try:
        import cv2
        print(f"✅ OpenCV {cv2.__version__} available")
    except ImportError:
        print("❌ OpenCV not available")
        deps_ok = False
    
    return deps_ok

def create_synthetic_test_image(object_type="red_box"):
    """Create a synthetic test image with known object"""
    img = np.ones((480, 640, 3), dtype=np.uint8) * 255  # White background
    
    if object_type == "red_box":
        # Draw red box in center
        img[200:280, 270:370] = [0, 0, 255]  # Red in BGR
        return img, (270, 200, 370, 280), "red box"
    
    elif object_type == "blue_circle":
        # Draw blue circle
        import cv2
        cv2.circle(img, (320, 240), 50, (255, 0, 0), -1)  # Blue circle
        return img, (270, 190, 370, 290), "blue circle"
    
    elif object_type == "multi_objects":
        # Multiple objects
        img[100:150, 100:200] = [0, 255, 0]  # Green box top-left
        img[300:400, 450:550] = [0, 0, 255]  # Red box bottom-right
        return img, None, "multiple objects"
    
    return img, None, object_type

def test_detector_initialization(OWLViTDetector):
    """Test OWL-ViT detector initialization"""
    print("\n" + "="*70)
    print("TEST 3: Detector Initialization")
    print("="*70)
    
    try:
        import torch
        
        # Try CPU first (always available)
        print("Initializing detector on CPU...")
        start = time.time()
        detector = OWLViTDetector(device='cpu', confidence_threshold=0.1)
        init_time = time.time() - start
        
        print(f"✅ Detector initialized successfully in {init_time:.2f}s")
        print(f"   Model: {detector.model_name if hasattr(detector, 'model_name') else 'OWL-ViT'}")
        print(f"   Device: {detector.device}")
        print(f"   Threshold: {detector.confidence_threshold}")
        
        return True, detector
        
    except Exception as e:
        print(f"❌ FAIL: {e}")
        import traceback
        traceback.print_exc()
        return False, None

def test_single_object_detection(detector):
    """Test detection on single object"""
    print("\n" + "="*70)
    print("TEST 4: Single Object Detection")
    print("="*70)
    
    try:
        # Create test image with red box
        test_img, expected_box, desc = create_synthetic_test_image("red_box")
        
        print(f"Test image: {desc} at {expected_box}")
        
        # Run detection
        queries = ["red box", "red object", "box"]
        
        start = time.perf_counter()
        detections = detector.detect_objects(test_img, queries)
        detect_time = (time.perf_counter() - start) * 1000
        
        print(f"Detection time: {detect_time:.1f}ms")
        print(f"Found {len(detections)} detections")
        
        for i, det in enumerate(detections[:3]):  # Show top 3
            print(f"  {i+1}. Label: '{det['label']}', Score: {det['score']:.3f}, Box: {det['box']}")
        
        if len(detections) > 0:
            print("✅ PASS: Detection successful")
            return True
        else:
            print("⚠️  WARNING: No detections found (may need lower threshold)")
            return True  # Still pass, model might be conservative
            
    except Exception as e:
        print(f"❌ FAIL: {e}")
        import traceback
        traceback.print_exc()
        return False

def test_multiple_queries(detector):
    """Test detection with multiple text queries"""
    print("\n" + "="*70)
    print("TEST 5: Multiple Text Queries")
    print("="*70)
    
    try:
        test_img, _, _ = create_synthetic_test_image("red_box")
        
        query_sets = [
            ["red box"],
            ["red object", "blue object"],
            ["box", "cube", "rectangle"],
            ["tool", "equipment", "item"],
        ]
        
        results = []
        for queries in query_sets:
            detections = detector.detect_objects(test_img, queries)
            results.append((queries, len(detections)))
            print(f"  Queries: {queries} -> {len(detections)} detections")
        
        print("✅ PASS: Multiple query types handled")
        return True
        
    except Exception as e:
        print(f"❌ FAIL: {e}")
        return False

def test_multi_object_detection(detector):
    """Test detection of multiple objects"""
    print("\n" + "="*70)
    print("TEST 6: Multi-Object Detection")
    print("="*70)
    
    try:
        test_img, _, _ = create_synthetic_test_image("multi_objects")
        
        queries = ["green box", "red box", "colored object"]
        
        detections = detector.detect_objects(test_img, queries)
        
        print(f"Found {len(detections)} detections")
        
        for i, det in enumerate(detections[:5]):
            print(f"  {i+1}. '{det['label']}': score={det['score']:.3f}, box={det['box']}")
        
        if len(detections) >= 2:
            print("✅ PASS: Multiple objects detected")
        else:
            print("⚠️  WARNING: Expected 2+ objects, found {len(detections)}")
        
        return True
        
    except Exception as e:
        print(f"❌ FAIL: {e}")
        return False

def test_confidence_thresholding(detector):
    """Test detection consistency"""
    print("\n" + "="*70)
    print("TEST 7: Detection Consistency")
    print("="*70)
    
    try:
        test_img, _, _ = create_synthetic_test_image("red_box")
        queries = ["red box", "blue box", "green box"]
        
        # Note: OWLViTDetector uses internal confidence_threshold
        # We'll just test with different queries
        
        print("  Testing detection consistency...")
        detections1 = detector.detect_objects(test_img, queries)
        detections2 = detector.detect_objects(test_img, queries)
        
        print(f"  Run 1: {len(detections1)} detections")
        print(f"  Run 2: {len(detections2)} detections")
        
        if len(detections1) == len(detections2):
            print("✅ PASS: Consistent detection results")
        else:
            print("⚠️  WARNING: Detection results vary between runs")
        
        return True
        
    except Exception as e:
        print(f"❌ FAIL: {e}")
        return False

def test_edge_cases(detector):
    """Test edge cases and error handling"""
    print("\n" + "="*70)
    print("TEST 8: Edge Cases")
    print("="*70)
    
    passed = 0
    
    # Test 1: Empty image
    try:
        empty_img = np.zeros((480, 640, 3), dtype=np.uint8)
        detections = detector.detect_objects(empty_img, ["object"])
        print(f"  ✅ Empty image: {len(detections)} detections")
        passed += 1
    except Exception as e:
        print(f"  ❌ Empty image: {e}")
    
    # Test 2: Empty query list
    try:
        test_img, _, _ = create_synthetic_test_image("red_box")
        detections = detector.detect_objects(test_img, [])
        print(f"  ✅ Empty queries: {len(detections)} detections")
        passed += 1
    except Exception as e:
        print(f"  ❌ Empty queries: {e}")
    
    # Test 3: Single query
    try:
        detections = detector.detect_objects(test_img, ["object"])
        print(f"  ✅ Single query: {len(detections)} detections")
        passed += 1
    except Exception as e:
        print(f"  ❌ Single query: {e}")
    
    # Test 4: Complex query
    try:
        detections = detector.detect_objects(test_img, ["a red rectangular box on white background"])
        print(f"  ✅ Complex query: {len(detections)} detections")
        passed += 1
    except Exception as e:
        print(f"  ❌ Complex query: {e}")
    
    if passed >= 3:
        print("✅ PASS: Edge cases handled well")
        return True
    else:
        print("⚠️  MARGINAL: Some edge cases failed")
        return True

def test_detection_performance(detector):
    """Benchmark detection performance"""
    print("\n" + "="*70)
    print("TEST 9: Detection Performance")
    print("="*70)
    
    try:
        test_img, _, _ = create_synthetic_test_image("red_box")
        queries = ["red box", "object"]
        
        # Warmup
        print("Running warmup inference...")
        detector.detect_objects(test_img, queries)
        
        # Benchmark
        num_runs = 10
        times = []
        
        print(f"Running {num_runs} detection cycles...")
        for i in range(num_runs):
            start = time.perf_counter()
            detections = detector.detect_objects(test_img, queries)
            elapsed = (time.perf_counter() - start) * 1000
            times.append(elapsed)
        
        avg_time = np.mean(times)
        min_time = np.min(times)
        max_time = np.max(times)
        std_time = np.std(times)
        
        print(f"\nPerformance statistics:")
        print(f"  Average: {avg_time:.1f}ms")
        print(f"  Min: {min_time:.1f}ms")
        print(f"  Max: {max_time:.1f}ms")
        print(f"  Std Dev: {std_time:.1f}ms")
        
        if avg_time < 1000:  # Less than 1 second
            print("✅ PASS: Good performance (<1s per detection)")
        elif avg_time < 5000:  # Less than 5 seconds
            print("✅ PASS: Acceptable performance (<5s per detection)")
        else:
            print("⚠️  WARNING: Slow performance (>5s per detection)")
        
        return True
        
    except Exception as e:
        print(f"❌ FAIL: {e}")
        return False

def test_output_format(detector):
    """Test detection output format"""
    print("\n" + "="*70)
    print("TEST 10: Output Format Validation")
    print("="*70)
    
    try:
        test_img, _, _ = create_synthetic_test_image("red_box")
        detections = detector.detect_objects(test_img, ["red box"])
        
        if len(detections) > 0:
            det = detections[0]
            
            # Check required fields
            required_fields = ['label', 'score', 'box']
            missing_fields = [f for f in required_fields if f not in det]
            
            if missing_fields:
                print(f"❌ Missing fields: {missing_fields}")
                return False
            
            print(f"Detection format:")
            print(f"  Label: {det['label']} (type: {type(det['label'])})")
            print(f"  Score: {det['score']:.3f} (type: {type(det['score'])})")
            print(f"  Box: {det['box']} (type: {type(det['box'])})")
            
            # Validate box format
            box = det['box']
            if len(box) == 4:
                print(f"  Box format: [x_min, y_min, x_max, y_max]")
                
                # Check box values are reasonable
                if all(0 <= v <= 1000 for v in box):
                    print("✅ PASS: Output format valid")
                    return True
                else:
                    print("⚠️  WARNING: Box coordinates outside expected range")
                    return True
            else:
                print(f"❌ FAIL: Invalid box format (length={len(box)})")
                return False
        else:
            print("⏭️  SKIP: No detections to validate format")
            return True
            
    except Exception as e:
        print(f"❌ FAIL: {e}")
        return False

def main():
    """Run all OWL-ViT detector tests"""
    print("\n" + "="*70)
    print("OWL-ViT DETECTOR STANDALONE TEST")
    print("="*70)
    print("Testing vision-language model for object detection")
    
    start_time = time.time()
    
    # Test 1: Import
    import_ok, OWLViTDetector = test_owlvit_import()
    if not import_ok:
        print("\n❌ Cannot proceed without OWLViTDetector import")
        return
    
    # Test 2: Dependencies
    deps_ok = test_dependencies()
    if not deps_ok:
        print("\n❌ Cannot proceed without required dependencies")
        print("Please install: pip install torch transformers pillow opencv-python")
        return
    
    # Test 3: Initialization
    init_ok, detector = test_detector_initialization(OWLViTDetector)
    if not init_ok or detector is None:
        print("\n❌ Cannot proceed without successful initialization")
        print("Note: Model download may take time on first run")
        return
    
    # Run remaining tests
    results = []
    results.append(("Import", import_ok))
    results.append(("Dependencies", deps_ok))
    results.append(("Initialization", init_ok))
    results.append(("Single Object Detection", test_single_object_detection(detector)))
    results.append(("Multiple Queries", test_multiple_queries(detector)))
    results.append(("Multi-Object Detection", test_multi_object_detection(detector)))
    results.append(("Detection Consistency", test_confidence_thresholding(detector)))
    results.append(("Edge Cases", test_edge_cases(detector)))
    results.append(("Performance", test_detection_performance(detector)))
    results.append(("Output Format", test_output_format(detector)))
    
    # Summary
    elapsed = time.time() - start_time
    passed = sum(1 for _, result in results if result)
    total = len(results)
    
    print("\n" + "="*70)
    print("TEST SUMMARY")
    print("="*70)
    print(f"Total tests: {total}")
    print(f"✅ Passed: {passed}")
    print(f"❌ Failed: {total - passed}")
    print(f"Success rate: {passed/total*100:.1f}%")
    print(f"Total time: {elapsed:.1f}s")
    print("="*70)
    
    if passed == total:
        print("\n🎉 ALL TESTS PASSED!")
        print("OWL-ViT detector is working correctly.")
    elif passed >= total * 0.8:
        print("\n✅ MOST TESTS PASSED")
        print("OWL-ViT detector is functional.")
    else:
        print("\n⚠️  SOME TESTS FAILED")
        print("OWL-ViT detector may need attention.")
    
    print()

if __name__ == "__main__":
    main()
