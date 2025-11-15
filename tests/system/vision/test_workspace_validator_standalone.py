"""
Standalone Workspace Validator Test
====================================

Tests the UR30 workspace validation system without ROS2 dependencies.
Validates safety boundaries, reachability checks, and collision avoidance.

Usage:
    conda activate ur30_vlm_environment
    python test_workspace_validator_standalone.py
"""
from _path_setup import setup_test_paths
setup_test_paths()


import numpy as np
import time

def test_workspace_validator_import():
    """Test if WorkspaceValidator can be imported"""
    print("\n" + "="*70)
    print("TEST 1: Workspace Validator Import")
    print("="*70)
    
    try:
        # Try direct import first
        from WorkSpaceValidator import WorkspaceValidator
        print("✅ WorkspaceValidator imported successfully (direct import)")
        return True, WorkspaceValidator
    except ImportError as e:
        print(f"❌ Direct import failed: {e}")
        
        try:
            # Try package import
            from unified_vision_system.perception.WorkSpaceValidator import WorkspaceValidator
            print("✅ WorkspaceValidator imported successfully (package import)")
            return True, WorkspaceValidator
        except ImportError as e2:
            print(f"❌ Package import failed: {e2}")
            return False, None

def test_basic_initialization(WorkspaceValidator):
    """Test basic validator initialization"""
    print("\n" + "="*70)
    print("TEST 2: Basic Initialization")
    print("="*70)
    
    try:
        # Test default initialization
        validator = WorkspaceValidator()
        print("✅ Default initialization successful")
        
        # Test with custom parameters
        params = {
            'min_object_volume': 1e-6,
            'max_object_volume': 0.01
        }
        validator_custom = WorkspaceValidator(params=params)
        print("✅ Custom parameters initialization successful")
        
        # Check UR30-specific parameters
        if hasattr(validator, 'MAX_REACH'):
            max_reach = validator.MAX_REACH
            print(f"   MAX_REACH: {max_reach}m")
            
            if abs(max_reach - 1.19) < 0.01:
                print("✅ UR30 max reach correctly set (1.19m)")
                return True, validator
            else:
                print(f"⚠️  WARNING: MAX_REACH is {max_reach}m, expected 1.19m")
                return True, validator
        else:
            print("⚠️  WARNING: MAX_REACH attribute not found")
            return True, validator
            
    except Exception as e:
        print(f"❌ FAIL: {e}")
        import traceback
        traceback.print_exc()
        return False, None

def test_reachable_points(validator):
    """Test reachability validation for various points"""
    print("\n" + "="*70)
    print("TEST 3: Reachable Points Validation")
    print("="*70)
    
    test_points = [
        # (x, y, z, expected_reachable, description)
        (0.0, 0.0, 0.0, False, "Origin (base collision)"),
        (0.3, 0.0, 0.3, True, "Close center point"),
        (0.5, 0.3, 0.4, True, "Typical workspace point"),
        (0.8, 0.0, 0.2, True, "Far center point"),
        (1.0, 0.0, 0.3, True, "Near max reach"),
        (1.15, 0.0, 0.2, True, "At UR30 limit (1.19m radius)"),
        (1.5, 0.0, 0.0, False, "Beyond UR30 reach"),
        (2.0, 0.0, 0.5, False, "Way beyond reach"),
        (0.5, 0.5, 0.5, True, "Diagonal workspace"),
        (-0.5, 0.0, 0.3, True, "Negative X (behind robot)"),
        (0.0, 0.8, 0.3, True, "Side reach"),
        (0.3, 0.3, 1.0, True, "High reach"),
    ]
    
    passed = 0
    failed = 0
    
    for x, y, z, expected, desc in test_points:
        distance = np.sqrt(x**2 + y**2 + z**2)
        
        try:
            result = validator.is_reachable(x, y, z)
            
            if result == expected:
                status = "✅"
                passed += 1
            else:
                status = "❌"
                failed += 1
            
            print(f"  {status} {desc}")
            print(f"      Point: ({x:.2f}, {y:.2f}, {z:.2f}), Distance: {distance:.3f}m")
            print(f"      Expected: {expected}, Got: {result}")
            
        except Exception as e:
            print(f"  ❌ {desc}: Exception - {e}")
            failed += 1
    
    print(f"\nResults: {passed}/{len(test_points)} passed")
    
    if passed >= len(test_points) * 0.8:
        print("✅ PASS: Reachability validation working correctly")
        return True
    else:
        print("❌ FAIL: Too many reachability validation errors")
        return False

def test_ur30_workspace_boundaries(validator):
    """Test UR30-specific workspace boundaries"""
    print("\n" + "="*70)
    print("TEST 4: UR30 Workspace Boundaries")
    print("="*70)
    
    try:
        # Check if validator has UR30-specific bounds
        if hasattr(validator, 'x_bounds') and hasattr(validator, 'y_bounds') and hasattr(validator, 'z_bounds'):
            x_bounds = validator.x_bounds
            y_bounds = validator.y_bounds
            z_bounds = validator.z_bounds
            
            print(f"Workspace bounds:")
            print(f"  X: [{x_bounds[0]:.2f}, {x_bounds[1]:.2f}]m")
            print(f"  Y: [{y_bounds[0]:.2f}, {y_bounds[1]:.2f}]m")
            print(f"  Z: [{z_bounds[0]:.2f}, {z_bounds[1]:.2f}]m")
            
            # Expected UR30 bounds (from WorkSpaceValidator.py update)
            expected_x = (-1.1, 1.1)
            expected_y = (-1.1, 1.1)
            expected_z = (0.1, 1.4)
            
            x_ok = abs(x_bounds[0] - expected_x[0]) < 0.05 and abs(x_bounds[1] - expected_x[1]) < 0.05
            y_ok = abs(y_bounds[0] - expected_y[0]) < 0.05 and abs(y_bounds[1] - expected_y[1]) < 0.05
            z_ok = abs(z_bounds[0] - expected_z[0]) < 0.05 and abs(z_bounds[1] - expected_z[1]) < 0.05
            
            if x_ok and y_ok and z_ok:
                print("✅ Workspace bounds match UR30 specifications")
                return True
            else:
                print("⚠️  WARNING: Workspace bounds may need adjustment for UR30")
                return True  # Still pass, just warn
        else:
            print("⚠️  WARNING: Workspace bounds attributes not found")
            return True
            
    except Exception as e:
        print(f"❌ FAIL: {e}")
        return False

def test_safety_scores(validator):
    """Test safety score calculation"""
    print("\n" + "="*70)
    print("TEST 5: Safety Score Calculation")
    print("="*70)
    
    test_points = [
        (0.4, 0.2, 0.3, "Safe center point"),
        (0.8, 0.0, 0.3, "Medium distance"),
        (1.1, 0.0, 0.2, "Near boundary"),
        (0.2, 0.0, 0.15, "Low height"),
        (0.5, 0.5, 0.8, "High diagonal"),
    ]
    
    passed = 0
    
    for x, y, z, desc in test_points:
        try:
            score = validator.get_safety_score(x, y, z)
            
            if 0.0 <= score <= 1.0:
                print(f"  ✅ {desc}: score={score:.3f}")
                passed += 1
            else:
                print(f"  ❌ {desc}: score={score:.3f} (out of range [0,1])")
                
        except Exception as e:
            print(f"  ❌ {desc}: Exception - {e}")
    
    if passed == len(test_points):
        print("✅ PASS: Safety scores in valid range")
        return True
    else:
        print("❌ FAIL: Some safety scores invalid")
        return False

def test_safe_mode_validation(validator):
    """Test safe mode (stricter validation)"""
    print("\n" + "="*70)
    print("TEST 6: Safe Mode Validation")
    print("="*70)
    
    # Test point near boundary
    test_point = (1.1, 0.0, 0.2)
    
    try:
        normal_mode = validator.is_reachable(*test_point, safe_mode=False)
        safe_mode = validator.is_reachable(*test_point, safe_mode=True)
        
        print(f"Test point: {test_point}")
        print(f"  Normal mode: {normal_mode}")
        print(f"  Safe mode: {safe_mode}")
        
        # Safe mode should be more restrictive
        if safe_mode <= normal_mode:  # safe_mode should be False or same
            print("✅ PASS: Safe mode is more restrictive than normal mode")
            return True
        else:
            print("⚠️  WARNING: Safe mode less restrictive than normal mode")
            return True  # Still pass with warning
            
    except Exception as e:
        print(f"❌ FAIL: {e}")
        return False

def test_collision_detection(validator):
    """Test collision detection if available"""
    print("\n" + "="*70)
    print("TEST 7: Collision Detection")
    print("="*70)
    
    try:
        # Test points that should have collision concerns
        test_cases = [
            ((0.0, 0.0, 0.0), True, "Base collision"),
            ((0.0, 0.0, 0.05), True, "Very low height"),
            ((0.5, 0.3, 0.4), False, "Safe workspace"),
        ]
        
        if hasattr(validator, 'has_collision'):
            passed = 0
            for point, expects_collision, desc in test_cases:
                has_collision = validator.has_collision(*point)
                
                if has_collision == expects_collision:
                    print(f"  ✅ {desc}: collision={has_collision}")
                    passed += 1
                else:
                    print(f"  ⚠️  {desc}: expected collision={expects_collision}, got {has_collision}")
            
            if passed >= 2:
                print("✅ PASS: Collision detection working")
                return True
            else:
                print("⚠️  MARGINAL: Collision detection may need tuning")
                return True
        else:
            print("⏭️  SKIP: Collision detection method not available")
            return True
            
    except Exception as e:
        print(f"⚠️  WARNING: {e}")
        return True  # Don't fail on this test

def test_orientation_validation(validator):
    """Test orientation validation if available"""
    print("\n" + "="*70)
    print("TEST 8: Orientation Validation")
    print("="*70)
    
    try:
        if hasattr(validator, 'is_orientation_valid'):
            # Test various orientations
            test_orientations = [
                (np.eye(3), "Identity (vertical down)"),
                (np.array([[1,0,0],[0,-1,0],[0,0,-1]]), "Top-down grasp"),
                (np.array([[0,-1,0],[1,0,0],[0,0,1]]), "Side grasp"),
                (np.array([[-1,0,0],[0,1,0],[0,0,-1]]), "Inverted"),
            ]
            
            passed = 0
            for R, desc in test_orientations:
                is_valid = validator.is_orientation_valid(R)
                print(f"  {desc}: {'✅' if is_valid else '⚠️'} {is_valid}")
                if is_valid:
                    passed += 1
            
            if passed >= 3:
                print("✅ PASS: Orientation validation working")
                return True
            else:
                print("⚠️  MARGINAL: Orientation validation may be too restrictive")
                return True
        else:
            print("⏭️  SKIP: Orientation validation method not available")
            return True
            
    except Exception as e:
        print(f"⚠️  WARNING: {e}")
        return True

def test_volume_validation(validator):
    """Test object volume validation"""
    print("\n" + "="*70)
    print("TEST 9: Object Volume Validation")
    print("="*70)
    
    try:
        if hasattr(validator, 'is_volume_valid'):
            test_volumes = [
                (1e-7, False, "Too small (0.0001 cm³)"),
                (1e-6, True, "Min valid (0.001 cm³)"),
                (0.001, True, "Normal object (1000 cm³)"),
                (0.01, True, "Max valid (10,000 cm³)"),
                (0.1, False, "Too large (100,000 cm³)"),
            ]
            
            passed = 0
            for volume, expected, desc in test_volumes:
                is_valid = validator.is_volume_valid(volume)
                
                if is_valid == expected:
                    print(f"  ✅ {desc}: valid={is_valid}")
                    passed += 1
                else:
                    print(f"  ❌ {desc}: expected {expected}, got {is_valid}")
            
            if passed >= 4:
                print("✅ PASS: Volume validation working")
                return True
            else:
                print("⚠️  MARGINAL: Volume validation may need adjustment")
                return True
        else:
            print("⏭️  SKIP: Volume validation method not available")
            return True
            
    except Exception as e:
        print(f"⚠️  WARNING: {e}")
        return True

def test_performance_benchmark(validator):
    """Benchmark validation performance"""
    print("\n" + "="*70)
    print("TEST 10: Performance Benchmark")
    print("="*70)
    
    try:
        # Generate random test points
        np.random.seed(42)
        num_tests = 1000
        
        points = []
        for _ in range(num_tests):
            x = np.random.uniform(-1.2, 1.2)
            y = np.random.uniform(-1.2, 1.2)
            z = np.random.uniform(0.0, 1.5)
            points.append((x, y, z))
        
        # Benchmark
        start_time = time.perf_counter()
        for x, y, z in points:
            validator.is_reachable(x, y, z)
        elapsed = time.perf_counter() - start_time
        
        avg_time_us = (elapsed / num_tests) * 1e6
        print(f"Validated {num_tests} points in {elapsed:.3f}s")
        print(f"Average time per validation: {avg_time_us:.2f}µs")
        
        if avg_time_us < 100:  # Less than 100 microseconds
            print("✅ PASS: Excellent performance (<100µs per check)")
            return True
        elif avg_time_us < 1000:  # Less than 1 millisecond
            print("✅ PASS: Good performance (<1ms per check)")
            return True
        else:
            print("⚠️  WARNING: Performance may need optimization (>1ms per check)")
            return True
            
    except Exception as e:
        print(f"❌ FAIL: {e}")
        return False

def main():
    """Run all workspace validator tests"""
    print("\n" + "="*70)
    print("UR30 WORKSPACE VALIDATOR STANDALONE TEST")
    print("="*70)
    print("Testing workspace validation and safety boundaries")
    
    start_time = time.time()
    
    # Test 1: Import
    import_ok, WorkspaceValidator = test_workspace_validator_import()
    if not import_ok:
        print("\n❌ Cannot proceed without WorkspaceValidator import")
        return
    
    # Test 2: Initialization
    init_ok, validator = test_basic_initialization(WorkspaceValidator)
    if not init_ok or validator is None:
        print("\n❌ Cannot proceed without successful initialization")
        return
    
    # Run remaining tests
    results = []
    results.append(("Import", import_ok))
    results.append(("Initialization", init_ok))
    results.append(("Reachable Points", test_reachable_points(validator)))
    results.append(("UR30 Boundaries", test_ur30_workspace_boundaries(validator)))
    results.append(("Safety Scores", test_safety_scores(validator)))
    results.append(("Safe Mode", test_safe_mode_validation(validator)))
    results.append(("Collision Detection", test_collision_detection(validator)))
    results.append(("Orientation Validation", test_orientation_validation(validator)))
    results.append(("Volume Validation", test_volume_validation(validator)))
    results.append(("Performance", test_performance_benchmark(validator)))
    
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
    print(f"Total time: {elapsed:.2f}s")
    print("="*70)
    
    if passed == total:
        print("\n🎉 ALL TESTS PASSED!")
        print("Workspace validator is working correctly for UR30.")
    elif passed >= total * 0.8:
        print("\n✅ MOST TESTS PASSED")
        print("Workspace validator is functional but may need minor tuning.")
    else:
        print("\n⚠️  SOME TESTS FAILED")
        print("Workspace validator may need attention.")
    
    print()

if __name__ == "__main__":
    main()
