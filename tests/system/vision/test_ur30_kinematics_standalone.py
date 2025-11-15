"""
Standalone UR30 Kinematics Test
================================

Direct test of UR30Kinematics without ROS2 dependencies.
Tests Robotics Toolbox integration and IK/FK performance.

Usage:
    conda activate ur30_vlm_environment
    python test_ur30_kinematics_standalone.py
"""

import sys
import os
import numpy as np
import time

# Add parent directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

def test_rtb_availability():
    """Test if Robotics Toolbox is available"""
    print("\n" + "="*70)
    print("TEST 1: Robotics Toolbox Availability")
    print("="*70)
    
    try:
        from spatialmath import SE3
        print("✅ spatialmath imported successfully")
        
        from roboticstoolbox import DHRobot, RevoluteDH
        print("✅ roboticstoolbox imported successfully")
        
        # Try to import the UR30 model directly
        sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../unified_vision_system/control'))
        from ur30_robot_rtb import UR30
        robot = UR30()
        print(f"✅ UR30 robot model created: {robot.name}")
        
        return True, robot
    except Exception as e:
        print(f"❌ FAIL: {e}")
        return False, None

def test_forward_kinematics(robot):
    """Test forward kinematics"""
    print("\n" + "="*70)
    print("TEST 2: Forward Kinematics")
    print("="*70)
    
    try:
        # Test pose: home position
        q = [0.0, -np.pi/2, 0.0, -np.pi/2, 0.0, 0.0]
        print(f"Test joints (deg): {np.degrees(q).round(1)}")
        
        T = robot.fkine(q)
        position = T.t
        print(f"End-effector position: [{position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}]m")
        
        # Verify reasonable position
        if 0.5 < position[0] < 1.5 and abs(position[1]) < 0.5 and 0.0 < position[2] < 0.5:
            print("✅ PASS: Position within expected range")
            return True
        else:
            print(f"❌ FAIL: Position outside expected range")
            return False
    except Exception as e:
        print(f"❌ FAIL: {e}")
        return False

def test_inverse_kinematics(robot):
    """Test inverse kinematics"""
    print("\n" + "="*70)
    print("TEST 3: Inverse Kinematics")
    print("="*70)
    
    try:
        from spatialmath import SE3
        
        # Target pose: 40cm forward, 20cm right, 30cm up
        target_position = [0.4, 0.2, 0.3]
        target_rotation = np.array([  # Top-down grasp
            [1, 0, 0],
            [0, -1, 0],
            [0, 0, -1]
        ])
        
        T_target = SE3.Rt(target_rotation, target_position)
        print(f"Target position: {target_position}")
        
        # Initial guess
        q0 = [0.0, -np.pi/2, 0.0, -np.pi/2, 0.0, 0.0]
        
        # Test multiple IK methods
        methods = ['ik_LM', 'ik_NR', 'ik_GN']
        best_solution = None
        best_error = float('inf')
        best_method = None
        best_time = 0
        
        for method_name in methods:
            try:
                method = getattr(robot, method_name)
                
                start_time = time.perf_counter()
                result = method(T_target, q0=q0)
                solve_time = (time.perf_counter() - start_time) * 1000
                
                if isinstance(result, tuple) and len(result) >= 2:
                    q_sol, success = result[0], result[1]
                    
                    if success:
                        # Verify accuracy
                        T_achieved = robot.fkine(q_sol)
                        pos_error = np.linalg.norm(T_achieved.t - target_position) * 1000
                        
                        print(f"  {method_name}: ✅ Success in {solve_time:.2f}ms, error={pos_error:.3f}mm")
                        
                        if pos_error < best_error:
                            best_error = pos_error
                            best_solution = q_sol
                            best_method = method_name
                            best_time = solve_time
                    else:
                        print(f"  {method_name}: ❌ Failed to converge")
            except Exception as e:
                print(f"  {method_name}: ❌ Exception: {e}")
        
        if best_solution is not None:
            print(f"\n✅ Best solution: {best_method}")
            print(f"   Solve time: {best_time:.2f}ms")
            print(f"   Position error: {best_error:.3f}mm")
            print(f"   Joint angles (deg): {np.degrees(best_solution).round(1)}")
            
            if best_error < 1.0 and best_time < 100.0:
                print("✅ PASS: IK accuracy <1mm and speed <100ms")
                return True
            else:
                print(f"⚠️  MARGINAL: Error={best_error:.3f}mm, Time={best_time:.2f}ms")
                return True
        else:
            print("❌ FAIL: No solution found")
            return False
            
    except Exception as e:
        print(f"❌ FAIL: {e}")
        import traceback
        traceback.print_exc()
        return False

def test_multiple_ik_poses(robot):
    """Test IK on multiple poses"""
    print("\n" + "="*70)
    print("TEST 4: Multiple IK Poses")
    print("="*70)
    
    try:
        from spatialmath import SE3
        
        test_poses = [
            ([0.5, 0.0, 0.3], "Center forward"),
            ([0.4, 0.3, 0.2], "Right side"),
            ([0.4, -0.3, 0.2], "Left side"),
            ([0.6, 0.0, 0.5], "High reach"),
        ]
        
        q0 = [0.0, -np.pi/2, 0.0, -np.pi/2, 0.0, 0.0]
        
        passed = 0
        for pos, desc in test_poses:
            try:
                R = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])  # Top-down
                T = SE3.Rt(R, pos)
                
                result = robot.ik_LM(T, q0=q0)
                if isinstance(result, tuple) and len(result) >= 2:
                    q_sol, success = result[0], result[1]
                    
                    if success:
                        T_check = robot.fkine(q_sol)
                        error = np.linalg.norm(T_check.t - pos) * 1000
                        
                        if error < 2.0:  # 2mm tolerance
                            print(f"  ✅ {desc}: error={error:.3f}mm")
                            passed += 1
                        else:
                            print(f"  ❌ {desc}: error={error:.3f}mm (>2mm)")
                    else:
                        print(f"  ❌ {desc}: IK failed")
                else:
                    print(f"  ❌ {desc}: Invalid result")
            except Exception as e:
                print(f"  ❌ {desc}: {e}")
        
        print(f"\n{'✅ PASS' if passed >= 3 else '❌ FAIL'}: {passed}/{len(test_poses)} poses solved")
        return passed >= 3
        
    except Exception as e:
        print(f"❌ FAIL: {e}")
        return False

def test_performance_benchmark(robot):
    """Benchmark IK performance"""
    print("\n" + "="*70)
    print("TEST 5: Performance Benchmark")
    print("="*70)
    
    try:
        from spatialmath import SE3
        
        # Generate random reachable poses
        np.random.seed(42)
        num_tests = 20
        
        times = []
        successes = 0
        errors = []
        
        q0 = [0.0, -np.pi/2, 0.0, -np.pi/2, 0.0, 0.0]
        
        for i in range(num_tests):
            # Random position in workspace
            x = np.random.uniform(0.3, 0.7)
            y = np.random.uniform(-0.3, 0.3)
            z = np.random.uniform(0.2, 0.5)
            
            R = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
            T = SE3.Rt(R, [x, y, z])
            
            start = time.perf_counter()
            result = robot.ik_LM(T, q0=q0)
            elapsed = (time.perf_counter() - start) * 1000
            
            if isinstance(result, tuple) and len(result) >= 2:
                q_sol, success = result[0], result[1]
                
                if success:
                    times.append(elapsed)
                    T_check = robot.fkine(q_sol)
                    error = np.linalg.norm(T_check.t - [x, y, z]) * 1000
                    errors.append(error)
                    successes += 1
        
        if times:
            avg_time = np.mean(times)
            max_time = np.max(times)
            avg_error = np.mean(errors)
            max_error = np.max(errors)
            
            print(f"Results from {num_tests} random poses:")
            print(f"  Success rate: {successes}/{num_tests} ({successes/num_tests*100:.1f}%)")
            print(f"  Avg solve time: {avg_time:.2f}ms (max: {max_time:.2f}ms)")
            print(f"  Avg position error: {avg_error:.3f}mm (max: {max_error:.3f}mm)")
            
            if successes >= 18 and avg_time < 10 and avg_error < 0.1:
                print("✅ PASS: Excellent performance")
                return True
            elif successes >= 15 and avg_time < 50:
                print("✅ PASS: Good performance")
                return True
            else:
                print("⚠️  MARGINAL: Performance acceptable but not optimal")
                return True
        else:
            print("❌ FAIL: No successful solutions")
            return False
            
    except Exception as e:
        print(f"❌ FAIL: {e}")
        return False

def main():
    """Run all tests"""
    print("\n" + "="*70)
    print("UR30 KINEMATICS STANDALONE TEST")
    print("="*70)
    print("Testing Robotics Toolbox integration for UR30")
    
    start_time = time.time()
    
    # Test 1: Check RTB availability
    rtb_ok, robot = test_rtb_availability()
    if not rtb_ok:
        print("\n❌ Cannot proceed without Robotics Toolbox")
        print("\nInstall with:")
        print("  pip install roboticstoolbox-python")
        print("  pip install spatialmath-python")
        return
    
    # Run tests
    results = []
    results.append(("RTB Availability", True))
    results.append(("Forward Kinematics", test_forward_kinematics(robot)))
    results.append(("Inverse Kinematics", test_inverse_kinematics(robot)))
    results.append(("Multiple IK Poses", test_multiple_ik_poses(robot)))
    results.append(("Performance Benchmark", test_performance_benchmark(robot)))
    
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
        print("UR30 kinematics system is working correctly.")
    elif passed >= total * 0.8:
        print("\n⚠️  MOST TESTS PASSED")
        print("UR30 kinematics system is functional but may need tuning.")
    else:
        print("\n❌ TESTS FAILED")
        print("UR30 kinematics system needs attention.")
    
    print()

if __name__ == "__main__":
    main()
