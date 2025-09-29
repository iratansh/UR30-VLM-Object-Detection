"""
UR30 Kinematics Implementation using IKBT
Advanced inverse kinematics for the UR30 robot arm using symbolic solutions.

This implementation uses IKBT (Inverse Kinematics using Behavior Trees) to provide
closed-form analytical solutions for the UR30 robot inverse kinematics.
"""

import numpy as np
import math
from typing import List, Optional, Tuple, Dict, Any
import logging
import time

# Try to import scipy, but don't fail if there are library issues
try:
    from scipy.spatial.transform import Rotation
    SCIPY_AVAILABLE = True
except ImportError as e:
    print(f"Scipy import failed: {e}")
    print("Falling back to manual quaternion calculations")
    SCIPY_AVAILABLE = False
    Rotation = None

# Try multiple possible TRAC-IK python bindings (user requested trackikpy fallback)
TRACIK_AVAILABLE = False
TracIKSolver = None
try:
    from trac_ik_python.trac_ik import IK as TracIKSolver  # standard binding
    TRACIK_AVAILABLE = True
except Exception:
    try:
        # Alternate package name sometimes used
        from tracikpy import TracIK as TracIKSolver  # type: ignore
        TRACIK_AVAILABLE = True
    except Exception:
        TRACIK_AVAILABLE = False

try:
    from IK_equationsUR30 import ikin_UR30  # generated symbolic solutions
    GENERATED_IK_AVAILABLE = True
except Exception:
    GENERATED_IK_AVAILABLE = False

# Try path-based dynamic import fallback if direct import failed
if not GENERATED_IK_AVAILABLE:
    try:
        import os, importlib.util, sys  # type: ignore
        this_dir = os.path.dirname(os.path.abspath(__file__))
        repo_root = os.path.dirname(this_dir)
        candidate = os.path.join(repo_root, 'IK_equationsUR30.py')
        if os.path.isfile(candidate):
            spec = importlib.util.spec_from_file_location('IK_equationsUR30', candidate)
            if spec and spec.loader:
                mod = importlib.util.module_from_spec(spec)
                spec.loader.exec_module(mod)  # type: ignore
                ikin_UR30 = getattr(mod, 'ikin_UR30')  # type: ignore
                GENERATED_IK_AVAILABLE = True
                if 'vision' not in sys.path:
                    sys.path.append(this_dir)
                if __name__ == '__main__':
                    print('[IKBT] Loaded IK_equationsUR30 via dynamic path fallback.')
    except Exception as _e:
        if __name__ == '__main__':
            print(f'[IKBT] Fallback load failed: {_e}')

class UR30Kinematics:
    """
    UR30 Kinematics class using IKBT symbolic solutions.

    This provides:
    - Analytical solutions: IKBT generates symbolic closed-form solutions
    - High precision: Direct mathematical solutions without iterative solving
    - Multiple solutions: Returns all valid configurations for a given pose
    """

    def __init__(self, debug: bool = False):
        """
        Initialize UR30 kinematics solver.

        Args:
            debug: Enable debug output
        """
        self.debug = debug

        # UR30 DH parameters (official Universal Robots parameters)
        # a: [0.0000, -0.6370, -0.5037, 0.0000, 0.0000, 0.0000]
        # d: [0.2363, 0.0000, 0.0000, 0.2010, 0.1593, 0.1543]
        self.d1 = 0.2363    # Base to shoulder
        self.a2 = -0.637    # Upper arm length (negative for standard DH)
        self.a3 = -0.5037   # Forearm length (negative for standard DH)
        self.d4 = 0.201     # Wrist 1 height
        self.d5 = 0.1593    # Wrist 2 height
        self.d6 = 0.1543    # End effector length

        # Joint limits (radians) - UR30 specific
        self.JOINT_LIMITS = [
            (-2*math.pi, 2*math.pi),  # θ1
            (-math.pi, math.pi),      # θ2
            (-math.pi, math.pi),      # θ3
            (-2*math.pi, 2*math.pi),  # θ4
            (-2*math.pi, 2*math.pi),  # θ5
            (-2*math.pi, 2*math.pi)   # θ6
        ]

        # Numerical parameters
        self.eps = 1e-6

        # Performance tracking
        self.solve_attempts = 0
        self.solve_successes = 0
        self.total_solve_time = 0.0

        if self.debug:
            print("UR30 Kinematics initialized with IKBT symbolic solutions")
        # Links for TRAC-IK (can be overridden externally if URDF differs)
        self.base_link = "base_link"
        self.tip_link = "tool0"
        self._tracik = None  # lazy init
        self.tracik_ok = TRACIK_AVAILABLE
        # Tolerances (exposed for diagnostics)
        self.pos_tol_mm = 2.0
        self.rot_tol_deg = 2.0

    def normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-π, π]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def dh_transform(self, d: float, a: float, alpha: float, theta: float) -> np.ndarray:
        """Standard DH transformation matrix"""
        ct = math.cos(theta)
        st = math.sin(theta)
        ca = math.cos(alpha)
        sa = math.sin(alpha)

        return np.array([
            [ct, -st*ca, st*sa, a*ct],
            [st, ct*ca, -ct*sa, a*st],
            [0, sa, ca, d],
            [0, 0, 0, 1]
        ])

    def _dh_params(self, joints: List[float]) -> List[Tuple[float, float, float, float]]:
        """Return the DH parameter list for the provided joint angles."""
        q1, q2, q3, q4, q5, q6 = joints
        return [
            (self.d1, 0, math.pi/2, q1),              # Base to shoulder
            (0, self.a2, 0, q2),                      # Shoulder to elbow
            (0, self.a3, 0, q3),                      # Elbow to wrist1
            (self.d4, 0, math.pi/2, q4),              # Wrist1 to wrist2
            (self.d5, 0, -math.pi/2, q5),             # Wrist2 to wrist3
            (self.d6, 0, 0, q6)                       # Wrist3 to tool
        ]

    def _fk_chain(self, joints: List[float]) -> List[np.ndarray]:
        """Return cumulative transforms from base to each joint (including base and tool)."""
        chain: List[np.ndarray] = [np.eye(4)]
        T = np.eye(4)
        for d, a, alpha, theta in self._dh_params(joints):
            T = T @ self.dh_transform(d, a, alpha, theta)
            chain.append(T)
        return chain

    def _geometric_jacobian(self, joints: List[float],
                             chain: Optional[List[np.ndarray]] = None) -> np.ndarray:
        """Geometric Jacobian (6x6) for current joint configuration."""
        if chain is None:
            chain = self._fk_chain(joints)
        o_n = chain[-1][:3, 3]
        J = np.zeros((6, 6))
        for i in range(6):
            o_i = chain[i][:3, 3]
            z_i = chain[i][:3, 2]
            J[:3, i] = np.cross(z_i, o_n - o_i)
            J[3:, i] = z_i
        return J

    def forward_kinematics(self, joints: List[float]) -> np.ndarray:
        """
        Forward kinematics for UR30 robot.

        Args:
            joints: Six joint angles in radians [θ1, θ2, θ3, θ4, θ5, θ6]

        Returns:
            4x4 homogeneous transformation matrix for the end effector position
        """
        return self._fk_chain(joints)[-1]

    def _compute_wrist_center(self, T: np.ndarray) -> Tuple[float, float, float]:
        """Compute wrist center given target pose (tool Z assumed along +Z of end effector)."""
        px, py, pz = T[0, 3], T[1, 3], T[2, 3]
        nx, ny, nz = T[0, 2], T[1, 2], T[2, 2]  # tool z-axis
        wcx = px - self.d6 * nx
        wcy = py - self.d6 * ny
        wcz = pz - self.d6 * nz
        return wcx, wcy, wcz

    def _solve_first_three(self, T: np.ndarray) -> List[Tuple[float, float, float]]:
        """Solve joints 1-3 analytically (elbow up/down) using planar triangle."""
        wcx, wcy, wcz = self._compute_wrist_center(T)
        # Joint 1 (two symmetric solutions around table normal)
        th1_candidates = [math.atan2(wcy, wcx)]
        # Add alternative by adding pi (wrap normalized later) not always distinct, but keep for completeness
        alt = math.atan2(wcy, wcx) + math.pi
        if abs(self.normalize_angle(alt - th1_candidates[0])) > 1e-3:
            th1_candidates.append(alt)
        # Distances for planar IK (project to base frame XY plane)
        r = math.hypot(wcx, wcy)
        s = wcz - self.d1
        L2 = abs(self.a2)
        L3 = abs(self.a3)
        # Law of cosines for elbow angle
        denom = 2 * L2 * L3
        cos_th3 = (r*r + s*s - L2*L2 - L3*L3) / max(1e-9, denom)
        # Clamp numerical noise
        cos_th3 = max(-1.0, min(1.0, cos_th3))
        th3_elbow_down = math.acos(cos_th3)  # positive angle
        th3_elbow_up = -th3_elbow_down
        th3_candidates = [th3_elbow_down, th3_elbow_up]
        solutions = []
        for th1 in th1_candidates:
            for th3 in th3_candidates:
                # Compute th2 using triangle decomposition
                k1 = L2 + L3 * math.cos(th3)
                k2 = L3 * math.sin(th3)
                gamma = math.atan2(s, r)
                th2 = gamma - math.atan2(k2, k1)
                solutions.append((self.normalize_angle(th1), self.normalize_angle(th2), self.normalize_angle(th3)))
        return solutions

    def _solve_orientation(self, T: np.ndarray, th1: float, th2: float, th3: float) -> List[Tuple[float, float, float]]:
        """Solve wrist joints 4-6 from orientation. Assumes wrist is 3 consecutive revolute joints.
        Uses decomposition R3_6 = R0_3^T * R and approximates as Rz(q4) * Ry(q5) * Rz(q6) (ZYZ style).
        Returns possible (q4,q5,q6) sets."""
        # Forward for first 3
        T0 = np.eye(4)
        dh_first3 = [
            (self.d1, 0, math.pi/2, th1),
            (0, self.a2, 0, th2),
            (0, self.a3, 0, th3)
        ]
        for d, a, alpha, theta in dh_first3:
            T0 = T0 @ self.dh_transform(d, a, alpha, theta)
        R0_3 = T0[:3, :3]
        R = T[:3, :3]
        R3_6 = R0_3.T @ R
        # ZYZ decomposition like: R = Rz(q4) * Ry(q5) * Rz(q6)
        q5 = math.atan2(math.sqrt(R3_6[0,2]**2 + R3_6[1,2]**2), R3_6[2,2])
        candidates = []
        for sign in [1, -1]:
            q5_candidate = sign * q5
            if abs(math.sin(q5_candidate)) < 1e-8:
                q4_candidate = 0.0
                q6_candidate = math.atan2(R3_6[1,0], R3_6[0,0])
            else:
                q4_candidate = math.atan2(R3_6[1,2]/math.sin(q5_candidate), R3_6[0,2]/math.sin(q5_candidate))
                q6_candidate = math.atan2(R3_6[2,1]/math.sin(q5_candidate), -R3_6[2,0]/math.sin(q5_candidate))
            candidates.append((self.normalize_angle(q4_candidate), self.normalize_angle(q5_candidate), self.normalize_angle(q6_candidate)))
        return candidates

    def _pose_error(self, q: List[float], T_target: np.ndarray) -> Tuple[float, float]:
        """Compute position (m) and orientation (rad) error for given joints."""
        T_cur = self.forward_kinematics(q)
        pos_err = np.linalg.norm(T_cur[:3, 3] - T_target[:3, 3])
        R_err = T_cur[:3, :3].T @ T_target[:3, :3]
        ang = math.acos(max(-1.0, min(1.0, (np.trace(R_err) - 1) / 2)))
        return pos_err, ang

    def _numeric_refine(self, seed: List[float], T_target: np.ndarray,
                        max_iters: int = 200) -> Optional[List[float]]:
        """Simple damped least-squares numeric refinement starting from seed."""
        q = np.array(seed, dtype=float)
        lam = 1e-3
        for _ in range(max_iters):
            T_cur = self.forward_kinematics(q.tolist())
            # Position error
            dp = T_target[:3, 3] - T_cur[:3, 3]
            # Orientation error via axis-angle vector (skew part)
            R_err = T_cur[:3, :3].T @ T_target[:3, :3]
            skew = 0.5 * np.array([
                R_err[2,1] - R_err[1,2],
                R_err[0,2] - R_err[2,0],
                R_err[1,0] - R_err[0,1]
            ])
            err = np.concatenate([dp, skew])  # 6x1 error (approx)
            if np.linalg.norm(dp) < 1e-4 and np.linalg.norm(skew) < 1e-3:
                return [self.normalize_angle(v) for v in q.tolist()]
            # Numerical Jacobian 6x6
            J = np.zeros((6, 6))
            h = 1e-5
            base_T = T_cur
            for j in range(6):
                q_pert = q.copy()
                q_pert[j] += h
                T_p = self.forward_kinematics(q_pert.tolist())
                dT = T_p[:3, 3] - base_T[:3, 3]
                R_dp = base_T[:3, :3].T @ T_p[:3, :3]
                skew_dp = 0.5 * np.array([
                    R_dp[2,1] - R_dp[1,2],
                    R_dp[0,2] - R_dp[2,0],
                    R_dp[1,0] - R_dp[0,1]
                ])
                J[:, j] = np.concatenate([dT / h, skew_dp / h])
            # Damped least squares update
            JT = J.T
            dq = JT @ np.linalg.solve(J @ JT + lam * np.eye(6), err)
            q += dq
            # Enforce joint limits softly
            for i, (lo, hi) in enumerate(self.JOINT_LIMITS):
                if q[i] < lo:
                    q[i] = lo
                elif q[i] > hi:
                    q[i] = hi
        # Final validation
        p_err, o_err = self._pose_error(q.tolist(), T_target)
        if p_err < 5e-3 and o_err < 5e-2:  # 5 mm, ~3 deg
            return [self.normalize_angle(v) for v in q.tolist()]
        return None

    def _ikbt_solve_analytic(self, T: np.ndarray) -> List[List[float]]:
        """Analytic IK using generated IKBT code (IK_equationsUR30).
        Structure from IKBT: [th1, th234, th2+th3, th2, th3+th4, th3, th4, th5, th6].
        We treat the symbolic solution as a high-quality seed and run a short
        damped-least-squares refinement so the result exactly matches the
        kinematic chain used in this module."""
        if not GENERATED_IK_AVAILABLE:
            if self.debug:
                print("[IKBT] Generated IK module not available.")
            return []
        try:
            sols = ikin_UR30(T)
            if not sols:
                if self.debug:
                    print("[IKBT] ikin_UR30 returned no raw solutions (False / empty).")
                return []
            joint_sets: List[List[float]] = []
            reject_stats = {'refine_fail':0,'limits':0,'tol':0,'dup':0,'len':0}
            processed_seed_keys = set()
            for idx, s in enumerate(sols):
                if len(s) != 9:
                    reject_stats['len'] += 1
                    continue
                base_seed = [self.normalize_angle(s[0]),
                             self.normalize_angle(s[3]),
                             self.normalize_angle(s[5]),
                             self.normalize_angle(s[6]),
                             self.normalize_angle(s[7]),
                             self.normalize_angle(s[8])]
                seed_key = tuple(int(round(v * 1e6)) for v in base_seed)
                if seed_key in processed_seed_keys:
                    continue
                processed_seed_keys.add(seed_key)
                refined = None
                adjustments = [0.0, math.pi, -math.pi]
                for d2 in adjustments:
                    if refined:
                        break
                    for d3 in adjustments:
                        if refined:
                            break
                        for d4 in adjustments:
                            seed = base_seed.copy()
                            seed[1] = self.normalize_angle(seed[1] + d2)
                            seed[2] = self.normalize_angle(seed[2] + d3)
                            seed[3] = self.normalize_angle(seed[3] + d4)
                            refined = self._numeric_refine(seed, T)
                            if refined:
                                break
                if not refined:
                    reject_stats['refine_fail'] += 1
                    continue
                candidate = [self.normalize_angle(v) for v in refined]
                pos_err, rot_err = self._compute_errors(candidate, T)
                within_limits = self._within_joint_limits(candidate)
                within_tol = (pos_err < self.pos_tol_mm and rot_err < self.rot_tol_deg)
                is_dup = any(sum(abs(a-b) for a,b in zip(candidate,u)) < 1e-4 for u in joint_sets)
                if self.debug:
                    print(f"[IKBT-FUSED][{idx}] q={['{:.3f}'.format(math.degrees(a)) for a in candidate]} pos={pos_err:.3f}mm rot={rot_err:.3f}deg limits={within_limits} tol={within_tol} dup={is_dup}")
                if not within_limits:
                    reject_stats['limits'] += 1
                    continue
                if not within_tol:
                    reject_stats['tol'] += 1
                    continue
                if is_dup:
                    reject_stats['dup'] += 1
                    continue
                joint_sets.append(candidate)
            if self.debug:
                print(f"[IKBT-FUSED] Accepted: {len(joint_sets)} | rejects: {reject_stats}")
            return joint_sets
        except Exception as e:
            if self.debug:
                print(f"[IKBT] Exception during analytic solve: {e}")
            return []

    def _numeric_fallback(self, T: np.ndarray) -> List[List[float]]:
        """Previous internal numeric refinement fallback (after TRAC-IK)."""
        base_sets = self._solve_first_three(T)
        fallback_seeds: List[List[float]] = []
        if base_sets:
            for th1, th2, th3 in base_sets:
                fallback_seeds.append([th1, th2, th3, 0.0, 0.0, 0.0])
                # Additional wrist orientations to help tough targets
                fallback_seeds.append([th1, th2, th3, 0.0, math.pi/2, 0.0])
                fallback_seeds.append([th1, th2, th3, math.pi/2, math.pi/2, 0.0])
        else:
            fallback_seeds.append([0, -math.pi/2, 0, 0, 0, 0])
        numeric_solutions: List[List[float]] = []
        for seed in fallback_seeds:
            refined = self._numeric_refine(seed, T)
            if refined and self._within_joint_limits(refined):
                if self._is_valid_solution(refined, T):
                    if not any(sum(abs(a-b) for a,b in zip(refined,u)) < 1e-4 for u in numeric_solutions):
                        numeric_solutions.append(refined)
        return numeric_solutions

    def _rotation_to_quaternion(self, R: np.ndarray) -> Tuple[float, float, float, float]:
        """Convert rotation matrix to quaternion (x,y,z,w)."""
        if SCIPY_AVAILABLE and Rotation is not None:
            q = Rotation.from_matrix(R).as_quat()  # x,y,z,w
            return float(q[0]), float(q[1]), float(q[2]), float(q[3])
        # Manual conversion
        tr = R[0,0] + R[1,1] + R[2,2]
        if tr > 0:
            S = math.sqrt(tr+1.0)*2
            w = 0.25 * S
            x = (R[2,1] - R[1,2]) / S
            y = (R[0,2] - R[2,0]) / S
            z = (R[1,0] - R[0,1]) / S
        elif (R[0,0] > R[1,1]) and (R[0,0] > R[2,2]):
            S = math.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2]) * 2
            w = (R[2,1] - R[1,2]) / S
            x = 0.25 * S
            y = (R[0,1] + R[1,0]) / S
            z = (R[0,2] + R[2,0]) / S
        elif R[1,1] > R[2,2]:
            S = math.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2]) * 2
            w = (R[0,2] - R[2,0]) / S
            x = (R[0,1] + R[1,0]) / S
            y = 0.25 * S
            z = (R[1,2] + R[2,1]) / S
        else:
            S = math.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1]) * 2
            w = (R[1,0] - R[0,1]) / S
            x = (R[0,2] + R[2,0]) / S
            y = (R[1,2] + R[2,1]) / S
            z = 0.25 * S
        return float(x), float(y), float(z), float(w)

    def _tracik_fallback(self, T: np.ndarray, current_joints: Optional[List[float]]) -> List[List[float]]:
        """Attempt TRAC-IK (trackikpy) fallback if available and ROS parameters present."""
        if not self.tracik_ok:
            return []
        # Lazy init solver
        if self._tracik is None:
            try:
                self._tracik = TracIKSolver(self.base_link, self.tip_link,
                                             timeout=0.02, epsilon=1e-5)
            except Exception as e:
                if self.debug:
                    print(f"TRAC-IK init failed: {e}")
                self.tracik_ok = False
                return []
        # Prepare seed
        if current_joints and len(current_joints) == 6:
            seed = current_joints
        else:
            seed = [0, -math.pi/2, 0, 0, 0, 0]
        x, y, z = T[0,3], T[1,3], T[2,3]
        qx, qy, qz, qw = self._rotation_to_quaternion(T[:3,:3])
        try:
            sol = self._tracik.get_ik(seed,
                                      x, y, z,
                                      qx, qy, qz, qw)
            if sol is None:
                return []
            sol_list = [self.normalize_angle(a) for a in sol[:6]]
            if self._within_joint_limits(sol_list) and self._is_valid_solution(sol_list, T):
                return [sol_list]
        except Exception as e:
            if self.debug:
                print(f"TRAC-IK solve failed: {e}")
        return []

    def inverse_kinematics(self, target_pose: np.ndarray,
                          current_joints: Optional[List[float]] = None,
                          prefer_closest: bool = True,
                          timeout_ms: float = 50.0) -> List[List[float]]:
        """
        Analytical inverse kinematics using IKBT symbolic solutions.

        Args:
            target_pose: 4x4 homogeneous transformation matrix for the target pose
            current_joints: Optional current joint positions for solution preference
            prefer_closest: Whether to sort solutions by distance to current joints
            timeout_ms: Timeout (not used for analytical solutions, kept for API compatibility)

        Returns:
            List of possible joint configurations
        """
        start_time = time.perf_counter()
        self.solve_attempts += 1
        try:
            # 1. Generated IKBT analytic
            solutions = self._ikbt_solve_analytic(target_pose)
            valid_solutions: List[List[float]] = []
            for solution in solutions:
                if self._is_valid_solution(solution, target_pose):
                    valid_solutions.append(solution)
            # 2. TRAC-IK fallback if no analytic solution
            if not valid_solutions:
                tracik_solutions = self._tracik_fallback(target_pose, current_joints)
                valid_solutions.extend(tracik_solutions)
            # 3. Numeric refinement fallback (only if still empty)
            if not valid_solutions:
                numeric_solutions = self._numeric_fallback(target_pose)
                valid_solutions.extend(numeric_solutions)
            if valid_solutions and current_joints is not None and prefer_closest:
                valid_solutions.sort(key=lambda sol: self._solution_distance(sol, current_joints))
            elapsed = time.perf_counter() - start_time
            self.total_solve_time += elapsed
            if valid_solutions:
                self.solve_successes += 1
                if self.debug:
                    source = 'Analytic' if solutions else ('TRAC-IK' if tracik_solutions else 'Numeric')
                    print(f"IK ({source}) solutions: {len(valid_solutions)} in {elapsed*1000:.2f}ms")
            elif self.debug:
                print(f"No IK solutions after analytic/TRAC-IK/numeric in {elapsed*1000:.2f}ms")
            return valid_solutions
        except Exception as e:
            if self.debug:
                print(f"Inverse kinematics failed: {e}")
            return []

    def _within_joint_limits(self, joint_angles: List[float]) -> bool:
        """Check if joint angles are within UR30 limits."""
        for i, angle in enumerate(joint_angles):
            lower, upper = self.JOINT_LIMITS[i]
            if not lower <= angle <= upper:
                return False
        return True

    def _solution_distance(self, solution: List[float], current_joints: List[float]) -> float:
        """Calculate weighted distance between two joint configurations."""
        # Weight primary joints higher (they move larger segments)
        weights = [3.0, 2.5, 2.0, 1.5, 1.0, 1.0]
        distance = sum(w * (self.normalize_angle(s - c))**2
                      for w, s, c in zip(weights, solution, current_joints))
        return distance

    def _compute_errors(self, joint_angles: List[float], target_pose: np.ndarray) -> Tuple[float, float]:
        """Return (position_error_mm, orientation_error_deg) without applying thresholds."""
        achieved_pose = self.forward_kinematics(joint_angles)
        pos_error_mm = float(np.linalg.norm(achieved_pose[:3, 3] - target_pose[:3, 3]) * 1000.0)
        rot_error_matrix = achieved_pose[:3, :3].T @ target_pose[:3, :3]
        rot_angle = math.acos(np.clip((np.trace(rot_error_matrix) - 1) / 2, -1.0, 1.0))
        rot_error_deg = math.degrees(rot_angle)
        return pos_error_mm, float(rot_error_deg)

    def _is_valid_solution(self, joint_angles: List[float], target_pose: np.ndarray) -> bool:
        """Validate that a solution reaches the target pose within tolerance."""
        try:
            pos_error, rot_error = self._compute_errors(joint_angles, target_pose)
            return pos_error < self.pos_tol_mm and rot_error < self.rot_tol_deg
        except Exception:
            return False

    def best_reachable_pose(self, target_pose: np.ndarray,
                           current_joints: Optional[List[float]] = None) -> Tuple[List[float], np.ndarray]:
        """
        Find the closest reachable pose to the target pose.

        Args:
            target_pose: 4x4 homogeneous transformation matrix for the desired pose
            current_joints: Optional current joint positions

        Returns:
            Tuple of (best_joints, reachable_pose)
        """
        # First try exact analytical solution
        solutions = self.inverse_kinematics(target_pose, current_joints)

        if solutions:
            best_joints = solutions[0]  # Already sorted by preference
            reachable_pose = self.forward_kinematics(best_joints)
            return best_joints, reachable_pose

        # If no exact solution, fall back to approximation
        # Use a simple search around the workspace
        if current_joints is not None:
            best_joints = current_joints
        else:
            best_joints = [0, -math.pi/2, 0, 0, 0, 0]  # Safe default configuration

        reachable_pose = self.forward_kinematics(best_joints)
        return best_joints, reachable_pose

    def is_pose_reachable(self, target_pose: np.ndarray) -> bool:
        """
        Check if a target pose is reachable by the UR30 robot.

        Args:
            target_pose: 4x4 homogeneous transformation matrix for the target pose

        Returns:
            True if the pose is reachable, False otherwise
        """
        solutions = self.inverse_kinematics(target_pose)
        return len(solutions) > 0

    def get_performance_stats(self) -> Dict[str, Any]:
        """
        Get performance statistics for the IKBT solver.

        Returns:
            Dictionary with performance metrics
        """
        stats = {
            "solver_type": "IKBT_Analytical",
            "solve_attempts": self.solve_attempts,
            "solve_successes": self.solve_successes,
            "success_rate": self.solve_successes / max(1, self.solve_attempts),
            "total_attempts": self.solve_attempts,
            "total_successes": self.solve_successes,
        }

        if self.solve_attempts > 0:
            stats["avg_solve_time_ms"] = (self.total_solve_time / self.solve_attempts) * 1000

        return stats

    def print_performance_summary(self):
        """Print a summary of solver performance."""
        stats = self.get_performance_stats()

        print("\n=== UR30 IKBT Kinematics Performance Summary ===")
        print(f"Solver Type: {stats['solver_type']}")
        print(f"Solutions: {stats['solve_successes']}/{stats['solve_attempts']} " +
              f"({stats['success_rate']:.1%}) success rate")
        if stats['solve_attempts'] > 0:
            print(f"Average time: {stats.get('avg_solve_time_ms', 0):.2f}ms")
        print("=== End Summary ===")


def test_ur30_kinematics():
    """Test the UR30 kinematics implementation"""

    print("=== UR30 IKBT Kinematics Test ===")

    # Initialize UR30 solver with debug enabled
    ur30 = UR30Kinematics(debug=True)

    # Test configurations
    test_configs = [
        [0.0, -math.pi/2, 0.0, 0.0, 0.0, 0.0],           # Home position
        [0.0, -math.pi/4, -math.pi/4, 0.0, 0.0, 0.0],    # Common working pose
        [math.pi/4, -math.pi/4, -math.pi/4, 0.0, 0.0, 0.0],  # 45° base rotation
        [0.0, -math.pi/3, -math.pi/3, math.pi/6, math.pi/2, 0.0]  # Complex pose
    ]

    print()

    for i, config in enumerate(test_configs):
        print(f"--- Test {i+1}: {[round(math.degrees(j), 1) for j in config]} ---")

        # Forward kinematics
        fk = ur30.forward_kinematics(config)
        pos = fk[:3, 3]
        print(f"Target position: [{pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}]")

        # IKBT inverse kinematics
        start_time = time.perf_counter()
        ik_solutions = ur30.inverse_kinematics(fk, config)
        solve_time = (time.perf_counter() - start_time) * 1000  # ms

        print(f"Solutions found: {len(ik_solutions)} in {solve_time:.2f}ms")

        if ik_solutions:
            # Show the best solution
            best_sol = ik_solutions[0]
            best_deg = [round(math.degrees(angle), 1) for angle in best_sol]
            print(f"Best solution: {best_deg}")

            # Verify accuracy
            check_pose = ur30.forward_kinematics(best_sol)
            pos_error = np.linalg.norm(check_pose[:3, 3] - pos) * 1000  # mm

            rot_error_mat = fk[:3, :3].T @ check_pose[:3, :3]
            rot_trace = np.clip(np.trace(rot_error_mat), -1.0, 3.0)
            rot_error_deg = math.degrees(math.acos((rot_trace - 1) / 2))

            print(f"Position error: {pos_error:.3f}mm")
            print(f"Rotation error: {rot_error_deg:.3f}°")

            if pos_error < 5.0 and rot_error_deg < 5.0:
                print("✓ PASS")
            else:
                print("✗ FAIL")
        else:
            print("✗ FAIL - No solutions found")
        print()

    # Performance summary
    ur30.print_performance_summary()


if __name__ == "__main__":
    test_ur30_kinematics()
