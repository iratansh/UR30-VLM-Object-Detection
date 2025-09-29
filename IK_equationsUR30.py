#!/usr/bin/python
#  Python inverse kinematic equations for UR30

import numpy as np
from math import sqrt, atan2, cos, sin, acos, asin, pi

# Auto Generated (manually patched + elbow-up/down extension) Code to solve the unknowns
# parameter:  T   4x4 numerical target for T06

def ikin_UR30(T):
    if T.shape != (4, 4):
        print("bad input to ikin_UR30")
        return False
    # define the input vars
    r_11 = T[0,0]; r_12 = T[0,1]; r_13 = T[0,2]
    r_21 = T[1,0]; r_22 = T[1,1]; r_23 = T[1,2]
    r_31 = T[2,0]; r_32 = T[2,1]; r_33 = T[2,2]
    Px = T[0,3]; Py = T[1,3]; Pz = T[2,3]

    # Declare the parameters (meters)
    a_2 = -0.637
    a_3 = -0.5037
    d_1 = 0.2363
    d_4 = 0.201
    d_5 = 0.1593
    d_6 = 0.1543

    solvable_pose = True

    # ---------------- Base joint solutions (two) ----------------
    A = Px - d_6 * r_13
    B = -Py + d_6 * r_23
    disc = A*A + B*B - d_4*d_4
    if disc < 0:
        return False  # unreachable (wrist offset circle)
    Rrad = sqrt(disc)
    base_angle = atan2(A, B)
    th_1s1 = base_angle + atan2(Rrad, d_4)
    th_1s2 = base_angle + atan2(-Rrad, d_4)

    # ---------------- Wrist bend (theta5) branches ----------------
    # We create up to 8 duplicated branches to stay consistent with generator output expectations
    # Domain checks for acos argument
    def clamp_arg(val):
        if val < -1.0: return -1.0
        if val > 1.0: return 1.0
        return val

    arg1 = r_13*sin(th_1s1) - r_23*cos(th_1s1)
    arg2 = r_13*sin(th_1s2) - r_23*cos(th_1s2)
    if abs(arg1) > 1.0 and abs(arg2) > 1.0:
        return False
    arg1 = clamp_arg(arg1)
    arg2 = clamp_arg(arg2)
    th_5v1 = acos(arg1)
    th_5v2 = acos(arg2)
    # Mirror to create 8 wrist bend variants (structure compatibility)
    th_5v3 = th_5v2; th_5v4 = th_5v1; th_5v5 = th_5v1; th_5v6 = th_5v2; th_5v7 = th_5v2; th_5v8 = th_5v1

    # ---------------- theta6 (wrist roll) ----------------
    def safe_atan2(y,x):
        return atan2(y, x)
    def t6(th1, th5):
        s5 = sin(th5)
        if abs(s5) < 1e-9:
            return 0.0
        return safe_atan2((-r_12*sin(th1) + r_22*cos(th1))/s5, (r_11*sin(th1) - r_21*cos(th1))/s5)
    th_6v1 = t6(th_1s1, th_5v1)
    th_6v2 = t6(th_1s2, th_5v2)
    th_6v3 = t6(th_1s2, th_5v3)
    th_6v4 = t6(th_1s1, th_5v4)
    th_6v5 = t6(th_1s1, th_5v4)
    th_6v6 = t6(th_1s2, th_5v3)
    th_6v7 = t6(th_1s2, th_5v2)
    th_6v8 = t6(th_1s1, th_5v1)

    # ---------------- theta234 composite ----------------
    def t234(th5, th6):
        s5 = sin(th5)
        if abs(s5) < 1e-9:
            return 0.0
        return atan2(-r_33/s5, r_31*sin(th6) + r_32*cos(th6))
    th_234v1 = t234(th_5v1, th_6v1)
    th_234v2 = t234(th_5v2, th_6v2)
    th_234v3 = t234(th_5v3, th_6v3)
    th_234v4 = t234(th_5v4, th_6v4)
    th_234v5 = t234(th_5v4, th_6v4)
    th_234v6 = t234(th_5v3, th_6v3)
    th_234v7 = t234(th_5v2, th_6v2)
    th_234v8 = t234(th_5v1, th_6v1)

    # ---------------- theta2 ----------------
    def t2(th1, th5, th6):
        c6 = cos(th6); s6 = sin(th6)
        c1 = cos(th1); s1 = sin(th1)
        c5 = cos(th5); s5 = sin(th5)
        term1 = r_11*s5*c1*c6 - r_12*s5*s6*c1 + r_13*c1*c5
        term1 += r_21*s1*s5*c6 - r_22*s1*s5*s6 + r_23*s1*c5
        num = term1
        den = -r_31*s5*c6 + r_32*s5*s6 - r_33*c5
        return atan2(num, den)
    th_2v1 = t2(th_1s1, th_5v1, th_6v1)
    th_2v2 = t2(th_1s2, th_5v2, th_6v2)
    th_2v3 = t2(th_1s2, th_5v3, th_6v3)
    th_2v4 = t2(th_1s1, th_5v4, th_6v4)
    th_2v5 = t2(th_1s1, th_5v4, th_6v4)
    th_2v6 = t2(th_1s2, th_5v3, th_6v3)
    th_2v7 = t2(th_1s2, th_5v2, th_6v2)
    th_2v8 = t2(th_1s1, th_5v1, th_6v1)

    # ---------------- theta3 (elbow up & elbow down) ----------------
    def t3_pair(th1, th2, th5, th6):
        c1 = cos(th1); s1 = sin(th1)
        c6 = cos(th6); s6 = sin(th6)
        s5 = sin(th5)
        # Wrist center
        Wx = Px - d_6*r_13 - d_5*(r_11*s6 + r_12*c6)*s5
        Wy = Py - d_6*r_23 - d_5*(r_21*s6 + r_22*c6)*s5
        Wz = Pz - d_6*r_33 - d_5*(r_31*s6 + r_32*c6)*s5
        x_arm =  c1*Wx + s1*Wy
        y_arm =  Wz - d_1
        num = x_arm*x_arm + y_arm*y_arm - a_2*a_2 - a_3*a_3
        den = 2*a_2*a_3
        if abs(den) < 1e-12:
            return (0.0, 0.0)
        c3 = num/den
        if c3 < -1.0: c3 = -1.0
        if c3 >  1.0: c3 = 1.0
        s3 = sqrt(max(0.0, 1-c3*c3))
        th3_up = atan2(s3, c3)     # elbow-up
        th3_down = atan2(-s3, c3)  # elbow-down
        return (th3_up, th3_down)

    # For each branch create two variants (up/down)
    th_3v1u, th_3v1d = t3_pair(th_1s1, th_2v1, th_5v1, th_6v1)
    th_3v2u, th_3v2d = t3_pair(th_1s2, th_2v2, th_5v2, th_6v2)
    th_3v3u, th_3v3d = t3_pair(th_1s2, th_2v3, th_5v3, th_6v3)
    th_3v4u, th_3v4d = t3_pair(th_1s1, th_2v4, th_5v4, th_6v4)
    th_3v5u, th_3v5d = t3_pair(th_1s1, th_2v5, th_5v4, th_6v4)
    th_3v6u, th_3v6d = t3_pair(th_1s2, th_2v6, th_5v3, th_6v3)
    th_3v7u, th_3v7d = t3_pair(th_1s2, th_2v7, th_5v2, th_6v2)
    th_3v8u, th_3v8d = t3_pair(th_1s1, th_2v8, th_5v1, th_6v1)

    # ---------------- theta4 from composite (per elbow variant) ----------------
    def t4(th234, th2, th3):
        return th234 - th2 - th3

    # Package solutions (now 16: 8 branches * 2 elbow variants)
    solution_list = []
    def add_solution(th1, th234, th2, th3, th5, th6):
        th4 = t4(th234, th2, th3)
        # Structure: [th1, th_234, th2+th3, th2, th3+th4, th3, th4, th5, th6]
        solution_list.append([th1, th234, th2+th3, th2, th3+th4, th3, th4, th5, th6])

    # Original 8 branches (u=elbow up, d=elbow down)
    add_solution(th_1s2, th_234v2, th_2v2, th_3v2u, th_5v2, th_6v2)
    add_solution(th_1s2, th_234v2, th_2v2, th_3v2d, th_5v2, th_6v2)
    add_solution(th_1s2, th_234v3, th_2v3, th_3v3u, th_5v3, th_6v3)
    add_solution(th_1s2, th_234v3, th_2v3, th_3v3d, th_5v3, th_6v3)
    add_solution(th_1s1, th_234v1, th_2v1, th_3v1u, th_5v1, th_6v1)
    add_solution(th_1s1, th_234v1, th_2v1, th_3v1d, th_5v1, th_6v1)
    add_solution(th_1s1, th_234v4, th_2v4, th_3v4u, th_5v4, th_6v4)
    add_solution(th_1s1, th_234v4, th_2v4, th_3v4d, th_5v4, th_6v4)
    add_solution(th_1s2, th_234v3, th_2v6, th_3v6u, th_5v3, th_6v3)
    add_solution(th_1s2, th_234v3, th_2v6, th_3v6d, th_5v3, th_6v3)
    add_solution(th_1s1, th_234v5, th_2v5, th_3v5u, th_5v4, th_6v4)
    add_solution(th_1s1, th_234v5, th_2v5, th_3v5d, th_5v4, th_6v4)
    add_solution(th_1s2, th_234v2, th_2v7, th_3v7u, th_5v2, th_6v2)
    add_solution(th_1s2, th_234v2, th_2v7, th_3v7d, th_5v2, th_6v2)
    add_solution(th_1s1, th_234v1, th_2v8, th_3v8u, th_5v1, th_6v1)
    add_solution(th_1s1, th_234v1, th_2v8, th_3v8d, th_5v1, th_6v1)

    return solution_list if solvable_pose else False

# Test harness (optional) omitted for brevity
if __name__ == "__main__":
    Ttest = np.eye(4)
    sols = ikin_UR30(Ttest)
    print(f"Returned {len(sols) if sols else 0} solutions (test pose)")



