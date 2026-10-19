'''
Воспроизводит все численные результаты отчета (задачи 1-5).
Печатает в консоль и сохраняет в results/results.txt
'''

import io
import os
import sys
import contextlib
import numpy as np
from scipy.spatial.transform import Rotation as R

from arm.fk import FK, load_robot
from arm.fk_checks import ortho_err, det_err
from arm.analytical_ik import piper_ik, check_limits
from arm.numerical_ik import num_ik, nearest_branch
from arm.jacobian import jacobian_geometric, jacobian_derivative, jacobian_fd

MODEL = "models/ar601m_arm.yaml"
TOL = 1e-9
CONFIGS = {
    "zero": [0, 0, 0, 0, 0, 0],
    "limit (q1 = -90)": [-90, 0, 0, 0, 0, 0],
    "generic": [20, -30, 15, 40, -25, 60],
}
SIDES = {1: "right (b = +1)", -1: "left (b = -1)"}

np.set_printoptions(precision=4, suppress=True)


def robot_for(b):
    robot = load_robot(MODEL)
    robot["arm_side"] = b
    return robot


def task1():
    print("# Task 1. Model check")
    robot = load_robot(MODEL)
    joints = robot["joints"]
    print("joints:", list(joints))
    print("axes:  ", [j["axis"] for j in joints.values()])
    print("limits:", [j["limits_deg"] for j in joints.values()])
    print("chain: ", robot["kinematic_chain"])
    ok = (list(joints) == ["q1", "q2", "q3", "q4", "q5", "q6"]
          and [j["axis"] for j in joints.values()] == ["z", "y", "x", "z", "y", "x"])
    print("check (6 revolute joints, order, axes):", "PASS" if ok else "FAIL")


def task2():
    print("\n# Task 2. Forward kinematics, tol =", TOL)
    for b, side in SIDES.items():
        robot = robot_for(b)
        print(f"\n## {side}")
        for name, q_deg in CONFIGS.items():
            T = FK(np.radians(q_deg), robot)
            Rm = T[:3, :3]
            print(f"\n{name}: q (deg) = {q_deg}")
            print(T)
            print(f"||R^T R - I|| = {ortho_err(Rm):.2e}, |det R - 1| = {det_err(Rm):.2e}",
                  "PASS" if ortho_err(Rm) < TOL and det_err(Rm) < TOL else "FAIL")
            if name == "generic":
                yaw, pitch, roll = R.from_matrix(Rm).as_euler("ZYX", degrees=True)
                print(f"ZYX: roll = {roll:.4f}, pitch = {pitch:.4f}, yaw = {yaw:.4f} deg")


def ik_targets(robot):
    targets = {name: FK(np.radians(q), robot) for name, q in CONFIGS.items()}
    T = np.eye(4)
    T[:3, 3] = [2.0, 0, 0]
    targets["unreachable (p = [2, 0, 0])"] = T
    targets["degenerate (q5 = 90)"] = FK(np.radians([20, -30, 15, 40, 90, 60]), robot)
    return targets


def residuals(q, T, robot):
    Tq = FK(q, robot)
    pos = np.linalg.norm(Tq[:3, 3] - T[:3, 3])
    rot = np.linalg.norm(R.from_matrix(T[:3, :3] @ Tq[:3, :3].T).as_rotvec())
    return pos, rot


def task3():
    print("\n# Task 3. Analytical IK, duplicate rule: all |dq_i| < 1e-9 rad (mod 2pi)")
    for b, side in SIDES.items():
        robot = robot_for(b)
        print(f"\n## {side}")
        for name, T in ik_targets(robot).items():
            all_br = piper_ik(T, robot, filter_limits=False)
            feasible = check_limits(all_br, robot, TOL)
            print(f"\ntarget {name}: {len(all_br)} branches, {len(feasible)} feasible")
            for k, q in enumerate(all_br):
                pos, rot = residuals(q, T, robot)
                mark = "feasible" if any(np.array_equal(q, f) for f in feasible) else "out of limits"
                print(f"  [{k}] q = {np.degrees(q)}  pos_err = {pos:.1e} m, rot_err = {rot:.1e} rad, {mark}")


def task4():
    print("\n# Task 4. Numerical IK (Levenberg-Marquardt, lambda = 0.01, tol = 1e-9, max_iter = 200)")
    starts = [np.radians([0, 0, 0, 0, 0, 0]), np.radians([0, 0, 0, -90, 0, 0])]
    for b, side in SIDES.items():
        robot = robot_for(b)
        print(f"\n## {side}")
        for name, T in ik_targets(robot).items():
            branches = piper_ik(T, robot, filter_limits=False)
            print(f"\ntarget {name}")
            for q0 in starts:
                res = num_ik(T, robot, q0)
                k, dist = nearest_branch(res["q"], branches)
                print(f"  q0 = {np.degrees(q0)} -> {res['status']}, {res['iterations']} it, "
                      f"q = {np.degrees(res['q'])}, pos_err = {res['pos_err']:.1e} m, "
                      f"rot_err = {res['rot_err']:.1e} rad, nearest branch = {k} (dist {dist:.1e} rad)")


def task5():
    print("\n# Task 5. Jacobian (base frame, v at tool tip), FD: central, h = 1e-6 rad")
    configs = {"generic": [20, -30, 15, 40, -25, 60], "near singular (q5 = 89.9)": [20, -30, 15, 40, 89.9, 60]}
    for b, side in SIDES.items():
        robot = robot_for(b)
        print(f"\n## {side}")
        for name, q_deg in configs.items():
            q = np.radians(q_deg)
            J1, J2, J3 = jacobian_geometric(q, robot), jacobian_derivative(q, robot), jacobian_fd(q, robot)
            smin = np.linalg.svd(J1, compute_uv=False)[-1]
            print(f"{name}: ||Jgeo-Jder|| = {np.linalg.norm(J1 - J2):.1e}, "
                  f"||Jgeo-Jfd|| = {np.linalg.norm(J1 - J3):.1e}, "
                  f"||Jder-Jfd|| = {np.linalg.norm(J2 - J3):.1e}, sigma_min = {smin:.2e}")


if __name__ == "__main__":
    buf = io.StringIO()

    class Tee:
        def write(self, s):
            sys.__stdout__.write(s)
            buf.write(s)

        def flush(self):
            sys.__stdout__.flush()

    with contextlib.redirect_stdout(Tee()):
        task1()
        task2()
        task3()
        task4()
        task5()

    os.makedirs("results", exist_ok=True)
    with open("results/results.txt", "w", encoding="utf-8") as f:
        f.write(buf.getvalue())
