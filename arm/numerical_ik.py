'''
Численное решение задачи IK методом Левенберга-Марквардта (damped least squares).
На каждой итерации смотрим, насколько FK(q) промахивается мимо цели, и немного двигаем q.
'''

import numpy as np
from scipy.spatial.transform import Rotation as R

from arm.fk import FK

def pose_error(T, T_target):
    '''
    Ошибка между текущей позой T и целью T_target.
    Возвращает e_p - ошибка позиции, в м. e_R - ошибка ориентации, вектор поворота в рад
    '''
    e_p = T_target[:3, 3] - T[:3, 3]
    e_R = R.from_matrix(T_target[:3, :3] @ T[:3, :3].T).as_rotvec() # поворот, который еще надо сделать

    return e_p, e_R

def numerical_jacobian(q, robot, h=1e-7):
    '''
    Якобиан 6x6 конечными разностями: чуть поворачиваем каждый сустав и смотрим, как сдвинулась поза.
    Строки: [v, w] в базовой СО, столбцы: суставы
    '''
    J = np.zeros((6, 6), dtype=float)
    T = FK(q, robot)

    for i in range(6):
        dq = np.zeros(6)
        dq[i] = h
        e_p, e_R = pose_error(T, FK(q + dq, robot)) # насколько сдвинулась поза
        J[:3, i] = e_p / h
        J[3:, i] = e_R / h

    return J

def num_ik(T, robot, q0, tol=1e-9, max_iter=200, lam=0.01):
    '''
    Решение задачи IK численно.
    T - целевая поза, np.array 4x4
    robot - dict робота из yaml-файла
    q0 - начальная конфигурация, рад
    Возвращает dict: статус, число итераций, итоговое q и невязки
    '''
    q = np.array(q0, dtype=float)
    status = "max_iterations"

    for it in range(1, max_iter + 1):
        e_p, e_R = pose_error(FK(q, robot), T)

        if np.linalg.norm(e_p) < tol and np.linalg.norm(e_R) < tol:
            status = "converged"
            break

        J = numerical_jacobian(q, robot)
        e = np.concatenate([e_p, e_R])
        dq = J.T @ np.linalg.solve(J @ J.T + lam**2 * np.identity(6), e) # шаг Левенберга-Марквардта

        if np.linalg.norm(dq) < tol * 1e-3:
            status = "stalled" # шаг крошечный, а ошибка большая - застряли
            break

        q = (q + dq + np.pi) % (2 * np.pi) - np.pi # углы в [-pi, pi)

    e_p, e_R = pose_error(FK(q, robot), T)

    return {
        "q0": np.array(q0, dtype=float),
        "status": status,
        "iterations": it,
        "q": q,
        "pos_err": np.linalg.norm(e_p),
        "rot_err": np.linalg.norm(e_R),
    }

def nearest_branch(q, branches):
    '''
    Ищет ближайшую аналитическую ветвь к решению q.
    Возвращает номер ветви и расстояние (рад, с учетом того что -pi == pi)
    '''
    if len(branches) == 0:
        return None, np.inf

    dist = [np.linalg.norm((q - b + np.pi) % (2 * np.pi) - np.pi) for b in branches]
    k = int(np.argmin(dist))

    return k, dist[k]


if __name__ == "__main__":
    from arm.fk import load_robot
    from arm.analytical_ik import piper_ik

    np.set_printoptions(precision=3, suppress=True)
    np.random.seed(42)

    robot = load_robot("models/ar601m_arm.yaml")
    T = FK(np.radians([20, -30, 15, 40, -25, 60]), robot)
    branches = piper_ik(T, robot)

    starts = [
        np.radians([0, 0, 0, 0, 0, 0]),
        np.radians([0, 0, 0, -90, 0, 0]),
        np.random.uniform(-np.pi, np.pi, 6),
    ]

    for q0 in starts:
        res = num_ik(T, robot, q0)
        k, dist = nearest_branch(res["q"], branches)
        print(f"q0 = {np.degrees(res['q0'])}")
        print(f"status: {res['status']}, iterations: {res['iterations']}")
        print(f"q = {np.degrees(res['q'])}")
        print(f"pos_err = {res['pos_err']:.2e} m, rot_err = {res['rot_err']:.2e} rad")
        print(f"nearest branch: {k}, distance = {dist:.2e} rad")
        print("-" * 50)
