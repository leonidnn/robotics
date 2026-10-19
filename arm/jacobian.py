'''
Дифференциальная кинематика: [v, w] = J(q) dq
v - линейная скорость кончика инструмента, w - угловая скорость инструмента.
Обе выражены в базовой СО робота.
J считаем двумя независимыми способами и сравниваем с конечными разностями по FK.
'''

import numpy as np
from scipy.spatial.transform import Rotation as R

from arm.fk import FK, rot

AXES = {"x": np.array([1.0, 0, 0]), "y": np.array([0, 1.0, 0]), "z": np.array([0, 0, 1.0])}

def skew(w):
    '''
    Кососимметричная матрица: skew(w) @ v == np.cross(w, v)
    '''
    return np.array([[0, -w[2], w[1]],
                     [w[2], 0, -w[0]],
                     [-w[1], w[0], 0]])

def vee(S):
    '''
    Обратно к skew: из кососимметричной матрицы достает вектор
    '''
    return np.array([S[2, 1], S[0, 2], S[1, 0]])

def joint_frames(q, robot):
    '''
    Для каждого сустава возвращает (o_i, z_i): точку, где он стоит, и его ось - обе в базовой СО.
    '''
    frames = []
    for name, joint in robot["joints"].items():
        T_i = FK(q, robot, until=name) # СО прямо перед суставом
        o_i = T_i[:3, 3]
        z_i = T_i[:3, :3] @ AXES[joint["axis"]] # локальная ось сустава -> в базовую СО
        frames.append((o_i, z_i))

    return frames

def jacobian_geometric(q, robot):
    '''
    Способ 1. Геометрический якобиан.
    Вращение сустава i с единичной скоростью вокруг оси z_i, проходящей через o_i, дает кончику
        v = z_i x (p - o_i),   w = z_i
    '''
    p = FK(q, robot)[:3, 3]
    J = np.zeros((6, 6), dtype=float)

    for i, (o_i, z_i) in enumerate(joint_frames(q, robot)):
        J[:3, i] = np.cross(z_i, p - o_i)
        J[3:, i] = z_i

    return J

def jacobian_derivative(q, robot):
    '''
    Способ 2. Прямое дифференцирование цепочки T = A_i * Rot(q_i) * B_i по q_i.
    d Rot(q_i) / d q_i = Rot(q_i) * skew(a_i), где a_i - локальная ось сустава.
    Тогда dT/dq_i = A_i * Rot(q_i) * skew(a_i) * B_i, и
        v = dp/dq_i,   w = vee(dR/dq_i * R^T)
    '''
    T = FK(q, robot)
    J = np.zeros((6, 6), dtype=float)

    for i, (name, joint) in enumerate(robot["joints"].items()):
        A = FK(q, robot, until=name) # все до сустава
        Rot = rot(angle=q[i], axis=joint["axis"])
        B = np.linalg.inv(A @ Rot) @ T # все после сустава

        S = np.zeros((4, 4))
        S[:3, :3] = skew(AXES[joint["axis"]])
        dT = A @ Rot @ S @ B

        J[:3, i] = dT[:3, 3]
        J[3:, i] = vee(dT[:3, :3] @ T[:3, :3].T)

    return J

def jacobian_fd(q, robot, h=1e-6):
    '''
    Конечные разности (центральные) по FK, шаг h рад:
        v_i = (p(q + h e_i) - p(q - h e_i)) / 2h
        w_i = rotvec(R(q + h e_i) * R(q - h e_i)^T) / 2h
    '''
    J = np.zeros((6, 6), dtype=float)

    for i in range(6):
        dq = np.zeros(6)
        dq[i] = h
        T_plus = FK(q + dq, robot)
        T_minus = FK(q - dq, robot)

        J[:3, i] = (T_plus[:3, 3] - T_minus[:3, 3]) / (2 * h)
        J[3:, i] = R.from_matrix(T_plus[:3, :3] @ T_minus[:3, :3].T).as_rotvec() / (2 * h)

    return J

def DifferentialKinematics(q, robot):
    '''
    Интерфейс из задания: J(q) 6x6, [v, w] = J dq в базовой СО
    '''
    return jacobian_geometric(q, robot)

def compare(q, robot, h=1e-6):
    '''
    Попарные невязки (норма Фробениуса) трех якобианов
    '''
    J_geo = jacobian_geometric(q, robot)
    J_der = jacobian_derivative(q, robot)
    J_fd = jacobian_fd(q, robot, h)

    print(f"q (град) = {np.degrees(q)}")
    print(f"||J_geo - J_der|| = {np.linalg.norm(J_geo - J_der):.2e}")
    print(f"||J_geo - J_fd||  = {np.linalg.norm(J_geo - J_fd):.2e}")
    print(f"||J_der - J_fd||  = {np.linalg.norm(J_der - J_fd):.2e}")
    print(f"sigma_min(J) = {np.linalg.svd(J_geo, compute_uv=False)[-1]:.2e}") # мера близости к сингулярности
    print("-" * 50)


if __name__ == "__main__":
    from arm.fk import load_robot

    np.set_printoptions(precision=3, suppress=True)
    robot = load_robot("models/ar601m_arm.yaml")

    print("Generic configuration:")
    compare(np.radians([20, -30, 15, 40, -25, 60]), robot)

    print("Near singularity (q5 = 89.9 deg, оси q4 и q6 почти совпадают):")
    compare(np.radians([20, -30, 15, 40, 89.9, 60]), robot)
