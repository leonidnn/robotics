'''
Аналитическое решение задачи IK. У нас 3 сустава находятся в одной точке, и координату этой точки можно легко найти.
Поэтому будем решать методом Пипера
'''

import numpy as np
from scipy.spatial.transform import Rotation as R

from arm.fk import FK, trans, sign_to_num

def move_back(T, robot, link):
    '''
    Сдвигает систему координат T назад вдоль звена link (в локальной СО T).
    Нужно, чтобы от кончика кисти вернуться в центр запястья.
    '''
    sign = robot["links"][link]["sign"]
    if type(sign) == str:
        sign = sign_to_num(sign, robot)
    d = robot["links"][link]["length"] * sign
    return T @ trans(d=-d, axis=robot["links"][link]["along_axis"])  # минус: идём назад

def joint3_ik(end_vec, robot, start_vec=0, tol=1e-9):
    '''
    Находит q1, q2, q3, при которых центр запястья попадает в точку end_vec.
    Возвращает список вариантов np.array([q1, q2, q3]) (до 4 штук), пустой - если недостижимо.
    '''
    b = robot["arm_side"]
    l1 = robot["links"]["l1"]["length"]
    l2 = robot["links"]["l2"]["length"]
    l3 = robot["links"]["l3"]["length"]
    r1 = robot["fixed_rotations"]["r1"]
    beta = np.radians(r1["angle_deg"]) * sign_to_num(r1["sign"], robot)  # фиксированный поворот 10°*b

    d = end_vec - start_vec - np.array([0, 0, l1])  # вектор от плеча до запястья

    # 1) q3 по расстоянию |d| (q1 и q2 его не меняют)
    s3 = (l2**2 + l3**2 - d @ d) / (2 * b * l2 * l3)
    if abs(s3) > 1 + tol:
        return []  # недостижимо
    s3 = np.clip(s3, -1, 1)

    q = []
    for q3 in [np.arcsin(s3), np.pi - np.arcsin(s3)]:
        # 2) q2 по высоте d_z (q1 её не меняет)
        c2 = -d[2] / (l3 * np.cos(q3))
        if abs(c2) > 1 + tol:
            continue
        c2 = np.clip(c2, -1, 1)

        for q2 in [np.arccos(c2), -np.arccos(c2)]:
            # 3) q1 по направлению в горизонтальной плоскости
            vx = -l3 * np.sin(q2) * np.cos(q3)  # куда смотрела бы рука при q1 = 0
            vy = -b * l2 + l3 * np.sin(q3)
            q1 = np.arctan2(d[1], d[0]) - np.arctan2(vy, vx) - beta

            q123 = np.array([q1, q2, q3])
            q.append((q123 + np.pi) % (2 * np.pi) - np.pi)  # углы в [-pi, pi)
    return q


def check_limits(solutions, robot, tol=1e-9):
    '''
    Оставляет только решения, у которых все углы внутри пределов суставов (границы включительно).
    '''
    limits = np.radians([j["limits_deg"] for j in robot["joints"].values()])  # shape (6, 2)
    return [q for q in solutions
            if np.all(q >= limits[:, 0] - tol) and np.all(q <= limits[:, 1] + tol)]

def check_duplicates(solutions, tol=1e-9):
    '''
    Удаляет повторы: два решения одинаковые, если все углы отличаются меньше чем на tol (рад).
    '''
    unique = []
    for q in solutions:
        diff = [np.abs((q - u + np.pi) % (2 * np.pi) - np.pi) for u in unique]  # разница с учётом -pi == pi
        if not any(np.all(dq < tol) for dq in diff):
            unique.append(q)
    return unique

def piper_ik(T, robot, tol=1e-9, filter_limits=True):
    '''
    Решение задачи обратной кинематики методом Пипера.
    T - generic pose, np.array, shape = 4x4
    robot - dir робота
    Функция аналитическая и расчитана исключительно на модель AR-601M arm
    Возвращает список всех решений np.array([q1..q6]) (до 8 штук), пустой - если недостижимо
    '''

    solutions = []

    T_3joint = move_back(T, robot, "l4")
    p_w = T_3joint[:3, 3] # Это центр запястья

    for q123 in joint3_ik(p_w, robot, tol=tol):
        R03 = FK(q123, robot, until="q4")[:3, :3] # поворот до запястья
        R36 = R03.T @ T[:3, :3] # поворот, который должно сделать запястье

        q456 = R.from_matrix(R36).as_euler("ZYX") # Rz(q4) Ry(q5) Rx(q6)
        q456_flip = q456 + np.array([np.pi, 0, np.pi]) # вторая ветвь запястья:
        q456_flip[1] = np.pi - q456[1] # (q4+pi, pi-q5, q6+pi)

        for wrist in (q456, q456_flip):
            q = np.concatenate([q123, wrist])
            solutions.append((q + np.pi) % (2 * np.pi) - np.pi)  # углы в [-pi, pi)

    solutions = check_duplicates(solutions, tol)

    if filter_limits: # False - вернуть все ветви до фильтрации по пределам
        solutions = check_limits(solutions, robot, tol)

    return solutions
