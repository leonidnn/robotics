import numpy as np
from scipy.spatial.transform import Rotation as R
import yaml
import sys

def trans(d:float, axis: str="x") -> np.array:
    '''
    Делает сдвиг на величину d по оси axis, возвращает Homogeneous transformation matrix
    axis: x, y, z. По умолчанию вдоль x
    d: расстояние сдвига в метрах
    '''
    axis_dict = {
        "x": 0,
        "y": 1,
        "z": 2
    }

    T = np.identity(4, dtype=float)
    T[axis_dict[axis], 3] += d

    return T

def rot(angle: float, axis: str="z") -> np.array:
    '''
    Делает поворот на величину angle вокруг оси axis, возвращает Homogeneous transformation matrix
    axis: x, y, z. По умолчанию вокруг z
    angle: угол поворота в радианах
    '''

    T = np.zeros((4, 4), dtype=float)
    T[:3, :3] +=  R.from_euler(axis, angle, degrees=False).as_matrix()
    T[3, 3] = 1

    return T

def load_robot(path) -> dict:
    with open(path) as stream:
        try:
            return(yaml.safe_load(stream))
        except yaml.YAMLError as exc:
            print("Не нашел файл по адресу", path)
            raise yaml.YAMLError

def FK(q: np.array, robot: dict):
    '''
    Решает задачу FK
    q - углы суставов робота
    robot - dict робота из yaml-файла
    '''

    T = np.identity(4, dtype=float)

    joint_num = 0
    for item in robot["kinematic_chain"]:
        if item in robot["links"]:
            T @= trans(d=robot["links"][item]["length"],
                       axis=robot["links"][item]["along_axis"])

        elif item in robot["joints"]:
            T @= rot(angle=q[joint_num],
                     axis=robot["joints"][item]["axis"])
            joint_num += 1

        elif item in robot["fixed_rotations"]:
            T @= rot(angle=np.degrees(robot["fixed_rotations"][item]["angle_deg"]),
                     axis=robot["fixed_rotations"][item]["axis"])

        else:
            sys.exit("Ошибка формата файла yaml, проблема в коде, не во вводе")

    return T

robot = load_robot("models/ar601m_arm.yaml")

print(FK(np.array((10, 12, 5, 2, 5, 3)), robot))
