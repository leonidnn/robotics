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

def sign_to_num(sign: str, robot: dict):
    if sign == "arm_side":
        return robot["arm_side"]
    elif sign == "-arm_side":
        return -robot["arm_side"]
    else:
        sys.exit("Неверный формат sign")

def FK(q: np.array, robot: dict, until=None) -> np.array:
    '''
    Решает задачу FK
    q - углы суставов робота в радианах
    robot - dict робота из yaml-файла
    until - item, до которого FK идёт не включительно
    '''

    if until is not None and until not in robot["kinematic_chain"]:
        sys.exit("В until указан несуществующий объект. Сверьтесь с yaml моделью робота")

    T = np.identity(4, dtype=float)

    joint_num = 0
    for item in robot["kinematic_chain"]: # Сначала проверяет, item является линк, джоинт или фиксд ротейшн, потом считает sign, потом домножает матрицу T
        if until == item:
            break
        
        if item in robot["links"]:
            sign = robot["links"][item]["sign"] # Наверное, надо было так все вещи задать, чтоб не писать милион скобок []
            if type(sign) == str:
                sign = sign_to_num(sign, robot)
            T @= trans(d=robot["links"][item]["length"] * sign,
                       axis=robot["links"][item]["along_axis"])

        elif item in robot["joints"]:
            T @= rot(angle=q[joint_num],
                     axis=robot["joints"][item]["axis"])
            joint_num += 1

        elif item in robot["fixed_rotations"]:
            sign = robot["fixed_rotations"][item]["sign"]
            if type(sign) == str:
                sign = sign_to_num(sign, robot)
            T @= rot(angle=np.radians(robot["fixed_rotations"][item]["angle_deg"]) * sign,
                     axis=robot["fixed_rotations"][item]["axis"])

        else:
            sys.exit("Формат yaml не согласован с форматом q")

    return T

if __name__ == "__main__":
    np.set_printoptions(precision=4, suppress=True)
    robot = load_robot("models/ar601m_arm.yaml")
    print(FK(np.zeros(6), robot))