import numpy as np
from scipy.spatial.transform import Rotation

from arm.fk import FK
from arm.fk import load_robot

def ortho_err(R):
    return np.linalg.norm(R @ np.transpose(R) - np.identity(3, dtype=float)) # Норма фромбениуса

def det_err(R):
    return abs(np.linalg.det(R) - 1)

def check_FK(FK, robot, tol_err=1e-9):
    joints_vectors = [
        np.radians([0, 0, 0, 0, 0, 0]), # zero configuration
        np.radians([-90, 0, 0, 0, 0, 0]), # one joint at a limit configuration
        np.radians([20, -30, 15, 40, -25, 60])
    ]


    for q in joints_vectors:
        T = FK(q, robot)
        R = T[:3, :3]
        p = T[:3, 3]
        r =  Rotation.from_matrix(R)
        euler_angles = r.as_euler("ZYX",degrees=True)

        print(f"the joint vector is\n{q}\nand resulting matrix is:\n{T}")

        print("orthogonality error:\n", ortho_err(R))

        print("determinant error:\n", det_err(R))

        if abs(ortho_err(R)) < tol_err and abs(det_err(R)) < tol_err:
            print(f"Проверки на ортогоальность и det=+1 пройдены успешно с толерантностью к погрешности {tol_err}")
        else:
            print("Проверки не прошли")

        print("generic pose orientation:")
        print("XYZ:", p)
        print("yaw, pitch, roll angles:", euler_angles)
        
        print("-" * 50)

if __name__ == "__main__":
    robot = load_robot("models/ar601m_arm.yaml")
    check_FK(FK, robot)