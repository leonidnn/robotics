import numpy as np
import pytest

from arm.fk import FK, load_robot
from arm.analytical_ik import piper_ik
from arm.numerical_ik import num_ik
from arm.jacobian import jacobian_geometric, jacobian_derivative, jacobian_fd

MODEL = "models/ar601m_arm.yaml"
Q_GENERIC = np.radians([20, -30, 15, 40, -25, 60])


@pytest.fixture(params=[1, -1], ids=["right", "left"])
def robot(request):
    r = load_robot(MODEL)
    r["arm_side"] = request.param
    return r


def test_model_loads_six_joints():
    r = load_robot(MODEL)
    assert list(r["joints"]) == ["q1", "q2", "q3", "q4", "q5", "q6"]
    assert [j["axis"] for j in r["joints"].values()] == ["z", "y", "x", "z", "y", "x"]
    assert [j["limits_deg"] for j in r["joints"].values()] == \
        [[-90, 90], [-105, 105], [-45, 45], [-90, 90], [-90, 90], [-90, 90]]


def test_fk_zero_pose(robot):
    b = robot["arm_side"]
    T = FK(np.zeros(6), robot)
    assert np.allclose(T[:3, 3], [0.1494336, -0.1564276 * b, 0.1], atol=1e-6)


def test_fk_rotation_is_valid(robot):
    Rm = FK(Q_GENERIC, robot)[:3, :3]
    assert np.linalg.norm(Rm.T @ Rm - np.eye(3)) < 1e-9
    assert abs(np.linalg.det(Rm) - 1) < 1e-9


def test_analytical_ik_roundtrip(robot):
    T = FK(Q_GENERIC, robot)
    all_branches = piper_ik(T, robot, filter_limits=False)
    assert len(all_branches) == 8
    for q in all_branches:
        assert np.linalg.norm(FK(q, robot) - T) < 1e-9
    feasible = piper_ik(T, robot)
    assert any(np.allclose(q, Q_GENERIC, atol=1e-9) for q in feasible)


def test_analytical_ik_unreachable(robot):
    T = np.eye(4)
    T[:3, 3] = [2.0, 0, 0]
    assert piper_ik(T, robot) == []


def test_numerical_ik_converges(robot):
    T = FK(Q_GENERIC, robot)
    res = num_ik(T, robot, np.zeros(6))
    assert res["status"] == "converged"
    assert res["pos_err"] < 1e-9 and res["rot_err"] < 1e-9


def test_jacobians_agree(robot):
    J1 = jacobian_geometric(Q_GENERIC, robot)
    J2 = jacobian_derivative(Q_GENERIC, robot)
    J3 = jacobian_fd(Q_GENERIC, robot)
    assert np.linalg.norm(J1 - J2) < 1e-12
    assert np.linalg.norm(J1 - J3) < 1e-8
