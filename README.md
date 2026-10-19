# AR-601M arm: kinematics, differential kinematics and singularities

Homework 1-3, Introduction to Robotics (MIPT, Center for Cognitive Modeling).

## Reproduce everything

From a clean checkout, at the repository root:

```bash
docker build -t ar601m . && docker run --rm -v "$PWD/results:/app/results" ar601m
```

PowerShell: replace `"$PWD/results"` with `"${PWD}/results"`.

The container runs the automated tests (`pytest`) and then `run_all.py`, which prints every
number used in the report and saves it to `results/results.txt`.

## Layout

| Path | Content |
|---|---|
| `models/ar601m_arm.yaml` | Robot model: joint order, axes, limits, fixed rotation, arm side, link offsets, kinematic chain |
| `arm/fk.py` | `load_robot`, `FK(q, robot)` (Task 2) |
| `arm/fk_checks.py` | Rotation checks for FK (Task 2) |
| `arm/analytical_ik.py` | `piper_ik(T, robot)` analytical IK (Task 3) |
| `arm/numerical_ik.py` | `num_ik(T, robot, q0)` Levenberg-Marquardt IK (Task 4) |
| `arm/jacobian.py` | `DifferentialKinematics(q, robot)`, two derivations + finite differences (Task 5) |
| `tests/` | Automated tests |
| `run_all.py` | Generates all reported results |
| `report/report.md` | Report, Tasks 1-6 |
