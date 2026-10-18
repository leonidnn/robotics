# w01_03_slides.pdf

## [Page 1]
Introduction to Robotics, Homework 1-3
The AR-601M arm
Forward and inverse kinematics, differential kinematics and singularities
MIPT
Center for Cognitive Modeling

## [Page 2]
**Robot and assignment scope**
AR-601M is an anthropomorphic robot with 41 degrees of freedom. This assignment concerns one arm only: a serial chain with six actuated joints from the shoulder to the tool.
The required results are:
* a kinematic model;
* forward and inverse kinematics;
* differential kinematics;
* a numerical analysis of singular configurations.

The joint vector is
$q=[q_{1},q_{2},q_{3},q_{4},q_{5},q_{6}]^{T}$

Oleg Bulichev
Introduction to Robotics (In2Ro)
1

## [Page 3]
*(Images: AR-601M robot, Arm geometry and axis annotations)*
AR-601M arm geometry
Oleg Bulichev
AR-601M robot
-5,7k2
11,7кг
13Kz
Шейный грел 2.3ке
$TopC-2.5oC3$
10
21.
Arm geometry and axis annotations
Introduction to Robotics (In2Ro)
2

## [Page 4]
**Fixed geometry**
$b=-1$: left arm

| Symbol | Offset | Value |
| :--- | :--- | :--- |
| $l_{1}$ | base to shoulder, along +z | 0.40 m |
| $l_{2}$ | shoulder offset, along -by | 0.18 m |
| $l_{3}$ | arm offset, along -z | 0.30 m |
| $l_{4}$ | wrist centre to tool, along +x | 0.12 m |

* $b=+1$: right arm
* Fixed rotation between $q_{1}$ and $q_{2}$: 10°b about z

All students use these values in every task. No student-specific link dimensions or alternative joint limits are used.

Oleg Bulichev
Introduction to Robotics (In2Ro)
3

## [Page 5]
**Joint axes and limits**

| Joint | Local axis | Lower limit | Upper limit |
| :--- | :--- | :--- | :--- |
| $q_{1}$ | z | $-90^{\circ}$ | $+90^{\circ}$ |
| $q_{2}$ | y | $-105^{\circ}$ | $+105^{\circ}$ |
| $q_{3}$ | x | $-45^{\circ}$ | $+45^{\circ}$ |
| $q_{4}$ | z | $-90^{\circ}$ | $+90^{\circ}$ |
| $q_{5}$ | y | $-90^{\circ}$ | $+90^{\circ}$ |
| $q_{6}$ | x | $-90^{\circ}$ | $+90^{\circ}$ |

* The limits are closed intervals.
* A configuration outside any interval is infeasible.
* The fixed $10^{\circ}$ b rotation is not an additional joint.

Oleg Bulichev
Introduction to Robotics (In2Ro)
4

## [Page 6]
**Required software interface**
The implementation must provide:
* FK(q, robot) -> T, where $T\in SE(3)$;
* IK(T, robot) -> list[q];
* DifferentialKinematics(q, robot) -> J, where $J\in\mathbb{R}^{6\times6}$ and $[v^{T},\omega^{T}]^{T}=J(q)\dot{q}$ .

robot must be loaded from a separate, human-readable model file containing the joint order, axes, limits, fixed rotation, arm side and link dimensions. Do not duplicate these values as hidden constants in the solver.

Oleg Bulichev
Introduction to Robotics (In2Ro)
5

## [Page 7]
**Tasks 1 and 2**

**Task 1. Kinematic model**
* Define the coordinate frames and the complete symbolic transformation chain.
* Create the robot model file.
* Verify the joint count, order, axes and limits when the file loads.

**Task 2. Forward kinematics**
* Implement FK.
* Evaluate the zero configuration, a limit configuration and a generic configuration.
* Report T, $||R^{T}R-I||$, | det R - 1| and the tolerance.
* Report the generic orientation as ZYX roll-pitch-yaw angles.

Oleg Bulichev
Introduction to Robotics (In2Ro)
6

## [Page 8]
**Tasks 3 and 4**

**Task 3. Analytical inverse kinematics**
* Derive and implement the solver.
* Return all distinct real branches before limit filtering and all feasible branches after filtering.
* Test both values of b.
* Check every returned branch with FK; report residuals and the duplicate-branch rule.
* Handle unreachable and degenerate targets explicitly.

**Task 4. Numerical inverse kinematics**
* Use the same targets as in Task 3 and at least two initial configurations.
* Report termination status, iterations, final configuration and pose residuals.
* Match each converged result to the nearest analytical branch.

Oleg Bulichev
Introduction to Robotics (In2Ro)
7

## [Page 9]
**Tasks 5 and 6**

**Task 5. Differential kinematics**
* Derive $J(q)$ by two independent methods.
* Compare both results with a finite-difference approximation of FK.
* State the velocity frame, finite-difference step and pairwise matrix residuals.
* Repeat the comparison near a singular configuration.

**Task 6. Singularity analysis**
* Sample the feasible joint space using a stated resolution and metric.
* Plot singular regions and report at least one singular or near-singular configuration.
* Include a stick figure with all joint axes and fixed offsets.
* State that Cartesian plots may overlap because different configurations can reach the same point.

Oleg Bulichev
Introduction to Robotics (In2Ro)
8

## [Page 10]
**Submission**
Submit one repository containing:
* pixi.toml or Dockerfile;
* robot model, source code and automated tests;
* a PDF or Markdown report covering Tasks 1-6 in order;
* a README with one command that reproduces every reported number and figure from a clean checkout.

Every task must state its inputs, arm side, conventions, numerical tolerance and verification result. Results that cannot be reproduced from the submitted repository are not accepted.

Oleg Bulichev
Introduction to Robotics (In2Ro)
9

## [Page 11]
**Deserve "A" grade!**
- Oleg Bulichev
bulichev.ov@mipt.ru
@Lupasic
Цифра, 521

---

# w01_03_hangouts (1).pdf

## [Page 1]
Introduction to Robotics
Homework 1-3
AR-601M arm: kinematics, differential kinematics and singularities

**1 Robot and scope**
Oleg Bulichev
AR-601M is an anthropomorphic robot with 41 degrees of freedom. This assignment concerns one arm only: a serial chain with six actuated joints from the shoulder to the tool. The geometry used in the assignment is shown in Figures 1 and 2.

*(Figure 1: AR-601M robot. The assignment covers one six-joint arm.)*

The joint variables are
$q=[q_{1},q_{2},q_{3},q_{4},q_{5},q_{6}]^{T}$.
The required results are a kinematic model, forward and inverse kinematics, differential kinematics, and a numerical map of singular configurations.
The choice of coordinate frames, the mathematical representation of the chain, and the solution methods are part of the assignment. They must be defined and justified in the report.

**2 Mechanism data and constraints**
**2.1 Joint order and axes**
The joints are revolute and occur in the following order from the base to the tool. Each axis is expressed in the local frame associated with that joint.

| Joint | Axis | Lower limit | Upper limit |
| :--- | :--- | :--- | :--- |
| $q_{1}$ | z | $-90^{\circ}$ | $+90^{\circ}$ |
| $q_{2}$ | y | $-105^{\circ}$ | $+105^{\circ}$ |
| $q_{3}$ | x | $-45^{\circ}$ | $+45^{\circ}$ |
| $q_{4}$ | z | $-90^{\circ}$ | $+90^{\circ}$ |
| $q_{5}$ | y | $-90^{\circ}$ | $+90^{\circ}$ |
| $q_{6}$ | x | $-90^{\circ}$ | $+90^{\circ}$ |

1

## [Page 2]
*(Figure 2: Arm geometry and axis annotations used to construct the kinematic model.)*
- 5,7кг
11,7кг
10
Шейный $q1en-2,3k2$
$ropc-2.5xz$

The limits above are closed intervals. A configuration outside any interval is infeasible.

**2.2 Fixed geometry**
The chain contains four fixed offsets:

| Symbol | Offset | Value |
| :--- | :--- | :--- |
| $l_{1}$ | base to shoulder, along +z | 0.40 m |
| $l_{2}$ | shoulder offset, along -by | 0.18 m |
| $l_{3}$ | arm offset, along -z | 0.30 m |
| $l_{4}$ | wrist centre to tool, along +x | 0.12 m |

Here $b=+1$ denotes the right arm and $b=-1$ denotes the left arm. The left arm is the mirror image of the right arm in the x-z plane. Therefore the sign of the $l_{2}$ offset changes with b.
A fixed rotation of $10^{\circ}$ about z is located between joints $q_{1}$ and $q_{2}$. Its sign is $+10^{\circ}$ for $b=+1$ and $-10^{\circ}$ for $b=-1$. This rotation is a fixed geometric parameter, not an additional joint.
Unless a task states otherwise, all translations are expressed in the local frame at their position in the serial chain. Angles used in program code must be converted consistently to radians.

**2.3 Common input data**
All students use the same geometric parameters and joint limits throughout Tasks 1-6. No student-specific parameters or alternative link dimensions are used.
2

## [Page 3]
**3 Required software interface**
The submitted implementation must provide these operations or direct language-equivalent interfaces:
* FK(q, robot) -> T, where $T\in SE(3)$;
* IK(T, robot) -> list [q];
* DifferentialKinematics (q, robot) -> J, where $J\in\mathbb{R}^{6\times6}$ maps joint velocities to tool linear and angular velocities.

The argument robot must be loaded from a separate, human-readable model file. The model file must contain the joint order, joint axes, joint limits, fixed rotation, link offsets, arm side b, and link lengths. Values must not be duplicated as hidden constants in the solver code.

**4 Tasks**

**Task 1. Kinematic model**
Construct the coordinate-frame model of the arm and encode it in the robot model file.
Required evidence:
* a diagram of the frames, joint axes and fixed offsets;
* the complete symbolic transformation chain from base to tool;
* the model-file contents or a report excerpt containing all model parameters;
* a program check that loads exactly six revolute joints in the stated order and with the stated limits.

**Task 2. Forward kinematics**
Implement FK. Evaluate it for:
* the zero configuration;
* one configuration with at least one joint at a limit;
* one generic configuration strictly inside all joint limits.
Required evidence for every configuration:
* the joint vector q and resulting matrix T;
* $||R^{T}R-I||$ and det $R-1$;
* the tolerance used for both checks.
Additionally report the generic pose orientation as ZYX roll-pitch-yaw angles. Use the same three configurations in all subsequent checks where applicable.

**Task 3. Analytical inverse kinematics**
Derive and implement an analytical inverse-kinematics solver. The solver must return all distinct real solution branches before joint-limit filtering and all feasible branches after filtering.
Required evidence:
* the derivation used in the implementation;
* results for both $b=+1$ and $b=-1$;
* explicit handling of unreachable and degenerate targets;
* the complete branch list for each test target;
3

## [Page 4]
* for each returned branch, the residual between the requested target and FK(q, robot);
* a stated rule for identifying duplicate branches.

At least one test target must be generated from a generic feasible joint vector so that reachability is known.

**Task 4. Numerical inverse kinematics**
Implement a numerical inverse-kinematics solver and apply it to the same targets used in Task 3. Use at least two initial configurations that converge to different solution branches.
Required evidence for every run:
* initial configuration;
* termination status;
* number of iterations;
* final configuration;
* final position and orientation residuals;
* nearest analytical branch from Task 3.

Failed runs must remain in the reported results and must include the failure condition.

**Task 5. Differential kinematics**
Derive and implement the differential kinematics of the arm. The result must be a $6\times6$ matrix $J(q)$ satisfying
$v = J(q)\dot{q}$
where v and w are the tool linear and angular velocities expressed in the frame stated in the report. Obtain $J(q)$ by two independent derivations and compare both implementations with a finite-difference approximation based on FK.
Required evidence:
* the two derivations;
* the finite-difference definition and step size;
* the configuration used for comparison;
* pairwise matrix residuals for all three results;
* one additional comparison at a configuration near a singularity.

**Task 6. Singularity analysis**
Sample the feasible joint space and evaluate a stated singularity metric derived from J at every sample. Produce a map or set of plots sufficient to identify singular regions.
Required evidence:
* the sampling ranges, resolution and total number of samples;
* the definition and numerical threshold of the singularity metric;
* the resulting plot or plots with labelled axes and units;
* at least one reported singular or near-singular configuration;
* a stick-figure drawing of that configuration showing all joint axes and fixed offsets;
* verification of the reported configuration using the differential kinematics matrix J.

The report must state that a joint-space sampling may map several different configurations to the same Cartesian point; consequently, a Cartesian plot of such samples
4

## [Page 5]
can contain overlapping branches and is not a one-to-one representation of the configuration space.

**5 Submission format**
Submit one repository containing all of the following:
* pixi.toml or Dockerfile at the repository root;
* the robot model file;
* source code implementing Tasks 1-6;
* automated tests;
* a PDF or Markdown report covering Tasks 1-6 in order;
* a README containing the single command required to reproduce all reported numerical results and figures from a clean checkout.

The submission must run with one of these forms:
* pixi run check; or
* documented docker build and docker run commands.

The report must contain the following information for every task:
* input data and arm side;
* assumptions and conventions;
* requested derivation, numerical result or figure;
* verification result and tolerance;
* a reference to the source file or test that reproduces the result.

Results that cannot be reproduced from the submitted repository are not accepted. Missing branches, omitted failed runs, unstated tolerances, or figures without defined axes and sampling parameters are treated as missing task requirements.

**6 Pre-submission checklist**
* The repository runs from a clean checkout using the documented command.
* The model file contains six joints and all fixed geometric parameters.
* The report states the arm side used for each result.
* Every reported IK branch is checked by forward kinematics.
* Both differential-kinematics implementations and the finite-difference result are compared.
* All plots contain labelled axes, units, legends where needed, and sampling data.
* Every numerical value and figure in the report is generated by submitted code.
5