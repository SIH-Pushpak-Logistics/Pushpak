# Milestone B — Baseline Hover (PASSED)

## Result
Hover held at **4.8425 m** vs 5.0 m commanded.
Drift 5.8e-5 m/sample. Lateral x=0.0199 y=0.0167, drift ~1e-5 m/sample.
Source: `gz topic -e -t /world/empty/dynamic_pose/info`, model `swarm_drone` id 14.

## T19 — ANSWERED, CLOSED
1000:1 tether mass ratio (0.001 kg `tether_dummy_link` between 1.5 kg base and
1.0 kg payload, two revolute joints, +/-0.5 rad limits) does NOT destabilise
hover and does NOT diverge the solver. Payload chain needs no redesign.

## Airframe had NO aerodynamics until this session
`ArduPilotPlugin` only writes `JointVelocityCmd`; it applies no force.
No LiftDrag existed. Every prior flight assumption was unfounded.
Fix: 8 `gz-sim-lift-drag-system` plugins (2 blades/rotor) in `drone.urdf.xacro`.
`area` = 0.002 x (2.751/1.75) = **0.00314**. Vehicle mass 2.751 kg vs iris 1.75 kg.
cp +/-0.084 transfers unchanged (both rotors radius 0.1 m).
Chirality: rotors 0,1 -> `forward 0 1 0`; rotors 2,3 -> `forward 0 -1 0`.

## Versions
ArduCopter V4.4.4 `865cffa5` | ros_gz `28e586a3` | ardupilot_gazebo `082a0fe`

## Operational notes
- Arm sequence: GUIDED -> arm -> takeoff, one shell, ~2 s gaps.
  STABILIZE auto-disarms after ~10 s idle.
- `EK3_SRC1_VELZ 1` = **OpticalFlow**, NOT barometer. Old comment was wrong.
- Four `MILESTONE B ONLY` lines in base_iris.param revert at Milestone E.
- MAVROS: services work, topics need `SR1_*` params (see T22).