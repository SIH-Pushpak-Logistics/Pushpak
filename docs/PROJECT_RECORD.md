# PUSHPAK · Project record

What has been built, in what order, and why the main decisions were taken. Status as of 2 Oct 2026.

## Summary

PUSHPAK is IIT Patna's entry to Smart India Hackathon 2026, problem statement SIH26177 (an AI-powered autonomous drone that aids search and rescue by detecting people and hazards; Robotics and Drones; Hardware category; team ID 169135). The idea-stage submission went in on 30 Sep 2026: a six-slide deck, an abstract, a demo video and links to this repository, the documentation folder and the bill of materials.

What exists: a full simulated search stack for the scout drone (Drone B) that flies without GPS, finds people with on-board AI, survives the loss of the command post, and lands itself if its own guidance process dies; a peer-to-peer telemetry link tested on two real laptops; a React command-post dashboard; and a rev-A wiring and power-board design for Drone B. What does not exist yet: any flown hardware, Drone A, hazard detection, rescue ordering and routing.

## Timeline

| Date (2026) | Milestone |
|---|---|
| Jul | First simulation work: drone bring-up, a ROS 2 drone simulation branch, 3D modelling of the site |
| Early–mid Sep | **v0.1 demo build.** Redis message bus; Gazebo + ArduPilot SITL; a GPS-denied visual-inertial pipeline with a single motion-command writer (16 Sep). Its external navigation came from Gazebo ground truth, a stated limitation. Frozen as tag `v0.1-fallback-demo` |
| 16 Sep | First pitch deck |
| 22 Sep | **v2 contract freeze** on `arch/v2-degraded-estimation`: README contract, `pushpak.proto`, `SurvivorDetection.msg`; v0.1 dead nodes purged; perception and radar stubs |
| 24 Sep | v2 container: pinned ROS 2 Humble snapshot, MAVROS 2.14.0, ros_gz, ardupilot_gazebo, PyTorch + Ultralytics (#24) |
| 25 Sep | Phase 1b: Redis and dead v0.1 nodes removed; camera info bridged (#25) |
| 26 Sep | Phase 3 decisions: Zenoh version pin, parameters-in-one-file rule (I-11), drone ID, IMU bias scaffold, radar citation, tripwires, hover-gate protocol (#26). **T-1 passed**: two laptops exchanged heartbeats with no router or broker; peer loss detected in 1.78 s (#28) |
| 27 Sep | **Phase 3: degraded estimation chain and hover gate PASS** (#30): drift tracked bias × time within 0.3 m; FS-3 threshold derived; FS-4 LAND in 1.05 s. Tripwire T-2 fired (no Jetson available) |
| 28 Sep | Routerless Rust Zenoh peer and two-laptop validation (#29); **Phase 4 lawnmower exploration** (#32); Zenoh → WebSocket gateway (#33); dashboard cleaned of v0.1 fabrications (#34); **FS-2 isolation hold** (#35); relaunch cleanup (#36); decisions and Phase 4 evidence recorded (#37); dual perception pipeline (#27) |
| 29 Sep | **Telemetry node**: the vehicle's only Zenoh session, survivor dedup, heartbeats, peer liveness (#38). **Phase 5**: detection flight 1 (3/3), bare-ground run (0 false alarms), run R1 recorded for the video (3/3). Offline rotation sweep (Raunak). Tripwire T-6 fired: no ArduPilot desk rig |
| 30 Sep | Perception node in the launch file with 12-rotation TTA (#40). Drone B wiring rev A and power-board review (Kanishk). **SIH idea submission at about 13:15 IST** |
| 2 Oct | `main` moved to `arch/v2` (3329075); 91 automated tests pass; 12 vs 24 rotation sweep: both 288/288, 12 rotations at half the model time; development moves from the RTX 3050 laptop to an RTX 4070 laptop; this documentation set |

## Decisions and why

| Decision | Reason |
|---|---|
| Rebuild v0.1 as v2 instead of polishing it | v0.1 navigated on ground truth. v2 had to earn the words GPS-denied, decentralized and decoupled |
| Python guidance node (`pushpak_brain`), Rust only for telemetry | Tripwire T-4: the Rust ROS client did not build in the container by day 2. Guidance maths lives in pure functions, testable without ROS |
| Zenoh 1.0 in peer mode, no router or broker | "Decentralized" defined as: the search never waits on the command post. Tested by killing the gateway mid-search |
| Messages ≤ 50 bytes, rejected otherwise | Thin, lossy links; only small reports leave the drone |
| One writer for motion commands (I-3), every tunable in one YAML file (I-11) | Decoupling and reproducibility; FS-4 lives outside the brain because it covers the brain dying |
| Never fuse absolute yaw; report drift, never hide it (I-1) | An honest GPS-denied system has no absolute reference; drift is measured as a sweep over bias values |
| robot_localization 15-state EKF feeding EKF3 | Fastest path to a working chain in a 10-day sprint; now known to fuse the IMU twice, and scheduled for replacement by one filter |
| Emulated radar as truth + noise + bias | Gazebo has no radar; the emulator lets the filter be tested under controlled noise and bias. It is labelled emulated everywhere |
| Rotation test-time augmentation for detection | A COCO person detector only finds top-down people who look upright; rotating the image recovered every tested pose offline |
| 12 rotations in flight instead of 24 | Halves detection work to fit the 2 Hz budget; measured 2 Oct: still 288/288 poses (lowest 0.69) at half the model time (35 vs 71 ms) |
| Visual-velocity pipeline (optical flow) parked; FS-3 deferred | Not ready before 30 Sep; the demo ran the gated, visual-absent configuration |
| Scope: one disaster (earthquake, collapsed buildings) | Depth on one scenario beats shallow coverage of five |
| Thermal and GNSS on the high survey drone (Drone A) | Wide-area survey needs height, GNSS and thermal; the scout must be small and GPS-free |
| Rust port of the brain dropped from the roadmap | No measurable gain for a 20 Hz loop; it competed with thermal, hazards and avoidance |

## Tripwires

Decided in advance and executed without debate when their deadline passed.

| # | Condition | Outcome |
|---|---|---|
| T-1 | Two laptops not exchanging heartbeats over Zenoh, no router | Passed 26 Sep |
| T-2 | Jetson not physically present | Fired 27 Sep: no Jetson; bench work on laptops |
| T-3 | TI radar not streaming on the Jetson within 24 h | Not reached (no Jetson) |
| T-4 | Rust ROS client not building | Fired day 2: brain in Python |
| T-5 | Rubble world (`collapse.sdf`) not merged by day 6 | Fired: demo in `swarm.sdf` |
| T-6 | ArduPilot desk rig not producing detections on the dashboard by 29 Sep | Fired: the team's flight controller has no ArduPilot build; hardware described as a design target |

## What was submitted (30 Sep)

Six slides on the official SIH template, exported as PDF:

1. Title: PUSHPAK, plain-language subtitle, PS SIH26177, team 169135, IIT Patna.
2. Proposed solution: the problem in plain words, what is new against tools responders use today, the four-step mission with status, the dashboard, and PS coverage (six features partial, two planned).
3. Technical approach: Drone B data flow, stack, hardware for both drones and the command post, method (SITL done → bench started → HIL → field), eight bottleneck → solution rows.
4. Feasibility: tiles (0.58 m, no wait, 1.05 s, 3/3), the drift-law chart, the fail-safe ladder, the Drone B wiring rev A, challenges and strategies, bench photos.
5. Impact: earthquake scope, survivor map from run R1, rescue order and route illustration, field reality (battery, relay swap, charging, setup, pins, Drone Rules 2021), who benefits, cost ₹3.87–4.50 lakh.
6. Research and references: repository QR code and links, roadmap phases 1–4, four references (Kramer 2020; Doer & Trommer 2020; Macintyre et al. 2011; Koenig & Likhachev 2002).

Colour rule on every slide: blue means measured in simulation, orange means designed or planned.

Also submitted on the portal: title and abstract, technology bucket Mechatronics, and the demo video.

## Machines

| Machine | Role |
|---|---|
| RTX 3050 laptop (Ubuntu 22.04) | Development and every logged run, 26 Sep–2 Oct |
| RTX 4070 laptop (Ubuntu 22.04) | Development machine from 2 Oct; the Docker image builds on it |
| MacBook | Dashboard viewing; second machine in the two-laptop Zenoh test |

## Team

| Person | Owns |
|---|---|
| Ashutosh (lead) | State machine, estimator and middleware; simulation and navigation stack; integration; gateway and dashboard since 28 Sep |
| Aditya | Simulation worlds and survivor decals; operator dashboard |
| Raunak | Perception pipeline (YOLO + rotation TTA, optical flow); offline rotation sweep; drift animation and analysis tools |
| Kanishk | Hardware: Drone B wiring and power board, bench rig, 5-inch test quad; Rust telemetry library and peer |
| Priya, Bhavya | Presentation and architecture defence |

## Pull requests merged into `arch/v2`

| # | Title |
|---|---|
| 24 | Infra/v2 container |
| 25 | Phase 1b: remove Redis and dead v0.1 nodes; bridge camera_info |
| 26 | Phase 3 decisions: zenoh pin, I-11, drone_id, S-7, radar citation, tripwires, hover-gate protocol |
| 27 | Complete dual perception pipeline |
| 28 | T-1 passed (Zenoh two-laptop); the demo network must allow device-to-device traffic |
| 29 | Routerless Zenoh telemetry peer and two-laptop validation |
| 30 | Phase 3: degraded estimation chain and hover gate (PASS) |
| 32 | Phase 4 exploration |
| 33 | Zenoh gateway (peer 0) to the dashboard WebSocket |
| 34 | Dashboard: null-safe link status; velocity/flow_debug and Redis leftovers removed; live drone ID |
| 35 | FS-2 isolation hold from `/pushpak/peers_alive` |
| 36 | Kill surviving ROS nodes on relaunch; SITL LAND panic not reproduced |
| 37 | 28 Sep decisions and Phase 4 evidence |
| 38 | Telemetry node: keyframes, survivors with dedup, heartbeat, peer liveness |
| 39 | Offline drift animation and TTA sweep tools; drift card reports absolute error; 12 vs 24 rotation sweep |
| 40 | Launch perception_node (12-rotation TTA, visual velocity off); parameter-type note |

