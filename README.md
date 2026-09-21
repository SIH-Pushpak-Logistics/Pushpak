# PUSHPAK — GPS-Denied Decentralized Search & Rescue

**Motto: GPS-DENIED · DECENTRALIZED · DECOUPLED.**
Every change strengthens at least one of these, or is registered in §13 as a scaffold
with a written removal gate.

| Ref | What it is |
|---|---|
| `v0.1-fallback-demo` (tag), `fallback/v0.1-demo` (branch) | Frozen submission floor. Redis bus. **ExtNav fed from Gazebo ground truth** — stated limitation. Never merge into it. |
| `arch/v2-degraded-estimation` | This document. All v2 work targets this branch. |
| `main` | Equals the fallback tag until v2 flies end-to-end. Not touched. |

---

## 1. Frozen Stack

ArduPilot SITL Copter-4.4 (`865cffa5`) · Gazebo Harmonic · ROS 2 Humble ·
`robot_localization` 15-state EKF · Rust (`ros2_rust`) guidance node ·
native `zenoh` Rust crate in peer mode · Protobuf via `prost` ·
Python gateway → React dashboard.

**Out of scope, by decision:** PX4, Micro XRCE-DDS, Gazebo Classic, ROS 2 Jazzy, Redis,
`zenoh-pico`, `zenohd` routers, wind/downwash/ground-effect/Coanda modelling, Doppler
ray tracing, airframe re-tuning, the Flood world.

---

## 2. Invariants

A PR that breaks one is rejected without review.

| # | Invariant |
|---|---|
| **I-1** | No absolute position sensor exists. All horizontal position is dead-reckoned. Drift is reported, never hidden. |
| **I-2** | `robot_localization` is the sole fusion point. EKF3 receives ExtNav from it and nothing else. No second filter anywhere. |
| **I-3** | `pushpak_brain` is the single publisher to `/mavros/setpoint_velocity/cmd_vel`. No node publishes to any `/mavros/setpoint_*` topic. Subscribing for monitoring is allowed. |
| **I-4** | One actuator interface for the whole mission: velocity. Never switch to position setpoints in flight. |
| **I-5** | Vertical is independent of planar. EKF3 takes height directly from the rangefinder (`EK3_SRC1_POSZ 2`). |
| **I-6** | Degradation is covariance, never reconfiguration. A failing sensor inflates its own R. It never stops publishing. No restarts, no runtime param writes, no remaps. |
| **I-7** | No central broker. Zenoh runs peer-to-peer. The ground station is a peer, never a router. Killing it must not change any drone's behaviour while another peer is alive. |
| **I-8** | Simulation code is quarantined. Sim-only nodes carry the `sim_` prefix and are absent from any hardware launch file. `/sim/*` topics are consumed only by `sim_` nodes and offline evaluation. No sim frame convention or magic constant appears in a non-`sim_` node. |
| **I-9** | No dead streams. A topic, stream, message or file with no consumer is deleted. |
| **I-10** | Every injected error parameter (noise σ, bias, bias random-walk) cites an external source in the code's PR description. Drift is reported as a sweep over bias values, never a single hand-picked figure. |

---

## 3. Control Loop

```
GAZEBO HARMONIC  (world name "empty" — bridge.yaml depends on it)
 |  camera 320x240 @30Hz -----------> /camera/image_raw, /camera/camera_info
 |  ros_imu @200Hz ------------------> /imu/raw
 |  gpu_lidar (1D ToF, 4 m) @30Hz ---> /drone/rangefinder/scan
 |  OdometryPublisher ---------------> /sim/ground_truth/odom      [SIM ONLY]
 |  imu_sensor + rangefinder --------> ArduPilotPlugin (JSON FDM 9002/9003)
 v
EMULATION / PERCEPTION
  altimeter_node            LaserScan -> /drone/tof_range (Range)
                                      -> /ekf/altitude_pose (Z only)
  sim_radar_emulator_node   /sim/ground_truth/odom (body twist)
                            + N(0, 0.15 m/s) + bias random walk
                            -> /radar/ego_velocity
  perception_node
    Pipeline A  Lucas-Kanade + gyro de-rotation + pinhole(camera_info, ToF)
                -> /visual/velocity   (R ramps 0.05 -> 1e6 under dust)
    Pipeline B  YOLOv8n, conf >= 0.85, 2 Hz
                -> /detections/survivor
 v
ESTIMATION
  robot_localization ekf_node @50Hz
    in : /imu/raw, /radar/ego_velocity, /visual/velocity, /ekf/altitude_pose
    out: /odometry/filtered                    SINGLE SOURCE OF TRUTH
  vio_bridge_node @30Hz
    /odometry/filtered -> /mavros/vision_pose/pose
                       -> /mavros/vision_speed/speed_twist_cov  -> EKF3
 v
GUIDANCE
  arm_takeoff_handshake.py   GUIDED -> arm -> takeoff 1.5 m
                             -> /pushpak/airborne (latched Bool)
                             then stays alive as FS-4 watchdog
  pushpak_brain (Rust) @20Hz, starts on /pushpak/airborne == true
    keyframe FIFO (5 Hz sample, 200 poses)
    exploration waypoint list -> P follower -> clamps -> failsafes
    survivor dedup (1.5 m radius)
    -> /mavros/setpoint_velocity/cmd_vel      SINGLE WRITER
    -> zenoh: keyframe / survivor / heartbeat
 v
TELEMETRY (peer-to-peer, no router)
  pushpak_brain <-> pushpak_peer / HITL rig <-> zenoh_gateway.py
  zenoh_gateway.py -> ws://<host>:8765 -> React dashboard
```

---

## 4. Frames and Clocks

- `odom`: ENU, `robot_localization` world frame, origin at the takeoff point.
- `base_link`: FLU body frame. All sensor twists are expressed in `base_link`.
- MAVROS `setpoint_velocity.mav_frame: LOCAL_NED`. `pushpak_brain` publishes world-frame
  ENU vectors; MAVROS converts to NED.
- `robot_localization` runs with `publish_tf: false`. MAVROS already publishes a
  `map -> base_link` transform; two parents for `base_link` breaks TF.
- Every node runs `use_sim_time: true`. Staleness is always header stamp vs node clock.
  **Never compare a sim-time stamp to `time.time()` or `time.monotonic()`.** v0.1 mixed
  them; with real-time factor below 1.0 the two drift apart.
- Zenoh `timestamp_ms` = sim time in ms.

---

## 5. ROS 2 Topic Contract

### Sensor inputs (bridge)
| Topic | Type | Rate |
|---|---|---|
| `/camera/image_raw` | `sensor_msgs/Image` | 30 Hz |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | 30 Hz |
| `/imu/raw` | `sensor_msgs/Imu` | 200 Hz |
| `/drone/rangefinder/scan` | `sensor_msgs/LaserScan` | 30 Hz |

`/imu/raw` comes from a **second** Gazebo IMU named `ros_imu` with identity pose.
ArduPilot keeps binding to the original `imu_sensor` (rolled 180°); `launch_sim.sh`'s
sentinel `swarm_drone::base_link::imu_sensor` verifies that binding on every boot.
`/mavros/imu/data` is **not** an estimation input: its orientation is EKF3's output,
and feeding it to `robot_localization` closes a yaw feedback loop through ExtNav.

### Simulation only
| Topic | Type | Only allowed consumer |
|---|---|---|
| `/sim/ground_truth/odom` | `nav_msgs/Odometry` | `sim_radar_emulator_node`, offline evaluation bags |

### Estimation bus
| Topic | Type | Producer | Rate |
|---|---|---|---|
| `/drone/tof_range` | `sensor_msgs/Range` | `altimeter_node` | 30 Hz |
| `/ekf/altitude_pose` | `geometry_msgs/PoseWithCovarianceStamped` | `altimeter_node` | 30 Hz |
| `/radar/ego_velocity` | `geometry_msgs/TwistWithCovarianceStamped` | `sim_radar_emulator_node` | 20 Hz |
| `/visual/velocity` | `geometry_msgs/TwistWithCovarianceStamped` | `perception_node` | ≥ 14 Hz |
| `/odometry/filtered` | `nav_msgs/Odometry` | `robot_localization` | 50 Hz |

`robot_localization` does not accept `sensor_msgs/Range`; `altimeter_node` republishes
altitude as a Z-only pose for that reason.

### Detection
| Topic | Type | Producer | Rate |
|---|---|---|---|
| `/detections/survivor` | `drone_interfaces/SurvivorDetection` | `perception_node` | ≤ 2 Hz |

### Guidance and actuation
| Topic | Type | Producer |
|---|---|---|
| `/pushpak/airborne` | `std_msgs/Bool`, QoS transient-local | `arm_takeoff_handshake.py` |
| `/mavros/vision_pose/pose` | `geometry_msgs/PoseStamped` | `vio_bridge_node` |
| `/mavros/vision_speed/speed_twist_cov` | `geometry_msgs/TwistWithCovarianceStamped` | `vio_bridge_node` |
| `/mavros/setpoint_velocity/cmd_vel` | `geometry_msgs/TwistStamped` | `pushpak_brain` **only** |

ArduPilot `GUID_TIMEOUT` (3 s): if `cmd_vel` stops, the vehicle brakes and drifts.
It does not hold position. FS-4 covers this.

---

## 6. `robot_localization` Fusion Matrix

| Input | x y z | roll pitch yaw | vx vy vz | vroll vpitch vyaw | ax ay az |
|---|---|---|---|---|---|
| `imu0` `/imu/raw` | – – – | ✓ ✓ **✗** | – – – | ✓ ✓ ✓ | tunable |
| `twist0` `/radar/ego_velocity` | – – – | – – – | ✓ ✓ ✓ | – – – | – – – |
| `twist1` `/visual/velocity` | – – – | – – – | ✓ ✓ – | – – – | – – – |
| `pose0` `/ekf/altitude_pose` | – – ✓ | – – – | – – – | – – – | – – – |

**Absolute yaw from the IMU is never fused.** Heading is integrated from `vyaw` and
drifts. That is the honest GPS-denied behaviour, and it breaks the ExtNav yaw loop.
`frequency: 50`, `two_d_mode: false`, `world_frame: odom`, `publish_tf: false`.
Roll/pitch from the sim IMU is noise-free; see scaffold S-6.

---

## 7. Custom Message — `drone_interfaces/msg/SurvivorDetection.msg`

```
std_msgs/Header header
float32 confidence
float32 bbox_x
float32 bbox_y
float32 bbox_w
float32 bbox_h
geometry_msgs/Point world_position
string image_path
```
`world_position` is in `odom`, computed from `/odometry/filtered` at capture time,
bbox centre, `camera_info` intrinsics, and ToF altitude.

---

## 8. Zenoh + Protobuf Contract — `proto/pushpak.proto`

Peer mode, multicast scouting, optional explicit `tcp/<ip>:7447` endpoints for networks
that block multicast. **No `zenohd` process runs anywhere in this system.**
Containers use `network_mode: host` — multicast discovery does not cross Docker bridges.

| Key | Message | Rate |
|---|---|---|
| `pushpak/keyframe/{drone_id}` | `SubMapKeyframe` | 5 Hz |
| `pushpak/survivor/{drone_id}` | `SurvivorEvent` | on new or updated survivor |
| `pushpak/heartbeat/{drone_id}` | `Heartbeat` | 2 Hz |

`drone_id`: `0` ground station gateway, `1` SITL scout, `2` HITL rig, `3+` `pushpak_peer`.
The gateway publishes heartbeats because it is a peer.

`status_flags` bits: `0` VIO_Active · `1` Radar_Active · `2` Survivor_Found ·
`3` Backtracking · `4` Isolated (FS-2 active).

```protobuf
syntax = "proto3";
package swarm.telemetry;

message SubMapKeyframe {
  uint32 timestamp_ms = 1;
  uint32 drone_id     = 2;
  sint32 pos_x_mm     = 3;
  sint32 pos_y_mm     = 4;
  sint32 pos_z_mm     = 5;
  sint32 roll_cdeg    = 6;
  sint32 pitch_cdeg   = 7;
  sint32 yaw_cdeg     = 8;
  uint32 status_flags = 9;
}

message SurvivorEvent {
  uint32 timestamp_ms   = 1;
  uint32 drone_id       = 2;
  uint32 survivor_id    = 3;
  sint32 pos_x_mm       = 4;
  sint32 pos_y_mm       = 5;
  sint32 pos_z_mm       = 6;
  uint32 confidence_pct = 7;
  uint32 hit_count      = 8;
}

message Heartbeat {
  uint32 timestamp_ms = 1;
  uint32 drone_id     = 2;
  uint32 status_flags = 3;
}
```

Measured serialized sizes (round-trip verified): `SubMapKeyframe` 29 B typical,
35 B worst case · `SurvivorEvent` ≤ 29 B · `Heartbeat` ≤ 11 B. All under 50 B.
Proto3 has no 16-bit integer type; `sint32` with zigzag encoding costs the same bytes
for centidegree values.

**Version pin: OPEN — owner Kanishk, `feat/zenoh-telemetry`.** One exact 1.x version,
shared by the Rust crate, the Python `eclipse-zenoh` package and the Dockerfile.

---

## 9. Dashboard WebSocket Contract

`ws://<host>:8765`, envelope `{"type": <string>, "data": <object>}`.
`zenoh_gateway.py` translates Zenoh messages into the v0.1 types so `dashboard/src/`
keeps working. `drone_id` becomes the string `drone_%02d`.

| WS type | Built from | Fields |
|---|---|---|
| `pose` | `SubMapKeyframe` | `timestamp` (s), `drone_id`, `x`,`y`,`z` (m), `yaw_deg` |
| `altitude` | `SubMapKeyframe` | `timestamp`, `drone_id`, `z` |
| `victims` | gateway's survivor table | list of `victim_id` (`v_%03d`), `confidence` (0–1), `world_x`, `world_y`, `first_seen`, `last_seen`, `hit_count`, `image_path` (`""` — not transmitted), `acked` (gateway-local) |
| `status` | heartbeat table | `timestamp`, `drone_id`, `state` (`ONLINE` if any peer heard < 2 s, else `OFFLINE`), `cached_packets` 0, `last_sync_sec`, `rssi_dbm` **`null`** |
| `ekf_health` | `status_flags` | `timestamp`, `drone_id`, `vio_active`, `radar_active`, `backtracking`, `isolated` |

`rssi_dbm` is `null` because nothing measures it. v0.1 fabricated RSSI; that stops.
`velocity` and `flow_debug` are no longer transmitted.
Allowed `dashboard/src/` edits: `LinkStatus.rssi_dbm` becomes `number | null`, and one new
`EkfHealthPanel.tsx`.

---

## 10. Failsafe Hierarchy

Evaluated in `pushpak_brain` at 20 Hz in this priority order. First match wins.

| Priority | Failsafe | Trigger | Action |
|---|---|---|---|
| 1 | **FS-5 FCU rejection** | MAVROS mode ≠ GUIDED, or disarmed, while airborne | Stop publishing, log, never fight the FCU. |
| 2 | **FS-3 Topological backtrack** | Visual covariance ≥ 1e6 **and** `Tr(Σ_v)` of `/odometry/filtered` > `backtrack_cov_threshold` | Abort exploration. Reverse the keyframe FIFO at ≤ 1.0 m/s, acceptance sphere 0.4 m, until the visual baseline returns or any peer is heard. |
| 3 | **FS-2 Isolated** | No heartbeat from **any** other peer, ground station included, for > 2.0 s | Halt exploration and hold (zero velocity). Set bit 4. |
| 4 | **FS-1 Visual dropout** | Visual covariance ≥ 1e6 | Continue on radar-inertial. Clear bit 0. |
| — | **FS-4 Guidance liveness** | `arm_takeoff_handshake.py` sees no `cmd_vel` for > 1.0 s after airborne | Sidecar commands `SetMode LAND`. Lives outside `pushpak_brain` because it covers `pushpak_brain` dying. |

`backtrack_cov_threshold`: **OPEN — owner Ashutosh**, derived from hover-gate data.

---

## 11. ArduPilot Parameter Lock — `firmware/ardupilot_config/base_iris.param`

Each parameter is declared exactly once. ArduPilot silently ignores misspelled names.

```
GPS_TYPE         0
AHRS_EKF_TYPE    3
COMPASS_USE      0
VISO_TYPE        1
VISO_DELAY_MS    0

EK3_SRC1_POSXY   6
EK3_SRC1_VELXY   6
EK3_SRC1_POSZ    2
EK3_SRC1_VELZ    0
EK3_SRC1_YAW     6

EK3_CHECK_SCALE  200
FS_EKF_THRESH    1.0
FS_EKF_ACTION    1

RNGFND1_TYPE     100
RNGFND1_MIN_CM   10
RNGFND1_MAX_CM   400
RNGFND1_ORIENT   25
RNGFND1_GNDCLEAR 10

ARMING_CHECK     1
```
`EK3_CHECK_SCALE` default is 100; 200 is the loosened gate. `FS_EKF_THRESH` is the trip
threshold itself. `RNGFND1_MAX_CM 400` matches a VL53L1X-class ToF; the Gazebo lidar
`<max>` is 4.0 to agree. Mass, inertia, `ATC_*`, `MOT_THST_EXPO` and all eight
`LiftDrag` blocks stay at v0.1 values; the Cinewhoop change is a visual mesh only.

---

## 12. Packages and Ownership

| Path | Owner |
|---|---|
| `Dockerfile`, `docker-compose.yml`, all `launch/`, `bridge.yaml`, `setup.py`, `package.xml`, `CMakeLists.txt` | Ashutosh — sole editor. Others request changes in their PR description. |
| `src/navigation_brain/.../altimeter_node.py`, `sim_radar_emulator_node.py`, `vio_bridge_node.py`, `config/ekf_15state.yaml`, `tools/arm_takeoff_handshake.py`, `src/pushpak_brain/`, `firmware/` | Ashutosh |
| `src/navigation_brain/.../perception_node.py` | Raunak |
| `proto/pushpak.proto` (frozen), `src/pushpak_telemetry/`, `src/pushpak_peer/`, `hitl/` | Kanishk |
| `src/drone_description/worlds/collapse.sdf`, `meshes/collapse/`, `tools/zenoh_gateway.py`, `dashboard/` | Aditya |
| `src/drone_interfaces/` | Frozen contract. Changes need team announcement. |

---

## 13. Scaffold Register

Each entry is a known compromise with a removal gate.

| # | Scaffold | Why it exists | Removal gate |
|---|---|---|---|
| S-1 | `sim_radar_emulator_node` | No radar in Gazebo | IWR6843 driver publishing on hardware |
| S-2 | `OdometryPublisher` / `/sim/ground_truth/odom` | Radar emulation source; offline evaluation | Same as S-1 |
| S-3 | `sensor_primer` model, IMU-retry patch, `launch_sim.sh` retries | gz-sim sensor registration race | Upstream `ardupilot_gazebo` fix |
| S-4 | `set_gp_origin` / `CommandHome` with a fixed lat/lon | EKF3 needs an arbitrary origin to run a local frame; no GPS is used | None needed on hardware — any arbitrary origin; documented, not removed |
| S-5 | `swarm.sdf` | Only world until `collapse.sdf` merges | `collapse.sdf` merged |
| S-6 | Noise-free roll/pitch from sim IMU | Gazebo IMU orientation is exact | Hardware IMU with onboard AHRS |

---

## 14. Branch and PR Protocol

```bash
cd sih_drone_root
git fetch origin
git checkout -b <feat/your-branch> origin/arch/v2-degraded-estimation
```
- PRs target `arch/v2-degraded-estimation`. Never `main`, never `fallback/*`.
- Every PR description states: which invariant(s) it serves, the verification commands
  run, and their output.
- Review submission: `git diff origin/arch/v2-degraded-estimation...<branch>`.
- A topic with no consumer closes the PR under I-9.
- Rosbags and model weights go on the shared drive, never in git.

Container commands from the host must source all three setups explicitly:
```bash
docker exec -i swarm_brain_container bash -s <<'EOF'
source /opt/ros/humble/setup.bash
source /bridge_ws/install/setup.bash
source /workspace/install/setup.bash
# commands here
EOF
```

---

## 15. Honest Limitations

State these before judges find them.

1. **Horizontal position drift is unbounded, and in simulation its magnitude is set by
   the injected radar bias.** White noise at σ = 0.15 m/s integrates to only ~0.2–0.3 m
   over 90 s. Drift of metres comes from bias: 0.05 m/s → 4.5 m, 0.15 m/s → 13.5 m over
   90 s. We therefore report drift as a sweep across cited bias values (I-10), and demo
   runs are bounded to 90 s.
2. **Radar is emulated** from simulator body velocity plus noise and bias. We test filter
   behaviour under noise, not radar physics.
3. **Heading drifts.** Absolute yaw is never fused (§6).
4. **Backtracking follows the estimate, not the truth.** The vehicle returns to where the
   filter believes it has been.
5. **No aerodynamic environment.** Wind, downwash, ground effect, wall suction: out of
   scope by decision.
6. **The v0.1 fallback** navigates on ground-truth ExtNav and is labelled as such.

---

## 16. Open Items

| Item | Owner | Due |
|---|---|---|
| Zenoh exact version pin | Kanishk | Day 2 |
| Radar noise, bias and bias random-walk values with citations | Ashutosh | Day 2 |
| Laplacian-variance dust threshold | Raunak | Day 3 |
| `backtrack_cov_threshold` | Ashutosh | Day 4 (hover gate) |