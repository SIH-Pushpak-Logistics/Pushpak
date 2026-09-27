# v2 integration audit — 27 September 2026

## Scope and source

This is an integration candidate based on `arch/v2-degraded-estimation`
`c617bb6`, incorporating telemetry `34efa44`, HITL `9166b72`, and the current
perception PR head `b86f894`. The perception remote had been rebased; the candidate
uses its current file contents. `main` was neither used nor modified.
Existing feature branches and their review state remain separate from this candidate.

Reviewed the ROS producers/consumers, launch and EKF configuration, frozen messages,
Rust transport/replay, adapter conversion and process lifecycle, browser data path,
and existing test/evaluation scripts. This is not a certification of all runtime
behaviour. ROS, Gazebo, CUDA inference and FCU hardware cannot be exercised on this
Windows host; no Docker executable was available.

## Findings and implemented corrections

| Finding | Correction |
| --- | --- |
| Dashboard still read Redis after v2 removed its producers | Added peer-mode Zenoh gateway using frozen protobuf and WebSocket snapshots; removed Redis server/dependencies |
| Scout had no ROS telemetry publisher; adapter was hardcoded to rig 2 | Shared adapter/launch supports scout 1 and rig 2 |
| Adapter repeated old odometry indefinitely and discarded frame identity | Reject stale/future odometry and non-odom positions; preserve source timestamps |
| Browser reconnect could survive component cleanup | Dispose guard prevents reconnect and state updates after unmount |
| Dashboard labelled odom Z as AGL and hardcoded drone 00 | Accurate height label and selected vehicle ID; unknown wire values remain unavailable |
| No source launch joined current SITL, perception and telemetry | Added `tools/integrated_sitl.launch.py`; explicit takeoff opt-in |
| ToF history could establish settled altitude after readings stopped | Evaluate freshness at decision time; clear invalid readings; require current settled readings before leaving WAIT_LINK |
| FS-4 finished after sending LAND even if FCU did not accept it | LANDING state retries until observed LAND or disarmed |

The gateway is a ground peer, not a router. Scout and rig must have a direct link.
The gateway tracks scout 1 and rig 2 separately. The dashboard shows both link
states and lets the operator select a vehicle without merging their odom origins.
It does not synthesize unsupported sensor fields or flight state.

## Verified locally

- Rust telemetry: **5 tests passed**, including all-message exchange and expiry.
- Rust replay peer: **2 tests passed**.
- Rust ROS publisher: **3 tests passed**, including ground-peer shutdown.
- Python adapter: **9 tests passed**, including child death, bounded writes,
  stale/future odometry, wrong frames, units and timestamps.
- Existing fake-ROS three-peer rehearsal: **passed** with real Rust/Zenoh processes.
- Perception mathematical core: **24 tests passed** against current PR27 code.
- Gateway state/validation: **3 tests passed**.
- Actual Rust scout 1 and rig 2 → Zenoh → gateway → WebSocket: **1 test passed**,
  checking separate positions and survivor IDs, scout loss while rig remains
  online, scout recovery, pose expiry and unit conversion.
- Live browser on local loopback: both vehicle positions and link states rendered;
  scout loss cleared only scout pose; recovery restored it; a Rust-published
  rig survivor event appeared with ID `2:42` and 93% confidence.
- Handshake state regressions with ROS imports stubbed: **2 tests passed**.
- Dashboard TypeScript/production build and oxlint: **passed**.
- Python compile checks and Git whitespace check: **passed**.

The browser check used three local processes, not three physical machines.
The fake ROS tests do not establish DDS QoS compatibility or ROS launch success.
Perception ROS integration tests still require Humble plus built interfaces.
The handshake tests do not verify FCU acceptance or physical LAND behaviour.

## Run the combined candidate on the ROS/Gazebo machine

From the candidate repository mounted at `/workspace`, with the existing v2 image:

```sh
source /opt/ros/humble/setup.bash
source /bridge_ws/install/setup.bash
colcon build --symlink-install
source install/setup.bash
cargo build --release --locked --manifest-path hitl/zenoh_publisher/Cargo.toml
ros2 launch tools/integrated_sitl.launch.py \
  listen_endpoints:='["tcp/0.0.0.0:7447"]'
```

This starts with `auto_takeoff:=false`. The owner must validate the integrated
ROS launch, camera calibration/ToF/odometry freshness, and existing hover gates
before requesting a simulation takeoff. `device:=cpu` is explicit for a machine
without CUDA; no performance claim is made for CPU inference.

On the ground machine (see dashboard README for browser setup):

```sh
python -m pip install -r tools/requirements-gateway.txt
python tools/zenoh_gateway.py --connect tcp/SCOUT_IP:7447 --connect tcp/RIG_IP:7447
```

For rig 2 use the adapter's existing launch with `drone_id:=2`, an explicit
`pose_source`, and endpoints connecting directly to scout 1 as well as ground 0.
Mock pose is demonstrator data, not real localisation. Physical camera/FCU drivers
and an odometry producer are still required for real rig detection coordinates.

## Still incomplete or unverified

1. **Mission behaviour:** `pushpak_brain` only holds zero velocity after airborne.
   Exploration, keyframe FIFO/backtracking, peer-isolation response (FS-2), visual
   degradation response (FS-1), topological backtrack (FS-3), and corresponding
   status flags are absent. Their parameter declarations do not implement them.
   Peer liveness currently reaches transport logging/dashboard, not brain control.
2. **Known v2 FCU problem:** README reports SITL panic on LAND with an uninitialised
   origin. Retrying LAND does not fix that underlying firmware/origin issue.
3. **Physical evidence:** actual three-machine scout/rig/ground acceptance, real
   camera inference, ROS timing/QoS and hardware I/O remain to be performed.
   Run `tools/capture_three_peer_evidence.sh` on the Ubuntu/Mac hosts to capture
   both ping directions and raw Ubuntu `pgrep` output; the old supplied logs
   contain neither. A local loopback run is not physical acceptance evidence.
4. **Tabletop IoT:** gas/ultrasonic/environment/PIR sensors have no drivers or
   agreed dashboard transport yet. The frozen drone protobuf has no such fields.
5. **Estimation limits:** current perception projection uses a level/downward-camera
   yaw approximation; VIO speed rotation is yaw-only. Existing README restricts
   the demo envelope. These are not general tilted-flight geometry solutions.
6. **Configuration/launch debt:** the README's common `pushpak_params.yaml` is not
   present. The existing launch harness also uses global process-name cleanup;
   it should run only in its dedicated simulation container.

This branch closes the tested telemetry-to-dashboard gap. It does not establish
that the full autonomous mission or physical tabletop presentation is complete.
