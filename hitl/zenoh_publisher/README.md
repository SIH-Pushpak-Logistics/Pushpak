# Tier 1 HITL Zenoh publisher

The rig is drone `2`. `ros_adapter.py` subscribes to the frozen ROS topics and
passes newline-delimited JSON to the Rust binary through stdin. The Rust binary
uses `pushpak_telemetry` to send the frozen protobuf messages as a Zenoh peer.
There is no `zenohd` process.

This split keeps ROS time and subscriptions in `rclpy` and Zenoh/protobuf in
the existing Rust crate. It does not fork the detector. The adapter expects
Raunak's `perception_node` to publish `drone_interfaces/SurvivorDetection` on
`/detections/survivor` with `world_position` in `odom`.

## Build on the Jetson

From the repository root, after sourcing ROS 2 Humble and the workspace that
builds `drone_interfaces`:

```bash
cargo test --locked --manifest-path hitl/zenoh_publisher/Cargo.toml
cargo build --release --locked --manifest-path hitl/zenoh_publisher/Cargo.toml
python3 -m unittest discover -s hitl/zenoh_publisher -p 'test_conversion.py'
```

Build natively on the Jetson. Python needs `rclpy`, `nav_msgs`, and the built
`drone_interfaces` message package. The Rust binary is found under
`hitl/zenoh_publisher/target/release/` by default; `rust_binary` can override
the path.

Before using hardware, run the local integration rehearsal. It replaces ROS
callbacks with test messages but runs the actual Python adapter, Rust publisher,
Protobuf encoder, and three Zenoh peer processes over loopback TCP:

```bash
cargo build --locked --manifest-path src/pushpak_peer/Cargo.toml
cargo build --locked --manifest-path hitl/zenoh_publisher/Cargo.toml
python3 hitl/zenoh_publisher/simulate_integration.py
```

The script checks Heartbeat, Keyframe, and SurvivorEvent delivery, switches from
mock to odometry pose, stops peer `0`, and checks that peers `1` and `2` keep
communicating while `0` is reported lost within two seconds. It requires no ROS
installation or Jetson and does not replace the physical three-machine test.

## Tier 1: fixed mock pose

The launch file exposes `pose_source` as a launch argument. From the repo
root, a direct launch is:

```bash
ros2 launch hitl/zenoh_publisher/launch/zenoh_publisher.launch.py \
  pose_source:=mock 'connect_endpoints:=["tcp/192.168.8.10:7447"]'
```

You can also run the adapter directly:

```bash
python3 hitl/zenoh_publisher/ros_adapter.py --ros-args \
  -p drone_id:=2 -p pose_source:=mock \
  -p mock_x_m:=0.0 -p mock_y_m:=0.0 -p mock_z_m:=1.0 \
  -p mock_yaw_deg:=0.0 \
  -p 'connect_endpoints:=["tcp/192.168.8.10:7447"]'
```

The endpoint is optional when multicast scouting works. Use an IP on the
travel-router network when it does not. The named peer at that address must
listen with `--listen tcp/0.0.0.0:7447`. `drone_id` is fixed to `2` by the
contract. `/detections/survivor` generates `SurvivorEvent` on each detection,
with nearby detections grouped into a stable `survivor_id` and increasing
`hit_count`. Heartbeats run at 2 Hz and keyframes at 5 Hz. Timestamps come
from the ROS clock (so set `use_sim_time` correctly for the running platform).

## Tier 3: odometry pose

Change only the parameter:

```bash
ros2 launch hitl/zenoh_publisher/launch/zenoh_publisher.launch.py \
  pose_source:=odometry
```

This subscribes to `/odometry/filtered`. Before the first odometry message,
the adapter sends heartbeats but no keyframes. It never substitutes the mock
pose in this mode.

## Ground-station-loss test

Use three machines on one travel router with IDs `0` (dashboard/gateway), `1`
(SITL), and `2` (rig). Ensure no machine runs `zenohd` (`pgrep -a zenohd`
prints nothing). Configure direct `tcp/<ip>:7447` peer connections between
the SITL host and rig as well as any dashboard connections. Verify `1` and `2`
log each other's heartbeats and keyframes. Then stop the ground-station peer
`0`. IDs `1` and `2` must keep exchanging traffic, while both report peer `0`
lost within the heartbeat timeout. A three-process test on one computer helps
check the setup, but the travel-router test is the acceptance result.

For a transport-only rehearsal, `pushpak_peer` can stand in for the gateway
and SITL sender. Replace the IP placeholders with travel-router addresses.
Start them in this order from the repository root:

```bash
# Dashboard machine, ID 0
cargo run --release --locked --manifest-path src/pushpak_peer/Cargo.toml -- \
  --drone-id 0 --keyframes src/pushpak_peer/testdata/keyframes.csv \
  --listen tcp/0.0.0.0:7447

# SITL host, ID 1
cargo run --release --locked --manifest-path src/pushpak_peer/Cargo.toml -- \
  --drone-id 1 --keyframes src/pushpak_peer/testdata/keyframes.csv \
  --listen tcp/0.0.0.0:7447 --connect tcp/<DASHBOARD_IP>:7447

# Rig, ID 2, after ROS setup and native build
python3 hitl/zenoh_publisher/ros_adapter.py --ros-args \
  -p drone_id:=2 -p pose_source:=mock \
  -p 'connect_endpoints:=["tcp/<SITL_IP>:7447","tcp/<DASHBOARD_IP>:7447"]'
```

Stop ID 0 using `Ctrl+C`. ID 1 and ID 2 should still report each other's
heartbeats and keyframes. This rehearsal uses a mock SITL sender, so the final
system test should replace it with the actual SITL telemetry producer.

The dashboard marker-under-2-seconds acceptance needs a real detector and
`zenoh_gateway.py`; the current `perception_node.py` in this checkout is a
stub, so that end-to-end result cannot yet be claimed from this code alone.
