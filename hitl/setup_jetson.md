# Peer 2 Jetson tabletop rig setup

This runbook prepares the physical, **unarmed** peer 2 rig. The final launch starts
MAVROS telemetry, a USB camera, ToF normalization, the 15-state EKF, perception and
the Zenoh publisher. It does not start `pushpak_brain`, publish actuator setpoints,
arm the flight controller or spin motors.

## 1. Fixed three-machine network

Use an isolated travel router on `192.168.8.0/24`. Reserve these addresses in the
router as well as setting them on each host:

| Peer | Host | Address | Services |
|---|---|---|---|
| 0 | ground Mac/laptop | `192.168.8.10` | Zenoh `7447`, WebSocket `8765`, dashboard `5173` |
| 1 | Ubuntu SITL scout | `192.168.8.11` | Zenoh `7447` |
| 2 | Jetson tabletop rig | `192.168.8.12` | Zenoh `7447` |
| - | travel router | `192.168.8.1` | DHCP/DNS only |

The canonical values are in `hitl/config/network.env`. Do not run `zenohd`; all
three processes open Zenoh in peer mode.

On each Ubuntu host, replace `Pushpak LAN` with the NetworkManager connection name:

```bash
nmcli connection show
sudo nmcli connection modify "Pushpak LAN" \
  ipv4.method manual ipv4.addresses 192.168.8.11/24 \
  ipv4.gateway 192.168.8.1 ipv4.dns 192.168.8.1
sudo nmcli connection up "Pushpak LAN"
```

Use `192.168.8.12/24` on the Jetson. On a macOS ground station:

```bash
networksetup -listallnetworkservices
sudo networksetup -setmanual "Wi-Fi" 192.168.8.10 255.255.255.0 192.168.8.1
sudo networksetup -setdnsservers "Wi-Fi" 192.168.8.1
```

Allow inbound TCP `7447`, `8765`, and `5173` on peer 0, and TCP `7447` on peers 1
and 2. If Ubuntu's firewall is enabled:

```bash
sudo ufw allow from 192.168.8.0/24 to any port 7447 proto tcp
```

## 2. Jetson software (once)

Use Ubuntu 22.04/JetPack with ROS 2 Humble. Install the JetPack build that supports
the Jetson model, then install its matching NVIDIA PyTorch wheel. Do not replace
that wheel with the x86 CUDA wheel pinned by the project Docker image.

```bash
sudo apt update
sudo apt install -y git curl build-essential python3-pip python3-rosdep \
  python3-colcon-common-extensions ros-humble-mavros ros-humble-mavros-extras \
  ros-humble-robot-localization ros-humble-v4l2-camera \
  ros-humble-camera-calibration ros-humble-cv-bridge ros-humble-message-filters
sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh

sudo mkdir -p /opt/pushpak
sudo chown "$USER":"$USER" /opt/pushpak
git clone https://github.com/SIH-Pushpak-Logistics/Pushpak.git /opt/pushpak
cd /opt/pushpak
git fetch origin
git switch --track origin/arch/v2-degraded-estimation

python3 -m pip install --user 'numpy<2' 'protobuf==3.20.3' \
  'ultralytics==8.4.155' 'eclipse-zenoh==1.0.0' websockets
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
source "$HOME/.cargo/env"

source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
cargo build --manifest-path hitl/zenoh_publisher/Cargo.toml --release
mkdir -p /opt/pushpak/detections
```

Copy the approved YOLO weights to `/opt/pushpak/yolov8n.pt`. The filename and model
must match the validation model; the launch refuses to proceed without the file.

## 3. Stable USB device names (once per physical unit)

Connect the flight controller and camera, then identify their immutable serial or
USB path. Never write a rule using only `ttyACM0` or `video0`.

```bash
udevadm info --query=property --name=/dev/ttyACM0 | sort
udevadm info --query=property --name=/dev/video0 | sort
```

Create `/etc/udev/rules.d/99-pushpak.rules`, replacing the two placeholder serials
with the reported `ID_SERIAL_SHORT` values:

```udev
SUBSYSTEM=="tty", ENV{ID_SERIAL_SHORT}=="FC_SERIAL_HERE", SYMLINK+="pushpak-fc", GROUP="dialout", MODE="0660"
SUBSYSTEM=="video4linux", ENV{ID_SERIAL_SHORT}=="CAMERA_SERIAL_HERE", ATTR{index}=="0", SYMLINK+="pushpak-camera", GROUP="video", MODE="0660"
```

Apply the rules and give the login user device access:

```bash
sudo usermod -aG dialout,video "$USER"
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Log out and back in, then verify both links with `readlink -f /dev/pushpak-fc` and
`readlink -f /dev/pushpak-camera`.

## 4. Camera and rangefinder preparation

Camera intrinsics are unit-specific and cannot be safely invented before the camera
arrives. Perform the standard checkerboard calibration once, then keep the resulting
file at the path used by `rig_params.yaml`:

```bash
source /opt/ros/humble/setup.bash
ros2 run v4l2_camera v4l2_camera_node --ros-args \
  -r __ns:=/camera -p video_device:=/dev/pushpak-camera \
  -p image_size:="[640,480]" -p pixel_format:=YUYV &
ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.025 \
  image:=/camera/image_raw camera:=/camera
# In the calibration window: CALIBRATE, SAVE, then COMMIT.
tar -xzf /tmp/calibrationdata.tar.gz -C /tmp
sudo cp /tmp/ost.yaml /opt/pushpak/camera_info.yaml
```

Set the flight controller's downward rangefinder so ArduPilot emits MAVLink
`DISTANCE_SENSOR`. The default rig launch reads MAVROS topic
`/mavros/distance_sensor/rangefinder_pub`. If an independent ROS driver publishes
`sensor_msgs/Range`, pass its topic at launch time, for example:

```bash
ros2 launch /opt/pushpak/hitl/rig.launch.py tof_topic:=/vl53l1x/range
```

Before continuing, a direct topic check must show changing finite metres within the
sensor limits:

```bash
ros2 topic echo --once /mavros/distance_sensor/rangefinder_pub sensor_msgs/msg/Range
```

## 5. Preflight on all three machines

Peer 0 also needs Python 3, Node.js 22 and npm. Install its dependencies while an
internet connection is available, before moving to the isolated test LAN:

```bash
cd /path/to/Pushpak
python3 -m pip install -r tools/requirements-gateway.txt
npm --prefix dashboard ci
```

Peer 1 needs the repository's normal ROS/SITL dependencies, approved YOLO weights at
`/opt/pushpak/yolov8n.pt`, a completed `colcon build`, and the release Rust publisher:

```bash
cd /opt/pushpak
source /opt/ros/humble/setup.bash
colcon build --symlink-install
cargo build --manifest-path hitl/zenoh_publisher/Cargo.toml --release
```

Keep props removed and the flight controller unarmed. All machines must report the
same Git commit. Run these concurrently and retain the generated files:

**Peer 0 (ground):**

```bash
cd /path/to/Pushpak
bash tools/capture_three_peer_evidence.sh ground 192.168.8.11 192.168.8.12 ~/pushpak-evidence
```

**Peer 1 (Ubuntu SITL):**

```bash
cd /opt/pushpak
bash tools/capture_three_peer_evidence.sh scout 192.168.8.10 192.168.8.12 ~/pushpak-evidence
```

**Peer 2 (Jetson rig):**

```bash
cd /opt/pushpak
bash tools/capture_three_peer_evidence.sh rig 192.168.8.10 192.168.8.11 ~/pushpak-evidence
```

Each file must end with `PREFLIGHT=PASS`. This captures both ping directions and the
raw `pgrep zenohd` result required by the HITL evidence record.

## 6. Start peers 0, 1 and 2

Open one terminal on each machine and start them in this order. The scripts reject a
running Zenoh router. Peer 1 explicitly sets `auto_takeoff:=false`; peer 2 has no
arming or actuator node.

```bash
# Ground Mac/laptop, peer 0
cd /path/to/Pushpak && bash hitl/start_peer0.sh

# Ubuntu SITL scout, peer 1
cd /opt/pushpak && bash hitl/start_peer1.sh

# Jetson tabletop rig, peer 2
cd /opt/pushpak && bash hitl/start_peer2.sh
```

Open `http://192.168.8.10:5173/`. Confirm both scout 1 and rig 2 move, stop peer 1
and confirm only scout changes to offline, restart peer 1 and confirm recovery, then
present a test survivor to the rig camera and confirm the survivor event appears.

## 7. Rig acceptance commands

Run these on peer 2 while the launch is active:

```bash
ros2 topic hz /imu/raw
ros2 topic hz /camera/image_raw
ros2 topic echo --once /camera/camera_info sensor_msgs/msg/CameraInfo
ros2 topic hz /drone/tof_range
ros2 topic hz /odometry/filtered
ros2 topic echo --once /detections/survivor drone_interfaces/msg/SurvivorDetection
ros2 node list
```

`/camera/camera_info` must contain nonzero `k[0]` and `k[4]`. The ToF and odometry
timestamps must advance. A survivor message is expected only when the confidence,
blur, altitude and pose freshness gates all pass.

Stop each peer with `Ctrl-C`. Peer 2 remains physically incapable of mission flight
under this launch because the guidance and takeoff nodes are absent.
