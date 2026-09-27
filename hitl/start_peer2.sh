#!/usr/bin/env bash
# Jetson tabletop rig, peer 2. This launch contains no arming or actuator node.
set -euo pipefail

root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
source "$root/hitl/config/network.env"
source /opt/ros/humble/setup.bash
source "$root/install/setup.bash"
cd "$root"

if pgrep -x zenohd >/dev/null; then
  echo 'Refusing to start: zenohd is running; Pushpak uses peer mode.' >&2
  exit 1
fi
for path in /dev/pushpak-fc /dev/pushpak-camera /opt/pushpak/camera_info.yaml \
            /opt/pushpak/yolov8n.pt \
            /opt/pushpak/hitl/zenoh_publisher/target/release/pushpak_hitl_zenoh_publisher; do
  if [ ! -e "$path" ]; then echo "Missing required rig input: $path" >&2; exit 1; fi
done
mkdir -p /opt/pushpak/detections

exec ros2 launch "$root/hitl/rig.launch.py"
