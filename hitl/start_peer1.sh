#!/usr/bin/env bash
# Ubuntu SITL scout, peer 1. Does not arm or take off.
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
test -x hitl/zenoh_publisher/target/release/pushpak_hitl_zenoh_publisher
test -f yolov8n.pt

ros2 launch tools/integrated_sitl.launch.py \
  auto_takeoff:=false \
  listen_endpoints:="[\"$PUSHPAK_SCOUT_ENDPOINT\"]" \
  connect_endpoints:="[\"$PUSHPAK_GROUND_ENDPOINT\",\"$PUSHPAK_RIG_ENDPOINT\"]"
