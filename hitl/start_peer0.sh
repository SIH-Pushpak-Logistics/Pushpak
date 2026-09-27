#!/usr/bin/env bash
# Ground station: Zenoh peer 0 gateway plus the dashboard.
set -euo pipefail

root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
source "$root/hitl/config/network.env"
cd "$root"

if pgrep -x zenohd >/dev/null; then
  echo 'Refusing to start: zenohd is running; Pushpak uses peer mode.' >&2
  exit 1
fi

python3 -c 'import google.protobuf, websockets, zenoh' 2>/dev/null || {
  echo 'Ground Python dependencies are missing; run the setup commands in hitl/setup_jetson.md.' >&2
  exit 1
}
if [ ! -d dashboard/node_modules ]; then
  echo 'Dashboard dependencies are missing; run npm --prefix dashboard ci before the test.' >&2
  exit 1
fi

python3 tools/zenoh_gateway.py \
  --drone-id 1 \
  --drone-id 2 \
  --listen "$PUSHPAK_GROUND_ENDPOINT" \
  --connect "$PUSHPAK_SCOUT_ENDPOINT" \
  --connect "$PUSHPAK_RIG_ENDPOINT" \
  --host 0.0.0.0 --port "$PUSHPAK_WEBSOCKET_PORT" &
gateway_pid=$!

cleanup() {
  kill "$gateway_pid" 2>/dev/null || true
  wait "$gateway_pid" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "Dashboard: http://$PUSHPAK_GROUND_IP:$PUSHPAK_DASHBOARD_PORT/"
npm --prefix dashboard run dev -- --host 0.0.0.0 --port "$PUSHPAK_DASHBOARD_PORT"
