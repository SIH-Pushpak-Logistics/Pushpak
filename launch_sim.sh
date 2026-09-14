#!/usr/bin/env bash
# Resilient SITL bring-up. Retries past the gz-sim EachNew sensor race.
set -uo pipefail

LOG="${LOG:-/workspace/sitl_run.log}"
SENTINEL='swarm_drone::base_link::imu_sensor'
BOOT_TIMEOUT="${BOOT_TIMEOUT:-15}"
MAX_ATTEMPTS="${MAX_ATTEMPTS:-6}"

for f in /opt/ros/humble/setup.bash \
         /bridge_ws/install/setup.bash \
         /workspace/install/setup.bash; do
    if [ ! -f "$f" ]; then
        echo "[launch_sim] FATAL: missing setup file: $f" >&2
        exit 3
    fi
done

set +u
. /opt/ros/humble/setup.bash
. /bridge_ws/install/setup.bash
. /workspace/install/setup.bash
set -u

redis-server --daemonize yes >/dev/null 2>&1
if ! redis-cli PING >/dev/null 2>&1; then
    echo "[launch_sim] FATAL: redis-server not responding on 6379." >&2
    exit 3
fi

if ! command -v ros2 >/dev/null 2>&1; then
    echo "[launch_sim] FATAL: ros2 not on PATH after sourcing setup files." >&2
    exit 3
fi

if ! ros2 pkg prefix drone_description >/dev/null 2>&1; then
    echo "[launch_sim] FATAL: package drone_description not found. Run 'colcon build' in /workspace." >&2
    exit 3
fi

cleanup() {
    for p in arducopter "gz sim" parameter_bridge mavros_node robot_state_publisher; do
        pkill -9 -f "$p" 2>/dev/null
    done
    local n=0
    while ss -tlnp 2>/dev/null | grep -q ':5760'; do
        sleep 1; n=$((n+1)); [ "$n" -gt 30 ] && break
    done
}

LAUNCH_PGID=""
on_exit() {
    echo
    echo "[launch_sim] shutting down"
    [ -n "$LAUNCH_PGID" ] && kill -9 -"$LAUNCH_PGID" 2>/dev/null
    cleanup
    exit 130
}
trap on_exit INT TERM

for attempt in $(seq 1 "$MAX_ATTEMPTS"); do
    cleanup
    sleep 2
    : > "$LOG"

    setsid ros2 launch drone_description sitl_bringup.launch.py > "$LOG" 2>&1 &
    LAUNCH_PGID=$!

    ok=0
    reason="sentinel-timeout"
    for ((t=0; t<BOOT_TIMEOUT; t++)); do
        sleep 1
        if grep -aq "$SENTINEL" "$LOG"; then ok=1; break; fi
        if ! kill -0 "$LAUNCH_PGID" 2>/dev/null; then reason="launch-died"; break; fi
    done

    if [ "$ok" -eq 1 ]; then
        echo "[launch_sim] IMU sensor registered on attempt $attempt — simulation live."
        echo "[launch_sim] log: $LOG"
        echo "[launch_sim] Ctrl+C to stop."
        echo "---"
        tail -f "$LOG" &
        TAIL_PID=$!
        wait "$LAUNCH_PGID"
        kill "$TAIL_PID" 2>/dev/null
        cleanup
        exit 0
    fi

    if [ "$reason" = "launch-died" ]; then
        echo "[launch_sim] attempt $attempt: launch process EXITED after ${t}s — NOT the sensor race." >&2
        echo "[launch_sim] tail of $LOG:" >&2
        tail -5 "$LOG" >&2
        kill -9 -"$LAUNCH_PGID" 2>/dev/null
        cleanup
        exit 4
    fi

    echo "[launch_sim] attempt $attempt failed (sentinel absent after ${BOOT_TIMEOUT}s) — retrying"
    kill -9 -"$LAUNCH_PGID" 2>/dev/null
done

echo "[launch_sim] FAILED after $MAX_ATTEMPTS attempts. Inspect $LOG"
exit 1
