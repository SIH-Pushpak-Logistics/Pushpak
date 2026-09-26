#!/usr/bin/env python3
"""Exercise fake ROS callbacks through the real Rust peer over loopback TCP.

Build the two debug binaries first. This needs Python and Rust, but no ROS,
Jetson, camera, or zenohd process.
"""

import os
import math
from pathlib import Path
import queue
import socket
import subprocess
import sys
import threading
import time
from types import ModuleType, SimpleNamespace


ROOT = Path(__file__).resolve().parents[2]
SUFFIX = '.exe' if os.name == 'nt' else ''
RIG_BINARY = ROOT / 'hitl/zenoh_publisher/target/debug' / ('pushpak_hitl_zenoh_publisher' + SUFFIX)
PEER_BINARY = ROOT / 'src/pushpak_peer/target/debug' / ('pushpak_peer' + SUFFIX)
KEYFRAMES = ROOT / 'src/pushpak_peer/testdata/keyframes.csv'


class FakeNode:
    overrides = {}

    def __init__(self, name):
        self.parameters = {}

    def declare_parameter(self, name, default):
        self.parameters[name] = self.overrides.get(name, default)

    def get_parameter(self, name):
        return SimpleNamespace(value=self.parameters[name])

    def create_subscription(self, message_type, topic, callback, qos):
        return callback

    def create_timer(self, period, callback):
        return callback

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=time.time_ns()))

    def get_logger(self):
        return SimpleNamespace(info=lambda message: None, warning=lambda message: print(message))

    def destroy_node(self):
        return None


def install_fake_ros():
    rclpy = ModuleType('rclpy')
    node = ModuleType('rclpy.node')
    node.Node = FakeNode
    interfaces = ModuleType('drone_interfaces')
    interfaces_msg = ModuleType('drone_interfaces.msg')
    interfaces_msg.SurvivorDetection = type('SurvivorDetection', (), {})
    nav_msgs = ModuleType('nav_msgs')
    nav_msgs_msg = ModuleType('nav_msgs.msg')
    nav_msgs_msg.Odometry = type('Odometry', (), {})
    sys.modules.update({
        'rclpy': rclpy,
        'rclpy.node': node,
        'drone_interfaces': interfaces,
        'drone_interfaces.msg': interfaces_msg,
        'nav_msgs': nav_msgs,
        'nav_msgs.msg': nav_msgs_msg,
    })


def stream_lines(process, output):
    for line in process.stdout:
        output.put(line.rstrip())


def wait_for(output, expected, timeout=10):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            line = output.get(timeout=min(0.5, max(0.01, deadline - time.monotonic())))
        except queue.Empty:
            continue
        print(line)
        if expected in line:
            return time.monotonic()
    raise TimeoutError(f'did not receive {expected!r} within {timeout}s')


def free_port():
    with socket.socket() as listener:
        listener.bind(('127.0.0.1', 0))
        return listener.getsockname()[1]


def main():
    for binary in (RIG_BINARY, PEER_BINARY):
        if not binary.is_file():
            raise SystemExit(f'build the debug binary first: {binary}')

    if os.name == 'nt':
        zenohd = subprocess.run(
            ['powershell', '-NoProfile', '-Command',
             'Get-Process zenohd -ErrorAction SilentlyContinue'],
            capture_output=True, text=True, check=False,
        )
    else:
        zenohd = subprocess.run(['pgrep', '-a', 'zenohd'], capture_output=True, text=True)
    if zenohd.stdout.strip():
        raise SystemExit('zenohd is running; stop it before this routerless test')

    endpoint = f'tcp/127.0.0.1:{free_port()}'
    peer = subprocess.Popen(
        [str(PEER_BINARY), '--drone-id', '1', '--keyframes', str(KEYFRAMES),
         '--listen', endpoint], cwd=ROOT, stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT, text=True, bufsize=1,
    )
    output = queue.Queue()
    threading.Thread(target=stream_lines, args=(peer, output), daemon=True).start()
    adapter = None
    ground = None
    timer_stop = threading.Event()
    timer_thread = None
    send_lock = threading.Lock()
    try:
        wait_for(output, 'pushpak_peer 1 ready')
        install_fake_ros()
        from ros_adapter import RosZenohAdapter

        FakeNode.overrides = {
            'rust_binary': str(RIG_BINARY),
            'connect_endpoints': [endpoint],
        }
        adapter = RosZenohAdapter()

        def run_fake_timers():
            next_heartbeat = time.monotonic()
            while not timer_stop.wait(0.2):
                with send_lock:
                    adapter.send_keyframe()
                    if time.monotonic() >= next_heartbeat:
                        adapter.send_heartbeat()
                        next_heartbeat = time.monotonic() + 0.5

        timer_thread = threading.Thread(target=run_fake_timers, daemon=True)
        timer_thread.start()
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            try:
                wait_for(output, 'HEARTBEAT peer=2', timeout=0.5)
                break
            except TimeoutError:
                continue
        else:
            raise TimeoutError('the peer did not receive the adapter heartbeat')

        wait_for(output, 'KEYFRAME peer=2')
        detection = SimpleNamespace(
            world_position=SimpleNamespace(x=1.2, y=-0.3, z=1.1),
            confidence=0.87,
            header=SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=0)),
        )
        with send_lock:
            adapter.on_detection(detection)
        wait_for(output, 'SURVIVOR peer=2 id=1 confidence=87pct')
        print('PASS: fake ROS detection -> adapter -> Rust protobuf -> Zenoh -> peer')

        odometry = SimpleNamespace(pose=SimpleNamespace(pose=SimpleNamespace(
            position=SimpleNamespace(x=2.0, y=-0.5, z=1.5),
            orientation=SimpleNamespace(x=0.0, y=0.0,
                                        z=math.sin(math.pi / 4),
                                        w=math.cos(math.pi / 4)),
        )))
        with send_lock:
            adapter.pose_source = 'odometry'
            adapter.on_odometry(odometry)
        wait_for(output, 'KEYFRAME peer=2', timeout=2)  # Drain any earlier mock frame.
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            try:
                line = output.get(timeout=0.5)
            except queue.Empty:
                continue
            print(line)
            if 'KEYFRAME peer=2' in line and 'pos=(2000,-500,1500)' in line:
                break
        else:
            raise TimeoutError('odometry pose did not reach the peer')
        with send_lock:
            adapter.pose_source = 'mock'
        print('PASS: odometry launch mode sends the filtered pose')

        ground = subprocess.Popen(
            [str(PEER_BINARY), '--drone-id', '0', '--keyframes', str(KEYFRAMES),
             '--connect', endpoint], cwd=ROOT,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )
        wait_for(output, 'HEARTBEAT peer=0')
        stopped_at = time.monotonic()
        ground.terminate()
        ground.wait(timeout=5)
        lost_at = wait_for(output, 'PEER LOST 0', timeout=2)
        if lost_at - stopped_at >= 2:
            raise TimeoutError('ground peer loss was not reported within 2s')
        wait_for(output, 'HEARTBEAT peer=2', timeout=2)
        print('PASS: rig and SITL traffic continues after ground peer 0 stops')
    finally:
        timer_stop.set()
        if timer_thread is not None:
            timer_thread.join(timeout=2)
        if adapter is not None:
            adapter.destroy_node()
        if ground is not None and ground.poll() is None:
            ground.terminate()
            ground.wait(timeout=5)
        if peer.poll() is None:
            peer.terminate()
        peer.wait(timeout=5)


if __name__ == '__main__':
    main()
