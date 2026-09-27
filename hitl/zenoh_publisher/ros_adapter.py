#!/usr/bin/env python3
"""Subscribe to the frozen ROS topics and feed the Rust Zenoh peer over stdin."""

import json
import math
from pathlib import Path
import queue
import subprocess
import sys
import threading

import rclpy
from rclpy.node import Node
from drone_interfaces.msg import SurvivorDetection
from nav_msgs.msg import Odometry

from conversion import (
    SurvivorTracker, centidegrees, confidence_percent, millimetres,
    quaternion_rpy_cdeg, timestamp_ms_from_stamp,
)


class RosZenohAdapter(Node):
    def __init__(self):
        super().__init__('hitl_zenoh_publisher')
        self.declare_parameter('drone_id', 2)
        self.declare_parameter('pose_source')
        self.declare_parameter('max_odom_age_s', 0.5)
        self.declare_parameter('mock_x_m', 0.0)
        self.declare_parameter('mock_y_m', 0.0)
        self.declare_parameter('mock_z_m', 1.0)
        self.declare_parameter('mock_yaw_deg', 0.0)
        self.declare_parameter('rust_binary', '')
        self.declare_parameter('connect_endpoints', [''])
        self.declare_parameter('listen_endpoints', [''])

        drone_id = self.get_parameter('drone_id').value
        if drone_id not in (1, 2):
            raise ValueError('ROS telemetry requires scout drone_id=1 or rig drone_id=2')
        self.max_odom_age_s = self.get_parameter('max_odom_age_s').value
        if not math.isfinite(self.max_odom_age_s) or self.max_odom_age_s <= 0:
            raise ValueError('max_odom_age_s must be finite and positive')
        self.pose_source = self.get_parameter('pose_source').value
        if self.pose_source not in ('mock', 'odometry'):
            raise ValueError('pose_source is required: choose mock or odometry')
        self.mock = tuple(self.get_parameter(name).value for name in (
            'mock_x_m', 'mock_y_m', 'mock_z_m', 'mock_yaw_deg'))
        if not all(map(math.isfinite, self.mock)):
            raise ValueError('mock pose must be finite')

        binary = self.get_parameter('rust_binary').value
        if not binary:
            binary = str(Path(__file__).resolve().parent / 'target' / 'release' /
                         'pushpak_hitl_zenoh_publisher')
        command = [binary, '--drone-id', str(drone_id)]
        for endpoint in self.get_parameter('connect_endpoints').value:
            if endpoint:
                command.extend(['--connect', endpoint])
        for endpoint in self.get_parameter('listen_endpoints').value:
            if endpoint:
                command.extend(['--listen', endpoint])
        self.peer = subprocess.Popen(command, stdin=subprocess.PIPE, text=True,
                                     bufsize=1)
        self.outbox = queue.Queue(maxsize=128)
        self.writer_stop = threading.Event()
        self.writer_failed = threading.Event()
        self.dropped_messages = 0
        self.writer = threading.Thread(target=self._write_messages, daemon=True)
        self.writer.start()
        self.latest_odom = None
        self.tracker = SurvivorTracker()
        self.status_flags = 0
        self.create_subscription(SurvivorDetection, '/detections/survivor',
                                 self.on_detection, 10)
        if self.pose_source == 'odometry':
            self.create_subscription(Odometry, '/odometry/filtered',
                                     self.on_odometry, 10)
        self.create_timer(0.5, self.send_heartbeat)
        self.create_timer(0.2, self.send_keyframe)
        self.create_timer(0.2, self.check_writer)
        if self.pose_source == 'mock':
            self.create_timer(5.0, self.warn_mock)
            self.warn_mock()
        self.get_logger().info(f'HITL Zenoh adapter running: pose_source={self.pose_source}')

    def clock_timestamp_ms(self):
        # Heartbeats and mock poses have no source message header.
        return (self.get_clock().now().nanoseconds // 1_000_000) & 0xffffffff

    def send(self, message):
        self.check_writer()
        try:
            self.outbox.put_nowait(json.dumps(message, separators=(',', ':')) + '\n')
        except queue.Full:
            self.dropped_messages += 1
            if self.dropped_messages == 1 or self.dropped_messages % 100 == 0:
                self.get_logger().warning(
                    f'Rust Zenoh pipe backlog full; dropped {self.dropped_messages} messages')

    def _write_messages(self):
        while not self.writer_stop.is_set():
            if self.peer.poll() is not None:
                self._writer_failure(f'exited with code {self.peer.returncode}')
                return
            try:
                line = self.outbox.get(timeout=0.1)
            except queue.Empty:
                continue
            try:
                self.peer.stdin.write(line)
                self.peer.stdin.flush()
            except (BrokenPipeError, OSError, ValueError):
                self._writer_failure('closed its input')
                return

    def _writer_failure(self, message):
        if not self.writer_stop.is_set() and not self.writer_failed.is_set():
            self.get_logger().error(f'Rust Zenoh peer stopped: {message}')
            self.writer_failed.set()

    def check_writer(self):
        if self.writer_failed.is_set():
            raise RuntimeError('Rust Zenoh peer stopped; terminating adapter')

    def warn_mock(self):
        self.get_logger().warning(
            'MOCK POSE ACTIVE: publishing invented position as telemetry; '
            'use pose_source:=odometry for real flight')

    def send_heartbeat(self):
        self.send({'kind': 'heartbeat', 'timestamp_ms': self.clock_timestamp_ms(),
                   'status_flags': self.status_flags})

    def on_odometry(self, message):
        self.latest_odom = message

    def send_keyframe(self):
        if self.pose_source == 'odometry':
            if self.latest_odom is None:
                return  # Do not present a mock pose as real odometry.
            header = self.latest_odom.header
            stamp_ns = header.stamp.sec * 1_000_000_000 + header.stamp.nanosec
            age = (self.get_clock().now().nanoseconds - stamp_ns) * 1e-9
            if header.frame_id != 'odom' or not 0 <= age <= self.max_odom_age_s:
                return  # Frozen wire format has no frame or freshness metadata.
            timestamp_ms = timestamp_ms_from_stamp(self.latest_odom.header.stamp)
            pose = self.latest_odom.pose.pose
            xyz = (pose.position.x, pose.position.y, pose.position.z)
            q = pose.orientation
            try:
                roll, pitch, yaw = quaternion_rpy_cdeg((q.x, q.y, q.z, q.w))
            except ValueError as exc:
                self.get_logger().warning(str(exc))
                return
        else:
            timestamp_ms = self.clock_timestamp_ms()
            xyz = self.mock[:3]
            roll, pitch, yaw = 0, 0, round(self.mock[3] * 100)
        try:
            x, y, z = map(millimetres, xyz)
        except ValueError as exc:
            self.get_logger().warning(str(exc))
            return
        self.send({'kind': 'keyframe', 'timestamp_ms': timestamp_ms,
                   'pos_x_mm': x, 'pos_y_mm': y, 'pos_z_mm': z,
                   'roll_cdeg': roll, 'pitch_cdeg': pitch, 'yaw_cdeg': yaw,
                   'status_flags': self.status_flags})

    def on_detection(self, message):
        if message.header.frame_id != 'odom':
            self.get_logger().warning('ignored survivor detection outside odom frame')
            return
        position = message.world_position
        xyz = (position.x, position.y, position.z)
        try:
            x, y, z = map(millimetres, xyz)
            confidence = confidence_percent(message.confidence)
            survivor_id, hit_count = self.tracker.observe(xyz)
        except ValueError as exc:
            self.get_logger().warning(f'ignored invalid survivor detection: {exc}')
            return
        timestamp_ms = timestamp_ms_from_stamp(message.header.stamp)
        self.status_flags |= 1 << 2  # Survivor_Found, per README §8.
        self.send({'kind': 'survivor', 'timestamp_ms': timestamp_ms,
                   'survivor_id': survivor_id, 'pos_x_mm': x, 'pos_y_mm': y,
                   'pos_z_mm': z, 'confidence_pct': confidence,
                   'hit_count': hit_count})

    def destroy_node(self):
        self.writer_stop.set()
        if self.peer.poll() is None:
            self.peer.terminate()
        try:
            self.peer.wait(timeout=3)
        except subprocess.TimeoutExpired:
            self.peer.kill()
            self.peer.wait(timeout=3)
        self.writer.join(timeout=3)
        try:
            self.peer.stdin.close()
        except (BrokenPipeError, OSError, ValueError):
            pass  # A dead child may already have closed the pipe.
        return super().destroy_node()


def main():
    rclpy.init(args=sys.argv)
    node = None
    try:
        node = RosZenohAdapter()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError as exc:
        if node is not None and not node.writer_failed.is_set():
            node.get_logger().error(str(exc))
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
