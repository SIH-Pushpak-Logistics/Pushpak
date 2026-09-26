#!/usr/bin/env python3
"""Subscribe to the frozen ROS topics and feed the Rust Zenoh peer over stdin."""

import json
import math
from pathlib import Path
import subprocess
import sys

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
        self.declare_parameter('pose_source', 'mock')
        self.declare_parameter('mock_x_m', 0.0)
        self.declare_parameter('mock_y_m', 0.0)
        self.declare_parameter('mock_z_m', 1.0)
        self.declare_parameter('mock_yaw_deg', 0.0)
        self.declare_parameter('rust_binary', '')
        self.declare_parameter('connect_endpoints', [''])
        self.declare_parameter('listen_endpoints', [''])

        drone_id = self.get_parameter('drone_id').value
        if drone_id != 2:
            raise ValueError('the HITL rig must use drone_id=2')
        self.pose_source = self.get_parameter('pose_source').value
        if self.pose_source not in ('mock', 'odometry'):
            raise ValueError('pose_source must be mock or odometry')
        self.mock = tuple(self.get_parameter(name).value for name in (
            'mock_x_m', 'mock_y_m', 'mock_z_m', 'mock_yaw_deg'))
        if not all(map(math.isfinite, self.mock)):
            raise ValueError('mock pose must be finite')

        binary = self.get_parameter('rust_binary').value
        if not binary:
            binary = str(Path(__file__).resolve().parent / 'target' / 'release' /
                         'pushpak_hitl_zenoh_publisher')
        command = [binary, '--drone-id', '2']
        for endpoint in self.get_parameter('connect_endpoints').value:
            if endpoint:
                command.extend(['--connect', endpoint])
        for endpoint in self.get_parameter('listen_endpoints').value:
            if endpoint:
                command.extend(['--listen', endpoint])
        self.peer = subprocess.Popen(command, stdin=subprocess.PIPE, text=True,
                                     bufsize=1)
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
        self.get_logger().info(f'HITL Zenoh adapter running: pose_source={self.pose_source}')

    def clock_timestamp_ms(self):
        # Heartbeats and mock poses have no source message header.
        return (self.get_clock().now().nanoseconds // 1_000_000) & 0xffffffff

    def send(self, message):
        if self.peer.poll() is not None:
            raise RuntimeError(f'Rust Zenoh peer exited with code {self.peer.returncode}')
        try:
            self.peer.stdin.write(json.dumps(message, separators=(',', ':')) + '\n')
            self.peer.stdin.flush()
        except BrokenPipeError as exc:
            raise RuntimeError('Rust Zenoh peer closed its input') from exc

    def send_heartbeat(self):
        self.send({'kind': 'heartbeat', 'timestamp_ms': self.clock_timestamp_ms(),
                   'status_flags': self.status_flags})

    def on_odometry(self, message):
        self.latest_odom = message

    def send_keyframe(self):
        if self.pose_source == 'odometry':
            if self.latest_odom is None:
                return  # Do not present a mock pose as real odometry.
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
        if self.peer.poll() is None:
            self.peer.stdin.close()
            try:
                self.peer.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.peer.terminate()
                self.peer.wait(timeout=3)
        return super().destroy_node()


def main():
    rclpy.init(args=sys.argv)
    node = None
    try:
        node = RosZenohAdapter()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
