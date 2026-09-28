#!/usr/bin/env python3
import json
import subprocess
import sys
import tempfile
import threading
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import UInt32MultiArray
from drone_interfaces.msg import SurvivorDetection

from pushpak_brain.telemetry_core import (MAX_PAYLOAD_B, PeerTable, SurvivorTracker, confidence_percent,
                                          millimetres, parse_heartbeat_key, quaternion_rpy_cdeg,
                                          timestamp_ms)

NAN = float('nan')
PARAMS = [
    ('drone_id', -1),
    ('keyframe_rate_hz', NAN),
    ('heartbeat_rate_hz', NAN),
    ('peers_alive_rate_hz', NAN),
    ('peer_timeout_s', NAN),
    ('survivor_dedup_radius_m', NAN),
    ('zenoh_listen', ''),
    ('zenoh_connect', ''),
    ('proto_path', ''),
]
BIT_SURVIVOR_FOUND = 1 << 2
BIT_ISOLATED = 1 << 4


def load_pb(proto_path):
    out = tempfile.mkdtemp(prefix='pushpak_pb_')
    folder, _, name = proto_path.rpartition('/')
    subprocess.run(['protoc', f'-I{folder}', f'--python_out={out}', name], check=True)
    sys.path.insert(0, out)
    import pushpak_pb2
    return pushpak_pb2


def endpoints(text):
    return [e.strip() for e in text.split(',') if e.strip()]


class PushpakTelemetry(Node):
    def __init__(self):
        super().__init__('pushpak_telemetry')
        for name, default in PARAMS:
            self.declare_parameter(name, default)
        p = {name: self.get_parameter(name).value for name, _ in PARAMS}
        if p['drone_id'] < 0:
            raise RuntimeError('drone_id must be set (README section 8)')
        for name in ('keyframe_rate_hz', 'heartbeat_rate_hz', 'peers_alive_rate_hz', 'peer_timeout_s',
                     'survivor_dedup_radius_m'):
            if not (p[name] > 0.0):
                raise RuntimeError(f'{name} must be set in pushpak_params.yaml (I-11), got {p[name]}')
        if not p['proto_path']:
            raise RuntimeError('proto_path must be set in pushpak_params.yaml (I-11)')
        self.get_logger().info('pushpak_telemetry params: ' + ', '.join(f'{k}={v}' for k, v in p.items()))
        self.drone_id = p['drone_id']
        self.pb = load_pb(p['proto_path'])
        self.tracker = SurvivorTracker(p['survivor_dedup_radius_m'])
        self.peers = PeerTable(self.drone_id, p['peer_timeout_s'])
        self.lock = threading.Lock()
        self.odom = None
        self.flags = 0
        self.counts = {'keyframe': 0, 'survivor': 0, 'heartbeat': 0, 'rx_heartbeat': 0, 'rx_rejected': 0,
                       'tx_oversize': 0, 'invalid': 0}
        self.alive_prev = None

        import zenoh
        conf = zenoh.Config()
        conf.insert_json5('mode', '"peer"')
        if endpoints(p['zenoh_listen']):
            conf.insert_json5('listen/endpoints', json.dumps(endpoints(p['zenoh_listen'])))
        if endpoints(p['zenoh_connect']):
            conf.insert_json5('connect/endpoints', json.dumps(endpoints(p['zenoh_connect'])))
        self.session = zenoh.open(conf)
        self.subscriber = self.session.declare_subscriber('pushpak/heartbeat/*', self.on_heartbeat)

        self.create_subscription(Odometry, '/odometry/filtered', self.odom_cb, 10)
        self.create_subscription(SurvivorDetection, '/detections/survivor', self.survivor_cb, 10)
        self.peers_pub = self.create_publisher(UInt32MultiArray, '/pushpak/peers_alive', 10)
        self.create_timer(1.0 / p['keyframe_rate_hz'], self.send_keyframe)
        self.create_timer(1.0 / p['heartbeat_rate_hz'], self.send_heartbeat)
        self.create_timer(1.0 / p['peers_alive_rate_hz'], self.publish_peers)
        self.create_timer(10.0, self.report)
        self.get_logger().info(f'pushpak_telemetry: Zenoh peer {self.drone_id} (no zenohd), '
                               f'listen={endpoints(p["zenoh_listen"])} connect={endpoints(p["zenoh_connect"])}')

    def on_heartbeat(self, sample):
        payload = bytes(sample.payload)
        drone_id = parse_heartbeat_key(str(sample.key_expr))
        ok = drone_id is not None and len(payload) <= MAX_PAYLOAD_B
        if ok:
            try:
                ok = self.pb.Heartbeat.FromString(payload).drone_id == drone_id
            except Exception:
                ok = False
        with self.lock:
            if not ok:
                self.counts['rx_rejected'] += 1
                return
            if drone_id != self.drone_id:
                self.counts['rx_heartbeat'] += 1
                self.peers.heard(drone_id, time.monotonic())

    def put(self, kind, msg):
        data = msg.SerializeToString()
        if len(data) > MAX_PAYLOAD_B:
            self.counts['tx_oversize'] += 1
            return False
        self.session.put(f'pushpak/{kind}/{self.drone_id}', data)
        self.counts[kind] += 1
        return True

    def odom_cb(self, msg):
        self.odom = msg

    def send_keyframe(self):
        m = self.odom
        if m is None:
            return
        pos, q = m.pose.pose.position, m.pose.pose.orientation
        try:
            x, y, z = millimetres(pos.x), millimetres(pos.y), millimetres(pos.z)
            roll, pitch, yaw = quaternion_rpy_cdeg(q.x, q.y, q.z, q.w)
        except ValueError as exc:
            self.counts['invalid'] += 1
            self.get_logger().warning(f'keyframe skipped: {exc}', throttle_duration_sec=5.0)
            return
        self.put('keyframe', self.pb.SubMapKeyframe(
            timestamp_ms=timestamp_ms(m.header.stamp.sec, m.header.stamp.nanosec), drone_id=self.drone_id,
            pos_x_mm=x, pos_y_mm=y, pos_z_mm=z, roll_cdeg=roll, pitch_cdeg=pitch, yaw_cdeg=yaw,
            status_flags=self.flags))

    def survivor_cb(self, msg):
        wp = msg.world_position
        try:
            x, y, z = millimetres(wp.x), millimetres(wp.y), millimetres(wp.z)
            conf = confidence_percent(msg.confidence)
            survivor_id, hits = self.tracker.observe(wp.x, wp.y)
        except ValueError as exc:
            self.counts['invalid'] += 1
            self.get_logger().warning(f'survivor detection ignored: {exc}')
            return
        self.flags |= BIT_SURVIVOR_FOUND
        sent = self.put('survivor', self.pb.SurvivorEvent(
            timestamp_ms=timestamp_ms(msg.header.stamp.sec, msg.header.stamp.nanosec), drone_id=self.drone_id,
            survivor_id=survivor_id, pos_x_mm=x, pos_y_mm=y, pos_z_mm=z, confidence_pct=conf,
            hit_count=hits))
        if sent and hits == 1:
            self.get_logger().info(f'survivor {survivor_id} new at ({wp.x:.2f}, {wp.y:.2f}) conf {conf}%')

    def send_heartbeat(self):
        now = self.get_clock().now().nanoseconds
        self.put('heartbeat', self.pb.Heartbeat(
            timestamp_ms=(now // 1_000_000) & 0xFFFFFFFF, drone_id=self.drone_id, status_flags=self.flags))

    def publish_peers(self):
        with self.lock:
            alive = self.peers.alive(time.monotonic())
        self.flags = (self.flags & ~BIT_ISOLATED) | (0 if alive else BIT_ISOLATED)
        self.peers_pub.publish(UInt32MultiArray(data=alive))
        if alive != self.alive_prev:
            self.get_logger().info(f'peers alive: {alive}')
            self.alive_prev = alive

    def report(self):
        with self.lock:
            counts = dict(self.counts)
        self.get_logger().info(f'telemetry counts {counts}')

    def destroy_node(self):
        try:
            self.session.close()
        except Exception:
            pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PushpakTelemetry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
