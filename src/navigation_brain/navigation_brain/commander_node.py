#!/usr/bin/env python3
import json
import math

import redis
import rclpy
from rclpy.node import Node

from swarm_utils.redis_bridge import RedisTelemetrySubscriber


class CommanderNode(Node):
    def __init__(self):
        super().__init__('commander_node')

        self.declare_parameter('drone_id', 'drone_00')
        self.declare_parameter('cluster_radius_m', 1.5)
        self.declare_parameter('image_width', 640)
        self.declare_parameter('hfov_rad', 1.047)
        self.declare_parameter('rate_hz', 5.0)
        self.drone_id = self.get_parameter('drone_id').get_parameter_value().string_value
        self.cluster_radius = self.get_parameter('cluster_radius_m').get_parameter_value().double_value
        self.img_w = self.get_parameter('image_width').get_parameter_value().integer_value
        self.hfov = self.get_parameter('hfov_rad').get_parameter_value().double_value
        rate = self.get_parameter('rate_hz').get_parameter_value().double_value

        self.fx = float(self.img_w) / (2.0 * math.tan(self.hfov / 2.0))

        self.det_stream = f'detections:{self.drone_id}'
        self.pose_stream = f'telemetry:{self.drone_id}:pose'
        self.victims_key = f'victims:{self.drone_id}'

        self.redis_sub = RedisTelemetrySubscriber(
            streams=[self.det_stream, self.pose_stream],
            logger=self.get_logger()
        )
        self.client = redis.Redis(
            host='localhost', port=6379, decode_responses=True,
            socket_timeout=0.2, socket_connect_timeout=0.5)

        self.victims = {}
        self.seen_det_ids = set()
        self.next_id = 1
        self.yaw_deg = 0.0

        self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info(
            f'commander active: {self.det_stream} -> HASH {self.victims_key} '
            f'(cluster {self.cluster_radius} m, fx {self.fx:.1f})')

    def ground_offset(self, bbox_cx, bbox_cy, altitude):
        img_h = self.img_w * 3.0 / 4.0
        du = bbox_cx - self.img_w / 2.0
        dv = bbox_cy - img_h / 2.0
        fwd = dv * altitude / self.fx
        lat = du * altitude / self.fx
        yaw = math.radians(self.yaw_deg)
        east = fwd * math.cos(yaw) - lat * math.sin(yaw)
        north = fwd * math.sin(yaw) + lat * math.cos(yaw)
        return east, north

    def tick(self):
        pose = self.redis_sub.get_latest(self.pose_stream)
        if pose:
            try:
                self.yaw_deg = float(pose.get('yaw_deg', self.yaw_deg))
            except (TypeError, ValueError):
                pass

        det = self.redis_sub.get_latest(self.det_stream)
        if not det:
            return
        det_id = det.get('det_id')
        if not det_id or det_id in self.seen_det_ids:
            return
        self.seen_det_ids.add(det_id)

        try:
            t = float(det.get('timestamp', 0.0))
            conf = float(det.get('confidence', 0.0))
            bx, by = float(det.get('bbox_x', 0.0)), float(det.get('bbox_y', 0.0))
            bw, bh = float(det.get('bbox_w', 0.0)), float(det.get('bbox_h', 0.0))
            dx, dy = float(det.get('drone_x', 0.0)), float(det.get('drone_y', 0.0))
            dz = float(det.get('drone_z', 0.0))
        except (TypeError, ValueError) as exc:
            self.get_logger().warn(f'malformed detection {det_id}: {exc}')
            return

        de, dn = self.ground_offset(bx + bw / 2.0, by + bh / 2.0, max(dz, 0.2))
        wx, wy = dx + de, dy + dn

        match = None
        for vid, v in self.victims.items():
            if math.hypot(v['world_x'] - wx, v['world_y'] - wy) <= self.cluster_radius:
                match = vid
                break

        if match is None:
            vid = f'v_{self.next_id:03d}'
            self.next_id += 1
            self.victims[vid] = {
                'victim_id': vid, 'confidence': round(conf, 4),
                'world_x': round(wx, 3), 'world_y': round(wy, 3),
                'first_seen': round(t, 3), 'last_seen': round(t, 3),
                'hit_count': 1,
                'image_path': det.get('image_path', ''),
                'acked': False,
            }
            self.get_logger().info(
                f'NEW VICTIM {vid} at ({wx:.2f}, {wy:.2f}) conf {conf:.2f}')
        else:
            v = self.victims[match]
            n = v['hit_count']
            v['world_x'] = round((v['world_x'] * n + wx) / (n + 1), 3)
            v['world_y'] = round((v['world_y'] * n + wy) / (n + 1), 3)
            v['hit_count'] = n + 1
            v['last_seen'] = round(t, 3)
            if conf > v['confidence']:
                v['confidence'] = round(conf, 4)
                v['image_path'] = det.get('image_path', v['image_path'])
            vid = match

        try:
            existing = self.client.hget(self.victims_key, vid)
            if existing:
                prev = json.loads(existing)
                self.victims[vid]['acked'] = prev.get('acked', False)
            self.client.hset(self.victims_key, vid, json.dumps(self.victims[vid]))
        except (redis.RedisError, ValueError) as exc:
            self.get_logger().warn(f'victim write failed: {exc}')


def main(args=None):
    rclpy.init(args=args)
    node = CommanderNode()
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