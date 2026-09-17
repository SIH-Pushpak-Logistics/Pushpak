#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

from swarm_utils.redis_bridge import RedisTelemetryPublisher


class PosePublisherNode(Node):
    def __init__(self):
        super().__init__('pose_publisher_node')

        self.declare_parameter('drone_id', 'drone_00')
        self.declare_parameter('publish_rate_hz', 10.0)
        self.drone_id = self.get_parameter('drone_id').get_parameter_value().string_value
        rate = self.get_parameter('publish_rate_hz').get_parameter_value().double_value

        self.redis_publisher = RedisTelemetryPublisher(
            stream_name=f'telemetry:{self.drone_id}:pose',
            logger=self.get_logger()
        )

        self.latest = None
        self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self.pose_cb, 10)
        self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info(
            f'pose_publisher active at {rate:.1f} Hz -> telemetry:{self.drone_id}:pose')

    def pose_cb(self, msg: PoseStamped):
        self.latest = msg

    def tick(self):
        if self.latest is None:
            return
        p = self.latest.pose.position
        q = self.latest.pose.orientation
        yaw = math.degrees(math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
        stamp = (self.latest.header.stamp.sec
                 + self.latest.header.stamp.nanosec * 1e-9)
        self.redis_publisher.send_payload(
            self.drone_id, stamp,
            x=float(p.x), y=float(p.y), z=float(p.z), yaw_deg=float(yaw))


def main(args=None):
    rclpy.init(args=args)
    node = PosePublisherNode()
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