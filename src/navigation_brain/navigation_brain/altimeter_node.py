#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PointStamped
from rclpy.qos import qos_profile_sensor_data
from swarm_utils.redis_bridge import RedisTelemetryPublisher


class AltimeterNode(Node):
    def __init__(self):
        super().__init__('altimeter_node')
        self.declare_parameter('drone_id', 'drone_00')
        self.drone_id = self.get_parameter('drone_id').get_parameter_value().string_value

        self.redis_publisher = RedisTelemetryPublisher(
            stream_name=f'telemetry:{self.drone_id}:altitude',
            logger=self.get_logger()
        )

        self.pub = self.create_publisher(PointStamped, '/drone/altitude', qos_profile_sensor_data)
        self.sub = self.create_subscription(
            LaserScan, '/drone/rangefinder/scan',
            self.scan_cb, qos_profile_sensor_data)
        self.get_logger().info('Altimeter node active: /drone/rangefinder/scan -> /drone/altitude')

    def scan_cb(self, msg: LaserScan):
        if not msg.ranges:
            return
        z = msg.ranges[0]

        if math.isnan(z) or math.isinf(z) or z <= 0.0:
            return
        if z < msg.range_min:
            z = msg.range_min
        elif z > msg.range_max:
            z = msg.range_max

        out = PointStamped()
        out.header = msg.header
        out.point.z = float(z)
        self.pub.publish(out)

        stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.redis_publisher.send_payload(
            self.drone_id, stamp_sec, z=float(z)
        )


def main(args=None):
    rclpy.init(args=args)
    node = AltimeterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
