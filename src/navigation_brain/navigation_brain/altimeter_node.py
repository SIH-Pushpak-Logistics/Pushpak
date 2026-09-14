#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PointStamped
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
            PoseStamped, '/mavros/local_position/pose',
            self.pose_cb, qos_profile_sensor_data)
        self.get_logger().info('Altimeter node up: /mavros/local_position/pose -> /drone/altitude')

    def pose_cb(self, msg):
        out = PointStamped()
        out.header = msg.header
        out.point.z = msg.pose.position.z
        self.pub.publish(out)

        stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.redis_publisher.send_payload(
            self.drone_id, stamp_sec, z=float(msg.pose.position.z)
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