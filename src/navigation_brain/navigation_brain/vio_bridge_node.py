#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TwistWithCovarianceStamped


class VioBridgeNode(Node):
    def __init__(self):
        super().__init__('vio_bridge_node')
        rate_hz = self.declare_parameter('rate_hz', 30.0).value
        self.max_age_s = self.declare_parameter('max_age_s', 0.1).value
        self.latest = None
        self.create_subscription(Odometry, '/odometry/filtered', self.odom_cb, 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/mavros/vision_pose/pose', 10)
        self.twist_pub = self.create_publisher(
            TwistWithCovarianceStamped, '/mavros/vision_speed/speed_twist_cov', 10)
        self.create_timer(1.0 / rate_hz, self.tick)
        self.get_logger().info(
            f'vio_bridge: /odometry/filtered -> ExtNav at {rate_hz} Hz, max_age_s={self.max_age_s}')

    def odom_cb(self, msg):
        self.latest = msg

    def tick(self):
        odom = self.latest
        if odom is None:
            return
        age = (self.get_clock().now() - Time.from_msg(odom.header.stamp)).nanoseconds * 1e-9
        if age < 0.0 or age > self.max_age_s:
            return

        pose = PoseStamped()
        pose.header.stamp = odom.header.stamp
        pose.header.frame_id = odom.header.frame_id
        pose.pose = odom.pose.pose
        self.pose_pub.publish(pose)

        q = odom.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        c, s = math.cos(yaw), math.sin(yaw)
        vb = odom.twist.twist.linear

        twist = TwistWithCovarianceStamped()
        twist.header.stamp = odom.header.stamp
        twist.header.frame_id = odom.header.frame_id
        twist.twist.twist.linear.x = c * vb.x - s * vb.y
        twist.twist.twist.linear.y = s * vb.x + c * vb.y
        twist.twist.twist.linear.z = vb.z

        rot = ((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0))
        cb = odom.twist.covariance
        cov = [0.0] * 36
        for i in range(3):
            for j in range(3):
                cov[i * 6 + j] = sum(rot[i][k] * cb[k * 6 + l] * rot[j][l]
                                     for k in range(3) for l in range(3))
        twist.twist.covariance = cov
        self.twist_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = VioBridgeNode()
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
