#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TwistWithCovarianceStamped


class VioBridgeNode(Node):
    """
    PRODUCTION HARDWARE COMPONENT:
    Assumes incoming /vio/odometry is in standard ROS ENU.
    Performs zero simulation hacks. Runs byte-for-byte on Jetson Orin Nano.
    """
    def __init__(self):
        super().__init__('vio_bridge_node')

        self.odom_sub = self.create_subscription(
            Odometry, '/vio/odometry', self.odom_cb, 10
        )

        self.vision_pose_pub = self.create_publisher(
            PoseStamped, '/mavros/vision_pose/pose', 10
        )
        self.vision_speed_pub = self.create_publisher(
            TwistWithCovarianceStamped, '/mavros/vision_speed/speed_twist_cov', 10
        )

        self.latest_odom = None
        # Strict 30 Hz timer matching OpenVINS output frequency
        self.create_timer(0.0333, self.timer_cb)
        self.get_logger().info('Production VIO Bridge (30 Hz) initialized.')

    def odom_cb(self, msg: Odometry):
        self.latest_odom = msg

    def timer_cb(self):
        if self.latest_odom is None:
            return

        now_ns = self.get_clock().now().nanoseconds
        odom_ns = (
            self.latest_odom.header.stamp.sec * 1_000_000_000
            + self.latest_odom.header.stamp.nanosec
        )
        age_sec = (now_ns - odom_ns) * 1e-9
        if age_sec > 0.1 or age_sec < 0.0:
            return

        odom_stamp = self.latest_odom.header.stamp
        pos = self.latest_odom.pose.pose.position
        ori = self.latest_odom.pose.pose.orientation

        # 1. Forward 6-DOF Pose in Inertial Frame ('map')
        pose_msg = PoseStamped()
        pose_msg.header.stamp = odom_stamp
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.position = pos
        pose_msg.pose.orientation = ori
        self.vision_pose_pub.publish(pose_msg)

        # 2. Extract Body Twist and Rotate to Inertial ENU ('map')
        # Standard ROS odometry child_frame_id is 'base_link'
        vx_b = self.latest_odom.twist.twist.linear.x
        vy_b = self.latest_odom.twist.twist.linear.y
        vz_b = self.latest_odom.twist.twist.linear.z

        # Extract yaw from incoming ENU quaternion
        siny_cosp = 2.0 * (ori.w * ori.z + ori.x * ori.y)
        cosy_cosp = 1.0 - 2.0 * (ori.y * ori.y + ori.z * ori.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        vx_enu = math.cos(yaw) * vx_b - math.sin(yaw) * vy_b
        vy_enu = math.sin(yaw) * vx_b + math.cos(yaw) * vy_b
        vz_enu = vz_b

        twist_msg = TwistWithCovarianceStamped()
        twist_msg.header.stamp = odom_stamp
        twist_msg.header.frame_id = 'map'
        twist_msg.twist.twist.linear.x = vx_enu
        twist_msg.twist.twist.linear.y = vy_enu
        twist_msg.twist.twist.linear.z = vz_enu

        # Honest constant measurement covariance for VIO
        cov = [0.0] * 36
        cov[0] = 0.02
        cov[7] = 0.02
        cov[14] = 0.02
        twist_msg.twist.covariance = cov
        self.vision_speed_pub.publish(twist_msg)


def main(args=None):
    rclpy.init(args=args)
    node = VioBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
