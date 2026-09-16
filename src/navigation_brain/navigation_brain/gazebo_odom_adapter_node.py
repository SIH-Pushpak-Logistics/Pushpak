#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


def quat_multiply(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return [
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2
    ]


class GazeboOdomAdapterNode(Node):
    """
    SIMULATION-ONLY SHIM:
    Translates Gazebo NWU ground truth into ROS Standard ENU Odometry.
    Outputs directly to /vio/odometry to mirror physical VIO interfaces.
    """
    def __init__(self):
        super().__init__('gazebo_odom_adapter_node')

        self.sub_raw = self.create_subscription(
            Odometry, '/sim/ground_truth/odom', self.raw_cb, 10
        )
        self.pub_enu = self.create_publisher(
            Odometry, '/vio/odometry', 10
        )

        # Gazebo NWU to ROS ENU rotation (+90 deg around Z)
        self.q_nwu_to_enu = [0.0, 0.0, math.sin(math.pi / 4.0), math.cos(math.pi / 4.0)]
        self.get_logger().info('Gazebo NWU->ENU Odometry Adapter active.')

    def raw_cb(self, msg: Odometry):
        pos_nwu = msg.pose.pose.position
        ori_nwu = msg.pose.pose.orientation

        # Kinematic coordinate swap: X_enu = -Y_nwu, Y_enu = X_nwu, Z_enu = Z_nwu
        enu_msg = Odometry()
        enu_msg.header.stamp = msg.header.stamp
        enu_msg.header.frame_id = 'odom'
        enu_msg.child_frame_id = 'base_link'

        enu_msg.pose.pose.position.x = -pos_nwu.y
        enu_msg.pose.pose.position.y = pos_nwu.x
        enu_msg.pose.pose.position.z = pos_nwu.z

        q_raw = [ori_nwu.x, ori_nwu.y, ori_nwu.z, ori_nwu.w]
        q_enu = quat_multiply(self.q_nwu_to_enu, q_raw)
        enu_msg.pose.pose.orientation.x = q_enu[0]
        enu_msg.pose.pose.orientation.y = q_enu[1]
        enu_msg.pose.pose.orientation.z = q_enu[2]
        enu_msg.pose.pose.orientation.w = q_enu[3]

        # Preserve body twist without corrupting derivatives
        enu_msg.twist = msg.twist

        self.pub_enu.publish(enu_msg)


def main(args=None):
    rclpy.init(args=args)
    node = GazeboOdomAdapterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
