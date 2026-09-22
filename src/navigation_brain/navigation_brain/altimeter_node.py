#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Range
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.qos import qos_profile_sensor_data

# VL53L1X-class ToF sensor: standard accuracy +/- 20 mm (sigma = 0.02 m)
# Variance = sigma^2 = 0.0004 m^2 (cited per Invariant I-10)
TOF_ACCURACY_M = 0.02
TOF_VARIANCE = TOF_ACCURACY_M ** 2
INFLATED_VARIANCE = 1e6
MAX_TOF_RANGE_M = 4.0


class AltimeterNode(Node):
    def __init__(self):
        super().__init__('altimeter_node')

        self.sub_scan = self.create_subscription(
            LaserScan,
            '/drone/rangefinder/scan',
            self.scan_callback,
            qos_profile_sensor_data
        )

        self.pub_range = self.create_publisher(
            Range,
            '/drone/tof_range',
            qos_profile_sensor_data
        )

        self.pub_pose = self.create_publisher(
            PoseWithCovarianceStamped,
            '/ekf/altitude_pose',
            10
        )

        self.get_logger().info(
            'v2 AltimeterNode active: /drone/rangefinder/scan -> '
            '/drone/tof_range (Range) & /ekf/altitude_pose (PoseWithCovarianceStamped)'
        )

    def scan_callback(self, msg: LaserScan):
        if not msg.ranges:
            return

        raw_z = msg.ranges[0]
        stamp = msg.header.stamp

        # Determine validity per REP-117 & sensor bounds (4.0m max per README §11)
        is_nan = math.isnan(raw_z)
        is_underflow = (not is_nan) and (raw_z < msg.range_min)
        is_overflow = (not is_nan) and (raw_z > min(msg.range_max, MAX_TOF_RANGE_M))
        is_valid = not (is_nan or is_underflow or is_overflow or raw_z <= 0.0)

        # 1. Publish sensor_msgs/msg/Range (REP-117 compliant)
        range_msg = Range()
        range_msg.header.stamp = stamp
        range_msg.header.frame_id = 'tof_link'
        range_msg.radiation_type = Range.INFRARED
        range_msg.field_of_view = 0.471  # ~27 deg FoV for VL53L1X
        range_msg.min_range = float(msg.range_min)
        range_msg.max_range = float(MAX_TOF_RANGE_M)

        if is_nan:
            range_msg.range = float('nan')
        elif is_underflow:
            range_msg.range = float('-inf')
        elif is_overflow:
            range_msg.range = float('inf')
        else:
            range_msg.range = float(raw_z)

        self.pub_range.publish(range_msg)

        # 2. Publish geometry_msgs/msg/PoseWithCovarianceStamped
        # Z-only pose for robot_localization ekf_node (world_frame: odom)
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = 'odom'

        # Diagonal covariance (6x6): 1e6 everywhere except Z
        cov = [0.0] * 36
        cov[0] = INFLATED_VARIANCE   # x
        cov[7] = INFLATED_VARIANCE   # y
        cov[21] = INFLATED_VARIANCE  # roll
        cov[28] = INFLATED_VARIANCE  # pitch
        cov[35] = INFLATED_VARIANCE  # yaw

        if is_valid:
            pose_msg.pose.pose.position.z = float(raw_z)
            cov[14] = float(TOF_VARIANCE)  # z variance
        else:
            pose_msg.pose.pose.position.z = 0.0
            cov[14] = float(INFLATED_VARIANCE)  # I-6: Inflate R on degradation, never halt

        pose_msg.pose.covariance = cov
        self.pub_pose.publish(pose_msg)


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
