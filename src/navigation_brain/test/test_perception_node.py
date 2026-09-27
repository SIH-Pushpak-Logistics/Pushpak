"""
Unit tests for PerceptionNode ROS 2 integration contracts.
"""

import math
import numpy as np
import pytest
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, Imu, Range
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistWithCovarianceStamped
from cv_bridge import CvBridge

from drone_interfaces.msg import SurvivorDetection
from navigation_brain.perception_node import PerceptionNode


@pytest.fixture(scope="module")
def ros_context():
    rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def test_odom_freshness_rejection(ros_context):
    node = PerceptionNode()
    assert node.conf_threshold == 0.85
    assert node.imgsz == 320

    # 1. No odom received yet -> returns None
    assert node._get_fresh_odom() is None

    # 2. Fresh odom received (0.1s old)
    now = node.get_clock().now()
    fresh_odom = Odometry()
    fresh_odom.header.stamp = (now - rclpy.time.Duration(seconds=0.1)).to_msg()
    fresh_odom.pose.pose.position.x = 3.0
    fresh_odom.pose.pose.position.y = 4.0
    fresh_odom.pose.pose.position.z = 1.5
    fresh_odom.pose.pose.orientation.w = 1.0

    node.odom_callback(fresh_odom)
    odom = node._get_fresh_odom()
    assert odom is not None
    assert math.isclose(odom.pose.pose.position.x, 3.0)

    # 3. Stale odom (> 0.5s limit)
    stale_odom = Odometry()
    stale_odom.header.stamp = (now - rclpy.time.Duration(seconds=0.6)).to_msg()
    node.odom_callback(stale_odom)
    assert node._get_fresh_odom() is None

    node.destroy_node()


def test_continuous_velocity_publishing_on_degraded_stream(ros_context):
    node = PerceptionNode()
    bridge = CvBridge()

    published_messages = []
    def vel_cb(msg):
        published_messages.append(msg)

    sub = node.create_subscription(TwistWithCovarianceStamped, "/visual/velocity", vel_cb, 10)

    # Prime sensor inputs
    now = node.get_clock().now()
    cinfo = CameraInfo()
    cinfo.header.stamp = now.to_msg()
    cinfo.width = 320
    cinfo.height = 240
    cinfo.k = [277.0, 0.0, 160.0, 0.0, 277.0, 120.0, 0.0, 0.0, 1.0]
    node.camera_info_callback(cinfo)

    # Feed 10 degraded (uniform flat) frames
    flat_frame = np.full((240, 320, 3), 128, dtype=np.uint8)

    for i in range(10):
        t = now + rclpy.time.Duration(seconds=i * 0.05)

        rng = Range()
        rng.header.stamp = t.to_msg()
        rng.range = 2.0
        node.range_callback(rng)

        imu = Imu()
        imu.header.stamp = t.to_msg()
        node.imu_callback(imu)

        img_msg = bridge.cv2_to_imgmsg(flat_frame, encoding="bgr8")
        img_msg.header.stamp = t.to_msg()
        node.image_callback(img_msg)

        rclpy.spin_once(node, timeout_sec=0.01)

    assert len(published_messages) == 10
    # Covariance on frame 5 onwards must be 1.0e6
    assert published_messages[4].twist.covariance[0] == 1.0e6
    assert published_messages[9].twist.covariance[0] == 1.0e6

    node.destroy_node()