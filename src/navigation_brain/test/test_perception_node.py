"""
Unit tests for PerceptionNode ROS 2 integration contracts.
"""

import glob
import math
import os
import cv2
import numpy as np
import pytest
import rclpy
from rclpy.parameter import Parameter
from sensor_msgs.msg import CameraInfo, Image, Imu, Range
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistWithCovarianceStamped
from cv_bridge import CvBridge

from drone_interfaces.msg import SurvivorDetection
from navigation_brain.perception_core import ImageQualityEvaluator
from navigation_brain.perception_node import PerceptionNode


def get_compatible_bridge():
    bridge = CvBridge()
    if hasattr(bridge, "cvtype_to_name") and 16 not in bridge.cvtype_to_name:
        bridge.cvtype_to_name[16] = "8UC3"
    return bridge


@pytest.fixture(scope="module")
def ros_context():
    rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def test_odom_freshness_rejection(ros_context):
    node = PerceptionNode(node_name="test_odom_freshness")
    assert node.conf_threshold == 0.85
    assert node.imgsz == 320

    # 1. No odom received yet -> returns None
    assert node._get_fresh_odom() is None

    # 2. Fresh odom received (0.1s old)
    now = node.get_clock().now()
    fresh_odom = Odometry()
    fresh_odom.header.stamp = (now - rclpy.time.Duration(seconds=0.1)).to_msg()
    fresh_odom.header.frame_id = "odom"
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
    stale_odom.header.frame_id = "odom"
    node.odom_callback(stale_odom)
    assert node._get_fresh_odom() is None

    node.destroy_node()


def test_continuous_velocity_publishing_on_degraded_stream(ros_context):
    node = PerceptionNode(node_name="test_continuous_vel")
    bridge = get_compatible_bridge()

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


# =============================================================
# Reviewer Item 1: SurvivorDetection Header Frame ID
# =============================================================

def test_survivor_detection_frame_id_from_odometry(ros_context, tmp_path):
    det_dir = str(tmp_path / "detections_frame_id")
    node = PerceptionNode(
        node_name="test_detection_frame_id",
        parameter_overrides=[
            Parameter("detection_dir", Parameter.Type.STRING, det_dir),
        ],
    )

    published_detections = []
    sub = node.create_subscription(
        SurvivorDetection,
        "/detections/survivor",
        lambda m: published_detections.append(m),
        10,
    )

    now = node.get_clock().now()
    now_sec = float(now.nanoseconds) * 1.0e-9

    # Prime intrinsics
    cinfo = CameraInfo()
    cinfo.header.stamp = now.to_msg()
    cinfo.width = 320
    cinfo.height = 240
    cinfo.k = [277.0, 0.0, 160.0, 0.0, 277.0, 120.0, 0.0, 0.0, 1.0]
    node.camera_info_callback(cinfo)

    # Prime fresh range (altitude = 2.0m)
    rng = Range()
    rng.header.stamp = now.to_msg()
    rng.range = 2.0
    node.range_callback(rng)

    # Prime fresh odometry with a non-default custom frame_id
    odom = Odometry()
    odom.header.stamp = (now - rclpy.time.Duration(seconds=0.05)).to_msg()
    odom.header.frame_id = "odom_custom"
    odom.pose.pose.position.x = 10.0
    odom.pose.pose.position.y = 20.0
    odom.pose.pose.position.z = 2.0
    odom.pose.pose.orientation.w = 1.0
    node.odom_callback(odom)

    # Simulate survivor detection callback
    frame = np.full((240, 320, 3), 100, dtype=np.uint8)
    node._publish_survivor_detection(
        frame=frame,
        image_stamp=now_sec,
        confidence=0.95,
        x1=150.0,
        y1=110.0,
        x2=170.0,
        y2=130.0,
    )
    rclpy.spin_once(node, timeout_sec=0.05)

    assert len(published_detections) == 1
    det = published_detections[0]
    # Frame ID must match odometry frame_id (proves not hardcoded to base_link or odom)
    assert det.header.frame_id == "odom_custom"
    assert math.isclose(det.confidence, 0.95, abs_tol=1e-4)
    # World position projected in odom frame
    assert math.isclose(det.world_position.x, 10.0, abs_tol=0.1)
    assert math.isclose(det.world_position.y, 20.0, abs_tol=0.1)
    assert math.isclose(det.world_position.z, 0.0, abs_tol=0.1)

    node.destroy_node()


# =============================================================
# Reviewer Item 2: Do Not Publish Detection Without Fresh Odometry
# =============================================================

def test_missing_odometry_blocks_survivor_detection(ros_context, tmp_path):
    det_dir = str(tmp_path / "detections_missing_odom")
    node = PerceptionNode(
        node_name="test_missing_odom_det",
        parameter_overrides=[
            Parameter("detection_dir", Parameter.Type.STRING, det_dir),
        ],
    )

    published_detections = []
    node.create_subscription(
        SurvivorDetection,
        "/detections/survivor",
        lambda m: published_detections.append(m),
        10,
    )

    now = node.get_clock().now()
    now_sec = float(now.nanoseconds) * 1.0e-9

    # Prime camera and range, but NO odometry
    cinfo = CameraInfo()
    cinfo.header.stamp = now.to_msg()
    cinfo.width = 320
    cinfo.height = 240
    cinfo.k = [277.0, 0.0, 160.0, 0.0, 277.0, 120.0, 0.0, 0.0, 1.0]
    node.camera_info_callback(cinfo)

    rng = Range()
    rng.header.stamp = now.to_msg()
    rng.range = 2.0
    node.range_callback(rng)

    frame = np.full((240, 320, 3), 100, dtype=np.uint8)
    node._publish_survivor_detection(
        frame=frame,
        image_stamp=now_sec,
        confidence=0.91,
        x1=100.0,
        y1=100.0,
        x2=160.0,
        y2=160.0,
    )
    rclpy.spin_once(node, timeout_sec=0.05)

    # 1. No SurvivorDetection message published
    assert len(published_detections) == 0

    # 2. Crop must still be saved to disk
    crops = glob.glob(os.path.join(det_dir, "survivor_*.jpg"))
    assert len(crops) == 1
    assert os.path.exists(crops[0])
    assert os.path.getsize(crops[0]) > 0

    node.destroy_node()


def test_stale_odometry_blocks_survivor_detection(ros_context, tmp_path):
    det_dir = str(tmp_path / "detections_stale_odom")
    node = PerceptionNode(
        node_name="test_stale_odom_det",
        parameter_overrides=[
            Parameter("detection_dir", Parameter.Type.STRING, det_dir),
            Parameter("odom_staleness_sec", Parameter.Type.DOUBLE, 0.5),
        ],
    )

    published_detections = []
    node.create_subscription(
        SurvivorDetection,
        "/detections/survivor",
        lambda m: published_detections.append(m),
        10,
    )

    now = node.get_clock().now()
    now_sec = float(now.nanoseconds) * 1.0e-9

    cinfo = CameraInfo()
    cinfo.header.stamp = now.to_msg()
    cinfo.width = 320
    cinfo.height = 240
    cinfo.k = [277.0, 0.0, 160.0, 0.0, 277.0, 120.0, 0.0, 0.0, 1.0]
    node.camera_info_callback(cinfo)

    rng = Range()
    rng.header.stamp = now.to_msg()
    rng.range = 2.0
    node.range_callback(rng)

    # Stale odometry (1.2 seconds old > 0.5s limit)
    stale_odom = Odometry()
    stale_odom.header.stamp = (now - rclpy.time.Duration(seconds=1.2)).to_msg()
    stale_odom.header.frame_id = "odom"
    stale_odom.pose.pose.position.x = 5.0
    stale_odom.pose.pose.position.y = 5.0
    stale_odom.pose.pose.position.z = 2.0
    stale_odom.pose.pose.orientation.w = 1.0
    node.odom_callback(stale_odom)

    frame = np.full((240, 320, 3), 100, dtype=np.uint8)
    node._publish_survivor_detection(
        frame=frame,
        image_stamp=now_sec,
        confidence=0.88,
        x1=120.0,
        y1=80.0,
        x2=180.0,
        y2=140.0,
    )
    rclpy.spin_once(node, timeout_sec=0.05)

    # 1. No SurvivorDetection published
    assert len(published_detections) == 0

    # 2. Crop must still be saved
    crops = glob.glob(os.path.join(det_dir, "survivor_*.jpg"))
    assert len(crops) == 1

    node.destroy_node()


def test_fresh_odometry_publishes_survivor_detection(ros_context, tmp_path):
    det_dir = str(tmp_path / "detections_fresh_odom")
    node = PerceptionNode(
        node_name="test_fresh_odom_det",
        parameter_overrides=[
            Parameter("detection_dir", Parameter.Type.STRING, det_dir),
            Parameter("odom_staleness_sec", Parameter.Type.DOUBLE, 0.5),
        ],
    )

    published_detections = []
    node.create_subscription(
        SurvivorDetection,
        "/detections/survivor",
        lambda m: published_detections.append(m),
        10,
    )

    now = node.get_clock().now()
    now_sec = float(now.nanoseconds) * 1.0e-9

    cinfo = CameraInfo()
    cinfo.header.stamp = now.to_msg()
    cinfo.width = 320
    cinfo.height = 240
    cinfo.k = [277.0, 0.0, 160.0, 0.0, 277.0, 120.0, 0.0, 0.0, 1.0]
    node.camera_info_callback(cinfo)

    rng = Range()
    rng.header.stamp = now.to_msg()
    rng.range = 2.0
    node.range_callback(rng)

    # Fresh odometry (0.05s old)
    fresh_odom = Odometry()
    fresh_odom.header.stamp = (now - rclpy.time.Duration(seconds=0.05)).to_msg()
    fresh_odom.header.frame_id = "odom"
    fresh_odom.pose.pose.position.x = 5.0
    fresh_odom.pose.pose.position.y = 10.0
    fresh_odom.pose.pose.position.z = 2.0
    fresh_odom.pose.pose.orientation.w = 1.0
    node.odom_callback(fresh_odom)

    frame = np.full((240, 320, 3), 100, dtype=np.uint8)
    node._publish_survivor_detection(
        frame=frame,
        image_stamp=now_sec,
        confidence=0.92,
        x1=160.0,
        y1=120.0,
        x2=180.0,
        y2=140.0,
    )
    rclpy.spin_once(node, timeout_sec=0.05)

    assert len(published_detections) == 1
    det = published_detections[0]
    assert det.header.frame_id == "odom"
    assert math.isfinite(det.world_position.x)
    assert math.isfinite(det.world_position.y)
    assert math.isfinite(det.world_position.z)
    # Never Point(0, 0, 0) fabricated fallback
    assert not (det.world_position.x == 0.0 and det.world_position.y == 0.0 and det.world_position.z == 0.0)

    node.destroy_node()


# =============================================================
# Reviewer Item 3: Laplacian Baseline Auto-Calibration
# =============================================================

def test_auto_laplacian_baseline_calibration(ros_context, tmp_path):
    bridge = get_compatible_bridge()
    node = PerceptionNode(
        node_name="test_auto_laplacian",
        parameter_overrides=[
            Parameter("laplacian_baseline", Parameter.Type.DOUBLE, 0.0),
            Parameter("laplacian_baseline_frames", Parameter.Type.INTEGER, 5),
            Parameter("detection_dir", Parameter.Type.STRING, str(tmp_path)),
        ],
    )

    assert node.auto_calibrate_baseline is True
    assert node.baseline_calibrated is False
    assert node.laplacian_baseline == 0.0
    assert node.laplacian_baseline_frames == 5

    now = node.get_clock().now()
    cinfo = CameraInfo()
    cinfo.header.stamp = now.to_msg()
    cinfo.width = 320
    cinfo.height = 240
    cinfo.k = [277.0, 0.0, 160.0, 0.0, 277.0, 120.0, 0.0, 0.0, 1.0]
    node.camera_info_callback(cinfo)

    # Generate 5 textured frames with distinct known variances
    synthetic_frames = []
    variances = []
    np.random.seed(123)
    for i in range(5):
        # Varying checkerboard / noise textures
        grid = np.random.randint(0, 50 * (i + 1), (240, 320), dtype=np.uint8)
        bgr = cv2.cvtColor(grid, cv2.COLOR_GRAY2BGR)
        synthetic_frames.append(bgr)
        variances.append(ImageQualityEvaluator.compute_variance(grid))

    expected_median = float(np.median(variances))
    assert expected_median > 0.0

    # Feed frames 0..3: calibration should still be in progress
    for i in range(4):
        t = now + rclpy.time.Duration(seconds=i * 0.05)
        rng = Range()
        rng.header.stamp = t.to_msg()
        rng.range = 2.0
        node.range_callback(rng)

        imu = Imu()
        imu.header.stamp = t.to_msg()
        node.imu_callback(imu)

        img_msg = bridge.cv2_to_imgmsg(synthetic_frames[i], encoding="bgr8")
        img_msg.header.stamp = t.to_msg()
        node.image_callback(img_msg)
        rclpy.spin_once(node, timeout_sec=0.01)

        assert not node.baseline_calibrated
        assert len(node.baseline_samples) == i + 1

    # Feed frame 4 (5th frame): triggers calibration completion
    t4 = now + rclpy.time.Duration(seconds=4 * 0.05)
    rng = Range()
    rng.header.stamp = t4.to_msg()
    rng.range = 2.0
    node.range_callback(rng)

    imu = Imu()
    imu.header.stamp = t4.to_msg()
    node.imu_callback(imu)

    img_msg4 = bridge.cv2_to_imgmsg(synthetic_frames[4], encoding="bgr8")
    img_msg4.header.stamp = t4.to_msg()
    node.image_callback(img_msg4)
    rclpy.spin_once(node, timeout_sec=0.01)

    # Baseline must be calibrated to the median
    assert node.baseline_calibrated is True
    assert math.isclose(node.laplacian_baseline, expected_median, rel_tol=1e-5)
    assert math.isclose(node.quality_evaluator.baseline, expected_median, rel_tol=1e-5)

    # Feed a 6th frame with high variance; baseline must remain stable
    extra_grid = np.random.randint(0, 255, (240, 320), dtype=np.uint8)
    extra_bgr = cv2.cvtColor(extra_grid, cv2.COLOR_GRAY2BGR)
    t5 = now + rclpy.time.Duration(seconds=5 * 0.05)
    img_msg5 = bridge.cv2_to_imgmsg(extra_bgr, encoding="bgr8")
    img_msg5.header.stamp = t5.to_msg()
    node.image_callback(img_msg5)
    rclpy.spin_once(node, timeout_sec=0.01)

    assert math.isclose(node.laplacian_baseline, expected_median, rel_tol=1e-5)
    assert len(node.baseline_samples) == 5

    node.destroy_node()


def test_manual_laplacian_baseline_override(ros_context, tmp_path):
    bridge = get_compatible_bridge()
    node = PerceptionNode(
        node_name="test_manual_laplacian",
        parameter_overrides=[
            Parameter("laplacian_baseline", Parameter.Type.DOUBLE, 250.0),
            Parameter("detection_dir", Parameter.Type.STRING, str(tmp_path)),
        ],
    )

    assert node.auto_calibrate_baseline is False
    assert node.baseline_calibrated is True
    assert node.laplacian_baseline == 250.0
    assert node.quality_evaluator.baseline == 250.0
    assert len(node.baseline_samples) == 0

    # Feed an image frame; auto-calibration should be completely skipped
    now = node.get_clock().now()
    frame = np.full((240, 320, 3), 100, dtype=np.uint8)
    img_msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
    img_msg.header.stamp = now.to_msg()
    node.image_callback(img_msg)
    rclpy.spin_once(node, timeout_sec=0.01)

    assert node.laplacian_baseline == 250.0
    assert len(node.baseline_samples) == 0

    node.destroy_node()


# =============================================================
# Reviewer Item 4: Covariance Ramp Parameters
# =============================================================

def test_covariance_parameters_and_defaults(ros_context):
    # 1. Defaults verification
    node_def = PerceptionNode(node_name="test_cov_params_default")
    assert node_def.nominal_covariance == 0.05
    assert node_def.degraded_covariance == 1.0e6
    assert node_def.ramp_frames == 5

    assert node_def.covariance_ramp.nominal_covariance == 0.05
    assert node_def.covariance_ramp.degraded_covariance == 1.0e6
    assert node_def.covariance_ramp.ramp_frames == 5
    node_def.destroy_node()

    # 2. Configured parameter overrides verification
    node_custom = PerceptionNode(
        node_name="test_cov_params_custom",
        parameter_overrides=[
            Parameter("nominal_covariance", Parameter.Type.DOUBLE, 0.10),
            Parameter("degraded_covariance", Parameter.Type.DOUBLE, 5.0e5),
            Parameter("ramp_frames", Parameter.Type.INTEGER, 8),
        ],
    )
    assert node_custom.nominal_covariance == 0.10
    assert node_custom.degraded_covariance == 5.0e5
    assert node_custom.ramp_frames == 8

    assert node_custom.covariance_ramp.nominal_covariance == 0.10
    assert node_custom.covariance_ramp.degraded_covariance == 5.0e5
    assert node_custom.covariance_ramp.ramp_frames == 8
    node_custom.destroy_node()
