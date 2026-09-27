#!/usr/bin/env python3
"""
Dual-Pipeline Perception Node for Pushpak UAV.

Architecture:
- Pipeline A: Lucas-Kanade optical flow, gyro de-rotation, pinhole velocity scaling,
  and dust/blur degradation covariance ramping. Publishes continuously on /visual/velocity (>=14 Hz).
- Pipeline B: YOLOv8 person detection worker, running asynchronously on GPU, throttled
  to <=2 Hz, publishing SurvivorDetection with 3D world coordinates in the odom frame.

All pure mathematical calculations (optical flow, de-rotation, degradation metric, covariance
ramping, and world projection) are decoupled into navigation_brain.perception_core.
This node serves as the ROS 2 integration layer.
"""

import math
import os
import threading
from typing import Optional

import cv2
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, Imu, Range

from drone_interfaces.msg import SurvivorDetection
from navigation_brain.perception_core import (
    CameraIntrinsics,
    CovarianceRamp,
    ImageQualityEvaluator,
    OpticalFlowTracker,
    WorldProjector,
)

try:
    from ultralytics import YOLO
    import torch
    HAVE_ULTRALYTICS = True
except ImportError:
    HAVE_ULTRALYTICS = False


class PerceptionNode(Node):
    """
    ROS 2 perception node implementing dual-pipeline visual navigation and survivor detection.
    """

    def __init__(self):
        super().__init__('perception_node')

        # ---------------------------------------------------------
        # Parameters
        # ---------------------------------------------------------
        self.declare_parameter('model_path', '/workspace/yolov8n.pt')
        self.declare_parameter('device', 'cuda:0')
        self.declare_parameter('imgsz', 320)
        self.declare_parameter('conf_threshold', 0.85)
        self.declare_parameter('max_rate_hz', 2.0)
        self.declare_parameter('odom_staleness_sec', 0.5)
        self.declare_parameter('laplacian_baseline', 453.151)
        self.declare_parameter('laplacian_ratio_threshold', 0.30)
        self.declare_parameter('detection_dir', '/workspace/detections')

        self.model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.device = self.get_parameter('device').get_parameter_value().string_value
        self.imgsz = self.get_parameter('imgsz').get_parameter_value().integer_value
        self.conf_threshold = self.get_parameter('conf_threshold').get_parameter_value().double_value
        self.max_rate_hz = self.get_parameter('max_rate_hz').get_parameter_value().double_value
        self.odom_staleness_sec = self.get_parameter('odom_staleness_sec').get_parameter_value().double_value
        self.laplacian_baseline = self.get_parameter('laplacian_baseline').get_parameter_value().double_value
        self.laplacian_ratio_threshold = self.get_parameter('laplacian_ratio_threshold').get_parameter_value().double_value
        self.detection_dir = self.get_parameter('detection_dir').get_parameter_value().string_value

        self.min_yolo_period_sec = 1.0 / max(0.1, self.max_rate_hz)

        # ---------------------------------------------------------
        # Core Pure-Python Mathematical Modules
        # ---------------------------------------------------------
        self.quality_evaluator = ImageQualityEvaluator(
            baseline=self.laplacian_baseline,
            ratio_threshold=self.laplacian_ratio_threshold,
        )
        self.covariance_ramp = CovarianceRamp(
            nominal_covariance=0.05,
            degraded_covariance=1.0e6,
            ramp_frames=5,
        )
        self.flow_tracker = OpticalFlowTracker(
            max_corners=250,
            quality_level=0.02,
            min_distance=7.0,
            block_size=7,
            min_features_valid=15,
            reseed_threshold=50,
        )
        self.world_projector = WorldProjector()

        # ---------------------------------------------------------
        # State and Synchronization
        # ---------------------------------------------------------
        self.bridge = CvBridge()
        self.state_lock = threading.Lock()

        self.intrinsics: Optional[CameraIntrinsics] = None
        self.camera_info_ready = False

        self.latest_altitude: Optional[float] = None
        self.latest_altitude_stamp: Optional[float] = None

        self.latest_gyro_x: float = 0.0
        self.latest_gyro_y: float = 0.0
        self.latest_gyro_z: float = 0.0
        self.latest_gyro_stamp: Optional[float] = None

        self.latest_odom: Optional[Odometry] = None
        self.latest_odom_stamp: Optional[float] = None
        self.odom_stale_logged = False

        # Pipeline B Worker State
        self.yolo_frame: Optional[np.ndarray] = None
        self.yolo_frame_stamp: Optional[float] = None
        self.last_yolo_stamp: Optional[float] = None
        self.detection_counter: int = 0

        self.yolo_condition = threading.Condition(self.state_lock)
        self.yolo_stop_event = threading.Event()

        try:
            os.makedirs(self.detection_dir, exist_ok=True)
        except OSError as exc:
            self.get_logger().error(f"Failed to create detection directory {self.detection_dir}: {exc}")

        # ---------------------------------------------------------
        # ROS Publishers
        # ---------------------------------------------------------
        self.velocity_pub = self.create_publisher(
            TwistWithCovarianceStamped,
            '/visual/velocity',
            10,
        )
        self.detection_pub = self.create_publisher(
            SurvivorDetection,
            '/detections/survivor',
            10,
        )

        # ---------------------------------------------------------
        # ROS Subscriptions
        # ---------------------------------------------------------
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            qos_profile_sensor_data,
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            qos_profile_sensor_data,
        )
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/raw',
            self.imu_callback,
            qos_profile_sensor_data,
        )
        self.range_sub = self.create_subscription(
            Range,
            '/drone/tof_range',
            self.range_callback,
            qos_profile_sensor_data,
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odom_callback,
            10,
        )

        # ---------------------------------------------------------
        # Start Pipeline B YOLO Worker Thread
        # ---------------------------------------------------------
        self.model = None
        self.yolo_thread = threading.Thread(
            target=self._yolo_worker,
            name='yolo_worker',
            daemon=True,
        )
        self.yolo_thread.start()

        self.get_logger().info(
            f"Perception node initialized: device={self.device}, imgsz={self.imgsz}, "
            f"conf={self.conf_threshold:.2f}, max_rate={self.max_rate_hz:.2f} Hz"
        )

    # =============================================================
    # Sensor Callbacks
    # =============================================================

    def camera_info_callback(self, msg: CameraInfo):
        """
        Cache camera intrinsics strictly from /camera/camera_info.
        """
        fx = float(msg.k[0])
        fy = float(msg.k[4])
        cx = float(msg.k[2])
        cy = float(msg.k[5])

        if fx <= 0.0 or fy <= 0.0 or not (math.isfinite(fx) and math.isfinite(fy)):
            self.get_logger().warning("Received invalid camera intrinsics from /camera/camera_info.")
            return

        with self.state_lock:
            self.intrinsics = CameraIntrinsics(
                fx=fx,
                fy=fy,
                cx=cx,
                cy=cy,
                width=int(msg.width),
                height=int(msg.height),
            )
            self.camera_info_ready = True

        self.get_logger().info(
            f"Camera intrinsics cached: fx={fx:.3f}, fy={fy:.3f}, cx={cx:.3f}, cy={cy:.3f}, "
            f"dim={msg.width}x{msg.height}"
        )

    def range_callback(self, msg: Range):
        """
        Cache ToF altitude and timestamp.
        """
        stamp_sec = self.stamp_to_sec(msg.header.stamp)
        with self.state_lock:
            self.latest_altitude = float(msg.range)
            self.latest_altitude_stamp = stamp_sec

    def imu_callback(self, msg: Imu):
        """
        Cache body angular velocity from /imu/raw.
        """
        stamp_sec = self.stamp_to_sec(msg.header.stamp)
        with self.state_lock:
            self.latest_gyro_x = float(msg.angular_velocity.x)
            self.latest_gyro_y = float(msg.angular_velocity.y)
            self.latest_gyro_z = float(msg.angular_velocity.z)
            self.latest_gyro_stamp = stamp_sec

    def odom_callback(self, msg: Odometry):
        """
        Cache latest EKF state from /odometry/filtered.
        """
        stamp_sec = self.stamp_to_sec(msg.header.stamp)
        with self.state_lock:
            self.latest_odom = msg
            self.latest_odom_stamp = stamp_sec
            if self.odom_stale_logged:
                self.odom_stale_logged = False

    # =============================================================
    # Image Callback — Pipeline A & Queue to Pipeline B
    # =============================================================

    def image_callback(self, msg: Image):
        """
        Primary camera callback.

        Executes Pipeline A synchronously:
        - Image decode to BGR and Grayscale
        - Quality/dust degradation check
        - Lucas-Kanade optical flow with gyro de-rotation
        - Velocity scaling and covariance ramp
        - Continuous publication on /visual/velocity

        Feeds Pipeline B asynchronously without blocking.
        """
        image_stamp = self.stamp_to_sec(msg.header.stamp)

        # 1. Convert ROS Image message
        try:
            bgr_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2GRAY)
        except CvBridgeError:
            try:
                frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
                gray = np.asarray(frame)
                bgr_frame = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
            except CvBridgeError as exc:
                self.get_logger().error(f"Image conversion failed: {exc}")
                return

        if gray is None or bgr_frame is None:
            return

        # 2. Queue latest color frame for Pipeline B
        self._queue_yolo_frame(bgr_frame, image_stamp)

        # 3. Evaluate image quality (dust/blur degradation)
        quality_result = self.quality_evaluator.evaluate(gray)

        # 4. Snapshot sensor state
        with self.state_lock:
            camera_ready = self.camera_info_ready
            intrinsics = self.intrinsics
            altitude = self.latest_altitude
            altitude_stamp = self.latest_altitude_stamp
            gyro_x = self.latest_gyro_x
            gyro_y = self.latest_gyro_y
            gyro_stamp = self.latest_gyro_stamp

        # 5. Check sensor prerequisites
        intrinsics_valid = camera_ready and intrinsics is not None and intrinsics.fx > 0.0
        altitude_valid = (
            altitude is not None
            and altitude_stamp is not None
            and 0.0 <= (image_stamp - altitude_stamp) <= 0.5
            and math.isfinite(altitude)
            and altitude > 0.0
        )
        gyro_valid = (
            gyro_stamp is not None
            and 0.0 <= (image_stamp - gyro_stamp) <= 0.5
            and math.isfinite(gyro_x)
            and math.isfinite(gyro_y)
        )

        if not (intrinsics_valid and altitude_valid and gyro_valid):
            self.flow_tracker.reinit_features(gray)
            self._publish_velocity(msg, vx=0.0, vy=0.0, is_degraded=True)
            return

        # 6. Compute optical flow velocity
        flow_result = self.flow_tracker.process_frame(
            gray=gray,
            timestamp_sec=image_stamp,
            altitude=altitude,
            gyro_x=gyro_x,
            gyro_y=gyro_y,
            intrinsics=intrinsics,
        )

        effective_degraded = quality_result.is_degraded or (not flow_result.is_valid)

        # 7. Publish visual velocity (never stops publishing)
        self._publish_velocity(
            msg,
            vx=flow_result.vx if flow_result.is_valid else 0.0,
            vy=flow_result.vy if flow_result.is_valid else 0.0,
            is_degraded=effective_degraded,
        )

    def _publish_velocity(
        self,
        image_msg: Image,
        vx: float,
        vy: float,
        is_degraded: bool,
    ):
        """
        Publish visual velocity on /visual/velocity with appropriate covariance.
        """
        cov_xy = self.covariance_ramp.update(is_degraded=is_degraded)
        cov_matrix = CovarianceRamp.build_covariance_matrix(cov_xy)

        msg = TwistWithCovarianceStamped()
        msg.header.stamp = image_msg.header.stamp
        msg.header.frame_id = 'base_link'

        msg.twist.twist.linear.x = float(vx) if math.isfinite(vx) else 0.0
        msg.twist.twist.linear.y = float(vy) if math.isfinite(vy) else 0.0
        msg.twist.twist.linear.z = 0.0
        msg.twist.twist.angular.x = 0.0
        msg.twist.twist.angular.y = 0.0
        msg.twist.twist.angular.z = 0.0
        msg.twist.covariance = cov_matrix

        self.velocity_pub.publish(msg)

    # =============================================================
    # Pipeline B — YOLO Worker & World Position Projection
    # =============================================================

    def _queue_yolo_frame(self, frame: np.ndarray, stamp: float):
        """
        Queue newest frame for Pipeline B.
        """
        with self.yolo_condition:
            self.yolo_frame = frame
            self.yolo_frame_stamp = stamp
            self.yolo_condition.notify()

    def _yolo_worker(self):
        """
        Background worker thread executing YOLO inference and publishing survivor detections.
        """
        if not HAVE_ULTRALYTICS:
            self.get_logger().error("Ultralytics YOLO not installed; Pipeline B disabled.")
            return

        # Load YOLO model
        try:
            self.get_logger().info(f"Loading YOLO model from {self.model_path} onto {self.device}")
            model = YOLO(self.model_path)
            model.to(self.device)
            self.model = model
            self.get_logger().info("YOLO model loaded successfully.")
        except Exception as exc:
            self.get_logger().error(f"Failed to load YOLO model: {exc}")
            return

        while rclpy.ok() and not self.yolo_stop_event.is_set():
            with self.yolo_condition:
                while self.yolo_frame is None and not self.yolo_stop_event.is_set():
                    self.yolo_condition.wait(timeout=0.1)

                if self.yolo_stop_event.is_set():
                    break

                frame = self.yolo_frame
                frame_stamp = self.yolo_frame_stamp
                self.yolo_frame = None
                self.yolo_frame_stamp = None

            if frame is None or frame_stamp is None:
                continue

            # Throttle to max_rate_hz
            if self.last_yolo_stamp is not None:
                elapsed = frame_stamp - self.last_yolo_stamp
                if 0.0 <= elapsed < self.min_yolo_period_sec:
                    continue

            self.last_yolo_stamp = frame_stamp

            # Run inference on CUDA with person class (0)
            try:
                results = self.model.predict(
                    source=frame,
                    device=self.device,
                    imgsz=self.imgsz,
                    conf=self.conf_threshold,
                    classes=[0],
                    verbose=False,
                )
            except Exception as exc:
                self.get_logger().warning(f"YOLO inference error: {exc}")
                continue

            if not results:
                continue

            res = results[0]
            boxes = getattr(res, 'boxes', None)
            if boxes is None or len(boxes) == 0:
                continue

            for box in boxes:
                try:
                    conf = float(box.conf.item() if hasattr(box.conf, 'item') else box.conf[0])
                    if conf < self.conf_threshold:
                        continue

                    xyxy = box.xyxy[0].tolist() if hasattr(box.xyxy, 'tolist') else list(box.xyxy[0])
                    x1, y1, x2, y2 = xyxy

                    self._publish_survivor_detection(
                        frame=frame,
                        image_stamp=frame_stamp,
                        confidence=conf,
                        x1=x1,
                        y1=y1,
                        x2=x2,
                        y2=y2,
                    )
                except Exception as exc:
                    self.get_logger().warning(f"Error processing YOLO detection box: {exc}")

    def _publish_survivor_detection(
        self,
        frame: np.ndarray,
        image_stamp: float,
        confidence: float,
        x1: float,
        y1: float,
        x2: float,
        y2: float,
    ):
        """
        Construct and publish SurvivorDetection message.
        """
        h, w = frame.shape[:2]
        x1_c = max(0.0, min(float(x1), float(w - 1)))
        y1_c = max(0.0, min(float(y1), float(h - 1)))
        x2_c = max(0.0, min(float(x2), float(w)))
        y2_c = max(0.0, min(float(y2), float(h)))

        if x2_c <= x1_c or y2_c <= y1_c:
            return

        bbox_w = x2_c - x1_c
        bbox_h = y2_c - y1_c
        center_u = (x1_c + x2_c) * 0.5
        center_v = (y1_c + y2_c) * 0.5

        # Save detection crop
        self.detection_counter += 1
        stamp_ns = int(image_stamp * 1.0e9)
        crop_path = os.path.join(self.detection_dir, f"survivor_{stamp_ns}_{self.detection_counter}.jpg")
        crop = frame[int(y1_c):int(y2_c), int(x1_c):int(x2_c)]

        if crop.size > 0 and crop.shape[0] > 0 and crop.shape[1] > 0:
            try:
                cv2.imwrite(crop_path, crop)
            except Exception as exc:
                self.get_logger().warning(f"Failed to write detection crop: {exc}")
                crop_path = ''

        # Compute 3D world position
        world_pt = self._calculate_world_position(center_u, center_v, image_stamp, w, h)

        msg = SurvivorDetection()
        msg.header.stamp = self._sec_to_stamp(image_stamp)
        msg.header.frame_id = 'base_link'
        msg.confidence = float(confidence)
        msg.bbox_x = float(x1_c)
        msg.bbox_y = float(y1_c)
        msg.bbox_w = float(bbox_w)
        msg.bbox_h = float(bbox_h)
        msg.world_position = world_pt if world_pt is not None else Point()
        msg.image_path = crop_path

        self.detection_pub.publish(msg)

    def _calculate_world_position(
        self,
        center_u: float,
        center_v: float,
        image_stamp: float,
        image_width: int,
        image_height: int,
    ) -> Optional[Point]:
        """
        Calculate ground survivor position in odom frame.
        """
        with self.state_lock:
            camera_ready = self.camera_info_ready
            intrinsics = self.intrinsics
            altitude = self.latest_altitude
            altitude_stamp = self.latest_altitude_stamp

        if not camera_ready or intrinsics is None:
            return None

        # Verify ToF freshness (<= 0.5s)
        if (
            altitude is None
            or altitude_stamp is None
            or not (0.0 <= image_stamp - altitude_stamp <= 0.5)
            or altitude <= 0.0
            or not math.isfinite(altitude)
        ):
            return None

        # Verify odometry freshness (<= 0.5s)
        odom = self._get_fresh_odom()
        if odom is None:
            return None

        pos = odom.pose.pose.position
        orient = odom.pose.pose.orientation

        coords = self.world_projector.project_bbox_to_world(
            center_u=center_u,
            center_v=center_v,
            altitude=altitude,
            intrinsics=intrinsics,
            drone_x=float(pos.x),
            drone_y=float(pos.y),
            drone_z=float(pos.z),
            drone_qx=float(orient.x),
            drone_qy=float(orient.y),
            drone_qz=float(orient.z),
            drone_qw=float(orient.w),
            image_width=image_width,
            image_height=image_height,
        )

        if coords is None:
            return None

        pt = Point()
        pt.x, pt.y, pt.z = coords
        return pt

    def _get_fresh_odom(self) -> Optional[Odometry]:
        """
        Return odometry from /odometry/filtered if age <= odom_staleness_sec.
        """
        with self.state_lock:
            odom = self.latest_odom
            odom_stamp = self.latest_odom_stamp

        if odom is None or odom_stamp is None:
            if not self.odom_stale_logged:
                self.get_logger().warning("No /odometry/filtered available; skipping world_position.")
                self.odom_stale_logged = True
            return None

        now_sec = self.stamp_to_sec(self.get_clock().now().to_msg())
        age_sec = now_sec - odom_stamp

        if not math.isfinite(age_sec) or age_sec < 0.0 or age_sec > self.odom_staleness_sec:
            if not self.odom_stale_logged:
                self.get_logger().warning(
                    f"Stale /odometry/filtered (age={age_sec:.3f}s, limit={self.odom_staleness_sec:.3f}s); "
                    "skipping world_position."
                )
                self.odom_stale_logged = True
            return None

        return odom

    # =============================================================
    # Helpers
    # =============================================================

    @staticmethod
    def stamp_to_sec(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9

    @staticmethod
    def _sec_to_stamp(seconds: float):
        from builtin_interfaces.msg import Time
        stamp = Time()
        if seconds < 0.0:
            seconds = 0.0
        sec = int(math.floor(seconds))
        nanosec = int(round((seconds - sec) * 1.0e9))
        if nanosec >= 1_000_000_000:
            sec += 1
            nanosec -= 1_000_000_000
        stamp.sec = sec
        stamp.nanosec = nanosec
        return stamp

    def destroy_node(self):
        self.yolo_stop_event.set()
        with self.yolo_condition:
            self.yolo_condition.notify_all()
        if self.yolo_thread.is_alive():
            self.yolo_thread.join(timeout=1.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()