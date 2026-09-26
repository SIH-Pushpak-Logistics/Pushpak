"""
Pure-Python perception core math and algorithms for Pushpak perception pipeline.

This module contains NO rclpy or ROS dependencies.
All ROS-specific communication and message conversion is handled in perception_node.py.
"""

from dataclasses import dataclass
import math
from typing import List, Optional, Tuple
import cv2
import numpy as np


@dataclass(frozen=True)
class CameraIntrinsics:
    """
    Pinhole camera intrinsic parameters.

    Attributes:
        fx: Focal length in x (pixels)
        fy: Focal length in y (pixels)
        cx: Principal point x (pixels)
        cy: Principal point y (pixels)
        width: Image width at calibration resolution
        height: Image height at calibration resolution
    """
    fx: float
    fy: float
    cx: float
    cy: float
    width: int = 0
    height: int = 0

    def scale_to_dimensions(self, new_width: int, new_height: int) -> "CameraIntrinsics":
        """
        Scale intrinsics when operating at a different image resolution.

        Focal lengths and principal points scale proportionally with resolution.
        If target dimensions are non-positive or match current, returns unscaled intrinsics.
        """
        if (
            new_width <= 0
            or new_height <= 0
            or self.width <= 0
            or self.height <= 0
            or (self.width == new_width and self.height == new_height)
        ):
            return CameraIntrinsics(
                fx=self.fx,
                fy=self.fy,
                cx=self.cx,
                cy=self.cy,
                width=self.width if self.width > 0 else new_width,
                height=self.height if self.height > 0 else new_height,
            )

        sx = float(new_width) / float(self.width)
        sy = float(new_height) / float(self.height)

        return CameraIntrinsics(
            fx=self.fx * sx,
            fy=self.fy * sy,
            cx=self.cx * sx,
            cy=self.cy * sy,
            width=new_width,
            height=new_height,
        )


@dataclass(frozen=True)
class ImageQualityResult:
    """
    Result of image quality evaluation.

    Attributes:
        variance: Raw Laplacian variance
        ratio: Ratio of variance to the clean baseline
        is_degraded: True if ratio is below the degradation threshold
    """
    variance: float
    ratio: float
    is_degraded: bool


class ImageQualityEvaluator:
    """
    Laplacian image quality evaluator for dust/blur degradation detection.

    Degradation is strictly evaluated as a ratio against the clean baseline,
    preserving scale invariance across scenes.
    """

    def __init__(
        self,
        baseline: float = 453.151,
        ratio_threshold: float = 0.30,
    ):
        self.baseline = float(baseline)
        self.ratio_threshold = float(ratio_threshold)

    def evaluate(self, gray: np.ndarray) -> ImageQualityResult:
        """
        Evaluate image sharpness via Laplacian variance.

        Args:
            gray: Grayscale image (uint8)

        Returns:
            ImageQualityResult with variance, baseline ratio, and degradation flag.
        """
        if gray is None or gray.size == 0:
            return ImageQualityResult(variance=0.0, ratio=0.0, is_degraded=True)

        try:
            laplacian = cv2.Laplacian(gray, cv2.CV_64F)
            variance = float(laplacian.var())
        except (cv2.error, ValueError):
            return ImageQualityResult(variance=0.0, ratio=0.0, is_degraded=True)

        if not math.isfinite(variance) or variance < 0.0 or self.baseline <= 0.0:
            return ImageQualityResult(variance=0.0, ratio=0.0, is_degraded=True)

        ratio = variance / self.baseline
        is_degraded = (not math.isfinite(ratio)) or (ratio < self.ratio_threshold)

        return ImageQualityResult(
            variance=variance,
            ratio=ratio,
            is_degraded=is_degraded,
        )


class CovarianceRamp:
    """
    Logarithmic covariance ramping over a fixed window of frames.

    Provides continuous transition between nominal and degraded covariance:
    - Nominal: 0.05
    - Degraded: 1.0e6
    - Ramp window: 5 frames
    """

    def __init__(
        self,
        nominal_covariance: float = 0.05,
        degraded_covariance: float = 1.0e6,
        ramp_frames: int = 5,
    ):
        self.nominal_covariance = float(nominal_covariance)
        self.degraded_covariance = float(degraded_covariance)
        self.ramp_frames = int(ramp_frames)

        self.ramp_index = 0
        self.current_covariance = self.nominal_covariance

    def update(self, is_degraded: bool) -> float:
        """
        Update the ramp state by one frame.

        Args:
            is_degraded: True if current frame is degraded or invalid.

        Returns:
            Current interpolated covariance value.
        """
        if is_degraded:
            self.ramp_index = min(self.ramp_index + 1, self.ramp_frames)
        else:
            self.ramp_index = max(self.ramp_index - 1, 0)

        alpha = float(self.ramp_index) / float(self.ramp_frames)

        log_nominal = math.log10(self.nominal_covariance)
        log_degraded = math.log10(self.degraded_covariance)

        covariance = 10.0 ** (log_nominal + alpha * (log_degraded - log_nominal))
        self.current_covariance = covariance

        return covariance

    @staticmethod
    def build_covariance_matrix(
        covariance_xy: float,
        unmeasured_covariance: float = 1.0e6,
    ) -> List[float]:
        """
        Construct a 36-element covariance matrix (row-major 6x6) for TwistWithCovariance.

        Linear x and y receive covariance_xy; unmeasured states receive unmeasured_covariance.
        """
        cov = [0.0] * 36
        cov[0] = float(covariance_xy)
        cov[7] = float(covariance_xy)
        cov[14] = float(unmeasured_covariance)
        cov[21] = float(unmeasured_covariance)
        cov[28] = float(unmeasured_covariance)
        cov[35] = float(unmeasured_covariance)
        return cov


@dataclass(frozen=True)
class OpticalFlowResult:
    """
    Result of optical flow velocity computation.

    Attributes:
        vx: Body-frame forward linear velocity (m/s)
        vy: Body-frame leftward linear velocity (m/s)
        feature_count: Number of valid tracked feature points
        is_valid: True if velocity is finite and feature_count >= threshold
    """
    vx: float
    vy: float
    feature_count: int
    is_valid: bool


class OpticalFlowTracker:
    """
    Lucas-Kanade optical flow tracker with gyroscope de-rotation and pinhole scaling.

    Downward-facing camera geometry:
        Image rows (v axis) -> body X (forward)
        Image cols (u axis) -> body Y (leftward)
    """

    def __init__(
        self,
        max_corners: int = 250,
        quality_level: float = 0.02,
        min_distance: float = 7.0,
        block_size: int = 7,
        min_features_valid: int = 15,
        reseed_threshold: int = 50,
    ):
        self.max_corners = max_corners
        self.quality_level = quality_level
        self.min_distance = min_distance
        self.block_size = block_size
        self.min_features_valid = min_features_valid
        self.reseed_threshold = reseed_threshold

        self.prev_gray: Optional[np.ndarray] = None
        self.prev_points: Optional[np.ndarray] = None
        self.prev_timestamp_sec: Optional[float] = None

    def reinit_features(self, gray: np.ndarray) -> int:
        """
        Detect fresh Shi-Tomasi corners on the given grayscale frame.
        """
        self.prev_gray = gray

        if gray is None or gray.size == 0:
            self.prev_points = None
            return 0

        points = cv2.goodFeaturesToTrack(
            gray,
            maxCorners=self.max_corners,
            qualityLevel=self.quality_level,
            minDistance=self.min_distance,
            blockSize=self.block_size,
        )

        if points is None or len(points) == 0:
            self.prev_points = None
            return 0

        self.prev_points = points.reshape(-1, 1, 2)
        return len(points)

    def process_frame(
        self,
        gray: np.ndarray,
        timestamp_sec: float,
        altitude: float,
        gyro_x: float,
        gyro_y: float,
        intrinsics: CameraIntrinsics,
    ) -> OpticalFlowResult:
        """
        Calculate body-frame vx and vy from optical flow with gyro de-rotation.

        Args:
            gray: Current grayscale frame (uint8)
            timestamp_sec: ROS time stamp in seconds
            altitude: Ground distance in meters (from ToF)
            gyro_x: Angular velocity about body X axis (rad/s)
            gyro_y: Angular velocity about body Y axis (rad/s)
            intrinsics: CameraIntrinsics containing fx, fy, cx, cy

        Returns:
            OpticalFlowResult containing vx, vy, feature_count, and validity flag.
        """
        # First frame initializes tracker
        if self.prev_timestamp_sec is None:
            self.prev_timestamp_sec = timestamp_sec
            count = self.reinit_features(gray)
            return OpticalFlowResult(vx=0.0, vy=0.0, feature_count=count, is_valid=False)

        dt = timestamp_sec - self.prev_timestamp_sec
        self.prev_timestamp_sec = timestamp_sec

        if dt <= 0.0 or not math.isfinite(dt):
            count = self.reinit_features(gray)
            return OpticalFlowResult(vx=0.0, vy=0.0, feature_count=count, is_valid=False)

        if self.prev_gray is None or self.prev_points is None or len(self.prev_points) == 0:
            count = self.reinit_features(gray)
            return OpticalFlowResult(vx=0.0, vy=0.0, feature_count=count, is_valid=False)

        # Scale intrinsics to current image dimensions if known
        h, w = gray.shape[:2]
        effective_intrinsics = intrinsics.scale_to_dimensions(w, h)
        fx = effective_intrinsics.fx
        fy = effective_intrinsics.fy

        if fx <= 0.0 or fy <= 0.0:
            count = self.reinit_features(gray)
            return OpticalFlowResult(vx=0.0, vy=0.0, feature_count=count, is_valid=False)

        # Calculate optical flow via Lucas-Kanade
        try:
            next_points, status, _ = cv2.calcOpticalFlowPyrLK(
                self.prev_gray,
                gray,
                self.prev_points,
                None,
                winSize=(21, 21),
                maxLevel=3,
                criteria=(
                    cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT,
                    30,
                    0.01,
                ),
            )
        except cv2.error:
            count = self.reinit_features(gray)
            return OpticalFlowResult(vx=0.0, vy=0.0, feature_count=count, is_valid=False)

        if next_points is None or status is None:
            count = self.reinit_features(gray)
            return OpticalFlowResult(vx=0.0, vy=0.0, feature_count=count, is_valid=False)

        valid_mask = status.flatten() == 1
        good_new = next_points[valid_mask].reshape(-1, 2)
        good_old = self.prev_points[valid_mask].reshape(-1, 2)
        feature_count = len(good_new)

        if feature_count == 0:
            count = self.reinit_features(gray)
            return OpticalFlowResult(vx=0.0, vy=0.0, feature_count=count, is_valid=False)

        dx = good_new[:, 0] - good_old[:, 0]
        dy = good_new[:, 1] - good_old[:, 1]

        u_raw = float(dx.mean())
        v_raw = float(dy.mean())

        # Gyroscope de-rotation (apparent optical flow induced by body rotation)
        u_rot = -gyro_x * dt * fx
        v_rot = -gyro_y * dt * fy

        # Pure translational flow
        u_trans = u_raw - u_rot
        v_trans = v_raw - v_rot

        # Pinhole velocity scaling (downward-facing camera geometry)
        vx = (v_trans * altitude) / (fy * dt)
        vy = (u_trans * altitude) / (fx * dt)

        # Update tracked history
        self.prev_gray = gray
        if feature_count < self.reseed_threshold:
            new_count = self.reinit_features(gray)
            if new_count == 0:
                self.prev_points = good_new.reshape(-1, 1, 2)
        else:
            self.prev_points = good_new.reshape(-1, 1, 2)

        is_valid = (
            feature_count >= self.min_features_valid
            and math.isfinite(vx)
            and math.isfinite(vy)
        )

        return OpticalFlowResult(
            vx=float(vx),
            vy=float(vy),
            feature_count=feature_count,
            is_valid=is_valid,
        )


class WorldProjector:
    """
    Project 2D image pixel coordinates into 3D world coordinates (odom frame).

    Downward-facing camera geometry:
        Top of image (v < cy) is forward (+X)
        Left of image (u < cx) is left (+Y)
    """

    @staticmethod
    def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
        """
        Compute yaw angle (radians) from orientation quaternion.
        """
        sin_yaw = 2.0 * (w * z + x * y)
        cos_yaw = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(sin_yaw, cos_yaw)

    @classmethod
    def project_bbox_to_world(
        cls,
        center_u: float,
        center_v: float,
        altitude: float,
        intrinsics: CameraIntrinsics,
        drone_x: float,
        drone_y: float,
        drone_z: float,
        drone_qx: float,
        drone_qy: float,
        drone_qz: float,
        drone_qw: float,
        image_width: int = 0,
        image_height: int = 0,
    ) -> Optional[Tuple[float, float, float]]:
        """
        Calculate ground point position in odom frame from image pixel coordinates.

        Args:
            center_u: Pixel x coordinate (column)
            center_v: Pixel y coordinate (row)
            altitude: Ground distance in meters (from ToF)
            intrinsics: Camera intrinsics
            drone_x, drone_y, drone_z: Drone position in odom frame
            drone_qx, drone_qy, drone_qz, drone_qw: Drone orientation quaternion
            image_width, image_height: Image dimensions if scaling is needed

        Returns:
            (world_x, world_y, world_z) in odom frame, or None if inputs are invalid.
        """
        if altitude <= 0.0 or not math.isfinite(altitude):
            return None

        # Scale intrinsics to image dimensions if provided
        effective_intrinsics = intrinsics.scale_to_dimensions(image_width, image_height)
        fx = effective_intrinsics.fx
        fy = effective_intrinsics.fy
        cx = effective_intrinsics.cx
        cy = effective_intrinsics.cy

        if fx <= 0.0 or fy <= 0.0:
            return None

        # Downward-facing camera body offsets:
        # Top of image (v < cy) -> +X (forward)
        # Left of image (u < cx) -> +Y (leftward)
        body_x = (cy - center_v) * altitude / fy
        body_y = (cx - center_u) * altitude / fx

        yaw = cls.quaternion_to_yaw(drone_qx, drone_qy, drone_qz, drone_qw)
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        world_x = drone_x + cos_yaw * body_x - sin_yaw * body_y
        world_y = drone_y + sin_yaw * body_x + cos_yaw * body_y
        world_z = drone_z - altitude

        if not (math.isfinite(world_x) and math.isfinite(world_y) and math.isfinite(world_z)):
            return None

        return (float(world_x), float(world_y), float(world_z))