"""
Unit tests for navigation_brain.perception_core.

Tests all pure-Python mathematical functions and classes without ROS dependencies:
- CameraIntrinsics scaling
- ImageQualityEvaluator ratio math and degradation flags
- CovarianceRamp logarithmic transitions and matrix generation
- WorldProjector downward-camera projection and yaw rotations
- OpticalFlowTracker Shi-Tomasi tracking, gyro de-rotation, and pinhole scaling
"""

import math
import numpy as np
import pytest

from navigation_brain.perception_core import (
    CameraIntrinsics,
    CovarianceRamp,
    ImageQualityEvaluator,
    OpticalFlowTracker,
    WorldProjector,
)


class TestCameraIntrinsics:
    """Tests for CameraIntrinsics dataclass and scaling logic."""

    def test_initialization(self):
        intrinsics = CameraIntrinsics(fx=277.0, fy=277.0, cx=160.0, cy=120.0, width=320, height=240)
        assert intrinsics.fx == 277.0
        assert intrinsics.fy == 277.0
        assert intrinsics.cx == 160.0
        assert intrinsics.cy == 120.0
        assert intrinsics.width == 320
        assert intrinsics.height == 240

    def test_scaling_proportional(self):
        intrinsics = CameraIntrinsics(fx=200.0, fy=200.0, cx=100.0, cy=100.0, width=200, height=200)
        # Double resolution to 400x400
        scaled = intrinsics.scale_to_dimensions(400, 400)
        assert scaled.fx == 400.0
        assert scaled.fy == 400.0
        assert scaled.cx == 200.0
        assert scaled.cy == 200.0
        assert scaled.width == 400
        assert scaled.height == 400

    def test_scaling_same_dimensions(self):
        intrinsics = CameraIntrinsics(fx=200.0, fy=200.0, cx=100.0, cy=100.0, width=200, height=200)
        scaled = intrinsics.scale_to_dimensions(200, 200)
        assert scaled == intrinsics

    def test_scaling_zero_dimensions_noop(self):
        intrinsics = CameraIntrinsics(fx=200.0, fy=200.0, cx=100.0, cy=100.0, width=0, height=0)
        scaled = intrinsics.scale_to_dimensions(400, 400)
        assert scaled.fx == 200.0
        assert scaled.fy == 200.0


class TestImageQualityEvaluator:
    """Tests for Laplacian degradation evaluation as a ratio to baseline."""

    def test_clean_synthetic_texture(self):
        evaluator = ImageQualityEvaluator(baseline=100.0, ratio_threshold=0.30)
        # High frequency checkerboard pattern
        img = np.zeros((100, 100), dtype=np.uint8)
        img[::2, ::2] = 255
        img[1::2, 1::2] = 255

        result = evaluator.evaluate(img)
        assert result.variance > 100.0
        assert result.ratio > 1.0
        assert not result.is_degraded

    def test_degraded_uniform_image(self):
        evaluator = ImageQualityEvaluator(baseline=453.151, ratio_threshold=0.30)
        # Perfectly uniform gray image (zero Laplacian variance)
        uniform_img = np.full((120, 160), 128, dtype=np.uint8)

        result = evaluator.evaluate(uniform_img)
        assert result.variance == 0.0
        assert result.ratio == 0.0
        assert result.is_degraded

    def test_degraded_ratio_below_threshold(self):
        evaluator = ImageQualityEvaluator(baseline=1000.0, ratio_threshold=0.30)
        # Low variance image
        low_var_img = np.zeros((100, 100), dtype=np.uint8)
        low_var_img[50:, :] = 10  # very slight edge

        result = evaluator.evaluate(low_var_img)
        assert result.ratio < 0.30
        assert result.is_degraded

    def test_empty_or_none_image(self):
        evaluator = ImageQualityEvaluator()
        result = evaluator.evaluate(None)
        assert result.is_degraded
        assert result.variance == 0.0

        empty_img = np.array([], dtype=np.uint8)
        result = evaluator.evaluate(empty_img)
        assert result.is_degraded


class TestCovarianceRamp:
    """Tests for 5-frame logarithmic covariance ramping."""

    def test_initial_nominal(self):
        ramp = CovarianceRamp(nominal_covariance=0.05, degraded_covariance=1.0e6, ramp_frames=5)
        assert ramp.current_covariance == 0.05

    def test_degradation_five_step_ramp(self):
        ramp = CovarianceRamp(nominal_covariance=0.05, degraded_covariance=1.0e6, ramp_frames=5)
        covs = [ramp.update(is_degraded=True) for _ in range(5)]

        # Must strictly increase monotonically
        for i in range(len(covs) - 1):
            assert covs[i] < covs[i + 1]

        # Final step must equal 1.0e6
        assert math.isclose(covs[-1], 1.0e6, rel_tol=1e-5)

        # Step 6 stays clamped at 1.0e6
        cov6 = ramp.update(is_degraded=True)
        assert math.isclose(cov6, 1.0e6, rel_tol=1e-5)

    def test_recovery_five_step_ramp(self):
        ramp = CovarianceRamp(nominal_covariance=0.05, degraded_covariance=1.0e6, ramp_frames=5)
        # Degrade fully
        for _ in range(5):
            ramp.update(is_degraded=True)

        # Recover over 5 steps
        recovery = [ramp.update(is_degraded=False) for _ in range(5)]
        for i in range(len(recovery) - 1):
            assert recovery[i] > recovery[i + 1]

        assert math.isclose(recovery[-1], 0.05, rel_tol=1e-5)

    def test_covariance_matrix_structure(self):
        matrix = CovarianceRamp.build_covariance_matrix(covariance_xy=0.05, unmeasured_covariance=1.0e6)
        assert len(matrix) == 36
        assert matrix[0] == 0.05
        assert matrix[7] == 0.05
        assert matrix[14] == 1.0e6
        assert matrix[21] == 1.0e6
        assert matrix[28] == 1.0e6
        assert matrix[35] == 1.0e6
        # Off-diagonal elements must be zero
        assert matrix[1] == 0.0
        assert matrix[6] == 0.0


class TestWorldProjector:
    """Tests for 2D bbox to 3D world/odom projection math."""

    @pytest.fixture
    def camera_intrinsics(self):
        return CameraIntrinsics(fx=200.0, fy=200.0, cx=100.0, cy=100.0, width=200, height=200)

    def test_nadir_center_projection(self, camera_intrinsics):
        # Target at optical center (100, 100), drone at (0, 0, 2), altitude 2.0, zero yaw
        pt = WorldProjector.project_bbox_to_world(
            center_u=100.0,
            center_v=100.0,
            altitude=2.0,
            intrinsics=camera_intrinsics,
            drone_x=0.0,
            drone_y=0.0,
            drone_z=2.0,
            drone_qx=0.0,
            drone_qy=0.0,
            drone_qz=0.0,
            drone_qw=1.0,
        )
        assert pt is not None
        x, y, z = pt
        assert math.isclose(x, 0.0, abs_tol=1e-4)
        assert math.isclose(y, 0.0, abs_tol=1e-4)
        assert math.isclose(z, 0.0, abs_tol=1e-4)

    def test_forward_left_projection(self, camera_intrinsics):
        # Top of image (v = 50 < cy = 100) -> body +X
        # Left of image (u = 50 < cx = 100) -> body +Y
        # altitude = 2.0, fy = 200 => body_x = (100 - 50) * 2 / 200 = 0.5
        # altitude = 2.0, fx = 200 => body_y = (100 - 50) * 2 / 200 = 0.5
        pt = WorldProjector.project_bbox_to_world(
            center_u=50.0,
            center_v=50.0,
            altitude=2.0,
            intrinsics=camera_intrinsics,
            drone_x=10.0,
            drone_y=20.0,
            drone_z=2.0,
            drone_qx=0.0,
            drone_qy=0.0,
            drone_qz=0.0,
            drone_qw=1.0,
        )
        assert pt is not None
        x, y, z = pt
        assert math.isclose(x, 10.5, abs_tol=1e-4)
        assert math.isclose(y, 20.5, abs_tol=1e-4)
        assert math.isclose(z, 0.0, abs_tol=1e-4)

    def test_yaw_rotation_ninety_deg(self, camera_intrinsics):
        # 90 degrees yaw: qz = sin(pi/4), qw = cos(pi/4)
        qz = math.sin(math.pi / 4.0)
        qw = math.cos(math.pi / 4.0)
        yaw = WorldProjector.quaternion_to_yaw(0.0, 0.0, qz, qw)
        assert math.isclose(yaw, math.pi / 2.0, abs_tol=1e-5)

        # Target forward in body (body_x = 0.5, body_y = 0.0)
        # At yaw = +90 deg, body forward (+X) maps to world +Y
        pt = WorldProjector.project_bbox_to_world(
            center_u=100.0,
            center_v=50.0,
            altitude=2.0,
            intrinsics=camera_intrinsics,
            drone_x=0.0,
            drone_y=0.0,
            drone_z=2.0,
            drone_qx=0.0,
            drone_qy=0.0,
            drone_qz=qz,
            drone_qw=qw,
        )
        assert pt is not None
        x, y, z = pt
        assert math.isclose(x, 0.0, abs_tol=1e-4)
        assert math.isclose(y, 0.5, abs_tol=1e-4)

    def test_invalid_altitude_returns_none(self, camera_intrinsics):
        pt = WorldProjector.project_bbox_to_world(
            center_u=100.0,
            center_v=100.0,
            altitude=-1.0,
            intrinsics=camera_intrinsics,
            drone_x=0.0,
            drone_y=0.0,
            drone_z=0.0,
            drone_qx=0.0,
            drone_qy=0.0,
            drone_qz=0.0,
            drone_qw=1.0,
        )
        assert pt is None


class TestOpticalFlowTracker:
    """Tests for Lucas-Kanade optical flow, de-rotation, and velocity scaling."""

    @pytest.fixture
    def intrinsics(self):
        return CameraIntrinsics(fx=200.0, fy=200.0, cx=100.0, cy=100.0, width=200, height=200)

    @pytest.fixture
    def textured_frame(self):
        # Generate synthetic texture with distinct features
        np.random.seed(42)
        frame = np.random.randint(0, 255, (200, 200), dtype=np.uint8)
        # Blur slightly so gradient flow works well
        import cv2
        frame = cv2.GaussianBlur(frame, (5, 5), 1.0)
        return frame

    def test_first_frame_initializes(self, intrinsics, textured_frame):
        tracker = OpticalFlowTracker()
        res = tracker.process_frame(
            gray=textured_frame,
            timestamp_sec=1.0,
            altitude=2.0,
            gyro_x=0.0,
            gyro_y=0.0,
            intrinsics=intrinsics,
        )
        assert res.vx == 0.0
        assert res.vy == 0.0
        assert not res.is_valid
        assert res.feature_count > 0

    def test_static_frame_zero_velocity(self, intrinsics, textured_frame):
        tracker = OpticalFlowTracker()
        tracker.process_frame(textured_frame, 1.0, 2.0, 0.0, 0.0, intrinsics)
        # Feed identical frame 0.05s later (zero displacement)
        res = tracker.process_frame(textured_frame, 1.05, 2.0, 0.0, 0.0, intrinsics)
        assert res.is_valid
        assert math.isclose(res.vx, 0.0, abs_tol=0.01)
        assert math.isclose(res.vy, 0.0, abs_tol=0.01)

    def test_gyro_derotation_cancellation(self, intrinsics, textured_frame):
        """
        Pure rotation: if camera rolls/pitches, the apparent flow must be canceled
        by gyro de-rotation, leaving true translational velocity near zero.
        """
        tracker = OpticalFlowTracker()
        tracker.process_frame(textured_frame, 1.0, 2.0, 0.0, 0.0, intrinsics)

        dt = 0.05
        gyro_x = 0.2  # rad/s roll
        # Simulated roll creates displacement in image columns: dx = -gyro_x * dt * fx
        dx_expected = -gyro_x * dt * intrinsics.fx  # = -0.2 * 0.05 * 200 = -2 pixels

        import cv2
        M = np.float32([[1, 0, dx_expected], [0, 1, 0]])
        rolled_frame = cv2.warpAffine(textured_frame, M, (200, 200))

        res = tracker.process_frame(
            gray=rolled_frame,
            timestamp_sec=1.0 + dt,
            altitude=2.0,
            gyro_x=gyro_x,
            gyro_y=0.0,
            intrinsics=intrinsics,
        )
        assert res.is_valid
        # With gyro de-rotation, vy should be canceled out near zero
        assert abs(res.vy) < 0.05
    def test_translation_positive_vx_vy(self, intrinsics, textured_frame):
        """
        Translating image: moving camera forward produces backward feature movement in image.
        Pinhole downward camera: image rows -> body X (+v_trans -> +vx).
        """
        tracker = OpticalFlowTracker()
        tracker.process_frame(textured_frame, 1.0, 2.0, 0.0, 0.0, intrinsics)

        dt = 0.05
        # Shift image down by 2 pixels (+dy = +2) and right by 2 pixels (+dx = +2)
        import cv2
        M = np.float32([[1, 0, 2], [0, 1, 2]])
        shifted = cv2.warpAffine(textured_frame, M, (200, 200))

        res = tracker.process_frame(shifted, 1.0 + dt, 2.0, 0.0, 0.0, intrinsics)
        assert res.is_valid
        # vx = (v_trans * altitude) / (fy * dt) = (2 * 2.0) / (200 * 0.05) = 4 / 10 = 0.4 m/s
        assert math.isclose(res.vx, 0.4, abs_tol=0.08)
        assert math.isclose(res.vy, 0.4, abs_tol=0.08)

    def test_resolution_scale_invariance_in_projection(self):
        """
        World projection should yield the same physical world position
        whether given 160x120 calibration intrinsics scaled to 320x240,
        or normalized coordinates.
        """
        # Calibration at 160x120
        calib_intrinsics = CameraIntrinsics(fx=138.6, fy=138.6, cx=80.0, cy=60.0, width=160, height=120)

        # Target at center on 320x240 image (160, 120)
        pt320 = WorldProjector.project_bbox_to_world(
            center_u=160.0,
            center_v=120.0,
            altitude=1.5,
            intrinsics=calib_intrinsics,
            drone_x=5.0,
            drone_y=5.0,
            drone_z=1.5,
            drone_qx=0.0,
            drone_qy=0.0,
            drone_qz=0.0,
            drone_qw=1.0,
            image_width=320,
            image_height=240,
        )
        assert pt320 is not None
        assert math.isclose(pt320[0], 5.0, abs_tol=1e-4)
        assert math.isclose(pt320[1], 5.0, abs_tol=1e-4)
        assert math.isclose(pt320[2], 0.0, abs_tol=1e-4)