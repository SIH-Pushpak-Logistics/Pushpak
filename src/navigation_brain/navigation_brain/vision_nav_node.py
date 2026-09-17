#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge, CvBridgeError
import cv2
import message_filters
from rclpy.qos import qos_profile_sensor_data
from swarm_utils.redis_bridge import RedisTelemetryPublisher

class VisionNavigationNode(Node):
    def __init__(self):
        super().__init__('vision_nav_node')
        
        self.declare_parameter('drone_id', 'drone_00')
        self.drone_id = self.get_parameter('drone_id').get_parameter_value().string_value
        self.get_logger().info(f'Initializing Vision Navigation Node for {self.drone_id}...')

        self.bridge = CvBridge()
        self.prev_gray = None
        self.prev_points = None
        self.prev_timestamp_sec = None

        self.hfov = 1.047
        self.fx = 138.56
        self.fy = 138.56

        self.latest_altitude = None
        self.latest_altitude_stamp = None
        self.altitude_max_age = 0.5
        self.min_usable_altitude = 0.15
        self.latest_gyro_x = 0.0
        self.latest_gyro_y = 0.0

        self.redis_publisher = RedisTelemetryPublisher(
            stream_name=f'telemetry:{self.drone_id}:velocity', 
            logger=self.get_logger()
        )

        self.debug_publisher = RedisTelemetryPublisher(
            stream_name=f'telemetry:{self.drone_id}:flow_debug',
            logger=self.get_logger()
        )

        self.camera_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, qos_profile_sensor_data
        )
        self.altitude_sub = self.create_subscription(
            PointStamped, '/drone/altitude', self.altitude_callback, qos_profile_sensor_data
        )
        self.imu_sub = self.create_subscription(
            Imu, '/mavros/imu/data', self.imu_callback, qos_profile_sensor_data
        )

    def altitude_callback(self, msg):
        self.latest_altitude = float(msg.point.z)
        self.latest_altitude_stamp = (
            msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        )

    def usable_altitude(self, image_stamp_sec):
        if self.latest_altitude is None or self.latest_altitude_stamp is None:
            return None
        if (image_stamp_sec - self.latest_altitude_stamp) > self.altitude_max_age:
            return None
        if self.latest_altitude < self.min_usable_altitude:
            return None
        return self.latest_altitude

    def imu_callback(self, msg):
        self.latest_gyro_x = float(msg.angular_velocity.x)
        self.latest_gyro_y = float(msg.angular_velocity.y)

    def image_callback(self, image_msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Failure: {str(e)}')
            return

        img_w = cv_image.shape[1]
        self.fx = float(img_w) / (2.0 * math.tan(self.hfov / 2.0))
        self.fy = self.fx

        timestamp_sec = (
            image_msg.header.stamp.sec + 
            image_msg.header.stamp.nanosec * 1e-9
        )

        altitude = self.usable_altitude(timestamp_sec)
        if altitude is None:
            self.prev_gray = None
            self.prev_points = None
            self.prev_timestamp_sec = timestamp_sec
            if self.latest_altitude is not None and (timestamp_sec - self.latest_altitude_stamp) <= self.altitude_max_age:
                self.redis_publisher.send_velocity_vector(
                    self.drone_id, timestamp_sec, 0.0, 0.0, 0.0, 0.0,
                    is_valid=True, features=100
                )
            else:
                self.redis_publisher.send_velocity_vector(
                    self.drone_id, timestamp_sec, 0.0, 0.0, 0.0, 0.0,
                    is_valid=False, features=0
                )
            return

        is_valid, vx, vy, num_features = self.process_vision_pipeline(
            cv_image, altitude, self.latest_gyro_x, self.latest_gyro_y, timestamp_sec
        )

        self.redis_publisher.send_velocity_vector(
            self.drone_id, timestamp_sec, vx, vy, 0.0, 0.0, is_valid=is_valid, features=num_features
        )

    def _reinit_features(self, gray_img):
        self.prev_gray = gray_img
        pts = cv2.goodFeaturesToTrack(
            gray_img, maxCorners=250, qualityLevel=0.02, minDistance=7, blockSize=7
        )
        if pts is not None and len(pts) > 0:
            self.prev_points = pts
            return False, 0.0, 0.0, len(pts)
        else:
            self.prev_points = None
            return False, 0.0, 0.0, 0

    def process_vision_pipeline(self, cv_frame, current_altitude, gyro_x, gyro_y, timestamp_sec):
        gray = cv2.cvtColor(cv_frame, cv2.COLOR_BGR2GRAY)

        if self.prev_timestamp_sec is None:
            self.prev_timestamp_sec = timestamp_sec
            return False, 0.0, 0.0, 0
        
        dt = timestamp_sec - self.prev_timestamp_sec
        self.prev_timestamp_sec = timestamp_sec

        if dt <= 0:
            return False, 0.0, 0.0, 0
        
        if self.prev_gray is None or self.prev_points is None or len(self.prev_points) == 0:
            return self._reinit_features(gray)
            
        next_points, status, error = cv2.calcOpticalFlowPyrLK(
            self.prev_gray, gray, self.prev_points, None,
            winSize=(21, 21), maxLevel=3
        )
        
        if next_points is None or status is None:
            return self._reinit_features(gray)
        
        good_new = next_points[status == 1]
        good_old = self.prev_points[status == 1]
        
        num_features = len(good_new)

        if num_features == 0:
            return self._reinit_features(gray)
        
        dx = good_new[:, 0] - good_old[:, 0]
        dy = good_new[:, 1] - good_old[:, 1]

        u_raw = float(dx.mean())
        v_raw = float(dy.mean())

        frame_diff = float(np.mean(np.abs(
            gray.astype(np.int16) - self.prev_gray.astype(np.int16)
        )))

        self.debug_publisher.send_payload(
            self.drone_id, timestamp_sec,
            u_raw=u_raw, v_raw=v_raw,
            u_med=float(np.median(dx)), v_med=float(np.median(dy)),
            u_std=float(dx.std()), v_std=float(dy.std()),
            frame_diff=frame_diff,
            gyro_x=gyro_x, gyro_y=gyro_y,
            dt=dt, altitude=current_altitude, features=num_features
        )

        # Dimensionally correct gyro de-rotation (omega * dt * f)
        # Pitching down (gyro_y < 0) sweeps features down (+v_raw); subtract with negative sign.
        # Rolling right (gyro_x > 0) sweeps features left (-u_raw); correct accordingly.
        u_rot = -gyro_x * dt * self.fx
        v_rot = -gyro_y * dt * self.fy

        u_trans = u_raw - u_rot
        v_trans = v_raw - v_rot

        # Geometric mapping for downward-looking camera (rpy="0 1.570796 0"):
        # Image row displacement (v) aligns with Body X (Forward).
        # Image col displacement (u) aligns with Body Y (Left/Lateral).
        vx = (v_trans * current_altitude) / (self.fy * dt)
        vy = (u_trans * current_altitude) / (self.fx * dt)

        self.prev_gray = gray
        if num_features < 50:
            pts = cv2.goodFeaturesToTrack(
                gray, maxCorners=250, qualityLevel=0.02, minDistance=7, blockSize=7
            )
            if pts is not None and len(pts) > 0:
                self.prev_points = pts
            else:
                self.prev_points = good_new.reshape(-1, 1, 2)
        else:
            self.prev_points = good_new.reshape(-1, 1, 2)

        return True, vx, vy, num_features

def main(args=None):
    rclpy.init(args=args)
    node = VisionNavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()