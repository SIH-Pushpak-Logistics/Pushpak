#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import Float32
from cv_bridge import CvBridge, CvBridgeError
import cv2
import message_filters
from swarm_utils.redis_bridge import RedisTelemetryPublisher

class VisionNavigationNode(Node):
    def __init__(self):
        super().__init__('vision_nav_node')
        
        # --- Swarm Identity ---
        self.declare_parameter('drone_id', 'drone_00')
        self.drone_id = self.get_parameter('drone_id').get_parameter_value().string_value
        self.get_logger().info(f'Initializing Vision Navigation Node for {self.drone_id}...')

        self.bridge = CvBridge()
        self.prev_gray = None
        self.prev_points = None
        self.prev_timestamp_sec = None

        self.fx = 320.0
        self.fy = 320.0

        # --- Redis Setup ---
        # CONTRACT FIXED: Enforcing strict naming convention for the LQR filter
        self.redis_publisher = RedisTelemetryPublisher(
            stream_name=f'telemetry:{self.drone_id}:raw_optical_flow', 
            logger=self.get_logger()
        )

        # --- Synchronized Inputs ---
        self.camera_sub = message_filters.Subscriber(self, Image, '/camera/image_raw')
        self.altitude_sub = message_filters.Subscriber(self, Float32, '/drone/altitude')
        self.imu_sub = message_filters.Subscriber(self, Imu, '/mavros/imu/data')
        
        # Exact time synchronization (0.05s slop)
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.camera_sub, self.altitude_sub, self.imu_sub], 
            queue_size=2, 
            slop=0.02
        )
        self.ts.registerCallback(self.synchronized_callback)

    def synchronized_callback(self, image_msg, alt_msg, imu_msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Failure: {str(e)}')
            return

        current_altitude = alt_msg.data
        gyro_x = imu_msg.angular_velocity.x
        gyro_y = imu_msg.angular_velocity.y

        timestamp_sec = (
            image_msg.header.stamp.sec + 
            image_msg.header.stamp.nanosec * 1e-9
        )

        is_valid, vx, vy = self.process_vision_pipeline(
            cv_image, current_altitude, gyro_x, gyro_y, timestamp_sec
        )

        self.redis_publisher.send_velocity_vector(
            self.drone_id, timestamp_sec, vx, vy, 0.0, 0.0, is_valid=is_valid
        )

    def _reinit_features(self, gray_img):
        """
        Consolidates feature tracking initialization to avoid DRY violations.
        """
        self.prev_gray = gray_img
        self.prev_points = cv2.goodFeaturesToTrack(
            gray_img, maxCorners=100, qualityLevel=0.3, minDistance=7, blockSize=7
        )
        return False, 0.0, 0.0

    def process_vision_pipeline(self, cv_frame, current_altitude, gyro_x, gyro_y, timestamp_sec):
        gray = cv2.cvtColor(cv_frame, cv2.COLOR_BGR2GRAY)

        # 1. STATE UPDATE: Handle time unconditionally to prevent dt leaks
        if self.prev_timestamp_sec is None:
            self.prev_timestamp_sec = timestamp_sec
            return False, 0.0, 0.0
        
        dt = timestamp_sec - self.prev_timestamp_sec
        self.prev_timestamp_sec = timestamp_sec  # UPDATE IMMEDIATELY

        if dt <= 0:
            return False, 0.0, 0.0
        
        # 2. OPTICAL FLOW EXECUTION
        if self.prev_gray is None:
            return self._reinit_features(gray)
            
        next_points, status, error = cv2.calcOpticalFlowPyrLK(
            self.prev_gray, gray, self.prev_points, None,
            winSize=(21, 21), maxLevel=3
        )
        
        if next_points is None:
            return self._reinit_features(gray)
        
        good_new = next_points[status == 1]
        good_old = self.prev_points[status == 1]

        if len(good_new) == 0:
            return self._reinit_features(gray)
        
        # 3. VELOCITY CALCULATION
        dx = good_new[:, 0] - good_old[:, 0]
        dy = good_new[:, 1] - good_old[:, 1]

        u_raw = dx.mean()
        v_raw = dy.mean()

        # Compensate for rotational movement
        u_translation = u_raw - (gyro_y * self.fx)
        v_translation = v_raw - (gyro_x * self.fy)

        # Convert pixel translation to metric velocity
        vx = (u_translation * current_altitude) / (self.fx * dt)
        vy = (v_translation * current_altitude) / (self.fy * dt)

        # 4. STATE PERSISTENCE
        self.prev_gray = gray
        self.prev_points = good_new.reshape(-1, 1, 2)

        # IDENTITY FIXED: Returning pure physical velocity, not a multiplied command
        return True, vx, vy

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