#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
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

        self.fx = 320.0
        self.fy = 320.0

        self.kp = 0.5

        # --- Redis Setup ---
        self.redis_publisher = RedisTelemetryPublisher(
            stream_name=f'telemetry:{self.drone_id}:velocity', 
            logger=self.get_logger()
        )

        # --- Synchronized Inputs ---
        self.camera_sub = message_filters.Subscriber(self, Image, '/camera/image_raw')
        self.altitude_sub = message_filters.Subscriber(self, Float32, '/drone/altitude')
        self.imu_sub = message_filters.Subscriber(
            self,
            Imu,
            '/mavros/imu/data'
        )
        # Exact time synchronization (0.05s slop)
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [
                self.camera_sub,
                self.altitude_sub,
                self.imu_sub
            ], 
            queue_size=2, 
            slop=0.02
        )
        self.ts.registerCallback(self.synchronized_callback)

    def synchronized_callback(
        self,
        image_msg,
        alt_msg,
        imu_msg
    ):
        """
        Callback only fires when a camera frame and altitude reading match in time.
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Failure: {str(e)}')
            return

        current_altitude = alt_msg.data
        
        gyro_x = imu_msg.angular_velocity.x
        gyro_y = imu_msg.angular_velocity.y

        timestamp_sec = (
            image_msg.header.stamp.sec
            + image_msg.header.stamp.nanosec *1e-9
        )

        is_valid, vx, vy = self.process_vision_pipeline(
            cv_image,
            current_altitude,
            gyro_x,
            gyro_y
        )

        self.redis_publisher.send_velocity_vector(
            self.drone_id,
            timestamp_sec,
            vx,
            vy,
            0.0,
            0.0,
            is_valid=is_valid
        )

    def process_vision_pipeline(
        self,
        cv_frame,
        current_altitude,
        gyro_x,
        gyro_y
    ):
        """
        RAUNAK: Your OpenCV math goes here. 
        You now have current_altitude (Z) perfectly synced with the frame to do your pinhole math.
        """

        # TODO: OpenCV Lucas-Kanade and Pinhole Conversion
        gray = cv2.cvtColor(
            cv_frame,
            cv2.COLOR_BGR2GRAY
        )
        if self.prev_gray is None:
            
            self.prev_gray = gray
                                                                                                
            self.prev_points = cv2.goodFeaturesToTrack(
                gray,
                maxCorners=100,
                qualityLevel=0.3,
                minDistance=7,
                blockSize=7
            )
            return False, 0.0, 0.0
        next_points, status, error = cv2.calcOpticalFlowPyrLK(
            self.prev_gray,
            gray,
            self.prev_points,
            None,
            winSize=(21, 21),
            maxLevel=3
        )
        if next_points is None:

            self.prev_gray = gray 

            self.prev_points = cv2.goodFeaturesToTrack(
                gray,
                maxCorners=100,
                qualityLevel=0.3,
                minDistance=7,
                blockSize=7
            )
            return False, 0.0, 0.0
        
        good_new = next_points[status == 1]
        good_old = self.prev_points[status == 1]

        if len(good_new) == 0:
               
           self.prev_gray = gray

           self.prev_points = cv2.goodFeaturesToTrack(
               gray,
               maxCorners=100,
               qualityLevel=0.3,
               minDistance=7,
               blockSize=7
           )
           return False, 0.0, 0.0
        
        dx = good_new[:, 0] - good_old[:, 0]
        dy = good_new[:, 1] - good_old[:, 1]

        u_raw = dx.mean()
        v_raw = dy.mean()

        u_translation = u_raw - (gyro_y * self.fx)
        v_translation = v_raw - (gyro_x * self.fy)

        vx = (u_translation * current_altitude) / self.fx
        vy = (v_translation * current_altitude) / self.fy

        vx_command = -self.kp * vx
        vy_command = -self.kp * vy

        self.prev_gray = gray
        self.prev_points = good_new.reshape(-1, 1, 2)

        return True, vx_command, vy_command


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