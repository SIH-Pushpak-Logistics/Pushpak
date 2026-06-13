#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from geometry_msgs.msg import TwistStamped
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

        # --- Redis Setup ---
        self.redis_publisher = RedisTelemetryPublisher(
            stream_name=f'telemetry:{self.drone_id}:velocity', 
            logger=self.get_logger()
        )

        # --- Synchronized Inputs ---
        self.camera_sub = message_filters.Subscriber(self, Image, '/camera/image_raw')
        self.altitude_sub = message_filters.Subscriber(self, Float32, '/drone/altitude')
        
        # Exact time synchronization (0.05s slop)
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.camera_sub, self.altitude_sub], 
            queue_size=10, 
            slop=0.05
        )
        self.ts.registerCallback(self.synchronized_callback)

    def synchronized_callback(self, image_msg, alt_msg):
        """
        Callback only fires when a camera frame and altitude reading match in time.
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Failure: {str(e)}')
            return

        current_altitude = alt_msg.data

        # Run vision pipeline
        velocity_vector = self.process_vision_pipeline(cv_image, current_altitude)

        # Send telemetry payload via the delegated publisher class
        self.redis_publisher.send_velocity_vector(
            self.drone_id,
            velocity_vector.twist.linear.x,
            velocity_vector.twist.linear.y,
            velocity_vector.twist.linear.z,
            velocity_vector.twist.angular.z
        )

    def process_vision_pipeline(self, cv_frame, current_altitude):
        """
        RAUNAK: Your OpenCV math goes here. 
        You now have current_altitude (Z) perfectly synced with the frame to do your pinhole math.
        """
        cmd_msg = TwistStamped()
        cmd_msg.header.stamp = self.get_clock().now().to_msg()
        cmd_msg.header.frame_id = f'{self.drone_id}_base_link'

        cmd_msg.twist.linear.x = 0.0
        cmd_msg.twist.linear.y = 0.0
        cmd_msg.twist.linear.z = 0.0
        cmd_msg.twist.angular.z = 0.0

        # TODO: OpenCV Lucas-Kanade and Pinhole Conversion
        
        return cmd_msg

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