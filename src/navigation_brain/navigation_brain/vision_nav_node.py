#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import TwistStamped
from cv_bridge import CvBridge, CvBridgeError

class VisionNavigationNode(Node):
    def __init__(self):
        super().__init__('vision_nav_node')
        self.get_logger().info('Initializing Vision Navigation Architecture Node...')

        # Initialize the bridge between ROS 2 images and OpenCV matrices
        self.bridge = CvBridge()

        # DATA CONTRACT INPUT: Subscriber to the simulated down-facing camera
        self.camera_subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # DATA CONTRACT OUTPUT: Publisher to the MAVROS local velocity setpoint
        self.velocity_publisher = self.create_publisher(
            TwistStamped,
            '/mavros/setpoint_velocity/cmd_vel',
            10
        )

    def image_callback(self, msg):
        try:
            # Convert raw ROS image message to OpenCV BGR matrix
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Translation Failure: {str(e)}')
            return

        # Execute algorithmic processing
        velocity_vector = self.process_vision_pipeline(cv_image)

        # Enforce publication contract
        self.velocity_publisher.publish(velocity_vector)

    def process_vision_pipeline(self, cv_frame):
        """
        RAUNAK: This is the exact scope of your task. 
        Your OpenCV algorithms, optical flow calculation, or tracking models 
        must execute inside this function. 
        
        Input: cv_frame (Contiguous numpy array from the camera)
        Output: Must return a geometry_msgs.msg.TwistStamped message
        """
        cmd_msg = TwistStamped()
        cmd_msg.header.stamp = self.get_clock().now().to_msg()
        cmd_msg.header.frame_id = 'base_link'

        # DEFAULT SAFETY STATE: Zero velocity override
        cmd_msg.twist.linear.x = 0.0
        cmd_msg.twist.linear.y = 0.0
        cmd_msg.twist.linear.z = 0.0
        cmd_msg.twist.angular.z = 0.0

        # TODO: Implement optical flow displacement vectors here
        
        return cmd_msg

def main(args=None):
    rclpy.init(args=args)
    node = VisionNavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down navigation node cleanly via interrupt.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()