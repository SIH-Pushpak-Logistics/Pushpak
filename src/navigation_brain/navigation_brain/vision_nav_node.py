#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import TwistStamped
from cv_bridge import CvBridge
import cv2
import redis
import queue
import threading

class VisionNavigationNode(Node):
    def __init__(self):
        super().__init__('vision_nav_node')
        self.get_logger().info('Initializing Vision Navigation...')

        self.bridge = CvBridge()
        self.camera_subscription = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)

        # ARCHITECTURE ENFORCEMENT: MAVROS publisher removed. 
        # Perception has no authority over motors.

        # --- RAUNAK: YOUR TASK BEGINS HERE ---
        # 1. Establish a Redis connection to host='localhost', port=6379
        # 2. Create a bounded queue.Queue(maxsize=5)
        # 3. Start a background threading.Thread that drains the queue and publishes to Redis
        pass 

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # --- RAUNAK: YOUR TASK ---
        # 1. Calculate X/Y drift using optical flow here.
        # 2. Push the resulting vector dict into your queue.
        # DO NOT use time.sleep() or blocking calls here.
        pass

def main(args=None):
    rclpy.init(args=args)
    node = VisionNavigationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()