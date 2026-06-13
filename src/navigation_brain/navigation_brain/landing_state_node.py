#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import redis
import queue
import threading
import time
import json

class AltitudeNavigationNode(Node):
    def __init__(self):
        super().__init__('altitude_nav_node')
        self.get_logger().info('Initializing Altitude Navigation...')

        # Hardcoded for this node instance, should ideally be a ROS parameter
        self.drone_id = "drone_01" 
        self.target_altitude = 5.0  # Target Z in meters

        # Input Layer: Strictly Perception/Sensor Data
        self.altitude_subscription = self.create_subscription(
            Float32, 
            '/drone/altitude', 
            self.altitude_callback, 
            10
        )

        # ARCHITECTURE ENFORCEMENT: MAVROS publisher strictly forbidden. 
        # This node calculates intent; it does not authorize motor actuation.

        # --- YOUR TASK BEGINS HERE ---
        # 1. Establish a Redis connection (host='localhost', port=6379).
        # 2. Create a bounded queue: queue.Queue(maxsize=5) to prevent memory bloating on block.
        # 3. Start a background threading.Thread (daemon=True) that continuously drains 
        #    the queue and pushes to Redis.
        pass 

    def altitude_callback(self, msg):
        current_z = msg.data
        
        # --- YOUR TASK ---
        # 1. Calculate the required $V_z$ (linear_z) to reach self.target_altitude.
        #    Implement your P/PI/PID logic here. 
        # 2. Construct the exact JSON payload dictated by the Data Contract:
        #    {
        #      "timestamp": <current_unix_epoch_float>,
        #      "drone_id": self.drone_id,
        #      "linear_x": 0.0,
        #      "linear_y": 0.0,
        #      "linear_z": <calculated_v_z>,
        #      "angular_z": 0.0
        #    }
        # 3. Push this dictionary into your queue.
        # 
        # CRITICAL: DO NOT block this callback. No time.sleep(), no synchronous Redis calls.
        pass

    def _redis_publisher_worker(self):
        # --- YOUR TASK ---
        # 1. Loop infinitely, blocking on queue.get().
        # 2. Publish the popped payload to the Redis stream: 
        #    Stream Key -> telemetry:<drone_id>:altitude_cmd
        # 3. Use XADD. Enforce maximum stream length (MAXLEN) to prevent memory leaks.
        pass

def main(args=None):
    rclpy.init(args=args)
    node = AltitudeNavigationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Altitude Navigation...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()