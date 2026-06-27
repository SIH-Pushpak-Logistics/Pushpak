#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from swarm_utils.redis_bridge import RedisTelemetryPublisher

class AltitudeNavigationNode(Node):
    def __init__(self):
        super().__init__('altitude_nav_node')
        self.get_logger().info('Initializing Altitude Navigation...')

        # Hardcoded for this node instance, should ideally be a ROS parameter
        self.declare_parameter('drone_id', 'drone_00')
        self.drone_id = self.get_parameter('drone_id').get_parameter_value().string_value
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

      # --- SHIVAM: YOUR TASK BEGINS HERE ---
        # 1. Instantiate the RedisTelemetryPublisher from swarm_utils.
        #    Target stream: f'telemetry:{self.drone_id}:altitude_cmd'
        pass 

    def altitude_callback(self, msg):
        current_z = msg.data
        
        # --- SHIVAM: YOUR TASK ---
        # 1. Calculate the required $V_z$ (linear_z) to reach self.target_altitude.
        # 2. Use your instantiated RedisTelemetryPublisher to send the vector.
        #    Call .send_velocity_vector(self.drone_id, 0.0, 0.0, calculated_vz, 0.0)
        pass

    def _redis_publisher_worker(self):
        # --- YOUR TASK ---
        #import swarm_utils and use the exact same self.redis_publisher.send_velocity_vector() method,
        #target altitude_cmd stream.
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