#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import redis
import time

class StateMachineNode(Node):
    def __init__(self):
        super().__init__('state_machine_node')
        
        # Swarm Identification
        self.declare_parameter('drone_id', 'drone_00')
        self.drone_id = self.get_parameter('drone_id').get_parameter_value().string_value
        
        self.get_logger().info(f'Initializing Master State Machine for {self.drone_id}...')

        # Output: Direct link to MAVROS local velocity controller
        self.velocity_publisher = self.create_publisher(
            TwistStamped,
            '/mavros/setpoint_velocity/cmd_vel',
            10
        )

        # Input: Redis Broker Connection
        try:
            self.redis_client = redis.Redis(host='localhost', port=6379, decode_responses=True)
            self.redis_client.ping()
            self.telemetry_stream = f'telemetry:{self.drone_id}:velocity'
            self.override_stream = f'emergency_override:{self.drone_id}'
            self.get_logger().info('Redis connection established. Listening for state commands.')
        except redis.ConnectionError as e:
            self.get_logger().error(f'FATAL: Redis connection failed: {e}')
            raise

        # Control Loop: Runs at 20Hz (Every 0.05 seconds)
        self.control_timer = self.create_timer(0.05, self.control_loop)
        
        # Stale data threshold (seconds)
        self.max_data_age = 0.5 

    def control_loop(self):
        """
        The absolute brain of the drone. It pulls the latest network state
        and enforces execution to MAVROS.
        """
        cmd_msg = TwistStamped()
        cmd_msg.header.stamp = self.get_clock().now().to_msg()
        cmd_msg.header.frame_id = f'{self.drone_id}_base_link'
        
        # Default failsafe state
        target_vx, target_vy, target_vz, target_wz = 0.0, 0.0, 0.0, 0.0
        
        # Step 1: Check for Global Emergency Overrides (Highest Priority)
        # xrevrange gets the absolute newest message from the end of the stream
        override_data = self.redis_client.xrevrange(self.override_stream, max='+', min='-', count=1)
        
        # Step 2: Check Local Vision Telemetry (Secondary Priority)
        vision_data = self.redis_client.xrevrange(self.telemetry_stream, max='+', min='-', count=1)

        executed_command_source = "FAILSAFE_HOVER"

        # Evaluate Override First
        if override_data:
            msg_id, payload = override_data[0]
            if not self.is_stale(payload.get('timestamp', 0)):
                target_vx = float(payload.get('linear_x', 0.0))
                target_vy = float(payload.get('linear_y', 0.0))
                target_vz = float(payload.get('linear_z', 0.0))
                target_wz = float(payload.get('angular_z', 0.0))
                executed_command_source = "GLOBAL_OVERRIDE"
                
        # If no valid override, evaluate local vision
        elif vision_data and executed_command_source == "FAILSAFE_HOVER":
            msg_id, payload = vision_data[0]
            if not self.is_stale(payload.get('timestamp', 0)):
                target_vx = float(payload.get('linear_x', 0.0))
                target_vy = float(payload.get('linear_y', 0.0))
                target_vz = float(payload.get('linear_z', 0.0))
                target_wz = float(payload.get('angular_z', 0.0))
                executed_command_source = "LOCAL_VISION"

        # Apply the determined vectors
        cmd_msg.twist.linear.x = target_vx
        cmd_msg.twist.linear.y = target_vy
        cmd_msg.twist.linear.z = target_vz
        cmd_msg.twist.angular.z = target_wz

        # Publish to the flight controller
        self.velocity_publisher.publish(cmd_msg)
        
        # Optional: Log the state transitions for debugging
        # self.get_logger().debug(f'Executing {executed_command_source} | Vx: {target_vx}')

    def is_stale(self, payload_timestamp_str):
        """
        Calculates the delta between the message timestamp and system time.
        """
        try:
            payload_time = float(payload_timestamp_str)
            current_time = time.time()
            if (current_time - payload_time) > self.max_data_age:
                self.get_logger().warn('Stale telemetry detected. Dropping command.')
                return True
            return False
        except ValueError:
            return True

def main(args=None):
    rclpy.init(args=args)
    node = StateMachineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down State Machine cleanly.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()