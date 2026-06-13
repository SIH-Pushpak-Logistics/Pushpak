#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import redis
import time

class StateMachineNode(Node):
    def __init__(self):
        super().__init__('state_machine_node')
        
        self.declare_parameter('drone_id', 'drone_00')
        self.drone_id = self.get_parameter('drone_id').get_parameter_value().string_value

        try:
            self.redis_client = redis.Redis(host='localhost', port=6379, decode_responses=True)
            self.redis_client.ping()
            self.telemetry_stream = f'telemetry:{self.drone_id}:velocity'
            self.altitude_stream = f'telemetry:{self.drone_id}:altitude_cmd'
            self.override_stream = f'emergency_override:{self.drone_id}'
        except redis.ConnectionError as e:
            self.get_logger().error(f'FATAL: Redis connection failed: {e}')
            raise SystemExit
        
        self.get_logger().info(f'Initializing Master State Machine for {self.drone_id}...')

        # The ONLY node allowed to talk to MAVROS
        self.velocity_publisher = self.create_publisher(
            TwistStamped,
            '/mavros/setpoint_velocity/cmd_vel',
            10
        )



        self.control_timer = self.create_timer(0.05, self.control_loop) # 20Hz
        self.max_data_age = 0.5 

    def control_loop(self):
        cmd_msg = TwistStamped()
        cmd_msg.header.stamp = self.get_clock().now().to_msg()
        cmd_msg.header.frame_id = f'{self.drone_id}_base_link'
        
        target_vx, target_vy, target_vz, target_wz = 0.0, 0.0, 0.0, 0.0
        executed_source = "FAILSAFE_HOVER"
        
        # 1. Pull the absolute newest data from all three streams
        override_data = self.redis_client.xrevrange(self.override_stream, max='+', min='-', count=1)
        vision_data = self.redis_client.xrevrange(self.telemetry_stream, max='+', min='-', count=1)
        altitude_data = self.redis_client.xrevrange(self.altitude_stream, max='+', min='-', count=1)

        # 2. Priority 1: Global Override dictates everything
        if override_data and not self.is_stale(override_data[0][1].get('timestamp', 0)):
            target_vx, target_vy, target_vz, target_wz = self.extract_vectors(override_data[0][1])
                
        # 3. Priority 2: Merge Local Vision (X/Y) and Local Altitude (Z)
        else:
            # Extract X/Y from Raunak's vision node
            if vision_data and not self.is_stale(vision_data[0][1].get('timestamp', 0)):
                payload = vision_data[0][1]
                target_vx = float(payload.get('linear_x', 0.0))
                target_vy = float(payload.get('linear_y', 0.0))
                target_wz = float(payload.get('angular_z', 0.0))
            
            # Extract Z from Shivam's altitude node
            if altitude_data and not self.is_stale(altitude_data[0][1].get('timestamp', 0)):
                payload = altitude_data[0][1]
                target_vz = float(payload.get('linear_z', 0.0))

        # 4. Execute
        cmd_msg.twist.linear.x = target_vx
        cmd_msg.twist.linear.y = target_vy
        cmd_msg.twist.linear.z = target_vz
        cmd_msg.twist.angular.z = target_wz

        self.velocity_publisher.publish(cmd_msg)

    def extract_vectors(self, payload):
        return (
            float(payload.get('linear_x', 0.0)),
            float(payload.get('linear_y', 0.0)),
            float(payload.get('linear_z', 0.0)),
            float(payload.get('angular_z', 0.0))
        )

    def is_stale(self, payload_timestamp_str):
        try:
            if (time.time() - float(payload_timestamp_str)) > self.max_data_age:
                self.get_logger().warn('Stale data. Defaulting to hover.')
                return True
            return False
        except ValueError:
            return True

def main(args=None):
    rclpy.init(args=args)
    node = StateMachineNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()