#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from swarm_utils.redis_bridge import RedisTelemetryPublisher, RedisTelemetrySubscriber

class AltitudeControlNode(Node):
    def __init__(self):
        super().__init__('altitude_control_node')
        self.get_logger().info('Initializing Dynamic Altitude & Landing Node...')

        self.declare_parameter('drone_id', 'drone_00')
        self.drone_id = self.get_parameter('drone_id').get_parameter_value().string_value

        # --- Network I/O (Redis Architecture) ---
        self.redis_publisher = RedisTelemetryPublisher(
            stream_name=f'telemetry:{self.drone_id}:altitude_cmd',
            logger=self.get_logger()
        )

        self.altitude_stream = f'telemetry:{self.drone_id}:altitude'
        self.target_stream = f'telemetry:{self.drone_id}:target_altitude'
        self.command_stream = f'telemetry:{self.drone_id}:commands'

        self.redis_subscriber = RedisTelemetrySubscriber(
            streams=[self.altitude_stream, self.target_stream, self.command_stream],
            logger=self.get_logger()
        )

        # --- Control Variables ---
        self.current_z = None
        self.target_altitude = 0.0  
        self.kp_z = 0.8             
        self.max_descent_speed = -0.7
        self.max_ascent_speed = 1.5
        self.sensor_timeout = 0.5 

        # --- Touchdown State Tracking ---
        self.cut_motors_intent = False 
        self.prev_z = None
        self.prev_time = None
        self.touchdown_counter = 0  # Persistence counter for noise rejection

        self.control_timer = self.create_timer(0.05, self.control_loop)

    def control_loop(self):
        current_time_sec = self.get_clock().now().nanoseconds / 1e9
        
        # 1. Poll Redis for Commands (State Reset)
        cmd_payload = self.redis_subscriber.get_latest(self.command_stream)
        if cmd_payload and cmd_payload.get('reset') == 'True':
            self.get_logger().info('State Reset Triggered via Redis. Wiping terminal flags.')
            self.cut_motors_intent = False
            self.touchdown_counter = 0
            self.prev_z = None
            self.prev_time = None

        # 2. Poll Redis for Target Altitude
        target_payload = self.redis_subscriber.get_latest(self.target_stream)
        if target_payload and 'target_altitude' in target_payload:
            self.target_altitude = float(target_payload.get('target_altitude', self.target_altitude))

        # 3. Poll Redis for Current Altitude
        alt_payload = self.redis_subscriber.get_latest(self.altitude_stream)
        payload_timestamp = None
        if alt_payload:
            self.current_z = float(alt_payload.get('z', 0.0))
            payload_timestamp = float(alt_payload.get('timestamp', 0.0))
        
        # --- Pre-Flight Safety Checks ---
        if self.cut_motors_intent:
            self.redis_publisher.send_velocity_vector(
                self.drone_id, current_time_sec, 0.0, 0.0, 0.0, 0.0, cut_motors=True
            )
            return

        if self.current_z is None or payload_timestamp is None:
            self.redis_publisher.send_velocity_vector(
                self.drone_id, current_time_sec, 0.0, 0.0, 0.0, 0.0, cut_motors=False
            )
            return 

        sensor_age = current_time_sec - payload_timestamp
        if sensor_age > self.sensor_timeout:
            self.get_logger().error(f'SENSOR TIMEOUT: Alt data is {sensor_age:.2f}s old. Aborting descent!')
            self.redis_publisher.send_velocity_vector(
                self.drone_id, current_time_sec, 0.0, 0.0, 0.0, 0.0, cut_motors=False
            )
            return

        error = self.target_altitude - self.current_z
        vz = 0.0
        
        is_landing = self.target_altitude <= 0.1

        # --- Terminal Descent Phase ---
        if self.current_z < 0.25 and is_landing:
            vz = -0.3 
            
            # The True Kinematic Touchdown Proof (Armored against noise)
            if self.prev_z is not None and self.prev_time is not None:
                dt = current_time_sec - self.prev_time
                if dt > 0:
                    actual_vz = (self.current_z - self.prev_z) / dt
                    
                    if abs(actual_vz) < 0.05:
                        self.touchdown_counter += 1
                        
                        # Demand 5 consecutive frames (250ms) of contradiction
                        if self.touchdown_counter >= 5:
                            self.get_logger().info('Kinematic contradiction detected & sustained. Ground blocking confirmed. Cutting motors.')
                            vz = 0.0
                            self.cut_motors_intent = True
                    else:
                        # Reset counter if the drone bounces or noise spikes
                        self.touchdown_counter = 0

            self.prev_z = self.current_z
            self.prev_time = current_time_sec

        # --- High Altitude Approach Phase ---
        else:
            vz = self.kp_z * error
            self.touchdown_counter = 0
            self.prev_z = None
            self.prev_time = None
            
        if vz > self.max_ascent_speed:
            vz = self.max_ascent_speed
        elif vz < self.max_descent_speed:
            vz = self.max_descent_speed

        self.redis_publisher.send_velocity_vector(
            self.drone_id, current_time_sec, 0.0, 0.0, vz, 0.0, cut_motors=self.cut_motors_intent
        )

def main(args=None):
    rclpy.init(args=args)
    node = AltitudeControlNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Altitude Control Node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()