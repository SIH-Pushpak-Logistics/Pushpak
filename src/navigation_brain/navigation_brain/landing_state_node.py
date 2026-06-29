#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Empty
from swarm_utils.redis_bridge import RedisTelemetryPublisher

class AltitudeControlNode(Node):
    def __init__(self):
        super().__init__('altitude_control_node')
        self.get_logger().info('Initializing Dynamic Altitude & Landing Node...')

        # --- Swarm Identity ---
        self.declare_parameter('drone_id', 'drone_00')
        self.drone_id = self.get_parameter('drone_id').get_parameter_value().string_value

        # --- Network I/O ---
        self.redis_publisher = RedisTelemetryPublisher(
            stream_name=f'telemetry:{self.drone_id}:altitude_cmd',
            logger=self.get_logger()
        )

        # --- Subscriptions ---
        self.altitude_sub = self.create_subscription(
            Float32, '/drone/altitude', self.altitude_callback, 10
        )
        self.target_sub = self.create_subscription(
            Float32, '/drone/target_altitude', self.target_callback, 10
        )
        self.reset_sub = self.create_subscription(
            Empty, '/drone/state_reset', self.reset_callback, 10
        )

        # --- Control Variables ---
        self.current_z = None
        self.last_sensor_time = None
        self.target_altitude = 0.0  
        self.kp_z = 0.8             
        self.max_descent_speed = -1.5 
        self.sensor_timeout = 0.5 # seconds

        # --- Blind State Failsafe Variables ---
        self.in_blind_descent = False
        self.blind_start_time = None  
        self.blind_duration = 1.0     
        self.blind_vz = -0.3          
        self.cut_motors_intent = False 

        # --- Execution Loop ---
        self.control_timer = self.create_timer(0.05, self.control_loop)

    def altitude_callback(self, msg):
        """ Ingests altitude data and updates the watchdog timer. """
        self.last_sensor_time = self.get_clock().now()

        if msg.data < 0.2:
            return
        
        self.current_z = msg.data

    def target_callback(self, msg):
        """ Dynamically updates the altitude objective. """
        self.target_altitude = msg.data

    def reset_callback(self, msg):
        """ Wipes the state machine clean for a new mission phase. """
        self.get_logger().info('State Reset Triggered. Wiping terminal flags.')
        self.in_blind_descent = False
        self.cut_motors_intent = False
        self.blind_start_time = None

    def control_loop(self):
        """ 20Hz Non-blocking control loop with failsafes. """
        
        # CRITICAL: Capture the exact simulation time for this control frame
        current_time_sec = self.get_clock().now().nanoseconds / 1e9
        
        # 1. State Persistence Override
        if self.cut_motors_intent:
            self.redis_publisher.send_velocity_vector(
                self.drone_id, current_time_sec, 0.0, 0.0, 0.0, 0.0, cut_motors=True
            )
            return

        # 2. Boot-Up Ghosting Prevention
        if self.current_z is None or self.last_sensor_time is None:
            self.redis_publisher.send_velocity_vector(
                self.drone_id, current_time_sec, 0.0, 0.0, 0.0, 0.0, cut_motors=False
            )
            return 

        # 3. Sensor Watchdog (Stale Data Check)
        sensor_age = current_time_sec - (self.last_sensor_time.nanoseconds / 1e9)
        if sensor_age > self.sensor_timeout:
            self.get_logger().error(f'SENSOR TIMEOUT: Alt data is {sensor_age:.2f}s old. Aborting descent!')
            self.redis_publisher.send_velocity_vector(
                self.drone_id, current_time_sec, 0.0, 0.0, 0.0, 0.0, cut_motors=False
            )
            return

        vz = 0.0

        # 4. Trigger State Change (Only if the TARGET is the ground)
        if self.current_z <= 0.25 and self.target_altitude <= 0.25 and not self.in_blind_descent:
            self.get_logger().info('Sensor minimum range reached. Initiating open-loop blind descent.')
            self.in_blind_descent = True
            self.blind_start_time = self.get_clock().now() 

        # 5. Open-Loop Blind Descent Execution
        if self.in_blind_descent:
            elapsed = current_time_sec - (self.blind_start_time.nanoseconds / 1e9)
            if elapsed < self.blind_duration:
                vz = self.blind_vz
            else:
                self.get_logger().info('Blind descent complete. Requesting motor cutoff.')
                vz = 0.0
                self.cut_motors_intent = True 
                
        # 6. Closed-Loop P-Controller Execution
        else:
            error = self.target_altitude - self.current_z
            vz = self.kp_z * error
            if vz < self.max_descent_speed:
                vz = self.max_descent_speed

        # 7. Push to Network
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