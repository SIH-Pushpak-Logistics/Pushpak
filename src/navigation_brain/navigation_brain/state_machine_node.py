#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import RCIn
from mavros_msgs.srv import CommandBool
from rclpy.qos import qos_profile_sensor_data
import redis

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
            self.get_logger().error(f'FATAL: Redis connection failed on boot: {e}')
            raise SystemExit
        
        self.get_logger().info(f'Initializing Master State Machine for {self.drone_id}...')

        self.velocity_publisher = self.create_publisher(
            TwistStamped,
            '/mavros/setpoint_velocity/cmd_vel',
            10
        )

        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.is_armed = True 

        # HARDWARE INTERRUPT
        self.rc_sub = self.create_subscription(
            RCIn, '/mavros/rc/in', self.rc_callback, qos_profile_sensor_data 
        )

        self.control_timer = self.create_timer(0.05, self.control_loop) # 20Hz
        
        # FIX 2: Tightened latency tolerance for reality
        self.max_data_age = 0.1 

    def rc_callback(self, msg):
        if len(msg.channels) > 7:
            ch8_pwm = msg.channels[7]
            if ch8_pwm > 1500 and self.is_armed:
                self.get_logger().fatal("HARDWARE KILL SWITCH ACTIVATED! DISARMING!")
                self.disarm_drone()

    def control_loop(self):
        if not self.is_armed:
            return 
        
        cmd_msg = TwistStamped()
        cmd_msg.header.stamp = self.get_clock().now().to_msg()
        cmd_msg.header.frame_id = f'{self.drone_id}_base_link'
        
        target_vx, target_vy, target_vz, target_wz = 0.0, 0.0, 0.0, 0.0
        
        # FIX 3: Mid-flight network resilience
        try:
            override_data = self.redis_client.xrevrange(self.override_stream, max='+', min='-', count=1)
            vision_data = self.redis_client.xrevrange(self.telemetry_stream, max='+', min='-', count=1)
            altitude_data = self.redis_client.xrevrange(self.altitude_stream, max='+', min='-', count=1)
        except redis.ConnectionError as e:
            self.get_logger().error(f"REDIS CRASH: {e}. Hovering!")
            self.velocity_publisher.publish(cmd_msg) # Publishes 0.0s
            return

        # Priority 1: Global Override dictates everything
        if override_data and not self.is_stale(override_data[0][1].get('timestamp', 0)):
            target_vx, target_vy, target_vz, target_wz = self.extract_vectors(override_data[0][1])
                
        # Priority 2: Merge Local Vision (X/Y) and Local Altitude (Z)
        else:
            # Extract X/Y from Raunak's vision node
            if vision_data and not self.is_stale(vision_data[0][1].get('timestamp', 0)):
                payload = vision_data[0][1]
                
                # --- THE BLIND STATE CHECK ---
                # Defaults to 'True' to prevent crashing before Raunak's PR is merged
                if payload.get('is_valid', 'True') == 'True':
                    target_vx = float(payload.get('linear_x', 0.0))
                    target_vy = float(payload.get('linear_y', 0.0))
                    target_wz = float(payload.get('angular_z', 0.0))
                else:
                    self.get_logger().warn("BLIND STATE: Optical flow lost! Zeroing X/Y velocities.")
                    target_vx, target_vy, target_wz = 0.0, 0.0, 0.0
                    # Note: We do not touch target_vz here. 
                    # We let Shivam's altitude node continue to manage the Z-axis descent.
            
            if altitude_data and not self.is_stale(altitude_data[0][1].get('timestamp', 0)):
                payload = altitude_data[0][1]
                target_vz = float(payload.get('linear_z', 0.0))
                
                # FIX 1: API Contract Fulfilled (The Missing Disarm)
                cut_motors = payload.get('cut_motors', 'False') == 'True'
                if cut_motors:
                    self.get_logger().warn("Altitude Node requested motor cutoff. Disarming.")
                    self.disarm_drone()
                    return

        # Execute
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

    def disarm_drone(self):
        self.is_armed = False
        if not self.arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('MAVROS arming service not available!')
            return
            
        req = CommandBool.Request()
        req.value = False
        self.arming_client.call_async(req)
        self.get_logger().info('Disarm command sent to flight controller.')

    def is_stale(self, payload_timestamp_str):
        try:
            current_ros_time = self.get_clock().now().nanoseconds / 1e9
            if (current_ros_time - float(payload_timestamp_str)) > self.max_data_age:
                return True
            return False
        except ValueError:
            return True