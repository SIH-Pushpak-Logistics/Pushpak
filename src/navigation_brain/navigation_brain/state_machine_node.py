#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import RCIn
from mavros_msgs.srv import CommandBool, SetMode
from rclpy.qos import qos_profile_sensor_data
from swarm_utils.redis_bridge import RedisTelemetrySubscriber

class StateMachineNode(Node):
    def __init__(self):
        super().__init__('state_machine_node')
        
        self.declare_parameter('drone_id', 'drone_00')
        self.drone_id = self.get_parameter('drone_id').get_parameter_value().string_value

        self.telemetry_stream = f'telemetry:{self.drone_id}:velocity'
        self.altitude_stream = f'telemetry:{self.drone_id}:altitude_cmd'
        self.override_stream = f'emergency_override:{self.drone_id}'

        # Initialize the decoupled background network thread
        self.redis_sub = RedisTelemetrySubscriber(
            streams=[self.telemetry_stream, self.altitude_stream, self.override_stream],
            logger=self.get_logger()
        )

        # Execution Publishers
        self.velocity_publisher = self.create_publisher(
            TwistStamped, '/mavros/setpoint_velocity/cmd_vel', 10
        )

        # MAVROS Service Clients
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        
        self.rc_sub = self.create_subscription(
            RCIn, '/mavros/rc/in', self.rc_callback, qos_profile_sensor_data 
        )

        # State Variables
        self.flight_state = 'BOOTING'
        self.boot_start_time = self.get_clock().now().nanoseconds / 1e9
        self.max_data_age = 0.3  # Adjusted for 15Hz physical reality

        self.get_logger().info(f'Master State Machine booting for {self.drone_id}...')
        self.control_timer = self.create_timer(0.05, self.control_loop) # Strict 20Hz

    def rc_callback(self, msg):
        """ Hardware interrupt for safety kills. Absolute authority. """
        if len(msg.channels) > 7:
            if msg.channels[7] > 1500:
                self.get_logger().fatal("HARDWARE KILL SWITCH ACTIVATED! DISARMING!")
                self.disarm_drone()

    def control_loop(self):
        cmd_msg = TwistStamped()
        cmd_msg.header.stamp = self.get_clock().now().to_msg()
        cmd_msg.header.frame_id = f'{self.drone_id}_base_link'
        
        # -------------------------------------------------------------
        # PHASE 1: THE OFFBOARD HANDSHAKE
        # -------------------------------------------------------------
        if self.flight_state == 'BOOTING':
            # MAVROS requires a continuous stream of setpoints before allowing OFFBOARD mode
            self.velocity_publisher.publish(cmd_msg)
            
            elapsed = (self.get_clock().now().nanoseconds / 1e9) - self.boot_start_time
            if elapsed > 2.0:
                self.get_logger().info('Setpoint stream established. Requesting OFFBOARD mode.')
                self.request_offboard_and_arm()
                self.flight_state = 'AWAITING_AUTHORITY'
            return

        if self.flight_state in ['AWAITING_AUTHORITY', 'ARMING_REQUESTED']:
            self.velocity_publisher.publish(cmd_msg) # Keep the 0.0 stream alive
            return
            
        if self.flight_state == 'DISARMED':
            return # Terminal state

        # -------------------------------------------------------------
        # PHASE 2: ACTIVE FLIGHT (Zero Network Blocking)
        # -------------------------------------------------------------
        target_vx, target_vy, target_vz, target_wz = 0.0, 0.0, 0.0, 0.0
        
        # Read directly from local RAM (Instantly)
        override_payload = self.redis_sub.get_latest(self.override_stream)
        vision_payload = self.redis_sub.get_latest(self.telemetry_stream)
        altitude_payload = self.redis_sub.get_latest(self.altitude_stream)

        # Priority 1: Global Swarm Override
        if override_payload and not self.is_stale(override_payload.get('timestamp', 0)):
            target_vx, target_vy, target_vz, target_wz = self.extract_vectors(override_payload)
                
        # Priority 2: Merge Local Perception Vectors
        else:
            if vision_payload and not self.is_stale(vision_payload.get('timestamp', 0)):
                if vision_payload.get('is_valid', 'True') == 'True':
                    target_vx = float(vision_payload.get('linear_x', 0.0))
                    target_vy = float(vision_payload.get('linear_y', 0.0))
                    target_wz = float(vision_payload.get('angular_z', 0.0))
                else:
                    self.get_logger().warn("BLIND STATE: Optical flow lost! Zeroing X/Y.")
            
            if altitude_payload and not self.is_stale(altitude_payload.get('timestamp', 0)):
                target_vz = float(altitude_payload.get('linear_z', 0.0))
                
                if altitude_payload.get('cut_motors', 'False') == 'True':
                    self.get_logger().warn("Altitude Node requested motor cutoff. Disarming.")
                    self.disarm_drone()
                    return

        cmd_msg.twist.linear.x = target_vx
        cmd_msg.twist.linear.y = target_vy
        cmd_msg.twist.linear.z = target_vz
        cmd_msg.twist.angular.z = target_wz
        self.velocity_publisher.publish(cmd_msg)

    def request_offboard_and_arm(self):
        """ The mandatory sequence to seize control from the physics engine. """
        if not self.set_mode_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('Set_mode service unavailable. Boot failed.')
            return
            
        mode_req = SetMode.Request()
        mode_req.custom_mode = 'OFFBOARD'
        
        future = self.set_mode_client.call_async(mode_req)
        future.add_done_callback(self.mode_change_callback)

    def mode_change_callback(self, future):
        try:
            response = future.result()
            if response.mode_sent:
                self.get_logger().info('OFFBOARD mode engaged. Requesting Motor Arming...')
                
                arm_req = CommandBool.Request()
                arm_req.value = True
                
                # Chain the callback. Do NOT declare success yet.
                arm_future = self.arming_client.call_async(arm_req)
                arm_future.add_done_callback(self.arming_callback)
                
                self.flight_state = 'ARMING_REQUESTED'
            else:
                self.get_logger().error('OFFBOARD mode rejected. Retrying handshake.')
                self.trigger_boot_retry()
        except Exception as e:
            self.get_logger().error(f'SetMode service call failed: {e}. Retrying.')
            self.trigger_boot_retry()

    def arming_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Motors Armed. Execution Authority Granted. Entering FLYING state.')
                self.flight_state = 'FLYING'
            else:
                self.get_logger().error('Arming rejected (EKF not settled?). Retrying handshake.')
                self.trigger_boot_retry()
        except Exception as e:
            self.get_logger().error(f'Arming service call failed: {e}. Retrying.')
            self.trigger_boot_retry()

    def trigger_boot_retry(self):
        """ Forces the state machine to reset the handshake loop. """
        self.flight_state = 'BOOTING'
        self.boot_start_time = self.get_clock().now().nanoseconds / 1e9

    def disarm_drone(self):
        self.flight_state = 'DISARMED'
        req = CommandBool.Request()
        req.value = False
        self.arming_client.call_async(req)
        self.get_logger().info('Terminal disarm sequence executed.')

    def extract_vectors(self, payload):
        return (
            float(payload.get('linear_x', 0.0)),
            float(payload.get('linear_y', 0.0)),
            float(payload.get('linear_z', 0.0)),
            float(payload.get('angular_z', 0.0))
        )

    def is_stale(self, payload_timestamp_str):
        try:
            current_ros_time = self.get_clock().now().nanoseconds / 1e9
            return (current_ros_time - float(payload_timestamp_str)) > self.max_data_age
        except (ValueError, TypeError):
            return True

def main(args=None):
    rclpy.init(args=args)
    node = StateMachineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()