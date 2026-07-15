#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, TwistWithCovarianceStamped
from mavros_msgs.msg import RCIn, State
from mavros_msgs.srv import CommandBool, SetMode
from rclpy.qos import qos_profile_sensor_data
from swarm_utils.redis_bridge import RedisTelemetrySubscriber
import numpy as np
import redis
import threading
import queue

class StateMachineNode(Node):
    def __init__(self):
        super().__init__('state_machine_node')
        
        self.declare_parameter('drone_id', 'drone_00')
        self.drone_id = self.get_parameter('drone_id').get_parameter_value().string_value

        self.telemetry_stream = f'telemetry:{self.drone_id}:velocity'
        self.altitude_stream = f'telemetry:{self.drone_id}:altitude_cmd'
        self.override_stream = f'emergency_override:{self.drone_id}'

        # Initialize the decoupled background network thread for ingestion
        self.redis_sub = RedisTelemetrySubscriber(
            streams=[self.telemetry_stream, self.altitude_stream, self.override_stream],
            logger=self.get_logger()
        )
        
        # Dedicated raw Redis connection for publishing swarm states
        self.redis_client = redis.Redis(host='localhost', port=6379, decode_responses=True)
        
        # --- ASYNCHRONOUS PUBLISHING PIPELINE ---
        # Bounded queue to prevent OOM kills and RT loop blocking
        self.state_publish_queue = queue.Queue(maxsize=10)
        self.state_publisher_thread = threading.Thread(target=self._state_publisher_worker, daemon=True)
        self.state_publisher_thread.start()

        # Execution Publishers (Control AND Estimation)
        self.velocity_publisher = self.create_publisher(
            TwistStamped, '/mavros/setpoint_velocity/cmd_vel', 10
        )
        self.vision_speed_pub = self.create_publisher(
            TwistWithCovarianceStamped, '/mavros/vision_speed/speed_twist_covariance', 10
        )

        # MAVROS Service Clients
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        
        self.rc_sub = self.create_subscription(
            RCIn, '/mavros/rc/in', self.rc_callback, qos_profile_sensor_data 
        )
        
        # MAVROS State Overwatch
        self.current_mavros_state = State()
        self.state_sub = self.create_subscription(
            State, '/mavros/state', self.mavros_state_callback, 10
        )

        # State Variables
        self.flight_state = 'BOOTING'
        self.boot_start_time = self.get_clock().now().nanoseconds / 1e9
        self.max_data_age = 0.3  # Adjusted for 15Hz physical reality
        self.feature_loss_counter = 0

        self.get_logger().info(f'Master State Machine booting for {self.drone_id}...')
        self.control_timer = self.create_timer(0.05, self.control_loop) # Strict 20Hz

    def _state_publisher_worker(self):
        """ Background thread to handle Redis network calls without blocking the RT loop. """
        while rclpy.ok():
            try:
                stream, payload = self.state_publish_queue.get(timeout=0.1)
                self.redis_client.xadd(stream, payload)
                self.state_publish_queue.task_done()
            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().error(f"Redis publisher thread error: {e}")

    def rc_callback(self, msg):
        """ Hardware interrupt for safety kills. Absolute authority. """
        if len(msg.channels) > 7:
            if msg.channels[7] > 1500:
                self.get_logger().fatal("HARDWARE KILL SWITCH ACTIVATED! DISARMING!")
                self.disarm_drone()
                
    def mavros_state_callback(self, msg):
        self.current_mavros_state = msg

    def control_loop(self):
        current_time = self.get_clock().now()
        
        cmd_msg = TwistStamped()
        cmd_msg.header.stamp = current_time.to_msg()
        cmd_msg.header.frame_id = f'{self.drone_id}_base_link'
        
        # -------------------------------------------------------------
        # PHASE 0: UNCONDITIONAL EKF HEARTBEAT (MOVED TO TOP)
        # -------------------------------------------------------------
        target_vx, target_vy, target_vz, target_wz = 0.0, 0.0, 0.0, 0.0
        variance = 9999.0  # Default to zero trust
        
        override_payload = self.redis_sub.get_latest(self.override_stream)
        vision_payload = self.redis_sub.get_latest(self.telemetry_stream)
        altitude_payload = self.redis_sub.get_latest(self.altitude_stream)

        # Evaluate EKF Covariance Unconditionally
        if vision_payload and not self.is_stale(vision_payload.get('timestamp', 0)):
            num_features = int(vision_payload.get('features', 0))
            
            if num_features < 5:
                self.feature_loss_counter += 1
                variance = 9999.0
                target_vx, target_vy = 0.0, 0.0
                
                # Only trigger emergency landing if we actually have authority
                if self.feature_loss_counter > 5 and self.flight_state == 'FLYING':
                    self.get_logger().fatal("PROLONGED VISION LOSS! Initiating EMERGENCY LANDING.")
                    self.set_mode("LAND")
                    try:
                        self.state_publish_queue.put_nowait((
                            f"swarm:{self.drone_id}:state", 
                            {"status": "EMERGENCY_LANDING"}
                        ))
                    except queue.Full:
                        pass
            else:
                self.feature_loss_counter = 0
                variance = 0.01 + (1.0 / num_features)
                target_vx = float(vision_payload.get('linear_x', 0.0))
                target_vy = float(vision_payload.get('linear_y', 0.0))
                target_wz = float(vision_payload.get('angular_z', 0.0))
        else:
            variance = 9999.0
            target_vx, target_vy = 0.0, 0.0

        # Publish the EKF Trust Metric to MAVROS constantly
        cov_msg = TwistWithCovarianceStamped()
        cov_msg.header.stamp = current_time.to_msg()
        cov_msg.header.frame_id = "camera_link"
        cov_msg.twist.twist.linear.x = target_vx
        cov_msg.twist.twist.linear.y = target_vy
        
        covariance = np.zeros(36, dtype=float)
        covariance[14], covariance[21], covariance[28], covariance[35] = -1.0, -1.0, -1.0, -1.0
        covariance[0] = variance
        covariance[7] = variance
        cov_msg.twist.covariance = covariance.tolist()
        
        self.vision_speed_pub.publish(cov_msg)

        # -------------------------------------------------------------
        # PHASE 1: THE GUIDED HANDSHAKE
        # -------------------------------------------------------------
        if self.flight_state == 'BOOTING':
            self.velocity_publisher.publish(cmd_msg)
            elapsed = (current_time.nanoseconds / 1e9) - self.boot_start_time
            # Increased wait time to 10 seconds to give EKF time to align
            if elapsed > 10.0: 
                self.get_logger().info('EKF alignment period complete. Requesting GUIDED mode.')
                self.request_guided_and_arm()
                self.flight_state = 'AWAITING_AUTHORITY'
            return

        if self.flight_state in ['AWAITING_AUTHORITY', 'ARMING_REQUESTED']:
            self.velocity_publisher.publish(cmd_msg)
            return
            
        if self.flight_state == 'DISARMED':
            return

        # -------------------------------------------------------------
        # PHASE 2: AUTHORITY OVERWATCH
        # -------------------------------------------------------------
        if self.flight_state == 'FLYING':
            if self.current_mavros_state.mode != "GUIDED":
                self.get_logger().error("CRITICAL: GUIDED mode lost externally! Relinquishing control.")
                self.trigger_boot_retry()
                return

        # -------------------------------------------------------------
        # PHASE 3: ACTIVE FLIGHT 
        # -------------------------------------------------------------
        # Resolve Control Priority
        if override_payload and not self.is_stale(override_payload.get('timestamp', 0)):
            target_vx, target_vy, target_vz, target_wz = self.extract_vectors(override_payload)
        else:
            if altitude_payload and not self.is_stale(altitude_payload.get('timestamp', 0)):
                target_vz = float(altitude_payload.get('linear_z', 0.0))
                if altitude_payload.get('cut_motors', 'False') == 'True':
                    self.get_logger().warn("Altitude Node requested motor cutoff. Disarming.")
                    self.disarm_drone()
                    return

        # Execute the Velocity Setpoint
        cmd_msg.twist.linear.x = target_vx
        cmd_msg.twist.linear.y = target_vy
        cmd_msg.twist.linear.z = target_vz
        cmd_msg.twist.angular.z = target_wz
        self.velocity_publisher.publish(cmd_msg)

    def request_guided_and_arm(self):
        if not self.set_mode_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('Set_mode service unavailable. Boot failed.')
            return
            
        mode_req = SetMode.Request()
        mode_req.custom_mode = 'GUIDED'
        
        future = self.set_mode_client.call_async(mode_req)
        future.add_done_callback(self.mode_change_callback)

    def mode_change_callback(self, future):
        try:
            response = future.result()
            if response.mode_sent:
                self.get_logger().info('GUIDED mode engaged. Requesting Motor Arming...')
                arm_req = CommandBool.Request()
                arm_req.value = True
                
                arm_future = self.arming_client.call_async(arm_req)
                arm_future.add_done_callback(self.arming_callback)
                self.flight_state = 'ARMING_REQUESTED'
            else:
                self.get_logger().error('GUIDED mode rejected. Retrying handshake.')
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

    def set_mode(self, custom_mode):
        if not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            return
        req = SetMode.Request()
        req.custom_mode = custom_mode
        self.set_mode_client.call_async(req)

    def trigger_boot_retry(self):
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