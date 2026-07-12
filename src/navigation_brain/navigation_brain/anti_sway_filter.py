#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3
from swarm_utils.redis_bridge import RedisTelemetryPublisher, RedisTelemetrySubscriber

class AntiSwayFilterNode(Node):
    def __init__(self):
        super().__init__('anti_sway_filter_node')
        self.get_logger().info('Initializing LQR Anti-Sway Filter with Alpha-Beta Tracking...')

        self.declare_parameter('drone_id', 'drone_00')
        self.drone_id = self.get_parameter('drone_id').get_parameter_value().string_value

        # --- Load LQR Gains from YAML (The K Matrix) ---
        self.declare_parameter('k_theta_y', 0.0) 
        self.declare_parameter('k_theta_dot_y', 0.0)
        self.declare_parameter('k_theta_x', 0.0)
        self.declare_parameter('k_theta_dot_x', 0.0)
        
        # Enforcing Velocity Dampers (Full-State Feedback)
        self.declare_parameter('k_v_x', 0.0)
        self.declare_parameter('k_v_y', 0.0)

        self.k_theta_y = self.get_parameter('k_theta_y').get_parameter_value().double_value
        self.k_theta_dot_y = self.get_parameter('k_theta_dot_y').get_parameter_value().double_value
        self.k_theta_x = self.get_parameter('k_theta_x').get_parameter_value().double_value
        self.k_theta_dot_x = self.get_parameter('k_theta_dot_x').get_parameter_value().double_value
        
        self.k_v_x = self.get_parameter('k_v_x').get_parameter_value().double_value
        self.k_v_y = self.get_parameter('k_v_y').get_parameter_value().double_value

        # --- Alpha-Beta Filter State Persistence ---
        self.alpha = 0.4 
        self.beta = 0.05 
        
        self.theta_x_est = 0.0
        self.theta_x_vel_est = 0.0
        self.theta_y_est = 0.0
        self.theta_y_vel_est = 0.0

        self.raw_vx = 0.0 
        self.raw_vy = 0.0
        self.vision_is_valid = False

        self.last_theta_time = None

        # --- Network I/O (Redis Architecture) ---
        self.redis_publisher = RedisTelemetryPublisher(
            stream_name=f'telemetry:{self.drone_id}:velocity',
            logger=self.get_logger()
        )

        self.raw_vision_stream = f'telemetry:{self.drone_id}:raw_optical_flow'
        self.redis_subscriber = RedisTelemetrySubscriber(
            streams=[self.raw_vision_stream],
            logger=self.get_logger()
        )

        # --- Subscriptions ---
        self.payload_angle_sub = self.create_subscription(
            Vector3, '/payload/swing_angle', self.angle_callback, 10
        )

        # --- Control Loop ---
        self.control_timer = self.create_timer(0.02, self.lqr_control_loop) # 50Hz

    def angle_callback(self, msg):
        current_time = self.get_clock().now()
        
        if self.last_theta_time is not None:
            dt = (current_time - self.last_theta_time).nanoseconds / 1e9
            if dt > 0:
                raw_x = msg.x
                raw_y = msg.y

                # 1. Prediction Step
                pred_x = self.theta_x_est + (self.theta_x_vel_est * dt)
                pred_y = self.theta_y_est + (self.theta_y_vel_est * dt)

                # 2. Calculate Residual
                residual_x = raw_x - pred_x
                residual_y = raw_y - pred_y

                # 3. Update Step
                self.theta_x_est = pred_x + (self.alpha * residual_x)
                self.theta_x_vel_est = self.theta_x_vel_est + ((self.beta / dt) * residual_x)

                self.theta_y_est = pred_y + (self.alpha * residual_y)
                self.theta_y_vel_est = self.theta_y_vel_est + ((self.beta / dt) * residual_y)
                
        self.last_theta_time = current_time

    def lqr_control_loop(self):
        current_time_sec = self.get_clock().now().nanoseconds / 1e9

        # Poll Redis for the latest raw perception data
        vision_payload = self.redis_subscriber.get_latest(self.raw_vision_stream)

        if vision_payload and vision_payload.get('is_valid', 'True') == 'True':
            self.raw_vx = float(vision_payload.get('linear_x', 0.0))
            self.raw_vy = float(vision_payload.get('linear_y', 0.0))
            self.vision_is_valid = True
        else:
            self.vision_is_valid = False

        if not self.vision_is_valid:
            self.redis_publisher.send_velocity_vector(
                self.drone_id, current_time_sec, 0.0, 0.0, 0.0, 0.0, is_valid=False
            )
            return

        # 1. Compute Full-State LQR Control Effort (u = -K * x)
        accel_x_correction = -(
            self.k_theta_x * self.theta_x_est + 
            self.k_theta_dot_x * self.theta_x_vel_est + 
            self.k_v_x * self.raw_vx
        )

        accel_y_correction = -(
            self.k_theta_y * self.theta_y_est + 
            self.k_theta_dot_y * self.theta_y_vel_est + 
            self.k_v_y * self.raw_vy
        )

        # 2. Integrate Acceleration into a Velocity Command
        dt = 0.02
        lqr_vx = accel_x_correction * dt
        lqr_vy = accel_y_correction * dt

        # 3. Superposition (Combine Base Hover + LQR Correction)
        final_vx = self.raw_vx + lqr_vx
        final_vy = self.raw_vy + lqr_vy

        # 4. Push to the Master State Machine via Redis
        self.redis_publisher.send_velocity_vector(
            self.drone_id, 
            current_time_sec, 
            final_vx, 
            final_vy, 
            0.0,  
            0.0, 
            is_valid=True
        )

def main(args=None):
    rclpy.init(args=args)
    node = AntiSwayFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Anti-Sway Filter...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()