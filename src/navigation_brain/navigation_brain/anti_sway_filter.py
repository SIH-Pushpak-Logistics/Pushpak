#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3
from drone_interfaces.msg import OpticalFlow  # FORCED API CONTRACT
import math
from swarm_utils.redis_bridge import RedisTelemetryPublisher

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

        self.k_theta_y = self.get_parameter('k_theta_y').get_parameter_value().double_value
        self.k_theta_dot_y = self.get_parameter('k_theta_dot_y').get_parameter_value().double_value
        self.k_theta_x = self.get_parameter('k_theta_x').get_parameter_value().double_value
        self.k_theta_dot_x = self.get_parameter('k_theta_dot_x').get_parameter_value().double_value

        # --- Alpha-Beta Filter State Persistence ---
        # Alpha controls position tracking (high = trust sensor, low = trust model)
        # Beta controls velocity tracking (high = fast response to changes, low = smooth)
        self.alpha = 0.4 
        self.beta = 0.05 
        
        self.theta_x_est = 0.0
        self.theta_x_vel_est = 0.0
        self.theta_y_est = 0.0
        self.theta_y_vel_est = 0.0

        # Raw velocities from Raunak's Optical Flow
        self.raw_vx = 0.0 
        self.raw_vy = 0.0
        self.vision_is_valid = False

        self.last_theta_time = None

        # --- Network I/O ---
        self.redis_publisher = RedisTelemetryPublisher(
            stream_name=f'telemetry:{self.drone_id}:velocity',
            logger=self.get_logger()
        )

        # --- Subscriptions ---
        # 1. Subscribe to the explicit API contract for optical flow
        self.vision_sub = self.create_subscription(
            OpticalFlow, '/internal/raw_optical_flow', self.vision_callback, 10
        )
        # 2. Subscribe to the secondary camera's payload tracking output
        self.payload_angle_sub = self.create_subscription(
            Vector3, '/payload/swing_angle', self.angle_callback, 10
        )

        # --- Control Loop ---
        self.control_timer = self.create_timer(0.02, self.lqr_control_loop) # 50Hz

    def vision_callback(self, msg):
        """ Ingest explicit optical flow velocity and validity state """
        self.raw_vx = msg.velocity.x
        self.raw_vy = msg.velocity.y
        self.vision_is_valid = msg.is_valid 

    def angle_callback(self, msg):
        """ 
        Ingest theta from the secondary camera and apply Alpha-Beta tracking.
        msg.x = raw theta_x
        msg.y = raw theta_y
        """
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
        """ Computes the control law u = -Kx using Alpha-Beta estimates """
        current_time_sec = self.get_clock().now().nanoseconds / 1e9

        if not self.vision_is_valid:
            self.redis_publisher.send_velocity_vector(
                self.drone_id, current_time_sec, 0.0, 0.0, 0.0, 0.0, is_valid=False
            )
            return

        # 1. Compute LQR Control Effort using Alpha-Beta Estimated States
        # u = -K * x 
        accel_x_correction = -(self.k_theta_x * self.theta_x_est + self.k_theta_dot_x * self.theta_x_vel_est)
        accel_y_correction = -(self.k_theta_y * self.theta_y_est + self.k_theta_dot_y * self.theta_y_vel_est)

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