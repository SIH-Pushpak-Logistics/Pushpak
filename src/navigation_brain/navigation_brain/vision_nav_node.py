#!/usr/bin/env python3
import queue
import threading
import time

import cv2
import redis
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import TwistStamped


class VisionNavigationNode(Node):
    def __init__(self):
        super().__init__('vision_nav_node')

        # --- Swarm Identity ---
        self.declare_parameter('drone_id', 'drone_00')
        self.drone_id = self.get_parameter('drone_id').get_parameter_value().string_value
        self.get_logger().info(f'Initializing Vision Navigation Node for {self.drone_id}...')

        # --- OpenCV Bridge ---
        self.bridge = CvBridge()

        # --- Redis Setup ---
        # Stream name is parameterized — swarm-safe, never hardcoded
        self.telemetry_stream = f'telemetry:{self.drone_id}:velocity'
        try:
            self.redis_client = redis.Redis(host='localhost', port=6379, decode_responses=True)
            self.redis_client.ping()
            self.get_logger().info(
                f'Redis connected. Publishing to stream: {self.telemetry_stream}'
            )
        except redis.RedisError as e:
            self.get_logger().error(f'FATAL: Redis connection failed: {e}')
            raise

        # --- Producer-Consumer Architecture ---
        # Camera thread ONLY puts data into this queue (non-blocking).
        # Worker thread ONLY reads from this queue and writes to Redis.
        # maxsize=5 ensures backpressure: old frames are dropped if worker falls behind.
        self._telemetry_queue: queue.Queue = queue.Queue(maxsize=5)

        # Background worker thread — keeps Redis I/O off the camera thread entirely
        self._worker_thread = threading.Thread(
            target=self._redis_worker,
            name=f'redis_worker_{self.drone_id}',
            daemon=True  # dies automatically when the ROS node shuts down
        )
        self._worker_thread.start()
        self.get_logger().info('Redis background worker thread started.')

        # --- ROS2 Interfaces ---
        # INPUT: down-facing camera feed
        self.camera_subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # OUTPUT: direct velocity publisher (used as fallback / future local control)
        self.velocity_publisher = self.create_publisher(
            TwistStamped,
            '/mavros/setpoint_velocity/cmd_vel',
            10
        )

    # ------------------------------------------------------------------
    # CAMERA THREAD  (runs inside ROS executor — must never block)
    # ------------------------------------------------------------------

    def image_callback(self, msg):
        """
        High-frequency callback. Rules:
          - No blocking I/O (no Redis, no heavy compute that can stall).
          - No get_logger().info() — use debug() only.
          - Enqueue telemetry; drop silently if queue is full (backpressure).
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Translation Failure: {str(e)}')
            return

        # Run vision pipeline — pure CPU/OpenCV, no network I/O
        velocity_vector = self.process_vision_pipeline(cv_image)

        self.velocity_publisher.publish(velocity_vector)

        # Build telemetry payload
        payload = {
            'timestamp': str(time.time()),
            'linear_x':  str(velocity_vector.twist.linear.x),
            'linear_y':  str(velocity_vector.twist.linear.y),
            'linear_z':  str(velocity_vector.twist.linear.z),
            'angular_z': str(velocity_vector.twist.angular.z),
        }

        # Non-blocking enqueue — if queue is full, drop this frame (not the worker)
        try:
            self._telemetry_queue.put_nowait(payload)
        except queue.Full:
            # Backpressure: worker is behind; silently drop oldest-frame candidate
            self.get_logger().debug('Telemetry queue full — frame dropped.')

    # ------------------------------------------------------------------
    # VISION PIPELINE  (pure OpenCV, no side-effects)
    # ------------------------------------------------------------------

    def process_vision_pipeline(self, cv_frame):
        """
        Scope: OpenCV algorithms only.
        Input:  cv_frame — BGR numpy array from camera
        Output: geometry_msgs/TwistStamped velocity command
        """
        cmd_msg = TwistStamped()
        cmd_msg.header.stamp = self.get_clock().now().to_msg()
        cmd_msg.header.frame_id = 'base_link'

        # Default safety state — hover in place
        cmd_msg.twist.linear.x = 0.0
        cmd_msg.twist.linear.y = 0.0
        cmd_msg.twist.linear.z = 0.0
        cmd_msg.twist.angular.z = 0.0

        obstacles = self.detect_obstacles(cv_frame)
        motion    = self.compute_optical_flow(cv_frame)
        cmd_msg   = self.calculate_safe_velocity(obstacles, motion)

        return cmd_msg

    def detect_obstacles(self, cv_frame):
        """
        Basic edge-based obstacle detection.
        Returns a dict with obstacle density metric.
        """
        gray        = cv2.cvtColor(cv_frame, cv2.COLOR_BGR2GRAY)
        edges       = cv2.Canny(gray, threshold1=50, threshold2=150)
        edge_pixels = cv2.countNonZero(edges)
        total_pixels = edges.shape[0] * edges.shape[1]

        obstacle_density = edge_pixels / total_pixels if total_pixels > 0 else 0.0

        self.get_logger().debug(f'Obstacle density: {obstacle_density:.4f}')
        return {'density': obstacle_density}

    def compute_optical_flow(self, cv_frame):
        """
        Future optical flow logic.
        """
        return {}

    def calculate_safe_velocity(self, obstacles, motion):
        """
        Future navigation decision logic.
        """
        cmd_msg = TwistStamped()
        cmd_msg.header.stamp = self.get_clock().now().to_msg()
        cmd_msg.header.frame_id = 'base_link'

        cmd_msg.twist.linear.x  = 0.0
        cmd_msg.twist.linear.y  = 0.0
        cmd_msg.twist.linear.z  = 0.0
        cmd_msg.twist.angular.z = 0.0

        return cmd_msg

    # ------------------------------------------------------------------
    # BACKGROUND WORKER THREAD  (Redis I/O lives here — never in camera thread)
    # ------------------------------------------------------------------

    def _redis_worker(self):
        """
        Dedicated thread that drains the telemetry queue and publishes
        each payload to the Redis stream. All Redis latency is absorbed here,
        away from the camera callback.
        """
        self.get_logger().debug('Redis worker: entering loop.')
        while True:
            try:
                # Block until a payload is available (no busy-wait)
                payload = self._telemetry_queue.get(block=True, timeout=1.0)
            except queue.Empty:
                # Timeout is normal — just loop and wait for next frame
                continue

            try:
                self.redis_client.xadd(
                    self.telemetry_stream,
                    payload,
                    maxlen=100,   # cap stream length to avoid unbounded memory growth
                    approximate=True
                )
                self.get_logger().debug(
                    f'Telemetry published to {self.telemetry_stream}'
                )
            except redis.RedisError as e:
                # Specific — not broad Exception; log and continue, do not crash
                self.get_logger().error(f'Redis publish failed: {e}')
            finally:
                self._telemetry_queue.task_done()


# ----------------------------------------------------------------------
# Entry Point
# ----------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = VisionNavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Vision Navigation Node cleanly.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
