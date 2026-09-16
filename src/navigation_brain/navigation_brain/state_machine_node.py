#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandHome, CommandTOL
from geographic_msgs.msg import GeoPointStamped
from swarm_utils.redis_bridge import RedisTelemetrySubscriber


class StateMachineNode(Node):
    def __init__(self):
        super().__init__('state_machine_node')

        self.declare_parameter('drone_id', 'drone_00')
        self.drone_id = self.get_parameter('drone_id').get_parameter_value().string_value

        self.origin_pub = self.create_publisher(
            GeoPointStamped, '/mavros/global_position/set_gp_origin', 10
        )

        # MAVROS Service Clients
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.set_home_client = self.create_client(CommandHome, '/mavros/cmd/set_home')
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self.pos_setpoint_pub = self.create_publisher(
            PoseStamped, '/mavros/setpoint_position/local', 10
        )

        self.mavros_state_sub = self.create_subscription(
            State, '/mavros/state', self.mavros_state_cb, 10
        )

        # Ingest AGL altitude from Redis
        self.altitude_stream = f'telemetry:{self.drone_id}:altitude'
        self.redis_sub = RedisTelemetrySubscriber(
            streams=[self.altitude_stream],
            logger=self.get_logger()
        )

        self.target_pose_x = 0.0
        self.target_pose_y = 0.0
        self.target_pose_z = 2.0

        self.target_sub = self.create_subscription(
            PoseStamped, '/command/target_pose', self.target_pose_cb, 10
        )

        self.flight_state = 'BOOTING'
        self.current_mavros_state = State()
        self.boot_start_time = self.get_clock().now().nanoseconds / 1e9

        self.current_z = 0.05
        self.target_takeoff_alt = 2.0
        self.home_service_sent = False
        self.guided_requested = False

        self.create_timer(1.0, self.handshake_timer_cb)
        self.create_timer(0.05, self.control_loop_cb)
        self.get_logger().info(f'Clean GUIDED State Machine initialized for {self.drone_id}.')

    def target_pose_cb(self, msg: PoseStamped):
        self.target_pose_x = msg.pose.position.x
        self.target_pose_y = msg.pose.position.y
        self.target_pose_z = msg.pose.position.z
        self.get_logger().info(f'New target waypoint received: ({self.target_pose_x:.2f}, {self.target_pose_y:.2f}, {self.target_pose_z:.2f})')

    def mavros_state_cb(self, msg: State):
        self.current_mavros_state = msg

    def control_loop_cb(self):
        if self.flight_state == 'RELINQUISHED':
            return

        # 1. Mode Overwatch & Airborne Armed Integrity
        if self.flight_state in ('ARMED', 'TAKEOFF_IN_PROGRESS', 'FLYING'):
            if self.current_mavros_state.mode != 'GUIDED':
                self.get_logger().warn(
                    f'Mode deviation ({self.current_mavros_state.mode} != GUIDED). Relinquishing control.'
                )
                self.flight_state = 'RELINQUISHED'
                return

            if self.flight_state == 'FLYING' and not self.current_mavros_state.armed:
                self.get_logger().warn('Disarmed while FLYING. Relinquishing control.')
                self.flight_state = 'RELINQUISHED'
                return

        # 2. Ingest Altitude with Freshness Gate (< 0.5s)
        alt_payload = self.redis_sub.get_latest(self.altitude_stream)
        alt_fresh = False
        if alt_payload:
            now_sec = self.get_clock().now().nanoseconds * 1e-9
            stamp = float(alt_payload.get('timestamp', 0.0))
            if 0.0 <= (now_sec - stamp) <= 0.5:
                self.current_z = float(alt_payload.get('z', self.current_z))
                alt_fresh = True

        if self.flight_state == 'TAKEOFF_IN_PROGRESS':
            if alt_fresh and self.current_z >= (self.target_takeoff_alt - 0.2):
                self.get_logger().info(f'Takeoff target reached ({self.current_z:.2f}m). Locking 2.0m HOVER.')
                self.flight_state = 'FLYING'

        elif self.flight_state == 'FLYING':
            hold_msg = PoseStamped()
            hold_msg.header.stamp = self.get_clock().now().to_msg()
            hold_msg.header.frame_id = 'map'
            hold_msg.pose.position.x = self.target_pose_x
            hold_msg.pose.position.y = self.target_pose_y
            hold_msg.pose.position.z = self.target_pose_z
            hold_msg.pose.orientation.z = -0.7071
            hold_msg.pose.orientation.w = 0.7071
            self.pos_setpoint_pub.publish(hold_msg)

    def handshake_timer_cb(self):
        if self.flight_state in ('FLYING', 'RELINQUISHED', 'TAKEOFF_IN_PROGRESS'):
            return

        now_sec = self.get_clock().now().nanoseconds / 1e9
        elapsed = now_sec - self.boot_start_time

        # Bounded origin publishing: only publish until home is acquired
        if self.flight_state in ('BOOTING', 'HOME_ACQUIRED'):
            origin_msg = GeoPointStamped()
            origin_msg.header.stamp = self.get_clock().now().to_msg()
            origin_msg.header.frame_id = 'earth'
            origin_msg.position.latitude = -35.363261
            origin_msg.position.longitude = 149.165230
            origin_msg.position.altitude = 584.0
            self.origin_pub.publish(origin_msg)

        if self.flight_state == 'BOOTING':
            if elapsed > 10.0 and not self.home_service_sent:
                if self.set_home_client.service_is_ready() and not self.current_mavros_state.armed:
                    self.get_logger().info('Locking Home reference...')
                    req = CommandHome.Request()
                    req.current_gps = False
                    req.latitude = -35.363261
                    req.longitude = 149.165230
                    req.altitude = 584.0
                    req.yaw = 0.0
                    future = self.set_home_client.call_async(req)
                    future.add_done_callback(self.home_resp_cb)
                    self.home_service_sent = True

        elif self.flight_state == 'HOME_ACQUIRED':
            if not self.guided_requested and self.set_mode_client.service_is_ready():
                self.get_logger().info('Home confirmed. Requesting GUIDED mode...')
                mode_req = SetMode.Request()
                mode_req.custom_mode = 'GUIDED'
                future = self.set_mode_client.call_async(mode_req)
                future.add_done_callback(self.guided_mode_cb)
                self.guided_requested = True

        elif self.flight_state == 'GUIDED_LOCKED':
            if self.current_mavros_state.mode != 'GUIDED':
                if self.set_mode_client.service_is_ready():
                    mode_req = SetMode.Request()
                    mode_req.custom_mode = 'GUIDED'
                    self.set_mode_client.call_async(mode_req)
                return

            if not self.current_mavros_state.armed:
                if self.arm_client.service_is_ready():
                    self.get_logger().info('GUIDED confirmed. Requesting Motor Arming...')
                    arm_req = CommandBool.Request()
                    arm_req.value = True
                    future = self.arm_client.call_async(arm_req)
                    future.add_done_callback(self.arm_resp_cb)
                    self.flight_state = 'ARMING_REQUESTED'
            else:
                self.get_logger().info('Vehicle armed. Executing takeoff...')
                self.flight_state = 'ARMED'
                self.execute_takeoff()

    def home_resp_cb(self, future):
        try:
            if future.result().success:
                self.get_logger().info('FCU Home locked.')
                self.flight_state = 'HOME_ACQUIRED'
            else:
                self.home_service_sent = False
        except Exception:
            self.home_service_sent = False

    def guided_mode_cb(self, future):
        try:
            if future.result().mode_sent:
                self.get_logger().info('GUIDED mode locked.')
                self.flight_state = 'GUIDED_LOCKED'
            else:
                self.guided_requested = False
        except Exception:
            self.guided_requested = False

    def arm_resp_cb(self, future):
        try:
            if future.result().success:
                self.get_logger().info('Motors ARMED. Executing takeoff...')
                self.flight_state = 'ARMED'
                self.execute_takeoff()
            else:
                self.flight_state = 'GUIDED_LOCKED'
        except Exception:
            self.flight_state = 'GUIDED_LOCKED'

    def execute_takeoff(self):
        if self.takeoff_client.service_is_ready():
            req = CommandTOL.Request()
            req.altitude = float(self.target_takeoff_alt)
            future = self.takeoff_client.call_async(req)
            future.add_done_callback(self.takeoff_resp_cb)

    def takeoff_resp_cb(self, future):
        try:
            if future.result().success:
                self.get_logger().info(f'Takeoff accepted! Climbing to {self.target_takeoff_alt:.1f}m.')
                self.flight_state = 'TAKEOFF_IN_PROGRESS'
            else:
                self.get_logger().error('Takeoff rejected by FCU. Relinquishing control.')
                self.flight_state = 'RELINQUISHED'
        except Exception as e:
            self.get_logger().error(f'Takeoff call failed: {e}. Relinquishing control.')
            self.flight_state = 'RELINQUISHED'


def main(args=None):
    rclpy.init(args=args)
    node = StateMachineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
