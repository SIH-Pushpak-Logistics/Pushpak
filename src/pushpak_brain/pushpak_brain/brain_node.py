#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

from pushpak_brain.exploration import guidance_step, lawnmower, path_length, validate_gains

NAN = float('nan')

PARAMS = [
    ('rate_hz', 20.0),
    ('drone_id', -1),
    ('kp', NAN),
    ('v_max_mps', NAN),
    ('explore_x_min_m', NAN),
    ('explore_x_max_m', NAN),
    ('explore_y_min_m', NAN),
    ('explore_y_max_m', NAN),
    ('explore_lane_spacing_m', NAN),
    ('explore_accept_radius_m', NAN),
    ('explore_start_delay_s', NAN),
    ('odom_max_age_s', NAN),
    ('keyframe_rate_hz', 5.0),
    ('keyframe_fifo_len', 200),
    ('survivor_dedup_radius_m', 1.5),
    ('isolation_timeout_s', 2.0),
    ('visual_dropout_variance', 1e6),
    ('backtrack_cov_threshold', 0.2),
    ('backtrack_speed_mps', 1.0),
    ('backtrack_accept_radius_m', 0.4),
]

REQUIRED_POSITIVE = ('explore_lane_spacing_m', 'explore_accept_radius_m', 'odom_max_age_s')


class PushpakBrain(Node):
    def __init__(self):
        super().__init__('pushpak_brain')
        for name, default in PARAMS:
            self.declare_parameter(name, default)
        p = {name: self.get_parameter(name).value for name, _ in PARAMS}
        self.rate_hz = p['rate_hz']
        self.drone_id = p['drone_id']
        if self.drone_id < 0:
            raise RuntimeError('drone_id must be set (README section 8)')
        validate_gains(p['kp'], p['v_max_mps'])
        for name in REQUIRED_POSITIVE:
            if not (math.isfinite(p[name]) and p[name] > 0.0):
                raise RuntimeError(f'{name} must be set in pushpak_params.yaml (I-11), got {p[name]}')
        if not (math.isfinite(p['explore_start_delay_s']) and p['explore_start_delay_s'] >= 0.0):
            raise RuntimeError('explore_start_delay_s must be set in pushpak_params.yaml (I-11)')
        self.kp = p['kp']
        self.v_max = p['v_max_mps']
        self.accept_radius = p['explore_accept_radius_m']
        self.start_delay = p['explore_start_delay_s']
        self.odom_max_age = p['odom_max_age_s']
        self.waypoints = lawnmower(p['explore_x_min_m'], p['explore_x_max_m'],
                                   p['explore_y_min_m'], p['explore_y_max_m'],
                                   p['explore_lane_spacing_m'])
        self.get_logger().info('pushpak_brain params: ' + ', '.join(f'{k}={v}' for k, v in p.items()))
        self.get_logger().info(
            f'lawnmower: {len(self.waypoints)} waypoints, {path_length((0.0, 0.0), self.waypoints):.1f} m '
            f'from origin: {self.waypoints}')

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.airborne = False
        self.airborne_t = None
        self.fcu = None
        self.released = False
        self.odom = None
        self.index = 0
        self.status = None
        self.create_subscription(Bool, '/pushpak/airborne', self.airborne_cb, latched)
        self.create_subscription(State, '/mavros/state', self.state_cb, 10)
        self.create_subscription(Odometry, '/odometry/filtered', self.odom_cb, 10)
        self.cmd_pub = self.create_publisher(TwistStamped, '/mavros/setpoint_velocity/cmd_vel', 10)
        self.create_timer(1.0 / self.rate_hz, self.tick)

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def airborne_cb(self, msg):
        if msg.data and not self.airborne:
            self.airborne_t = self.now_s()
            self.get_logger().info(f'airborne at t={self.airborne_t:.2f}: exploring after '
                                   f'{self.start_delay:.1f} s')
        self.airborne = msg.data

    def state_cb(self, msg):
        self.fcu = msg

    def odom_cb(self, msg):
        self.odom = msg

    def tick(self):
        if not self.airborne or self.released:
            return
        if self.fcu is None or self.fcu.mode != 'GUIDED' or not self.fcu.armed:
            self.released = True
            mode = None if self.fcu is None else self.fcu.mode
            armed = None if self.fcu is None else self.fcu.armed
            self.get_logger().error(
                f'FS-5: FCU mode={mode} armed={armed} while airborne; setpoints stopped, not fighting the FCU')
            return
        now = self.now_s()
        pos, age = None, None
        if self.odom is not None:
            pos = (self.odom.pose.pose.position.x, self.odom.pose.pose.position.y)
            age = now - Time.from_msg(self.odom.header.stamp).nanoseconds * 1e-9
        vx, vy, index, status = guidance_step(
            pos, age, self.odom_max_age, now - self.airborne_t, self.start_delay,
            self.waypoints, self.index, self.kp, self.v_max, self.accept_radius)
        if status != self.status or index != self.index:
            where = 'none' if pos is None else f'({pos[0]:.2f}, {pos[1]:.2f})'
            age_txt = 'none' if age is None else f'{age:.3f}'
            self.get_logger().info(
                f'guidance: {status} waypoint {index}/{len(self.waypoints)} est {where} odom_age {age_txt} s')
        self.status, self.index = status, index
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.twist.linear.x = vx
        msg.twist.linear.y = vy
        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PushpakBrain()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
