#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import State
from std_msgs.msg import Bool

NAN = float('nan')

PARAMS = [
    ('rate_hz', 20.0),
    ('drone_id', -1),
    ('kp', NAN),
    ('v_max_mps', NAN),
    ('keyframe_rate_hz', 5.0),
    ('keyframe_fifo_len', 200),
    ('survivor_dedup_radius_m', 1.5),
    ('isolation_timeout_s', 2.0),
    ('visual_dropout_variance', 1e6),
    ('backtrack_cov_threshold', NAN),
    ('backtrack_speed_mps', 1.0),
    ('backtrack_accept_radius_m', 0.4),
]


class PushpakBrain(Node):
    def __init__(self):
        super().__init__('pushpak_brain')
        for name, default in PARAMS:
            self.declare_parameter(name, default)
        self.rate_hz = self.get_parameter('rate_hz').value
        self.drone_id = self.get_parameter('drone_id').value
        if self.drone_id < 0:
            raise RuntimeError('drone_id must be set (README section 8)')
        self.get_logger().info('pushpak_brain params: ' + ', '.join(
            f'{p.name}={p.value}' for p in self.get_parameters([n for n, _ in PARAMS])))

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.airborne = False
        self.fcu = None
        self.released = False
        self.create_subscription(Bool, '/pushpak/airborne', self.airborne_cb, latched)
        self.create_subscription(State, '/mavros/state', self.state_cb, 10)
        self.cmd_pub = self.create_publisher(TwistStamped, '/mavros/setpoint_velocity/cmd_vel', 10)
        self.create_timer(1.0 / self.rate_hz, self.tick)

    def airborne_cb(self, msg):
        if msg.data and not self.airborne:
            self.get_logger().info('airborne: holding zero velocity')
        self.airborne = msg.data

    def state_cb(self, msg):
        self.fcu = msg

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
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
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
