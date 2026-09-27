#!/usr/bin/env python3
import math
import statistics

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from geographic_msgs.msg import GeoPointStamped
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import HomePosition, State, StatusText
from mavros_msgs.srv import CommandBool, CommandHome, CommandTOL, SetMode
from sensor_msgs.msg import Range
from std_msgs.msg import Bool


class ArmTakeoffHandshake(Node):
    def __init__(self):
        super().__init__('arm_takeoff_handshake')
        self.takeoff_alt_m = self.declare_parameter('takeoff_alt_m', 1.5).value
        self.airborne_margin_m = self.declare_parameter('airborne_margin_m', 0.2).value
        self.settle_window_s = self.declare_parameter('settle_window_s', 1.0).value
        self.settle_band_m = self.declare_parameter('settle_band_m', 0.05).value
        self.retry_period_s = self.declare_parameter('retry_period_s', 2.0).value
        self.fs4_timeout_s = self.declare_parameter('fs4_timeout_s', 1.0).value
        self.extnav_timeout_s = self.declare_parameter('extnav_timeout_s', 0.5).value
        self.ground_samples = self.declare_parameter('ground_samples', 20).value
        self.stuck_warn_s = self.declare_parameter('stuck_warn_s', 30.0).value
        self.origin_lat = self.declare_parameter('origin_lat', -35.363261).value
        self.origin_lon = self.declare_parameter('origin_lon', 149.165230).value
        self.origin_alt = self.declare_parameter('origin_alt', 584.0).value

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.airborne_pub = self.create_publisher(Bool, '/pushpak/airborne', latched)
        self.origin_pub = self.create_publisher(GeoPointStamped, '/mavros/global_position/set_gp_origin', 10)

        self.fcu = None
        self.extnav_time = None
        self.local_pose_time = None
        self.home_seen = False
        self.origin_sends = 0
        self.tof_ground = []
        self.tof_hist = []
        self.tof_latest = None
        self.tof_zero = None
        self.last_cmd_time = None

        self.create_subscription(State, '/mavros/state', self.state_cb, 10)
        self.create_subscription(PoseStamped, '/mavros/vision_pose/pose', self.extnav_cb, 10)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.local_cb,
                                 qos_profile_sensor_data)
        self.create_subscription(HomePosition, '/mavros/home_position/home', self.home_cb, 10)
        self.create_subscription(Range, '/drone/tof_range', self.tof_cb, qos_profile_sensor_data)
        self.create_subscription(TwistStamped, '/mavros/setpoint_velocity/cmd_vel', self.cmd_cb, 10)
        self.create_subscription(StatusText, '/mavros/statustext/recv', self.statustext_cb, 10)

        self.srv = {
            'mode': self.create_client(SetMode, '/mavros/set_mode'),
            'arm': self.create_client(CommandBool, '/mavros/cmd/arming'),
            'home': self.create_client(CommandHome, '/mavros/cmd/set_home'),
            'takeoff': self.create_client(CommandTOL, '/mavros/cmd/takeoff'),
        }
        self.futures = {}
        self.last_sent = {}

        self.phase = 'WAIT_LINK'
        self.phase_start = self.get_clock().now()
        self.stuck_warned = False
        self.create_timer(0.1, self.tick)
        self.get_logger().info(
            f'handshake: takeoff {self.takeoff_alt_m} m, FS-4 timeout {self.fs4_timeout_s} s, '
            f'origin ({self.origin_lat}, {self.origin_lon}, {self.origin_alt})')

    def age(self, t):
        return (self.get_clock().now() - t).nanoseconds * 1e-9

    def state_cb(self, msg):
        self.fcu = msg

    def extnav_cb(self, msg):
        self.extnav_time = self.get_clock().now()

    def local_cb(self, msg):
        self.local_pose_time = self.get_clock().now()

    def home_cb(self, msg):
        self.home_seen = True

    def cmd_cb(self, msg):
        self.last_cmd_time = self.get_clock().now()

    def statustext_cb(self, msg):
        self.get_logger().info(f'handshake FCU: {msg.text}')

    def tof_cb(self, msg):
        if not math.isfinite(msg.range):
            return
        now = self.get_clock().now()
        self.tof_latest = msg.range
        self.tof_hist.append((now, msg.range))
        self.tof_hist = [(t, r) for t, r in self.tof_hist if self.age(t) <= self.settle_window_s]
        if self.phase == 'WAIT_LINK':
            self.tof_ground.append(msg.range)
            self.tof_ground = self.tof_ground[-self.ground_samples:]

    def settled(self):
        values = [r for _, r in self.tof_hist]
        return len(values) >= 10 and max(values) - min(values) < self.settle_band_m

    def goto(self, phase, why):
        self.get_logger().info(f'handshake: {self.phase} -> {phase} ({why})')
        self.phase = phase
        self.phase_start = self.get_clock().now()
        self.stuck_warned = False

    def due(self, name):
        t = self.last_sent.get(name)
        return t is None or self.age(t) >= self.retry_period_s

    def request(self, name, req):
        self.last_sent[name] = self.get_clock().now()
        client = self.srv[name]
        if not client.service_is_ready():
            self.get_logger().warn(f'handshake: {name} service not ready')
            return
        self.futures[name] = client.call_async(req)

    def publish_origin(self):
        msg = GeoPointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.position.latitude = self.origin_lat
        msg.position.longitude = self.origin_lon
        msg.position.altitude = self.origin_alt
        self.origin_pub.publish(msg)
        self.origin_sends += 1

    def tick(self):
        if (self.phase not in ('FLYING', 'DONE') and not self.stuck_warned
                and self.age(self.phase_start) > self.stuck_warn_s):
            self.stuck_warned = True
            self.get_logger().warn(f'handshake: still in {self.phase} after {self.stuck_warn_s:.0f} s')

        if self.phase == 'WAIT_LINK':
            if (self.fcu is not None and self.fcu.connected
                    and self.extnav_time is not None and self.age(self.extnav_time) < self.extnav_timeout_s
                    and len(self.tof_ground) >= self.ground_samples):
                self.tof_zero = statistics.median(self.tof_ground)
                self.goto('ORIGIN', f'link up, ExtNav flowing, ToF on ground {self.tof_zero:.3f} m')

        elif self.phase == 'ORIGIN':
            if self.origin_sends > 0 and self.home_seen:
                self.goto('WAIT_POSITION', 'origin sent, home reported')
            elif self.due('home'):
                self.publish_origin()
                self.request('home', CommandHome.Request(
                    current_gps=False, latitude=self.origin_lat,
                    longitude=self.origin_lon, altitude=self.origin_alt))

        elif self.phase == 'WAIT_POSITION':
            if self.local_pose_time is not None and self.age(self.local_pose_time) < 1.0:
                self.goto('GUIDED', 'FCU reports a local position')

        elif self.phase == 'GUIDED':
            if self.fcu.mode == 'GUIDED':
                self.goto('ARM', 'mode GUIDED')
            elif self.due('mode'):
                self.request('mode', SetMode.Request(custom_mode='GUIDED'))

        elif self.phase == 'ARM':
            if self.fcu.armed:
                self.goto('TAKEOFF', 'armed')
            elif self.fcu.mode != 'GUIDED':
                self.goto('GUIDED', f'mode fell back to {self.fcu.mode}')
            elif self.due('arm'):
                self.request('arm', CommandBool.Request(value=True))

        elif self.phase == 'TAKEOFF':
            f = self.futures.get('takeoff')
            if f is not None and f.done() and f.result() is not None and f.result().success:
                self.goto('CLIMB', f'takeoff to {self.takeoff_alt_m} m accepted')
            elif not self.fcu.armed:
                self.goto('ARM', 'disarmed before takeoff was accepted')
            elif self.due('takeoff'):
                self.request('takeoff', CommandTOL.Request(altitude=float(self.takeoff_alt_m)))

        elif self.phase == 'CLIMB':
            rise = None if self.tof_latest is None else self.tof_latest - self.tof_zero
            if not self.fcu.armed:
                self.goto('DONE', 'disarmed during climb')
            elif rise is not None and rise >= self.takeoff_alt_m - self.airborne_margin_m and self.settled():
                self.airborne_pub.publish(Bool(data=True))
                self.last_cmd_time = self.get_clock().now()
                self.goto('FLYING', f'airborne, ToF {self.tof_latest:.3f} m (rise {rise:.3f} m); FS-4 armed')

        elif self.phase == 'FLYING':
            silent = self.age(self.last_cmd_time)
            if self.fcu.armed and self.fcu.mode == 'GUIDED' and silent > self.fs4_timeout_s:
                self.get_logger().error(f'FS-4: no cmd_vel for {silent:.2f} s; commanding LAND')
                self.request('mode', SetMode.Request(custom_mode='LAND'))
                self.goto('DONE', 'FS-4 fired')


def main(args=None):
    rclpy.init(args=args)
    node = ArmTakeoffHandshake()
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
