#!/usr/bin/env python3
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool


class SimRadarEmulatorNode(Node):
    def __init__(self):
        super().__init__('sim_radar_emulator_node')
        self.sigma = self.declare_parameter('sigma', 0.15).value
        self.bias = np.array([
            self.declare_parameter('bias_initial_x', 0.0).value,
            self.declare_parameter('bias_initial_y', 0.0).value,
            self.declare_parameter('bias_initial_z', 0.0).value,
        ], dtype=float)
        self.sigma_bias = self.declare_parameter('sigma_bias', 0.0).value
        seed = self.declare_parameter('seed', 1).value
        rate_hz = self.declare_parameter('rate_hz', 20.0).value
        self.stale_timeout_s = self.declare_parameter('stale_timeout_s', 0.2).value
        self.fault_sigma_factor = self.declare_parameter('fault_sigma_factor', 10.0).value
        self.inflated_variance = self.declare_parameter('inflated_variance', 1e6).value

        noise_seq, bias_seq = np.random.SeedSequence(seed).spawn(2)
        self.rng_noise = np.random.default_rng(noise_seq)
        self.rng_bias = np.random.default_rng(bias_seq)

        self.latest = None
        self.fault = False
        self.last_tick = None

        self.create_subscription(Odometry, '/sim/ground_truth/odom', self.odom_cb, 10)
        self.create_subscription(Bool, '/sim/fault/radar', self.fault_cb, 10)
        self.pub = self.create_publisher(TwistWithCovarianceStamped, '/radar/ego_velocity', 10)
        self.create_timer(1.0 / rate_hz, self.tick)
        self.get_logger().info(
            f'sim radar: sigma={self.sigma} bias_initial={self.bias.tolist()} '
            f'sigma_bias={self.sigma_bias} seed={seed} rate_hz={rate_hz}')

    def odom_cb(self, msg):
        self.latest = msg

    def fault_cb(self, msg):
        if msg.data != self.fault:
            self.get_logger().warn(f'radar fault injection: {msg.data}')
        self.fault = msg.data

    def tick(self):
        now = self.get_clock().now()
        if self.last_tick is not None and self.sigma_bias > 0.0:
            dt = (now - self.last_tick).nanoseconds * 1e-9
            if dt > 0.0:
                self.bias += self.rng_bias.normal(0.0, self.sigma_bias * math.sqrt(dt), 3)
        self.last_tick = now

        noise = self.rng_noise.normal(0.0, 1.0, 3)
        out = TwistWithCovarianceStamped()
        out.header.frame_id = 'base_link'

        fresh = False
        if self.latest is not None:
            age = (now - Time.from_msg(self.latest.header.stamp)).nanoseconds * 1e-9
            fresh = 0.0 <= age <= self.stale_timeout_s

        if fresh:
            v = self.latest.twist.twist.linear
            sigma_eff = self.sigma * (self.fault_sigma_factor if self.fault else 1.0)
            meas = np.array([v.x, v.y, v.z]) + self.bias + sigma_eff * noise
            variance = self.inflated_variance if self.fault else self.sigma ** 2
            out.header.stamp = self.latest.header.stamp
        else:
            meas = np.zeros(3)
            variance = self.inflated_variance
            out.header.stamp = now.to_msg()

        out.twist.twist.linear.x = float(meas[0])
        out.twist.twist.linear.y = float(meas[1])
        out.twist.twist.linear.z = float(meas[2])
        cov = [0.0] * 36
        for i in (0, 7, 14):
            cov[i] = float(variance)
        for i in (21, 28, 35):
            cov[i] = float(self.inflated_variance)
        out.twist.covariance = cov
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = SimRadarEmulatorNode()
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
