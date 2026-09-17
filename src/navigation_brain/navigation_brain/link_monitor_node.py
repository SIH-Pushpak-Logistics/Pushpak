#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from swarm_utils.redis_bridge import RedisTelemetryPublisher


class LinkMonitorNode(Node):
    def __init__(self):
        super().__init__('link_monitor_node')

        self.declare_parameter('drone_id', 'drone_00')
        self.declare_parameter('publish_rate_hz', 2.0)
        self.declare_parameter('link_state', 'ONLINE')
        self.declare_parameter('packet_rate_hz', 20.0)
        self.drone_id = self.get_parameter('drone_id').get_parameter_value().string_value
        rate = self.get_parameter('publish_rate_hz').get_parameter_value().double_value
        self.packet_rate = self.get_parameter('packet_rate_hz').get_parameter_value().double_value

        self.redis_publisher = RedisTelemetryPublisher(
            stream_name=f'link:{self.drone_id}:status',
            logger=self.get_logger()
        )

        self.period = 1.0 / rate
        self.state = 'ONLINE'
        self.cached = 0
        self.last_sync = 0.0
        self.create_timer(self.period, self.tick)
        self.get_logger().info(
            f'link_monitor active -> link:{self.drone_id}:status '
            f'(set link_state param to ONLINE/DEGRADED/OFFLINE)')

    def tick(self):
        now = self.get_clock().now().nanoseconds / 1e9
        requested = self.get_parameter('link_state').get_parameter_value().string_value.upper()
        if requested not in ('ONLINE', 'DEGRADED', 'OFFLINE'):
            requested = 'ONLINE'

        if requested == 'OFFLINE':
            self.cached += int(self.packet_rate * self.period)
        elif self.state == 'OFFLINE':
            self.get_logger().info(f'link restored, flushing {self.cached} cached packets')
            self.cached = 0
            self.last_sync = now
        elif requested == 'ONLINE':
            self.last_sync = now

        if requested != self.state:
            self.get_logger().warn(f'link {self.state} -> {requested}')
        self.state = requested

        rssi = {'ONLINE': -55.0, 'DEGRADED': -82.0, 'OFFLINE': -110.0}[self.state]
        self.redis_publisher.send_payload(
            self.drone_id, now,
            state=self.state,
            cached_packets=self.cached,
            last_sync_sec=float(self.last_sync),
            rssi_dbm=float(rssi))


def main(args=None):
    rclpy.init(args=args)
    node = LinkMonitorNode()
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