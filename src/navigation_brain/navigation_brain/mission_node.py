#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from swarm_utils.redis_bridge import RedisTelemetryPublisher


class MissionNode(Node):
    def __init__(self):
        super().__init__('mission_node')

        self.declare_parameter('drone_id', 'drone_00')
        self.declare_parameter('target_altitude', 0.0)
        self.declare_parameter('reset', False)

        self.drone_id = self.get_parameter('drone_id').get_parameter_value().string_value

        self.target_publisher = RedisTelemetryPublisher(
            stream_name=f'telemetry:{self.drone_id}:target_altitude',
            logger=self.get_logger()
        )
        self.command_publisher = RedisTelemetryPublisher(
            stream_name=f'telemetry:{self.drone_id}:commands',
            logger=self.get_logger()
        )

        self.timer = self.create_timer(0.5, self.publish_loop)
        self.get_logger().info(
            f'Mission node up. Set target with: '
            f'ros2 param set /mission_node target_altitude 5.0'
        )

    def publish_loop(self):
        now_sec = self.get_clock().now().nanoseconds / 1e9

        target = self.get_parameter('target_altitude').get_parameter_value().double_value
        reset = self.get_parameter('reset').get_parameter_value().bool_value

        self.target_publisher.send_payload(
            self.drone_id, now_sec, target_altitude=float(target)
        )
        self.command_publisher.send_payload(
            self.drone_id, now_sec, reset=str(reset)
        )


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()