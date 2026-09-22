#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class SimRadarEmulatorNode(Node):
    def __init__(self):
        super().__init__('sim_radar_emulator_node')
        self.get_logger().info('Sim radar emulator node initialized (stub).')

def main(args=None):
    rclpy.init(args=args)
    node = SimRadarEmulatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
