#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')
        self.declare_parameter('model_path', '/workspace/yolov8n.pt')
        self.declare_parameter('device', 'cuda:0')
        self.declare_parameter('imgsz', 320)
        self.declare_parameter('conf_threshold', 0.85)
        self.declare_parameter('max_rate_hz', 2.0)
        self.get_logger().info('Perception node initialized (stub).')

def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
