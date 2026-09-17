#!/usr/bin/env python3
import json
import redis
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

class RedisToMarkerNode(Node):
    def __init__(self):
        super().__init__('redis_to_marker_node')
        self.declare_parameter('drone_id', 'drone_00')
        self.declare_parameter('rate_hz', 2.0)
        self.drone_id = self.get_parameter('drone_id').get_parameter_value().string_value
        rate = self.get_parameter('rate_hz').get_parameter_value().double_value

        self.client = redis.Redis(
            host='localhost', port=6379, decode_responses=True,
            socket_timeout=0.2, socket_connect_timeout=0.5
        )
        self.victims_key = f'victims:{self.drone_id}'
        self.pub = self.create_publisher(MarkerArray, '/visualization/victims', 10)
        self.create_timer(1.0 / rate, self.timer_cb)
        self.get_logger().info(f'Redis-to-Marker bridge active: {self.victims_key} -> /visualization/victims')

    def timer_cb(self):
        try:
            records = self.client.hgetall(self.victims_key)
        except redis.RedisError:
            return

        if not records:
            return

        array = MarkerArray()
        now = self.get_clock().now().to_msg()

        for idx, (vid, val_str) in enumerate(records.items()):
            try:
                data = json.loads(val_str)
            except Exception:
                continue

            marker = Marker()
            marker.header.stamp = now
            marker.header.frame_id = 'map'
            marker.ns = 'victims'
            marker.id = idx
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = float(data.get('world_x', 0.0))
            marker.pose.position.y = float(data.get('world_y', 0.0))
            marker.pose.position.z = 0.5
            marker.scale.x = 0.6
            marker.scale.y = 0.6
            marker.scale.z = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.1
            marker.color.b = 0.1
            marker.color.a = 0.8
            array.markers.append(marker)

        self.pub.publish(array)

def main(args=None):
    rclpy.init(args=args)
    node = RedisToMarkerNode()
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
