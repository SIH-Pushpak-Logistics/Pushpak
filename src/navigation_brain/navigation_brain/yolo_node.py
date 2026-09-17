#!/usr/bin/env python3
"""Victim detection node. Publishes to detections:{drone_id}.

RAUNAK: everything except run_inference() is done. Fill in that one method.
Contract: docs/STREAM_CONTRACTS.md
"""
import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError
from rclpy.qos import qos_profile_sensor_data
import cv2

from swarm_utils.redis_bridge import RedisTelemetryPublisher

DETECTION_DIR = '/workspace/detections'


class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_node')

        self.declare_parameter('drone_id', 'drone_00')
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('confidence', 0.4)
        self.declare_parameter('max_rate_hz', 5.0)
        self.drone_id = self.get_parameter('drone_id').get_parameter_value().string_value
        self.model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.conf_threshold = self.get_parameter('confidence').get_parameter_value().double_value
        self.min_interval = 1.0 / self.get_parameter('max_rate_hz').get_parameter_value().double_value

        os.makedirs(DETECTION_DIR, exist_ok=True)

        self.bridge = CvBridge()
        self.last_infer_sec = 0.0
        self.det_counter = 0
        self.drone_x = 0.0
        self.drone_y = 0.0
        self.drone_z = 0.0

        self.redis_publisher = RedisTelemetryPublisher(
            stream_name=f'detections:{self.drone_id}',
            logger=self.get_logger()
        )

        self.model = self.load_model()

        self.create_subscription(
            Image, '/camera/image_raw', self.image_cb, qos_profile_sensor_data)
        self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self.pose_cb, 10)

        self.get_logger().info(
            f'yolo_node active: conf>={self.conf_threshold}, '
            f'max {1.0 / self.min_interval:.1f} Hz -> detections:{self.drone_id}')

    def pose_cb(self, msg: PoseStamped):
        self.drone_x = msg.pose.position.x
        self.drone_y = msg.pose.position.y
        self.drone_z = msg.pose.position.z

    # ------------------------------------------------------------------
    # RAUNAK: these two methods are yours.
    # ------------------------------------------------------------------
    def load_model(self):
        try:
            from ultralytics import YOLO
            model = YOLO(self.model_path)
            self.get_logger().info(f'loaded model {self.model_path}')
            return model
        except Exception as exc:
            self.get_logger().error(f'model load failed: {exc}. Running in NO-OP mode.')
            return None

    def run_inference(self, frame):
        """Return raw person detections in the stream contract format.

        {'class_name': 'person', 'confidence': float,
         'bbox': (x, y, w, h)}

        Coordinates are pixels with a top-left origin.
        Returns [] on any inference/parsing failure and never raises.
        """
        if self.model is None:
            return []

        try:
            results = self.model(frame, verbose=False)
            out = []

            for result in results:
                names = result.names

                for box in result.boxes:
                    conf = float(box.conf[0])

                    if conf < self.conf_threshold:
                        continue

                    class_id = int(box.cls[0])
                    class_name = names[class_id]

                    if class_name != 'person':
                        continue

                    x1, y1, x2, y2 = (
                        float(value) for value in box.xyxy[0]
                    )

                    width = x2 - x1
                    height = y2 - y1

                    if width <= 0 or height <= 0:
                        continue

                    out.append({
                        'class_name': 'person',
                        'confidence': conf,
                        'bbox': (x1, y1, width, height),
                    })

            return out

        except Exception as exc:
            self.get_logger().warn(f'inference failed: {exc}')
            return []

    # ------------------------------------------------------------------

    def image_cb(self, msg: Image):
        stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if stamp_sec - self.last_infer_sec < self.min_interval:
            return
        self.last_infer_sec = stamp_sec

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as exc:
            self.get_logger().error(f'cv_bridge: {exc}')
            return

        for det in self.run_inference(frame):
            self.det_counter += 1
            det_id = f'{self.drone_id}_det_{self.det_counter:05d}'
            x, y, w, h = det['bbox']
            crop_path = os.path.join(DETECTION_DIR, f'{det_id}.jpg')
            try:
                ih, iw = frame.shape[:2]
                x0, y0 = max(0, int(x)), max(0, int(y))
                x1, y1 = min(iw, int(x + w)), min(ih, int(y + h))
                if x1 > x0 and y1 > y0:
                    cv2.imwrite(crop_path, frame[y0:y1, x0:x1])
            except Exception as exc:
                self.get_logger().warn(f'crop save failed: {exc}')

            self.redis_publisher.send_payload(
                self.drone_id, stamp_sec,
                det_id=det_id,
                class_name=det['class_name'],
                confidence=float(det['confidence']),
                bbox_x=float(x), bbox_y=float(y),
                bbox_w=float(w), bbox_h=float(h),
                drone_x=float(self.drone_x),
                drone_y=float(self.drone_y),
                drone_z=float(self.drone_z),
                image_path=crop_path,
            )
            self.get_logger().info(
                f'{det_id} {det["class_name"]} {det["confidence"]:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = YoloNode()
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