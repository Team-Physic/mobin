#!/usr/bin/env python3

# Copyright 2026 JungSeong
# Licensed under the Apache License, Version 2.0

from pathlib import Path

from cv_bridge import CvBridge
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D
from vision_msgs.msg import Detection2DArray
from vision_msgs.msg import ObjectHypothesisWithPose


def build_detection_array(header, detections):
    """검출 결과 행을 vision_msgs Detection2DArray로 바꾼다."""
    message = Detection2DArray(header=header)
    for class_id, score, x_min, y_min, x_max, y_max in detections:
        detection = Detection2D(header=header)
        detection.bbox.center.position.x = (x_min + x_max) * 0.5
        detection.bbox.center.position.y = (y_min + y_max) * 0.5
        detection.bbox.size_x = x_max - x_min
        detection.bbox.size_y = y_max - y_min
        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = class_id
        hypothesis.hypothesis.score = score
        detection.results.append(hypothesis)
        message.detections.append(detection)
    return message


class YoloWorldDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_world_detector')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('detections_topic', '/detections')
        self.declare_parameter(
            'model_path', '/ws/model/yolo-world/yolov8s-worldv2.pt')
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('confidence', 0.05)
        self.declare_parameter('image_size', 640)
        self.declare_parameter('prompts', ['person'])

        model_path = Path(self.get_parameter('model_path').value)
        self.device = self.get_parameter('device').value
        self.confidence = float(self.get_parameter('confidence').value)
        self.image_size = int(self.get_parameter('image_size').value)
        self.prompts = list(self.get_parameter('prompts').value)
        if not model_path.is_file():
            raise FileNotFoundError(f'YOLO-World model not found: {model_path}')
        if not 0.0 <= self.confidence <= 1.0:
            raise ValueError('confidence must be between 0 and 1')
        if self.image_size < 1 or not self.prompts:
            raise ValueError('image_size must be positive and prompts non-empty')

        try:
            from ultralytics import YOLOWorld
        except ImportError as error:
            raise RuntimeError(
                'ultralytics is missing; rebuild the Docker image') from error

        self.bridge = CvBridge()
        self.model = YOLOWorld(str(model_path))
        self.model.set_classes(self.prompts)
        self.publisher = self.create_publisher(
            Detection2DArray,
            self.get_parameter('detections_topic').value,
            qos_profile_sensor_data)
        self.subscription = self.create_subscription(
            Image,
            self.get_parameter('image_topic').value,
            self.on_image,
            qos_profile_sensor_data)
        self.get_logger().info(
            f'YOLO-World ready: model={model_path} device={self.device} '
            f'prompts={self.prompts}')

    def on_image(self, message):
        image = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
        result = self.model.predict(
            image,
            conf=self.confidence,
            imgsz=self.image_size,
            device=self.device,
            verbose=False)[0]
        rows = []
        for box in result.boxes:
            class_index = int(box.cls.item())
            x_min, y_min, x_max, y_max = box.xyxy[0].tolist()
            rows.append((
                result.names[class_index], float(box.conf.item()),
                x_min, y_min, x_max, y_max))
        self.publisher.publish(build_detection_array(message.header, rows))


def main(args=None):
    rclpy.init(args=args)
    node = YoloWorldDetectorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
