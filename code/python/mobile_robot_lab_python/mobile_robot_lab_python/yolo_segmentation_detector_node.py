#!/usr/bin/env python3

# Copyright 2026 JungSeong
# Licensed under the Apache License, Version 2.0

# YOLO segmentation으로 Camera image의 person instance를 검출한다.
# 입력은 원본 Camera Image이며 Detection2DArray와 mono8 Instance Label Map을 같은 timestamp로 발행한다.
# 원본 해상도의 binary mask를 직접 사용하여 내부 구멍과 분리된 사람 영역을 보존한다.
# label 0은 배경, 1..N은 현재 frame의 detection 순서이며 지속적인 tracking ID가 아니다.
# inference device와 confidence는 ROS parameter로 받아 CPU와 CUDA 실행을 모두 지원한다.

from pathlib import Path

from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D
from vision_msgs.msg import Detection2DArray
from vision_msgs.msg import ObjectHypothesisWithPose


def build_detection_array(header, detections):
    """Frame-local label과 bbox를 Detection2DArray로 변환한다."""
    message = Detection2DArray(header=header)
    for label, class_id, score, x_min, y_min, x_max, y_max in detections:
        detection = Detection2D(header=header)
        detection.id = str(label)
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


def build_instance_label_map(height, width, masks):
    """원본 영상 크기의 binary mask를 frame-local mono8 label로 변환한다."""
    if height < 1 or width < 1:
        raise ValueError('image dimensions must be positive')
    if len(masks) > 255:
        raise ValueError('mono8 Instance Label Map supports at most 255 instances')
    labels = np.zeros((height, width), dtype=np.uint8)
    for index, mask in enumerate(masks, start=1):
        binary = np.asarray(mask, dtype=bool)
        if binary.shape != (height, width):
            raise ValueError('instance mask must match original image dimensions')
        # 윤곽선을 재구성하지 않아 구멍·분리 영역을 유지한다. 겹침은 기존처럼 뒤 label이 우선한다.
        labels[binary] = index
    return labels


class YoloSegmentationDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_segmentation_detector')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('detections_topic', '/detections')
        self.declare_parameter(
            'instance_labels_topic', '/segmentation/instance_labels')
        self.declare_parameter(
            'model_path', '/ws/model/yolo-seg/yolo11s-seg.pt')
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('confidence', 0.25)
        self.declare_parameter('image_size', 640)
        self.declare_parameter('target_classes', ['person'])

        model_path = Path(self.get_parameter('model_path').value)
        self.device = self.get_parameter('device').value
        self.confidence = float(self.get_parameter('confidence').value)
        self.image_size = int(self.get_parameter('image_size').value)
        self.target_classes = list(
            self.get_parameter('target_classes').value)
        if not model_path.is_file():
            raise FileNotFoundError(
                f'YOLO segmentation model not found: {model_path}')
        if not 0.0 <= self.confidence <= 1.0:
            raise ValueError('confidence must be between 0 and 1')
        if self.image_size < 1 or not self.target_classes:
            raise ValueError(
                'image_size must be positive and target_classes non-empty')

        try:
            from ultralytics import YOLO
        except ImportError as error:
            raise RuntimeError(
                'ultralytics is missing; rebuild the Docker image') from error

        self.bridge = CvBridge()
        self.model = YOLO(str(model_path))
        names = self.model.names
        name_items = names.items() if isinstance(names, dict) else enumerate(names)
        class_by_name = {name: int(index) for index, name in name_items}
        missing = [name for name in self.target_classes if name not in class_by_name]
        if missing:
            raise ValueError(f'target classes absent from model: {missing}')
        self.class_ids = [class_by_name[name] for name in self.target_classes]
        self.detections_publisher = self.create_publisher(
            Detection2DArray,
            self.get_parameter('detections_topic').value,
            qos_profile_sensor_data)
        self.labels_publisher = self.create_publisher(
            Image,
            self.get_parameter('instance_labels_topic').value,
            qos_profile_sensor_data)
        self.subscription = self.create_subscription(
            Image,
            self.get_parameter('image_topic').value,
            self.on_image,
            qos_profile_sensor_data)
        self.get_logger().info(
            f'YOLO segmentation ready: model={model_path} '
            f'device={self.device} classes={self.target_classes}')

    def on_image(self, message):
        image = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
        result = self.model.predict(
            image,
            conf=self.confidence,
            imgsz=self.image_size,
            classes=self.class_ids,
            device=self.device,
            retina_masks=True,
            verbose=False)[0]

        boxes = list(result.boxes) if result.boxes is not None else []
        # retina_masks가 letterbox를 제거하고 원본 영상 크기로 복원한 binary mask를 사용한다.
        masks = result.masks.data[:255].cpu().numpy() if result.masks is not None else []
        count = min(len(boxes), len(masks), 255)
        rows = []
        for index in range(count):
            box = boxes[index]
            class_index = int(box.cls.item())
            x_min, y_min, x_max, y_max = box.xyxy[0].tolist()
            label = index + 1
            rows.append((
                label, result.names[class_index], float(box.conf.item()),
                x_min, y_min, x_max, y_max))

        labels = build_instance_label_map(
            image.shape[0], image.shape[1], masks[:count])
        labels_message = self.bridge.cv2_to_imgmsg(labels, encoding='mono8')
        labels_message.header = message.header
        # 두 message는 같은 inference 결과이며 label i는 detections[i-1]에 대응한다.
        self.detections_publisher.publish(
            build_detection_array(message.header, rows))
        self.labels_publisher.publish(labels_message)


def main(args=None):
    rclpy.init(args=args)
    node = YoloSegmentationDetectorNode()
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
