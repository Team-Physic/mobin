#!/usr/bin/env python3
# 실제 YOLO11-seg 추론을 detector callback에 연결해 원본 mask 전달을 검증한다.
# 입력은 설치된 Ultralytics 예제 이미지의 홀수 크기 crop과 검출 없는 빈 영상이다.
# 결과 label·header를 모델의 binary mask와 비교하고 요약 JSON만 stdout에 쓴다.
# 외부 다운로드·ROS 통신·파일 생성은 없으며 Camera 좌표는 원본 pixel 기준이다.
# 인자 없음. /ws/model/yolo-seg/yolo11s-seg.pt 모델을 CPU에서 사용한다.

import json
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import cv2
from cv_bridge import CvBridge
import numpy as np
from std_msgs.msg import Header
import ultralytics
from ultralytics import YOLO

from mobile_robot_lab_python.yolo_segmentation_detector_node import YoloSegmentationDetectorNode


def main():
    """실제 추론 결과의 mask·label 대응과 원본 해상도·timestamp 보존을 확인한다."""
    model = YOLO('/ws/model/yolo-seg/yolo11s-seg.pt')
    bridge = CvBridge()
    observed = []
    def predict(image, **kwargs):
        result = model.predict(image, **kwargs)
        observed.append(result[0])
        return result
    node = SimpleNamespace(
        bridge=bridge, model=SimpleNamespace(predict=predict), confidence=.25,
        image_size=640, class_ids=[0], device='cpu',
        detections_publisher=Mock(), labels_publisher=Mock())
    bus = cv2.imread(str(Path(ultralytics.__file__).parent / 'assets/bus.jpg'))
    assert bus is not None
    rows = []
    for name, image in [('people_odd_size', bus[:1001, :797]),
                        ('empty', np.zeros((479, 631, 3), np.uint8))]:
        message = bridge.cv2_to_imgmsg(image, encoding='bgr8')
        message.header = Header(frame_id='camera_rgb_optical_frame')
        message.header.stamp.sec = 19
        YoloSegmentationDetectorNode.on_image(node, message)
        result = observed[-1]
        expected = np.zeros(image.shape[:2], np.uint8)
        masks = result.masks.data.cpu().numpy() if result.masks is not None else []
        for index, mask in enumerate(masks[:255], start=1):
            assert mask.shape == image.shape[:2]
            expected[mask.astype(bool)] = index
        output = node.labels_publisher.publish.call_args.args[0]
        labels = bridge.imgmsg_to_cv2(output)
        np.testing.assert_array_equal(labels, expected)
        assert output.header == message.header
        detections = node.detections_publisher.publish.call_args.args[0]
        assert detections.header == message.header
        assert len(detections.detections) == len(masks)
        if name == 'people_odd_size':
            assert len(masks) > 0
        else:
            assert len(masks) == 0
        rows.append({'case': name, 'shape_hw': list(labels.shape),
                     'instances': len(masks), 'pixel_differences': int((labels != expected).sum())})
    print(json.dumps({'inference': rows, 'ultralytics': ultralytics.__version__}, indent=2))


if __name__ == '__main__':
    main()
