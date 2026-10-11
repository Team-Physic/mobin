# Copyright 2026 JungSeong
# Licensed under the Apache License, Version 2.0

# YOLO segmentation ROS message 변환을 model inference 없이 검증한다.
# Detection ID와 mono8 label 값의 대응, bbox 보존, 빈 배경 처리가 핵심 검증 대상이다.
# 원본 binary mask의 구멍·분리 영역과 해상도 계약을 검사하여 polygon 재채움 회귀를 막는다.
# 모든 좌표는 원본 Camera image pixel 기준이며 외부 file을 생성하지 않는다.

import unittest
from types import SimpleNamespace
from unittest.mock import MagicMock, Mock

from cv_bridge import CvBridge
import numpy as np
from std_msgs.msg import Header

from mobile_robot_lab_python.yolo_segmentation_detector_node import (
    build_detection_array,
    build_instance_label_map,
    YoloSegmentationDetectorNode,
)


class YoloSegmentationDetectorNodeTest(unittest.TestCase):
    def test_detection_id_matches_instance_label(self):
        header = Header(frame_id='camera_rgb_optical_frame')
        message = build_detection_array(
            header, [(1, 'person', 0.75, 10.0, 20.0, 50.0, 80.0)])

        self.assertEqual(message.header.frame_id, header.frame_id)
        self.assertEqual(len(message.detections), 1)
        detection = message.detections[0]
        self.assertEqual(detection.id, '1')
        self.assertEqual(detection.results[0].hypothesis.class_id, 'person')
        self.assertAlmostEqual(detection.results[0].hypothesis.score, 0.75)
        self.assertAlmostEqual(detection.bbox.center.position.x, 30.0)
        self.assertAlmostEqual(detection.bbox.center.position.y, 50.0)
        self.assertAlmostEqual(detection.bbox.size_x, 40.0)
        self.assertAlmostEqual(detection.bbox.size_y, 60.0)

    def test_instance_label_map_uses_zero_background_and_one_based_labels(self):
        masks = np.zeros((2, 8, 10), dtype=np.uint8)
        masks[0, 1:5, 1:5] = 1
        masks[1, 2:7, 6:9] = 1
        labels = build_instance_label_map(8, 10, masks)

        self.assertEqual(labels.dtype, np.uint8)
        self.assertEqual(labels[0, 0], 0)
        self.assertEqual(labels[2, 2], 1)
        self.assertEqual(labels[3, 7], 2)

    def test_binary_mask_preserves_holes_and_disconnected_regions(self):
        mask = np.zeros((12, 16), dtype=np.uint8)
        mask[1:9, 1:9] = 1
        mask[3:6, 3:6] = 0
        mask[9:11, 12:15] = 1
        original = mask.copy()
        labels = build_instance_label_map(12, 16, [mask])
        np.testing.assert_array_equal(labels, original)
        np.testing.assert_array_equal(mask, original)

    def test_mask_size_must_match_original_image(self):
        with self.assertRaisesRegex(ValueError, 'original image dimensions'):
            build_instance_label_map(8, 10, [np.ones((4, 5))])

    def test_empty_and_overlapping_masks_keep_label_contract(self):
        np.testing.assert_array_equal(build_instance_label_map(2, 3, []), np.zeros((2, 3)))
        labels = build_instance_label_map(2, 3, [np.ones((2, 3)), np.eye(2, 3)])
        self.assertEqual(labels[0, 0], 2)
        self.assertEqual(labels[0, 1], 1)

    def test_on_image_requests_native_masks_and_preserves_header(self):
        bridge = CvBridge()
        message = bridge.cv2_to_imgmsg(np.zeros((12, 16, 3), np.uint8), encoding='bgr8')
        message.header = Header(frame_id='camera_rgb_optical_frame')
        message.header.stamp.sec = 7
        mask = np.ones((1, 12, 16), np.uint8)
        mask[0, 4:8, 6:10] = 0
        tensor = MagicMock()
        tensor.__getitem__.return_value.cpu.return_value.numpy.return_value = mask
        box = SimpleNamespace(cls=np.array(0), conf=np.array(.9),
                              xyxy=np.array([[0, 0, 16, 12]]))
        result = SimpleNamespace(boxes=[box], masks=SimpleNamespace(data=tensor), names={0: 'person'})
        node = SimpleNamespace(bridge=bridge, model=Mock(), confidence=.25, image_size=640,
                               class_ids=[0], device='cpu', detections_publisher=Mock(),
                               labels_publisher=Mock())
        node.model.predict.return_value = [result]
        YoloSegmentationDetectorNode.on_image(node, message)
        self.assertTrue(node.model.predict.call_args.kwargs['retina_masks'])
        output = node.labels_publisher.publish.call_args.args[0]
        np.testing.assert_array_equal(bridge.imgmsg_to_cv2(output), mask[0])
        self.assertEqual(output.header, message.header)
        detections = node.detections_publisher.publish.call_args.args[0]
        self.assertEqual(detections.header, message.header)
        self.assertEqual(detections.detections[0].id, '1')


if __name__ == '__main__':
    unittest.main()
