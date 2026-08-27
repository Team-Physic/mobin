# Copyright 2026 JungSeong
# Licensed under the Apache License, Version 2.0

import unittest

from std_msgs.msg import Header

from mobile_robot_lab_python.yolo_world_detector_node import (
    build_detection_array)


class YoloWorldDetectorNodeTest(unittest.TestCase):
    def test_build_detection_array_preserves_header_class_score_and_bbox(self):
        header = Header(frame_id='camera_rgb_optical_frame')
        message = build_detection_array(
            header, [('person', 0.75, 10.0, 20.0, 50.0, 80.0)])

        self.assertEqual(message.header.frame_id, header.frame_id)
        self.assertEqual(len(message.detections), 1)
        detection = message.detections[0]
        self.assertEqual(detection.results[0].hypothesis.class_id, 'person')
        self.assertAlmostEqual(detection.results[0].hypothesis.score, 0.75)
        self.assertAlmostEqual(detection.bbox.center.position.x, 30.0)
        self.assertAlmostEqual(detection.bbox.center.position.y, 50.0)
        self.assertAlmostEqual(detection.bbox.size_x, 40.0)
        self.assertAlmostEqual(detection.bbox.size_y, 60.0)


if __name__ == '__main__':
    unittest.main()
