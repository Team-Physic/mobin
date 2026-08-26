# Copyright 2026 JungSeong
# Licensed under the Apache License, Version 2.0

import unittest

from mobile_robot_lab_python.camera_lidar_fusion import associate_bbox
from mobile_robot_lab_python.camera_lidar_fusion import BoundingBox
from mobile_robot_lab_python.camera_lidar_fusion import Intrinsics
from mobile_robot_lab_python.camera_lidar_fusion import Point3


class CameraLidarFusionTest(unittest.TestCase):
    def test_selects_only_points_projected_inside_yolo_bbox(self):
        points = [
            Point3(-0.1, 0.0, 2.0),
            Point3(0.0, 0.0, 2.0),
            Point3(0.1, 0.0, 2.0),
            Point3(2.0, 0.0, 2.0),
            Point3(0.0, 0.0, -1.0),
        ]
        result = associate_bbox(
            points,
            ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
            Point3(0.0, 0.0, 0.0),
            Intrinsics(100.0, 100.0, 320.0, 240.0),
            BoundingBox(310.0, 230.0, 330.0, 250.0),
        )
        self.assertIsNotNone(result)
        self.assertEqual(result.point_count, 3)
        self.assertEqual(len(result.matched_points_lidar), 3)
        self.assertAlmostEqual(result.position_lidar.z, 2.0)
        self.assertAlmostEqual(result.lidar_range, 2.0)

    def test_rejects_bbox_with_too_few_lidar_points(self):
        result = associate_bbox(
            [Point3(0.0, 0.0, 2.0)],
            ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
            Point3(0.0, 0.0, 0.0),
            Intrinsics(100.0, 100.0, 320.0, 240.0),
            BoundingBox(310.0, 230.0, 330.0, 250.0),
        )
        self.assertIsNone(result)


if __name__ == '__main__':
    unittest.main()
