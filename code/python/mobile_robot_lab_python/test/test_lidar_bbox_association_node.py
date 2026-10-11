# Copyright 2026 JungSeong
# Licensed under the Apache License, Version 2.0

# Python association node가 TF quaternion을 회전행렬로 바꾸고 ground 높이를 판정하는 계산을 검증한다.
# 입력은 합성 quaternion과 meter point이며 ROS graph나 실제 sensor message 동기화를 실행하지 않는다.
# 회전 방향과 Ground Reference Frame의 z threshold 경계가 예상값과 일치해야 한다.

import math
import unittest

from geometry_msgs.msg import Quaternion

from mobile_robot_lab_python.lidar_bbox_association_node import (
    quaternion_rotation,
    remove_ground,
)
from mobile_robot_lab_python.camera_lidar_fusion import Point3


class LidarBboxAssociationNodeTest(unittest.TestCase):
    def test_quaternion_rotation_matches_quarter_turn(self):
        rotation = quaternion_rotation(Quaternion(
            z=math.sin(math.pi / 4), w=math.cos(math.pi / 4)))
        self.assertAlmostEqual(rotation[0][0], 0.0, places=12)
        self.assertAlmostEqual(rotation[0][1], -1.0, places=12)
        self.assertAlmostEqual(rotation[1][0], 1.0, places=12)

    def test_remove_ground_uses_transformed_base_footprint_height(self):
        identity = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
        points = [Point3(1.0, 0.0, -0.30), Point3(1.0, 0.0, 0.20)]
        kept = remove_ground(points, identity, Point3(0.0, 0.0, 0.30), 0.05)
        self.assertEqual(kept, [points[1]])


if __name__ == '__main__':
    unittest.main()
