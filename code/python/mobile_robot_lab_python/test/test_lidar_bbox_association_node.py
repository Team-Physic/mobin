# Copyright 2026 JungSeong
# Licensed under the Apache License, Version 2.0

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
