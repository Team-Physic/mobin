# Python 장애물 회피 기준선의 LaserScan sector와 forward·turn·blocked 상태전이를 검증한다.
# 합성 입력은 radian 각도와 meter 거리이며 ROS node나 Gazebo를 실행하지 않는다.
# NaN·infinity 처리, stop/clear hysteresis와 지정 회전방향의 차단 조건을 확인한다.

import math
import unittest

from mobile_robot_lab_python.obstacle_avoidance import AvoidancePolicy
from mobile_robot_lab_python.obstacle_avoidance import SectorDistances
from mobile_robot_lab_python.obstacle_avoidance import sector_distances


class ObstacleAvoidanceTest(unittest.TestCase):
    def test_sector_mapping_and_invalid_values(self):
        ranges = [math.inf] * 360
        ranges[0] = 0.4
        ranges[30] = 0.8
        ranges[330] = 0.6
        ranges[5] = math.nan
        result = sector_distances(
            ranges, 0.0, 2.0 * math.pi / 360.0, 0.1, 3.5,
            math.radians(15.0), math.radians(60.0))
        self.assertAlmostEqual(result.front, 0.4)
        self.assertAlmostEqual(result.left, 0.8)
        self.assertAlmostEqual(result.right, 0.6)

    def test_hysteresis_and_turn_direction(self):
        policy = AvoidancePolicy(0.45, 0.55, 0.40, 0.15, 0.8)
        self.assertEqual(
            policy.command(SectorDistances(1.0, 1.0, 1.0)),
            (0.15, 0.0, 'forward'))
        self.assertEqual(
            policy.command(SectorDistances(0.4, 0.3, 0.8)),
            (0.0, -0.8, 'turn_right'))
        self.assertEqual(
            policy.command(SectorDistances(0.5, 1.0, 1.0)),
            (0.0, -0.8, 'turn_right'))
        self.assertEqual(
            policy.command(SectorDistances(0.6, 1.0, 1.0)),
            (0.15, 0.0, 'forward'))

    def test_preferred_right_turn(self):
        policy = AvoidancePolicy(
            0.45, 0.55, 0.40, 0.15, 0.8, preferred_turn_direction='right')
        self.assertEqual(
            policy.command(SectorDistances(1.0, 1.0, 0.3)),
            (0.0, 0.0, 'blocked'))
        self.assertEqual(
            policy.command(SectorDistances(1.0, 1.0, 0.5)),
            (0.0, -0.8, 'turn_right'))
        self.assertEqual(
            policy.command(SectorDistances(1.0, 1.0, 0.6)),
            (0.15, 0.0, 'forward'))


if __name__ == '__main__':
    unittest.main()
