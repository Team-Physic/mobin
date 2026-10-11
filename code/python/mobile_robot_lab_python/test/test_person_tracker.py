# Copyright 2026 JungSeong
# Licensed under the Apache License, Version 2.0

# Python 거리 기반 tracker의 Hungarian assignment, ID 생성과 timeout 제거를 검증한다.
# 입력은 odom의 meter 위치와 second timestamp이며 ROS message를 사용하지 않는다.
# 두 사람이 교차하거나 detection 순서가 바뀌어도 최소 전체거리 조합을 선택해야 한다.

import unittest

from mobile_robot_lab_python.person_tracker import PersonTracker


class PersonTrackerTest(unittest.TestCase):
    def test_hungarian_assignment_preserves_two_crossing_ids(self):
        tracker = PersonTracker(distance_gate=0.75)
        first = tracker.update([(0.0, 0.0, 0.0), (0.3, 0.0, 0.0)], 0.0)
        self.assertEqual([track.track_id for track in first], [1, 2])

        second = tracker.update([(0.1, 0.0, 0.0), (-0.4, 0.0, 0.0)], 0.1)
        positions = {track.track_id: track.position for track in second}
        self.assertEqual(positions[1], (-0.4, 0.0, 0.0))
        self.assertEqual(positions[2], (0.1, 0.0, 0.0))

    def test_detection_outside_gate_starts_new_track(self):
        tracker = PersonTracker(distance_gate=0.75)
        tracker.update([(0.0, 0.0, 0.0)], 0.0)
        tracks = tracker.update([(1.0, 0.0, 0.0)], 0.1)
        self.assertEqual([track.track_id for track in tracks], [1, 2])


if __name__ == '__main__':
    unittest.main()
