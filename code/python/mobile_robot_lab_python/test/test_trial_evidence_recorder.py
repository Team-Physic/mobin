# Copyright 2026 JungSeong
# Licensed under the Apache License, Version 2.0

# Trial recorder가 danger 전후 시간구간과 Camera bbox 가시성 조건을 선택하는지 검증한다.
# 입력은 second 단위 safety event와 pixel 단위 bbox·image 크기이며 ROS graph를 실행하지 않는다.
# Image 경계에서 잘린 bbox를 제외하고 danger 2초 전부터 safe 2초 후 구간을 반환해야 한다.

import unittest

from mobile_robot_lab_python.trial_evidence_recorder import bbox_is_mostly_visible
from mobile_robot_lab_python.trial_evidence_recorder import CPA_REASONS
from mobile_robot_lab_python.trial_evidence_recorder import has_visible_association
from mobile_robot_lab_python.trial_evidence_recorder import select_event_window
from mobile_robot_lab_python.trial_evidence_recorder import STALE_REASONS


class PortfolioWindowTest(unittest.TestCase):
    def test_selects_two_seconds_around_danger_and_safe(self):
        events = [
            {'time': 4.0, 'reason': 'person_cpa_caution'},
            {'time': 6.0, 'reason': 'person_cpa_danger'},
            {'time': 8.0, 'reason': 'stable_safe_hold'},
            {'time': 9.0, 'reason': 'safe'},
        ]
        self.assertEqual(
            select_event_window(events, 20.0, CPA_REASONS),
            (4.0, 11.0, True))

    def test_reports_missing_target_event(self):
        self.assertEqual(
            select_event_window(
                [{'time': 2.0, 'reason': 'safe'}], 10.0, CPA_REASONS),
            (0.0, 10.0, False))

    def test_selects_stale_independently_from_cpa(self):
        events = [
            {'time': 3.0, 'reason': 'person_cpa_danger'},
            {'time': 8.0, 'reason': 'scan_stale'},
            {'time': 10.0, 'reason': 'safe'},
        ]
        self.assertEqual(
            select_event_window(events, 20.0, STALE_REASONS),
            (6.0, 12.0, True))

    def test_rejects_bbox_cut_by_image_edge(self):
        self.assertTrue(bbox_is_mostly_visible(
            (320.0, 240.0, 100.0, 200.0), 640, 480))
        self.assertFalse(bbox_is_mostly_visible(
            (610.0, 240.0, 60.0, 200.0), 640, 480))
        self.assertFalse(bbox_is_mostly_visible(
            (650.0, 240.0, 100.0, 200.0), 640, 480))

    def test_requires_same_visible_instance_to_succeed_in_fusion(self):
        detections = [
            ('1', (30.0, 240.0, 60.0, 200.0)),
            ('2', (320.0, 240.0, 100.0, 200.0)),
        ]
        self.assertFalse(has_visible_association(
            detections, {1_020_000_000: {'1'}},
            1_000_000_000, 640, 480))
        self.assertTrue(has_visible_association(
            detections, {1_020_000_000: {'2'}},
            1_000_000_000, 640, 480))


if __name__ == '__main__':
    unittest.main()
