# Copyright 2026 JungSeong
# Licensed under the Apache License, Version 2.0

# Tracking evaluator의 unnamed Gazebo pose 선택과 TP·FP·FN·IDSW 지표 계산을 검증한다.
# GT와 예측은 같은 odom 평면의 meter 좌표이며 distance gate 밖의 조합은 match가 아니다.
# 완전 추적, 누락, 오검출과 ID 변경에서 MOTA·IDF1·HOTA 요약값을 확인한다.

import unittest
import csv
import json
import os
import tempfile
from types import SimpleNamespace

from mobile_robot_lab_python.tracking_metrics import TrackingMetrics
from mobile_robot_lab_python.tracking_evaluation_roi import camera_visibility
from mobile_robot_lab_python.tracking_evaluation_roi import lidar_visibility
from mobile_robot_lab_python.tracking_evaluation_roi import nearest_sample
from mobile_robot_lab_python.tracking_evaluator_node import TrackingEvaluatorNode
from mobile_robot_lab_python.tracking_evaluator_node import select_unnamed_models


class TrackingMetricsTest(unittest.TestCase):
    def test_selects_robot_and_following_actors_from_unnamed_poses(self):
        poses = [(4.0, 5.0, 0.0), (-2.0, -0.5, 0.0),
                 (-2.5, -2.0, 0.0), (0.0, -3.0, 0.0), (0.0, 0.0, 0.0)]
        self.assertEqual(
            select_unnamed_models(poses, (-2.01, -0.49), 2), poses[1:4])

    def test_perfect_tracks_score_one(self):
        metrics = TrackingMetrics(0.75)
        metrics.update({'a': (0.0, 0.0)}, {1: (0.0, 0.0)})
        metrics.update({'a': (0.1, 0.0)}, {1: (0.1, 0.0)})
        summary = metrics.summary()
        self.assertEqual(summary['mota'], 1.0)
        self.assertEqual(summary['idf1'], 1.0)
        self.assertEqual(summary['hota_at_0.75m'], 1.0)

    def test_id_switch_is_counted(self):
        metrics = TrackingMetrics(0.75)
        metrics.update({'a': (0.0, 0.0)}, {1: (0.0, 0.0)})
        metrics.update({'a': (0.1, 0.0)}, {2: (0.1, 0.0)})
        self.assertEqual(metrics.summary()['id_switches'], 1)

    def test_outside_gate_is_false_positive_and_negative(self):
        metrics = TrackingMetrics(0.75)
        metrics.update({'a': (0.0, 0.0)}, {1: (1.0, 0.0)})
        summary = metrics.summary()
        self.assertEqual(summary['true_positives'], 0)
        self.assertEqual(summary['false_positives'], 1)
        self.assertEqual(summary['false_negatives'], 1)

    def test_common_sensor_roi_and_timestamp_limit(self):
        intrinsics = (500.0, 500.0, 320.0, 240.0)
        self.assertTrue(camera_visibility(
            (0.0, 0.0, 2.0), intrinsics, 640, 480))
        self.assertFalse(camera_visibility(
            (2.0, 0.0, 2.0), intrinsics, 640, 480))
        self.assertTrue(lidar_visibility(
            (2.0, 0.0, 0.2), 0.1, 20.0,
            -3.141593, 3.141593, -0.261799, 0.261799))
        self.assertFalse(lidar_visibility(
            (2.0, 0.0, 1.0), 0.1, 20.0,
            -3.141593, 3.141593, -0.261799, 0.261799))
        samples = [(1_000_000_000, 'old'), (1_040_000_000, 'nearest')]
        self.assertEqual(nearest_sample(
            samples, 1_090_000_000, 50_000_000)[1], 'nearest')
        self.assertIsNone(nearest_sample(
            samples, 1_090_000_001, 50_000_000))

    def test_roi_gap_preserves_identity_for_id_switch(self):
        metrics = TrackingMetrics(0.75)
        metrics.update({'person_a': (0.0, 0.0)}, {7: (0.0, 0.0)})
        # ROI 밖 frame은 update하지 않아도 GT identity 이력은 유지된다.
        metrics.update({'person_a': (0.1, 0.0)}, {12: (0.1, 0.0)})
        self.assertEqual(metrics.summary()['id_switches'], 1)

    def test_writes_association_and_id_switch_artifacts(self):
        evaluator = SimpleNamespace(
            association_rows=[{
                'timestamp': 1.0, 'outcome': 'TP',
                'ground_truth_id': 'person_a', 'track_id': 7,
                'distance_m': 0.1,
            }],
            id_switch_events=[{
                'timestamp': 2.0, 'ground_truth_id': 'person_a',
                'previous_track_id': 7, 'new_track_id': 12,
            }])
        with tempfile.TemporaryDirectory() as directory:
            TrackingEvaluatorNode.save_association_rows(evaluator, directory)
            TrackingEvaluatorNode.save_id_switch_events(evaluator, directory)
            with open(os.path.join(
                    directory, 'roi-associations.csv'), newline='') as stream:
                rows = list(csv.DictReader(stream))
            with open(os.path.join(
                    directory, 'id-switch-events.json')) as stream:
                events = json.load(stream)
        self.assertEqual(rows[0]['outcome'], 'TP')
        self.assertEqual(events[0]['new_track_id'], 12)


if __name__ == '__main__':
    unittest.main()
