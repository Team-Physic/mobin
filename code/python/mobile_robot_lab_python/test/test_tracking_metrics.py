# Copyright 2026 JungSeong
# Licensed under the Apache License, Version 2.0

import unittest

from mobile_robot_lab_python.tracking_metrics import TrackingMetrics


class TrackingMetricsTest(unittest.TestCase):
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


if __name__ == '__main__':
    unittest.main()
