# Copyright 2026 JungSeong
# Licensed under the Apache License, Version 2.0

# Frame별 GT ID와 예측 track ID의 2D 위치를 Hungarian assignment로 연결해 MOT 지표를 누적한다.
# 입력 위치는 같은 odom frame의 meter 좌표이며 기본 0.75 m gate 안의 조합만 TP로 인정한다.
# 출력은 TP·FP·FN·IDSW, MOTA·IDF1과 단일 threshold의 DetA·AssA·HOTA 요약이다.
# 이 module은 관측 가능성을 판정하지 않으므로 호출자가 평가할 GT 집합을 먼저 제한해야 한다.

from collections import Counter
from collections import defaultdict
import math

import numpy as np
from scipy.optimize import linear_sum_assignment


class TrackingMetrics:
    """한 거리 threshold에서 중심점 기반 MOT 지표를 누적한다."""

    def __init__(self, distance_gate=0.75):
        if distance_gate <= 0.0:
            raise ValueError('distance_gate must be positive')
        self.distance_gate = distance_gate
        self.frames = 0
        self.gt_total = 0
        self.pred_total = 0
        self.true_positives = 0
        self.false_positives = 0
        self.false_negatives = 0
        self.id_switches = 0
        self.pair_counts = Counter()
        self.gt_counts = Counter()
        self.pred_counts = Counter()
        self.previous_prediction = {}

    def update(self, ground_truth, predictions):
        self.frames += 1
        self.gt_total += len(ground_truth)
        self.pred_total += len(predictions)
        self.gt_counts.update(ground_truth.keys())
        self.pred_counts.update(predictions.keys())
        # Frame별 위치 gate를 통과한 일대일 조합만 TP로 누적한다.
        matches = self._matches(ground_truth, predictions)
        self.true_positives += len(matches)
        self.false_negatives += len(ground_truth) - len(matches)
        self.false_positives += len(predictions) - len(matches)
        for ground_truth_id, prediction_id in matches:
            previous = self.previous_prediction.get(ground_truth_id)
            # ROI 이탈 여부와 무관하게 같은 GT의 이전 predicted ID와 다르면 ID switch다.
            if previous is not None and previous != prediction_id:
                self.id_switches += 1
            self.previous_prediction[ground_truth_id] = prediction_id
            self.pair_counts[(ground_truth_id, prediction_id)] += 1
        return matches

    def _matches(self, ground_truth, predictions):
        if not ground_truth or not predictions:
            return []
        ground_truth_ids = list(ground_truth)
        prediction_ids = list(predictions)
        costs = np.full(
            (len(ground_truth_ids), len(prediction_ids)),
            self.distance_gate * 1000.0,
            dtype=float)
        for row, ground_truth_id in enumerate(ground_truth_ids):
            for column, prediction_id in enumerate(prediction_ids):
                costs[row, column] = math.dist(
                    ground_truth[ground_truth_id], predictions[prediction_id])
        rows, columns = linear_sum_assignment(costs)
        return [
            (ground_truth_ids[row], prediction_ids[column])
            for row, column in zip(rows, columns)
            if costs[row, column] <= self.distance_gate
        ]

    def summary(self):
        mota = 1.0
        if self.gt_total:
            mota = 1.0 - (
                self.false_negatives + self.false_positives + self.id_switches
            ) / self.gt_total
        idtp = self._global_identity_true_positives()
        idfp = self.pred_total - idtp
        idfn = self.gt_total - idtp
        denominator = 2 * idtp + idfp + idfn
        idf1 = 2 * idtp / denominator if denominator else 1.0
        detection_denominator = (
            self.true_positives + self.false_negatives + self.false_positives)
        deta = (
            self.true_positives / detection_denominator
            if detection_denominator else 1.0)
        assa = self._association_accuracy()
        return {
            'frames': self.frames,
            'gt_detections': self.gt_total,
            'predicted_detections': self.pred_total,
            'true_positives': self.true_positives,
            'false_positives': self.false_positives,
            'false_negatives': self.false_negatives,
            'id_switches': self.id_switches,
            'mota': mota,
            'idf1': idf1,
            'deta': deta,
            'assa': assa,
            f'hota_at_{self.distance_gate:.2f}m': math.sqrt(deta * assa),
        }

    def _global_identity_true_positives(self):
        ground_truth_ids = list(self.gt_counts)
        prediction_ids = list(self.pred_counts)
        if not ground_truth_ids or not prediction_ids:
            return 0
        counts = np.zeros((len(ground_truth_ids), len(prediction_ids)))
        for row, ground_truth_id in enumerate(ground_truth_ids):
            for column, prediction_id in enumerate(prediction_ids):
                counts[row, column] = self.pair_counts[
                    (ground_truth_id, prediction_id)]
        # 전체 timeline에서 GT ID와 predicted ID의 일관된 대응 수가 최대인 조합을 고른다.
        rows, columns = linear_sum_assignment(-counts)
        return int(sum(counts[row, column] for row, column in zip(rows, columns)))

    def _association_accuracy(self):
        if not self.true_positives:
            return 1.0 if not self.gt_total and not self.pred_total else 0.0
        weighted_sum = 0.0
        for (ground_truth_id, prediction_id), count in self.pair_counts.items():
            union = (
                self.gt_counts[ground_truth_id]
                + self.pred_counts[prediction_id]
                - count)
            weighted_sum += count * count / union
        return weighted_sum / self.true_positives
