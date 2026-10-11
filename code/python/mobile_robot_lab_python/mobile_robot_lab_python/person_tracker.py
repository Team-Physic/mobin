# Copyright 2026 JungSeong
# Licensed under the Apache License, Version 2.0

# Odom frame의 3D 사람 위치를 거리 기반 Hungarian assignment로 기존 ID에 연결하는 Python 기준선이다.
# 입력은 meter 위치 목록과 second timestamp, 출력은 ID·현재위치·최근 trajectory를 가진 track tuple이다.
# 최대거리 밖의 detection은 새 ID가 되고 max_age를 넘긴 track은 제거된다.
# Velocity·covariance·Kalman prediction이 없어 production C++ tracker의 비교 기준으로만 사용한다.

from dataclasses import dataclass, field
import math

import numpy as np
from scipy.optimize import linear_sum_assignment


@dataclass
class PersonTrack:
    track_id: int
    position: tuple
    last_seen: float
    trajectory: list = field(default_factory=list)


class PersonTracker:
    """Odom frame의 사람 위치를 거리 기반 integer ID로 연결한다."""

    def __init__(self, distance_gate=0.75, max_age=1.0, history_size=50):
        if distance_gate <= 0.0 or max_age < 0.0 or history_size < 1:
            raise ValueError('invalid tracker configuration')
        self.distance_gate = distance_gate
        self.max_age = max_age
        self.history_size = history_size
        self.next_id = 1
        self.tracks = []

    def update(self, detections, stamp):
        # Simulation clock이 뒤로 이동하면 이전 timeline의 track을 재사용하지 않는다.
        if self.tracks and stamp < max(track.last_seen for track in self.tracks):
            self.tracks.clear()
        self.tracks = [
            track for track in self.tracks
            if stamp - track.last_seen <= self.max_age
        ]

        # 한 frame 전체 조합을 함께 풀어 greedy 순서에 따른 중복 연결을 막는다.
        matches = self._hungarian_matches(detections)
        matched_detections = set()
        for track_index, detection_index in matches:
            track = self.tracks[track_index]
            track.position = detections[detection_index]
            track.last_seen = stamp
            track.trajectory.append(track.position)
            track.trajectory[:] = track.trajectory[-self.history_size:]
            matched_detections.add(detection_index)

        for index, position in enumerate(detections):
            if index in matched_detections:
                continue
            self.tracks.append(PersonTrack(
                self.next_id, position, stamp, [position]))
            self.next_id += 1
        return tuple(self.tracks)

    def _hungarian_matches(self, detections):
        track_count = len(self.tracks)
        detection_count = len(detections)
        if track_count == 0 or detection_count == 0:
            return []

        size = track_count + detection_count
        unmatched_cost = self.distance_gate
        invalid_cost = unmatched_cost * 1000.0
        costs = np.zeros((size, size), dtype=float)
        costs[:track_count, :detection_count] = invalid_cost
        costs[:track_count, detection_count:] = unmatched_cost
        costs[track_count:, :detection_count] = unmatched_cost

        for track_index, track in enumerate(self.tracks):
            for detection_index, detection in enumerate(detections):
                distance = math.dist(track.position, detection)
                if distance <= self.distance_gate:
                    costs[track_index, detection_index] = distance

        rows, columns = linear_sum_assignment(costs)
        return [
            (row, column)
            for row, column in zip(rows, columns)
            if row < track_count and column < detection_count and
            costs[row, column] <= self.distance_gate
        ]
