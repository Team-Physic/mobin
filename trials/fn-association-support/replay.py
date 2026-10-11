#!/usr/bin/env python3
# 동일한 기록의 instance point를 실제 C++ association·tracker에 넣어 MOT 지표를 비교한다.
# GT는 공통 Camera–LiDAR FoV의 평가에만 사용하며 후보 선택에 전달하지 않는다.
# 압축 JSON 입력은 이 파일 옆에서 읽고 결과 JSON은 stdout으로 출력한다.
# 위치 인자 library: 필수, replay_bridge.cpp로 빌드한 신뢰할 수 있는 shared library의 경로.
import ctypes as C
import gzip
import json
from pathlib import Path
import sys
from collections import Counter

import numpy as np
from mobile_robot_lab_python.tracking_evaluation_roi import camera_visibility, lidar_visibility
from mobile_robot_lab_python.tracking_metrics import TrackingMetrics


def evaluate(library):
    """고정된 frame·GT·0.75 m gate에서 선택한 C++ 구현의 지표를 반환한다."""
    with gzip.open(Path(__file__).with_name('frames.json.gz'), 'rt') as stream:
        data = json.load(stream)
    lib = C.CDLL(str(Path(library).resolve()))
    array = np.ctypeslib.ndpointer(dtype=np.float64, flags='C_CONTIGUOUS')
    lib.assoc.argtypes = [array, C.c_int, array]
    lib.tracker_create.restype = C.c_void_p
    lib.tracker_free.argtypes = [C.c_void_p]
    lib.tracker_update.argtypes = [C.c_void_p, array, C.c_int, C.c_double, array]
    tracker = lib.tracker_create()
    metrics = TrackingMetrics()
    counts = Counter()

    def transform(point, rigid):
        return np.asarray(point) @ np.asarray(rigid[0]).T + rigid[1]

    def visible(point, frame):
        return camera_visibility(
            transform(point, frame['camod']), data['intrinsics'],
            data['width'], data['height']) and lidar_visibility(
                transform(point, frame['lidarod']), .1, 20,
                -3.141593, 3.141593, -.261799, .261799)

    try:
        for frame in data['frames']:
            observations = []
            for instance in frame['instances']:
                points = np.ascontiguousarray(instance['points'], dtype=np.float64).reshape(-1, 3)
                selected = np.zeros(4)
                status = lib.assoc(points, len(points), selected)
                counts[str(status)] += 1
                if status == 0:
                    observations.append(transform(selected[:3], frame['odcam'])[:2])
            inputs = np.ascontiguousarray(observations, dtype=np.float64).reshape(-1, 2)
            # 이 고정 fixture는 한 frame 최대 3명이다. 1000행은 C++ 결과용 충분한 공간이다.
            output = np.zeros((1000, 3))
            count = lib.tracker_update(tracker, inputs, len(inputs), frame['t'], output)
            if count < 0:
                raise RuntimeError('replay tracker output exceeds 1000 rows')
            ground_truth = {
                key: value[:2] for key, value in frame['gt'].items()
                if visible(value, frame)}
            predictions = {
                int(row[0]): row[1:] for row in output[:count]
                if visible(np.r_[row[1:], .9], frame)}
            if ground_truth or predictions:
                metrics.update(ground_truth, predictions)
    finally:
        lib.tracker_free(tracker)
    return {**metrics.summary(), 'association_status': dict(counts)}


if __name__ == '__main__':
    if len(sys.argv) != 2:
        raise SystemExit('usage: replay.py LIBRARY.so')
    print(json.dumps(evaluate(sys.argv[1]), indent=2))
