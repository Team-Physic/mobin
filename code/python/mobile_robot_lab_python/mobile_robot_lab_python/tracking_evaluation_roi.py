# Copyright 2026 JungSeong
# Licensed under the Apache License, Version 2.0

# Tracking 평가 시 Camera와 3D LiDAR가 함께 볼 수 있는 사람 기준점만 선택한다.
# 입력은 sensor frame의 meter 단위 3D point, Camera 내부 파라미터와 LiDAR FoV다.
# 최근접 timestamp sample 선택과 sensor별 가시성 판정을 ROS 없이 시험할 수 있게 제공한다.
# ROI 밖 sample은 검출 실패가 아니므로 호출자가 TP·FP·FN 계산 전에 제외한다.

import math


def nearest_sample(samples, target_timestamp, maximum_difference):
    """target과 가장 가까우며 허용 차이 이내인 `(timestamp, value)`를 반환한다."""
    if not samples:
        return None
    timestamp, value = min(
        samples, key=lambda sample: abs(sample[0] - target_timestamp))
    if abs(timestamp - target_timestamp) > maximum_difference:
        return None
    return timestamp, value


def camera_visibility(point_camera, intrinsics, width, height):
    """Camera optical frame point를 pixel로 투영해 image 내부 여부를 반환한다."""
    x, y, z = point_camera
    if z <= 0.0:
        return False
    fx, fy, cx, cy = intrinsics
    u = fx * x / z + cx
    v = fy * y / z + cy
    return 0.0 <= u < width and 0.0 <= v < height


def lidar_visibility(
        point_lidar, minimum_range, maximum_range,
        horizontal_minimum, horizontal_maximum,
        vertical_minimum, vertical_maximum):
    """LiDAR frame point가 거리·수평각·수직각 측정 범위 안인지 반환한다."""
    x, y, z = point_lidar
    distance = math.sqrt(x * x + y * y + z * z)
    if not minimum_range <= distance <= maximum_range:
        return False
    horizontal = math.atan2(y, x)
    horizontal_span = horizontal_maximum - horizontal_minimum
    horizontal_visible = (
        horizontal_span >= 2.0 * math.pi - 1e-6
        or (horizontal - horizontal_minimum) % (2.0 * math.pi)
        <= horizontal_span)
    vertical = math.atan2(z, math.hypot(x, y))
    return (
        horizontal_visible
        and vertical_minimum <= vertical <= vertical_maximum)


def transform_point(point, transform):
    """TransformStamped의 target←source rigid transform을 3D point에 적용한다."""
    q = transform.transform.rotation
    tx = transform.transform.translation.x
    ty = transform.transform.translation.y
    tz = transform.transform.translation.z
    x, y, z = point
    # Quaternion으로 만든 회전행렬을 source point에 곱한 뒤 target-frame translation을 더한다.
    return (
        (1.0 - 2.0 * (q.y * q.y + q.z * q.z)) * x
        + 2.0 * (q.x * q.y - q.z * q.w) * y
        + 2.0 * (q.x * q.z + q.y * q.w) * z + tx,
        2.0 * (q.x * q.y + q.z * q.w) * x
        + (1.0 - 2.0 * (q.x * q.x + q.z * q.z)) * y
        + 2.0 * (q.y * q.z - q.x * q.w) * z + ty,
        2.0 * (q.x * q.z - q.y * q.w) * x
        + 2.0 * (q.y * q.z + q.x * q.w) * y
        + (1.0 - 2.0 * (q.x * q.x + q.y * q.y)) * z + tz,
    )
