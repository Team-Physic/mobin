#!/usr/bin/env python3

# Copyright 2026 JungSeong
# Licensed under the Apache License, Version 2.0

# LiDAR point의 rigid transform, pinhole Camera 투영과 bbox 내부 point 선택을 제공하는 순수 계산 module이다.
# 입력은 LiDAR frame의 meter 좌표, T_camera_lidar, Camera 내부 파라미터와 pixel bbox다.
# 출력은 매칭 point의 median 3D 위치와 LiDAR 원점 거리이며 ROS message나 전역 상태를 사용하지 않는다.
# Camera optical frame은 x=right, y=down, z=forward이고 distortion 보정된 image를 가정한다.

from dataclasses import dataclass
import math
import statistics


@dataclass(frozen=True)
class Point3:
    x: float
    y: float
    z: float


@dataclass(frozen=True)
class Intrinsics:
    fx: float
    fy: float
    cx: float
    cy: float


@dataclass(frozen=True)
class BoundingBox:
    u_min: float
    v_min: float
    u_max: float
    v_max: float


@dataclass(frozen=True)
class Association:
    position_lidar: Point3
    position_camera: Point3
    lidar_range: float
    point_count: int
    matched_points_lidar: tuple


def transform_point(point, rotation, translation):
    """T_camera_lidar를 LiDAR point에 적용해 Camera frame 좌표를 반환한다."""
    return Point3(
        rotation[0][0] * point.x + rotation[0][1] * point.y +
        rotation[0][2] * point.z + translation.x,
        rotation[1][0] * point.x + rotation[1][1] * point.y +
        rotation[1][2] * point.z + translation.y,
        rotation[2][0] * point.x + rotation[2][1] * point.y +
        rotation[2][2] * point.z + translation.z,
    )


def project_point(point_camera, intrinsics):
    """Camera optical frame point를 왜곡 보정된 image pixel에 투영한다."""
    if point_camera.z <= 0.0:
        return None
    return (
        intrinsics.fx * point_camera.x / point_camera.z + intrinsics.cx,
        intrinsics.fy * point_camera.y / point_camera.z + intrinsics.cy,
    )


def associate_bbox(points_lidar, rotation, translation, intrinsics, bbox,
                   min_points=3):
    """YOLO bbox 안에 투영된 LiDAR point의 median 3D 위치를 반환한다."""
    if min_points < 1:
        raise ValueError('min_points must be positive')
    if bbox.u_min > bbox.u_max or bbox.v_min > bbox.v_max:
        raise ValueError('invalid bounding box')
    matches = []
    for point_lidar in points_lidar:
        point_camera = transform_point(point_lidar, rotation, translation)
        pixel = project_point(point_camera, intrinsics)
        if pixel is None:
            continue
        u, v = pixel
        if bbox.u_min <= u <= bbox.u_max and bbox.v_min <= v <= bbox.v_max:
            matches.append((point_lidar, point_camera))

    if len(matches) < min_points:
        return None

    lidar = Point3(*(
        statistics.median(getattr(pair[0], axis) for pair in matches)
        for axis in ('x', 'y', 'z')))
    camera = Point3(*(
        statistics.median(getattr(pair[1], axis) for pair in matches)
        for axis in ('x', 'y', 'z')))
    return Association(
        lidar, camera, math.sqrt(lidar.x ** 2 + lidar.y ** 2 + lidar.z ** 2),
        len(matches), tuple(pair[0] for pair in matches))
