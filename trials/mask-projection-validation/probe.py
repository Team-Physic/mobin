#!/usr/bin/env python3
# Binary mask 보존과 동일 pixel 깊이 기반 가림 제외를 실제 ROS association에서 검증한다.
# 합성 입력은 Camera optical 좌표[m], 영상 160×120 pixel, 동일 timestamp를 사용한다.
# 모델 추론·Gazebo는 실행하지 않으며 결과 JSON은 stdout, 임시 ROS 로그는 /tmp에 쓴다.
# 사람 정답은 검증에만 사용한다. 희소 cloud의 벽 오채택과 이웃 pixel 보존도 기록한다.
# 인자 없음. ROS_DOMAIN_ID는 다른 ROS 실행과 격리된 값으로 지정한다(재현 명령: 86).

import hashlib
import inspect
import json
from pathlib import Path
import subprocess
import tempfile
import time

import cv2
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header
from tf2_ros import StaticTransformBroadcaster
import torch
import ultralytics
from ultralytics.engine.results import Masks
from ultralytics.utils.ops import masks2segments
from vision_msgs.msg import Detection2DArray, Detection3DArray

from mobile_robot_lab_python.yolo_segmentation_detector_node import (
    build_detection_array, build_instance_label_map,
)


def polygon_baseline(mask):
    """수정 전 외곽 polygon 재채움을 비교용으로만 재현한다."""
    polygons = Masks(torch.from_numpy(mask[None]), mask.shape).xy
    labels = np.zeros_like(mask)
    for index, polygon in enumerate(polygons, start=1):
        points = np.asarray(polygon, dtype=np.float32)
        if points.size == 0:
            continue
        points[:, 0] = np.clip(points[:, 0], 0, mask.shape[1] - 1)
        points[:, 1] = np.clip(points[:, 1], 0, mask.shape[0] - 1)
        cv2.fillPoly(labels, [np.rint(points).astype(np.int32)], index)
    return labels


def main():
    """Mask 보존과 실제 ROS 노드 출력을 검증하고 관측 결과를 JSON으로 남긴다."""
    rectangle = np.zeros((120, 160), np.uint8)
    rectangle[40:81, 70:91] = 1
    hole = rectangle.copy()
    hole[57:64, 77:84] = 0
    split = np.zeros_like(rectangle)
    split[40:55, 70:80] = 1
    split[65:81, 82:92] = 1
    mask_results = {}
    for name, original in [('rectangle', rectangle), ('hole', hole), ('split', split)]:
        converted = build_instance_label_map(*original.shape, [original])
        previous = polygon_baseline(original)
        mask_results[name] = {
            'original_pixels': int(original.sum()),
            'added_pixels': int(((converted > 0) & (original == 0)).sum()),
            'removed_pixels': int(((converted == 0) & (original > 0)).sum()),
            'previous_added_pixels': int(((previous > 0) & (original == 0)).sum()),
        }
        np.testing.assert_array_equal(converted, original)
    assert mask_results['rectangle']['added_pixels'] == 0
    assert mask_results['hole']['added_pixels'] == 0
    assert mask_results['hole']['previous_added_pixels'] == 49

    # Camera에서 가린 벽 점은 x=0.8m의 LiDAR에서 보인다.
    # LiDAR→벽 선분의 z=2m 교차 x는 약 0.48m: 사람 반폭 0.2m 밖이다.
    person = [[x, y, 2.0] for x in [-0.12, 0, 0.12] for y in [-0.2, 0, 0.2]]
    wall = [[x, y, 5.0] for x in [-0.03, 0, 0.03] for y in [-0.02, 0.02]]
    crossings = [0.8 + (p[0] - 0.8) * 2.0 / p[2] for p in wall]
    assert all(x > 0.2 for x in crossings)
    occluder = [[(i % 10 - 4.5) * 0.002, (i // 10 - 2.5) * 0.002, 1.0]
                for i in range(60)]
    same_pixel_wall = [[x, y, 5.0] for x in [-0.01, 0, 0.01] for y in [-0.01, 0.01]]
    partial_person = [[0.0, y, 2.0] for y in [-0.2, 0, 0.2]]
    far_person = [[0.0, y, 5.0] for y in [-0.01, 0.0, 0.01]]
    cases = [
        # 이름, mask, cloud, 기대 출력 깊이, 기대 association 점 수, 사람 정답 깊이[m]
        ('person_only', rectangle, person, 2.0, 9, 2.0),
        ('perfect_hole_excludes_front_object', hole, person + occluder, 2.0, 8, 2.0),
        ('polygon_baseline_accepts_front_object', polygon_baseline(hole), person + occluder, 1.0, 60, 2.0),
        ('binary_mask_excludes_front_object', build_instance_label_map(*hole.shape, [hole]), person + occluder, 2.0, 8, 2.0),
        ('perfect_mask_accepts_hidden_wall_only', rectangle, wall, 5.0, 6, 2.0),
        ('perfect_mask_mixed_person_and_wall', rectangle, person + wall[:3], 2.0, 9, 2.0),
        ('same_pixel_wall_removed_partial_person_preserved', rectangle,
         partial_person + same_pixel_wall, 2.0, 3, 2.0),
        ('adjacent_pixel_does_not_erase_partial_person', rectangle,
         [[-0.01, 0, 1.0]] + far_person, 5.0, 3, 5.0),
        ('close_depths_preserved', rectangle,
         [[0, 0, depth] for depth in [2.0, 2.03, 2.04]], 2.03, 3, 2.03),
        ('same_pixel_front_sample_blocks_far_points', rectangle,
         [[0, 0, 1.0]] + far_person, None, 0, 5.0),
    ]

    rclpy.init()
    node = rclpy.create_node('mask_projection_probe')
    bridge = CvBridge()
    pubs = {
        'info': node.create_publisher(CameraInfo, '/camera/camera_info', qos_profile_sensor_data),
        'labels': node.create_publisher(Image, '/segmentation/instance_labels', qos_profile_sensor_data),
        'detections': node.create_publisher(Detection2DArray, '/detections', qos_profile_sensor_data),
        'cloud': node.create_publisher(PointCloud2, '/calib/points', qos_profile_sensor_data),
    }
    received = {}
    def on_output(message):
        stamp = message.header.stamp
        received[(stamp.sec, stamp.nanosec)] = [d.bbox.center.position.z for d in message.detections]
    subscription = node.create_subscription(Detection3DArray, '/fusion/detections_3d', on_output, 10)
    point_counts = {}
    def on_points(message):
        stamp = message.header.stamp
        point_counts[(stamp.sec, stamp.nanosec)] = message.width * message.height
    points_subscription = node.create_subscription(
        PointCloud2, '/fusion/associated_points', on_points, 10)
    diagnostics = {}
    def on_diagnostics(message):
        for status in message.status:
            if status.name == 'lidar_bbox_association/association':
                stamp = message.header.stamp
                diagnostics[(stamp.sec, stamp.nanosec)] = {
                    item.key: int(item.value) for item in status.values}
    diagnostics_subscription = node.create_subscription(
        DiagnosticArray, '/fusion/diagnostics', on_diagnostics, 10)
    broadcaster = StaticTransformBroadcaster(node)
    camera = TransformStamped()
    camera.header.frame_id = 'base_footprint'
    camera.child_frame_id = 'probe_camera_optical'
    camera.transform.translation.z = 1.0
    q = camera.transform.rotation
    q.x, q.y, q.z, q.w = -0.5, 0.5, -0.5, 0.5
    lidar = TransformStamped()
    lidar.header.frame_id = 'probe_camera_optical'
    lidar.child_frame_id = 'probe_lidar_optical'
    lidar.transform.translation.x = 0.8
    lidar.transform.rotation.w = 1.0
    broadcaster.sendTransform([camera, lidar])
    executable = '/ws/install/mobile_robot_lab_cpp/lib/mobile_robot_lab_cpp/lidar_bbox_association_cpp'
    results = {}
    with tempfile.TemporaryDirectory(prefix='mask-probe-') as tmp:
        with open(Path(tmp) / 'node.log', 'w+') as log:
            process = subprocess.Popen([
                executable, '--ros-args', '-p', 'tracking_frame:=probe_camera_optical',
            ], stdout=log, stderr=subprocess.STDOUT)
            try:
                deadline = time.monotonic() + 15
                while not all(pub.get_subscription_count() for pub in pubs.values()):
                    assert process.poll() is None, 'association node exited'
                    assert time.monotonic() < deadline, 'ROS discovery timeout'
                    rclpy.spin_once(node, timeout_sec=0.1)
                for name, labels, points, expected_depth, expected_count, person_depth in cases:
                    stamp = node.get_clock().now().to_msg()
                    image_header = Header(stamp=stamp, frame_id='probe_camera_optical')
                    cloud_header = Header(stamp=stamp, frame_id='probe_lidar_optical')
                    info = CameraInfo(header=image_header, width=160, height=120)
                    info.k = [100., 0., 80., 0., 100., 60., 0., 0., 1.]
                    image = bridge.cv2_to_imgmsg(labels, encoding='mono8')
                    image.header = image_header
                    detections = build_detection_array(image_header, [(1, 'person', .99, 70., 40., 90., 80.)])
                    cloud = create_cloud_xyz32(cloud_header, [[x - .8, y, z] for x, y, z in points])
                    key = (stamp.sec, stamp.nanosec)
                    deadline = time.monotonic() + 5
                    next_publish = 0.0
                    while key not in received or key not in point_counts or key not in diagnostics:
                        assert time.monotonic() < deadline, f'output timeout: {name}'
                        # 출력 callback을 기다리는 동안 동일 입력을 폭주시키지 않는다.
                        if time.monotonic() >= next_publish:
                            pubs['info'].publish(info)
                            pubs['labels'].publish(image)
                            pubs['detections'].publish(detections)
                            pubs['cloud'].publish(cloud)
                            next_publish = time.monotonic() + 1.0
                        rclpy.spin_once(node, timeout_sec=0.1)
                    depths = received[key]
                    assert (not depths if expected_depth is None else
                            len(depths) == 1 and abs(depths[0] - expected_depth) < 1e-5), (name, depths)
                    assert point_counts[key] == expected_count, (name, point_counts[key])
                    counts = diagnostics[key]
                    assert counts['ground_rejection'] == 0, (name, counts)
                    if name == 'same_pixel_front_sample_blocks_far_points':
                        assert counts['no_points'] > 0 and counts['camera_occluded_points'] >= 3
                    results[name] = {
                        'input_points': len(points), 'observed_depth_m': depths[0] if depths else None,
                        'associated_points': point_counts[key],
                        'person_surface_depth_m': person_depth,
                        'person_selected': bool(depths) and abs(depths[0] - person_depth) < 1e-5,
                        'cumulative_diagnostics': counts,
                    }
            finally:
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
                log.seek(0)
                node_log = log.read()
    node.destroy_subscription(subscription)
    node.destroy_subscription(points_subscription)
    node.destroy_subscription(diagnostics_subscription)
    node.destroy_node()
    rclpy.shutdown()
    print(json.dumps({
        'scope': 'synthetic masks + analytic visibility + real ROS C++ node; no YOLO inference or Gazebo',
        'versions': {'ultralytics': ultralytics.__version__, 'opencv': cv2.__version__, 'numpy': np.__version__},
        'masks2segments_source': inspect.getsource(masks2segments),
        'mask_roundtrip': mask_results,
        'wall_ray_x_at_person_depth_m': crossings,
        'ros_cases': results,
        'binary_sha256': hashlib.sha256(Path(executable).read_bytes()).hexdigest(),
        'node_log': node_log,
        'checks': 'binary masks preserved; same-pixel occlusion filtered; neighbors and close depths preserved; wall-only failure remains',
    }, indent=2))


if __name__ == '__main__':
    main()
