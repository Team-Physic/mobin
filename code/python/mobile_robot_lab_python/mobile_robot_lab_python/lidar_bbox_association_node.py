#!/usr/bin/env python3

# Copyright 2026 JungSeong
# Licensed under the Apache License, Version 2.0

# YOLO bbox와 동기화한 LiDAR point를 Camera pixel에 투영해 사람의 3D 위치를 만드는 Python 기준선이다.
# Detection과 PointCloud의 timestamp 차이를 제한하고 TF로 LiDAR→Camera·ground·odom 변환을 조회한다.
# bbox 내부 point의 median 위치를 PoseArray로, 선택 point와 Euclidean track Marker를 debug topic에 발행한다.
# 현재 production graph는 Instance Mask·RANSAC·Kalman을 구현한 C++ node를 사용하며 이 module은 비교용이다.

import math

from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Point
from message_filters import ApproximateTimeSynchronizer
from message_filters import Subscriber
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer
from tf2_ros import TransformException
from tf2_ros import TransformListener
from vision_msgs.msg import Detection2DArray
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from mobile_robot_lab_python.camera_lidar_fusion import associate_bbox
from mobile_robot_lab_python.camera_lidar_fusion import BoundingBox
from mobile_robot_lab_python.camera_lidar_fusion import Intrinsics
from mobile_robot_lab_python.camera_lidar_fusion import Point3
from mobile_robot_lab_python.camera_lidar_fusion import transform_point
from mobile_robot_lab_python.person_tracker import PersonTracker


def quaternion_rotation(quaternion):
    """geometry_msgs quaternion을 정규화해 3×3 회전행렬로 변환한다."""
    norm = math.sqrt(sum(
        value * value for value in (
            quaternion.x, quaternion.y, quaternion.z, quaternion.w)))
    if norm == 0.0:
        raise ValueError('zero-length transform quaternion')
    x = quaternion.x / norm
    y = quaternion.y / norm
    z = quaternion.z / norm
    w = quaternion.w / norm
    return (
        (1 - 2 * (y * y + z * z), 2 * (x * y - z * w),
         2 * (x * z + y * w)),
        (2 * (x * y + z * w), 1 - 2 * (x * x + z * z),
         2 * (y * z - x * w)),
        (2 * (x * z - y * w), 2 * (y * z + x * w),
         1 - 2 * (x * x + y * y)),
    )


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def transform_parts(transform):
    vector = transform.transform.translation
    return (
        quaternion_rotation(transform.transform.rotation),
        Point3(vector.x, vector.y, vector.z),
    )


def remove_ground(points, rotation, translation, minimum_height):
    """Point를 Ground Reference Frame으로 변환해 최소 z보다 높은 점만 남긴다."""
    return [
        point for point in points
        if transform_point(point, rotation, translation).z > minimum_height
    ]


def track_markers(tracks, header):
    markers = MarkerArray()
    clear = Marker(header=header, action=Marker.DELETEALL)
    markers.markers.append(clear)
    for track in tracks:
        position = Point(
            x=track.position[0], y=track.position[1], z=track.position[2])

        sphere = Marker(
            header=header, ns='person_position', id=track.track_id,
            type=Marker.SPHERE, action=Marker.ADD)
        sphere.pose.position = position
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.25
        sphere.color.r = 1.0
        sphere.color.g = 0.2
        sphere.color.a = 0.9
        markers.markers.append(sphere)

        label = Marker(
            header=header, ns='person_id', id=track.track_id,
            type=Marker.TEXT_VIEW_FACING, action=Marker.ADD)
        label.pose.position = Point(
            x=position.x, y=position.y, z=position.z + 0.35)
        label.pose.orientation.w = 1.0
        label.scale.z = 0.22
        label.color.r = label.color.g = label.color.b = label.color.a = 1.0
        label.text = f'person {track.track_id}'
        markers.markers.append(label)

        trajectory = Marker(
            header=header, ns='person_trajectory', id=track.track_id,
            type=Marker.LINE_STRIP, action=Marker.ADD)
        trajectory.pose.orientation.w = 1.0
        trajectory.scale.x = 0.05
        trajectory.color.g = 1.0
        trajectory.color.a = 0.9
        trajectory.points = [Point(x=x, y=y, z=z) for x, y, z in track.trajectory]
        markers.markers.append(trajectory)
    return markers


class LidarBboxAssociationNode(Node):
    def __init__(self):
        super().__init__('lidar_bbox_association')
        self.declare_parameter('points_topic', '/calib/points')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('detections_topic', '/detections')
        self.declare_parameter('target_class', 'person')
        self.declare_parameter('min_score', 0.5)
        self.declare_parameter('min_points', 3)
        self.declare_parameter('max_time_delta', 0.05)
        self.declare_parameter('sync_queue_size', 30)
        self.declare_parameter('ground_frame', 'base_footprint')
        self.declare_parameter('ground_height', 0.05)
        self.declare_parameter('tracking_frame', 'odom')
        self.declare_parameter('assignment_distance', 0.75)
        self.declare_parameter('track_max_age', 1.0)
        self.declare_parameter('trajectory_length', 50)

        self.target_class = self.get_parameter('target_class').value
        self.min_score = float(self.get_parameter('min_score').value)
        self.min_points = int(self.get_parameter('min_points').value)
        self.max_time_delta = float(self.get_parameter('max_time_delta').value)
        self.sync_queue_size = int(
            self.get_parameter('sync_queue_size').value)
        self.ground_frame = self.get_parameter('ground_frame').value
        self.ground_height = float(self.get_parameter('ground_height').value)
        self.tracking_frame = self.get_parameter('tracking_frame').value
        assignment_distance = float(
            self.get_parameter('assignment_distance').value)
        track_max_age = float(self.get_parameter('track_max_age').value)
        trajectory_length = int(
            self.get_parameter('trajectory_length').value)
        if (self.min_points < 1 or self.max_time_delta < 0.0 or
                self.sync_queue_size < 1):
            raise ValueError(
                'min_points and sync_queue_size must be positive; '
                'max_time_delta must be non-negative')
        self.tracker = PersonTracker(
            assignment_distance, track_max_age, trajectory_length)

        self.camera_info = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(
            CameraInfo, self.get_parameter('camera_info_topic').value,
            self.on_camera_info, qos_profile_sensor_data)
        self.detections_subscriber = Subscriber(
            self, Detection2DArray,
            self.get_parameter('detections_topic').value,
            qos_profile=qos_profile_sensor_data)
        self.points_subscriber = Subscriber(
            self, PointCloud2, self.get_parameter('points_topic').value,
            qos_profile=qos_profile_sensor_data)
        self.synchronizer = ApproximateTimeSynchronizer(
            [self.detections_subscriber, self.points_subscriber],
            queue_size=self.sync_queue_size,
            slop=self.max_time_delta)
        self.synchronizer.registerCallback(self.on_detection_points)
        self.points_publisher = self.create_publisher(
            PointCloud2, '/fusion/associated_points', 10)
        self.pose_publisher = self.create_publisher(
            PoseArray, '/fusion/detections_3d', 10)
        self.marker_publisher = self.create_publisher(
            MarkerArray, '/fusion/tracks', 10)

    def on_camera_info(self, message):
        self.camera_info = message

    def on_detection_points(self, detections, cloud):
        if self.camera_info is None:
            return
        cloud_stamp = stamp_seconds(cloud.header.stamp)
        detection_stamp = stamp_seconds(detections.header.stamp)
        time_delta = abs(cloud_stamp - detection_stamp)
        if time_delta > self.max_time_delta:
            self.get_logger().warn(
                f'rejecting stale detections: time_delta={time_delta:.3f}s',
                throttle_duration_sec=2.0)
            return

        camera_frame = self.camera_info.header.frame_id
        try:
            camera_transform = self.tf_buffer.lookup_transform(
                camera_frame, cloud.header.frame_id,
                Time.from_msg(cloud.header.stamp),
                timeout=Duration(seconds=0.05))
            ground_transform = self.tf_buffer.lookup_transform(
                self.ground_frame, cloud.header.frame_id,
                Time.from_msg(cloud.header.stamp),
                timeout=Duration(seconds=0.05))
            tracking_transform = self.tf_buffer.lookup_transform(
                self.tracking_frame, cloud.header.frame_id,
                Time.from_msg(cloud.header.stamp),
                timeout=Duration(seconds=0.05))
        except TransformException as error:
            self.get_logger().warn(
                f'TF from {cloud.header.frame_id} unavailable: '
                f'{error}',
                throttle_duration_sec=2.0)
            return

        rotation, translation = transform_parts(camera_transform)
        ground_rotation, ground_translation = transform_parts(ground_transform)
        tracking_rotation, tracking_translation = transform_parts(
            tracking_transform)
        intrinsics = Intrinsics(
            self.camera_info.k[0], self.camera_info.k[4],
            self.camera_info.k[2], self.camera_info.k[5])
        # 같은 cloud timestamp의 TF를 사용해 움직이는 robot에서도 세 변환의 시점을 일치시킨다.
        raw_points = [
            Point3(float(raw[0]), float(raw[1]), float(raw[2]))
            for raw in point_cloud2.read_points(
                cloud, field_names=('x', 'y', 'z'), skip_nans=True)
        ]
        lidar_points = remove_ground(
            raw_points, ground_rotation, ground_translation,
            self.ground_height)

        poses = PoseArray()
        poses.header.stamp = cloud.header.stamp
        poses.header.frame_id = self.tracking_frame
        associated_points = []
        tracking_positions = []
        for detection in detections.detections:
            if not detection.results:
                continue
            best = max(
                detection.results,
                key=lambda result: result.hypothesis.score)
            if best.hypothesis.score < self.min_score:
                continue
            class_id = best.hypothesis.class_id
            if self.target_class and class_id != self.target_class:
                continue
            center = detection.bbox.center.position
            half_width = detection.bbox.size_x * 0.5
            half_height = detection.bbox.size_y * 0.5
            # Python 기준선은 bbox 내부 모든 point의 median을 사용하므로 뒤쪽 표면이 섞일 수 있다.
            association = associate_bbox(
                lidar_points, rotation, translation, intrinsics,
                BoundingBox(
                    center.x - half_width, center.y - half_height,
                    center.x + half_width, center.y + half_height),
                self.min_points)
            if association is None:
                continue

            tracking_position = transform_point(
                association.position_lidar,
                tracking_rotation, tracking_translation)
            pose = Pose()
            pose.position.x = tracking_position.x
            pose.position.y = tracking_position.y
            pose.position.z = tracking_position.z
            pose.orientation.w = 1.0
            poses.poses.append(pose)
            tracking_positions.append((
                tracking_position.x, tracking_position.y, tracking_position.z))
            associated_points.extend(
                (point.x, point.y, point.z)
                for point in association.matched_points_lidar)
            self.get_logger().info(
                f'class={class_id} '
                f'range={association.lidar_range:.3f}m '
                f'points={association.point_count}')

        self.pose_publisher.publish(poses)
        self.points_publisher.publish(point_cloud2.create_cloud_xyz32(
            cloud.header, associated_points))
        tracks = self.tracker.update(tracking_positions, cloud_stamp)
        self.marker_publisher.publish(track_markers(tracks, poses.header))


def main(args=None):
    rclpy.init(args=args)
    node = LidarBboxAssociationNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
