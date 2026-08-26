#!/usr/bin/env python3

# Copyright 2026 JungSeong
# Licensed under the Apache License, Version 2.0

import math

from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
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

from mobile_robot_lab_python.camera_lidar_fusion import associate_bbox
from mobile_robot_lab_python.camera_lidar_fusion import BoundingBox
from mobile_robot_lab_python.camera_lidar_fusion import Intrinsics
from mobile_robot_lab_python.camera_lidar_fusion import Point3


def quaternion_rotation(quaternion):
    """Convert a geometry_msgs quaternion to a 3x3 rotation matrix."""
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


class LidarBboxAssociationNode(Node):
    def __init__(self):
        super().__init__('lidar_bbox_association')
        self.declare_parameter('points_topic', '/calib/points')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('detections_topic', '/detections')
        self.declare_parameter('target_class', 'cart')
        self.declare_parameter('min_score', 0.5)
        self.declare_parameter('min_points', 3)
        self.declare_parameter('max_time_delta', 0.1)

        self.target_class = self.get_parameter('target_class').value
        self.min_score = float(self.get_parameter('min_score').value)
        self.min_points = int(self.get_parameter('min_points').value)
        self.max_time_delta = float(self.get_parameter('max_time_delta').value)
        if self.min_points < 1 or self.max_time_delta < 0.0:
            raise ValueError(
                'min_points must be positive and max_time_delta non-negative')

        self.camera_info = None
        self.detections = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(
            CameraInfo, self.get_parameter('camera_info_topic').value,
            self.on_camera_info, qos_profile_sensor_data)
        self.create_subscription(
            Detection2DArray, self.get_parameter('detections_topic').value,
            self.on_detections, qos_profile_sensor_data)
        self.create_subscription(
            PointCloud2, self.get_parameter('points_topic').value,
            self.on_points, qos_profile_sensor_data)
        self.points_publisher = self.create_publisher(
            PointCloud2, '/fusion/associated_points', 10)
        self.pose_publisher = self.create_publisher(
            PoseArray, '/fusion/detections_3d', 10)

    def on_camera_info(self, message):
        self.camera_info = message

    def on_detections(self, message):
        self.detections = message

    def on_points(self, cloud):
        if self.camera_info is None or self.detections is None:
            return
        cloud_stamp = stamp_seconds(cloud.header.stamp)
        detection_stamp = stamp_seconds(self.detections.header.stamp)
        time_delta = abs(cloud_stamp - detection_stamp)
        if time_delta > self.max_time_delta:
            self.get_logger().warn(
                f'rejecting stale detections: time_delta={time_delta:.3f}s',
                throttle_duration_sec=2.0)
            return

        camera_frame = self.camera_info.header.frame_id
        try:
            transform = self.tf_buffer.lookup_transform(
                camera_frame, cloud.header.frame_id,
                Time.from_msg(cloud.header.stamp),
                timeout=Duration(seconds=0.05))
        except TransformException as error:
            self.get_logger().warn(
                f'TF {cloud.header.frame_id} -> {camera_frame} unavailable: '
                f'{error}',
                throttle_duration_sec=2.0)
            return

        rotation = quaternion_rotation(transform.transform.rotation)
        vector = transform.transform.translation
        translation = Point3(vector.x, vector.y, vector.z)
        intrinsics = Intrinsics(
            self.camera_info.k[0], self.camera_info.k[4],
            self.camera_info.k[2], self.camera_info.k[5])
        lidar_points = [
            Point3(float(point[0]), float(point[1]), float(point[2]))
            for point in point_cloud2.read_points(
                cloud, field_names=('x', 'y', 'z'), skip_nans=True)
        ]

        poses = PoseArray(header=cloud.header)
        associated_points = []
        for detection in self.detections.detections:
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
            association = associate_bbox(
                lidar_points, rotation, translation, intrinsics,
                BoundingBox(
                    center.x - half_width, center.y - half_height,
                    center.x + half_width, center.y + half_height),
                self.min_points)
            if association is None:
                continue

            pose = Pose()
            pose.position.x = association.position_lidar.x
            pose.position.y = association.position_lidar.y
            pose.position.z = association.position_lidar.z
            pose.orientation.w = 1.0
            poses.poses.append(pose)
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
