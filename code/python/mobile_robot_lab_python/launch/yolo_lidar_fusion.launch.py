#!/usr/bin/env python3

# Copyright 2026 JungSeong
# Licensed under the Apache License, Version 2.0

# YOLO person segmentation, LiDAR association과 C++ tracker를 하나의 실행으로 연결한다.
# model/device와 동기화, 동일 pixel 가림 깊이 오차, RANSAC 및 전경 지지 비율을 전달한다.
# Detection과 Instance Label Map은 같은 Camera timestamp를 사용하고 3D output은 odom 기준이다.
# 주요 입력: foreground_support_ratio는 무차원 실수 >1, 기본 2.0이며 클수록 복수 후보 채택이 엄격하다.
# camera_occlusion_depth_tolerance: 선택 실수 >=0, 기본 0.05[m]. 이 값 이내 깊이 차이는 보존한다.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'model_path',
            default_value='/ws/model/yolo-seg/yolo11s-seg.pt'),
        DeclareLaunchArgument('device', default_value='cpu'),
        DeclareLaunchArgument('target_class', default_value='person'),
        DeclareLaunchArgument('confidence', default_value='0.25'),
        DeclareLaunchArgument('max_time_delta', default_value='0.05'),
        DeclareLaunchArgument('assignment_distance', default_value='0.75'),
        DeclareLaunchArgument('foreground_support_ratio', default_value='2.0'),
        DeclareLaunchArgument('camera_occlusion_depth_tolerance', default_value='0.05'),
        DeclareLaunchArgument('publish_annotated_image', default_value='false'),
        DeclareLaunchArgument('ground_inlier_distance', default_value='0.05'),
        DeclareLaunchArgument('ground_max_tilt_degrees', default_value='20.0'),
        DeclareLaunchArgument('ground_max_origin_distance', default_value='0.15'),
        DeclareLaunchArgument('ground_min_inliers', default_value='50'),
        Node(
            package='mobile_robot_lab_python',
            executable='yolo_segmentation_detector',
            parameters=[{
                'model_path': LaunchConfiguration('model_path'),
                'device': ParameterValue(
                    LaunchConfiguration('device'), value_type=str),
                'confidence': ParameterValue(
                    LaunchConfiguration('confidence'), value_type=float),
            }],
            output='screen'),
        Node(
            package='mobile_robot_lab_cpp',
            executable='lidar_bbox_association_cpp',
            parameters=[{
                'target_class': LaunchConfiguration('target_class'),
                'min_score': ParameterValue(
                    LaunchConfiguration('confidence'), value_type=float),
                'max_time_delta': ParameterValue(
                    LaunchConfiguration('max_time_delta'), value_type=float),
                'foreground_support_ratio': ParameterValue(
                    LaunchConfiguration('foreground_support_ratio'), value_type=float),
                'camera_occlusion_depth_tolerance': ParameterValue(
                    LaunchConfiguration('camera_occlusion_depth_tolerance'), value_type=float),
                'publish_annotated_image': ParameterValue(
                    LaunchConfiguration('publish_annotated_image'),
                    value_type=bool),
                'ground_inlier_distance': ParameterValue(
                    LaunchConfiguration('ground_inlier_distance'),
                    value_type=float),
                'ground_max_tilt_degrees': ParameterValue(
                    LaunchConfiguration('ground_max_tilt_degrees'),
                    value_type=float),
                'ground_max_origin_distance': ParameterValue(
                    LaunchConfiguration('ground_max_origin_distance'),
                    value_type=float),
                'ground_min_inliers': ParameterValue(
                    LaunchConfiguration('ground_min_inliers'),
                    value_type=int),
            }],
            output='screen'),
        Node(
            package='mobile_robot_lab_cpp',
            executable='person_tracker_cpp',
            parameters=[{
                'euclidean_gate': ParameterValue(
                    LaunchConfiguration('assignment_distance'),
                    value_type=float),
            }],
            output='screen'),
    ])
