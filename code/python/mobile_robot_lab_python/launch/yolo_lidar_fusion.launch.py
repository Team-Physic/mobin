#!/usr/bin/env python3

# Copyright 2026 JungSeong
# Licensed under the Apache License, Version 2.0

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'model_path',
            default_value='/ws/model/yolo-world/yolov8s-worldv2.pt'),
        DeclareLaunchArgument('device', default_value='cpu'),
        DeclareLaunchArgument('target_class', default_value='person'),
        DeclareLaunchArgument('confidence', default_value='0.05'),
        DeclareLaunchArgument('max_time_delta', default_value='0.05'),
        DeclareLaunchArgument('assignment_distance', default_value='0.75'),
        Node(
            package='mobile_robot_lab_python',
            executable='yolo_world_detector',
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
