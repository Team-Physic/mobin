#!/usr/bin/env python3

# Copyright 2026 JungSeong
# Licensed under the Apache License, Version 2.0

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('mobile_robot_lab_python')
    config_dir = os.path.join(package_share, 'config')
    return LaunchDescription([
        Node(
            package='twist_mux',
            executable='twist_mux',
            parameters=[os.path.join(config_dir, 'twist_mux.yaml')],
            remappings=[('/cmd_vel_out', '/cmd_vel_mux')],
            output='screen'),
        Node(
            package='mobile_robot_lab_cpp',
            executable='semantic_safety_cpp',
            parameters=[os.path.join(config_dir, 'semantic_safety.yaml')],
            output='screen'),
        Node(
            package='mobile_robot_lab_cpp',
            executable='cmd_vel_watchdog_cpp',
            parameters=[os.path.join(config_dir, 'semantic_safety.yaml')],
            output='screen'),
    ])
