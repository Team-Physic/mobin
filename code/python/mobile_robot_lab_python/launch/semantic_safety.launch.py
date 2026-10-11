#!/usr/bin/env python3

# Copyright 2026 JungSeong
# Licensed under the Apache License, Version 2.0

# Nav2와 teleop 명령을 우선순위로 합치고 모든 명령을 동일한 semantic safety gate에 통과시킨다.
# 입력·threshold는 twist_mux.yaml과 semantic_safety.yaml에서 읽어 세 C++ node에 전달한다.
# 출력 흐름은 `/cmd_vel_mux`에서 `/cmd_vel_safe`, watchdog의 최종 `/cmd_vel` 순서다.
# 이 launch는 simulation이나 perception을 시작하지 않으므로 해당 topic publisher가 별도로 필요하다.

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
