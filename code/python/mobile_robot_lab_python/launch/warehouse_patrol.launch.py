#!/usr/bin/env python3

# Copyright 2026 JungSeong
# Licensed under the Apache License, Version 2.0

# Warehouse simulation, Nav2, person perception, tracking과 safety node의 lifecycle을 구성한다.
# 입력 launch argument로 world와 사람 seed, YOLO model/device, evaluator output을 선택한다.
# perception은 Camera/LiDAR timestamp를 유지하고 tracking과 safety는 odom 좌표계를 공유한다.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from launch_ros.actions import SetRemap
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    lab_share = get_package_share_directory('mobile_robot_lab_python')
    tb3_share = get_package_share_directory('turtlebot3_gazebo')
    nav2_share = get_package_share_directory('nav2_bringup')
    # Nav2 Jazzy는 slam 값을 PythonExpression에 넣으므로 대소문자와 무관한 bool 표현으로 전달한다.
    slam_enabled = PythonExpression([
        "'", LaunchConfiguration('slam'), "'.lower() == 'true'",
    ])
    world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            tb3_share, 'launch', 'turtlebot3_avoidance.launch.py')),
        launch_arguments={
            'world_dir': LaunchConfiguration('world_dir'),
            'world': LaunchConfiguration('world'),
            'scenario': LaunchConfiguration('scenario'),
            'implementation': 'none',
            'pedestrian_count': LaunchConfiguration('pedestrian_count'),
            'pedestrian_random_seed': LaunchConfiguration('pedestrian_seed'),
            'pedestrian_path_mode': 'random_walk',
            'x_pose': LaunchConfiguration('x_pose'),
            'y_pose': LaunchConfiguration('y_pose'),
            'yaw': LaunchConfiguration('yaw'),
            'gazebo_gui': LaunchConfiguration('gazebo_gui'),
            'launch_rviz': LaunchConfiguration('launch_rviz'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('launch_sim')),
    )
    nav2 = GroupAction([
        SetRemap(src='/cmd_vel', dst='/cmd_vel_nav'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                nav2_share, 'launch', 'bringup_launch.py')),
            launch_arguments={
                'slam': slam_enabled,
                'map': LaunchConfiguration('map'),
                'use_sim_time': 'true',
                'params_file': LaunchConfiguration('nav2_params_file'),
                'autostart': 'true',
            }.items()),
    ], condition=IfCondition(LaunchConfiguration('run_navigation')))
    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            lab_share, 'launch', 'yolo_lidar_fusion.launch.py')),
        launch_arguments={
            'model_path': LaunchConfiguration('model_path'),
            'device': LaunchConfiguration('device'),
            'confidence': LaunchConfiguration('confidence'),
            'target_class': 'person',
            'publish_annotated_image': LaunchConfiguration(
                'publish_annotated_image'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('run_perception')))
    safety = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            lab_share, 'launch', 'semantic_safety.launch.py')),
        condition=IfCondition(LaunchConfiguration('run_safety')))
    patrol = Node(
        package='mobile_robot_lab_python',
        executable='warehouse_patrol',
        parameters=[os.path.join(
            lab_share, 'config', 'warehouse_patrol.yaml')],
        output='screen',
        condition=IfCondition(LaunchConfiguration('run_patrol')))
    ground_truth_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/default/dynamic_pose/info'
            '@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ],
        remappings=[
            ('/world/default/dynamic_pose/info', '/evaluation/world_poses'),
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('run_evaluator')))
    evaluator = Node(
        package='mobile_robot_lab_python',
        executable='tracking_evaluator',
        parameters=[{
            'use_sim_time': True,
            'output_path': LaunchConfiguration('evaluation_output_path'),
            'person_name_prefix': LaunchConfiguration(
                'evaluation_person_name_prefix'),
            'duration_seconds': ParameterValue(
                LaunchConfiguration('evaluation_duration_seconds'),
                value_type=float),
            'expected_person_count': ParameterValue(
                LaunchConfiguration('evaluation_expected_person_count'),
                value_type=int),
            'robot_initial_world_x': ParameterValue(
                LaunchConfiguration('x_pose'), value_type=float),
            'robot_initial_world_y': ParameterValue(
                LaunchConfiguration('y_pose'), value_type=float),
            'robot_initial_world_yaw': ParameterValue(
                LaunchConfiguration('yaw'), value_type=float),
            'max_time_difference_seconds': ParameterValue(
                LaunchConfiguration('evaluation_max_time_difference_seconds'),
                value_type=float),
            'person_reference_height': ParameterValue(
                LaunchConfiguration('evaluation_person_reference_height'),
                value_type=float),
            'lidar_frame': LaunchConfiguration('evaluation_lidar_frame'),
            'lidar_minimum_range': ParameterValue(
                LaunchConfiguration('evaluation_lidar_minimum_range'),
                value_type=float),
            'lidar_maximum_range': ParameterValue(
                LaunchConfiguration('evaluation_lidar_maximum_range'),
                value_type=float),
            'lidar_horizontal_minimum': ParameterValue(
                LaunchConfiguration('evaluation_lidar_horizontal_minimum'),
                value_type=float),
            'lidar_horizontal_maximum': ParameterValue(
                LaunchConfiguration('evaluation_lidar_horizontal_maximum'),
                value_type=float),
            'lidar_vertical_minimum': ParameterValue(
                LaunchConfiguration('evaluation_lidar_vertical_minimum'),
                value_type=float),
            'lidar_vertical_maximum': ParameterValue(
                LaunchConfiguration('evaluation_lidar_vertical_maximum'),
                value_type=float),
        }],
        output='screen',
        condition=IfCondition(LaunchConfiguration('run_evaluator')))

    return LaunchDescription([
        DeclareLaunchArgument('launch_sim', default_value='true'),
        DeclareLaunchArgument('slam', default_value='false'),
        DeclareLaunchArgument('run_patrol', default_value='true'),
        DeclareLaunchArgument('run_navigation', default_value='true'),
        DeclareLaunchArgument('run_evaluator', default_value='true'),
        DeclareLaunchArgument('run_perception', default_value='true'),
        DeclareLaunchArgument('run_safety', default_value='true'),
        DeclareLaunchArgument('scenario', default_value='multi_person'),
        DeclareLaunchArgument(
            'evaluation_output_path',
            default_value='/ws/data/results/tracking_metrics.json'),
        DeclareLaunchArgument(
            'evaluation_person_name_prefix',
            default_value='avoidance_person_'),
        DeclareLaunchArgument(
            'evaluation_duration_seconds', default_value='0.0'),
        DeclareLaunchArgument(
            'evaluation_expected_person_count', default_value='0'),
        DeclareLaunchArgument(
            'evaluation_max_time_difference_seconds', default_value='0.05'),
        DeclareLaunchArgument(
            'evaluation_person_reference_height', default_value='0.9'),
        DeclareLaunchArgument(
            'evaluation_lidar_frame', default_value='base_scan'),
        DeclareLaunchArgument(
            'evaluation_lidar_minimum_range', default_value='0.1'),
        DeclareLaunchArgument(
            'evaluation_lidar_maximum_range', default_value='20.0'),
        DeclareLaunchArgument(
            'evaluation_lidar_horizontal_minimum', default_value='-3.141593'),
        DeclareLaunchArgument(
            'evaluation_lidar_horizontal_maximum', default_value='3.141593'),
        DeclareLaunchArgument(
            'evaluation_lidar_vertical_minimum', default_value='-0.261799'),
        DeclareLaunchArgument(
            'evaluation_lidar_vertical_maximum', default_value='0.261799'),
        DeclareLaunchArgument(
            'publish_annotated_image', default_value='false'),
        DeclareLaunchArgument(
            'map', default_value='/ws/data/maps/small_warehouse.yaml'),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=os.path.join(nav2_share, 'params', 'nav2_params.yaml')),
        DeclareLaunchArgument(
            'world_dir',
            default_value='/opt/aws_warehouse/worlds/small_warehouse'),
        DeclareLaunchArgument(
            'world', default_value='small_warehouse_harmonic.world'),
        DeclareLaunchArgument('pedestrian_count', default_value='3'),
        DeclareLaunchArgument('pedestrian_seed', default_value='42'),
        DeclareLaunchArgument('model_path', default_value=(
            '/ws/model/yolo-seg/yolo11s-seg.pt')),
        DeclareLaunchArgument('device', default_value='0'),
        DeclareLaunchArgument('confidence', default_value='0.25'),
        DeclareLaunchArgument('x_pose', default_value='-2.0'),
        DeclareLaunchArgument('y_pose', default_value='-0.5'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('gazebo_gui', default_value='true'),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        world,
        TimerAction(period=8.0, actions=[nav2]),
        TimerAction(period=8.0, actions=[perception, safety]),
        TimerAction(period=10.0, actions=[patrol]),
        TimerAction(period=10.0, actions=[ground_truth_bridge, evaluator]),
    ])
