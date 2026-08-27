#!/usr/bin/env python3

# Copyright 2026 JungSeong
# Licensed under the Apache License, Version 2.0

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
from launch_ros.actions import Node
from launch_ros.actions import SetRemap


def generate_launch_description():
    lab_share = get_package_share_directory('mobile_robot_lab_python')
    tb3_share = get_package_share_directory('turtlebot3_gazebo')
    nav2_share = get_package_share_directory('nav2_bringup')
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
                'slam': LaunchConfiguration('slam'),
                'map': LaunchConfiguration('map'),
                'use_sim_time': 'true',
                'params_file': LaunchConfiguration('nav2_params_file'),
                'autostart': 'true',
            }.items()),
    ])
    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            lab_share, 'launch', 'yolo_lidar_fusion.launch.py')),
        launch_arguments={
            'model_path': LaunchConfiguration('model_path'),
            'device': LaunchConfiguration('device'),
            'target_class': 'person',
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
        }],
        output='screen',
        condition=IfCondition(LaunchConfiguration('run_evaluator')))

    return LaunchDescription([
        DeclareLaunchArgument('launch_sim', default_value='true'),
        DeclareLaunchArgument('slam', default_value='false'),
        DeclareLaunchArgument('run_patrol', default_value='true'),
        DeclareLaunchArgument('run_evaluator', default_value='true'),
        DeclareLaunchArgument('run_perception', default_value='true'),
        DeclareLaunchArgument('run_safety', default_value='true'),
        DeclareLaunchArgument('scenario', default_value='multi_person'),
        DeclareLaunchArgument(
            'evaluation_output_path',
            default_value='/ws/data/results/tracking_metrics.json'),
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
            '/ws/model/yolo-world/yolov8s-worldv2.pt')),
        DeclareLaunchArgument('device', default_value='0'),
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
