#!/usr/bin/env python3

# Copyright 2026 JungSeong
# Licensed under the Apache License, Version 2.0

# Map frame의 고정 waypoint 목록을 Nav2 FollowWaypoints action으로 반복 요청하는 patrol node다.
# 입력은 평탄화한 x(m)·y(m)·yaw(rad), lap 수와 시작 지연시간(second) parameter다.
# Goal 수락·완료·누락 결과를 `/patrol/status`에 발행해 evaluator가 완주 여부를 기록한다.
# 이 node는 경로를 직접 계산하지 않고 Nav2 action server의 planning과 control을 사용한다.

import math

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


class WarehousePatrolNode(Node):
    def __init__(self):
        super().__init__('warehouse_patrol')
        self.frame_id = self.declare_parameter('frame_id', 'map').value
        self.waypoints = list(self.declare_parameter(
            'waypoints', [
                -3.0, -4.0, 0.0,
                3.0, -4.0, 1.57,
                3.0, 4.0, 3.14,
                -3.0, 4.0, -1.57,
            ]).value)
        self.laps = int(self.declare_parameter('laps', 3).value)
        self.start_delay = float(
            self.declare_parameter('start_delay', 12.0).value)
        if len(self.waypoints) < 3 or len(self.waypoints) % 3 != 0:
            raise ValueError('waypoints must be flattened x,y,yaw triples')
        if self.laps < 1 or self.start_delay < 0.0:
            raise ValueError('laps must be positive and start_delay non-negative')
        self.client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        self.status_publisher = self.create_publisher(
            String, '/patrol/status', 10)
        self.timer = self.create_timer(self.start_delay, self.start)

    def start(self):
        self.timer.cancel()
        if not self.client.wait_for_server(timeout_sec=30.0):
            self.publish_status('failed: follow_waypoints unavailable')
            return
        goal = FollowWaypoints.Goal()
        goal.poses = self.make_poses() * self.laps
        self.publish_status(f'started: {self.laps} laps')
        future = self.client.send_goal_async(goal)
        future.add_done_callback(self.on_goal_response)

    def make_poses(self):
        poses = []
        stamp = self.get_clock().now().to_msg()
        for index in range(0, len(self.waypoints), 3):
            x, y, yaw = self.waypoints[index:index + 3]
            pose = PoseStamped()
            pose.header.frame_id = self.frame_id
            pose.header.stamp = stamp
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.orientation.z = math.sin(float(yaw) * 0.5)
            pose.pose.orientation.w = math.cos(float(yaw) * 0.5)
            poses.append(pose)
        return poses

    def on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.publish_status('failed: patrol goal rejected')
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.on_result)

    def on_result(self, future):
        result = future.result()
        missed = len(result.result.missed_waypoints)
        if missed:
            self.publish_status(f'failed: {missed} missed waypoints')
        else:
            self.publish_status('completed')

    def publish_status(self, text):
        self.status_publisher.publish(String(data=text))
        self.get_logger().info(text)


def main(args=None):
    rclpy.init(args=args)
    node = WarehousePatrolNode()
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
