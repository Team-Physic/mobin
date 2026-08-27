#!/usr/bin/env python3

# Copyright 2026 JungSeong
# Licensed under the Apache License, Version 2.0

import json
import math
import os

from mobile_robot_lab_msgs.msg import TrackedPersonArray
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String

from mobile_robot_lab_python.tracking_metrics import TrackingMetrics


def yaw(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z
               + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y ** 2 + quaternion.z ** 2))


class TrackingEvaluatorNode(Node):
    def __init__(self):
        super().__init__('tracking_evaluator')
        self.output_path = self.declare_parameter(
            'output_path',
            '/ws/data/results/tracking_metrics.json').value
        self.robot_name_prefix = self.declare_parameter(
            'robot_name_prefix', 'turtlebot3_').value
        self.person_name_prefix = self.declare_parameter(
            'person_name_prefix', 'avoidance_person_').value
        self.max_ground_truth_age = float(self.declare_parameter(
            'max_ground_truth_age', 0.1).value)
        self.metrics = TrackingMetrics(float(self.declare_parameter(
            'distance_gate', 0.75).value))
        self.world_poses = {}
        self.world_pose_receipt = None
        self.odom_pose = None
        self.started_at = self.get_clock().now()
        self.patrol_status = 'not_started'
        self.minimum_person_distance = math.inf
        self.collision_count = 0
        self.in_collision = False
        self.collision_distance = float(self.declare_parameter(
            'collision_distance', 0.65).value)
        self.safety_reason_counts = {}
        self.tracking_latencies = []
        self.create_subscription(
            TFMessage, '/evaluation/world_poses', self.on_world_poses, 10)
        self.create_subscription(
            Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(
            TrackedPersonArray, '/tracking/persons', self.on_tracks, 10)
        self.create_subscription(
            String, '/patrol/status', self.on_patrol_status, 10)
        self.create_subscription(
            DiagnosticArray, '/safety/state', self.on_safety_state, 10)

    def on_world_poses(self, message):
        self.world_poses = {
            transform.child_frame_id: (
                transform.transform.translation.x,
                transform.transform.translation.y,
                yaw(transform.transform.rotation),
            )
            for transform in message.transforms
        }
        self.world_pose_receipt = self.get_clock().now()
        self.update_collision_state()

    def on_odom(self, message):
        self.odom_pose = (
            message.pose.pose.position.x,
            message.pose.pose.position.y,
            yaw(message.pose.pose.orientation),
        )

    def on_patrol_status(self, message):
        self.patrol_status = message.data

    def on_safety_state(self, message):
        for status in message.status:
            if status.name != 'semantic_safety':
                continue
            reason = next((
                item.value for item in status.values
                if item.key == 'reason'), status.message or 'unknown')
            self.safety_reason_counts[reason] = (
                self.safety_reason_counts.get(reason, 0) + 1)

    def update_collision_state(self):
        robot_world = next((
            pose for name, pose in self.world_poses.items()
            if name.startswith(self.robot_name_prefix)), None)
        if robot_world is None:
            return
        distances = [
            math.hypot(pose[0] - robot_world[0], pose[1] - robot_world[1])
            for name, pose in self.world_poses.items()
            if name.startswith(self.person_name_prefix)
        ]
        if not distances:
            self.in_collision = False
            return
        nearest = min(distances)
        self.minimum_person_distance = min(
            self.minimum_person_distance, nearest)
        colliding = nearest <= self.collision_distance
        # 한 번의 접촉이 여러 frame에서 관측되어도 collision 한 건으로 센다.
        if colliding and not self.in_collision:
            self.collision_count += 1
        self.in_collision = colliding

    def on_tracks(self, message):
        latency = (self.get_clock().now()
                   - Time.from_msg(message.header.stamp)).nanoseconds * 1e-9
        if latency >= 0.0:
            self.tracking_latencies.append(latency)
        if self.odom_pose is None or self.world_pose_receipt is None:
            return
        age = (self.get_clock().now()
               - self.world_pose_receipt).nanoseconds * 1e-9
        if age > self.max_ground_truth_age:
            return
        robot_world = next((
            pose for name, pose in self.world_poses.items()
            if name.startswith(self.robot_name_prefix)), None)
        if robot_world is None:
            return
        ground_truth = {
            name: self.world_to_odom(pose, robot_world)
            for name, pose in self.world_poses.items()
            if name.startswith(self.person_name_prefix)
        }
        predictions = {
            person.track_id: (person.position.x, person.position.y)
            for person in message.persons
        }
        self.metrics.update(ground_truth, predictions)

    def world_to_odom(self, pose, robot_world):
        world_x, world_y, _ = pose
        robot_world_x, robot_world_y, robot_world_yaw = robot_world
        robot_odom_x, robot_odom_y, robot_odom_yaw = self.odom_pose
        relative_x = world_x - robot_world_x
        relative_y = world_y - robot_world_y
        angle = robot_odom_yaw - robot_world_yaw
        return (
            robot_odom_x + math.cos(angle) * relative_x
            - math.sin(angle) * relative_y,
            robot_odom_y + math.sin(angle) * relative_x
            + math.cos(angle) * relative_y,
        )

    def save(self):
        directory = os.path.dirname(self.output_path)
        if directory:
            os.makedirs(directory, exist_ok=True)
        temporary = self.output_path + '.tmp'
        summary = self.metrics.summary()
        ordered_latencies = sorted(self.tracking_latencies)
        p95_index = max(0, math.ceil(len(ordered_latencies) * 0.95) - 1)
        summary.update({
            'collision_count': self.collision_count,
            'collision_distance': self.collision_distance,
            'minimum_person_distance': (
                self.minimum_person_distance
                if math.isfinite(self.minimum_person_distance) else None),
            'patrol_completed': self.patrol_status == 'completed',
            'patrol_status': self.patrol_status,
            'safety_reason_counts': self.safety_reason_counts,
            'duration_seconds': (
                self.get_clock().now() - self.started_at).nanoseconds * 1e-9,
            'tracking_latency_p95_seconds': (
                ordered_latencies[p95_index] if ordered_latencies else None),
        })
        with open(temporary, 'w', encoding='utf-8') as output:
            json.dump(summary, output, indent=2, sort_keys=True)
            output.write('\n')
        os.replace(temporary, self.output_path)


def main(args=None):
    rclpy.init(args=args)
    node = TrackingEvaluatorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.save()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
