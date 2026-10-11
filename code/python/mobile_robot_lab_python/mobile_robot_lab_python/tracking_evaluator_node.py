#!/usr/bin/env python3

# Copyright 2026 JungSeong
# Licensed under the Apache License, Version 2.0

# Gazebo 사람 ground truth와 `/tracking/persons`를 Camera–LiDAR 공통 ROI에서 비교한다.
# World pose와 odometry를 track timestamp ±50 ms로 정렬한 뒤 odom 평면의 MOT 지표를 누적한다.
# 종료 시 summary, frame별 association CSV와 ID switch JSON을 원자적으로 저장한다.
# Ground truth는 runtime tracker나 safety node에 전달하지 않으며 평가에만 사용한다.

from collections import defaultdict
from collections import deque
import csv
import json
import math
import os

from mobile_robot_lab_msgs.msg import TrackedPersonArray
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo
from tf2_ros import Buffer
from tf2_ros import TransformException
from tf2_ros import TransformListener
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String

from mobile_robot_lab_python.tracking_evaluation_roi import camera_visibility
from mobile_robot_lab_python.tracking_evaluation_roi import lidar_visibility
from mobile_robot_lab_python.tracking_evaluation_roi import nearest_sample
from mobile_robot_lab_python.tracking_evaluation_roi import transform_point
from mobile_robot_lab_python.tracking_metrics import TrackingMetrics


def yaw(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z
               + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y ** 2 + quaternion.z ** 2))


def stamp_nanoseconds(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def select_unnamed_models(poses, expected_robot, person_count, max_error=0.5):
    """이름이 사라진 Pose_V에서 예상 robot과 바로 뒤 actor pose를 고른다."""
    if not poses or person_count < 1:
        return None
    robot_index = min(
        range(len(poses)),
        key=lambda index: math.dist(poses[index][:2], expected_robot))
    if (math.dist(poses[robot_index][:2], expected_robot) > max_error
            or robot_index + person_count >= len(poses)):
        return None
    return poses[robot_index:robot_index + person_count + 1]


class TrackingEvaluatorNode(Node):
    def __init__(self):
        super().__init__('tracking_evaluator')
        self.output_path = self.declare_parameter(
            'output_path',
            '/ws/data/results/tracking_metrics.json').value
        self.duration_seconds = float(self.declare_parameter(
            'duration_seconds', 0.0).value)
        self.robot_name_prefix = self.declare_parameter(
            'robot_name_prefix', 'turtlebot3_').value
        self.person_name_prefix = self.declare_parameter(
            'person_name_prefix', 'avoidance_person_').value
        self.expected_person_count = int(self.declare_parameter(
            'expected_person_count', 0).value)
        self.robot_initial_world_x = float(self.declare_parameter(
            'robot_initial_world_x', -2.0).value)
        self.robot_initial_world_y = float(self.declare_parameter(
            'robot_initial_world_y', -0.5).value)
        self.robot_initial_world_yaw = float(self.declare_parameter(
            'robot_initial_world_yaw', 0.0).value)
        self.max_time_difference_nanoseconds = round(
            float(self.declare_parameter(
                'max_time_difference_seconds', 0.05).value) * 1e9)
        self.person_reference_height = float(self.declare_parameter(
            'person_reference_height', 0.9).value)
        self.camera_info_topic = self.declare_parameter(
            'camera_info_topic', '/camera/camera_info').value
        self.lidar_frame = self.declare_parameter(
            'lidar_frame', 'base_scan').value
        self.lidar_minimum_range = float(self.declare_parameter(
            'lidar_minimum_range', 0.1).value)
        self.lidar_maximum_range = float(self.declare_parameter(
            'lidar_maximum_range', 20.0).value)
        self.lidar_horizontal_minimum = float(self.declare_parameter(
            'lidar_horizontal_minimum', -math.pi).value)
        self.lidar_horizontal_maximum = float(self.declare_parameter(
            'lidar_horizontal_maximum', math.pi).value)
        self.lidar_vertical_minimum = float(self.declare_parameter(
            'lidar_vertical_minimum', -0.261799).value)
        self.lidar_vertical_maximum = float(self.declare_parameter(
            'lidar_vertical_maximum', 0.261799).value)
        self.metrics = TrackingMetrics(float(self.declare_parameter(
            'distance_gate', 0.75).value))
        self.world_poses = {}
        self.odom_pose = None
        self.world_samples = deque(maxlen=300)
        self.odom_samples = deque(maxlen=300)
        self.camera_info = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.started_at = self.get_clock().now()
        self.saved = False
        self.patrol_status = 'not_started'
        self.minimum_person_distance = math.inf
        self.collision_count = 0
        self.in_collision = False
        self.collision_distance = float(self.declare_parameter(
            'collision_distance', 0.65).value)
        self.safety_reason_counts = {}
        self.tracking_latencies = []
        self.timing_excluded_frames = 0
        self.roi_unavailable_frames = 0
        self.excluded_predictions = 0
        self.coverage = defaultdict(lambda: {
            'samples': 0, 'camera': 0, 'lidar': 0, 'common': 0})
        self.association_rows = []
        self.id_switch_events = []
        self.create_subscription(
            TFMessage, '/evaluation/world_poses', self.on_world_poses, 10)
        self.create_subscription(
            Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(
            CameraInfo, self.camera_info_topic, self.on_camera_info,
            qos_profile_sensor_data)
        self.create_subscription(
            TrackedPersonArray, '/tracking/persons', self.on_tracks, 10)
        self.create_subscription(
            String, '/patrol/status', self.on_patrol_status, 10)
        self.create_subscription(
            DiagnosticArray, '/safety/state', self.on_safety_state, 10)
        if self.duration_seconds > 0.0:
            self.create_timer(0.2, self.check_duration)

    def check_duration(self):
        if ((self.get_clock().now() - self.started_at).nanoseconds * 1e-9
                >= self.duration_seconds):
            self.save()
            rclpy.shutdown()

    def on_world_poses(self, message):
        named_poses = {
            transform.child_frame_id: (
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
                yaw(transform.transform.rotation),
            )
            for transform in message.transforms
            if transform.child_frame_id
        }
        if named_poses:
            self.world_poses = named_poses
        elif self.odom_pose is not None:
            odom_x, odom_y, _, _ = self.odom_pose
            initial_yaw = self.robot_initial_world_yaw
            expected_robot = (
                self.robot_initial_world_x
                + math.cos(initial_yaw) * odom_x
                - math.sin(initial_yaw) * odom_y,
                self.robot_initial_world_y
                + math.sin(initial_yaw) * odom_x
                + math.cos(initial_yaw) * odom_y,
            )
            poses = [(
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
                yaw(transform.transform.rotation),
            ) for transform in message.transforms]
            # Pose_V 이름은 bridge에서 사라진다. odom+spawn으로 robot을 찾고 뒤 actor만 쓴다.
            top_level = select_unnamed_models(
                poses, expected_robot, self.expected_person_count)
            if top_level is None:
                return
            self.world_poses = {
                self.robot_name_prefix + 'ground_truth': (
                    top_level[0]
                ),
                **{
                    self.person_name_prefix + str(index): pose
                    for index, pose in enumerate(top_level[1:], start=1)
                },
            }
        if not message.transforms:
            return
        sample_time = stamp_nanoseconds(message.transforms[0].header.stamp)
        if sample_time == 0.0:
            sample_time = self.get_clock().now().nanoseconds
        self.world_samples.append((sample_time, dict(self.world_poses)))
        self.update_collision_state()

    def on_odom(self, message):
        self.odom_pose = (
            message.pose.pose.position.x,
            message.pose.pose.position.y,
            message.pose.pose.position.z,
            yaw(message.pose.pose.orientation),
        )
        self.odom_samples.append((
            stamp_nanoseconds(message.header.stamp), self.odom_pose))

    def on_camera_info(self, message):
        self.camera_info = message

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
        """Track 시각에 맞춘 GT·odom과 공통 sensor ROI 안의 prediction만 MOT 평가한다."""
        latency = (self.get_clock().now()
                   - Time.from_msg(message.header.stamp)).nanoseconds * 1e-9
        if latency >= 0.0:
            self.tracking_latencies.append(latency)
        if self.camera_info is None:
            self.roi_unavailable_frames += 1
            return
        track_time = stamp_nanoseconds(message.header.stamp)
        world_sample = nearest_sample(
            self.world_samples, track_time,
            self.max_time_difference_nanoseconds)
        odom_sample = nearest_sample(
            self.odom_samples, track_time,
            self.max_time_difference_nanoseconds)
        # GT와 odom 중 하나라도 허용 시간차 밖이면 검출 실패가 아닌 시간 정렬 실패다.
        if world_sample is None or odom_sample is None:
            self.timing_excluded_frames += 1
            return
        world_poses = world_sample[1]
        odom_pose = odom_sample[1]
        robot_world = next((
            pose for name, pose in world_poses.items()
            if name.startswith(self.robot_name_prefix)), None)
        if robot_world is None:
            return
        try:
            lookup_time = Time.from_msg(message.header.stamp)
            camera_transform = self.tf_buffer.lookup_transform(
                self.camera_info.header.frame_id, 'odom', lookup_time,
                timeout=Duration(seconds=0.05))
            lidar_transform = self.tf_buffer.lookup_transform(
                self.lidar_frame, 'odom', lookup_time,
                timeout=Duration(seconds=0.05))
        except TransformException:
            self.roi_unavailable_frames += 1
            return

        all_ground_truth = {
            name: self.world_to_odom(
                pose, robot_world, odom_pose, self.person_reference_height)
            for name, pose in world_poses.items()
            if name.startswith(self.person_name_prefix)
        }
        all_predictions = {
            person.track_id: (
                person.position.x, person.position.y,
                self.person_reference_height)
            for person in message.persons
        }
        ground_truth = self.visible_points(
            all_ground_truth, camera_transform, lidar_transform,
            update_coverage=True)
        predictions = self.visible_points(
            all_predictions, camera_transform, lidar_transform)
        self.excluded_predictions += len(all_predictions) - len(predictions)
        planar_ground_truth = {
            key: value[:2] for key, value in ground_truth.items()}
        planar_predictions = {
            key: value[:2] for key, value in predictions.items()}
        if not planar_ground_truth and not planar_predictions:
            return
        previous = dict(self.metrics.previous_prediction)
        matches = self.metrics.update(
            planar_ground_truth, planar_predictions)
        self.record_associations(
            track_time * 1e-9, planar_ground_truth, planar_predictions,
            matches, previous)

    def visible_points(
            self, points, camera_transform, lidar_transform,
            update_coverage=False):
        """odom 3D point를 두 sensor frame으로 변환해 공통 ROI subset을 만든다."""
        visible = {}
        intrinsics = (
            self.camera_info.k[0], self.camera_info.k[4],
            self.camera_info.k[2], self.camera_info.k[5])
        for identifier, point in points.items():
            camera_visible = camera_visibility(
                transform_point(point, camera_transform), intrinsics,
                self.camera_info.width, self.camera_info.height)
            lidar_visible = lidar_visibility(
                transform_point(point, lidar_transform),
                self.lidar_minimum_range, self.lidar_maximum_range,
                self.lidar_horizontal_minimum,
                self.lidar_horizontal_maximum,
                self.lidar_vertical_minimum, self.lidar_vertical_maximum)
            if update_coverage:
                for key in ('__all__', str(identifier)):
                    self.coverage[key]['samples'] += 1
                    self.coverage[key]['camera'] += int(camera_visible)
                    self.coverage[key]['lidar'] += int(lidar_visible)
                    self.coverage[key]['common'] += int(
                        camera_visible and lidar_visible)
            if camera_visible and lidar_visible:
                visible[identifier] = point
        return visible

    def record_associations(
            self, timestamp, ground_truth, predictions, matches, previous):
        """Frame의 TP·FP·FN 행과 같은 GT에 대한 Track ID 변경 사건을 보존한다."""
        matched_ground_truth = {match[0] for match in matches}
        matched_predictions = {match[1] for match in matches}
        for ground_truth_id, prediction_id in matches:
            distance = math.dist(
                ground_truth[ground_truth_id], predictions[prediction_id])
            self.association_rows.append({
                'timestamp': timestamp, 'outcome': 'TP',
                'ground_truth_id': ground_truth_id,
                'track_id': prediction_id, 'distance_m': distance})
            previous_id = previous.get(ground_truth_id)
            if previous_id is not None and previous_id != prediction_id:
                self.id_switch_events.append({
                    'timestamp': timestamp,
                    'ground_truth_id': ground_truth_id,
                    'previous_track_id': previous_id,
                    'new_track_id': prediction_id,
                })
        for ground_truth_id in ground_truth.keys() - matched_ground_truth:
            self.association_rows.append({
                'timestamp': timestamp, 'outcome': 'FN',
                'ground_truth_id': ground_truth_id,
                'track_id': '', 'distance_m': ''})
        for prediction_id in predictions.keys() - matched_predictions:
            self.association_rows.append({
                'timestamp': timestamp, 'outcome': 'FP',
                'ground_truth_id': '',
                'track_id': prediction_id, 'distance_m': ''})

    def world_to_odom(self, pose, robot_world, odom_pose, height=0.0):
        """같은 시각의 world robot pose와 odom pose로 사람 torso를 odom으로 옮긴다."""
        world_x, world_y, world_z, _ = pose
        robot_world_x, robot_world_y, robot_world_z, robot_world_yaw = robot_world
        robot_odom_x, robot_odom_y, robot_odom_z, robot_odom_yaw = odom_pose
        relative_x = world_x - robot_world_x
        relative_y = world_y - robot_world_y
        angle = robot_odom_yaw - robot_world_yaw
        return (
            robot_odom_x + math.cos(angle) * relative_x
            - math.sin(angle) * relative_y,
            robot_odom_y + math.sin(angle) * relative_x
            + math.cos(angle) * relative_y,
            robot_odom_z + world_z + height - robot_world_z,
        )

    def save(self):
        if self.saved:
            return
        self.saved = True
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
            'timing_excluded_frames': self.timing_excluded_frames,
            'roi_unavailable_frames': self.roi_unavailable_frames,
            'roi_excluded_predictions': self.excluded_predictions,
            'sensor_coverage': self.coverage_summary(),
        })
        with open(temporary, 'w', encoding='utf-8') as output:
            json.dump(summary, output, indent=2, sort_keys=True)
            output.write('\n')
        os.replace(temporary, self.output_path)
        self.save_association_rows(directory)
        self.save_id_switch_events(directory)

    def coverage_summary(self):
        """전체와 GT 사람별 Camera·LiDAR·공통 ROI sample 비율을 반환한다."""
        result = {}
        for identifier, counts in self.coverage.items():
            total = counts['samples']
            result[identifier] = {
                **counts,
                'camera_ratio': counts['camera'] / total if total else 0.0,
                'lidar_ratio': counts['lidar'] / total if total else 0.0,
                'common_ratio': counts['common'] / total if total else 0.0,
            }
        return result

    def save_association_rows(self, directory):
        path = os.path.join(directory or '.', 'roi-associations.csv')
        temporary = path + '.tmp'
        fields = [
            'timestamp', 'outcome', 'ground_truth_id',
            'track_id', 'distance_m']
        with open(temporary, 'w', encoding='utf-8', newline='') as output:
            writer = csv.DictWriter(output, fieldnames=fields)
            writer.writeheader()
            writer.writerows(self.association_rows)
        os.replace(temporary, path)

    def save_id_switch_events(self, directory):
        path = os.path.join(directory or '.', 'id-switch-events.json')
        temporary = path + '.tmp'
        with open(temporary, 'w', encoding='utf-8') as output:
            json.dump(self.id_switch_events, output, indent=2)
            output.write('\n')
        os.replace(temporary, path)


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
