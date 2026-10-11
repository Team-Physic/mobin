#!/usr/bin/env python3

# Copyright 2026 JungSeong
# Licensed under the Apache License, Version 2.0

# Runtime trial의 Camera–LiDAR association과 safety command를 발표용 artifact로 기록한다.
# Detection·fusion·annotated image timestamp, mux/safe command와 safety diagnostic을 ROS clock으로 수집한다.
# 조건을 만족한 association PNG, CPA·stale 분리 그래프와 event·sample·evidence JSON을 저장한다.
# 기존 output directory 생성은 허용하지만 같은 이름의 파일 교체 정책은 호출 script가 관리한다.

import json
import os

import cv2
from cv_bridge import CvBridge
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import TwistStamped
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from vision_msgs.msg import Detection3DArray


CPA_REASONS = {'person_cpa_danger'}
STALE_REASONS = {'scan_stale', 'odom_stale', 'tracking_stale_speed_limit'}
DANGER_REASONS = CPA_REASONS | {'lidar_emergency'}


def select_event_window(events, end_time, trigger_reasons):
    """첫 대상 event 2초 전부터 safe 복귀 2초 후까지의 구간을 고른다."""
    trigger = next((event['time'] for event in events
                    if event['reason'] in trigger_reasons), None)
    if trigger is None:
        return 0.0, end_time, False
    safe = next((event['time'] for event in events
                 if event['time'] > trigger and event['reason'] == 'safe'), None)
    return max(0.0, trigger - 2.0), min(end_time, (safe or end_time) + 2.0), True


def bbox_is_mostly_visible(bbox, image_width, image_height, minimum_ratio=0.8):
    """bbox 면적 80% 이상이 image 내부이고 경계에서 잘리지 않았는지 확인한다."""
    center_x, center_y, width, height = bbox
    if width <= 0.0 or height <= 0.0:
        return False
    x_min = center_x - width * 0.5
    y_min = center_y - height * 0.5
    x_max = center_x + width * 0.5
    y_max = center_y + height * 0.5
    visible_width = max(0.0, min(x_max, image_width) - max(x_min, 0.0))
    visible_height = max(0.0, min(y_max, image_height) - max(y_min, 0.0))
    visible_ratio = visible_width * visible_height / (width * height)
    # Ultralytics는 bbox를 image 경계에서 clip하므로 경계 접촉 자체도 잘림으로 처리한다.
    touches_edge = (
        x_min <= 1.0 or y_min <= 1.0
        or x_max >= image_width - 1.0
        or y_max >= image_height - 1.0)
    return visible_ratio >= minimum_ratio and not touches_edge


def has_visible_association(
        detection_boxes, successful_associations, stamp,
        image_width, image_height):
    """같은 frame-local instance가 50 ms 이내 fusion과 80% bbox 조건을 모두 만족하는지 판정한다."""
    return any(
        abs(stamp - fused_stamp) <= 50_000_000
        and any(
            instance_id in fused_ids
            and bbox_is_mostly_visible(box, image_width, image_height)
            for instance_id, box in detection_boxes)
        for fused_stamp, fused_ids in successful_associations.items())


class TrialEvidenceRecorder(Node):
    def __init__(self):
        super().__init__('trial_evidence_recorder')
        self.output_dir = self.declare_parameter(
            'output_dir', '/ws/trials/current').value
        self.duration = float(self.declare_parameter(
            'duration_seconds', 60.0).value)
        if self.duration <= 0.0:
            raise ValueError('duration_seconds must be positive')
        os.makedirs(self.output_dir, exist_ok=True)
        self.bridge = CvBridge()
        self.started_at = None
        self.finished = False
        self.commands = {'mux': [], 'safe': []}
        self.safety_events = []
        self.successful_associations = {}
        self.association_rejection_counts = {}
        self.association_image_saved = False
        self.detection_boxes = {}
        self.annotated_images = {}
        self.person_detection_frames = 0
        self.fused_detection_frames = 0
        self.visible_association_candidates = 0
        self.create_subscription(
            TwistStamped, '/cmd_vel_mux',
            lambda message: self.on_command('mux', message), 10)
        self.create_subscription(
            TwistStamped, '/cmd_vel_safe',
            lambda message: self.on_command('safe', message), 10)
        self.create_subscription(
            DiagnosticArray, '/safety/state', self.on_safety, 10)
        self.create_subscription(
            DiagnosticArray, '/fusion/diagnostics',
            self.on_fusion_diagnostics, 10)
        self.create_subscription(
            Detection2DArray, '/detections', self.on_detections,
            qos_profile_sensor_data)
        self.create_subscription(
            Detection3DArray, '/fusion/detections_3d',
            self.on_fused_detections, 10)
        self.create_subscription(
            Image, '/camera/detections_annotated',
            self.on_annotated_image, qos_profile_sensor_data)
        self.create_timer(0.2, self.check_duration)

    def elapsed(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.started_at is None and now > 0.0:
            self.started_at = now
        return max(0.0, now - (self.started_at or now))

    def on_command(self, source, message):
        self.commands[source].append({
            'time': self.elapsed(),
            'linear_x': message.twist.linear.x,
            'angular_z': message.twist.angular.z,
        })

    def on_safety(self, message):
        for status in message.status:
            if status.name != 'semantic_safety':
                continue
            values = {item.key: item.value for item in status.values}
            event = {
                'time': self.elapsed(),
                'reason': values.get('reason', status.message or 'unknown'),
            }
            for key in (
                    'selected_track_id', 't_cpa', 'd_cpa', 'd_safe',
                    'danger_t_cpa', 'scan_age', 'odom_age', 'tracking_age',
                    'scan_timeout', 'odom_timeout', 'tracking_timeout'):
                try:
                    event[key] = float(values[key])
                except (KeyError, ValueError):
                    event[key] = None
            self.safety_events.append(event)

    def on_fused_detections(self, message):
        if message.detections:
            self.fused_detection_frames += 1
            stamp = (
                message.header.stamp.sec * 1_000_000_000
                + message.header.stamp.nanosec)
            self.successful_associations[stamp] = {
                detection.id for detection in message.detections}
            self.try_save_association()

    def on_fusion_diagnostics(self, message):
        for status in message.status:
            if status.name != 'lidar_bbox_association/association':
                continue
            self.association_rejection_counts = {
                item.key: int(item.value) for item in status.values}

    def on_detections(self, message):
        stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        self.detection_boxes[stamp] = [
            (detection.id, (
                detection.bbox.center.position.x,
                detection.bbox.center.position.y,
                detection.bbox.size_x, detection.bbox.size_y))
            for detection in message.detections
            if detection.results and detection.results[0].hypothesis.class_id == 'person'
        ]
        if self.detection_boxes[stamp]:
            self.person_detection_frames += 1
        while len(self.detection_boxes) > 60:
            self.detection_boxes.pop(next(iter(self.detection_boxes)))
        self.try_save_association()

    def on_annotated_image(self, message):
        if self.association_image_saved:
            return
        stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        self.annotated_images[stamp] = message
        while len(self.annotated_images) > 10:
            self.annotated_images.pop(next(iter(self.annotated_images)))
        self.try_save_association()

    def try_save_association(self):
        if self.association_image_saved:
            return
        # 같은 frame-local instance가 image 내부 80% 이상이고 3D fusion에도 남을 때만 저장한다.
        candidate = next((
            (stamp, message)
            for stamp, message in self.annotated_images.items()
            if stamp in self.detection_boxes
            and has_visible_association(
                self.detection_boxes[stamp], self.successful_associations,
                stamp, message.width, message.height)
        ), None)
        if candidate is None:
            return
        _, message = candidate
        self.visible_association_candidates += 1
        image = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
        self.association_image_saved = bool(cv2.imwrite(
            os.path.join(self.output_dir, 'association.png'), image))
        if self.association_image_saved:
            self.annotated_images.clear()

    def check_duration(self):
        if self.started_at is not None and self.elapsed() >= self.duration:
            self.save()
            rclpy.shutdown()

    def save_json(self, name, value):
        path = os.path.join(self.output_dir, name)
        temporary = path + '.tmp'
        with open(temporary, 'w', encoding='utf-8') as output:
            json.dump(value, output, indent=2, sort_keys=True)
            output.write('\n')
        os.replace(temporary, path)

    @staticmethod
    def plot_points(samples, key, start, end, bounds, value_range):
        left, top, right, bottom = bounds
        value_min, value_max = value_range
        selected = [
            sample for sample in samples
            if start <= sample['time'] <= end
            and sample.get(key) is not None
            and np.isfinite(sample[key])]
        if not selected:
            return np.empty((0, 1, 2), dtype=np.int32)
        duration = max(end - start, 1e-6)
        span = max(value_max - value_min, 1e-6)
        return np.array([[
            int(left + (sample['time'] - start) / duration * (right - left)),
            int(bottom - (
                min(max(sample[key], value_min), value_max) - value_min
            ) / span * (bottom - top)),
        ] for sample in selected], dtype=np.int32).reshape((-1, 1, 2))

    @staticmethod
    def panel(image, title, bounds):
        left, top, right, bottom = bounds
        cv2.rectangle(image, (left, top), (right, bottom), (210, 210, 210), 1)
        cv2.putText(image, title, (left, top - 14), cv2.FONT_HERSHEY_SIMPLEX,
                    0.58, (20, 20, 20), 2, cv2.LINE_AA)

    def draw_series(self, image, samples, key, start, end, bounds,
                    value_range, color):
        points = self.plot_points(
            samples, key, start, end, bounds, value_range)
        if len(points) > 1:
            cv2.polylines(image, [points], False, color, 2, cv2.LINE_AA)

    @staticmethod
    def draw_state_transitions(image, events, start, end, reasons):
        """연속한 같은 reason은 하나의 전이로 압축해 정지 원인을 표시한다."""
        previous = None
        duration = max(end - start, 1e-6)
        for event in events:
            reason = event['reason']
            if reason == previous or reason not in reasons:
                previous = reason
                continue
            previous = reason
            if not start <= event['time'] <= end:
                continue
            x = int(110 + (event['time'] - start) / duration * 1240)
            color = (30, 150, 30) if reason == 'safe' else (20, 20, 220)
            cv2.line(image, (x, 60), (x, 820), color, 1, cv2.LINE_AA)
            cv2.putText(image, reason, (min(x + 4, 1120), 52),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, color, 1, cv2.LINE_AA)

    @staticmethod
    def finish_plot(image, path, start, end):
        cv2.putText(image, f'{start:.1f}s', (110, 875),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (20, 20, 20), 1, cv2.LINE_AA)
        cv2.putText(image, f'{end:.1f}s', (1305, 875),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (20, 20, 20), 1, cv2.LINE_AA)
        return bool(cv2.imwrite(path, image))

    def save_cpa_plot(self, end_time):
        start, end, found = select_event_window(
            self.safety_events, end_time, CPA_REASONS)
        result = {'start': start, 'end': end, 'event_found': found, 'saved': False}
        if not found:
            return result
        image = np.full((900, 1400, 3), 255, dtype=np.uint8)
        distance_bounds = (110, 80, 1350, 280)
        time_bounds = (110, 350, 1350, 550)
        command_bounds = (110, 620, 1350, 820)
        self.panel(image, 'CPA distance: d_cpa / d_safe (m)', distance_bounds)
        self.panel(image, 'Time to CPA / danger threshold (s)', time_bounds)
        self.panel(image, 'Linear command: mux / safe (m/s)', command_bounds)
        self.draw_state_transitions(
            image, self.safety_events, start, end,
            CPA_REASONS | {'stable_safe_hold', 'safe'})
        self.draw_series(image, self.safety_events, 'd_cpa', start, end,
                         distance_bounds, (0.0, 2.0), (0, 90, 230))
        self.draw_series(image, self.safety_events, 'd_safe', start, end,
                         distance_bounds, (0.0, 2.0), (180, 80, 180))
        self.draw_series(image, self.safety_events, 't_cpa', start, end,
                         time_bounds, (0.0, 6.0), (0, 90, 230))
        self.draw_series(image, self.safety_events, 'danger_t_cpa', start, end,
                         time_bounds, (0.0, 6.0), (180, 80, 180))
        self.draw_series(image, self.commands['mux'], 'linear_x', start, end,
                         command_bounds, (-0.25, 0.25), (220, 80, 20))
        self.draw_series(image, self.commands['safe'], 'linear_x', start, end,
                         command_bounds, (-0.25, 0.25), (30, 150, 30))
        result['saved'] = self.finish_plot(
            image, os.path.join(self.output_dir, 'cpa-stop.png'), start, end)
        return result

    def save_stale_plot(self, end_time):
        start, end, found = select_event_window(
            self.safety_events, end_time, STALE_REASONS)
        result = {'start': start, 'end': end, 'event_found': found, 'saved': False}
        if not found:
            return result
        image = np.full((900, 1400, 3), 255, dtype=np.uint8)
        scan_bounds = (110, 80, 1350, 280)
        tracking_bounds = (110, 350, 1350, 550)
        command_bounds = (110, 620, 1350, 820)
        self.panel(image, 'Scan age / timeout (s)', scan_bounds)
        self.panel(image, 'Odom and tracking age / timeout (s)', tracking_bounds)
        self.panel(image, 'Linear command: mux / safe (m/s)', command_bounds)
        self.draw_state_transitions(
            image, self.safety_events, start, end, STALE_REASONS | {'safe'})
        self.draw_series(image, self.safety_events, 'scan_age', start, end,
                         scan_bounds, (0.0, 1.0), (0, 90, 230))
        self.draw_series(image, self.safety_events, 'scan_timeout', start, end,
                         scan_bounds, (0.0, 1.0), (180, 80, 180))
        self.draw_series(image, self.safety_events, 'tracking_age', start, end,
                         tracking_bounds, (0.0, 1.0), (0, 90, 230))
        self.draw_series(image, self.safety_events, 'tracking_timeout', start, end,
                         tracking_bounds, (0.0, 1.0), (180, 80, 180))
        self.draw_series(image, self.safety_events, 'odom_age', start, end,
                         tracking_bounds, (0.0, 1.0), (220, 120, 20))
        self.draw_series(image, self.safety_events, 'odom_timeout', start, end,
                         tracking_bounds, (0.0, 1.0), (40, 170, 170))
        self.draw_series(image, self.commands['mux'], 'linear_x', start, end,
                         command_bounds, (-0.25, 0.25), (220, 80, 20))
        self.draw_series(image, self.commands['safe'], 'linear_x', start, end,
                         command_bounds, (-0.25, 0.25), (30, 150, 30))
        result['saved'] = self.finish_plot(
            image, os.path.join(self.output_dir, 'stale-failsafe.png'), start, end)
        return result

    def save(self):
        if self.finished:
            return
        self.finished = True
        end_time = self.elapsed()
        cpa_window = self.save_cpa_plot(end_time)
        stale_window = self.save_stale_plot(end_time)
        reasons = {event['reason'] for event in self.safety_events}
        self.save_json('command-samples.json', self.commands)
        self.save_json('safety-events.json', self.safety_events)
        self.save_json('evidence.json', {
            'duration_seconds': end_time,
            'association_image_saved': self.association_image_saved,
            'cpa_window': cpa_window,
            'stale_window': stale_window,
            'person_cpa_danger_seen': 'person_cpa_danger' in reasons,
            'lidar_emergency_seen': 'lidar_emergency' in reasons,
            'stable_safe_hold_seen': 'stable_safe_hold' in reasons,
            'person_detection_frames': self.person_detection_frames,
            'fused_detection_frames': self.fused_detection_frames,
            'visible_association_candidates': self.visible_association_candidates,
            'association_outcomes': self.association_rejection_counts,
        })


def main(args=None):
    rclpy.init(args=args)
    node = TrialEvidenceRecorder()
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
