#!/usr/bin/env python3

# Copyright 2026 JungSeong
# Licensed under the Apache License, Version 2.0

import math
from dataclasses import dataclass
from typing import Optional
from typing import Sequence

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


@dataclass(frozen=True)
class SectorDistances:
    front: Optional[float]
    left: Optional[float]
    right: Optional[float]


def sector_distances(
        ranges: Sequence[float], angle_min: float, angle_increment: float,
        range_min: float, range_max: float, front_half_angle: float,
        side_angle: float) -> SectorDistances:
    minima = {'front': None, 'left': None, 'right': None}
    if not ranges or not math.isfinite(angle_increment) or angle_increment == 0.0:
        return SectorDistances(None, None, None)

    for index, raw_range in enumerate(ranges):
        distance = float(raw_range)
        if math.isinf(distance) and distance > 0.0:
            distance = range_max
        if (not math.isfinite(distance) or distance < range_min or
                distance > range_max):
            continue

        raw_angle = angle_min + index * angle_increment
        angle = math.atan2(math.sin(raw_angle), math.cos(raw_angle))
        sector = None
        if abs(angle) <= front_half_angle:
            sector = 'front'
        elif front_half_angle < angle <= side_angle:
            sector = 'left'
        elif -side_angle <= angle < -front_half_angle:
            sector = 'right'
        if sector is not None:
            current = minima[sector]
            minima[sector] = distance if current is None else min(current, distance)

    return SectorDistances(**minima)


class AvoidancePolicy:
    def __init__(self, stop_distance: float, clear_distance: float,
                 side_distance: float, linear_velocity: float,
                 angular_velocity: float, preferred_turn_direction='auto'):
        self.stop_distance = stop_distance
        self.clear_distance = clear_distance
        self.side_distance = side_distance
        self.linear_velocity = linear_velocity
        self.angular_velocity = angular_velocity
        self.preferred_turn_direction = preferred_turn_direction
        self.avoiding = False
        self.turn_direction = None

    def choose_turn_direction(self, distances: SectorDistances):
        if self.preferred_turn_direction != 'auto':
            return self.preferred_turn_direction
        return 'left' if distances.left >= distances.right else 'right'

    def command(self, distances: SectorDistances):
        if any(value is None for value in (
                distances.front, distances.left, distances.right)):
            self.avoiding = True
            return 0.0, 0.0, 'invalid_scan'

        if distances.front <= self.stop_distance and not self.avoiding:
            self.avoiding = True
            self.turn_direction = self.choose_turn_direction(distances)
        elif self.avoiding and distances.front >= self.clear_distance:
            self.avoiding = False
            self.turn_direction = None

        if not self.avoiding:
            return self.linear_velocity, 0.0, 'forward'

        left_blocked = distances.left <= self.side_distance
        right_blocked = distances.right <= self.side_distance
        if left_blocked and right_blocked:
            return 0.0, 0.0, 'blocked'
        preferred_side_blocked = (
            (self.turn_direction == 'left' and left_blocked) or
            (self.turn_direction == 'right' and right_blocked))
        if (self.preferred_turn_direction == self.turn_direction and
                preferred_side_blocked):
            return 0.0, 0.0, 'blocked'
        if self.turn_direction == 'left' and left_blocked:
            self.turn_direction = 'right'
        elif self.turn_direction == 'right' and right_blocked:
            self.turn_direction = 'left'
        elif self.turn_direction is None:
            self.turn_direction = self.choose_turn_direction(distances)
        if self.turn_direction == 'left':
            return 0.0, self.angular_velocity, 'turn_left'
        return 0.0, -self.angular_velocity, 'turn_right'


class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')
        self.linear_velocity = self.declare_parameter(
            'linear_velocity', 0.15).value
        self.angular_velocity = self.declare_parameter(
            'angular_velocity', 0.8).value
        self.stop_distance = self.declare_parameter(
            'stop_distance', 0.45).value
        self.clear_distance = self.declare_parameter(
            'clear_distance', 0.55).value
        self.side_distance = self.declare_parameter(
            'side_distance', 0.40).value
        self.scan_timeout = self.declare_parameter(
            'scan_timeout', 0.5).value
        self.front_half_angle = math.radians(self.declare_parameter(
            'front_half_angle_deg', 15.0).value)
        self.side_angle = math.radians(self.declare_parameter(
            'side_angle_deg', 60.0).value)
        self.preferred_turn_direction = self.declare_parameter(
            'preferred_turn_direction', 'auto').value
        self._validate_parameters()

        self.policy = AvoidancePolicy(
            self.stop_distance, self.clear_distance, self.side_distance,
            self.linear_velocity, self.angular_velocity,
            self.preferred_turn_direction)
        self.distances = SectorDistances(None, None, None)
        self.last_scan_time = None
        self.last_mode = None
        self.publisher = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        self.subscription = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, qos_profile_sensor_data)
        self.timer = self.create_timer(0.05, self.update)

    def _validate_parameters(self):
        valid = (
            self.linear_velocity >= 0.0 and self.angular_velocity > 0.0 and
            self.stop_distance > 0.0 and
            self.clear_distance > self.stop_distance and
            self.side_distance > 0.0 and self.scan_timeout > 0.0 and
            self.front_half_angle > 0.0 and
            self.front_half_angle < self.side_angle <= math.pi and
            self.preferred_turn_direction in {'auto', 'left', 'right'})
        if not valid:
            raise ValueError('invalid obstacle avoidance parameters')

    def scan_callback(self, message: LaserScan):
        self.distances = sector_distances(
            message.ranges, message.angle_min, message.angle_increment,
            message.range_min, message.range_max, self.front_half_angle,
            self.side_angle)
        self.last_scan_time = self.get_clock().now()

    def publish(self, linear: float, angular: float):
        command = TwistStamped()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = 'base_link'
        command.twist.linear.x = linear
        command.twist.angular.z = angular
        self.publisher.publish(command)

    def update(self):
        timed_out = self.last_scan_time is None
        if self.last_scan_time is not None:
            age = (self.get_clock().now() - self.last_scan_time).nanoseconds / 1e9
            timed_out = age > self.scan_timeout
        if timed_out:
            self.policy.avoiding = True
            self.publish(0.0, 0.0)
            mode = 'scan_timeout'
        else:
            linear, angular, mode = self.policy.command(self.distances)
            self.publish(linear, angular)
        if mode != self.last_mode:
            self.get_logger().info(f'avoidance mode: {mode}')
            self.last_mode = mode


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
