#!/usr/bin/env python3

# 실행 중인 ROS 2 TF tree에서 LiDAR와 Camera의 simulation ground-truth transform을 읽는다.
# 입력은 새 JSON 경로이며 base_link, base_scan, camera frame 사이 transform을 최대 10초 기다린다.
# 결과는 calibration 추정값과 비교할 translation·quaternion JSON이고 기존 파일은 덮어쓰지 않는다.
# Simulation과 robot_state_publisher가 먼저 실행되어 TF를 발행하고 있어야 한다.

import argparse
import json
from pathlib import Path

import rclpy
from rclpy.duration import Duration
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


def values(transform):
    translation = transform.translation
    rotation = transform.rotation
    return [
        translation.x,
        translation.y,
        translation.z,
        rotation.x,
        rotation.y,
        rotation.z,
        rotation.w,
    ]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('output')
    args = parser.parse_args()
    output = Path(args.output)
    if output.exists():
        raise FileExistsError(f'refusing to overwrite ground truth: {output}')

    rclpy.init()
    node = rclpy.create_node('capture_calibration_ground_truth')
    buffer = Buffer()
    listener = TransformListener(buffer, node, spin_thread=True)

    try:
        def lookup(target, source):
            # source point를 target frame으로 변환하는 transform을 반환한다.
            stamped = buffer.lookup_transform(
                target,
                source,
                Time(),
                timeout=Duration(seconds=10.0),
            )
            return values(stamped.transform)

        result = {
            'frames': {
                'base': 'base_link',
                'lidar': 'base_scan',
                'camera_link': 'camera_link',
                'camera_optical': 'camera_rgb_optical_frame',
            },
            'T_lidar_camera': lookup(
                'base_scan', 'camera_rgb_optical_frame'),
            'T_base_lidar': lookup('base_link', 'base_scan'),
            'T_camera_link_camera_optical': lookup(
                'camera_link', 'camera_rgb_optical_frame'),
        }

        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(result, indent=2) + '\n')
        print(output)
        print(json.dumps(result, indent=2))
    finally:
        del listener
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
