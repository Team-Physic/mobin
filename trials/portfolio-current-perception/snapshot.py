#!/usr/bin/env python3
# 실제 ROS 출력 중 Camera·LiDAR·track timestamp가 정확히 같은 한 순간을 보존한다.
# 두 사람의 현재 측정과 confirmed track이 존재할 때만 raw/annotation PNG와 CDR 묶음을 저장한다.
# TF는 그 시각의 odom<-sensor 변환을 조회하며 track 상태·점·timestamp를 변경하지 않는다.
# 위치 인자: capture|replay|verify 필수, directory 필수(새 증거 경로 또는 기존 묶음).
# ROS_DOMAIN_ID: capture=87, replay=88을 실행자가 선택한다. capture 최대 wall 600초, 단위 ns·m.
import itertools
import json
from pathlib import Path
import sys
import time

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import serialize_message, deserialize_message
from rclpy.time import Time
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from vision_msgs.msg import Detection2DArray, Detection3DArray
from visualization_msgs.msg import Marker, MarkerArray
from mobile_robot_lab_msgs.msg import TrackedPersonArray
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener
from rosgraph_msgs.msg import Clock

TOPICS = {
    '/camera/image_raw': Image, '/camera/detections_annotated': Image,
    '/segmentation/instance_labels': Image, '/detections': Detection2DArray,
    '/calib/points': PointCloud2, '/fusion/detections_3d': Detection3DArray,
    '/fusion/ground_points': PointCloud2, '/fusion/non_ground_points': PointCloud2,
    '/fusion/associated_points': PointCloud2, '/fusion/ground_plane': Marker,
    '/tracking/persons': TrackedPersonArray, '/tracking/markers': MarkerArray,
}


def stamp(message):
    """MarkerArray를 포함한 원본 message의 관측 시각을 정수 ns로 반환한다."""
    header = message.markers[0].header if isinstance(message, MarkerArray) else message.header
    return header.stamp.sec * 10**9 + header.stamp.nanosec


def confirmed_pair(tracks, detections, timestamp):
    """해당 시각에 갱신된 confirmed 두 명을 서로 다른 3D 관측과 0.75m 이내로 대응시킨다."""
    people = [p for p in tracks.persons if p.state == p.CONFIRMED and
              abs(p.last_measurement_stamp.sec * 10**9 + p.last_measurement_stamp.nanosec - timestamp) <= 1]
    positions = [d.results[0].pose.pose.position for d in detections.detections if d.results]
    for pair in itertools.combinations(people, 2):
        for observed in itertools.permutations(positions, 2):
            if all((p.position.x-q.x)**2 + (p.position.y-q.y)**2 <= 0.75**2
                   for p, q in zip(pair, observed)):
                return [p.track_id for p in pair]
    return []


def capture(node, directory):
    """stamp가 완전히 일치하는 실제 출력과 해당 시각 TF를 고정 재생용으로 보존한다."""
    if directory.exists():
        raise SystemExit('capture directory exists; preserve previous evidence')
    caches = {topic: {} for topic in TOPICS}
    camera_info = []
    buffer = Buffer(cache_time=Duration(seconds=20))
    listener = TransformListener(buffer, node)
    subscriptions = []
    def receive(topic, message):
        cache = caches[topic]
        cache[stamp(message)] = message
        while len(cache) > 300:
            cache.pop(next(iter(cache)))
    for topic, cls in TOPICS.items():
        qos = qos_profile_sensor_data if topic in (
            '/segmentation/instance_labels', '/detections') else 10
        subscriptions.append(node.create_subscription(
            cls, topic, lambda message, topic=topic: receive(topic, message), qos))
    subscriptions.append(node.create_subscription(CameraInfo, '/camera/camera_info',
        lambda message: camera_info.__setitem__(slice(None), [message]), qos_profile_sensor_data))
    started = time.monotonic()
    progress_at = started
    checked = set()
    while time.monotonic() - started < 600:
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.monotonic() - progress_at >= 30:
            print(json.dumps({'waiting_seconds': round(time.monotonic()-started),
                'cached': {topic: len(cache) for topic, cache in caches.items()},
                'latest_stamps': {topic: next(reversed(cache), None) for topic, cache in caches.items()},
                'exact_common_frames': len(set.intersection(*(set(cache) for cache in caches.values()))),
                'camera_info_received': bool(camera_info),
                'two_confirmed_frames': sum(sum(p.state == p.CONFIRMED for p in message.persons) >= 2
                    for message in caches['/tracking/persons'].values())}), flush=True)
            progress_at = time.monotonic()
        common = set.intersection(*(set(cache) for cache in caches.values())) - checked
        for timestamp in sorted(common):
            bundle = {topic: cache[timestamp] for topic, cache in caches.items()}
            ids = confirmed_pair(bundle['/tracking/persons'], bundle['/fusion/detections_3d'], timestamp)
            if not ids or len(bundle['/detections'].detections) < 2:
                checked.add(timestamp)
                continue
            if not camera_info or bundle['/fusion/ground_plane'].action != Marker.ADD:
                continue
            frames = {'base_footprint', 'base_scan', camera_info[0].header.frame_id}
            try:
                transforms = [buffer.lookup_transform('odom', frame, Time(nanoseconds=timestamp))
                              for frame in sorted(frames)]
            except Exception:
                continue
            directory.mkdir(parents=True)
            bundle['/camera/camera_info'] = camera_info[0]
            bundle['/tf'] = TFMessage(transforms=transforms)
            clock = Clock()
            clock.clock = Time(nanoseconds=timestamp).to_msg()
            bundle['/clock'] = clock
            records = []
            for i, (topic, message) in enumerate(bundle.items()):
                name = f'{i:02d}.cdr'
                (directory/name).write_bytes(serialize_message(message))
                cls = type(message)
                records.append({'topic': topic, 'type': cls.__module__.split('.')[0]+'/msg/'+cls.__name__, 'file': name})
            bridge = CvBridge()
            for topic, name in [('/camera/image_raw', 'gazebo-camera.png'),
                                ('/camera/detections_annotated', 'camera-association.png')]:
                assert cv2.imwrite(str(directory/name), bridge.imgmsg_to_cv2(bundle[topic], 'bgr8'))
            metadata = {'timestamp_ns': timestamp, 'confirmed_ids': ids,
                'selection': 'exact header timestamp equality across all 12 sensor/fusion/tracking topics; two confirmed tracks updated at this timestamp, distinct 3D observations within 0.75m',
                'topics': records, 'wall_wait_seconds': time.monotonic()-started,
                'tf': 'odom <- each displayed sensor frame, looked up at selected timestamp; no timestamp rewriting',
                'camera_info': 'latest calibration, not a simultaneous exposure',
                'limits': 'selected example, not aggregate accuracy; annotation header follows detection timestamp; raw-image synchronizer pairing is checked separately'}
            (directory/'snapshot.json').write_text(json.dumps(metadata, indent=2))
            print(json.dumps(metadata), flush=True)
            return
    raise RuntimeError('no exact-time two-confirmed snapshot within 600 wall seconds')


def replay(node, directory):
    """원본 message를 변경 없이 반복 발행한다. 격리 domain의 RViz 표시용이며 추론·추적은 실행하지 않는다."""
    metadata = json.loads((directory/'snapshot.json').read_text())
    outputs = []
    for record in metadata['topics']:
        cls = get_message(record['type'])
        outputs.append((node.create_publisher(cls, record['topic'], 10),
            deserialize_message((directory/record['file']).read_bytes(), cls)))
    while rclpy.ok():
        for publisher, message in outputs:
            publisher.publish(message)
        rclpy.spin_once(node, timeout_sec=0.1)


def verify(node, directory):
    """보존 후에도 timestamp·실제 confirmed 상태·서로 다른 관측의 계약이 유지되는지 검사한다."""
    metadata = json.loads((directory/'snapshot.json').read_text())
    messages = {record['topic']: deserialize_message(
        (directory/record['file']).read_bytes(), get_message(record['type']))
        for record in metadata['topics']}
    timestamp = metadata['timestamp_ns']
    assert all(stamp(messages[topic]) == timestamp for topic in TOPICS)
    assert len(messages['/detections'].detections) >= 2
    assert confirmed_pair(messages['/tracking/persons'], messages['/fusion/detections_3d'], timestamp) == metadata['confirmed_ids']
    assert all(stamp(marker) == timestamp for marker in messages['/tracking/markers'].markers)
    assert all(stamp(transform) == timestamp for transform in messages['/tf'].transforms)
    for name in ('gazebo-camera.png', 'camera-association.png'):
        image = cv2.imread(str(directory/name))
        assert image is not None and image.shape[:2] == (
            messages['/camera/image_raw'].height, messages['/camera/image_raw'].width)
    print(f'verified: {len(TOPICS)} equal topic stamps, two confirmed IDs {metadata["confirmed_ids"]}, TF and PNG dimensions')


def main():
    if len(sys.argv) != 3 or sys.argv[1] not in ('capture', 'replay', 'verify'):
        raise SystemExit('usage: snapshot.py capture|replay|verify directory')
    rclpy.init(args=[])
    node = rclpy.create_node('portfolio_snapshot')
    try:
        {'capture': capture, 'replay': replay, 'verify': verify}[sys.argv[1]](node, Path(sys.argv[2]))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
