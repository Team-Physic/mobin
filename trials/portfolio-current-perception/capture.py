#!/usr/bin/env python3
# 실제 Gazebo의 Camera 원본·association 화면과 시간별 track·diagnostics를 보존한다.
# 영상은 ROS Image를 PNG로 저장하며 합성하거나 내용·검출 결과를 수정하지 않는다.
# Frame 선택은 5초 간격과 첫 부분 bbox다. track 좌표는 odom[m], 시간은 simulation[s]다.
# 인자 없음. 출력은 이 script 디렉터리이며 기존 capture.json이 있으면 덮어쓰지 않는다.
# ROS_DOMAIN_ID=87, GZ_PARTITION=mobin-portfolio-capture 환경의 실행과 함께 사용한다.
import json
from pathlib import Path
import time
import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from diagnostic_msgs.msg import DiagnosticArray
from vision_msgs.msg import Detection2DArray
from mobile_robot_lab_msgs.msg import TrackedPersonArray


def stamp(message):
    return message.header.stamp.sec + message.header.stamp.nanosec * 1e-9


def main():
    """최대 wall 240초 또는 첫 track 이후 simulation 65초 동안 관측 증거를 수집한다."""
    out = Path(__file__).parent
    if (out / 'capture.json').exists():
        raise SystemExit('capture.json exists; preserve the previous run')
    rclpy.init()
    node = rclpy.create_node('portfolio_capture')
    bridge = CvBridge()
    raw, annotated, boxes = {}, {}, {}
    tracks, diagnostics, saved = [], [], []
    started = time.monotonic()
    first_track = None
    last_saved = -100.0
    event_saved = set()
    def on_raw(message):
        raw[stamp(message)] = message
        while len(raw) > 80:
            raw.pop(next(iter(raw)))
        save_ready()
    def on_image(message):
        annotated[stamp(message)] = message
        while len(annotated) > 16:
            annotated.pop(next(iter(annotated)))
        save_ready()
    def on_boxes(message):
        boxes[stamp(message)] = [{
            'instance_id': d.id, 'bbox_xywh': [d.bbox.center.position.x,
            d.bbox.center.position.y, d.bbox.size_x, d.bbox.size_y],
        } for d in message.detections]
        while len(boxes) > 80:
            boxes.pop(next(iter(boxes)))
        save_ready()
    def on_tracks(message):
        nonlocal first_track
        t = stamp(message)
        if first_track is None:
            first_track = t
        tracks.append({'t': t, 'frame': message.header.frame_id, 'persons': [{
            'id': p.track_id, 'state': p.state, 'x': p.position.x, 'y': p.position.y,
            'vx': p.velocity.x, 'vy': p.velocity.y,
            'uncertainty_radius': p.uncertainty_radius,
        } for p in message.persons]})
    def on_diagnostics(message):
        for status in message.status:
            if status.name == 'lidar_bbox_association/association':
                diagnostics.append({'t': stamp(message), **{
                    item.key: int(item.value) for item in status.values}})
    def save_ready():
        nonlocal last_saved
        for t in list(annotated):
            if t not in raw or t not in boxes or not boxes[t]:
                continue
            image = annotated[t]
            partial = any(
                x-w/2 <= 1 or y-h/2 <= 1 or x+w/2 >= image.width-1 or y+h/2 >= image.height-1
                for d in boxes[t] for x,y,w,h in [d['bbox_xywh']])
            reasons = []
            if t - last_saved >= 5:
                reasons.append('periodic_with_person_detection')
            if partial and 'partial_bbox' not in event_saved:
                reasons.append('partial_bbox')
                event_saved.add('partial_bbox')
            if not reasons or len(saved) >= 16:
                continue
            index = len(saved)
            name = f'frame-{index:02d}'
            assert cv2.imwrite(str(out / f'{name}-camera.png'), bridge.imgmsg_to_cv2(raw[t], 'bgr8'))
            assert cv2.imwrite(str(out / f'{name}-association.png'), bridge.imgmsg_to_cv2(image, 'bgr8'))
            saved.append({'name': name, 't': t, 'reasons': reasons,
                          'width': image.width, 'height': image.height,
                          'detections': boxes[t], 'camera_frame': image.header.frame_id})
            print(f'saved {name} at {t:.2f}s; detections={len(boxes[t])}; {reasons}', flush=True)
            last_saved = t
            annotated.pop(t, None)
    subscriptions = [
        node.create_subscription(Image, '/camera/image_raw', on_raw, qos_profile_sensor_data),
        node.create_subscription(Image, '/camera/detections_annotated', on_image, 10),
        node.create_subscription(Detection2DArray, '/detections', on_boxes, qos_profile_sensor_data),
        node.create_subscription(TrackedPersonArray, '/tracking/persons', on_tracks, 10),
        node.create_subscription(DiagnosticArray, '/fusion/diagnostics', on_diagnostics, 10),
    ]
    try:
        while time.monotonic() - started < 240:
            rclpy.spin_once(node, timeout_sec=0.2)
            if first_track is not None and tracks[-1]['t'] - first_track >= 65:
                break
    finally:
        result = {'scope': 'actual Gazebo camera and ROS perception; no scene or detection editing',
                  'wall_duration_s': time.monotonic()-started, 'saved_frames': saved,
                  'tracks': tracks, 'diagnostics': diagnostics,
                  'selection': 'person detections every >=5 simulation seconds; first partial bbox; max16 pairs',
                  'limitations': 'selected screenshots do not measure overall accuracy; bbox clipping is not pixel-level occlusion ground truth'}
        (out / 'capture.json').write_text(json.dumps(result, indent=2))
        node.destroy_node()
        rclpy.shutdown()
    assert saved and tracks and diagnostics, 'incomplete capture; inspect logs'
    print(f'complete: {len(saved)} frame pairs, {len(tracks)} track messages', flush=True)


if __name__ == '__main__':
    main()
