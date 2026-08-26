#!/usr/bin/env python3
import argparse
import json
import math
from pathlib import Path


def normalize(q):
    n = math.sqrt(sum(v * v for v in q))
    if n == 0.0:
        raise ValueError('zero quaternion')
    return tuple(v / n for v in q)


def multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def conjugate(q):
    x, y, z, w = q
    return (-x, -y, -z, w)


def rotate(q, p):
    x, y, z, _ = multiply(multiply(q, (*p, 0.0)), conjugate(q))
    return (x, y, z)


def split(t):
    if len(t) != 7 or not all(math.isfinite(v) for v in t):
        raise ValueError(f'invalid transform: {t}')
    return tuple(t[:3]), normalize(t[3:])


def compose(a, b):
    at, aq = split(a)
    bt, bq = split(b)
    rbt = rotate(aq, bt)
    return [
        at[0] + rbt[0], at[1] + rbt[1], at[2] + rbt[2],
        *normalize(multiply(aq, bq)),
    ]


def inverse(t):
    xyz, q = split(t)
    qi = conjugate(q)
    ti = rotate(qi, (-xyz[0], -xyz[1], -xyz[2]))
    return [*ti, *qi]


def quaternion_to_rpy(q):
    x, y, z, w = normalize(q)
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    s = max(-1.0, min(1.0, 2 * (w * y - z * x)))
    pitch = math.asin(s)
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def load_result(path):
    data = json.loads(Path(path).read_text())
    return data['results']['T_lidar_camera']


def compare(estimate, truth):
    et, eq = split(estimate)
    gt, gq = split(truth)
    translation = math.sqrt(sum((a - b) ** 2 for a, b in zip(et, gt)))
    dot = abs(sum(a * b for a, b in zip(eq, gq)))
    rotation = math.degrees(2 * math.acos(max(-1.0, min(1.0, dot))))
    return translation, rotation


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('calib_json')
    parser.add_argument('ground_truth_json')
    parser.add_argument('--metrics')
    parser.add_argument('--camera-joint', action='store_true')
    args = parser.parse_args()

    estimate = load_result(args.calib_json)
    rig = json.loads(Path(args.ground_truth_json).read_text())
    translation, rotation = compare(estimate, rig['T_lidar_camera'])
    metrics = {
        'translation_error_m': translation,
        'rotation_error_deg': rotation,
        'pass': translation <= 0.05 and rotation <= 3.0,
    }
    print(json.dumps(metrics, indent=2))

    if args.metrics:
        Path(args.metrics).write_text(json.dumps(metrics, indent=2) + '\n')

    if args.camera_joint:
        base_camera_optical = compose(rig['T_base_lidar'], estimate)
        base_camera_link = compose(
            base_camera_optical,
            inverse(rig['T_camera_link_camera_optical']))
        xyz, quaternion = split(base_camera_link)
        rpy = quaternion_to_rpy(quaternion)
        print('camera_joint xyz:', ' '.join(f'{v:.9f}' for v in xyz))
        print('camera_joint rpy:', ' '.join(f'{v:.9f}' for v in rpy))

    raise SystemExit(0 if metrics['pass'] else 1)


if __name__ == '__main__':
    main()
