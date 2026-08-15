# PATCH-04: Extrinsic을 URDF에 적용하고 정량 검증

## 이 PATCH에서 만드는 것

calibration 결과의 방향을 확인하고 시뮬레이터 ground truth와 translation/rotation 오차를 계산한 뒤, 허용 오차 안에 있을 때만 추정값을 URDF의 `camera_joint`에 적용한다.

이 문서의 transform 표기는 다음 한 가지 규칙만 사용한다.

```text
p_A = T_A_B * p_B
```

calibration 결과 `T_lidar_camera`도 같은 규칙이다.

```text
p_lidar = T_lidar_camera * p_camera_optical
```

## 시작 조건

- PATCH-03의 `data/results/calib.json`이 있다.
- `results.T_lidar_camera`가 `[x, y, z, qx, qy, qz, qw]` 7개 값이다.
- 아직 추정 결과로 URDF를 수정하지 않았다.

```bash
cd /home/swlinux/Desktop/workspace/mobin
git diff -- forks/turtlebot3_simulations/turtlebot3_gazebo/urdf/turtlebot3_waffle_pi_3d.urdf
```

PATCH-01에서 추가한 `calib_lidar_joint` 외에 camera pose가 바뀌어 있다면 먼저 그 이유를 확인한다.

## 추가/수정할 파일

```text
mobile-robot-calibration-repo/
├── data/results/
│   ├── ground-truth.json
│   └── metrics.json
├── scripts/extrinsic_math.py
└── forks/turtlebot3_simulations/turtlebot3_gazebo/
    └── urdf/turtlebot3_waffle_pi_3d.urdf
```

Gazebo 실제 센서 pose가 있는 `model.sdf`는 이 PATCH에서 수정하지 않는다. SDF까지 추정값으로 바꾸면 비교 기준 자체가 움직인다.

## 1. ground truth를 기록한다

PATCH-01 기준 pose를 그대로 썼다면 알려진 transform은 다음과 같다.

```text
T_base_lidar:
  xyz = [0.000, 0.000, 0.180]
  quaternion = [0, 0, 0, 1]

T_base_camera_optical:
  xyz = [0.076, 0.000, 0.093]
  rpy = [-1.57, 0, -1.57]

T_camera_link_camera_optical:
  xyz = [0.003, 0.011, 0.009]
  rpy = [-1.57, 0, -1.57]
```

따라서 `T_lidar_camera = inverse(T_base_lidar) * T_base_camera_optical`이다.

simulation을 원래 URDF로 실행하고 TF에서도 확인한다.

```bash
ros2 run tf2_ros tf2_echo base_scan camera_rgb_optical_frame
```

PATCH-01의 기준값이면 translation은 대략 `[0.076, 0.000, -0.087]`이다. rotation은 TF 출력의 quaternion을 그대로 복사한다.

`data/results/ground-truth.json`을 만든다.

```json
{
  "T_lidar_camera": [
    0.076,
    0.0,
    -0.087,
    -0.49999984146591736,
    0.49960183664463337,
    -0.49999984146591736,
    0.5003981633553667
  ],
  "T_base_lidar": [0.0, 0.0, 0.180, 0.0, 0.0, 0.0, 1.0],
  "T_camera_link_camera_optical": [
    0.003,
    0.011,
    0.009,
    -0.49999984146591736,
    0.49960183664463337,
    -0.49999984146591736,
    0.5003981633553667
  ]
}
```

SDF/URDF pose를 PATCH-01의 예시와 다르게 정했다면 위 숫자를 그대로 쓰지 않는다. `tf2_echo`와 실제 URDF chain에서 다시 기록한다.

## 2. 비교와 변환을 한 스크립트에 둔다

새 수학 라이브러리를 설치하지 않고 Python 표준 라이브러리만 쓴다.

`scripts/extrinsic_math.py`:

```python
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
```

실행 권한을 준다.

```bash
chmod +x scripts/extrinsic_math.py
```

## 3. 수학 스크립트의 최소 self-check를 한다

identity transform을 합성했을 때 입력이 그대로여야 한다.

```bash
python3 - <<'PY'
import importlib.util

spec = importlib.util.spec_from_file_location('extrinsic_math', 'scripts/extrinsic_math.py')
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)

t = [1, 2, 3, 0, 0, 0, 1]
i = [0, 0, 0, 0, 0, 0, 1]
assert all(abs(a - b) < 1e-12 for a, b in zip(m.compose(t, i), t))
assert all(abs(a - b) < 1e-12 for a, b in zip(m.compose(t, m.inverse(t)), i))
print('extrinsic math self-check: PASS')
PY
```

## 4. 추정값을 ground truth와 비교한다

```bash
python3 scripts/extrinsic_math.py \
  data/results/calib.json \
  data/results/ground-truth.json \
  --metrics data/results/metrics.json
```

허용 기준은 다음과 같다.

```text
translation_error_m <= 0.05
rotation_error_deg <= 3.0
```

스크립트 exit code도 확인할 수 있다.

```bash
echo $?
```

`0`이면 통과, `1`이면 실패다. 실패한 결과를 URDF에 적용하지 않는다.

## 5. 적용할 `camera_joint` 값을 계산한다

비교를 통과했을 때만 실행한다.

```bash
python3 scripts/extrinsic_math.py \
  data/results/calib.json \
  data/results/ground-truth.json \
  --camera-joint
```

스크립트가 사용하는 식은 다음과 같다.

```text
T_base_camera_optical
  = T_base_lidar * T_lidar_camera

T_base_camera_link
  = T_base_camera_optical * inverse(T_camera_link_camera_optical)
```

출력 예시는 다음 형태다.

```text
camera_joint xyz: X Y Z
camera_joint rpy: R P Y
```

## 6. URDF의 camera joint 한 곳만 바꾼다

`forks/turtlebot3_simulations/turtlebot3_gazebo/urdf/turtlebot3_waffle_pi_3d.urdf`에서 다음 기존 블록을 찾는다.

```xml
<joint name="camera_joint" type="fixed">
  <origin xyz="0.073 -0.011 0.084" rpy="0 0 0"/>
  <parent link="base_link"/>
  <child link="camera_link"/>
</joint>
```

`origin`의 `xyz`, `rpy`만 스크립트 출력으로 바꾼다. 다음 joint들은 바꾸지 않는다.

- `camera_rgb_joint`
- `camera_rgb_optical_joint`
- `calib_lidar_joint`

또한 ground truth를 보존하기 위해 `models/turtlebot3_waffle_pi_3d/model.sdf`의 camera와 LiDAR pose는 바꾸지 않는다. 원본 `turtlebot3_waffle_pi` 파일도 수정하지 않는다.

## 7. 다시 빌드하고 TF를 확인한다

```bash
cd /home/swlinux/Desktop/workspace/mobin/docker
TURTLEBOT3_MODEL=waffle_pi_3d docker compose run --rm shell
```

컨테이너 안에서:

```bash
source /opt/ros/jazzy/setup.bash
cd /ws
colcon build --symlink-install --packages-select turtlebot3_gazebo
source /ws/install/setup.bash
test "$TURTLEBOT3_MODEL" = "waffle_pi_3d"
ros2 launch turtlebot3_gazebo turtlebot3_calibration.launch.py
```

다른 shell에서:

```bash
ros2 run tf2_ros tf2_echo base_scan camera_rgb_optical_frame
```

출력 transform이 `calib.json`의 `T_lidar_camera`와 허용 가능한 숫자 반올림 범위에서 같아야 한다.

## 8. 적용 전후 projection을 비교한다

calibration tool의 viewer는 `calib.json` transform을 직접 사용하므로 URDF 적용 확인과는 별개다. 적용 여부는 다음 두 가지로 나눠 확인한다.

1. `viewer`: 추정 transform 자체의 image–point projection 품질
2. `tf2_echo`: 추정 transform이 URDF TF chain에 올바른 방향으로 반영됐는지

새 bag을 같은 5개 pose에서 다시 기록해 calibration을 재실행했을 때도 오차 범위가 유지되는지 확인한다. 원본 bag과 결과는 덮어쓰지 말고 `data/bags/repeat-*`, `data/results-repeat/`처럼 별도 경로를 사용한다.

## 완료 조건

- `metrics.json`에 translation/rotation error와 pass 여부가 있다.
- translation error가 5 cm 이하이다.
- rotation error가 3도 이하이다.
- `tf2_echo base_scan camera_rgb_optical_frame`이 calibration 결과와 같은 방향이다.
- SDF ground truth는 변경되지 않았다.
- 반복 dataset에서도 비슷한 오차가 나온다.

## 실패할 때 확인 순서

### translation은 비슷하지만 rotation이 약 180도 틀린다

camera optical frame을 사용했는지 확인한다. `camera_rgb_frame`과 `camera_rgb_optical_frame`을 혼용하면 축 convention 때문에 큰 회전 오차가 난다.

### inverse를 쓰면 숫자가 더 그럴듯해 보인다

그 이유만으로 inverse를 적용하지 않는다. `calib.json` 정의는 `p_lidar = T_lidar_camera * p_camera`다. `tf2_echo base_scan camera_rgb_optical_frame`도 같은 방향으로 비교한다.

### 기준을 넘는다

URDF를 수정하지 말고 다음 순서로 돌아간다.

1. PATCH-01 frame ID와 intensity field
2. PATCH-02 공통 시야, 반사도 변화, bag 정지 상태
3. PATCH-03 manual correspondence와 viewer projection
4. `ground-truth.json`을 현재 SDF/URDF pose에서 만들었는지

## 이 PATCH에서 하지 않는 것

- SDF sensor pose 자동 수정
- 실물 로봇 calibration 값 배포
- 시간 offset calibration
- camera intrinsic 재보정

이 PATCH는 fixed spatial extrinsic 한 가지에만 범위를 제한한다.
