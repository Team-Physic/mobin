# Simulation PATCH-04: Extrinsic을 URDF에 적용하고 정량 검증

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

## 이미 Extrinsic을 아는 경우

**두 sensor의 실제 좌표계 사이 transform을 정확히 안다면 calibration을 다시 할 필요가 없다.** 그 값을 TF/URDF에 넣으면 된다. Simulation에서는 URDF/SDF 값이 ground truth이므로, calibration은 모르는 값을 얻기보다 알고리즘이 정답을 복원하는지 검증하는 실습이다.

| 알고 있는 값 | 의미 | 판단 |
|---|---|---|
| 자·각도기로 잰 sensor 외형의 거리와 각도 | housing 사이의 기계적 측정 | `initial_guess`로 사용 |
| CAD 조립 pose | 설계상 장착 위치 | `initial_guess` 또는 simulation ground truth |
| Camera optical center와 LiDAR 측정 원점 사이의 실제 transform | 두 측정 좌표계를 직접 연결 | 충분히 정확하면 calibration 생략 가능 |

외형 측정만으로 끝내기 어려운 이유는 Camera optical center가 housing 표면에 없고, LiDAR 측정 원점과 내부 축도 외형만 보고 정확히 찾기 어렵기 때문이다. 브래킷 공차·조립 오차·충격도 CAD 값과 실제 값을 다르게 만든다.

각도 오차는 거리가 멀수록 큰 위치 오차가 된다. 대략 `횡방향 오차 = 거리 × tan(각도 오차)`이다. 1° 오차면 1 m에서 약 1.7 cm, 10 m에서 약 17.5 cm 어긋난다. 따라서 자로 잰 값은 초기 투영에는 충분할 수 있지만, point를 image pixel에 정확히 겹쳐야 하면 sensor data로 보정하고 PATCH-04에서 ground truth 또는 독립 검증 data와 비교한다.

## 시작 조건

- Simulation PATCH-03의 `data/results/calib.json`이 있다.
- `results.T_lidar_camera`가 `[x, y, z, qx, qy, qz, qw]` 7개 값이다.
- 아직 추정 결과로 URDF를 수정하지 않았다.

```bash
cd /home/swlinux/Desktop/workspace/mobin
git diff -- forks/turtlebot3_simulations/turtlebot3_gazebo/urdf/turtlebot3_waffle_pi_3d.urdf
```

Simulation PATCH-01에서 유지한 `scan_joint`와 Camera fixed joint의 pose가 바뀌어 있다면 먼저 그 이유를 확인한다.

## 추가/수정할 파일

```text
mobile-robot-calibration-repo/
├── data/results/
│   ├── ground-truth.json
│   └── metrics.json
├── code/scripts/capture-ground-truth.py
├── code/scripts/extrinsic_math.py
└── forks/turtlebot3_simulations/turtlebot3_gazebo/
    └── urdf/turtlebot3_waffle_pi_3d.urdf
```

Gazebo 실제 센서 pose가 있는 `model.sdf`는 이 PATCH에서 수정하지 않는다. SDF까지 추정값으로 바꾸면 비교 기준 자체가 움직인다.

## 1. ground truth JSON을 자동 생성한다

`tf2_echo` 숫자를 수동으로 복사하지 않는다. Simulation PATCH-01의 원래 URDF로 `sim` service를 실행한 뒤, TF tree에서 필요한 transform 세 개를 조회해 JSON으로 저장한다. 이 값은 URDF baseline이다. PATCH-01처럼 SDF sensor pose와 URDF fixed joint가 같은 경우에만 simulation ground truth로 사용한다.

| JSON key | TF 조회 | 의미 |
|---|---|---|
| `T_lidar_camera` | target=`base_scan`, source=`camera_rgb_optical_frame` | Camera optical point를 LiDAR frame으로 변환 |
| `T_base_lidar` | target=`base_link`, source=`base_scan` | LiDAR point를 robot base frame으로 변환 |
| `T_camera_link_camera_optical` | target=`camera_link`, source=`camera_rgb_optical_frame` | optical frame point를 Camera body frame으로 변환 |

**이 script는 `camera_joint`를 수정하기 전에 한 번만 실행한다.** 수정 후 다시 실행하면 추정값이 반영된 TF를 ground truth로 덮어쓰게 된다.

`code/scripts/capture-ground-truth.py` | [main()](../../code/scripts/capture-ground-truth.py#L26)은 TF 세 개를 조회하고 [values()](../../code/scripts/capture-ground-truth.py#L12)로 translation과 quaternion 7개 값을 JSON 배열로 변환한다.

```python
#!/usr/bin/env python3
# code/scripts/capture-ground-truth.py | main()
# code/scripts/capture-ground-truth.py | values()
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
```

실행 권한을 준다.

```bash
chmod +x code/scripts/capture-ground-truth.py
```

Simulation PATCH-02와 같은 방법으로 `waffle_pi_3d` simulation을 background에서 실행한다. `robot_state_publisher` 로그가 나온 뒤, host의 script를 실행 중인 `sim` container의 Python stdin으로 전달한다. `data/`는 `/ws/data`로 bind mount되므로 container가 쓴 JSON이 host에 남는다.

```bash
cd /home/swlinux/Desktop/workspace/mobin/docker

docker compose \
  -f compose.yaml \
  -f compose.nvidia.yaml \
  exec -T sim bash -lc '
    source /opt/ros/jazzy/setup.bash
    source /ws/install/setup.bash
    python3 - /ws/data/results/ground-truth.json
  ' < ../code/scripts/capture-ground-truth.py
```

생성 결과를 host에서 검사한다.

```bash
cd /home/swlinux/Desktop/workspace/mobin
python3 -m json.tool data/results/ground-truth.json
```

**통과 기준:** 세 transform이 모두 `[x, y, z, qx, qy, qz, qw]` 7개 유한값이며, `frames.lidar`가 `base_scan`, `frames.camera_optical`이 `camera_rgb_optical_frame`이다. TF lookup timeout이 나면 JSON을 수동으로 만들지 말고 `sim` service와 `robot_state_publisher`를 먼저 확인한다.

## 2. 비교와 변환을 한 스크립트에 둔다

새 수학 라이브러리를 설치하지 않고 Python 표준 라이브러리만 쓴다.

`code/scripts/extrinsic_math.py` | [main()](../../code/scripts/extrinsic_math.py#L82)은 [compare()](../../code/scripts/extrinsic_math.py#L73)로 추정·GT 오차를 계산하고, 통과한 결과의 `camera_joint`를 [compose()](../../code/scripts/extrinsic_math.py#L42)와 [inverse()](../../code/scripts/extrinsic_math.py#L52)로 계산한다.

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
chmod +x code/scripts/extrinsic_math.py
```

## 3. 수학 스크립트의 최소 self-check를 한다

identity transform을 합성했을 때 입력이 그대로여야 한다.

```bash
python3 - <<'PY'
import importlib.util

spec = importlib.util.spec_from_file_location('extrinsic_math', 'code/scripts/extrinsic_math.py')
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
python3 code/scripts/extrinsic_math.py \
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
python3 code/scripts/extrinsic_math.py \
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

1. Simulation PATCH-01 frame ID와 intensity field
2. Simulation PATCH-02 공통 시야, 반사도 변화, bag 정지 상태
3. Simulation PATCH-03 manual correspondence와 viewer projection
4. `ground-truth.json`을 현재 SDF/URDF pose에서 만들었는지

## 이 PATCH에서 하지 않는 것

- SDF sensor pose 자동 수정
- 실물 로봇 calibration 값 배포
- 시간 offset calibration
- camera intrinsic 재보정

이 PATCH는 fixed spatial extrinsic 한 가지에만 범위를 제한한다.
