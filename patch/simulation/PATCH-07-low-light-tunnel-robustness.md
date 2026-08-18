# Simulation PATCH-07: 저조도 터널 환경에서 Extrinsic 강건성 평가

## 왜 이 시나리오를 추천하는가

정적/동적 장애물은 navigation 동작을 구분하는 기준이지 extrinsic calibration 자체의 난이도를 잘 설명하지 못한다. 추가 실습으로는 **실내 calibration 결과를 저조도 터널 환경에서도 재현할 수 있는지 평가하는 것**이 더 유용하다.

이 시나리오는 다음을 한 번에 보여 준다.

- 조명이 약해져 camera texture가 사라질 때 calibration이 어떻게 나빠지는가
- 국부 조명과 그림자가 grayscale–LiDAR intensity 상관관계에 어떤 영향을 주는가
- 안개처럼 보이는 시각 열화에서 결과를 무조건 신뢰하지 않고 실패를 검출하는 방법
- 환경만 바뀌고 실제 extrinsic ground truth는 같다는 controlled experiment

새 로봇을 추가하지 않고 Simulation PATCH-01의 같은 sensor rig와 Simulation PATCH-02의 같은 구조물을 재사용한다. 비교 변수를 환경으로만 제한하기 위해서다.

## 이 PATCH의 위치

Simulation PATCH-04까지 완료한 뒤 수행하는 선택 확장이다. Simulation PATCH-05와 Simulation PATCH-06의 장애물 회피 실습과는 독립적이다.

```text
Simulation PATCH-02 밝은 실내 dataset ──┐
                            ├─ 같은 ground truth와 비교
Simulation PATCH-07 저조도 터널 dataset ─┘
```

## 추가/수정할 파일

```text
mobile-robot-calibration-repo/
├── data/
│   ├── bags-low-light/
│   └── results-low-light/
├── scripts/run-calibration.sh
└── forks/turtlebot3_simulations/turtlebot3_gazebo/
    ├── launch/turtlebot3_low_light_tunnel.launch.py
    └── worlds/turtlebot3_low_light_tunnel.world
```

새 model을 만들지 않는다. Simulation PATCH-02의 `calibration_scene`을 그대로 include하고 world의 벽, 조명, fog만 추가한다.

## 1. 밝은 실내 baseline을 보존한다

Simulation PATCH-04 결과를 덮어쓰지 않는다.

```bash
cd /home/swlinux/Desktop/workspace/mobile-robot-calibration-repo
test -f data/results/calib.json
test -f data/results/metrics.json
```

baseline에서 기록할 값은 다음과 같다.

- `data/results/calib.json`의 `T_lidar_camera`
- `translation_error_m`
- `rotation_error_deg`
- viewer의 정성 projection 결과

## 2. calibration 실행 스크립트의 경로만 재사용 가능하게 한다

Simulation PATCH-03의 `scripts/run-calibration.sh`에서 두 줄을 다음처럼 바꾼다.

```bash
BAGS_DIR=${BAGS_DIR:-"$ROOT_DIR/data/bags"}
RESULTS_DIR=${RESULTS_DIR:-"$ROOT_DIR/data/results"}
```

기본 동작은 바뀌지 않는다. Simulation PATCH-07에서만 환경변수로 다른 dataset을 선택한다. 새 스크립트를 복사하지 않는다.

디렉터리를 만든다.

```bash
mkdir -p data/bags-low-light data/results-low-light
```

`.gitignore`에 추가한다.

```gitignore
data/bags-low-light/
data/results-low-light/
```

## 3. low-light tunnel world를 만든다

Simulation PATCH-02 world를 복사한다.

```bash
cp forks/turtlebot3_simulations/turtlebot3_gazebo/worlds/turtlebot3_calibration.world \
   forks/turtlebot3_simulations/turtlebot3_gazebo/worlds/turtlebot3_low_light_tunnel.world
```

기존 OpenRobotics `Sun` include를 제거한다. ground plane과 `calibration_scene` include는 유지한다.

### scene을 어둡게 하고 얕은 fog를 추가한다

기존 `<scene>` 블록을 다음으로 교체한다.

```xml
<scene>
  <ambient>0.015 0.015 0.020 1</ambient>
  <background>0.005 0.005 0.008 1</background>
  <shadows>true</shadows>
  <fog>
    <color>0.04 0.04 0.05 1</color>
    <type>linear</type>
    <start>3.0</start>
    <end>12.0</end>
  </fog>
</scene>
```

fog를 너무 짙게 시작하지 않는다. 먼저 camera의 먼 물체 contrast만 낮추고 가까운 대응점은 남겨야 실패 경계를 관찰할 수 있다.

### 터널 벽과 천장을 추가한다

다음 static model을 `<world>` 안에 추가한다.

```xml
<model name="tunnel_shell">
  <static>true</static>

  <link name="left_wall">
    <pose>4.0 2.2 1.5 0 0 0</pose>
    <collision name="collision">
      <geometry><box><size>8.0 0.15 3.0</size></box></geometry>
    </collision>
    <visual name="visual">
      <geometry><box><size>8.0 0.15 3.0</size></box></geometry>
      <material>
        <ambient>0.10 0.10 0.11 1</ambient>
        <diffuse>0.16 0.16 0.18 1</diffuse>
      </material>
    </visual>
  </link>

  <link name="right_wall">
    <pose>4.0 -2.2 1.5 0 0 0</pose>
    <collision name="collision">
      <geometry><box><size>8.0 0.15 3.0</size></box></geometry>
    </collision>
    <visual name="visual">
      <geometry><box><size>8.0 0.15 3.0</size></box></geometry>
      <material>
        <ambient>0.10 0.10 0.11 1</ambient>
        <diffuse>0.16 0.16 0.18 1</diffuse>
      </material>
    </visual>
  </link>

  <link name="ceiling">
    <pose>4.0 0 3.0 0 0 0</pose>
    <collision name="collision">
      <geometry><box><size>8.0 4.4 0.15</size></box></geometry>
    </collision>
    <visual name="visual">
      <geometry><box><size>8.0 4.4 0.15</size></box></geometry>
      <material>
        <ambient>0.08 0.08 0.09 1</ambient>
        <diffuse>0.12 0.12 0.14 1</diffuse>
      </material>
    </visual>
  </link>
</model>
```

### 서로 다른 밝기의 고정 spot light 두 개를 추가한다

```xml
<light name="near_work_light" type="spot">
  <pose>1.8 -1.4 2.4 0 0.55 0.35</pose>
  <diffuse>0.9 0.75 0.55 1</diffuse>
  <specular>0.2 0.2 0.2 1</specular>
  <attenuation>
    <range>7.0</range>
    <constant>0.5</constant>
    <linear>0.08</linear>
    <quadratic>0.02</quadratic>
  </attenuation>
  <direction>1.0 0.1 -0.5</direction>
  <spot>
    <inner_angle>0.25</inner_angle>
    <outer_angle>0.65</outer_angle>
    <falloff>1.0</falloff>
  </spot>
</light>

<light name="far_service_light" type="spot">
  <pose>5.2 1.2 2.5 0 0.65 -2.8</pose>
  <diffuse>0.25 0.32 0.55 1</diffuse>
  <specular>0.1 0.1 0.15 1</specular>
  <attenuation>
    <range>5.0</range>
    <constant>0.8</constant>
    <linear>0.12</linear>
    <quadratic>0.03</quadratic>
  </attenuation>
  <direction>-1.0 -0.1 -0.45</direction>
  <spot>
    <inner_angle>0.20</inner_angle>
    <outer_angle>0.55</outer_angle>
    <falloff>1.0</falloff>
  </spot>
</light>
```

빛은 움직이지 않는다. 환경 강건성만 보려는 dataset에 flicker와 동적 shadow까지 동시에 넣지 않는다.

## 4. low-light launch를 만든다

Simulation PATCH-02 launch를 복사한다.

```bash
cp forks/turtlebot3_simulations/turtlebot3_gazebo/launch/turtlebot3_calibration.launch.py \
   forks/turtlebot3_simulations/turtlebot3_gazebo/launch/turtlebot3_low_light_tunnel.launch.py
```

새 launch에서 world 파일명만 바꾼다.

```python
world = os.path.join(
    get_package_share_directory('turtlebot3_gazebo'),
    'worlds',
    'turtlebot3_low_light_tunnel.world')
```

pose 인자, spawn, bridge, camera image bridge는 calibration launch와 동일하게 유지한다.

## 5. 빌드하고 센서 차이를 확인한다

```bash
cd /home/swlinux/Desktop/workspace/mobile-robot-calibration-repo/docker
docker compose run --rm shell
```

컨테이너 안에서:

```bash
source /opt/ros/jazzy/setup.bash
cd /ws
colcon build --symlink-install --packages-select turtlebot3_gazebo
source /ws/install/setup.bash
ros2 launch turtlebot3_gazebo turtlebot3_low_light_tunnel.launch.py
```

확인할 것은 다음과 같다.

- image의 어두운 영역에도 구조물 모서리가 일부 남아 있는가
- 가까운 work light 아래는 밝고 먼 영역은 contrast가 낮은가
- `/calib/points`의 geometry, frame ID, update rate는 baseline과 같은가
- LiDAR intensity field가 사라지지 않았는가

```bash
ros2 topic hz /calib/points
ros2 topic echo /calib/points --field fields --once
ros2 topic hz /camera/image_raw
```

point cloud 자체의 pose나 noise를 바꾸지 않는다. 환경만 바뀌어야 비교가 가능하다.

## 6. 같은 5개 pose에서 low-light bag을 기록한다

Simulation PATCH-02의 pose 표를 그대로 사용한다. 예:

```bash
ros2 launch turtlebot3_gazebo turtlebot3_low_light_tunnel.launch.py \
  x_pose:=0.0 y_pose:=0.0 yaw:=0.0
```

별도 shell에서:

```bash
ros2 bag record \
  -o /ws/data/bags-low-light/pose-01 \
  /calib/points \
  /camera/image_raw \
  /camera/camera_info \
  /tf \
  /tf_static \
  /clock
```

`pose-02`부터 `pose-05`까지 동일하게 기록한다. baseline과 같은 12~20초, 같은 sensor 설정, 같은 정지 조건을 사용한다.

## 7. 별도 결과 디렉터리에서 calibration한다

```bash
BAGS_DIR="$PWD/data/bags-low-light" \
RESULTS_DIR="$PWD/data/results-low-light" \
./scripts/run-calibration.sh preprocess

BAGS_DIR="$PWD/data/bags-low-light" \
RESULTS_DIR="$PWD/data/results-low-light" \
./scripts/run-calibration.sh initial

BAGS_DIR="$PWD/data/bags-low-light" \
RESULTS_DIR="$PWD/data/results-low-light" \
./scripts/run-calibration.sh calibrate

BAGS_DIR="$PWD/data/bags-low-light" \
RESULTS_DIR="$PWD/data/results-low-light" \
./scripts/run-calibration.sh viewer
```

manual correspondence는 빛이 잘 드는 점만 6개 고르지 않는다. 밝은/어두운 영역과 가까운/먼 깊이에 분산한다.

## 8. 같은 ground truth로 정량 비교한다

```bash
python3 scripts/extrinsic_math.py \
  data/results-low-light/calib.json \
  data/results/ground-truth.json \
  --metrics data/results-low-light/metrics.json
```

두 환경의 결과를 출력한다.

```bash
python3 - <<'PY'
import json
from pathlib import Path

for name, path in (
    ('bright indoor', 'data/results/metrics.json'),
    ('low-light tunnel', 'data/results-low-light/metrics.json'),
):
    m = json.loads(Path(path).read_text())
    print(
        f"{name:16s} "
        f"translation={m['translation_error_m']:.4f} m "
        f"rotation={m['rotation_error_deg']:.3f} deg "
        f"pass={m['pass']}")
PY
```

## 완료 조건

- baseline과 low-light가 같은 sensor ground truth를 사용한다.
- low-light 5개 bag과 별도 `calib.json`이 있다.
- low-light 결과도 5 cm / 3도 기준을 통과한다.
- viewer에서 밝은 영역뿐 아니라 어두운/먼 영역의 projection도 확인했다.
- 성능이 나빠졌다면 어떤 조건에서 correspondence 또는 NID registration이 무너졌는지 기록했다.

이 실험은 반드시 성공값을 만드는 것이 목적이 아니다. 조도 저하가 어느 지점부터 calibration 결과를 신뢰할 수 없게 만드는지 발견하는 것도 올바른 결과다.

## 단계적 난이도 조절

처음부터 모든 값을 바꾸지 않는다. 다음 순서로 한 요소씩만 조정한다.

1. fog 없이 낮은 ambient와 spot light만 사용
2. `<fog><start>3</start><end>12</end>` 추가
3. fog end를 `8`로 낮춰 먼 contrast 감소
4. camera noise의 `stddev`를 `0.007 -> 0.015`로 증가

각 단계마다 별도 bag/result 디렉터리를 사용한다. 이렇게 해야 어떤 조건이 실패를 만들었는지 알 수 있다.

## 시뮬레이션 해석의 한계

- SDF fog는 렌더링 scene의 가시성 효과다. 실제 분진의 입자별 산란, LiDAR false return, wet surface 반사까지 물리적으로 재현한다고 보면 안 된다.
- camera auto-exposure, motion blur, lens flare도 현재 Waffle Pi camera SDF에 정밀 모델링되어 있지 않다.
- 따라서 이 PATCH는 **환경 domain shift와 실패 검출 실습**이지 실제 광산/수중 센서 인증 시험이 아니다.

Gazebo Harmonic은 GPU LiDAR, camera, Gaussian noise, laser retroreflection을 지원하고, SDFormat scene은 ambient light와 fog를 정의할 수 있으므로 이 범위의 controlled experiment에는 적합하다.

## 왜 수중이나 휴머노이드를 먼저 선택하지 않았는가

수중에서는 RGB camera–공기식 LiDAR 조합 자체가 일반적인 선택이 아니며 굴절, 흡수, 부유물, housing port 모델이 필요하다. Gazebo에 DVL 같은 수중 센서는 있지만 현재 `direct_visual_lidar_calibration` 실습과는 다른 sensor pair가 된다.

휴머노이드도 가능하지만 조건이 있다.

- camera와 LiDAR가 같은 머리 rigid link에 붙어 있으면 이 PATCH를 그대로 적용할 수 있다.
- camera는 머리, LiDAR는 torso처럼 서로 다른 관절 link에 붙어 있으면 상대 transform이 관절각에 따라 변한다.
- 후자의 문제는 하나의 고정 `T_lidar_camera`를 구하는 문제가 아니라 joint encoder를 포함한 kinematic/hand-eye calibration이다. 현재 calibration tool에 여러 head pose bag을 한꺼번에 넣으면 안 된다.

따라서 고정 extrinsic을 먼저 확실히 실습한 뒤, 다음 연구 단계로 humanoid head–torso kinematic calibration을 분리하는 것이 좋다.

## 참고 자료

- Gazebo Harmonic Sensors: <https://gazebosim.org/docs/harmonic/sensors/>
- Gazebo feature comparison: <https://gazebosim.org/docs/harmonic/comparison/>
- SDFormat 1.8 Scene/Fog: <https://sdformat.org/spec/1.8/scene/>
- Direct Visual LiDAR Calibration data collection: <https://koide3.github.io/direct_visual_lidar_calibration/collection/>
