# Simulation PATCH-06: Python/C++ 장애물 회피 node

- 상태: **구현·단위검사·headless 통합검사 완료**
- 입력: `sensor_msgs/msg/LaserScan` `/scan`
- 출력: `geometry_msgs/msg/TwistStamped` `/cmd_vel`

## 결론

같은 parameter와 topic 계약을 쓰는 Python·C++ 회피 node를 각각 만들었다. Sensor data가 없거나 오래되면 정지한다. 기본값은 더 열린 방향을 선택하지만 `crossing`에서는 사람+cart를 마주치면 오른쪽 회피를 우선한다.

**회피 controller는 LiDAR 거리만 사용하는 안전 baseline이다.** 별도 `lidar_bbox_association` node는 `/detections`, `/calib/points`, `/camera/camera_info`, TF를 받아 YOLO bbox와 LiDAR point를 연결한다. 아직 이 semantic 결과로 `/cmd_vel`을 바꾸지는 않는다.

## What I Made

```text
code/python/mobile_robot_lab_python/
├── mobile_robot_lab_python/
│   ├── obstacle_avoidance.py
│   ├── camera_lidar_fusion.py
│   └── lidar_bbox_association_node.py
└── test/{test_camera_lidar_fusion,test_lidar_bbox_association_node}.py

code/cpp/mobile_robot_lab_cpp/
├── include/mobile_robot_lab_cpp/{scan_utils,camera_lidar_fusion}.hpp
├── src/obstacle_avoidance.cpp
└── test/camera_lidar_fusion_test.cpp
```

| 구현 | executable | 선택값 |
|---|---|---|
| Python `rclpy` | `obstacle_avoidance_py` | `AVOIDANCE_IMPLEMENTATION=python` |
| C++ `rclcpp` | `obstacle_avoidance_cpp` | `AVOIDANCE_IMPLEMENTATION=cpp` |

## 개념

| 개념 | 이 node에서의 의미 |
|---|---|
| LaserScan | LiDAR 한 회전의 거리 배열과 각도 정보 |
| sector | 정면·왼쪽·오른쪽 각도 범위에서 가장 가까운 거리 |
| hysteresis | 0.45 m에서 회피 시작, 0.55 m가 되어야 전진 재개 |
| scan timeout | 0.5초간 새 scan이 없으면 0 속도 발행 |
| turn latch | 회피 중 회전 방향을 유지해 거리 noise에 따른 좌우 흔들림 방지 |

| 구분 | 현재 PATCH-06 | Camera–LiDAR semantic fusion |
|---|---|---|
| Camera 입력 | 사용하지 않음 | image에서 cart bbox 또는 mask 검출 |
| LiDAR 입력 | `/scan`의 방향별 최단 거리 | calibration extrinsic으로 3D point를 image pixel에 투영 |
| 판단 | 물체 종류와 무관하게 거리 임계값으로 회피 | cart 영역에 겹친 point로 cart의 거리·3D 위치 계산 |
| AI model | 없음 | YOLO·RT-DETR 같은 object detector가 별도로 필요 |

[MediaPipe Solutions](https://developers.google.com/edge/mediapipe/solutions/guide)에도 object detection task와 custom model 경로가 있다. 그러나 MediaPipe 자체가 pallet jack class를 보장하는 detector는 아니다. 이미 학습·검증한 YOLO가 있으므로 inference framework만 바꾸는 작업은 중복이다. **기존 YOLO bbox를 재사용하고, 이 PATCH에는 Python/C++ 계산 core와 ROS 2 association node를 추가했다.**

단일 camera pixel `(u,v)`만으로는 깊이를 모르므로 `(x,y,z)`를 하나로 복원할 수 없다. 실제 계산 방향은 다음과 같다.

```text
LiDAR point (x,y,z)
  → PATCH-04의 T_camera_lidar로 Camera optical frame 변환
  → CameraInfo의 fx, fy, cx, cy로 pixel (u,v)에 투영
  → YOLO bbox 안에 들어온 point만 선택
  → 선택 point의 median (x,y,z)와 LiDAR range 계산
```

| 입력 | 출처 | 역할 |
|---|---|---|
| YOLO bbox `(u_min,v_min,u_max,v_max)` | 기존 YOLO 프로젝트 | 영상에서 cart가 차지한 영역 |
| LiDAR points | `/calib/points` | 각 pixel 후보의 실제 3D 깊이 |
| `T_camera_lidar` | PATCH-04 calibration 결과 | LiDAR 좌표를 Camera optical 좌표로 변환 |
| `fx,fy,cx,cy` | `/camera/camera_info` | Camera 3D 좌표를 image pixel로 투영 |

현재 계산은 rectified pinhole image 또는 distortion이 0인 simulation을 전제로 한다. 실물 raw image에 distortion이 있으면 image를 rectification한 뒤 그 영상에서 얻은 YOLO bbox를 사용한다.

`waffle_pi_3d`와 warehouse용 3배 model `waffle_pi_3d_large`도 `/scan`을 발행하므로 이 node를 실행할 수 있다. 다만 controller는 `/calib/points`의 전체 3D point cloud가 아니라 `LaserScan` 거리 배열만 쓴다.

## Why?

PATCH-05의 scenario만 있으면 robot은 자동으로 움직이지 않는다. 다음 data flow가 필요하다.

```text
Gazebo LiDAR → /scan → sector 최솟값 → 회피 상태 결정 → /cmd_vel → DiffDrive
```

Python은 알고리즘을 빠르게 읽고 수정하기 좋다. C++은 실시간 경로의 allocation·latency를 더 직접 통제하기 좋다. 둘을 동일 입출력으로 만들어 결과를 비교할 수 있게 했다.

### 퀀텀에어로 Computer Vision 직무 관점

검토한 퀀텀에어로 지원 CV는 이미 YOLOv8, ONNX, Camera–LiDAR calibration 경험을 제시한다. LiDAR-only 회피는 ROS 2와 safety fallback 증거지만 Computer Vision 핵심 결과로는 약하다. 외부 workspace의 개인 CV 경로는 GitHub 문서에 넣지 않는다.

[현재 Computer Vision 공고](https://www.saramin.co.kr/zf_user/jobs/relay/pop-view?rec_idx=54833402)는 detection·segmentation, YOLO, 평가 지표를 요구하고 LiDAR raw data, Camera–LiDAR sensor fusion, ROS 2, ONNX/TensorRT를 우대한다. 따라서 새 MediaPipe demo보다 다음 연결이 더 직접적이다.

| 구현 | 직무에 보여주는 것 |
|---|---|
| 기존 YOLO bbox 재사용 | 이미 만든 detector를 ROS 2 perception 입력으로 연결 |
| bbox 안 LiDAR point association | calibration 결과를 실제 3D 위치 추정에 사용 |
| median 3D position·point count | outlier에 덜 민감한 거리와 association 품질 |
| LiDAR-only fallback 유지 | detector miss·지연에도 unknown obstacle 안전 회피 |
| 후속 MCAP 평가 | 3D position error, end-to-end latency, detection miss 측정 |

이 PATCH는 **fusion 계산 core, ROS 2 topic 연결, 단위검사까지 구현**한다. YOLO weight가 이 repository에 없으므로 detector 자체가 실행됐다고 주장하지 않는다. `lidar_bbox_association`은 0.1초보다 오래된 detection을 거부해 과거 bbox와 현재 point cloud를 잘못 결합하지 않는다.

## What was problem

| 문제 | 잘못된 결과 | 반영한 처리 |
|---|---|---|
| scan 시작 전 전진 | sensor 없이 이동 | `scan_timeout` 상태에서 정지 |
| `NaN`, 범위 밖 값 | 잘못된 최솟값 | 무시 |
| `+inf`만 있는 열린 방향 | sector가 invalid가 됨 | `range_max`로 해석 |
| 정지 경계 noise | 전진·회전 반복 | stop/clear 두 임계값 사용 |
| 좌우 거리가 비슷함 | `turn_left/right` 빠른 반복 | 회전 방향을 회피 종료까지 유지 |
| crossing에서 좌·우 선택이 실행마다 달라짐 | 사람+cart 앞에서 결과 재현이 어려움 | `preferred_turn_direction=right`를 scenario override로 전달 |
| 두 model이 첫 횡단에서 만나지 않음 | robot `0.15 m/s`, cart `0.35 m/s`로 교차점 도착시각이 약 9초 차이 | `crossing`의 robot만 `linear_velocity=0.25 m/s`로 설정 |
| 3배 robot이 감지 전에 접촉 | collision은 커졌지만 거리 임계값은 원본 | large model에 stop/clear/side=`1.0/1.2/0.7 m` 적용 |
| Python/C++ 동시 실행 | `/cmd_vel` publisher 충돌 | launch가 하나만 선택 |

## How it changed

### YOLO bbox와 LiDAR point 연결

| 구현 | 함수 | 결과 |
|---|---|---|
| Python | [associate_bbox()](../../code/python/mobile_robot_lab_python/mobile_robot_lab_python/camera_lidar_fusion.py) | bbox 안 point의 LiDAR/Camera median 3D 위치와 거리 |
| C++ | [associate_bbox()](../../code/cpp/mobile_robot_lab_cpp/include/mobile_robot_lab_cpp/camera_lidar_fusion.hpp) | Python과 같은 projection·선택 규칙 |

```python
# code/python/.../camera_lidar_fusion.py | associate_bbox()
point_camera = transform_point(point_lidar, rotation, translation)
pixel = project_point(point_camera, intrinsics)
if pixel is None:
    continue
u, v = pixel
# YOLO bbox 안에 투영된 LiDAR point만 cart의 3D 후보로 남긴다.
if bbox.u_min <= u <= bbox.u_max and bbox.v_min <= v <= bbox.v_max:
    matches.append((point_lidar, point_camera))
```

세 점보다 적으면 association 실패를 반환한다. 통과하면 평균보다 outlier 영향이 작은 축별 median과 LiDAR 원점 기준 거리를 반환한다.

### ROS 2 association node

| topic | type | 역할 |
|---|---|---|
| `/detections` | `vision_msgs/msg/Detection2DArray` | 기존 YOLO의 class·score·bbox 입력 |
| `/calib/points` | `sensor_msgs/msg/PointCloud2` | LiDAR 3D point 입력 |
| `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | `fx, fy, cx, cy` 입력 |
| `/fusion/associated_points` | `sensor_msgs/msg/PointCloud2` | bbox 안에 투영된 point만 발행 |
| `/fusion/detections_3d` | `geometry_msgs/msg/PoseArray` | bbox별 median 3D 위치 발행 |

`LidarBboxAssociationNode.on_points()`는 point cloud 시각의 `cloud.header.frame_id → camera_info.header.frame_id` TF를 조회한다. 이 TF가 PATCH-04에서 검증한 extrinsic이다. Detection과 point cloud의 timestamp 차이가 `max_time_delta`보다 크거나 TF가 없으면 결과를 만들지 않는다.

```python
# code/python/.../lidar_bbox_association_node.py | LidarBboxAssociationNode.on_points()
transform = self.tf_buffer.lookup_transform(
    camera_frame, cloud.header.frame_id,
    Time.from_msg(cloud.header.stamp),
    timeout=Duration(seconds=0.05))

# 같은 bbox에 연결된 LiDAR point를 별도 cloud로 발행해 RViz/Rerun에서 확인한다.
self.points_publisher.publish(point_cloud2.create_cloud_xyz32(
    cloud.header, associated_points))
```

| parameter | 기본값 | 의미 |
|---|---:|---|
| `target_class` | `cart` | 이 class ID만 association. 빈 문자열은 모든 class |
| `min_score` | `0.5` | YOLO confidence 하한 |
| `min_points` | `3` | bbox에 필요한 최소 LiDAR point 수 |
| `max_time_delta` | `0.1 s` | detection–point cloud 허용 timestamp 차이 |

### 공통 parameter

[avoidance.yaml](../../forks/turtlebot3_simulations/turtlebot3_gazebo/params/avoidance.yaml)은 두 구현에 똑같이 전달된다.

| parameter | 기본값 | 의미 |
|---|---:|---|
| `linear_velocity` | 0.15 m/s | 전진 속도 |
| `angular_velocity` | 0.8 rad/s | 제자리 회전 속도 |
| `stop_distance` | 0.45 m | 회피 시작 거리 |
| `clear_distance` | 0.55 m | 전진 재개 거리 |
| `side_distance` | 0.40 m | 회전할 측면이 막혔다고 보는 거리 |
| `scan_timeout` | 0.5 s | sensor fault 정지 시간 |
| `front_half_angle_deg` | 15° | 정면 sector 반쪽 각도 |
| `side_angle_deg` | 60° | 좌·우 판단 범위 |
| `preferred_turn_direction` | `auto` | `auto`는 막히면 반대쪽 전환; `left`·`right`는 지정 방향이 열릴 때까지 정지 |

`crossing` launch는 `linear_velocity=0.25 m/s`, `preferred_turn_direction=right`를 덮어쓴다. 이는 `X_POSE=-2.0`인 robot과 `x=0.9 m` 통로를 지나는 사람+cart가 **첫 +y 횡단에서 만나게 하는 scenario 전용 값**이다. YAML의 다른 scenario 기본속도 `0.15 m/s`는 바꾸지 않는다.

사람+cart가 오른쪽 sector를 점유한 동안 robot은 `blocked`로 정지한다. 통과 후 오른쪽 sector가 열리면 `turn_right`를 실행한다. 따라서 지정 방향이 막혔는데도 그쪽으로 회전하지 않고, 요구한 오른쪽 회피를 임의로 왼쪽 회피로 바꾸지도 않는다.

`clear_distance > stop_distance`여야 한다. 두 구현은 시작 시 parameter를 검사하고 잘못되면 실행을 중단한다.

### Python

| 파일 위치 | 함수 | 역할 |
|---|---|---|
| `code/python/mobile_robot_lab_python/mobile_robot_lab_python/obstacle_avoidance.py` | [sector_distances()](../../code/python/mobile_robot_lab_python/mobile_robot_lab_python/obstacle_avoidance.py#L25) | 입력: `LaserScan` 각도·거리<br>처리: angle 정규화 후 세 sector 최솟값 계산<br>결과: front·left·right 거리 |
| `code/python/mobile_robot_lab_python/mobile_robot_lab_python/obstacle_avoidance.py` | [AvoidancePolicy.command()](../../code/python/mobile_robot_lab_python/mobile_robot_lab_python/obstacle_avoidance.py#L75) | 입력: 세 sector 거리<br>판정: hysteresis·회전 방향·blocked<br>결과: linear·angular command |
| `code/python/mobile_robot_lab_python/mobile_robot_lab_python/obstacle_avoidance.py` | [ObstacleAvoidanceNode.__init__()](../../code/python/mobile_robot_lab_python/mobile_robot_lab_python/obstacle_avoidance.py#L113) | 입력: ROS parameter<br>처리: publisher·subscriber·20 Hz timer 생성<br>결과: 회피 node 준비 |
| `code/python/mobile_robot_lab_python/mobile_robot_lab_python/obstacle_avoidance.py` | [ObstacleAvoidanceNode.update()](../../code/python/mobile_robot_lab_python/mobile_robot_lab_python/obstacle_avoidance.py#L174) | 입력: 최근 scan·timestamp<br>판정: timeout 또는 policy mode<br>결과: 안전 `cmd_vel` 발행 |

핵심 판단:

```python
# code/python/mobile_robot_lab_python/mobile_robot_lab_python/obstacle_avoidance.py | AvoidancePolicy.command()
if distances.front <= self.stop_distance and not self.avoiding:
    self.avoiding = True
    # crossing은 right, 다른 scenario의 auto는 더 열린 쪽을 선택한다.
    self.turn_direction = self.choose_turn_direction(distances)
elif self.avoiding and distances.front >= self.clear_distance:
    self.avoiding = False
    self.turn_direction = None

# ... 생략: 왼쪽·오른쪽 sector의 점유 여부 계산

# 지정한 오른쪽 sector가 아직 점유됐으면 속도 0으로 기다린다.
if (self.preferred_turn_direction == self.turn_direction and
        preferred_side_blocked):
    return 0.0, 0.0, 'blocked'
```

### C++

| 파일 위치 | 함수 | 역할 |
|---|---|---|
| `code/cpp/mobile_robot_lab_cpp/include/mobile_robot_lab_cpp/scan_utils.hpp` | [sector_distances()](../../code/cpp/mobile_robot_lab_cpp/include/mobile_robot_lab_cpp/scan_utils.hpp#L28) | 입력: `LaserScan` 각도·거리<br>처리: Python과 같은 sector·invalid range 규칙 적용<br>결과: 세 sector 거리 |
| `code/cpp/mobile_robot_lab_cpp/src/obstacle_avoidance.cpp` | [ObstacleAvoidanceNode()](../../code/cpp/mobile_robot_lab_cpp/src/obstacle_avoidance.cpp#L24) | 입력: ROS parameter<br>처리: 검증 후 ROS entity 생성<br>결과: 회피 node 준비 |
| `code/cpp/mobile_robot_lab_cpp/src/obstacle_avoidance.cpp` | [scan_callback()](../../code/cpp/mobile_robot_lab_cpp/src/obstacle_avoidance.cpp#L75) | 입력: 새 `LaserScan`<br>처리: sector와 scan 시각 저장<br>결과: `update()`용 상태 갱신 |
| `code/cpp/mobile_robot_lab_cpp/src/obstacle_avoidance.cpp` | [update()](../../code/cpp/mobile_robot_lab_cpp/src/obstacle_avoidance.cpp#L102) | 입력: 최근 scan·timestamp<br>판정: timeout·hysteresis·회전 방향<br>결과: `cmd_vel` 발행 |

```cpp
// code/cpp/mobile_robot_lab_cpp/src/obstacle_avoidance.cpp | ObstacleAvoidanceNode::update()
const bool timed_out = !has_scan_ ||
  (now() - last_scan_time_).seconds() > scan_timeout_;
if (timed_out) {
  // LiDAR가 끊기면 마지막 전진 명령을 유지하지 않는다.
  publish_mode(0.0, 0.0, "scan_timeout");
  return;
}
```

## Build와 test

```bash
cd /home/swlinux/Desktop/workspace/mobin/docker

docker compose run --rm shell bash -lc '
  source /opt/ros/jazzy/setup.bash
  cd /ws
  colcon build --symlink-install --packages-select \
    turtlebot3_gazebo mobile_robot_lab_python mobile_robot_lab_cpp
  colcon test --packages-select \
    mobile_robot_lab_python mobile_robot_lab_cpp
  colcon test-result --verbose
'
```

NVIDIA 설정을 build shell에도 전달하려면 다음처럼 실행한다.

```bash
docker compose \
  -f compose.yaml \
  -f compose.nvidia.yaml \
  run --rm shell bash -lc '
    source /opt/ros/jazzy/setup.bash
    cd /ws
    colcon build --symlink-install --packages-select \
      turtlebot3_gazebo mobile_robot_lab_python mobile_robot_lab_cpp
  '
```

`colcon test-result` 통과 결과:

```text
Summary: 7 tests, 0 errors, 0 failures, 0 skipped
```

검사 범위:

- 0 rad, +30°, -30°가 front/left/right로 들어가는지
- `NaN` 무시와 `+inf → range_max` 처리
- 빈 scan이 invalid인지
- stop/clear hysteresis와 회전 방향 유지
- Camera 뒤의 LiDAR point가 projection에서 제외되는지
- YOLO bbox 안의 point만 3D 위치에 포함되는지
- bbox 안 point가 세 개보다 적으면 association이 실패하는지
- TF quaternion이 올바른 3×3 회전행렬로 바뀌는지

## Scenario 실행

검증한 crossing scenario:

```bash
cd /home/swlinux/Desktop/workspace/mobin/docker

TURTLEBOT3_MODEL=waffle_pi_3d_large \
TURTLEBOT3_WORLD_DIR=/opt/aws_warehouse/worlds/small_warehouse \
TURTLEBOT3_WORLD=small_warehouse_harmonic.world \
AVOIDANCE_SCENARIO=crossing \
AVOIDANCE_IMPLEMENTATION=cpp \
AVOIDANCE_START_DELAY=10.0 \
X_POSE=-2.0 Y_POSE=-0.5 YAW=0.0 \
GAZEBO_GUI=true LAUNCH_RVIZ=true \
docker compose \
  -f compose.yaml \
  -f compose.nvidia.yaml \
  up --force-recreate avoidance
```

Python으로 바꾸려면 `AVOIDANCE_IMPLEMENTATION=python`만 바꾼다. `static`, `crossing`, `mixed`도 같은 방식이다. 대기 시간은 `AVOIDANCE_START_DELAY`의 초 단위 값으로 조절한다.

`crossing`에서는 launch가 `linear_velocity=0.25 m/s`, `preferred_turn_direction=right`를 자동으로 덮어쓴다. `waffle_pi_3d_large`에서는 확대 collision에 맞춰 정지거리도 자동 확대한다.

## LiDAR–bbox association 실행

YOLO node가 `/detections`를 발행하는 상태에서 개발 shell로 실행한다.

```bash
cd /home/swlinux/Desktop/workspace/mobin/docker
docker compose -f compose.yaml -f compose.nvidia.yaml exec avoidance bash

source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash
ros2 run mobile_robot_lab_python lidar_bbox_association --ros-args \
  -p target_class:=cart \
  -p min_score:=0.5 \
  -p min_points:=3 \
  -p max_time_delta:=0.1
```

확인:

```bash
ros2 topic hz /detections
ros2 topic hz /fusion/associated_points
ros2 topic echo /fusion/detections_3d --once
```

RViz에서 `/calib/points`를 회색, `/fusion/associated_points`를 초록색으로 함께 표시한다. 초록 point가 cart bbox의 영상 영역에 대응해야 한다. **bbox 안에는 뒤쪽 선반 point도 들어올 수 있다.** 이 baseline의 정량 평가와 depth clustering 개선은 PATCH-13에서 수행한다.

## 실행 중 확인

별도 terminal:

```bash
cd /home/swlinux/Desktop/workspace/mobin/docker
docker compose exec avoidance bash
```

container 안:

```bash
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash

ros2 topic info /scan
ros2 topic info /cmd_vel
ros2 topic echo /cmd_vel --field twist --once
ros2 node info /obstacle_avoidance
```

정상 연결은 `/scan` publisher/subscription 각 1개, `/cmd_vel` publisher/subscription 각 1개다.

## 검증 결과

2026-08-23 실제 headless 실행 결과:

| 조합 | 관측 결과 |
|---|---|
| C++ + `waffle_pi_3d` + `static` | `scan_timeout → forward → turn_left → forward` |
| Python + `waffle_pi` + `mixed` | `forward → turn_left → forward` |
| moving cart plugin | 3초 사이 y pose 변화 확인 |
| 회피 Python unit tests | 3개 모두 통과 |

2026-08-26 `waffle_pi_3d_large + crossing + C++` 재검증 결과:

```text
avoidance mode: scan_timeout
avoidance mode: forward
avoidance mode: blocked
avoidance mode: forward
avoidance mode: blocked
avoidance mode: turn_right
avoidance mode: forward
```

사람+cart는 같은 model link의 pose `(0.9,-1.3)`에서 시작해 첫 경로를 +y로 함께 이동했다. Robot은 접근 중 오른쪽 sector가 점유되자 `blocked`로 정지했고, 통과 후 `turn_right`로 전환했다. 사람만 따로 이동하는 pedestrian 경로는 없다.

회전 방향 latch 전에는 C++ 로그가 0.3초 간격으로 좌우 전환했다. latch 반영 후 같은 정적 scenario에서 한 방향을 유지하고 `forward`로 복귀했다.

## 추가 구현하면 좋은 것

| 우선순위 | 항목 | 추가 시점 |
|---|---|---|
| 높음 | 최소 장애물 거리·collision·완주시간 metric node | CI에서 성공/실패를 자동 판정할 때 |
| 높음 | 3D point cloud 기반 높이 filter | 낮은 턱·상부 돌출물을 구분할 때 |
| 중간 | velocity를 포함한 TTC(Time To Collision) | 빠른 이동 obstacle 또는 고속 주행 전 |
| 중간 | 후진 recovery | 좌우가 모두 막힌 `blocked` 상태를 탈출할 때 |
| 이후 | Nav2 global planner와 costmap | 목표 pose까지 창고를 주행해야 할 때 |

현재 `blocked`는 안전하게 정지한다. 자동 후진은 뒤쪽 sensor와 후방 clearance를 검증하기 전 추가하지 않는다.

## 완료 조건

- Python/C++ package build 성공
- 단위검사 0 failure
- launch에서 둘 중 한 controller만 실행
- scan timeout 시 `(linear.x, angular.z)=(0, 0)`
- 정면 장애물에서 회전 mode 진입
- 장애물이 사라지면 `forward` 복귀
- 같은 시각의 detection과 point cloud에서 association topic 발행
- stale detection 또는 TF 누락 시 잘못된 3D 결과를 발행하지 않음
