# Simulation PATCH-05: AWS Warehouse와 장애물 scenario

- 상태: **구현·headless 검증 완료**
- 기준: `aws-robomaker-small-warehouse-world@ee0af733315e78432408c3cd98d378ecee5f767c`
- 실행 환경: ROS 2 Jazzy, Gazebo Harmonic 8

## 결론

AWS Small Warehouse의 ROS 2 world를 Gazebo Harmonic에서 여는 이식본을 만들었다. 같은 world에 정적 box, 통로를 가로지르는 이동 cart, 두 장애물을 함께 넣는 세 scenario를 추가했다.

**Calibration 결과와 무관하게 실행 가능하다.** PATCH-05는 LiDAR–Camera 투영이 아니라 `/scan` 기반 주행 환경을 검증한다.

## What I Made

| 결과 | 현재 구현 |
|---|---|
| 창고 world | `small_warehouse_harmonic.world` |
| 정적 장애물 | 0.45 × 0.45 × 0.60 m box |
| 이동 장애물 | AWS pallet jack와 CC0 사람 mesh를 한 link로 묶어 simulation time 기준 왕복 |
| warehouse robot | 원본의 visual·collision·joint·sensor pose를 3배로 만든 `waffle_pi_3d_large` |
| scenario | `static`, `crossing`, `mixed` |
| 실행 진입점 | `turtlebot3_avoidance.launch.py`와 Compose `avoidance` service |
| 초기 대기 | Gazebo·sensor는 먼저 시작하고 controller는 기본 10초 후 시작 |
| 초기 GUI 시점 | robot과 moving cart 첫 waypoint `(0.9, -1.3)`이 함께 보이는 넓은 구도 |
| cart 시작 대기 | camera 밖 `(-50,-50)`에 생성한 뒤 10초 후 첫 waypoint로 이동해 20 Hz로 왕복 |
| 렌더링 선택 | CPU, `/dev/dri`, NVIDIA overlay 모두 `avoidance`에 전달 |

## Why?

기본 TurtleBot3 world만으로도 node 동작은 볼 수 있다. 그러나 창고는 선반, 벽, 좁은 통로, 시야 가림이 함께 있어 다음 단계의 회피·dataset·domain randomization 기준 환경으로 쓰기 좋다.

원본 world는 ROS 2 branch여도 Gazebo Classic 형식과 asset 경로가 남아 있었다. **ROS 2 호환과 Gazebo Harmonic 호환은 같은 뜻이 아니다.** Harmonic system plugin, resource path, physics collision을 별도로 맞춰야 했다.

## What was problem

| 증상 | 원인 | 수정 |
|---|---|---|
| `model://...` 해석 실패 | world와 model 검색 root가 모두 필요 | `GZ_SIM_RESOURCE_PATH`에 fork root와 `models/` 추가 |
| model load 오류 | static roof/ground에 불필요한 inertial | 해당 inertial 제거 |
| `/scan`이 전부 `inf` | 강제로 지정한 Bullet Featherstone과 sensor scene 조합 | 기본 Harmonic physics engine 사용 |
| `/odom`은 변하지만 실제 model은 고정 | 복잡한 ground mesh collision과 `mu=100` | visual mesh는 유지하고 평평한 box collision 사용 |
| 이동 장애물 속도가 PC 부하에 영향 | wall-clock 기반 이동 가능성 | `UpdateInfo.simTime`으로 이동 계산 |
| NVIDIA overlay가 `avoidance`에 미적용 | 새 service가 overlay에 없음 | DRI/NVIDIA compose 파일에 service 추가 |
| GUI가 열리자마자 robot 이동 | controller node를 world와 동시 시작 | `TimerAction`으로 기본 10초 지연 |
| 초기 화면에 moving cart가 안 보임 | 제거된 `GzScene3D` compatibility plugin은 `camera_pose`를 보장하지 않음 | Harmonic의 `MinimalScene.camera_pose`와 `GzSceneManager` 사용 |
| 대기 중 cart가 미리 노출됨 | 첫 waypoint에서 10초간 정지 | camera 밖 staging 좌표에서 기다리고 이동 시작 순간 통로에 진입 |
| moving cart가 파란 box로만 보임 | 동작 검증용 기본 box visual 사용 | warehouse fork의 pallet jack visual mesh 연결 |
| 상세 collision mesh 사용 시 cart가 바닥 아래로 내려감 | 정적 asset용 non-convex mesh를 동적 model collision에 사용 | pallet jack 전체 범위를 감싸는 단순 box collision 사용 |
| pose command 이동 중 cart 높이가 내려감 | 경로 pose와 중력이 동시에 적용 | kinematic obstacle link의 gravity 비활성화 |
| VS Code에서 DAE가 안 보임 | extension의 COLLADA·texture 해석이 Gazebo와 다름 | 빈 Gazebo world에 원본 `model.sdf`만 spawn해 확인 |
| entity는 있으나 cart mesh가 안 보임 | 대기 중에도 physics 1 kHz마다 같은 `WorldPoseCmd` 발행 | 대기 중 pose command를 보내지 않고 이동 갱신을 20 Hz로 제한 |
| server에는 cart가 있지만 GUI에 없음 | GUI scene 준비 전 3초에 dynamic model 생성 | obstacle spawn을 world 시작 8초 후로 지연 |
| 사람 DAE를 actor로 만들면 skeleton 경고 | `Casual female`은 보행 skeleton이 없는 정적 mesh | actor를 쓰지 않고 cart와 같은 link의 visual·collision으로 결합 |
| cart가 경로 옆을 보며 미끄러짐 | pose의 회전을 identity로 고정 | 현재 segment 방향으로 yaw 계산 |

## How it changed

```text
forks/aws-robomaker-small-warehouse-world/
├── worlds/small_warehouse/small_warehouse_harmonic.world
└── models/
    ├── aws_robomaker_warehouse_PalletJackB_01/
    ├── aws_robomaker_warehouse_GroundB_01/model.sdf
    └── aws_robomaker_warehouse_RoofB_01/model.sdf

forks/turtlebot3_simulations/turtlebot3_gazebo/
├── include/turtlebot3_gazebo/warehouse_obstacle.hpp
├── src/warehouse_obstacle.cpp
├── models/warehouse_obstacles/
│   ├── static_box/
│   └── moving_cart/
│       └── casual_female/  # 원본 CC0 asset과 SOURCE.md
├── launch/turtlebot3_avoidance.launch.py
└── launch/turtlebot3_world.launch.py

code/scripts/generate-scaled-tb3.py
```

`small_warehouse_harmonic.world`의 GUI camera는 `(x,y,z)=(-3.0,-5.0,3.0) m`, yaw `1.107 rad`, pitch `0.490 rad`에서 robot과 cart 통로를 함께 본다. 이것은 robot sensor camera가 아니라 **Gazebo 작업 화면의 초기 시점**만 변경한다. Cart는 camera 밖 `(-50,-50)`에 생성되어 10초간 기다린 뒤, plugin이 첫 waypoint `(0.9,-1.3)`로 옮기는 순간부터 화면에 나타난다.

## 핵심 코드

| 파일 위치 | 함수 또는 설정 | 변경 요약 |
|---|---|---|
| `forks/turtlebot3_simulations/turtlebot3_gazebo/src/warehouse_obstacle.cpp` | [WarehouseObstaclePlugin::Configure()](../../forks/turtlebot3_simulations/turtlebot3_gazebo/src/warehouse_obstacle.cpp#L18) | 이전: plugin 설정 없음<br>변경: speed·waypoint를 읽고 path 검증<br>효과: 잘못된 이동 경로 거부 |
| `forks/turtlebot3_simulations/turtlebot3_gazebo/src/warehouse_obstacle.cpp` | [WarehouseObstaclePlugin::PreUpdate()](../../forks/turtlebot3_simulations/turtlebot3_gazebo/src/warehouse_obstacle.cpp#L56) | 10초 대기 후 20 Hz 이동, segment 방향 yaw 적용 |
| `forks/turtlebot3_simulations/turtlebot3_gazebo/models/warehouse_obstacles/moving_cart/model.sdf` | [cart와 operator visual](../../forks/turtlebot3_simulations/turtlebot3_gazebo/models/warehouse_obstacles/moving_cart/model.sdf) | 이전: pallet jack만 표시<br>변경: CC0 사람 visual·collision을 같은 link에 결합<br>효과: 사람과 cart가 벌어지지 않고 함께 이동 |
| `forks/turtlebot3_simulations/turtlebot3_gazebo/launch/turtlebot3_avoidance.launch.py` | [launch_setup()](../../forks/turtlebot3_simulations/turtlebot3_gazebo/launch/turtlebot3_avoidance.launch.py#L28) | 이전: cart가 첫 waypoint에서 대기하며 camera에 노출<br>변경: obstacle은 8초에 `(-50,-50)`에 생성, controller는 `controller_delay` 후 생성<br>효과: cart가 이동을 시작하기 전에는 camera에 나타나지 않음 |
| `forks/turtlebot3_simulations/turtlebot3_gazebo/launch/turtlebot3_world.launch.py` | [generate_launch_description()](../../forks/turtlebot3_simulations/turtlebot3_gazebo/launch/turtlebot3_world.launch.py#L35) | 이전: package world만 선택<br>변경: `world_dir/world` 인자 수용<br>효과: AWS warehouse 경로 실행 |
| `forks/aws-robomaker-small-warehouse-world/worlds/small_warehouse/small_warehouse_harmonic.world` | [MinimalScene.camera_pose](../../forks/aws-robomaker-small-warehouse-world/worlds/small_warehouse/small_warehouse_harmonic.world#L258) | 이전: 제거된 `GzScene3D` 사용<br>변경: Harmonic의 `MinimalScene`에서 moving cart 첫 waypoint를 향한 camera pose 설정<br>효과: compatibility 변환 없이 초기 시점 적용 |
| `docker/compose.yaml` | [services.avoidance](../../docker/compose.yaml#L58) | 이전: 지연 값 전달 없음<br>변경: `AVOIDANCE_START_DELAY` 전달<br>효과: Compose에서 대기 시간 조정 |
| `code/scripts/generate-scaled-tb3.py` | [main()](../../code/scripts/generate-scaled-tb3.py) | `waffle_pi_3d`의 SDF·URDF를 3배 변환하고 large bridge 설정 복사 |

이동 거리는 실제 경과 시간이 아니라 Gazebo simulation time으로 계산한다.

```cpp
// forks/turtlebot3_simulations/turtlebot3_gazebo/src/warehouse_obstacle.cpp | WarehouseObstaclePlugin::PreUpdate()
const std::chrono::duration<double> elapsed = info.simTime - start_time_;
// spawn pose가 첫 waypoint이므로 대기 중에는 visual pose를 덮어쓰지 않는다.
if (elapsed.count() < start_delay_) {
  return;
}

// physics는 1 kHz지만 GUI와 obstacle pose는 20 Hz만 갱신한다.
const std::chrono::duration<double> since_update =
  info.simTime - last_pose_update_;
if (last_pose_update_ != std::chrono::steady_clock::duration::zero() &&
  since_update.count() < 0.05)
{
  return;
}
last_pose_update_ = info.simTime;

const double moving_seconds = elapsed.count() - start_delay_;
const double travel = std::fmod(moving_seconds * speed_, total_distance_);

// 현재 선분 안의 비율로 두 waypoint 사이를 보간한다.
const double ratio = (travel - accumulated) / segment_distances_[segment];
const auto direction = waypoints_[segment + 1] - waypoints_[segment];
const auto position = waypoints_[segment] + direction * ratio;
const double yaw = std::atan2(direction.Y(), direction.X());
model_.SetWorldPoseCmd(
  ecm, gz::math::Pose3d(position, gz::math::Quaterniond(0.0, 0.0, yaw)));
```

따라서 PC가 잠시 느려져도 simulation 속도 기준 cart 경로는 동일하다.

## Scenario 정의

| `AVOIDANCE_SCENARIO` | 생성 model | 목적 |
|---|---|---|
| `static` | 정면 x=1.2 m의 box | 정지거리와 좌·우 회전 확인 |
| `crossing` | 사람+cart가 x=0.9 m의 초록 통로를 +y로 이동 | 약 20초에 robot 진행선과 교차하는 하나의 운반 조합 회피 |
| `mixed` | y=0.45 m box + 왕복 cart | 부분 폐색과 동적 교차 동시 확인 |

이동 cart의 SDF 값은 다음과 같다.

```xml
<!-- models/warehouse_obstacles/moving_cart/model.sdf -->
<collision name="collision">
  <pose>0.07 -0.015 0.49 0 0 0</pose>
  <geometry><box><size>1.16 0.54 0.96</size></box></geometry>
</collision>
<gravity>false</gravity>
<visual name="visual">
  <geometry>
    <mesh>
      <uri>model://aws_robomaker_warehouse_PalletJackB_01/meshes/aws_robomaker_warehouse_PalletJackB_01_visual.DAE</uri>
    </mesh>
  </geometry>
</visual>
<visual name="cart_operator_visual">
  <pose>-0.95 0 0.02 0.04 0 1.5708</pose>
  <geometry>
    <mesh><uri>casual_female/meshes/casual_female.dae</uri></mesh>
  </geometry>
</visual>
<speed>0.35</speed>
<start_delay>10.0</start_delay>
<waypoint>0.9 -1.3 0.023</waypoint>
<waypoint>0.9 2.5 0.023</waypoint>
<waypoint>0.9 -1.3 0.023</waypoint>
```

사람은 독립된 pedestrian entity가 아니다. 사람 mesh와 cart mesh가 **같은 `link`에 속하므로 하나의 pose를 공유하며 항상 함께 이동한다.** `crossing`은 이 결합 model 전체가 robot 진행선을 가로지르는 scenario 이름이다.

Cart entity는 launch 약 8초에 camera 밖 `(-50,-50)`에 생성된다. SDF의 `start_delay=10.0`이 끝나는 약 18초에 plugin이 첫 waypoint `(0.9,-1.3)`로 옮기고 곧바로 +y 이동을 시작한다. `y=-1.3 → -0.5 m`를 `0.35 m/s`로 이동하므로 robot 진행선 도착은 약 20.3초다. `crossing` controller는 약 10초에 출발해 `x=-2.0 → 0.9 m`를 `0.25 m/s`로 접근한다.

`model://` 뒤 이름은 `GZ_SIM_RESOURCE_PATH`에 등록된 model directory에서 찾는다. Compose는 `/opt/aws_warehouse/models`를 이미 등록하므로 host 절대경로를 SDF에 넣지 않는다. `z=0.023 m`는 원본 warehouse world의 pallet jack 배치 높이와 같다. Collision은 visual의 약 `1.16 × 0.54 × 0.96 m` 범위를 감싸는 box다. 상세 mesh는 외형에만 사용한다. 이 cart는 충돌 힘으로 움직이는 물체가 아니라 plugin이 경로 pose를 지정하는 장애물이므로 link의 gravity를 끈다.

## License와 출처

| 범위 | license | 처리 |
|---|---|---|
| AWS warehouse 원본 | MIT-0 | fork의 `LICENSE` 보존, 원본 commit과 변경 사실을 Harmonic world 주석에 기록 |
| TurtleBot3 fork | Apache-2.0 | 기존 header 보존, 새 plugin·launch에 Apache-2.0 표시 |
| moving cart SDF·plugin | Apache-2.0 package 내부 | 이동 logic과 SDF 변경 파일의 license 유지 |
| pallet jack visual DAE·texture | AWS warehouse의 MIT-0 | asset을 복사하지 않고 AWS fork의 원본 경로를 참조하며 `LICENSE` 보존 |
| 사람 visual DAE·texture | OpenRobotics `Casual female` v4, CC0-1.0 | 원본 전체와 [`SOURCE.md`](../../forks/turtlebot3_simulations/turtlebot3_gazebo/models/warehouse_obstacles/moving_cart/casual_female/SOURCE.md) 보존 |

MIT-0 asset 수정·재배포는 가능하다. 다만 원본 저작권·license 파일을 지우거나, Harmonic 이식본을 원본 그대로라고 표시하면 안 된다.

### Fuel 외 asset도 사용할 수 있는가

**Gazebo Fuel 전용 asset만 사용할 필요는 없다.** SDF가 읽을 수 있는 DAE·OBJ·STL 등으로 변환하고 texture URI를 맞추면 직접 제작 CAD, Unity Asset Store, Fab/Unreal Marketplace 출처 mesh도 기술적으로 불러올 수 있다. 문제는 파일 형식보다 **각 asset의 license**다.

| 출처 | Gazebo 변환 | public GitHub 포함 판단 |
|---|---|---|
| 직접 제작·CC0·호환 CC-BY | 가능 | license·출처 조건을 지키면 가장 단순 |
| Fab Standard License | 호환 tool 사용 가능 | asset 원본을 독립 파일로 재배포하면 안 됨. 구매자가 별도 확보하게 하는 편이 안전 |
| Unity Asset Store 일반 asset | 변환 자체는 가능 | Licensed Product 안의 포함과 원본 재배포는 다름. provider·Restricted Asset 조건도 확인 |
| Unreal Engine 전용·reference-only·Restricted Asset | 조건부 또는 불가 | Gazebo 사용·재배포 권한을 확인하기 전 repository에 넣지 않음 |

근거: [Fab Standard License](https://www.fab.com/eula?lang=en)는 호환 tool 사용을 허용하지만 asset의 standalone 재배포를 금지한다. [Unity Asset Store Terms](https://unity.com/legal/as-terms)도 일반 asset을 제품에 포함하는 사용과 asset 자체 재배포를 구분하며, provider별 제한이 추가될 수 있다.

Fuel에서 작업복 착용 사람을 검색했지만 목적에 맞고 license가 명확한 model을 찾지 못했다. 따라서 사용자가 제시한 [OpenRobotics Casual female](https://app.gazebosim.org/OpenRobotics/fuel/models/Casual%20female) v4를 사용했다. Fuel metadata의 license는 CC0-1.0이며 archive SHA-256은 `e03944df8f010c7d6cf85fe61b2a9bf2899eed971e73fb76461f4705edeffe5d`다.

이 DAE에는 보행 skeleton이 없다. 별도 Gazebo actor로 만들면 `Mesh skeleton ... not found`가 발생한다. 현재 구현은 사람 visual·단순 box collision을 pallet-jack link에 고정해 **손잡이 위치에서 함께 이동**시킨다. 실제 보행 관절·손 접촉 animation은 아니다.

## Gazebo 빈 world에서 model만 확인

`.sdf`는 visual, collision, pose와 외부 mesh URI를 조합한다. VS Code DAE viewer는 mesh 하나만 읽고 Gazebo의 `model://` 검색 규칙이나 SDF 설정을 적용하지 않는다. 따라서 **실제 simulation에 표시될 model은 Gazebo에서 확인한다.** [Gazebo Harmonic Model Insertion](https://gazebosim.org/docs/harmonic/fuel_insert/)의 local model 절차와 같이 model root를 `GZ_SIM_RESOURCE_PATH`에 등록하고 빈 world에 spawn한다.

이 프로젝트의 Compose 환경에는 다음 값이 이미 설정돼 있다.

```text
GZ_SIM_RESOURCE_PATH=/opt/aws_warehouse:/opt/aws_warehouse/models
```

첫 terminal에서 빈 Gazebo world를 연다.

```bash
cd /home/swlinux/Desktop/workspace/mobin/docker
xhost +si:localuser:root

docker compose \
  -f compose.yaml \
  -f compose.nvidia.yaml \
  run --rm shell \
  gz sim --force-version 8 -v 4 empty.sdf
```

첫 명령을 실행한 채 두 번째 terminal에서 pallet jack SDF만 spawn한다.

```bash
cd /home/swlinux/Desktop/workspace/mobin/docker

docker compose \
  -f compose.yaml \
  -f compose.nvidia.yaml \
  run --rm shell bash -lc '
    source /opt/ros/jazzy/setup.bash
    ros2 run ros_gz_sim create \
      --world empty \
      --name preview_pallet_jack \
      --file /opt/aws_warehouse/models/aws_robomaker_warehouse_PalletJackB_01/model.sdf
  '
```

확인할 항목은 다음과 같다.

| 확인 대상 | 정상 결과 |
|---|---|
| visual | pallet jack 손잡이·fork·wheel이 표시 |
| texture | 회색·검정 계열 material이 mesh에 적용 |
| collision | Gazebo GUI의 `View → Collisions`에서 visual과 비슷한 범위 표시 |
| terminal | mesh URI 또는 texture를 찾지 못했다는 error 없음 |

이 검사는 **정적 원본 asset의 외형·resource path만 분리 검증**한다. 왕복 이동과 LiDAR 회피는 아래 `crossing` scenario로 검증한다.

## Build

이미지는 한 번 빌드하고 ROS package는 shared colcon volume에 빌드한다.

```bash
cd /home/swlinux/Desktop/workspace/mobin/docker
docker compose build

docker compose run --rm shell bash -lc '
  source /opt/ros/jazzy/setup.bash
  cd /ws
  colcon build --symlink-install \
    --packages-select turtlebot3_gazebo
'
```

`docker compose build`는 OS·ROS·Rerun dependency image를 만든다. `colcon build`는 bind mount된 TurtleBot3 source와 `warehouse_obstacle` shared library를 `/ws/install`에 만든다.

## 실행

NVIDIA GPU 사용:

아래 값은 `docker/compose.yaml`의 `avoidance` service가 launch argument 또는 container 환경변수로 전달한다.

| 환경변수 | 사용할 수 있는 값 | 설정 시 얻는 것 |
|---|---|---|
| `TURTLEBOT3_MODEL` | `burger`, `burger_cam`, `waffle`, `waffle_pi`, `waffle_pi_3d`, `waffle_pi_3d_large` | Calibration은 원본 3D, warehouse 가시성은 large 선택 |
| `TURTLEBOT3_WORLD_DIR` | container 안의 world directory | package 밖 AWS warehouse world 선택 |
| `TURTLEBOT3_WORLD` | directory 안의 `.world` 파일명 | 실행할 world 결정 |
| `AVOIDANCE_SCENARIO` | `static`, `crossing`, `mixed` | box, moving cart, 두 장애물 동시 생성 중 선택 |
| `AVOIDANCE_IMPLEMENTATION` | `python`, `cpp` | `/cmd_vel` controller 하나 선택 |
| `AVOIDANCE_START_DELAY` | `0.0` 이상의 초 | robot controller 시작 지연. GUI·sensor 준비 시간 확보 |
| `X_POSE`, `Y_POSE` | meter 실수 | robot의 world 초기 x·y 위치 |
| `YAW` | radian 실수 | robot의 world 초기 진행 방향 |
| `GAZEBO_GUI` | `true`, `false` | Gazebo 3D 창 표시 여부 |
| `LAUNCH_RVIZ` | `true`, `false` | RViz 동시 실행 여부 |

| Compose 선택 | hardware | 얻는 것 |
|---|---|---|
| `-f compose.yaml -f compose.nvidia.yaml` | NVIDIA GPU와 Container Toolkit | Gazebo·RViz hardware rendering, 향후 ONNX/TensorRT용 GPU 전달 |
| `-f compose.yaml -f compose.dri.yaml` | Intel/AMD `/dev/dri` | host Mesa GPU rendering |
| `-f compose.yaml` | GPU 전달 없음 | `LIBGL_ALWAYS_SOFTWARE=1` CPU rendering |

```bash
cd /home/swlinux/Desktop/workspace/mobin/docker
xhost +si:localuser:root

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

GPU 없이 headless CPU 검증:

```bash
GAZEBO_GUI=false LAUNCH_RVIZ=false \
TURTLEBOT3_MODEL=waffle_pi_3d_large \
TURTLEBOT3_WORLD_DIR=/opt/aws_warehouse/worlds/small_warehouse \
TURTLEBOT3_WORLD=small_warehouse_harmonic.world \
AVOIDANCE_SCENARIO=static \
AVOIDANCE_IMPLEMENTATION=cpp \
AVOIDANCE_START_DELAY=0.0 \
X_POSE=0.0 Y_POSE=0.0 YAW=0.0 \
docker compose up --force-recreate avoidance
```

## 검증 결과

2026-08-23 로컬 headless 실행에서 확인했다.

| 검사 | 결과 |
|---|---|
| Harmonic server | `World [default] initialized` |
| model URI | Error Code 14 없음 |
| isolated pallet jack preview | 빈 `empty.sdf`에 `preview_pallet_jack` entity 생성 성공 |
| static obstacle | entity creation 성공 |
| moving cart 생성 직후 | pose `(-50.0,-50.0,0.023) m`; camera 밖 staging 확인 |
| moving cart delay 만료 후 | pose `(0.918,1.015,0.021) m`; 통로에서 +y 이동 확인 |
| `waffle_pi_3d` 정적 회피 | `scan_timeout → forward → turn_left → forward` |

cart pose 확인:

```bash
docker compose exec avoidance bash -lc '
  source /opt/ros/jazzy/setup.bash
  source /ws/install/setup.bash
  gz model -m avoidance_moving_cart --pose
'
```

`AVOIDANCE_START_DELAY`는 robot controller만 늦춘다. 사람+cart는 별도로 camera 밖에 생성된 뒤 `moving_cart/model.sdf`의 `start_delay`만큼 기다린다.

2026-08-25 headless 검증에서 `AVOIDANCE_START_DELAY=6.0`으로 실행했다. delay 만료 전에 `obstacle_avoidance_cpp` process가 없었고, 만료 후 process 생성과 `scan_timeout → forward` 로그를 확인했다.

같은 날 pallet jack 적용 후 상세 collision mesh 사용 시 z가 내려가는 문제를 재현했다. 단순 box collision과 `gravity=false`로 수정한 뒤 당시 경로에서 `y=-1.0404 → -0.2102 m` 동안 `z=0.023 m`를 유지했고 asset 관련 Gazebo error가 없었다. 이후 가시 영역 경로로 옮기고 host GUI에서 texture와 구도를 직접 확인했다.

2026-08-26 사람과 pallet-jack를 같은 link로 유지하면서 crossing 경로를 `(0.9,-1.3) → (0.9,2.5)`로 옮겼다. 둘은 camera 밖에서 10초 대기한 뒤 첫 waypoint에 나타나 사진의 초록 통로 방향인 +y로 함께 움직인다.

같은 날 headless pose 검사에서 생성 직후 `(-50.0,-50.0,0.023) m`, delay 만료 후 `(0.918,1.015,0.021) m`를 확인했다. 첫 값은 camera 밖 staging, 두 번째 값은 통로의 +y 이동 구간이다. GUI pixel 검출이 아니라 Gazebo world pose로 검증한 결과다.

같은 날 warehouse 전용 `waffle_pi_3d_large`를 3배로 생성해 다시 build했다. `/scan`, `/calib/points`, `/camera/image_raw`가 발행됐고 `base_link → camera_rgb_optical_frame` translation은 원본의 3배인 `(0.228, 0.000, 0.279) m`였다. 즉 Gazebo geometry만 커진 것이 아니라 ROS TF도 같은 배율이다.

## 추가 구현하면 좋은 것

| 우선순위 | 항목 | 추가 시점 |
|---|---|---|
| 높음 | contact sensor 또는 collision count 자동 판정 | 회피 성공을 로그가 아닌 수치로 CI 검사할 때 |
| 높음 | episode reset와 random seed | PATCH-10 dataset을 반복 수집할 때 |
| 중간 | 보행 skeleton이 있는 사람 actor | 발걸음·손 접촉 animation까지 평가할 때 |
| 중간 | warehouse map/SLAM 정합 | 목표점 주행과 Nav2를 추가할 때 |

현재 plugin은 `SetWorldPoseCmd()`로 cart와 사람을 하나의 rigid model처럼 이동한다. 경로·LiDAR collision 재현에는 충분하지만 사람의 보행 관절, 손 접촉, 자연스러운 가속도는 모델링하지 않는다.

## 완료 조건

- `colcon build --packages-select turtlebot3_gazebo` 성공
- 세 scenario 값이 잘못되면 launch가 즉시 오류 출력
- static/moving entity creation 성공
- cart pose가 simulation time에 따라 변함
- cart 이동 중 z가 `0.023 m`로 유지
- `/scan`이 유한 거리를 포함하고 controller가 회전 mode에 진입
- delay 만료 전에 controller process가 없고, 만료 후에만 생성
- `crossing` 또는 `mixed` 실행 시 이동 전 사람+cart가 camera에 보이지 않음
- delay 만료 후 첫 waypoint에 나타나 사람+cart가 함께 +y로 이동
