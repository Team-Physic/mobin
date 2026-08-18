# Simulation PATCH-02: Calibration scene과 MCAP 기록

- 작성일: 2026-08-16
- 브랜치: `feature/scene_recording`
- 선행 조건: Simulation PATCH-01의 `waffle_pi_3d` build와 sensor topic 검증
- 대상: `docker/compose.yaml`, `forks/turtlebot3_simulations/turtlebot3_gazebo/`, `data/bags/`
- 결론: **서로 다른 geometry·camera 밝기·LiDAR intensity를 가진 정적 scene을 만들고, sensor extrinsic은 고정한 채 robot pose 5개에서 ROS 2 MCAP을 기록한다.**

### Why?

이 단계의 목적은 calibration 값을 계산하는 것이 아니라 **계산에 필요한 sensor data를 재현 가능하게 만드는 것**이다.

Calibration에서 Camera와 LiDAR의 물리적 장착 위치는 움직이지 않는다. 알고리즘은 LiDAR frame의 point를 camera optical frame으로 변환하는 translation 3개와 rotation 3개를 추정한다.

$$
p_C
=
T_{C \leftarrow L}p_L
=
R_{C \leftarrow L}p_L+t_{C \leftarrow L}
$$

| 기호 | 의미 |
|---|---|
| `p_L` | LiDAR frame에서 측정한 3D point |
| `T_{C \leftarrow L}` | LiDAR frame에서 camera optical frame으로 보내는 extrinsic |
| `R_{C \leftarrow L}` | 추정할 rotation 3-DoF |
| `t_{C \leftarrow L}` | 추정할 translation 3-DoF |
| `p_C` | camera optical frame으로 변환된 point |

변환된 `p_C`를 camera intrinsic으로 image의 `(u, v)` pixel에 투영한다. 여기서 투영은 3D point가 camera image의 어느 2D 위치에 보이는지 계산하는 작업이다.

`forks/direct_visual_lidar_calibration/src/vlcal/calib/cost_calculator_nid.cpp` | [`CostCalculatorNID::calculate()`](../../forks/direct_visual_lidar_calibration/src/vlcal/calib/cost_calculator_nid.cpp#L21)는 candidate extrinsic으로 LiDAR point를 camera frame으로 변환하고, 투영된 pixel의 밝기와 같은 point의 LiDAR intensity를 joint histogram에 누적한다.

```cpp
// forks/direct_visual_lidar_calibration/src/vlcal/calib/cost_calculator_nid.cpp | CostCalculatorNID::calculate()
// Candidate extrinsic이 LiDAR point를 camera optical frame으로 변환한다.
const Eigen::Vector4d pt_camera = T_camera_lidar * points->points[i];

// Camera frame의 3D point가 보이는 image pixel을 계산한다.
const Eigen::Array2i pt_2d = proj->project(pt_camera.head<3>()).cast<int>();

// 같은 sample에서 camera 밝기와 LiDAR intensity를 읽는다.
const double pixel = image.at<std::uint8_t>(pt_2d.y(), pt_2d.x()) / 255.0;
const double lidar_intensity = points->intensities[i];

// 두 sensor 값의 동시 분포를 joint histogram에 누적한다.
hist(image_bin, lidar_bin)++;

// Optimizer가 최소화할 NID를 반환한다.
const double NID = (Hrs - MI) / Hrs;
return NID;
```

NID는 Normalized Information Distance다. LiDAR intensity와 camera 밝기의 joint distribution이 candidate extrinsic에서 얼마나 일관적인지 나타내며, optimizer는 NID가 더 작은 extrinsic을 선택한다. **두 sensor 값이 숫자로 같아야 한다는 뜻은 아니다.**

평평한 단색 벽 하나만 기록하면 다음 정보가 부족하다.

| 부족한 장면 정보 | calibration에 생기는 문제 |
|---|---|
| 물체 거리가 거의 같음 | translation이 달라도 비슷한 pixel에 투영될 수 있음 |
| 모서리·곡면·기울기가 없음 | rotation 변화에 따른 투영 차이가 작음 |
| image 밝기가 거의 같음 | 다른 pixel로 이동해도 histogram이 비슷함 |
| LiDAR intensity가 거의 같음 | image와 비교할 intensity pattern이 부족함 |

따라서 가까운·중간·먼 물체, box·panel·cylinder, 여러 방향, 밝고 어두운 표면을 함께 둔다. **Target-less는 checkerboard를 검출하지 않는다는 뜻이지, 장면 설계가 필요 없다는 뜻이 아니다.**

Scene과 robot을 bag 기록 중 정지시키는 이유도 명확하다. 움직이는 물체나 움직이는 robot을 spinning LiDAR가 순차 측정하면 한 cloud 안에서도 point별 측정 시각이 달라진다. 이 PATCH는 motion compensation 변수를 제거하고 static extrinsic과 장면 품질만 먼저 검증한다.

## 1. LiDAR range와 intensity를 구분한다

### 개념

LiDAR는 하나의 반사 지점에 대해 위치와 반사 신호 세기를 함께 제공할 수 있다.

| LiDAR 값 | 실제 sensor의 측정 근거 | `PointCloud2`에서의 의미 |
|---|---|---|
| range | laser 왕복시간 | 발사 방향과 결합해 `x`, `y`, `z` 계산 |
| intensity | 되돌아온 laser 신호의 크기 | 같은 point의 `intensity` field |

실물 intensity는 재질, LiDAR 파장, 거리, 입사각, 표면 오염, sensor gain의 영향을 받는다. 가까운 물체라고 항상 intensity가 큰 것은 아니다. 제조사별 scale도 같지 않다.

Gazebo에서는 두 값이 다음처럼 나뉜다.

| Gazebo 설정 | 만드는 sensor 값 |
|---|---|
| `collision/geometry`와 ray 교차점 | range와 `x`, `y`, `z` |
| `collision/laser_retro` | SDFormat가 정의한 해당 collision의 LiDAR intensity |
| `visual/laser_retro` | Gazebo Harmonic `gpu_lidar` rendering 경로가 실제로 읽는 intensity |
| `visual/material` | camera image에 렌더링되는 색과 밝기 |

[SDFormat 1.8 `collision` 명세](https://sdformat.org/spec/1.8/collision/)는 `collision/laser_retro`를 laser sensor가 반환할 intensity 값으로 정의한다. 그러나 현재 사용하는 Gazebo Harmonic의 GPU LiDAR는 rendering scene의 visual을 측정한다. [`SdfEntityCreator::CreateEntities(const sdf::Visual*)`](https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/SdfEntityCreator.cc#L835-L857)는 `visual`에 `laser_retro`가 있을 때만 `LaserRetro` component를 만들고, [`RenderUtilPrivate::CreateVisual()`](https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/rendering/RenderUtil.cc#L3507-L3560)가 그 값을 rendering visual에 전달한다.

```cpp
// Gazebo Sim 8 | SdfEntityCreator::CreateEntities(const sdf::Visual*)
// visual에 값이 있어야 rendering entity에 LaserRetro component가 생긴다.
if (_visual->HasLaserRetro()) {
  this->dataPtr->ecm->CreateComponent(
    visualEntity, components::LaserRetro(_visual->LaserRetro()));
}

// Gazebo Sim 8 | RenderUtilPrivate::CreateVisual()
// GPU LiDAR가 보는 rendering visual로 값을 전달한다.
auto laserRetro = _ecm.Component<components::LaserRetro>(_entity);
if (laserRetro != nullptr) {
  visual.SetLaserRetro(laserRetro->Data());
}
```

따라서 이 scene은 같은 값을 `visual/laser_retro`와 `collision/laser_retro`에 모두 둔다. `collision` 값은 SDFormat 의미를 유지하고, `visual` 값은 Harmonic GPU rendering 경로에 실제 intensity를 전달한다. **`collision`에만 값을 두면 `/scan`과 `/calib/points`의 intensity가 모두 `0.0`이 될 수 있다.** `laser_retro`는 range를 바꾸지 않으며 물체를 LiDAR에서 숨기는 설정도 아니다.

이 실습의 `laser_retro=150~2000`은 실제 반사율 백분율이나 SI 단위가 아니다. 첫 smoke test에서 물체별 intensity 순서를 확실히 나누기 위한 simulator 입력값이다.

Raw intensity scale이 그대로 NID에 들어가는 것도 아니다. `forks/direct_visual_lidar_calibration/src/vlcal/preprocess/preprocess.cpp` | [`Preprocess::get_image_and_points()`](../../forks/direct_visual_lidar_calibration/src/vlcal/preprocess/preprocess.cpp#L406)는 point를 intensity 순서로 정렬한 뒤 256단계의 0 이상 1 미만 값으로 변환한다.

```cpp
// forks/direct_visual_lidar_calibration/src/vlcal/preprocess/preprocess.cpp | Preprocess::get_image_and_points()
// Raw intensity의 절대 scale 대신 point 사이의 intensity 순서를 사용한다.
std::sort(indices.begin(), indices.end(), [&](const int lhs, const int rhs) {
  return points->intensities[lhs] < points->intensities[rhs];
});

// 정렬 순위를 256단계의 0 이상 1 미만 값으로 변환한다.
const int bins = 256;
for (int i = 0; i < indices.size(); i++) {
  const double value = std::floor(bins * static_cast<double>(i) / indices.size()) / bins;
  points->intensities[indices[i]] = value;
}
```

따라서 이 scene에서는 raw 숫자의 절대 크기보다 **서로 다른 물체에서 intensity 변화가 실제로 발생하는지**가 중요하다.

Camera grayscale과 LiDAR intensity는 서로 다른 파장의 sensor 값이므로 실물에서는 밝고 어두운 순서도 다를 수 있다. 첫 simulation은 밝은 visual에 큰 `laser_retro`를 주어 대응을 쉽게 만들지만, 이 결과만으로 실제 환경 calibration 성능을 주장하지 않는다.

## 2. 시작 전에 source 상태를 확인한다

**이 시점에는 simulation을 아직 실행하지 않았으므로 topic을 조회하지 않는다.** `ros2 topic type`과 `ros2 topic echo`는 publisher가 실행 중일 때만 성공한다. Runtime topic 검사는 Section 9에서 `up -d sim`과 entity 생성을 확인한 뒤 수행한다.

먼저 host terminal에서 Simulation PATCH-01의 source 파일을 확인한다.

```bash
cd /home/swlinux/Desktop/workspace/mobin

ls -l forks/turtlebot3_simulations/turtlebot3_gazebo/models/turtlebot3_waffle_pi_3d/model.config
ls -l forks/turtlebot3_simulations/turtlebot3_gazebo/models/turtlebot3_waffle_pi_3d/model.sdf
ls -l forks/turtlebot3_simulations/turtlebot3_gazebo/params/turtlebot3_waffle_pi_3d_bridge.yaml
ls -l forks/turtlebot3_simulations/turtlebot3_gazebo/urdf/turtlebot3_waffle_pi_3d.urdf
grep -n 'calib/points' forks/turtlebot3_simulations/turtlebot3_gazebo/params/turtlebot3_waffle_pi_3d_bridge.yaml
```

| 정적 검사 | 통과 조건 |
|---|---|
| robot model manifest | `model.config` 존재 |
| robot SDF | `model.sdf` 존재 |
| ROS-Gazebo bridge | YAML에 `calib/points` mapping 존재 |
| robot description | `turtlebot3_waffle_pi_3d.urdf` 존재 |

`ls: cannot access .../model.config` 또는 `grep` 결과가 없으면 PATCH-02를 진행하지 않고 Simulation PATCH-01의 누락을 먼저 수정한다.

## 3. 생성·수정할 위치를 확인한다

```text
# patch/simulation/PATCH-02-calibration-scene-recording.md | planned file layout
/home/swlinux/Desktop/workspace/mobin/
├── .gitignore
├── data/
│   ├── bags/
│   └── results/
├── docker/
│   └── compose.yaml
└── forks/turtlebot3_simulations/turtlebot3_gazebo/
    ├── launch/
    │   ├── spawn_turtlebot3.launch.py
    │   └── turtlebot3_world.launch.py
    ├── models/
    │   └── calibration_scene/
    │       ├── model.config
    │       └── model.sdf
    └── worlds/
        └── turtlebot3_calibration.world
```

`turtlebot3_calibration.launch.py`는 새로 만들지 않는다. 기존 `turtlebot3_world.launch.py`가 이미 world를 parameter로 받으므로 `x_pose`, `y_pose`, `yaw`만 추가해 재사용한다.

Scene model의 경로는 다음 세 가지로 보인다.

| 구분 | 경로 | 수정 여부 |
|---|---|---|
| host 원본 | `/home/swlinux/Desktop/workspace/mobin/forks/turtlebot3_simulations/turtlebot3_gazebo/models/calibration_scene/` | 여기서 생성·수정 |
| container source | `/ws/src/turtlebot3_simulations/turtlebot3_gazebo/models/calibration_scene/` | bind mount된 같은 파일 |
| container install | `/ws/install/turtlebot3_gazebo/share/turtlebot3_gazebo/models/calibration_scene/` | build 결과이므로 직접 수정 금지 |

`docker/compose.yaml`의 `../forks/turtlebot3_simulations:/ws/src/turtlebot3_simulations` mount 때문에 host 원본과 container source는 같다. `CMakeLists.txt`가 `launch models params rviz urdf worlds` 전체를 install하므로 새 model과 world를 위한 CMake 수정은 필요 없다.

## 4. bag 저장 경로를 mount한다

host terminal:

```bash
cd /home/swlinux/Desktop/workspace/mobin
mkdir -p data/bags data/results
```

`docker/compose.yaml`의 공통 volume에 `../data:/ws/data:rw`가 있는지 확인한다. 현재 파일에 이미 있으면 중복 추가하지 않는다.

```yaml
# docker/compose.yaml | x-tb3-common.volumes
  volumes:
    - ../forks/turtlebot3_simulations:/ws/src/turtlebot3_simulations:rw
    - ../data:/ws/data:rw
    - tb3_build:/ws/build
    - tb3_install:/ws/install
    - tb3_log:/ws/log
    - /tmp/.X11-unix:/tmp/.X11-unix:rw
```

`.gitignore`에는 대용량 bag과 생성 결과를 제외한다.

```gitignore
# .gitignore | generated calibration data
data/bags/
data/results/*
!data/results/.gitkeep
```

## 5. calibration scene model을 만든다

host에서 정확한 디렉터리를 만든다.

```bash
cd /home/swlinux/Desktop/workspace/mobin
mkdir -p forks/turtlebot3_simulations/turtlebot3_gazebo/models/calibration_scene
```

1. `forks/turtlebot3_simulations/turtlebot3_gazebo/models/calibration_scene/model.config`
2. `forks/turtlebot3_simulations/turtlebot3_gazebo/models/calibration_scene/model.sdf`

`model.config`는 Gazebo가 model 이름과 실제 SDF 본문을 찾는 manifest다.

```xml
<!-- forks/turtlebot3_simulations/turtlebot3_gazebo/models/calibration_scene/model.config | Gazebo model manifest -->
<model>
  <name>calibration_scene</name>
  <version>1.0</version>
  <sdf version="1.8">model.sdf</sdf>
  <author>
    <name>Mobin practice</name>
  </author>
  <description>Static geometry and intensity scene for Camera-LiDAR calibration</description>
</model>
```

`model.sdf`는 7개 물체를 가진 하나의 static model이다. 아래 내용을 그대로 저장한다.

```xml
<!-- forks/turtlebot3_simulations/turtlebot3_gazebo/models/calibration_scene/model.sdf | calibration scene geometry -->
<sdf version="1.8">
  <model name="calibration_scene">
    <static>true</static>

    <link name="near_bright_box">
      <pose>2.0 -0.8 0.60 0 0 0.35</pose>
      <visual name="visual">
        <laser_retro>1800</laser_retro>
        <geometry>
          <box><size>0.45 0.45 1.20</size></box>
        </geometry>
        <material>
          <ambient>0.85 0.85 0.85 1</ambient>
          <diffuse>0.85 0.85 0.85 1</diffuse>
        </material>
      </visual>
      <collision name="collision">
        <laser_retro>1800</laser_retro>
        <geometry>
          <box><size>0.45 0.45 1.20</size></box>
        </geometry>
      </collision>
    </link>

    <link name="near_dark_box">
      <pose>2.3 0.75 0.45 0 0 -0.40</pose>
      <visual name="visual">
        <laser_retro>250</laser_retro>
        <geometry>
          <box><size>0.35 0.70 0.90</size></box>
        </geometry>
        <material>
          <ambient>0.15 0.15 0.15 1</ambient>
          <diffuse>0.15 0.15 0.15 1</diffuse>
        </material>
      </visual>
      <collision name="collision">
        <laser_retro>250</laser_retro>
        <geometry>
          <box><size>0.35 0.70 0.90</size></box>
        </geometry>
      </collision>
    </link>

    <link name="mid_gray_panel">
      <pose>3.4 0.10 0.75 0 0 0.15</pose>
      <visual name="visual">
        <laser_retro>900</laser_retro>
        <geometry>
          <box><size>0.08 2.20 1.50</size></box>
        </geometry>
        <material>
          <ambient>0.50 0.50 0.50 1</ambient>
          <diffuse>0.50 0.50 0.50 1</diffuse>
        </material>
      </visual>
      <collision name="collision">
        <laser_retro>900</laser_retro>
        <geometry>
          <box><size>0.08 2.20 1.50</size></box>
        </geometry>
      </collision>
    </link>

    <link name="mid_bright_cylinder">
      <pose>3.0 -1.20 0.65 0 0 0</pose>
      <visual name="visual">
        <laser_retro>1500</laser_retro>
        <geometry>
          <cylinder>
            <radius>0.25</radius>
            <length>1.30</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0.75 0.75 0.75 1</ambient>
          <diffuse>0.75 0.75 0.75 1</diffuse>
        </material>
      </visual>
      <collision name="collision">
        <laser_retro>1500</laser_retro>
        <geometry>
          <cylinder>
            <radius>0.25</radius>
            <length>1.30</length>
          </cylinder>
        </geometry>
      </collision>
    </link>

    <link name="mid_dark_cylinder">
      <pose>3.8 1.10 0.40 0 0 0</pose>
      <visual name="visual">
        <laser_retro>350</laser_retro>
        <geometry>
          <cylinder>
            <radius>0.18</radius>
            <length>0.80</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0.20 0.20 0.20 1</ambient>
          <diffuse>0.20 0.20 0.20 1</diffuse>
        </material>
      </visual>
      <collision name="collision">
        <laser_retro>350</laser_retro>
        <geometry>
          <cylinder>
            <radius>0.18</radius>
            <length>0.80</length>
          </cylinder>
        </geometry>
      </collision>
    </link>

    <link name="far_bright_panel">
      <pose>5.2 -0.70 1.00 0 0 -0.20</pose>
      <visual name="visual">
        <laser_retro>2000</laser_retro>
        <geometry>
          <box><size>0.08 1.60 2.00</size></box>
        </geometry>
        <material>
          <ambient>0.90 0.90 0.90 1</ambient>
          <diffuse>0.90 0.90 0.90 1</diffuse>
        </material>
      </visual>
      <collision name="collision">
        <laser_retro>2000</laser_retro>
        <geometry>
          <box><size>0.08 1.60 2.00</size></box>
        </geometry>
      </collision>
    </link>

    <link name="far_dark_panel">
      <pose>5.8 0.95 0.70 0 0 0.25</pose>
      <visual name="visual">
        <laser_retro>150</laser_retro>
        <geometry>
          <box><size>0.08 1.50 1.40</size></box>
        </geometry>
        <material>
          <ambient>0.10 0.10 0.10 1</ambient>
          <diffuse>0.10 0.10 0.10 1</diffuse>
        </material>
      </visual>
      <collision name="collision">
        <laser_retro>150</laser_retro>
        <geometry>
          <box><size>0.08 1.50 1.40</size></box>
        </geometry>
      </collision>
    </link>
  </model>
</sdf>
```

각 `link/pose`는 scene model frame 기준 `x y z roll pitch yaw`다. 모든 물체를 robot 전방 `x > 0`에 두고 높이·거리·yaw를 다르게 했다. `visual`과 `collision`의 geometry가 같으므로 camera가 보는 외곽과 LiDAR가 측정하는 외곽이 일치한다.

## 6. calibration world를 만든다

host에서 upstream empty world를 복사한다.

```bash
cd /home/swlinux/Desktop/workspace/mobin
cp forks/turtlebot3_simulations/turtlebot3_gazebo/worlds/empty_world.world \
   forks/turtlebot3_simulations/turtlebot3_gazebo/worlds/turtlebot3_calibration.world
```

새 `turtlebot3_calibration.world`에서 Sun `<include>` 다음에 calibration model을 추가한다.

```xml
<!-- forks/turtlebot3_simulations/turtlebot3_gazebo/worlds/turtlebot3_calibration.world | calibration model include -->
    <include>
      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Sun</uri>
    </include>

    <include>
      <uri>model://calibration_scene</uri>
    </include>
```

기존 `<scene>...</scene>`을 새로 하나 더 추가하지 말고 다음 내용으로 교체한다.

```xml
<!-- forks/turtlebot3_simulations/turtlebot3_gazebo/worlds/turtlebot3_calibration.world | fixed rendering scene -->
    <scene>
      <ambient>0.35 0.35 0.35 1</ambient>
      <background>0.05 0.05 0.05 1</background>
      <shadows>true</shadows>
    </scene>
```

Actor, 움직이는 plugin, wind, 자동으로 변하는 light는 추가하지 않는다.

## 7. 기존 launch에 robot pose를 추가한다

새 calibration 전용 launch를 복사하지 않는다. 기존 launch 두 개에 `yaw`와 pose 전달만 추가한다.

### 개념

| 값 | 기준과 단위 | 바뀌는 것 | 바뀌지 않는 것 |
|---|---|---|---|
| `x_pose`, `y_pose` | world frame, m | world 안의 robot 위치 | robot이 바라보는 방향, Camera–LiDAR의 상대 위치·각도 |
| `yaw` | world의 `+Z`축, rad | robot 전체가 바라보는 방향 | Camera–LiDAR의 상대 위치·각도 |
| Camera–LiDAR extrinsic | LiDAR frame에서 Camera optical frame으로 가는 변환 | 이 단계에서는 바꾸지 않음 | 기록하는 모든 bag에서 동일해야 함 |

`x_pose`와 `y_pose`만 바꾸면 robot이 옆이나 앞뒤로 이동하지만 바라보는 방향은 그대로다. `yaw`를 바꾸면 robot 본체와 본체에 고정된 Camera·LiDAR가 **함께 회전**한다. 그러면 같은 calibration scene에서도 다른 panel, 경계선, 거리 변화가 두 sensor의 공통 시야에 들어온다. 여러 방향에서 얻은 관측은 한 방향의 관측만 사용할 때보다 하나의 Camera–LiDAR extrinsic을 더 분명하게 제약한다.

예를 들어 `yaw=0.18`은 robot을 world의 `+Z`축 주위로 약 `10.3°` 회전한 상태로 생성한다. 위에서 내려다볼 때 양수는 반시계 방향이다. `ros_gz_sim create`의 `-Y`가 이 **robot 초기 방향**을 지정한다. Camera link나 LiDAR link를 robot에 고정하는 URDF/SDF pose는 수정하지 않는다.

따라서 다섯 번 기록할 때 달라지는 값은 robot의 world pose이고, 찾으려는 Camera–LiDAR extrinsic은 모두 같다. `yaw`는 calibration 결과에 임의의 회전을 더하는 값이 아니다. 현재 기록 계획에 `0.10`, `-0.10`, `0.18`, `-0.18 rad` 방향이 포함되므로 launch argument가 필요하다. 모든 기록을 `yaw=0`에서 수행한다면 이 argument는 생략할 수 있지만 해당 관측 방향들은 재현할 수 없다.

### `spawn_turtlebot3.launch.py`

`forks/turtlebot3_simulations/turtlebot3_gazebo/launch/spawn_turtlebot3.launch.py` | `generate_launch_description()`에서 `x_pose`와 `y_pose` 다음에 `yaw`를 정의한다.

```python
# forks/turtlebot3_simulations/turtlebot3_gazebo/launch/spawn_turtlebot3.launch.py | generate_launch_description()
x_pose = LaunchConfiguration('x_pose', default='0.0')
y_pose = LaunchConfiguration('y_pose', default='0.0')
yaw = LaunchConfiguration('yaw', default='0.0')

declare_yaw_cmd = DeclareLaunchArgument(
    'yaw',
    default_value='0.0',
    description='Initial robot yaw in radians')
```

`ros_gz_sim create` argument의 `-z` 앞에 yaw를 전달한다.

```python
# forks/turtlebot3_simulations/turtlebot3_gazebo/launch/spawn_turtlebot3.launch.py | generate_launch_description()
arguments=[
    '-name', TURTLEBOT3_MODEL,
    '-file', urdf_path,
    '-x', x_pose,
    '-y', y_pose,
    '-Y', yaw,
    '-z', '0.01'
],
```

`LaunchDescription`에 declaration을 추가한다.

```python
# forks/turtlebot3_simulations/turtlebot3_gazebo/launch/spawn_turtlebot3.launch.py | generate_launch_description()
ld.add_action(declare_x_position_cmd)
ld.add_action(declare_y_position_cmd)
ld.add_action(declare_yaw_cmd)
```

### `turtlebot3_world.launch.py`

`forks/turtlebot3_simulations/turtlebot3_gazebo/launch/turtlebot3_world.launch.py` | `generate_launch_description()`의 기존 pose 정의에 yaw를 추가한다.

```python
# forks/turtlebot3_simulations/turtlebot3_gazebo/launch/turtlebot3_world.launch.py | generate_launch_description()
x_pose = LaunchConfiguration('x_pose', default='-2.0')
y_pose = LaunchConfiguration('y_pose', default='-0.5')
yaw = LaunchConfiguration('yaw', default='0.0')
```

Spawn include에 세 pose 값을 모두 전달한다.

```python
# forks/turtlebot3_simulations/turtlebot3_gazebo/launch/turtlebot3_world.launch.py | generate_launch_description()
launch_arguments={
    'x_pose': x_pose,
    'y_pose': y_pose,
    'yaw': yaw,
}.items()
```

`ld.add_action(set_env_vars_resources)`보다 앞에서 top-level argument를 선언한다.

```python
# forks/turtlebot3_simulations/turtlebot3_gazebo/launch/turtlebot3_world.launch.py | generate_launch_description()
ld.add_action(DeclareLaunchArgument('x_pose', default_value='-2.0'))
ld.add_action(DeclareLaunchArgument('y_pose', default_value='-0.5'))
ld.add_action(DeclareLaunchArgument('yaw', default_value='0.0'))
```

기존 world, GUI, RViz argument와 기본값은 변경하지 않는다.

### `docker/compose.yaml`

Compose에서 pose를 선택할 수 있도록 공통 environment에 세 값을 추가한다.

```yaml
# docker/compose.yaml | x-tb3-common.environment
    X_POSE: ${X_POSE:--2.0}
    Y_POSE: ${Y_POSE:--0.5}
    YAW: ${YAW:-0.0}
```

`services.sim.command`의 launch 명령 끝에 pose를 전달한다.

```yaml
# docker/compose.yaml | services.sim.command
exec ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py world:=$${TURTLEBOT3_WORLD} x_pose:=$${X_POSE} y_pose:=$${Y_POSE} yaw:=$${YAW} gazebo_gui:=$${GAZEBO_GUI} launch_rviz:=$${LAUNCH_RVIZ}
```

모든 launch argument를 `ros2 launch`와 같은 shell command line에 둔다. 줄바꿈하면 `world:=...`부터 별도 명령으로 실행되어 launch에는 기본값만 전달된다. `exec`는 `sim` container가 종료 signal을 ROS launch에 직접 전달하게 한다. Compose가 값을 먼저 치환하지 않도록 container 변수 앞에는 `$$`를 사용한다.

## 8. source를 build한다

host terminal에서 NVIDIA override를 포함한 development shell을 일회성으로 실행한다.

```bash
cd /home/swlinux/Desktop/workspace/mobin/docker

TURTLEBOT3_MODEL=waffle_pi_3d \
docker compose \
  -f compose.yaml \
  -f compose.nvidia.yaml \
  run --rm shell bash -lc '
    source /opt/ros/jazzy/setup.bash &&
    cd /ws &&
    colcon build --symlink-install --packages-select turtlebot3_gazebo
  '
```

`colcon build`는 source 파일을 수정하는 명령이 아니다. `turtlebot3_gazebo`의 Python launch, model, world를 `/ws/install/turtlebot3_gazebo/share/turtlebot3_gazebo/`에 설치해 ROS 2와 Gazebo가 찾게 한다.

설치 결과를 확인한다.

```bash
docker compose \
  -f compose.yaml \
  -f compose.nvidia.yaml \
  run --rm shell bash -lc '
    test -f /ws/install/turtlebot3_gazebo/share/turtlebot3_gazebo/models/calibration_scene/model.sdf &&
    test -f /ws/install/turtlebot3_gazebo/share/turtlebot3_gazebo/worlds/turtlebot3_calibration.world
  '
```

## 9. 첫 pose에서 simulation을 실행한다

### Terminal A: sim 시작

```bash
cd /home/swlinux/Desktop/workspace/mobin/docker
xhost +si:localuser:root

TURTLEBOT3_MODEL=waffle_pi_3d \
TURTLEBOT3_WORLD=turtlebot3_calibration.world \
X_POSE=0.0 \
Y_POSE=0.0 \
YAW=0.0 \
GAZEBO_GUI=false \
LAUNCH_RVIZ=true \
docker compose \
  -f compose.yaml \
  -f compose.nvidia.yaml \
  up --force-recreate sim

docker compose \
  -f compose.yaml \
  -f compose.nvidia.yaml \
  logs -f sim
```

`up sim`이 먼저 실행되어야 `exec sim bash`로 같은 running container에 들어갈 수 있다. Log에서 world server 시작과 robot entity 생성 성공을 확인한 뒤 `Ctrl-C`로 log follow만 종료한다. `-d`로 실행했으므로 container는 계속 동작한다.

### Terminal B: running sim 안에서 sensor 확인

```bash
cd /home/swlinux/Desktop/workspace/mobin/docker

docker compose \
  -f compose.yaml \
  -f compose.nvidia.yaml \
  exec sim bash
```

Container 안:

```bash
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash

ros2 node list
ros2 topic list -t | grep -E 'calib/points|camera/image_raw|camera/camera_info'

ros2 topic type /calib/points
ros2 topic hz /calib/points
ros2 topic hz /camera/image_raw
ros2 topic echo /calib/points --field fields --once
ros2 topic echo /scan --field intensities --once
ros2 topic echo /camera/image_raw --field header --once
ros2 topic echo /camera/camera_info --field header --once
ros2 topic echo /camera/camera_info --field distortion_model --once
```

| GUI·topic 검사 | 통과 조건 |
|---|---|
| topic type | `/calib/points`가 `sensor_msgs/msg/PointCloud2` |
| point fields | `x`, `y`, `z`, `intensity` 존재 |
| LiDAR intensity | `/scan.intensities`에 `0.0`이 아닌 값과 둘 이상의 값이 존재 |
| camera topics | `Image`와 `CameraInfo`가 각각 1회 이상 수신됨 |
| camera frame | 두 camera message가 `camera_rgb_optical_frame` 사용 |
| distortion model | `plumb_bob` 또는 calibration tool이 지원하는 model |
| Gazebo | 7개 물체와 robot이 보이고 물체가 바닥 위에 있음 |
| RViz image | 가까운·중간·먼 물체와 밝기 차이가 보임 |
| RViz PointCloud2 | 위아래 여러 scan line과 서로 다른 intensity가 보임 |
| Camera·LiDAR 공통 시야 | 같은 box·panel·cylinder가 image와 cloud 양쪽에 존재 |

RViz PointCloud2 display의 Color Transformer를 `Intensity`로 설정한다. 모든 point가 같은 색이면 bag을 기록하지 않는다.

`plumb_bob`은 일반 렌즈의 방사·접선 왜곡을 표현한다. `fisheye`는 어안·초광각 model이다. 실제 `CameraInfo.distortion_model`을 확인한 뒤 Simulation PATCH-03의 calibration option과 동일하게 사용한다.

## 10. 5개 정적 pose에서 MCAP을 기록한다

Robot pose를 바꾸면 장면을 보는 방향과 깊이 구성이 달라지지만 Camera-LiDAR fixed joint는 변하지 않는다. 여러 view는 하나의 view에서 애매했던 translation·rotation을 추가로 제한한다.

| bag | `X_POSE` [m] | `Y_POSE` [m] | `YAW` [rad] |
|---|---:|---:|---:|
| `pose-01` | 0.00 | 0.00 | 0.00 |
| `pose-02` | 0.20 | -0.35 | 0.10 |
| `pose-03` | -0.15 | 0.35 | -0.10 |
| `pose-04` | 0.45 | 0.15 | 0.18 |
| `pose-05` | 0.35 | -0.20 | -0.18 |

이 반복은 host의 `scripts/record-calibration-poses.sh`가 수행한다.

| 자동 수행 단계 | 동작 |
|---|---|
| 기존 출력 검사 | `pose-01`~`pose-05` 중 하나라도 있으면 기록 전에 중단 |
| simulation 재생성 | 표의 `X_POSE`, `Y_POSE`, `YAW`를 전달해 `sim` 재생성 |
| sensor 준비 확인 | 각 topic type이 ROS graph에 나타날 때까지 최대 60초 재시도한 뒤 message를 실제로 1회 수신 |
| MCAP 기록 | 기본 15초 동안 6개 topic 기록 |
| 정상 종료 | `timeout`이 `SIGINT`를 보내 MCAP metadata 작성 기회 제공 |
| 결과 검사 | 각 bag의 `metadata.yaml` 존재 확인 |
| 자원 정리 | 완료·실패·사용자 중단 시 `sim` 정지 |

Host script 자체는 `set -euo pipefail`을 사용한다. 다만 container 안에서 ROS setup을 source하는 block은 `set -eo pipefail`만 사용한다. `/opt/ros/jazzy/setup.bash`가 선택적 `AMENT_TRACE_SETUP_FILES`를 미정의 상태로 조회할 수 있어 `set -u`를 함께 쓰면 `unbound variable`로 중단되기 때문이다.

Container 생성 직후에는 bridge와 sensor가 아직 ROS graph에 등록되지 않아 `ros2 topic echo`가 `Could not determine the type for the passed topic`으로 즉시 종료될 수 있다. Script는 `ros2 topic type`을 1초 간격으로 재시도해 type discovery가 끝난 뒤 `ros2 topic echo --once`를 실행한다.

핵심 pose 반복과 기록 명령은 다음과 같다.

```bash
# scripts/record-calibration-poses.sh | POSES, record_pose()
POSES=(
  "pose-01 0.00 0.00 0.00"
  "pose-02 0.20 -0.35 0.10"
  "pose-03 -0.15 0.35 -0.10"
  "pose-04 0.45 0.15 0.18"
  "pose-05 0.35 -0.20 -0.18"
)

# 지정 시간이 지나면 SIGINT로 종료해 MCAP index와 metadata를 마무리한다.
timeout --signal=INT --kill-after=10s "${RECORD_SECONDS}s" \
  ros2 bag record --storage mcap \
  -o "/ws/data/bags/${bag_name}" \
  /calib/points /camera/image_raw /camera/camera_info \
  /tf /tf_static /clock
```

ROS 2 Jazzy의 기본 storage가 MCAP이어도 `--storage mcap`을 명시해 다른 ROS 2 환경에서 형식이 바뀌지 않게 한다. 기존 bag은 자동 삭제하거나 덮어쓰지 않는다.

먼저 실행 권한과 도움말을 확인한다.

```bash
cd /home/swlinux/Desktop/workspace/mobin
chmod +x scripts/record-calibration-poses.sh
./scripts/record-calibration-poses.sh --help
```

기본 실행은 pose마다 15초 기록하며 Gazebo GUI와 RViz를 열지 않는다.

```bash
./scripts/record-calibration-poses.sh
```

첫 번째 argument로 pose별 기록 시간을 바꿀 수 있다. 계획 범위의 최댓값인 20초로 기록하는 예:

```bash
./scripts/record-calibration-poses.sh 20
```

기록 중 화면도 확인하려면 X11 접근을 허용하고 두 환경변수를 켠다.

```bash
xhost +si:localuser:root
GAZEBO_GUI=true LAUNCH_RVIZ=true \
  ./scripts/record-calibration-poses.sh 15
xhost -si:localuser:root
```

완료 후 `data/bags/pose-01`부터 `pose-05`까지 생성된다. 기존 경로가 있으면 script가 시작 전에 중단하므로, 보존할 bag은 직접 다른 이름으로 이동한 뒤 다시 실행한다.

## 11. 각 bag의 품질을 확인한다

Running sim container 또는 development shell:

```bash
ros2 bag info /ws/data/bags/pose-01
find /ws/data/bags/pose-01 -maxdepth 1 -type f -name '*.mcap' -print
```

| 검사 | 통과 조건 |
|---|---|
| Storage id | `mcap` |
| `/calib/points` | `sensor_msgs/msg/PointCloud2`, count > 0 |
| `/camera/image_raw` | `sensor_msgs/msg/Image`, count > 0 |
| `/camera/camera_info` | `sensor_msgs/msg/CameraInfo`, count > 0 |
| `/tf`, `/tf_static` | frame transform message 존재 |
| `/clock` | simulation time message 존재 |
| duration | 계획한 12~20초 범위 |

Bag을 재생할 때 running simulation의 같은 topic과 섞지 않는다. 먼저 sim service를 정지한다.

```bash
cd /home/swlinux/Desktop/workspace/mobin/docker
docker compose \
  -f compose.yaml \
  -f compose.nvidia.yaml \
  stop sim

docker compose \
  -f compose.yaml \
  -f compose.nvidia.yaml \
  run --rm shell bash -lc '
    source /opt/ros/jazzy/setup.bash &&
    source /ws/install/setup.bash &&
    ros2 bag play /ws/data/bags/pose-01 --clock
  '
```

## 12. Rerun에서 MCAP을 확인한다

[Rerun MCAP 공식 문서](https://rerun.io/docs/howto/logging-and-ingestion/mcap)에 따르면 Rerun은 ROS 2 MCAP을 직접 열고 common ROS 2 message를 시각화할 수 있다. Rerun에는 bag 디렉터리가 아니라 그 안의 `.mcap` 파일을 전달한다.

먼저 현재 image에 Rerun이 실제 설치되어 있는지 확인한다.

```bash
cd /home/swlinux/Desktop/workspace/mobin/docker
docker compose run --rm shell rerun --version
```

`rerun: command not found`이면 실행 중인 container에서 임시 설치하지 않는다. [Simulation PATCH-00](PATCH-00-jazzy-gazebo-docker-setup.md)의 Rerun 설치가 포함된 `docker/sim/Dockerfile`로 `docker compose build`를 다시 실행한다.

Rerun이 설치되어 있으면 host에서 실제 MCAP 파일명을 확인한다.

```bash
find /home/swlinux/Desktop/workspace/mobin/data/bags/pose-01 \
  -maxdepth 1 -type f -name '*.mcap' -print
```

Compose가 `../data`를 `/ws/data`로 이미 mount하므로 추가 volume은 필요 없다. 기본 Compose의 native Viewer는 `llvmpipe`에서 필요한 `R32Float` render target을 지원하지 않아 종료될 수 있다. **컨테이너는 Rerun server만 실행하고 host browser가 WebGL로 rendering하는 Web Viewer를 사용한다.**

먼저 빈 Web Viewer server가 기동되는지 확인한다. 이 명령은 종료하지 말고 그대로 둔다.

```bash
cd /home/swlinux/Desktop/workspace/mobin/docker
docker compose run --rm shell \
  rerun --web-viewer --renderer=webgl
```

host browser에서 다음 주소를 연다. terminal에 query parameter가 포함된 전체 접속 주소가 출력되면 그 주소를 사용한다.

```text
http://127.0.0.1:9090
```

실제 파일명에 맞춰 MCAP을 바로 열 때는 `scripts/normalize_rerun_tf.py`로 TF frame의 `/` prefix를 정리한 `.rrd`를 만든 뒤 Web Viewer로 연다.

개념:

| 용어 | 의미 |
|---|---|
| `/` prefix mismatch | Rerun MCAP importer는 TF의 `child_frame`/`parent_frame`을 `/base_scan`처럼 저장하지만 sensor header의 `frame_id`는 `base_scan`으로 저장한다. 같은 frame이 문자열이 달라 transform path가 끊어진다. |
| normalized RRD | TF의 앞 `/`를 제거하고 root `/`를 `odom` frame으로 연결한 Rerun recording |
| Web Viewer URL | `--web-viewer-port`가 viewer page, `--port`가 data server. `?url=...`는 viewer가 data server를 바라보는 주소 |

```bash
cd /home/swlinux/Desktop/workspace/mobin/docker

docker compose \
  -f compose.yaml \
  -f compose.nvidia.yaml \
  run --rm \
  -v "$PWD/../scripts:/ws/scripts:ro" \
  shell bash -lc '
    mkdir -p /ws/data/rerun
    for mcap in /ws/data/bags/pose-*/*_0.mcap; do
      pose="$(basename "$(dirname "$mcap")")"
      python3 /ws/scripts/normalize_rerun_tf.py \
        "$mcap" "/ws/data/rerun/${pose}_normalized.rrd"
    done
  '

docker compose \
  -f compose.yaml \
  -f compose.nvidia.yaml \
  run --rm shell \
  rerun --web-viewer --renderer=webgl \
  --web-viewer-port 9095 --port 9880 \
  /ws/data/rerun
```

host browser에서 다음 주소를 연다. terminal 출력의 query parameter가 다르면 terminal 값을 우선한다.

```text
http://127.0.0.1:9095?url=rerun%2Bhttp%3A%2F%2Flocalhost%3A9880%2Fproxy&renderer=webgl
```

| Rerun 검사 | 확인할 사실 |
|---|---|
| timeline | Camera와 LiDAR message가 같은 ROS 2 시간축에 나타남 |
| Camera | `/camera/image_raw`가 image로 표시됨 |
| CameraInfo | camera intrinsic message를 조회할 수 있음 |
| 3D LiDAR | `/calib/points`가 3D point로 표시됨 |
| TF | Camera와 LiDAR frame 관계가 기록 중 변하지 않음 |

같은 timeline에 보인다는 사실은 exact timestamp equality를 의미하지 않는다. Rerun은 sensor message의 시간 순서와 차이를 찾는 debugging 도구다. 허용 timestamp 차이와 pair 선택은 Simulation PATCH-03에서 수치로 검증한다.

Robot mesh가 없어도 sensor data 검사는 가능하다. Mesh가 필요할 때만 `/robot_description` 기록 또는 Rerun URDF importer를 추가한다.

## 13. 완료 조건

- host의 `models/calibration_scene/model.config`와 `model.sdf`가 fork 안에 존재
- `turtlebot3_calibration.world`가 `model://calibration_scene`을 include
- 기존 launch 하나가 world와 `x_pose`, `y_pose`, `yaw`를 모두 전달
- `colcon build --packages-select turtlebot3_gazebo` 성공
- Camera와 3D LiDAR가 가까운·중간·먼 geometry를 공통으로 관측
- `/calib/points`에 `x`, `y`, `z`, `intensity`가 있고 intensity 변화가 보임
- `pose-01`부터 `pose-05`까지 서로 다른 robot pose의 MCAP 존재
- 각 bag에 Image, CameraInfo, PointCloud2, TF, clock message 존재
- bag 기록 동안 scene, robot, Camera-LiDAR fixed joint가 정지
- `data/bags/`가 Git status에 나타나지 않음

```bash
cd /home/swlinux/Desktop/workspace/mobin
git -C forks/turtlebot3_simulations status --short
git status --short
```

Fork status에는 새 model·world와 launch 수정만 나타나야 한다. 상위 repository status에는 `data/bags/`가 나타나지 않아야 한다.

GUI 검사가 끝나면 host에서 container를 종료하고 X11 허용을 회수한다.

```bash
cd /home/swlinux/Desktop/workspace/mobin/docker
docker compose -f compose.yaml -f compose.nvidia.yaml down
xhost -si:localuser:root
```

## 14. 실패할 때 확인한다

### `topic does not appear to be published yet`

Section 9의 `up -d sim`보다 먼저 topic 명령을 실행했다면 정상적인 경고다. Topic은 설정 파일에 적혀 있다는 이유만으로 생기지 않으며, Gazebo sensor와 ROS bridge publisher가 실행되어야 ROS graph에 나타난다.

Simulation 실행 뒤에도 topic이 없으면 host에서 service와 log를 확인한다.

```bash
cd /home/swlinux/Desktop/workspace/mobin/docker
docker compose -f compose.yaml -f compose.nvidia.yaml ps
docker compose -f compose.yaml -f compose.nvidia.yaml logs sim
```

확인 순서:

1. `sim` service가 `running`인가
2. world server가 시작됐는가
3. `waffle_pi_3d` entity creation이 성공했는가
4. `parameter_bridge`와 `image_bridge`가 종료되지 않았는가
5. bridge YAML의 `calib/points`와 SDF sensor topic 이름이 같은가

이 조건을 통과한 뒤 container에서 `ros2 topic list -t`를 다시 실행한다.

### `model://calibration_scene`을 찾지 못한다

```bash
docker compose \
  -f compose.yaml \
  -f compose.nvidia.yaml \
  run --rm shell bash -lc '
    source /opt/ros/jazzy/setup.bash &&
    source /ws/install/setup.bash &&
    printenv GZ_SIM_RESOURCE_PATH &&
    find /ws/install/turtlebot3_gazebo/share/turtlebot3_gazebo/models/calibration_scene -maxdepth 1 -type f -print
  '
```

Install 경로에 파일이 없으면 source만 만들고 `colcon build`를 다시 하지 않은 상태다. 파일은 있는데 model을 못 찾으면 launch가 `GZ_SIM_RESOURCE_PATH`에 package의 `models` 경로를 추가했는지 확인한다.

### Camera와 cloud가 다른 물체를 본다

Robot spawn yaw, Camera optical axis, LiDAR horizontal FoV를 확인한다. Camera-LiDAR fixed joint를 pose마다 수정하지 않는다. Scene 물체 위치 또는 robot의 전체 spawn pose만 바꾼다.

### 모든 point intensity가 같다

다음 세 경계를 순서대로 확인한다.

1. `model.sdf`의 같은 `laser_retro` 값이 각 `visual`과 `collision`의 직접 자식인가
2. `ros2 topic echo /scan --field intensities --once`에 `0.0`이 아닌 값이 있는가
3. ROS bridge 뒤 `/calib/points`의 `intensity` field에도 변화가 있는가

`/calib/points`에 `intensity` field가 존재하는 것만으로는 통과가 아니다. 실제 값의 최소·최대가 같으면 RViz `Color Transformer=Intensity`에서도 모든 point가 같은 색이다. `laser_retro`를 바꿔도 range와 point 위치는 그대로인 것이 정상이다.

### Rerun에 image만 보이고 point cloud가 없다

`rerun mcap info --full <file.mcap>`로 `/calib/points` schema가 decode되는지 확인한다. Rerun importer가 해당 PointCloud2 field layout을 지원하지 않으면 MCAP 기록 실패로 판단하지 말고 RViz replay 결과와 구분한다.

## 이 PATCH에서 하지 않는 것

- NID optimizer 실행
- extrinsic 결과를 URDF fixed joint에 반영
- 움직이는 robot의 point-level motion compensation
- camera auto-exposure와 실제 LiDAR noise의 물리 모델링
- MCAP 또는 result 자동 삭제

Extrinsic 계산은 Simulation PATCH-03, URDF 반영과 오차 평가는 Simulation PATCH-04에서 수행한다.
