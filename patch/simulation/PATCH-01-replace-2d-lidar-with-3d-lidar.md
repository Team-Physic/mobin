# Simulation PATCH-01: 기존 2D LiDAR를 3D LiDAR로 교체

## 이 PATCH에서 만드는 것

원본 `turtlebot3_waffle_pi`는 그대로 둔다. 복사한 `turtlebot3_waffle_pi_3d`에서 **기존 `base_scan` LiDAR의 측정 방식을 한 줄짜리 2D scan에서 위아래 여러 줄을 측정하는 3D scan으로 교체**한다.

새 LiDAR link를 하나 더 만들지 않는다. 다음 항목을 그대로 재사용한다.

| 유지할 항목 | 이유 |
|---|---|
| `base_scan` link와 `scan_joint` | 기존 LiDAR의 설치 위치와 TF를 그대로 사용 |
| `base_scan` visual·collision | Gazebo와 RViz에서 센서 외형을 계속 표시 |
| Gazebo topic `scan` | 교체 전후의 sensor topic 기준을 하나로 유지 |
| ROS topic `/calib/points` | Calibration 입력 이름을 다른 PATCH와 통일 |

```text
파생 모델 waffle_pi_3d
  base_link
     └─ scan_joint (fixed)
          └─ base_scan
               └─ 기존 LiDAR sensor 설정을 3D scan으로 교체
                    └─ Gazebo scan/points
                           └─ ros_gz_bridge
                                └─ ROS /calib/points (PointCloud2)
```

원본 `waffle_pi`에는 기존 2D LiDAR가 남는다. 교체 모델이 잘못되거나 기존 장애물 회피와 비교해야 할 때 즉시 원본으로 돌아갈 수 있다.

## 개념

| 용어 | 의미 | 이 PATCH에서의 사용 |
|---|---|---|
| 2D LiDAR | 한 높이를 한 줄로 훑어 거리 배열을 생성 | 원본 `waffle_pi`의 `/scan` |
| 3D LiDAR | 가로뿐 아니라 위아래 여러 방향을 훑어 XYZ point를 생성 | 파생 `waffle_pi_3d`의 `/calib/points` |
| `gpu_lidar` | Gazebo의 LiDAR simulation 방식 이름. 실제 제품명이나 3D LiDAR라는 뜻은 아님 | 기존 sensor `type`을 유지하고 `<vertical>` 설정으로 3D 측정 구성 |
| `PointCloud2` | XYZ와 intensity 같은 field를 가진 여러 point를 전달하는 ROS 2 message | Calibration 프로그램의 LiDAR 입력 |
| `base_scan` | LiDAR 측정값의 원점과 축을 나타내는 좌표계 | 교체 뒤에도 LiDAR message의 `frame_id`로 사용 |
| Extrinsic | LiDAR 좌표를 Camera 좌표로 바꾸는 위치 3개와 회전 3개 | Calibration이 구하는 `T_camera_lidar` |

**2D LiDAR로도 Camera–LiDAR extrinsic calibration은 가능하다.** 다만 checkerboard나 평면 표적, 여러 자세의 측정, 2D LiDAR 전용 algorithm이 보통 필요하다. 이 실습의 `direct_visual_lidar_calibration`은 intensity가 있는 3D `PointCloud2`를 입력으로 사용하므로 3D 측정으로 교체한다.

### NID가 하는 일

LiDAR intensity와 Camera pixel 밝기는 서로 다른 값이므로 숫자가 같을 필요가 없다. Calibration은 candidate extrinsic으로 LiDAR point를 Camera image 위의 pixel 위치에 대응시킨 뒤, 두 값의 통계적 관계를 NID로 평가한다. **NID가 작아지는 위치와 회전이 더 잘 맞는 candidate**다.

```cpp
// forks/direct_visual_lidar_calibration/src/vlcal/calib/cost_calculator_nid.cpp
// CostCalculatorNID::calculate()

// Candidate extrinsic으로 LiDAR point를 Camera 좌표계로 변환한다.
const Eigen::Vector4d pt_camera = T_camera_lidar * points->points[i];

// Camera 앞의 3D point가 영상의 어느 pixel에 보이는지 계산한다.
const Eigen::Array2i pt_2d = proj->project(pt_camera.head<3>()).cast<int>();

// 해당 pixel 밝기와 LiDAR intensity의 조합을 histogram에 누적한다.
hist(image_bin, lidar_bin)++;

// 현재 candidate extrinsic의 NID cost를 반환한다.
const double NID = (Hrs - MI) / Hrs;
return NID;
```

`T_camera_lidar`가 틀리면 LiDAR point가 엉뚱한 pixel에 놓여 NID가 커진다. 맞는 값에 가까워지면 같은 벽·모서리·물체에서 얻은 intensity와 image brightness의 통계적 대응이 강해져 NID가 작아진다.

### 참고 : PCL은 무엇이고 언제 확인하는가

PCL(Point Cloud Library)은 point cloud를 읽고, 변환하고, 필터링하고, 정합하는 C++ library다. **LiDAR sensor를 동작시키는 driver도 아니고, ROS topic 자체도 아니다.**

| 상태 | 판단 |
|---|---|
| `/calib/points`가 없거나 type이 다름 | Gazebo sensor 또는 bridge 문제. PCL 단계 전 |
| `PointCloud2`에 `x`, `y`, `z`가 없음 | 올바른 point cloud가 아님 |
| point 수가 0이거나 NaN만 존재 | sensor 범위, world, 변환 문제 |
| `intensity`가 없음 | 현재 NID calibration 입력 조건을 만족하지 않음 |
| point가 보이고 filtering 결과도 합리적 | PCL 처리 경로가 정상일 가능성이 높음 |
| point가 겹쳐 보이지만 두 frame을 합치면 어긋남 | PCL 고장보다 TF·timestamp·extrinsic을 먼저 의심 |

이 PATCH에서는 별도 PCL filter node를 만들지 않는다. 먼저 Gazebo와 bridge가 유효한 `PointCloud2`를 만드는지만 검증한다.

## 시작 조건

- Simulation PATCH-00 Docker image와 Compose 구성이 준비되어 있다.
- `forks/turtlebot3_simulations`가 `jazzy` 기반 실습 branch에 있다.
- 원본 모델 파일에 의도하지 않은 수정이 없다.

호스트에서 확인한다.

```bash
cd /home/swlinux/Desktop/workspace/mobin
git -C forks/turtlebot3_simulations status --short --branch
git -C forks/turtlebot3_simulations rev-parse HEAD
```

실습 branch가 아직 없다면 다음처럼 만든다.

```bash
git -C forks/turtlebot3_simulations switch -c practice/replace-lidar-with-3d
```

## 변경할 파일

```text
forks/turtlebot3_simulations/turtlebot3_gazebo/
├── models/turtlebot3_waffle_pi_3d/
│   ├── model.config
│   └── model.sdf
├── params/turtlebot3_waffle_pi_3d_bridge.yaml
├── rviz/tb3_waffle_pi_3d.rviz
└── urdf/turtlebot3_waffle_pi_3d.urdf
```

## 1. 원본에서 파생 모델을 복사한다

```bash
cd /home/swlinux/Desktop/workspace/mobin/forks/turtlebot3_simulations/turtlebot3_gazebo

mkdir -p models/turtlebot3_waffle_pi_3d
cp models/turtlebot3_waffle_pi/model.sdf \
  models/turtlebot3_waffle_pi_3d/model.sdf
cp urdf/turtlebot3_waffle_pi.urdf \
  urdf/turtlebot3_waffle_pi_3d.urdf
cp params/turtlebot3_waffle_pi_bridge.yaml \
  params/turtlebot3_waffle_pi_3d_bridge.yaml
cp rviz/tb3_gazebo.rviz \
  rviz/tb3_waffle_pi_3d.rviz
```

`models/turtlebot3_waffle_pi_3d/model.config`를 만든다. **XML declaration은 반드시 첫 줄**이어야 한다. License 설명은 그 아래에 둔다.

```xml
<?xml version="1.0"?>
<!--
  Derived from ROBOTIS turtlebot3_waffle_pi.
  Modified for an unofficial 3D LiDAR calibration practice model.
  SPDX-License-Identifier: Apache-2.0
-->
<model>
  <name>TurtleBot3(Waffle Pi 3D)</name>
  <version>1.0</version>
  <sdf version="1.8">model.sdf</sdf>
  <author>
    <name>ROBOTIS and practice repository contributors</name>
  </author>
  <description>Waffle Pi whose existing LiDAR is configured as a 3D scanner</description>
</model>
```

복사한 SDF의 model 이름을 바꾼다.

```xml
<model name="turtlebot3_waffle_pi_3d">
```

복사한 URDF의 robot 이름을 바꾼다.

```xml
<robot name="turtlebot3_waffle_pi_3d" xmlns:xacro="http://ros.org/wiki/xacro">
```

복사한 파일의 기존 copyright·license header는 지우지 않는다. 새 attribution comment를 URDF에 넣는다면 XML declaration 다음에 둔다.

## 2. 기존 `base_scan` sensor를 3D 측정으로 교체한다

`models/turtlebot3_waffle_pi_3d/model.sdf`에서 다음 구조를 찾는다.

```xml
<link name="base_scan">
  <!-- 기존 inertial, collision, visual -->
  <sensor name="hls_lfcd_lds" type="gpu_lidar">
    <!-- 기존 2D scan 설정 -->
  </sensor>
</link>
```

`base_scan` link, inertial, collision, visual은 삭제하지 않는다. **`<sensor>` 블록만 다음 내용으로 교체**한다.

```xml
<sensor name="hls_lfcd_lds" type="gpu_lidar">
  <always_on>true</always_on>
  <visualize>true</visualize>
  <pose>-0.064 0 0.121 0 0 0</pose>
  <update_rate>10</update_rate>
  <topic>scan</topic>
  <gz_frame_id>base_scan</gz_frame_id>
  <ray>
    <scan>
      <horizontal>
        <samples>640</samples>
        <resolution>1</resolution>
        <min_angle>0</min_angle>
        <max_angle>6.28</max_angle>
      </horizontal>
      <vertical>
        <samples>32</samples>
        <resolution>1</resolution>
        <min_angle>-0.261799</min_angle>
        <max_angle>0.261799</max_angle>
      </vertical>
    </scan>
    <range>
      <min>0.10</min>
      <max>20.0</max>
      <resolution>0.01</resolution>
    </range>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.005</stddev>
    </noise>
  </ray>
</sensor>
```

`<vertical>`이 핵심이다. 세로 방향을 약 `-15°`부터 `+15°`까지 32개 방향으로 나눈다. “32채널 LiDAR” 대신 이 문서에서는 **위아래 측정선 32개**라고 표현한다.

`type="gpu_lidar"`는 그대로 둔다. 이 값은 Gazebo 계산 방식이며, 2D와 3D를 구분하는 부분은 `<vertical>` 유무와 설정값이다.

새 `calib_lidar_link`, 새 joint, 새 cylinder를 추가하지 않는다. 기존 `base_scan` visual이 센서 외형을 이미 표시하며, `scan_joint`가 설치 위치를 이미 정의한다.

## 3. URDF의 기존 `base_scan`을 그대로 사용한다

`urdf/turtlebot3_waffle_pi_3d.urdf`에는 이미 다음 구조가 있다.

```xml
<joint name="scan_joint" type="fixed">
  <parent link="base_link"/>
  <child link="base_scan"/>
  <origin xyz="-0.064 0 0.122" rpy="0 0 0"/>
</joint>

<link name="base_scan">
  <visual>
    <!-- 기존 LDS mesh -->
  </visual>
  <!-- 기존 collision과 inertial -->
</link>
```

이 부분은 수정하지 않는다. 3D 여부는 Gazebo sensor가 결정한다. URDF는 **센서 외형과 `base_link -> base_scan` 고정 transform**을 담당한다.

## 4. Camera message의 frame ID를 optical frame으로 맞춘다

복사한 `model.sdf`의 Camera sensor에서 다음 한 줄만 바꾼다.

```xml
<!-- 변경 전 -->
<gz_frame_id>camera_rgb_frame</gz_frame_id>

<!-- 변경 후 -->
<gz_frame_id>camera_rgb_optical_frame</gz_frame_id>
```

`gz_frame_id`는 sensor message의 `header.frame_id`에 들어갈 좌표계 이름이다. Camera pose를 회전시키는 설정이 아니다.

| frame | 축 방향 | 역할 |
|---|---|---|
| `camera_rgb_frame` | `x=forward`, `y=left`, `z=up` | Camera 본체 설치 방향 |
| `camera_rgb_optical_frame` | `x=right`, `y=down`, `z=forward` | 3D point를 image pixel 위치로 계산할 때 사용 |

URDF의 `camera_rgb_frame -> camera_rgb_optical_frame` fixed joint가 두 좌표계 사이의 실제 회전을 제공한다. Camera sensor의 pose와 이 joint는 바꾸지 않는다.

## 5. 3D point cloud bridge를 추가한다

`params/turtlebot3_waffle_pi_3d_bridge.yaml` 마지막에 추가한다.

```yaml
# Point cloud from the replacement 3D LiDAR
- ros_topic_name: "calib/points"
  gz_topic_name: "scan/points"
  ros_type_name: "sensor_msgs/msg/PointCloud2"
  gz_type_name: "gz.msgs.PointCloudPacked"
  direction: GZ_TO_ROS
```

기존 `scan -> /scan` bridge는 일단 남긴다. 하지만 세로 측정선이 추가된 뒤의 `/scan`이 기존 단일 평면 `LaserScan`과 완전히 같은 의미라고 가정하지 않는다. **Calibration 입력은 `/calib/points`만 사용**한다. Simulation PATCH-06에서 장애물 회피용 2D scan이 필요하면 point cloud의 특정 높이 범위를 2D `LaserScan`으로 변환한다.

## 6. 빌드한다

호스트에서 개발 shell을 연다.

```bash
cd /home/swlinux/Desktop/workspace/mobin/docker
docker compose \
  -f compose.yaml \
  -f compose.nvidia.yaml \
  run --rm shell
```

이 명령은 NVIDIA 설정을 상속한 작업용 `shell` container를 새로 만든다. 아직 `sim`을 실행하기 전이므로 `exec sim bash`는 사용할 수 없다.

컨테이너 안에서 실행한다.

```bash
source /opt/ros/jazzy/setup.bash
cd /ws
colcon build --symlink-install --packages-select turtlebot3_gazebo
source /ws/install/setup.bash
```

## 7. 파생 모델을 실행한다

X11 권한 부여
```bash
xhost +si:localuser:root
```

시뮬레이션 실행

```bash
cd /home/swlinux/Desktop/workspace/mobin/docker

TURTLEBOT3_MODEL=waffle_pi_3d \
TURTLEBOT3_WORLD=turtlebot3_house.world \
GAZEBO_GUI=true \
LAUNCH_RVIZ=true \
docker compose \
  -f compose.yaml \
  -f compose.nvidia.yaml \
  up sim
```

두 번째 terminal에서 실행 중인 NVIDIA `sim` container의 shell을 연다.

```bash
cd /home/swlinux/Desktop/workspace/mobin/docker
docker compose \
  -f compose.yaml \
  -f compose.nvidia.yaml \
  exec sim bash
```

`exec`는 별도 container를 만들지 않는다. 첫 terminal에서 실행 중인 `sim`의 ROS graph, Gazebo topic, GPU 환경을 그대로 사용한다.

## 8. topic, field, TF를 검증한다

개발 shell 안에서 실행한다. Docker image에는 `rg`가 없을 수 있으므로 `grep`을 사용한다.

```bash
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash

gz topic -l | grep -E 'scan|points'
ros2 topic type /calib/points
ros2 topic hz /calib/points
ros2 topic echo /calib/points --field header --once
ros2 topic echo /calib/points --field fields --once
ros2 run tf2_ros tf2_echo base_link base_scan
ros2 run tf2_ros tf2_echo base_scan camera_rgb_optical_frame
```

`ros2 topic hz`는 계속 실행된다. 주파수를 확인한 뒤 `Ctrl-C`로 끝내고 다음 명령을 실행한다.

기대 결과:

```text
/calib/points type: sensor_msgs/msg/PointCloud2
header.frame_id: base_scan
fields: x, y, z, intensity
base_link -> base_scan translation: 약 [-0.064, 0, 0.122]
```

Camera frame도 확인한다.

```bash
ros2 topic echo /camera/image_raw --field header --once
ros2 topic echo /camera/camera_info --field header --once
```

두 message의 `frame_id`가 모두 `camera_rgb_optical_frame`이어야 한다.

## 9. RViz에서 확인한다

```bash
rviz2 -d /ws/install/turtlebot3_gazebo/share/turtlebot3_gazebo/rviz/tb3_waffle_pi_3d.rviz
```

| RViz 설정 | 값 |
|---|---|
| Fixed Frame | `base_link` |
| PointCloud2 topic | `/calib/points` |
| Style | `Points` |
| Color Transformer | `Intensity` |
| TF | `base_scan`, `camera_rgb_optical_frame` |

Point cloud가 `base_scan` 위치를 기준으로 주변 물체 표면에 나타나야 한다. 센서 외형은 기존 `base_scan` LDS mesh로 보인다.

## 10. 원본 모델이 유지됐는지 확인한다

호스트에서 original file의 diff를 확인한다.

```bash
cd /home/swlinux/Desktop/workspace/mobin
git -C forks/turtlebot3_simulations diff -- \
  turtlebot3_gazebo/models/turtlebot3_waffle_pi/model.sdf \
  turtlebot3_gazebo/urdf/turtlebot3_waffle_pi.urdf \
  turtlebot3_gazebo/params/turtlebot3_waffle_pi_bridge.yaml
```

출력이 비어 있어야 한다. 원본 실행도 확인한다.

```bash
cd /home/swlinux/Desktop/workspace/mobin/docker

TURTLEBOT3_MODEL=waffle_pi \
TURTLEBOT3_WORLD=turtlebot3_house.world \
GAZEBO_GUI=true \
LAUNCH_RVIZ=true \
docker compose \
  -f compose.yaml \
  -f compose.nvidia.yaml \
  up sim
```

원본에서는 기존 2D `/scan`이 발행되고 `/calib/points`는 없어야 한다.

## 완료 조건

- 파생 모델에 `calib_lidar_link`나 두 번째 LiDAR sensor가 없다.
- 기존 `base_scan` sensor에 `<vertical>` 측정 설정이 있다.
- `/calib/points`가 `sensor_msgs/msg/PointCloud2`로 발행된다.
- `PointCloud2.fields`에 `x`, `y`, `z`, `intensity`가 있다.
- `/calib/points.header.frame_id`가 `base_scan`이다.
- Camera image와 CameraInfo의 `frame_id`가 `camera_rgb_optical_frame`이다.
- Gazebo와 RViz에서 기존 LiDAR 외형이 보인다.
- 원본 `waffle_pi`의 SDF, URDF, bridge YAML에 diff가 없다.

## 실패할 때 확인 순서

### `/calib/points`가 없다

```bash
gz topic -l | grep -E 'scan|points'
```

`scan/points`가 없으면 SDF의 `<sensor type="gpu_lidar">`, `<vertical>`, Sensors system을 확인한다. Gazebo topic은 있는데 ROS topic만 없으면 bridge YAML의 topic과 message type을 확인한다.

### `intensity` field가 없다

```bash
ros2 topic echo /calib/points --field fields --once
```

이 상태에서는 Simulation PATCH-02로 넘어가지 않는다. 0으로 채운 가짜 intensity도 사용하지 않는다. Gazebo point message와 `PointCloudPacked -> PointCloud2` bridge가 intensity field를 제공하는지 먼저 확인한다.

### TF는 보이지만 point cloud 위치가 이상하다

```bash
ros2 run tf2_ros tf2_echo base_link base_scan
ros2 topic echo /calib/points --field header --once
```

`frame_id`가 `base_scan`인지 확인한다. SDF sensor pose와 URDF `scan_joint`를 동시에 임의 수정하지 않는다. 두 곳에 offset을 중복 적용하면 point cloud가 실제 센서 위치와 어긋난다.

## 이 PATCH에서 하지 않는 것

- 원본 `turtlebot3_waffle_pi`의 2D LiDAR 제거
- 두 번째 LiDAR와 `calib_lidar_link` 추가
- 실제 LiDAR 제조사 driver 추가
- point cloud filter node 추가
- calibration 계산과 결과 적용

Calibration 장면 기록은 Simulation PATCH-02, 계산은 Simulation PATCH-03, 결과 적용과 검증은 Simulation PATCH-04에서 진행한다.
