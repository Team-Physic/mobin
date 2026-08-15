# PATCH-02: Calibration 월드와 rosbag 기록

## 이 PATCH에서 만드는 것

카메라 영상의 명암과 LiDAR intensity가 모두 풍부하게 변하는 정적 calibration 월드를 만들고, 서로 다른 5개 pose에서 반복 가능한 ROS 2 bag을 기록한다.

이 calibration 방식은 checkerboard를 요구하지 않는 target-less 방식이다. 그러나 아무 벽이나 촬영하면 되는 것은 아니다. 깊이, 표면 방향, 영상 texture, LiDAR 반사도 변화가 함께 있어야 6-DoF가 잘 관측된다.

## 시작 조건

- PATCH-01의 `/calib/points`가 `x`, `y`, `z`, `intensity` field를 가진다.
- `/camera/image_raw`와 `/camera/camera_info`가 발행된다.
- Camera와 3D LiDAR는 로봇에 rigid하게 고정되어 있다.

```bash
ros2 topic type /calib/points
ros2 topic echo /calib/points --field fields --once
ros2 topic echo /camera/camera_info --field distortion_model --once
```

`distortion_model`은 현재 도구가 자동 인식할 수 있는 `plumb_bob` 또는 `fisheye`여야 한다.

## 추가/수정할 파일

```text
mobile-robot-calibration-repo/
├── .gitignore
├── data/
│   └── bags/
├── docker/
│   └── compose.yaml
└── forks/turtlebot3_simulations/turtlebot3_gazebo/
    ├── launch/
    │   ├── spawn_turtlebot3.launch.py
    │   └── turtlebot3_calibration.launch.py
    ├── models/calibration_scene/
    │   ├── model.config
    │   └── model.sdf
    └── worlds/turtlebot3_calibration.world
```

`CMakeLists.txt`는 이미 `launch`, `models`, `worlds` 디렉터리 전체를 설치하므로 수정하지 않는다.

## 1. bag 저장 위치를 컨테이너에 마운트한다

호스트에서 디렉터리를 만든다.

```bash
cd /home/swlinux/Desktop/workspace/mobin
mkdir -p data/bags data/results
```

`docker/compose.yaml`의 공통 `volumes`에 한 줄을 추가한다.

```yaml
    - ../data:/ws/data:rw
```

최종 공통 mount에는 적어도 다음 항목이 있어야 한다.

```yaml
  volumes:
    - ../forks/turtlebot3_simulations:/ws/src/turtlebot3_simulations:rw
    - ../data:/ws/data:rw
    - tb3_build:/ws/build
    - tb3_install:/ws/install
    - tb3_log:/ws/log
    - /tmp/.X11-unix:/tmp/.X11-unix:rw
```

`.gitignore`에는 대용량 bag과 생성 결과를 제외한다. 빈 디렉터리를 Git으로 유지할 필요는 없다.

```gitignore
data/bags/
data/results/*
!data/results/.gitkeep
```

## 2. calibration scene model을 만든다

`models/calibration_scene/model.config`:

```xml
<?xml version="1.0"?>
<model>
  <name>calibration_scene</name>
  <version>1.0</version>
  <sdf version="1.8">model.sdf</sdf>
  <author>
    <name>mobile-robot-calibration-repo</name>
  </author>
  <description>Static geometry and reflectance scene for Camera-LiDAR calibration</description>
</model>
```

`models/calibration_scene/model.sdf`는 하나의 static model로 만든다.

```xml
<?xml version="1.0"?>
<sdf version="1.8">
  <model name="calibration_scene">
    <static>true</static>

    <!-- 아래의 link들을 이 위치에 넣는다. -->
  </model>
</sdf>
```

각 물체는 visual과 collision을 같은 geometry로 만들고, collision에 `laser_retro`를 직접 둔다. 예를 들어 가까운 밝은 박스는 다음과 같다.

```xml
<link name="near_bright_box">
  <pose>2.0 -0.8 0.60 0 0 0.35</pose>
  <visual name="visual">
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
```

위 블록을 복사해 다음 표대로 최소 7개 물체를 만든다. 이름과 수치는 그대로 사용해 첫 dataset을 재현한다.

| 이름 | geometry / size | pose `x y z roll pitch yaw` | grayscale | `laser_retro` |
|---|---|---|---:|---:|
| `near_bright_box` | box `0.45 0.45 1.20` | `2.0 -0.8 0.60 0 0 0.35` | 0.85 | 1800 |
| `near_dark_box` | box `0.35 0.70 0.90` | `2.3 0.75 0.45 0 0 -0.40` | 0.15 | 250 |
| `mid_gray_panel` | box `0.08 2.20 1.50` | `3.4 0.10 0.75 0 0 0.15` | 0.50 | 900 |
| `mid_bright_cylinder` | cylinder `r=0.25, l=1.30` | `3.0 -1.20 0.65 0 0 0` | 0.75 | 1500 |
| `mid_dark_cylinder` | cylinder `r=0.18, l=0.80` | `3.8 1.10 0.40 0 0 0` | 0.20 | 350 |
| `far_bright_panel` | box `0.08 1.60 2.00` | `5.2 -0.70 1.00 0 0 -0.20` | 0.90 | 2000 |
| `far_dark_panel` | box `0.08 1.50 1.40` | `5.8 0.95 0.70 0 0 0.25` | 0.10 | 150 |

같은 명암의 반복 격자만 만들지 않는다. 서로 다른 높이, 깊이, 회전, 곡면을 두는 것이 핵심이다. 모든 물체는 로봇 전방 `x > 0`에 있어 PATCH-01의 전방 180도 LiDAR와 카메라 공통 시야에 들어온다.

## 3. calibration world를 만든다

기존 `worlds/empty_world.world`를 복사해 `worlds/turtlebot3_calibration.world`를 만든다.

```bash
cp forks/turtlebot3_simulations/turtlebot3_gazebo/worlds/empty_world.world \
   forks/turtlebot3_simulations/turtlebot3_gazebo/worlds/turtlebot3_calibration.world
```

새 world의 `<world>` 안에 scene model을 추가한다.

```xml
<include>
  <uri>model://calibration_scene</uri>
</include>
```

world에는 actor, 움직이는 plugin, wind를 넣지 않는다. 조명은 첫 calibration에서 고정한다.

```xml
<scene>
  <ambient>0.35 0.35 0.35 1</ambient>
  <background>0.05 0.05 0.05 1</background>
  <shadows>true</shadows>
</scene>
```

## 4. spawn launch에 yaw 인자를 한 번만 추가한다

`launch/spawn_turtlebot3.launch.py`는 현재 `x_pose`, `y_pose`만 받는다. `yaw`를 추가하면 calibration과 장애물 시나리오에서 같은 spawn 파일을 재사용할 수 있다.

```python
yaw = LaunchConfiguration('yaw', default='0.0')

declare_yaw_cmd = DeclareLaunchArgument(
    'yaw', default_value='0.0',
    description='Initial yaw in radians')
```

`ros_gz_sim create` arguments에 다음 두 항목을 추가한다.

```python
'-Y', yaw
```

LaunchDescription에는 declaration을 추가한다.

```python
ld.add_action(declare_yaw_cmd)
```

기존 인자의 이름이나 기본값은 바꾸지 않는다.

## 5. calibration launch를 만든다

`turtlebot3_world.launch.py`를 복사한다.

```bash
cp forks/turtlebot3_simulations/turtlebot3_gazebo/launch/turtlebot3_world.launch.py \
   forks/turtlebot3_simulations/turtlebot3_gazebo/launch/turtlebot3_calibration.launch.py
```

새 launch에서 다음 세 가지만 바꾼다.

1. world 파일을 `turtlebot3_calibration.world`로 변경한다.
2. 기본 pose를 `x=0`, `y=0`, `yaw=0`으로 둔다.
3. `yaw`를 spawn include에 전달한다.

핵심 부분은 다음과 같아야 한다.

```python
x_pose = LaunchConfiguration('x_pose', default='0.0')
y_pose = LaunchConfiguration('y_pose', default='0.0')
yaw = LaunchConfiguration('yaw', default='0.0')

world = os.path.join(
    get_package_share_directory('turtlebot3_gazebo'),
    'worlds',
    'turtlebot3_calibration.world')

# spawn_turtlebot_cmd의 launch_arguments
launch_arguments={
    'x_pose': x_pose,
    'y_pose': y_pose,
    'yaw': yaw,
}.items()
```

## 6. 빌드하고 월드를 확인한다

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

Gazebo GUI에서 모든 물체가 보이고 로봇이 물체 내부나 바닥 아래에 생성되지 않는지 확인한다.

다른 shell에서 센서 상태를 확인한다.

```bash
ros2 topic hz /calib/points
ros2 topic hz /camera/image_raw
ros2 topic echo /calib/points --field fields --once
```

RViz PointCloud2의 Color Transformer를 `Intensity`로 했을 때 물체별 색 차이가 보여야 한다. 모든 점이 같은 intensity라면 기록하지 않는다.

## 7. 5개 pose에서 bag을 기록한다

다음 pose를 차례대로 사용한다.

| bag | x [m] | y [m] | yaw [rad] |
|---|---:|---:|---:|
| `pose-01` | 0.00 | 0.00 | 0.00 |
| `pose-02` | 0.20 | -0.35 | 0.10 |
| `pose-03` | -0.15 | 0.35 | -0.10 |
| `pose-04` | 0.45 | 0.15 | 0.18 |
| `pose-05` | 0.35 | -0.20 | -0.18 |

각 bag마다 simulation을 해당 pose로 새로 실행한다. 예:

```bash
ros2 launch turtlebot3_gazebo turtlebot3_calibration.launch.py \
  x_pose:=0.0 y_pose:=0.0 yaw:=0.0
```

센서 주기가 안정된 뒤 별도 shell에서 기록한다.

Rerun이 바로 열 수 있도록 `ros2 bag record`에 MCAP 저장 형식을 명시한다.

```bash
ros2 bag record --storage mcap \
  -o /ws/data/bags/pose-01 \
  /calib/points \
  /camera/image_raw \
  /camera/camera_info \
  /tf \
  /tf_static \
  /clock
```

ROS 2 Jazzy의 기본 저장 형식도 MCAP이지만 `--storage mcap`을 적어 두면 다른 ROS 2 환경에서 실행해도 결과 형식이 바뀌지 않는다. 8절의 `ros2 bag info` 결과에서 `Storage id`가 `mcap`인지 확인한다.

12~20초 동안 로봇과 장면을 완전히 정지한 상태로 유지하고 `Ctrl-C`로 종료한다. 시뮬레이션의 32채널 cloud와 5개 pose를 쓰므로 첫 실습에서는 주행 중 누적을 하지 않는다. 움직이면서 기록하는 spinning-LiDAR 절차는 timestamp와 motion compensation을 별도로 검증할 때 확장한다.

나머지 pose도 출력 경로와 launch 인자를 바꾸어 같은 방식으로 기록한다. 기존 출력 디렉터리에 덮어쓰지 않는다.

## 8. bag별 품질을 확인한다

호스트 또는 ROS 2가 설치된 컨테이너에서 실행한다.

```bash
ros2 bag info /ws/data/bags/pose-01
```

5개 모두 다음 타입과 0보다 큰 message count를 가져야 한다.

```text
/calib/points       sensor_msgs/msg/PointCloud2
/camera/image_raw   sensor_msgs/msg/Image
/camera/camera_info sensor_msgs/msg/CameraInfo
/tf                 tf2_msgs/msg/TFMessage
/tf_static          tf2_msgs/msg/TFMessage
/clock              rosgraph_msgs/msg/Clock
```

bag을 재생할 때는 simulation과 동시에 재생하지 않는다.

```bash
ros2 bag play /ws/data/bags/pose-01 --clock
```

다른 shell에서 RViz나 topic echo로 image, cloud, frame ID를 확인한다.

## 9. Rerun에서 MCAP 열기

Rerun에는 rosbag 디렉토리가 아니라 그 안의 `.mcap` 파일을 전달한다. 먼저 host에서 파일 위치를 확인한다.

```bash
find /home/swlinux/Desktop/workspace/mobin/data/bags/pose-01 \
  -maxdepth 1 -type f -name '*.mcap' -print
```

출력된 파일을 읽기 전용으로 container에 연결해 연다. 아래 경로는 실제 파일명에 맞게 바꾼다.

```bash
cd /home/swlinux/Desktop/workspace/mobin/docker

RERUN_BAG=/home/swlinux/Desktop/workspace/mobin/data/bags/pose-01/pose-01_0.mcap
test -f "$RERUN_BAG"

xhost +si:localuser:root
docker compose run --rm \
  -v "$RERUN_BAG:/ws/data/pose-01.mcap:ro" \
  shell rerun --renderer gl /ws/data/pose-01.mcap
```

기본 Compose의 `LIBGL_ALWAYS_SOFTWARE=1`과 `--renderer gl`을 사용하므로 전용 GPU가 없어도 Mesa software rendering을 시도한다. 느리면 PATCH-00의 11절 또는 12절에 있는 Compose override를 같은 `docker compose run` 명령에 적용한다.

Viewer에서 다음 항목을 확인한다.

| 확인 항목 | 정상 기준 |
|---|---|
| Camera | `/camera/image_raw` 영상이 시간 막대를 움직일 때 바뀜 |
| Camera 정보 | `/camera/camera_info`가 Camera 시야 모델로 연결됨 |
| 3D LiDAR | `/calib/points`가 3D 점으로 표시됨 |
| 좌표 관계 | `/tf`, `/tf_static`을 이용해 Camera와 LiDAR frame 관계가 유지됨 |
| 시간 | 영상과 point cloud를 같은 시점에서 멈춰 비교 가능 |

Robot mesh가 보이지 않아도 sensor 기록 확인은 가능하다. mesh까지 필요하면 MCAP에 `/robot_description`을 넣거나 [URDF importer](https://rerun.io/docs/howto/logging-and-ingestion/urdf)로 URDF를 별도 불러온다.

## 완료 조건

- `data/bags/pose-01`부터 `pose-05`까지 5개 bag이 있다.
- 각 bag에 Image, CameraInfo, PointCloud2가 들어 있다.
- cloud에는 `intensity` field와 실제 intensity 변화가 있다.
- 각 bag의 첫 image와 point cloud는 같은 정지 pose에서 취득된다.
- 공통 시야에 가까운/중간/먼 구조와 평면/모서리/곡면이 모두 있다.
- bag은 Git status에 나타나지 않는다.

```bash
git status --short
```

## 실패할 때 확인 순서

### 모델이 로드되지 않는다

```bash
gz sim -v 4 /ws/install/turtlebot3_gazebo/share/turtlebot3_gazebo/worlds/turtlebot3_calibration.world
```

`GZ_SIM_RESOURCE_PATH`에 설치된 `models` 경로가 포함되는지, `model.config`의 SDF 버전과 파일명이 맞는지 확인한다.

### camera와 cloud가 같은 장면을 보지 않는다

로봇 spawn yaw와 LiDAR horizontal 범위를 먼저 확인한다. 센서 pose를 bag마다 바꾸지 말고 scene 물체 또는 로봇 초기 pose만 조정한다.

### intensity가 모두 같다

`laser_retro`가 visual이 아니라 각 `collision`의 직접 자식인지 확인한다. 서로 다른 값이 GPU LiDAR message와 bridge를 통과하는지도 `gz topic`과 ROS topic 양쪽에서 확인한다.

## 이 PATCH에서 하지 않는 것

- 움직이는 물체 추가
- camera auto-exposure 모델링
- bag 자동 삭제나 덮어쓰기
- calibration 실행

calibration 실행은 PATCH-03에서 로컬 calibration fork로 build한 이미지를 사용한다.
