# PATCH-00: ROS 2 Jazzy + Gazebo Harmonic Docker 환경 구성

## 목적

호스트에 ROS 2/Gazebo Simulator/RViz를 직접 설치하지 않고 Docker 안에서 다음 환경을 만든다.

- ROS 2 Jazzy Desktop
- Gazebo Harmonic과 `ros_gz`
- 선택 가능한 TurtleBot3 model과 world simulation
- Gazebo GUI와 RViz의 선택 실행
- 로컬 `forks/turtlebot3_simulations/`을 사용하는 colcon overlay
- 같은 이미지로 실행되는 Gazebo `sim` 서비스와 개발용 `shell` 서비스
- GPU가 없어도 동작하는 CPU rendering
- 선택적인 Intel/AMD `/dev/dri` 및 NVIDIA GPU rendering
- 기록한 Camera·LiDAR 데이터를 시간순으로 확인하는 선택적 Rerun Viewer
- 두 upstream 리포의 commit 기록

## 최종 디렉토리 구조

```text
mobile-robot-calibration-repo/
├── .gitignore
├── README.md
├── docker/
│   ├── compose.yaml
│   ├── compose.dri.yaml
│   ├── compose.nvidia.yaml
│   └── sim/
│       └── Dockerfile
├── forks/
│   ├── direct_visual_lidar_calibration/
│   └── turtlebot3_simulations/
└── patch/
    └── PATCH-00-jazzy-gazebo-docker-setup.md
```

## 1. 프로젝트 루트와 호스트 환경 확인

### 개념

컨테이너는 별도의 물리 PC가 아니므로 화면과 GPU는 host의 자원을 사용한다.

| 개념 | 쉬운 설명 | 이 단계에서 확인하는 이유 |
|---|---|---|
| Docker daemon | 컨테이너를 실제로 만들고 실행하는 background service | daemon이 꺼져 있으면 모든 Docker 명령이 실패함 |
| `DISPLAY`, `xhost` | 컨테이너의 GUI를 host 화면에 표시할 위치와 접근 권한 | Gazebo와 RViz 창을 표시하려면 필요 |
| `/dev/dri` | Intel/AMD GPU를 Linux 프로그램에 보여 주는 장치 경로 | 해당 GPU로 rendering할 수 있는지 판단 |
| NVIDIA runtime | NVIDIA GPU와 host driver를 컨테이너에 연결하는 기능 | NVIDIA override 사용 가능 여부 판단 |

GPU를 사용할 수 없어도 CPU rendering으로 실행할 수 있다.

```bash
cd /home/swlinux/Desktop/workspace/mobile-robot-calibration-repo

docker --version
docker compose version
docker info >/dev/null

printf 'DISPLAY=%s\n' "$DISPLAY"
command -v xhost

test -e /dev/dri && ls -l /dev/dri || true
nvidia-smi || true
docker info | grep -i runtime || true
```

#### 판단 기준

| 확인 결과 | 의미 | 다음 작업 |
|---|---|---|
| `docker info` 실패 | Docker daemon 또는 사용자 권한 문제 | Docker부터 해결 |
| `DISPLAY`가 비어 있음 | X11 GUI 전달 불가 | SSH X forwarding 또는 headless 구성 |
| `/dev/dri` 존재 | Intel/AMD GPU 전달 가능 | DRI override 선택 가능 |
| host와 container에서 `nvidia-smi` 성공 | NVIDIA GPU 전달 가능 | NVIDIA override 선택 가능 |
| GPU 사용 불가 | hardware rendering 불가 | 기본 CPU rendering 사용 |

## 2. Fork remote와 TurtleBot3 Jazzy 브랜치 확인

### 개념

TurtleBot3에는 ROS 2 배포판별 source가 있으므로 Jazzy용 branch를 선택해야 한다. <br><br>
두 fork의 생성과 clone은 [Fork workflow와 license 준수](../docs/fork_workflow_and_licensing.md)를 먼저 따른다. `status --short`는 branch 전환 전에 저장하지 않은 수정이 없는지 확인한다.

ROS 2 Jazzy와 맞추기 위해 simulation 리포도 공식 `jazzy` branch를 사용한다.

```bash
git -C forks/turtlebot3_simulations status --short
git -C forks/turtlebot3_simulations branch --show-current
git -C forks/turtlebot3_simulations remote -v
```

첫 명령에 수정 파일이 없고 두 번째 명령이 `jazzy`이면 그대로 진행한다. 다른 branch라면 저장하지 않은 변경이 없는 상태에서 전환한다.

```bash
git -C forks/turtlebot3_simulations switch jazzy
```

확인:

```bash
git -C forks/turtlebot3_simulations status --short --branch
git -C forks/turtlebot3_simulations rev-parse HEAD
```

## 3. Dockerfile 작성

| 개념 | 쉬운 설명 | 이 단계에서 필요한 이유 |
|---|---|---|
| Dockerfile | 기본 image와 설치할 package를 적은 제작 설명서 | 동일한 ROS 2 환경을 반복 생성 |
| image | 프로그램 설치가 끝난 실행 환경 원본 | `shell`과 `sim`이 공통으로 사용 |
| container | image를 실제로 실행한 instance | ROS 2 명령과 Gazebo가 실행되는 공간 |
| Gazebo Harmonic | 로봇 움직임, 충돌, camera, LiDAR를 계산하는 simulator | TurtleBot3와 sensor data 생성 |
| `ros_gz` | Gazebo data와 ROS 2 topic을 연결하는 package | simulated sensor data를 ROS 2 node에서 사용 |
| `LABEL` | image에 저장하는 metadata | 사용한 upstream commit 기록 |

`LABEL`은 simulation 동작을 바꾸지 않는다.

```bash
mkdir -p docker/sim
```

`docker/sim/Dockerfile`:

```dockerfile
FROM osrf/ros:jazzy-desktop-full

ARG TB3_SIM_COMMIT=unknown
ARG CALIB_COMMIT=unknown

LABEL lab.turtlebot3_simulations.commit="${TB3_SIM_COMMIT}"
LABEL lab.direct_visual_lidar_calibration.commit="${CALIB_COMMIT}"

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-ros-gz \
    ros-jazzy-turtlebot3 \
    ros-dev-tools \
    mesa-utils \
    libgl1-mesa-dri \
 && rosdep update \
 && rm -rf /var/lib/apt/lists/*

ENV TURTLEBOT3_MODEL=waffle_pi

WORKDIR /ws

CMD ["bash"]
```

설치 항목:

| 항목 | 제공 기능 | 이 image에서의 용도 |
|---|---|---|
| `osrf/ros:jazzy-desktop-full` | ROS 2 Jazzy, RViz, GUI 도구 | 기본 image |
| `ros-jazzy-ros-gz` | ROS 2와 Gazebo bridge/launch package | Gazebo sensor와 ROS 2 연결 |
| `ros-jazzy-turtlebot3` | TurtleBot3 description과 의존성 | robot model과 launch 사용 |
| `ros-dev-tools` | `colcon`, `rosdep` 등 | source build와 dependency 확인 |
| `mesa-utils`, `libgl1-mesa-dri` | OpenGL과 software renderer | GPU가 없을 때 CPU rendering |
| `TURTLEBOT3_MODEL=waffle_pi` | 기본 TurtleBot3 model 지정 | launch마다 model 입력 생략 |
| `LABEL` | image metadata | 두 upstream commit 기록 |

## 4. compose.yaml 작성

### 개념

같은 ROS 2 image를 작업용과 simulation용으로 나누어 실행한다.

| 구성 | 쉬운 설명 | 이 프로젝트에서 하는 일 |
|---|---|---|
| `shell` | 명령을 직접 입력하는 작업용 terminal | `colcon build`, topic 확인, node 실행 |
| `sim` | 정해진 launch 명령을 자동 실행하는 방식 | Gazebo와 TurtleBot3 world 시작 |
| bind mount | host 폴더를 container 안에서 그대로 보는 연결 | 로컬 TurtleBot3 source 공유 |
| named volume | 여러 container가 함께 사용하는 Docker 저장 공간 | `build`, `install`, `log` 결과 공유 |

먼저 `shell`에서 build하면 `sim`이 공유 volume의 `/ws/install`을 읽어 실행한다.

`docker/compose.yaml`:

```yaml
# docker/compose.yaml | x-tb3-common.environment, services.sim.command
name: mobile-robot-calibration-repo

x-tb3-common: &tb3-common
  image: tb3-jazzy-lab:local
  build:
    context: ./sim
    args:
      TB3_SIM_COMMIT: ${TB3_SIM_COMMIT:-unknown}
      CALIB_COMMIT: ${CALIB_COMMIT:-unknown}
  network_mode: host
  environment:
    DISPLAY: ${DISPLAY}
    QT_X11_NO_MITSHM: "1"
    LIBGL_ALWAYS_SOFTWARE: "1"
    TURTLEBOT3_MODEL: ${TURTLEBOT3_MODEL:-waffle_pi}
    TURTLEBOT3_WORLD: ${TURTLEBOT3_WORLD:-turtlebot3_world.world}
    GAZEBO_GUI: ${GAZEBO_GUI:-true}
    LAUNCH_RVIZ: ${LAUNCH_RVIZ:-false}
    GZ_IP: ${GZ_IP:-127.0.0.1}
  volumes:
    - ../forks/turtlebot3_simulations:/ws/src/turtlebot3_simulations:rw
    - tb3_build:/ws/build
    - tb3_install:/ws/install
    - tb3_log:/ws/log
    - /tmp/.X11-unix:/tmp/.X11-unix:rw

services:
  shell:
    <<: *tb3-common
    stdin_open: true
    tty: true
    command: bash

  sim:
    <<: *tb3-common
    command: >
      bash -lc "
        source /opt/ros/jazzy/setup.bash &&
        test -f /ws/install/setup.bash ||
          { echo 'ERROR: 먼저 shell 서비스에서 colcon build를 실행하세요.'; exit 1; };
        source /ws/install/setup.bash &&
        ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py world:=$${TURTLEBOT3_WORLD} gazebo_gui:=$${GAZEBO_GUI} launch_rviz:=$${LAUNCH_RVIZ}
      "

volumes:
  tb3_build:
  tb3_install:
  tb3_log:
```

## 5. Intel/AMD `/dev/dri` override 작성

| 개념 | 쉬운 설명 | 이 단계에서 필요한 이유 |
|---|---|---|
| `/dev/dri` | Intel/AMD GPU를 프로그램에 보여 주는 Linux 장치 경로 | 컨테이너가 host GPU로 rendering하도록 전달 |
| Compose override | 기본 Compose에 선택 설정만 덧붙이는 파일 | CPU 기본 설정을 유지하면서 GPU 기능 추가 |
| `LIBGL_ALWAYS_SOFTWARE=0` | CPU 강제 rendering을 해제하는 설정 | GPU renderer 사용 허용 |

`/dev/dri`가 없는 PC에서는 `compose.dri.yaml`을 사용하지 않는다.

`docker/compose.dri.yaml`:

```yaml
services:
  shell:
    devices:
      - /dev/dri:/dev/dri
    environment:
      LIBGL_ALWAYS_SOFTWARE: "0"

  sim:
    devices:
      - /dev/dri:/dev/dri
    environment:
      LIBGL_ALWAYS_SOFTWARE: "0"
```

이 override는 `/dev/dri`가 존재할 때만 사용한다. 기본 Compose에 장치를 넣지 않아 GPU 없는 환경에서도 시작할 수 있게 한다.

## 6. NVIDIA override 작성

`docker/compose.nvidia.yaml`:

```yaml
services:
  shell:
    gpus: all
    environment:
      LIBGL_ALWAYS_SOFTWARE: "0"
      NVIDIA_DRIVER_CAPABILITIES: graphics,display,utility,compute
      __NV_PRIME_RENDER_OFFLOAD: "1"
      __GLX_VENDOR_LIBRARY_NAME: nvidia

  sim:
    gpus: all
    environment:
      LIBGL_ALWAYS_SOFTWARE: "0"
      NVIDIA_DRIVER_CAPABILITIES: graphics,display,utility,compute
      __NV_PRIME_RENDER_OFFLOAD: "1"
      __GLX_VENDOR_LIBRARY_NAME: nvidia
```

NVIDIA override 사용 전 확인:

```bash
nvidia-smi
docker run --rm --gpus all nvidia/cuda:12.8.0-base-ubuntu24.04 nvidia-smi
```

두 번째 명령이 실패하면 NVIDIA Container Toolkit을 먼저 점검한다. 이 경우에도 CPU Compose는 사용할 수 있다.

## 7. upstream commit 설정 및 이미지 빌드

```bash
export TB3_SIM_COMMIT="$(git -C forks/turtlebot3_simulations rev-parse HEAD)"
export CALIB_COMMIT="$(git -C forks/direct_visual_lidar_calibration rev-parse HEAD)"

printf 'TurtleBot3 simulations: %s\n' "$TB3_SIM_COMMIT"
printf 'Calibration toolbox:   %s\n' "$CALIB_COMMIT"

cd docker
docker compose build
```

## 8. colcon build

```bash
docker compose run --rm shell bash -lc '
  source /opt/ros/jazzy/setup.bash &&
  rosdep check --from-paths src --ignore-src &&
  colcon build --symlink-install
'
```

launch argument 확인:

```bash
docker compose run --rm shell bash -lc '
  source /opt/ros/jazzy/setup.bash &&
  source /ws/install/setup.bash &&
  ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py --show-args
'
```

출력에 `world`, `gazebo_gui`, `launch_rviz`와 각 기본값이 표시되어야 한다.

## 9. X11 권한 부여

| 개념 | 쉬운 설명 | 이 단계에서 필요한 이유 |
|---|---|---|
| X11 socket | 컨테이너의 GUI를 host desktop으로 전달하는 통로 | Gazebo와 RViz 창 표시 |
| `DISPLAY` | GUI를 표시할 X server 위치 | 컨테이너가 출력 위치 선택 |
| `xhost` | X server 접근 권한을 관리하는 명령 | container root만 임시 허용하고 종료 후 회수 |

Gazebo 실행 직전에 Docker root 사용자에게만 로컬 X11 접근을 임시 허용한다.

```bash
xhost +si:localuser:root
```

`xhost +local:`처럼 모든 local client를 허용하는 넓은 설정은 사용하지 않는다. 작업 종료 후 반드시 회수한다.

```bash
xhost -si:localuser:root
```

## 10. CPU로 Gazebo 실행

### 개념

| 방식 | 쉬운 설명 | 이 단계에서의 의미 |
|---|---|---|
| CPU software rendering | GPU 대신 CPU로 3D 화면 계산 | 별도 GPU 설정 없이 첫 실행 검증 |
| GPU rendering | GPU와 driver로 3D 화면 계산 | CPU 검증 후 선택적으로 사용 |
| `llvmpipe` | Mesa의 CPU renderer 이름 | `glxinfo`에 표시되면 CPU rendering 정상 |

가장 먼저 GPU override 없이 기본값으로 실행한다.

```bash
docker compose up sim
```

기본값은 `waffle_pi`, `turtlebot3_world.world`, Gazebo GUI 실행, RViz 미실행이다.

### 실행 parameter

| host 환경변수 | 기본값 | 선택값 |
|---|---|---|
| `TURTLEBOT3_MODEL` | `waffle_pi` | `burger`, `burger_cam`, `waffle`, `waffle_pi` |
| `TURTLEBOT3_WORLD` | `turtlebot3_world.world` | 아래 world file 중 하나 |
| `GAZEBO_GUI` | `true` | `true`, `false` |
| `LAUNCH_RVIZ` | `false` | `true`, `false` |

| world file | 용도 |
|---|---|
| `empty_world.world` | 장애물이 없는 빈 공간 |
| `turtlebot3_world.world` | 기본 TurtleBot3 실습장 |
| `turtlebot3_house.world` | 실내 주택 환경 |
| `turtlebot3_dqn_stage1.world` ~ `turtlebot3_dqn_stage4.world` | 강화학습 단계별 장애물 환경 |

`waffle`, `waffle_pi`, `burger_cam`에는 camera 구성이 있어 Camera-LiDAR 실습에 적합하다. `burger`는 camera가 없으므로 calibration 실습 기본값으로 사용하지 않는다.

### Gazebo GUI 및 RViz 시각화

RViz만 실행:

```bash
GAZEBO_GUI=false LAUNCH_RVIZ=true docker compose up sim
```

Gazebo 및 RViz 모두 실행:

```bash
GAZEBO_GUI=true LAUNCH_RVIZ=true docker compose up sim
```

robot과 world까지 선택한 예시 :

```bash
TURTLEBOT3_MODEL=burger_cam \
TURTLEBOT3_WORLD=turtlebot3_dqn_stage1.world \
GAZEBO_GUI=true \
LAUNCH_RVIZ=true \
docker compose up sim
```

값은 명령을 실행한 한 번의 container에만 적용된다. `docker compose down` 후 환경변수 없이 다시 실행하면 기본값으로 돌아간다.

실행 순서:

1. ROS 2 underlay와 로컬 overlay source
2. `TURTLEBOT3_MODEL`로 URDF와 Gazebo model 선택
3. `TURTLEBOT3_WORLD`를 `world` launch argument로 전달
4. Gazebo server 시작
5. `GAZEBO_GUI`, `LAUNCH_RVIZ` 조건에 맞는 창 시작

CPU renderer 확인:

```bash
docker compose run --rm shell glxinfo -B
```

renderer에 `llvmpipe`가 표시될 수 있다. 느리지만 정상적인 software rendering이다.

## 11. Intel/AMD GPU로 실행

CPU 실행이 성공하고 `/dev/dri`가 존재할 때 실행한다.

```bash
docker compose \
  -f compose.yaml \
  -f compose.dri.yaml \
  up sim
```

renderer 확인:

```bash
docker compose \
  -f compose.yaml \
  -f compose.dri.yaml \
  run --rm shell glxinfo -B
```

## 12. NVIDIA GPU로 실행

NVIDIA Container Toolkit 검사가 성공한 경우에만 실행한다.

```bash
TURTLEBOT3_MODEL=burger_cam \
TURTLEBOT3_WORLD=turtlebot3_house.world \
GAZEBO_GUI=true \
LAUNCH_RVIZ=false \
docker compose \
  -f compose.yaml \
  -f compose.nvidia.yaml \
  up sim
```

컨테이너 GPU 확인:

```bash
docker compose \
  -f compose.yaml \
  -f compose.nvidia.yaml \
  run --rm shell nvidia-smi
```

NVIDIA 실행이 실패하면 override를 빼고 CPU 경로로 복귀한다.

```bash
docker compose -f compose.yaml up sim
```

## 13. 개발 shell 사용

Gazebo가 실행 중일 때 새 terminal에서 실행한다.

```bash
cd /home/swlinux/Desktop/workspace/mobile-robot-calibration-repo/docker
docker compose run --rm shell
```

컨테이너 내부:

```bash
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash
```

코드 수정 후 재빌드:

```bash
colcon build --symlink-install
source /ws/install/setup.bash
```

### 개발 shell은 언제 사용하는가

개발 `shell`은 ROS 2 명령을 직접 입력하는 일회용 container다. Gazebo를 자동 실행하는 `sim`과 같은 image, source, `build/install/log` volume을 사용한다.

| 상황 | 개발 `shell`에서 하는 일 | `sim`과의 관계 |
|---|---|---|
| 최초 실행 전 | TurtleBot3 source를 `colcon build` | build가 끝나야 `sim` 실행 가능 |
| source 수정 후 | 변경된 package rebuild와 test | 새 build 결과를 다음 `sim`이 사용 |
| Gazebo 실행 중 | topic, node, TF, sensor 주기 확인 | host network를 통해 실행 중인 `sim`에 연결 |
| node 개발·debug | `ros2 run`, parameter 변경, log 확인 | 전체 simulation을 다시 만들지 않고 node만 반복 실행 |
| GUI/GPU 문제 확인 | `rviz2`, `glxinfo -B`, `nvidia-smi` 실행 | rendering 문제를 simulation과 분리해 진단 |

최초 build 또는 source 수정 후 rebuild:

```bash
cd /home/swlinux/Desktop/workspace/mobile-robot-calibration-repo/docker
docker compose run --rm shell bash -lc '
  source /opt/ros/jazzy/setup.bash &&
  colcon build --symlink-install
'
```

build 완료 후 simulation 실행:

```bash
docker compose up sim
```

실행 중인 simulation을 조사하려면 새 terminal에서 개발 shell을 연다.

```bash
cd /home/swlinux/Desktop/workspace/mobile-robot-calibration-repo/docker
docker compose run --rm shell

source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash
ros2 topic list
```

`--rm`은 종료한 shell container만 삭제한다. host source와 named volume의 build 결과는 유지된다. world만 실행할 때는 개발 shell 대신 `docker compose up sim`을 사용한다.

## 14. ROS topic과 TF 검증

```bash
ros2 topic list
```

최소 예상 topic:

```text
/camera/camera_info
/camera/image_raw
/clock
/imu
/joint_states
/odom
/scan
/tf
/tf_static
```

타입과 발행 주기 확인:

```bash
ros2 topic type /scan
ros2 topic type /camera/image_raw
ros2 topic type /camera/camera_info
ros2 topic hz /scan
ros2 topic hz /camera/image_raw
```

예상 타입:

```text
sensor_msgs/msg/LaserScan
sensor_msgs/msg/Image
sensor_msgs/msg/CameraInfo
```

TF 확인:

```bash
ros2 run tf2_ros tf2_echo base_link base_scan
ros2 run tf2_ros tf2_echo base_link camera_rgb_optical_frame
```

RViz는 10절처럼 `LAUNCH_RVIZ=true`로 `sim` 시작 시 함께 실행한다. 이미 실행 중인 `sim`에 RViz를 별도로 붙이려면 개발 shell 안에서 다음 명령을 사용한다.

```bash
ros2 run rviz2 rviz2 -d \
  /ws/install/turtlebot3_gazebo/share/turtlebot3_gazebo/rviz/tb3_gazebo.rviz \
  --ros-args -p use_sim_time:=true
```

## 15. RVIz VS Rerun : 시간 축에서의 Data Visualization

### 개념

| 도구 | 쉬운 설명 | 이 프로젝트에서 맡는 역할 |
|---|---|---|
| RViz | 실행 중인 ROS 2 topic과 TF를 바로 구독해 보여 주는 도구 | `/scan`, `/calib/points`, Camera, TF를 실시간 점검 |
| Rerun | 여러 sensor 기록을 **같은 시간축에서 다시 살펴보는 도구** | 기록된 Camera 영상, 3D 점, TF를 멈춤·이동·반복하며 비교 |
| MCAP | 시간 정보와 함께 ROS 2 message를 저장하는 파일 형식 | PATCH-02에서 기록한 데이터를 Rerun으로 전달 |

중요 : **Rerun은 RViz보다 무조건 나은 프로그램이 아니다.** 서로 상호 보완적인 관계이며, 따라서 둘을 함께 사용한다.

| 비교 항목 | RViz | Rerun | 이 프로젝트의 선택 |
|---|---|---|---|
| 실행 중인 ROS 2 topic 확인 | 별도 변환 없이 바로 구독 | 공식 native ROS 2 구독 기능은 아직 없음 | 실시간 점검은 RViz |
| 과거 시점으로 이동 | rosbag 재생을 별도로 제어 | Viewer의 시간 막대에서 바로 이동·정지·반복 | 기록 분석은 Rerun |
| Camera·3D 점·TF 동시 확인 | 가능하지만 재생과 display를 직접 맞춰야 함 | 지원되는 MCAP message를 같은 시간축으로 자동 배치 | Calibration 입력 확인은 Rerun |
| ROS plugin과 조작 기능 | ROS 생태계의 display, Interactive Marker, Nav2 도구가 풍부 | ROS 전용 기능은 제한적 | robot 운용·debug는 RViz |
| 사용자 계산 결과 표시 | Marker message나 plugin 필요 | Python/C++ SDK로 residual, 검출 결과, 그래프를 함께 기록 가능 | PATCH-03 이후 결과 분석에 Rerun 확장 가능 |
| Gazebo world 조작 | 불가 | 불가 | world 조작은 Gazebo GUI |

공식 [ROS 2 연동 문서](https://rerun.io/docs/howto/integrations/ros2-nav-turtlebot)는 Rerun에 native ROS 지원이 아직 없으며 ROS message를 변환해 기록하는 node가 필요하다고 설명한다. 과거 C++ ROS 2 bridge 예제는 2026-07-27에 archive·deprecated 상태가 되었으므로 이 프로젝트의 기본 의존성으로 추가하지 않는다. 대신 공식 [MCAP importer](https://rerun.io/docs/howto/logging-and-ingestion/mcap)를 사용한다.

MCAP importer는 이 실습의 핵심 message인 `sensor_msgs/Image`, `sensor_msgs/CameraInfo`, `sensor_msgs/PointCloud2`, `tf2_msgs/TFMessage`를 영상, Camera 모델, 3D 점, 좌표 변환으로 해석한다. 지원 목록은 공식 [MCAP message 형식 표](https://rerun.io/docs/concepts/logging-and-ingestion/mcap/message-formats)에서 확인한다.

### 적용 범위

| 단계 | 사용할 도구 | 이유 |
|---|---|---|
| PATCH-00~01의 simulation 실행과 sensor topic 확인 | Gazebo GUI + RViz | live ROS 2 상태를 바로 확인해야 함 |
| PATCH-02에서 기록한 bag 품질 확인 | Rerun | Camera, point cloud, TF의 기록 시점을 함께 이동하며 확인 가능 |
| PATCH-03~04의 calibration 결과 비교 | Rerun 확장 가능 | 초기값·추정값·ground truth와 residual을 같은 시간축에 기록 가능 |

첫 적용에서는 **기록된 MCAP 열기만** 수행한다. live ROS 2 bridge, Rerun 전용 ROS node, C++ SDK 연결, 자동 layout 파일은 필요해질 때 추가한다.

### Docker image에 Rerun 설치

공식 Python package인 `rerun-sdk`에는 SDK와 Viewer가 함께 들어 있다. 재현 가능한 image를 위해 확인 당시 최신 버전인 `0.35.0`을 고정한다.

`docker/sim/Dockerfile`에서 기존 `ARG` 아래에 다음 값을 추가한다.

```dockerfile
# docker/sim/Dockerfile | optional Rerun Viewer version
ARG RERUN_VERSION=0.35.0
```

기존 `apt-get install` 목록에 `python3-venv`를 추가한다.

```dockerfile
# docker/sim/Dockerfile | packages required by the Rerun virtual environment
    python3-venv \
```

기존 package 설치 `RUN` 다음에 추가한다.

```dockerfile
# docker/sim/Dockerfile | install the Rerun Viewer without changing system Python packages
RUN python3 -m venv --system-site-packages /opt/rerun \
 && /opt/rerun/bin/pip install --no-cache-dir "rerun-sdk==${RERUN_VERSION}"

ENV PATH="/opt/rerun/bin:${PATH}"
```

ROS 2의 system Python package를 덮어쓰지 않도록 별도 virtual environment를 사용한다. MCAP을 여는 데 C++ SDK는 필요하지 않으므로 설치하지 않는다.

image를 다시 만들고 CLI 설치를 확인한다.

```bash
cd /home/swlinux/Desktop/workspace/mobin/docker
docker compose build
docker compose run --rm shell rerun --version
```

출력에 `0.35.0`이 포함되어야 한다.

### 제한 사항

- Rerun을 실행한다고 현재 ROS 2 topic이 자동으로 표시되지는 않는다. live 연결에는 별도 변환 node가 필요하다.
- `LaserScan`은 현재 MCAP 자동 시각화 지원 표에 없으므로 기존 2D `/scan` 확인은 RViz를 사용한다.
- `PointCloud2`와 Camera 영상이 있다고 calibration 결과가 자동 계산되지는 않는다. Rerun은 입력과 결과를 확인하는 도구다.
- Camera 영상 위에 보정 전후 LiDAR projection과 residual을 직접 겹치려면 PATCH-03 이후 Python 또는 C++ 코드에서 계산 결과를 Rerun SDK로 기록해야 한다.
- 현재 단계에서는 문서와 설치 절차만 추가한다. Viewer 실행과 MCAP 표시 여부는 image rebuild 및 PATCH-02 기록 후 검증한다.

## 16. 종료와 재실행

| 대상 | `docker compose down` 이후 | 주의점 |
|---|---|---|
| service container | 삭제됨 | 다음 실행 때 다시 생성됨 |
| named volume | 유지됨 | build/install 결과를 재사용할 수 있음 |
| `down -v` 실행 시 volume | 삭제됨 | 다음 실행 전에 전체 build 필요 |
| `xhost` 권한 | Docker와 별개로 유지됨 | 종료할 때 별도 명령으로 회수 |

```bash
docker compose down
xhost -si:localuser:root
```

다음 실행부터 build/install volume을 그대로 재사용한다.

```bash
xhost +si:localuser:root
docker compose up sim
```

`docker compose down -v`는 사용하지 않는다. `-v`를 붙이면 colcon build/install/log volume이 삭제되어 다시 빌드해야 한다.
