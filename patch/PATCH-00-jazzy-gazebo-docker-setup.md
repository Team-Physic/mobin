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
mobin/
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
cd /home/swlinux/Desktop/workspace/mobin

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

| 방식 | 의미 | 확인값 |
|---|---|---|
| CPU software rendering | GPU 없이 CPU로 3D 화면 계산 | `glxinfo -B`의 `llvmpipe` |
| GPU rendering | host GPU와 driver로 화면 계산 | Intel/AMD 또는 NVIDIA renderer 이름 |

첫 실행은 override 없이 CPU 경로를 검증한다.
```bash
docker compose up sim
```
기본값은 `waffle_pi`, `turtlebot3_world.world`, Gazebo GUI 실행, RViz 미실행이다.
### 실행 parameter

| 환경변수 | 기본값 | 선택값 |
|---|---|---|
| `TURTLEBOT3_MODEL` | `waffle_pi` | `burger`, `burger_cam`, `waffle`, `waffle_pi` |
| `TURTLEBOT3_WORLD` | `turtlebot3_world.world` | 아래 world file |
| `GAZEBO_GUI` | `true` | `true`, `false` |
| `LAUNCH_RVIZ` | `false` | `true`, `false` |

| world file | 용도 |
|---|---|
| `empty_world.world` | 빈 공간 |
| `turtlebot3_world.world` | 기본 실습장 |
| `turtlebot3_house.world` | 실내 주택 |
| `turtlebot3_dqn_stage1.world` ~ `turtlebot3_dqn_stage4.world` | 단계별 장애물 환경 |

Camera–LiDAR 실습에는 camera가 있는 `waffle`, `waffle_pi`, `burger_cam`을 사용한다. `burger`에는 camera가 없다.

실행 조합:
```bash
# RViz만 표시
GAZEBO_GUI=false LAUNCH_RVIZ=true docker compose up sim

# Gazebo와 RViz 모두 표시
GAZEBO_GUI=true LAUNCH_RVIZ=true docker compose up sim

# robot과 world도 선택
TURTLEBOT3_MODEL=burger_cam \
TURTLEBOT3_WORLD=turtlebot3_dqn_stage1.world \
GAZEBO_GUI=false LAUNCH_RVIZ=true docker compose up sim
```
환경변수는 해당 명령으로 생성한 container에만 적용된다. CPU renderer 확인:
```bash
docker compose run --rm shell glxinfo -B
```
`llvmpipe`는 느리지만 정상적인 software renderer다.
## 11. Intel/AMD GPU로 실행

CPU 실행이 성공하고 `/dev/dri`가 존재할 때만 override를 추가한다.
```bash
# simulation
docker compose -f compose.yaml -f compose.dri.yaml up sim
# renderer 확인
docker compose -f compose.yaml -f compose.dri.yaml run --rm shell glxinfo -B
```
## 12. NVIDIA GPU로 실행

NVIDIA Container Toolkit 검사가 성공한 경우에만 override를 추가한다.
```bash
# simulation
TURTLEBOT3_MODEL=burger_cam TURTLEBOT3_WORLD=turtlebot3_house.world \
 docker compose -f compose.yaml -f compose.nvidia.yaml up sim
# GPU 확인
docker compose -f compose.yaml -f compose.nvidia.yaml run --rm shell nvidia-smi
# 실패 시 CPU 경로
docker compose -f compose.yaml up sim
```
## 13. 개발 shell 사용

개발 `shell`은 ROS 2 명령을 직접 입력하는 일회용 container다. `sim`과 같은 image, source, `build/install/log` volume을 사용한다.

| 상황 | `shell`에서 실행 | 결과 사용 위치 |
|---|---|---|
| 최초 실행·source 수정 | `colcon build --symlink-install` | 다음 `sim` 실행 |
| simulation 조사 | `ros2 topic`, `ros2 node`, `tf2_echo` | 실행 중인 `sim`과 host network로 통신 |
| node 개발·debug | `ros2 run`, parameter 변경, log 확인 | simulation을 유지한 채 node만 재실행 |
| GUI/GPU 진단 | `rviz2`, `glxinfo -B`, `nvidia-smi` | rendering 문제 분리 확인 |

Gazebo 실행 중 새 terminal에서 연다.
```bash
cd /home/swlinux/Desktop/workspace/mobin/docker
docker compose run --rm shell
```
컨테이너 안에서 overlay를 source한다.
```bash
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash
ros2 topic list
```
source 수정 후에는 재빌드한다.
```bash
cd /ws
colcon build --symlink-install
source /ws/install/setup.bash
```
최초 build의 dependency 검사와 전체 명령은 8절을 따른다. `--rm`은 종료한 shell container만 삭제하며 host source와 named volume은 유지한다. world만 실행할 때는 `docker compose up sim`을 사용한다.
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
## 15. RViz와 Rerun의 역할
### 개념

| 도구 | 정확한 역할 |
|---|---|
| RViz | 실행 중인 ROS 2 topic과 TF를 구독해 `/scan`, point cloud, Camera, TF를 실시간 확인 |
| Rerun | 기록된 Camera·3D point·TF를 같은 시간축에서 정지·이동·반복하며 비교 |
| MCAP | timestamp가 포함된 ROS 2 message 저장 형식; 기록 절차는 PATCH-02에서 수행 |

Rerun은 RViz의 대체품이 아니다.

| 작업 | 선택 | 이유 |
|---|---|---|
| live topic, TF, Nav2 확인 | RViz | ROS 2 display와 조작 기능 사용 |
| 기록 시점별 Camera·point cloud·TF 비교 | Rerun | viewer 시간 막대 사용 |
| Gazebo world 조작 | Gazebo GUI | RViz와 Rerun은 world를 조작하지 않음 |

Rerun에는 native ROS 2 구독 기능이 없으므로 [공식 ROS 2 연동 문서](https://rerun.io/docs/howto/integrations/ros2-nav-turtlebot)를 따라 변환 node 또는 MCAP importer가 필요하다. 이 프로젝트는 archive된 과거 bridge 대신 [공식 MCAP importer](https://rerun.io/docs/howto/logging-and-ingestion/mcap)를 사용한다. 지원 message는 [형식 표](https://rerun.io/docs/concepts/logging-and-ingestion/mcap/message-formats)에서 확인한다.

PATCH-00~01의 live 상태는 RViz로 확인한다. PATCH-02에서 기록한 `Image`, `CameraInfo`, `PointCloud2`, `TFMessage`는 Rerun에서 같은 시간축으로 확인한다. Calibration 계산과 projection 표시는 PATCH-03 이후 별도 Python/C++ 코드가 기록해야 한다.
### Docker image에 Rerun 설치

`docker/sim/Dockerfile`에 다음 항목을 반영한다.
```dockerfile
# docker/sim/Dockerfile | optional Rerun Viewer and isolated Python environment
ARG RERUN_VERSION=0.35.0
# 기존 apt-get install 목록에 추가
    python3-venv \
# 기존 package 설치 RUN 다음에 추가
RUN python3 -m venv --system-site-packages /opt/rerun \
 && /opt/rerun/bin/pip install --no-cache-dir "rerun-sdk==${RERUN_VERSION}"
ENV PATH="/opt/rerun/bin:${PATH}"
```
별도 virtual environment를 사용해 ROS 2 system Python package를 덮어쓰지 않는다. 다시 build하고 확인한다.
```bash
cd /home/swlinux/Desktop/workspace/mobin/docker
docker compose build
docker compose run --rm shell rerun --version
```
출력에 `0.35.0`이 포함되어야 한다.

제한 사항:
- Rerun 실행만으로 live ROS 2 topic이 표시되지 않는다.
- `LaserScan` 확인은 RViz를 사용한다.
- Rerun은 calibration을 계산하지 않는다. 보정 전후 projection과 residual은 PATCH-03 이후 코드에서 기록한다.
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
