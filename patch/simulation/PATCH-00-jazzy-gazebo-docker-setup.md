# Simulation PATCH-00: ROS 2 Jazzy + Gazebo Harmonic Docker 환경 구성

## 목적

호스트에 ROS 2/Gazebo Simulator/RVIz를 직접 설치하지 않고 Docker 안에서 다음 환경을 만든다.

- ROS 2 Jazzy Desktop
- Gazebo Harmonic과 `ros_gz`
- 선택 가능한 TurtleBot3 model과 world simulation
- Gazebo GUI와 RViz의 선택 실행
- ROS 2 sensor 기록을 시간축으로 확인하는 Rerun Viewer
- 로컬 `forks/turtlebot3_simulations/`을 사용하는 colcon overlay
- 같은 이미지로 실행되는 Gazebo `sim` 서비스와 개발용 `shell` 서비스
- GPU가 없어도 동작하는 CPU rendering
- 선택적인 Intel/AMD `/dev/dri` 및 NVIDIA GPU rendering
- 두 upstream 리포의 commit 기록

## 최종 디렉토리 구조

```text
mobin/
├── .gitignore
├── README.md
├── code/
│   ├── scripts/                        # 실행 script
│   ├── python/                         # rclpy package
│   └── cpp/                            # rclcpp package
├── docker/                              # Compose·Dockerfile
├── forks/                               # 독립 upstream fork
└── patch/
    └── Simulation PATCH-00-jazzy-gazebo-docker-setup.md
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
두 fork의 생성과 clone은 [Fork workflow와 license 준수](../../docs/how_to_fork_and_license.md)를 먼저 따른다. `status --short`는 branch 전환 전에 저장하지 않은 수정이 없는지 확인한다.

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
| Rerun | image와 point cloud 같은 기록 data를 같은 시간축에서 보는 Viewer | 이후 MCAP의 sensor timestamp와 공간 관계 확인 |
| `LABEL` | image에 저장하는 metadata | 사용한 upstream commit 기록 |

`LABEL`은 simulation 동작을 바꾸지 않는다.

```bash
mkdir -p docker/sim
```

`docker/sim/Dockerfile`:

```dockerfile
# docker/sim/Dockerfile | ROS 2, Gazebo, Rerun 공통 image
FROM osrf/ros:jazzy-desktop-full

ARG TB3_SIM_COMMIT=unknown
ARG CALIB_COMMIT=unknown
ARG RERUN_VERSION=0.35.0

LABEL lab.turtlebot3_simulations.commit="${TB3_SIM_COMMIT}"
LABEL lab.direct_visual_lidar_calibration.commit="${CALIB_COMMIT}"

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-ros-gz \
    ros-jazzy-turtlebot3 \
    ros-dev-tools \
    mesa-utils \
    libgl1-mesa-dri \
    python3-venv \
    libgtk-3-0 \
    libxkbcommon-x11-0 \
    libxcb-render0 \
    libxcb-shape0 \
    libxcb-xfixes0 \
 && python3 -m venv /opt/rerun \
 && /opt/rerun/bin/pip install --no-cache-dir "rerun-sdk==${RERUN_VERSION}" \
 && ln -s /opt/rerun/bin/rerun /usr/local/bin/rerun \
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
| `rerun-sdk` | `/opt/rerun`에 격리하고 Viewer만 symlink | ROS build Python을 바꾸지 않고 MCAP 시각화 |
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
name: mobin

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

`docker compose build`는 `docker/sim/Dockerfile`을 읽어 ROS 2, Gazebo, Rerun과 system package가 설치된 `tb3-jazzy-lab:local` image를 만든다. **결과물은 container의 실행 기반인 Docker image**이며, 두 commit 값은 build argument로 전달되어 image `LABEL`에 기록된다. 이 단계는 `forks/turtlebot3_simulations/`의 ROS 2 package를 compile하지 않는다.

```bash
export TB3_SIM_COMMIT="$(git -C forks/turtlebot3_simulations rev-parse HEAD)"
export CALIB_COMMIT="$(git -C forks/direct_visual_lidar_calibration rev-parse HEAD)"

printf 'TurtleBot3 simulations: %s\n' "$TB3_SIM_COMMIT"
printf 'Calibration toolbox:   %s\n' "$CALIB_COMMIT"

cd docker
docker compose build
```

## 8. colcon build

`colcon build`는 앞 단계의 image로 `shell` container를 실행하고, `/ws/src/turtlebot3_simulations`의 ROS 2 source package를 compile한다. **결과물은 `/ws/build`, `/ws/install`, `/ws/log`의 workspace 산출물**이며 named volume에 저장되어 `sim` service가 사용한다. 이 단계는 Dockerfile의 package를 다시 설치하거나 Docker image를 변경하지 않는다.

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

## 10. Gazebo와 RViz 실행

기본 Compose는 CPU software rendering을 사용한다. `glxinfo -B`의 renderer가 `llvmpipe`여도 느릴 뿐 정상이다.

```bash
docker compose up sim
docker compose run --rm shell glxinfo -B
```

| 환경변수 | 기본값 | 선택값 |
|---|---|---|
| `TURTLEBOT3_MODEL` | `waffle_pi` | `burger`, `burger_cam`, `waffle`, `waffle_pi` |
| `TURTLEBOT3_WORLD` | `turtlebot3_world.world` | `empty_world.world`, `turtlebot3_house.world`, `turtlebot3_dqn_stage1.world`~`stage4.world` |
| `GAZEBO_GUI` | `true` | `true`, `false` |
| `LAUNCH_RVIZ` | `false` | `true`, `false` |

`burger`에는 camera가 없다. Camera-LiDAR 실습에는 `burger_cam`, `waffle`, `waffle_pi`를 쓴다.

```bash
# RViz만
GAZEBO_GUI=false LAUNCH_RVIZ=true docker compose up sim

# Gazebo와 RViz, model과 world 지정
TURTLEBOT3_MODEL=burger_cam \
TURTLEBOT3_WORLD=turtlebot3_dqn_stage1.world \
GAZEBO_GUI=true LAUNCH_RVIZ=true \
docker compose up sim
```

환경변수는 해당 명령으로 만든 container에만 적용된다.

### Rerun은 RViz를 대체하는가

| 도구 | 주 용도 | 이 프로젝트에서 확인하는 것 |
|---|---|---|
| RViz | 실행 중인 ROS 2 topic과 TF를 실시간 표시 | frame 연결, 현재 image와 point cloud |
| Rerun | 저장된 data를 timeline으로 재생·비교 | Camera와 LiDAR message의 시간 순서, image와 point cloud의 대응 |

둘은 대체 관계가 아니다. Rerun 설치만으로 ROS 2 topic을 자동 구독하지도 않는다. 이 PATCH에서는 Viewer를 image에 넣고 Dockerfile 변경 후 image를 다시 만들어 GUI 기동까지만 확인한다. MCAP 기록과 실제 `.mcap` 열기는 [Simulation PATCH-02](PATCH-02-calibration-scene-recording.md)에서 수행한다.

```bash
docker compose build
docker compose run --rm shell rerun --version
docker compose run --rm shell rerun --renderer=gl
```

마지막 명령에서 빈 Rerun Viewer 창이 표시되면 GUI 전달까지 정상이다. CPU rendering은 느릴 수 있다. 여러 sensor가 같은 timeline에 보이는 것은 timestamp가 정확히 같다는 뜻이 아니며, 차이의 크기는 기록 후 비교한다. 지원되는 ROS 2 `Image`, `CameraInfo`, `PointCloud2`와 MCAP 실행법은 [Rerun MCAP 공식 문서](https://rerun.io/docs/howto/logging-and-ingestion/mcap)를 따른다.

## 11. GPU로 실행

CPU 실행이 성공한 뒤 해당 hardware override 하나만 추가한다.

```bash
# Intel/AMD: /dev/dri가 있을 때
docker compose -f compose.yaml -f compose.dri.yaml up sim
docker compose -f compose.yaml -f compose.dri.yaml run --rm shell glxinfo -B

# NVIDIA: host와 container의 nvidia-smi가 성공할 때
docker compose -f compose.yaml -f compose.nvidia.yaml up -d sim
docker compose -f compose.yaml -f compose.nvidia.yaml run --rm shell nvidia-smi
```

GPU 실행이 실패하면 override를 빼고 `docker compose -f compose.yaml up sim`으로 돌아간다.

## 12. 개발 shell 사용

`shell`은 최초 build, source 수정 후 rebuild, topic·TF·GPU 확인, 개별 node debug에 쓴다. `sim`과 image 및 `build/install/log` volume을 공유한다.

```bash
# 실행 중인 NVIDIA sim container에 접속
docker compose -f compose.yaml -f compose.nvidia.yaml exec sim bash

# sim과 별도로 일회용 작업 container 실행
docker compose run --rm shell
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash
ros2 topic list
```

`exec sim bash`는 먼저 `up -d sim`이 실행 중이어야 한다. `run --rm shell`은 sim 실행 여부와 무관하다. source를 수정했다면 8절의 `colcon build`를 다시 실행한다. `--rm`은 일회용 container만 삭제하고 named volume과 host source는 유지한다.

## 13. `/cmd_vel` CLI 시험

bridge는 ROS 2 `TwistStamped`를 Gazebo `Twist`로 바꾼다. 먼저 type과 subscriber를 확인한다.

```bash
ros2 topic type /cmd_vel
ros2 topic info --verbose /cmd_vel
```

type이 `geometry_msgs/msg/TwistStamped`, subscription count가 1 이상일 때 0.1 m/s로 5초 전진한 뒤 정지한다.

```bash
ros2 topic pub --rate 10 --times 50 --wait-matching-subscriptions 1 \
  /cmd_vel geometry_msgs/msg/TwistStamped \
  "{twist: {linear: {x: 0.1}, angular: {z: 0.0}}}"

ros2 topic pub --once --wait-matching-subscriptions 1 \
  /cmd_vel geometry_msgs/msg/TwistStamped \
  "{twist: {linear: {x: 0.0}, angular: {z: 0.0}}}"
```

publish되지만 움직이지 않으면 `ros2 topic hz /clock`을 확인한다. `/clock`이 없으면 command가 아니라 Gazebo server 기동 문제다.

## 14. ROS topic과 TF 검증

| 대상 | 확인 |
|---|---|
| Camera | `/camera/image_raw`, `/camera/camera_info` |
| Robot | `/imu`, `/joint_states`, `/odom`, `/scan` |
| 시간·좌표 | `/clock`, `/tf`, `/tf_static` |

```bash
ros2 topic list
ros2 topic type /scan
ros2 topic type /camera/image_raw
ros2 topic hz /scan
ros2 topic hz /camera/image_raw
ros2 run tf2_ros tf2_echo base_link base_scan
ros2 run tf2_ros tf2_echo base_link camera_rgb_optical_frame
```

이미 실행 중인 sim에 RViz만 추가하려면 개발 shell에서 실행한다.

```bash
ros2 run rviz2 rviz2 -d \
  /ws/install/turtlebot3_gazebo/share/turtlebot3_gazebo/rviz/tb3_gazebo.rviz \
  --ros-args -p use_sim_time:=true
```

## 15. 종료와 재실행

```bash
docker compose down
xhost -si:localuser:root
```

`down`은 service container만 삭제하고 named volume은 유지한다. `docker compose down -v`는 `build/install/log` volume까지 삭제하므로 새로 전체 build할 때만 사용한다. 다음 실행에서는 `xhost +si:localuser:root` 후 `docker compose up sim`을 실행한다.
