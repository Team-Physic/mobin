# Python/C++ 이중 구현 구성안

- 작성일: 2026-08-03
- 브랜치: `main`
- 코드 기준: `turtlebot3_simulations@45633014`, `direct_visual_lidar_calibration@02a0dc03`, 상위 working tree
- 대상: 루트 `code/python/`, `code/cpp/`, Docker Compose, PATCH-04, PATCH-06, PATCH-10
- 결론: **공통 Gazebo 환경은 한 번만 유지한다. PATCH-06 장애물 회피의 Python/C++ package는 구현·검증 완료했고, extrinsic 이중 구현은 후속 범위다.**

### Why?

현재 보고서의 구현 위치가 섞여 있다.

- PATCH-04: `code/scripts/extrinsic_math.py` Python 단일 구현
- PATCH-06: TurtleBot3 fork의 `turtlebot3_drive.cpp` C++ 단일 구현
- 같은 문제를 두 언어로 작성·비교하기 어려움
- 학습 코드와 upstream 코드 경계 불명확

학습 코드를 상위 리포의 `code/python/`, `code/cpp/`로 분리한다. SDF, URDF, world, bridge, bag은 공통 자산이므로 복제하지 않는다.

### What I Made

이 문서는 디렉터리 구조와 공통 인터페이스를 정의한다. 현재 `mobile_robot_lab_python`과 `mobile_robot_lab_cpp`에 PATCH-06 회피 node와 test가 있으며, extrinsic 관련 파일은 아직 계획 상태다.

### What was problem

디렉터리만 나누면 비교가 되지 않는다. 두 구현이 다음 항목까지 같아야 한다.

- topic과 message type
- ROS parameter 이름과 기본값
- transform 방향과 quaternion 순서
- JSON 입력·출력 형식
- 완료 기준과 test fixture

두 회피 node를 동시에 실행하면 `/cmd_vel` 제어권도 충돌한다. launch에서 하나만 선택해야 한다.

### How it changed

```text
# mobile-robot-calibration-repo | language split
기존 위치
├── code/scripts/extrinsic_math.py
└── forks/turtlebot3_simulations/.../turtlebot3_drive.cpp

현재 위치
├── code/python/mobile_robot_lab_python/  # rclpy 구현
├── code/cpp/mobile_robot_lab_cpp/        # rclcpp 구현
└── forks/                             # 공통 simulator 자산
```

## 1. 최종 구조

```text
# mobile-robot-calibration-repo | target repository layout
mobile-robot-calibration-repo/
├── code/
│   ├── python/mobile_robot_lab_python/
│   │   ├── package.xml
│   │   ├── setup.py
│   │   ├── setup.cfg
│   │   ├── resource/mobile_robot_lab_python
│   │   ├── mobile_robot_lab_python/
│   │   │   ├── __init__.py
│   │   │   ├── extrinsic_math.py
│   │   │   └── obstacle_avoidance.py
│   │   └── test/
│   │       ├── test_extrinsic_math.py
│   │       └── test_obstacle_avoidance.py
│   ├── cpp/mobile_robot_lab_cpp/
│   │   ├── package.xml
│   │   ├── CMakeLists.txt
│   │   ├── include/mobile_robot_lab_cpp/
│   │   │   ├── extrinsic_math.hpp
│   │   │   └── scan_utils.hpp
│   │   ├── src/
│   │   │   ├── extrinsic_math.cpp
│   │   │   ├── extrinsic_math_main.cpp
│   │   │   └── obstacle_avoidance.cpp
│   │   └── test/
│   │       ├── extrinsic_math_test.cpp
│   │       └── scan_utils_test.cpp
│   └── scripts/                      # 공통 실행 script
├── docker/                          # 공통 Jazzy/Gazebo image
├── forks/                           # 공통 URDF/SDF/world/upstream
├── data/                            # 공통 bag/result
├── patch/
└── docs/
```

`code/python/`, `code/cpp/`는 언어 그룹. 실제 colcon package는 한 단계 아래에 둔다.

## 2. 중복 범위

| 기능 | Python | C++ | 공통 |
|---|---|---|---|
| Extrinsic 결과 비교·URDF 변환 | 작성 | 작성 | `calib.json`, ground truth |
| 장애물 회피 node | 작성 | 작성 | `/scan`, `/cmd_vel`, parameter |
| Gazebo world·sensor·bridge | 미작성 | 미작성 | TurtleBot3 fork에서 한 번 |
| 동적 장애물 Gazebo system plugin | 미작성 | simulator용 C++만 | PATCH-05 |
| upstream calibration 실행 | wrapper만 | upstream binary | 로컬 fork image |
| raw episode collector | 작성 | 작성 | MCAP·manifest contract |
| LeRobot exporter | 작성 | 미작성 | 공식 Python API |

Gazebo plugin까지 Python으로 복제하지 않는다. 학습 비교 대상은 PATCH-04와 PATCH-06이다.

## 3. 호스트 디렉터리 생성

```bash
cd /home/swlinux/Desktop/workspace/mobin
mkdir -p code/python code/cpp
```

## 4. Compose mount 추가

`docker/compose.yaml` 공통 `volumes`:

```yaml
# docker/compose.yaml | x-tb3-common.volumes
    - ../code/python:/ws/src/python:rw
    - ../code/cpp:/ws/src/cpp:rw
```

최종 공통 mount:

```yaml
# docker/compose.yaml | x-tb3-common.volumes
  volumes:
    - ../forks/turtlebot3_simulations:/ws/src/turtlebot3_simulations:rw
    - ../code/python:/ws/src/python:rw
    - ../code/cpp:/ws/src/cpp:rw
    - ../data:/ws/data:rw
    - tb3_build:/ws/build
    - tb3_install:/ws/install
    - tb3_log:/ws/log
    - /tmp/.X11-unix:/tmp/.X11-unix:rw
```

## 5. ROS 2 package 생성

```bash
cd /home/swlinux/Desktop/workspace/mobin/docker
docker compose run --rm shell
```

컨테이너 안:

```bash
source /opt/ros/jazzy/setup.bash

ros2 pkg create mobile_robot_lab_python \
  --build-type ament_python \
  --license Apache-2.0 \
  --destination-directory /ws/src/python \
  --dependencies rclpy sensor_msgs geometry_msgs

ros2 pkg create mobile_robot_lab_cpp \
  --build-type ament_cmake \
  --license Apache-2.0 \
  --destination-directory /ws/src/cpp \
  --dependencies rclcpp sensor_msgs geometry_msgs
```

확인:

```bash
colcon list | grep mobile_robot_lab
```

## 6. 빈 package 선행 build

```bash
cd /ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install \
  --packages-select mobile_robot_lab_python mobile_robot_lab_cpp
source /ws/install/setup.bash
ros2 pkg list | grep mobile_robot_lab
```

실패 시 알고리즘 작성 중단. package metadata와 mount부터 수정한다.

## 7. Extrinsic 공통 계약

```text
# code/python/.../extrinsic_math.py and code/cpp/.../extrinsic_math.cpp | interface contract
입력 1: data/results/calib.json
입력 2: data/results/ground-truth.json
옵션:   --metrics <output.json>
옵션:   --camera-joint
출력:   translation_error_m, rotation_error_deg, pass
```

```text
# code/python/.../extrinsic_math.py and code/cpp/.../extrinsic_math.cpp | transform contract
p_lidar = T_lidar_camera * p_camera_optical
T = [x, y, z, qx, qy, qz, qw]
translation_error_m <= 0.05
rotation_error_deg <= 3.0
```

실행 이름:

```bash
ros2 run mobile_robot_lab_python extrinsic_math_py CALIB_JSON GROUND_TRUTH_JSON
ros2 run mobile_robot_lab_cpp extrinsic_math_cpp CALIB_JSON GROUND_TRUTH_JSON
```

## 8. PATCH-04 Python 구현

대상:

```text
# code/python/mobile_robot_lab_python | PATCH-04 files
mobile_robot_lab_python/extrinsic_math.py
test/test_extrinsic_math.py
setup.py
```

순서:

1. `results.T_lidar_camera` 7값·유한값 검사
2. quaternion 정규화
3. translation Euclidean error 계산
4. `abs(q_estimate dot q_truth)`로 quaternion 부호 동치 처리
5. `2 * acos(dot)` rotation error 계산
6. PATCH-04 transform chain으로 `camera_joint xyz/rpy` 계산
7. identity, inverse, known rotation pytest 작성
8. console script 등록

```python
# code/python/mobile_robot_lab_python/setup.py | setup()
entry_points={
    'console_scripts': [
        'extrinsic_math_py = mobile_robot_lab_python.extrinsic_math:main',
    ],
},
```

## 9. PATCH-04 C++ 구현

대상:

```text
# code/cpp/mobile_robot_lab_cpp | PATCH-04 files
include/mobile_robot_lab_cpp/extrinsic_math.hpp
src/extrinsic_math.cpp
src/extrinsic_math_main.cpp
test/extrinsic_math_test.cpp
CMakeLists.txt
package.xml
```

순서:

1. 같은 7값 transform 구조 정의
2. quaternion normalize, multiply, inverse, rotate 구현
3. 같은 JSON validation·metric·exit code 구현
4. `extrinsic_math_cpp` executable 등록
5. Python과 같은 test vector 사용

새 대형 선형대수 library는 추가하지 않는다. JSON parser는 image 내 기존 dependency를 먼저 확인한다. 없을 때만 `nlohmann_json` 추가.

```cmake
# code/cpp/mobile_robot_lab_cpp/CMakeLists.txt | extrinsic targets
add_library(extrinsic_math src/extrinsic_math.cpp)
target_include_directories(extrinsic_math PUBLIC include)

add_executable(extrinsic_math_cpp src/extrinsic_math_main.cpp)
target_link_libraries(extrinsic_math_cpp extrinsic_math)

install(TARGETS extrinsic_math extrinsic_math_cpp
  DESTINATION lib/${PROJECT_NAME})
```

## 10. Extrinsic parity test

```bash
ros2 run mobile_robot_lab_python extrinsic_math_py \
  /ws/data/results/calib.json \
  /ws/data/results/ground-truth.json \
  --metrics /ws/data/results/metrics-python.json

ros2 run mobile_robot_lab_cpp extrinsic_math_cpp \
  /ws/data/results/calib.json \
  /ws/data/results/ground-truth.json \
  --metrics /ws/data/results/metrics-cpp.json
```

허용 차이:

- `pass`: 완전 일치
- translation error: `<= 1e-9 m`
- rotation error: `<= 1e-9 deg`
- `camera_joint xyz/rpy`: 각 원소 `<= 1e-9`

차이 초과 시 tolerance 확대 금지. quaternion 순서, radian/degree, inverse부터 확인한다.

## 11. 장애물 회피 공통 계약

```text
# Python/C++ obstacle avoidance | ROS interface contract
SUB /scan     sensor_msgs/msg/LaserScan
PUB /cmd_vel  geometry_msgs/msg/TwistStamped
```

```yaml
# forks/turtlebot3_simulations/turtlebot3_gazebo/params/avoidance.yaml | shared parameters
linear_velocity: 0.15
angular_velocity: 0.8
stop_distance: 0.45
clear_distance: 0.55
side_distance: 0.40
scan_timeout: 0.5
front_half_angle_deg: 15.0
side_angle_deg: 60.0
```

## 12. PATCH-06 Python 구현

대상:

```text
# code/python/mobile_robot_lab_python | PATCH-06 files
mobile_robot_lab_python/obstacle_avoidance.py
test/test_obstacle_avoidance.py
setup.py
```

순서:

1. `rclpy.node.Node` 상속
2. 공통 parameter 선언·검증
3. `/scan` SensorDataQoS subscribe
4. `atan2(sin(angle), cos(angle))` angle 정규화
5. front/left/right sector minimum 계산
6. 양의 Inf는 `range_max`, NaN·음의 Inf는 무효
7. timeout·sector 없음이면 zero command
8. stop/clear hysteresis 적용
9. `TwistStamped` 발행
10. sector·invalid range pytest 작성

```python
# code/python/mobile_robot_lab_python/setup.py | setup()
entry_points={
    'console_scripts': [
        'extrinsic_math_py = mobile_robot_lab_python.extrinsic_math:main',
        'obstacle_avoidance_py = mobile_robot_lab_python.obstacle_avoidance:main',
    ],
},
```

## 13. PATCH-06 C++ 구현

대상:

```text
# code/cpp/mobile_robot_lab_cpp | PATCH-06 files
include/mobile_robot_lab_cpp/scan_utils.hpp
src/obstacle_avoidance.cpp
test/scan_utils_test.cpp
CMakeLists.txt
package.xml
```

기존 PATCH-06 알고리즘을 새 package에 작성한다. `turtlebot3_drive.cpp` 직접 수정은 제거한다.

순서:

1. `rclcpp::Node` 상속
2. Python과 같은 parameter·validation
3. 같은 sector·range 정책
4. 같은 timeout·hysteresis state
5. `TwistStamped` 발행
6. `obstacle_avoidance_cpp` executable 등록
7. Python과 같은 scan fixture test

```cmake
# code/cpp/mobile_robot_lab_cpp/CMakeLists.txt | obstacle target
add_executable(obstacle_avoidance_cpp src/obstacle_avoidance.cpp)
ament_target_dependencies(
  obstacle_avoidance_cpp
  geometry_msgs
  rclcpp
  sensor_msgs)

install(TARGETS obstacle_avoidance_cpp
  DESTINATION lib/${PROJECT_NAME})
```

## 14. launch 구현 선택

`turtlebot3_avoidance.launch.py`에 argument 추가:

```text
# turtlebot3_avoidance.launch.py | implementation contract
implementation=python: mobile_robot_lab_python/obstacle_avoidance_py
implementation=cpp:    mobile_robot_lab_cpp/obstacle_avoidance_cpp
그 외 값: launch error
```

```bash
ros2 launch turtlebot3_gazebo turtlebot3_avoidance.launch.py \
  scenario:=static implementation:=python

ros2 launch turtlebot3_gazebo turtlebot3_avoidance.launch.py \
  scenario:=static implementation:=cpp
```

두 node 동시 실행 금지.

## 15. 동일 scenario 비교

각 언어로 `static`, `crossing`, `mixed`를 별도 실행·기록한다.

```text
# data/bags | comparison artifact names
avoidance-python-static
avoidance-python-crossing
avoidance-python-mixed
avoidance-cpp-static
avoidance-cpp-crossing
avoidance-cpp-mixed
```

공통 topic:

```bash
ros2 bag record /scan /odom /cmd_vel /tf /tf_static /clock
```

완료 기준:

- 충돌 없음
- 최소 `/scan` 거리 `>= 0.25 m`
- scan 중단 후 `<= 0.5 s` 내 zero command
- 같은 parameter·시작 pose·장애물 궤적
- 같은 입력에서 같은 회피 방향 결정

wall-clock 성능은 비교하지 않는다. ROS timer·scheduler 차이는 정상.

## 16. 전체 build·test

```bash
source /opt/ros/jazzy/setup.bash
cd /ws

rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install \
  --packages-select \
    turtlebot3_gazebo \
    mobile_robot_lab_python \
    mobile_robot_lab_cpp

source /ws/install/setup.bash

colcon test \
  --packages-select mobile_robot_lab_python mobile_robot_lab_cpp
colcon test-result --verbose

ros2 pkg executables mobile_robot_lab_python
ros2 pkg executables mobile_robot_lab_cpp
```

현재 실행 파일은 `mobile_robot_lab_python obstacle_avoidance_py`, `mobile_robot_lab_cpp obstacle_avoidance_cpp`다. `extrinsic_math_py/cpp`는 PATCH-04 이중 구현 시 추가한다.

## 17. PATCH 반영 순서

| PATCH | 반영 내용 |
|---|---|
| PATCH-00 | `code/python/`, `code/cpp/` mount와 package build |
| PATCH-01 | 변경 없음. sensor/SDF/bridge 공통 |
| PATCH-02 | 변경 없음. world/bag 공통 |
| PATCH-03 | upstream calibration과 shell wrapper 공통 |
| PATCH-04 | Python/C++ extrinsic + parity test |
| PATCH-05 | 동적 장애물 plugin은 공통 simulator C++ |
| PATCH-06 | Python/C++ 회피 node + `implementation` arg |
| PATCH-07 | 두 언어로 같은 low-light 결과 검증 |
| PATCH-10 | Python/C++ raw collector parity, Python LeRobot exporter |

## 18. 최종 완료 조건

- `code/python/`, `code/cpp/` 아래 package가 각각 존재
- 두 package 같은 workspace에서 build·test 통과
- Extrinsic 결과 수치 tolerance 안에서 일치
- 회피 node가 같은 topic, type, parameter 사용
- launch argument로 언어 하나만 선택
- calibration fork 변경은 별도 실습 branch에 commit
- 학습용 회피 node가 TurtleBot3 upstream source 밖에 존재

## 19. 제외 범위

- pybind11
- Python/C++ shared library
- 공통 interface package
- benchmark framework
- Gazebo plugin Python 복제

독립 구현 두 개와 동일 입력 비교면 학습 목적 충족. 중복 유지가 실제 문제가 될 때만 공통 library 검토.
