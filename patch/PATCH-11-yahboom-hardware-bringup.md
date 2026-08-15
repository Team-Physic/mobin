# PATCH-11: Yahboom MicroROS-Pi5 실물 연결과 인터페이스 고정

- 작성일: 2026-08-15
- 선행 조건: PATCH-06, PATCH-10의 topic·dataset 계약 이해
- 대상: 향후 `python/`, `cpp/`, `config/yahboom/`, 실물 MicroROS-Pi5
- 결론: **Yahboom 기본 image와 firmware를 먼저 보존한 채 실제 node·topic·TF를 측정한다. 시뮬레이션의 `TwistStamped`와 실물의 `Twist` 차이는 작은 adapter에서만 변환한다.**

## 검증한 공개 자료

| 자료 | 확인한 내용 | 이 PATCH에서의 사용 |
|---|---|---|
| [Yahboom MicroROS-Pi5 학습 페이지](https://www.yahboom.net/study/MicroROS-Pi5) | micro-ROS agent, 속도 보정, URDF, LiDAR 회피, Nav2, rosbag, TF2 과정과 source·hardware·3D model 다운로드 제공 | 실물 bring-up 순서와 vendor 원본 위치 확인 |
| [Yahboom MicroROS-Car-Pi5](https://github.com/YahboomTechnology/MicroROS-Car-Pi5) | Raspberry Pi 5, ROS 2 Humble, ESP32, encoder motor, MS200 2D LiDAR, camera 구성 | 실제 센서와 OS 기준 확인 |
| [Yahboom MicroROS-Board](https://github.com/YahboomTechnology/MicroROS-Board) | ESP32 계열 보드의 motor·encoder·IMU·LiDAR·servo 및 micro-ROS 과정 | 저수준 제어 경계 확인 |

Yahboom GitHub 저장소와 연결된 다운로드에서 명시적 license를 확인하지 못했다. **공개 다운로드 가능과 재배포 허용은 같은 뜻이 아니다.** 원본 source·PDF·CAD는 로컬 실습에만 사용하고, 이 저장소에는 복사하지 않는다. 수정물을 배포하기 전 Yahboom에 license를 확인한다.

## 1. ROS 2 버전을 억지로 통일하지 않는다

| 영역 | 유지할 환경 | 이유 |
|---|---|---|
| 현재 Gazebo 개발 PC | ROS 2 Jazzy | PATCH-00 환경과 일치 |
| Yahboom Raspberry Pi 5 | vendor ROS 2 Humble image | motor·sensor·micro-ROS 구성을 먼저 보존 |
| ESP32 | vendor firmware | encoder PID와 hardware I/O를 첫 단계에서 다시 작성하지 않음 |

Jazzy package를 실물 Pi에 바로 덮어쓰지 않는다. ROS 2 distribution이 달라도 DDS 호환 여부를 실제 topic으로 확인하고, 문제가 있으면 같은 distribution의 bridge process를 추가한다.

## 2. 실물 원본을 백업한다

Pi에서 vendor workspace와 시작 script의 위치를 먼저 기록한다.

```bash
date -Iseconds
cat /etc/os-release
printenv ROS_DISTRO ROS_DOMAIN_ID RMW_IMPLEMENTATION
find /root /home -maxdepth 3 -type f \
  \( -name 'start_agent_rpi5.sh' -o -name 'ros2_humble.sh' \) 2>/dev/null
find /root /home -maxdepth 4 -type d -name 'yahboomcar_ws' 2>/dev/null
```

다음 항목을 `artifacts/yahboom/inventory/`에 text로 보관한다.

| 항목 | 목적 |
|---|---|
| OS image version과 ROS distribution | 재설치 기준 |
| workspace package 목록과 Git commit | 실행 source 식별 |
| ESP32 firmware version·설정값 | motor behavior 재현 |
| udev device와 serial baud rate | micro-ROS 연결 복원 |
| wheel radius·wheel separation·PID 값 | PATCH-10 system identification 초기값 |

`artifacts/`에는 password, Wi-Fi key, token을 저장하지 않는다.

## 3. micro-ROS agent와 robot node를 확인한다

공식 과정의 script로 agent와 Humble container를 시작한다. 이름이 다르면 2절에서 찾은 실제 경로를 쓴다.

```bash
sh ~/start_agent_rpi5.sh
```

새 terminal에서:

```bash
sh ~/ros2_humble.sh
source /opt/ros/humble/setup.bash
ros2 node list
ros2 node info /YB_Car_Node
ros2 topic list -t
```

문서에는 `/odom`과 `/odom_raw`가 혼재할 수 있다. 문서를 추측해 고르지 않고 실행 중인 node의 실제 출력으로 고정한다.

```bash
ros2 topic type /cmd_vel
ros2 topic info /cmd_vel --verbose
ros2 topic type /scan
ros2 topic hz /scan
ros2 topic echo /scan --field header --once
ros2 run tf2_tools view_frames
```

예상되는 실물 command type은 `geometry_msgs/msg/Twist`다. 결과가 다르면 아래 계약보다 runtime 결과를 우선하고 문서를 갱신한다.

## 4. 프로젝트 내부 계약을 정한다

학습·회피 node는 transport message가 아니라 다음 값만 다룬다.

| 값 | 단위 | 규칙 |
|---|---:|---|
| `linear_x` | m/s | 전진 양수, 실물 제한 적용 전 command |
| `angular_z` | rad/s | 반시계 양수 |
| `stamp` | ROS time | command 생성 시각 |
| `source` | 문자열 | teleop, avoidance, policy 구분 |

계획 파일:

```text
python/mobile_robot_lab_python/mobile_robot_lab_python/yahboom_cmd_adapter.py
cpp/mobile_robot_lab_cpp/src/yahboom_cmd_adapter.cpp
config/yahboom/interfaces.yaml
```

| 경로 | 입력 | 출력 |
|---|---|---|
| simulation adapter | 내부 command | `/cmd_vel` `TwistStamped` |
| Yahboom adapter | 내부 command | `/cmd_vel` `Twist` |

두 구현을 동시에 실물에 연결하지 않는다. Python으로 계약을 먼저 검증하고, 제어 주기나 지연이 기준을 넘을 때만 C++ 구현을 사용한다.

## 5. 바퀴를 띄운 상태에서 최소 command를 검증한다

**로봇 바퀴를 지면에서 띄우고 작업자 손이 바퀴에서 떨어진 상태**에서만 처음 실행한다.

```bash
ros2 topic pub --rate 10 --times 10 /cmd_vel \
  geometry_msgs/msg/Twist \
  '{linear: {x: 0.03}, angular: {z: 0.0}}'

ros2 topic pub --once /cmd_vel \
  geometry_msgs/msg/Twist \
  '{linear: {x: 0.0}, angular: {z: 0.0}}'
```

다음을 확인한다.

| 검사 | 통과 조건 |
|---|---|
| 방향 | 양의 `linear.x`에서 네 바퀴가 전진 방향 |
| 정지 | zero command 뒤 즉시 감속·정지 |
| encoder | 구동 바퀴의 tick 방향이 일관됨 |
| odometry | 전진에서 x 증가, 제자리 반시계 회전에서 yaw 증가 |
| watchdog | publisher 종료 뒤 지속 주행하지 않음 |

watchdog이 없거나 확인되지 않으면 지면 주행으로 넘어가지 않는다.

## 6. 실물 sensor contract를 기록한다

| 정보 | 확인 명령 | 산출물 |
|---|---|---|
| topic type·publisher | `ros2 topic info --verbose` | `topics.txt` |
| 평균 주기 | `ros2 topic hz` | `rates.txt` |
| frame ID | `ros2 topic echo ... --field header --once` | `frames.txt` |
| TF tree | `view_frames` | `frames.pdf`, YAML |
| QoS | `ros2 topic info --verbose` | `qos.txt` |

최소 입력은 `/scan`, IMU, odometry, encoder 또는 joint state다. Camera는 처음부터 RL 입력으로 묶지 않는다. LiDAR 기반 회피가 실물에서 먼저 통과해야 한다.

## 7. 완료 조건

- vendor image, workspace, firmware 설정을 복원할 수 있는 inventory 존재
- 실제 `/YB_Car_Node`의 subscriber·publisher·service 목록 기록
- `/cmd_vel` 실제 type과 simulation type 차이 문서화
- 바퀴를 띄운 command·zero command·watchdog 검사 통과
- `/scan`, IMU, odometry의 주기와 frame ID 기록
- Python adapter와 C++ adapter가 같은 입력에서 같은 `linear.x`, `angular.z` 생성
- Yahboom 자산을 저장소에 넣지 않고 license 확인 필요 상태 표시

이 PATCH에서는 장애물 회피나 RL policy를 실행하지 않는다. **실물 인터페이스가 측정으로 고정된 뒤 PATCH-12로 진행한다.**
