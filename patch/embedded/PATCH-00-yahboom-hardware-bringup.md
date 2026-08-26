# Embedded PATCH-00: Yahboom MicroROS-Pi5 실물 연결과 인터페이스 고정

- 작성일: 2026-08-15
- 선행 조건: Simulation PATCH-06, Simulation PATCH-10의 topic·dataset 계약 이해
- 대상: 향후 `code/python/`, `code/cpp/`, `config/yahboom/`, 실물 MicroROS-Pi5
- 결론: **Yahboom 기본 image와 firmware를 먼저 보존한 채 실제 node·topic·TF를 측정한다. 시뮬레이션의 `TwistStamped`와 실물의 `Twist` 차이는 작은 adapter에서만 변환한다.**

## 검증한 공개 자료

| 자료 | 확인한 내용 | 이 PATCH에서의 사용 |
|---|---|---|
| [Yahboom MicroROS-Pi5 학습 페이지](https://www.yahboom.net/study/MicroROS-Pi5) | micro-ROS agent, 속도 보정, URDF, LiDAR 회피, Nav2, rosbag, TF2 과정과 source·hardware·3D model 다운로드 제공 | 실물 bring-up 순서와 vendor 원본 위치 확인 |
| [Yahboom MicroROS-Car-Pi5](https://github.com/YahboomTechnology/MicroROS-Car-Pi5) | Raspberry Pi 5, ROS 2 Humble, ESP32, encoder motor, MS200 2D LiDAR, camera 구성 | 실제 센서와 OS 기준 확인 |
| [Yahboom MicroROS-Board](https://github.com/YahboomTechnology/MicroROS-Board) | ESP32 계열 보드의 motor·encoder·IMU·LiDAR·servo 및 micro-ROS 과정 | 저수준 제어 경계 확인 |

Yahboom GitHub 저장소와 연결된 다운로드에서 명시적 license를 확인하지 못했다. **공개 다운로드 가능과 재배포 허용은 같은 뜻이 아니다.** 원본 source·PDF·CAD는 로컬 실습에만 사용하고, 이 저장소에는 복사하지 않는다. 수정물을 배포하기 전 Yahboom에 license를 확인한다.

## Embedded

### 개념

임베디드 개발자는 PC용 범용 프로그램만 만드는 사람이 아니라 **특정 장치의 sensor·actuator·전원·시간·memory 제약 안에서 software가 반복 가능하게 동작하도록 만드는 사람**이다. 회사와 제품에 따라 담당 계층이 다르므로 firmware 개발자, Embedded Linux 개발자, robot control 개발자가 같은 직무는 아니다.

| 개념 | 쉬운 설명 | 이 프로젝트의 실제 대상 |
|---|---|---|
| MCU | CPU, flash, RAM, GPIO와 통신 주변장치가 한 chip에 들어간 제어용 컴퓨터 | Yahboom ESP32가 encoder를 읽고 motor를 제어 |
| MPU/SoC | Linux와 여러 process를 실행할 수 있는 고성능 processor system | Raspberry Pi 5가 ROS 2, camera, logging 실행 |
| firmware | MCU가 부팅한 뒤 hardware register·driver·제어 loop를 실행하는 software | ESP32 motor·encoder·IMU·micro-ROS code |
| RTOS | 우선순위와 주기로 task 실행을 조정하는 작은 운영체제 | motor I/O, communication, diagnostic task 분리 |
| real-time | 평균 속도가 아니라 정해진 deadline 안에 결과를 내는 성질 | command timeout 전에 motor safe state 진입 |
| bootloader | application보다 먼저 실행되어 초기화·검증·firmware loading 수행 | 향후 firmware update와 rollback의 시작점 |
| BSP | 특정 board의 clock, pin, memory, peripheral을 OS·RTOS가 쓰게 하는 묶음 | vendor Pi image와 ESP32 board 설정 |
| device driver | UART·I2C·SPI·CAN·sensor 같은 hardware를 일관된 API로 노출 | Linux serial device와 ESP32 peripheral driver |
| middleware | driver와 application 사이에서 통신·serialization API 제공 | DDS, ROS 2, micro-ROS |

**RTOS를 사용한다고 자동으로 real-time이 되지는 않는다.** task priority, interrupt 처리시간, 공유 자원, 최악 실행시간과 deadline miss를 실제로 측정해야 한다.

```text
# planned embedded boundary
Raspberry Pi 5 / Linux / ROS 2
        ↓ Twist command, configuration
micro-ROS agent
        ↓ serial 또는 UDP transport
ESP32 / firmware / motor·encoder loop
        ↓ PWM·GPIO·encoder
physical motor and sensor
```

### 채용공고에서 반복되는 역량

아래 공고는 2026-08-15에 확인했다. 공고 내용과 마감 상태는 바뀔 수 있으므로 링크의 최신 원문을 다시 확인한다. 공고 문장을 저장소에 복사하지 않고 이 프로젝트와 연결되는 요구역량만 요약한다.

| 근거 | 반복 요구역량 | 이 로드맵의 실습 |
|---|---|---|
| [에이치오피 임베디드 개발자](https://www.jobkorea.co.kr/Recruit/GI_Read/49687448) | 공개 metadata의 C, Python, SLAM, embedded hardware, firmware | C/C++ firmware와 Python/ROS 2 검증 분리 |
| [로보톰 임베디드 로봇 제어](https://www.jobkorea.co.kr/Recruit/GI_Read/49350280) | C/C++, ROS 2, LiDAR, CAN, BLE, OTA | Embedded PATCH-00~02와 Embedded PATCH-05 |
| [D.Hive 임베디드 firmware](https://www.wanted.co.kr/wd/366867) | MCU, RTOS, sensor driver, ROS 2 bridge, power, UART/CAN, OTA, 계측 | Embedded PATCH-05 protocol·timing·power 검증 |
| [Polaris3D Embedded Linux](https://www.wanted.co.kr/wd/307655) | kernel/system log, C++, driver·IPC·network, GDB·strace·perf, BSP | Pi 5 service와 장애 재현·원인 분석 |
| [아르비젼 Embedded software](https://www.wanted.co.kr/wd/358905) | MCU firmware와 UART·SPI·I2C·CAN driver | ESP32 peripheral과 protocol 단위 시험 |
| [아그모 Embedded AI](https://www.wanted.co.kr/wd/371576) | edge model 최적화, sensor fusion, data lifecycle | Simulation PATCH-11~12의 onboard benchmark |

[임베디드 개발 계층에 대한 현업자 설명](https://gall.dcinside.com/mini/board/view/?id=embedded&no=4348&page=1)은 SoC의 HW→bootloader→OS→driver→middleware→application과 MCU의 HW→firmware 또는 HW→RTOS→application 구분을 이해하는 보조 자료다. 개인 경험에 기반한 글이므로 정의의 단독 근거로 쓰지 않고, 위 채용공고와 [Zephyr 공식 문서](https://docs.zephyrproject.org/latest/index.html), [FreeRTOS 공식 문서](https://www.freertos.org/Documentation/00-Overview), [micro-ROS 개념 문서](https://micro.ros.org/docs/concepts/)로 교차 확인한다.

### 신입·1~2년차 설계 범위

| 직접 설계·검증할 것 | 기존 구현을 먼저 재사용할 것 | 이 단계에서 맡지 않을 것 |
|---|---|---|
| topic·frame·단위·rate·timeout 계약 | Yahboom vendor image와 motor firmware | custom SoC·BSP 전체 porting |
| command sequence와 stale-command 판정 | micro-ROS transport와 DDS | 안전 인증 또는 양산용 OTA |
| UART/UDP packet log와 dropped sequence 측정 | Linux·ESP32 공식 driver | motor power stage 회로 재설계 |
| service 재시작·sensor disconnect fault 시험 | 검증된 bootloader | kernel driver가 불필요한 장치의 custom driver |

신입은 vendor system을 재현하고 관측 가능한 계약을 만든다. 1~2년차 범위는 작은 adapter·protocol·watchdog를 설계하고 fault를 재현해 root cause와 regression test까지 남기는 수준이다. **vendor firmware 전체 재작성은 학습량이 아니라 위험과 검증 범위만 키우므로 제외한다.**

### GitHub에 남길 증거

| 문서 | 기록할 내용 |
|---|---|
| `docs/embedded/00_system_inventory.md` | board revision, OS·ROS·firmware version, topic·TF, commit |
| `docs/embedded/00_interface_contract.md` | message type, field, unit, rate, timestamp, timeout, error 상태 |
| `docs/embedded/00_bringup_report.md` | 실행 명령, 실제 log, 측정 결과, 실패와 해결, 남은 제한 |

password, Wi-Fi key, token은 문서와 log에서 제거한다. 측정하지 않은 rate·latency는 `예상`이나 `계획`으로 표시하며 성공 결과처럼 쓰지 않는다.

### SW 실습

| 실습 | 실행 위치·입력 | 산출물 | 통과 조건 |
|---|---|---|---|
| ROS 2 inventory | Pi 5에서 실행 중인 node·topic·service·TF 조회 | `00_system_inventory.md` | 실제 message type, frame ID, QoS, 평균 rate 기록 |
| command adapter | 저장한 `linear_x`, `angular_z`, `stamp`, `source` test vector를 Python·C++ adapter에 입력 | 자동 비교 결과 | 두 구현의 출력 type과 수치가 계약대로 일치 |
| transport fault | micro-ROS agent 또는 robot node를 중단·재시작 | disconnect·recovery log | stale command가 계속 전달되지 않고 재시작 절차가 재현됨 |
| interface 문서화 | runtime 출력과 firmware·workspace version 정리 | `00_interface_contract.md`, `00_bringup_report.md` | 추정값과 실측값이 구분되고 secret이 없음 |

보드가 없어도 adapter의 변환과 test vector 비교는 PC에서 수행할 수 있다. 다만 **실제 topic rate·watchdog·motor 방향은 실물 측정 전까지 통과로 표시하지 않는다.**

### HW 실습

| 실습 | 필요한 장비 | 측정값·산출물 | 통과 조건 |
|---|---|---|---|
| board·배선 inventory | Yahboom, Pi 5, ESP32, camera, LiDAR | board revision, port, cable, 전원 구성 | 사진·표와 실제 연결이 일치하고 secret이 없음 |
| 공중 구동 | 바퀴를 띄울 지지대, 즉시 전원 차단 수단 | 바퀴 방향, encoder 부호, odometry 변화 | 전진·회전 부호가 계약과 일치 |
| watchdog | command publisher 중단 수단 | 마지막 command부터 정지까지 시간 | publisher 종료 뒤 지속 주행하지 않음 |
| sensor 점검 | LiDAR 가림판, IMU 자세 변경 | `/scan` range 변화, IMU 축, timestamp | 물리 입력 변화와 ROS message 변화가 일치 |
| 전원 재기동 | 안정된 전원과 복구 image | 부팅·agent·node 재시작 순서 | 백업한 절차로 동일 interface 복원 |

**HW 통과 기준은 명령이 발행됐다는 log가 아니라 바퀴·encoder·sensor·watchdog의 실제 반응이다.**

## 1. ROS 2 버전을 억지로 통일하지 않는다

| 영역 | 유지할 환경 | 이유 |
|---|---|---|
| 현재 Gazebo 개발 PC | ROS 2 Jazzy | Simulation PATCH-00 환경과 일치 |
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
| wheel radius·wheel separation·PID 값 | Simulation PATCH-10 system identification 초기값 |

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
code/python/mobile_robot_lab_python/mobile_robot_lab_python/yahboom_cmd_adapter.py
code/cpp/mobile_robot_lab_cpp/src/yahboom_cmd_adapter.cpp
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

이 PATCH에서는 장애물 회피나 RL policy를 실행하지 않는다. **실물 인터페이스가 측정으로 고정된 뒤 Embedded PATCH-01로 진행한다.**
