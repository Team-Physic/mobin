# Mobin Parallel Robot Development Roadmap

Simulation과 Embedded 실습을 별도 트랙으로 관리한다. 두 디렉터리는 각각 `PATCH-00`부터 시작한다.

| 트랙 | 실행 대상 | 검증하는 것 | 검증하지 못하는 것 |
|---|---|---|---|
| `simulation/` | Docker, Gazebo Harmonic, ROS 2, Isaac Lab | 알고리즘, sensor model, 반복 가능한 scenario, 학습 | 실제 전원·열·통신·제동·제작 공차 |
| `embedded/` | Raspberry Pi 5, ESP32/MCU, sensor, motor, 제작물 | 실제 I/O, timing, 전원, 열, fault, actuator 반응 | 충분하지 않은 simulation coverage |

**두 트랙은 병렬로 시작할 수 있다. 실물 command를 허용하는 통합 지점에서만 선행 결과를 함께 검사한다.**

## 디렉터리

```text
# patch/README.md | roadmap layout
patch/
├── simulation/  # PC·Docker·Gazebo·학습 중심
└── embedded/    # board·firmware·전장·실물 검증 중심
    ├── core/    # 임베디드 공통 core lab
    ├── core/PATCH-00-embedded-job-concepts.md
    ├── core/PATCH-01-prerequire-esp32-connection.md
    ├── core/PATCH-02-esp32-c-mcu-baremetal.md
    ├── core/PATCH-03-esp32-rtos-timing-watchdog.md
    ├── core/PATCH-04-peripheral-driver-protocol.md
    ├── core/PATCH-05-pi5-embedded-linux-bsp.md
    ├── core/PATCH-06-debug-test-ci-hil.md
    ├── core/PATCH-07-board-soldering-rtos-hil.md
    ├── core/PATCH-08-product-engineering-and-certification.md
    └── Yahboom·Humanoid 프로젝트 적용 PATCH
```

Embedded PATCH의 `SW 실습`은 PC·Pi·CI에서 수행하는 firmware test, ROS 2 node, protocol, logging, model runtime을 뜻한다. `HW 실습`은 실제 board·sensor·motor·전원·계측기가 있어야 하는 측정을 뜻한다. **SW test 통과를 HW 통과로 기록하지 않는다.**

## 내려받은 리포

| 디렉터리 | 기준 branch / commit | 용도 |
|---|---|---|
| `forks/turtlebot3_simulations/` | `jazzy` / `45633014a14e8f438495b532a723e4ad45cbbd31` | Gazebo Sim robot, sensor, world |
| `forks/direct_visual_lidar_calibration/` | `main` / `02a0dc039f5509708f384be4ff3228e0ae09352d` | 3D LiDAR-Camera extrinsic calibration |
| `forks/aws-robomaker-small-warehouse-world/` | `ros2` / `ee0af733315e78432408c3cd98d378ecee5f767c` | ROS 2 warehouse asset의 Harmonic 이식 |

Simulation PATCH-11 구현 시 [ROBOTIS `turtlebot3_machine_learning`](https://github.com/ROBOTIS-GIT/turtlebot3_machine_learning)의 공식 `jazzy` branch를 fork해 `forks/turtlebot3_machine_learning/`에 추가한다. 지금은 dependency로 고정하지 않는다.

Fork, `origin`/`upstream`, 실습 branch, license 의무는 [Fork workflow와 license 준수](../docs/how_to_fork_and_license.md)를 따른다.

## 기본 선택

| 항목 | 기본값 |
|---|---|
| host | Ubuntu 24.04, ROS 2·Gazebo는 Docker에서 실행 |
| robot | `TURTLEBOT3_MODEL=waffle_pi` |
| calibration 파생 모델 | `waffle_pi_3d`, 기존 2D 측정을 3D `PointCloud2` 측정으로 교체 |
| camera topic | `/camera/image_raw`, `/camera/camera_info` |
| calibration 초기값 | manual initial guess |
| source 관리 | `forks/`의 독립 Git 저장소에서 수정 |

## Simulation 트랙

1. [Simulation PATCH-00: Jazzy·Gazebo Docker 환경](simulation/PATCH-00-jazzy-gazebo-docker-setup.md)
2. [Simulation PATCH-01: 2D LiDAR 측정을 3D LiDAR 측정으로 교체](simulation/PATCH-01-replace-2d-lidar-with-3d-lidar.md)
3. [Simulation PATCH-02: Calibration scene과 MCAP 기록](simulation/PATCH-02-calibration-scene-recording.md)
4. [Simulation PATCH-03: Extrinsic 계산](simulation/PATCH-03-run-calibration.md)
5. [Simulation PATCH-04: URDF 반영과 정량 검증](simulation/PATCH-04-apply-and-verify.md)
6. [Simulation PATCH-05: AWS Warehouse의 Gazebo Harmonic 이식](simulation/PATCH-05-obstacle-scenarios.md)
7. [Simulation PATCH-06: 장애물 회피 node](simulation/PATCH-06-obstacle-avoidance.md)
8. [Simulation PATCH-07: 저조도 터널 calibration 강건성](simulation/PATCH-07-low-light-tunnel-robustness.md)
9. [Simulation PATCH-08: Behavior Tree calibration workflow](simulation/PATCH-08-behavior-tree-calibration-orchestration.md)
10. [Simulation PATCH-09: GitHub Actions CI/CD](simulation/PATCH-09-github-actions-ci-cd.md)
11. [Simulation PATCH-10: Sim2Real dataset과 domain randomization](simulation/PATCH-10-sim2real-dataset-collection.md)
12. [Simulation PATCH-11: Mobile Robot 강화학습](simulation/PATCH-11-mobile-robot-reinforcement-learning.md)
13. [Simulation PATCH-12: Humanoid Isaac Lab Sim2Real](simulation/PATCH-12-humanoid-isaac-lab-sim2real.md)
14. [Simulation PATCH-13: LiDAR–BBox Association과 3D Tracking 평가](simulation/PATCH-13-lidar-association-and-3d-tracking-evaluation.md)

Simulation PATCH-00~06은 앞 PATCH의 완료 조건을 따른다. PATCH-07·08은 PATCH-04 뒤 병렬 진행 가능하다. PATCH-09 기본 CI는 PATCH-00 뒤 시작하고, 구현된 test만 추가한다. PATCH-10은 PATCH-02의 기록 계약과 PATCH-05~07의 scenario 결과를 사용한다. PATCH-11은 PATCH-05·06·10이 필요하다. PATCH-12 asset 작업은 Embedded PATCH-04 URDF가 생성된 뒤 시작한다. PATCH-13은 PATCH-06 association 결과와 PATCH-10 기록 형식을 사용한다.

## Embedded 트랙

0. [Embedded Core PATCH-00: 임베디드 채용공고 개념 기반 core 재설계](embedded/core/PATCH-00-embedded-job-concepts.md) — C/MCU·RTOS·protocol·Linux/BSP·debug·board
1. [Embedded Core PATCH-01: ESP32 연결·serial 확인](embedded/core/PATCH-01-prerequire-esp32-connection.md) — USB-UART·port·esptool
2. [Embedded Core PATCH-02: ESP32 C/MCU bare-metal](embedded/core/PATCH-02-esp32-c-mcu-baremetal.md) — GPIO·timer·ISR·memory·linker
3. [Embedded Core PATCH-03: ESP32 FreeRTOS task·timing·watchdog](embedded/core/PATCH-03-esp32-rtos-timing-watchdog.md) — task·queue·mutex·timer·watchdog·jitter
4. [Embedded Core PATCH-04: UART·I2C·SPI·CAN](embedded/core/PATCH-04-peripheral-driver-protocol.md) — packet·fault·loopback
5. [Embedded Core PATCH-05: Pi 5 Embedded Linux·BSP](embedded/core/PATCH-05-pi5-embedded-linux-bsp.md) — gpiod·overlay·SocketCAN·systemd
6. [Embedded Core PATCH-06: debug·test·CI·HIL](embedded/core/PATCH-06-debug-test-ci-hil.md) — GDB·logic analyzer·unit test·CI·HIL
7. [Embedded Core PATCH-07: 납땜 보드·Pi 5 HAT·RTOS HIL](embedded/core/PATCH-07-board-soldering-rtos-hil.md) — soldering·e-stop·board safety
8. [Embedded Core PATCH-08: 제품화·양산·인증](embedded/core/PATCH-08-product-engineering-and-certification.md) — DFM/DFT·firmware release·KC/CE/FCC/RoHS·양산 test·reliability·cost

### 프로젝트 적용

1. [Yahboom PATCH-00: 실물 bring-up과 interface](embedded/PATCH-00-yahboom-hardware-bringup.md) — SW: adapter·topic 계약, HW: motor·sensor·watchdog
2. [Yahboom PATCH-01: 안전 장애물 회피](embedded/PATCH-01-yahboom-safe-obstacle-avoidance.md) — SW: controller·fault test, HW: 정지거리·장애물 시험
3. [Yahboom PATCH-02: RL Sim2Real](embedded/PATCH-02-yahboom-rl-sim2real.md) — SW: replay·shadow·artifact, HW: Pi 부하·guarded 주행
4. [Humanoid PATCH-03: reference와 요구사항](embedded/PATCH-03-humanoid-reference-and-requirements.md) — SW: license·architecture·budget, HW: 부품 실측·single-joint rig
5. [Humanoid PATCH-04: CAD와 URDF](embedded/PATCH-04-humanoid-cad-and-urdf.md) — SW: CAD·export·validator, HW: 출력 공차·fit·mass·thermal
6. [Humanoid PATCH-05: 전장과 ROS 2 제어](embedded/PATCH-05-humanoid-electronics-and-control.md) — SW: firmware·protocol·fake hardware, HW: 전원·MCU·HIL

Embedded Core PATCH-01~06은 ESP32·Pi 5로 채용공고 개념을 익히는 공통 실습이다. Core PATCH-07의 safety chain과 Humanoid PATCH-05의 power budget·single-joint rig를 통과한 뒤에만 motor를 실제로 구동한다. Yahboom·Humanoid PATCH는 core lab과 병렬로 조사할 수 있지만, motor 명령은 core UART packet·RTOS watchdog·board safety가 끝난 뒤 허용한다.

제품 판매를 목표로 하려면 prototype 동작으로 끝내지 않고 Core PATCH-08의 DFM·인증·양산 test·reliability·문서·비용 gate까지 통과해야 한다.

## 통합 지점

| 통합 지점 | 필요한 Simulation 결과 | 필요한 Embedded 결과 | 허용되는 다음 행동 |
|---|---|---|---|
| Yahboom 결정론적 제어 | Simulation PATCH-06 controller·metric | Embedded PATCH-00 interface·watchdog | Embedded PATCH-01 저속 실물 시험 |
| Yahboom RL | Simulation PATCH-11 policy, PATCH-10 dataset | Embedded PATCH-01 safety baseline | Embedded PATCH-02 replay→shadow→guarded |
| Humanoid asset | Simulation physics 요구사항 | Embedded PATCH-04 URDF·mass·collision | Simulation PATCH-12 USD 변환·학습 |
| Humanoid 실물 policy | Simulation PATCH-12 policy·benchmark | Embedded PATCH-05 timing·power·fault 통과 | tethered rig부터 제한 실행 |

## 의도적으로 제외한 범위

- 원본 `waffle_pi`의 2D LDS는 수정하지 않는다. 파생 `waffle_pi_3d`만 변경한다.
- Calibration 기록에는 동적 장애물을 넣지 않는다. 움직이는 물체는 image와 point cloud의 같은 표면 대응을 깨뜨린다.
- Nav2 전체 stack은 Simulation PATCH-06의 단순 회피 뒤 목표점 주행이 필요할 때 추가한다.
- Yahboom source·PDF·3D model은 명시적 재배포 license 확인 전 저장소에 복사하지 않는다.
- 첫 Humanoid에서 손·팔·계단·vision policy를 동시에 만들지 않는다.
- 공개 Humanoid CAD를 자체 설계인 것처럼 복제하지 않는다. component별 license와 attribution을 확인한다.
