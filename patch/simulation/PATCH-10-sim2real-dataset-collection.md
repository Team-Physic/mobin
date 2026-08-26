# Simulation PATCH-10: Sim2Real Dataset 수집과 Domain Randomization

- 작성일: 2026-08-07
- 브랜치: 상위 `main`, TurtleBot3 fork `jazzy`
- 코드 기준: `turtlebot3_simulations@45633014`, `direct_visual_lidar_calibration@02a0dc03`, 상위 working tree
- 대상: 향후 `config/`, `data/`, `code/python/mobile_robot_lab_python/`, `code/cpp/mobile_robot_lab_cpp/`
- 결론: **ROS 2 sensor·action은 episode 단위 MCAP으로 먼저 보존하고 Python에서 LeRobot으로 변환한다. 실제 robot 측정으로 nominal 제어·물리 parameter를 구한 후 그 주변을 randomization하며, 모든 seed와 적용값을 manifest에 기록한다.**

### Why?

Simulation PATCH-02는 Camera-LiDAR calibration용 rosbag 기록을 계획한다. IL/RL 데이터로 확장하려면 episode 경계, action, task 결과, controller 설정, randomization 적용값과 split 정보가 추가로 필요하다.

| 현재 부족한 정보 | 필요한 이유 |
|---|---|
| episode 시작·종료·성공 여부 | 성공/실패 trajectory 구분 |
| observation과 실제 적용 action의 동기 | 학습 pair 구성 |
| world, robot, controller 설정 | 실행 조건 재현 |
| random seed와 실제 적용값 | domain randomization 재현 |
| collision, timeout, drop count | 품질 검사와 reward 계산 |
| train/validation/test 구분 | 같은 episode frame이 섞이는 data leakage 방지 |

ROS 원본 topic, TF, 가변 길이 `PointCloud2`, replay 가능성은 MCAP에 보존한다. LeRobot은 이 원본에서 생성하는 학습용 파생 데이터로 둔다. 변환 규격이 바뀌어도 simulation을 다시 실행할 필요가 없다.

### 개념

| 개념 | 쉬운 설명 | 이 PATCH에서 필요한 이유 |
|---|---|---|
| LeRobot | robot·teleoperator·sensor·policy·학습·dataset 도구를 제공하는 전체 오픈소스 프로젝트 | 데이터 수집부터 학습·시각화까지 연결할 때 사용 |
| Episode | reset부터 성공, 충돌, timeout 또는 사용자 중단까지 한 번의 실행 | 데이터 분할·평가의 최소 단위 |
| Observation | camera, LiDAR, IMU, odometry처럼 정책이 판단에 쓰는 입력 | action과 같은 시간축으로 저장 |
| Action | robot에 최종 적용된 선속도·각속도 명령 | 사람이거나 policy가 내린 행동을 학습 label로 사용 |
| System identification | 실제 robot 반응에 가까운 mass, friction, wheel, motor, delay 값을 찾는 과정 | randomization 중심값과 범위를 현실에 맞춤 |
| Domain randomization | 물리·sensor·환경값을 episode마다 바꾸는 방법 | 한 simulation 조건에 대한 과적합 완화 |
| Sim2Real gap | simulator와 실제 robot의 동역학·sensor·지연·환경 차이 | simulation 성능과 실제 성능을 따로 측정해야 하는 이유 |

`turtlebot3_dqn_stage1.world`부터 `stage4.world`는 강화학습용 **Gazebo 환경 파일**이다. 미래 상태를 예측하는 신경망 기반 **world model**은 아니다. 이번 PATCH는 learned world model을 만들지 않는다.

### What I Made

구현 전 데이터 계약과 실행 순서만 정의한다. 현재 단계에서 collector package나 LeRobot 의존성을 추가하지 않는다.

```text
# mobile-robot-calibration-repo | planned data flow
Teleoperation / policy
          ↓
ROS 2 action + sensor topics
          ↓
MCAP + episode manifest ───→ ros2 bag replay / calibration debug
          ↓
Python LeRobot exporter
          ↓
LeRobot ─────────→ visualization / IL·RL training
```

### What was problem

rosbag만 실행하면 sensor message는 남지만 어떤 실험인지 판단하기 어렵다. seed만 저장해도 부족하다. library version이나 sampling 순서가 바뀔 수 있으므로 실제로 적용된 parameter 값까지 기록해야 재현할 수 있다.

또한 임의의 넓은 randomization 범위는 현실과 무관한 trajectory를 만든다. 실제 robot response로 nominal parameter를 먼저 추정하고, 측정 오차와 제조 공차 안에서 범위를 정해야 한다.

### How it changed

| 이전 계획 | Simulation PATCH-10 계획 | 효과 |
|---|---|---|
| calibration rosbag | episode MCAP + manifest | replay와 학습 metadata 동시 보존 |
| simulation 기본값 고정 | 측정 기반 nominal + randomization profile | 물리 조건별 성능 비교 |
| Python/C++ algorithm 비교 | Python/C++ raw collector contract 추가 | 언어별 수집 결과 parity 확인 |
| 별도 dataset 형식 없음 | Python LeRobot exporter | 공식 학습·시각화 도구 사용 |

## 1. 계획 디렉터리

```text
# mobile-robot-calibration-repo | planned Simulation PATCH-10 layout
mobile-robot-calibration-repo/
├── config/
│   ├── dataset/collector.yaml
│   └── randomization/
│       ├── nominal.yaml
│       ├── train.yaml
│       └── stress.yaml
├── data/
│   ├── raw/<run_id>/<episode_id>/
│   │   ├── episode.mcap
│   │   └── manifest.json
│   └── lerobot/<dataset_name>/
├── code/python/mobile_robot_lab_python/mobile_robot_lab_python/
│   ├── episode_orchestrator.py
│   ├── domain_randomizer.py
│   └── lerobot_exporter.py
└── code/cpp/mobile_robot_lab_cpp/src/
    └── dataset_collector.cpp
```

`data/`는 Git에 commit하지 않는다. schema 예제와 작은 test fixture만 `test/fixtures/`에 둔다.

## 2. Python/C++ 역할

| 기능 | Python | C++ | 결정 |
|---|---|---|---|
| episode 반복·종료 판정 | 구현 | 미구현 | orchestration 한 곳만 유지 |
| seed sampling·Gazebo parameter 적용 | 구현 | 미구현 | 실험 자동화는 Python |
| 고주기 ROS topic 수집 | 구현 | 구현 | 같은 MCAP·manifest contract |
| MCAP 기록 | `rosbag2_py` 또는 `ros2 bag` 재사용 | `rosbag2_cpp` 재사용 | 자체 bag format 금지 |
| LeRobot 변환 | 구현 | 미구현 | 공식 Python API 사용 |
| 회피 controller | 기존 계획 유지 | 기존 계획 유지 | action producer와 recorder 분리 |

LeRobot writer를 C++로 다시 만들지 않는다. C++ collector 결과도 같은 MCAP·manifest를 거쳐 Python exporter로 변환한다.

## 3. Episode lifecycle

| 순서 | 동작 | 저장 증거 |
|---:|---|---|
| 1 | `run_id`, `episode_id`, seed 생성 | manifest 초안 |
| 2 | randomization 값을 한 번 sampling | 요청값과 실제 적용값 |
| 3 | world와 robot reset | world, robot model, 시작 pose |
| 4 | sensor 준비와 ROS time 진행 확인 | topic별 첫 timestamp |
| 5 | MCAP 기록 시작 | `episode.mcap` |
| 6 | teleoperation 또는 controller 실행 | requested/applied action |
| 7 | 성공·충돌·timeout·사용자 중단 판정 | termination reason |
| 8 | recorder 종료와 bag 검사 | topic count, drop count, SHA-256 |
| 9 | manifest 완료를 원자적으로 반영 | `manifest.json` |
| 10 | 선택 episode만 LeRobot으로 변환 | 파생 dataset과 변환 log |

중간 실패 episode도 삭제하지 않는다. `valid=false`와 실패 사유를 기록한다.

## 4. ROS 원본 topic

현재 Waffle Pi SDF와 bridge 설정에서 확인된 topic 기준이다.

| ROS topic | type | 주기/용도 | 보존 |
|---|---|---|---|
| `/camera/image_raw` | `sensor_msgs/msg/Image` | 30 Hz RGB | 필수 |
| `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | intrinsics | 필수 |
| `/scan` | `sensor_msgs/msg/LaserScan` | 10 Hz, 360 sample | 필수 |
| `/imu` | `sensor_msgs/msg/Imu` | 200 Hz | 필수 |
| `/odom` | `nav_msgs/msg/Odometry` | 30 Hz | 필수 |
| `/joint_states` | `sensor_msgs/msg/JointState` | wheel state | 필수 |
| `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | frame 관계 | 필수 |
| `/cmd_vel` | `geometry_msgs/msg/TwistStamped` | applied action | 필수 |
| `/clock` | `rosgraph_msgs/msg/Clock` | simulation time | simulation 필수 |
| `/calib/points` | `sensor_msgs/msg/PointCloud2` | Simulation PATCH-01 3D LiDAR | calibration 실험 필수 |

각 sensor의 원본 timestamp와 주기를 유지한다. exporter만 10 Hz 기준 frame으로 sampling한다.

## 5. Manifest contract

| field | 규칙 |
|---|---|
| `schema_version` | reader 호환성 version |
| `run_id`, `episode_id` | UUID, 디렉터리와 동일 |
| `seed` | randomization 재현 키 |
| `task`, `world`, `robot_model` | 실험 입력 |
| `source_commits` | 두 fork의 실제 SHA |
| `collector_impl`, `controller_impl` | Python/C++/teleop 구분 |
| `controller_params` | 실제 적용 gain·limit |
| `physics_params` | mass, friction, wheel, motor, delay |
| `sensor_params` | noise, latency, dropout |
| `environment_params` | light, texture, obstacle pose |
| `calibration_tf_hash` | 사용 extrinsic 식별 |
| `start_ros_time`, `end_ros_time` | episode 시간 |
| `success`, `collision_count` | 평가 label |
| `termination_reason` | success/collision/timeout/operator/error |
| `topic_counts`, `drop_counts` | 품질 정보 |
| `bag_sha256` | 원본 무결성 |

## 6. LeRobot 파생 feature

| feature | source | 변환 |
|---|---|---|
| `observation.images.front` | `/camera/image_raw` | RGB video |
| `observation.state` | `/odom`, `/imu`, `/joint_states` | 고정 길이 float vector |
| `observation.lidar` | `/scan` | 360개 range vector |
| `action` | `/cmd_vel` | applied `[linear_x, angular_z]` |
| `timestamp` | header 또는 `/clock` | episode 시작 기준 second |
| `frame_index`, `episode_index` | exporter | LeRobot index |
| `task_index` | manifest `task` | task metadata 연결 |
| `next.reward` | 평가 결과 | 명시한 reward 규칙 |
| `next.done`, `next.success` | 종료 판정 | 마지막 frame label |

가변 길이 3D `PointCloud2`는 첫 구현에서 LeRobot tensor로 넣지 않는다. MCAP에 보존하고 필요할 때 range image나 고정 개수 point feature로 변환한다.

[공식 LeRobot dataset 도구](https://huggingface.co/docs/lerobot/en/using_dataset_tools)의 episode split·merge, feature 편집, video 변환, local visualization을 재사용한다.

## 7. 동기화 규칙

| 규칙 | 결정 |
|---|---|
| 원본 clock | simulation은 `/clock`, 실제 robot은 message header의 monotonic ROS time |
| exporter 주기 | 첫 구현 10 Hz |
| camera/scan 선택 | 기준 timestamp보다 늦지 않은 가장 가까운 sample |
| 허용 지연 | `collector.yaml`에 sensor별 상한 기록 |
| 누락 sample | 0으로 채우지 않고 frame 제외와 drop count 증가 |
| timestamp 역행 | episode invalid 처리 |
| requested/applied action | 둘 다 있으면 별도 기록, 학습 action은 applied 사용 |

Python/C++ collector는 동일 fixture bag으로 timestamp, feature shape, 종료 판정 parity를 검사한다.

## 8. System identification

임의의 randomization 범위 대신 실제 robot의 저속 step, ramp, 회전 response로 simulation nominal parameter를 맞춘다.

제안식 — 향후 `code/python/.../domain_randomizer.py | fit_nominal_parameters()`:

$$
\theta^* = \arg\min_{\theta \in \Theta}
\sum_k \left\|y_k^{real} - y_k^{sim}(\theta)\right\|_W^2
$$

| 기호 | 의미·단위 |
|---|---|
| `theta` | mass `[kg]`, wheel radius/separation `[m]`, friction `[-]`, motor gain `[-]`, delay `[s]` |
| `Theta` | 실측값과 제조 공차로 제한한 feasible range |
| `k` | 동일 command sequence의 timestamp index |
| `y_real`, `y_sim` | 선속도 `[m/s]`, 각속도 `[rad/s]`, pose `[m, rad]` response |
| `W` | 단위가 다른 오차의 비중을 정하는 고정 weight matrix |
| `theta*` | weighted response error가 가장 작은 nominal parameter |

Decision variable은 `theta`, objective는 real/simulation response 오차 최소화다. 동일 command와 시간 기준을 사용하며 tuning episode와 held-out 평가 episode를 분리한다. 첫 구현은 battery voltage, motor temperature, jerk를 모델링하지 않는다. 목적함수가 작을수록 선택한 simulation dynamics가 측정 response와 가깝다.

## 9. Domain randomization

| 범주 | 대상 | 범위 근거 |
|---|---|---|
| Robot dynamics | mass, inertia, wheel radius/separation, friction, motor gain, acceleration limit, delay | `theta*`와 측정 공차 |
| Camera | brightness, exposure proxy, noise, latency, frame drop | 실제 camera 반복 측정 |
| LiDAR | range noise, min/max range, latency, ray/point drop | 정지 target 반복 측정 |
| IMU | bias, noise, latency | 정지·등속 구간 측정 |
| Environment | floor friction, light, texture, obstacle pose/velocity | task 의미를 유지하는 범위 |
| Extrinsic | 작은 translation/rotation perturbation | 별도 robustness split, label 필수 |

Extrinsic ground truth를 몰래 흔들지 않는다. `nominal_calibration`, `perturbed_calibration` split을 분리한다.

| profile | 목적 | seed 규칙 |
|---|---|---|
| `nominal` | randomization 없는 baseline | 고정 |
| `train` | 측정 범위 내 sampling | 학습용 목록 |
| `validation` | tuning·model 선택 | train과 겹치지 않음 |
| `stress` | 범위 경계·범위 밖 failure | train에 포함 금지 |
| `real` | 실제 robot 결과 | hardware run ID |

## 10. 제어 parameter tuning

첫 대상은 Simulation PATCH-06 회피 node의 distance threshold, linear speed, angular gain, timeout이다. 별도 optimizer framework 없이 grid/random search부터 쓴다.

| metric | 방향 | 단위 |
|---|---:|---|
| success rate | 최대화 | `%` |
| collision count | 최소화 | count/episode |
| completion time | 최소화 | `s` |
| path tracking error | 최소화 | `m` RMS |
| action smoothness | 최소화 | `m/s`, `rad/s` 변화량 |

train seed에서 후보를 만들고 validation seed에서 하나를 선택한다. test/stress/real 결과는 마지막에 한 번만 보고한다.

## 11. World와 scenario

| 단계 | world | 수집 목적 |
|---:|---|---|
| 1 | `empty_world.world` | 직진·회전 system identification |
| 2 | `turtlebot3_world.world` | nominal teleoperation과 회피 baseline |
| 3 | `turtlebot3_dqn_stage1.world` | 단순 goal reaching |
| 4 | `turtlebot3_dqn_stage2.world` | obstacle 증가 |
| 5 | `turtlebot3_dqn_stage3.world` | 좁은 통로·다중 장애물 |
| 6 | `turtlebot3_dqn_stage4.world` | generalization 평가 |
| 7 | Simulation PATCH-07 저조도 tunnel | sensor/domain robustness |

DQN world 4개와 model asset은 현재 fork에 존재한다. 수집 전에 headless smoke test와 model URI 검사를 수행한다. Jazzy에서 asset이 제거된 AutoRace world는 제외한다.

## 12. 구현 순서

| 단계 | 할 일 | 검증 |
|---:|---|---|
| 1 | manifest JSON schema와 작은 fixture 정의 | invalid field 실패, valid fixture 통과 |
| 2 | Python episode orchestrator 구현 | 같은 seed로 같은 적용값 |
| 3 | Python raw collector 구현 | MCAP topic count와 manifest 일치 |
| 4 | C++ raw collector 구현 | 같은 fixture의 Python/C++ parity 통과 |
| 5 | LeRobot exporter 구현 | dataset load와 local visualization |
| 6 | nominal DQN episode 수집 | replay와 종료 label 확인 |
| 7 | 실제 robot system identification | `theta*`, 범위, held-out 오차 저장 |
| 8 | train randomization 수집 | seed 중복·누락 없음 |
| 9 | controller parameter 선택 | train/validation 분리 |
| 10 | stress와 real 평가 | nominal 대비 metric 표 |

## 13. 완료 조건

- 같은 source commit, config, seed로 같은 적용값과 manifest 생성
- MCAP replay에서 필수 topic과 TF 복원
- timestamp 역행 없음, sensor별 sample/drop count 기록
- Python/C++ collector가 동일 raw contract와 fixture 결과 생성
- LeRobot load와 camera/state/action timeline 확인
- train/validation/test/stress episode와 seed가 겹치지 않음
- nominal, randomized, real 결과를 같은 metric으로 비교
- 실제 robot 결과가 없으면 “Sim2Real gap 감소”라고 결론내리지 않음

이번 PATCH는 재현 가능한 데이터 수집과 비교 규약까지다. 모델 학습은 dataset 품질 검증 후 Simulation PATCH-11에서 진행한다. 실제 Yahboom system identification은 Embedded PATCH-00의 실물 interface 확인 뒤 완료한다.
