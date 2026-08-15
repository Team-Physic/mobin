# PATCH-18: 자체 Humanoid Isaac Lab 학습과 Sim2Real

- 작성일: 2026-08-15
- 선행 조건: PATCH-16 URDF, PATCH-17 hardware·safety interface
- 대상: 향후 `humanoid/isaac_lab/`, `humanoid/deployment/`
- 결론: **URDF를 USD로 변환한 뒤 physics·actuator·contact를 먼저 검증한다. stand→weight shift→저속 보행 curriculum으로 PPO를 학습하고, system identification 기반 randomization·shadow mode·safety tether를 거쳐 실물에 적용한다.**

## 검증한 기준 자료

| 자료 | 사용 |
|---|---|
| [Isaac Lab](https://github.com/isaac-sim/IsaacLab) | BSD-3-Clause 기반 robot learning framework |
| [Isaac Lab 문서](https://docs.isaacsim.omniverse.nvidia.com/latest/isaac_lab_tutorials/index.html) | environment·task·training 구성 |
| [Isaac Sim URDF Import](https://docs.isaacsim.omniverse.nvidia.com/latest/importer_exporter/import_urdf.html) | URDF→USD, collision·self-collision·robot type option |
| [Isaac Sim Joint Gain Tuning](https://docs.isaacsim.omniverse.nvidia.com/latest/robot_setup_tutorials/joint_tuning.html) | import 후 zero gain과 stiffness·damping 검증 |
| [RSL-RL](https://github.com/leggedrobotics/rsl_rl) | Isaac Lab에서 쓰는 robotics PPO 학습기 |
| [Isaac Lab policy exporter](https://isaac-sim.github.io/IsaacLab/main/_modules/isaaclab_rl/rsl_rl/exporter.html) | TorchScript·ONNX export 경로 |
| [Berkeley Humanoid Lite](https://github.com/HybridRobotics/Berkeley-Humanoid-Lite) | custom humanoid의 Isaac Lab task→실물 구조 참고 |

Isaac Lab과 Isaac Sim version은 서로 맞는 release 조합으로 고정한다. `latest` 문서의 명령을 설치된 version에 그대로 적용하지 않고 해당 release 문서를 확인한다.

## 1. 계획 디렉터리

```text
humanoid/
├── isaac_lab/
│   └── source/mobin_humanoid/
│       ├── assets/mobin_humanoid_cfg.py
│       └── tasks/locomotion/
│           ├── stand_env_cfg.py
│           ├── walk_env_cfg.py
│           └── agents/rsl_rl_ppo_cfg.py
├── assets/usd/
├── config/randomization/humanoid.yaml
└── deployment/
    ├── policy_node.py
    ├── observation_contract.yaml
    └── safety_limits.yaml
```

Isaac Lab 자체를 fork 안에 복사하지 않는다. 고정 release dependency로 설치하고 프로젝트 extension만 작성한다.

## 2. URDF를 USD로 가져온다

PATCH-16에서 생성한 순수 URDF를 source로 사용한다.

| import 항목 | 결정 |
|---|---|
| robot type | Humanoid |
| base | locomotion용 floating base |
| visual | merge 가능 여부를 성능 측정 후 선택 |
| collision | PATCH-16 단순 collision 사용 |
| self-collision | 켠 상태와 끈 상태를 비교하고 의도한 pair만 조정 |
| output | version이 붙은 USD와 import manifest |

import manifest에 URDF hash, importer version, option, output USD hash를 기록한다. USD GUI에서 직접 고친 값은 physics layer 또는 재현 가능한 script로 옮긴다.

## 3. 학습 전에 asset을 검증한다

| 시험 | 통과 조건 |
|---|---|
| zero-gravity joint motion | joint axis·limit·direction이 URDF와 일치 |
| gravity drop | collision이 터지거나 바닥을 관통하지 않음 |
| fixed-base pose | command 없이 NaN·explosion 없음 |
| self-collision | 정상 joint range를 잘못 막지 않음 |
| mass sum | CAD와 USD 전체 질량 일치 |
| center of mass | CAD와 허용 오차 안에서 일치 |
| contact | 양 발이 평평하게 바닥 접촉 |

URDF importer는 joint stiffness·damping을 0으로 둘 수 있다. 공식 Gain Tuner 절차로 actuator gain과 force·velocity limit를 설정하고 실제 single-joint rig response와 비교한다.

## 4. Actuator model을 실측한다

PATCH-15·17 rig에서 동일 command sequence를 simulation과 real actuator에 넣는다.

| 식별값 | 측정 |
|---|---|
| position loop gain·damping | step response |
| command latency | command timestamp→motion 시작 |
| torque/current limit | 하중별 current와 정지 조건 |
| backlash·deadband | 방향 전환 오차 |
| thermal derating | 시간·온도별 허용 출력 |

nominal model은 held-out trajectory에서도 position·velocity response 오차가 줄어야 한다. 학습 reward가 좋아지는 값이 아니라 실물 response에 가까운 값을 선택한다.

## 5. Task를 작은 curriculum으로 만든다

| 단계 | task | 성공 조건 |
|---:|---|---|
| 1 | fixed-base joint tracking | target tracking, limit 위반 없음 |
| 2 | stand | 정해진 시간 torso 높이·자세 유지 |
| 3 | weight shift | 좌우 center of pressure 이동·낙상 없음 |
| 4 | zero-velocity balance | 외란 뒤 자세 회복 |
| 5 | 저속 직진 | command velocity 추종·낙상 없음 |
| 6 | 회전·속도 변경 | yaw·velocity 추종 |
| 7 | 작은 바닥 변화 | held-out terrain에서 안정 |

앞 task의 checkpoint를 다음 task 초기값으로 사용할 수 있지만, 각 단계의 독립 evaluation 결과를 보존한다.

## 6. Observation·action·reward 계약

| 항목 | 첫 구현 |
|---|---|
| observation | base angular velocity, projected gravity, joint position·velocity, previous action, velocity command |
| action | nominal pose 주변 joint position target offset |
| policy rate | 요구사항에서 정한 값 |
| low-level rate | PATCH-17 actuator loop rate |
| privileged observation | simulation critic에만 사용하면 명시 |

reward는 최소한 다음 범주를 분리 기록한다.

| reward | 목적 |
|---|---|
| velocity tracking | 명령 속도 추종 |
| orientation·height | torso 자세 유지 |
| foot contact·slip | 안정된 접촉 |
| joint limit·torque | hardware 한계 보호 |
| action rate | 급격한 target 변화 억제 |
| termination | fall·위험 자세 종료 |

reward 합계만 저장하지 않고 항별 평균을 남긴다. safety limit 위반을 다른 reward가 상쇄하지 못하도록 termination 또는 hard constraint로 둔다.

## 7. PPO 학습과 평가

Isaac Lab의 RSL-RL workflow를 사용한다.

```bash
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py \
  --task Mobin-Humanoid-Walk-v0 \
  --headless

./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/play.py \
  --task Mobin-Humanoid-Walk-Play-v0 \
  --checkpoint <CHECKPOINT>
```

실제 task entry point와 CLI는 고정한 Isaac Lab release에 맞춰 수정한다.

평가 split:

| split | 조건 |
|---|---|
| nominal | 식별한 중심값 |
| randomized | train 범위의 새 seed |
| held-out | train에 없는 mass·friction·latency 조합 |
| stress | hardware 한계 근처, 학습 제외 |

## 8. Humanoid Domain Randomization

| 범주 | 실측 근거 |
|---|---|
| link mass·center of mass | CAD와 조립품 측정 차이 |
| joint zero offset | calibration 반복 분포 |
| actuator gain·delay·backlash | single-joint rig |
| ground friction·restitution | 바닥 재질 시험 |
| IMU bias·noise·latency | 정지·회전 bag |
| control packet drop | 실제 bus diagnostic |
| external push | 명시한 안전 범위의 robustness 평가 |

randomization 범위를 넓히기 전에 nominal model 오류를 먼저 줄인다. 너무 넓은 범위는 불필요하게 보수적이거나 떨리는 policy를 만들 수 있다.

## 9. 학습 장치와 추론 장치를 분리한다

Isaac Lab PPO 학습은 NVIDIA GPU가 있는 workstation에서 수행한다. Raspberry Pi 5에는 학습 전체를 옮기지 않고 완성된 policy의 inference만 배포한다.

| 파일·runtime | 실행 장치 |
|---|---|
| 학습 checkpoint | workstation GPU에서 평가·재학습 |
| TorchScript | PyTorch runtime이 지원하는 CPU·GPU |
| ONNX | ONNX Runtime provider가 지원하는 CPU·NPU·GPU |
| TensorRT engine | NVIDIA GPU가 있는 Jetson·workstation |

**가중치 파일 자체가 CPU 전용인 것은 아니다.** 같은 policy도 export 형식, operator 지원, runtime backend에 따라 CPU·NPU·GPU에서 실행된다.

[Isaac Lab RSL-RL exporter](https://isaac-sim.github.io/IsaacLab/main/_modules/isaaclab_rl/rsl_rl/exporter.html)는 TorchScript와 ONNX export를 제공한다. observation normalization이나 recurrent state가 있으면 단순 network weight만 떼지 않고 [Isaac Lab LEAPP export](https://isaac-sim.github.io/IsaacLab/release/3.0.0-beta2/source/policy_deployment/05_leapp/exporting_policies_with_leapp.html)처럼 preprocessing·action semantics까지 묶어 배포한다.

## 10. Raspberry Pi 5에서 실제 deadline을 측정한다

첫 locomotion policy는 joint·IMU 같은 proprioceptive observation을 입력으로 받는 작은 MLP로 제한한다. 이 형태는 Pi 5 CPU에서 먼저 검증할 가치가 있지만, **충분히 빠르다고 문서만 보고 확정하지 않는다.**

동일한 observation fixture로 10,000회 inference를 실행해 다음 값을 저장한다.

| 측정값 | 이유 |
|---|---|
| inference p50·p95·max | 평균이 가리는 지연 spike 확인 |
| observation→command p50·p95·max | ROS 2 변환·후처리까지 포함한 실제 지연 |
| control deadline miss | 목표 policy rate를 놓친 횟수 |
| CPU·memory | 다른 ROS 2 node와 함께 실행 가능한지 확인 |
| 온도·throttling | 장시간 실행에서 성능 저하 확인 |
| action parity | workstation runtime과 출력 오차 확인 |

100 Hz policy의 주기는 10 ms다. 이 프로젝트의 첫 승격 기준은 다음과 같이 둔다.

| Pi 5 측정 결과 | 결정 |
|---|---|
| inference p95 < 3 ms, end-to-end p95 < 8 ms, deadline miss 0 | Pi 5 CPU 유지 |
| inference p95 3–7 ms | thread 수·ONNX Runtime·model 크기·quantization부터 조정 |
| inference p95 > 7 ms 또는 thermal throttling | accelerator나 compute board 변경 |

이 수치는 Pi 5의 보장 성능이 아니라 **현재 프로젝트가 100 Hz loop에 두는 engineering gate**다. policy rate가 달라지면 budget도 다시 정한다.

## 11. Accelerator를 필요할 때만 선택한다

| workload | 첫 선택 | 변경 조건 |
|---|---|---|
| joint·IMU MLP policy | Pi 5 CPU | 위 deadline 실패 |
| camera detector·encoder | Pi 5 + AI HAT+ | 사용 operator와 model 변환이 지원되고 end-to-end deadline 통과 |
| end-to-end camera policy·큰 transformer | Jetson Orin Nano 계열 | Pi 경로에서 operator·memory·deadline 실패 |
| motor current·position loop | MCU/servo controller | accelerator와 무관하게 유지 |

[Raspberry Pi AI HAT 문서](https://www.raspberrypi.com/documentation/accessories/ai-hat-plus.html)는 AI HAT+의 13/26 TOPS와 AI HAT+ 2의 40 TOPS 구성을 설명한다. TOPS만으로 policy 실행 가능 여부를 판단하지 않는다. Hailo compiler가 model operator를 지원하고 ROS 2 전처리·복사까지 포함한 latency가 기준을 통과해야 한다.

[Jetson Orin Nano Super 문서](https://docs.nvidia.com/jetson/orin-nano-devkit/user-guide/index.html)는 최대 67 INT8 TOPS, 7–25 W 범위와 CUDA·TensorRT 환경을 제공한다. ONNX model은 [TensorRT ONNX 배포 절차](https://docs.nvidia.com/deeplearning/tensorrt/latest/getting-started/quick-start-onnx-deployment.html)로 engine을 만들 수 있다. 대신 전력·냉각·질량·CAD envelope가 커지므로 vision workload가 실제로 요구할 때만 선택한다.

```text
# humanoid deployment | planned compute boundary
workstation NVIDIA GPU: Isaac Lab training
             ↓ ONNX 또는 TorchScript
Raspberry Pi 5: ROS 2, state estimation, policy 50–100 Hz
             ↓ bounded joint target
MCU/servo controller: actuator I/O 500–1000 Hz, watchdog, hard limits
```

가장 단순한 기본안은 **Pi 5 CPU + MCU**다. accelerator 후보를 모두 장착하는 구조는 만들지 않는다. benchmark 실패가 확인된 workload만 이동한다.

## 12. Policy export와 실물 승격

Isaac Lab exporter로 TorchScript 또는 ONNX 중 Raspberry Pi 5에서 deadline을 만족하는 하나를 선택한다. 둘 다 유지하지 않는다.

승격 순서:

1. 고정 observation fixture에서 학습 runtime과 export runtime action parity
2. 실물 rosbag replay
3. 실시간 shadow mode
4. actuator off 상태에서 command·limit 확인
5. 전신 지지대에 고정
6. safety tether와 보조자, 낮은 자세에서 stand
7. 평지 저속 보행

policy output은 PATCH-17 joint limit·rate limit·fault handler를 우회할 수 없다. emergency stop은 policy process와 독립적으로 작동해야 한다.

## 13. 최종 Sim2Real 평가

| metric | simulation·real 공통 정의 |
|---|---|
| fall rate | episode당 fall 여부 |
| survival time | 종료 전 시간 |
| velocity tracking RMSE | command와 base velocity 차이 |
| orientation error | roll·pitch RMS |
| foot slip | contact 중 foot 이동 거리 |
| energy proxy | joint current 또는 torque×velocity 적분 |
| policy latency | observation부터 joint target까지 p50·p95·max |
| safety intervention | limit·watchdog·operator stop 횟수와 원인 |

실물 결과가 simulation보다 나쁘면 sensor, actuator, timing, contact gap을 분리해 한 항목씩 갱신하고 이전 held-out test로 regression을 확인한다.

## 14. 완료 조건

- URDF hash·import option·USD hash가 연결된 재현 가능한 asset
- joint axis·limit·mass·center of mass·collision·contact 검사 통과
- single-joint 실측에 맞춘 actuator nominal model과 held-out 오차 저장
- stand→weight shift→walk curriculum별 checkpoint와 metric 보존
- train·held-out·stress seed 분리, randomization 근거 기록
- export runtime parity와 Raspberry Pi 5 control deadline 통과
- 10,000회 benchmark의 p50·p95·max·deadline miss·온도 기록
- Pi 5 CPU, AI HAT+, Jetson 중 실제 측정으로 하나의 배포 경로 선택
- 선택한 compute board의 전력·냉각·질량을 PATCH-16 CAD와 PATCH-17 power budget에 반영
- replay→shadow→tether→저속 실물 승격 순서 준수
- simulation과 real metric을 같은 정의로 보고
- emergency stop·watchdog·joint limit가 policy와 독립적으로 작동

**PATCH-18이 최종 목표의 첫 완성점이다. 자체 CAD Humanoid가 동일한 robot description·action 계약으로 Isaac Lab에서 학습되고, 제한된 실물 task에서 정량 검증된다.**
