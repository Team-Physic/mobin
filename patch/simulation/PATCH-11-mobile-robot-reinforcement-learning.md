# Simulation PATCH-11: Mobile Robot 강화학습 학습과 Simulation 검증

- 작성일: 2026-08-15
- 선행 조건: Simulation PATCH-05·06, Simulation PATCH-10, Embedded PATCH-01
- 대상: 향후 `forks/turtlebot3_machine_learning/`, `code/python/`, `config/rl/mobile/`
- 결론: **첫 강화학습은 2D LiDAR 기반 discrete-action DQN으로 제한한다. 공식 TurtleBot3 Jazzy 구현을 baseline으로 재현한 뒤, 같은 관측·action 계약으로 warehouse와 domain randomization 평가를 추가한다.**

## 검증한 기준 자료

| 자료 | 확인한 내용 | 결정 |
|---|---|---|
| [TurtleBot3 Machine Learning e-Manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/machine_learning/) | Laser Distance Sensor 기반 DQN, environment·agent·test node 실행, Jazzy 설치 절차 | 첫 mobile RL baseline |
| [ROBOTIS turtlebot3_machine_learning](https://github.com/ROBOTIS-GIT/turtlebot3_machine_learning) | `humble`, `jazzy`, `main` active branch, Apache-2.0 | 구현 시 내 fork의 `jazzy` branch 사용 |
| Simulation PATCH-10 | episode MCAP, manifest, system identification, domain randomization | 학습·평가 데이터 규약 |

처음부터 Camera, 3D LiDAR, learned world model, continuous-action PPO를 한 policy에 넣지 않는다. 2D LiDAR DQN이 실패하면 센서 융합 문제인지 RL 문제인지 구분하기 어렵기 때문이다.

## Embedded

학습은 workstation에서 수행하고 Pi 5에는 inference만 배포한다. model 파일이 CPU 전용인 것이 아니라 runtime backend, 지원 operator, memory와 latency가 실행 장치를 결정한다.

| 개념 | 실습에서 확인할 값 |
|---|---|
| inference latency | observation 수신부터 action 생성까지 p50·p95·max |
| end-to-end latency | sensor timestamp부터 bounded `cmd_vel` 발행까지 |
| quantization | 변환 전후 action parity와 latency·memory 차이 |
| deadline miss | 목표 policy period 안에 끝나지 못한 횟수 |

학습 자료는 [TurtleBot3 공식 DQN 과정](https://emanual.robotis.com/docs/en/platform/turtlebot3/machine_learning/)으로 baseline을 이해한 뒤 [ONNX Runtime performance](https://onnxruntime.ai/docs/performance/)로 Pi profiling을 진행한다.

신입은 observation/action contract와 deterministic replay test를 만든다. 1~2년차는 Pi 5에서 10,000회 benchmark, temperature, memory를 측정하고 model 축소·thread·quantization 중 하나를 근거로 선택할 수 있다. accelerator 구매나 model 재설계는 benchmark 실패 전에는 범위에 넣지 않는다.

GitHub에는 `docs/embedded/sim11_model_contract.md`와 `docs/embedded/sim11_inference_budget.md`를 남긴다. Simulation metric과 Pi 실측 metric을 같은 표에 쓰되 둘을 같은 결과로 합치지 않는다.

## 1. 공식 baseline을 fork한다

GitHub에서 `ROBOTIS-GIT/turtlebot3_machine_learning`을 내 계정으로 fork한 뒤:

```bash
cd /home/swlinux/Desktop/workspace/mobin
git clone --branch jazzy \
  https://github.com/<MY_GITHUB_ID>/turtlebot3_machine_learning.git \
  forks/turtlebot3_machine_learning

git -C forks/turtlebot3_machine_learning remote add upstream \
  https://github.com/ROBOTIS-GIT/turtlebot3_machine_learning.git

git -C forks/turtlebot3_machine_learning rev-parse HEAD
git -C forks/turtlebot3_machine_learning status --short --branch
```

`LICENSE`를 보존하고 기준 commit을 `patch/README.md`와 image label에 추가한다. fork 생성과 clone은 이 PATCH를 실제 구현할 때 수행한다.

## 2. 관측과 action을 고정한다

| 항목 | 첫 구현 |
|---|---|
| observation | 정규화한 2D `/scan`, goal 거리·방향, 현재 선속도·각속도 |
| action | 정지, 직진, 좌회전, 우회전 같은 유한 집합 |
| policy 출력 | action index와 confidence/logit |
| 실제 command | action table이 만든 `[linear_x, angular_z]` |
| episode 종료 | goal, collision, timeout, sensor timeout |

학습 node가 ROS message type을 직접 결정하지 않는다. action table 결과를 Embedded PATCH-00 내부 command로 보내 simulation·Yahboom adapter가 각각 변환한다.

## 3. reward를 명시한다

최소 reward:

$$
r_t = w_p(d_{t-1}-d_t) - w_c I_{collision} - w_o I_{near} - w_s |\Delta u_t| + w_g I_{goal}
$$

| 항 | 의미 |
|---|---|
| `d_(t-1)-d_t` | goal에 가까워진 거리 |
| `I_collision` | 충돌 시 1 |
| `I_near` | 안전 거리 안에 장애물이 있으면 1 |
| `|Delta u_t|` | 급격한 action 변경 penalty |
| `I_goal` | goal 도착 시 1 |

reward 항, weight, 종료 조건을 config와 manifest에 저장한다. reward만 높고 충돌이 늘어나는 policy는 선택하지 않는다.

## 4. baseline부터 재현한다

공식 e-Manual의 environment·agent·test 흐름을 현재 Jazzy fork에서 먼저 실행한다.

```bash
ros2 run turtlebot3_dqn dqn_environment
ros2 run turtlebot3_dqn dqn_agent --ros-args \
  -p max_training_episodes:=1000 \
  -p model_file:=mobin_baseline.h5 \
  -p verbose:=true
```

실제 option 이름은 fork한 Jazzy source의 parameter 선언과 `--help`로 다시 확인한다. 문서와 source가 다르면 source를 따른다.

baseline 재현 결과에 다음을 남긴다.

- source commit과 dependency version
- random seed
- world·stage
- episode reward, success, collision, length
- checkpoint hash
- evaluation video 또는 MCAP

## 5. 학습과 평가 world를 분리한다

| split | world | 사용 |
|---|---|---|
| train | DQN stage 1~3 | policy parameter update |
| validation | train과 다른 seed·장애물 pose | checkpoint 선택 |
| test | DQN stage 4 | 마지막 일반화 평가 |
| warehouse | Simulation PATCH-05 AWS warehouse | 좁은 통로·선반 환경 전이 평가 |
| stress | Simulation PATCH-10 범위 경계 | noise·latency·friction 실패 지점 확인 |

test·warehouse 결과를 보고 reward나 hyperparameter를 바꾸면 그 결과는 validation으로 재분류하고 새 test seed를 만든다.

## 6. Domain Randomization은 실측 뒤 적용한다

Simulation PATCH-10에서 구한 nominal 값 주변만 바꾼다.

| randomization | 이유 |
|---|---|
| wheel radius·separation | 조립 오차와 회전 response 차이 |
| motor gain·command delay | ESP32 PID와 통신 차이 |
| floor friction | 바닥 재질 차이 |
| LiDAR noise·dropout·latency | MS200과 simulation scan 차이 |
| obstacle pose | 경로 암기 방지 |

nominal policy와 randomized policy를 같은 held-out seed에서 비교한다. randomization을 넣었다는 사실만으로 Sim2Real 개선을 주장하지 않는다.

## 7. Python과 C++ 경계

| 기능 | 언어 | 이유 |
|---|---|---|
| DQN 학습·evaluation orchestration | Python | 공식 구현과 ML 생태계 재사용 |
| scan 전처리 reference | Python | 빠른 검증 |
| scan 전처리·safety supervisor | C++ | 실물 제어 지연과 deterministic behavior |
| neural network 재구현 | 하지 않음 | 학습 framework와 결과 불일치 방지 |

C++로 DQN을 다시 작성하지 않는다. 배포 형식이 정해지면 공식 runtime API로 같은 model을 호출한다.

## 8. 선택 기준

policy 후보는 validation에서 다음 순서로 고른다.

1. collision·sensor-timeout 안전 조건 통과
2. Embedded PATCH-01 결정론적 baseline 이상의 success rate
3. completion time
4. action smoothness

평균만 보고하지 않는다. seed별 값, 중앙값, 하위 성능 구간을 함께 저장한다.

## 9. 완료 조건

- 공식 Jazzy DQN baseline을 수정 전 재현
- observation shape·normalization·action table 고정 및 version 기록
- train/validation/test seed와 world 분리
- nominal·randomized policy를 같은 test 조건에서 비교
- Embedded PATCH-01보다 충돌이 많으면 실물 배포 대상에서 제외
- checkpoint·config·commit·metric·MCAP hash를 manifest에 저장
- Camera와 3D LiDAR 없이 2D LiDAR policy의 원인 분석 가능

**통과한 checkpoint만 Embedded PATCH-02의 shadow mode 실물 평가 대상으로 넘긴다.**
