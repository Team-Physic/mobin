# Embedded PATCH-01: Yahboom 안전 장애물 회피 적용

- 작성일: 2026-08-15
- 선행 조건: Simulation PATCH-06 회피 node 검증, Embedded PATCH-00 실물 interface·watchdog 확인
- 대상: 향후 `python/`, `cpp/`, `config/yahboom/`, 실물 MicroROS-Pi5
- 결론: **강화학습보다 먼저 결정론적 LiDAR 회피를 실물에 적용한다. 동일한 관측·명령·평가 기준을 simulation과 Yahboom에서 사용하고, safety supervisor가 모든 command를 제한한다.**

## 개념

| 구성 | 하는 일 |
|---|---|
| avoidance controller | `/scan`을 보고 전진 또는 회전 command 생성 |
| command adapter | 내부 command를 simulation `TwistStamped` 또는 Yahboom `Twist`로 변환 |
| safety supervisor | 속도 제한, scan timeout, command timeout, emergency stop 적용 |
| evaluator | 충돌, 최소 거리, 성공 시간, 정지 거리 기록 |

정책이나 controller가 직접 `/cmd_vel`에 쓰지 않는다. 출력은 반드시 safety supervisor를 지난다.

```text
/scan → avoidance controller → requested command
                                  ↓
operator stop ───────────→ safety supervisor → adapter → /cmd_vel
```

## Embedded

| 개념 | 쉬운 설명 | 확인 방법 |
|---|---|---|
| watchdog | 새 command나 sensor가 일정 시간 없으면 출력을 안전 상태로 바꾸는 감시기 | publisher를 종료하고 정지까지 걸린 시간 측정 |
| fail-safe | 고장이 났을 때 위험이 작은 상태로 전환하는 설계 | LiDAR disconnect·NaN·process crash fault 주입 |
| safety supervisor | controller 종류와 무관하게 마지막 속도·거리·timeout 제한을 강제 | 정상·회피·RL command에 같은 제한 적용 |
| stopping distance | 판단 지연과 제동 중 robot이 추가로 움직이는 실제 거리 | 여러 속도에서 반복 측정하고 최댓값 기록 |

학습은 [ROS 2 Jazzy topic statistics](https://docs.ros.org/en/ros2_documentation/jazzy/Tutorials/Advanced/Topic-Statistics-Tutorial/Topic-Statistics-Tutorial.html)로 message age·period를 확인하고, [ROS 2 logging](https://docs.ros.org/en/ros2_documentation/jazzy/Tutorials/Demos/Logging-and-logger-configuration.html)으로 timeout 원인과 상태 전이를 기록하는 순서로 진행한다.

신입은 bounded speed, scan·command timeout, emergency stop input과 fault injection test를 설계한다. 1~2년차는 measured stopping distance와 C++/Python 공통 test vector를 만든다. **safety 인증을 주장하거나 MCU hard limit를 ROS 2 node 하나로 대체하지 않는다.**

GitHub에는 `docs/embedded/01_safety_contract.md`와 `docs/embedded/01_fault_injection_report.md`를 남긴다. 각 fault마다 입력, 기대 safe state, 측정 정지시간, pass/fail, 재현 명령을 기록한다.

### SW 실습

| 실습 | 입력·방법 | 산출물 | 통과 조건 |
|---|---|---|---|
| scan sector 계산 | 정상·NaN·Inf·빈 `LaserScan` fixture | Python·C++ test 결과 | 좌·정면·우 거리와 invalid 판정 일치 |
| controller state machine | 접근·정지·회피·복귀 순서의 고정 fixture | state transition log | 같은 입력에서 상태와 command 부호가 일치 |
| safety supervisor | stale scan, stale command, emergency stop을 fault로 주입 | `/safety/events`와 zero command | 모든 fault가 지정된 safe state로 전이 |
| simulation 회귀 | static·crossing·no-scan scenario | collision·minimum range·latency metric | 정한 기준을 모두 통과하고 seed·config 저장 |
| 진단 | topic statistics와 ROS 2 log | message age·period·timeout 원인 | timeout이 단순 정지가 아니라 원인별로 식별됨 |

### HW 실습

| 실습 | 물리 조건 | 측정값·산출물 | 통과 조건 |
|---|---|---|---|
| emergency stop | 바퀴 공중, 최저 속도 | 입력부터 motor 정지까지 시간 | controller 상태와 무관하게 정지 |
| sensor fault | LiDAR 가림·분리, agent 중단 | fault 발생부터 zero command까지 시간 | 마지막 command가 지속되지 않음 |
| 정지 거리 | 넓은 실내에서 속도별 반복 주행 | 감지 거리, command latency, 실제 이동 거리 | `stop_distance`가 측정 최댓값보다 큼 |
| 정적 장애물 | 큰 종이 상자와 폭이 다른 통로 | 10회 collision·minimum range·oscillation | 접촉 없이 회피하거나 안전 정지 |
| 제한된 동적 시험 | 보조 관찰자, 즉시 중단 수단, 저속 | 횡단 물체 감지·정지 결과 | stage 1~4 통과 후에만 수행 |

**SW fault test는 state machine을 검증하고, HW fault test는 실제 motor가 멈추는지를 검증한다. 둘 중 하나로 다른 하나를 대체하지 않는다.**

## 1. Simulation PATCH-06 로직을 별도 package로 옮긴다

TurtleBot3 upstream package 안의 회피 알고리즘을 실물에서도 쓰도록 프로젝트 package로 분리한다.

```text
python/mobile_robot_lab_python/mobile_robot_lab_python/
├── avoidance_controller.py
└── safety_supervisor.py
cpp/mobile_robot_lab_cpp/
├── include/mobile_robot_lab_cpp/scan_sectors.hpp
├── src/avoidance_controller.cpp
└── src/safety_supervisor.cpp
config/avoidance/
├── simulation.yaml
└── yahboom.yaml
```

Python과 C++ 모두 작성하되 동작 계약은 하나다.

| 입력 상태 | 출력 |
|---|---|
| scan 없음·timeout·유효 거리 없음 | 정지 |
| 정면 거리가 `stop_distance` 이하 | 더 열린 쪽으로 제자리 회전 |
| 회피 중이고 `clear_distance` 미만 | 회전 유지 |
| 정면·좌우가 안전 | 제한 속도로 전진 |

## 2. safety supervisor를 controller와 분리한다

필수 parameter:

| parameter | 의미 | 초기 원칙 |
|---|---|---|
| `max_linear_velocity` | 허용 최대 선속도 | vendor 최대값이 아닌 저속 |
| `max_angular_velocity` | 허용 최대 각속도 | 전도·미끄럼 없는 저속 |
| `max_linear_acceleration` | command 변화율 제한 | Simulation PATCH-10 실측으로 조정 |
| `scan_timeout` | 최신 scan이 없으면 정지 | 실제 scan 주기의 여러 배가 되지 않게 설정 |
| `command_timeout` | 새 controller command가 없으면 정지 | controller 주기보다 길고 위험 지속 시간보다 짧게 설정 |
| `stop_distance` | 회피를 시작하는 거리 | 실제 정지 거리보다 크게 설정 |

다음 조건은 parameter가 아니라 고정 안전 규칙이다.

- NaN, 음수, 범위 밖 LiDAR 값만 남으면 정지
- operator emergency stop이 활성화되면 다른 command 무시
- node 시작 직후 첫 유효 scan 전에는 정지
- shutdown과 예외 처리에서 zero command 발행
- 통신이 완전히 끊겨도 MCU 또는 motor controller watchdog이 정지

## 3. simulation 회귀 검증

Simulation PATCH-05 world에서 같은 config와 seed를 사용한다.

| scenario | 통과 조건 |
|---|---|
| static | 충돌 없이 회피 후 전진 복귀 |
| crossing | 움직이는 장애물이 정면에 오면 감속·정지 또는 회전 |
| mixed | scan timeout을 인위적으로 만들면 정지 |
| no scan | 첫 command가 zero |

Python과 C++ 구현은 저장된 `LaserScan` fixture에서 다음 결과가 같아야 한다.

- 정면·좌·우 최소 거리
- 회피 상태 전환
- 선속도·각속도 부호
- timeout에서 zero command

부동소수점 값은 작은 허용 오차를 두고 비교한다.

## 4. 실물 시험장을 단계적으로 넓힌다

| 단계 | 환경 | 속도 | 다음 단계 조건 |
|---:|---|---|---|
| 1 | 바퀴를 띄움 | 최소 | scan 가림에 회전 command, timeout에 zero |
| 2 | 넓은 실내, 로봇 주변 2 m 비움 | 최소 | 직진·정지·제자리 회전 방향 정상 |
| 3 | 큰 종이 상자 1개 | 저속 | 10회 모두 접촉 없이 정지 또는 회피 |
| 4 | 통로 폭을 바꾼 정적 장애물 | 저속 | 끼임·진동 없이 탈출 또는 안전 정지 |
| 5 | 사람이 멀리서 천천히 횡단 | 저속 | 안전 거리 밖 정지, 작업자 즉시 중단 가능 |

사람을 첫 동적 장애물로 쓰지 않는다. stage 1~4가 통과한 뒤 보조 관찰자와 emergency stop을 둔 상태에서만 stage 5를 수행한다.

## 5. 같은 metric으로 비교한다

| metric | 계산 | 목적 |
|---|---|---|
| success rate | 목표 구간 통과 episode / 전체 episode | task 성공 |
| collision count | bumper 또는 관찰 label 합 | 안전 |
| minimum range | episode의 정면 최소 거리 | 안전 여유 |
| stop latency | 위험 감지부터 zero command까지 시간 | 제어 지연 |
| command age | 현재 시각 - 마지막 command stamp | 통신 건전성 |
| oscillation count | 짧은 시간의 회전 방향 반복 횟수 | 회피 안정성 |

simulation과 real에서 metric 이름·단위를 같게 유지한다. 센서가 다른 metric은 `unavailable`로 기록하고 0으로 만들지 않는다.

## 6. Yahboom vendor 회피 예제의 사용 범위

공식 과정의 `laser_Avoidance`는 `/scan`을 받아 `/cmd_vel`을 내는 최소 예제로 참고한다. 그러나 고정 threshold, sleep 기반 timing, timeout 처리 여부를 실제 source에서 확인한 뒤 사용한다. **vendor 예제가 실행된다는 사실만으로 이 프로젝트의 안전 조건을 통과한 것은 아니다.**

## 7. 완료 조건

- controller 출력이 safety supervisor 없이 `/cmd_vel`에 연결되지 않음
- Python/C++ fixture parity 통과
- simulation static·crossing·timeout 회귀 시험 통과
- 실물 바퀴 공중 시험과 정적 상자 10회 시험 결과 저장
- scan·command timeout에서 zero command 확인
- parameter, source commit, hardware inventory, metric을 Simulation PATCH-10 manifest에 기록
- 충돌 0회라도 정지 거리·지연 수치를 함께 보고

**이 결과가 Simulation PATCH-11 강화학습 policy의 최소 성능·안전 기준이다. RL이 이 baseline보다 나쁘면 실물 배포하지 않는다.**
