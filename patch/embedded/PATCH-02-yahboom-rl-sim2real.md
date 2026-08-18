# Embedded PATCH-02: Yahboom 강화학습 Policy Sim2Real 검증

- 작성일: 2026-08-15
- 선행 조건: Simulation PATCH-10·11과 Embedded PATCH-01 완료
- 대상: 향후 `python/`, `cpp/`, `config/deployment/yahboom/`, Raspberry Pi 5
- 결론: **학습 policy를 바로 motor에 연결하지 않는다. 기록 재생, 실물 shadow mode, 바퀴 공중 시험, 저속 지상 시험 순서로 승격하고 모든 출력은 Embedded PATCH-01 safety supervisor를 통과시킨다.**

## 개념

| mode | policy가 하는 일 | motor command |
|---|---|---|
| replay | 저장된 실물 sensor를 읽고 action 계산 | 발행하지 않음 |
| shadow | 실시간 sensor로 action 계산·기록 | 기존 안전 controller가 발행 |
| guarded | policy action을 safety supervisor가 제한 | 제한된 command만 발행 |

shadow mode에서 policy와 Embedded PATCH-01 controller의 판단을 같은 timestamp로 비교할 수 있다. 실물 주행 위험 없이 disagreement와 inference 지연을 찾는 단계다.

## Embedded

| 개념 | 쉬운 설명 | 이 단계에서의 판단 |
|---|---|---|
| deployment artifact | 재현 가능한 model, runtime, config, source commit 묶음 | 파일 이름이 아니라 hash와 contract로 식별 |
| shadow mode | 실시간 입력으로 policy를 실행하되 motor에는 보내지 않는 방식 | 기존 controller와 action·latency 비교 |
| action parity | workstation과 Pi runtime이 같은 입력에서 허용 오차 안의 action을 내는지 확인 | 변환 model의 정확성 판정 |
| rollback | 새 artifact 실패 시 검증된 이전 version으로 복귀 | 실물 연결 전에 수동 절차부터 검증 |

[ONNX Runtime performance 문서](https://onnxruntime.ai/docs/performance/)는 profiling·threading·quantization 선택의 기준이고, [GitHub Releases 문서](https://docs.github.com/en/repositories/releasing-projects-on-github/about-releases)는 tag에 source와 배포 artifact를 연결하는 기준이다.

신입은 replay와 hash·contract 검증을 구현한다. 1~2년차는 Pi 5에서 end-to-end latency, memory, temperature를 측정하고 shadow disagreement를 분석한다. **원격 OTA와 자동 rollback은 전원 차단 복구·서명·dual-bank 검증 전에는 구현 완료로 표시하지 않는다.**

GitHub에는 `docs/embedded/02_deployment_manifest.md`, `docs/embedded/02_shadow_report.md`, `docs/embedded/02_rollback_runbook.md`를 남긴다. model binary와 대용량 MCAP은 Git history에 넣지 않고 Release asset 또는 CI artifact의 hash와 위치만 기록한다.

### SW 실습

| 실습 | 입력·방법 | 산출물 | 통과 조건 |
|---|---|---|---|
| artifact 고정 | model, normalization, action table, runtime | SHA-256이 포함된 deployment manifest | 같은 manifest로 동일 runtime을 복원 |
| MCAP replay | 기록한 실물 sensor episode | action, invalid input, inference latency log | NaN·stale input에서 zero, action 범위 정상 |
| runtime parity | 동일 fixture를 workstation과 Pi runtime에 입력 | action 차이 표 | 정한 허용 오차 안에서 action 일치 |
| shadow 분석 | policy와 baseline을 같은 sensor timestamp로 실행 | disagreement·latency report | motor publisher 없이 위험 disagreement 분류 |
| rollback 연습 | 새 artifact를 의도적으로 실패 처리 | 수동 rollback log | 검증된 이전 artifact와 config로 복귀 |

### HW 실습

| 실습 | 물리 조건 | 측정값·산출물 | 통과 조건 |
|---|---|---|---|
| Pi 5 부하 시험 | 실제 Pi 5, 냉각장치, 최종 runtime | p50·p95·max latency, memory, 온도, throttling | control period를 넘는 반복 deadline miss 없음 |
| 공중 guarded 시험 | 바퀴 공중, 최저 속도, emergency stop | action 방향, clamp, timeout 반응 | 모든 command가 safety supervisor를 통과 |
| 지상 단계 시험 | 넓은 실내→상자→새 배치 | collision, minimum range, success rate | 단계별 중단 조건 없이 통과 |
| 전원·통신 fault | agent 중단, sensor 분리, 허용된 전원 재기동 | 정지시간과 recovery log | motor가 안전 상태로 전이하고 수동 복구 가능 |
| Sim2Real 비교 | 같은 장애물 배치와 metric | baseline·RL·simulation 비교표 | RL collision 0, success rate가 baseline 이상 |

**모델 파일이 Pi 5에서 열리는 것은 HW 검증이 아니다. 실제 온도·지연·전원·motor 반응을 측정해야 guarded 단계 통과다.**

## 1. 배포 artifact를 고정한다

| artifact | 반드시 기록할 값 |
|---|---|
| model | 파일 SHA-256, 학습 run ID |
| observation | feature 순서, 단위, normalization mean·scale |
| action table | index별 `linear_x`, `angular_z` |
| runtime | Python·TensorFlow/Keras version |
| source | project와 RL fork commit |
| safety config | 속도·가속도·거리·timeout 제한 |

첫 배포는 학습에 사용한 runtime을 그대로 쓴다. Raspberry Pi 5에서 제어 deadline을 넘을 때만 TFLite·ONNX 같은 변환을 검토한다. 변환본은 원본과 고정 fixture에서 action parity를 통과해야 한다.

## 2. 실물 MCAP replay로 검사한다

Simulation PATCH-10의 real episode를 재생해 policy output만 별도 topic에 기록한다.

```text
real MCAP → observation builder → policy → /policy/cmd_vel_requested
                                      └── latency·action log
```

검사 항목:

| 검사 | 통과 조건 |
|---|---|
| feature shape | 학습 때와 완전히 같음 |
| NaN·Inf | policy 입력에 들어가지 않음 |
| timestamp | 역행 없음, 너무 오래된 sensor에서 zero |
| action range | action table 밖 값 없음 |
| inference time | 설정한 control period 안에 안정적으로 완료 |

## 3. shadow mode를 실행한다

실제 `/scan`, odometry를 policy가 구독하되 `/cmd_vel` publisher는 생성하지 않는다. 다음을 MCAP에 함께 기록한다.

| topic | 내용 |
|---|---|
| `/policy/cmd_vel_requested` | policy가 원한 command |
| `/baseline/cmd_vel_requested` | Embedded PATCH-01 controller command |
| `/cmd_vel_applied` | 실제 motor로 간 command |
| `/safety/events` | clamp, timeout, emergency stop 이유 |

위험 구간에서 policy가 전진하고 baseline이 정지하는 episode를 우선 검토한다. disagreement가 많으면 실물 action을 허용하지 않고 observation·domain gap을 수정한다.

## 4. guarded mode 승격 순서

| 단계 | 조건 | 중단 조건 |
|---:|---|---|
| 1 | 바퀴 공중, 최소 속도 | 방향 오류, timeout 후 회전 지속 |
| 2 | 넓은 실내, 장애물 없음 | odometry 불일치, command deadline miss |
| 3 | 큰 정적 상자 1개 | safety distance 침범 |
| 4 | 학습에 없던 정적 배치 | 충돌 또는 반복 stuck |
| 5 | 느린 동적 장애물 | operator stop 지연, 불안정 action |

각 단계는 별도 run ID를 갖는다. 실패한 단계에서 parameter를 바꾸면 같은 단계부터 새 run으로 다시 시작한다.

## 5. Sim2Real gap을 분리 측정한다

| gap | 비교 방법 | 조정 위치 |
|---|---|---|
| LiDAR | 같은 벽 거리에서 range 분포·dropout·latency | Simulation PATCH-10 sensor randomization |
| command response | step command의 속도 rise time·overshoot | motor delay·gain·friction |
| odometry | 직진·제자리 회전의 누적 오차 | wheel radius·separation |
| compute | observation부터 applied command까지 지연 | runtime·control rate |
| geometry | 실제 폭·LiDAR 위치와 URDF 차이 | robot description |

한 번에 여러 parameter를 임의로 바꾸지 않는다. held-out command sequence의 simulation-real 오차가 줄었는지 확인한 변경만 nominal config에 반영한다.

## 6. 실제 평가표

동일한 장애물 배치에서 Embedded PATCH-01와 RL policy를 비교한다.

| metric | baseline | RL | 통과 기준 |
|---|---:|---:|---|
| collision | 측정 | 측정 | RL 0회 |
| success rate | 측정 | 측정 | RL이 baseline 이상 |
| minimum range | 측정 | 측정 | safety limit 이상 |
| completion time | 측정 | 측정 | 안전 통과 뒤 비교 |
| p95 inference latency | 해당 없음 | 측정 | control period 미만 |
| safety intervention | 측정 | 측정 | 원인별 보고 |

simulation 결과와 real 결과를 같은 표의 별도 행으로 둔다. 실제 결과가 없으면 “Sim2Real 성공”이라고 쓰지 않는다.

## 7. 완료 조건

- model·normalization·action table·runtime을 하나의 deployment manifest로 고정
- real MCAP replay에서 feature와 action 검사 통과
- shadow mode에서 motor command 없이 disagreement·latency 기록
- 모든 policy command가 safety supervisor와 Yahboom adapter를 통과
- 단계별 guarded 시험과 중단 사유 보존
- Embedded PATCH-01 baseline 대비 collision 0, success rate 동등 이상
- 실제 결과와 simulation 결과의 차이 및 조정 근거 기록

**Embedded PATCH-02가 중간 목표의 완료점이다: Yahboom 실물에서 안전 회피와 강화학습 로직을 재현 가능한 방식으로 비교한다.**
