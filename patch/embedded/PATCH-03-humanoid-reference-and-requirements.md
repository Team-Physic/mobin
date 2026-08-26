# Embedded PATCH-03: 공개 Humanoid 검증과 자체 설계 요구사항

- 작성일: 2026-08-15
- 시작 조건: 없음. 요구사항 초안과 reference 조사는 다른 트랙과 병렬 수행
- 확정 조건: Embedded PATCH-02 또는 동등한 mobile Sim2Real의 interface·dataset·safety 실측
- 대상: 향후 `humanoid/requirements/`, `humanoid/references/`
- 결론: **Berkeley Humanoid Lite를 Isaac Lab→실물 전체 흐름의 주 참고 자료로 사용한다. CAD와 저비용 제작은 ToddlerBot·Open Duck Mini·onshape-to-robot에서 비교하되, 기존 설계의 license 조건 때문에 자체 CAD는 처음부터 별도 원본으로 만든다.**

## Embedded

Humanoid의 embedded architecture는 compute board 선택이 아니라 sensor 입력, policy rate, actuator loop, power, 열, 질량과 fault 책임을 계층별로 나누는 작업이다.

| 개념 | 이 PATCH에서 결정할 질문 |
|---|---|
| compute partition | Pi 5가 실행할 ROS 2·policy와 MCU가 실행할 actuator loop의 경계 |
| control rate | policy, state estimation, motor I/O가 각각 필요한 주기와 deadline |
| power budget | board·sensor·actuator의 평균·peak current와 regulator 여유 |
| hardware interface | UART, CAN, servo bus 중 필요한 bandwidth·fault detection |
| maintainability | board, battery, cable, actuator를 분해·교체할 수 있는가 |

공개 Humanoid architecture와 known issue를 읽고 component datasheet로 voltage·current·interface를 확인한 뒤 single-joint rig에서 가정을 측정한다. 신입은 reference 비교표, interface block diagram, 요구사항의 단위와 측정법을 정의한다. 1~2년차는 actuator step response·temperature·current를 측정하고 trade-off를 ADR로 결정할 수 있다. custom motor driver, battery 보호회로, safety certification은 전문가 review 없이 단독 설계하지 않는다.

GitHub에는 `docs/embedded/03_system_architecture.md`, `docs/embedded/03_component_evidence.md`, `docs/embedded/adr/`를 남긴다. 각 ADR에는 선택지, 측정 근거, 선택, 포기한 조건, 재검토 기준을 쓴다.

### SW 실습

| 실습 | 입력·방법 | 산출물 | 통과 조건 |
|---|---|---|---|
| reference·license 조사 | 고정 commit의 code·CAD·asset 문서 확인 | `SOURCES.md` | component별 license와 사용 방식 분리 |
| 요구사항 관리 | 질량·관절·전원·rate·latency에 단위와 근거 부여 | `requirements.md` | 모든 값이 measured·datasheet·CAD·assumed 중 하나 |
| compute partition | ROS 2·policy·state estimation·motor I/O 책임 배치 | system architecture와 ADR | Pi·MCU 책임과 fault 시 safe owner가 명시됨 |
| actuator 계산 | distal mass와 lever arm으로 정적 torque 후보 계산 | 계산표와 후보 비교 | 단위·safety factor·모델 한계 기록 |
| interface budget | sensor·actuator message 크기와 rate 추정 | bus bandwidth·latency budget | peak traffic에도 목표 update rate의 여유 존재 |

### HW 실습

| 실습 | 필요한 시제품·도구 | 측정값·산출물 | 통과 조건 |
|---|---|---|---|
| 부품 실측 | actuator, Pi 5, battery, sensor, 저울·캘리퍼 | 질량·크기·connector 위치 | datasheet·CAD와 차이 기록 |
| single-joint rig | actuator, 전원, 고정 지그, 부하 | step 지연·overshoot·backlash | 요구 range·속도·반복 오차 충족 |
| 하중·열 시험 | 목표 하중, 전류·온도 측정기 | current·temperature·처짐의 시간 기록 | 정한 제한 안에서 반복 동작 |
| fault 시험 | 통신 차단, 전원 차단, hard stop 근접 | torque-off·safe pose·mechanical 여유 | 위험한 command 유지 없음 |
| 배선·정비성 mock-up | 실제 connector·cable·board | 굽힘·분리·교체 절차 | 관절 운동 중 간섭 없고 부품 교체 가능 |

**SW 계산은 actuator 후보를 거르는 1차 근거다. 최종 선택은 single-joint rig의 전류·온도·응답 측정으로 확정한다.**

## 1. 공개 Humanoid 자료 검증

| 자료 | 공개 범위 | license 확인 | 이 프로젝트에서 가져올 것 | 그대로 복제하지 않을 것 |
|---|---|---|---|---|
| [Berkeley Humanoid Lite](https://github.com/HybridRobotics/Berkeley-Humanoid-Lite) | Isaac Lab task, URDF/MJCF/USD, low-level, policy training·sim2sim·real deployment | code MIT, 기타 asset CC BY-SA 4.0 | 전체 architecture와 검증 순서 | CAD 형상을 자체 설계인 것처럼 사용 |
| [Berkeley Humanoid Lite Assets](https://github.com/HybridRobotics/Berkeley-Humanoid-Lite-Assets) | Onshape→URDF/MJCF, mesh, USD 생성 흐름 | CC BY-SA 4.0 | CAD export와 asset 분리 구조 | share-alike 의무 검토 없이 asset 포함 |
| [ToddlerBot](https://github.com/hshi74/toddlerbot) | Onshape, STL, low-level, RL, real deployment | code MIT, design은 README상 비상업 사용 조건 | 작은 robot packaging·실물 절차 비교 | design을 상업 가능 자산으로 간주 |
| [Open Duck Mini](https://github.com/apirrone/Open_Duck_Mini) | Onshape CAD, print·assembly, Raspberry Pi runtime | repository Apache-2.0, 외부 Onshape 문서 적용 범위는 별도 확인 필요 | 저비용 출력물·배선·onboard inference 참고 | Isaac Lab 예제로 간주 |
| [Open Duck Playground](https://github.com/apirrone/Open_Duck_Playground) | MuJoCo Playground RL, ONNX inference | 저장소 license를 clone 시 재확인 | 작은 biped action·reward·export 비교 | Isaac Lab source와 혼합 복사 |
| [onshape-to-robot](https://github.com/Rhoban/onshape-to-robot) | Onshape assembly를 URDF·SDF·MJCF로 export | MIT | 반복 가능한 CAD→robot description 도구 | tool 출력의 물리값을 검증 없이 신뢰 |
| [onshape-to-robot examples](https://github.com/Rhoban/onshape-to-robot-examples) | 20-DOF Sigmaban humanoid와 collision·frame 예제 | MIT | link 분리, 단순 collision, frame 구성 예제 | Sigmaban 형상 복제 |

**GitHub 공개 여부, code license, CAD/design license는 서로 다르다.** 각 저장소를 실제로 fork하거나 asset을 포함할 때 고정 commit의 `LICENSE`, README, submodule별 license를 다시 기록한다.

## 2. 주 reference 선택 이유

Berkeley Humanoid Lite가 최종 목표와 가장 가깝다.

| 필요한 흐름 | Berkeley 자료 |
|---|---|
| CAD 원본 | Onshape 기반 공개 asset |
| robot description | URDF, MJCF, USD |
| simulator | Isaac Lab |
| 학습 | locomotion task·policy training |
| 검증 | sim2sim과 real deployment |
| 실물 제어 | 별도 low-level package |

단, release note에는 3D-printed cycloidal actuator의 내구성과 controller connector·배선 신뢰성 문제가 명시되어 있다. 이 문제도 설계 입력으로 취급한다. **저렴하다는 이유만으로 고토크 3D-printed 감속기를 첫 자체 actuator로 만들지 않는다.**

## 3. 첫 자체 Humanoid의 범위를 제한한다

첫 기체는 사람과 같은 모든 기능이 아니라 **안전하게 서기·자세 유지·저속 보행을 검증하는 소형 biped MVP**다.

| 항목 | 첫 목표 | 이후 확장 |
|---|---|---|
| 관절 | 좌우 다리 12-DOF 후보: hip 3, knee 1, ankle 2씩 | 팔, 손, 움직이는 head |
| task | stand, weight shift, 저속 평지 보행 | 경사·계단·조작 |
| perception | joint encoder, IMU | camera, depth, foot force sensor |
| compute | Raspberry Pi 5에서 state·policy·logging | 별도 accelerator |
| low-level | MCU 또는 smart-servo bus controller | custom motor controller |
| 구조 | service 가능한 modular link | 외장과 표현 기능 |

12-DOF는 확정값이 아니라 첫 kinematic 검토안이다. CAD 시작 전에 actuator torque, 전체 mass, battery, 배선 통로를 계산해 관절 수를 확정한다.

## 4. 요구사항을 숫자로 만든다

`humanoid/requirements/requirements.md`에 다음 값을 단위와 함께 확정한다.

| 범주 | 반드시 정할 값 |
|---|---|
| 크기 | 전체 높이, hip 높이, 발 길이·폭 |
| 질량 | 전체 목표, link별 budget, battery·compute mass |
| 관절 | axis, range, 최대 속도, 연속·peak torque |
| 보행 | 목표 속도, step length, 최소 stance time |
| 전원 | battery voltage·capacity, peak current, fuse |
| 계산 | policy rate, low-level loop rate, 허용 end-to-end latency |
| 안전 | joint soft/hard limit, fall detection, emergency stop |
| 제작 | printer 크기·재료, insert·bearing·fastener 규격 |

각 값에는 다음 상태 중 하나를 붙인다.

| 상태 | 뜻 |
|---|---|
| measured | 실제 부품·시제품 측정 |
| datasheet | 제조사 문서 값 |
| CAD | CAD mass property 계산 |
| assumed | 아직 검증하지 않은 설계 가정 |

`assumed` 값으로 torque와 안전 한계를 최종 확정하지 않는다.

## 5. Actuator 후보를 먼저 시험한다

전체 robot을 출력하기 전 **한 관절 test rig**를 만든다.

정적 screening 식:

$$
\tau_{required} \ge S \cdot m_{distal} g r_{perp}
$$

| 기호 | 의미 |
|---|---|
| `m_distal` | 해당 관절이 지지하는 아래쪽 질량 |
| `r_perp` | 관절축에서 질량중심까지 수직 거리 |
| `S` | 가속·충격·모델 오차를 위한 safety factor |

이 식은 정지 상태의 1차 검사일 뿐이다. 보행 peak torque는 trajectory simulation과 실제 rig에서 확인한다.

| rig 시험 | 기록 |
|---|---|
| 무부하 position step | 지연, overshoot, 반복 오차 |
| 정격 하중 유지 | 전류, 온도, 처짐 |
| 반복 왕복 | backlash, 열화, connector 풀림 |
| 전원 차단·통신 끊김 | 안전한 정지 동작 |
| hard stop 접근 | software limit과 mechanical stop 여유 |

## 6. License와 출처 기록

`humanoid/references/SOURCES.md`에 다음 표를 유지한다.

| field | 예시 내용 |
|---|---|
| project | Berkeley Humanoid Lite |
| URL | 원본 repository·release |
| commit/tag | 실제 참조한 고정값 |
| component | code, CAD, mesh, document 구분 |
| license | 해당 component의 license |
| use | 아이디어 참고, 수정, 포함 여부 |
| attribution | 배포물에 넣을 고지 |

자체 CAD에는 기존 mesh를 import해 윤곽을 따라 그리지 않는다. 외부 부품의 제조사 STEP은 조립 간섭 확인용으로 두고 배포 허가를 별도로 확인한다.

## 7. 완료 조건

- reference별 code·design·asset license를 분리 기록
- Berkeley Humanoid Lite를 주 reference로 선택한 근거와 알려진 hardware 위험 기록
- 첫 biped MVP의 포함·후속 범위 확정
- 높이·질량·관절·전원·제어 주기의 수치 요구사항 초안 작성
- actuator 후보별 datasheet와 single-joint rig 결과 저장
- CAD 시작 전에 관절 수·axis·actuator·battery 배치 review 통과
- 출처 없는 CAD·mesh가 project tree에 없음

**Embedded PATCH-03 산출물은 robot 파일이 아니라 검증 가능한 설계 요구사항이다. 이를 통과해야 Embedded PATCH-04에서 CAD를 시작한다.**
