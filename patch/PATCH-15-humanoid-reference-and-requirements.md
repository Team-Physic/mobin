# PATCH-15: 공개 Humanoid 검증과 자체 설계 요구사항

- 작성일: 2026-08-15
- 선행 조건: PATCH-14 또는 mobile Sim2Real에서 얻은 interface·dataset·safety 경험
- 대상: 향후 `humanoid/requirements/`, `humanoid/references/`
- 결론: **Berkeley Humanoid Lite를 Isaac Lab→실물 전체 흐름의 주 참고 자료로 사용한다. CAD와 저비용 제작은 ToddlerBot·Open Duck Mini·onshape-to-robot에서 비교하되, 기존 설계의 license 조건 때문에 자체 CAD는 처음부터 별도 원본으로 만든다.**

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

**PATCH-15 산출물은 robot 파일이 아니라 검증 가능한 설계 요구사항이다. 이를 통과해야 PATCH-16에서 CAD를 시작한다.**
