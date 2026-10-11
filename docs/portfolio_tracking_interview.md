# Mobin — 추적 설계 및 검증 방법 면접 보조 자료

2026-09-15. [포트폴리오 수록 원고](portfolio-selected-content.md)의 근거 및 설명 자료. 기존 코드와 보존된 실행 기록을 대조했으며 이번 문서 작성에서 시뮬레이션을 새로 실행하지 않았다.

## 1. 무엇을 성과로 말할 수 있는가

| 구분 | 확인한 내용 | 해석 범위 |
|---|---|---|
| 동일 입력 전후 비교 | FN 570→3, F1 15.51→91.07%, ID switch 8→8 | 거리 후보 선택 조건 변경의 효과. Kalman Filter·gate·Hungarian 각각의 효과를 분리한 실험은 아님 |
| 추적 설계 | 위치·속도 예측, 불확실성 기반 후보 제한, 일대일 대응, 상태 관리 | 코드로 확인한 구현. 추적 안정성 개선율로 환산하지 않음 |
| 합성 ROS 검증 | 인위적으로 만든 mask와 점군 10사례 | 입력과 정답을 통제해 연결 노드의 특정 동작 및 잔여 실패를 재현 |
| Gazebo 통합 평가 | 사람 3명, 60초, F1 88.07%, ID switch 15회 | 센서 및 인식·추적을 연결한 별도 실행의 결과 |

근거: 동일 입력 [수정 전](../trials/fn-association-support/before.json) / [수정 후](../trials/fn-association-support/after.json), [Gazebo 결과](../trials/portfolio-current-perception/summary.json), [합성 결과](../trials/mask-projection-validation/visibility.json).

FN 감소율은 `(570−3)/570×100=99.47%`, 반올림하여 99.5%다. F1은 `2TP/(2TP+FP+FN)`이며 15.51→91.07%는 **75.56%p 증가**다. 서로 다른 실행의 FN 130 또는 FN 20을 FN 3과 전후 비교하지 않는다.

## 2. 추적 설계 — 각 알고리즘의 역할

**관측**은 현재 센서 입력으로 만든 사람 위치 후보다. **Track**은 여러 시각에 걸쳐 유지하는 ID와 위치·속도 추정 상태다. 잘못된 관측이 입력될 수 있으므로 Track도 실제 사람임을 보증하지 않는다.

| 순서 | 처리와 목적 | 코드 근거 |
|---|---|---|
| 1 | LiDAR에서 선택한 3D 위치를 공통 추적 좌표계로 변환. 기본 좌표계는 `odom` | [연결 노드의 좌표 변환](../code/cpp/mobile_robot_lab_cpp/src/lidar_bbox_association_node.cpp#L578) |
| 2 | 입력 위치의 x·y와 메시지 timestamp를 추적기에 전달. z는 추적 상태에서 제외 | [PersonTrackerNode::on_detections](../code/cpp/mobile_robot_lab_cpp/src/person_tracker_node.cpp#L81) |
| 3 | 기존 Track들을 현재 관측 시각까지 예측 | [ConstantVelocityTracker::predict](../code/cpp/mobile_robot_lab_cpp/include/mobile_robot_lab_cpp/constant_velocity_tracker.hpp#L246) |
| 4 | 각 Track과 관측의 거리 및 불확실성을 비교해 가능한 대응만 남김 | [ConstantVelocityTracker::assign](../code/cpp/mobile_robot_lab_cpp/include/mobile_robot_lab_cpp/constant_velocity_tracker.hpp#L274) |
| 5 | Hungarian으로 비용 합이 최소인 일대일 대응 선택. 미대응도 허용 | [hungarian_assignment](../code/cpp/mobile_robot_lab_cpp/include/mobile_robot_lab_cpp/constant_velocity_tracker.hpp#L66) |
| 6 | 대응된 Track 보정, 미대응 Track 상태 및 수명 관리, 미대응 관측으로 새 Track 생성 | [ConstantVelocityTracker::update](../code/cpp/mobile_robot_lab_cpp/include/mobile_robot_lab_cpp/constant_velocity_tracker.hpp#L157) |

좌표 변환은 연결 노드에서 수행한다. 추적 노드는 입력 header 좌표계를 보존하므로 자체적으로 모든 입력을 `odom`으로 변환한다고 설명하면 부정확하다.

### Constant Velocity Kalman Filter: 어디로 이동할지 예측

상태는 `x=[px, py, vx, vy]ᵀ`. 위치 단위 m, 속도 m/s, 시간 간격 Δt는 입력 timestamp 차이(초)다. 관측 `z=[px,py]ᵀ`는 위치만 제공한다. 속도는 연속 위치 관측과 상태 보정으로 추정한다.

현재 속도를 짧은 시간 동안 유지한다고 가정한다.

$$p^- = p + v\Delta t,\qquad v^- = v$$
$$x^- = Fx,\qquad P^- = FPF^T+Q$$

`F`는 위치에 `vΔt`를 더하는 전이행렬, `P`는 상태 공분산, `Q`는 모델로 설명하지 못한 움직임의 불확실성이다. `predict()`의 Q는 가속도 분산에 Δt⁴/4, Δt³/2, Δt² 항을 곱하는 구조다. **사람이 실제로 항상 일정 속도로 걷는다는 주장이 아니다.** 방향 전환이나 가속이 생기면 다음 관측으로 보정하며, 큰 변화에서는 대응을 놓칠 수 있다.

아래는 실제 코드의 예측 핵심이다.

```cpp
track.estimate = transition * track.estimate;
track.covariance = transition * track.covariance * transition.transpose() +
  variance * process_noise;
```

**설명용 가상 예:** 위치 (2,1)m, 속도 (0.5,0)m/s에서 0.2초 뒤 예측 위치는 (2.1,1)m. 이 숫자는 실험에서 측정한 오차나 성능이 아니다.

대응된 관측이 있으면 [update_measurement()](../code/cpp/mobile_robot_lab_cpp/include/mobile_robot_lab_cpp/constant_velocity_tracker.hpp#L325)에서 보정한다.

$$r=z-Hx^-,\quad S=HP^-H^T+R,\quad K=P^-H^TS^{-1}$$
$$x^+=x^-+Kr$$

`H`는 상태에서 위치를 선택하는 행렬, `R`은 관측 잡음 공분산, `r`은 관측과 예측의 차이, `S`는 그 차이의 공분산, `K`는 보정 가중치다. 공분산 보정은 Joseph 형태 `(I−KH)P⁻(I−KH)ᵀ+KRKᵀ`를 사용한다.

### Mahalanobis gate: 같은 사람일 가능성이 낮은 대응 제외

[assign()](../code/cpp/mobile_robot_lab_cpp/include/mobile_robot_lab_cpp/constant_velocity_tracker.hpp#L274)은 다음 두 조건을 모두 검사한다.

$$d^2=r^TS^{-1}r\leq5.991,\qquad \lVert r\rVert\leq0.75\text{ m}$$

5.991과 0.75m는 **코드 기본 설정**이다. 모든 과거 실행에서 같은 값이 적용됐거나 실측 최적화로 선정됐다는 뜻은 아니다. 평가기의 0.75m 위치 대응 기준과 추적기의 gate는 목적이 다른 설정이다.

Mahalanobis 거리는 예측 불확실성을 고려한다. 불확실성이 큰 방향의 위치 차이는 상대적으로 덜 불리하게 평가한다. 다만 불확실성이 커졌다는 이유로 멀리 떨어진 다른 사람까지 대응하지 않도록 절대거리 제한을 함께 둔다.

```cpp
if (euclidean <= config_.euclidean_gate &&
  mahalanobis_squared <= config_.mahalanobis_gate_squared)
{
  costs[row][column] = mahalanobis_squared;
}
```

**설명용 가상 예:** `r=(0.3,0)m`, `S=diag(0.09,0.09)m²`이면 d²=1이므로 두 조건을 통과한다. 같은 위치 차이에서 `S=diag(0.01,0.01)m²`로 매우 확신하는 추정이라면 d²=9여서 제외된다. 후자는 원리 설명을 위한 다른 잡음 설정이며, 기본 관측 표준편차 0.20m의 실제 실행 예가 아니다. 위치 차이가 0.8m면 d²가 작아도 절대거리 조건에서 제외된다.

### Hungarian: 여러 사람의 대응을 함께 선택

각 Track이 먼저 가까운 관측을 차지하면 뒤의 Track에 불리한 대응만 남을 수 있다. Hungarian은 허용된 대응 비용의 **전체 합**을 최소화하고 한 관측이 여러 Track을 동시에 갱신하지 못하게 한다.

다음은 계산 원리를 설명하는 **가상 비용표**이며 실제 기록에서 추출한 값이 아니다. 네 조합 모두 거리 gate를 통과했다고 가정한다.

| Mahalanobis 제곱거리 비용 | 관측 A | 관측 B |
|---|---:|---:|
| 기존 Track 1 | 1.0 | 2.0 |
| 기존 Track 2 | 1.1 | 5.0 |

- Track 1부터 가장 작은 비용을 선택하면 1→A, 2→B: 합계 **6.0**.
- 전체 합을 비교하면 1→B, 2→A: 합계 **3.1**로 더 작다.

실제 코드는 가상 행과 열을 추가하여 **미대응**을 선택할 수 있게 한다. 기본 미대응 비용은 각 5.991, 금지 비용은 그 1,000배다. 유효한 후보가 없으면 억지로 다른 사람과 연결하지 않는다. Hungarian은 정해진 비용의 최솟값을 찾으며, 그 비용이 실제 사람의 정체성을 완전히 표현한다는 보장은 없다.

### Track 상태와 시간 처리

근거: [update()](../code/cpp/mobile_robot_lab_cpp/include/mobile_robot_lab_cpp/constant_velocity_tracker.hpp#L157), [confirmed()](../code/cpp/mobile_robot_lab_cpp/include/mobile_robot_lab_cpp/constant_velocity_tracker.hpp#L362), [TrackerConfig](../code/cpp/mobile_robot_lab_cpp/include/mobile_robot_lab_cpp/constant_velocity_tracker.hpp#L49).

| 조건 | 현재 동작 |
|---|---|
| 기존 Track과 연결되지 않은 관측 | 새 ID 생성, 초기 속도 0, 첫 hit 기록 |
| 최근 최대 5회 갱신 중 hit 3회 이상 | confirmed. 반드시 5회가 모두 쌓여야 하는 것은 아님 |
| 이번 갱신에서 미대응 | lost로 표시하고 예측 상태 유지 |
| 마지막 실제 측정 이후 max_age 초과 | 대응 처리 후 삭제 검사. 기본 max_age=0.5초 |
| timestamp 역행 | 이전 Track 상태 초기화 |

주의: 삭제 검사는 **대응 및 보정 이후**다. 따라서 “0.5초를 넘은 Track은 대응 후보에서도 무조건 제외한다”고 설명하면 코드와 다르다. 또한 타이머가 아니라 입력에 따른 갱신 경로에서 처리한다.

기본 관측 표준편차는 0.20m, 가속도 표준편차는 1.0m/s², 초기 속도 표준편차는 1.0m/s다. 이 값들의 최적성을 입증한 별도 실험은 여기 제시하지 않는다.

## 3. 모의 상황을 어떻게 만들었는가

### 시험 목적과 입력 생성

목적은 카메라 영상 인식률을 측정하는 것이 아니라, **정답을 아는 mask 및 3D 점을 넣었을 때 연결 노드가 어떤 점과 깊이를 출력하는지** 확인하는 것이다.

구현: [probe.py의 main()](../trials/mask-projection-validation/probe.py#L52), 결과: [visibility.json](../trials/mask-projection-validation/visibility.json).

| 구성 요소 | 실제 생성 방법 |
|---|---|
| 사람 영역 | 160×120 배열에 직사각형, 내부 구멍, 분리된 두 영역을 직접 생성 |
| 카메라 정보 | 초점거리 fx=fy=100픽셀, 주점 (80,60)픽셀의 CameraInfo 생성 |
| 사람 및 배경 측정 | Camera optical 좌표에서 점의 x·y·z를 지정. z를 1m, 2m, 5m 등으로 바꾸어 깊이 관계 통제 |
| 센서 위치 관계 | LiDAR를 카메라 오른쪽 0.8m에 둔 합성 TF 구성. 실제 TurtleBot 장착 간격을 측정한 값이 아님 |
| 검출 메시지 | 사람 bbox와 confidence 0.99를 직접 지정. YOLO 추론 결과가 아님 |
| ROS 전달 | CameraInfo, instance label Image, Detection2DArray, PointCloud2를 동일 timestamp로 발행 |
| 실행 대상 | 실제 `lidar_bbox_association_cpp` 노드. 시험 출력 좌표계는 `probe_camera_optical` |
| 판정 | 출력 깊이와 연결된 점 개수를 사전에 지정한 기대값과 비교. 깊이 허용 오차 10⁻⁵m |

위 허용 오차는 수치적으로 구성한 입력의 **assert 기준**이며 센서 거리 정확도 10⁻⁵m를 달성했다는 뜻이 아니다. 반복 발행은 출력을 받기 위한 절차이며, 서로 독립적인 실환경 표본 수가 늘어나는 것은 아니다.

이 시험에서 Gazebo 렌더링, 실제 레이저 반사, YOLO 추론, Kalman 추적은 수행하지 않았다. 별도 [YOLO 추론 확인 기록](../trials/mask-projection-validation/inference.json)과도 구분한다.

### 왜 사람 뒤 벽의 점이 들어오는가

카메라와 LiDAR의 설치 위치가 다르면 같은 물체를 보는 방향도 다르다. LiDAR에서 사람 옆으로 지나 벽에 도달한 빛은 LiDAR로 돌아온다. **벽에서 사람으로 돌아온 빛을 카메라가 검출하는 상황이 아니다.** 그 벽의 3D 측정점을 카메라 영상에 투영할 때 사람 영역과 겹칠 수 있다.

기존 [레이저 왕복과 영상 투영 SVG](../trials/turtlebot-portfolio-photo/laser-return-and-projection.svg)를 함께 사용한다. 합성 시험에서는 LiDAR x=0.8m, 사람 깊이 2m, 벽 깊이 5m로 두었고, 벽을 향한 선분이 사람 깊이를 지날 때 x=0.468~0.492m로 계산되어 가정한 사람 반폭 0.2m 밖을 지난다. 이는 지정한 기하 조건의 확인이며 재질이나 다중 반사를 시뮬레이션한 결과가 아니다.

필터는 같은 영상 픽셀에 실제로 들어온 측정값 중 최소 깊이보다 5cm 초과 뒤쪽인 점을 제외한다. 앞쪽 사람의 반사점이 아예 없으면 비교 근거가 없으므로 벽만 채택할 수 있다.

### ROS 합성 10사례의 실제 규모와 결과

원본 이름과 수치는 [visibility.json의 ros_cases](../trials/mask-projection-validation/visibility.json)에 대응한다. ‘채택 점 수’는 사람 정답 점 수와 항상 같지 않다.

| 사례 | 입력 점 수 | 출력 깊이 / 채택 점 수 | 확인한 의미 |
|---|---:|---|---|
| 사람만 입력 | 9 | 2m / 9 | 기본 사람 위치 선택 |
| 구멍을 보존한 mask + 앞 물체 | 69 | 2m / 8 | 구멍 영역의 앞 물체 배제 |
| 기존 polygon 변환 + 같은 입력 | 69 | 1m / 60 | **기존 오류 재현:** 앞 물체 오채택 |
| 원본 binary mask 전달 + 같은 입력 | 69 | 2m / 8 | 원본 mask 보존으로 해당 오류 해소 |
| 사람 점 없이 벽만 입력 | 6 | 5m / 6 | **미해결:** 벽을 사람 위치로 오채택 |
| 사람 및 벽 점 혼합 | 12 | 2m / 9 | 이 구성에서 사람 깊이 선택 |
| 같은 픽셀의 사람 3점 + 벽 6점 | 9 | 2m / 3 | 벽 6점 제외, 사람 3점 보존 |
| 이웃 픽셀 앞 물체 + 먼 사람 | 4 | 5m / 3 | 필터를 이웃 픽셀까지 확장하지 않아 사람 점 보존 |
| 2.00·2.03·2.04m의 가까운 깊이 | 3 | 약 2.03m / 3 | 5cm 이내 차이는 보존 |
| 같은 픽셀 앞 물체 + 먼 사람 | 4 | 출력 없음 / 0 | **한계 재현:** 앞 물체 때문에 먼 사람 점 제외 |

10사례 중에는 수정 전 비교와 알려진 실패가 포함된다. “10사례 모두 사람 검출 성공”, “부분 가림 대응률 100%”, “벽 제거 정확도 100%”로 쓰지 않는다. 사람 3점 보존은 **특정 9점 입력 한 사례**에서 확인한 기능 동작이다.

### 합성 시험과 Gazebo의 차이

| 항목 | 합성 ROS 시험 | 별도 Gazebo 통합 평가 |
|---|---|---|
| 입력 생성 | 직접 만든 mask·검출·점군 | 가상 환경 센서 출력과 인식 결과 |
| 주된 목적 | 원인을 통제한 연결 로직 확인 | 센서부터 추적 출력까지 연결한 결과 측정 |
| 규모 | 구성한 10사례 | 사람 3명, seed 42, 정지 로봇, 60 simulation초 |
| 주요 출력 | 깊이, 채택 점 수, 제외 점 수 | TP 310 / FP 64 / FN 20, ID switch 15 |
| 포함하지 않는 주장 | 전체 인식·추적 정확도 | 실물 검증, 다양한 seed의 평균, 전후 개선율 |

Gazebo의 GT 330은 **누적 사람 관측 330건**이며 330명이 아니다. 평가 갱신은 317회다. 359.800초에 촬영한 두 confirmed 인물의 사진은 이후 별도 촬영 조건으로 만든 예시여서 60초 평가 결과의 특정 프레임 성능으로 설명하지 않는다.

회귀 시험은 [보존 로그](../trials/mask-projection-validation/visibility-tests.log)의 **C++ 실행 파일 5개 + Python 테스트 27개 통과**다. 서로 집계 단위가 다르므로 “32종의 독립 상황에서 실환경 검증”이라고 쓰지 않는다.

## 4. 면접 질문 및 답변

**왜 Kalman Filter를 사용했나?** 3D 관측을 프레임별 위치로 끝내지 않고 속도와 다음 위치를 추정하기 위해 사용했다. 짧은 관측 누락에서는 예측값을 유지하되, 장기 가림에서도 동일 ID가 보장되는 것은 아니다.

**Mahalanobis만 사용하지 않은 이유는?** 불확실성이 커진 Track이 먼 관측까지 허용할 수 있어 절대거리 제한도 함께 적용했다. 두 기준을 동시에 통과한 조합만 Hungarian 비용에 넣는다.

**Hungarian이면 교차 시 ID가 유지되나?** 비용 합을 최소화할 뿐 사람 정체성을 보장하지 않는다. 현재는 위치와 공분산 중심의 대응이며 외형 기반 재식별이 없다. 같은 입력 전후 비교에서도 ID switch는 8회로 동일했다.

**미검출 99.5% 개선은 추적 알고리즘 덕분인가?** 동일한 추적기와 평가 조건을 두고 거리 후보 선택 조건을 바꾼 결과다. Kalman Filter나 Hungarian 도입 효과를 별도로 측정한 숫자가 아니다.

**mask가 정확하면 왜 벽을 선택하나?** mask는 카메라에서 사람으로 보이는 2D 영역이다. 설치 위치가 다른 LiDAR에서 보인 벽 점도 그 영역에 투영될 수 있다. 사람의 앞쪽 측정점이 없으면 깊이 필터가 벽을 제거할 근거도 없다.

**모의 시험이 얼마나 현실적인가?** 깊이, mask 형태, 픽셀 중첩을 통제한 작은 기능 시험이다. 실제 사람 형상·의복·반사율·센서 노이즈의 다양성을 검증하지 않았다. 통합 동작은 별도 Gazebo 실행으로 확인했으며 실물 검증 성과와 구분했다.

**confirmed는 무엇을 보증하나?** 최근 관측이 정해진 횟수 이상 대응됐다는 상태다. 벽 오채택이 반복되어도 confirmed가 될 수 있으므로 사람 정답의 보증으로 쓰지 않는다.

**다음 검증의 우선순위는?** 동일 입력에서 gate와 추적 설정을 하나씩 바꿔 위치 대응과 ID switch를 함께 비교하고, 교차·급회전·가림 길이를 통제한 추적 시험을 추가하는 것이다. 현재 달성한 성과가 아닌 후속 계획이다.

## 5. 1분 설명 원고

> 카메라와 3D LiDAR를 연결해 사람 위치를 만들고, Kalman Filter로 위치와 속도를 추정하는 시스템을 개인 설계 및 구현했습니다. 추적에서는 불확실성과 절대거리로 가능한 대응을 제한하고 Hungarian으로 사람 ID와 새 관측을 일대일 연결했습니다. 주요 개선은 사람과 배경의 거리 후보가 함께 나오면 관측 전체를 버리던 문제였습니다. 유효 후보들의 점 수를 비교하도록 조건을 수정해 동일 저장 입력 997개에서 FN을 570건에서 3건으로 줄였습니다. 검증은 직접 만든 mask와 점군을 실제 ROS 연결 노드에 넣는 원인별 시험과 Gazebo 통합 평가로 구분했습니다. 벽만 들어오는 오채택과 사람 교차 시 ID 변경은 남아 있어, 해결한 문제와 아직 검증이 필요한 범위를 구분해 기록했습니다.
