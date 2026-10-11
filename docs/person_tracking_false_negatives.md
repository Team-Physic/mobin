# 사람 추적 FN: 전경 후보의 과도한 거부 수정

- 작성일: 2026-09-07
- 코드 기준: `feature/avoidance`, HEAD `73f81c4` + 기존 working tree + 이번 수정
- 상태: **Binary mask 보존과 동일 pixel 가림 제외 구현·검증 완료.** 앞쪽 반사점 없는 벽 오채택은 미해결. 기존 FN 감소와 이번 합성 검증은 분리한다.
- 범위: Camera–LiDAR association. Tracker·GT·평가 gate는 수정하지 않았다.

**최신 검증:** 같은 pixel의 사람 3점+벽 6점에서 변경 전 ROS binary는 관측을 거부했고, 변경 후는 사람 3점·2m 관측을 발행했다. 합성 ROS 10사례·C++ 5개·Python 27개 검증 통과. 앞쪽 반사점 없는 벽 5m 오채택은 남는다. 아래 「동일 pixel 가림 제외 구현」 참조.

## Why?

9월 4일 기록은 FN 130이었다. [frame별 결과](../trials/seeded-multi-person-tracking/roi-associations.csv)를 timestamp별로 분해하니 123건은 평가 ROI 안에 예측 track이 없는 시점, 7건은 예측이 있지만 GT와 연결되지 않은 시점이었다. 평가 매칭만 바꿔서는 대부분을 해결할 수 없었다.

[당시 diagnostics](../trials/seeded-multi-person-tracking/evidence.json)는 association 성공 18건, 복수 후보 거부 207건을 기록했다. 원시 센서 bag은 없었으므로 당시 FN 130을 그대로 재생할 수 없었다. **새로운 seed 42·사람 3명·60초 실행에서 원시 센서 입력을 기록한 뒤 같은 입력으로 수정 전후를 비교했다.**

## What was problem

**코드상 직접 원인은 후보 개수만으로 관측을 거부하는 조건이었다.**

`trials/fn-association-support/baseline-association.hpp` | [analyze_instance_points()](../trials/fn-association-support/baseline-association.hpp#L111)의 보존된 수정 전 원문:

```cpp
// trials/fn-association-support/baseline-association.hpp | analyze_instance_points()
  if (candidates.size() > 1) {
    return {AssociationStatus::MULTIPLE_DEPTH_CANDIDATES, std::nullopt};
  }
```

Mask 안의 point를 깊이별로 나누면 실제 사람 쪽 전경과 뒤쪽 표면이 함께 남을 수 있다. 기존 코드는 어느 쪽의 point 지지가 더 강한지 확인하지 않았다. 따라서 사람 후보에 point가 충분해도 배경 후보가 하나 추가되면 관측 전체가 사라졌다.

**새 기록에서 확인한 근거:** 997개 시간 정렬 입력, 796개 person instance 중 727개에서 복수 후보가 있었다. 그 727개 모두 최근접 후보의 대표 위치가 어느 사람 GT의 평면 위치로부터 0.75 m 이내였다. GT는 이 사실을 확인하는 평가에만 사용했으며 런타임 후보 선택에는 전달하지 않았다. [후보별 기록](../trials/fn-association-support/candidate-examples.json)

| 실제 기록 | 최근접 후보 | 뒤쪽 후보 | 수정 전 판정 |
|---|---|---|---|
| simulation 1.2초 | 깊이 4.135 m, 86 points, GT 오차 0.139 m | 깊이 8.618 m, 10 points, GT 오차 4.836 m | 후보가 2개라 전체 거부 |
| simulation 39.5초 | 깊이 3.858 m, 48 points | 깊이 6.885 m의 3 points + 8.151 m의 26 points | 후보가 3개라 전체 거부 |

깊이는 Camera optical frame의 z(m), GT 오차는 odom 평면 거리(m)다. 뒤쪽 point가 mask 안에 들어온 물리적 원인을 가림·시점 차이·mask 경계 중 하나로 확정한 것은 아니다. **이번에 입증하고 수정한 것은 그런 추가 후보 때문에 유효 전경까지 일괄 거부하던 정책이다.**

## What I Made

`code/cpp/mobile_robot_lab_cpp/include/mobile_robot_lab_cpp/camera_lidar_fusion.hpp` | [analyze_instance_points()](../code/cpp/mobile_robot_lab_cpp/include/mobile_robot_lab_cpp/camera_lidar_fusion.hpp#L158)는 다음 조건을 사용한다.

$$N_{\mathrm{front}} \ge r\sum_{j>0}N_j,\qquad r=2.0$$

- $N_{\mathrm{front}}$: Camera 깊이가 가장 작은 유효 후보의 point 수.
- $N_j$: 나머지 유효 후보 각각의 point 수. 모두 합산한다.
- $r$: 최소 지지 비율. 단위 없는 유한한 실수이며 1보다 커야 한다.
- 유효 후보는 기존과 동일하게 `min_points=3` 이상, 깊이 분리 간격은 `depth_window=0.35 m`다.
- 후보가 하나면 뒤쪽 합계가 0이므로 채택한다. 여러 후보면 최근접 후보가 나머지 전체의 기본 2배 이상일 때만 채택한다.
- 비율을 키우면 더 엄격하게 거부한다. GT·사람 위치 정답은 이 식에 들어가지 않는다.

```cpp
// code/cpp/mobile_robot_lab_cpp/include/mobile_robot_lab_cpp/camera_lidar_fusion.hpp | analyze_instance_points()
  const auto [selected_begin, selected_end] = candidates.front();
  std::size_t background_count = 0;
  for (std::size_t index = 1; index < candidates.size(); ++index) {
    background_count += candidates[index].second - candidates[index].first;
  }
  // ... 생략: 설명 주석
  if (static_cast<double>(selected_end - selected_begin) <
    foreground_support_ratio * static_cast<double>(background_count))
  {
    return {AssociationStatus::MULTIPLE_DEPTH_CANDIDATES, std::nullopt};
  }
```

1.2초 사례는 `86 ≥ 2×10`이므로 전경을 채택한다. 39.5초 사례는 `48 < 2×(3+26)`이므로 **거부를 유지한다**. 단순히 가장 가까운 후보를 무조건 고르거나 point가 많은 배경을 사람으로 선택하지 않는다.

이것은 **point 지지에 기반한 휴리스틱**이다. 근접 오검출·센서 시점 차이·occlusion을 완전하게 판정하는 알고리즘은 아니다. 이번 수정으로 sensor pose나 TF 자체를 바꾸지는 않았다.

## 검토 보완: 최근접 후보는 사람의 의미적 증거가 아니다

사용자 지적(2026-09-07): 최근접 후보라고 반드시 사람은 아니다. 기존 CONTEXT.md의 Foreground Depth Candidate 정의도 같은 한계를 명시한다.

- 코드로 확인한 사실: 깊이 순 첫 유효 후보를 선택하고 point 수 비율만 검사한다. 이 함수에는 후보가 사람인지 독립적으로 판별하는 검사가 없다.
- 반례(가상 입력): mask 영역에 들어온 앞쪽 비사람 후보가 60 points, 뒤쪽 실제 사람이 20 points라면 `60 ≥ 2×20`이므로 잘못된 앞쪽 후보를 채택한다. 비율은 의미적 신뢰도나 사람일 확률이 아니다.
- 기존 검증의 한계: 최근접 후보가 어느 사람 GT와 0.75 m 이내라는 결과는 해당 mask의 사람에게 point가 실제로 속한다는 증명이 아니다. 같은 기록의 FN 감소는 확인했지만 새로운 가림 조건으로 일반화할 수 없다.
- 결론: 이번 변경은 전경 우세 조건에서 과도한 거부를 완화한 휴리스틱이다. 사람 소속 판정 자체를 해결한 것으로 표현하지 않는다.
- 사용자 확정: 앞쪽 비사람 물체가 person 영역에 혼입되는 경우를 필수 검증에 포함한다.
- 실제 반례 실행: [probe source](../trials/fn-association-support/foreground-occluder-probe.cpp)의 `main()`은 2 m 비사람 60점과 5 m 사람 20점을 입력한다. 현재 코드는 2 m를 채택했고 [실행 로그](../trials/fn-association-support/foreground-occluder-probe.log)의 종료 코드 1로 실패했다. 단위 수준 합성 입력이며 실제 sensor occlusion 재현은 아직 아니다.
- 필수 완료 조건: 앞쪽 비사람을 사람 관측으로 발행하지 않는다. 모호한 관측의 거부를 허용할지, 뒤쪽 사람의 회복까지 요구할지는 다음 설계 결정이다. 단일 유효 후보에 비사람만 남는 경우도 별도 검증해야 한다.
- 정보의 한계: 같은 좌표·점 수를 가진 입력에 의미적 정답만 바꾸면 현재 함수는 같은 결과를 낸다. point 지지 임계값만으로 두 경우를 구분할 수 없다. 구현 변경은 아직 하지 않았으며 전경 지지 정책을 완료된 해결책으로 취급하지 않는다.

## How it changed

| 파일 위치 | 함수 또는 설정 | 변경 요약 |
|---|---|---|
| `code/cpp/mobile_robot_lab_cpp/include/mobile_robot_lab_cpp/camera_lidar_fusion.hpp` | [analyze_instance_points()](../code/cpp/mobile_robot_lab_cpp/include/mobile_robot_lab_cpp/camera_lidar_fusion.hpp#L158) | 이전: 복수 후보 일괄 거부.<br>변경: 최근접 후보와 나머지 합계의 point 지지 비교.<br>효과: 작은 배경 후보 때문에 유효 전경이 사라지는 문제 완화. |
| `code/cpp/mobile_robot_lab_cpp/include/mobile_robot_lab_cpp/camera_lidar_fusion.hpp` | [associate_instance_points()](../code/cpp/mobile_robot_lab_cpp/include/mobile_robot_lab_cpp/camera_lidar_fusion.hpp#L245) | 이전: 기존 후보 판정을 감싼 wrapper.<br>변경: 최소 전경 지지 비율 전달.<br>효과: 직접 instance 입력에도 같은 판정 사용. |
| `code/cpp/mobile_robot_lab_cpp/include/mobile_robot_lab_cpp/camera_lidar_fusion.hpp` | [associate_bbox()](../code/cpp/mobile_robot_lab_cpp/include/mobile_robot_lab_cpp/camera_lidar_fusion.hpp#L260) | 이전: bbox 후보의 기존 판정 사용.<br>변경: 최소 전경 지지 비율 전달.<br>효과: C++ bbox 비교 경로에도 동일한 기준 사용. |
| `code/cpp/mobile_robot_lab_cpp/src/lidar_bbox_association_node.cpp` | [LidarBboxAssociationNode()](../code/cpp/mobile_robot_lab_cpp/src/lidar_bbox_association_node.cpp#L106) | 이전: 전경 지지 비율 parameter 없음.<br>변경: `foreground_support_ratio=2.0` 선언, 유한한 값 `>1` 검증.<br>효과: 센서 조건에 맞춰 startup 설정 가능. |
| `code/cpp/mobile_robot_lab_cpp/src/lidar_bbox_association_node.cpp` | [process_pair()](../code/cpp/mobile_robot_lab_cpp/src/lidar_bbox_association_node.cpp#L344) | 이전: 단일 후보만 허용하는 association 호출.<br>변경: 설정된 전경 지지 비율 전달.<br>효과: ROS 입력에 새 조건 반영. 기존 diagnostics 이름 유지. |
| `code/python/mobile_robot_lab_python/launch/yolo_lidar_fusion.launch.py` | [generate_launch_description()](../code/python/mobile_robot_lab_python/launch/yolo_lidar_fusion.launch.py#L19) | 이전: 비율 launch argument 없음.<br>변경: `foreground_support_ratio` argument 선언·C++ node 전달.<br>효과: 코드 수정 없이 직접 perception launch에서 비율 선택. |

삭제한 함수는 없다. 기존 tracker, Kalman filter, 평가 거리 0.75 m, GT 집합 선택 코드는 수정하지 않았다. README와 PATCH-07의 단일 후보 제한 설명도 현재 규칙으로 교체했다.

## 검증 결과

### 동일 입력 C++ 비교: 인과 확인용

| 항목 | 수정 전 | 수정 후 |
|---|---:|---:|
| 입력 frame | 997 | 997 |
| 평가 GT 표본 | 635 | 635 |
| TP | 65 | 632 |
| FN | 570 | 3 |
| FP | 138 | 121 |
| ID switch | 8 | 8 |
| IDF1 | 5.49% | 42.22% |
| Association 성공 | 57 | 654 |
| 복수 후보 거부 | 727 | 130 |
| Point 부족 | 12 | 12 |

근거: [수정 전 JSON](../trials/fn-association-support/before.json), [수정 후 JSON](../trials/fn-association-support/after.json), [조건·입력 hash](../trials/fn-association-support/environment.json), [source hash](../trials/fn-association-support/source-sha256.json).

**FN은 99.47% 감소했다.** FN만 줄이기 위해 FP를 늘린 결과도 아니다. 다만 FP 121, ID switch 8, IDF1 42.22%가 남아 있으므로 고성능·안정적 추적 완성이라고 표현하지 않는다.

비교에서는 같은 label·detection timestamp와 가장 가까운 cloud(50 ms 이내)를 사용하고, 기존 C++ ground filter의 출력을 고정했다. 실제 C++ association과 tracker를 binding으로 호출했다. GT는 기존 evaluator의 model 선택·odom 변환·공통 기하학적 FoV 기준을 사용했다. 입력은 Camera optical 좌표로 보존했으며 동일한 변환으로 odom 관측을 만들었다.

온라인 ApproximateTime synchronizer와 오프라인 최근접 pair 선택은 다르다. 따라서 이것은 **동일 입력의 C++ core 비교**이며 기존 9월 4일 FN 130과의 직접 전후 비교가 아니다. 기존 평가기처럼 GT와 예측이 모두 없는 frame은 지표의 `frames`에서 제외돼, 그 값은 전후 674/644로 다르다. 입력 997개와 평가 GT 635개는 동일하다.

### ROS 노드 재생: 연결 검증용

수정된 ROS association·tracker·evaluator에 기록한 센서 topic을 재생했다. Camera·TF·odom·mask·detections·GT를 사용했고 기존 fusion/tracking 출력은 재생 입력에서 제외했다.

- GT 334개, TP 329, **FN 5**, FP 60, ID switch 10, IDF1 41.22%.
- 시간 정렬 제외 0, ROI 조회 실패 0.
- 이것은 수정 후 ROS 연결 검증이다. 처리 frame·GT 수가 live capture와 달라 절대 FN끼리의 개선율은 계산하지 않는다.
- 주행 제어를 새로 실행한 closed-loop 안전 검증은 아니다. 기록된 충돌 거리 항목도 이번 수정의 안전 성과로 해석하지 않는다.

근거: [ROS 재생 summary](../trials/fn-association-support/ros-replay-after.json), [association CSV](../trials/fn-association-support/ros-replay-associations.csv), [ID switch 기록](../trials/fn-association-support/ros-replay-id-switches.json).

### 회귀 테스트

기존 C++ 5개·Python 23개, **28개 테스트 통과**. [로그](../trials/fn-association-support/tests.log)

추가 회귀 조건은 전경 6/배경 3의 정확한 2배 경계, 기준 2.01에서 거부, 여러 배경 군집의 합산, 배경 우세 거부, 잘못된 비율 거부다. 기존 복수 후보·단일 후보·point 부족 테스트도 유지했다. 수정 전 binary는 전경 6/배경 3을 status 2로 거부했고 수정 후는 status 0, 깊이 2 m·6 points를 반환했다.

검증 후 평소 실행에 사용하는 `/ws/install`에도 두 패키지를 빌드했다. [설치 빌드 로그](../trials/fn-association-support/production-build.log). 임시 센서 bag·빌드·검증 컨테이너는 정리하고 재현 입력과 결과만 보존한다.

## 같은 결과 재검증

`trials/fn-association-support/frames.json.gz`에 필터링된 같은 입력을 보존했다. 원시 영상·bag 전체 대신 약 2 MiB 입력, 후보 사례, 결과·코드 hash를 유지한다. PCL filtering 자체를 다시 시험하는 fixture는 아니며 이번 association 변경의 전후 비교용이다.

아래 명령은 `tb3-jazzy-lab:local`의 shell 환경, `/ws` mount 기준이다. 컴파일 산출물은 전용 `/tmp` 디렉터리에 만들고 종료 시 정리한다.

```bash
cd /ws
replay_tmp=$(mktemp -d /tmp/mobin-fn-replay-XXXXXX)
trap 'rm -rf -- "$replay_tmp"' EXIT
for variant in before after; do
  flag=
  if [ "$variant" = before ]; then flag=-DFN_BASELINE_ASSOCIATION; fi
  g++ -std=c++17 -O1 -shared -fPIC $flag \
    trials/fn-association-support/replay_bridge.cpp \
    -Icode/cpp/mobile_robot_lab_cpp/include -I/usr/include/eigen3 \
    -o "$replay_tmp/$variant.so"
  PYTHONPATH=/ws/src/python/mobile_robot_lab_python \
    python3 trials/fn-association-support/replay.py "$replay_tmp/$variant.so"
done
```

보존한 스크립트로 다시 실행해 before/after JSON 전체가 최초 비교와 **정확히 일치**함을 확인했다. `baseline-association.hpp`는 사용자 요청의 원인·전후 설명을 위한 수정 전 snapshot이며 production include에는 사용하지 않는다.

## 면접 참고 답변

> 사람 추적 누락을 분석하면서 FN 대부분이 평가 범위 내 track 부재에서 발생한다는 것을 확인했습니다. 원시 센서를 다시 기록해 보니 사람 쪽에 충분한 LiDAR point가 있어도, 소수의 배경 후보가 함께 있다는 이유로 관측 전체를 버리고 있었습니다. 후보 개수 대신 최근접 후보의 point 수가 나머지 후보 합계의 두 배 이상인지 확인하도록 바꿨습니다. 같은 입력과 같은 평가 기준의 C++ 비교에서 FN은 570에서 3으로, FP는 138에서 121로 줄었습니다. 별도 ROS 재생도 확인했지만 FP와 ID switch는 남아 있어 안정적 다인 추적 전체를 해결했다고 주장하지 않았습니다.

“기존 FN 130을 3으로 줄였다”, “정확도 99% 달성”, “충돌 없는 주행 완성”으로 바꾸면 근거와 달라진다. 지원서의 개인 기여 표현은 직접 이해·검토·수행한 범위에 맞춘다.

## 구현 토론: 1차 결정 항목 (미확정)

확정 요구: 앞쪽 비사람 혼입을 포함해 여러 조건에서 검증한다. 최근접·최대 point 수만으로 사람 소속을 확정하지 않는다.

다음 세 결정은 사용자 답변 후 확정한다. 아직 구현 결정이나 ADR이 아니다.

1. 근거가 부족한 frame에서 측정을 보류할 수 있는가? 기존 track의 예측 유지와 새 측정 생성은 구분한다.
2. 기존 RGB·LiDAR·TF와 현재 모델 출력을 우선 활용할 것인가, 추가 모델/센서까지 이번 범위에 포함할 것인가?
3. 검증 대상 사람 범위는 서서 걷는 성인에 한정되는가, 앉기·숙이기·일부 신체만 보이는 상황도 포함하는가? 사람 크기·자세 gate를 먼저 고정하면 후자를 배제할 수 있다.

설계 출발점 제안: 깊이 후보를 사람으로 즉시 확정하지 않고, 각 후보를 지지·반박하는 관측 근거와 구분 불가능 조건을 정의한다. 점 수 비율을 다른 단일 휴리스틱으로 교체하는 것만으로 완료하지 않는다.

### 1차 사용자 답변

- Q2 확정: 기존 RGB·LiDAR·TF와 현재 모델 출력을 우선 활용한다.
- Q3 확정: 서서 걷는 사람과 신체 일부만 보이는 사람을 우선 포함한다. 앉기·숙이기는 이번 필수 범위에 넣지 않는다.
- Q1 미확정: “3D 측정 보류”의 의미를 설명한 뒤 결정한다. 제안 의미는 Camera의 person 검출을 취소하는 것이 아니라, 해당 frame에서 근거 부족한 사람의 3D 위치를 새 측정으로 확정하지 않는 것이다. 기존 track의 예측은 측정과 구분하며 구체적 유지·만료 정책은 아직 결정하지 않았다.

### 식별 가능성에 관한 추가 질문

사용자 질문: 두 후보 중 적어도 하나가 사람임을 보장할 수 있는가?

설명: 일반적인 실제 센서 입력만으로 무조건 보장하지 못한다. Camera의 사람 검출이 맞다는 것과 그 사람의 LiDAR 반사점이 후보 집합에 포함됐다는 것은 별도 조건이다. 제한된 조건에서 후보 집합이 실제 사람 point를 포함한다는 가정을 검증할 수는 있지만, 검출·관측·후보 생성 실패를 배제하지 않은 채 보장이라고 표현하지 않는다. 후보 선택의 설계에는 “어느 후보도 해당 사람의 유효 측정이 아님”을 포함해야 한다. 복수 가설 유지 방식의 채택 여부는 미확정이다.

참고 연구: [Frustum PointNets 공식 구현](https://github.com/charlesq34/frustum-pointnets)은 2D 영역 제안 이후 별도 3D instance segmentation을 수행한다. 이 구조는 추가 판별 근거의 예시이며 정확성 보장이나 본 프로젝트에서의 검증 결과가 아니다.

### 정정: 완전한 visible mask와 합성 반례의 범위

사용자 지적: 정확한 instance segmentation 영역에 LiDAR 점이 있으면 사람 점 아닌가?

- 완전한 visible-person mask라면 사람 앞을 가린 비사람 표면의 pixel은 원칙적으로 mask에서 제외된다. 앞쪽 비사람 60점 반례는 association에 그러한 점을 직접 주입했을 때의 실패를 증명하며, 정확한 mask·TF·동기화에서 그 입력이 실제로 생성된다는 증명은 아니다.
- 2D mask는 Camera pixel의 소속이다. Camera와 LiDAR 시점이 다르면 Camera에 가린 뒤쪽 표면을 LiDAR가 볼 수 있고, 그 점을 투영하면 사람 pixel에 겹칠 수 있다. 정확한 투영만으로 Camera 가시성이 보장되지는 않는다.
- 현재 image_pixel()은 변환·투영·영상 경계 검사를 수행한다. process_pair()는 투영 pixel의 label과 ground filtering으로 instance 입력을 만든다. 별도의 Camera 가시성 검사는 이 경로에 없다.
- 현재 mask는 result.masks.xy polygon을 fillPoly로 rasterize한다. 후속 합성 검증에서 원래 binary mask의 구멍·분리 영역 보존 실패를 확인했다(아래 Mask·투영 경로 검증). 이것이 기존 기록의 배경 혼입에 기여한 비율은 아직 확정하지 않는다.
- 구현 토론의 우선 검토 제안: mask 보존, 시간·좌표 정렬, 시점별 가시성부터 확인한다. 추가 3D 분류기나 복수 가설 추적의 필요성은 그 뒤 판단한다. 이 제안은 아직 사용자 합의 전이다.

## Mask·투영 경로 검증: 수정 전 기록 (2026-09-07)

### 목적과 범위

“person mask에 투영된 LiDAR 점이면 반드시 사람 점인가”를 실제 구현으로 확인했다. **Production 코드는 변경하지 않았다.** 별도 ROS domain 86에서 현재 설치된 C++ 노드를 실행하고, 합성 CameraInfo·mask·Detection2D·PointCloud2·static TF를 발행했다. 원본 binary mask는 정답으로 구성했으며 YOLO 모델 추론·Gazebo·실제 영상의 segmentation 정확도는 이번 검증에 포함하지 않는다.

검증 결과는 [result.json](../trials/mask-projection-validation/result.json), 전체 출력은 [run.log](../trials/mask-projection-validation/run.log)에 보존했다. Ultralytics 8.3.220, OpenCV 4.11.0, NumPy 1.26.4를 사용했다. 실행 binary와 관련 source의 SHA-256도 결과에 포함했다.

### 1. Mask 변환: 실제 구멍과 분리 영역 보존 실패

`code/python/mobile_robot_lab_python/mobile_robot_lab_python/yolo_segmentation_detector_node.py` | [YoloSegmentationDetectorNode.on_image()](../code/python/mobile_robot_lab_python/mobile_robot_lab_python/yolo_segmentation_detector_node.py#L121)는 binary mask 대신 polygon을 사용한다.

```python
# code/python/mobile_robot_lab_python/mobile_robot_lab_python/yolo_segmentation_detector_node.py | YoloSegmentationDetectorNode.on_image()
        polygons = list(result.masks.xy) if result.masks is not None else []
```

설치된 Ultralytics의 `masks2segments()` 원문은 result.json에 보존했다. 내부 `cv2.findContours(x, cv2.RETR_EXTERNAL, ...)`는 외곽 contour를 추출하고, 분리된 contour는 `merge_multi_segment()`로 병합한다.

`code/python/mobile_robot_lab_python/mobile_robot_lab_python/yolo_segmentation_detector_node.py` | [build_instance_label_map()](../code/python/mobile_robot_lab_python/mobile_robot_lab_python/yolo_segmentation_detector_node.py#L44)는 그 polygon을 채운다.

```python
# code/python/mobile_robot_lab_python/mobile_robot_lab_python/yolo_segmentation_detector_node.py | build_instance_label_map()
        cv2.fillPoly(labels, [np.rint(points).astype(np.int32)], index)
```

| 합성 원본 mask | 원본 사람 pixel | 추가된 사람 pixel | 삭제된 사람 pixel |
|---|---:|---:|---:|
| 사각형 | 861 | 0 | 0 |
| 내부에 7×7 빈 영역 | 812 | **49** | 0 |
| 분리된 두 영역 | 310 | **10** | 0 |

원본과 변환 mask의 해상도는 모두 160×120이다. 따라서 이번 차이는 resize나 추론 오차 없이 발생했다. **원본 segmentation이 정확하더라도 현재 변환이 비사람 영역을 추가할 수 있음을 확인했다.** 49와 10은 이 합성 입력의 값이며 실영상 평균 오차가 아니다.

### 2. 실제 C++ ROS 노드 출력

모든 입력의 timestamp를 같게 하고 정확한 static TF를 제공했다. Camera optical frame의 z를 깊이[m]로 사용하며 사람 표면은 2m다. 합성 LiDAR 원점은 Camera optical x 방향으로 0.8m 떨어져 있다. 이는 시점 차이를 재현하기 위한 검증 값이며 실제 로봇 baseline 측정값이 아니다.

| 입력 조건 | 실제 출력 깊이 | 의미적 판정 |
|---|---:|---|
| 완전한 mask + 사람 점만 9개 | 2m | 정상 |
| 정확한 빈 영역 mask + 사람 9점·앞쪽 물체 60점 | 2m | 정상: 앞쪽 물체 제외 |
| 동일 point cloud + polygon 경유로 빈 영역이 채워진 mask | **1m** | **실패: 앞쪽 물체를 사람으로 발행** |
| 완전한 mask + 사람 반사점 없이 뒤쪽 벽 점 6개 | **5m** | **실패: 벽을 사람으로 발행** |
| 완전한 mask + 사람 9점·뒤쪽 벽 3점 | 2m | 이 입력에서는 전경 지지 규칙이 사람을 선택 |

표의 실패는 실제 `/fusion/detections_3d` 출력이다. Python에서 association을 재구현한 결과가 아니다. 테스트 assertion은 **이 관측 동작이 재현됨**을 확인한다. 스크립트 종료 코드 0을 알고리즘의 모든 상황 통과로 해석하지 않는다.

`code/cpp/mobile_robot_lab_cpp/src/lidar_bbox_association_node.cpp` | [image_pixel()](../code/cpp/mobile_robot_lab_cpp/src/lidar_bbox_association_node.cpp#L213)는 TF·pinhole 투영·영상 경계를 검사한다. [process_pair()](../code/cpp/mobile_robot_lab_cpp/src/lidar_bbox_association_node.cpp#L344)의 실제 mask gate는 다음과 같다.

```cpp
// code/cpp/mobile_robot_lab_cpp/src/lidar_bbox_association_node.cpp | process_pair()
        if (pixel && labels->image.at<std::uint8_t>(pixel->y, pixel->x) == *label) {
          ++points_before_ground;
          if (!plane || point_plane_distance(points_in_ground_frame[index], *plane) >
            ground_config_.inlier_distance)
          {
            instance_points.push_back(point);
          }
        }
```

이번 cloud는 지면 추정을 위한 50점 이상의 inlier가 없으므로 기존의 plane 부재 fallback 경로를 사용했다. 지면 제거의 성능을 시험한 결과는 아니다. 물체와 사람 검증점은 ground frame에서 지면 위에 있다.

벽 사례는 기하학적으로도 확인했다. Camera에서 벽 점을 향하는 선분은 z=2m에서 사람 영역을 통과한다. 반면 LiDAR에서 벽 점을 향하는 선분은 같은 깊이에서 x=0.468~0.492m를 지나, 사람의 가로 범위 ±0.2m를 벗어난다. 즉 **Camera에서 가린 벽을 LiDAR는 관측할 수 있다.** 사람 반사점이 없는 sparse cloud를 입력했으므로 후보는 벽 하나이고 전경 비율 검사는 이를 거부하지 않는다. 빛·LiDAR의 실제 센서 시뮬레이션 대신 선분 교차로 가시성을 확인한 합성 사례다.

### 판단과 다음 검토

1. 사용자 주장 중 “정확한 visible mask가 앞쪽 비사람 물체를 제외한다”는 조건은 정상 대조군에서 확인했다.
2. 현재 mask 변환은 그 전제를 깨뜨릴 수 있다. 우선 수정 후보는 binary mask의 구멍·분리 영역을 보존하는 것이다. 원본 해상도 복원과 instance 겹침 처리도 검증해야 하므로 이번 확인만으로 구현을 교체하지 않았다.
3. 완전한 mask에서도 시점별 가시성 차이는 남는다. Mask 변환 수정만으로 모든 association 문제가 해결되지는 않는다.
4. 기존 60초 기록에는 원본 binary mask·원시 영상·raw cloud가 현재 남아 있지 않다. 저장된 필터링 point만으로 두 원인의 빈도나 기존 FN 기여율을 역산하지 않는다.

### 재현

`trials/mask-projection-validation/probe.py` | 기존 `roundtrip()`은 당시 Ultralytics `Masks.xy`와 프로젝트 polygon 함수를 호출했다. 현재 재현 스크립트에서는 [polygon_baseline()](../trials/mask-projection-validation/probe.py#L37)이 이전 경로를 보존하고, 현재 binary 경로와 비교한다. [main()](../trials/mask-projection-validation/probe.py#L51)은 3개 mask 비교와 5개 ROS 출력 사례를 검증한다.

Repository root에서 실행한다. 기존 ROS 설치 공간과 `tb3-jazzy-lab:local` 이미지가 필요하다. Domain 86은 다른 실행이 사용하지 않는 상태여야 한다.

```bash
docker compose -f docker/compose.yaml run --rm --no-deps \
  --name mobin-mask-validation --entrypoint /bin/bash \
  -e ROS_DOMAIN_ID=86 -e ROS_LOCALHOST_ONLY=1 \
  -e ROS_LOG_DIR=/tmp/ros-log -e YOLO_CONFIG_DIR=/tmp/yolo-config \
  -e PYTHONDONTWRITEBYTECODE=1 shell -lc \
  'mkdir -p /tmp/yolo-config; source /opt/ros/jazzy/setup.bash; source /ws/install/setup.bash; python3 /ws/trials/mask-projection-validation/probe.py'
```

임시 node 로그는 스크립트 종료 시 삭제하고, ROS·Ultralytics 임시 파일은 `--rm` 컨테이너와 함께 삭제했다. Host에는 검증 source·의도한 결과 JSON·로그만 보존했다. Production 함수 변경은 없다.

## Binary mask 직접 사용 수정 (2026-09-07)

사용자 요청에 따라 polygon 경유를 제거했다. 원본 영상 크기로 복원된 YOLO binary mask를 그대로 label map에 사용한다. 여기서 보존하는 원본은 **`retina_masks=True` 추론이 반환한 binary mask**이며, 이전 저해상도 mask를 polygon으로 변환한 결과와 동일하다는 뜻은 아니다.

| 파일 위치 | 함수 또는 설정 | 변경 요약 |
|---|---|---|
| `code/python/mobile_robot_lab_python/mobile_robot_lab_python/yolo_segmentation_detector_node.py` | [build_instance_label_map()](../code/python/mobile_robot_lab_python/mobile_robot_lab_python/yolo_segmentation_detector_node.py#L44) | 이전: 외곽 polygon 재채움.<br>변경: binary mask의 참 pixel에 label 부여, 해상도 불일치 거부.<br>효과: 구멍·분리 영역 보존. 겹침은 기존처럼 뒤 label 우선. |
| 같은 파일 | [YoloSegmentationDetectorNode.on_image()](../code/python/mobile_robot_lab_python/mobile_robot_lab_python/yolo_segmentation_detector_node.py#L121) | 이전: `masks.xy` 추출.<br>변경: `retina_masks=True`, `masks.data` 사용.<br>효과: letterbox가 제거된 원본 영상 해상도의 mask 전달. |
| `trials/mask-projection-validation/probe.py` | [polygon_baseline()](../trials/mask-projection-validation/probe.py#L37), [main()](../trials/mask-projection-validation/probe.py#L51) | 이전: `roundtrip()`이 production polygon 함수를 호출.<br>변경: `roundtrip()` 삭제, 비교용 polygon 경로를 분리하고 binary 경로 추가.<br>효과: 이전 오채택과 현재 정상 채택을 동일 입력으로 비교. |
| `trials/mask-projection-validation/inference_check.py` | [main()](../trials/mask-projection-validation/inference_check.py#L23) | 이전: 실제 추론을 통한 직접 전달 검사 없음.<br>변경: 실제 모델 추론을 callback에 연결하고 pixel·header 비교.<br>효과: 홀수 영상 크기와 검출 없는 입력의 계약 확인. |

```python
# code/python/mobile_robot_lab_python/mobile_robot_lab_python/yolo_segmentation_detector_node.py | build_instance_label_map()
        binary = np.asarray(mask, dtype=bool)
        if binary.shape != (height, width):
            raise ValueError('instance mask must match original image dimensions')
        # ... 생략: 설명 주석
        labels[binary] = index
```

[설치된 Ultralytics와 공식 API](https://docs.ultralytics.com/reference/models/yolo/segment/predict/)에서 `retina_masks=True`가 원본 영상 크기를 사용하는 `process_mask_native()` 경로임을 확인했다. 임의의 nearest resize나 polygon 재구성은 추가하지 않았다. 겹친 instance는 한 pixel에 두 ID를 저장할 수 없으므로 기존 label 우선순위를 유지한다. 이 때문에 겹침 부분까지 각 instance의 독립 mask를 모두 보존한다고 표현하지 않는다.

### 수정 후 검증

| 검증 | 결과 |
|---|---|
| 사각형·구멍·분리 mask 비교 | 추가·삭제 pixel 모두 **0** |
| 동일 69점 입력: 앞쪽 물체 60점 + 사람 9점 | 이전 polygon 경로: 1m 물체 선택 / 현재 binary 경로: **2m 사람 선택** |
| 정확한 mask + 사람 반사점 없이 가린 벽 6점 | **5m 벽 오채택 유지**, 가시성 문제 미해결 |
| 실제 YOLO11s-seg CPU 추론: 1001×797 영상 | 사람 4 instance, 모델 mask 대비 label 차이 **0 pixel** |
| 실제 YOLO 추론: 479×631 빈 영상 | 0 instance, 배경 label 정상 |
| Python 전체 회귀 테스트 | **27 passed** |
| ROS 설치 공간 | Python 패키지 빌드 완료 |

[수정 후 ROS·mask 결과](../trials/mask-projection-validation/after.json), [실제 추론 결과](../trials/mask-projection-validation/inference.json), [테스트 로그](../trials/mask-projection-validation/tests.log), [빌드 로그](../trials/mask-projection-validation/build.log).

실제 추론 입력은 설치된 Ultralytics의 `bus.jpg`를 1001×797로 잘라 사용했다. 외부 이미지를 다운로드하거나 기존 사용자 자료를 수정하지 않았다. 합성 ROS 검증은 이전과 동일한 C++ binary를 사용한다. Tracker·전경 지지 조건·평가기는 이번 수정에서 바꾸지 않았다. 이번 결과로 기존 FN 130이나 570의 새로운 개선율을 계산하지 않는다. 추론 지연·GPU 메모리 영향은 benchmark하지 않았다.

재현은 위 Docker shell 환경에서 `python3 /ws/trials/mask-projection-validation/probe.py`, `python3 /ws/trials/mask-projection-validation/inference_check.py`, `python3 -m pytest /ws/src/python/mobile_robot_lab_python/test -q -p no:cacheprovider`를 실행한다. `probe.py`는 이제 수정 전·후 경로 6개 ROS 사례를 비교한다.

### 남은 가시성 문제

사용자는 「확실히 가린 점만 제거」 정책을 선택했다. 동일 pixel 비교를 구현했으며 이웃 pixel로 확장하지 않는다. 이는 관측된 깊이에 따른 제한적 제외 판정이며, pixel 안의 시선 차이·센서 오차까지 제거한 물리적 가림 증명은 아니다. 앞쪽 반사점이 없는 경우의 최종 측정 보류 정책은 미확정이며 기존 association 판정을 유지한다.

### 레이저 반사와 가상 투영의 구분

사용자 질문에 따라 기존 단면 설명을 회전·센서 시점 전환·왕복 재생이 가능한 3D 개념도로 갱신했다. 센서 간격과 재생 시간은 설명을 위해 과장했으며 실제 로봇 측정값이 아니다.

- 실제 광 경로: LiDAR에서 출발해 사람 옆의 빈 경로로 벽에 도달한다. 벽에서 반사·산란된 빛 중 일부가 LiDAR 수신기로 돌아온다. 사람에게 다시 반사된다는 가정이나 다중 반사를 사용하지 않는다.
- Camera 가림: Camera 원점에서 같은 벽 점을 향하는 시선은 사람 표면에 막힌다. LiDAR 원점에서 해당 벽 점을 향하는 선분은 사람 표면에 막히지 않는다.
- 영상 투영: 측정된 벽 점의 좌표를 Camera pixel로 계산하는 연산이다. 빛이 벽에서 Camera나 사람을 향해 이동하는 경로를 뜻하지 않는다.

3D 사람 geometry와 두 선분의 교차를 검사해 Camera 선분은 차단되고 LiDAR 선분은 통과함을 확인했다. 회전 버튼·확대·초기화·두 sensor 시점·레이어 표시·왕복 재생과 reduced-motion 분기를 확인했다. [LiDAR 왕복 측정 원리: Ouster](https://ouster.com/insights/what-is-lidar).

## 동일 pixel 가림 제외 구현 (2026-09-07)

### 원인 → 수정 → 확인

**원인:** Binary mask를 정확히 보존해도 Camera와 LiDAR의 원점이 다르면, Camera에서 가린 벽 점이 LiDAR에 관측되어 사람 mask에 투영될 수 있다. 이전 association은 앞 표면과의 pixel별 깊이 관계를 검사하지 않았다. 사람 3점과 뒤쪽 벽 6점이 함께 남으면 전경 지지 부족으로 관측 전체를 거부했다.

**수정:** 같은 cloud의 모든 유효 점을 Camera optical frame으로 변환하고, 동일 반올림 pixel의 최소 깊이보다 허용 오차 초과 뒤쪽인 점만 instance 후보에서 제외한다. 사람 label·지면 필터 전의 전체 점으로 깊이 근거를 만든다. 이웃 pixel로 가림 영역을 넓히거나 앞 표면 깊이를 보간하지 않는다.

`code/cpp/mobile_robot_lab_cpp/include/mobile_robot_lab_cpp/camera_lidar_fusion.hpp` | [camera_occluded_points()](../code/cpp/mobile_robot_lab_cpp/include/mobile_robot_lab_cpp/camera_lidar_fusion.hpp#L102)의 판정:

$$z_i-\min_{j:q_j=q_i}z_j>\epsilon$$

$q_i$는 기존 mask 조회와 같은 반올림 pixel, $z_i$는 Camera optical frame의 전방 깊이[m], $\epsilon$은 `camera_occlusion_depth_tolerance`[m]다. 같은 cloud를 사용하고 TF 조회 시각은 cloud timestamp다. 기본 0.05m는 초기값이며 실측 오차 상한이 아니다. 허용 오차가 커질수록 더 많은 점을 보존한다. 동일 pixel 안에서도 실제 시선은 다를 수 있고, 노이즈·calibration·Camera 시간차는 남으므로 물리적 가림의 절대 보증은 아니다.

```cpp
// code/cpp/mobile_robot_lab_cpp/include/mobile_robot_lab_cpp/camera_lidar_fusion.hpp | camera_occluded_points()
    // 이웃 pixel로 전파하지 않으며 허용 오차 이내의 깊이 차이는 보존한다.
    occluded[index] = offset != invalid_pixel && depth - nearest[offset] > depth_tolerance;
```

| 파일 위치 | 함수 또는 설정 | 변경 요약 |
|---|---|---|
| `code/cpp/mobile_robot_lab_cpp/include/mobile_robot_lab_cpp/camera_lidar_fusion.hpp` | [camera_occluded_points()](../code/cpp/mobile_robot_lab_cpp/include/mobile_robot_lab_cpp/camera_lidar_fusion.hpp#L102) | 이전: pixel별 깊이 검사 없음.<br>변경: 동일 pixel 최소 Camera 깊이와 비교.<br>효과: 허용 오차 초과 뒤쪽 점만 제외 표시. |
| `code/cpp/mobile_robot_lab_cpp/src/lidar_bbox_association_node.cpp` | [LidarBboxAssociationNode()](../code/cpp/mobile_robot_lab_cpp/src/lidar_bbox_association_node.cpp#L106) | 이전: 가림 깊이 오차 설정 없음.<br>변경: 기본 0.05m parameter 선언, 유한한 값 >=0 검사.<br>효과: 센서 조건에 맞는 허용 오차 설정 가능. |
| 같은 파일 | [process_pair()](../code/cpp/mobile_robot_lab_cpp/src/lidar_bbox_association_node.cpp#L344) | 이전: mask·지면 검사 후 바로 후보 구성.<br>변경: 전체 cloud로 가림 제외 mask를 계산하고 instance 후보에서 제외.<br>효과: 뒤쪽 점 제거, 가림에 의한 점 부족을 지면 거부로 오집계하지 않음. |
| 같은 파일 | [publish_association_diagnostics()](../code/cpp/mobile_robot_lab_cpp/src/lidar_bbox_association_node.cpp#L317) | 이전: association 결과 건수만 제공.<br>변경: `camera_occluded_points` 누적 점 수 추가.<br>효과: 제외된 입력 점을 별도로 확인. Instance 거부 건수와 합산 금지. |
| `code/python/mobile_robot_lab_python/launch/yolo_lidar_fusion.launch.py` | [generate_launch_description()](../code/python/mobile_robot_lab_python/launch/yolo_lidar_fusion.launch.py#L19) | 이전: 가림 parameter 전달 없음.<br>변경: 선택 launch 인자 `camera_occlusion_depth_tolerance` 전달.<br>효과: 기본 0.05m, 유한한 실수 >=0으로 조정. |

기존 전경 지지 기반 후보 선택·tracker·평가기는 변경하지 않았다. `/fusion/non_ground_points`는 가림 제외 전 지면 분리 결과다. 가림 후 남은 점이 `min_points=3`보다 적으면 기존 `no_points` 경로를 사용한다. 가림 필터를 통과했다는 사실이 사람 소속을 증명하지 않는다.

### 검증

| 합성 입력 또는 검사 | 관측 결과 |
|---|---|
| 사람 3점(2m) + 동일 pixel 벽 6점(5m) | 변경 전 ROS: 관측 없음. 변경 후: **사람 3점·2m**, 벽 제외 |
| 가까운 점이 이웃 pixel에만 존재 | 5m의 부분 관측 사람 3점 보존 |
| 동일 pixel 깊이 2.00·2.03·2.04m | 3점 모두 보존, 대표 깊이 약 2.03m |
| 동일 pixel 앞쪽 1점 + 뒤쪽 3점 | 뒤쪽 제외, 남은 1점은 부족해 관측 없음. `ground_rejection=0` 확인 |
| 사람 반사점 없이 벽 6점만 관측 | **5m 벽 오채택 유지**, 미해결 사례 |
| 기존 mask 구멍·분리 영역 및 ROS 대조군 | 보존 확인; polygon 비교 경로의 1m 오채택도 재현 |
| C++ 회귀 | Debug 빌드에서 5/5 통과. 순서 독립성·Camera TF·허용 오차 경계·비유한 입력·이웃 pixel 검증 |
| Python 회귀 | 27 passed |

[변경 전 실패 로그](../trials/mask-projection-validation/visibility-before-run.log), [변경 후 10개 ROS 사례](../trials/mask-projection-validation/visibility.json), [ROS 실행 로그](../trials/mask-projection-validation/visibility-run.log), [테스트 로그](../trials/mask-projection-validation/visibility-tests.log), [빌드 로그](../trials/mask-projection-validation/visibility-build.log).

재현은 앞의 Docker 환경에서 `python3 /ws/trials/mask-projection-validation/probe.py`를 실행한다. 현재 스크립트는 ROS 출력 깊이와 associated point 수, diagnostics를 검증한다. 메시지 재전송이 발생할 수 있어 누적 diagnostics를 고유 사례 수로 해석하지 않는다. C++는 `ctest --test-dir /ws/build/mobile_robot_lab_cpp --output-on-failure`, Python은 앞의 pytest 명령을 사용한다. C++ assert가 활성화된 Debug 빌드로 검사했다. 빌드에는 PCL/FLANN의 CMake policy 경고가 있었고 컴파일 실패는 없었다.

이 검증은 합성 mask·점과 실제 ROS C++ node를 사용했다. 실제 보행·전신 가림·Gazebo 전체 tracking 재평가나 속도 benchmark가 아니다. 기존 FN 130 또는 570에 대한 이번 변경의 개선율은 계산하지 않는다.

### 면접 참고: “벽만 들어오는 오채택”

실제 사람은 2m 앞에 있지만 카메라 시점과 떨어진 LiDAR의 희소 광선이 사람을 비껴가 벽 5m만 측정할 수 있다. 그 벽 점이 카메라 영상의 사람 mask에 투영된다. **같은 pixel에 사람의 2m 반사점이 없으면**, 가림 필터가 비교할 앞쪽 깊이가 없다. 벽 5m는 통과하고 후단 association이 이를 사람 위치로 발행한다. “벽만”은 영상이 아니라 해당 영역의 LiDAR 입력에 벽 점만 있다는 의미다. 레이저가 사람을 관통하거나 벽에서 사람으로 재반사된다는 뜻이 아니다.

설명 문장: “Mask 재구성 손실과 센서 시점 차이에 따른 혼입을 분리했습니다. Binary mask를 직접 보존하고, 앞쪽 깊이가 함께 관측된 동일 pixel에서 뒤쪽 점을 제외해 합성 ROS 실패 사례를 해결했습니다. 앞쪽 반사점이 없는 입력에는 이 방법의 근거가 없으므로, 모든 상황의 사람 소속 판정이 해결됐다고 주장하지 않았습니다.”

검증 전용 `mobin-visibility` 컨테이너와 내부 임시 로그를 정리했다. Host의 임시 C++ 실행 파일과 이번 CTest 임시 로그를 제거했고, 의도한 검증 source·결과·[source SHA-256](../trials/mask-projection-validation/visibility-source-sha256.json)은 보존했다. 설치 빌드 결과는 유지했다. 수정한 PATCH-07은 497줄이며, 기존 PATCH-02의 974줄은 이번 변경 범위 밖이다.
