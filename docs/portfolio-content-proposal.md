# RoboDine 확장(Mobin): 현재 로직과 포트폴리오 수록안

- 갱신일: 2026-09-07
- 선별 기준: [senior_feedback.txt](/home/swlinux/Desktop/workspace/ai-job-search/feedback/senior_feedback.txt)
- 개인 기여: **Mobin 전체 설계·구현 단독 수행** — 사용자 확인. RoboDine 본 프로젝트의 팀 활동과 구분
- 코드 기준: `feature/avoidance`, HEAD `73f81c4`와 현재 미커밋 구현
- 강조 직무: **Perception·Tracking**
- 상태: **실제 LaTeX 18쪽 반영 완료, PDF 컴파일·시각 검증 완료**
- 결론: Camera–LiDAR 사람 관측과 추적을 구현하고, 관측 거부·mask 손실·시점별 가림을 분리해 수정·검증했다. **모든 상황의 사람 소속 판정과 안정적인 다인 추적을 완성한 상태는 아니다.**

### Why?

사람이 카메라에 검출돼도 LiDAR 관측과 연결되지 않으면 추적 입력이 사라진다. 반대로 영상의 사람 영역에 투영됐다는 이유만으로 점을 채택하면 물체나 벽을 사람 위치로 오인할 수 있다. 현재 작업의 강점은 이 두 실패를 코드·합성 ROS 사례·고정 입력 평가로 나눠 확인한 과정이다.

실제 수록할 짧은 원고는 [시니어 피드백 반영 수록본](portfolio-selected-content.md)에 정리했다. 아래 전체 함수·설정 목록은 면접 참고용이며 슬라이드에 모두 넣지 않는다.

이 문서는 기존 [원인·수정·재현 보고서](person_tracking_false_negatives.md)의 이력을 현재 상태로 요약하고, 실제 포트폴리오에 옮길 문장과 증거 범위를 제시한다. 로직 요약 단계에서는 프로그램을 변경하지 않았다. 후속 사용자 요청으로 최신 Gazebo 실행과 RViz 캡처를 추가했으며 결과는 별도 실행으로 보존한다.

## 1. 현재 반영된 로직

[현재 처리 흐름도 — 16:9 검토용](assets/perception_tracking_flow.html). 센서 시점 차이는 [기존 3D 설명](/home/swlinux/.codex/visualizations/2026/09/07/01a07a89-9d7f-78c3-b524-4f7d4d63f9ac/mask-and-visibility.html)을 사용한다. 3D 설명은 개념도이며 실제 센서 배치의 측정값이 아니다.

| 단계 | 현재 동작 | 판단의 의미와 한계 |
|---|---|---|
| 1. 사람 segmentation | YOLO11-seg 추론에서 `retina_masks=True`, 원본 영상 크기의 `masks.data` 사용 | 외부 학습 모델 활용. 자체 모델 학습 성과가 아님 |
| 2. Instance Label Map | Binary mask의 참 pixel에 `mono8` label 부여. 구멍·분리 영역 보존, 해상도 불일치 거부 | 0=배경, 1..N=frame-local instance. Track ID가 아님. 겹친 pixel은 뒤 label 우선 |
| 3. 시각·좌표 검증 | Label–Detection timestamp는 정확히 일치. Detection–cloud 차이는 기본 50ms 이하. Cloud 시각의 TF 사용 | 도착 시각이 아니라 메시지 관측 시각 기준. 50ms 이내라도 움직임에 따른 오차는 가능 |
| 4. Camera 투영·가림 제외 | 전체 유효 cloud에서 같은 반올림 pixel의 최소 Camera z를 구하고, 기본 5cm 초과 뒤쪽 점만 instance 후보에서 제외 | **이웃 pixel로 확장하지 않음.** 허용 오차 이내 점 보존. 최소 깊이 점을 사람으로 확정하는 단계가 아님 |
| 5. 제약을 둔 지면 제거 | Camera FoV의 점으로 PCL RANSAC. `base_footprint` 기준 기울기 ≤20°, 원점 거리 ≤0.15m, inlier ≥50. Plane에서 0.05m 이내 점 제외 | 유효 plane이 없으면 지면 판정으로는 제거하지 않음. 가림 깊이 근거는 지면 제거 전 전체 점에서 확보 |
| 6. Instance별 깊이 후보 | Label 일치·가림 제외·지면 제외를 통과한 점을 Camera z로 정렬. 이웃 깊이 간격 >0.35m에서 분리, 후보당 최소 3점 | 전신 형태가 없어도 점 후보를 만들지만 신체 일부·보행의 모든 조건을 검증한 것은 아님 |
| 7. 후보 선택·3D 관측 | 최근접 유효 후보 점 수가 다른 유효 후보 합계의 기본 2배 이상이면 채택. 축별 중앙값을 대표 위치로 삼아 `odom`으로 변환 | 다른 유효 후보가 없으면 그대로 채택. 따라서 **벽 점만 들어온 경우에도 오채택 가능** |
| 8. 지속 추적 | `odom`의 `(x,y,vx,vy)`를 Constant Velocity Kalman으로 예측·보정. Euclidean·Mahalanobis gate와 Hungarian 일대일 할당 | Geometry 기반 연관. 외형 특징이나 사람 재식별 모델은 없음 |
| 9. Track 수명 | 최근 최대 5회 갱신에서 hit ≥3이면 confirmed, 미연결은 lost. 마지막 측정에서 >0.5초 경과하면 삭제. 시간 역행 시 reset | 업데이트는 Detection3D 메시지를 받을 때 실행. 별도 타이머로 무입력 기간의 track을 갱신하는 구조가 아님 |
| 10. 관측·평가 기록 | Association 결과별 diagnostics, 공통 Camera–LiDAR ROI 내 GT/track 평가, FN·FP·ID switch·IDF1 및 실행 증거 보존 | GT는 평가에만 사용. ROI는 기하학적 시야이며 가림의 정답 판정이 아님 |

가림 필터의 5cm는 **조정 가능한 초기값**이며 센서 오차를 실측해 보증한 상한이 아니다. 같은 pixel도 조금 다른 시선을 포함하므로 「가림 제외」를 물리적 가림의 절대 증명으로 표현하지 않는다. `/fusion/non_ground_points`는 가림 제외 전 지면 분리 결과다.

### 함수별 구현 위치

| 파일 위치 | 함수 | 역할 |
|---|---|---|
| `code/python/mobile_robot_lab_python/mobile_robot_lab_python/yolo_segmentation_detector_node.py` | [YoloSegmentationDetectorNode.on_image()](../code/python/mobile_robot_lab_python/mobile_robot_lab_python/yolo_segmentation_detector_node.py#L121), [build_instance_label_map()](../code/python/mobile_robot_lab_python/mobile_robot_lab_python/yolo_segmentation_detector_node.py#L44) | 입력: Camera image와 추론 mask.<br>처리: 원본 크기 binary mask를 label로 변환.<br>결과: 같은 header의 Detection2D와 Instance Label Map. |
| `code/cpp/mobile_robot_lab_cpp/src/lidar_bbox_association_node.cpp` | [LidarBboxAssociationNode::process_pair()](../code/cpp/mobile_robot_lab_cpp/src/lidar_bbox_association_node.cpp#L344) | 입력: 동기화된 label·detection·cloud, CameraInfo·TF.<br>처리: 시간·해상도 검증, 가림·지면·instance 조건 검사.<br>결과: `odom` Detection3D와 associated points. |
| `code/cpp/mobile_robot_lab_cpp/include/mobile_robot_lab_cpp/camera_lidar_fusion.hpp` | [camera_occluded_points()](../code/cpp/mobile_robot_lab_cpp/include/mobile_robot_lab_cpp/camera_lidar_fusion.hpp#L102) | 입력: 전체 cloud와 Camera 변환·내부 파라미터.<br>처리: 동일 pixel 최소 z와 허용 오차 비교.<br>결과: 뒤쪽 점의 제외 mask. |
| `code/cpp/mobile_robot_lab_cpp/include/mobile_robot_lab_cpp/ground_plane_filter.hpp` | [estimate_ground_plane()](../code/cpp/mobile_robot_lab_cpp/include/mobile_robot_lab_cpp/ground_plane_filter.hpp#L51) | 입력: Ground Reference Frame의 점과 FoV 후보.<br>처리: 제약 RANSAC·normal·원점거리·inlier 검사.<br>결과: 유효 plane 또는 `nullopt`. |
| `code/cpp/mobile_robot_lab_cpp/include/mobile_robot_lab_cpp/camera_lidar_fusion.hpp` | [analyze_instance_points()](../code/cpp/mobile_robot_lab_cpp/include/mobile_robot_lab_cpp/camera_lidar_fusion.hpp#L158) | 입력: 필터를 통과한 instance 점.<br>처리: 깊이 후보 분리·전경 지지 비교·축별 중앙값.<br>결과: 선택 관측 또는 정보 부족·복수 후보 거부. |
| `code/cpp/mobile_robot_lab_cpp/include/mobile_robot_lab_cpp/constant_velocity_tracker.hpp` | [ConstantVelocityTracker::update()](../code/cpp/mobile_robot_lab_cpp/include/mobile_robot_lab_cpp/constant_velocity_tracker.hpp#L157), [predict()](../code/cpp/mobile_robot_lab_cpp/include/mobile_robot_lab_cpp/constant_velocity_tracker.hpp#L246), [assign()](../code/cpp/mobile_robot_lab_cpp/include/mobile_robot_lab_cpp/constant_velocity_tracker.hpp#L274), [update_measurement()](../code/cpp/mobile_robot_lab_cpp/include/mobile_robot_lab_cpp/constant_velocity_tracker.hpp#L325) | 입력: timestamp와 odom 2D 관측.<br>처리: 예측·gate·Hungarian·Kalman 보정·수명 관리.<br>결과: ID·위치·속도·covariance·track 상태. |
| `code/cpp/mobile_robot_lab_cpp/src/person_tracker_node.cpp` | [PersonTrackerNode::on_detections()](../code/cpp/mobile_robot_lab_cpp/src/person_tracker_node.cpp#L81) | 입력: Detection3DArray.<br>처리: 유한한 xy 추출, tracker 갱신.<br>결과: TrackedPersonArray와 RViz marker. |
| `code/python/mobile_robot_lab_python/mobile_robot_lab_python/tracking_evaluator_node.py` | [on_tracks()](../code/python/mobile_robot_lab_python/mobile_robot_lab_python/tracking_evaluator_node.py#L255) | 입력: track 시각과 GT·odom·센서 시야.<br>처리: 시간 정렬·공통 ROI·거리 기반 평가 대응.<br>결과: 누적 평가와 frame별 대응 기록. |
| `code/python/mobile_robot_lab_python/mobile_robot_lab_python/tracking_metrics.py` | [TrackingMetrics.summary()](../code/python/mobile_robot_lab_python/mobile_robot_lab_python/tracking_metrics.py#L76) | 입력: 누적 검출·ID 대응.<br>처리: TP·FP·FN·IDF1 계산.<br>결과: 자체 평가기 지표. |
| `code/python/mobile_robot_lab_python/mobile_robot_lab_python/trial_evidence_recorder.py` | [on_fusion_diagnostics()](../code/python/mobile_robot_lab_python/mobile_robot_lab_python/trial_evidence_recorder.py#L164), [save()](../code/python/mobile_robot_lab_python/mobile_robot_lab_python/trial_evidence_recorder.py#L359) | 입력: fusion diagnostics와 실행 중 수집 자료.<br>처리: 누적 값·event·그림 보존.<br>결과: 실행 증거 파일. |

설정은 `code/python/mobile_robot_lab_python/launch/yolo_lidar_fusion.launch.py` | [generate_launch_description()](../code/python/mobile_robot_lab_python/launch/yolo_lidar_fusion.launch.py#L19)에서 전달한다. `camera_occlusion_depth_tolerance`는 유한한 실수 ≥0[m], `foreground_support_ratio`는 유한한 실수 >1이다. `code/cpp/mobile_robot_lab_cpp/src/lidar_bbox_association_node.cpp` | [publish_association_diagnostics()](../code/cpp/mobile_robot_lab_cpp/src/lidar_bbox_association_node.cpp#L317)의 `camera_occluded_points`는 누적 **점 수**로, instance 성공·거부 건수와 합산하지 않는다.

### 핵심 판단의 최소 원문

위 함수들의 상세 원문과 변경 전 경로는 [개발 보고서](person_tracking_false_negatives.md)에 있다. 현재 선택 규칙을 설명할 때는 다음 원문을 기준으로 한다.

```python
# code/python/mobile_robot_lab_python/mobile_robot_lab_python/yolo_segmentation_detector_node.py | build_instance_label_map()
        binary = np.asarray(mask, dtype=bool)
        if binary.shape != (height, width):
            raise ValueError('instance mask must match original image dimensions')
        # ... 생략: 설명 주석
        labels[binary] = index
```

```cpp
// code/cpp/mobile_robot_lab_cpp/include/mobile_robot_lab_cpp/camera_lidar_fusion.hpp | camera_occluded_points()
    // 최소 깊이가 있는 같은 pixel에서만 뒤쪽 제외 여부를 결정한다.
    occluded[index] = offset != invalid_pixel && depth - nearest[offset] > depth_tolerance;
```

```cpp
// code/cpp/mobile_robot_lab_cpp/include/mobile_robot_lab_cpp/camera_lidar_fusion.hpp | analyze_instance_points()
  // 최근접 후보가 나머지 유효 후보 전체보다 충분한 점 지지를 갖는지 검사한다.
  if (static_cast<double>(selected_end - selected_begin) <
    foreground_support_ratio * static_cast<double>(background_count))
  {
    return {AssociationStatus::MULTIPLE_DEPTH_CANDIDATES, std::nullopt};
  }
```

```cpp
// code/cpp/mobile_robot_lab_cpp/include/mobile_robot_lab_cpp/constant_velocity_tracker.hpp | assign()
        // 불확실성과 절대거리 양쪽 gate를 통과한 대응만 비용 행렬에 넣는다.
        if (euclidean <= config_.euclidean_gate &&
          mahalanobis_squared <= config_.mahalanobis_gate_squared)
        {
          costs[row][column] = mahalanobis_squared;
        }
// ... 생략: dummy row 비용 구성
    const auto assignment = hungarian_assignment(costs);
```

### 주변 구현: 본문보다 보조 설명에 적합

Safety에는 CPA·방향별 LiDAR 긴급 정지·scan/odom 입력 만료 정지·tracking 입력 만료 속도 제한·정지 복귀 대기가 구현되어 있다. `code/cpp/mobile_robot_lab_cpp/include/mobile_robot_lab_cpp/semantic_safety.hpp` | [SemanticSafety::evaluate()](../code/cpp/mobile_robot_lab_cpp/include/mobile_robot_lab_cpp/semantic_safety.hpp#L169)와 `code/cpp/mobile_robot_lab_cpp/src/cmd_vel_watchdog_node.cpp` | [CmdVelWatchdogNode::update()](../code/cpp/mobile_robot_lab_cpp/src/cmd_vel_watchdog_node.cpp#L49)를 근거로 **인식·추적 출력을 소비하는 제어 경로**라고 설명할 수 있다. 안전 보증이나 현재 조합의 무충돌 주행 검증으로 표현하지 않는다.

Calibration은 외부 `direct_visual_lidar_calibration` 도구를 사용한 MCAP·전처리·변환 추정 경험으로 한정한다. 자체 calibration 알고리즘 개발이나 실물 정확도 개선은 현재 수록 근거가 없다. `code/scripts/run-trial.sh` | [run_runtime()](../code/scripts/run-trial.sh#L73)의 다인 평가에서는 navigation·patrol을 껐으므로 순찰 완료 성과로 쓰지 않는다.

## 2. 최근 수정: 원인은 무엇이었고 어떻게 개선했는가

| 문제 | 확인한 원인 | 반영한 수정 | 관측한 결과 |
|---|---|---|---|
| 사람 검출이 있어도 3D 관측이 대량 거부됨 | 깊이 후보가 둘 이상이면 점 수와 무관하게 전체 거부 | 최근접 후보와 나머지 전체의 점 지지를 비교 | 고정 입력 비교에서 association 성공 57→654, FN 570→3 |
| 원본 mask 밖 물체 점이 사람으로 선택됨 | 외곽 polygon 재채움이 mask 구멍과 분리 영역을 변형 | `masks.xy` 경유 제거, 원본 크기 binary mask 직접 사용 | 구멍·분리 영역 추가 pixel 49/10→0/0. 합성 ROS에서 1m 물체 대신 2m 사람 선택 |
| 앞쪽 사람과 뒤쪽 벽이 같은 pixel에 남음 | Camera 투영 위치만 확인하고 pixel별 깊이 가림 관계는 검사하지 않음 | 동일 pixel에서 5cm 초과 뒤쪽 점 제외 | 합성 사람 3점+벽 6점에서 관측 없음→사람 3점·2m |

세 수정은 **서로 다른 원인**을 다룬다. Mask 손실 수정과 가림 필터의 성과를 첫 번째 FN 개선 수치에 합산하지 않는다. 첫 수정은 사람 소속을 확정하는 해법이 아니라, 관측을 과도하게 거부하던 조건의 완화다.

최근 수정은 binary mask 전달, 동일 pixel 가림 제외와 진단·설정 전달, 전경 후보 선택이다. 정확한 함수별 변경 전후는 [FN 수정 보고서](person_tracking_false_negatives.md)의 변경 표에서 확인한다. 이번 문서 작업에 따른 새 production 함수 변경은 없다.

## 3. 검증 수치와 사용 범위

### A. 전경 후보 선택 조건만 비교한 고정 입력 실험

2026-09-07 새로 기록한 seed 42·사람 3명·약 60초 자료에서 **동일한 997개 정렬 입력, GT 사람 관측 635개**를 C++ association·tracker에 재생했다. GT 635는 사람 635명이 아니라 시간에 걸쳐 누적한 관측 수다. Tracker·GT·0.75m 평가 gate는 고정했다.

| 지표 | 수정 전 | 전경 지지 조건 적용 후 |
|---|---:|---:|
| TP | 65 | 632 |
| FN | **570** | **3** |
| FP | 138 | 121 |
| ID switch | 8 | 8 |
| IDF1 | 5.49% | 42.22% |
| Association 성공 | 57 | 654 |

근거: [before.json](../trials/fn-association-support/before.json), [after.json](../trials/fn-association-support/after.json), [보존한 입력](../trials/fn-association-support/frames.json.gz), `trials/fn-association-support/replay.py` | [evaluate()](../trials/fn-association-support/replay.py#L18).

이 자료의 `frames` 지표는 674→644다. 재생 입력 997개는 같지만 ROI 내 GT 또는 prediction이 있을 때만 지표를 갱신하기 때문이다. 따라서 「동일한 997개 평가 frame」 대신 **「동일한 997개 입력을 재생」**이라고 쓴다. 이 비교는 mask·지면 필터 이후 저장된 instance 점을 사용하며 최신 binary mask·가림 필터 전체를 다시 평가한 결과가 아니다.

9월 4일의 FN 130은 다른 입력·실행 결과다. 당시 raw bag이 없어 동일 재생하지 못했으며 **FN 130→3으로 쓰지 않는다.** 수정 후 ROS 연결 재생의 TP 329/FN 5/GT 334도 처리 표본이 달라 위 표에 합치지 않는다. [실험 구분과 ROS 재생 근거](person_tracking_false_negatives.md).

### B. Binary mask 보존 검증

| 검사 | 확인된 결과 |
|---|---|
| 사각형·구멍·분리 mask | 현재 경로의 추가·삭제 pixel 모두 0 |
| 사람 9점+앞쪽 비사람 60점의 동일 입력 | Polygon 비교 경로는 1m 물체 선택, binary 경로는 2m 사람 선택 |
| 실제 YOLO11s-seg CPU 추론, 높이 1001×너비 797 영상 | 사람 4 instance, 추론 binary mask와 label 차이 0 pixel |
| 높이 479×너비 631 빈 영상 | 0 instance·배경 label 정상 |

근거: [mask·ROS 결과](../trials/mask-projection-validation/after.json), [실제 추론 결과](../trials/mask-projection-validation/inference.json). **Mask 전달 보존 검증**이며 모델의 사람 segmentation 정확도가 100%라는 뜻은 아니다.

### C. 최신 동일 pixel 가림 제외 및 회귀 검증

| 검사 | 확인된 결과 |
|---|---|
| 부분 관측 사람 3점(2m)+같은 pixel 벽 6점(5m) | 벽만 제외, 사람 3점·2m 발행 |
| 앞쪽 점이 이웃 pixel에만 존재 | 사람 3점 보존 |
| 같은 pixel의 깊이 2.00·2.03·2.04m | 5cm 이내이므로 모두 보존 |
| 앞쪽 1점+뒤쪽 3점 | 뒤쪽 제외 후 최소 3점 부족으로 관측 없음. 지면 거부와 구분 |
| 앞쪽 반사점 없이 벽 6점만 입력 | **5m 벽 오채택 유지** |
| 실제 ROS C++ node 합성 검증 | 10개 사례의 기대 결과 확인. 미해결 오채택도 기대 결과로 포함 |
| 회귀 테스트 | C++ 5개 + Python 27개 = **32개 통과** |

근거: [변경 전 실패](../trials/mask-projection-validation/visibility-before-run.log), [최신 10사례](../trials/mask-projection-validation/visibility.json), [회귀 로그](../trials/mask-projection-validation/visibility-tests.log), [빌드 로그](../trials/mask-projection-validation/visibility-build.log), [검증 source SHA-256](../trials/mask-projection-validation/visibility-source-sha256.json).

「10개 사례 통과」는 사람 판정 성공률 100%가 아니다. 미해결 벽 오채택이 그대로 재현되는지도 검사했다. 신체 일부 관측은 합성 점 수준에서 확인했으며, 다양한 실제 보행·자세·가림을 포괄한 평가가 아니다. 최신 조합의 별도 Gazebo 60초 실행 결과는 [수록 원고](portfolio-selected-content.md)에 추가했다. 동일 입력의 최신 전체 조합 전후 비교와 실시간 성능 benchmark는 아직 없다.

## 4. 포트폴리오에 옮길 문구

다음은 코드·검증 근거에 맞춘 **문장 후보**다. 사용자는 Mobin의 전체 설계·구현을 직접 수행했다고 확인했다. 개인 기여를 명시하고 YOLO·PCL·ROS 등 외부 도구 활용과 구분한다.

### 제목과 한 문장 소개

**RoboDine 개인 확장 — Camera–LiDAR 사람 위치 추정 및 Tracking**

> ROS 2·Gazebo에서 instance segmentation 기반 Camera–LiDAR 사람 관측과 Hungarian·Kalman 추적 파이프라인을 구성하고, 관측 거부·mask 변형·센서 시점 차이에 따른 실패를 분리해 수정·검증했습니다.

### 이력서·프로젝트 요약용 3개 bullet

- YOLO instance mask와 동기화된 LiDAR를 결합해 사람의 `odom` 위치를 관측하고, 제약 RANSAC 지면 제거와 Hungarian·Constant Velocity Kalman 기반 ID·위치·속도 추적 구현.
- 복수 깊이 후보 일괄 거부 조건을 전경 점 지지 비교로 수정. 단일 기록의 동일 997개 입력 재생에서 FN 570→3, FP 138→121, IDF1 5.49→42.22% 확인. GT 635개·자체 평가기 기준이며 ID switch 8회 유지.
- YOLO binary mask 직접 전달과 동일 pixel 깊이 비교를 추가해 mask 구멍 손실 및 관측된 앞 표면 뒤쪽 점 혼입에 대응. 실제 추론 mask 전달 검사, 합성 ROS 10사례와 C++·Python 32개 회귀 검증 수행.

두 번째 bullet의 수치는 전경 조건 변경 실험에만 해당한다. 세 번째 변경까지 포함한 전체 시스템의 최신 성능 수치로 묶지 않는다.

### 기술 스택

`ROS 2 Jazzy · Gazebo · C++17 · Python · YOLO11-seg · OpenCV · PCL · Eigen · TF2 · message_filters · PointCloud2 · rosbag/MCAP`

개인 기여의 초점은 **센서 데이터 연결, 후보 선택 정책, 실패 재현과 검증**이다. 외부 YOLO 모델 학습·PCL·Kalman·Hungarian 알고리즘 자체의 발명으로 설명하지 않는다.

## 5. 실제 TeX 포트폴리오 반영 내용

기준은 [presentation.tex](/home/swlinux/Desktop/workspace/ai-job-search/cv/ppt/presentation.tex)와 출력 [presentation_tex.pdf](/home/swlinux/Desktop/workspace/ai-job-search/cv/ppt/presentation_tex.pdf)다. 실제 소스명은 `presentation.tex`다. 기존 `PPT.pptx`를 별도 기준으로 삼지 않는다. 현재 TeX는 18개 frame이며 RoboDine와 Mobin 확장은 10–15쪽이다. [후보자 프로필](/home/swlinux/Desktop/workspace/ai-job-search/.claude/skills/job-application-assistant/01-candidate-profile.md:96)에 따라 별도 네 번째 프로젝트가 아닌 RoboDine 확장으로 분류한다.

| 위치 | 반영 내용 |
|---|---|
| 기존 10쪽 RoboDine 요약 | 주요 업무에 “Gazebo 기반 Camera–LiDAR 사람 관측·추적과 실패 원인 검증” 추가. 기존 실물 성과와 simulation 확장 구분 |
| 기존 11–12쪽 | 자세 추정·장애물 회피 성과 유지. 기존 14% 개선 수치를 Mobin 성과와 섞지 않음 |
| 신규 13쪽 | **3. Camera–LiDAR 사람 관측 및 추적** |
| 신규 14쪽 | **4. Association 실패 원인 분해와 검증** |
| 신규 15쪽 | **정량 검증과 남은 한계**: TP·TN·FP·FN·Precision·Recall·F1·IDSW·IDF1과 해석 |
| 기존 13–15쪽 | 구조물 안정성 내용은 16–18쪽으로 이동. 프로젝트 번호 `2-3` 유지 |

### 신규 13쪽: Camera–LiDAR 사람 관측 및 추적

**문제:** 사람 검출 영역에 섞인 지면·배경 점 때문에 3D 관측이 누락되거나 잘못 연결됨.

**구현:** 원본 binary mask와 timestamp·TF 기반 투영, 동일 pixel 가림 제외, 제약 RANSAC, instance별 깊이 후보 선택. `odom` 관측은 Hungarian·Kalman으로 ID·위치·속도를 추적.

**그림:** Gazebo에서 보존한 **359.800초·confirmed ID 98/99**의 Camera association 원본과 headless RViz 3D 렌더만 수록한다. 설정 패널·메뉴는 제외한다. 구체적 이미지·표 해설과 캡션은 [최종 수록 원고](portfolio-selected-content.md)를 따른다. 기존 9월 4일 화면과 HTML 흐름도는 참고 자료로 유지한다.

**하단 범례:** 노랑·주황 지면 점, 반투명 초록 지면 marker, 채택 점·흰 ID·불확실성·속도·추정 이력을 설명. 배경 오채택과 ID 전환 한계는 14–15쪽에 명시.

### 신규 14쪽: Association 실패 원인 분해와 검증

| 원인 | 수정 | 결과 |
|---|---|---|
| 복수 후보 일괄 거부 | 전경 후보 점 지지 비교 | FN 570→3, FP 138→121, IDF1 5.49→42.22%, ID switch 8→8 |
| Mask 외곽 재채움 | 원본 binary mask 직접 전달 | 추가 pixel 49/10→0/0, 합성 물체 오채택 개선 |
| 동일 pixel 뒤쪽 점 혼입 | 최소 깊이 대비 5cm 초과 점 제외 | 합성 사람 3점+벽 6점에서 사람 3점 보존 |

**하단 검증 조건:** 첫 행은 단일 기록 997개 입력·GT 635개·자체 평가기의 전경 조건 비교. 뒤 두 행은 별도 mask·합성 ROS 검증. 최신 C++·Python 32개 회귀 통과. 전체 조합의 실물·보행·다인 강건성은 미검증.

### 신규 15쪽: 정량 검증과 남은 한계

기존 정책과 점 지지 비교의 동일 입력 결과(A), 최신 별도 60초 실행(B)을 구분했다. TP·FP·FN에서 Precision·Recall·F1을 계산하고, ID switch·IDF1을 함께 제시한다. TN은 음성 객체 모집단 미정의로 N/A이며 Accuracy는 산출하지 않는다. 분모·ROI·0.75m gate·모든 track 상태 포함 조건과 각 표가 의미하는 범위를 명시한다.

기존 검정 상단 띠·연회색 본문·Noto Sans CJK KR을 유지했다. HTML 흐름도는 참고용으로 남기고 실제 Camera association·RViz 렌더만 수록했다. 요약 보강과 3개 frame을 추가해 전체 18쪽으로 구성했다. 기존 `cropimg` page는 참조 원본 `presentation.pdf`의 번호이므로 바꾸지 않았다. 원본 PNG는 `assets/mobin`에 복사하고 RViz 주변 여백만 TeX에서 crop한다.

## 6. 면접 답변

### 60초 설명

> 카메라에 사람이 보여도 LiDAR 연결이 실패하면 추적 관측이 사라지는 문제를 분석했습니다. 먼저 깊이 후보가 여러 개라는 이유만으로 전체 관측을 버리는 조건을 확인해, 전경 점 지지를 비교하도록 수정했습니다. 같은 기록을 재생했을 때 FN이 570에서 3으로 줄었지만 FP와 ID 전환은 남았습니다. 이후 최근접 점이 반드시 사람은 아니라는 점을 검토하면서, mask를 polygon으로 재채우는 과정의 구멍 손실과 센서 시점 차이에 따른 벽 혼입을 분리했습니다. 원본 binary mask를 보존하고, 같은 pixel에 앞쪽 반사점이 있을 때 뒤쪽 점을 제외하도록 수정해 합성 ROS 사례로 확인했습니다. 앞쪽 반사점이 없으면 이 필터만으로는 사람 깊이를 확정할 수 없다는 한계도 확인했습니다.

### 예상 질문과 답변 기준

| 질문 | 답변 |
|---|---|
| 왜 최근접 점을 사람이라고 하지 않는가? | 거리 순서는 물체의 의미를 결정하지 않는다. 가림 필터는 관측된 뒤쪽 점 제외용이고, 현재 후보 선택의 전경 지지 가정도 남은 휴리스틱이다. |
| Instance segmentation이 정확하면 해결되는가? | 먼저 원본 mask를 보존해야 한다. 그래도 LiDAR가 본 벽이 Camera에서는 사람 뒤에 가려질 수 있어 pixel 투영만으로 소속이 확정되지 않는다. |
| 벽 점만 들어오면 왜 실패하는가? | Camera에는 2m 사람이 있지만 LiDAR가 사람을 비껴가 5m 벽만 측정한 경우다. 같은 pixel에 비교할 2m 반사점이 없어 벽을 제외하지 못하고, 단일 후보가 채택된다. |
| 왜 이웃 pixel까지 제거하지 않았는가? | 주변의 가까운 점을 확장하면 부분적으로 보이는 사람 점도 지울 수 있다. 현재 정책은 같은 pixel의 깊이 근거만 사용하며, 이웃·작은 깊이 차이는 보존한다. |
| FN 감소가 tracker 개선인가? | 해당 실험에서 tracker와 평가 조건은 고정했다. Association 관측 거부 조건만 바꿔 입력 관측이 증가한 결과다. |
| IDF1 42.22%는 무엇인가? | 전체 기록의 실제 사람 ID와 track ID 대응에 대한 지표다. 검출 정확도나 안정적인 tracking 완성률이 아니다. FP 121·ID switch 8도 함께 제시한다. |
| 정확히 무엇을 검증했는가? | 첫 수정은 고정 입력 C++ 재생, mask 전달은 실제 YOLO 추론과 pixel 비교, 최신 필터는 합성 입력을 실제 ROS node에 넣어 확인했다. 서로 같은 수준의 E2E 검증은 아니다. |
| 다음에 보완할 부분은? | 앞쪽 반사점 부재·경계/시간차 오차·실제 부분 가림·사람 교차의 ID 연속성이다. 과거 track으로 association 후보를 판정하는 기능은 아직 구현하지 않았다. |

## 7. 수록 범위와 주장 경계

**수록 가능한 내용:** 센서 메시지·시간·좌표 계약, 원본 mask 보존, 제약 지면 제거, 제한적인 가림 필터, 전경 후보 선택, Hungarian·Kalman 추적, 자체 평가·회귀·실패 증거 수집.

**성과로 확정하지 않을 내용:** 모든 상황의 robust association, 실물 검증, 최신 전체 시스템의 FN 개선율, 무충돌 자율주행·순찰 완료, 독자 YOLO 학습·calibration 알고리즘, `hota_at_0.75m`를 표준 전체 HOTA 점수로 표현하는 것. 메시지 age는 순수 tracker 연산 지연이 아니며, GT 거리 threshold 진입 횟수는 물리 접촉 횟수가 아니다.

RoboDine 본 프로젝트는 팀 활동, Mobin 확장은 **본인 단독 설계·구현**으로 표시한다. 이는 사용자 확인에 근거한다. 외부 사전학습 모델·PCL·ROS는 활용 기술로 명시한다. 개별 개발 기간은 Git 날짜로 추정하지 않는다.

사용자의 실제 LaTeX 수정 요청에 따라 **RoboDine 요약 보강 + Mobin 3쪽 추가 + 검증 조건과 한계 병기**를 반영했다.

## 검토 기록

이번 정리에서 문서 링크·함수 정의 위치 42개, 최신 검증 source SHA-256, 입력 997개와 합성 ROS 10사례를 대조했다. 흐름도는 SVG 접근성·XML·글꼴 텍스트 폭 검사를 통과했다. HTML 흐름도는 포트폴리오 수록 대상에서 제외했다. 실제 TeX 편집 후 XeLaTeX로 PDF를 빌드하고 변경된 10·13–15쪽을 렌더링해 확인했다. 전체 18쪽의 배치를 검토했고, 나머지 14개 frame 원문이 기존과 동일함을 확인했다. 15쪽에는 Precision·Recall·IDSW의 개념·계산식·예시와 F1·IDF1의 차이도 본문에 반영했다.

PATCH 문서 길이 확인에서 기존 PATCH-02가 974줄로 500줄 목표를 초과했다. 이번 요약 범위 밖이므로 수정하지 않았다.

## 시니어 피드백 반영 기준

| 피드백 | 선별·반영 | 제외·한계 |
|---|---|---|
| 5분 내 역량 파악 | 단독 기여·문제·데이터·판단·검증 중심 3쪽 구성 | 함수·parameter 목록은 본문 제외 |
| 문제부터 적용까지의 사이클 | Gazebo 데이터 구성·수정·검증·시각화 설명 | Mobin의 실물 적용은 미수행, 기존 RoboDine 실물 성과와 구분 |
| 합리적 baseline·규모·방법 | 실제 기존 정책·997입력·GT635·0.75m gate·FP/IDF1 병기 | Worst 대비 수치·FN130→3·일반 정확도 표현 제외 |
| 본인 기여·데이터·실패 접근 | 단독 설계·구현 확인, seed·사람 수·GT 역할 명시 | 도구 이름과 결과만 나열하지 않음 |
| 최악 사례·예외 처리 | 벽-only 오채택·부분 관측·시간차·ID 전환 제시 | 합성 테스트 통과를 100% 사람 판정으로 쓰지 않음 |
| 설계 선택 설명 | 일괄 거부·polygon 재채움의 실패, 이웃 pixel 확장을 피한 이유 | 논의만 한 대안을 실제 실험으로 꾸미지 않음 |
| 현장의 조명·가림·시간축 | 최신 Gazebo·RViz·시간별 track 기록을 보조 증거로 제공 | 실물 조명 변화·장기 ID 유지 미검증 |
| 최신 기술도 정량 근거 필요 | VLM/VLA 키워드 추가하지 않음 | 튜토리얼 실행을 성과로 추가하지 않음 |

특정 지원 회사가 아직 없으므로 Perception·Tracking 공통 원고로 작성한다. 전체 18쪽 중 Mobin은 3쪽으로 구성해 역할·문제·검증을 빠르게 읽도록 했다.
