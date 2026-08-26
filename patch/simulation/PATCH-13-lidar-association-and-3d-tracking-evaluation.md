# Simulation PATCH-13: LiDAR–BBox Association과 3D Tracking 평가

- 상태: **후속 학습·구현 계획**
- 선행: PATCH-04 extrinsic 검증, PATCH-06 `lidar_bbox_association`
- 목적: association과 tracking 개선을 화면 인상이 아니라 수치로 증명

## 결론

PATCH-06은 YOLO bbox 안에 투영된 LiDAR point와 median 3D 위치를 발행한다. PATCH-13에서는 이 결과가 실제 cart point인지 평가하고, 여러 cart를 시간에 따라 같은 ID로 유지하는 3D Multi-Object Tracking으로 확장한다.

**LiDAR–bbox association과 tracking은 다른 문제다.**

| 문제 | 질문 | 한 frame의 출력 |
|---|---|---|
| LiDAR–bbox association | 이 bbox에 어떤 LiDAR point가 속하는가? | bbox별 point 집합과 3D 위치 |
| temporal tracking | 이전 frame의 cart와 현재 cart가 같은 물체인가? | 지속되는 `track_id`와 상태 |

## 현재 기준선

```text
/detections + /calib/points + /camera/camera_info + TF
  → LiDAR point를 image pixel로 투영
  → bbox 안 point 선택
  → median 3D 위치
  → /fusion/associated_points + /fusion/detections_3d
```

| 구현 | 위치 | 현재 한계 |
|---|---|---|
| Python ROS node | `code/python/mobile_robot_lab_python/mobile_robot_lab_python/lidar_bbox_association_node.py` | bbox 안 배경 point도 함께 선택 가능 |
| Python core | `code/python/mobile_robot_lab_python/mobile_robot_lab_python/camera_lidar_fusion.py` | 한 frame의 기하 연산만 수행 |
| C++ core | `code/cpp/mobile_robot_lab_cpp/include/mobile_robot_lab_cpp/camera_lidar_fusion.hpp` | ROS node와 temporal state 없음 |

## 1. LiDAR–bbox association을 보이는 방법

RViz 또는 Rerun에 다음 항목을 같은 timestamp로 표시한다.

| 표시 | 색 | 의미 |
|---|---|---|
| `/calib/points` | 회색 | 원본 LiDAR cloud |
| `/fusion/associated_points` | 초록 | bbox 안으로 투영된 point |
| `/fusion/detections_3d` | 노랑 marker | 선택 point의 median 3D 위치 |
| Gazebo cart GT cuboid | 파랑 wireframe | 실제 cart의 3D 범위 |
| 잘못 선택된 point | 빨강 | GT cart 밖인데 선택된 point |

한 장의 예시 화면만으로 성공을 주장하지 않는다. Cart가 선반 앞을 지나가면 같은 2D bbox 안에 cart와 뒤쪽 선반 point가 동시에 들어올 수 있기 때문이다.

## 2. Association ground truth 생성

Simulation에서는 Gazebo의 cart world pose와 SDF collision box를 GT로 사용한다.

```text
LiDAR point
  → world frame으로 변환
  → cart world pose의 역변환으로 cart local frame 변환
  → collision box 내부인지 검사
```

현재 cart collision은 local pose `(0.07, -0.015, 0.49)`와 size `(1.16, 0.54, 0.96) m`다. 각 축에서 다음 범위 안이면 cart GT point로 표시한다.

```text
abs(x - 0.07)  <= 1.16 / 2 + tolerance
abs(y + 0.015) <= 0.54 / 2 + tolerance
abs(z - 0.49)  <= 0.96 / 2 + tolerance
```

`tolerance`는 LiDAR noise와 mesh·box 차이를 흡수하는 값이다. 첫 실험은 `0.03 m`로 고정하고 sensitivity table을 함께 남긴다.

## 3. Association 평가 지표

| 지표 | 계산 | 해석 |
|---|---|---|
| point precision | 선택 point 중 GT cart point 비율 | 배경 point 혼입이 적은가 |
| point recall | GT cart point 중 선택된 비율 | cart point를 놓치지 않는가 |
| association success rate | GT가 보이는 frame 중 3D 결과 생성 비율 | sparse cloud에서도 결과가 나오는가 |
| center MAE | 추정 center와 GT center 거리 평균 | `(u,v) → (x,y,z)` 정확도 |
| depth MAE | 추정 range와 GT range 차이 평균 | 회피 거리로 쓸 수 있는가 |
| p95 latency | detection stamp부터 association 발행까지 p95 | 실시간 경로 지연 |
| timestamp rejection rate | stale 판정 frame 비율 | sensor sync 상태 |

CSV에는 최소 다음 열을 기록한다.

```text
stamp,scene_id,object_id,bbox_score,selected_points,gt_points,
true_selected_points,estimated_x,estimated_y,estimated_z,
gt_x,gt_y,gt_z,time_delta_ms,latency_ms
```

## 4. Association 개선 순서

같은 MCAP과 GT를 재사용해 한 항목씩 비교한다.

| 실험 | 규칙 | 확인할 개선 |
|---|---|---|
| A | bbox 안 모든 point의 median | PATCH-06 baseline |
| B | bbox 중앙 60%만 사용 | 경계의 배경 point 감소 |
| C | Camera depth 방향 cluster 중 가장 가까운 cluster | 뒤쪽 선반 제거 |
| D | 이전 3D 위치 주변의 motion gate 추가 | frame 간 위치 jump 감소 |
| E | sensor pose를 point timestamp에 보간 | 이동 중 timing 오차 감소 |

복잡한 learned fusion부터 추가하지 않는다. C까지로 point precision과 center MAE가 충분히 좋아지는지 먼저 측정한다.

## 5. Multi-Object Tracking 개념

Multi-Object Tracking은 여러 detection을 frame 사이에서 연결해 동일 물체에 같은 ID를 유지하는 작업이다. Detector가 물체를 찾고, association은 현재 detection이 기존 track 중 무엇과 같은지 결정한다.

첫 기준선은 다음으로 제한한다.

```text
3D detection center
  → constant-velocity Kalman filter predict
  → 예측 center와 새 center의 거리 cost matrix
  → distance gate
  → Hungarian assignment
  → matched update / unmatched lost / new track 생성
```

| 상태 | 의미 |
|---|---|
| tentative | 아직 연속 관측 횟수가 부족한 후보 |
| confirmed | 동일 물체로 확정된 track |
| lost | 일시 가림 또는 detector miss |
| deleted | 허용 miss 시간을 넘긴 track |

## 6. 여러 cart 평가 scenario

| scenario | 구성 | 주로 드러나는 실패 |
|---|---|---|
| parallel | 2~5개 cart가 나란히 이동 | 가까운 물체끼리 ID 혼동 |
| crossing | 서로 교차 | ID switch |
| occlusion | 선반 뒤에서 사라졌다 재등장 | fragmentation, longest gap |
| detector miss | 일정 frame detection 제거 | track 유지 능력 |
| false positive | 임의 bbox 삽입 | 가짜 track 생성 |
| timing gap | detection timestamp 지연 | 과거 관측 association |

모든 scenario는 seed, cart 수, 속도, detector drop 확률을 metadata에 남긴다. 개선 전후에 같은 seed를 사용한다.

## 7. Tracking 지표

[TrackEval](https://github.com/JonathonLuiten/TrackEval)은 HOTA, CLEAR MOT, Identity 계열 지표의 공식·공개 구현을 제공한다. [HOTA 논문](https://pmc.ncbi.nlm.nih.gov/articles/PMC7881978/)은 detection과 association을 분리해 분석하도록 HOTA를 DetA·AssA 등으로 분해한다.

| 지표 | 보여주는 것 | 방향 |
|---|---|---|
| HOTA | detection과 association의 균형 | 높을수록 좋음 |
| DetA | 물체를 찾은 정확도 | 높을수록 좋음 |
| AssA | 같은 ID를 유지한 정확도 | 높을수록 좋음 |
| IDF1 | GT identity와 예측 identity가 일치한 비율 | 높을수록 좋음 |
| MOTA | FP·FN·ID switch를 함께 반영 | 높을수록 좋음 |
| IDSW | 같은 물체의 ID가 바뀐 횟수 | 낮을수록 좋음 |
| Frag | 하나의 track이 끊긴 횟수 | 낮을수록 좋음 |
| TID | 물체 등장 후 track 확정까지 시간 | 낮을수록 좋음 |
| LGD | 가장 오래 놓친 구간 | 낮을수록 좋음 |
| FPS, p95 latency | 처리량과 worst-case 지연 | FPS는 높게, 지연은 낮게 |

[nuScenes tracking benchmark](https://www.nuscenes.org/tracking)는 3D tracking에서 ground-plane center distance로 match하고 AMOTA·AMOTP와 FP·FN·IDS·Frag·TID·LGD·FPS를 함께 보고한다. 이 프로젝트는 작은 warehouse라 nuScenes leaderboard와 직접 비교하지 않고 지표 정의와 결과 표 형식만 참고한다.

## 8. 결과 표

Detection 결과를 고정해야 tracker 개선을 분리해 설명할 수 있다. 같은 detection JSON을 baseline과 개선 tracker에 모두 입력한다.

| 버전 | HOTA↑ | DetA↑ | AssA↑ | IDF1↑ | IDSW↓ | Frag↓ | center MAE↓ | p95 ms↓ |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| nearest-center baseline | 기록 | 기록 | 기록 | 기록 | 기록 | 기록 | 기록 | 기록 |
| + Kalman prediction | 기록 | 기록 | 기록 | 기록 | 기록 | 기록 | 기록 | 기록 |
| + class·velocity gate | 기록 | 기록 | 기록 | 기록 | 기록 | 기록 | 기록 | 기록 |

**DetA는 같고 AssA·IDF1이 오르며 IDSW가 줄면 tracking association 개선의 증거다.** Detector 입력까지 바뀌었다면 별도 실험으로 분리한다.

## 예정 코드 위치

```text
code/python/mobile_robot_lab_python/
├── evaluation/export_tracking_ground_truth.py
├── evaluation/evaluate_lidar_bbox_association.py
└── evaluation/run_trackeval.py

code/cpp/mobile_robot_lab_cpp/
├── include/mobile_robot_lab_cpp/multi_object_tracker.hpp
├── src/multi_object_tracker.cpp
└── test/multi_object_tracker_test.cpp
```

평가와 plot은 Python, 실시간 tracker는 C++로 나눈다. 같은 JSON schema를 사용해 알고리즘과 평가기를 분리한다.

## 완료 조건

- MCAP, GT, detection 입력을 고정하고 재실행 가능한 명령 제공
- association point precision·recall·center MAE·p95 latency 자동 산출
- 2~5개 cart의 고정 seed crossing·occlusion case 생성
- baseline과 개선 tracker가 동일 detection 입력 사용
- HOTA·DetA·AssA·IDF1·MOTA·IDSW·Frag·TID·LGD·FPS 결과 저장
- 실패 frame을 Rerun에서 timestamp로 다시 열 수 있음
- 개선 주장마다 수치와 실패 사례를 함께 제시
