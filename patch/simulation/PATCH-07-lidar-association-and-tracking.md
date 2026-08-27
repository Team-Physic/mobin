# Simulation PATCH-07: Person LiDAR–BBox Association과 Tracking

- 작성일: 2026-08-27
- 브랜치: `feature/avoidance`
- 코드 기준: working tree
- 상태: **코드 구현·container build·unit/topic pipeline 검증 완료**
- 선행: PATCH-04 extrinsic 검증, PATCH-06 perception baseline
- 결론: **Camera가 검출한 사람 bbox와 3D LiDAR point를 연결하고, `odom`에서 Constant Velocity Kalman Filter와 Hungarian assignment로 2~5명을 추적한다.**

### Why?

현재 `code/python/mobile_robot_lab_python/mobile_robot_lab_python/lidar_bbox_association_node.py` | [LidarBboxAssociationNode.on_detection_points()](../../code/python/mobile_robot_lab_python/mobile_robot_lab_python/lidar_bbox_association_node.py#L187)는 bbox 안으로 투영된 LiDAR point의 median을 계산하고 `/fusion/detections_3d`를 발행한다. `code/python/mobile_robot_lab_python/mobile_robot_lab_python/person_tracker.py` | [PersonTracker._hungarian_matches()](../../code/python/mobile_robot_lab_python/mobile_robot_lab_python/person_tracker.py#L57)는 현재 위치 사이의 Euclidean distance만 사용한다.

이 기준선에는 velocity prediction, covariance, tentative track, 가림 복구가 없다. 사람이 교차하거나 관측이 잠깐 끊기면 다른 사람에게 ID가 넘어갈 수 있고, 위치 오차를 안전거리로 바꿀 수도 없다.

PATCH-07은 다음 질문에 답할 수 있는 구조를 만든다.

| 질문 | 출력 또는 지표 |
|---|---|
| Camera bbox에 연결된 LiDAR point가 사람 위치를 따르는가? | `/fusion/associated_points` overlay, center MAE |
| 같은 사람에게 같은 ID를 유지하는가? | HOTA·IDF1·MOTA·ID switch |
| 사람의 위치와 속도를 얼마나 확신하는가? | Kalman covariance와 uncertainty radius |
| 결과가 안전 제어에 전달 가능한가? | typed `TrackedPersonArray` message |

## What I Made

이 PATCH는 package, topic, message, 좌표계, 상태전이, parameter와 검증 계약을 코드로 고정한다.

### 1. 전체 data flow

```text
# patch/simulation/PATCH-07-lidar-association-and-tracking.md | implemented data flow
/camera/image_raw
  → yolo_world_detector(Python, GPU inference)
  → /detections                  vision_msgs/Detection2DArray

/detections + /calib/points + /camera/camera_info + TF
  → lidar_bbox_association_cpp(C++, geometry)
  → /fusion/detections_3d        vision_msgs/Detection3DArray, frame=odom
  → /fusion/associated_points    sensor_msgs/PointCloud2, frame=LiDAR

/fusion/detections_3d
  → person_tracker_cpp(C++, Kalman + Hungarian)
  → /tracking/persons            mobile_robot_lab_msgs/TrackedPersonArray
  → /tracking/markers            visualization_msgs/MarkerArray
```

Ultralytics model 호출은 Python API를 유지한다. Point projection, ground removal, temporal tracking과 safety 판단은 C++로 이동한다. **Python과 C++가 서로 다른 tracker state를 동시에 유지하지 않는다.**

### 2. package 경계

| 디렉터리 | 구현 파일 | 역할 |
|---|---|---|
| `code/cpp/mobile_robot_lab_msgs/` | `msg/TrackedPerson.msg`, `msg/TrackedPersonArray.msg` | tracker와 safety/evaluator 사이의 typed contract |
| `code/cpp/mobile_robot_lab_cpp/include/mobile_robot_lab_cpp/` | `constant_velocity_tracker.hpp` | ROS와 분리한 Kalman·association core |
| 같은 C++ package | `src/lidar_bbox_association_node.cpp` | bbox–point association과 `odom` 변환 |
| 같은 C++ package | `src/person_tracker_node.cpp` | track lifecycle과 message/Marker 발행 |
| 같은 C++ package | `test/constant_velocity_tracker_test.cpp` | crossing, miss, gate, covariance 검증 |
| `code/python/mobile_robot_lab_python/mobile_robot_lab_python/` | `tracking_metrics.py`, `tracking_evaluator_node.py` | Gazebo GT와 예측 JSON 평가 |

Custom message가 필요한 이유는 `PoseArray`가 ID, velocity, covariance, tentative/confirmed 상태를 담지 못하기 때문이다. ROS 2 Jazzy [`vision_msgs/Detection3D`](https://docs.ros.org/en/jazzy/p/vision_msgs/msg/Detection3D.html)는 ID와 pose를 담지만 velocity와 track lifecycle은 표현하지 않는다. `MarkerArray`는 화면 표시용이며 safety 입력으로 사용하지 않는다.

### 3. message 계약

`code/cpp/mobile_robot_lab_msgs/msg/TrackedPerson.msg`:

```text
# code/cpp/mobile_robot_lab_msgs/msg/TrackedPerson.msg
uint8 TENTATIVE=0
uint8 CONFIRMED=1
uint8 LOST=2

int32 track_id
uint8 state
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
float64[4] position_covariance
float64 uncertainty_radius
builtin_interfaces/Time last_measurement_stamp
```

`position_covariance` 순서는 `[Pxx, Pxy, Pyx, Pyy]`다. `position`, `velocity`와 covariance는 모두 `TrackedPersonArray.header.frame_id=odom` 기준이다.

`code/cpp/mobile_robot_lab_msgs/msg/TrackedPersonArray.msg`:

```text
# code/cpp/mobile_robot_lab_msgs/msg/TrackedPersonArray.msg
std_msgs/Header header
TrackedPerson[] persons
```

### 4. Camera–LiDAR synchronization과 ground removal

| parameter | 기본값 | 판정 |
|---|---:|---|
| `max_time_delta` | `0.05 s` | bbox와 cloud timestamp 차이가 50 ms를 넘으면 pair 거부 |
| `sync_queue_size` | `30` | 허용시간 안의 message pair 검색용 queue |
| `ground_frame` | `base_footprint` | 바닥 높이 판단 좌표계 |
| `ground_height` | `0.05 m` | 변환된 point의 z가 5 cm 이하면 제거 |
| `min_points` | `3` | bbox association에 필요한 최소 point 수 |

`code/cpp/mobile_robot_lab_cpp/src/lidar_bbox_association_node.cpp` | [LidarBboxAssociationNode::on_pair()](../../code/cpp/mobile_robot_lab_cpp/src/lidar_bbox_association_node.cpp#L122)는 두 header timestamp를 정수 nanosecond로 비교한다. 따라서 정확히 50 ms인 pair는 허용하고, 50 ms를 초과한 pair만 거부한다.

```cpp
// code/cpp/mobile_robot_lab_cpp/src/lidar_bbox_association_node.cpp | LidarBboxAssociationNode::on_pair()
// 정수 nanosecond 차이로 50 ms 경계의 부동소수점 반올림 오류를 제거한다.
const auto delta_nanoseconds = std::abs(
  (rclcpp::Time(detections->header.stamp) - rclcpp::Time(cloud->header.stamp)).nanoseconds());
const auto max_delta_nanoseconds =
  rclcpp::Duration::from_seconds(max_time_delta_).nanoseconds();
if (delta_nanoseconds > max_delta_nanoseconds) {
  return;
}
```

`code/python/mobile_robot_lab_python/mobile_robot_lab_python/lidar_bbox_association_node.py` | [remove_ground()](../../code/python/mobile_robot_lab_python/mobile_robot_lab_python/lidar_bbox_association_node.py#L70)의 C++ 이식식:

$$
p_B=T_{B\leftarrow L}p_L,\qquad p_B.z>0.05\ \mathrm{m}
$$

$p_L$은 LiDAR frame point, $T_{B\leftarrow L}$은 LiDAR에서 `base_footprint`로의 TF, $p_B$는 바닥 기준 point다. 이 식은 5 cm 이하의 바닥 return만 제거한다. 경사로·계단·발끝처럼 낮은 실제 물체도 제거될 수 있으므로 threshold를 parameter로 남긴다.

### 5. Kalman state

각 사람 track의 상태는 `odom` 평면에서 다음 네 값이다.

$$
x_k=[p_x,\ p_y,\ v_x,\ v_y]^T
$$

$p_x,p_y$의 단위는 m, $v_x,v_y$는 m/s다. `odom`은 짧은 시간 동안 연속적이므로 사람 trajectory와 robot velocity를 같은 좌표계에서 비교할 수 있다.

Constant Velocity prediction 식:

$$
\hat{x}_{k|k-1}=
\begin{bmatrix}
1&0&\Delta t&0\\
0&1&0&\Delta t\\
0&0&1&0\\
0&0&0&1
\end{bmatrix}x_{k-1|k-1}
$$

한 frame 사이에서 velocity가 일정하다고 가정한다. 실제 사람의 가속, 급회전과 정지는 process noise $Q$가 흡수한다. 관측이 끊기면 covariance가 커지고, 다시 관측되면 measurement update로 줄어든다.

### 6. uncertainty radius

Position covariance $P_{xy}$의 가장 큰 eigenvalue를 $\lambda_{max}$라고 한다.

$$
r_{uncertainty}
=
\operatorname{clamp}
\left(2.447\sqrt{\lambda_{max}},\ 0.05,\ 0.40\right)\ \mathrm{m}
$$

`2.447`은 2차원 Gaussian의 95% confidence ellipse scale이다. 가장 긴 ellipse 축을 감싸는 원형 여유로 바꾸므로 방향과 무관하게 safety node가 사용할 수 있다. `0.40 m` cap은 장시간 놓친 track 하나가 무한히 커지는 것을 막으며, `0.5 s` track timeout이 별도로 track을 삭제한다.

### 7. Hungarian assignment와 gate

각 predicted track $i$와 detection $j$에 대해 innovation과 covariance를 계산한다.

$$
y_{ij}=z_j-H\hat{x}_i,\qquad
d_{M,ij}^2=y_{ij}^TS_i^{-1}y_{ij}
$$

$z_j$는 현재 사람 측정위치(m), $H\hat{x}_i$는 같은 timestamp의 predicted position, $S_i$는 predicted position covariance와 measurement covariance의 합이다. Hungarian algorithm은 gate를 통과한 조합 중 전체 cost 합이 가장 작은 assignment를 선택한다.

후보 조건은 둘 다 만족해야 한다.

$$
\lVert z_j-H\hat{x}_i\rVert_2\le0.75\ \mathrm{m}
$$

$$
d_{M,ij}^2\le5.991
$$

`5.991`은 2차원 95% chi-square gate다. Mahalanobis gate가 covariance를 고려하고, `0.75 m` hard gate가 uncertainty가 큰 track이 먼 사람에게 붙는 것을 막는다.

### 8. track lifecycle

| 상태 | 진입 조건 | 위험 판단 | 다음 상태 |
|---|---|---|---|
| `tentative` | unmatched detection 최초 관측 | 즉시 포함 | 최근 5 update 중 3 hit면 `confirmed` |
| `confirmed` | 3-of-5 충족 | 포함 | miss면 `lost` |
| `lost` | association miss | predicted position으로 포함 | 재관측 시 `confirmed`, `0.5 s` 초과 시 삭제 |
| deleted | 마지막 measurement가 `0.5 s`보다 오래됨 | 제외 | message와 Marker에서 제거 |

Track이 `tentative`여도 안전 계산에 포함한다. 오검출 하나로 장기 ID를 만들지는 않지만, 첫 검출을 무시해 충돌 대응이 늦어지는 것도 막는다.

Predicted FoV 안에 있고 주변 LiDAR point가 남아 있으면 detector miss 또는 짧은 가림으로 본다. FoV 밖이며 LiDAR support도 없으면 실제 퇴장 가능성이 높지만, 삭제 시각은 동일하게 `0.5 s`를 사용한다.

### 9. MarkerArray

`/tracking/markers`는 `odom`에서 다음을 표시한다.

| marker | 표시 | 목적 |
|---|---|---|
| sphere | 현재 Kalman position | 측정과 예측 위치 확인 |
| text | `person <id>`, state | ID switch와 tentative 확인 |
| arrow | `(vx,vy)` | 이동방향과 속도 확인 |
| line strip | 최근 trajectory | 교차·가림 이후 ID 연속성 확인 |
| ellipse | 95% position covariance | uncertainty가 관측·가림에 따라 변하는지 확인 |

`/fusion/associated_points`는 초록색, 원본 `/calib/points`는 회색으로 RViz에 함께 표시한다. Marker는 제어 입력이 아니다.

## What was problem

| 현재 기준선 | 구체적인 한계 | 설계 변경 |
|---|---|---|
| bbox 안 모든 point의 median | 뒤쪽 선반 point가 섞일 수 있음 | associated cloud overlay와 center MAE로 실패 구간 확인 |
| 현재 위치 Euclidean distance | 교차하는 사람의 운동방향을 무시 | Kalman prediction + Mahalanobis cost |
| 첫 관측 즉시 영구 ID | 한 frame false positive가 track 생성 | tentative 3-of-5 |
| `track_max_age=1.0 s` | 확정한 `0.5 s` 소실 기준과 불일치 | `0.5 s`로 변경 |
| Python Marker가 사실상 output contract | velocity·covariance를 다른 node가 읽을 수 없음 | custom `TrackedPersonArray` 추가 |

## How it changed

### 구현 순서

1. `mobile_robot_lab_msgs` message를 build한다.
2. ROS와 분리된 `constant_velocity_tracker.hpp` deterministic test를 실행한다.
3. 기존 Python association과 같은 MCAP을 C++ association에 입력해 output center 차이를 비교한다.
4. `person_tracker_cpp`의 `/tracking/persons`, `/tracking/markers`를 확인한다.
5. `tracking_evaluator`가 Gazebo GT를 runtime node와 분리해 기록하는지 확인한다.
6. 아래 기준을 통과한 뒤 PATCH-08 semantic safety가 `/tracking/persons`를 사용한다.

### test matrix

| test | 입력 | 합격 조건 |
|---|---|---|
| stationary noise | 정지 사람 + 위치 noise | velocity가 0으로 수렴, covariance 유한 |
| crossing | 두 사람이 교차 | ID switch 0회 |
| 2-frame miss | confirmed track 관측 누락 | 같은 ID로 복구 |
| timeout | 0.5 s 이상 miss | track 삭제 |
| far detection | 실제거리 0.76 m | Mahalanobis 값과 무관하게 거부 |
| statistical gate | $d_M^2=5.992$ | association 거부 |
| clock reversal | simulation reset | 기존 track 삭제 후 ID/state 초기화 |

### 평가

Simulation에서 Gazebo entity pose를 evaluator 전용 GT로 기록한다. Runtime tracker에는 GT를 전달하지 않는다.

| 지표 | 보여주는 것 | 초기 판정 |
|---|---|---|
| associated cloud overlay | bbox–LiDAR 연결 위치 | 원본 cloud와 RViz에서 겹쳐 오연결 frame 확인 |
| center MAE | `odom` 사람 중심 위치 오차 | baseline 기록 후 개선 비교 |
| HOTA·DetA·AssA | detection/association 균형 | unseen seed에서 보고 |
| IDF1·MOTA·IDSW | ID 유지와 전체 tracking 오류 | baseline 대비 개선 |
| p95 latency | real-time worst-case 지연 | 20 Hz safety input을 막지 않아야 함 |

절대 HOTA·IDF1 목표는 baseline 측정 전에 만들지 않는다. 동일 GT와 거리 gate로 seed 사이 결과를 비교한다. 현재 `tracking_metrics.py`의 `HOTA@0.75m`는 단일 거리 threshold 요약값이며 여러 threshold를 평균하는 공식 HOTA benchmark와 구분한다.

## 완료 조건

### 현재 검증 증거

| 검증 | 결과 |
|---|---|
| `colcon build --packages-up-to mobile_robot_lab_python` | custom message·C++ association/tracker·Python node build 성공 |
| `colcon test` | C++ 4개 + Python 13개, 총 17개 통과 |
| `ros2 interface show mobile_robot_lab_msgs/msg/TrackedPersonArray` | `header`, ID·state·position·velocity·covariance message 확인 |
| 실행 중 topic type | `/detections`→`Detection2DArray`, `/fusion/detections_3d`→`Detection3DArray`, `/tracking/persons`→`TrackedPersonArray`, `/tracking/markers`→`MarkerArray` |
| positive association E2E | `person` 2개를 `odom`의 `Detection3DArray`로 수신: confidence `0.928`, `0.905`; 이어서 track ID `98`, `99`의 position·velocity·covariance 수신 |
| timestamp/TF 재검증 | Gazebo 재시작 뒤 `perception`도 재생성하여 TF buffer를 초기화. 정수 nanosecond 경계 수정 후 `TF_OLD_DATA`·50 ms stale 경고 없이 association 수신 |

- Person-only C++ tracking core와 deterministic test 통과
- timestamp 차이 50 ms, ground 5 cm, 실제거리 0.75 m, Mahalanobis 5.991 검증
- tentative 3-of-5, lost 0.5 s lifecycle 검증
- `/tracking/persons`에 `odom` position·velocity·covariance·state 발행
- `/tracking/markers`에서 ID·velocity·trajectory·covariance 확인
- unseen seed 결과에 HOTA@0.75m·IDF1·MOTA·IDSW 기록
- Python 기준선과 C++ runtime tracker를 동시에 실행하지 않음
