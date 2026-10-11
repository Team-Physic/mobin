# Simulation PATCH-07: Person Camera–LiDAR Association과 Tracking

- 작성일: 2026-09-04
- 브랜치: `feature/avoidance`
- 코드 기준: working tree
- 상태: **Instance segmentation·RANSAC·공통 ROI evaluator 구현, 28개 regression test 통과, seed 42 live baseline 측정 완료**
- 선행: PATCH-04 extrinsic 검증, PATCH-06 perception baseline
- 결론: **Camera가 분할한 사람 instance와 3D LiDAR point를 연결하고, `odom`에서 Constant Velocity Kalman Filter와 Hungarian assignment로 2~5명을 추적한다.**

### Why?

Python 기준선 `lidar_bbox_association_node.py`는 bbox 안으로 투영된 LiDAR point의 median만 계산한다. 현재 runtime은 `yolo_segmentation_detector_node.py`의 person Instance Label Map과 C++ association을 사용한다. Python 기준선은 production graph에 실행하지 않는다.

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
  → yolo_segmentation_detector(Python, GPU inference)
  → /detections                  vision_msgs/Detection2DArray
  → /segmentation/instance_labels sensor_msgs/Image, mono8

/detections + /segmentation/instance_labels + /calib/points + CameraInfo + TF
  → lidar_bbox_association_cpp(C++, geometry)
  → /fusion/detections_3d        vision_msgs/Detection3DArray, frame=odom
  → /fusion/associated_points    sensor_msgs/PointCloud2, frame=LiDAR
  → /fusion/ground_points        sensor_msgs/PointCloud2, frame=LiDAR
  → /fusion/non_ground_points    sensor_msgs/PointCloud2, frame=LiDAR
  → /fusion/ground_plane         visualization_msgs/Marker, frame=base_footprint
  → /fusion/diagnostics          diagnostic_msgs/DiagnosticArray

/fusion/detections_3d
  → person_tracker_cpp(C++, Kalman + Hungarian)
  → /tracking/persons            mobile_robot_lab_msgs/TrackedPersonArray
  → /tracking/markers            visualization_msgs/MarkerArray
```

Ultralytics `yolo11s-seg.pt` 호출만 Python에 둔다. Point projection, PCL RANSAC ground removal, temporal tracking과 safety 판단은 C++다. **Python과 C++가 서로 다른 tracker state를 동시에 유지하지 않는다.**

Official pretrained weight는 Git에 commit하지 않는다. Repository root에서 다음처럼 준비한다.

```bash
mkdir -p model/yolo-seg
curl -fL \
  https://github.com/ultralytics/assets/releases/download/v8.4.0/yolo11s-seg.pt \
  -o model/yolo-seg/yolo11s-seg.pt
test -s model/yolo-seg/yolo11s-seg.pt
```

Model과 Ultralytics package의 배포 조건은 [Ultralytics licensing](https://www.ultralytics.com/license)을 별도로 확인한다. 현재 weight는 `.gitignore`의 `/model/**/*.pt` 규칙으로 제외된다.

### 2. package 경계

| 디렉터리 | 구현 파일 | 역할 |
|---|---|---|
| `code/cpp/mobile_robot_lab_msgs/` | `msg/TrackedPerson.msg`, `msg/TrackedPersonArray.msg` | tracker와 safety/evaluator 사이의 typed contract |
| `code/cpp/mobile_robot_lab_cpp/include/mobile_robot_lab_cpp/` | `constant_velocity_tracker.hpp` | ROS와 분리한 Kalman·association core |
| 같은 C++ package | `ground_plane_filter.hpp` | constrained PCL RANSAC과 plane 검증 |
| 같은 C++ package | `src/lidar_bbox_association_node.cpp` | mask–point association과 `odom` 변환 |
| 같은 C++ package | `src/person_tracker_node.cpp` | track lifecycle과 message/Marker 발행 |
| 같은 C++ package | `test/constant_velocity_tracker_test.cpp` | crossing, miss, gate, covariance 검증 |
| Python package | `yolo_segmentation_detector_node.py` | person bbox·mono8 Instance Label Map 발행 |
| `code/python/mobile_robot_lab_python/mobile_robot_lab_python/` | `tracking_evaluation_roi.py`, `tracking_metrics.py`, `tracking_evaluator_node.py` | 공통 sensor ROI의 Gazebo GT와 예측 평가 |

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

### 4. Instance Label Map과 Camera–LiDAR synchronization

`/segmentation/instance_labels`는 흑백 Camera 영상이 아니다. 각 pixel에 이번 frame의 person 번호를 저장한다.

원본 영상 해상도의 YOLO binary mask(`retina_masks=True`, `masks.data`)를 직접 `mono8` label로 변환한다. Polygon 외곽선 추출·재채움은 하지 않아 mask의 구멍과 분리 영역을 보존한다. Mask 해상도가 원본 영상과 다르면 거부하며, instance끼리 겹친 pixel은 기존처럼 뒤 label이 우선한다.

| `mono8` 값 | 의미 |
|---:|---|
| `0` | 배경 |
| `1..N` | 같은 frame의 `Detection2D.id`와 일치하는 person instance |

이 번호는 frame-local label이다. 지속적인 사람 ID는 후단 C++ tracker가 별도로 만든다. 현재 목표가 2~5명이므로 `mono16`은 사용하지 않는다.

| parameter | 기본값 | 판정 |
|---|---:|---|
| `max_time_delta` | `0.05 s` | detection과 cloud timestamp 차이가 50 ms를 넘으면 pair 거부 |
| `sync_queue_size` | `30` | 허용시간 안의 message pair 검색용 queue |
| `ground_frame` | `base_footprint` | 바닥 높이 판단 좌표계 |
| `ground_inlier_distance` | `0.05 m` | 추정 plane에서 5 cm 이내 point를 ground로 분류 |
| `ground_max_tilt_degrees` | `20°` | 수평 바닥 normal과 허용하는 최대 각도 |
| `ground_max_origin_distance` | `0.15 m` | `base_footprint` 원점에서 plane까지 허용 거리 |
| `ground_min_inliers` | `50` | 유효 plane에 필요한 최소 RANSAC inlier 수 |
| `min_points` | `3` | 하나의 depth candidate에 필요한 최소 point 수 |
| `foreground_support_ratio` | `2.0` | 최근접 후보 point 수 / 나머지 유효 후보 point 합계의 최소 비율. 유한한 실수 `>1` |
| `camera_occlusion_depth_tolerance` | `0.05 m` | 동일 반올림 pixel의 최소 깊이보다 이 값 초과 뒤쪽 점만 instance 후보에서 제외. 유한한 실수 `>=0` |

`code/cpp/mobile_robot_lab_cpp/src/lidar_bbox_association_node.cpp` | [LidarBboxAssociationNode::process_pair()](../../code/cpp/mobile_robot_lab_cpp/src/lidar_bbox_association_node.cpp#L344)는 두 header timestamp를 정수 nanosecond로 비교한다. 따라서 정확히 50 ms인 pair는 허용하고, 50 ms를 초과한 pair만 거부한다.

```cpp
// code/cpp/mobile_robot_lab_cpp/src/lidar_bbox_association_node.cpp | LidarBboxAssociationNode::process_pair()
// 정수 nanosecond 차이로 50 ms 경계의 부동소수점 반올림 오류를 제거한다.
const auto delta_nanoseconds = std::abs(
  (rclcpp::Time(detections->header.stamp) - rclcpp::Time(cloud->header.stamp)).nanoseconds());
const auto max_delta_nanoseconds =
  rclcpp::Duration::from_seconds(max_time_delta_).nanoseconds();
if (delta_nanoseconds > max_delta_nanoseconds) {
  return;
}
```

### 5. Constrained PCL RANSAC ground removal

$$
d(p,\pi)=\frac{|ax+by+cz+d|}{\sqrt{a^2+b^2+c^2}}
$$

$p$는 `base_footprint`로 변환한 LiDAR point, $\pi$는 RANSAC plane이다. Camera FoV의 point로 plane을 추정하고 다음 조건을 모두 만족할 때만 ground를 제거한다.

1. Plane normal이 위쪽이며 수평축에서 `20°` 이내다.
2. `base_footprint` 원점과 plane의 수직거리가 `0.15 m` 이하다.
3. Plane에서 `0.05 m` 이내인 inlier가 50개 이상이다.

유효 plane이 없으면 **지면 판정으로는 point를 제거하지 않는다.** 선반이나 벽을 바닥으로 잘못 선택했을 때 사람 point가 사라지는 것보다 false ground를 보존하는 편이 안전하다. `/fusion/ground_points`, `/fusion/non_ground_points`, `/fusion/ground_plane`, `/fusion/diagnostics`는 가림 제외 전 지면 분리 결과를 보여준다.

### 6. Mask 내부 depth candidate

`code/cpp/mobile_robot_lab_cpp/include/mobile_robot_lab_cpp/camera_lidar_fusion.hpp` | [camera_occluded_points()](../../code/cpp/mobile_robot_lab_cpp/include/mobile_robot_lab_cpp/camera_lidar_fusion.hpp#L102)는 지면·label 필터 전 전체 cloud로 동일 반올림 pixel의 최소 Camera 깊이를 구한다. 그보다 `camera_occlusion_depth_tolerance` 초과 뒤쪽인 점만 instance 후보에서 제외한다. 이웃 pixel로 확장하지 않으며 허용 오차 이내 점을 보존한다. 기본 5cm는 실측 보증값이 아니다. 같은 pixel 안의 시선 차이·오차·희소 cloud 때문에 완전한 가림 증명은 아니다. 앞쪽 반사점 없이 벽만 관측되는 오채택은 남는다. [합성 ROS 검증과 한계](../../docs/person_tracking_false_negatives.md#동일-pixel-가림-제외-구현-2026-09-07).

가림 제외 후 각 person label과 겹치는 non-ground point를 Camera optical frame의 `z` 깊이로 정렬하고 `depth_window=0.35 m`보다 큰 간격에서 candidate를 나눈다. `min_points` 이상인 candidate가 0개면 정보 부족으로 거부한다. 최근접 유효 candidate의 point 수가 나머지 유효 candidate 합계의 `foreground_support_ratio`배 이상일 때 Person Observation을 만든다. 기본 비율은 `2.0`이며 candidate가 하나뿐이면 그대로 채택한다. 비슷한 지지나 배경 우세는 거부한다. Point 수 기반 전경 가정이며 사람임을 보장하는 판정은 아니다.

### 7. Kalman state

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

### 8. uncertainty radius

Position covariance $P_{xy}$의 가장 큰 eigenvalue를 $\lambda_{max}$라고 한다.

$$
r_{uncertainty}
=
\operatorname{clamp}
\left(2.447\sqrt{\lambda_{max}},\ 0.05,\ 0.40\right)\ \mathrm{m}
$$

`2.447`은 2차원 Gaussian의 95% confidence ellipse scale이다. 가장 긴 ellipse 축을 감싸는 원형 여유로 바꾸므로 방향과 무관하게 safety node가 사용할 수 있다. `0.40 m` cap은 장시간 놓친 track 하나가 무한히 커지는 것을 막으며, `0.5 s` track timeout이 별도로 track을 삭제한다.

### 9. Hungarian assignment와 gate

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

### 10. track lifecycle

| 상태 | 진입 조건 | 위험 판단 | 다음 상태 |
|---|---|---|---|
| `tentative` | unmatched detection 최초 관측 | 즉시 포함 | 최근 5 update 중 3 hit면 `confirmed` |
| `confirmed` | 3-of-5 충족 | 포함 | miss면 `lost` |
| `lost` | association miss | predicted position으로 포함 | 재관측 시 `confirmed`, `0.5 s` 초과 시 삭제 |
| deleted | 마지막 measurement가 `0.5 s`보다 오래됨 | 제외 | message와 Marker에서 제거 |

Track이 `tentative`여도 안전 계산에 포함한다. 오검출 하나로 장기 ID를 만들지는 않지만, 첫 검출을 무시해 충돌 대응이 늦어지는 것도 막는다.

Predicted FoV 안에 있고 주변 LiDAR point가 남아 있으면 detector miss 또는 짧은 가림으로 본다. FoV 밖이며 LiDAR support도 없으면 실제 퇴장 가능성이 높지만, 삭제 시각은 동일하게 `0.5 s`를 사용한다.

### 11. MarkerArray

`/tracking/markers`는 `odom`에서 다음을 표시한다.

| marker | 표시 | 목적 |
|---|---|---|
| sphere | 현재 Kalman position | 측정과 예측 위치 확인 |
| text | `person <id>`, state | ID switch와 tentative 확인 |
| arrow | `(vx,vy)` | 이동방향과 속도 확인 |
| line strip | 최근 trajectory | 교차·가림 이후 ID 연속성 확인 |
| ellipse | 95% position covariance | uncertainty가 관측·가림에 따라 변하는지 확인 |

`/fusion/associated_points`는 초록색, 원본 `/calib/points`는 회색으로 RViz에 함께 표시한다. Marker는 제어 입력이 아니다.

### 12. Camera–LiDAR 공통 ROI tracking 평가

`code/python/mobile_robot_lab_python/mobile_robot_lab_python/tracking_evaluator_node.py` | [TrackingEvaluatorNode.on_tracks()](../../code/python/mobile_robot_lab_python/mobile_robot_lab_python/tracking_evaluator_node.py#L254)는 track timestamp와 가장 가까운 Gazebo GT·`/odom` sample을 각각 선택한다. 둘 중 하나라도 차이가 `50 ms`를 넘으면 FN으로 만들지 않고 `timing_excluded_frames`를 증가시킨다.

사람의 평가 기준점은 Gazebo model origin에서 위로 `0.9 m`인 torso point다. GT와 prediction 모두 같은 point 정의를 사용한다. 이 point를 `odom`에서 Camera optical frame과 `base_scan`으로 각각 변환한다.

| 공통 ROI 조건 | 기본값 | 의미 |
|---|---:|---|
| Camera | `K`, image width·height | `z>0`이고 투영 pixel `(u,v)`가 image 안 |
| LiDAR 거리 | `0.1~20.0 m` | SDF range와 동일 |
| LiDAR 수평각 | `-π~π rad` | 360° |
| LiDAR 수직각 | `±0.261799 rad` | 위아래 약 `±15°` |

`code/python/mobile_robot_lab_python/mobile_robot_lab_python/tracking_evaluation_roi.py` | [camera_visibility()](../../code/python/mobile_robot_lab_python/mobile_robot_lab_python/tracking_evaluation_roi.py#L23)는 Camera 내부 파라미터로 다음 pixel을 계산한다.

$$
u=f_xX_C/Z_C+c_x,\qquad v=f_yY_C/Z_C+c_y
$$

$(X_C,Y_C,Z_C)$는 Camera optical frame의 torso point(m), $(f_x,f_y,c_x,c_y)$는 `CameraInfo.K`의 pixel 단위 값이다. $Z_C>0$이고 `0≤u<width`, `0≤v<height`일 때 Camera ROI다.

두 ROI 안의 GT가 매칭되지 않으면 FN, 두 ROI 안의 prediction이 매칭되지 않으면 FP다. ROI 밖 sample은 TP·FP·FN에서 제외한다. GT ID 이력은 ROI 이탈 중에도 삭제하지 않으므로 재진입한 같은 사람에 다른 Track ID가 연결되면 IDSW가 증가한다.

## What was problem

| 현재 기준선 | 구체적인 한계 | 설계 변경 |
|---|---|---|
| bbox 안 모든 point의 median | 뒤쪽 선반 point가 섞일 수 있음 | person mask + 지지가 충분한 최근접 depth candidate만 허용 |
| 현재 위치 Euclidean distance | 교차하는 사람의 운동방향을 무시 | Kalman prediction + Mahalanobis cost |
| 첫 관측 즉시 영구 ID | 한 frame false positive가 track 생성 | tentative 3-of-5 |
| `track_max_age=1.0 s` | 확정한 `0.5 s` 소실 기준과 불일치 | `0.5 s`로 변경 |
| Python Marker가 사실상 output contract | velocity·covariance를 다른 node가 읽을 수 없음 | custom `TrackedPersonArray` 추가 |

## How it changed

### 실행해서 검증 자료 만들기

#### 1. Model과 image를 준비한다

Repository root에서 weight 존재 여부를 먼저 확인한다. 파일이 없을 때만 앞의 download 명령을 실행한다.

```bash
cd /home/swlinux/Desktop/workspace/mobin
test -s model/yolo-seg/yolo11s-seg.pt
```

Docker image와 ROS 2 overlay는 서로 다른 결과물이다. `docker compose build`는 Ultralytics·PCL 등 image 내부 dependency를 준비하고, `colcon build`는 `code/cpp`, `code/python`의 현재 source로 ROS 2 package를 compile/install한다.

```bash
cd /home/swlinux/Desktop/workspace/mobin/docker
docker compose build

docker compose -f compose.yaml -f compose.nvidia.yaml run --rm shell bash -lc '
  source /opt/ros/jazzy/setup.bash
  cd /ws
  colcon build --packages-select \
    mobile_robot_lab_msgs mobile_robot_lab_cpp mobile_robot_lab_python \
    --symlink-install
'
```

#### 2. Regression test를 기록한다

Repository root의 script가 실행환경과 test log를 `trials/regression-test/`에 저장한다.

```bash
cd /home/swlinux/Desktop/workspace/mobin
./code/scripts/run-trial.sh regression-test
```

같은 이름의 기존 결과는 덮어쓰지 않는다. 다시 실행하려면 기존 결과를 지우지 말고 실행시각을 붙여 보관한다.

```bash
mv trials/regression-test \
  "trials/regression-test-$(date +%Y%m%d-%H%M%S)"
./code/scripts/run-trial.sh regression-test
```

`test.log` 마지막의 실패가 0개여야 한다. 현재 검증 기준은 **C++ 5개와 Python 23개, 총 28개 통과**다.

#### 3. Seed 42의 3명 tracking trial을 실행한다

이 명령은 사람 3명, seed 42, NVIDIA `device=0`, 60초 실행을 고정한다. Robot에는 `/cmd_vel_nav`로 `0.10 m/s` 직진 명령을 보내고, Camera annotation과 evaluator 결과를 함께 기록한다.

```bash
cd /home/swlinux/Desktop/workspace/mobin
YOLO_DEVICE=0 GAZEBO_GUI=true LAUNCH_RVIZ=true \
  ./code/scripts/run-trial.sh seeded-multi-person-tracking
```

기존 디렉터리가 있으면 먼저 위와 같은 방식으로 이름에 실행시각을 붙인다. 실행 중 Gazebo와 RViz가 열리며, 종료 후 다음 파일이 생성된다.

| 파일 | 확인할 내용 | 포트폴리오 사용 조건 |
|---|---|---|
| `environment.txt` | commit, CPU, memory, GPU·driver, Docker version | 결과 표의 실행환경 근거 |
| `association.png` | 모든 YOLO bbox, 성공 point·range, 실패 `YOLO only` | 같은 instance가 image 내부에 80% 이상 보이고 association 성공 |
| `evidence.json` | detection·fusion frame 수, capture 여부, association 결과별 누적 횟수 | `association_image_saved=true`, `association_outcomes` 존재 |
| `summary.json` | ROI TP·FP·FN, IDF1·MOTA·IDSW, sensor coverage, 시간 제외 수 | `true_positives>0`; threshold 없이 baseline 보고 |
| `roi-associations.csv` | frame별 TP·FP·FN, GT ID, Track ID, 거리 | 오차·누락 frame 역추적 |
| `id-switch-events.json` | GT ID에 연결된 이전·새 Track ID와 시각 | IDSW 원인 frame 확인 |
| `recorder.log` | recorder runtime 오류 | error 없이 60초 종료 |

#### 4. 실행 중 topic을 직접 확인한다

Trial이 실행 중일 때 새 terminal에서 같은 `patrol` container에 들어간다.

```bash
cd /home/swlinux/Desktop/workspace/mobin/docker
docker compose -f compose.yaml -f compose.nvidia.yaml exec patrol bash
```

Container 안에서 ROS 2 환경을 읽고 topic을 확인한다.

```bash
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash

ros2 topic hz /segmentation/instance_labels
ros2 topic echo /fusion/diagnostics --once
ros2 topic echo /fusion/detections_3d --once
ros2 topic echo /tracking/persons --once
```

| 확인값 | 정상 판정 |
|---|---|
| Instance Label Map | `encoding: mono8`; 배경 0, 사람 instance 1 이상 |
| ground diagnostic | status `OK`, plane origin distance `≤0.15 m`, inlier `≥50` |
| association diagnostic | 성공과 `no_points`·`multiple_depth_candidates`·`ground_rejection`·`tf_unavailable` 누적 횟수. `camera_occluded_points`는 별도 누적 점 수이며 횟수와 합산하지 않음 |
| 3D detection | `header.frame_id: odom`, 사람 detection 1개 이상 |
| tracked person | `track_id`, position, velocity, covariance, state 존재 |

#### 5. RViz와 Camera 결과를 캡처한다

RViz의 `Fixed Frame`을 `odom`으로 설정한다. `Add`로 다음 display를 추가한다.

| RViz display | topic | 권장 표시 | 증명하는 것 |
|---|---|---|---|
| `PointCloud2` | `/calib/points` | 회색, 작은 point | 원본 LiDAR 입력 |
| `PointCloud2` | `/fusion/ground_points` | 갈색 | RANSAC이 제거한 바닥 |
| `PointCloud2` | `/fusion/non_ground_points` | 청록색 | association 후보로 남은 point |
| `PointCloud2` | `/fusion/associated_points` | 초록색, 큰 point | person instance에 연결된 point |
| `Marker` | `/fusion/ground_plane` | 반투명 | 채택된 ground plane |
| `MarkerArray` | `/tracking/markers` | 기본색 | ID·위치·속도·trajectory·covariance |

Camera overlay는 RViz `Image` 또는 다음 명령으로 연다.

```bash
ros2 run rqt_image_view rqt_image_view /camera/detections_annotated
```

`association.png`는 recorder가 자동 저장하므로 화면을 다시 촬영할 필요가 없다. RViz에서는 ground/non-ground 분리와 `tracking/markers`가 동시에 보이는 장면을 `trials/seeded-multi-person-tracking/tracking-rviz.png`로 저장한다.

### 결과 채택 기준

**Script가 종료됐다는 사실만으로 tracking 성공을 주장하지 않는다.** 다음 순서로 판정한다.

1. `evidence.json`: Camera detection과 3D fusion frame이 모두 존재한다.
2. `association.png`: 초록 point가 사람의 mask 영역과 실제 깊이에 놓인다.
3. `/fusion/detections_3d`: 위치가 `odom` 기준이며 TF 오류가 없다.
4. `summary.json`: `true_positives>0`, `timing_excluded_frames`, Camera·LiDAR·공통 coverage를 확인한 뒤 IDF1·MOTA를 읽는다.
5. `roi-associations.csv`와 `id-switch-events.json`: FP·FN·IDSW 발생 시각을 Camera/RViz 기록과 대조한다.
6. `tracking-rviz.png`: 서로 다른 사람의 ID가 교차 전후 유지되는지 확인한다.

2026-09-04의 `seeded-multi-person-tracking` trial은 `TP=9`, `FP=20`, `FN=130`, `IDSW=2`, `IDF1=0.0595`를 기록했다. 전체 GT torso sample 중 Camera–LiDAR 공통 ROI 비율은 `16.67%`, 시간 정렬 제외는 0 frame이었다. **실행·산출물 생성은 검증됐지만 tracking 성능은 낮은 baseline이다.**

같은 실행에서 fusion node가 처리한 YOLO person 230건 중 성공은 18건, `multiple_depth_candidates=207`, `no_points=5`, `ground_rejection=0`, `tf_unavailable=0`이었다. 당시에는 후보가 2개 이상이면 모두 거부했다. 현재는 위의 전경 지지 비율을 사용하므로 이 숫자를 현재 구현 성능으로 제시하지 않는다. 2026-09-07에는 원시 입력을 새로 기록해 수정 전후를 동일 입력으로 비교했다. 기존 9월 4일 결과는 별도 baseline이며 새로운 평가의 FN 절대값과 직접 비교하지 않는다.

[LidarBboxAssociationNode::process_pair()](../../code/cpp/mobile_robot_lab_cpp/src/lidar_bbox_association_node.cpp#L344)는 association과 무관하게 동기화 frame의 모든 YOLO bbox를 그린다. 성공 bbox만 초록 point와 `range`를 표시하고 실패 bbox에는 `YOLO only`를 표시한다. 다만 이 topic은 Camera·label·detection·cloud가 50 ms 안에 묶인 frame만 발행하므로 detector 자체의 모든 frame을 보여주는 topic은 아니다.

발표자료 약 2장 중 PATCH-07 결과는 첫 장에 둔다.

| 배치 | 담을 자료 | 한 문장 주장 |
|---|---|---|
| 큰 그림 | `association.png` | Camera person instance와 같은 영역에 투영된 LiDAR point로 3D 위치를 계산했다. |
| 보조 그림 | `tracking-rviz.png` | `odom`에서 사람별 ID·위치·속도·covariance를 유지했다. |
| 작은 표 | `environment.txt`, `summary.json` | hardware·seed·frame 수·TP/FP/FN·IDF1·IDSW·p95 latency를 동일 실행에서 기록했다. |

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

Simulation에서 Gazebo entity pose를 evaluator 전용 GT로 기록한다. Runtime tracker에는 GT를 전달하지 않는다. Camera·LiDAR·공통 ROI coverage는 전체와 사람별로 기록한다.

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
| `colcon test` | C++ 5개 + Python 23개, 총 28개 통과 |
| YOLO11s-seg weight load | Ultralytics가 `task=segment`, COCO class 0=`person`으로 인식 |
| 기존 positive Camera image GPU inference | `cuda=True`, person `boxes=1`, `masks=1`, 1:1 대응 |
| `ros2 interface show mobile_robot_lab_msgs/msg/TrackedPersonArray` | `header`, ID·state·position·velocity·covariance message 확인 |
| 실행 중 topic type | `/segmentation/instance_labels`→`Image`, ground/non-ground→`PointCloud2`, ground plane→`Marker`, diagnostics→`DiagnosticArray`, detection output→`Detection3DArray` |
| Instance Label Map runtime | 실제 Camera stream에서 `encoding=mono8` 수신 |
| RANSAC runtime | `ground=5698`, `non-ground=14782`, ROI inlier=834, origin distance=`0.106810 m`, status=`OK` |
| seed 42 live association | 60초 동안 detector person frame 432, fusion detection frame 16, `association.png` 저장 |
| association 원인 집계 | YOLO 230건 중 성공 18, 복수 depth 207, point 부족 5, ground·TF 거부 0 |
| 공통 ROI evaluator | `TP=9`, `FP=20`, `FN=130`, `IDSW=2`, `IDF1=0.0595`, time exclusion 0 frame |
| timestamp/TF 재검증 | Gazebo 재시작 뒤 `perception`도 재생성하여 TF buffer를 초기화. 정수 nanosecond 경계 수정 후 `TF_OLD_DATA`·50 ms stale 경고 없이 association 수신 |

- Person-only C++ tracking core와 deterministic test 통과
- timestamp 차이 50 ms, RANSAC 초기값 `0.05 m/20°/0.15 m/50`, 실제거리 0.75 m, Mahalanobis 5.991 검증
- tentative 3-of-5, lost 0.5 s lifecycle 검증
- `/tracking/persons`에 `odom` position·velocity·covariance·state 발행
- `/tracking/markers`에서 ID·velocity·trajectory·covariance 확인
- unseen seed 결과에 HOTA@0.75m·IDF1·MOTA·IDSW 기록
- GT·odom 최근접 sample이 각각 50 ms를 넘으면 `timing_excluded_frames`로 분리
- ROI 이탈·재진입 사이에도 GT ID를 보존하고 새 Track ID 연결을 IDSW로 기록
- Python 기준선과 C++ runtime tracker를 동시에 실행하지 않음
