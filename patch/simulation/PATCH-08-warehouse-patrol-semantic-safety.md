# Simulation PATCH-08: Warehouse Patrol과 Semantic Safety

- 작성일: 2026-08-27
- 브랜치: `feature/avoidance`
- 코드 기준: working tree
- 상태: **코드·unit test·random pedestrian runtime 검증 완료, map/AMCL/Nav2 정량 평가는 대기**
- 선행: PATCH-05 warehouse, PATCH-06 LiDAR baseline, PATCH-07 person tracking
- 결론: **Nav2가 창고 외곽 patrol command를 만들고, semantic safety node가 사람 CPA와 LiDAR emergency distance를 검사해 최종 `/cmd_vel`을 발행한다.**

### Why?

현재 `code/cpp/mobile_robot_lab_cpp/src/obstacle_avoidance.cpp` | [`ObstacleAvoidanceNode`](../../code/cpp/mobile_robot_lab_cpp/src/obstacle_avoidance.cpp#L22)는 `/scan`의 front·left·right minimum distance만으로 전진 또는 회전을 선택한다. 목표 pose, map localization, 사람 identity와 접근속도를 사용하지 않는다.

이 기준선은 센서와 제동 동작을 확인하는 회귀시험으로 유지한다. 새 scenario에서는 Nav2가 경로를 계획하고, semantic safety가 그 command를 감속·정지만 한다. **Safety node가 새 경로를 계획하지 않는다.**

## What I Made

### 1. node와 command 흐름

```text
# patch/simulation/PATCH-08-warehouse-patrol-semantic-safety.md | implemented command flow
Nav2 /cmd_vel_nav ─┐
                   ├→ twist_mux /cmd_vel_mux
Teleop /cmd_vel_teleop ─┘
                              │
/tracking/persons ─────────────┤
/scan ─────────────────────────┤→ semantic_safety_cpp → /cmd_vel_safe
/odom ─────────────────────────┘                         │
                                                       ↓
                                              cmd_vel_watchdog_cpp → /cmd_vel
```

`twist_mux` priority는 teleop `100`, Nav2 `10`이다. Teleop 종료 후 `1 s` 동안 Nav2 command를 보류해 오래된 command가 즉시 재개되지 않게 한다. Semantic safety는 mux 뒤에 있어 Nav2와 teleop 모두 동일한 safety gate를 통과한다.

### 2. 추가 파일

| 디렉터리 | 구현 파일 | 역할 |
|---|---|---|
| `code/cpp/mobile_robot_lab_cpp/include/mobile_robot_lab_cpp/` | `semantic_safety.hpp` | CPA, safety radius, stale, stable-safe core |
| 같은 C++ package | `src/semantic_safety_node.cpp` | ROS subscription, 20 Hz safe command, diagnostics/Marker |
| 같은 C++ package | `src/cmd_vel_watchdog_node.cpp` | safe command가 0.3 s 끊기면 zero command 발행 |
| 같은 C++ package | `test/semantic_safety_test.cpp` | CPA와 fail-safe deterministic test |
| `code/python/mobile_robot_lab_python/launch/` | `warehouse_patrol.launch.py` | map, AMCL, Nav2, perception, tracking, safety 시작 |
| `code/python/mobile_robot_lab_python/config/` | `warehouse_patrol.yaml` | patrol pose와 Nav2 parameter |
| 같은 config | `twist_mux.yaml`, `semantic_safety.yaml` | command priority와 safety threshold |
| `code/python/mobile_robot_lab_python/mobile_robot_lab_python/` | `tracking_evaluator_node.py`, `tracking_metrics.py` | GT tracking·collision·patrol 결과 기록 |
| `code/scripts/` | `run-warehouse-evaluation.sh`, `aggregate-warehouse-evaluation.py` | unseen seed 반복 실행·집계 |
| `forks/turtlebot3_simulations/turtlebot3_gazebo/` | C++ pedestrian manager와 aisle graph | 2~5명 random walking, reservation |

새 bringup package는 만들지 않는다. 기존 Python package가 launch/config를 이미 설치하므로 그대로 재사용한다.

### 3. SLAM과 AMCL 단계

| 단계 | 사람 | 목적 | 결과 |
|---|---|---|---|
| map 생성 | 생성하지 않음 | 정적 warehouse occupancy map 작성 | `.yaml` + `.pgm` |
| localization 평가 | 2~5명 이동 | 저장 map과 `/scan`으로 robot pose 추정 | `/amcl_pose`, `map→odom` TF |
| patrol | 2~5명 이동 | 고정된 `map` waypoint 순환 | Nav2 command와 lap metric |

SLAM 평가와 AMCL 평가를 같은 실행에서 섞지 않는다. 이동하는 사람은 map에 포함하지 않으며, AMCL의 beam filtering이 동적 return을 견디는지를 별도 측정한다.

### 4. patrol route

Patrol waypoint는 `map` frame에서 창고 외곽을 한 바퀴 도는 고정 pose 목록이다. 동일 route를 반복해야 seed 사이의 충돌위험과 완주시간을 비교할 수 있다.

```yaml
# code/python/mobile_robot_lab_python/config/warehouse_patrol.yaml | patrol route
frame_id: map
waypoints:
  - {x: -3.0, y: -4.0, yaw: 0.0}
  - {x:  3.0, y: -4.0, yaw: 1.57}
  - {x:  3.0, y:  4.0, yaw: 3.14}
  - {x: -3.0, y:  4.0, yaw: -1.57}
```

위 좌표는 설계 예시다. 실제 warehouse collision과 inflation layer를 RViz에서 확인한 뒤 collision-free pose로 기록해야 한다.

### 5. pedestrian manager

`forks/turtlebot3_simulations/turtlebot3_gazebo/launch/turtlebot3_avoidance.launch.py` | [select_people()](../../forks/turtlebot3_simulations/turtlebot3_gazebo/launch/turtlebot3_avoidance.launch.py#L36)는 현재 2~5개 asset과 고정 x lane을 seed로 선택한다. PATCH-08에서는 한 C++ manager가 모든 사람을 함께 관리한다.

| 항목 | 확정값 |
|---|---|
| 사람 수 | seed 기반 `2–5명` |
| 속도 | entity별 `0.5–1.4 m/s` |
| 경로 | aisle centerline node/edge graph |
| 생성 | node 사이 최소간격 확인 후 배치 |
| 충돌 방지 | node와 edge reservation |
| 재현 | 개발 seed와 unseen 평가 seed 분리 |

한 entity가 다음 edge를 예약하지 못하면 현재 node 앞에서 정지하고 재시도한다. 가까운 두 사람의 collision을 pose 보정으로 숨기지 않는다. 기존 `crossing` 사람+cart scenario는 deterministic 회귀시험으로 유지한다.

### 6. robot velocity

`/cmd_vel_mux`는 요구속도다. CPA에는 `/odom.twist`의 실제 추정속도를 사용한다. `Odometry.twist`가 `base_link` 기준이면 robot yaw로 `odom`에 회전한다.

$$
v_R^{odom}=R_z(\psi_{odom\leftarrow base})v_R^{base}
$$

$v_R^{base}$는 `/odom.twist.twist.linear`의 m/s 값, $\psi$는 `odom`에서 본 robot yaw, $v_R^{odom}$은 사람 velocity와 비교할 robot velocity다. `/odom`은 simulation GT가 아니라 wheel/physics 기반 추정값이다.

### 7. CPA

사람과 robot의 상대 위치와 속도:

$$
\Delta P=P_P-P_R,\qquad \Delta V=V_P-V_R
$$

가장 가까워질 시간:

$$
t_{cpa}=\operatorname{clamp}\left(
-\frac{\Delta P\cdot\Delta V}{\Delta V\cdot\Delta V},
0,t_{horizon}\right)
$$

$\Delta V\cdot\Delta V$가 거의 0이면 상대위치가 변하지 않으므로 $t_{cpa}=0$으로 둔다. 모든 값은 `odom` 평면 기준이다.

CPA 거리:

$$
d_{cpa}=\lVert\Delta P+\Delta Vt_{cpa}\rVert_2
$$

$t_{cpa}$가 작고 $d_{cpa}$가 안전거리보다 작으면 가까운 미래에 robot과 사람의 보호원이 겹친다는 의미다.

### 8. safety distance

$$
r_P=0.35+r_{uncertainty}
$$

$$
d_{safe}=r_R+r_P+margin
$$

$r_P$는 사람 기본반경과 PATCH-07 Kalman uncertainty의 합, $r_R$은 robot footprint를 감싸는 반경, `margin`은 제동·timestamp·model mismatch 여유다. 실제 사람 몸집을 추정한다고 주장하지 않는다. **`0.35 m`는 사람 주위에 확보할 기본 보호반경**이다.

### 9. 세 단계 command

| 상태 | 조건 | output |
|---|---|---|
| safe | 모든 사람 `t_cpa>4 s` 또는 `d_cpa>d_safe` | mux command 그대로 |
| caution | 가장 위험한 사람의 `2<t_cpa≤4 s`, `d_cpa≤d_safe` | linear·angular 모두 `0.5` 배율 |
| danger | `0≤t_cpa≤2 s`, `d_cpa≤d_safe` | linear·angular `0` |

Linear와 angular를 같은 비율로 줄여 Nav2가 요청한 경로 곡률을 유지한다. 모든 track을 계산하고 가장 위험한 track의 상태를 최종 상태로 사용한다.

### 10. LiDAR emergency

LiDAR emergency는 semantic 상태보다 우선한다. 사람 검출 여부와 무관하게 방향별 minimum range가 threshold 이하면 즉시 정지한다.

| command 방향 | 검사 영역 |
|---|---|
| `linear.x>0` | front sector |
| `linear.x<0` | rear sector |
| `angular.z>0` | left/front swept sector |
| `angular.z<0` | right/front swept sector |

단일 전방 minimum만 사용하지 않는다. 회전 중 robot footprint가 쓸고 지나가는 측면을 함께 검사한다.

### 11. stale와 output watchdog

| 입력 | stale 기준 | 동작 |
|---|---:|---|
| `/scan` | `0.2 s` | 즉시 정지 |
| `/odom` | `0.2 s` | CPA 계산 불가, 즉시 정지 |
| `/tracking/persons` | `0.5 s` | LiDAR emergency 유지, 최고 `0.15 m/s` 제한 |

Semantic safety는 `20 Hz`로 `/cmd_vel_safe`에 새 `TwistStamped`를 발행한다. `forks/turtlebot3_simulations/turtlebot3_gazebo/models/turtlebot3_waffle_pi_3d_large/model.sdf` | [`DiffDrive` 설정](../../forks/turtlebot3_simulations/turtlebot3_gazebo/models/turtlebot3_waffle_pi_3d_large/model.sdf#L448)에는 command timeout 항목이 없으므로 별도 watchdog이 이를 받아 `/cmd_vel`로 전달한다. `/cmd_vel_safe`가 `0.3 s` 끊기면 watchdog은 zero command를 반복 발행한다. **Semantic safety가 비정상 종료되어도 Gazebo에 마지막 이동 command가 남지 않아야 한다.**

Danger, emergency 또는 stale 정지 후 모든 조건이 safe인 상태가 `1 s` 연속 유지되어야 다시 이동한다. 한 frame safe만으로 출발하지 않는다.

### 12. debug output

| topic | type | 내용 |
|---|---|---|
| `/safety/state` | `diagnostic_msgs/DiagnosticArray` | safe/caution/danger, stale source, selected track ID |
| `/safety/markers` | `visualization_msgs/MarkerArray` | robot/person velocity, CPA point, safety circle |
| `/cmd_vel_mux` | `geometry_msgs/TwistStamped` | safety 전 command |
| `/cmd_vel_safe` | `geometry_msgs/TwistStamped` | semantic safety 출력 |
| `/cmd_vel` | `geometry_msgs/TwistStamped` | watchdog을 통과해 robot에 전달된 최종 command |

RViz에서는 `odom` frame으로 `/tracking/markers`와 `/safety/markers`를 함께 표시한다. Rerun에는 `/odom`, track, CPA, command와 state transition을 같은 timestamp 축으로 기록한다.

## What was problem

| 현재 기준선 | 제한 | 설계 변경 |
|---|---|---|
| 자체 전진/회전 controller | 목표점과 global route 없음 | Nav2 waypoint patrol |
| `/scan` minimum만 사용 | 접근속도와 사람 identity 없음 | tracked-person CPA 추가 |
| Camera 결과가 command에 영향 없음 | semantic fusion이 시각화에서 끝남 | mux 뒤 semantic safety gate |
| multi-person 고정 평행 lane | random patrol·교차·가림 평가 부족 | aisle graph random walk + reservation |
| 실행 영상 중심 검증 | 개선 여부를 수치로 비교 불가 | unseen seed와 자동 evaluator |

## How it changed

### 구현 순서

1. 정적 warehouse에서 SLAM Toolbox로 map을 만들고 저장한다.
2. 저장 map, AMCL, Nav2로 사람 없는 3-lap patrol을 통과한다.
3. `twist_mux`와 semantic safety를 추가하되 사람 입력을 비워 LiDAR/stale test를 통과한다.
4. PATCH-07 `/tracking/persons`를 연결하고 deterministic crossing을 회귀시험한다.
5. pedestrian manager와 aisle graph를 추가한다.
6. 개발 seed로 threshold를 조정하고 unseen seed 20개 이상을 한 번만 평가한다.

### 실행

먼저 변경된 message·C++ node·Python launch·Gazebo plugin을 같은 overlay에 build한다.

```bash
cd /home/swlinux/Desktop/workspace/mobin/docker
docker compose run --rm shell bash -lc '
  source /opt/ros/jazzy/setup.bash
  colcon build --symlink-install
'
```

정적 warehouse map을 처음 만들 때는 사람·perception·safety·patrol을 끈다. 별도 terminal에서 `/cmd_vel`로 robot을 천천히 움직여 warehouse를 스캔한 뒤 map을 저장한다.

```bash
WAREHOUSE_SCENARIO=none SLAM=true RUN_PATROL=false \
RUN_EVALUATOR=false RUN_PERCEPTION=false RUN_SAFETY=false \
GAZEBO_GUI=true LAUNCH_RVIZ=true \
docker compose -f compose.yaml -f compose.nvidia.yaml up -d patrol

docker compose -f compose.yaml -f compose.nvidia.yaml exec patrol \
  /ws/scripts/save-warehouse-map.sh /ws/data/maps/small_warehouse
```

저장된 map과 AMCL로 2~5명 random walk·Nav2 patrol·semantic safety를 함께 실행한다.

```bash
PEDESTRIAN_COUNT=3 PEDESTRIAN_RANDOM_SEED=42 \
SLAM=false NAV2_MAP=/ws/data/maps/small_warehouse.yaml \
YOLO_DEVICE=0 GAZEBO_GUI=true LAUNCH_RVIZ=true \
docker compose -f compose.yaml -f compose.nvidia.yaml \
  up --force-recreate patrol
```

개발 seed 조정이 끝난 뒤에만 unseen seed 20개를 자동 실행한다.

```bash
cd /home/swlinux/Desktop/workspace/mobin
RUN_COUNT=20 SEED_START=1001 RUN_TIMEOUT_SECONDS=300 \
  ./code/scripts/run-warehouse-evaluation.sh
```

### test matrix

| test | 입력 | 합격 조건 |
|---|---|---|
| diverging | 사람과 robot이 멀어짐 | safe 유지 |
| crossing risk | `t_cpa=1.5 s`, `d_cpa<d_safe` | danger 정지 |
| near but parallel | 현재거리는 가깝지만 평행 이동 | CPA 조건에 따른 정상 판정 |
| scan stale | 0.21 s 무수신 | 정지 |
| odom stale | 0.21 s 무수신 | 정지 |
| tracking stale | 0.51 s 무수신, scan 정상 | `0.15 m/s` 제한 |
| stable-safe | danger 해제 후 0.99 s | 계속 정지 |
| watchdog | safety node 종료 | 0.3 s 안에 base 정지 |
| teleop release | teleop 종료 | 1 s 뒤 Nav2 재개 |

### 평가 기준

| 지표 | 합격 기준 |
|---|---:|
| unseen seed | `20개 이상` |
| 충돌 | `0회` |
| patrol 완주율 | `≥95%` |
| `d_safe` 위반 | `0회` |
| stale fail-safe | 주입 시험 `100%` 작동 |
| tracking 지표 | PATCH-07 baseline 대비 개선율 보고 |

평가는 3 laps 또는 5분 중 먼저 도달한 시점까지 수행한다. Gazebo entity pose는 evaluator만 읽고 runtime safety node에는 전달하지 않는다.

## 완료 조건

### 현재 검증 증거

| 검증 | 결과 |
|---|---|
| `semantic_safety_test` | CPA, stale, emergency, stable-safe core 통과 |
| semantic safety launch | `/cmd_vel_mux`, `/cmd_vel_safe`가 모두 `TwistStamped`임을 확인 |
| seed `42`, 3명 spawn | `casual_female`, `scrubs`, `malevisitoronphone` entity 생성 성공 |
| pedestrian pose 3초 비교 | node·edge 동시 예약 수정 후 첫 사람 pose가 `(-2.500,0.524)`에서 `(-2.500,0.399)`로 변해 random walk plugin 작동 확인 |
| 미완료 | 정적 map 생성, AMCL localization, 3-lap patrol, unseen seed 20회 평가는 실제 map 생성 뒤 수행 |

- 저장 map과 AMCL로 warehouse localization 재현
- 고정 `map` waypoint를 사람 없이 3 laps 완주
- teleop 100/Nav2 10 priority와 1 s handback 검증
- CPA + uncertainty + direction-dependent LiDAR emergency test 통과
- `/scan`·`/odom`·tracking stale fault injection 통과
- deterministic crossing 회귀시험 유지
- 2~5명 aisle random walk에서 collision 없이 entity spacing 유지
- unseen seed 20개 이상에서 충돌 0회, 완주율 95% 이상
