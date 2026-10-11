#!/usr/bin/env bash

# Portfolio용 regression, crossing, multi-person tracking, safety trial을 고정 조건으로 실행한다.
# 첫 번째 인자로 네 trial 중 하나를 받고 Docker Compose build/test 또는 Gazebo runtime을 구성한다.
# 실행환경, image·plot, association 거부 원인, ROI summary·IDSW event를 trials/<trial-name>/에 저장한다.
# 기존 trial 디렉터리는 덮어쓰지 않으며 runtime 종료 시 이 repository의 Compose service를 내린다.
# 주요 입력:
# - `$1=regression-test`: ROS package build와 C++·Python test만 실행한다.
# - `$1=deterministic_crossing`: 사람+cart 1개가 교차하는 45초 runtime과 safety 전이를 기록한다.
# - `$1=seeded-multi-person-tracking`: seed 42의 사람 3명을 60초 추적하고 evaluator 결과를 기록한다.
# - `$1=safety-test`: Gazebo 없이 CPA·stale·emergency·stable-safe C++ test를 실행한다.
# - `YOLO_DEVICE`: `0`은 첫 CUDA GPU, `cpu`는 CPU inference다. 기본값은 `0`이다.
# - `YOLO_CONFIDENCE`: person detection 최소 confidence `0.0~1.0`. 기본값은 `0.25`다.
# - `GAZEBO_GUI`, `LAUNCH_RVIZ`: `true/false`로 각 GUI 실행 여부를 정한다. 기본값은 모두 `true`다.
# - `EVALUATION_MAX_TIME_DIFFERENCE_SECONDS=0.05`: GT·odom과 track의 최대 시간차다.
# - `EVALUATION_PERSON_REFERENCE_HEIGHT=0.9`: Gazebo model origin 위 torso 기준점 높이(m)다.
# - `EVALUATION_LIDAR_*`: 3D LiDAR SDF의 frame, 0.1~20 m 거리, 수평 360°·수직 ±15°다.

set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
trial_name="${1:-}"
case "${trial_name}" in
  regression-test|deterministic_crossing|seeded-multi-person-tracking|safety-test) ;;
  *)
    echo "usage: $0 {regression-test|deterministic_crossing|seeded-multi-person-tracking|safety-test}" >&2
    exit 2
    ;;
esac

trial_dir="${repo_dir}/trials/${trial_name}"
if [[ -e "${trial_dir}" ]]; then
  echo "ERROR: existing trial is never overwritten: ${trial_dir}" >&2
  exit 1
fi
mkdir -p "${trial_dir}"

{
  printf 'trial=%s\n' "${trial_name}"
  printf 'git_commit=%s\n' "$(git -C "${repo_dir}" rev-parse HEAD)"
  printf 'started_at=%s\n' "$(date --iso-8601=seconds)"
  uname -a
  lscpu | sed -n 's/^Model name:[[:space:]]*/cpu=/p'
  free -h | sed -n '2s/^/memory=/p'
  docker --version
  docker compose version
  nvidia-smi --query-gpu=name,driver_version --format=csv,noheader 2>/dev/null || printf 'gpu=unavailable\n'
} > "${trial_dir}/environment.txt"

cd "${repo_dir}/docker"
compose=(docker compose -f compose.yaml -f compose.nvidia.yaml)

run_regression() {
  "${compose[@]}" run --rm -T shell bash -lc '
    source /opt/ros/jazzy/setup.bash
    cd /ws
    colcon build --packages-select mobile_robot_lab_msgs mobile_robot_lab_cpp mobile_robot_lab_python --symlink-install
    source /ws/install/setup.bash
    colcon test --packages-select mobile_robot_lab_cpp mobile_robot_lab_python --event-handlers console_direct+
    colcon test-result --verbose
  ' 2>&1 | tee "${trial_dir}/test.log"
}

cleanup() {
  if [[ -n "${publisher_pid:-}" ]]; then
    kill "${publisher_pid}" >/dev/null 2>&1 || true
    wait "${publisher_pid}" >/dev/null 2>&1 || true
  fi
  # 같은 ROS/Gazebo domain의 이전 Mobin service가 sensor clock을 섞지 않도록 모두 정리한다.
  "${compose[@]}" down --timeout 15 >/dev/null 2>&1 || true
}

run_runtime() {
  local scenario="$1"
  local people="$2"
  local prefix="$3"
  local duration="$4"
  export TURTLEBOT3_MODEL=waffle_pi_3d_large
  export TURTLEBOT3_WORLD_DIR=/opt/aws_warehouse/worlds/small_warehouse
  export TURTLEBOT3_WORLD=small_warehouse_harmonic.world
  export WAREHOUSE_SCENARIO="${scenario}"
  export PEDESTRIAN_COUNT="${people}"
  export PEDESTRIAN_RANDOM_SEED=42
  export EVALUATION_PERSON_NAME_PREFIX="${prefix}"
  export EVALUATION_OUTPUT_PATH="/ws/trials/${trial_name}/summary.json"
  export EVALUATION_DURATION_SECONDS="${duration}"
  export EVALUATION_EXPECTED_PERSON_COUNT="${people}"
  export EVALUATION_MAX_TIME_DIFFERENCE_SECONDS=0.05
  export EVALUATION_PERSON_REFERENCE_HEIGHT=0.9
  export EVALUATION_LIDAR_FRAME=base_scan
  export EVALUATION_LIDAR_MINIMUM_RANGE=0.1
  export EVALUATION_LIDAR_MAXIMUM_RANGE=20.0
  export EVALUATION_LIDAR_HORIZONTAL_MINIMUM=-3.141593
  export EVALUATION_LIDAR_HORIZONTAL_MAXIMUM=3.141593
  export EVALUATION_LIDAR_VERTICAL_MINIMUM=-0.261799
  export EVALUATION_LIDAR_VERTICAL_MAXIMUM=0.261799
  export RUN_NAVIGATION=false
  export RUN_PATROL=false
  export RUN_EVALUATOR=true
  export RUN_PERCEPTION=true
  export RUN_SAFETY=true
  export PUBLISH_ANNOTATED_IMAGE=true
  export YOLO_DEVICE="${YOLO_DEVICE:-0}"
  export YOLO_CONFIDENCE="${YOLO_CONFIDENCE:-0.25}"
  export GAZEBO_GUI="${GAZEBO_GUI:-true}"
  export LAUNCH_RVIZ="${LAUNCH_RVIZ:-true}"
  export X_POSE=-2.0 Y_POSE=-0.5 YAW=0.0

  if [[ -n "${DISPLAY:-}" ]]; then
    xhost +si:localuser:root >/dev/null
  fi
  trap cleanup EXIT INT TERM
  cleanup
  "${compose[@]}" up -d --force-recreate patrol
  "${compose[@]}" run --rm -T shell bash -lc '
    source /opt/ros/jazzy/setup.bash
    source /ws/install/setup.bash
    for _ in $(seq 1 120); do
      ros2 topic type /safety/state >/dev/null 2>&1 && exit 0
      sleep 1
    done
    echo "ERROR: /safety/state did not start" >&2
    exit 1
  '

  "${compose[@]}" run --rm -T shell bash -lc '
    source /opt/ros/jazzy/setup.bash
    source /ws/install/setup.bash
    ros2 topic pub --rate 10 --times 900 --wait-matching-subscriptions 1 \
      /cmd_vel_nav geometry_msgs/msg/TwistStamped \
      "{twist: {linear: {x: 0.10}, angular: {z: 0.0}}}"
  ' > "${trial_dir}/command-publisher.log" 2>&1 &
  publisher_pid=$!

  timeout --signal=INT --kill-after=15 180 \
    "${compose[@]}" run --rm -T shell bash -lc "
      source /opt/ros/jazzy/setup.bash
      source /ws/install/setup.bash
      ros2 run mobile_robot_lab_python trial_evidence_recorder --ros-args \
        -p use_sim_time:=true \
        -p output_dir:=/ws/trials/${trial_name} \
        -p duration_seconds:=${duration}
    " 2>&1 | tee "${trial_dir}/recorder.log"
  cleanup
  trap - EXIT INT TERM
  python3 - "${trial_dir}" "${trial_name}" <<'PY'
import json
import pathlib
import sys

trial_dir = pathlib.Path(sys.argv[1])
trial_name = sys.argv[2]
evidence = json.loads((trial_dir / 'evidence.json').read_text())
summary_path = trial_dir / 'summary.json'
association_path = trial_dir / 'roi-associations.csv'
id_switch_path = trial_dir / 'id-switch-events.json'
errors = []
required_outcomes = {
    'yolo_detections', 'success', 'no_points',
    'multiple_depth_candidates', 'ground_rejection', 'tf_unavailable',
}
if not required_outcomes.issubset(evidence.get('association_outcomes', {})):
    errors.append('association outcome counters missing')
if not evidence['association_image_saved']:
    errors.append('successful LiDAR-bbox association image missing')
if not summary_path.exists():
    errors.append('tracking summary missing')
else:
    summary = json.loads(summary_path.read_text())
    if summary.get('frames', 0) <= 0 or summary.get('gt_detections', 0) <= 0:
        errors.append('tracking ground-truth frames missing')
    if '__all__' not in summary.get('sensor_coverage', {}):
        errors.append('sensor coverage missing')
if not association_path.exists():
    errors.append('ROI association CSV missing')
if not id_switch_path.exists():
    errors.append('ID switch event JSON missing')
if trial_name == 'deterministic_crossing' and not evidence['person_cpa_danger_seen']:
    errors.append('person_cpa_danger was not observed')
if trial_name == 'deterministic_crossing' and not evidence['stable_safe_hold_seen']:
    errors.append('stable_safe_hold was not observed')
if errors:
    raise SystemExit('TRIAL FAILED: ' + '; '.join(errors))
print('trial checks passed')
PY
}

case "${trial_name}" in
  regression-test)
    run_regression
    ;;
  deterministic_crossing)
    run_runtime crossing 1 avoidance_moving_cart 45.0
    ;;
  seeded-multi-person-tracking)
    run_runtime multi_person 3 avoidance_person_ 60.0
    ;;
  safety-test)
    "${compose[@]}" run --rm -T shell bash -lc '
      source /opt/ros/jazzy/setup.bash
      cd /ws
      colcon build --packages-select mobile_robot_lab_cpp --symlink-install
      source /ws/install/setup.bash
      /ws/build/mobile_robot_lab_cpp/semantic_safety_test
    ' 2>&1 | tee "${trial_dir}/safety-test.log"
    ;;
esac

printf 'saved: %s\n' "${trial_dir}"
