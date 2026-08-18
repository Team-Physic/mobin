#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
DOCKER_DIR="${ROOT_DIR}/docker"
BAGS_DIR="${ROOT_DIR}/data/bags"
RECORD_SECONDS="${1:-${RECORD_SECONDS:-15}}"
GAZEBO_GUI="${GAZEBO_GUI:-false}"
LAUNCH_RVIZ="${LAUNCH_RVIZ:-false}"

COMPOSE=(
  docker compose
  -f "${DOCKER_DIR}/compose.yaml"
  -f "${DOCKER_DIR}/compose.nvidia.yaml"
)

POSES=(
  "pose-01 0.00 0.00 0.00"
  "pose-02 0.20 -0.35 0.10"
  "pose-03 -0.15 0.35 -0.10"
  "pose-04 0.45 0.15 0.18"
  "pose-05 0.35 -0.20 -0.18"
)

usage() {
  echo "Usage: $0 [record-seconds]"
  echo "Default: 15 seconds per pose"
  echo "Optional env: GAZEBO_GUI=true LAUNCH_RVIZ=true"
}

if [[ "${RECORD_SECONDS}" == "-h" || "${RECORD_SECONDS}" == "--help" ]]; then
  usage
  exit 0
fi

if [[ ! "${RECORD_SECONDS}" =~ ^[1-9][0-9]*$ ]]; then
  echo "ERROR: record-seconds must be a positive integer: ${RECORD_SECONDS}" >&2
  exit 2
fi

mkdir -p "${BAGS_DIR}"

for pose in "${POSES[@]}"; do
  read -r bag_name _ <<< "${pose}"
  if [[ -e "${BAGS_DIR}/${bag_name}" ]]; then
    echo "ERROR: bag already exists: ${BAGS_DIR}/${bag_name}" >&2
    echo "Move or rename the existing bag before recording." >&2
    exit 1
  fi
done

recording=false

cleanup() {
  if [[ "${recording}" == "true" ]]; then
    "${COMPOSE[@]}" exec -T sim \
      bash -lc "pkill -INT -f 'ros2 bag record' || true" >/dev/null 2>&1 || true
  fi
  "${COMPOSE[@]}" stop sim >/dev/null 2>&1 || true
}

trap cleanup EXIT
trap 'exit 130' INT TERM

wait_for_sensor_messages() {
  "${COMPOSE[@]}" exec -T sim bash -lc '
    # ROS setup scripts may read optional, unset AMENT_* variables.
    set -eo pipefail
    source /opt/ros/jazzy/setup.bash
    source /ws/install/setup.bash

    for topic in /calib/points /camera/image_raw /camera/camera_info; do
      echo "Waiting for ${topic} type ..."
      deadline=$((SECONDS + 60))
      until ros2 topic type "${topic}" >/dev/null 2>&1; do
        if (( SECONDS >= deadline )); then
          echo "ERROR: topic type was not discovered in 60 seconds: ${topic}" >&2
          exit 1
        fi
        sleep 1
      done

      echo "Waiting for ${topic} message ..."
      timeout 60 ros2 topic echo "${topic}" --once \
        --qos-reliability reliable >/dev/null
    done
  '
}

record_pose() {
  local bag_name="$1"
  local x_pose="$2"
  local y_pose="$3"
  local yaw="$4"

  echo
  echo "[${bag_name}] pose=(${x_pose}, ${y_pose}, yaw=${yaw})"

  TURTLEBOT3_MODEL=waffle_pi_3d \
  TURTLEBOT3_WORLD=turtlebot3_calibration.world \
  X_POSE="${x_pose}" \
  Y_POSE="${y_pose}" \
  YAW="${yaw}" \
  GAZEBO_GUI="${GAZEBO_GUI}" \
  LAUNCH_RVIZ="${LAUNCH_RVIZ}" \
    "${COMPOSE[@]}" up -d --force-recreate sim

  wait_for_sensor_messages

  recording=true
  set +e
  "${COMPOSE[@]}" exec -T sim bash -lc \
    "source /opt/ros/jazzy/setup.bash &&
     source /ws/install/setup.bash &&
     timeout --signal=INT --kill-after=10s ${RECORD_SECONDS}s \
       ros2 bag record --storage mcap \
       -o /ws/data/bags/${bag_name} \
       /calib/points \
       /camera/image_raw \
       /camera/camera_info \
       /tf \
       /tf_static \
       /clock"
  local status=$?
  set -e
  recording=false

  if [[ ${status} -ne 124 ]]; then
    echo "ERROR: ros2 bag record failed for ${bag_name}: exit ${status}" >&2
    exit "${status}"
  fi

  if [[ ! -f "${BAGS_DIR}/${bag_name}/metadata.yaml" ]]; then
    echo "ERROR: metadata.yaml was not created for ${bag_name}" >&2
    exit 1
  fi

  echo "[${bag_name}] saved: ${BAGS_DIR}/${bag_name}"
}

for pose in "${POSES[@]}"; do
  read -r bag_name x_pose y_pose yaw <<< "${pose}"
  record_pose "${bag_name}" "${x_pose}" "${y_pose}" "${yaw}"
done

echo
echo "Recorded all calibration bags in ${BAGS_DIR}"
