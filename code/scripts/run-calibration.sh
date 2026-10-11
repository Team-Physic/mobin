#!/usr/bin/env bash

# Direct Visual–LiDAR Calibration의 preprocess, initial guess, optimize, viewer 단계를 실행한다.
# 첫 번째 인자로 단계를 선택하고 data/bags를 read-only 입력, data/results를 출력으로 사용한다.
# X11을 전달한 calibration Docker container에서 GUI를 열며 image·point topic 이름을 고정한다.
# 기존 calib.json을 자동 덮어쓰지 않으므로 각 단계의 결과를 확인하며 순서대로 실행해야 한다.
# 주요 입력:
# - `$1`: 필수 단계. `preprocess`, `initial`, `calibrate`, `viewer` 중 하나다.
# - `CALIB_IMAGE`: 실행할 Docker image. 기본값은 `direct-visual-lidar-calibration:fork`다.
# - `DISPLAY`: GUI를 표시할 X11 display. 기본값은 `:0`이다.
# - `LIBGL_ALWAYS_SOFTWARE`: `1`이면 CPU renderer, `0`이면 사용 가능한 GPU renderer를 요청한다. 기본값은 `1`이다.

set -euo pipefail

ROOT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
BAGS_DIR="$ROOT_DIR/data/bags"
RESULTS_DIR="$ROOT_DIR/data/results"
IMAGE=${CALIB_IMAGE:-direct-visual-lidar-calibration:fork}
STEP=${1:-}

usage() {
  echo "Usage: $0 {preprocess|initial|calibrate|viewer}"
}

test -n "$STEP" || { usage; exit 2; }
mkdir -p "$RESULTS_DIR"

docker_gui() {
  docker run --rm -it \
    --network host \
    -e DISPLAY="${DISPLAY:-:0}" \
    -e QT_X11_NO_MITSHM=1 \
    -e LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}" \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "$BAGS_DIR:/bags:ro" \
    -v "$RESULTS_DIR:/results:rw" \
    "$IMAGE" "$@"
}

case "$STEP" in
  preprocess)
    "$ROOT_DIR/code/scripts/check-calibration-bags.sh" "$BAGS_DIR"
    test ! -f "$RESULTS_DIR/calib.json" || {
      echo "ERROR: data/results/calib.json already exists; move it aside before preprocessing" >&2
      exit 1
    }
    docker_gui ros2 run direct_visual_lidar_calibration preprocess \
      /bags /results \
      --camera_info_topic /camera/camera_info \
      --image_topic /camera/image_raw \
      --points_topic /calib/points \
      --intensity_channel intensity \
      --visualize
    ;;
  initial)
    test -f "$RESULTS_DIR/calib.json" || {
      echo "ERROR: run preprocess first" >&2
      exit 1
    }
    docker_gui ros2 run direct_visual_lidar_calibration \
      initial_guess_manual /results
    ;;
  calibrate)
    test -f "$RESULTS_DIR/calib.json" || {
      echo "ERROR: run preprocess and initial first" >&2
      exit 1
    }
    docker_gui ros2 run direct_visual_lidar_calibration calibrate /results
    ;;
  viewer)
    test -f "$RESULTS_DIR/calib.json" || {
      echo "ERROR: no calibration result" >&2
      exit 1
    }
    docker_gui ros2 run direct_visual_lidar_calibration viewer /results
    ;;
  *)
    usage
    exit 2
    ;;
esac
