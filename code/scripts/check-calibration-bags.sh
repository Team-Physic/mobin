#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
BAGS_DIR=${1:-"$ROOT_DIR/data/bags"}
IMAGE=${CALIB_IMAGE:-direct-visual-lidar-calibration:fork}

test -d "$BAGS_DIR" || {
  echo "ERROR: bag directory not found: $BAGS_DIR" >&2
  exit 1
}

docker run --rm \
  -v "$BAGS_DIR:/bags:ro" \
  "$IMAGE" \
  bash -lc '
    set -euo pipefail
    count=0
    for bag in /bags/*; do
      test -f "$bag/metadata.yaml" || continue
      count=$((count + 1))
      info=$(ros2 bag info "$bag")
      echo "== $(basename "$bag") =="
      echo "$info"
      for expected in \
        "Topic: /calib/points | Type: sensor_msgs/msg/PointCloud2" \
        "Topic: /camera/image_raw | Type: sensor_msgs/msg/Image" \
        "Topic: /camera/camera_info | Type: sensor_msgs/msg/CameraInfo"; do
        grep -Fq "$expected" <<<"$info" || {
          echo "ERROR: $(basename "$bag") missing $expected" >&2
          exit 1
        }
      done
    done
    test "$count" -ge 5 || {
      echo "ERROR: expected at least 5 bags, found $count" >&2
      exit 1
    }
  '
