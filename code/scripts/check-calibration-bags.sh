#!/usr/bin/env bash

# Calibration 전에 rosbag 디렉터리와 필수 Camera·LiDAR topic 구성을 검사한다.
# 첫 번째 인자로 bag root를 받고, 생략하면 repository의 data/bags를 사용한다.
# Calibration Docker image에서 각 metadata.yaml을 읽으며 원본 bag은 read-only로 mount한다.
# 유효 bag이 5개 미만이거나 필수 topic/type이 하나라도 없으면 실패로 종료한다.
# 주요 입력:
# - `$1`: 검사할 bag root directory. 선택값, 기본값은 `<repository>/data/bags`다.
# - `CALIB_IMAGE`: `ros2 bag info`를 실행할 Docker image. 기본값은 `direct-visual-lidar-calibration:fork`다.

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
