#!/usr/bin/env bash

# 실행 중인 SLAM Toolbox의 occupancy grid를 Nav2가 읽는 warehouse map 파일로 저장한다.
# 첫 번째 인자는 container 내부 출력 prefix이며 기본값은 /ws/data/maps/small_warehouse다.
# ROS 2 overlay를 source한 뒤 map_saver_cli로 YAML과 image 파일을 같은 prefix에 생성한다.
# `/map`의 transient-local message가 발행 중이어야 하며 10초 안에 받지 못하면 실패한다.
# 주요 입력:
# - `$1`: 확장자를 제외한 출력 prefix. 선택값, 기본값은 `/ws/data/maps/small_warehouse`다.
# - `<prefix>.yaml`과 occupancy image가 생성되므로 Docker container 안의 writable 경로를 지정한다.

set -eo pipefail

output="${1:-/ws/data/maps/small_warehouse}"
mkdir -p "$(dirname "$output")"

source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash
set -u
ros2 run nav2_map_server map_saver_cli -f "$output" --ros-args \
  -p map_subscribe_transient_local:=true \
  -p save_map_timeout:=10.0
