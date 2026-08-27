#!/usr/bin/env bash
set -euo pipefail

output="${1:-/ws/data/maps/small_warehouse}"
mkdir -p "$(dirname "$output")"

source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash
ros2 run nav2_map_server map_saver_cli -f "$output"
