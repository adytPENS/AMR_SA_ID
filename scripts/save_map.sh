#!/usr/bin/env bash
set -eo pipefail

PROJECT_ROOT="/home/vmx/studica_ws"
MAP_NAME="${1:-arena_map}"
MAP_PATH="$PROJECT_ROOT/maps/$MAP_NAME"

source /opt/ros/humble/setup.bash
source "$PROJECT_ROOT/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
set -u

mkdir -p "$PROJECT_ROOT/maps"
ros2 run nav2_map_server map_saver_cli -f "$MAP_PATH"

echo "Peta disimpan sebagai:"
echo "  ${MAP_PATH}.yaml"
echo "  ${MAP_PATH}.pgm"
