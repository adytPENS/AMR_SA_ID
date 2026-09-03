#!/usr/bin/env bash
# Membuka pasangan YAML/PGM map tersimpan di RViz dari desktop Raspberry Pi.
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
MAP_FILE="${1:-$PROJECT_ROOT/maps/Navigation.yaml}"
RVIZ_CONFIG="$PROJECT_ROOT/src/studica_control/config/navigation_map.rviz"

if [[ ! -f "$MAP_FILE" ]]; then
  echo "ERROR: map tidak ditemukan: $MAP_FILE" >&2
  exit 1
fi

source /opt/ros/humble/setup.bash
source "$PROJECT_ROOT/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

cleanup() {
  trap - EXIT INT TERM
  kill "${MAP_SERVER_PID:-}" 2>/dev/null || true
  pkill -TERM -f '[/]opt/ros/humble/lib/nav2_map_server/map_server' \
    2>/dev/null || true
  wait "${MAP_SERVER_PID:-}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# Hindari dua node /map_server dengan nama sama dari percobaan sebelumnya.
pkill -TERM -f '[/]opt/ros/humble/lib/nav2_map_server/map_server' \
  2>/dev/null || true
pkill -TERM -f '[/]opt/ros/humble/bin/ros2 run nav2_map_server map_server' \
  2>/dev/null || true
sleep 1

ros2 run nav2_map_server map_server --ros-args \
  -p yaml_filename:="$MAP_FILE" &
MAP_SERVER_PID=$!

for _ in {1..10}; do
  ros2 lifecycle nodes 2>/dev/null | grep -Fxq /map_server && break
  sleep 1
done

STATE="$(ros2 lifecycle get /map_server 2>/dev/null || true)"
if grep -q '^unconfigured' <<<"$STATE"; then
  ros2 lifecycle set /map_server configure
fi

STATE="$(ros2 lifecycle get /map_server 2>/dev/null || true)"
if grep -q '^inactive' <<<"$STATE"; then
  ros2 lifecycle set /map_server activate
fi

STATE="$(ros2 lifecycle get /map_server 2>/dev/null || true)"
if ! grep -q '^active' <<<"$STATE"; then
  echo "ERROR: map_server gagal menjadi active: ${STATE:-tidak tersedia}" >&2
  exit 1
fi

echo "Membuka peta di RViz (Fixed Frame=map, Topic=/map)..."
rviz2 -d "$RVIZ_CONFIG"
