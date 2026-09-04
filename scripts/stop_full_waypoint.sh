#!/usr/bin/env bash
# Emergency shutdown untuk stack waypoint satu-terminal.
set -o pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"

source /opt/ros/humble/setup.bash
source "$PROJECT_ROOT/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

echo "Mengirim STOP navigator dan disable Titan..."
timeout 3 ros2 service call /waypoint_navigator/stop \
  std_srvs/srv/Trigger "{}" >/dev/null 2>&1 || true
timeout 3 ros2 service call /titan0/titan_cmd \
  studica_control/srv/SetData "{params: 'disable'}" \
  >/dev/null 2>&1 || true
timeout 3 ros2 service call /titan1/titan_cmd \
  studica_control/srv/SetData "{params: 'disable'}" \
  >/dev/null 2>&1 || true

echo "Menghentikan proses waypoint..."
pkill -TERM -f "$PROJECT_ROOT/scripts/start_full_keyboard.sh" 2>/dev/null || true
pkill -TERM -f "$PROJECT_ROOT/src/studica_control/src/components/examples/python/titan_keyboard_teleop.py" 2>/dev/null || true
pkill -TERM -f "$PROJECT_ROOT/scripts/start_full_navigation.sh" 2>/dev/null || true
pkill -TERM -f "$PROJECT_ROOT/scripts/start_full_waypoint.sh" 2>/dev/null || true
pkill -TERM -f "$PROJECT_ROOT/scripts/start_waypoint_mode.sh" 2>/dev/null || true
pkill -TERM -f "$PROJECT_ROOT/src/studica_control/src/components/examples/python/waypoint_navigator.py" 2>/dev/null || true
pkill -TERM -f "$PROJECT_ROOT/src/studica_control/src/components/examples/python/drive_controller.py" 2>/dev/null || true
pkill -TERM -f "$PROJECT_ROOT/src/studica_control/src/components/examples/python/wheel_odometry.py" 2>/dev/null || true
pkill -TERM -f "$PROJECT_ROOT/src/studica_control/src/components/examples/python/front_scan_filter.py" 2>/dev/null || true
pkill -TERM -f "$PROJECT_ROOT/src/studica_control/src/components/examples/python/nav2_waypoint_runner.py" 2>/dev/null || true

echo "Menghentikan driver YDLIDAR..."
pkill -TERM -f 'ydlidar_ros2_driver.*ydlidar_launch.py' 2>/dev/null || true
pkill -TERM -f 'ydlidar_ros2_driver_node' 2>/dev/null || true
pkill -TERM -f 'static_transform_publisher.*laser_frame' 2>/dev/null || true

echo "Menghentikan hardware VMX..."
sudo pkill -TERM -f "$PROJECT_ROOT/install/studica_control/lib/studica_control/manual_composition" \
  2>/dev/null || true
sleep 2

if pgrep -af \
    'start_full_keyboard|titan_keyboard_teleop.py|start_full_waypoint|start_waypoint_mode|manual_composition|waypoint_navigator.py|drive_controller.py|wheel_odometry.py|ydlidar_ros2_driver_node'; then
  echo "WARNING: masih ada proses di atas. Jalankan skrip ini sekali lagi." >&2
  exit 1
fi

echo "Semua proses waypoint dan hardware sudah berhenti."
