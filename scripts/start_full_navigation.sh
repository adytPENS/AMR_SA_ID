#!/usr/bin/env bash
# Navigasi kompetisi satu-terminal: static map + AMCL + Nav2 + tombol/lampu.
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
YDLIDAR_ROOT="/home/vmx/ydlidar_ros2_ws"
YDLIDAR_PARAMS="$YDLIDAR_ROOT/src/ydlidar_ros2_driver/params/Tmini.yaml"
HARDWARE_EXEC="$PROJECT_ROOT/install/studica_control/lib/studica_control/manual_composition"
HARDWARE_CONFIG="$PROJECT_ROOT/src/studica_control/config/titan_m1_test.yaml"
NAV_CONFIG="${1:-$PROJECT_ROOT/src/studica_control/config/navigation_waypoints.yaml}"
NAV2_PARAMS="$PROJECT_ROOT/src/studica_control/config/nav2_navigation.yaml"
ODOM_NODE="$PROJECT_ROOT/src/studica_control/src/components/examples/python/wheel_odometry.py"
ODOM_PARAMS="$PROJECT_ROOT/src/studica_control/config/wheel_odometry.yaml"
DRIVE_NODE="$PROJECT_ROOT/src/studica_control/src/components/examples/python/drive_controller.py"
DRIVE_CONFIG="$PROJECT_ROOT/src/studica_control/config/drive_controller.yaml"
FILTER_NODE="$PROJECT_ROOT/src/studica_control/src/components/examples/python/front_scan_filter.py"
RUNNER_NODE="$PROJECT_ROOT/src/studica_control/src/components/examples/python/nav2_waypoint_runner.py"

source /opt/ros/humble/setup.bash
source "$YDLIDAR_ROOT/install/setup.bash"
source "$PROJECT_ROOT/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
set -u

readarray -t NAV_VALUES < <(python3 - "$NAV_CONFIG" <<'PY'
import sys, yaml
with open(sys.argv[1], encoding='utf-8') as f: c=yaml.safe_load(f) or {}
print('true' if c.get('configured', False) else 'false')
print(c.get('map_file', ''))
PY
)
if [[ "${NAV_VALUES[0]}" != "true" ]]; then
  echo "ERROR: configured=false pada $NAV_CONFIG" >&2
  echo "Isi HOME/waypoint/sequence, lalu ubah menjadi configured: true." >&2
  exit 1
fi
MAP_FILE="${NAV_VALUES[1]}"
if [[ ! -f "$MAP_FILE" ]]; then
  echo "ERROR: map tidak ditemukan: $MAP_FILE" >&2
  exit 1
fi

cleanup() {
  trap - EXIT INT TERM
  echo "Menghentikan navigasi kompetisi dan motor..."
  timeout 2 ros2 service call /competition_navigation/stop \
    std_srvs/srv/Trigger "{}" >/dev/null 2>&1 || true
  for motor in 0 1 2 3; do
    timeout 1 ros2 service call /titan0/titan_cmd studica_control/srv/SetData \
      "{params: 'set_speed', initparams: {n_encoder: $motor, speed: 0.0}}" \
      >/dev/null 2>&1 || true
  done
  timeout 2 ros2 service call /titan0/titan_cmd studica_control/srv/SetData \
    "{params: 'disable'}" >/dev/null 2>&1 || true
  kill "${RUNNER_PID:-}" "${NAV2_PID:-}" "${FILTER_PID:-}" \
       "${DRIVE_PID:-}" "${ODOM_PID:-}" "${LIDAR_PID:-}" \
       "${HARDWARE_PID:-}" 2>/dev/null || true
  pkill -TERM -f '[y]dlidar_ros2_driver_node' 2>/dev/null || true
  sudo pkill -TERM -f "$HARDWARE_EXEC" 2>/dev/null || true
  wait "${RUNNER_PID:-}" "${NAV2_PID:-}" "${FILTER_PID:-}" \
       "${DRIVE_PID:-}" "${ODOM_PID:-}" "${LIDAR_PID:-}" \
       "${HARDWARE_PID:-}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "Menjalankan LiDAR dan filter depan -80..+80 derajat..."
ros2 launch ydlidar_ros2_driver ydlidar_launch.py \
  params_file:="$YDLIDAR_PARAMS" &
LIDAR_PID=$!
python3 "$FILTER_NODE" &
FILTER_PID=$!
if ! timeout 45 ros2 topic echo --no-daemon --spin-time 2 \
    --qos-profile sensor_data --once /scan_front sensor_msgs/msg/LaserScan \
    >/dev/null 2>&1; then
  echo "ERROR: /scan_front tidak menghasilkan data." >&2
  exit 1
fi

echo "Menjalankan hardware VMX..."
sudo -E env RMW_IMPLEMENTATION=rmw_cyclonedds_cpp "$HARDWARE_EXEC" \
  --ros-args -r __node:=control_server --params-file "$HARDWARE_CONFIG" &
HARDWARE_PID=$!
sleep 3
bash "$PROJECT_ROOT/scripts/init_titan_encoders.sh"
ros2 service call /titan0/titan_cmd studica_control/srv/SetData \
  "{params: 'enable'}"

python3 "$ODOM_NODE" --ros-args --params-file "$ODOM_PARAMS" &
ODOM_PID=$!
python3 "$DRIVE_NODE" --config "$DRIVE_CONFIG" &
DRIVE_PID=$!

echo "Menjalankan Map Server, AMCL, planner A*, dan controller DWB..."
ros2 launch nav2_bringup bringup_launch.py \
  map:="$MAP_FILE" params_file:="$NAV2_PARAMS" \
  use_sim_time:=false autostart:=true use_composition:=False &
NAV2_PID=$!

echo "Menunggu Nav2 /navigate_to_pose (maksimal 45 detik)..."
READY=false
for _ in {1..45}; do
  if ros2 action list 2>/dev/null | grep -Fxq /navigate_to_pose; then
    READY=true
    break
  fi
  sleep 1
done
if [[ "$READY" != "true" ]]; then
  echo "ERROR: Nav2 tidak siap." >&2
  exit 1
fi

python3 "$RUNNER_NODE" --config "$NAV_CONFIG" &
RUNNER_PID=$!
sleep 2
if ! kill -0 "$RUNNER_PID" 2>/dev/null; then
  echo "ERROR: competition waypoint runner gagal dibuat." >&2
  exit 1
fi

echo
echo "NAVIGASI SIAP — motor STOP, lampu merah solid."
echo "Letakkan robot tepat di HOME, lalu tekan START DIO 10."
echo "STOP DIO 11 menghentikan perjalanan; Ctrl+C mematikan seluruh stack."
wait "$RUNNER_PID"
