#!/usr/bin/env bash
# Menjalankan uji trace dinding kiri dalam satu terminal.
set -eo pipefail

PROJECT_ROOT="/home/vmx/studica_ws"
YDLIDAR_ROOT="/home/vmx/ydlidar_ros2_ws"
HARDWARE="$PROJECT_ROOT/install/studica_control/lib/studica_control/manual_composition"
FOLLOWER="$PROJECT_ROOT/src/studica_control/src/components/examples/python/left_wall_follower.py"

source /opt/ros/humble/setup.bash
source "$YDLIDAR_ROOT/install/setup.bash"
source "$PROJECT_ROOT/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

cleanup() {
  trap - EXIT INT TERM
  timeout 2 ros2 service call /left_wall_follower/stop std_srvs/srv/Trigger "{}" >/dev/null 2>&1 || true
  timeout 2 ros2 service call /titan0/titan_cmd studica_control/srv/SetData "{params: 'disable'}" >/dev/null 2>&1 || true
  timeout 2 ros2 service call /titan1/titan_cmd studica_control/srv/SetData "{params: 'disable'}" >/dev/null 2>&1 || true
  kill "${FOLLOW_PID:-}" "${ODOM_PID:-}" "${DRIVE_PID:-}" "${LIDAR_PID:-}" "${HW_PID:-}" 2>/dev/null || true
  pkill -TERM -f '[y]dlidar_ros2_driver_node' 2>/dev/null || true
  sudo pkill -TERM -f "$HARDWARE" 2>/dev/null || true
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

ros2 launch ydlidar_ros2_driver ydlidar_launch.py params_file:="$YDLIDAR_ROOT/src/ydlidar_ros2_driver/params/Tmini.yaml" &
LIDAR_PID=$!
if ! timeout 45 ros2 topic echo --no-daemon --spin-time 2 --qos-profile sensor_data --once /scan sensor_msgs/msg/LaserScan >/dev/null 2>&1; then
  echo "ERROR: /scan LiDAR belum tersedia." >&2
  exit 1
fi

sudo -E env RMW_IMPLEMENTATION=rmw_cyclonedds_cpp "$HARDWARE" --ros-args -r __node:=control_server --params-file "$PROJECT_ROOT/src/studica_control/config/titan_m1_test.yaml" &
HW_PID=$!
sleep 3
bash "$PROJECT_ROOT/scripts/init_titan_encoders.sh"
ros2 service call /titan0/titan_cmd studica_control/srv/SetData "{params: 'enable'}"

python3 "$PROJECT_ROOT/src/studica_control/src/components/examples/python/drive_controller.py" --config "$PROJECT_ROOT/src/studica_control/config/drive_controller.yaml" &
DRIVE_PID=$!
python3 "$PROJECT_ROOT/src/studica_control/src/components/examples/python/wheel_odometry.py" --ros-args --params-file "$PROJECT_ROOT/src/studica_control/config/wheel_odometry.yaml" &
ODOM_PID=$!
python3 "$FOLLOWER" &
FOLLOW_PID=$!
sleep 3

echo "TRACE DINDING KIRI SIAP — motor STOP, lampu merah solid."
echo "Letakkan dinding di kiri robot sekitar 40 cm, lalu tekan START DIO 10."
echo "STOP DIO 11 atau Ctrl+C untuk berhenti."
wait "$FOLLOW_PID"
