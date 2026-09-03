#!/usr/bin/env bash
# Satu-terminal: VMX + LiDAR + odometri + controller + SLAM + keyboard.
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
YDLIDAR_ROOT="/home/vmx/ydlidar_ros2_ws"
YDLIDAR_PARAMS="$YDLIDAR_ROOT/src/ydlidar_ros2_driver/params/Tmini.yaml"
HARDWARE_EXEC="$PROJECT_ROOT/install/studica_control/lib/studica_control/manual_composition"
HARDWARE_CONFIG="$PROJECT_ROOT/src/studica_control/config/titan_m1_test.yaml"
ODOM_NODE="$PROJECT_ROOT/src/studica_control/src/components/examples/python/wheel_odometry.py"
ODOM_PARAMS="$PROJECT_ROOT/src/studica_control/config/wheel_odometry.yaml"
DRIVE_NODE="$PROJECT_ROOT/src/studica_control/src/components/examples/python/drive_controller.py"
DRIVE_CONFIG="$PROJECT_ROOT/src/studica_control/config/drive_controller.yaml"
TELEOP_NODE="$PROJECT_ROOT/src/studica_control/src/components/examples/python/titan_keyboard_teleop.py"
SLAM_PARAMS="$PROJECT_ROOT/src/studica_control/config/slam_toolbox.yaml"

source /opt/ros/humble/setup.bash
source "$YDLIDAR_ROOT/install/setup.bash"
source "$PROJECT_ROOT/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
set -u

cleanup() {
  trap - EXIT INT TERM
  echo "Menghentikan mapping dan motor..."
  for motor in 0 1 2 3; do
    timeout 1 ros2 service call /titan0/titan_cmd studica_control/srv/SetData \
      "{params: 'set_speed', initparams: {n_encoder: $motor, speed: 0.0}}" \
      >/dev/null 2>&1 || true
  done
  timeout 2 ros2 service call /titan0/titan_cmd studica_control/srv/SetData \
    "{params: 'disable'}" >/dev/null 2>&1 || true
  kill "${SLAM_PID:-}" "${DRIVE_PID:-}" "${ODOM_PID:-}" \
       "${LIDAR_PID:-}" "${HARDWARE_PID:-}" 2>/dev/null || true
  pkill -TERM -f '[y]dlidar_ros2_driver_node' 2>/dev/null || true
  pkill -TERM -f '[s]tatic_transform_publisher.*laser_frame' 2>/dev/null || true
  sudo pkill -TERM -f "$HARDWARE_EXEC" 2>/dev/null || true
  wait "${SLAM_PID:-}" "${DRIVE_PID:-}" "${ODOM_PID:-}" \
       "${LIDAR_PID:-}" "${HARDWARE_PID:-}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "Menjalankan LiDAR..."
ros2 launch ydlidar_ros2_driver ydlidar_launch.py \
  params_file:="$YDLIDAR_PARAMS" &
LIDAR_PID=$!
if ! timeout 45 ros2 topic echo --no-daemon --spin-time 2 \
    --qos-profile sensor_data --once /scan sensor_msgs/msg/LaserScan \
    >/dev/null 2>&1; then
  echo "ERROR: /scan tidak menghasilkan data." >&2
  exit 1
fi

echo "Menjalankan VMX dan Titan..."
sudo -E env RMW_IMPLEMENTATION=rmw_cyclonedds_cpp "$HARDWARE_EXEC" \
  --ros-args -r __node:=control_server --params-file "$HARDWARE_CONFIG" &
HARDWARE_PID=$!
sleep 3
bash "$PROJECT_ROOT/scripts/init_titan_encoders.sh"
ros2 service call /titan0/titan_cmd studica_control/srv/SetData \
  "{params: 'enable'}"

python3 "$ODOM_NODE" --ros-args --params-file "$ODOM_PARAMS" &
ODOM_PID=$!
for _ in {1..15}; do
  ros2 service list 2>/dev/null | grep -Fxq /wheel_odometry/reset && break
  sleep 1
done
if ! ros2 service list 2>/dev/null | grep -Fxq /wheel_odometry/reset; then
  echo "ERROR: reset odometri tidak tersedia." >&2
  exit 1
fi
ros2 service call /wheel_odometry/reset std_srvs/srv/Empty "{}"

python3 "$DRIVE_NODE" --config "$DRIVE_CONFIG" &
DRIVE_PID=$!
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=false \
  slam_params_file:="$SLAM_PARAMS" &
SLAM_PID=$!

echo "Menunggu peta /map..."
if ! timeout 30 ros2 topic echo --no-daemon --spin-time 2 --once \
    /map nav_msgs/msg/OccupancyGrid >/dev/null 2>&1; then
  echo "ERROR: SLAM tidak menghasilkan /map." >&2
  exit 1
fi

echo
echo "MAPPING SIAP: W maju | S mundur | A kiri | D kanan | E stop | Q selesai"
echo "Gerakkan perlahan mengelilingi tepi dan koridor, lalu kembali ke S/home."
echo "Simpan dari terminal kedua: ./scripts/save_map.sh arena_map"
python3 "$TELEOP_NODE" --linear-speed 0.12 --angular-speed 0.55 \
  --release-timeout 0.35
