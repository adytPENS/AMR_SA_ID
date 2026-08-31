#!/usr/bin/env bash
set -eo pipefail

PROJECT_ROOT="/home/vmx/studica_ws"
ODOM_NODE="$PROJECT_ROOT/src/studica_control/src/components/examples/python/wheel_odometry.py"
ODOM_PARAMS="$PROJECT_ROOT/src/studica_control/config/wheel_odometry.yaml"
SLAM_PARAMS="$PROJECT_ROOT/src/studica_control/config/slam_toolbox.yaml"

source /opt/ros/humble/setup.bash
source "$PROJECT_ROOT/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
set -u

cleanup() {
  echo "Menghentikan wheel odometry dan SLAM Toolbox..."
  kill "${SLAM_PID:-}" "${ODOM_PID:-}" 2>/dev/null || true
  wait "${SLAM_PID:-}" "${ODOM_PID:-}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

TOPIC_LIST="$(timeout 15 ros2 topic list)"

if ! grep -Fxq '/scan' <<<"$TOPIC_LIST"; then
  echo "ERROR: /scan belum tersedia. Jalankan driver YDLIDAR dahulu." >&2
  exit 1
fi

for motor in 0 1 2 3; do
  if ! grep -Fxq "/titan0/m_${motor}/encoder" <<<"$TOPIC_LIST"; then
    echo "ERROR: encoder M${motor} belum tersedia. Jalankan control_server dahulu." >&2
    exit 1
  fi
done

python3 "$ODOM_NODE" --ros-args --params-file "$ODOM_PARAMS" &
ODOM_PID=$!

sleep 2
if ! kill -0 "$ODOM_PID" 2>/dev/null; then
  echo "ERROR: wheel_odometry gagal berjalan." >&2
  exit 1
fi

ros2 launch slam_toolbox online_async_launch.py \
  use_sim_time:=false \
  slam_params_file:="$SLAM_PARAMS" &
SLAM_PID=$!

echo "Mapping aktif. Pastikan tersedia: map -> odom -> base_link -> laser_frame"
echo "Tekan Ctrl+C untuk menghentikan mapping."
wait "$SLAM_PID"
