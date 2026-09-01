#!/usr/bin/env bash
# Menjalankan seluruh stack waypoint dalam satu terminal.
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
HARDWARE_EXEC="$PROJECT_ROOT/install/studica_control/lib/studica_control/manual_composition"
HARDWARE_CONFIG="$PROJECT_ROOT/src/studica_control/config/titan_m1_test.yaml"
WAYPOINT_CONFIG="${1:-$PROJECT_ROOT/src/studica_control/config/waypoints.yaml}"

source /opt/ros/humble/setup.bash
source "$PROJECT_ROOT/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
set -u

cleanup() {
  trap - EXIT INT TERM
  echo "Menghentikan seluruh mode waypoint..."
  timeout 2 ros2 service call /waypoint_navigator/stop \
    std_srvs/srv/Trigger "{}" >/dev/null 2>&1 || true
  timeout 2 ros2 service call /titan0/titan_cmd \
    studica_control/srv/SetData "{params: 'disable'}" \
    >/dev/null 2>&1 || true
  kill "${MODE_PID:-}" "${HARDWARE_PID:-}" 2>/dev/null || true
  wait "${MODE_PID:-}" "${HARDWARE_PID:-}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "Pastikan robot sudah berada di S/home dan menghadap arah start."
echo "Menjalankan hardware, IMU, Titan, dan output lampu..."
sudo -E env RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  "$HARDWARE_EXEC" \
  --ros-args \
  -r __node:=control_server \
  --params-file "$HARDWARE_CONFIG" &
HARDWARE_PID=$!

echo "Menunggu hardware stabil (IMU otomatis zero yaw saat dibuat)..."
sleep 3
if ! kill -0 "$HARDWARE_PID" 2>/dev/null; then
  echo "ERROR: proses hardware berhenti saat startup." >&2
  exit 1
fi

echo "Inisialisasi encoder M0-M3..."
bash "$PROJECT_ROOT/scripts/init_titan_encoders.sh"

echo "Mengaktifkan Titan..."
ros2 service call /titan0/titan_cmd studica_control/srv/SetData \
  "{params: 'enable'}"

echo "Menjalankan odometri, drive controller, dan navigator..."
bash "$PROJECT_ROOT/scripts/start_waypoint_mode.sh" "$WAYPOINT_CONFIG" &
MODE_PID=$!

echo "Menunggu odometri, drive controller, dan navigator selesai dibuat (20 detik)..."
# Pada VMX aktual, start_waypoint_mode membutuhkan sekitar 17 detik untuk
# discovery topic, reset odometri, lalu membuat subscriber tombol fisik.
# Jangan tampilkan prompt sebelum subscriber navigator benar-benar terbentuk.
sleep 20
if ! kill -0 "$MODE_PID" 2>/dev/null; then
  echo "ERROR: mode waypoint berhenti sebelum navigator siap." >&2
  exit 1
fi

echo
echo "SEMUA SIAP — motor masih STOP dan lampu merah solid."
echo "Tekan push START fisik DIO 10 untuk mulai waypoint."
echo "Push STOP DIO 11 menghentikan robot; Ctrl+C mematikan seluruh stack."

wait "$MODE_PID"
