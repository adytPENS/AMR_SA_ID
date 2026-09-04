#!/usr/bin/env bash
# Menjalankan seluruh stack waypoint dalam satu terminal.
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
HARDWARE_EXEC="$PROJECT_ROOT/install/studica_control/lib/studica_control/manual_composition"
HARDWARE_CONFIG="$PROJECT_ROOT/src/studica_control/config/titan_m1_test.yaml"
WAYPOINT_CONFIG="${1:-$PROJECT_ROOT/src/studica_control/config/waypoints.yaml}"
YDLIDAR_ROOT="/home/vmx/ydlidar_ros2_ws"
YDLIDAR_SETUP="$YDLIDAR_ROOT/install/setup.bash"
# Gunakan profil yang sudah terbukti menerbitkan /scan pada pengujian unit ini.
YDLIDAR_PARAMS="$YDLIDAR_ROOT/src/ydlidar_ros2_driver/params/Tmini.yaml"

source /opt/ros/humble/setup.bash
if [[ -f "$YDLIDAR_SETUP" ]]; then
  source "$YDLIDAR_SETUP"
fi
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
  timeout 2 ros2 service call /titan1/titan_cmd \
    studica_control/srv/SetData "{params: 'disable'}" \
    >/dev/null 2>&1 || true
  kill "${MODE_PID:-}" "${LIDAR_PID:-}" "${HARDWARE_PID:-}" \
    2>/dev/null || true
  pkill -TERM -f 'ydlidar_ros2_driver_node' 2>/dev/null || true
  pkill -TERM -f 'static_transform_publisher.*laser_frame' 2>/dev/null || true
  wait "${MODE_PID:-}" "${LIDAR_PID:-}" "${HARDWARE_PID:-}" \
    2>/dev/null || true
}
trap cleanup EXIT INT TERM

AVOIDANCE_ENABLED="$(python3 - "$WAYPOINT_CONFIG" <<'PY'
import sys
import yaml

with open(sys.argv[1], encoding='utf-8') as stream:
    config = yaml.safe_load(stream) or {}
print('true' if config.get('obstacle_avoidance', {}).get('enabled', True)
      else 'false')
PY
)"

if [[ "$AVOIDANCE_ENABLED" == "true" ]]; then
  if [[ ! -f "$YDLIDAR_SETUP" || ! -f "$YDLIDAR_PARAMS" ]]; then
    echo "ERROR: obstacle avoidance aktif, tetapi instalasi/config YDLIDAR tidak ditemukan." >&2
    echo "Periksa: $YDLIDAR_SETUP" >&2
    echo "Periksa: $YDLIDAR_PARAMS" >&2
    exit 1
  fi

  echo "Menjalankan YDLIDAR untuk obstacle avoidance..."
  ros2 launch ydlidar_ros2_driver ydlidar_launch.py \
    params_file:="$YDLIDAR_PARAMS" &
  LIDAR_PID=$!

  echo "Menunggu data /scan dari LiDAR (maksimal 45 detik)..."
  # Berikan tipe pesan secara eksplisit agar subscriber dapat dibuat sebelum
  # publisher /scan selesai terdaftar pada ROS discovery. --no-daemon juga
  # mencegah daftar topic lama dari daemon menyebabkan hasil pemeriksaan keliru.
  if ! timeout 45 ros2 topic echo \
      --no-daemon --spin-time 2 --qos-profile sensor_data --once \
      /scan sensor_msgs/msg/LaserScan >/dev/null 2>&1; then
    echo "ERROR: LiDAR tidak mengirim data /scan dalam 45 detik." >&2
    echo "Periksa daya, kabel USB/serial, dan port YDLIDAR." >&2
    exit 1
  fi
  echo "LiDAR siap: /scan tersedia."
else
  echo "Obstacle avoidance nonaktif; driver LiDAR tidak dijalankan."
fi

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

echo "BOOTING — mematikan seluruh lampu sampai robot benar-benar siap..."
for topic in /light_control/cmd /light_red/cmd /light_green/cmd /light_yellow/cmd; do
  ros2 topic pub --once "$topic" std_msgs/msg/Bool "{data: false}" \
    >/dev/null
done

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

# start_waypoint_mode adalah proses pembungkus sehingga masih dapat hidup
# beberapa saat walaupun waypoint_navigator di dalamnya gagal. Pastikan
# service navigator benar-benar terdaftar sebelum menampilkan SEMUA SIAP.
NAVIGATOR_READY=false
for _ in {1..10}; do
  if ros2 service list 2>/dev/null | grep -Fxq /waypoint_navigator/start; then
    NAVIGATOR_READY=true
    break
  fi
  sleep 1
done
if [[ "$NAVIGATOR_READY" != "true" ]]; then
  echo "ERROR: /waypoint_navigator/start tidak tersedia; navigator gagal dibuat." >&2
  exit 1
fi

# Ini adalah batas resmi antara BOOTING dan READY. Sampai seluruh pemeriksaan
# di atas berhasil, navigator mempertahankan C/R/G/Y dalam keadaan OFF dan
# menolak perintah START.
READY_RESPONSE="$(ros2 service call /waypoint_navigator/set_ready \
  std_srvs/srv/Trigger "{}" 2>&1)"
if ! grep -Fq 'success=True' <<<"$READY_RESPONSE" && \
   ! grep -Fq 'success: true' <<<"$READY_RESPONSE"; then
  echo "ERROR: navigator gagal masuk status READY." >&2
  echo "$READY_RESPONSE" >&2
  exit 1
fi

echo
echo "SEMUA SIAP — motor masih STOP dan lampu merah solid."
echo "Tekan push START fisik DIO 10 untuk mulai waypoint."
echo "Push STOP DIO 11 menghentikan robot; Ctrl+C mematikan seluruh stack."

wait "$MODE_PID"
