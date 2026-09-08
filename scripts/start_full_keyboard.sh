#!/usr/bin/env bash
# Keyboard base + OMS dalam satu terminal: VMX, dua Titan, drive, dan teleop.
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
HARDWARE="$PROJECT_ROOT/install/studica_control/lib/studica_control/manual_composition"
HARDWARE_CONFIG="$PROJECT_ROOT/src/studica_control/config/titan_m1_test.yaml"
DRIVE="$PROJECT_ROOT/src/studica_control/src/components/examples/python/drive_controller.py"
DRIVE_CONFIG="$PROJECT_ROOT/src/studica_control/config/drive_controller.yaml"
TELEOP="$PROJECT_ROOT/src/studica_control/src/components/examples/python/titan_keyboard_teleop.py"
YDLIDAR_ROOT="/home/vmx/ydlidar_ros2_ws"
YDLIDAR_SETUP="$YDLIDAR_ROOT/install/setup.bash"
YDLIDAR_PARAMS="$YDLIDAR_ROOT/src/ydlidar_ros2_driver/params/Tmini.yaml"

source /opt/ros/humble/setup.bash
if [[ -f "$YDLIDAR_SETUP" ]]; then
  source "$YDLIDAR_SETUP"
fi
source "$PROJECT_ROOT/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

if [[ ! -t 0 ]]; then
  echo "ERROR: keyboard memerlukan terminal interaktif. Gunakan SSH dengan -t atau terminal remote desktop." >&2
  exit 1
fi
if [[ ! -x "$HARDWARE" ]]; then
  echo "ERROR: executable hardware tidak ditemukan: $HARDWARE" >&2
  exit 1
fi

if pgrep -f '(^|/)manual_composition([[:space:]]|$)' >/dev/null; then
  echo "ERROR: manual_composition sudah berjalan. Hentikan launcher lama sebelum membuka akses VMX baru." >&2
  exit 1
fi

LOG_DIR="$PROJECT_ROOT/log/keyboard"
mkdir -p "$LOG_DIR"
RUN_LOG="$LOG_DIR/$(date +%Y%m%d-%H%M%S)-$$.log"
exec > >(tee -a "$RUN_LOG") 2>&1
echo "Log startup lengkap: $RUN_LOG"

lights_off() {
  for topic in /light_control/cmd /light_red/cmd /light_green/cmd /light_yellow/cmd; do
    timeout 5 ros2 topic pub --once "$topic" std_msgs/msg/Bool \
      "{data: false}" >/dev/null 2>&1 || true
  done
}

start_boot_lights_off() {
  BOOT_LIGHT_PIDS=()
  for topic in /light_control/cmd /light_red/cmd /light_green/cmd /light_yellow/cmd; do
    ros2 topic pub --rate 5 "$topic" std_msgs/msg/Bool \
      "{data: false}" >/dev/null 2>&1 &
    BOOT_LIGHT_PIDS+=("$!")
  done
}

stop_boot_lights_off() {
  if ((${#BOOT_LIGHT_PIDS[@]})); then
    kill "${BOOT_LIGHT_PIDS[@]}" 2>/dev/null || true
    # ros2 CLI can keep spinning after SIGTERM while DDS is shutting down.
    # Never let that delay creation of the interactive keyboard node.
    for _ in {1..20}; do
      local any_running=false
      for pid in "${BOOT_LIGHT_PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
          any_running=true
          break
        fi
      done
      [[ "$any_running" == false ]] && break
      sleep 0.05
    done
    for pid in "${BOOT_LIGHT_PIDS[@]}"; do
      if kill -0 "$pid" 2>/dev/null; then
        kill -KILL "$pid" 2>/dev/null || true
      fi
    done
    wait "${BOOT_LIGHT_PIDS[@]}" 2>/dev/null || true
    BOOT_LIGHT_PIDS=()
  fi
}

BOOT_LIGHT_PIDS=()

cleanup() {
  trap - EXIT INT TERM
  echo "STOP base dan OMS..."
  stop_boot_lights_off
  for topic in /titan1/m_2/cmd /titan1/m_3/cmd /oms_slide/cmd; do
    timeout 1 ros2 topic pub --once "$topic" std_msgs/msg/Float64 \
      "{data: 0.0}" >/dev/null 2>&1 || true
  done
  for titan in titan0 titan1; do
    timeout 2 ros2 service call "/$titan/titan_cmd" \
      studica_control/srv/SetData "{params: 'disable'}" \
      >/dev/null 2>&1 || true
  done
  lights_off
  kill "${DRIVE_PID:-}" "${ODOM_PID:-}" "${LIDAR_PID:-}" \
    "${HARDWARE_PID:-}" 2>/dev/null || true
  wait "${DRIVE_PID:-}" "${ODOM_PID:-}" "${LIDAR_PID:-}" \
    "${HARDWARE_PID:-}" 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

echo "Menjalankan VMX, Titan base 42, dan Titan OMS 10..."
# Invoke the executable directly: sudoers grants NOPASSWD to this path,
# not to /usr/bin/env. -n prevents a background password prompt.
sudo -n -E "$HARDWARE" \
  --ros-args --params-file "$HARDWARE_CONFIG" &
HARDWARE_PID=$!

echo "BOOTING — menunggu output lampu lalu mematikan semuanya..."
start_boot_lights_off

# Resolve the last --human-config just like argparse, then start LiDAR only
# when the selected mission contains a base-motion action.
MISSION_CONFIG="$PROJECT_ROOT/config/human_interaction.yaml"
NEXT_IS_HUMAN_CONFIG=false
for argument in "$@"; do
  if [[ "$NEXT_IS_HUMAN_CONFIG" == true ]]; then
    MISSION_CONFIG="$argument"
    NEXT_IS_HUMAN_CONFIG=false
  elif [[ "$argument" == "--human-config" ]]; then
    NEXT_IS_HUMAN_CONFIG=true
  elif [[ "$argument" == --human-config=* ]]; then
    MISSION_CONFIG="${argument#--human-config=}"
  fi
done
if [[ ! -f "$MISSION_CONFIG" && -f "$PROJECT_ROOT/src/studica_control/$MISSION_CONFIG" ]]; then
  MISSION_CONFIG="$PROJECT_ROOT/src/studica_control/$MISSION_CONFIG"
fi
NEEDS_LIDAR="$(python3 - "$MISSION_CONFIG" <<'PY'
import sys
import yaml

with open(sys.argv[1], encoding='utf-8') as stream:
    config = yaml.safe_load(stream) or {}
base_actions = {'goto', 'forward', 'trace', 'heading', 'robot_movetheta',
                'turn', 'robot_move'}
print('true' if any(step.get('action') in base_actions
                    for step in config.get('mission', [])
                    if isinstance(step, dict)) else 'false')
PY
)"
if [[ "$NEEDS_LIDAR" == true ]]; then
  if [[ ! -f "$YDLIDAR_SETUP" || ! -f "$YDLIDAR_PARAMS" ]]; then
    echo "ERROR: misi bergerak memerlukan YDLIDAR, tetapi instalasi/config tidak ditemukan." >&2
    exit 1
  fi
  echo "Menjalankan YDLIDAR untuk keselamatan gerak misi..."
  ros2 launch ydlidar_ros2_driver ydlidar_launch.py \
    params_file:="$YDLIDAR_PARAMS" &
  LIDAR_PID=$!
  echo "Menunggu data /scan dari LiDAR (maksimal 45 detik)..."
  if ! timeout 45 ros2 topic echo --no-daemon --spin-time 2 \
      --qos-profile sensor_data --once /scan sensor_msgs/msg/LaserScan \
      >/dev/null 2>&1; then
    echo "ERROR: LiDAR tidak mengirim data /scan; periksa daya dan kabel." >&2
    exit 1
  fi
  echo "LiDAR siap: /scan tersedia."
fi

python3 "$SCRIPT_DIR/init_titan_encoders.py" --wait-only \
  --service /titan0/titan_cmd --service /titan1/titan_cmd

bash "$PROJECT_ROOT/scripts/init_titan_encoders.sh" \
  --service /titan0/titan_cmd --service /titan1/titan_cmd
for titan in titan0 titan1; do
  ros2 service call "/$titan/titan_cmd" studica_control/srv/SetData \
    "{params: 'enable'}"
done

python3 "$PROJECT_ROOT/src/studica_control/src/components/examples/python/wheel_odometry.py" \
  --ros-args --params-file "$PROJECT_ROOT/src/studica_control/config/wheel_odometry.yaml" &
ODOM_PID=$!

python3 "$DRIVE" --config "$DRIVE_CONFIG" &
DRIVE_PID=$!
sleep 2
if ! kill -0 "$ODOM_PID" 2>/dev/null; then
  echo "ERROR: wheel odometry gagal berjalan." >&2
  exit 1
fi
if ! kill -0 "$DRIVE_PID" 2>/dev/null; then
  echo "ERROR: drive controller gagal berjalan; lihat pesan Python di atas." >&2
  exit 1
fi

echo
echo "HARDWARE SIAP — menjalankan node keyboard..."
echo "KONTROL: W/S/A/D base | Z follow kuning | I/K lift | J/L rotate | M simpan default OMS + READY | E/X STOP | Q keluar"
echo "SERVO: P posisi awal standard (opsional; M otomatis) | G/H slide | R/T wrist | Y/U gripper"
echo "Uji OMS tanpa beban menggunakan duty 0.10."
# Publisher BOOT berhenti di sini. Pin tetap LOW sampai node keyboard selesai
# dibuat lalu menyalakan indikator merah READY.
stop_boot_lights_off
python3 "$TELEOP" --linear-speed 0.15 --angular-speed 0.8 \
  --oms-speed 0.10 --rotate-duty 33.3 --slide-speed 15\
  --oms-default-file "$PROJECT_ROOT/config/oms_default.json" \
  --human-config "$PROJECT_ROOT/config/human_interaction.yaml" \
  --servo-config "$PROJECT_ROOT/src/studica_control/config/keyboard_servos.yaml" "$@"
