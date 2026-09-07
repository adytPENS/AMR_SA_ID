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

source /opt/ros/humble/setup.bash
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

cleanup() {
  trap - EXIT INT TERM
  echo "STOP base dan OMS..."
  for topic in /titan1/m_2/cmd /titan1/m_3/cmd /oms_slide/cmd; do
    timeout 1 ros2 topic pub --once "$topic" std_msgs/msg/Float64 \
      "{data: 0.0}" >/dev/null 2>&1 || true
  done
  for titan in titan0 titan1; do
    timeout 2 ros2 service call "/$titan/titan_cmd" \
      studica_control/srv/SetData "{params: 'disable'}" \
      >/dev/null 2>&1 || true
  done
  kill "${DRIVE_PID:-}" "${ODOM_PID:-}" "${HARDWARE_PID:-}" 2>/dev/null || true
  wait "${DRIVE_PID:-}" "${ODOM_PID:-}" "${HARDWARE_PID:-}" 2>/dev/null || true
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

python3 "$SCRIPT_DIR/init_titan_encoders.py" --wait-only \
  --service /titan0/titan_cmd --service /titan1/titan_cmd

bash "$PROJECT_ROOT/scripts/init_titan_encoders.sh"
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
echo "KEYBOARD SIAP: W/S/A/D base | Z follow kuning | I/K lift | J/L rotate | M simpan default OMS + READY | E/X STOP | Q keluar"
echo "SERVO: P posisi awal standard | G/H slide | R/T wrist | Y/U gripper"
echo "Uji OMS tanpa beban menggunakan duty 0.10."
python3 "$TELEOP" --linear-speed 0.15 --angular-speed 0.8 \
  --oms-speed 0.10 --rotate-duty 33.3 --slide-speed 15\
  --oms-default-file "$PROJECT_ROOT/config/oms_default.json" \
  --human-config "$PROJECT_ROOT/config/human_interaction.yaml" \
  --servo-config "$PROJECT_ROOT/src/studica_control/config/keyboard_servos.yaml" "$@"
