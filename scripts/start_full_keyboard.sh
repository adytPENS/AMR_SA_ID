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

cleanup() {
  trap - EXIT INT TERM
  echo "STOP base dan OMS..."
  for topic in /titan1/m_2/cmd /titan1/m_3/cmd; do
    timeout 1 ros2 topic pub --once "$topic" std_msgs/msg/Float64 \
      "{data: 0.0}" >/dev/null 2>&1 || true
  done
  for titan in titan0 titan1; do
    timeout 2 ros2 service call "/$titan/titan_cmd" \
      studica_control/srv/SetData "{params: 'disable'}" \
      >/dev/null 2>&1 || true
  done
  kill "${DRIVE_PID:-}" "${HARDWARE_PID:-}" 2>/dev/null || true
  sudo pkill -TERM -f "$HARDWARE" 2>/dev/null || true
  wait "${DRIVE_PID:-}" "${HARDWARE_PID:-}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "Menjalankan VMX, Titan base 42, dan Titan OMS 10..."
sudo -E env RMW_IMPLEMENTATION=rmw_cyclonedds_cpp "$HARDWARE" \
  --ros-args -r __node:=control_server --params-file "$HARDWARE_CONFIG" &
HARDWARE_PID=$!

for titan in titan0 titan1; do
  ready=false
  for _ in {1..20}; do
    if ros2 service list 2>/dev/null | grep -Fxq "/$titan/titan_cmd"; then
      ready=true
      break
    fi
    sleep 1
  done
  if [[ "$ready" != true ]]; then
    echo "ERROR: service /$titan/titan_cmd tidak tersedia." >&2
    exit 1
  fi
done

bash "$PROJECT_ROOT/scripts/init_titan_encoders.sh"
for titan in titan0 titan1; do
  ros2 service call "/$titan/titan_cmd" studica_control/srv/SetData \
    "{params: 'enable'}"
done

python3 "$DRIVE" --config "$DRIVE_CONFIG" &
DRIVE_PID=$!
sleep 2

echo
echo "KEYBOARD SIAP: W/S/A/D base | I/K lift | J/L rotate | E STOP | Q keluar"
echo "Uji OMS tanpa beban menggunakan duty 0.10."
python3 "$TELEOP" --linear-speed 0.15 --angular-speed 0.8 \
  --oms-speed 0.10 "$@"
