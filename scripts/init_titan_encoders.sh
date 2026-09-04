#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
DIST_PER_TICK="0.000308604386"

source /opt/ros/humble/setup.bash
source "$PROJECT_ROOT/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
set -u

SERVICE="/titan0/titan_cmd"
SERVICE_READY=false
echo "Menunggu discovery $SERVICE (maksimal 45 detik)..."
for _ in {1..15}; do
  if timeout 5 ros2 service list --no-daemon --spin-time 2 2>/dev/null | \
      grep -Fxq "$SERVICE"; then
    SERVICE_READY=true
    break
  fi
  sleep 1
done
if [[ "$SERVICE_READY" != true ]]; then
  echo "ERROR: $SERVICE tidak ditemukan setelah 45 detik." >&2
  echo "Pastikan hanya satu control_server aktif dan RMW memakai CycloneDDS." >&2
  exit 1
fi

for motor in 0 1 2 3; do
  echo "Inisialisasi encoder M${motor}..."
  ros2 service call "$SERVICE" studica_control/srv/SetData \
    "{params: 'setup_encoder', initparams: {n_encoder: $motor}}"
  ros2 service call "$SERVICE" studica_control/srv/SetData \
    "{params: 'configure_encoder', initparams: {n_encoder: $motor, dist_per_tick: $DIST_PER_TICK}}"
  ros2 service call "$SERVICE" studica_control/srv/SetData \
    "{params: 'reset_encoder', initparams: {n_encoder: $motor}}"
done

echo "Encoder M0-M3 siap dengan skala ${DIST_PER_TICK} m/tick."
