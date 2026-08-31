#!/usr/bin/env bash
set -eo pipefail

PROJECT_ROOT="/home/vmx/studica_ws"
DIST_PER_TICK="0.000308604386"

source /opt/ros/humble/setup.bash
source "$PROJECT_ROOT/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
set -u

SERVICE="/titan0/titan_cmd"
if ! timeout 10 ros2 service type "$SERVICE" >/dev/null 2>&1; then
  echo "ERROR: $SERVICE belum tersedia. Jalankan control_server dahulu." >&2
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
