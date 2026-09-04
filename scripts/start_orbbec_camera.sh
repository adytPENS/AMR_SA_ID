#!/usr/bin/env bash

set -euo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_SETUP="/opt/ros/humble/setup.bash"
WORKSPACE_SETUP="${WORKSPACE_DIR}/install/setup.bash"

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "ERROR: ROS 2 Humble tidak ditemukan di ${ROS_SETUP}" >&2
  exit 1
fi

if [[ ! -f "${WORKSPACE_SETUP}" ]]; then
  echo "ERROR: workspace belum dibangun." >&2
  echo "Jalankan ${WORKSPACE_DIR}/scripts/build_orbbec_camera.sh terlebih dahulu." >&2
  exit 1
fi

if ! lsusb | grep -qi '2bc5:065c'; then
  echo "ERROR: depth sensor Orbbec Gemini E (2bc5:065c) tidak terdeteksi." >&2
  echo "Periksa kabel USB dan daya kamera." >&2
  exit 1
fi

# ROS/colcon setup files tidak kompatibel dengan nounset saat dimuat.
set +u
source "${ROS_SETUP}"
source "${WORKSPACE_SETUP}"
set -u

echo "Menjalankan Orbbec Gemini E..."
exec ros2 launch studica_control orbbec_gemini_e_launch.py "$@"
