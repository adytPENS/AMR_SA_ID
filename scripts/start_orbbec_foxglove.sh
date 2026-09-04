#!/usr/bin/env bash

set -eo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_SETUP="/opt/ros/humble/setup.bash"
WORKSPACE_SETUP="${WORKSPACE_DIR}/install/setup.bash"

if [[ ! -f "${WORKSPACE_SETUP}" ]]; then
  echo "ERROR: workspace belum dibangun." >&2
  echo "Jalankan ${WORKSPACE_DIR}/scripts/build_orbbec_camera.sh terlebih dahulu." >&2
  exit 1
fi

if ! lsusb | grep -qi '2bc5:065c'; then
  echo "ERROR: Orbbec Gemini E (2bc5:065c) tidak terdeteksi." >&2
  exit 1
fi

source "${ROS_SETUP}"
source "${WORKSPACE_SETUP}"

if ! ros2 pkg prefix foxglove_bridge >/dev/null 2>&1; then
  echo "ERROR: foxglove_bridge belum terpasang." >&2
  echo "Pasang dengan: sudo apt install ros-humble-foxglove-bridge" >&2
  exit 1
fi

camera_pid=''

cleanup() {
  if [[ -n "${camera_pid}" ]] && kill -0 "${camera_pid}" 2>/dev/null; then
    kill -INT "${camera_pid}" 2>/dev/null || true
    wait "${camera_pid}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

echo "Menjalankan Orbbec Gemini E dan object detector..."
ros2 launch studica_control object_detection_launch.py &
camera_pid=$!

echo
echo "Foxglove siap melalui port 8765."
echo "Di VS Code: buka tab PORTS, lalu Forward a Port: 8765."
echo "Di Foxglove laptop: Open connection > Foxglove WebSocket."
echo "Alamat koneksi: ws://localhost:8765"
echo "Foxglove Image topic: /object_detection/debug_image/compressed"
echo "Detection JSON topic: /object_detection/results"
echo "Tekan Ctrl+C untuk menghentikan kamera dan bridge."
echo

ros2 launch foxglove_bridge foxglove_bridge_launch.xml \
  address:=127.0.0.1 port:=8765
