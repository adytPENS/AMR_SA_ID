#!/usr/bin/env bash

set -eo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUNTIME_DIR="/run/user/$(id -u)"

# Direct GUI output to the active Raspberry Pi desktop, even when launched by SSH.
export DISPLAY="${DISPLAY:-:0}"
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-${RUNTIME_DIR}}"
export WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-wayland-0}"
if [[ -z "${XAUTHORITY:-}" ]]; then
  auth_file=$(find "${RUNTIME_DIR}" -maxdepth 1 -name '.mutter-Xwaylandauth.*' -print -quit)
  if [[ -n "${auth_file}" ]]; then
    export XAUTHORITY="${auth_file}"
  fi
fi

set +u
source /opt/ros/humble/setup.bash
source "${WORKSPACE_DIR}/install/setup.bash"
set -u

camera_pid=''
cleanup() {
  if [[ -n "${camera_pid}" ]] && kill -0 "${camera_pid}" 2>/dev/null; then
    kill -INT "${camera_pid}" 2>/dev/null || true
    wait "${camera_pid}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

ros2 launch studica_control object_detection_launch.py &
camera_pid=$!
sleep 3

echo "Viewer tampil di monitor Raspberry Pi. Tekan Q/Esc pada gambar untuk keluar."
ros2 run studica_control local_detection_viewer.py
