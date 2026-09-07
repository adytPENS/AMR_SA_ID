#!/usr/bin/env bash

set -eo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUNTIME_DIR="/run/user/$(id -u)"
export DISPLAY="${DISPLAY:-:0}"
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-${RUNTIME_DIR}}"
export WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-wayland-0}"
if [[ -z "${XAUTHORITY:-}" && -d "${RUNTIME_DIR}" ]]; then
  auth_file=$(find "${RUNTIME_DIR}" -maxdepth 1 -name '.mutter-Xwaylandauth.*' -print -quit 2>/dev/null || true)
  [[ -z "${auth_file}" ]] || export XAUTHORITY="${auth_file}"
fi

set +u
source /opt/ros/humble/setup.bash
source "${WORKSPACE_DIR}/install/setup.bash"
set -u
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

echo "Window will appear on the Raspberry Pi monitor."
echo "R -> drag color ROI -> Enter -> type name -> Enter to save."
echo "Untuk follow kuning: simpan profil bernama yellow atau kuning, lalu centang."
echo "Tick saved profiles to detect them together. Q/Esc closes the app."
exec ros2 launch studica_control color_roi_tracker_launch.py "$@"
