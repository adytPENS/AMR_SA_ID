#!/usr/bin/env bash

set -eo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUNTIME_DIR="/run/user/$(id -u)"
export DISPLAY="${DISPLAY:-:0}"
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-${RUNTIME_DIR}}"
export WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-wayland-0}"
if [[ -z "${XAUTHORITY:-}" ]]; then
  auth_file=$(find "${RUNTIME_DIR}" -maxdepth 1 -name '.mutter-Xwaylandauth.*' -print -quit)
  [[ -z "${auth_file}" ]] || export XAUTHORITY="${auth_file}"
fi

set +u
source /opt/ros/humble/setup.bash
source "${WORKSPACE_DIR}/install/setup.bash"
set -u

echo "Window will appear on the Raspberry Pi monitor."
echo "Press R -> drag Object #1 -> Enter. Press Q/Esc to stop."
exec ros2 launch studica_control color_roi_tracker_launch.py
