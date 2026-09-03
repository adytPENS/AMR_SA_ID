#!/usr/bin/env bash
# Bridge ROS 2 lokal ke Foxglove melalui SSH tunnel.
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"

source /opt/ros/humble/setup.bash
source "$PROJECT_ROOT/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

if ! ros2 pkg prefix foxglove_bridge >/dev/null 2>&1; then
  echo "ERROR: foxglove_bridge belum terpasang." >&2
  echo "Jalankan: sudo apt install ros-humble-foxglove-bridge" >&2
  exit 1
fi

echo "Foxglove bridge: ws://127.0.0.1:8765"
echo "Gunakan SSH tunnel dari PC: ssh -L 8765:localhost:8765 vmx@IP_RASPBERRY"
ros2 launch foxglove_bridge foxglove_bridge_launch.xml \
  address:=127.0.0.1 port:=8765
