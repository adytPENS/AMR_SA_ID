#!/usr/bin/env bash
# GUI only: attach to the existing ROS graph without starting hardware.
set -eo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
source /opt/ros/humble/setup.bash
source "$PROJECT_ROOT/install/setup.bash"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
if [[ -z "${DISPLAY:-}" ]]; then
  echo "The GUI requires a desktop or SSH X forwarding (ssh -X). DISPLAY is not set." >&2
  exit 1
fi
exec /usr/bin/python3 "$PROJECT_ROOT/src/studica_control/src/components/examples/python/robot_dashboard.py" "$@"
