#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
DIST_PER_TICK="0.000308604386"

source /opt/ros/humble/setup.bash
source "$PROJECT_ROOT/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
set -u

exec python3 "$SCRIPT_DIR/init_titan_encoders.py" "$@"
