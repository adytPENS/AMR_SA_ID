#!/usr/bin/env bash
# Reuse the existing hardware server; never start another VMX owner.
set -eo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
source /opt/ros/humble/setup.bash
source "$PROJECT_ROOT/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
exec python3 "$PROJECT_ROOT/src/studica_control/src/components/examples/python/servo_calibration.py" \
  --config "$PROJECT_ROOT/src/studica_control/config/keyboard_servos.yaml" "$@"
