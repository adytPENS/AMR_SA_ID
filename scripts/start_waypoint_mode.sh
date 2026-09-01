#!/usr/bin/env bash
set -eo pipefail

PROJECT_ROOT="/home/vmx/studica_ws"
ODOM_NODE="$PROJECT_ROOT/src/studica_control/src/components/examples/python/wheel_odometry.py"
ODOM_PARAMS="$PROJECT_ROOT/src/studica_control/config/wheel_odometry.yaml"
NAV_NODE="$PROJECT_ROOT/src/studica_control/src/components/examples/python/waypoint_navigator.py"
DRIVE_NODE="$PROJECT_ROOT/src/studica_control/src/components/examples/python/drive_controller.py"
DRIVE_CONFIG="$PROJECT_ROOT/src/studica_control/config/drive_controller.yaml"
WAYPOINTS="${1:-$PROJECT_ROOT/src/studica_control/config/waypoints.yaml}"

source /opt/ros/humble/setup.bash
source "$PROJECT_ROOT/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
set -u

cleanup() {
  # Prevent SIGINT/SIGTERM received by child commands from re-entering cleanup.
  trap - EXIT INT TERM
  echo "Menghentikan waypoint, drive controller, dan wheel odometry..."
  timeout 2 ros2 service call /waypoint_navigator/stop std_srvs/srv/Trigger "{}" \
    >/dev/null 2>&1 || true
  # SetData/set_speed updates the duty stored by Titan's resend timer, so a
  # stopped publisher cannot leave the last nonzero command running forever.
  for motor in 0 1 2 3; do
    timeout 2 ros2 service call /titan0/titan_cmd \
      studica_control/srv/SetData \
      "{params: 'set_speed', initparams: {n_encoder: $motor, speed: 0.0}}" \
      >/dev/null 2>&1 || true
  done
  kill "${NAV_PID:-}" "${DRIVE_PID:-}" "${ODOM_PID:-}" 2>/dev/null || true
  wait "${NAV_PID:-}" "${DRIVE_PID:-}" "${ODOM_PID:-}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

TOPIC_LIST="$(timeout 15 ros2 topic list)"
REQUIRE_SCAN="$(python3 - "$WAYPOINTS" <<'PY'
import sys
import yaml

with open(sys.argv[1], encoding='utf-8') as stream:
    config = yaml.safe_load(stream) or {}
print('true' if config.get('obstacle_avoidance', {}).get('enabled', True) else 'false')
PY
)"
REQUIRE_BUTTON="$(python3 - "$WAYPOINTS" <<'PY'
import sys
import yaml

with open(sys.argv[1], encoding='utf-8') as stream:
    config = yaml.safe_load(stream) or {}
print('true' if config.get('start_button', {}).get('enabled', False) else 'false')
PY
)"

REQUIRED_TOPICS=(
  /imu
  /titan0/m_0/encoder
  /titan0/m_1/encoder
  /titan0/m_2/encoder
  /titan0/m_3/encoder
)
if [[ "$REQUIRE_SCAN" == "true" ]]; then
  REQUIRED_TOPICS+=(/scan)
fi
if [[ "$REQUIRE_BUTTON" == "true" ]]; then
  REQUIRED_TOPICS+=(/start_button/state)
fi

for topic in "${REQUIRED_TOPICS[@]}"; do
  if ! grep -Fxq "$topic" <<<"$TOPIC_LIST"; then
    echo "ERROR: $topic belum tersedia." >&2
    exit 1
  fi
done

python3 "$ODOM_NODE" --ros-args --params-file "$ODOM_PARAMS" &
ODOM_PID=$!

for _ in 1 2 3 4 5 6 7 8 9 10; do
  SERVICE_LIST="$(timeout 5 ros2 service list || true)"
  if grep -Fxq '/wheel_odometry/reset' <<<"$SERVICE_LIST"; then
    break
  fi
  sleep 1
done

if ! grep -Fxq '/wheel_odometry/reset' <<<"${SERVICE_LIST:-}"; then
  echo "ERROR: service reset odometri tidak muncul." >&2
  exit 1
fi

ros2 service call /wheel_odometry/reset std_srvs/srv/Empty "{}"

python3 "$DRIVE_NODE" --config "$DRIVE_CONFIG" &
DRIVE_PID=$!

python3 "$NAV_NODE" --config "$WAYPOINTS" &
NAV_PID=$!

echo "Mode waypoint siap, motor masih STOP."
echo "Mulai dari terminal lain:"
echo "ros2 service call /waypoint_navigator/start std_srvs/srv/Trigger \"{}\""
echo "Tekan Ctrl+C di sini untuk emergency stop."
wait "$NAV_PID"
