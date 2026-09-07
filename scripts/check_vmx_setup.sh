#!/usr/bin/env bash
# Read-only preflight; does not start hardware or install anything.
set -uo pipefail
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
fail=0
check() {
  local label="$1"; shift
  if "$@"; then printf 'OK    %s\n' "$label"
  else printf 'FAIL  %s\n' "$label"; fail=$((fail + 1)); fi
}
warn_file() { [[ -e "$1" ]] || printf 'INFO  Belum tersedia: %s\n' "$1"; }
printf 'Preflight VMX: %s\n' "$WS"
check 'Jalankan sebagai user biasa' test "$EUID" -ne 0
check 'Ubuntu 22.04' bash -c '. /etc/os-release; [[ "$ID" == ubuntu && "$VERSION_ID" == 22.04 ]]'
check 'Arsitektur aarch64 (target VMX project)' test "$(uname -m)" = aarch64
check 'Workspace writable' test -w "$WS"
for cmd in cmake make g++ colcon rosdep ssh tar curl; do
  check "Command $cmd" bash -c 'command -v "$1" >/dev/null' _ "$cmd"
done
for path in /opt/ros/humble/setup.bash /usr/local/include/vmxpi/VMXPi.h \
  /usr/local/lib/vmxpi/libvmxpi_hal_cpp.so \
  /usr/local/lib/studica_drivers/libstudica_drivers.so \
  "$WS/src/OrbbecSDK_ROS2/orbbec_camera/SDK/lib/arm64/libOrbbecSDK.so"; do
  check "$path" test -r "$path"
done
if [[ -f /opt/ros/humble/setup.bash ]]; then
  set +u
  source /opt/ros/humble/setup.bash
  [[ ! -f "$WS/install/local_setup.bash" ]] || source "$WS/install/local_setup.bash"
  set -u
  for pkg in ament_cmake rclcpp_action rclpy launch_ros rmw_cyclonedds_cpp cv_bridge image_transport nav2_bringup slam_toolbox; do
    check "ROS $pkg" ros2 pkg prefix "$pkg"
  done
  check 'Python OpenCV, NumPy, YAML, rclpy' /usr/bin/python3 -c 'import cv2, numpy, yaml, rclpy'
fi
for path in /etc/udev/rules.d/99-obsensor-libusb.rules /etc/sudoers.d/studica_control \
  "$WS/install/studica_control/lib/studica_control/manual_composition" \
  /home/vmx/ydlidar_ros2_ws/install/setup.bash; do warn_file "$path"; done
printf '\nStorage dan RAM (pastikan cukup untuk build):\n'
df -h "$WS"
free -h
printf '\nUSB (Orbbec biasanya vendor 2bc5):\n'
if command -v lsusb >/dev/null; then
  lsusb || printf 'INFO  USB tidak dapat diperiksa dari sesi ini; ulangi langsung di terminal VMX.\n'
else
  printf 'INFO  Install usbutils untuk memeriksa USB.\n'
fi
printf '\nSelesai: %d pemeriksaan gagal. INFO adalah langkah lanjutan/opsional.\n' "$fail"
printf 'Ini bukan pengujian gerak, kamera streaming, atau kelengkapan semua dependency.\n'
((fail == 0))
