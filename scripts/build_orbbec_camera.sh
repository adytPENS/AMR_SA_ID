#!/usr/bin/env bash

set -euo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_SETUP="/opt/ros/humble/setup.bash"

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "ERROR: ROS 2 Humble tidak ditemukan di ${ROS_SETUP}" >&2
  exit 1
fi

if [[ ! -d "${WORKSPACE_DIR}/src/OrbbecSDK_ROS2" ]]; then
  echo "ERROR: driver OrbbecSDK_ROS2 tidak ditemukan di workspace." >&2
  exit 1
fi

# ROS Humble setup.bash tidak kompatibel dengan nounset saat dimuat.
set +u
source "${ROS_SETUP}"
set -u
cd "${WORKSPACE_DIR}"

# Raspberry Pi memiliki RAM terbatas. Paksa seluruh build berjalan satu per satu.
export MAKEFLAGS="-j1"
export CMAKE_BUILD_PARALLEL_LEVEL=1

echo "Workspace : ${WORKSPACE_DIR}"
echo "ROS       : ${ROS_DISTRO}"
echo "Build Orbbec dengan satu worker (hemat memori)..."

colcon build \
  --executor sequential \
  --packages-up-to studica_control \
  --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

echo
echo "Build selesai. Jalankan kamera dengan:"
echo "  ${WORKSPACE_DIR}/scripts/start_orbbec_camera.sh"
