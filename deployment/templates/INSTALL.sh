#!/usr/bin/env bash
# Run with bash from USB; only package installation/system configuration uses sudo.
set -Eeuo pipefail
BUNDLE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
MODE="${1:---install}"
case "$MODE" in
  --help|-h) echo 'Usage: bash INSTALL.sh [--check|--install]'; exit 0 ;;
  --check|--install) ;;
  *) echo 'Unknown option. Use --check or --install.' >&2; exit 2 ;;
esac
[[ $# -le 1 ]] || { echo 'Too many arguments.' >&2; exit 2; }
fail() { echo "ERROR: $*" >&2; exit 1; }
[[ $EUID -ne 0 ]] || fail 'Log in as vmx and run bash INSTALL.sh without sudo.'
[[ $(id -un) == vmx && "$HOME" == /home/vmx ]] || fail 'This project requires user vmx with home /home/vmx.'
source /etc/os-release
[[ "$ID" == ubuntu && "$VERSION_ID" == 22.04 && $(uname -m) == aarch64 ]] ||
  fail 'Supported target: Studica Ubuntu 22.04 ARM64 image. OS conversion is not automatic.'
for file in /usr/local/include/vmxpi/VMXPi.h /usr/local/lib/vmxpi/libvmxpi_hal_cpp.so; do
  [[ -r "$file" ]] || fail "Studica VMX HAL missing: $file. Restore the matching Studica SDK/image first."
done
for cmd in sha256sum tar python3 flock sudo; do command -v "$cmd" >/dev/null || fail "Missing base command: $cmd"; done
(cd "$BUNDLE" && sha256sum --check SHA256SUMS) || fail 'Package checksum failed. Copy the complete original folder again.'
# Check archive safety and required source files before any system change.
python3 - "$BUNDLE/payload.tar.gz" <<'PY'
import sys, tarfile, posixpath
required = {'studica_ws/src/studica_control/package.xml',
            'studica_ws/src/OrbbecSDK_ROS2/orbbec_camera/SDK/lib/arm64/libOrbbecSDK.so',
            'ydlidar_ros2_ws/src/ydlidar_ros2_driver/params/Tmini.yaml', 'YDLidar-SDK/CMakeLists.txt'}
with tarfile.open(sys.argv[1]) as archive:
    members = archive.getmembers()
    names = {m.name for m in members}
    if required - names:
        raise SystemExit(f'Missing payload files: {required - names}')
    for m in members:
        if m.name.startswith('/') or '..' in m.name.split('/') or m.isdev() or m.isfifo() or m.islnk():
            raise SystemExit(f'Unsafe archive member: {m.name}')
        if m.name.split('/')[0] not in {'studica_ws', 'ydlidar_ros2_ws', 'YDLidar-SDK'}:
            raise SystemExit(f'Unexpected archive root: {m.name}')
        if m.issym():
            target = posixpath.normpath(posixpath.join(posixpath.dirname(m.name), m.linkname))
            if m.linkname.startswith('/') or target.split('/')[0] != m.name.split('/')[0]:
                raise SystemExit(f'External symlink: {m.name}')
        # Prevent extraction through a symlink directory.
        if m.issym() and any(n.startswith(m.name + '/') for n in names):
            raise SystemExit(f'Symlink directory: {m.name}')
print('Payload structure verified.')
PY
available_kb=$(df -Pk /home/vmx | awk 'NR==2 {print $4}')
[[ "$available_kb" -ge 8388608 ]] || fail 'At least 8 GiB free space is required (more if many packages are missing).'
if [[ "$MODE" == --check ]]; then
  echo 'CHECK PASSED: supported host, HAL, bundle integrity and disk space.'
  echo 'No changes made. Internet, apt availability and hardware operation are not tested.'
  exit 0
fi
# Prevent competing installer runs. The descriptor remains open until exit.
exec 9> /home/vmx/.studica-usb-install.lock
flock -n 9 || fail 'Another installer is running.'
STAMP="$(date +%Y%m%d-%H%M%S)-$$"
LOG_DIR=/home/vmx/studica-deploy-logs
mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/install-$STAMP.log"
exec > >(tee -a "$LOG_FILE") 2>&1
trap 'rc=$?; echo "INSTALL FAILED (exit $rc) at line $LINENO. See $LOG_FILE. Backups are kept; no automatic rollback."; exit "$rc"' ERR
sudo -v
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
export LANG=C.UTF-8 LC_ALL=C.UTF-8
# Do not inherit overlays or Python environments from the source machine/current shell.
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
unset VIRTUAL_ENV CONDA_PREFIX ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION ROS_LOCALHOST_ONLY
unset CYCLONEDDS_URI RMW_IMPLEMENTATION BUILDING_PACKAGE
STAGE=$(mktemp -d /home/vmx/.studica-deploy-stage.XXXXXX)
echo "Staging payload in $STAGE. Log: $LOG_FILE"
tar --extract --gzip --file "$BUNDLE/payload.tar.gz" --directory "$STAGE" --no-same-owner --no-same-permissions
chmod -R u+rwX "$STAGE"

echo '[1/7] Installing build tools and configuring ROS apt source...'
sudo apt-get update
# --no-remove prevents apt from solving dependency conflicts by removing image packages.
sudo apt-get install -y --no-remove curl ca-certificates software-properties-common \
  openssh-server rsync git tar gzip unzip build-essential cmake pkg-config python3
sudo add-apt-repository -y universe
# Respect an existing ROS repository. If broken, apt/ros-base installation fails visibly.
if ! grep -rEq '^[[:space:]]*(deb .*|URIs:.*)packages\.ros\.org/ros2/ubuntu' \
    /etc/apt/sources.list /etc/apt/sources.list.d 2>/dev/null; then
  curl -fLsS --retry 3 https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest -o "$STAGE/ros-release.json"
  asset_url=$(python3 - "$STAGE/ros-release.json" <<'PY'
import json, sys
release = json.load(open(sys.argv[1]))
assets = [a['browser_download_url'] for a in release['assets']
          if a['name'].startswith('ros2-apt-source_') and a['name'].endswith('jammy_all.deb')]
if len(assets) != 1 or not assets[0].startswith('https://github.com/ros-infrastructure/ros-apt-source/releases/download/'):
    raise SystemExit('Cannot identify official Jammy ros2-apt-source asset.')
print(assets[0])
PY
  )
  curl -fLsS --retry 3 "$asset_url" -o "$STAGE/ros2-apt-source.deb"
  sudo dpkg -i "$STAGE/ros2-apt-source.deb"
fi
sudo apt-get update
sudo apt-get install -y --no-remove ros-humble-ros-base python3-colcon-common-extensions python3-rosdep \
  python3-opencv python3-numpy python3-yaml libopencv-dev libeigen3-dev \
  libusb-1.0-0-dev libudev-dev libgflags-dev libgoogle-glog-dev nlohmann-json3-dev libdw-dev \
  ros-humble-rmw-cyclonedds-cpp ros-humble-image-transport-plugins \
  ros-humble-compressed-image-transport ros-humble-foxglove-bridge ros-humble-joy \
  ros-humble-rviz2 usbutils v4l-utils
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then sudo rosdep init; fi
rosdep update --rosdistro humble
set +u
source /opt/ros/humble/setup.bash
set -u
rosdep install --from-paths "$STAGE/studica_ws/src" "$STAGE/ydlidar_ros2_ws/src" \
  --ignore-src --rosdistro humble -y

echo '[2/7] Backing up old workspaces and placing fresh source...'
BACKUP="/home/vmx/studica-backups/$STAMP"
mkdir -p "$BACKUP"
# Never delete or merge old builds. Fixed locations match the project's launch scripts.
for name in studica_ws ydlidar_ros2_ws YDLidar-SDK; do
  if [[ -e "/home/vmx/$name" || -L "/home/vmx/$name" ]]; then
    mv -- "/home/vmx/$name" "$BACKUP/$name"
  fi
  mv -- "$STAGE/$name" "/home/vmx/$name"
done
WS=/home/vmx/studica_ws
find "$WS/scripts" -type f -name '*.sh' -exec chmod u+x {} +
export MAKEFLAGS=-j1 CMAKE_BUILD_PARALLEL_LEVEL=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

echo '[3/7] Building Studica and YDLidar native drivers...'
make -C "$WS/drivers" -j1
sudo make -C "$WS/drivers" install
cmake -S /home/vmx/YDLidar-SDK -B /home/vmx/YDLidar-SDK/build \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=OFF -DBUILD_TEST=OFF
cmake --build /home/vmx/YDLidar-SDK/build --parallel 1
sudo cmake --install /home/vmx/YDLidar-SDK/build

echo '[4/7] Building ROS workspaces (one worker; this can take a while)...'
(cd /home/vmx/ydlidar_ros2_ws && colcon build --executor sequential --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release)
(cd "$WS" && colcon build --executor sequential --packages-up-to studica_control --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release)

echo '[5/7] Configuring linker, hardware permissions and SSH...'
cat > "$STAGE/studica_ws.conf" <<'CONF'
/usr/local/lib
/usr/local/lib/vmxpi
/usr/local/lib/studica_drivers
/home/vmx/studica_ws/install/studica_control/lib
/opt/ros/humble/lib
/opt/ros/humble/lib/aarch64-linux-gnu
/opt/ros/humble/opt/rviz_ogre_vendor/lib
CONF
# Validate sudoers before replacing the system file; only the hardware process needs root.
cat > "$STAGE/studica_control" <<'CONF'
vmx ALL=(root) SETENV: NOPASSWD: /home/vmx/studica_ws/install/studica_control/lib/studica_control/manual_composition
CONF
sudo visudo -cf "$STAGE/studica_control"
for path in /etc/ld.so.conf.d/studica_ws.conf /etc/sudoers.d/studica_control \
  /etc/udev/rules.d/99-obsensor-libusb.rules /etc/udev/rules.d/99-studica-usb.rules; do
  if sudo test -e "$path"; then sudo cp -a --parents "$path" "$BACKUP/"; fi
done
sudo install -m 0644 "$STAGE/studica_ws.conf" /etc/ld.so.conf.d/studica_ws.conf
sudo install -m 0440 "$STAGE/studica_control" /etc/sudoers.d/studica_control
sudo ldconfig
sudo visudo -c
sudo install -m 0644 "$WS/src/OrbbecSDK_ROS2/orbbec_camera/scripts/99-obsensor-libusb.rules" /etc/udev/rules.d/
printf '%s\n' 'KERNEL=="i2c-[0-9]*", GROUP="root", MODE="0660"' > "$STAGE/99-studica-usb.rules"
sudo install -m 0644 "$STAGE/99-studica-usb.rules" /etc/udev/rules.d/
sudo usermod -aG video,dialout vmx
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo systemctl enable --now ssh
# Repair only VS Code's own server folder if present, not the entire home directory.
if [[ -d /home/vmx/.vscode-server && ! -L /home/vmx/.vscode-server ]]; then
  sudo chown -R --no-dereference vmx:"$(id -gn vmx)" /home/vmx/.vscode-server
fi

echo '[6/7] Configuring new terminals...'
[[ ! -e /home/vmx/.bashrc ]] || cp -a /home/vmx/.bashrc "$BACKUP/bashrc"
python3 - <<'PY'
from pathlib import Path
p = Path('/home/vmx/.bashrc')
s = p.read_text() if p.exists() else ''
start, end = '# BEGIN STUDICA USB SETUP', '# END STUDICA USB SETUP'
if start in s:
    a = s.index(start)
    b = s.find(end, a)
    if b < 0:
        raise SystemExit('Unfinished STUDICA block in .bashrc; repair it before rerunning.')
    s = s[:a] + s[b + len(end):]
block = '''# BEGIN STUDICA USB SETUP
source /opt/ros/humble/setup.bash
[ ! -f /home/vmx/ydlidar_ros2_ws/install/local_setup.bash ] || source /home/vmx/ydlidar_ros2_ws/install/local_setup.bash
[ ! -f /home/vmx/studica_ws/install/local_setup.bash ] || source /home/vmx/studica_ws/install/local_setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# END STUDICA USB SETUP
'''
p.write_text(s.rstrip() + '\n\n' + block)
PY

echo '[7/7] Checking installed software...'
set +u
source /home/vmx/ydlidar_ros2_ws/install/local_setup.bash
source "$WS/install/local_setup.bash"
set -u
for pkg in studica_control orbbec_camera orbbec_camera_msgs ydlidar_ros2_driver nav2_bringup slam_toolbox foxglove_bridge; do
  ros2 pkg prefix "$pkg"
done
/usr/bin/python3 -c 'import rclpy, cv2, numpy, yaml; from studica_control.srv import SetData'
for lib in "$WS/install/studica_control/lib/studica_control/manual_composition" \
  "$WS/install/orbbec_camera/lib/liborbbec_camera.so"; do
  linkage=$(ldd "$lib")
  echo "$linkage"
  if [[ "$linkage" == *'not found'* ]]; then fail "Unresolved libraries in $lib"; fi
done
echo "INSTALL COMPLETE. Backup: $BACKUP"
echo "Log: $LOG_FILE"
echo 'Log out and back in (or reboot) to activate video/dialout groups.'
echo 'Reconnect the camera. Read README.md for camera and robot checks.'
echo 'No robot process was started. VS Code Server installs on the first Remote SSH connection.'
echo 'Open /home/vmx/studica_ws in VS Code. Run hostname -I to find the address.'
