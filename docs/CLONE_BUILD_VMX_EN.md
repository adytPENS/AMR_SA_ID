# Clone from GitHub and build on a new VMX

Guide for https://github.com/adytPENS/AMR_SA_ID.
Run these commands on the target VMX as user `vmx`, with internet access.
This project targets Ubuntu 22.04 ARM64 (`aarch64`), ROS 2 Humble, and VMX HAL.

## 1. Check the system

```bash
whoami
cat /etc/os-release
uname -m
ls /opt/ros
ls /usr/local/include/vmxpi/VMXPi.h
ls /usr/local/lib/vmxpi/libvmxpi_hal_cpp.so
```

If ROS is missing, follow the [official ROS 2 Humble installation guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html), including the ROS apt repository setup.
If VMX HAL is missing, install the appropriate SDK/image from [Studica](https://learn.studica.com/docs/ws/vmx/os-images).
Cloning does not install ROS or VMX HAL. Both must be available before building.

## 2. Clone the project

Use a new destination directory. If `~/studica_ws` already contains other work,
save or move that workspace first.

```bash
sudo apt update
sudo apt install -y git
cd ~
git clone https://github.com/adytPENS/AMR_SA_ID.git studica_ws
cd ~/studica_ws
```

At the time of writing, `src/OrbbecSDK_ROS2` is stored as a gitlink without
`.gitmodules`. Cloning the main repository does not download the camera source.
Fetch it manually and check out the commit referenced by this project:

```bash
git clone https://github.com/orbbec/OrbbecSDK_ROS2.git src/OrbbecSDK_ROS2
ORBBEC_COMMIT=$(git rev-parse HEAD:src/OrbbecSDK_ROS2)
git -C src/OrbbecSDK_ROS2 checkout "$ORBBEC_COMMIT"
ls src/studica_control/config/
```

A detached HEAD in the camera repository is expected when checking out a pinned version.
The configuration directory contains `D_nav.yaml` and `misi_forward_gripper.yaml`.

## 3. Install dependencies

These commands assume the ROS Humble apt repository is already configured.

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake pkg-config \
  python3-colcon-common-extensions python3-rosdep \
  ros-humble-ros-base ros-humble-rmw-cyclonedds-cpp \
  python3-opencv python3-numpy python3-yaml python3-tk \
  libopencv-dev libeigen3-dev libusb-1.0-0-dev libudev-dev \
  libgflags-dev libgoogle-glog-dev nlohmann-json3-dev libdw-dev \
  ros-humble-image-transport-plugins \
  ros-humble-compressed-image-transport usbutils v4l-utils

source /opt/ros/humble/setup.bash
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi
rosdep update
cd ~/studica_ws
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
```

Resolve dependency installation failures before building. Use system Python, not Conda/venv.
Rosdep installs the dependencies declared in `package.xml`, including navigation requirements.

Optional packages by feature:

| Feature | apt package |
| --- | --- |
| SSH access from a laptop | `openssh-server` |
| Gamepad | `ros-humble-joy` |
| Foxglove | `ros-humble-foxglove-bridge` |

To enable SSH:

```bash
sudo apt install -y openssh-server
sudo systemctl enable --now ssh
hostname -I
```

From your laptop, use `ssh vmx@IP_VMX` (replace `IP_VMX` with the VMX IP address)
or VS Code Remote - SSH.

## 4. Build the Studica driver

```bash
cd ~/studica_ws/drivers
make clean
make -j1
sudo make install
```

## 5. Build the ROS workspace

Use a terminal that has not sourced an old workspace overlay. Do not copy
`build/`, `install/`, or `log/` from another machine, and do not run `sudo colcon build`.

```bash
cd ~/studica_ws
bash scripts/build_orbbec_camera.sh
source /opt/ros/humble/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

The script builds `studica_control` and its workspace dependencies, including Orbbec,
with one worker to limit RAM usage. Continue only after the build succeeds.

## 6. Set permissions and verify the installation

```bash
cd ~/studica_ws
bash scripts/setup_permissions.sh "$PWD"
sudo visudo -c
ldd install/studica_control/lib/studica_control/manual_composition
ros2 pkg prefix studica_control
bash scripts/check_vmx_setup.sh
```

Run the permissions script as user `vmx`; it uses sudo for the required system
configuration. Resolve any libraries reported as `not found` and any `FAIL`
results from the setup check before running the hardware.

Add the following to `~/.bashrc` once, using `nano ~/.bashrc`:

```bash
source /opt/ros/humble/setup.bash
[ ! -f ~/studica_ws/install/setup.bash ] || source ~/studica_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

Then run `source ~/.bashrc`.

## 7. Set up the Orbbec camera

```bash
cd ~/studica_ws
sudo install -m 0644 \
  src/OrbbecSDK_ROS2/orbbec_camera/scripts/99-obsensor-libusb.rules \
  /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo usermod -aG video "$(id -un)"
```

Log out and back in, then unplug and reconnect the camera. Run:

```bash
cd ~/studica_ws
source install/setup.bash
ros2 run orbbec_camera list_devices_node
bash scripts/start_orbbec_camera.sh
```

The project configuration uses Gemini E. Other models require configuration changes.

## 8. Set up LiDAR, transfer maps, and test the robot

The separate YDLidar SDK and driver workspace are not included when cloning the main repository.
Prepare versions matching the original robot and build them on the new VMX:

```text
/home/vmx/YDLidar-SDK
/home/vmx/ydlidar_ros2_ws
```

The mapping script expects `install/setup.bash` and
`src/ydlidar_ros2_driver/params/Tmini.yaml` in the LiDAR workspace.
Map files in directories excluded by `.gitignore` must be transferred separately.
See the [detailed setup guide (Indonesian)](VMX_STANDARD_SETUP_ID.md#7-lidar-dan-konfigurasi-robot).

Check CAN IDs, wiring, servo/sensor ports, motor polarity, and wheel parameters
before motion tests. For initial tests, lift the wheels and keep the motor stop ready.
A successful build does not verify hardware operation. Test sensors, actuators,
the camera, and LiDAR if used, then reboot and repeat the tests on the target VMX.

## 9. Download future updates

Save local changes before updating. Check `git status` in both the main workspace
and the camera repository. Preserve any local configuration changes before proceeding.

```bash
cd ~/studica_ws
git status
git pull --ff-only origin main
git -C src/OrbbecSDK_ROS2 fetch origin
ORBBEC_COMMIT=$(git rev-parse HEAD:src/OrbbecSDK_ROS2)
git -C src/OrbbecSDK_ROS2 checkout "$ORBBEC_COMMIT"

cd ~/studica_ws/drivers
make -j1
sudo make install
cd ~/studica_ws
bash scripts/build_orbbec_camera.sh
source install/setup.bash
bash scripts/setup_permissions.sh "$PWD"
```

If `git pull --ff-only` fails because of local changes or divergent history,
resolve those changes before continuing with the build.

Additional reference: [Standard VMX setup (Indonesian)](VMX_STANDARD_SETUP_ID.md).
