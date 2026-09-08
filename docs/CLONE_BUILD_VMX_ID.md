# Clone GitHub dan build pada VMX baru

Panduan untuk repository https://github.com/adytPENS/AMR_SA_ID.
Jalankan perintah di VMX tujuan sebagai user `vmx`, dengan koneksi internet.
Target project: Ubuntu 22.04 ARM64 (`aarch64`), ROS 2 Humble, dan VMX HAL.

## 1. Periksa sistem

```bash
whoami
cat /etc/os-release
uname -m
ls /opt/ros
ls /usr/local/include/vmxpi/VMXPi.h
ls /usr/local/lib/vmxpi/libvmxpi_hal_cpp.so
```

Jika ROS belum tersedia, ikuti [instalasi resmi ROS 2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html), termasuk konfigurasi repository apt ROS.
Jika VMX HAL belum tersedia, pasang SDK/image yang sesuai dari [Studica](https://learn.studica.com/docs/ws/vmx/os-images).
ROS dan VMX HAL tidak ikut saat clone. Jangan lanjut ke build jika keduanya belum tersedia.

## 2. Clone program

Gunakan folder tujuan baru. Jika `~/studica_ws` sudah berisi pekerjaan lain,
simpan atau pindahkan workspace tersebut terlebih dahulu.

```bash
sudo apt update
sudo apt install -y git
cd ~
git clone https://github.com/adytPENS/AMR_SA_ID.git studica_ws
cd ~/studica_ws
```

Saat panduan ini dibuat, `src/OrbbecSDK_ROS2` tersimpan sebagai gitlink tanpa
`.gitmodules`. Clone utama belum mengambil isi repository kamera. Ambil secara
manual pada commit yang direferensikan project:

```bash
git clone https://github.com/orbbec/OrbbecSDK_ROS2.git src/OrbbecSDK_ROS2
ORBBEC_COMMIT=$(git rev-parse HEAD:src/OrbbecSDK_ROS2)
git -C src/OrbbecSDK_ROS2 checkout "$ORBBEC_COMMIT"
ls src/studica_control/config/
```

Checkout kamera dalam keadaan detached HEAD adalah normal untuk versi yang dipatok.
File `D_nav.yaml` dan `misi_forward_gripper.yaml` ada di folder konfigurasi tersebut.

## 3. Install dependency

Perintah ini mengasumsikan repository apt ROS Humble sudah aktif.

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

Selesaikan dependency yang gagal sebelum build. Gunakan Python sistem, bukan Conda/venv.
Rosdep memasang dependency yang tercatat dalam `package.xml`, termasuk kebutuhan navigasi.

Paket tambahan sesuai fitur:

| Fitur | Paket apt |
| --- | --- |
| SSH dari laptop | `openssh-server` |
| Gamepad | `ros-humble-joy` |
| Foxglove | `ros-humble-foxglove-bridge` |

Untuk mengaktifkan SSH:

```bash
sudo apt install -y openssh-server
sudo systemctl enable --now ssh
hostname -I
```

Dari laptop, gunakan `ssh vmx@IP_VMX` atau VS Code Remote - SSH.

## 4. Build driver Studica

```bash
cd ~/studica_ws/drivers
make clean
make -j1
sudo make install
```

## 5. Build workspace ROS

Gunakan terminal yang belum memuat overlay workspace lama. Jangan menyalin
`build/`, `install/`, atau `log/` dari mesin lain dan jangan `sudo colcon build`.

```bash
cd ~/studica_ws
bash scripts/build_orbbec_camera.sh
source /opt/ros/humble/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

Skrip membangun `studica_control` dan dependency workspace, termasuk Orbbec,
dengan satu worker untuk membatasi penggunaan RAM. Lanjutkan hanya jika build berhasil.

## 6. Permission dan verifikasi

```bash
cd ~/studica_ws
bash scripts/setup_permissions.sh "$PWD"
sudo visudo -c
ldd install/studica_control/lib/studica_control/manual_composition
ros2 pkg prefix studica_control
bash scripts/check_vmx_setup.sh
```

Jalankan skrip permission sebagai user `vmx`; skrip menggunakan sudo untuk
konfigurasi sistem yang diperlukan. Selesaikan library `not found` dan hasil
`FAIL` dari pemeriksaan setup sebelum menjalankan hardware.

Tambahkan sekali ke `~/.bashrc` menggunakan `nano ~/.bashrc`:

```bash
source /opt/ros/humble/setup.bash
[ ! -f ~/studica_ws/install/setup.bash ] || source ~/studica_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

Kemudian jalankan `source ~/.bashrc`.

## 7. Kamera Orbbec

```bash
cd ~/studica_ws
sudo install -m 0644 \
  src/OrbbecSDK_ROS2/orbbec_camera/scripts/99-obsensor-libusb.rules \
  /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo usermod -aG video "$(id -un)"
```

Logout/login ulang dan cabut-pasang kamera, lalu:

```bash
cd ~/studica_ws
source install/setup.bash
ros2 run orbbec_camera list_devices_node
bash scripts/start_orbbec_camera.sh
```

Konfigurasi project menggunakan Gemini E. Model lain memerlukan penyesuaian.

## 8. LiDAR, peta, dan uji robot

YDLidar SDK dan workspace driver terpisah tidak ikut clone repository utama.
Siapkan versi yang sesuai robot lama dan build pada VMX baru:

```text
/home/vmx/YDLidar-SDK
/home/vmx/ydlidar_ros2_ws
```

Skrip mapping mengharapkan `install/setup.bash` dan
`src/ydlidar_ros2_driver/params/Tmini.yaml` di workspace LiDAR.
File peta dalam folder yang diabaikan `.gitignore` perlu ditransfer terpisah.
Lihat [panduan setup lengkap](VMX_STANDARD_SETUP_ID.md#7-lidar-dan-konfigurasi-robot).

Cocokkan CAN ID, wiring, port servo/sensor, polaritas motor, dan parameter roda
sebelum uji gerak. Uji awal dengan roda terangkat dan penghentian motor siap.
Build berhasil belum membuktikan hardware berfungsi; uji sensor, aktuator,
kamera, LiDAR bila dipakai, lalu reboot dan uji ulang pada VMX tujuan.

## 9. Mengambil update berikutnya

Simpan perubahan lokal sebelum update; periksa `git status` pada workspace
utama dan repository kamera. Jangan menimpa konfigurasi lokal yang belum disimpan.

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

Jika `git pull --ff-only` gagal karena perubahan lokal atau riwayat bercabang,
selesaikan perubahan tersebut sebelum melanjutkan build.

Panduan tambahan: [Setup VMX standar](VMX_STANDARD_SETUP_ID.md).
