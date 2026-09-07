# Setup VMX standar untuk studica_ws

Panduan berdasarkan source project ini, ditinjau 6 September 2026. Target yang
terlihat pada mesin pengembangan: Ubuntu 22.04, ARM64, ROS 2 Humble. Jangan
menganggap semua image Studica sudah berisi versi OS, ROS, dan SDK yang sama.

`CMakeLists.txt` mengatur compile dan install hasil build, **bukan** menginstal
SSH, VS Code Server, paket apt, VMX HAL, atau permission Linux. Dependency ROS
tercatat di `src/studica_control/package.xml`; gunakan rosdep untuk memasangnya.

## 1. Identifikasi VMX sebelum mengubah sistem

```bash
cat /etc/os-release
uname -m
ls /opt/ros
ls /usr/local/include/vmxpi/VMXPi.h
ls /usr/local/lib/vmxpi/libvmxpi_hal_cpp.so
```

Target panduan ini Ubuntu 22.04 + aarch64 + Humble. Bila berbeda, gunakan image
Studica yang sesuai hardware dan project; jangan asal upgrade distribusi atau
menyalin library ARM dari mesin lain. Backup konfigurasi robot sebelum reimage.
VMX HAL berasal dari Studica, bukan dari folder workspace atau pip. Bila hilang,
pulihkan SDK/image sesuai [dokumentasi Studica](https://learn.studica.com/docs/ws/vmx/os-images).
Untuk ROS yang belum tersedia, ikuti [instalasi resmi Humble untuk Ubuntu](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
terlebih dahulu, termasuk repository apt ROS. Perintah apt berikut mengasumsikan
repository tersebut sudah aktif.

## 2. SSH dan VS Code

Pada VMX:

```bash
sudo apt update
sudo apt install -y openssh-server curl wget ca-certificates tar gzip unzip rsync git
sudo systemctl enable --now ssh
hostname -I
```

Pada laptop: install VS Code dan extension **Remote - SSH**, lalu uji
`ssh vmx@IP_VMX` dari terminal. Setelah berhasil, pilih **Remote-SSH: Connect to
Host**, masuk sebagai `vmx`, dan buka `/home/vmx/studica_ws`.

VS Code Server otomatis dipasang oleh Remote SSH di VMX, umumnya dalam
`~/.vscode-server`; tidak perlu install VS Code desktop atau Node.js di VMX.
Laptop/VMX perlu jalur download yang berfungsi pada koneksi pertama. Jika VMX
tidak memiliki internet tetapi laptop punya, gunakan setting VS Code laptop:

```json
"remote.SSH.localServerDownload": "always"
```

Uji koneksi sebelum kompetisi/offline; update VS Code dapat membutuhkan server
versi baru. Lihat [mekanisme instalasi server](https://code.visualstudio.com/docs/remote/ssh)
dan [syarat Linux terkini](https://code.visualstudio.com/docs/remote/linux).

Jika gagal: cek panel Output → Remote - SSH, `df -h ~`, `ls -ld ~ ~/.vscode-server`,
dan pastikan SSH terminal biasa berhasil. Jika direktori server **memang dimiliki
root akibat instalasi sebelumnya**, perbaiki sebagai user `vmx`:

```bash
sudo chown -R --no-dereference "$(id -un):$(id -gn)" "$HOME/.vscode-server"
```

## 3. Transfer project tanpa membawa masalah permission

Gunakan user `vmx`, lokasi `/home/vmx/studica_ws`; beberapa skrip masih memakai
path tersebut secara tetap. Salin **source project modifikasi ini**, bukan hanya
clone repository Studica upstream yang belum tentu memuat program tambahan.
Transfer `src/`, `drivers/`, `scripts/`, `config/`, `maps/`, dan dokumentasi.
Jangan transfer `build/`, `install/`, `log/`, object driver, atau cache Python.
`install/` bisa berisi symlink dan path absolut ke mesin lama.

Pilihan paling mudah: rename workspace lama sebagai backup, buat folder baru
sebagai `vmx`, lalu upload source ke sana. Contoh dari laptop Linux/WSL, setelah
folder tujuan disiapkan (tanpa `--delete`):

```bash
rsync -rtv --no-perms --no-owner --no-group \
  --exclude build/ --exclude install/ --exclude log/ --exclude logs/ \
  --exclude .git/ --exclude __pycache__/ --exclude '*.o' --exclude '*.d' \
  --exclude 'libstudica_drivers.so' \
  ./studica_ws/ vmx@IP_VMX:/home/vmx/studica_ws/
```

Jika workspace hasil copy sudah mengalami `Permission denied`, cek dahulu:

```bash
whoami
ls -ld /home/vmx/studica_ws /home/vmx/studica_ws/src
namei -l /home/vmx/studica_ws/scripts/start_full_keyboard.sh
findmnt -T /home/vmx/studica_ws
```

Untuk workspace yang seharusnya milik user `vmx`, jalankan **login sebagai vmx**:

```bash
sudo chown -R --no-dereference "$(id -un):$(id -gn)" /home/vmx/studica_ws
chmod -R u+rwX /home/vmx/studica_ws
find /home/vmx/studica_ws/scripts -type f -name '*.sh' -exec chmod u+x {} +
```

Jangan `chmod -R 777`, jangan `sudo colcon build`, dan jangan menjalankan editor
sebagai root. Bila filesystem read-only atau `noexec`, chown/chmod bukan solusinya;
periksa mount/SD card. Error `$'\r'` atau `bad interpreter` setelah transfer Windows
menandakan line ending CRLF: ubah file skrip ke LF melalui VS Code.

## 4. Paket yang diperlukan

| Bagian | Kebutuhan |
|---|---|
| Build | build-essential, cmake, pkg-config, colcon, rosdep |
| Hardware | VMX HAL dari Studica + library dari `drivers/Makefile` |
| ROS dasar | Humble ros-base, dependency package.xml, CycloneDDS |
| Python vision | python3-opencv, python3-numpy, python3-yaml |
| Orbbec | Source dan SDK bundled di `src/OrbbecSDK_ROS2`, dependency ROS/C++, udev, akses video |
| Navigasi | nav2_bringup, nav2_msgs, slam_toolbox; dipasang lewat rosdep |
| LiDAR | YDLidar SDK dan workspace driver terpisah, khusus mode yang menggunakan scan |
| Visualisasi tambahan | foxglove_bridge; joy untuk gamepad; opsional |

```bash
sudo apt install -y build-essential cmake pkg-config python3-colcon-common-extensions \
  python3-rosdep ros-humble-ros-base ros-humble-rmw-cyclonedds-cpp \
  python3-opencv python3-numpy python3-yaml \
  libopencv-dev libeigen3-dev libusb-1.0-0-dev libudev-dev \
  libgflags-dev libgoogle-glog-dev nlohmann-json3-dev libdw-dev \
  ros-humble-image-transport-plugins ros-humble-compressed-image-transport \
  usbutils v4l-utils

# Hanya jika rosdep belum pernah diinisialisasi:
# sudo rosdep init
rosdep update
source /opt/ros/humble/setup.bash
cd /home/vmx/studica_ws
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
```

Jika rosdep mengatakan belum diinisialisasi, jalankan `sudo rosdep init`, lalu
ulangi update/install. Jangan mengabaikan dependency yang gagal resolve/install.
Gunakan Python sistem `/usr/bin/python3` untuk ROS ini; environment Conda/venv
berbeda dapat membuat `rclpy` atau OpenCV tidak terbaca.

Opsional sesuai fitur:

```bash
sudo apt install -y ros-humble-foxglove-bridge ros-humble-joy
```

## 5. Build driver dan workspace pada VMX tujuan

Jalankan dari terminal baru yang belum source overlay workspace lama. Bila
menggunakan folder lama, **pindahkan** `build`, `install`, dan `log` ke backup
terlebih dahulu; jangan memakai cache hasil transfer.

```bash
cd /home/vmx/studica_ws/drivers
make clean
make -j1
sudo make install
cd /home/vmx/studica_ws
bash scripts/build_orbbec_camera.sh
source /opt/ros/humble/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
bash scripts/setup_permissions.sh "$PWD"
sudo visudo -c
```

Skrip build sudah membatasi parallelism ke satu worker untuk RAM Raspberry Pi.
Skrip permission existing memasang konfigurasi linker, sudoers untuk executable
hardware, dan rule I2C. Ini perubahan sistem yang diperlukan launch hardware saat
ini karena memakai pigpio/root. Jalankan skrip sebagai `vmx`, **bukan**
`sudo bash scripts/setup_permissions.sh`. Build tetap dilakukan sebagai user biasa.
Jika lokasi workspace berubah, jalankan ulang setup permission.

Untuk memeriksa library yang belum ditemukan:

```bash
ldd install/studica_control/lib/studica_control/manual_composition
ldd /usr/local/lib/vmxpi/libvmxpi_hal_cpp.so
```

Jika ada `not found`, selesaikan library tersebut sebelum launch.

Tambahkan sekali ke `~/.bashrc` bila belum ada:

```bash
source /opt/ros/humble/setup.bash
[ ! -f /home/vmx/studica_ws/install/setup.bash ] || source /home/vmx/studica_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

## 6. Orbbec Gemini E

Project ini menggunakan `gemini_e.launch.py`, driver bundled versi 1.5.22, dan
SDK ARM64 1.10.37. Pertahankan versi source/SDK yang ada; jangan mengganti dengan
branch terbaru tanpa memeriksa dukungan model/firmware. Untuk model selain Gemini E,
launch dan konfigurasi harus disesuaikan.

```bash
cd /home/vmx/studica_ws
sudo install -m 0644 src/OrbbecSDK_ROS2/orbbec_camera/scripts/99-obsensor-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo usermod -aG video "$(id -un)"
```

Logout/login ulang agar group aktif, lalu cabut-pasang kamera. Mem-build driver
saja tidak memasang rule ke `/etc`: CMake bundled menaruhnya di share package.
Gemini E pada konfigurasi project memakai USB 2.0; gunakan kabel data dan suplai
daya yang stabil. Periksa tanpa menjalankan motor:

```bash
lsusb
v4l2-ctl --list-devices
source /opt/ros/humble/setup.bash
source /home/vmx/studica_ws/install/setup.bash
ros2 run orbbec_camera list_devices_node
bash /home/vmx/studica_ws/scripts/start_orbbec_camera.sh
```

Di terminal ROS kedua: `ros2 topic list`, kemudian cek topic image/depth yang
benar-benar muncul dengan `ros2 topic hz NAMA_TOPIC`. Jika kamera tidak terlihat
pada `lsusb`, periksa kabel/daya dahulu; jika terlihat tetapi access denied,
periksa udev dan group. Tutup proses kamera lain jika device busy.

Viewer OpenCV memerlukan display grafis. Terminal Remote SSH biasa tidak otomatis
menyediakan display; untuk melihat dari laptop gunakan Foxglove bridge atau desktop
remote yang sudah dikonfigurasi. Kamera headless tetap dapat publish topic.

## 7. LiDAR dan konfigurasi robot

`start_full_mapping.sh` mengharapkan
`/home/vmx/ydlidar_ros2_ws/install/setup.bash` dan
`src/ydlidar_ros2_driver/params/Tmini.yaml` di workspace itu. Keduanya **tidak ikut**
ketika menyalin studica_ws. Siapkan YDLidar SDK dan driver versi yang sudah digunakan
robot, build ulang pada VMX tujuan, dan cocokkan port serial/baudrate/model di YAML.
Jangan menganggap dependency ini selesai melalui rosdep studica_ws.

Jika serial device memakai group dialout, tambahkan user lalu login ulang:

```bash
sudo usermod -aG dialout "$(id -un)"
ls -l /dev/serial/by-id/
```

Sebelum menjalankan mode robot, cocokkan CAN ID Titan, port servo/sensor, polaritas
motor, ukuran roda, encoder, dan konfigurasi YAML dengan robot tujuan. Uji awal
dengan roda terangkat dan penghentian motor siap digunakan.

## 8. Urutan penerimaan VMX baru

1. SSH dan edit/save file melalui VS Code berhasil sebagai `vmx`.
2. `bash scripts/check_vmx_setup.sh` dijalankan; selesaikan FAIL dan tinjau INFO.
3. rosdep sukses, driver dan colcon build sukses, `ldd` tanpa `not found`.
4. Orbbec bisa enumerate dan mengirim RGB/depth sebagai user biasa.
5. Hardware launch berhasil; uji sensor lalu aktuator satu per satu.
6. Jika memakai LiDAR, pastikan `/scan` tersedia sebelum mapping/navigation.
7. Uji mode yang dipakai kompetisi, reboot, lalu ulangi start tanpa instalasi tambahan.
8. Simpan backup image yang sudah lulus uji dan source/config yang sama.

Panduan dan preflight bukan bukti hardware sudah lulus: pengujian streaming,
CAN/motor, dan reboot tetap harus dilakukan pada VMX tujuan.
