# Panduan Remote Keyboard AMR

Panduan ini menjalankan remote keyboard menggunakan arsitektur terbaru:

```text
Keyboard -> /cmd_vel -> inverse kinematics -> PID M0-M3 -> Titan
```

Gunakan tiga terminal. Untuk pengujian pertama, angkat robot dengan penyangga
yang kuat atau kosongkan area lantai. Tutup semua program motor lama agar tidak
ada dua node yang mengirim perintah gerak bersamaan.

## Terminal 1 — VMX, Titan, encoder, dan IMU

```bash
source /opt/ros/humble/setup.bash
source /home/vmx/studica_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

sudo -E env RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  /home/vmx/studica_ws/install/studica_control/lib/studica_control/manual_composition \
  --ros-args \
  -r __node:=control_server \
  --params-file /home/vmx/studica_ws/src/studica_control/config/titan_m1_test.yaml
```

Masukkan password `sudo` pengguna VMX ketika diminta. Jangan menyimpan password
di source code atau repository. Biarkan Terminal 1 tetap aktif.

## Terminal 2 — Drive controller, kinematika, dan PID

```bash
source /opt/ros/humble/setup.bash
source /home/vmx/studica_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

python3 \
  /home/vmx/studica_ws/src/studica_control/src/components/examples/python/drive_controller.py \
  --config /home/vmx/studica_ws/src/studica_control/config/drive_controller.yaml
```

Tunggu sampai muncul informasi seperti:

```text
Drive model=differential_all_terrain
Menunggu /cmd_vel; motor STOP
```

Jika feedback encoder belum siap, motor tetap STOP. Pastikan topic encoder
M0-M3 tersedia sebelum menjalankan keyboard.

## Terminal 3 — Aktifkan Titan dan keyboard

```bash
source /opt/ros/humble/setup.bash
source /home/vmx/studica_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

Aktifkan Titan:

```bash
ros2 service call /titan0/titan_cmd \
  studica_control/srv/SetData \
  "{params: 'enable'}"
```

Jalankan keyboard dengan kecepatan rendah dahulu:

```bash
python3 \
  /home/vmx/studica_ws/src/studica_control/src/components/examples/python/titan_keyboard_teleop.py \
  --linear-speed 0.15 \
  --angular-speed 0.8
```

## Tombol kontrol

```text
W = maju
S = mundur
A = putar kiri
D = putar kanan
G = maju sesuai target jarak encoder
E = stop
Q = stop dan keluar
```

Tombol arah harus ditahan. Ketika dilepas, keyboard mengirim `/cmd_vel` nol;
watchdog drive controller juga menghentikan motor jika perintah terputus.

Uji berurutan: tekan `W` sebentar, lalu `S`, `A`, dan `D`. Jangan menggunakan
`G` sebelum empat arah dasar dan penghentian otomatis dipastikan benar.

## Mengubah kecepatan

Contoh setelah pengujian pelan berhasil:

```bash
python3 \
  /home/vmx/studica_ws/src/studica_control/src/components/examples/python/titan_keyboard_teleop.py \
  --linear-speed 0.40 \
  --angular-speed 1.5
```

Nilai `--linear-speed` menggunakan m/s. Nilai `--angular-speed` menggunakan
rad/s. Batas kecepatan roda dan gain PID berada di:

```text
/home/vmx/studica_ws/src/studica_control/config/drive_controller.yaml
```

## Emergency stop

Tekan `E` atau `Q` pada terminal keyboard. Jika respons program tidak normal,
jalankan dari terminal lain:

```bash
source /opt/ros/humble/setup.bash
source /home/vmx/studica_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 service call /titan0/titan_cmd \
  studica_control/srv/SetData \
  "{params: 'disable'}"
```

Jika motor masih bergerak, putuskan daya 12 V Titan menggunakan emergency stop
fisik. Jangan memegang roda yang sedang berputar.

## Menghentikan seluruh program

1. Tekan `Q` atau `Ctrl+C` pada Terminal 3.
2. Tekan `Ctrl+C` pada Terminal 2.
3. Tekan `Ctrl+C` pada Terminal 1.

Urutan ini memastikan `/cmd_vel` dan duty motor menjadi nol sebelum hardware
server ditutup.

