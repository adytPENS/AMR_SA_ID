# Panduan Pengujian VMX2, Titan, IMU, dan YDLIDAR

Dokumen ini mencatat konfigurasi dan prosedur pengujian robot AMR berbasis
VMX2 Pi, Titan motor controller, navX IMU, dan YDLIDAR T-mini Plus.

## 1. Konfigurasi robot

- Sistem operasi: Ubuntu 22.04
- ROS: ROS 2 Humble
- RMW: Cyclone DDS
- Titan CAN ID: `42`
- Motor: Studica Maverick 50.9:1, 12 V, encoder 1221.6 count/rev
- Diameter roda: `0.12 m`
- Jarak per tick: `0.000308604386 m/tick`
- Jenis penggerak: empat roda biasa, skid-steer/differential
- Jarak roda kiri-kanan: `0.35 m`
- Jarak roda depan-belakang: `0.29 m`
- Ukuran badan robot: `0.44 x 0.40 m`

Pemetaan motor:

| Kanal | Posisi | Duty untuk maju |
|---|---|---:|
| M0 | depan kanan | negatif |
| M1 | belakang kanan | negatif |
| M2 | depan kiri | positif |
| M3 | belakang kiri | positif |

Posisi LiDAR terhadap `base_link`:

```text
x = +0.20 m
y =  0.00 m
z = +0.10 m
roll = pitch = yaw = 0
```

Orientasi LiDAR sudah diperiksa di RViz: objek di depan robot tampil pada
arah depan robot.

## 2. Persiapan setiap terminal

Untuk program di workspace Studica:

```bash
source /opt/ros/humble/setup.bash
source /home/vmx/studica_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

Untuk driver YDLIDAR:

```bash
source /opt/ros/humble/setup.bash
source /home/vmx/ydlidar_ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

Gunakan nilai `RMW_IMPLEMENTATION` yang sama pada seluruh terminal agar node
ROS dapat saling menemukan.

## 3. Menjalankan VMX2, Titan, dan IMU

Jalankan komponen hardware menggunakan executable langsung. Cara ini
menghindari error `sudo: ros2: command not found` dan error metadata
`ros2cli`.

```bash
source /opt/ros/humble/setup.bash
source /home/vmx/studica_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

sudo -E /home/vmx/studica_ws/install/studica_control/lib/studica_control/manual_composition \
  --ros-args \
  -r __node:=control_server \
  --params-file /home/vmx/studica_ws/src/studica_control/config/titan_m1_test.yaml
```

Biarkan terminal ini berjalan. Jangan menjalankan dua instance
`manual_composition` secara bersamaan.

## 4. Membaca IMU

Konfigurasi `titan_m1_test.yaml` mengaktifkan Titan dan IMU dalam satu proses.
Pastikan proses pada bagian 3 sedang berjalan, lalu lihat topic:

```bash
ros2 topic list | grep imu
```

Baca IMU terus-menerus:

```bash
ros2 topic echo /imu
```

Baca satu pesan saja:

```bash
ros2 topic echo /imu --once
```

Periksa frekuensi:

```bash
ros2 topic hz /imu
```

Jika `/imu` tidak muncul, pastikan proses memakai file
`titan_m1_test.yaml` terbaru dan tidak ada `manual_composition` lain yang
sedang mengakses VMX.

## 5. Menjalankan motor dengan keyboard

### Keselamatan

- Uji pertama dengan roda diangkat dari lantai.
- Gunakan duty rendah, misalnya `0.10`.
- Tombol `E` menghentikan semua motor.
- `Ctrl+C` juga menghentikan semua motor melalui blok pembersihan program.

Pastikan proses Titan pada bagian 3 masih berjalan, lalu buka terminal baru:

```bash
source /opt/ros/humble/setup.bash
source /home/vmx/studica_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

python3 /home/vmx/studica_ws/src/studica_control/src/components/examples/python/titan_keyboard_teleop.py \
  --duty 0.10 --turn-duty 0.18
```

Kontrol:

| Tombol | Gerakan |
|---|---|
| W | maju |
| S | mundur |
| A | putar kiri |
| D | putar kanan |
| G | maju otomatis sesuai target encoder (default 1 meter) |
| E | stop atau batalkan gerakan G |
| Q | stop dan keluar |

Program memakai key-repeat terminal. Setelah tombol dilepas, motor berhenti
otomatis dalam waktu sekitar `0.65 detik`.

Untuk uji otomatis satu meter, jalankan dengan duty rendah:

```bash
python3 /home/vmx/studica_ws/src/studica_control/src/components/examples/python/titan_keyboard_teleop.py \
  --duty 0.10 --distance 1.0
```

Tekan `G` satu kali. Program mengambil posisi awal empat encoder dan berhenti
ketika rata-rata perjalanan roda mencapai target. `E` selalu dapat dipakai
untuk membatalkan gerakan. Safety timeout dan deteksi encoder stall juga akan
menghentikan motor jika pembacaan bermasalah.

## 6. Menguji satu motor

Contoh menjalankan M1 selama 3 detik dengan duty rendah:

```bash
python3 /home/vmx/studica_ws/src/studica_control/src/components/examples/python/titan_m1_test.py \
  --motor 1 \
  --duty 0.10 \
  --duration 3
```

Nilai `--motor` mengikuti kanal Titan yang sebenarnya: `0`, `1`, `2`, atau
`3`.

## 7. Membaca encoder Titan

Daftar topic:

```bash
ros2 topic list | grep titan0
```

Baca salah satu encoder:

```bash
ros2 topic echo /titan0/m_0/encoder
```

Ganti `m_0` menjadi `m_1`, `m_2`, atau `m_3` untuk motor lain. Putar roda
dengan tangan ketika motor berhenti. Nilai harus berubah.

Jika perlu, aktifkan kanal encoder melalui service:

```bash
ros2 service call /titan0/titan_cmd studica_control/srv/SetData \
  "{params: 'setup_encoder', initparams: {n_encoder: 0}}"
```

Ulangi dengan `n_encoder` 1, 2, dan 3.

Status saat dokumen ini dibuat: topic encoder terbit sekitar 20 Hz, tetapi
semua nilainya masih `0.0`. Periksa kabel encoder motor yang terpisah dari
dua kabel daya motor. Encoder membutuhkan `5V`, `GND`, channel `A`, dan
channel `B`, serta harus masuk ke kanal Titan yang sesuai.

## 8. Menjalankan YDLIDAR T-mini Plus

Perangkat yang telah terdeteksi:

```text
Port       : /dev/ttyUSB0
Baud rate  : 230400
Topic      : /scan
Frame      : laser_frame
Frekuensi  : sekitar 10 Hz
```

Jalankan driver:

```bash
source /opt/ros/humble/setup.bash
source /home/vmx/ydlidar_ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 launch ydlidar_ros2_driver ydlidar_launch.py \
  params_file:=/home/vmx/ydlidar_ros2_ws/src/ydlidar_ros2_driver/params/Tmini.yaml
```

Periksa topic dan frekuensi di terminal lain:

```bash
ros2 topic echo /scan --once
ros2 topic hz /scan
```

Hasil pengujian yang baik adalah sekitar `10.035 Hz` dengan interval sekitar
`0.097` sampai `0.102 detik`.

Transformasi pada `ydlidar_launch.py` harus menggunakan:

```python
arguments=[
    '0.20', '0.0', '0.10',
    '0', '0', '0', '1',
    'base_link', 'laser_frame'
]
```

File driver berada di workspace terpisah:

```text
/home/vmx/ydlidar_ros2_ws/src/ydlidar_ros2_driver/launch/ydlidar_launch.py
```

## 9. Melihat LiDAR dengan RViz

Biarkan driver LiDAR berjalan, lalu buka terminal baru:

```bash
source /opt/ros/humble/setup.bash
source /home/vmx/ydlidar_ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
rviz2
```

Pengaturan RViz:

```text
Global Options / Fixed Frame : base_link
Display                      : LaserScan
Topic                        : /scan
Reliability Policy           : Best Effort
Durability Policy            : Volatile
Style                        : Points
Size (Pixels)                : 2 atau 3
Decay Time                   : 0
```

Jika muncul `Error subscribing: empty topic name`, isi properti Topic dengan
`/scan`. Jika status menunjukkan nol pesan meskipun `/scan` aktif, ubah
Reliability Policy dari `Reliable` menjadi `Best Effort`.

## 10. Urutan mematikan sistem

1. Tekan `E` pada program keyboard.
2. Hentikan teleop dengan `Q` atau `Ctrl+C`.
3. Hentikan RViz dengan `Ctrl+C` atau tutup jendela.
4. Hentikan driver LiDAR dengan `Ctrl+C`.
5. Hentikan `manual_composition` paling akhir dengan `Ctrl+C`.

Pastikan semua motor sudah berhenti sebelum mematikan suplai 12 V.

## 11. Syarat sebelum membuat peta

SLAM membutuhkan rangkaian transformasi berikut:

```text
map -> odom -> base_link -> laser_frame
```

Saat ini `/scan`, `base_link -> laser_frame`, dan keempat encoder sudah
berfungsi. Node `wheel_odometry.py` menyediakan `/odom` serta
`odom -> base_link`. Validasi arah dan skala odometri dilakukan sebelum peta
final digunakan untuk navigasi.

### Menjalankan odometri dan SLAM tanpa build C++

Encoder M0-M3 sudah tervalidasi. Arah encoder M0 dan M1 dibalik agar semua
jarak bernilai positif ketika robot maju. Setelah `control_server` aktif,
inisialisasi seluruh encoder dengan:

```bash
cd /home/vmx/studica_ws
bash scripts/init_titan_encoders.sh
```

Jalankan hardware Titan dan LiDAR terlebih dahulu, kemudian pada terminal
mapping:

```bash
cd /home/vmx/studica_ws
bash scripts/start_mapping.sh
```

Skrip ini memeriksa `/scan` dan keempat topic encoder, lalu menjalankan:

```text
wheel_odometry.py -> /odom dan odom -> base_link
slam_toolbox      -> /map dan map -> odom
```

Pada terminal lain, buka RViz:

```bash
source /opt/ros/humble/setup.bash
source /home/vmx/studica_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
rviz2
```

Atur `Fixed Frame` menjadi `map`, kemudian tambahkan display `Map` dengan
topic `/map`, `LaserScan` dengan topic `/scan` dan QoS `Best Effort`, serta
`TF`.

Gerakkan robot perlahan memakai keyboard dengan duty `0.10`. Hindari gerakan
cepat dan putaran mendadak. Untuk hasil loop closure yang baik, kembalikan
robot mendekati posisi awal sebelum menyimpan peta.

Simpan peta di terminal baru:

```bash
cd /home/vmx/studica_ws
bash scripts/save_map.sh arena_map
```

Hasilnya:

```text
/home/vmx/studica_ws/maps/arena_map.yaml
/home/vmx/studica_ws/maps/arena_map.pgm
```

## 12. Backup ke GitHub

Periksa perubahan:

```bash
cd /home/vmx/studica_ws
git status
```

Tambahkan hanya source dan dokumentasi yang diperlukan:

```bash
git add docs/VMX2_HARDWARE_TEST_GUIDE.md
git add src/studica_control/config/titan_m1_test.yaml
git add src/studica_control/src/components/examples/python/titan_keyboard_teleop.py
git add src/studica_control/src/components/examples/python/titan_m1_test.py
```

Buat commit dan kirim ke GitHub:

```bash
git commit -m "Document VMX2 hardware test procedures"
git push origin main
```

Repository remote:

```text
https://github.com/adytPENS/AMR_SA_ID.git
```

Jangan commit direktori `build/`, `install/`, `log/`, atau executable hasil
kompilasi apabila source code-nya sudah tersedia.
