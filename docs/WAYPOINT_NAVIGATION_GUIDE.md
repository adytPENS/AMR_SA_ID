# Navigasi Waypoint Tanpa SLAM

Program ini ditujukan untuk pembelajaran dan tugas ketika koordinat titik
diberikan oleh juri. Robot memakai pose lokal dari encoder dan IMU, lalu
bergerak menuju A, B, C, dan D sesuai urutan yang diberikan.

## Sistem koordinat

Robot harus ditempatkan pada titik asal dengan arah yang diketahui:

```text
                    +Y (kiri)
                       ^
                       |
                       |
  titik start (0,0) ---+----> +X (depan robot)
```

Semua koordinat memakai meter. Ada dua mode:

```text
coordinate_mode: map    titik dan start_pose mengikuti frame peta juri
coordinate_mode: local  titik langsung relatif terhadap start robot
```

Pada mode `map`, isi pose robot saat start:

```yaml
coordinate_mode: map
start_pose: {x: 0.50, y: 0.30, yaw_deg: 90.0}
```

Program melakukan translasi dan rotasi secara otomatis agar odometri tetap
dimulai dari `(0,0,0)`. Contoh titik peta:

```yaml
A: {x: 2.0, y: 1.0}
```

Tanpa global localization, robot harus selalu ditempatkan pada origin dan
heading yang sama sebelum odometri di-reset. Kesalahan encoder akan
terakumulasi sepanjang lintasan.

## Mengisi koordinat dan urutan

Edit:

```text
/home/vmx/studica_ws/src/studica_control/config/waypoints.yaml
```

Contoh:

```yaml
coordinate_mode: map
start_pose: {x: 0.40, y: 0.30, yaw_deg: 0.0}

waypoints:
  A: {x: 1.20, y: 0.50}
  B: {x: 2.60, y: 0.50}
  C: {x: 2.60, y: 1.70}
  D: {x: 0.80, y: 1.70}

sequence: [A, B, D, C]
```

## Menjalankan

Jalankan `control_server`, zero IMU, inisialisasi encoder, dan driver LiDAR
seperti pada panduan hardware. Jangan menjalankan keyboard bersamaan dengan
navigator karena keduanya menerbitkan perintah motor.

Cara paling mudah adalah menjalankan skrip berikut setelah hardware siap:

```bash
cd /home/vmx/studica_ws
bash scripts/start_waypoint_mode.sh
```

Skrip menjalankan odometri, mereset pose ke `(0,0,0)`, dan memuat navigator.
Motor masih STOP sampai service start dipanggil.

Cara manual untuk menjalankan odometri tanpa SLAM:

```bash
source /opt/ros/humble/setup.bash
source /home/vmx/studica_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

python3 /home/vmx/studica_ws/src/studica_control/src/components/examples/python/wheel_odometry.py \
  --ros-args \
  --params-file /home/vmx/studica_ws/src/studica_control/config/wheel_odometry.yaml
```

Letakkan robot pada origin, lalu reset pose:

```bash
ros2 service call /wheel_odometry/reset std_srvs/srv/Empty "{}"
```

Jalankan navigator:

```bash
python3 /home/vmx/studica_ws/src/studica_control/src/components/examples/python/waypoint_navigator.py \
  --config /home/vmx/studica_ws/src/studica_control/config/waypoints.yaml
```

Program belum bergerak sampai service start dipanggil:

```bash
ros2 service call /waypoint_navigator/start std_srvs/srv/Trigger "{}"
```

Stop darurat dari terminal mana pun:

```bash
ros2 service call /waypoint_navigator/stop std_srvs/srv/Trigger "{}"
```

`Ctrl+C` pada terminal navigator juga mengirim nol ke semua motor.

Urutan dari juri dapat diberikan tanpa mengedit YAML:

```bash
python3 /home/vmx/studica_ws/src/studica_control/src/components/examples/python/waypoint_navigator.py \
  --config /home/vmx/studica_ws/src/studica_control/config/waypoints.yaml \
  --sequence A,B,D,C
```

## Obstacle avoidance

LiDAR memeriksa sektor depan. Pada jarak kurang dari `0.55 m`, robot:

1. berhenti;
2. membandingkan ruang kiri dan kanan;
3. berputar sekitar 55 derajat ke sisi yang lebih lapang;
4. maju sekitar 0.60 m;
5. menghitung ulang arah menuju waypoint.

Ini adalah avoidance reaktif, bukan perencanaan jalur global. Sistem dapat
gagal pada lorong sempit, obstacle berbentuk U, atau lingkungan padat. Tombol
stop/operator tetap wajib tersedia.

Jika garis lurus antara dua titik tertutup dinding tetap pada peta, tambahkan
titik perantara pada koridor, misalnya `AB1` dan `AB2`, lalu masukkan ke
sequence. Obstacle avoidance terutama ditujukan untuk block yang baru
diletakkan juri, bukan menggantikan perencanaan rute dari peta.

Karena LiDAR saat ini dipasang rendah, pastikan sektor depan tidak membaca
rangka atau roda robot sebagai obstacle. Uji arah kiri/kanan LiDAR sebelum
mengaktifkan avoidance pada lintasan lomba.
