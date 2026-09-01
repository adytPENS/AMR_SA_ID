# Navigasi Waypoint AMR — Bahasa Indonesia

English version: [WAYPOINT_NAVIGATION_GUIDE_EN.md](WAYPOINT_NAVIGATION_GUIDE_EN.md)

Panduan ini menjelaskan alur lengkap untuk menjalankan robot dari koordinat
waypoint A, B, C, dan D menggunakan encoder, IMU, PID speed setiap motor, dan
LiDAR untuk obstacle avoidance.

## 1. Arsitektur program

```text
Koordinat A/B/C/D + urutan
              |
              v
     waypoint_navigator.py
              |  /cmd_vel (linear.x, angular.z)
              v
       drive_controller.py
              |  inverse kinematics sesuai model base
              v
        target speed M0-M3
              |
              v
       MotorSpeedPID M0-M3 <--- encoder M0-M3
              |
              v
         duty Titan M0-M3

IMU + encoder ---> wheel_odometry.py ---> /odom ---> waypoint navigator
LiDAR ----------> /scan ----------------> obstacle avoidance
```

Hanya `drive_controller.py` yang boleh menerbitkan duty Titan. Keyboard dan
waypoint hanya menerbitkan `/cmd_vel`. Jangan menjalankan keyboard dan waypoint
bersamaan karena keduanya merupakan sumber `/cmd_vel`.

## 2. File yang digunakan dan diubah

Siswa umumnya hanya perlu mengubah dua file YAML berikut:

| File | Fungsi | Kapan diubah |
|---|---|---|
| `config/waypoints.yaml` | Koordinat, urutan, speed waypoint, dan obstacle avoidance | Saat menerima peta/titik dari juri |
| `config/drive_controller.yaml` | Model base, diameter roda, jarak roda, PID, polarity | Saat model atau ukuran robot berubah |

File pendukung yang biasanya tidak perlu diedit saat lomba:

| File | Fungsi |
|---|---|
| `config/titan_m1_test.yaml` | Titan CAN ID 42, encoder, IMU, dan DIO |
| `config/wheel_odometry.yaml` | Track width, frame odom, dan penggunaan yaw IMU |
| `waypoint_navigator.py` | Menghitung arah/jarak tujuan dan menerbitkan `/cmd_vel` |
| `drive_controller.py` | Kinematika, PID M0-M3, watchdog, dan output duty Titan |
| `drive_kinematics.py` | Inverse kinematics empat model base |
| `motor_speed_pid.py` | PID speed reusable untuk satu motor |
| `wheel_odometry.py` | Menghasilkan `/odom` dari encoder dan IMU |
| `scripts/start_waypoint_mode.sh` | Menjalankan odometry, drive controller, dan navigator |

Semua path di atas berada di bawah:

```text
/home/vmx/studica_ws/src/studica_control/
```

Kecuali skrip startup yang berada di:

```text
/home/vmx/studica_ws/scripts/start_waypoint_mode.sh
```

## 3. Pilih profil pengujian atau lomba

### Pengujian dasar tanpa obstacle avoidance

Gunakan:

```text
/home/vmx/studica_ws/src/studica_control/config/waypoints_test.yaml
```

Profil ini memiliki satu titik lokal sejauh 0,50 m dan:

```yaml
obstacle_avoidance:
  enabled: false
```

LiDAR tidak diperlukan hanya untuk pengujian ini. Area harus kosong dan
operator harus siap menghentikan robot. Mode ini bukan obstacle avoidance.

### Waypoint dengan obstacle avoidance

Gunakan:

```text
/home/vmx/studica_ws/src/studica_control/config/waypoints.yaml
```

Konfigurasinya memiliki:

```yaml
obstacle_avoidance:
  enabled: true
```

Pada mode ini LiDAR wajib aktif dan topic `/scan` wajib tersedia. Jika `/scan`
tidak ada atau stale, navigator menolak start atau menghentikan robot.

## 4. Sistem koordinat

Konvensi koordinat ROS:

```text
                    +Y (kiri)
                       ^
                       |
  posisi start --------+--------> +X (depan robot)

  yaw positif = putar kiri / CCW
```

Semua jarak menggunakan meter dan sudut `yaw_deg` menggunakan derajat.

### Mode local

Waypoint langsung relatif terhadap posisi start robot:

```yaml
coordinate_mode: local
start_pose: {x: 0.0, y: 0.0, yaw_deg: 0.0}

waypoints:
  A: {x: 0.50, y: 0.00}
```

### Mode map

Waypoint dan pose start mengikuti koordinat peta juri:

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

Robot harus ditempatkan tepat pada `start_pose` dan menghadap sesuai `yaw_deg`
sebelum start. Navigator mengubah koordinat peta menjadi koordinat odometri
lokal setelah `/wheel_odometry/reset`.

## 5. Mengatur speed waypoint

Edit bagian `motion` pada `waypoints.yaml`:

```yaml
motion:
  linear_speed: 0.30
  minimum_linear_speed: 0.10
  angular_speed: 0.90
  heading_kp: 1.20
  distance_kp: 0.50
  max_heading_correction: 0.30
  position_tolerance: 0.12
  turn_tolerance_deg: 8.0
  drive_heading_limit_deg: 25.0
  turn_timeout: 7.0
```

- `linear_speed`: batas kecepatan maju dalam m/s.
- `minimum_linear_speed`: kecepatan minimum mendekati titik.
- `angular_speed`: kecepatan putar di tempat dalam rad/s.
- `position_tolerance`: radius waypoint dianggap tercapai.
- `turn_tolerance_deg`: toleransi heading sebelum mulai maju.

PID dan batas duty tidak diatur di file waypoint. Pengaturan tersebut berada
di `config/drive_controller.yaml` agar keyboard, waypoint, dan Nav2 memakai
controller motor yang sama.

## 6. Menjalankan — urutan terminal lengkap

Sebelum mulai, tutup semua keyboard, waypoint, drive controller, dan program
tes motor lama. Untuk pengujian pertama, gunakan penyangga yang kuat atau area
lantai kosong.

### Terminal 1 — VMX, Titan, encoder, dan IMU

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

Masukkan password `sudo` ketika diminta dan biarkan Terminal 1 aktif.

### Terminal 2 — zero IMU dan inisialisasi encoder

```bash
source /opt/ros/humble/setup.bash
source /home/vmx/studica_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

Letakkan robot pada posisi start dan arahkan lurus sesuai `yaw_deg`, lalu:

```bash
ros2 service call /imu/get_imu_data \
  studica_control/srv/SetData \
  "{params: 'zero_yaw'}"

cd /home/vmx/studica_ws
bash scripts/init_titan_encoders.sh

ros2 service call /titan0/titan_cmd \
  studica_control/srv/SetData \
  "{params: 'enable'}"
```

### Terminal 3 — LiDAR, hanya jika obstacle avoidance aktif

Langkah ini wajib untuk `waypoints.yaml` dan tidak diperlukan untuk
`waypoints_test.yaml` selama `obstacle_avoidance.enabled: false`.

```bash
source /opt/ros/humble/setup.bash
source /home/vmx/ydlidar_ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 launch ydlidar_ros2_driver ydlidar_launch.py \
  params_file:=/home/vmx/ydlidar_ros2_ws/src/ydlidar_ros2_driver/params/Tmini.yaml
```

Periksa dari terminal lain:

```bash
ros2 topic hz /scan
```

T-mini Plus sebelumnya menghasilkan sekitar 10 Hz.

### Terminal 4 — mode waypoint

Untuk pengujian awal 0,50 m tanpa obstacle avoidance:

```bash
cd /home/vmx/studica_ws
bash scripts/start_waypoint_mode.sh \
  /home/vmx/studica_ws/src/studica_control/config/waypoints_test.yaml
```

Untuk titik A/B/C/D dengan obstacle avoidance:

```bash
cd /home/vmx/studica_ws
bash scripts/start_waypoint_mode.sh \
  /home/vmx/studica_ws/src/studica_control/config/waypoints.yaml
```

Skrip ini melakukan:

1. memeriksa `/imu` dan encoder M0-M3;
2. memeriksa `/scan` jika obstacle avoidance aktif;
3. menjalankan `wheel_odometry.py`;
4. mereset `/odom` menjadi `(0,0,0)`;
5. menjalankan `drive_controller.py`;
6. menjalankan `waypoint_navigator.py`;
7. menunggu perintah start dengan motor tetap STOP.

Tunggu sampai muncul:

```text
Mode waypoint siap, motor masih STOP.
```

Biarkan Terminal 4 aktif.

### Terminal 5 — start, monitoring, dan stop

```bash
source /opt/ros/humble/setup.bash
source /home/vmx/studica_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

Mulai seluruh urutan waypoint:

```bash
ros2 service call /waypoint_navigator/start \
  std_srvs/srv/Trigger "{}"
```

Stop navigator:

```bash
ros2 service call /waypoint_navigator/stop \
  std_srvs/srv/Trigger "{}"
```

Hard-disable Titan:

```bash
ros2 service call /titan0/titan_cmd \
  studica_control/srv/SetData \
  "{params: 'disable'}"
```

## 7. Alur gerakan waypoint

Setelah start diterima:

```text
reset odom
   -> pilih waypoint pertama
   -> TURN_TO_GOAL
   -> DRIVE_TO_GOAL
   -> waypoint tercapai
   -> pilih waypoint berikutnya
   -> semua waypoint selesai
   -> /cmd_vel nol
```

Saat heading error lebih dari `drive_heading_limit_deg`, robot berputar di
tempat. Setelah heading masuk toleransi, robot maju sambil melakukan koreksi
heading. Speed berkurang saat mendekati waypoint.

## 8. Alur obstacle avoidance

Obstacle avoidance hanya aktif jika:

```yaml
obstacle_avoidance:
  enabled: true
```

dan `/scan` tersedia. Alurnya:

```text
obstacle depan < stop_distance
   -> STOP
   -> bandingkan ruang kiri dan kanan
   -> putar ke sisi lebih lapang
   -> maju avoid_step_distance
   -> hitung ulang arah waypoint
```

Contoh parameter:

```yaml
obstacle_avoidance:
  enabled: true
  stop_distance: 0.55
  clear_distance: 0.75
  avoid_angle_deg: 55.0
  avoid_step_distance: 0.60
  timeout: 12.0
```

Ini adalah avoidance reaktif, bukan global planner. Untuk dinding tetap atau
lorong berbelok, tambahkan waypoint perantara pada jalur yang aman.

## 9. Monitoring

Jalankan satu per satu dari terminal monitoring:

```bash
ros2 topic echo /cmd_vel
ros2 topic echo /odom
ros2 topic hz /scan
ros2 topic hz /titan0/m_0/encoder
ros2 topic echo /titan0/m_0/cmd
```

Lihat node aktif:

```bash
ros2 node list
```

Node penting:

```text
/control_server
/wheel_odometry
/drive_controller
/waypoint_navigator
```

## 10. Mengubah urutan tanpa mengedit YAML

Jika menjalankan navigator secara manual, urutan dapat dioverride:

```bash
python3 \
  /home/vmx/studica_ws/src/studica_control/src/components/examples/python/waypoint_navigator.py \
  --config /home/vmx/studica_ws/src/studica_control/config/waypoints.yaml \
  --sequence A,B,D,C
```

Untuk penggunaan normal melalui skrip, lebih sederhana mengubah:

```yaml
sequence: [A, B, D, C]
```

pada `waypoints.yaml`.

## 11. Troubleshooting

### `/scan belum tersedia`

- Jika memakai `waypoints.yaml`, jalankan LiDAR karena avoidance aktif.
- Jika hanya menguji tanpa LiDAR, gunakan `waypoints_test.yaml`.
- Jangan menonaktifkan LiDAR pada mode obstacle avoidance.

### `feedback encoder stale/belum tersedia`

```bash
cd /home/vmx/studica_ws
bash scripts/init_titan_encoders.sh
ros2 topic hz /titan0/m_0/encoder
```

Periksa M0-M3 dan kabel encoder jika salah satu tidak mengirim.

### `/wheel_odometry/reset belum tersedia`

Pastikan Terminal 4 masih aktif dan `wheel_odometry.py` tidak berhenti karena
topic encoder atau IMU hilang.

### Robot tidak bergerak

```bash
ros2 service call /titan0/titan_cmd \
  studica_control/srv/SetData \
  "{params: 'enable'}"
ros2 topic echo /cmd_vel
```

Jika `/cmd_vel` nonzero tetapi duty nol, lihat error PID/encoder pada Terminal
4. Jika `/cmd_vel` selalu nol, navigator belum di-start atau sudah STOP.

### Robot tidak berhenti

```bash
ros2 service call /titan0/titan_cmd \
  studica_control/srv/SetData \
  "{params: 'disable'}"
```

Jika masih bergerak, gunakan emergency stop fisik untuk memutus daya 12 V
Titan. Jangan menyentuh roda yang sedang bergerak.

## 12. Menghentikan seluruh sistem

1. Panggil service `/waypoint_navigator/stop` dari Terminal 5.
2. Tekan `Ctrl+C` satu kali pada Terminal 4.
3. Tekan `Ctrl+C` pada Terminal 3 jika LiDAR dijalankan.
4. Tekan `Ctrl+C` pada Terminal 1.

Cleanup Terminal 4 mengirim `/cmd_vel` nol, menghentikan drive controller dan
odometry, serta menulis duty nol ke M0-M3 melalui service Titan.

## 13. Tombol start fisik (opsional)

Dalam mode lomba, semua node harus sudah aktif dan motor berada pada kondisi
STOP. Tombol memicu:

```text
tekan START -> debounce -> validasi odom/LiDAR -> reset odom -> jalankan urutan
```

Konfigurasi robot saat ini menggunakan DIO 10 dan tombol active-low:

```yaml
# config/titan_m1_test.yaml
dio:
  enabled: true
  sensors: ["start_button"]
  start_button:
    pin: 10
    type: "input"
    interrupt_edge: "falling"
    debounce_ms: 250
```

```yaml
# config/waypoints.yaml
start_button:
  enabled: true
  topic: "/start_button/state"
  active_high: false
  debounce_ms: 250
```

Active-low berarti kondisi normal menghasilkan `true/HIGH`, tombol ditekan
menghasilkan `false/LOW`, dan interrupt terjadi pada falling edge. Uji tombol
tanpa gerakan terlebih dahulu:

```bash
ros2 service call /titan0/titan_cmd studica_control/srv/SetData \
  "{params: 'disable'}"
ros2 topic echo /start_button/state
```

Output harus berubah dari `data: true` menjadi `data: false` ketika ditekan,
lalu kembali `true` ketika dilepas. Jangan menjalankan mode waypoint sebelum
hasil ini benar.

Jangan pernah menghubungkan 12 V ke input DIO VMX2. Tombol start bukan
pengganti emergency stop fisik.
