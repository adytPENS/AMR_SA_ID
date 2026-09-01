# Drive Controller AMR — Bahasa Indonesia

Panduan ini menjelaskan arsitektur kontrol terbaru untuk VMX2 Pi, Titan CAN
ID 42, empat motor Maverick 50.9:1, dan encoder pada setiap roda.

## Arsitektur

```text
Keyboard / Waypoint / Nav2
          |
          v
       /cmd_vel                  geometry_msgs/Twist
          |
          v
  Inverse Kinematics            pilih model base dari YAML
          |
          v
 Target speed M0-M3             meter/detik
          |
          v
 PID speed per motor            feedback encoder
          |
          v
 Duty Titan M0-M3
```

Keyboard tidak mengatur duty, nomor motor, PID, atau kinematika. Semua sumber
gerak menggunakan `/cmd_vel`, sedangkan `drive_controller.py` menjadi satu-
satunya node yang menerbitkan duty motor.

## Konfigurasi robot

Edit:

```text
/home/vmx/studica_ws/src/studica_control/config/drive_controller.yaml
```

Model yang tersedia:

```yaml
model: differential
model: differential_all_terrain
model: mecanum
model: x_drive
```

Robot saat ini memakai:

```yaml
drive:
  model: differential_all_terrain
  wheel_diameter: 0.12
  track_width: 0.35
  wheelbase: 0.29
  max_wheel_speed: 0.75

motors:
  order: [front_right, rear_right, front_left, rear_left]
  titan_channels: [0, 1, 2, 3]
  electrical_polarity: [-1.0, -1.0, 1.0, 1.0]
```

`track_width` adalah jarak titik tengah roda kiri–kanan. `wheelbase` adalah
jarak titik tengah roda depan–belakang. Semua ukuran menggunakan meter.

Differential dan differential all-terrain tidak dapat bergerak lateral.
Mecanum dan X-drive menerima `linear.y`.

## PID speed

Setiap roda memiliki controller sendiri:

```yaml
speed_pid:
  enabled: true
  kp: 0.25
  ki: 0.10
  kd: 0.0
  integral_limit: 0.50
  feedback_timeout: 0.30
  duty_limit: 0.70
```

PID memakai perubahan jarak encoder untuk menghitung speed dalam m/s. PID
memiliki anti-windup, tidak boleh membalik arah perintah, dan menghentikan
robot jika feedback encoder stale.

## Menjalankan keyboard dengan PID

### Terminal 1 — hardware

```bash
source /opt/ros/humble/setup.bash
source /home/vmx/studica_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

sudo -E env RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  /home/vmx/studica_ws/install/studica_control/lib/studica_control/manual_composition \
  --ros-args -r __node:=control_server \
  --params-file /home/vmx/studica_ws/src/studica_control/config/titan_m1_test.yaml
```

### Terminal 2 — drive controller

```bash
source /opt/ros/humble/setup.bash
source /home/vmx/studica_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

python3 /home/vmx/studica_ws/src/studica_control/src/components/examples/python/drive_controller.py \
  --config /home/vmx/studica_ws/src/studica_control/config/drive_controller.yaml
```

### Terminal 3 — enable dan keyboard

```bash
source /opt/ros/humble/setup.bash
source /home/vmx/studica_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 service call /titan0/titan_cmd studica_control/srv/SetData \
  "{params: 'enable'}"

python3 /home/vmx/studica_ws/src/studica_control/src/components/examples/python/titan_keyboard_teleop.py \
  --linear-speed 0.15 --angular-speed 0.8
```

Kontrol: `W` maju, `S` mundur, `A` putar kiri, `D` putar kanan, `G` target
jarak, `E` stop, dan `Q` keluar. Saat tombol dilepas, keyboard menerbitkan
`/cmd_vel` nol.

Setelah uji pelan berhasil, contoh kecepatan lebih tinggi:

```bash
python3 /home/vmx/studica_ws/src/studica_control/src/components/examples/python/titan_keyboard_teleop.py \
  --linear-speed 0.40 --angular-speed 1.5
```

## Monitoring

```bash
ros2 topic echo /cmd_vel
ros2 topic hz /titan0/m_0/encoder
ros2 topic echo /titan0/m_0/cmd
ros2 topic echo /imu
ros2 topic hz /scan
```

## Emergency stop

Drive controller menghentikan motor bila `/cmd_vel` tidak diterima selama
0.35 detik. Hard stop Titan:

```bash
ros2 service call /titan0/titan_cmd studica_control/srv/SetData \
  "{params: 'disable'}"
```

Emergency stop fisik yang memutus daya aktuator tetap diperlukan.

## Waypoint

`waypoint_navigator.py` sudah dimigrasikan untuk menerbitkan `/cmd_vel` saja.
Skrip berikut menjalankan wheel odometry, drive controller, dan navigator dalam
satu mode yang benar:

```bash
cd /home/vmx/studica_ws
bash scripts/start_waypoint_mode.sh \
  /home/vmx/studica_ws/src/studica_control/config/waypoints_test.yaml
```

Motor tetap STOP sampai service start dipanggil dari terminal lain:

```bash
ros2 service call /waypoint_navigator/start std_srvs/srv/Trigger "{}"
```

Jangan jalankan keyboard saat navigator aktif karena keduanya merupakan sumber
`/cmd_vel`. Keduanya aman memakai drive controller yang sama, tetapi hanya satu
sumber gerak boleh aktif pada satu waktu.
