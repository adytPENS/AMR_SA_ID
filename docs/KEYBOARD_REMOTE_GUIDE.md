# Panduan Remote Keyboard AMR

Panduan ini menjalankan remote keyboard menggunakan arsitektur terbaru:

```text
Keyboard -> /cmd_vel -> inverse kinematics -> PID M0-M3 -> Titan
```

Konfigurasi saat ini juga memuat Titan OMS kedua pada CAN ID 10:

```text
Keyboard I/K -> /titan1/m_2/cmd -> naik/turun gripper
Keyboard J/L -> /titan1/m_3/cmd -> rotasi gripper CCW/CW
```

Cara termudah adalah launcher satu terminal:

```bash
cd /home/vmx/studica_ws
./scripts/start_full_keyboard.sh
```

Launcher menyalakan kedua Titan, encoder base, drive controller, dan keyboard;
ketika Q/Ctrl+C digunakan, kedua Titan dinonaktifkan. Bagian tiga terminal di
bawah digunakan untuk diagnosis jika launcher satu terminal gagal.

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

Aktifkan Titan OMS CAN ID 10:

```bash
ros2 service call /titan1/titan_cmd \
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
G = servo pin 18 mendorong gripper ke depan
H = servo pin 18 menarik gripper ke belakang
R = servo pin 19 menaikkan ujung gripper
T = servo pin 19 menurunkan ujung gripper
Y = servo continuous pin 20 membuka end effector
U = servo continuous pin 20 menutup end effector
I = gripper naik (Titan CAN 10 M2)
K = gripper turun (Titan CAN 10 M2)
J = gripper berputar CCW (Titan CAN 10 M3)
L = gripper berputar CW (Titan CAN 10 M3)
E = stop
Q = stop dan keluar
```

Tombol harus ditahan. Jika keyboard tidak menerima pengulangan tombol selama
`release-timeout` (default 0,65 detik), program mengirim nol ke `/cmd_vel`, M2,
M3, serta servo continuous pin 18/19/20. Tombol E/Q, Ctrl+C, serta keluarnya
program juga mengirim nol lima kali ke seluruh aktuator yang harus berhenti.
Watchdog drive controller menghentikan motor base jika `/cmd_vel` terputus.

Uji OMS pertama dengan robot tidak membawa objek:

```bash
python3 \
  /home/vmx/studica_ws/src/studica_control/src/components/examples/python/titan_keyboard_teleop.py \
  --linear-speed 0.15 --angular-speed 0.8 --oms-speed 0.10
```

Jika arah M2 terbalik, tambahkan `--lift-polarity -1`. Jika arah CCW/CW M3
terbalik, tambahkan `--rotate-polarity -1`. Jangan melebihi batas mekanis.
M2 lift harus dilengkapi limit switch sebelum pengujian kecepatan/beban tinggi.

Kecepatan M2 dan M3 dapat diatur terpisah. Contoh lift duty 0,20 dan rotasi
duty 0,35:

```bash
./scripts/start_full_keyboard.sh --lift-speed 0.20 --rotate-speed 0.35
```

`--oms-speed` tetap tersedia sebagai nilai bersama jika opsi terpisah tidak
diberikan.

Secara default OMS memakai PID software berdasarkan `/titan1/m_2/rpm` dan
`/titan1/m_3/rpm`. Target lift dibedakan untuk melawan gravitasi: naik 40 RPM,
turun 25 RPM (Maverick 61:1, maksimum 100 RPM). Rotasi memakai target 35 RPM
(Maverick 26.9:1, maksimum 227 RPM):

```bash
./scripts/start_full_keyboard.sh \
  --lift-up-rpm 40 --lift-down-rpm 25 \
  --rotate-rpm 35 --rotate-polarity -1
```

Saat mulai naik, program memberi boost duty 0,65 selama 0,20 detik. Saat mulai
berotasi, program memberi boost duty 0,60 selama 0,25 detik untuk melepaskan
gesekan awal gear 3D-print. Setelah itu PID kembali menjaga target RPM rendah.

Sesudah uji tanpa beban berhasil, target dapat dinaikkan bertahap. Untuk
diagnosis encoder, PID dapat dimatikan sementara dan kembali memakai duty:

```bash
./scripts/start_full_keyboard.sh --no-oms-pid \
  --lift-speed 0.20 --rotate-speed 0.20 --rotate-polarity -1
```

Jika feedback RPM hilang lebih dari 0,4 detik, kontrol OMS otomatis menjadi
nol. PID hanya mengatur kecepatan selama tombol ditekan; ini bukan position
hold dan belum menggantikan limit switch mekanis lift.

Kecepatan awal pin 18 adalah 40 agar slide mempunyai tenaga lebih besar. Pin 19
adalah 25 dan pin 20 adalah 15 dari rentang 1..100. Kalibrasi dan pastikan
linkage tidak menyentuh mechanical stop:

```bash
./scripts/start_full_keyboard.sh \
  --slide-speed 15 --wrist-speed 15 --gripper-speed 10 \
  --rotate-polarity -1
```

Jika arah salah gunakan `--slide-polarity -1`, `--wrist-polarity -1`, atau
`--gripper-polarity -1`. Ketiga servo hanya bergerak selama tombolnya ditahan.

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

ros2 service call /titan1/titan_cmd \
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
