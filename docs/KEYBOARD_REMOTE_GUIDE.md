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

sudo -n -E \
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
R = wrist pin 19 ke left_value YAML
T = wrist pin 19 ke right_value YAML
Y = EoF pin 20 ke left_value YAML
U = EoF pin 20 ke right_value YAML
P = inisialisasi servo standard yang belum mempunyai target posisi
I = gripper naik (Titan CAN 10 M2)
K = gripper turun (Titan CAN 10 M2)
J = gripper berputar CCW (Titan CAN 10 M3)
L = gripper berputar CW (Titan CAN 10 M3)
E = stop
Q = stop dan keluar
```

Tombol harus ditahan. Jika keyboard tidak menerima pengulangan tombol selama
`release-timeout` (default 0,65 detik), program mengirim nol ke `/cmd_vel`, M2,
M3, serta servo continuous pin 18. Wrist 19 dan gripper 20 adalah servo
standard: saat tombol dilepas/E, target sudut terakhir dipertahankan, tidak
dikirim nol (nol berarti posisi tengah). Tombol E/Q, Ctrl+C, serta keluarnya
program mengirim nol lima kali ke base, motor OMS, dan slide continuous.
Setelah Q/Ctrl+C launcher menutup hardware server; hold servo tidak dijamin
setelah PWM dimatikan. E bukan pemutusan daya servo.
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
./scripts/start_full_keyboard.sh --no-oms-pid --lift-speed 0.20 --rotate-speed 0.35
```

`--oms-speed` tetap tersedia sebagai nilai duty bersama jika opsi terpisah tidak
diberikan. Opsi duty tersebut berlaku saat PID dimatikan; saat PID aktif,
gunakan target RPM berikut.

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

Untuk memperlambat **hanya rotasi OMS J/L**, tanpa mengubah kecepatan base:

```bash
./scripts/start_full_keyboard.sh --rotate-rpm 10 --rotate-boost-time 0
```

Ini menurunkan target dari 35 ke 10 RPM dan mematikan boost awal. Menurunkan
RPM saja tidak mengurangi boost. Jika tetap terlalu cepat, minimum duty PID
default 0,18 bisa membatasi kecepatan rendah; perlu uji tanpa beban sebelum
menurunkannya lewat `--rotate-minimum-duty`. `--angular-speed` hanya untuk
belok base A/D, bukan OMS. Nilai RPM adalah target motor, bukan RPM ujung
gripper setelah transmisi gear.

Sesudah uji tanpa beban berhasil, target dapat dinaikkan bertahap. Untuk
diagnosis encoder, PID dapat dimatikan sementara dan kembali memakai duty:

```bash
./scripts/start_full_keyboard.sh --no-oms-pid \
  --lift-speed 0.20 --rotate-speed 0.20 --rotate-polarity -1
```

Jika feedback RPM hilang lebih dari 0,4 detik, kontrol OMS otomatis menjadi
nol. PID hanya mengatur kecepatan selama tombol ditekan; ini bukan position
hold dan belum menggantikan limit switch mekanis lift.

Slide pin 18 continuous memakai `--slide-speed` default 40 (skala 1..100).
Wrist pin 19 dan EoF/gripper pin 20 memakai target sudut tetap dari
`src/studica_control/config/keyboard_servos.yaml`:

```yaml
wrist:
  left_value: 30.0   # R
  right_value: -30.0 # T
eof:
  left_value: 30.0   # Y
  right_value: -30.0 # U
```

Nilai dalam derajat, bukan duty; sesuaikan dengan mekanisme. Sekali tekan
langsung mengirim target. Menahan tombol tidak menambah sudut lagi.
R/T/Y/U tidak memerlukan P terlebih dahulu. P tetap tersedia untuk posisi
awal servo yang belum mempunyai target. Opsi `--wrist-speed`,
`--gripper-speed` dan polarity tidak digunakan oleh tombol target YAML.
Restart keyboard setelah mengedit YAML. Batas min/max sudut tetap diperiksa.

Keyboard mengirim sudut melalui `/oms_wrist/set_servo` dan
`/oms_gripper/set_servo`, sama seperti `servo_example.py`. Satu request
per servo diproses pada satu waktu; target terbaru dikirim sesudahnya.
Service belum tersedia atau perintah ditolak akan terlihat di terminal.

`/oms_wrist/state` dan `/oms_gripper/state` hanya perintah terakhir, **bukan
sensor posisi fisik**. Rentang driver -150..150 derajat belum tentu aman untuk
linkage yang terpasang. Kalibrasi batas mekanis menggunakan
`--wrist-min-angle`, `--wrist-max-angle`, `--gripper-min-angle`, dan
`--gripper-max-angle`. Posisi awal dapat diatur lewat `--wrist-start-angle`
dan `--gripper-start-angle` dan harus berada dalam batas tersebut. Jangan
menjalankan publisher servo lain bersamaan.

Contoh memperlambat perubahan target servo:

```bash
./scripts/start_full_keyboard.sh \
  --slide-speed 15 --wrist-speed 15 --gripper-speed 10 \
  --rotate-polarity -1
```

Jika arah salah gunakan `--slide-polarity -1`, `--wrist-polarity -1`, atau
`--gripper-polarity -1`. Setelah tombol dilepas (timeout default 0,65 detik),
slide berhenti dan target posisi kedua servo standard tidak berubah lagi.

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


## Follow warna kuning dengan Z

Terminal 1 (kamera dan tracker, jendela tampil pada monitor Raspberry Pi):

```bash
/home/vmx/studica_ws/scripts/start_color_roi_tracker.sh
```

Centang profil `yellow` atau `kuning`. Profil `yellow` sudah tersimpan di
`config/color_profiles.yaml`. Jika perlu kalibrasi ulang: R, pilih ROI kuning,
Enter, beri nama `yellow` atau `kuning`, Enter. Jika jarak `N/A`, robot hanya berputar kanan/kiri untuk memusatkan kuning.
Maju/mundur hanya aktif saat jarak dalam meter tersedia. Gunakan mode profil default; mode
`multiple_colors:=true` hanya mendeteksi merah, hijau, dan biru.

Terminal 2 (driver robot, drive controller, keyboard base/OMS/servo):

```bash
/home/vmx/studica_ws/scripts/start_yellow_follow_keyboard.sh
```

Skrip ini memakai `start_full_keyboard.sh`; jalankan satu keyboard saja agar
publisher `/cmd_vel` tidak saling menimpa. Fitur Z juga tersedia bila langsung
menjalankan `start_full_keyboard.sh`. Fokuskan terminal keyboard saat menekan tombol.

- Z: aktif/nonaktif follow kuning, menjaga jarak sekitar 35–40 cm.
- W/S/A/D: batalkan follow dan kendalikan base manual.
- E/X: batalkan follow dan stop; Q: keluar.
- Tombol OMS/servo membatalkan follow sebelum menjalankan kontrol manual.

Batas kecepatan mengikuti opsi keyboard (default skrip 0,15 m/s).
Objek hilang atau data tracker lebih lama dari 0,5 detik menghasilkan perintah stop.
Depth tidak valid menghasilkan gerak putar saja, tanpa maju/mundur. Mode follow tetap aktif dan melanjutkan saat
kuning dengan depth valid kembali terlihat; tekan E/X untuk membatalkannya.
Terminal menampilkan alasan berhenti ketika tracker/depth belum tersedia.


Untuk menjalankan keyboard melalui SSH dari komputer lain:

```bash
ssh -t vmx@192.168.50.39 /home/vmx/studica_ws/scripts/start_yellow_follow_keyboard.sh
```

Alamat IP dapat berubah. Opsi `-t` menyediakan terminal untuk pembacaan tombol.
Launcher memanggil `sudo -n -E manual_composition` secara langsung sesuai aturan
NOPASSWD host; jangan menyisipkan `env` setelah sudo. Pesan `KEYBOARD SIAP`
harus muncul sebelum tombol dapat mengendalikan robot.

## Arsitektur driver standar

Driver memakai `MultiThreadedExecutor` bawaan ROS dengan jumlah thread otomatis.
Pembacaan encoder, RPM, dan Cypher mengikuti implementasi standar driver.
Optimasi pembatasan dua worker dan pengabaian pembacaan Cypher telah dibatalkan.
Alur kontrol tetap keyboard → `/cmd_vel` → drive controller/PID → Titan.
Fitur Z untuk mengikuti kuning dan sampling depth 10 Hz tetap tersedia.

Jika versi optimasi sempat dibuild, build ulang source ini secara lokal untuk
mengembalikan binary driver standar. Tidak perlu build ulang hanya karena
perubahan dokumentasi. Panduan build hemat RAM ada di bawah.

## Build lokal Raspberry Pi dengan RAM terbatas

Jalankan build dari terminal pada monitor Raspberry Pi. SSH boleh diputus
setelah terminal lokal dibuka; jangan menjalankan perintah ini di sesi SSH
yang akan ditutup. Beri tahu pengguna dan tunggu kesiapan sebelum memulai
build; pengguna memilih menjalankan build sendiri secara lokal.

Sebelum build, keluar dari keyboard dengan Q dan hentikan tracker/kamera
serta launcher robot lainnya dengan Ctrl+C. Tutup aplikasi berat untuk
menyediakan RAM. Opsi berikut membatasi kompilasi ke satu proses; build
lebih lama tetapi penggunaan RAM lebih rendah daripada build paralel.

```bash
cd /home/vmx/studica_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS="-j1 -l1" \
colcon build --packages-select studica_control \
  --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Tunggu sampai muncul ringkasan `1 package finished` tanpa paket gagal.
Jika muncul `Failed`, `Killed`, atau error kehabisan memori, simpan pesan
error dan jangan lanjut menjalankan robot dengan hasil build tersebut.
Periksa memori jika diperlukan:

```bash
free -h
```

Setelah build berhasil, muat kembali workspace:

```bash
source /home/vmx/studica_ws/install/setup.bash
```

Build ini memasang perubahan driver untuk digunakan pada peluncuran berikutnya.
Keberhasilan kompilasi belum membuktikan penurunan CPU atau kestabilan kontrol;
uji kembali keyboard, kamera, dan waypoint setelah restart program.
