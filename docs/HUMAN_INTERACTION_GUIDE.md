# Human interaction: misi YAML

Jalankan dari root workspace:

```bash
bash scripts/start_full_keyboard.sh
```

Program mulai dalam mode manual. Atur OMS memakai I/K, J/L, G/H,
R/T dan Y/U. Tekan M: motor berhenti, feedback baru lift/putar ditunggu,
kemudian default disimpan ke `config/oms_default.json`. Wrist/EoF menyimpan
target sudut, bukan feedback posisi fisik. Slide adalah acuan manual dengan
kontrol durasi. Encoder default hanya berlaku pada sesi hardware yang sama.

State: MANUAL → CAPTURE_DEFAULT → READY → (START fisik) RUNNING → DONE.
Data hilang, timeout atau kegagalan PID masuk ERROR. E/X atau STOP fisik
membatalkan misi dan menghentikan bodi, lift, putar dan slide. Servo standard
mempertahankan target terakhir. Q keluar. Untuk mengulang, E lalu M dan START.
START harus pernah dilepas sebelum tekan baru dikenali; tidak auto-start
ketika program menerima kondisi tombol yang sudah ditekan.

## File yang diedit

- `config/human_interaction.yaml`: misi default. Contoh aktif awal hanya lampu
  kuning 2 detik, padam, selesai; tidak menggerakkan robot.
- `config/human_interaction_examples.yaml`: semua instruksi aktif/uncommented,
  dengan komentar penjelasan. Ini katalog demonstrasi, bukan rute yang telah
  diuji. Salin langkah yang diperlukan ke `mission` pada file default.
- `src/studica_control/config/keyboard_servos.yaml`: sudut wrist/EoF hasil kalibrasi.
- `src/studica_control/src/components/examples/python/mission_runner.py`:
  fungsi `do_*` dan state machine tanpa ROS.
- `src/studica_control/src/components/examples/python/mission_ros.py`:
  sensor ROS, output motor, PID lift dan lampu.

Restart keyboard setelah mengubah YAML. Memilih file lain:

```bash
bash scripts/start_full_keyboard.sh --human-config config/human_interaction_examples.yaml
```

File contoh mengandung gerakan OMS yang perlu kalibrasi; START akan menolak
jika `lift_cm_per_rev` / `rotate_deg_per_rev` masih null. Tidak ada gerakan misi
sebelum kedua parameter yang diperlukan valid.

## Instruksi

| action | Parameter utama | Selesai saat |
|---|---|---|
| `goto` | `x`, `y`, `speed_mps` | dekat koordinat tujuan |
| `forward` | `until: {travel_cm: 100}` | panjang lintasan odom mencapai jarak |
| `forward` | `until: {lidar_front_cm: 10}` | LiDAR depan <= jarak target |
| `trace` | `side`, `wall_distance_cm`, `until` | jarak tempuh/front threshold tercapai |
| `heading` / `robot_movetheta` | `angle_deg` | arah absolut tercapai |
| `turn` | `direction: left/right`, `angle_deg` | sudut relatif tercapai |
| `robot_move` | `vx_mps`, `duration_s` | durasi habis (negatif = mundur) |
| `oms_rotate` | `angle_deg` atau `target: default` | encoder mencapai/melewati target |
| `lift` | `delta_cm` atau `target: default` | encoder mencapai/melewati target |
| `slide` | `direction: forward/backward`, `duration_s`, `duty_percent` | durasi habis |
| `wrist` / `gripper` | `position: "on"/"off"/default`, `settle_s` | service berhasil + settle selesai |
| `detect_camera` | `colour`, opsional `shape`, `save_as`, `on_not_found` | ada objek cocok atau timeout |
| `light` | `color: red/green/yellow/"off"`, `mode: steady/blink` | langsung lanjut; lampu tetap aktif |
| `wait` | `seconds` | waktu habis |
| `stop` | — | misi berakhir; langkah berikut diabaikan |

Setiap langkah mempunyai `timeout_s` (default 30 detik). Untuk langkah durasi,
timeout harus lebih besar dari durasi. Tidak ada `eval` atau eksekusi Python
bebas dari YAML. Parameter/action tidak dikenal ditolak agar typo terlihat.
Nilai `"on"` / `"off"` harus dikutip: YAML tanpa kutip dapat mengubahnya menjadi
boolean. `on/off` memilih left_value/right_value hasil kalibrasi; bukan daya.

`start_pose` adalah koordinat robot saat START ditekan. Runner mengubah odom
menjadi frame tersebut tanpa me-reset publisher odometry. `goto` menggunakan
frame ini; kembali home bisa memakai koordinat `start_pose.x/y`. Heading positif
berlawanan jarum jam. `turn right 270` benar-benar berputar relatif ke kanan,
bukan mencari arah akhir lewat putaran kiri 90 derajat.

Trace mengikuti dinding berdasarkan jarak sektor LiDAR samping ±15°, dengan
koreksi proporsional. Ini kontrol reaktif sederhana, belum path planner atau
penghindaran rintangan. Jarak tempuh dihitung sebagai panjang lintasan odom.
LiDAR depan/belakang di bawah `front_stop_cm` menghentikan gerakan dengan ERROR;
target `lidar_front_cm` harus lebih besar dari batas tersebut.

## Kalibrasi OMS dan slide

Ukur perubahan encoder saat lift berpindah sejumlah cm dan OMS berputar sejumlah
derajat. Rumus: `lift_cm_per_rev = perubahan_cm / perubahan_encoder` dan
`rotate_deg_per_rev = perubahan_derajat / perubahan_encoder`. Satuan encoder
Titan OMS di konfigurasi sekarang adalah putaran poros output motor. Sertakan
tanda negatif jika encoder turun saat gerakan positif (lift naik / putar kiri).
Jangan menganggap poros motor dan gripper selalu 1:1.

`lift_positive_duty_sign` / `rotate_positive_duty_sign` menentukan tanda duty
untuk gerakan naik/kiri. Default -1 mengikuti tombol I/J pada keyboard.
Lift memakai PID RPM yang sama dengan keyboard, tanpa boost. Putar OMS memakai
duty langsung (default 33,3%), tanpa PID atau boost. Nilai target yang terlewati
mengakhiri langkah; belum ada kompensasi inersia atau jaminan presisi fisik.

Slide memakai duty persen dan delay detik, tanpa feedback posisi. Keluar dan
kembali harus disetel terpisah. `slide` level atas tetap disalin ke snapshot
untuk acuan; langkah mission memakai `duration_s`/`duty_percent` masing-masing
atau `settings.slide_duty_percent`. Wrist/gripper tidak memakai pembacaan posisi
aktual; `settle_s` hanya waktu tunggu setelah perintah diterima. Jangan menjalankan
`gripper position: default` ketika ingin tetap menggenggam objek.

## Sensor dan kamera

Launcher keyboard sekarang menjalankan wheel_odometry bersama drive dan hardware.
Semua aksi bodi memerlukan `/odom`, `/imu`, encoder keempat roda dan `/scan` yang
segar. Driver LiDAR dan kamera perlu dijalankan terpisah; jangan menjalankan
launcher waypoint penuh karena akan membuka server hardware/kontrol motor kedua.
Contoh untuk instalasi YDLIDAR yang sudah dipakai di workspace ini:

```bash
source /opt/ros/humble/setup.bash
source /home/vmx/ydlidar_ros2_ws/install/setup.bash
ros2 launch ydlidar_ros2_driver ydlidar_launch.py params_file:=/home/vmx/ydlidar_ros2_ws/src/ydlidar_ros2_driver/params/Tmini.yaml
```

Kamera: `bash scripts/start_color_roi_tracker.sh`; aktifkan profil HSV objek.
`colour` harus sama dengan nama profil (misalnya `kuning`, bukan otomatis
alias `yellow`). Hasil `/color_tracker/result` kini menyertakan bentuk kontur
2D `rectangle`, `triangle`, `circle` atau `unknown`. Warna dan bentuk harus
cocok pada objek yang sama; bounding box saja tidak dianggap rectangle.
Bentuk tergantung sudut pandang/segmentasi dan bukan pengenal objek 3D.

`detect_camera` membaca frame baru sejak langkah dimulai dan tidak memutar OMS.
Hasil masuk `/human_interaction/results` sebagai JSON; posisi gambar/depth ikut
tersimpan jika tersedia dari kamera. Belum ada variable substitution/percabangan
berdasarkan hasil pada YAML. `on_not_found: continue` hanya melanjutkan bila
kamera masih mengirim frame tetapi tidak ada target sampai timeout; data kamera
hilang/kedaluwarsa tetap ERROR. Status langkah ada di `/human_interaction/status`.

Urutan scan sambil berputar, penyelarasan gripper dan pengambilan otomatis adaptif
belum termasuk aksi ini. Deteksi dan putar adalah langkah terpisah/sekuensial;
contoh bukan implementasi pencarian terus-menerus sambil OMS bergerak.

## Validasi

Tes simulasi tanpa motor: `python3 -m pytest src/studica_control/test scripts/tests -q`.
Tes mencakup transformasi koordinat, arah relatif >180°, stale sensor, STOP,
kalibrasi encoder bertanda, target servo, pencocokan warna+bentuk dan timeout.
Pengujian ini tidak menggantikan uji posisi dan dinamika mekanik pada robot.
