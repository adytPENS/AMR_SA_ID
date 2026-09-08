# GUI monitoring dan kontrol robot

Dashboard desktop Tkinter terhubung langsung ke ROS 2. Tidak menjalankan hardware atau mengirim gerakan otomatis saat dibuka.

## Menjalankan

Jalankan dari desktop robot / remote desktop atau SSH dengan X forwarding. Dependensi GUI: `sudo apt install python3-tk`.

Terminal hardware (hanya satu proses yang mengakses VMX):

```bash
cd /home/vmx/studica_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch studica_control studica_launch.py params_file:="$PWD/src/studica_control/config/titan_m1_test.yaml"
```

Terminal GUI:

```bash
cd /home/vmx/studica_ws
./scripts/start_robot_dashboard.sh
```

Jika hardware sudah aktif, cukup jalankan GUI. Restart hardware dengan konfigurasi di atas untuk memuat tambahan buzzer pin 9. Jangan jalankan launcher keyboard/navigasi bersamaan dengan kontrol manual motor.

Setelah `colcon build --packages-select studica_control` dan source ulang install, GUI juga tersedia sebagai `ros2 run studica_control robot_dashboard.py`. Launcher script bisa dipakai tanpa build ulang jika interface ROS proyek sudah terpasang.

## Fitur

- **Motor & Encoder:** M0 kanan depan, M1 kanan belakang, M2 kiri depan, M3 kiri belakang, sesuai `drive_controller.yaml`. Duty 0–100%, pembalik arah, tekan-tahan CW/CCW, reset encoder, encoder/RPM/angle dan limit switch jika diterbitkan hardware. Encoder topic berisi jarak sesuai `dist_per_tick`, bukan raw count. Raw count tersedia lewat layanan Titan `get_encoder_count`.
- **Semua Sensor:** penemuan topik otomatis, nilai IMU, DIO, encoder, tegangan, range, odometri, deteksi, dan sensor lainnya yang aktif. Data lebih dari 2 detik ditandai STALE. Gambar/point cloud berupa metadata, LiDAR berupa ringkasan jarak; belum ada video atau peta visual.
- **Servo / DIO / Lampu:** tombol HIGH/LOW untuk kontrol lampu pin 12, merah 13, hijau 14, kuning 15, serta buzzer pin 9. Feedback adalah keadaan pin, bukan verifikasi lampu menyala/buzzer berbunyi. Untuk lampu steady, aktifkan pin kontrol sesuai wiring tower. Buzzer diasumsikan aktif-HIGH. Pin 9 hanya dialokasikan pada DIO `buzzer`, jangan sekaligus dipakai komponen lain.
- Servo: pilih layanan perangkat dan masukkan sudut untuk standard atau speed -100 sampai 100 untuk continuous; gunakan 0 untuk menghentikan continuous. Tombol servo mengirim sekali dan nilainya bertahan sampai diperintah lagi.
- Pilihan light tower lewat layanan `/set` tersedia bila komponen light tower diaktifkan. Pada konfigurasi robot saat ini lampu menggunakan DIO, sehingga gunakan tombol langsung. Layanan light tower mengganti warna aktif, bukan mengaktifkan setiap warna secara independen.
- **Layanan Lanjutan:** memilih layanan ROS, mengisi request JSON dari schema, dan membaca balasan. Mendukung layanan yang tipe interfacenya terpasang, termasuk Titan kedua (`/titan1/titan_cmd`), IMU, dan kamera. Isi `params` untuk SetData sesuai perintah komponen, lalu field `initparams` yang diperlukan. Panel ini dapat menjalankan perintah yang menetap; batas tekan-tahan hanya berlaku pada tombol CW/CCW.

`./scripts/start_robot_dashboard.sh --titan titan1` memilih kanal Titan kedua. Label posisi roda tetap mengacu pada drivetrain; untuk Titan OMS gunakan nomor kanal M0–M3 sebagai identitas, bukan label roda.

## Perilaku stop

Kontrol motor awalnya nonaktif. CW berarti duty positif sebelum pembalik arah; arah fisik bergantung wiring, inversi driver dan sisi pandang poros. Uji dengan duty rendah. Tombol motor berhenti saat dilepas, GUI kehilangan fokus, ditutup, atau setelah 3 detik. GUI menolak gerak jika ada publisher lain pada kanal yang sama ketika tombol ditekan.

STOP SEMUA / Escape mengirim zero ke empat kanal controller terpilih dan memanggil `disable`. Periksa balasannya di log. Untuk mengaktifkan controller kembali, pilih `/titan0/titan_cmd` (atau controller terpilih), set `params` menjadi `enable`, jalankan, lalu aktifkan kontrol manual. STOP ini tidak menghentikan Titan lain atau servo.

Stop GUI adalah perintah perangkat lunak. Driver Titan saat ini terus mengirim duty terakhir dan tidak memiliki timeout penerimaan topik; GUI crash atau koneksi terputus dapat membuat motor terus bergerak. Batas 3 detik bergantung event loop GUI tetap berjalan. Gunakan stop hardware untuk pengujian fisik. Menutup GUI tidak mematikan lampu/buzzer atau servo yang sudah diperintah.

## Pengujian tanpa hardware

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
/usr/bin/python3 -m unittest discover -s src/studica_control/test -p test_robot_dashboard.py -v
```
