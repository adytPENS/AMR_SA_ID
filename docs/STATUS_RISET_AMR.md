# Status Riset AMR dan OMS

Dokumen ini mencatat kondisi implementasi terakhir agar pengujian berikutnya
tidak menganggap fitur eksperimental sebagai fitur tervalidasi.

## Sudah berhasil diuji pada robot

- Base empat motor memakai `differential_all_terrain`, PID kecepatan, encoder,
  IMU, dan `/cmd_vel`.
- Keyboard W/S/A/D dan penghentian otomatis setelah tombol tidak lagi ditekan.
- Waypoint berbasis odometri/IMU ketika obstacle avoidance dinonaktifkan.
- START DIO 10 dan STOP DIO 11 active-low.
- Light tower: control 12, red 13, green 14, yellow 15.
- Sequence waypoint dapat diubah melalui YAML.
- Titik tugas berhenti minimal 5 detik; titik transit berawalan P tanpa jeda.
- YDLIDAR Tmini Plus menerbitkan `/scan`; area depan yang dipercaya -80..+80°.
- Uji dasar trace dinding kiri telah bergerak, tetapi belum final untuk lomba.

## Waypoint koridor tanpa Nav2

Gunakan `config/waypoints_corridor.yaml`. Titik P mengarahkan robot melalui
tengah celah/dinding dan tidak berhenti. A/B/C berhenti 5,2 detik. Mode
`odometry_or_lidar` dan perlambatan pendekatan sudah tersedia, tetapi perlu
kalibrasi fisik terhadap ukuran zona dan offset LiDAR.

Obstacle avoidance pada navigator lama belum stabil untuk seluruh bentuk
koridor. Untuk pengujian rute dasar, gunakan `enabled: false`, arena kosong,
dan siapkan STOP fisik.

## Nav2

Static map, AMCL, planner, controller, filter scan depan, dan runner telah
dibuat. Pengujian terakhir menemukan aktivasi lifecycle `bt_navigator` belum
stabil pada VMX. Mode ini belum menjadi pilihan utama sampai pengujian lanjutan
berhasil.

## OMS Titan CAN ID 10

Konfigurasi hardware sekarang mengenali `titan1` pada CAN ID 10:

- M2: lift gripper; I naik dan K turun.
- M3: rotasi gripper; J CCW dan L CW.
- Duty awal keyboard default 0,20 dan dapat diturunkan dengan `--oms-speed`.
- Saat tidak ada tombol, E/Q, Ctrl+C, atau program berakhir, command M2/M3 nol.

Kontrol OMS ini baru lolos validasi sintaks/config, belum uji arah hardware.
Uji awal disarankan pada duty 0,10 tanpa beban. M2 memerlukan encoder, limit
switch atas/bawah, current limit, dan position hold/PID sebelum digunakan untuk
mengangkat objek. M3 memerlukan PID posisi bila sudut harus berulang/presisi.
Servo mempunyai loop posisi internal dan tidak memerlukan PID eksternal biasa.

## Berkas utama

- `config/titan_m1_test.yaml`: VMX, Titan base 42, Titan OMS 10, DIO, dan IMU.
- `config/drive_controller.yaml`: model kinematika dan PID base.
- `config/waypoints.yaml`: waypoint dasar.
- `config/waypoints_corridor.yaml`: waypoint dengan titik transit P.
- `titan_keyboard_teleop.py`: keyboard base dan OMS.
- `scripts/start_full_waypoint.sh`: startup waypoint satu terminal.
- `scripts/start_full_keyboard.sh`: keyboard base dan OMS satu terminal.
- `scripts/stop_full_waypoint.sh`: emergency shutdown kedua Titan dan stack.
