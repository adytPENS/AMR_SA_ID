# Build Ulang dan Uji Waypoint

Panduan ini dijalankan langsung dari terminal Raspberry Pi/VMX. Sebaiknya tutup
SSH dan VS Code Remote terlebih dahulu agar proses build lebih ringan.

## 1. Build ulang paket

```bash
cd /home/vmx/studica_ws
source /opt/ros/humble/setup.bash

MAKEFLAGS=-j1 CMAKE_BUILD_PARALLEL_LEVEL=1 \
colcon build --packages-select studica_control --executor sequential
```

Tunggu sampai terminal menampilkan hasil seperti berikut:

```text
Summary: 1 package finished
```

Jika build gagal, jangan menjalankan robot. Periksa pesan `ERROR` yang muncul
di terminal terlebih dahulu.

## 2. Aktifkan hasil build

Setelah build berhasil, jalankan:

```bash
source /home/vmx/studica_ws/install/setup.bash
```

Perintah `source` perlu dijalankan kembali pada setiap terminal baru.

## 3. Jalankan waypoint

Pastikan hanya satu program yang memakai VMX HAL. Kemudian jalankan:

```bash
cd /home/vmx/studica_ws
./scripts/start_full_waypoint.sh
```

Jika `obstacle_avoidance.enabled: true`, skrip ini otomatis menjalankan driver
YDLIDAR dengan profil `Tmini.yaml` yang telah diuji dan menunggu data `/scan`.
Tidak perlu membuka terminal LiDAR terpisah.
Jika obstacle avoidance `false`, LiDAR tidak dijalankan.

Tunggu sampai muncul informasi bahwa mode waypoint siap dan motor masih STOP.
Robot tidak boleh langsung bergerak.

- Tekan tombol fisik START pada DIO 10 untuk mulai bergerak.
- Tekan tombol fisik STOP pada DIO 11 untuk menghentikan robot.
- Saat START ditekan, program melakukan zero yaw dan reset pose ke `(0, 0, 0)`,
  lalu menunggu odometri nol terkonfirmasi sebelum motor bergerak. Karena itu,
  tempatkan robot pada titik `S/home` dan arahkan bagian depan robot ke arah
  start lintasan sebelum menekan START.
- Saat belum dimulai, lampu merah menyala solid.
- Saat bergerak, lampu hijau berkedip.
- Saat berhenti sementara di waypoint, lampu kuning berkedip.
- Setelah kembali dan selesai di titik `S`, lampu merah berkedip.

## 4. Hentikan seluruh program

Cara utama adalah menekan tombol STOP DIO 11. Setelah robot berhenti, tekan
`Ctrl+C` satu kali pada terminal yang menjalankan waypoint.

Jika terminal sulit dihentikan, buka terminal lokal lain dan jalankan:

```bash
cd /home/vmx/studica_ws
./scripts/stop_full_waypoint.sh
```

Tunggu sampai proses berhenti sebelum menutup terminal atau mematikan perangkat.

## Catatan keselamatan

- Angkat roda dari lantai pada pengujian pertama setelah perubahan program.
- Sediakan akses cepat ke tombol STOP DIO 11.
- Jangan menjalankan dua instance `start_full_waypoint.sh` secara bersamaan.
- Jangan menjalankan contoh DIO, servo, atau program VMX HAL lain bersamaan
  dengan waypoint.
- Konfigurasi urutan, koordinat, dan obstacle avoidance berada di
  `src/studica_control/config/waypoints.yaml`.

## Membuat peta lapangan

Jalankan seluruh mode mapping dari satu terminal:

```bash
cd /home/vmx/studica_ws
./scripts/stop_full_waypoint.sh
./scripts/start_full_mapping.sh
```

Gunakan `W/A/S/D` untuk bergerak perlahan, `E` untuk stop, dan `Q` untuk
selesai. Jelajahi tepi lapangan dan semua koridor, lalu kembali mendekati
`S/home` agar SLAM dapat melakukan loop closure.

Saat mapping masih aktif, simpan dari terminal kedua:

```bash
cd /home/vmx/studica_ws
./scripts/save_map.sh arena_map
```

Hasil disimpan sebagai `maps/arena_map.yaml` dan `maps/arena_map.pgm`.

## Navigasi dengan map panitia

Untuk navigasi kompetisi berbasis static map, AMCL, dan Nav2, baca
`docs/NAV2_COMPETITION_GUIDE.md`. Setelah koordinat diverifikasi dan
`configured: true`, jalankan:

```bash
cd /home/vmx/studica_ws
./scripts/stop_full_waypoint.sh
./scripts/start_full_navigation.sh
```

### Melihat mapping melalui Foxglove

Pasang bridge satu kali pada Raspberry Pi:

```bash
sudo apt install ros-humble-foxglove-bridge
```

Dari PowerShell PC, sambungkan SSH sekaligus membuat tunnel:

```powershell
ssh -L 8765:localhost:8765 vmx@192.168.50.39
```

Di sesi SSH tersebut jalankan:

```bash
cd /home/vmx/studica_ws
./scripts/start_foxglove_bridge.sh
```

Di Foxglove Studio pilih `Open connection` -> `Foxglove WebSocket`, kemudian
hubungkan ke `ws://localhost:8765`. Tambahkan panel 3D dan topic `/map`,
`/scan`, `/odom`, `/tf`, serta `/tf_static`.
