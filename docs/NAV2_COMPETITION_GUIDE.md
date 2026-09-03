# Navigasi Kompetisi Berbasis Map dan Nav2

Mode ini menggunakan peta `.yaml/.pgm` dari panitia, AMCL untuk lokalisasi,
NavFn A* untuk rute global, DWB untuk gerak lokal, dan LiDAR depan untuk
obstacle yang tidak terdapat pada peta.

Mode waypoint lama tetap tersedia dan tidak diubah menjadi Nav2.

## 1. File utama

```text
maps/Navigation.yaml
maps/Navigation.pgm
src/studica_control/config/navigation_waypoints.yaml
src/studica_control/config/nav2_navigation.yaml
scripts/start_full_navigation.sh
```

## 2. Mengambil koordinat

```bash
cd /home/vmx/studica_ws
python3 scripts/map_coordinate_picker.py maps/Navigation.yaml
```

Klik pusat posisi HOME dan setiap waypoint. Catat koordinat X/Y. `yaw_deg`
adalah arah depan robot pada titik tersebut: 0 menuju +X, 90 menuju +Y,
180 menuju -X, dan -90 menuju -Y.

## 3. Mengisi waypoint

Edit:

```bash
nano /home/vmx/studica_ws/src/studica_control/config/navigation_waypoints.yaml
```

Isi `home`, `waypoints`, dan `sequence`. Titik terakhir harus `S` dengan pose
yang sama dengan HOME. Setelah semua diperiksa, ubah:

```yaml
configured: true
```

Selama nilainya `false`, tombol START akan selalu ditolak.

## 4. Menjalankan

Pastikan baterai cukup, robot diangkat pada pengujian pertama, dan hanya satu
stack VMX aktif.

```bash
cd /home/vmx/studica_ws
./scripts/stop_full_waypoint.sh
./scripts/start_full_navigation.sh
```

Tunggu `NAVIGASI SIAP`. Letakkan pusat robot tepat pada HOME dengan heading
yang benar, kemudian tekan START DIO 10. Saat START, program mereset odometri
dan yaw, mengirim pose HOME ke AMCL, kemudian menjalankan sequence.

STOP DIO 11 membatalkan goal aktif. `Ctrl+C` menghentikan seluruh stack.

## 5. Aturan status

```text
Menunggu START / gagal : merah solid
Bergerak               : hijau berkedip
Berhenti di waypoint   : kuning berkedip, minimal 5 detik
Selesai di HOME        : merah berkedip (dapat diubah ke red_solid)
```

## 6. LiDAR depan

`front_scan_filter.py` hanya meneruskan -80 sampai +80 derajat sebagai
`/scan_front`. DWB dikonfigurasi tanpa kecepatan linear negatif sehingga
robot tidak mundur ke area yang tidak terlihat. Nav2 dapat berhenti, berputar
di tempat, memperbarui costmap, dan menghitung ulang jalur A*.

## 7. Parameter fisik awal

Footprint awal pada `nav2_navigation.yaml` adalah panjang 0,50 m dan lebar
0,44 m. Inflation radius 0,36 m. Ukur badan aktual termasuk bagian yang paling
menonjol dan koreksi footprint sebelum uji court.

Kecepatan awal dibatasi 0,22 m/s dan 0,8 rad/s. Jangan menaikkannya sebelum
lokalisasi, stopping distance, dan obstacle avoidance tervalidasi.
