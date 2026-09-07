# Profil warna dari ROI

Jalankan dari desktop Raspberry Pi atau SSH yang memiliki akses ke desktop Pi:

```bash
cd /home/vmx/studica_ws
./scripts/start_color_roi_tracker.sh
```

Hentikan program kamera lama dengan Ctrl+C sebelum menjalankan kembali.
Jangan gunakan `multiple_colors:=true` untuk profil sendiri: opsi itu memakai
ambang merah/hijau/biru bawaan.

## Memakai profil yang tersimpan

Selain kamera, muncul jendela **Color profiles - click checkboxes**.
Centang kotak di kiri nama untuk menampilkan deteksi profil tersebut. Hilangkan
centang untuk menyembunyikannya. Beberapa profil dapat aktif bersamaan.
Pilihan checkbox disimpan otomatis, termasuk jika semuanya dinonaktifkan.
Gunakan Previous/Next jika terdapat lebih dari delapan profil.

Profil `kuning` pada mesin ini menggunakan kalibrasi pengguna yang sudah berhasil:
HSV bawah `[9, 172, 60]`, atas `[31, 255, 235]`. File kalibrasi lama tetap disimpan.

## Menambah warna atau label objek

1. Klik **Add ROI (R)** atau tekan R di jendela aplikasi.
2. Drag kotak pada bagian warna objek yang diinginkan, lalu tekan Enter/Space.
3. Klik jendela daftar profil untuk fokus keyboard, ketik nama, misalnya `merah`
   atau `Object 2 - hijau`.
4. Tekan Enter atau klik **Save (Enter)**. Profil baru otomatis dicentang.

Esc saat mengisi nama membatalkan penambahan tanpa mengubah profil tersimpan.
Nama harus unik (maksimal 32 karakter ASCII). Nama yang sama ditolak agar ROI
baru tidak menimpa kalibrasi yang sudah bagus. Untuk mengganti nama, klik nama
profil, klik **Rename selected (L)**, edit dengan Backspace, lalu Enter.

Rentang HSV dan status checkbox disimpan di:

```text
/home/vmx/studica_ws/config/color_profiles.yaml
```

Untuk memindahkan ke robot lain, salin file tersebut dan sesuaikan `profiles_file`
di `src/studica_control/config/color_roi_tracker.yaml`. Nilai kosong berarti
`color_profiles.yaml` di folder yang sama dengan `calibration_file`.
File disimpan secara atomik agar kegagalan penulisan tidak merusak file sebelumnya.

## Keluaran

- Kamera: bounding box berlabel persis seperti nama profil, pusat pixel X/Y,
  dan jarak jika tersedia.
- M: tampilkan/sembunyikan mask gabungan profil aktif.
- `/color_tracker/image/compressed`: overlay untuk Foxglove bila bridge berjalan.
- `/color_tracker/result`: JSON dengan `mode: saved_profiles`, `count`, dan
  `objects`. Tiap objek berisi `label`, `color` (alias nama profil), `bbox`,
  `pixel_x`, `pixel_y`, `distance_m`, `x_m`, `y_m`, `hsv_low`, `hsv_high`.

Schema mode ROI kini menggunakan daftar `objects` karena bisa mendeteksi banyak
profil; konsumen schema lama perlu membaca daftar ini. Koordinat pixel mengacu
ke gambar pemrosesan (default 320x180); dimensi gambar juga dikirim dalam JSON.

Q/Esc di luar pengisian nama menutup aplikasi. Tekan Ctrl+C di terminal untuk
menghentikan launch kamera. Data depth basi/tidak tersedia ditampilkan sebagai
jarak N/A, bukan pengukuran lama.

Label diberikan pengguna, bukan hasil pengenalan jenis objek. Profil dengan
rentang warna tumpang tindih dapat menghasilkan dua label di benda yang sama.
Area lain dengan warna serupa juga dapat terdeteksi. Program mendeteksi ulang
setiap frame, belum mempertahankan identitas objek ketika objek berpapasan.

## English quick start

Run `./scripts/start_color_roi_tracker.sh`. In the **Color profiles** window,
tick saved profiles to detect them together. Click **Add ROI**, drag a color
sample, press Enter, type a unique name in the profile window, then **Save**.
Click a profile name and **Rename selected** to rename it without changing its
HSV range. Checkbox selections and named ranges persist across restarts.
