# Vision Raspberry Pi — named color profiles

Seluruh GUI berbahasa Inggris. Program menggunakan kamera USB/V4L2 dan berjalan mandiri tanpa ROS atau motor.

## Mulai dan ajarkan warna

```bash
cd /home/vmx/studica_ws
./scripts/start_object_feature_detector.sh
```

1. Klik **START**; objek boleh sudah berada di gambar. Tidak perlu kalibrasi latar.
2. Klik **Add Color Sample (ROI)**. Gambar dibekukan selama memilih sampel.
3. Drag kotak kecil pada bagian warna objek, hindari latar, bayangan, dan pantulan. Lepas mouse.
4. Isi **Object name**, periksa HSV jika diperlukan, lalu **Save**.
5. Program mendeteksi warna tersebut di seluruh gambar, termasuk ketika objek berpindah dari lokasi sampel.
6. Ulangi untuk objek lain. **Profiles / Edit** membuka daftar dengan 15 baris dan scrollbar; jumlah profil tidak dibatasi 15.

Nama adalah label yang diajarkan pengguna, bukan hasil pengenalan jenis benda berbasis AI.

## Edit dan seleksi

- Klik **[x] / [ ]** pada kolom **Enabled** untuk mengaktifkan/nonaktifkan profil.
- Pilih baris lalu **Edit** untuk mengubah nama, HSV minimum/maksimum, dan status aktif.
- **Resample** mengambil ulang warna dari gambar kamera untuk profil terpilih, dengan nama tetap.
- **Delete** menghapus profil setelah konfirmasi.
- **Enable All / Disable All** mengubah semua profil sekaligus.
- Perubahan profil tersimpan otomatis di `config/object_feature_profiles.json` dan dimuat saat aplikasi dibuka ulang.
- HSV OpenCV: H 0–179; S/V 0–255. H minimum lebih besar dari maksimum berarti rentang melintasi merah 179/0. S/V minimum harus <= maksimum.

Profil dengan rentang warna tumpang tindih mengikuti prioritas urutan daftar: piksel yang sudah terdeteksi pada profil lebih awal tidak dihitung lagi. Dua objek dengan warna sama tetap bisa terhitung terpisah bila kontur tidak bersentuhan, tetapi warna saja tidak dapat membedakan identitasnya. Nama profil berbeda tidak otomatis membedakan dua benda berwarna sama.

## Format bounding box

Isi **Box label**, lalu **Apply** untuk melihat preview dan menerapkan. Klik **Save Settings** agar format tersimpan saat restart.

Contoh:

```text
{roi} | #{id:02d}
{roi} | {color} | {shape}
{roi} X:{x} Y:{y} W:{w} H:{h}
{roi} | Count:{count}
{type}: {barcode}
```

`{roi}` = nama profil, `{id}` = nomor deteksi pada frame (bukan ID tracking permanen), `{color}` = estimasi warna, `{shape}` = estimasi bentuk 2D, `{count}` = jumlah objek pada profil. `{x}`, `{y}`, `{w}`, `{h}` = bounding box dalam piksel gambar kamera. Fitur yang tidak dipilih ditampilkan sebagai `-`. Barcode dipindai satu kali pada seluruh gambar; gunakan `{barcode}` dan `{type}` untuk isi kodenya.

Centang **Color**, **Shape**, **Count**, dan/atau **Barcode** sesuai kebutuhan. Shape dan Count menggunakan profil warna yang aktif sebagai segmentasi. **Min area** mengabaikan kontur kecil; mulai 1200 lalu sesuaikan ukuran objek. Count adalah jumlah saat ini, bukan total barang yang pernah lewat.

## Pemeriksaan dan batasan

```bash
./scripts/start_object_feature_detector.sh --check
./scripts/start_object_feature_detector.sh --camera 1
/usr/bin/python3 -m unittest discover -s scripts/tests -p 'test_object_feature_profiles.py' -v
```

Pengujian sintetis mencakup 15 profil, nonaktifkan profil, merah melintasi batas hue, objek diam di lokasi lain, prioritas warna tumpang tindih, dan validasi label/HSV. Kinerja 15 objek pada kamera langsung belum diukur. Objek bersentuhan dapat menyatu; pencahayaan memengaruhi HSV.

Program ini belum menerima topik kamera ROS Orbbec. Launcher `start_color_roi_tracker.sh` yang lama memakai Orbbec/ROS dan tetap terpisah. Kamera CSI/libcamera juga belum didukung langsung. Jalankan GUI dari desktop/VNC. Pada pemeriksaan sesi ini `/dev/video*` belum tersedia sehingga pengujian langsung belum dilakukan.

Dependensi sistem bila diperlukan:

```bash
sudo apt update
sudo apt install python3-opencv python3-numpy python3-tk python3-pil python3-pil.imagetk
# Optional extra barcode backend:
sudo apt install python3-pyzbar libzbar0
```

## YOLO Shapes + HSV + Add Object

Mode tambahan memakai YOLO untuk bounding box/kelas bentuk, lalu mencari profil HSV aktif di dalam setiap objek. Dengan model segmentation, warna dihitung hanya di mask objek; dengan model detection biasa, latar dalam bounding box bisa memengaruhi hasil. Profil warna dianggap hadir bila mencakup minimal 8% area box/mask. Beberapa warna dalam satu box tetap dihitung sebagai satu deteksi.

### Install dependensi tambahan

```bash
sudo apt-get install python3-venv
cd /home/vmx/studica_ws
./scripts/setup_vision_yolo.sh
./scripts/start_object_feature_detector.sh
```

Setup menggunakan `.venv-vision` terpisah; launcher memilihnya setelah instalasi sukses. Memerlukan koneksi internet untuk paket. Pada sesi implementasi, instalasi terhenti karena `python3-venv` belum tersedia dan sudo memerlukan password pengguna. Ultralytics serta bobot bentuk belum terpasang.

### Alur penggunaan

1. **Load Shape Model**: pilih file `.pt` lokal yang dilatih untuk kelas bentuk. Daftar kelas ditampilkan setelah berhasil dimuat. Model COCO biasa tidak memiliki kelas bentuk yang dimaksud. Model tidak dilatih atau diunduh oleh Add Object.
2. Aktifkan **YOLO Shapes**. **Confidence** 0–1 adalah ambang skor YOLO (default 0.50). Input inferensi 320 piksel, CPU, satu pekerjaan sekaligus agar GUI tidak menunggu inferensi.
3. Buat profil warna dengan **Add Color Sample (ROI)** seperti sebelumnya.
4. **Add Object**: isi nama, pilih kelas YOLO yang persis sesuai model, pilih satu/beberapa warna, dan isi format label. Semua warna terpilih harus ditemukan pada deteksi yang sama. Tidak memilih warna berarti warna apa pun; `Any` berarti kelas apa pun.
5. Contoh nama `Red Blue Block`, kelas `cuboid`, warna `red` dan `blue`, label `{name} | {shape} | {colors} | {confidence:.2f}`.
6. **Objects / Edit**: edit nama, kelas, warna, label, status Enabled, atau hapus aturan. Aturan pertama yang cocok menang. Deteksi tanpa aturan cocok diberi nama `Unknown`; kelas model tetap tersedia melalui `{shape}`.
7. Aturan tersimpan otomatis bersama profil. Format global **Box label** berlaku jika label aturan kosong. Konfigurasi versi 2 tetap bisa dimuat; penyimpanan baru menggunakan versi 3. Model yang tersimpan harus dimuat lagi lewat **Load Shape Model** saat restart.

Checkbox Color/Shape mengatur informasi yang ditampilkan; pencocokan aturan tetap menggunakan kelas model dan profil HSV. Count pada mode YOLO menghitung semua deteksi yang lolos Min area, termasuk Unknown. Nomor ID adalah nomor per-frame, bukan ID tracking permanen. Min area pada mode YOLO adalah luas bounding box dalam piksel².

Hasil YOLO digambar pada frame yang memang dianalisis. Kecepatan tampilan hasil mengikuti inferensi, belum ada benchmark Raspberry Pi. Mode ROI memakai gambar RGB saat ini dan menghentikan pengajuan inferensi baru selama pengambilan sampel.

Delapan pengujian sintetis/mock berhasil, termasuk satu objek dua warna, pencocokan kelas, prioritas aturan, dan penolakan hasil dari sesi kamera lama. Pengujian ini tidak membuktikan akurasi model YOLO atau performa kamera langsung.
