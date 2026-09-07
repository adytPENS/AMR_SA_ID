# Mode M: mengajar posisi default OMS

Jalankan `bash scripts/start_full_keyboard.sh`. Program mulai dalam mode manual.
W/S/A/D menggerakkan bodi, I/K lift, J/L putar OMS, G/H slide,
R/T wrist, Y/U gripper. Tekan P bila target servo standard belum diinisialisasi.
Atur mekanisme di posisi default yang diinginkan, lalu tekan M.

M menghentikan bodi, lift, putaran dan slide, menunggu feedback encoder baru
setelah jeda 0,3 detik, lalu menyimpan `config/oms_default.json` secara atomik.
Selama CAPTURE_DEFAULT, READY, dan ERROR, tombol gerak manual tidak aktif.
E/X membatalkan mode M dan kembali ke manual; Q keluar. STOP fisik
menghentikan gerakan dan menahan kontrol selama tombol ditekan.
Setelah E/X, atur ulang posisi dan tekan M lagi untuk mengganti acuan.

State yang sudah dijalankan: MANUAL → CAPTURE_DEFAULT → READY.
Feedback yang tidak tersedia atau penyimpanan gagal masuk ERROR.
Status diterbitkan pada `/human_interaction/status` dan terminal.
Tombol START dikenali pada tepi tekan baru (setelah pernah dilepas).

## Data yang disimpan

- Lift dan putar: encoder poros output motor dalam putaran, sesuai konfigurasi Titan.
- Wrist/gripper: target sudut perintah, bukan pengukuran fisik.
- Slide: posisi manual saat M menjadi acuan, tanpa pengukuran posisi.
- Gripper tidak boleh dikembalikan ke target terbuka saat membawa objek.

Encoder absolut yang disimpan hanya berlaku selama sesi hardware yang sama.
File tidak otomatis dimuat untuk menggerakkan OMS setelah reboot/reset encoder;
ajarkan lagi dengan M. File ini belum menyimpan koordinat home navigasi.

## Slide dengan delay

Edit `config/human_interaction.yaml`. `extend_duty_percent` dan
`retract_duty_percent` memakai tanda arah yang sama dengan perintah servo
continuous. Durasi masing-masing menggunakan detik; nilai awal 0 berarti
belum disetel. Nilai ini disalin ke snapshot saat M. Timer slide untuk urutan
pengambilan belum dijalankan; konfigurasi ini tidak mengubah tombol G/H manual.

## Batas implementasi saat ini

START belum menggerakkan misi otomatis: navigasi ke A/B, scan kamera 270°,
penyelarasan objek, pengambilan, pengembalian OMS dan perjalanan home belum
terhubung. Pesan START menjelaskan bahwa konfigurasi misi belum tersedia.
Rasio mekanis sudut OMS, warna target, koordinat/rute dan posisi pengambilan
belum ditentukan. B/commissioning belum diimplementasikan.
