# Kalibrasi wrist dan end effector

Program memakai service yang sama dengan servo_example.py. Servo harus sudah
berada dalam mode standard. Yang disimpan adalah target perintah, bukan posisi
aktual dari sensor.

1. Terminal pertama: jalankan `bash scripts/start_full_keyboard.sh`, tekan E,
   lalu biarkan keyboard diam. Jangan menjalankan servo_example bersamaan.
2. Terminal kedua: jalankan `bash scripts/start_servo_calibration.sh`.
3. R/T menambah/mengurangi sudut wrist. Y/U menambah/mengurangi sudut EoF.
   Awalnya setiap karakter tombol mengubah 1 derajat; +/- mengubah langkah
   antara 1 dan 10 derajat. Key repeat terminal juga menghasilkan langkah.
4. Setelah posisi fisik tepat: 1 merekam wrist kiri, 2 wrist kanan,
   3 EoF kiri, 4 EoF kanan. Label kiri/kanan adalah posisi mekanis yang kamu pilih,
   tidak harus urutan angka sudut tertentu. Penandaan menunggu service berhasil;
   keberhasilan service bukan konfirmasi bahwa mekanisme sudah selesai bergerak.
5. S menyimpan ke `src/studica_control/config/keyboard_servos.yaml`.
   Endpoint yang tidak ditandai tetap memakai nilai lama. Q keluar tanpa
   penyimpanan otomatis. Servo mempertahankan target terakhir.
6. Keluar dari keyboard lama dan jalankan ulang untuk membaca YAML baru.

Tidak ada gerakan servo otomatis saat program dibuka. Jika state awal belum
tersedia, tombol gerak pertama mengirim sudut awal 0 derajat; dapat diubah dengan
`--wrist-start` dan `--eof-start`. Jika state valid tersedia, target terakhir itu
menjadi awal. State ini bukan pembacaan posisi poros yang diputar tangan.

Program tidak menyalakan server hardware baru dan tidak mengendalikan bodi,
lift OMS atau slide. Jika service belum ada, pesan menunggu muncul di terminal.
Source: `src/studica_control/src/components/examples/python/servo_calibration.py`.
