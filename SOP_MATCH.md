# SOP Match Procedures (Standard Operating Procedure)

Dokumen ini berisi prosedur standar untuk operasional robot KRSBI-B sebelum, selama, dan setelah pertandingan.
Tujuan dari SOP ini adalah untuk memastikan keamanan, kesiapan, dan performa maksimal robot.

## 🛠️ Persiapan Pra-Pertandingan

### 1. Hardware Checklist

- [ ] **Baterai**: Pastikan voltase baterai LiPo > 12.0V (Cell > 4.0V).
- [ ] **Mekanik**: Cek roda omni, pastikan rollers bebas hambatan dan tidak ada debris.
- [ ] **Kabel**: Periksa koneksi USB ke Arduino/STM32, Camera, dan IMU.
- [ ] **Kamera**: Pastikan lensa bersih dan tidak blur.
- [ ] **USB Ports**: Pastikan mapping port sesuai (biasanya `/dev/ttyUSB0` untuk mikrokontroler).
- [ ] **Lampu Indikator**: LED Power menyala, LED Status mikrokontroler kedip normal.

### 2. Software & Network Checklist

- [ ] **PC**: Pastikan PC Robot (NUC/Jetson) terhubung ke WiFi lapangan (jika ada) atau WiFi tim.
- [ ] **Remote**: Pastikan SSH bisa connect.
- [ ] **Parameter**: Cek `config/control_config.yaml` dan pastikan PID/Velocity Limits sesuai lapangan.
- [ ] **Vision Calibration**: Jalankan tool kalibrasi warna/thresholding jika pencahayaan lapangan berbeda.
- [ ] **Game Controller**: Pastikan IP Referee Box sudah benar di `game_controller.py`.

## 🚀 Prosedur Start-Up (Urutan Start)

1.  **Nyalakan Robot**: Switch ON Power Utama.
2.  **Tunggu Booting**: Tunggu sampai PC siap (biasanya 30-60 detik).
3.  **SSH Akses**:
    ```bash
    ssh team@robot-ip
    ```
4.  **Source Environment**:
    ```bash
    source ~/ros2_ws/install/setup.bash
    ```
5.  **Jalankan Driver Hardware (Comm)**:
    ```bash
    ros2 launch krsbi_comm communication_bringup.launch.py port:=/dev/ttyUSB0
    ```
    _Pastikan tidak ada error "Serial Exception"._
6.  **Jalankan Vision**:
    ```bash
    ros2 launch krsbi_vision vision_bringup.launch.py use_yolo:=true
    ```
    _Cek frame rate di rostopic hz /krsbi/vision/ball._
7.  **Jalankan Control & Decision**:
    ```bash
    ros2 launch krsbi_decision decision_bringup.launch.py role:=striker
    ```
    _Robot sekarang dalam mode STANDBY menunggu perintah referee._

## ⚽ Selama Pertandingan

### 1. Monitoring

- Pantau status baterai melalui `rqt` atau `krsbi_interface`.
- Pantau `krsbi/game/state` (harus sinkron dengan wasit).
- Jika robot macet/error, siap-siap tombol **EMERGENCY STOP** (Software atau Hardware).

### 2. Manual Override (Jika Diperbolehkan Wasit)

- Jika otomatisasi gagal, gunakan joystick:
  ```bash
  ros2 launch teleop_twist_joy teleop.launch.py
  ```
  _Pastikan safety switch diaktifkan._

### 3. Masalah Umum & Solusi Cepat

- **Robot berputar tidak terkendali**: Cek IMU calibration atau Matikan Odometry Fusion.
  ```bash
  ros2 param set /localization use_imu false
  ```
- **Bola tidak terdeteksi**: Cek eksposur kamera (terlalu gelap/terang).
- **Koneksi putus**: Cek kabel USB atau restart node `krsbi_comm`.

## 🏁 Pasca-Pertandingan

1.  **Shutdown Software**: `Ctrl+C` pada semua terminal.
2.  **Matikan PC**: `sudo shutdown now`.
3.  **Matikan Power Utama**: Switch OFF.
4.  **Cabut Baterai**: Segera cabut dan charge baterai untuk match berikutnya.
5.  **Data Log**: Salin `rosbag` jika merekam data untuk analisis.
    ```bash
    ros2 bag record -a -o match_log_game1
    ```

## 🚨 Emergency Procedures

1.  **Tabrakan/Jatuh**: SEGERA tekan tombol Emergency Stop fisik di robot.
2.  **Asap/Bau Gosong**: MATIKAN total power, cabut baterai, gunakan APAR jika api terlihat.
3.  **Hilang Kendali (Robot Lari Sendiri)**: Angkat robot (jika aman) agar roda menggantung, matikan power.

---

**Catatan**: Selalu utamakan keselamatan tim dan perlengkapan. Jangan menyentuh komponen panas (motor driver) sesaat setelah match.
