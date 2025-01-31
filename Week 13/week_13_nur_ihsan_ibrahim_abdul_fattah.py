# -*- coding: utf-8 -*-
"""Week 13_Nur Ihsan Ibrahim Abdul Fattah.ipynb

Automatically generated by Colab.

Original file is located at
    https://colab.research.google.com/drive/1sWNl97zz4j4-fiwjjSlOfl7pGQR_5D5c

Extended Kalman Filter (EKF): Robot Navigasi dengan GPS dan IMU

Tujuan EKF: Memperkirakan posisi dan orientasi robot secara optimal dengan memanfaatkan linierisasi data dari sensor (misalnya GPS dan IMU).
"""

import numpy as np
import matplotlib.pyplot as plt

# Parameter EKF
dt = 0.1  # Waktu langkah
state = np.array([0, 0, 0])  # [x, y, theta] posisi awal
covariance = np.eye(3) * 0.1  # Covariance matrix awal
process_noise = np.diag([0.01, 0.01, 0.001])  # Proses noise (Q)
measurement_noise = np.diag([5, 5])  # Noise GPS (R)

# Model Gerak
def motion_model(state, control, dt):
    x, y, theta = state
    v, omega = control
    x_new = x + v * np.cos(theta) * dt
    y_new = y + v * np.sin(theta) * dt
    theta_new = theta + omega * dt
    return np.array([x_new, y_new, theta_new])

# Jacobian untuk model gerak
def jacobian_motion(state, control, dt):
    _, _, theta = state
    v, _ = control
    F = np.array([
        [1, 0, -v * np.sin(theta) * dt],
        [0, 1,  v * np.cos(theta) * dt],
        [0, 0,  1]
    ])
    return F

# Model Pengamatan (GPS)
def measurement_model(state):
    return state[:2]

# Jacobian untuk pengamatan
def jacobian_measurement():
    return np.array([
        [1, 0, 0],
        [0, 1, 0]
    ])

# Data GPS dan IMU Simulasi
np.random.seed(42)
true_positions = [np.array([0, 0, 0])]
gps_data = []
imu_controls = []

for t in range(100):
    # Kontrol IMU (kecepatan linear dan sudut)
    v = 1.0
    omega = 0.1
    imu_controls.append([v, omega])

    # Posisi sebenarnya
    true_position = motion_model(true_positions[-1], [v, omega], dt)
    true_positions.append(true_position)

    # Data GPS dengan noise
    gps = measurement_model(true_position) + np.random.multivariate_normal([0, 0], measurement_noise)
    gps_data.append(gps)

# EKF Implementasi
estimated_positions = [state]
for i in range(len(gps_data)):
    # Predict step
    control = imu_controls[i]
    state_pred = motion_model(estimated_positions[-1], control, dt)
    F = jacobian_motion(estimated_positions[-1], control, dt)
    covariance_pred = F @ covariance @ F.T + process_noise

    # Update step
    z = gps_data[i]
    H = jacobian_measurement()
    y = z - measurement_model(state_pred)
    S = H @ covariance_pred @ H.T + measurement_noise
    K = covariance_pred @ H.T @ np.linalg.inv(S)

    state_est = state_pred + K @ y
    covariance = (np.eye(3) - K @ H) @ covariance_pred
    estimated_positions.append(state_est)

# Plot Hasil
true_positions = np.array(true_positions)
gps_data = np.array(gps_data)
estimated_positions = np.array(estimated_positions)

plt.figure(figsize=(10, 6))
plt.plot(true_positions[:, 0], true_positions[:, 1], 'g-', label='True Path')
plt.scatter(gps_data[:, 0], gps_data[:, 1], c='r', s=20, label='GPS Data (Noise)')
plt.plot(estimated_positions[:, 0], estimated_positions[:, 1], 'b-', label='EKF Estimate')
plt.legend()
plt.title("Extended Kalman Filter (EKF) for GPS+IMU")
plt.xlabel("X Position (m)")
plt.ylabel("Y Position (m)")
plt.grid()
plt.show()

"""Hasil Analisis Output: Program ini mensimulasikan penerapan Extended Kalman Filter (EKF) untuk mengestimasi posisi kendaraan dengan menggabungkan data kontrol dari IMU dan pengamatan GPS yang mengandung noise. Simulasi dilakukan selama 100 langkah waktu dengan kendaraan bergerak pada lintasan yang ditentukan oleh kecepatan linear konstan (1 m/s) dan kecepatan sudut kecil (0.1 rad/s). Posisi sebenarnya dihitung menggunakan model gerak, sementara data GPS dihasilkan dengan menambahkan noise Gaussian untuk mensimulasikan pengukuran dunia nyata. Dalam setiap langkah, EKF melakukan prediksi posisi menggunakan model gerak, memperbarui estimasi berdasarkan data GPS dengan memanfaatkan Jacobian model, dan menyaring noise untuk meningkatkan akurasi. Hasilnya menunjukkan bahwa jalur estimasi EKF (garis biru) berhasil mendekati jalur sebenarnya (garis hijau), meskipun data GPS (titik merah) memiliki penyimpangan yang signifikan akibat noise. Hal ini membuktikan kemampuan EKF dalam mengintegrasikan informasi dari dua sumber berbeda, menyaring ketidakpastian, dan menghasilkan estimasi posisi yang lebih konsisten dan akurat dibandingkan pengamatan GPS langsung.

Unscented Kalman Filter (UKF) untuk Estimasi Navigasi Robot Menggunakan Data GPS dan IMU

Tujuan UKF: Mengestimasi posisi dan orientasi robot dalam sistem non-linear menggunakan sigma points untuk meningkatkan akurasi tanpa memerlukan Jacobian.
"""

# Install FilterPy
!pip install filterpy

# Import modul yang diperlukan
import numpy as np
import matplotlib.pyplot as plt
from filterpy.kalman import UnscentedKalmanFilter as UKF
from filterpy.kalman import MerweScaledSigmaPoints

# UKF Setup
def fx(state, dt, control):
    x, y, theta = state
    v, omega = control
    x_new = x + v * np.cos(theta) * dt
    y_new = y + v * np.sin(theta) * dt
    theta_new = theta + omega * dt
    return np.array([x_new, y_new, theta_new])

def hx(state):
    return state[:2]  # Observasi (x, y)

# Sigma points untuk UKF
points = MerweScaledSigmaPoints(n=3, alpha=0.1, beta=2., kappa=1)
ukf = UKF(dim_x=3, dim_z=2, fx=fx, hx=hx, dt=0.1, points=points)
ukf.x = np.array([0., 0., 0.])  # State awal
ukf.P *= 0.1
ukf.Q = np.diag([0.01, 0.01, 0.01])  # Noise proses
ukf.R = np.diag([5, 5])  # Noise pengamatan GPS

# Simulasi Data
np.random.seed(42)
dt = 0.1
gps_data = []
controls = []
true_states = [np.array([0, 0, 0])]

for t in range(100):
    # Kontrol gerakan (kecepatan dan rotasi)
    control = np.array([1.0, 0.1])
    controls.append(control)

    # Gerak robot sebenarnya
    true_state = fx(true_states[-1], dt, control)
    true_states.append(true_state)

    # Pengamatan GPS dengan noise
    gps = true_state[:2] + np.random.multivariate_normal([0, 0], np.diag([5, 5]))
    gps_data.append(gps)

# Jalankan UKF
ukf_positions = []
for i, control in enumerate(controls):
    ukf.predict(control=control)
    ukf.update(gps_data[i])
    ukf_positions.append(ukf.x)

# Plot hasil
true_states = np.array(true_states)
gps_data = np.array(gps_data)
ukf_positions = np.array(ukf_positions)

plt.figure(figsize=(10, 6))
plt.plot(true_states[:, 0], true_states[:, 1], 'g-', label='True Path')  # Jalur sebenarnya
plt.scatter(gps_data[:, 0], gps_data[:, 1], c='r', s=20, label='GPS Data (Noise)')  # Data GPS
plt.plot(ukf_positions[:, 0], ukf_positions[:, 1], '-', color='purple', label='UKF Estimate')  # Estimasi UKF
plt.legend()
plt.title("Unscented Kalman Filter (UKF) for GPS+IMU")
plt.xlabel("X Position (m)")
plt.ylabel("Y Position (m)")
plt.grid()
plt.show()

"""Hasil Analisis Output: Program ini menggunakan Unscented Kalman Filter (UKF) untuk memperkirakan posisi kendaraan dengan memanfaatkan data kontrol IMU dan pengamatan GPS yang berisik. Simulasi dilakukan selama 100 langkah waktu, dengan kendaraan bergerak pada jalur yang ditentukan oleh kecepatan linear (1 m/s) dan kecepatan sudut kecil (0.1 rad/s). Jalur sebenarnya dihitung menggunakan model gerak nonlinier, sementara data GPS dihasilkan dengan menambahkan noise Gaussian untuk mensimulasikan ketidakpastian pengamatan. UKF memprediksi posisi kendaraan menggunakan kontrol IMU dan memperbarui estimasi berdasarkan data GPS, memanfaatkan sigma points untuk menangkap distribusi nonlinier sistem. Hasil menunjukkan bahwa estimasi jalur UKF (garis ungu) berhasil mendekati jalur sebenarnya (garis hijau), meskipun data GPS (titik merah) memiliki noise signifikan. Kemampuan UKF untuk menyaring noise dan mengintegrasikan informasi dari kedua sumber data menghasilkan estimasi posisi yang lebih halus, konsisten, dan akurat dibandingkan data GPS langsung.

Tracking Objek Bergerak dengan Kalman Filter

Tujuan: Objek bergerak dengan pola sinusoidal, sensor mengukur posisi objek dengan noise, dan Kalman Filter digunakan untuk memperkirakan posisi sebenarnya dari objek.
"""

import numpy as np
import matplotlib.pyplot as plt

# Fungsi Model Gerak (Linear)
def motion_model(state, dt):
    # State: [posisi_x, kecepatan_x, posisi_y, kecepatan_y]
    F = np.array([
        [1, dt, 0,  0],
        [0,  1, 0,  0],
        [0,  0, 1, dt],
        [0,  0, 0,  1]
    ])
    return F @ state

# Model Pengamatan (Hanya Posisi)
def measurement_model(state):
    return np.array([state[0], state[2]])  # [posisi_x, posisi_y]

# Jacobian untuk Pengamatan
def jacobian_measurement():
    return np.array([
        [1, 0, 0, 0],
        [0, 0, 1, 0]
    ])

# Inisialisasi Variabel
dt = 0.1  # Timestep
state = np.array([0, 1, 0, 1])  # [pos_x, vel_x, pos_y, vel_y]
covariance = np.eye(4) * 0.1  # Covariance Matrix
process_noise = np.eye(4) * 0.01  # Proses noise (Q)
measurement_noise = np.eye(2) * 0.5  # Noise sensor posisi (R)

# Simulasi Data
np.random.seed(42)
true_states = [state]
measurements = []

for t in range(100):
    # Gerak objek sebenarnya (sinusoidal)
    state[0] += np.sin(0.1 * t) * 0.1  # Posisi X
    state[2] += np.cos(0.1 * t) * 0.1  # Posisi Y
    state = motion_model(state, dt)
    true_states.append(state)

    # Sensor membaca posisi dengan noise
    measurement = measurement_model(state) + np.random.multivariate_normal([0, 0], measurement_noise)
    measurements.append(measurement)

# Jalankan Kalman Filter
estimated_states = [np.array([0, 1, 0, 1])]
for i in range(len(measurements)):
    # Predict step
    F = np.array([
        [1, dt, 0,  0],
        [0,  1, 0,  0],
        [0,  0, 1, dt],
        [0,  0, 0,  1]
    ])
    state_pred = F @ estimated_states[-1]
    covariance_pred = F @ covariance @ F.T + process_noise

    # Update step
    z = measurements[i]
    H = jacobian_measurement()
    y = z - H @ state_pred
    S = H @ covariance_pred @ H.T + measurement_noise
    K = covariance_pred @ H.T @ np.linalg.inv(S)

    state_est = state_pred + K @ y
    covariance = (np.eye(4) - K @ H) @ covariance_pred
    estimated_states.append(state_est)

# Plot Hasil
true_states = np.array(true_states)
measurements = np.array(measurements)
estimated_states = np.array(estimated_states)

plt.figure(figsize=(10, 6))
plt.plot(true_states[:, 0], true_states[:, 2], 'g-', label='True Path')  # Jalur sebenarnya
plt.scatter(measurements[:, 0], measurements[:, 1], c='r', s=20, label='Sensor Data (Noisy)')  # Data Sensor
plt.plot(estimated_states[:, 0], estimated_states[:, 2], 'b--', label='KF Estimate')  # Estimasi KF
plt.legend()
plt.title("Kalman Filter: Tracking Objek Bergerak dengan Sensor Noisy")
plt.xlabel("X Position (m)")
plt.ylabel("Y Position (m)")
plt.grid()
plt.show()

"""Hasil Analisis Output : Program ini menggunakan Kalman Filter (KF) untuk melacak posisi objek yang bergerak dalam pola sinusoidal dengan model gerak linear dan pengamatan sensor yang berisik. Objek diperbarui setiap 0.1 detik dengan posisi sebenarnya dihitung menggunakan model gerak, sementara data pengamatan dihasilkan dari posisi objek yang ditambahkan noise Gaussian untuk mensimulasikan ketidakpastian sensor. KF memprediksi posisi objek berdasarkan model gerak dan memperbarui estimasi dengan data sensor menggunakan proses prediksi dan pembaruan secara iteratif. Hasil simulasi menunjukkan bahwa estimasi jalur KF (garis biru putus-putus) berhasil mendekati jalur sebenarnya (garis hijau) meskipun data sensor (titik merah) mengandung noise signifikan. KF mampu menyaring noise dan mengintegrasikan data model gerak serta pengamatan sensor untuk menghasilkan estimasi posisi yang lebih akurat, halus, dan konsisten.

Tracking Drone dengan Gerakan Parabola

Tujuan: Drone terbang dengan gerakan parabola di 2D. Sensor hanya memberikan data posisi drone (x, y) dengan noise. Kalman Filter digunakan untuk memperkirakan posisi dan kecepatan drone.
"""

import numpy as np
import matplotlib.pyplot as plt

# Fungsi Model Gerak
def motion_model(state, dt):
    # State: [posisi_x, kecepatan_x, posisi_y, kecepatan_y]
    F = np.array([
        [1, dt, 0,  0],
        [0,  1, 0,  0],
        [0,  0, 1, dt],
        [0,  0, 0,  1]
    ])
    return F @ state

# Model Pengamatan (Hanya Posisi)
def measurement_model(state):
    return np.array([state[0], state[2]])  # [posisi_x, posisi_y]

# Jacobian untuk Pengamatan
def jacobian_measurement():
    return np.array([
        [1, 0, 0, 0],
        [0, 0, 1, 0]
    ])

# Inisialisasi Variabel
dt = 0.1  # Timestep
state = np.array([0, 5, 0, 15])  # [pos_x, vel_x, pos_y, vel_y]
covariance = np.eye(4) * 0.1  # Covariance Matrix
process_noise = np.eye(4) * 0.01  # Proses noise (Q)
measurement_noise = np.eye(2) * 0.5  # Noise sensor posisi (R)

# Simulasi Data
np.random.seed(42)
true_states = [state]
measurements = []

for t in range(100):
    # Gerakan parabola: Y dipengaruhi gravitasi
    state[3] -= 0.98 * dt  # Gravitasi (penurunan kecepatan Y)
    state = motion_model(state, dt)
    true_states.append(state)

    # Sensor membaca posisi dengan noise
    measurement = measurement_model(state) + np.random.multivariate_normal([0, 0], measurement_noise)
    measurements.append(measurement)

# Jalankan Kalman Filter
estimated_states = [np.array([0, 5, 0, 15])]
for i in range(len(measurements)):
    # Predict step
    F = np.array([
        [1, dt, 0,  0],
        [0,  1, 0,  0],
        [0,  0, 1, dt],
        [0,  0, 0,  1]
    ])
    state_pred = F @ estimated_states[-1]
    covariance_pred = F @ covariance @ F.T + process_noise

    # Update step
    z = measurements[i]
    H = jacobian_measurement()
    y = z - H @ state_pred
    S = H @ covariance_pred @ H.T + measurement_noise
    K = covariance_pred @ H.T @ np.linalg.inv(S)

    state_est = state_pred + K @ y
    covariance = (np.eye(4) - K @ H) @ covariance_pred
    estimated_states.append(state_est)

# Plot Hasil
true_states = np.array(true_states)
measurements = np.array(measurements)
estimated_states = np.array(estimated_states)

plt.figure(figsize=(10, 6))
plt.plot(true_states[:, 0], true_states[:, 2], 'g-', label='True Path')  # Jalur sebenarnya
plt.scatter(measurements[:, 0], measurements[:, 1], c='r', s=20, label='Sensor Data (Noisy)')  # Data Sensor
plt.plot(estimated_states[:, 0], estimated_states[:, 2], 'b--', label='KF Estimate')  # Estimasi KF
plt.legend()
plt.title("Kalman Filter: Tracking Drone dengan Gerakan Parabola")
plt.xlabel("X Position (m)")
plt.ylabel("Y Position (m)")
plt.grid()
plt.show()

"""Hasil Analisis Output: Program ini menggunakan Kalman Filter (KF) untuk melacak gerakan parabola sebuah objek, seperti drone, yang dipengaruhi oleh gravitasi. Gerakan objek dimodelkan menggunakan posisi dan kecepatan di sumbu X dan Y, di mana kecepatan vertikal (Y) berkurang akibat gravitasi pada setiap langkah waktu sebesar 0.1 detik. Data sensor yang digunakan hanya mencakup posisi X dan Y yang terpengaruh noise Gaussian untuk mensimulasikan pengamatan yang tidak sempurna. KF memprediksi posisi objek menggunakan model gerak dan memperbarui estimasi dengan data sensor melalui proses iteratif. Hasil simulasi menunjukkan bahwa jalur estimasi KF (garis biru putus-putus) berhasil mendekati jalur sebenarnya (garis hijau) meskipun data sensor (titik merah) memiliki noise signifikan. Ini menegaskan kemampuan KF dalam menyaring noise dan mengintegrasikan informasi model gerak serta pengamatan sensor untuk memberikan estimasi posisi yang akurat dan konsisten, bahkan dalam skenario dengan ketidakpastian tinggi.







"""