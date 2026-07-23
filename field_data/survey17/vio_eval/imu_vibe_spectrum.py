#!/usr/bin/env python3
"""survey17: accel/gyro spectra per flight phase — is vibration aliasing visible
in the 200 Hz MAVLink RAW_IMU stream that feeds OpenVINS?"""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.signal import welch

S = "/home/jetson/Ardupilot_no_GPS_drone_project/field_data/survey17"
T0 = 1784630291.0294251

d = np.genfromtxt(f"{S}/imu.csv", delimiter=",", names=True)
t = d["stamp_ros"] - T0
acc = np.column_stack([d["ax"], d["ay"], d["az"]])
gyr = np.column_stack([d["wx"], d["wy"], d["wz"]])

WINDOWS = [
    ("static pad 215-255 s (motors off)", 215, 255, "#52514e"),
    ("hover/low pass 300-360 s", 300, 360, "#2a78d6"),
    ("full-throttle climb 400-440 s", 400, 440, "#c22f2e"),
    ("cruise 500-700 s", 500, 700, "#eb6834"),
]

fig, axes = plt.subplots(2, 1, figsize=(12, 9), sharex=True)
fig.patch.set_facecolor("#fcfcfb")
for ax in axes:
    ax.set_facecolor("#fcfcfb")
    ax.grid(True, color="#e8e7e3", lw=0.8)

print(f"{'window':38s} {'acc std xyz (m/s^2)':>26s} {'gyro std xyz (rad/s)':>26s}")
for name, lo, hi, col in WINDOWS:
    m = (t >= lo) & (t <= hi)
    fs = 1.0 / np.median(np.diff(t[m]))
    astd = acc[m].std(0)
    gstd = gyr[m].std(0)
    print(f"{name:38s} {astd[0]:8.3f}{astd[1]:8.3f}{astd[2]:8.3f}"
          f"   {gstd[0]:8.4f}{gstd[1]:8.4f}{gstd[2]:8.4f}")
    # PSD of the norm-removed accel (vibration, not motion) and gyro-x
    a_hp = acc[m] - acc[m].mean(0)
    f, Pa = welch(a_hp, fs=fs, nperseg=2048, axis=0)
    g_hp = gyr[m] - gyr[m].mean(0)
    _, Pg = welch(g_hp, fs=fs, nperseg=2048, axis=0)
    axes[0].semilogy(f, Pa.sum(1), color=col, lw=1.4, label=name)
    axes[1].semilogy(f, Pg.sum(1), color=col, lw=1.4, label=name)
    # report top spectral lines above 5 Hz
    ptot = Pa.sum(1)
    band = f > 5
    idx = np.argsort(ptot[band])[::-1][:4]
    lines = ", ".join(f"{f[band][i]:.1f}Hz" for i in sorted(idx))
    print(f"{'':38s} accel peak lines >5 Hz: {lines}")

axes[0].set_ylabel("accel PSD sum(xyz) ((m/s$^2$)$^2$/Hz)")
axes[1].set_ylabel("gyro PSD sum(xyz) ((rad/s)$^2$/Hz)")
axes[1].set_xlabel("frequency (Hz)  — Nyquist 100 Hz: content above here is folded down")
axes[0].set_title("survey17 IMU spectra by flight phase (200 Hz MAVLink RAW_IMU)",
                  loc="left")
axes[0].legend(fontsize=9)
fig.tight_layout()
out = f"{S}/vio_eval/imu_vibe_spectrum.png"
fig.savefig(out, dpi=115, facecolor=fig.get_facecolor())
print(out)
