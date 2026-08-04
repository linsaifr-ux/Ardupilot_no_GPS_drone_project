#!/usr/bin/env python3
"""survey24 (flight-1 spectrum flight): decode ArduPilot IMU batch sampler
(ISBH/ISBD, ~2 kHz sensor-rate dataflash log) and FFT the hover segment to
find the true prop/motor vibration line for INS_HNTCH_FREQ.

Hover window picked from CTUN: t=15-95s, Alt~0.9-1.0m steady, ThO~0.218-0.222
(matches INS_HNTCH_REF=0.22 already loaded from MOT_THST_HOVER). Climb window
100-150s (Alt 1->21m) reported too for comparison (throttle a bit higher).
"""
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.signal import welch
from pymavlink import mavutil

FN = "/home/jetson/Ardupilot_no_GPS_drone_project/field_data/survey24/2026-07-23 17-32-24.bin"
OUT = "/home/jetson/Ardupilot_no_GPS_drone_project/field_data/survey24/vibe_analysis/batch_spectrum.png"

WINDOWS = [
    ("hover 15-95s (thr~0.22, matches HNTCH_REF)", 15, 95),
    ("climb+high-hover 100-150s", 100, 150),
]

def load():
    mlog = mavutil.mavlink_connection(FN)
    isbh = {}
    isbd = {}
    t0 = None
    while True:
        m = mlog.recv_match(type=["ISBH", "ISBD", "CTUN"], blocking=False)
        if m is None:
            break
        d = m.to_dict()
        if d["mavpackettype"] == "CTUN" and t0 is None:
            t0 = d["TimeUS"]
        elif d["mavpackettype"] == "ISBH":
            isbh[d["N"]] = d
        elif d["mavpackettype"] == "ISBD":
            isbd.setdefault(d["N"], []).append(d)
    return t0, isbh, isbd


def reconstruct(t0, isbh, isbd, type_, instance, lo, hi):
    """Concatenate all batch blocks of (type_, instance) whose SampleUS falls
    in [t0+lo, t0+hi] seconds. Returns (fs, x, y, z) as float arrays (raw
    int16 units - fine for frequency content, no need for mul scaling)."""
    xs, ys, zs = [], [], []
    fs_list = []
    for n, h in sorted(isbh.items()):
        if h["type"] != type_ or h["instance"] != instance:
            continue
        tsec = (h["SampleUS"] - t0) / 1e6
        if not (lo <= tsec <= hi):
            continue
        pkts = sorted(isbd.get(n, []), key=lambda d: d["seqno"])
        for p in pkts:
            xs.extend(p["x"])
            ys.extend(p["y"])
            zs.extend(p["z"])
        fs_list.append(h["smp_rate"])
    if not xs:
        return None
    fs = float(np.mean(fs_list))
    return fs, np.array(xs, dtype=float), np.array(ys, dtype=float), np.array(zs, dtype=float)


def main():
    t0, isbh, isbd = load()
    insts = sorted(set(h["instance"] for h in isbh.values()))
    print(f"batch header instances found: {insts}")

    fig, axes = plt.subplots(2, 2, figsize=(13, 8), sharex=True)
    fig.patch.set_facecolor("#fcfcfb")
    for ax in axes.flat:
        ax.set_facecolor("#fcfcfb")
        ax.grid(True, color="#e8e7e3", lw=0.8)

    colors = ["#2a78d6", "#c22f2e"]
    for (name, lo, hi), col in zip(WINDOWS, colors):
        for row, (type_, label) in enumerate([(1, "gyro"), (0, "accel")]):
            for inst in insts:
                r = reconstruct(t0, isbh, isbd, type_, inst, lo, hi)
                if r is None:
                    continue
                fs, x, y, z = r
                n_samp = len(x)
                nperseg = min(1024, n_samp)
                sig = np.column_stack([x, y, z])
                sig = sig - sig.mean(0)
                f, P = welch(sig, fs=fs, nperseg=nperseg, axis=0)
                ptot = P.sum(1)
                ax = axes[row, 0 if inst == insts[0] else 1]
                ax.semilogy(f, ptot, color=col, lw=1.3, label=f"{name}")
                band = f > 5
                idx = np.argsort(ptot[band])[::-1][:5]
                lines = ", ".join(f"{f[band][i]:.1f}Hz" for i in sorted(idx))
                print(f"[{label} inst{inst}] {name:38s} fs={fs:.1f}Hz n={n_samp:6d} "
                      f"peak lines >5Hz: {lines}")
            axes[row, 0].set_ylabel(f"{label} PSD sum(xyz)")

    for row, label in enumerate(["gyro", "accel"]):
        for col_i, inst in enumerate(insts):
            axes[row, col_i].set_title(f"{label} instance {inst}", loc="left", fontsize=10)
    axes[1, 0].set_xlabel("frequency (Hz) -- Nyquist ~1000 Hz, no stream aliasing here")
    axes[1, 1].set_xlabel("frequency (Hz)")
    axes[0, 0].legend(fontsize=8)
    fig.suptitle("survey24 flight-1: ArduPilot batch-sampler spectra (~2 kHz sensor rate, pre-notch)")
    fig.tight_layout()
    fig.savefig(OUT, dpi=115, facecolor=fig.get_facecolor())
    print(OUT)


if __name__ == "__main__":
    main()
