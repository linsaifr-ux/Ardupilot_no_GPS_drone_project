#!/usr/bin/env python3
"""survey25 (Flight 2, pre-notch/333Hz-stream): OpenVINS vs GPS ground truth.

Timeline (video-relative s, from telemetry.csv/meta.json):
0-110 static, 112-167 climb (0->100m), 167-319 cruise legs @ ~100m AGL,
319-375 descent, 375-471 low hover before land.
"""
import csv, math
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

S = "/home/jetson/Ardupilot_no_GPS_drone_project/field_data/survey25"
import json
T0 = json.load(open(f"{S}/meta.json"))["video_start_unix"]

rows = []
with open(f"{S}/telemetry.csv") as f:
    for r in csv.DictReader(f):
        try:
            rows.append((float(r["unix_time"]), float(r["lat"]), float(r["lon"]), float(r["alt_agl"])))
        except ValueError:
            pass
tel = np.array(rows)
lat0, lon0 = tel[0, 1], tel[0, 2]
latm = 111132.954 - 559.822 * math.cos(2 * math.radians(lat0))
lonm = 111412.84 * math.cos(math.radians(lat0)) - 93.5 * math.cos(3 * math.radians(lat0))
gxy = np.column_stack([(tel[:, 2] - lon0) * lonm, (tel[:, 1] - lat0) * latm])
tg = tel[:, 0]


def yaw_align(src, dst, scale=False):
    ms, md = src.mean(0), dst.mean(0)
    s, d = src - ms, dst - md
    yaw = math.atan2(np.sum(s[:, 0] * d[:, 1] - s[:, 1] * d[:, 0]),
                     np.sum(s[:, 0] * d[:, 0] + s[:, 1] * d[:, 1]))
    c, si = math.cos(yaw), math.sin(yaw)
    R = np.array([[c, -si], [si, c]])
    k = 1.0
    if scale:
        sr = (R @ s.T).T
        k = np.sum(sr * d) / np.sum(sr * sr)
    t = md - k * (R @ ms)
    return k * (R @ src.T).T + t, k


def load(csvf):
    return np.genfromtxt(csvf, delimiter=",", names=True)


def seg(v, lo, hi):
    m = (v["t"] >= T0 + lo) & (v["t"] <= T0 + hi)
    tv = v["t"][m]
    xy = np.column_stack([v["px"][m], v["py"][m]])
    gt = np.column_stack([np.interp(tv, tg, gxy[:, i]) for i in range(2)])
    agl = np.interp(tv, tg, tel[:, 3])
    ma = tv <= tv[0] + 20
    alt = v["pz"][m] - v["pz"][m][ma].mean() + agl[ma].mean()
    return tv, xy, gt, alt, agl


def stats(name, v, lo, hi):
    tv, xy, gt, alt, agl = seg(v, lo, hi)
    if len(tv) < 5:
        print(f"{name}  [{lo}-{hi}s] -- no VIO data in window (not initialized / diverged out)")
        return None
    al, _ = yaw_align(xy, gt)
    sc, k = yaw_align(xy, gt, scale=True)
    e = np.linalg.norm(al - gt, axis=1)
    es = np.linalg.norm(sc - gt, axis=1)
    ea = np.abs(alt - agl)
    path = np.linalg.norm(np.diff(gt, axis=0), axis=1).sum()
    print(f"{name}  [{lo}-{hi}s, {path:.0f} m gt path, n={len(tv)}]")
    print(f"  4DOF ATE2D rmse {np.sqrt((e**2).mean()):7.1f}  max {e.max():7.1f} m"
          f"   | best-fit scale {k:.3f}  scale-corr rmse {np.sqrt((es**2).mean()):7.1f} m")
    print(f"  alt vs baro AGL (init-anchored): rmse {np.sqrt((ea**2).mean()):5.1f}"
          f"  max {ea.max():5.1f} m")
    return dict(tv=tv, xy=xy, gt=gt, al=al, sc=sc, k=k,
                rmse=np.sqrt((e**2).mean()), rmse_sc=np.sqrt((es**2).mean()))


def diverge_time(v, lo, align_win=30.0, thresh=100.0):
    m = v["t"] >= T0 + lo
    tv = v["t"][m]
    if len(tv) < 5:
        return None
    xy = np.column_stack([v["px"][m], v["py"][m]])
    gt = np.column_stack([np.interp(tv, tg, gxy[:, i]) for i in range(2)])
    ma = tv <= tv[0] + align_win
    if ma.sum() < 5:
        return None
    ms, md = xy[ma].mean(0), gt[ma].mean(0)
    s, d = xy[ma] - ms, gt[ma] - md
    yaw = math.atan2(np.sum(s[:, 0] * d[:, 1] - s[:, 1] * d[:, 0]),
                     np.sum(s[:, 0] * d[:, 0] + s[:, 1] * d[:, 1]))
    c, si = math.cos(yaw), math.sin(yaw)
    R = np.array([[c, -si], [si, c]])
    est = (R @ (xy - ms).T).T + md
    e = np.linalg.norm(est - gt, axis=1)
    idx = np.argmax(e > thresh)
    if e[idx] <= thresh:
        return None
    return tv[idx] - T0


print("=== survey25 (Flight 2, pre-notch, 333Hz RAW_IMU stream) ===\n")
vf = load(f"{S}/vio_eval/vio_full.csv")
vc = load(f"{S}/vio_eval/vio_cruise.csv")

print(f"vio_full.csv covers t={vf['t'].min()-T0:.1f} to {vf['t'].max()-T0:.1f}s "
      f"({len(vf['t'])} states)")
print(f"vio_cruise.csv covers t={vc['t'].min()-T0:.1f} to {vc['t'].max()-T0:.1f}s "
      f"({len(vc['t'])} states)\n")

r1 = stats("static+climb (full run, static init)  ", vf, 0, 170)
r2 = stats("cruise (full run continuation)        ", vf, 170, 320)
r3 = stats("cruise (dyn-init run)                 ", vc, 160, 320)
r4 = stats("cruise+descent (dyn-init run)         ", vc, 160, 400)

dt_f = diverge_time(vf, 5)
dt_c = diverge_time(vc, 160)
print(f"\ndivergence (>100m, early-aligned): full run {dt_f and f'{dt_f:.0f}s'}, "
      f"cruise(dyn) run {dt_c and f'{dt_c:.0f}s'}")

# ---- plot ----
fig, axes = plt.subplots(1, 2, figsize=(14, 6.5))
fig.patch.set_facecolor("#fcfcfb")
for ax in axes:
    ax.set_facecolor("#fcfcfb")
    ax.grid(True, color="#e8e7e3", lw=0.8)
    ax.set_aspect("equal")

axes[0].plot(gxy[:, 0], gxy[:, 1], color="#52514e", lw=2, label="GPS ground truth (full flight)")
if r1:
    axes[0].plot(r1["al"][:, 0], r1["al"][:, 1], color="#2a78d6", lw=1.3, label="VIO full-run (static init) 4DOF-aligned, 0-170s")
if r2:
    axes[0].plot(r2["al"][:, 0], r2["al"][:, 1], color="#c22f2e", lw=1.3, label="VIO full-run continuation, 170-320s")
axes[0].scatter([gxy[0, 0]], [gxy[0, 1]], color="k", zorder=5, s=30, marker="^", label="takeoff")
axes[0].legend(fontsize=8, loc="best")
axes[0].set_title("Full run (static init @ t=0)", loc="left")

axes[1].plot(gxy[:, 0], gxy[:, 1], color="#52514e", lw=2, label="GPS ground truth (full flight)")
if r3:
    axes[1].plot(r3["al"][:, 0], r3["al"][:, 1], color="#2a78d6", lw=1.3, label="VIO dyn-init 4DOF-aligned, 160-320s")
if r4:
    axes[1].plot(r4["sc"][:, 0], r4["sc"][:, 1], color="#eb6834", lw=1.1, ls="--",
                 label=f"VIO dyn-init scale-corrected (k={r4['k']:.2f}), 160-400s")
axes[1].scatter([gxy[0, 0]], [gxy[0, 1]], color="k", zorder=5, s=30, marker="^", label="takeoff")
axes[1].legend(fontsize=8, loc="best")
axes[1].set_title("Cruise run (dyn init @ t=160s)", loc="left")

fig.suptitle("survey25 (Flight 2, 100m AGL survey) OpenVINS vs GPS -- pre-notch, 333Hz RAW_IMU stream")
fig.tight_layout()
out = f"{S}/vio_eval/vio_path_compare.png"
fig.savefig(out, dpi=115, facecolor=fig.get_facecolor())
print(out)
