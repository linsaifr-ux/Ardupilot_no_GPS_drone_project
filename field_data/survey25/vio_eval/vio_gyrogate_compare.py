#!/usr/bin/env python3
"""survey25 cruise: gyro-gated (skip camera feed when |omega|>0.20 rad/s,
falling back to IMU-only propagation through fast rotation) vs ungated
baseline, both dyn-init at t=200s near the first big turn (207-216s)."""
import csv, math, json
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

S = "/home/jetson/Ardupilot_no_GPS_drone_project/field_data/survey25"
T0 = json.load(open(f"{S}/meta.json"))["video_start_unix"]
rows = []
with open(f"{S}/telemetry.csv") as f:
    for r in csv.DictReader(f):
        try:
            rows.append((float(r["unix_time"]), float(r["lat"]), float(r["lon"])))
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
    yaw = math.atan2(np.sum(s[:, 0] * d[:, 1] - s[:, 1] * d[:, 0]), np.sum(s[:, 0] * d[:, 0] + s[:, 1] * d[:, 1]))
    c, si = math.cos(yaw), math.sin(yaw)
    R = np.array([[c, -si], [si, c]])
    k = 1.0
    if scale:
        sr = (R @ s.T).T
        k = np.sum(sr * d) / np.sum(sr * sr)
    t = md - k * (R @ ms)
    return k * (R @ src.T).T + t, k


def err_curve(v, lo=200, align_win=20):
    m = v["t"] >= T0 + lo
    tv = v["t"][m]
    xy = np.column_stack([v["px"][m], v["py"][m]])
    gt = np.column_stack([np.interp(tv, tg, gxy[:, i]) for i in range(2)])
    ma = tv <= tv[0] + align_win
    ms, md = xy[ma].mean(0), gt[ma].mean(0)
    s, d = xy[ma] - ms, gt[ma] - md
    yaw = math.atan2(np.sum(s[:, 0] * d[:, 1] - s[:, 1] * d[:, 0]), np.sum(s[:, 0] * d[:, 0] + s[:, 1] * d[:, 1]))
    c, si = math.cos(yaw), math.sin(yaw)
    R = np.array([[c, -si], [si, c]])
    est = (R @ (xy - ms).T).T + md
    e = np.linalg.norm(est - gt, axis=1)
    return tv - T0, e, est, gt


vc = np.genfromtxt(f"{S}/vio_eval/vio_cruise_v2.csv", delimiter=",", names=True)
vg = np.genfromtxt(f"{S}/vio_eval/vio_cruise_gyrogate.csv", delimiter=",", names=True)

fig, axes = plt.subplots(1, 2, figsize=(14, 6.5))
fig.patch.set_facecolor("#fcfcfb")
for ax in axes:
    ax.set_facecolor("#fcfcfb")
    ax.grid(True, color="#e8e7e3", lw=0.8)

t_c, e_c, est_c, gt_c = err_curve(vc)
t_g, e_g, est_g, gt_g = err_curve(vg)
ax = axes[0]
ax.semilogy(t_c, np.maximum(e_c, 0.1), color="#c22f2e", lw=1.4, label="ungated baseline")
ax.semilogy(t_g, np.maximum(e_g, 0.1), color="#2a78d6", lw=1.4, label="gyro-gated (>0.20 rad/s skips camera)")
ax.axvspan(207, 216, color="#eb6834", alpha=0.15, label="1st turn (207-216s)")
ax.set_xlabel("video time (s)")
ax.set_ylabel("2D position error vs GPS (m, log)")
ax.set_xlim(200, 330)
ax.legend(fontsize=9)
ax.set_title("Error growth: gating wins the 1st turn big, both diverge by ~t=280s (multi-turn survey)", loc="left", fontsize=10)

ax = axes[1]
m = (t_c <= t_c.min() + 60)
ax.plot(gt_c[m][:, 0], gt_c[m][:, 1], color="#52514e", lw=2.5, label="GPS ground truth")
ax.plot(est_c[m][:, 0], est_c[m][:, 1], color="#c22f2e", lw=1.3, label="ungated (scale collapses)")
mg = (t_g <= t_g.min() + 60)
ax.plot(est_g[mg][:, 0], est_g[mg][:, 1], color="#2a78d6", lw=1.3, ls="--", label="gyro-gated (scale ~0.95)")
ax.set_aspect("equal")
ax.legend(fontsize=9)
ax.set_title("First 60s (turn + exit leg): gated stays on the GPS path", loc="left", fontsize=10)

fig.suptitle("survey25: does gating vision during fast rotation (>0.20 rad/s) fix the turn-triggered divergence?")
fig.tight_layout()
out = f"{S}/vio_eval/vio_gyrogate_compare.png"
fig.savefig(out, dpi=115, facecolor=fig.get_facecolor())
print(out)

print(f"\n0-30s post-init: ungated rmse {np.sqrt((e_c[t_c<=t_c.min()+30]**2).mean()):.1f}m  "
      f"gated rmse {np.sqrt((e_g[t_g<=t_g.min()+30]**2).mean()):.1f}m")
print(f"0-60s post-init: ungated rmse {np.sqrt((e_c[t_c<=t_c.min()+60]**2).mean()):.1f}m  "
      f"gated rmse {np.sqrt((e_g[t_g<=t_g.min()+60]**2).mean()):.1f}m")
