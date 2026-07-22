#!/usr/bin/env python3
"""survey17: VIO altitude error vs two references:
  A) GPS-frame relative alt  (alt_amsl - alt_amsl[0])
  B) baro AGL                (alt_agl, EKF baro-primary rel_alt)
XY error is always vs GPS ENU (4-DOF yaw+translation alignment, same as
compare_vio_gps.py). Vertical error reported with mean-offset alignment and
with pad-anchored (init-window) alignment.
"""
import csv, math, sys
import numpy as np

SURVEY = "/home/jetson/Ardupilot_no_GPS_drone_project/field_data/survey17"
VIDEO_T0 = 1784630291.0294251

def load_tel():
    rows = []
    with open(f"{SURVEY}/telemetry.csv") as f:
        for r in csv.DictReader(f):
            try:
                rows.append((float(r["unix_time"]), float(r["lat"]), float(r["lon"]),
                             float(r["alt_amsl"]), float(r["alt_agl"])))
            except ValueError:
                pass
    return np.array(rows)

def enu_xy(tel):
    lat0, lon0 = tel[0, 1], tel[0, 2]
    latm = 111132.954 - 559.822 * math.cos(2 * math.radians(lat0))
    lonm = 111412.84 * math.cos(math.radians(lat0)) - 93.5 * math.cos(3 * math.radians(lat0))
    return np.column_stack([(tel[:, 2] - lon0) * lonm, (tel[:, 1] - lat0) * latm])

def yaw_align_xy(src, dst):
    ms, md = src.mean(0), dst.mean(0)
    s, d = src - ms, dst - md
    yaw = math.atan2(np.sum(s[:, 0] * d[:, 1] - s[:, 1] * d[:, 0]),
                     np.sum(s[:, 0] * d[:, 0] + s[:, 1] * d[:, 1]))
    c, si = math.cos(yaw), math.sin(yaw)
    R = np.array([[c, -si], [si, c]])
    t = md - R @ ms
    return R, t, yaw

def stats(e):
    return (f"rmse {np.sqrt((e**2).mean()):6.2f}  mean {np.abs(e).mean():6.2f}  "
            f"median {np.median(np.abs(e)):6.2f}  p95 {np.percentile(np.abs(e),95):6.2f}  "
            f"max {np.abs(e).max():6.2f} m")

def eval_segment(name, vio_csv, t_lo, t_hi, tel, xy, anchor_s=20.0):
    v = np.genfromtxt(vio_csv, delimiter=",", names=True)
    tv = v["t"]
    m = (tv >= VIDEO_T0 + t_lo) & (tv <= VIDEO_T0 + t_hi) & (tv >= tel[0,0]) & (tv <= tel[-1,0])
    tv = tv[m]
    pv = np.column_stack([v["px"][m], v["py"][m], v["pz"][m]])
    tg = tel[:, 0]
    gxy = np.column_stack([np.interp(tv, tg, xy[:, i]) for i in range(2)])
    ref_gps = np.interp(tv, tg, tel[:, 3] - tel[0, 3])   # GPS-frame amsl, relative
    ref_baro = np.interp(tv, tg, tel[:, 4])              # baro AGL (rel_alt)

    R, t, yaw = yaw_align_xy(pv[:, :2], gxy)
    exy = np.linalg.norm((R @ pv[:, :2].T).T + t - gxy, axis=1)

    print(f"\n=== {name}  (video t {t_lo:.0f}-{t_hi:.0f}s, {len(tv)} samples, "
          f"span {tv[-1]-tv[0]:.0f}s) ===")
    print(f"  XY vs GPS (4DOF): {stats(exy)}")

    for rname, ref in (("GPS amsl-rel", ref_gps), ("baro AGL    ", ref_baro)):
        zerr_mean = (pv[:, 2] - pv[:, 2].mean()) - (ref - ref.mean())
        ma = tv <= tv[0] + anchor_s
        zerr_anchor = (pv[:, 2] - pv[ma, 2].mean()) - (ref - ref[ma].mean())
        print(f"  alt vs {rname} [mean-aligned]  : {stats(zerr_mean)}")
        print(f"  alt vs {rname} [init-anchored] : {stats(zerr_anchor)}")
    return tv, pv, ref_gps, ref_baro

tel = load_tel()
xy = enu_xy(tel)

d = tel[:, 3] - tel[0, 3] - tel[:, 4] + tel[0, 4]
print(f"reference disagreement (amsl-rel minus baro AGL, zeroed at start): "
      f"min {d.min():.2f}  max {d.max():.2f} m  -> the two references differ by at most this")

segs = [
    ("early flight (vio_full)",  f"{SURVEY}/vio_eval/vio_full.csv",   225, 430),
    ("cruise (vio_cruise)",      f"{SURVEY}/vio_eval/vio_cruise.csv", 448, 775),
]
out = []
for name, path, lo, hi in segs:
    out.append((name,) + eval_segment(name, path, lo, hi, tel, xy))

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
fig, axs = plt.subplots(1, 2, figsize=(16, 6))
for ax, (name, tv, pv, rg, rb) in zip(axs, out):
    ts = tv - VIDEO_T0
    ma = ts <= ts[0] + 20.0
    ax.plot(ts, rb, "k-", lw=1.2, label="baro AGL (rel_alt)")
    ax.plot(ts, rg + (rb[ma].mean() - rg[ma].mean()), "b--", lw=0.9,
            label="GPS amsl-rel (shifted)")
    ax.plot(ts, pv[:, 2] - pv[ma, 2].mean() + rb[ma].mean(), "r-", lw=1,
            label="VIO alt (init-anchored)")
    ax.set_title(name); ax.set_xlabel("video time (s)"); ax.set_ylabel("alt (m)")
    ax.legend(); ax.grid(alpha=0.3)
fig.tight_layout()
fig.savefig(f"{SURVEY}/vio_eval/vio_alt_baro_compare.png", dpi=110)
print(f"\nplot saved: {SURVEY}/vio_eval/vio_alt_baro_compare.png")
