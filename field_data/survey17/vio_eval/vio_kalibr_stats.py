#!/usr/bin/env python3
"""survey17: segment stats — uncalibrated baseline vs Kalibr-calibrated runs.

Same methodology as vio_path_compare.py / README tables: per-segment 4-DOF
(yaw+translation) alignment, optional best-fit scale, alt vs baro AGL
init-anchored. Divergence time = first crossing of a 2D error threshold
under early-window alignment.
"""
import csv, math
import numpy as np

S = "/home/jetson/Ardupilot_no_GPS_drone_project/field_data/survey17"
T0 = 1784630291.0294251

rows = []
with open(f"{S}/telemetry.csv") as f:
    for r in csv.DictReader(f):
        try:
            rows.append((float(r["unix_time"]), float(r["lat"]), float(r["lon"]),
                         float(r["alt_agl"])))
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
    al, _ = yaw_align(xy, gt)
    sc, k = yaw_align(xy, gt, scale=True)
    e = np.linalg.norm(al - gt, axis=1)
    es = np.linalg.norm(sc - gt, axis=1)
    ea = np.abs(alt - agl)
    path = np.linalg.norm(np.diff(gt, axis=0), axis=1).sum()
    print(f"{name}  [{lo}-{hi}s, {path:.0f} m]")
    print(f"  4DOF ATE2D rmse {np.sqrt((e**2).mean()):6.1f}  max {e.max():6.1f} m"
          f"   | best-fit scale {k:.3f}  scale-corr rmse {np.sqrt((es**2).mean()):6.1f} m")
    print(f"  alt vs baro AGL (init-anchored): rmse {np.sqrt((ea**2).mean()):5.1f}"
          f"  max {ea.max():5.1f} m")


def diverge_time(v, lo, align_win=60.0, thresh=100.0):
    m = v["t"] >= T0 + lo
    tv = v["t"][m]
    xy = np.column_stack([v["px"][m], v["py"][m]])
    gt = np.column_stack([np.interp(tv, tg, gxy[:, i]) for i in range(2)])
    ma = tv <= tv[0] + align_win
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


for tag, f_full, f_cruise in [
        ("BASELINE (uncalibrated)", "vio_full.csv", "vio_cruise.csv"),
        ("KALIBR-CALIBRATED", "vio_full_kalibr.csv", "vio_cruise_kalibr.csv")]:
    print(f"=== {tag} ===")
    vf = load(f"{S}/vio_eval/{f_full}")
    vc = load(f"{S}/vio_eval/{f_cruise}")
    stats("early  ", vf, 225, 430)
    stats("cruise (baseline clip 448-775)", vc, 448, 775)
    stats("cruise (to descent  448-850)", vc, 448, 850)
    dt_f = diverge_time(vf, 225)
    dt_c = diverge_time(vc, 448)
    print(f"  divergence (>100 m, early-aligned): full run {dt_f and f'{dt_f:.0f} s'},"
          f" cruise run {dt_c and f'{dt_c:.0f} s'}")
    print()
