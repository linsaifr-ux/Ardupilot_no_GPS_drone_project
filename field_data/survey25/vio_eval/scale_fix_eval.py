#!/usr/bin/env python3
"""Evaluate scale-corrected trajectories with the exact same fixed-transform methodology as
method_comparison.py (yaw+translation fixed from t=200-220s, propagated forward, scale=1 rigid —
the corrector's own scale stays in the trajectory, which is the whole point)."""
import csv, math, json, sys
import numpy as np

S = "/home/jetson/Ardupilot_no_GPS_drone_project/field_data/survey25"
V = f"{S}/vio_eval"
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


def report(name, fn, lo=200):
    v = np.genfromtxt(fn, delimiter=",", names=True)
    t, e, est, gt = err_curve(v, lo=lo)
    line = f"{name:28s}:"
    for wlo, whi in [(200, 230), (200, 260), (200, 320)]:
        m = (t >= wlo) & (t < whi)
        if m.any():
            line += f"  [{wlo}-{whi}] rmse={np.sqrt((e[m]**2).mean()):8.1f} max={e[m].max():8.1f}"
    over = t[e > 100]
    horizon = over[0] if len(over) else t[-1]
    line += f"  | horizon(>100m)={horizon:.0f}s"
    print(line)
    return t, e, est, gt


print("=== baselines (uncorrected) ===")
for name, fn in [("ungated", "vio_cruise_v2.csv"), ("gyrogate", "vio_cruise_gyrogate.csv"),
                 ("aglprior", "vio_cruise_aglprior_fixed.csv"), ("combined", "vio_cruise_combined.csv")]:
    report(name, fn)

print("\n=== scale-corrected (causal, GPS-free refs) ===")
for name, fn in [("aglprior+baro", "vio_cruise_aglprior_scalefix_baro.csv"),
                 ("aglprior+anchor(AnyLocSim)", "vio_cruise_aglprior_scalefix_anchor.csv"),
                 ("aglprior+both", "vio_cruise_aglprior_scalefix_both.csv")]:
    report(name, fn)
