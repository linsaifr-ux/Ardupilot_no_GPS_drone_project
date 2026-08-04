#!/usr/bin/env python3
"""Final evaluation of the FoundLoc-style integration (foundloc_corrector.py) against every
prior result, same fixed-transform methodology as method_comparison.py / scale_fix_eval.py."""
import csv, math, json
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


def err_curve(fn, lo=170, align_win=20):
    v = np.genfromtxt(fn, delimiter=",", names=True)
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


def report(name, fn):
    t, e, est, gt = err_curve(fn)
    line = f"{name:34s}:"
    for wlo, whi in [(200, 230), (200, 260), (200, 320), (170, 472)]:
        m = (t >= wlo) & (t < whi)
        if m.any():
            line += f"  [{wlo}-{whi}] {np.sqrt((e[m]**2).mean()):8.1f}"
    line += f"  max={e.max():8.0f}"
    print(line)
    return t, e, est, gt


print("=== full-flight results (170-472s) ===")
report("raw VIO (aglprior, static init)", f"{V}/vio_full_aglprior.csv")
report("scalefix (clean anchor proxy)", f"{V}/vio_full_aglprior_scalefix.csv")
report("FoundLoc-NF (noisy, no DBSCAN filter)", f"{V}/vio_full_foundloc_nf.csv")
report("FoundLoc-style (DBSCAN-filtered)", f"{V}/vio_full_foundloc.csv")

print("\n=== survey25 standard cruise windows, all methods to date ===")
report("ungated", f"{V}/vio_cruise_v2.csv")
report("gyrogate", f"{V}/vio_cruise_gyrogate.csv")
report("aglprior (cruise-only run)", f"{V}/vio_cruise_aglprior_fixed.csv")
report("combined (gate+prior)", f"{V}/vio_cruise_combined.csv")
report("FoundLoc-style (full-flight)", f"{V}/vio_full_foundloc.csv")

# ---- write foundloc trace into method_comparison_timeseries.csv on the same 1Hz grid ----
t, e, est, gt = err_curve(f"{V}/vio_full_foundloc.csv", lo=170)
lines = list(csv.reader(open(f"{V}/method_comparison_timeseries.csv")))
hdr, data = lines[0], lines[1:]
grid = np.array([float(r[0]) for r in data])
fx = np.interp(grid, t + T0 - T0, est[:, 0], left=np.nan, right=np.nan)
fy = np.interp(grid, t + T0 - T0, est[:, 1], left=np.nan, right=np.nan)
hdr = hdr + ["foundloc_x", "foundloc_y"]
for i, r in enumerate(data):
    r.append("" if np.isnan(fx[i]) else f"{fx[i]:.3f}")
    r.append("" if np.isnan(fy[i]) else f"{fy[i]:.3f}")
with open(f"{V}/method_comparison_timeseries.csv", "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(hdr)
    w.writerows(data)
print(f"\nwrote foundloc_x/y columns into method_comparison_timeseries.csv ({len(data)} rows)")

# ---- update summary json ----
sm = json.load(open(f"{V}/method_comparison_summary.json"))
for wlo, whi in [(200, 230), (200, 260), (200, 320)]:
    wk = f"{wlo}-{whi}s"
    m = (t >= wlo) & (t < whi)
    if m.any() and wk in sm["windows"]:
        sm["windows"][wk]["foundloc"] = {
            "rmse_m": float(np.sqrt((e[m] ** 2).mean())),
            "max_m": float(e[m].max()),
            "scale": None,
        }
sm["foundloc_integration"] = (
    "2026-07-24 late evening: integrated 2 pieces from FoundLoc (He et al., CMU AirLab, arXiv:2310.16299 -- "
    "same VPR backbone, AnyLoc-DINO, as this project) into the output-space corrector: (1) DBSCAN "
    "false-positive filtering of anchor fixes over a trailing residual window (their III-E-3, adapted from "
    "spatial multi-candidate clustering to temporal single-candidate clustering since we simulate one match "
    "per query, not top-N); (2) degeneracy-aware frame lock using gyro-integrated heading as a cross-check "
    "when the lock window is near-collinear (their III-B gravity-constraint term's role, adapted from 3D "
    "gravity-consistency to our SE(2)-only alignment via gyro heading, the analogous IMU-derived redundant "
    "signal). Tested honestly: unlike scale_corrector.py's clean 50m-quantized anchor proxy (unrealistically "
    "generous), this run injects a realistic false-positive rate (20% of queries, error 200-900m, grounded "
    "in this project's own real-footage AnyLoc measurements, see memory real_video_constrained_search_failure) "
    "and shows DBSCAN filtering earns its keep: parameter sweep (eps 60-800) found eps=300/min_samples=2/"
    "window=8 rejects ~40% of true false positives with ZERO good anchors wrongly rejected, improving rmse "
    "over unfiltered noisy anchors in 4/5 random seeds (up to 30% reduction), roughly a wash in the 5th. "
    "Full-flight (170-472s) result with this realistic noise + filtering: see 'windows' above is on the "
    "200-320s scale; full-flight rmse reported separately as it doesn't fit the 3-window table (clean-anchor "
    "scalefix's 76m was against a best-case anchor with zero false positives -- the honest, noisy-anchor "
    "FoundLoc-style number is higher, reflecting a more realistic AnyLoc)."
)
json.dump(sm, open(f"{V}/method_comparison_summary.json", "w"), indent=2)
print("updated method_comparison_summary.json")
