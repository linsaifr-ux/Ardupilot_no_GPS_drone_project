#!/usr/bin/env python3
"""survey25: scale-vs-shape decomposition of the 4 method outputs (Frank's question:
'survey17 was only scale error, the path shape was identical -- why not just fix the scale?').

For each method, over a sliding window: fit a full similarity (yaw+scale+translation) to GPS
and report (a) the best-fit scale trajectory, (b) the residual AFTER scale correction
('shape-rmse' -- how good the path could ever get with a perfect scale oracle). The
shape-rmse curve of the best method is the CEILING any output-space scale fix can reach;
scale_corrector.py then tries to reach it with GPS-free references only.

Windows where GPS itself is nearly stationary (descent hover, <15 m of GT motion) are
skipped -- a similarity fit to a stationary target is degenerate (k->0 hides divergence).
"""
import csv, math, json
import numpy as np

V = "."
rows = list(csv.DictReader(open(f"{V}/method_comparison_timeseries.csv")))
t = np.array([float(r["t_rel_s"]) for r in rows])
gps = np.column_stack([[float(r["gps_x"]) for r in rows], [float(r["gps_y"]) for r in rows]])
METHODS = ["ungated", "gyrogate", "aglprior", "combined"]


def series(k):
    return np.column_stack([[float(r[k + "_x"]) for r in rows], [float(r[k + "_y"]) for r in rows]])


def sim_fit(est, gt):
    """similarity (yaw+scale+trans) est->gt; returns scale, residual rmse, per-point residual"""
    ms, md = est.mean(0), gt.mean(0)
    s, d = est - ms, gt - md
    yaw = math.atan2(np.sum(s[:, 0] * d[:, 1] - s[:, 1] * d[:, 0]), np.sum(s[:, 0] * d[:, 0] + s[:, 1] * d[:, 1]))
    c, si = math.cos(yaw), math.sin(yaw)
    R = np.array([[c, -si], [si, c]])
    sr = (R @ s.T).T
    denom = np.sum(sr * sr)
    k = np.sum(sr * d) / denom if denom > 0 else 1.0
    resid = k * sr + md - gt
    e = np.linalg.norm(resid, axis=1)
    return k, float(np.sqrt((e ** 2).mean())), e


def gt_motion(gt):
    return float(np.linalg.norm(np.diff(gt, axis=0), axis=1).sum())


WIN, STEP = 30, 10
print(f"sliding {WIN}s window, {STEP}s step; GPS-stationary windows (<15 m GT motion) skipped\n")
results = {m: [] for m in METHODS}
for m in METHODS:
    est = series(m)
    for lo in np.arange(t[0], t[-1] - WIN, STEP):
        w = (t >= lo) & (t < lo + WIN)
        if w.sum() < 10 or gt_motion(gps[w]) < 15:
            continue
        k, r, _ = sim_fit(est[w], gps[w])
        results[m].append((float(lo), k, r))

print(f"{'window':>10s} | " + " | ".join(f"{m:>22s}" for m in METHODS))
print(f"{'':>10s} | " + " | ".join(f"{'scale / shape-rmse':>22s}" for _ in METHODS))
los = sorted({lo for m in METHODS for lo, _, _ in results[m]})
for lo in los:
    cells = []
    for m in METHODS:
        hit = [x for x in results[m] if x[0] == lo]
        cells.append(f"{hit[0][1]:6.2f} / {hit[0][2]:8.1f}m" if hit else f"{'--':>22s}")
    print(f"{lo:6.0f}-{lo+WIN:.0f}s | " + " | ".join(f"{c:>22s}" for c in cells))

# ceiling: per-method, longest span where shape-rmse stays under thresholds
print("\n=== scale-oracle ceiling (shape holds = output-space scale fix can work) ===")
for m in METHODS:
    good30 = [lo for lo, k, r in results[m] if r < 30]
    good60 = [lo for lo, k, r in results[m] if r < 60]
    horizon30 = max(good30) + WIN if good30 else None
    # find first break
    first_bad = next((lo for lo, k, r in sorted(results[m]) if r > 60), None)
    print(f"{m:10s}: shape<30m windows end at t={horizon30}, first shape>60m window starts at t={first_bad}")

summary = {m: [(lo, k, r) for lo, k, r in sorted(results[m])] for m in METHODS}
with open(f"{V}/scale_fix_analysis.json", "w") as f:
    json.dump(summary, f, indent=1)
print("\nwrote scale_fix_analysis.json")
