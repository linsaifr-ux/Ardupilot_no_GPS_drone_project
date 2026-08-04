#!/usr/bin/env python3
"""Full-pipeline (VIO + VPR anchors + fusion corrector + slew) sweep over VPR query PERIOD, for
the NGPS-style (SuperPoint+LightGlue+homography-vs-mosaic) VPR method built and evaluated
earlier this session (see ngps_style_eval_survey32.py, ngps_style_eval_result_p{period}.json for
period in {0.5,1.0,2.0,3.0,4.0}).

Reuses the EXACT same corrector (foundloc_corrector_survey32.run_corrector, anchor_win=20 --
this project's official offline config, see real_anchor_eval_survey32.py) and the same slew
limiter (control/vpe_slew.VpeSlewLimiter) and the same fixed-4DOF cruise-window evaluation
methodology (err_curve/report) already established for the AnyLoc real-anchor pipeline -- the
only thing that changes per run is which anchor stream feeds the corrector, so any RMSE
difference across periods is attributable to VPR query frequency, not a methodology change.

Only successful NGPS-style localizations (status=="ok") become anchors -- failed queries (no
match / too few RANSAC inliers) contribute nothing, exactly as they would in a real deployment
(no anchor update that tick).

Run with system python (has numpy/scipy/sklearn already, no kornia/lightglue needed for this
stage -- that GPU-side matching already happened in ngps_style_eval_survey32.py):
    python3 field_data/survey32/vio_eval/ngps_freq_sweep_corrector.py
"""
import csv, json, math, sys, os
import numpy as np

ROOT = "/home/jetson/Ardupilot_no_GPS_drone_project"
sys.path.insert(0, f"{ROOT}/field_data/survey32/vio_eval")
sys.path.insert(0, f"{ROOT}/control")
from foundloc_corrector_survey32 import build_argparser, run_corrector, lat0, lon0, latm, lonm, T0, S, V
from vpe_slew import VpeSlewLimiter

TRAJ = f"{V}/vio_full_survey32.csv"
ANCHOR_WIN = 20.0  # this project's official offline corrector config (real_anchor_eval_survey32.py)
PERIODS = [0.5, 1.0, 2.0, 3.0, 4.0]
CRUISE_WINDOW = (116, 197)
ALIGN_WIN = 20


def load_ngps_anchors(period):
    path = f"{V}/ngps_style_eval_result_p{period}.json"
    d = json.load(open(path))
    ok = [r for r in d["results"] if r["status"] == "ok"]
    ok.sort(key=lambda r: r["t_rel"])
    anchor_t = np.array([T0 + r["t_rel"] for r in ok])
    if "est_lat" not in ok[0] or ok[0]["est_lat"] is None:
        raise RuntimeError(f"{path}: results are missing est_lat/est_lon -- re-run "
                            f"ngps_style_eval_survey32.py (patched to record them) before this script")
    anchor_xy = np.column_stack([
        (np.array([r["est_lon"] for r in ok]) - lon0) * lonm,
        (np.array([r["est_lat"] for r in ok]) - lat0) * latm,
    ])
    return anchor_t, anchor_xy, len(d["results"]), len(ok)


def apply_slew(in_csv, out_csv):
    v = np.genfromtxt(in_csv, delimiter=",", names=True)
    names = list(v.dtype.names)
    tv = v["t"]
    limiter = VpeSlewLimiter()
    pub = np.zeros((len(tv), 2))
    for k in range(len(tv)):
        pub[k] = limiter.update(v["px"][k], v["py"][k], tv[k])
    with open(out_csv, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(names)
        for k in range(len(tv)):
            row = [f"{v[nm][k]:.6f}" for nm in names]
            row[names.index("px")] = f"{pub[k, 0]:.4f}"
            row[names.index("py")] = f"{pub[k, 1]:.4f}"
            w.writerow(row)


def _load_gps_xy():
    rows = []
    with open(f"{S}/telemetry.csv") as f:
        for r in csv.DictReader(f):
            try:
                rows.append((float(r["unix_time"]), float(r["lat"]), float(r["lon"])))
            except ValueError:
                pass
    tel = np.array(rows)
    return tel[:, 0], np.column_stack([(tel[:, 2] - lon0) * lonm, (tel[:, 1] - lat0) * latm])


TG, GXY = _load_gps_xy()


def err_curve(fn, lo, align_win=ALIGN_WIN):
    v = np.genfromtxt(fn, delimiter=",", names=True)
    m = v["t"] >= T0 + lo
    tv = v["t"][m]
    xy = np.column_stack([v["px"][m], v["py"][m]])
    gt = np.column_stack([np.interp(tv, TG, GXY[:, i]) for i in range(2)])
    ma = tv <= tv[0] + align_win
    ms, md = xy[ma].mean(0), gt[ma].mean(0)
    s, d = xy[ma] - ms, gt[ma] - md
    yaw = math.atan2(np.sum(s[:, 0] * d[:, 1] - s[:, 1] * d[:, 0]), np.sum(s[:, 0] * d[:, 0] + s[:, 1] * d[:, 1]))
    c, si = math.cos(yaw), math.sin(yaw)
    R = np.array([[c, -si], [si, c]])
    est = (R @ (xy - ms).T).T + md
    e = np.linalg.norm(est - gt, axis=1)
    return tv - T0, e


def report(fn, windows):
    row = {}
    for wlo, whi in windows:
        t, e = err_curve(fn, wlo)
        m = t < whi
        if m.any():
            row[f"{wlo}-{whi}"] = dict(rmse=float(np.sqrt((e[m] ** 2).mean())), max=float(e[m].max()),
                                        n=int(m.sum()))
    return row


def main():
    print("=" * 100)
    print(f"Full-pipeline VPR-frequency sweep (NGPS-style anchors -> corrector anchor_win={ANCHOR_WIN} -> slew)")
    print("=" * 100)
    rows = []
    for period in PERIODS:
        path = f"{V}/ngps_style_eval_result_p{period}.json"
        if not os.path.exists(path):
            print(f"period {period}: {path} not found, skipping")
            continue
        anchor_t, anchor_xy, n_total, n_ok = load_ngps_anchors(period)
        coverage = 100.0 * n_ok / n_total if n_total else 0.0
        ap = build_argparser()
        unslewed = f"{V}/vio_cruise_ngps_p{period}_unslewed.csv"
        fused = f"{V}/vio_cruise_ngps_p{period}_final.csv"
        args = ap.parse_args([TRAJ, unslewed])
        args.anchor_win = ANCHOR_WIN
        run_corrector(TRAJ, anchor_t, anchor_xy, args, is_fp=None, out_path=unslewed, verbose=False)
        apply_slew(unslewed, fused)
        cruise = report(fused, [CRUISE_WINDOW])[f"{CRUISE_WINDOW[0]}-{CRUISE_WINDOW[1]}"]
        rows.append(dict(period=period, n_attempted=n_total, n_ok=n_ok, coverage_pct=coverage,
                          cruise_rmse_m=cruise["rmse"], cruise_max_m=cruise["max"], n_eval=cruise["n"]))
        print(f"period={period:4.1f}s  attempted={n_total:4d}  ok={n_ok:4d} ({coverage:5.1f}%)  "
              f"cruise RMSE={cruise['rmse']:7.2f}m  max={cruise['max']:7.2f}m")

    with open(f"{V}/ngps_freq_sweep_result.json", "w") as f:
        json.dump(dict(config=dict(anchor_win=ANCHOR_WIN, align_win=ALIGN_WIN,
                                    cruise_window=list(CRUISE_WINDOW)), rows=rows), f, indent=2)
    print(f"\nwritten: {V}/ngps_freq_sweep_result.json")

    if rows:
        best = min(rows, key=lambda r: r["cruise_rmse_m"])
        print(f"\nBEST cruise RMSE: period={best['period']}s -> {best['cruise_rmse_m']:.2f}m "
              f"(coverage {best['coverage_pct']:.1f}%)")


if __name__ == "__main__":
    main()
