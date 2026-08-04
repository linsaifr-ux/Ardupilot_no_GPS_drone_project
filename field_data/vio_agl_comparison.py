#!/usr/bin/env python3
"""Does higher AGL cause worse raw monocular VIO error?

Compares three same-rig flights at different cruise altitudes using the identical fixed-4DOF
(yaw+translation, no scale) alignment methodology already established for survey32/33
(field_data/survey32/vio_eval/real_anchor_eval_survey32.py's err_curve/report): align on the
first 20s of each flight's cruise window, then measure RMSE/max/mean error against GPS truth
over the whole window.

  survey30:  ~5m AGL cruise, t=145.6-265.6s (video-relative)
  survey31: ~10m AGL cruise, t=98.8-174.4s
  survey32: ~100m AGL cruise, t=116.0-197.2s (already-computed VIO trajectory, reused as-is)

No AnyLoc/fusion involved -- this is raw VIO only, isolating whether altitude itself (not the
fusion corrector) is the source of error.
"""
import csv, json, math
import numpy as np

ROOT = "/home/jetson/Ardupilot_no_GPS_drone_project/field_data"

FLIGHTS = [
    # name, agl_label, dir, vio_csv, window(lo,hi)
    ("survey30", "~5m AGL",   "survey30", "vio_eval/vio_full_survey30.csv", (145.6, 265.6)),
    ("survey31", "~10m AGL",  "survey31", "vio_eval/vio_full_survey31.csv", (98.8, 174.4)),
    ("survey32", "~100m AGL", "survey32", "vio_eval/vio_full_survey32.csv", (116.0, 197.2)),
]

ALIGN_WIN = 20.0


def load_gps_xy(sdir):
    rows = []
    with open(f"{ROOT}/{sdir}/telemetry.csv") as f:
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
    return tel[:, 0], gxy


def video_t0(sdir):
    return json.load(open(f"{ROOT}/{sdir}/meta.json"))["video_start_unix"]


def err_curve(vio_csv, tg, gxy, t0, lo, hi, align_win=ALIGN_WIN):
    v = np.genfromtxt(vio_csv, delimiter=",", names=True)
    m = (v["t"] >= t0 + lo) & (v["t"] <= t0 + hi)
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
    # ground-truth path length over the window, for normalization
    seg = np.linalg.norm(np.diff(gt, axis=0), axis=1)
    path_len = float(seg.sum())
    return tv - t0, e, path_len, est, gt


def main():
    results = []
    for name, agl_label, sdir, vio_rel, (lo, hi) in FLIGHTS:
        t0 = video_t0(sdir)
        tg, gxy = load_gps_xy(sdir)
        vio_csv = f"{ROOT}/{sdir}/{vio_rel}"
        t, e, path_len, est, gt = err_curve(vio_csv, tg, gxy, t0, lo, hi)
        dur = hi - lo
        rmse = float(np.sqrt((e ** 2).mean()))
        mean_e = float(e.mean())
        max_e = float(e.max())
        speed = path_len / dur
        pct_of_path = 100.0 * max_e / path_len if path_len > 0 else float("nan")
        results.append(dict(name=name, agl=agl_label, window=[lo, hi], dur_s=dur, n=len(t),
                             path_len_m=path_len, mean_speed_mps=speed,
                             rmse_m=rmse, mean_err_m=mean_e, max_err_m=max_e,
                             max_err_pct_path=pct_of_path))
        print(f"{name:10s} {agl_label:9s}  window {lo:6.1f}-{hi:6.1f}s (dur {dur:5.1f}s, n={len(t):4d})  "
              f"path_len={path_len:6.1f}m  speed={speed:4.2f}m/s   "
              f"RMSE={rmse:7.1f}m  mean={mean_e:7.1f}m  max={max_e:7.1f}m  "
              f"(max={pct_of_path:5.1f}% of path)")

    with open(f"{ROOT}/vio_agl_comparison_result.json", "w") as f:
        json.dump(results, f, indent=2)
    print(f"\nwritten: {ROOT}/vio_agl_comparison_result.json")


if __name__ == "__main__":
    main()
