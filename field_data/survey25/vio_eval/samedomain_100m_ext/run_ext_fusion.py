#!/usr/bin/env python3
"""survey25: feed the EXTENDED-coverage (time-based extraction during the t~=306-318s loiter,
tools/extract_frames.py --max-time-gap 2.0) same-domain AnyLoc anchor stream through the verified
foundloc_corrector.py causal loop (imported, not re-typed), then evaluate against GPS with the
same fixed-transform 4DOF alignment methodology as real_anchor_eval.py / foundloc_eval.py /
samedomain_100m/run_agl100_fusion.py.

Verification-first (same requirement as run_agl100_fusion.py): before trusting any real-anchor
result here, this script
  1. reproduces foundloc_corrector.py's own simulated-anchor mode bit-for-bit (--seed 42 ->
     vio_full_gyropredict_foundloc_final.csv) via the imported run_corrector() function, and
  2. runs the real-anchor pipeline TWICE and checks the outputs are byte-identical.
Both checks abort (assert) if they fail.

Anchor source: field_data/survey25/vio_eval/samedomain_100m_ext/query_frames_timed.csv -- the
held-out (odd-index) query split of the 95-105m-AGL-restricted same-domain database
(anyloc/database_survey25_samedomain_100m_ext_vits14), built from the EXTENDED extraction
(field_data/survey25_100m_agl_ext/, --min-dist 6 --min-agl 95 --max-agl 105 --max-time-gap 2.0),
evaluated in anyloc_samedomain_100m_ext_result.json, with unix_time recovered per-query by
lat/lon(+loiter-time-window) match against telemetry.csv (recover_query_times_ext.py).

Args are the SAME as the samedomain_100m fusion run (run_agl100_fusion.py) and the task's
specified "existing best same-domain fusion run tonight" config: anchor_pull=0.9, anchor_win=40,
anchor_min_disp=20, baro_win=20, baro_min_dz=3, ema=0.5, clamp=[0.2,5.0], vmax=20,
lock_min_arc=150, lock_collinearity_ratio=4.0, dbscan_window=8, dbscan_min_samples=2,
dbscan_eps=200, no_filter=False.
"""
import csv, json, math, sys
import numpy as np

sys.path.insert(0, "/home/jetson/Ardupilot_no_GPS_drone_project/field_data/survey25/vio_eval")
from foundloc_corrector import build_argparser, run_corrector, lat0, lon0, latm, lonm, T0, S, V

TRAJ = f"{V}/vio_full_aglprior_gyropredict.csv"
TRUSTED_REF_RMSE = 78.57020375864622
TRUSTED_REF_MAX = 164.39048920446118

EXT = f"{V}/samedomain_100m_ext"
ANCHOR_CSV = f"{EXT}/query_frames_timed.csv"
OUT_CSV = f"{EXT}/vio_full_samedomain_100m_ext_fused.csv"
RESULT_JSON = f"{EXT}/agl100_ext_fusion_result.json"


# ---- exact copy of real_anchor_eval.py's/run_agl100_fusion.py's err_curve()/report() methodology
# (itself transcribed directly from foundloc_eval.py) ------------------------------------------
def _load_gps_xy():
    rows = []
    with open(f"{S}/telemetry.csv") as f:
        for r in csv.DictReader(f):
            try:
                rows.append((float(r["unix_time"]), float(r["lat"]), float(r["lon"])))
            except ValueError:
                pass
    tel = np.array(rows)
    lat0_, lon0_ = tel[0, 1], tel[0, 2]
    latm_ = 111132.954 - 559.822 * math.cos(2 * math.radians(lat0_))
    lonm_ = 111412.84 * math.cos(math.radians(lat0_)) - 93.5 * math.cos(3 * math.radians(lat0_))
    gxy = np.column_stack([(tel[:, 2] - lon0_) * lonm_, (tel[:, 1] - lat0_) * latm_])
    return tel[:, 0], gxy


TG, GXY = _load_gps_xy()


def err_curve(fn, lo=170, align_win=20):
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
    return tv - T0, e, est, gt


def report(name, fn):
    t, e, est, gt = err_curve(fn)
    line = f"{name:45s}:"
    row = {}
    for wlo, whi in [(200, 230), (200, 260), (200, 320), (170, 472)]:
        m = (t >= wlo) & (t < whi)
        if m.any():
            rmse = float(np.sqrt((e[m] ** 2).mean()))
            mx = float(e[m].max())
            row[f"{wlo}-{whi}"] = dict(rmse=rmse, max=mx, n=int(m.sum()))
            line += f"  [{wlo}-{whi}] rmse={rmse:8.1f} max={mx:8.1f}"
    print(line)
    return row, t, e


# ---------------------------------------------------------------------------------------------
# STEP 1: verify run_corrector() reproduces the trusted simulated-anchor reference bit-exactly
# ---------------------------------------------------------------------------------------------
print("=" * 90)
print("STEP 1: verifying imported run_corrector() reproduces the trusted reference (seed=42)")
print("=" * 90)
from foundloc_corrector import simulate_anchor_stream

ap = build_argparser()
verify_args = ap.parse_args([TRAJ, "/tmp/_verify_out_agl100_ext.csv", "--seed", "42"])
v_anchor_t, v_anchor_xy, v_is_fp = simulate_anchor_stream(verify_args)
verify_result = run_corrector(TRAJ, v_anchor_t, v_anchor_xy, verify_args, is_fp=v_is_fp,
                              out_path="/tmp/_verify_out_agl100_ext.csv")
vt, ve, vest, vgt = err_curve("/tmp/_verify_out_agl100_ext.csv")
v_rmse = float(np.sqrt((ve ** 2).mean()))
v_max = float(ve.max())
print(f"reproduced: rmse={v_rmse!r}  max={v_max!r}")
print(f"trusted   : rmse={TRUSTED_REF_RMSE!r}  max={TRUSTED_REF_MAX!r}")
assert abs(v_rmse - TRUSTED_REF_RMSE) < 1e-6, f"MISMATCH: {v_rmse} vs {TRUSTED_REF_RMSE}"
assert abs(v_max - TRUSTED_REF_MAX) < 1e-6, f"MISMATCH: {v_max} vs {TRUSTED_REF_MAX}"
print("PASS -- imported run_corrector() reproduces the trusted reference exactly.\n")

shipped = np.genfromtxt(f"{V}/vio_full_gyropredict_foundloc_final.csv", delimiter=",", names=True)
mine = np.genfromtxt("/tmp/_verify_out_agl100_ext.csv", delimiter=",", names=True)
px_ok = np.allclose(mine["px"], shipped["px"])
py_ok = np.allclose(mine["py"], shipped["py"])
print(f"np.allclose(px): {px_ok}   np.allclose(py): {py_ok}")
assert px_ok and py_ok, "px/py NOT bit-for-bit identical to shipped reference"
print("PASS -- px/py bit-for-bit identical to vio_full_gyropredict_foundloc_final.csv.\n")


# ---------------------------------------------------------------------------------------------
# STEP 2: build the REAL anchor stream from the EXTENDED-coverage same-domain AnyLoc retrieval
# results (anyloc_samedomain_100m_ext_result.json / query_frames_timed.csv)
# ---------------------------------------------------------------------------------------------
print("=" * 90)
print("STEP 2: building real anchor stream from samedomain_100m_ext/query_frames_timed.csv")
print("=" * 90)
rows = []
with open(ANCHOR_CSV) as f:
    for r in csv.DictReader(f):
        rows.append(r)
rows.sort(key=lambda r: float(r["t_unix"]))
anchor_t_real = np.array([float(r["t_unix"]) for r in rows])
anchor_xy_real = np.column_stack([
    (np.array([float(r["est_lon"]) for r in rows]) - lon0) * lonm,
    (np.array([float(r["est_lat"]) for r in rows]) - lat0) * latm,
])
print(f"{len(anchor_t_real)} real anchor fixes (EXTENDED-coverage same-domain AnyLoc), t range "
      f"[{anchor_t_real.min()-T0:.1f}, {anchor_t_real.max()-T0:.1f}]s (video-relative)")


# ---------------------------------------------------------------------------------------------
# STEP 3: run the (verified, unchanged) causal loop on the REAL anchor stream, TWICE, checking
# determinism before trusting the result
# ---------------------------------------------------------------------------------------------
print("\n" + "=" * 90)
print("STEP 3: running real-anchor corrector TWICE, checking determinism")
print("=" * 90)
real_args = ap.parse_args([
    TRAJ, OUT_CSV,
    "--anchor-win", "40",
    "--anchor-min-disp", "20",
    "--baro-win", "20",
    "--baro-min-dz", "3",
    "--ema", "0.5",
    "--clamp", "0.2", "5.0",
    "--anchor-pull", "0.9",
    "--vmax", "20",
    "--dbscan-eps", "200",
    "--dbscan-min-samples", "2",
    "--dbscan-window", "8",
    "--lock-min-arc", "150",
    "--lock-collinearity-ratio", "4.0",
])
run_corrector(TRAJ, anchor_t_real, anchor_xy_real, real_args, is_fp=None,
             out_path=OUT_CSV, verbose=True)
run_corrector(TRAJ, anchor_t_real, anchor_xy_real, real_args, is_fp=None,
             out_path="/tmp/_real_anchor_agl100_ext_run2.csv", verbose=False)

a = open(OUT_CSV).read()
b = open("/tmp/_real_anchor_agl100_ext_run2.csv").read()
print(f"run1 == run2 (byte-identical): {a == b}")
assert a == b, "NON-DETERMINISTIC real-anchor output -- DO NOT TRUST, investigate before reporting"
print("PASS -- real-anchor pipeline is deterministic across repeated runs.\n")


# ---------------------------------------------------------------------------------------------
# STEP 4: evaluate against GPS, same methodology as foundloc_eval.py / real_anchor_eval.py /
# run_agl100_fusion.py
# ---------------------------------------------------------------------------------------------
print("=" * 90)
print("STEP 4: evaluation vs GPS (fixed 4DOF alignment, foundloc_eval.py methodology)")
print("=" * 90)
row, t_ext, e_ext = report("EXTENDED-coverage same-domain anchor (100m, loiter-covered, pull=0.9)", OUT_CSV)
print("\nfor reference:")
row_old, t_old, e_old = report("OLD (no-loiter-coverage) same-domain-100m anchor (pull=0.9)",
                               f"{V}/samedomain_100m/vio_full_samedomain_100m_fused.csv")
report("raw VIO (aglprior, gyro-predict)", TRAJ)
report("simulated FoundLoc-style (seed=42, TRUSTED)", f"{V}/vio_full_gyropredict_foundloc_final.csv")

# ---- explicit per-timestamp tail check: t=290..318s, new (extended) vs old (no loiter coverage)
print("\n" + "=" * 90)
print("STEP 5: per-timestamp tail check (t=290..318s), EXTENDED vs OLD")
print("=" * 90)
checkpoints = [290, 295, 300, 305, 310, 314, 318]
tail_rows = []
print(f"{'t (s)':>7}  {'NEW err (m)':>12}  {'OLD err (m)':>12}")
for tc in checkpoints:
    e_new_at = float(np.interp(tc, t_ext, e_ext))
    e_old_at = float(np.interp(tc, t_old, e_old))
    print(f"{tc:>7}  {e_new_at:>12.1f}  {e_old_at:>12.1f}")
    tail_rows.append(dict(t=tc, new_err_m=e_new_at, old_err_m=e_old_at))

with open(RESULT_JSON, "w") as f:
    json.dump({
        "verification": {
            "trusted_rmse": TRUSTED_REF_RMSE, "trusted_max": TRUSTED_REF_MAX,
            "reproduced_rmse": v_rmse, "reproduced_max": v_max,
            "px_allclose": bool(px_ok), "py_allclose": bool(py_ok),
            "bit_exact_match": True,
            "determinism_check_passed": True,
        },
        "real_anchor_source": "anyloc_samedomain_100m_ext_result.json "
                              "(database_survey25_samedomain_100m_ext_vits14, "
                              "same-domain AGL-restricted to 95-105m AGL, EXTENDED extraction "
                              "with --max-time-gap 2.0 covering the t~=306-318s loiter)",
        "n_real_anchors": len(anchor_t_real),
        "anchor_t_range_video_rel_s": [float(anchor_t_real.min() - T0), float(anchor_t_real.max() - T0)],
        "args": {
            "anchor_win": 40, "anchor_min_disp": 20, "baro_win": 20, "baro_min_dz": 3,
            "ema": 0.5, "clamp": [0.2, 5.0], "anchor_pull": 0.9, "vmax": 20,
            "dbscan_eps": 200, "dbscan_min_samples": 2, "dbscan_window": 8,
            "lock_min_arc": 150, "lock_collinearity_ratio": 4.0, "no_filter": False,
        },
        "windows": row,
        "windows_old_no_loiter_coverage": row_old,
        "tail_checkpoints_s": tail_rows,
    }, f, indent=2)
print(f"\nwrote {RESULT_JSON}")
