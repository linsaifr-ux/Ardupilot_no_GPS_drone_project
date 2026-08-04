#!/usr/bin/env python3
"""survey25: same verification-first methodology as real_anchor_eval.py (see that file's
docstring), applied to the NEW real anchor stream built with the corrected North-up rotation
(anyloc/test_accuracy_survey25_time.py --rotate, sign bug fixed 2026-07-25 -- see
instructions/vpe_jump_runaway_diagnosis.md) instead of anyloc_real_fix1plus2_full.json's
unrotated value-facet stream. Not merged into real_anchor_eval.py to avoid touching that
already-verified script; this is a standalone parallel run against a different anchor source.
"""
import csv, json, math, sys
import numpy as np

sys.path.insert(0, "/home/jetson/Ardupilot_no_GPS_drone_project/field_data/survey25/vio_eval")
from foundloc_corrector import build_argparser, run_corrector, lat0, lon0, latm, lonm, T0, S, V

TRAJ = f"{V}/vio_full_aglprior_gyropredict.csv"
TRUSTED_REF_RMSE = 78.57020375864622
TRUSTED_REF_MAX = 164.39048920446118


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
    line = f"{name:40s}:"
    row = {}
    for wlo, whi in [(200, 230), (200, 260), (200, 320), (170, 472)]:
        m = (t >= wlo) & (t < whi)
        if m.any():
            rmse = float(np.sqrt((e[m] ** 2).mean()))
            mx = float(e[m].max())
            row[f"{wlo}-{whi}"] = dict(rmse=rmse, max=mx, n=int(m.sum()))
            line += f"  [{wlo}-{whi}] rmse={rmse:8.1f} max={mx:8.1f}"
    print(line)
    return row


print("=" * 90)
print("STEP 1: verifying imported run_corrector() reproduces the trusted reference (seed=42)")
print("=" * 90)
from foundloc_corrector import simulate_anchor_stream

ap = build_argparser()
verify_args = ap.parse_args([TRAJ, "/tmp/_verify_out2.csv", "--seed", "42"])
v_anchor_t, v_anchor_xy, v_is_fp = simulate_anchor_stream(verify_args)
run_corrector(TRAJ, v_anchor_t, v_anchor_xy, verify_args, is_fp=v_is_fp,
             out_path="/tmp/_verify_out2.csv")
vt, ve, vest, vgt = err_curve("/tmp/_verify_out2.csv")
v_rmse = float(np.sqrt((ve ** 2).mean()))
v_max = float(ve.max())
print(f"reproduced: rmse={v_rmse!r}  max={v_max!r}")
assert abs(v_rmse - TRUSTED_REF_RMSE) < 1e-6, f"MISMATCH: {v_rmse} vs {TRUSTED_REF_RMSE}"
assert abs(v_max - TRUSTED_REF_MAX) < 1e-6, f"MISMATCH: {v_max} vs {TRUSTED_REF_MAX}"
print("PASS -- imported run_corrector() reproduces the trusted reference exactly.\n")


print("=" * 90)
print("STEP 2: building real anchor stream from anyloc_real_fix1_rotated_full.json "
      "(rotation-sign-fixed, default feature mode, db=database_survey25_z20_vits14)")
print("=" * 90)
real = json.load(open(f"{V}/anyloc_real_fix1_rotated_full.json"))
results = real["results"]
results.sort(key=lambda r: r["t_unix"])
anchor_t_real = np.array([r["t_unix"] for r in results])
anchor_xy_real = np.column_stack([
    (np.array([r["est_lon"] for r in results]) - lon0) * lonm,
    (np.array([r["est_lat"] for r in results]) - lat0) * latm,
])
print(f"{len(anchor_t_real)} real anchor fixes, t range "
      f"[{anchor_t_real.min()-T0:.1f}, {anchor_t_real.max()-T0:.1f}]s (video-relative)")

print("\n" + "=" * 90)
print("STEP 3: running real-anchor corrector TWICE, checking determinism")
print("=" * 90)
real_args = ap.parse_args([TRAJ, f"{V}/vio_full_real_anchor_rotated.csv"])
run_corrector(TRAJ, anchor_t_real, anchor_xy_real, real_args, is_fp=None,
             out_path=f"{V}/vio_full_real_anchor_rotated.csv", verbose=True)
run_corrector(TRAJ, anchor_t_real, anchor_xy_real, real_args, is_fp=None,
             out_path="/tmp/_real_anchor_rotated_run2.csv", verbose=False)

a = open(f"{V}/vio_full_real_anchor_rotated.csv").read()
b = open("/tmp/_real_anchor_rotated_run2.csv").read()
print(f"run1 == run2 (byte-identical): {a == b}")
assert a == b, "NON-DETERMINISTIC real-anchor output -- DO NOT TRUST, investigate before reporting"
print("PASS -- real-anchor pipeline is deterministic across repeated runs.\n")

print("=" * 90)
print("STEP 4: evaluation vs GPS (fixed 4DOF alignment, foundloc_eval.py methodology)")
print("=" * 90)
row = report("real-AnyLoc anchor (Fix1 + rotation-fixed)", f"{V}/vio_full_real_anchor_rotated.csv")
print("\nfor reference:")
report("real-AnyLoc anchor (Fix1+2, unrotated, prior best)", f"{V}/vio_full_real_anchor_fix1plus2.csv")
report("raw VIO (aglprior, gyro-predict)", TRAJ)
report("simulated FoundLoc-style (seed=42, TRUSTED)", f"{V}/vio_full_gyropredict_foundloc_final.csv")

with open(f"{V}/real_anchor_rotated_eval_result.json", "w") as f:
    json.dump({
        "verification": {
            "trusted_rmse": TRUSTED_REF_RMSE, "trusted_max": TRUSTED_REF_MAX,
            "reproduced_rmse": v_rmse, "reproduced_max": v_max,
            "bit_exact_match": True,
            "determinism_check_passed": True,
        },
        "real_anchor_source": "anyloc_real_fix1_rotated_full.json (database_survey25_z20_vits14, "
                              "default feature mode, query frames rotated to North-up via "
                              "corrected -heading sign, replicating AnyLoc paper's Nardo-Air-R)",
        "n_real_anchors": len(anchor_t_real),
        "windows": row,
    }, f, indent=2)
print(f"\nwrote {V}/real_anchor_rotated_eval_result.json")
