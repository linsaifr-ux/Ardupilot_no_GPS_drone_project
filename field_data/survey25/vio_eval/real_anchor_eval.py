#!/usr/bin/env python3
"""survey25: feed REAL AnyLoc retrieval results (not simulated) through the verified
foundloc_corrector.py causal loop (imported, not re-typed), then evaluate against GPS with
the exact same fixed-transform 4DOF alignment methodology as foundloc_eval.py.

Verification-first, per the explicit lesson from earlier tonight's session (a hand-copied
version of this exact loop had 3 subtle bugs that went undetected until checked against a
trusted reference number): before trusting any real-anchor result, this script
  1. reproduces foundloc_corrector.py's own simulated-anchor mode bit-for-bit (--seed 42 ->
     vio_full_gyropredict_foundloc_final.csv, rmse 78.57020375864622m / max 164.39048920446118m)
     via the imported run_corrector() function, and
  2. runs the real-anchor pipeline TWICE and checks the outputs are byte-identical (real
     matches at fixed timestamps have no seed/randomness, unlike the simulation).
Both checks abort the script (assert) if they fail -- no real-anchor number is reported
unless both pass.
"""
import csv, json, math, sys
import numpy as np

sys.path.insert(0, "/home/jetson/Ardupilot_no_GPS_drone_project/field_data/survey25/vio_eval")
from foundloc_corrector import build_argparser, run_corrector, lat0, lon0, latm, lonm, T0, S, V

TRAJ = f"{V}/vio_full_aglprior_gyropredict.csv"
TRUSTED_REF_RMSE = 78.57020375864622
TRUSTED_REF_MAX = 164.39048920446118


# ---- exact copy of foundloc_eval.py's err_curve()/report() methodology (not re-typed from
# memory in a way that could drift -- transcribed directly from foundloc_eval.py, same file
# this session verified against the 78.57m trusted number) ----------------------------------
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


# ---------------------------------------------------------------------------------------------
# STEP 1: verify run_corrector() reproduces the trusted simulated-anchor reference bit-exactly
# ---------------------------------------------------------------------------------------------
print("=" * 90)
print("STEP 1: verifying imported run_corrector() reproduces the trusted reference (seed=42)")
print("=" * 90)
from foundloc_corrector import simulate_anchor_stream

ap = build_argparser()
verify_args = ap.parse_args([TRAJ, "/tmp/_verify_out.csv", "--seed", "42"])
v_anchor_t, v_anchor_xy, v_is_fp = simulate_anchor_stream(verify_args)
verify_result = run_corrector(TRAJ, v_anchor_t, v_anchor_xy, verify_args, is_fp=v_is_fp,
                              out_path="/tmp/_verify_out.csv")
vt, ve, vest, vgt = err_curve("/tmp/_verify_out.csv")
v_rmse = float(np.sqrt((ve ** 2).mean()))
v_max = float(ve.max())
print(f"reproduced: rmse={v_rmse!r}  max={v_max!r}")
print(f"trusted   : rmse={TRUSTED_REF_RMSE!r}  max={TRUSTED_REF_MAX!r}")
assert abs(v_rmse - TRUSTED_REF_RMSE) < 1e-6, f"MISMATCH: {v_rmse} vs {TRUSTED_REF_RMSE}"
assert abs(v_max - TRUSTED_REF_MAX) < 1e-6, f"MISMATCH: {v_max} vs {TRUSTED_REF_MAX}"
print("PASS -- imported run_corrector() reproduces the trusted reference exactly.\n")


# ---------------------------------------------------------------------------------------------
# STEP 2: build the REAL anchor stream from actual AnyLoc retrieval (Fix1+2: rebuilt z20 DB +
# value-facet L10 feature extraction -- the config that measurably beat baseline in the real
# full-window run: mean 273.8m vs baseline 620.3m, see anyloc_real_baseline.json /
# anyloc_real_fix1plus2_full.json)
# ---------------------------------------------------------------------------------------------
print("=" * 90)
print("STEP 2: building real anchor stream from anyloc_real_fix1plus2_full.json")
print("=" * 90)
real = json.load(open(f"{V}/anyloc_real_fix1plus2_full.json"))
results = real["results"]
results.sort(key=lambda r: r["t_unix"])
anchor_t_real = np.array([r["t_unix"] for r in results])
anchor_xy_real = np.column_stack([
    (np.array([r["est_lon"] for r in results]) - lon0) * lonm,
    (np.array([r["est_lat"] for r in results]) - lat0) * latm,
])
print(f"{len(anchor_t_real)} real anchor fixes, t range "
      f"[{anchor_t_real.min()-T0:.1f}, {anchor_t_real.max()-T0:.1f}]s (video-relative)")


# ---------------------------------------------------------------------------------------------
# STEP 3: run the (verified, unchanged) causal loop on the REAL anchor stream, TWICE, and check
# determinism before trusting the result (real matches at fixed timestamps -> no seed randomness
# unlike the simulation, so identical output is expected and required)
# ---------------------------------------------------------------------------------------------
print("\n" + "=" * 90)
print("STEP 3: running real-anchor corrector TWICE, checking determinism")
print("=" * 90)
real_args = ap.parse_args([TRAJ, f"{V}/vio_full_real_anchor_fix1plus2.csv"])
run_corrector(TRAJ, anchor_t_real, anchor_xy_real, real_args, is_fp=None,
             out_path=f"{V}/vio_full_real_anchor_fix1plus2.csv", verbose=True)
run_corrector(TRAJ, anchor_t_real, anchor_xy_real, real_args, is_fp=None,
             out_path="/tmp/_real_anchor_run2.csv", verbose=False)

a = open(f"{V}/vio_full_real_anchor_fix1plus2.csv").read()
b = open("/tmp/_real_anchor_run2.csv").read()
print(f"run1 == run2 (byte-identical): {a == b}")
assert a == b, "NON-DETERMINISTIC real-anchor output -- DO NOT TRUST, investigate before reporting"
print("PASS -- real-anchor pipeline is deterministic across repeated runs.\n")


# ---------------------------------------------------------------------------------------------
# STEP 4: evaluate against GPS, same methodology as foundloc_eval.py
# ---------------------------------------------------------------------------------------------
print("=" * 90)
print("STEP 4: evaluation vs GPS (fixed 4DOF alignment, foundloc_eval.py methodology)")
print("=" * 90)
row = report("real-AnyLoc anchor (Fix1+2, DBSCAN-filtered)", f"{V}/vio_full_real_anchor_fix1plus2.csv")
print("\nfor reference:")
report("raw VIO (aglprior, gyro-predict)", TRAJ)
report("clean-anchor scalefix (simulated, 0% FP)", f"{V}/vio_full_aglprior_scalefix.csv")
report("simulated FoundLoc-style (seed=42, TRUSTED)", f"{V}/vio_full_gyropredict_foundloc_final.csv")

with open(f"{V}/real_anchor_eval_result.json", "w") as f:
    json.dump({
        "verification": {
            "trusted_rmse": TRUSTED_REF_RMSE, "trusted_max": TRUSTED_REF_MAX,
            "reproduced_rmse": v_rmse, "reproduced_max": v_max,
            "bit_exact_match": True,
            "determinism_check_passed": True,
        },
        "real_anchor_source": "anyloc_real_fix1plus2_full.json (database_survey25_z20_vf_L10_vits14, "
                              "dinov2_vits14 layer 10 value facet)",
        "n_real_anchors": len(anchor_t_real),
        "windows": row,
    }, f, indent=2)
print(f"\nwrote {V}/real_anchor_eval_result.json")
