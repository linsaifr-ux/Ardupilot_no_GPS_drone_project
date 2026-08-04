#!/usr/bin/env python3
"""survey32: feed REAL AnyLoc retrieval results (survey33-built database, not simulated) through
the verified foundloc_corrector logic (imported from foundloc_corrector_survey32.py, a pure
path-substituted copy of the survey25-verified foundloc_corrector.py -- see that file's diff
against the original before trusting this), then evaluate against GPS with the same fixed-4DOF-
alignment methodology as survey25's real_anchor_eval.py.

Official corrector configuration for this project (found 2026-07-27, see
field_data/survey32/vio_eval/README.md section 10 for the full derivation):

  --anchor-win 20 (not the module default of 40): the scale-estimation logic needs a trailing
  anchor_win of anchor history before it can compute a real scale correction. Anchors only start
  at t=116.0s (cruise onset), so with the default 40s window that gate doesn't clear until
  t=156s -- a real, measured 27.7x divergence spike (t=154s: AnyLoc's own retrieval error was
  5.8m, but the OLD (anchor_win=40) fused output was 160.0m off) sits entirely inside that
  40s blind spot. Rotation re-locking (run_corrector_relock) was tried FIRST and did NOT fix this
  -- the max error at t=154s was identical (160.0m) across every relock parameter tried, because
  relock only changes the ROTATION fit cadence, not the anchor_win-gated SCALE update, which is
  unchanged code in both functions. Halving anchor_win to 20s (so the scale gate clears at
  t=136s instead of t=156s) cut the divergence window's RMSE by 5.3x (110.7m -> 20.8m) with no
  measurable cost elsewhere (full-flight RMSE 725.0m -> 712.0m, slightly better).

  Slew limiter (control/vpe_slew.py, VpeSlewLimiter) applied as a post-process on the
  anchor_win=20 output: the RAW anchor-win=20 corrector output has real single-tick position
  jumps up to 65.4m (implied speed ~985 m/s) at anchor-pull moments -- the "pull toward anchor"
  step is applied as one instantaneous CSV-row addition, uncorrelated with real drone dynamics,
  and would trip a real autopilot's EK3_GLITCH_RAD=50m glitch-rejection gate exactly the way this
  whole project's "jump runaway" problem originally happened. Slewing this output through
  VpeSlewLimiter (this project's existing, previously-built rate limiter) both fixes smoothness
  (max step 65.4m -> 1.17m, zero >5m steps, all >20m steps eliminated) AND improves accuracy
  further (cruise RMSE 34.8m -> 28.5m, full-flight RMSE 712.0m -> 573.2m) -- smoothing out the
  overshoot-prone raw pulls reduces error, it doesn't trade accuracy for smoothness.

  This script's output (vio_cruise_real_anchor_survey32.csv) is the FINAL slewed result --
  the one used everywhere else in this project (papers, postview video, SITL live corrector).

Verification-first, per this project's established discipline: no trusted bit-exact simulated-
anchor reference exists for survey32 (that number is specific to survey25's own VIO trajectory),
so the check kept is real_anchor_eval.py's OTHER discipline -- run the real-anchor pipeline
TWICE and require byte-identical output before trusting any downstream number (real matches at
fixed timestamps have no RNG, so this must pass deterministically) -- applied to BOTH the raw
corrector output and the slewed output (VpeSlewLimiter is a pure deterministic sequential filter,
so this should also pass trivially, but it's checked rather than assumed).
"""
import csv, json, math, sys, os
import numpy as np

ROOT = "/home/jetson/Ardupilot_no_GPS_drone_project"
sys.path.insert(0, f"{ROOT}/field_data/survey32/vio_eval")
sys.path.insert(0, f"{ROOT}/control")
from foundloc_corrector_survey32 import build_argparser, run_corrector, lat0, lon0, latm, lonm, T0, S, V
from vpe_slew import VpeSlewLimiter

TRAJ = f"{V}/vio_full_survey32.csv"
ANCHOR_WIN = 20.0  # official config -- see module docstring


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


def err_curve(fn, lo, align_win=20):
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


def report(name, fn, windows):
    row = {}
    line = f"{name:45s}:"
    for wlo, whi in windows:
        t, e, est, gt = err_curve(fn, wlo)
        m = t < whi
        if m.any():
            rmse = float(np.sqrt((e[m] ** 2).mean()))
            mx = float(e[m].max())
            row[f"{wlo}-{whi}"] = dict(rmse=rmse, max=mx, n=int(m.sum()))
            line += f"  [{wlo}-{whi}] rmse={rmse:8.1f} max={mx:8.1f}"
    print(line)
    return row


def step_stats(fn):
    """Per-timestep position step (smoothness) -- confirms no single-tick glitch-radius-tripping
    jumps in the published output. Not just asserted -- verified per this project's discipline."""
    v = np.genfromtxt(fn, delimiter=",", names=True)
    t = v["t"] - T0
    xy = np.column_stack([v["px"], v["py"]])
    step = np.hypot(np.diff(xy[:, 0]), np.diff(xy[:, 1]))
    return dict(mean=float(step.mean()), max=float(step.max()),
                n_over_5m=int((step > 5).sum()), n_over_20m=int((step > 20).sum()),
                n_over_50m=int((step > 50).sum()))


WINDOWS = [(116, 197), (65, 326)]  # cruise-only (100m AGL, no takeoff/climb/descent/landing) + full flight for context

print("=" * 90)
print("STEP 1: building real anchor stream from anyloc_vs_survey33_db_cruise.json")
print("=" * 90)
real = json.load(open(f"{V}/anyloc_vs_survey33_db_cruise.json"))
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
print(f"STEP 2: running real-anchor corrector (anchor_win={ANCHOR_WIN}) TWICE, checking determinism")
print("=" * 90)
ap = build_argparser()
UNSLEWED = f"{V}/vio_cruise_real_anchor_survey32_anchorwin20_unslewed.csv"
real_args = ap.parse_args([TRAJ, UNSLEWED])
real_args.anchor_win = ANCHOR_WIN
run_corrector(TRAJ, anchor_t_real, anchor_xy_real, real_args, is_fp=None, out_path=UNSLEWED, verbose=True)
run_corrector(TRAJ, anchor_t_real, anchor_xy_real, real_args, is_fp=None, out_path="/tmp/_survey32_run2.csv", verbose=False)
a = open(UNSLEWED).read()
b = open("/tmp/_survey32_run2.csv").read()
print(f"run1 == run2 (byte-identical): {a == b}")
assert a == b, "NON-DETERMINISTIC real-anchor output -- DO NOT TRUST, investigate before reporting"
print("PASS -- real-anchor pipeline is deterministic across repeated runs.\n")

print("=" * 90)
print("STEP 3: applying slew limiter (control/vpe_slew.py) as a post-process, checking determinism")
print("=" * 90)
FUSED = f"{V}/vio_cruise_real_anchor_survey32.csv"  # official, final output


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
            row[names.index("px")] = f"{pub[k,0]:.4f}"
            row[names.index("py")] = f"{pub[k,1]:.4f}"
            w.writerow(row)


apply_slew(UNSLEWED, FUSED)
apply_slew(UNSLEWED, "/tmp/_survey32_slew_run2.csv")
a = open(FUSED).read()
b = open("/tmp/_survey32_slew_run2.csv").read()
print(f"slew run1 == run2 (byte-identical): {a == b}")
assert a == b, "NON-DETERMINISTIC slew output -- DO NOT TRUST, investigate before reporting"
print("PASS -- slew limiter is deterministic across repeated runs.\n")

print("=" * 90)
print("STEP 4: smoothness check (per-timestep position step, i.e. would this trip a real")
print("        autopilot's EK3_GLITCH_RAD=50m gate?)")
print("=" * 90)
for label, fn in [("unslewed (anchor_win=20 only)", UNSLEWED), ("SLEWED (final, official)", FUSED)]:
    st = step_stats(fn)
    print(f"{label:32s}: mean={st['mean']:.3f}m max={st['max']:.3f}m  "
          f">5m:{st['n_over_5m']} >20m:{st['n_over_20m']} >50m:{st['n_over_50m']}")

print("\n" + "=" * 90)
print("STEP 5: evaluation vs GPS (fixed 4DOF alignment, same methodology as survey25)")
print("=" * 90)
row = report("real-AnyLoc anchor (survey33 DB, fused+slewed, FINAL)", FUSED, WINDOWS)
print("\nfor reference:")
report("unslewed (anchor_win=20 only)", UNSLEWED, WINDOWS)
report("raw VIO (no correction)", TRAJ, WINDOWS)

with open(f"{V}/real_anchor_eval_result_survey32_cruise.json", "w") as f:
    json.dump({
        "verification": {"determinism_check_passed": True, "slew_determinism_check_passed": True},
        "config": {"anchor_win": ANCHOR_WIN, "slew_corr_rate_mps": 2.5},
        "real_anchor_source": "anyloc_vs_survey33_db_cruise.json (database_survey33_vits14, dinov2_vits14 default features, cruise-only t=116-197s)",
        "n_real_anchors": len(anchor_t_real),
        "windows": row,
        "smoothness": {label: step_stats(fn) for label, fn in
                       [("unslewed", UNSLEWED), ("slewed_final", FUSED)]},
    }, f, indent=2)
print(f"\nwrote {V}/real_anchor_eval_result_survey32_cruise.json")
