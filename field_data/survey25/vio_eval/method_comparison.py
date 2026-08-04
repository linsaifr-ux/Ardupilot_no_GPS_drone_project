#!/usr/bin/env python3
"""survey25 cruise: 4-way comparison (ungated / hard gyro-gate / AGL depth-prior / combined)
vs GPS ground truth. Reuses the exact 4DOF (yaw + optional scale) alignment methodology from
vio_gyrogate_compare.py: fix the transform from a 20s window right after dyn-init (t=200s, near
the first big turn), then propagate that FIXED transform forward -- no re-aligning per window,
so divergence is visible rather than hidden."""
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
    # best-fit scale (reported, not applied to "est" -- matches vio_gyrogate_compare.py's est_c/est_g
    # which use scale=1 rigid align; scale reported separately per the existing table convention)
    sr = (R @ s.T).T
    k = np.sum(sr * d) / np.sum(sr * sr)
    est = (R @ (xy - ms).T).T + md
    e = np.linalg.norm(est - gt, axis=1)
    return tv - T0, e, est, gt, k


methods = {
    "ungated": "vio_cruise_v2.csv",
    "gyrogate": "vio_cruise_gyrogate.csv",
    "aglprior": "vio_cruise_aglprior_fixed.csv",
    "combined": "vio_cruise_combined.csv",
}

data = {}
for name, fn in methods.items():
    v = np.genfromtxt(f"{V}/{fn}", delimiter=",", names=True)
    t, e, est, gt, k = err_curve(v)
    data[name] = dict(t=t, e=e, est=est, gt=gt, scale=k)
    print(f"{name}: n={len(t)} t range [{t.min():.1f},{t.max():.1f}]")

windows = [(200, 230), (200, 260), (200, 320)]
summary = {"windows": {}, "bug_description": "", "fix_description": ""}
print("\n=== metrics table ===")
for lo, hi in windows:
    wkey = f"{lo}-{hi}s"
    summary["windows"][wkey] = {}
    print(f"\n-- window {wkey} --")
    for name in methods:
        t, e = data[name]["t"], data[name]["e"]
        m = (t >= lo) & (t < hi)
        if not m.any():
            print(f"{name:10s}: no data in window")
            summary["windows"][wkey][name] = None
            continue
        rmse = float(np.sqrt((e[m] ** 2).mean()))
        maxe = float(e[m].max())
        scale = float(data[name]["scale"])
        print(f"{name:10s}: rmse={rmse:8.1f}m  max={maxe:9.1f}m  best-fit-scale={scale:.3f}")
        summary["windows"][wkey][name] = {"rmse_m": rmse, "max_m": maxe, "scale": scale}

# ---- tidy merged timeseries CSV for visualization, 1 Hz common grid ----
t_min = max(data[n]["t"].min() for n in methods)
t_max = min(600, max(data[n]["t"].max() for n in methods))  # cap; individual series still emit past their own divergence
# use the union range actually available per-method (don't truncate to intersection -- keep divergence visible)
t_max_union = max(data[n]["t"].max() for n in methods)
grid = np.arange(200, math.floor(t_max_union) + 1, 1.0)

out_rows = []
gt_interp_x = np.interp(grid, data["ungated"]["t"], data["ungated"]["gt"][:, 0])
gt_interp_y = np.interp(grid, data["ungated"]["t"], data["ungated"]["gt"][:, 1])
method_interp = {}
for name in methods:
    t = data[name]["t"]
    est = data[name]["est"]
    tmax = t.max()
    x = np.interp(grid, t, est[:, 0], right=np.nan)
    y = np.interp(grid, t, est[:, 1], right=np.nan)
    x[grid > tmax] = np.nan
    y[grid > tmax] = np.nan
    method_interp[name] = (x, y)

with open(f"{V}/method_comparison_timeseries.csv", "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["t_rel_s", "gps_x", "gps_y", "ungated_x", "ungated_y", "gyrogate_x", "gyrogate_y",
                "aglprior_x", "aglprior_y", "combined_x", "combined_y"])
    for i, tr in enumerate(grid):
        row = [f"{tr:.1f}", f"{gt_interp_x[i]:.3f}", f"{gt_interp_y[i]:.3f}"]
        for name in ["ungated", "gyrogate", "aglprior", "combined"]:
            x, y = method_interp[name]
            row.append("" if np.isnan(x[i]) else f"{x[i]:.3f}")
            row.append("" if np.isnan(y[i]) else f"{y[i]:.3f}")
        out_rows.append(row)
        w.writerow(row)
print(f"\nwrote {V}/method_comparison_timeseries.csv ({len(out_rows)} rows, grid {grid[0]:.0f}-{grid[-1]:.0f}s)")

summary["bug_description"] = (
    "All 4 pre-fix AGL depth-prior attempts (vio_cruise_aglprior.csv..aglprior4.csv) diverged to "
    "20-150km 'dist traveled' within the first ~15-20s of processing (frame ~200-800 of 4084), far "
    "worse than the ungated baseline (~400m error). Root cause was NOT the depth-prior geometry "
    "(verified empirically: instrumented cos_view calc, both the optical-axis version and a "
    "world-nadir-corrected version track almost identically through the 207-216s turn -- this "
    "flight's turns are yaw-dominated with the nadir camera staying close to vertical throughout, "
    "so optical-axis-vs-true-nadir was never actually the failure mode here). Root cause was a build/"
    "tooling problem: run_video_msckf_aglprior, run_video_msckf_gyrogate, and run_video_msckf_gyrogate_soft "
    "were never registered as real CMake targets (no flags.make/link.txt were ever generated for them by "
    "the previous session -- only bare .o files existed under CMakeFiles/<name>.dir/, placed there by hand). "
    "`make run_video_msckf_aglprior` therefore silently did nothing on every invocation (GNU Make treats an "
    "existing file with no build rule and no newer prerequisite as already up to date) -- so all 4 prior "
    "'attempts' most likely re-ran the SAME stale Jul-24 00:37 binary regardless of source edits made "
    "between attempts, meaning whatever fixes were tried on disk were never actually tested."
)
summary["fix_description"] = (
    "Rebuilt for real: recompiled UpdaterMSCKF.cpp.o and relinked libov_msckf_lib.so and "
    "run_video_msckf_aglprior directly from the recorded CMakeFiles/ov_msckf_lib.dir/{flags.make,link.txt} "
    "(a `cmake .` reconfigure was not usable in this shell -- ROS2/ament_cmake is sourced here and the "
    "project's ROS2.cmake path requires an installed ov_core ament package that was never built, so it "
    "fails find_package(ov_core); the original working build must have come from a shell with neither "
    "catkin nor ament_cmake sourced, landing in the ROS-agnostic branch instead -- this is a pre-existing "
    "environment gap, not something touched here). Also added proper CMake add_executable/target_link_libraries "
    "entries for all 3 project binaries in ov_msckf/cmake/ROS1.cmake so a from-scratch build in a clean "
    "(non-ROS2) shell will produce them automatically going forward. No algorithmic change was made to the "
    "depth-prior math itself -- the on-disk UpdaterMSCKF.cpp logic (blend=0.5 toward AGL/cos_view depth, "
    "12x sigma inflation on chi2-gated corrected features) was already reasonable; it had simply never been "
    "compiled into the binary that was actually being run. A harmless env-gated debug instrumentation "
    "(AGL_DEBUG_COSVIEW) was added to UpdaterMSCKF.cpp to verify the cos_view hypothesis and left in place "
    "(no-op unless that env var is set)."
)

with open(f"{V}/method_comparison_summary.json", "w") as f:
    json.dump(summary, f, indent=2)
print(f"wrote {V}/method_comparison_summary.json")
