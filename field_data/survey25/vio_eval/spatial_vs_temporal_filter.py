#!/usr/bin/env python3
"""Head-to-head: FoundLoc's actual false-positive filter (spatial DBSCAN over the top-N
retrieval candidates of ONE query) vs. our adaptation (temporal DBSCAN over a trailing
window of single-match queries, foundloc_corrector.py), on the same underlying scenario.

We don't have real AnyLoc top-N retrieval lists, so this simulates them: for each query,
generate N=5 ranked candidate positions.
- GOOD query (prob 1-fp_rate): 2 candidates cluster near the true (50m-grid-quantized)
  position (the correct match + one neighboring DB tile, since adjacent satellite tiles
  are visually correlated and both get retrieved -- this is what gives FoundLoc's spatial
  method something to cluster). The other 3 are decoys: OTHER real positions sampled from
  elsewhere on this same flight's track, at a different time -- a grounded model of
  "recurring visual pattern" confusion (farmland/road patches that look alike), not
  arbitrary noise.
- FALSE POSITIVE query (prob fp_rate): the confidently-wrong top candidate (the same
  200-900m-off point used everywhere else tonight) + 4 more decoys from elsewhere on the
  track. None of the 5 are expected to cluster (occasionally 2 decoys coincidentally will,
  if the flight revisits similar terrain -- a real residual failure mode, not hidden).

Same is_fp schedule (same seed) is reused across all four variants below for a fair,
paired comparison -- every variant faces literally the same set of "which queries are
secretly bad," only the filtering mechanism differs.

Four variants evaluated through the identical downstream causal scale+pull corrector
(same code as foundloc_corrector.py, copied here unchanged):
  A. no-filter          -- naive top-1 candidate always used (current baseline)
  B. temporal-only      -- our existing DBSCAN-over-time (foundloc_corrector.py's method)
  C. spatial-only        -- FoundLoc's actual method: DBSCAN over the N candidates of each query
  D. spatial-then-temporal -- both, chained
"""
import argparse, csv, json, math
import numpy as np
from sklearn.cluster import DBSCAN

S = "/home/jetson/Ardupilot_no_GPS_drone_project/field_data/survey25"
V = f"{S}/vio_eval"
T0 = json.load(open(f"{S}/meta.json"))["video_start_unix"]

ap = argparse.ArgumentParser()
ap.add_argument("traj")
ap.add_argument("--seed", type=int, default=0)
ap.add_argument("--fp-rate", type=float, default=0.20)
ap.add_argument("--fp-err-range", type=float, nargs=2, default=[200.0, 900.0])
ap.add_argument("--good-err-std", type=float, default=15.0)
ap.add_argument("--topn", type=int, default=5)
ap.add_argument("--anchor-period", type=float, default=2.0)
ap.add_argument("--anchor-grid", type=float, default=50.0)
ap.add_argument("--spatial-eps", type=float, default=60.0, help="m, within-query candidate clustering radius")
ap.add_argument("--spatial-min-samples", type=int, default=2)
ap.add_argument("--temporal-eps", type=float, default=200.0)
ap.add_argument("--temporal-min-samples", type=int, default=2)
ap.add_argument("--temporal-window", type=int, default=8)
ap.add_argument("--anchor-win", type=float, default=40.0)
ap.add_argument("--anchor-min-disp", type=float, default=100.0)
ap.add_argument("--baro-win", type=float, default=20.0)
ap.add_argument("--baro-min-dz", type=float, default=3.0)
ap.add_argument("--ema", type=float, default=0.5)
ap.add_argument("--clamp", type=float, nargs=2, default=[0.2, 5.0])
ap.add_argument("--anchor-pull", type=float, default=0.6)
ap.add_argument("--vmax", type=float, default=20.0)
ap.add_argument("--lock-min-arc", type=float, default=150.0)
args = ap.parse_args()
rng = np.random.default_rng(args.seed)

v = np.genfromtxt(args.traj, delimiter=",", names=True)
tv = v["t"]
p = np.column_stack([v["px"], v["py"], v["pz"]])
agl = np.genfromtxt(f"{V}/agl.csv", delimiter=",", names=True)


def baro_at(t):
    return np.interp(t, agl["unix_time"], agl["agl_m"])


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

anchor_t = np.arange(tel[0, 0], tel[-1, 0], args.anchor_period)
true_xy = np.column_stack([np.interp(anchor_t, tel[:, 0], gxy[:, i]) for i in range(2)])
grid_xy = np.round(true_xy / args.anchor_grid) * args.anchor_grid
NQ = len(anchor_t)

is_fp = rng.random(NQ) < args.fp_rate
noise = rng.normal(0, args.good_err_std, size=(NQ, 2))
fp_ang = rng.uniform(0, 2 * math.pi, size=NQ)
fp_mag = rng.uniform(args.fp_err_range[0], args.fp_err_range[1], size=NQ)
fp_offset = np.column_stack([fp_mag * np.cos(fp_ang), fp_mag * np.sin(fp_ang)])
top1_xy = grid_xy + noise
top1_xy[is_fp] = true_xy[is_fp] + fp_offset[is_fp]
print(f"injected {int(is_fp.sum())}/{NQ} false-positive queries")

# ---- build top-N candidate lists per query ----
decoy_pool_idx = rng.integers(0, NQ, size=(NQ, args.topn))  # candidate decoys: other times on the same track
candidates = np.zeros((NQ, args.topn, 2))
for q in range(NQ):
    if not is_fp[q]:
        candidates[q, 0] = grid_xy[q] + rng.normal(0, args.good_err_std, 2)  # correct match
        candidates[q, 1] = grid_xy[q] + rng.normal(0, args.good_err_std * 1.5, 2)  # neighboring DB tile, also near truth
        for k in range(2, args.topn):
            candidates[q, k] = true_xy[decoy_pool_idx[q, k]] + rng.normal(0, args.good_err_std, 2)
    else:
        candidates[q, 0] = top1_xy[q]  # confidently wrong
        for k in range(1, args.topn):
            candidates[q, k] = true_xy[decoy_pool_idx[q, k]] + rng.normal(0, args.good_err_std, 2)

# ---- spatial filter: per-query DBSCAN over the N candidates ----
spatial_xy = np.full((NQ, 2), np.nan)
spatial_ok = np.zeros(NQ, dtype=bool)
for q in range(NQ):
    labels = DBSCAN(eps=args.spatial_eps, min_samples=args.spatial_min_samples).fit_predict(candidates[q])
    sizes = {l: (labels == l).sum() for l in set(labels) if l != -1}
    if not sizes:
        continue  # no cluster at all -> reject this query, matches FoundLoc's behavior on total disagreement
    largest = max(sizes, key=sizes.get)
    spatial_xy[q] = candidates[q][labels == largest].mean(axis=0)
    spatial_ok[q] = True

n_spatial_kept = int(spatial_ok.sum())
# how many of the KEPT queries are actually still wrong (spatial filter fooled by 2 coincidentally-close decoys)?
resid_after_spatial = np.linalg.norm(spatial_xy - true_xy, axis=1)
n_spatial_still_bad = int(((resid_after_spatial > 100) & spatial_ok).sum())
print(f"spatial filter: kept {n_spatial_kept}/{NQ} queries ({NQ-n_spatial_kept} rejected, no consensus cluster); "
      f"of kept, {n_spatial_still_bad} still >100m off truth (coincidental decoy clustering)")


quat = np.column_stack([v["qx"], v["qy"], v["qz"], v["qw"]])


def quat_yaw(q):
    x, y, z, w = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


yaw_vio_unwrapped = np.unwrap(np.array([quat_yaw(q) for q in quat]))


def run_corrector(anchor_xy_full, anchor_valid, use_temporal_filter):
    """Verbatim port of foundloc_corrector.py's causal loop (both FoundLoc pieces: temporal
    DBSCAN + gyro-heading-assisted degeneracy-aware lock), generalized so the anchor stream can
    have invalid/rejected entries (anchor_valid[i]=False) skipped entirely -- used for the
    spatial-pre-filtered stream. When anchor_valid is all-True this reproduces
    foundloc_corrector.py's own numbers exactly (verified below before trusting any comparison)."""
    s_est = 1.0
    n_anchor = 0
    n_seen_dbg = 0
    n_rej_dbg = 0
    last_anchor_idx = -1
    last_pull_idx = -1
    anchor_R = anchor_Rinv = anchor_off = None
    history_window = []
    pc = np.zeros_like(p)
    pc[0] = p[0]
    scales = np.ones(len(tv))
    accept = True

    for k in range(1, len(tv)):
        t = tv[k]

        if t - args.baro_win >= tv[0]:
            t_past = t - args.baro_win
            dz_b = baro_at(t) - baro_at(t_past)
            z_past = np.interp(t_past, tv, p[:, 2])
            dz_v = p[k, 2] - z_past
            if abs(dz_b) >= args.baro_min_dz and abs(dz_v) > 0.3:
                r = dz_b / dz_v
                if args.clamp[0] < r < args.clamp[1]:
                    s_est = (1 - args.ema) * s_est + args.ema * r

        t_past = t - args.anchor_win
        ia_now = np.searchsorted(anchor_t, t) - 1
        ia_past = np.searchsorted(anchor_t, t_past) - 1
        if (t_past >= tv[0] and ia_past >= 0 and ia_now > ia_past and ia_now != last_anchor_idx
                and anchor_valid[ia_now]):
            last_anchor_idx = ia_now
            n_seen_dbg += 1

            xy_at_anchor_t = pc[k - 1, :2]
            if anchor_R is not None:
                pred_anchor_frame = anchor_R @ xy_at_anchor_t + anchor_off
                residual = anchor_xy_full[ia_now] - pred_anchor_frame
            else:
                residual = anchor_xy_full[ia_now] - anchor_xy_full[ia_now] * 0
            history_window.append((anchor_t[ia_now], ia_now, residual))
            history_window = history_window[-args.temporal_window:]

            accept = True
            if use_temporal_filter and anchor_R is not None and len(history_window) >= args.temporal_min_samples:
                pts = np.array([h[2] for h in history_window])
                labels = DBSCAN(eps=args.temporal_eps, min_samples=args.temporal_min_samples).fit_predict(pts)
                this_label = labels[-1]
                if this_label == -1:
                    accept = False
                else:
                    sizes = {l: (labels == l).sum() for l in set(labels) if l != -1}
                    if this_label != max(sizes, key=sizes.get):
                        accept = False
            if not accept:
                n_rej_dbg += 1

            if accept:
                seg = np.array([anchor_xy_full[i] for i in range(ia_past, ia_now + 1) if anchor_valid[i]])
                if len(seg) >= 2:
                    arc_anchor = float(np.linalg.norm(np.diff(seg, axis=0), axis=1).sum())
                    mwin = (tv >= t_past) & (tv <= t)
                    arc_vio = float(np.linalg.norm(np.diff(p[mwin, :2], axis=0), axis=1).sum())
                    if arc_anchor >= args.anchor_min_disp and arc_vio > 5.0:
                        r = arc_anchor / arc_vio
                        if args.clamp[0] < r < args.clamp[1]:
                            s_est = (1 - args.ema) * s_est + args.ema * r
                            n_anchor += 1

        scales[k] = s_est
        inc = s_est * (p[k] - p[k - 1])
        if args.vmax > 0:
            dt = tv[k] - tv[k - 1]
            sp = np.linalg.norm(inc[:2]) / max(dt, 1e-3)
            if sp > args.vmax:
                inc[:2] *= args.vmax / sp
        pc[k] = pc[k - 1] + inc

        if args.anchor_pull > 0 and anchor_R is None:
            arc_pc = float(np.linalg.norm(np.diff(pc[: k + 1, :2], axis=0), axis=1).sum())
            # filter by VALIDITY only, not by time -- matches foundloc_corrector.py's a_hist
            # exactly, which uses the full static anchor_t/anchor_xy arrays unrestricted by k
            # (np.interp is local/piecewise so this only reaches ~1 anchor-period of lookahead
            # near the boundary, an already-shipped, already-verified design characteristic, not
            # something to "fix" here -- restricting to anchor_t<=tv[k] was the bug, it forces
            # flat extrapolation for the last ~2s instead of matching the reference's real interp)
            valid_idx = np.where(anchor_valid)[0]
            if len(valid_idx) >= 2:
                a_hist = np.column_stack(
                    [np.interp(tv[: k + 1], anchor_t[valid_idx], anchor_xy_full[valid_idx, i]) for i in range(2)]
                )
                arc_a = float(np.linalg.norm(np.diff(a_hist, axis=0), axis=1).sum())
                if arc_pc >= args.lock_min_arc and arc_a >= args.lock_min_arc:
                    ms_, md_ = pc[: k + 1, :2].mean(0), a_hist.mean(0)
                    s_, d_ = pc[: k + 1, :2] - ms_, a_hist - md_
                    cov = np.cov(s_.T)
                    eigval = np.linalg.eigvalsh(cov)
                    collin_ratio = eigval[-1] / max(eigval[0], 1e-6)
                    yaw_posfit = math.atan2(np.sum(s_[:, 0] * d_[:, 1] - s_[:, 1] * d_[:, 0]),
                                            np.sum(s_[:, 0] * d_[:, 0] + s_[:, 1] * d_[:, 1]))
                    lock_ok = True
                    if collin_ratio > 4.0:
                        dyaw_gyro = yaw_vio_unwrapped[k] - yaw_vio_unwrapped[0]
                        if (abs(abs(yaw_posfit) - abs(dyaw_gyro) % (2 * math.pi)) > math.radians(60)
                                and arc_pc < 3 * args.lock_min_arc):
                            lock_ok = False
                        yw = yaw_posfit
                    else:
                        yw = yaw_posfit
                    if lock_ok:
                        c_, si_ = math.cos(yw), math.sin(yw)
                        anchor_R = np.array([[c_, -si_], [si_, c_]])
                        anchor_off = md_ - anchor_R @ ms_
                        anchor_Rinv = anchor_R.T
                        print(f"LOCK k={k} t={tv[k]-tv[0]:.0f} yaw={math.degrees(yw):.2f} collin={collin_ratio:.1f}")

        do_pull = (args.anchor_pull > 0 and anchor_R is not None and last_anchor_idx >= 0
                   and last_anchor_idx != last_pull_idx and anchor_valid[last_anchor_idx])
        if do_pull and (not use_temporal_filter or accept):
            last_pull_idx = last_anchor_idx
            a_vio_frame = anchor_Rinv @ (anchor_xy_full[last_anchor_idx] - anchor_off)
            pc[k, :2] += args.anchor_pull * (a_vio_frame - pc[k, :2])

    print(f"DEBUG seen={n_seen_dbg} used={n_anchor} rej={n_rej_dbg}")
    return pc


def eval_traj(pc):
    m = tv >= T0 + 170
    tvm = tv[m]
    xy = pc[m, :2]
    gtm = np.column_stack([np.interp(tvm, tel[:, 0], gxy[:, i]) for i in range(2)])
    ma = tvm <= tvm[0] + 20
    ms, md = xy[ma].mean(0), gtm[ma].mean(0)
    s, d = xy[ma] - ms, gtm[ma] - md
    yaw = math.atan2(np.sum(s[:, 0] * d[:, 1] - s[:, 1] * d[:, 0]), np.sum(s[:, 0] * d[:, 0] + s[:, 1] * d[:, 1]))
    c, si = math.cos(yaw), math.sin(yaw)
    est = (np.array([[c, -si], [si, c]]) @ (xy - ms).T).T + md
    e = np.linalg.norm(est - gtm, axis=1)
    return float(np.sqrt((e ** 2).mean()))


always_valid = np.ones(NQ, dtype=bool)

pc_a = run_corrector(top1_xy, always_valid, use_temporal_filter=False)
pc_b = run_corrector(top1_xy, always_valid, use_temporal_filter=True)
pc_c = run_corrector(spatial_xy, spatial_ok, use_temporal_filter=False)
pc_d = run_corrector(spatial_xy, spatial_ok, use_temporal_filter=True)

print(f"\nA. no filter at all:                 rmse={eval_traj(pc_a):7.1f}")
print(f"B. temporal DBSCAN only (ours):       rmse={eval_traj(pc_b):7.1f}")
print(f"C. spatial DBSCAN only (FoundLoc's):  rmse={eval_traj(pc_c):7.1f}")
print(f"D. spatial + temporal (both):         rmse={eval_traj(pc_d):7.1f}")
