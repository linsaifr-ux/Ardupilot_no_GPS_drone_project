#!/usr/bin/env python3
"""survey25: FoundLoc-style fusion corrector (He et al., CMU AirLab, arXiv:2310.16299).

Ports the two concretely useful, previously-missing pieces from FoundLoc into this project's
own output-space scale/anchor corrector (scale_corrector.py, 2026-07-24 morning round):

1. DBSCAN false-positive filtering (FoundLoc III-E-3): FoundLoc clusters the top-N VPR
   retrievals per query and keeps only the largest geographic cluster, discarding scattered
   false matches. Our simulated single-match-per-query anchor stream has no top-N to cluster
   spatially in one shot, so the direct analogue is FoundLoc's OWN temporal answer to the same
   problem (III-B "Map Alignment" runs over a sliding window of the N most recent keyframes,
   not single matches) -- here: cluster the trailing window of recent anchor fixes' RESIDUALS
   against the current running position estimate, keep only the largest DBSCAN cluster, and
   only let those anchors update scale/pull. Outlier (false-positive) anchors are rejected
   from state updates, matching FoundLoc's FoundLoc vs FoundLoc-NF (no-filter) ablation (Table
   II: NF has ~2-3x higher ATE and far higher variance).

2. Gravity/heading-assisted frame lock (FoundLoc III-B eq. 3, the g^L/g^W coplanarity term):
   FoundLoc's ICP-based rigid alignment is under-determined when query/reference positions are
   near-collinear (UAVs fly straight legs) -- pure position-fit yaw becomes ill-conditioned.
   FoundLoc resolves this with a 3D gravity-consistency constraint from the IMU. Our alignment
   is already constrained to planar SE(2) (roll/pitch are not being solved), so the literal 3D
   term does not apply -- the direct analogue is to use the SAME kind of redundant IMU
   information (gyro-integrated relative heading, well-conditioned even in perfectly straight
   flight, unlike position-fit yaw) to validate/replace the position-fit yaw when the
   accumulation window is measured to be near-collinear (eigenvalue ratio of the window's point
   spread). Implemented below as `lock_frame()`.

The rest of the pipeline (arc-length scale ratio during vertical motion / cruise, absolute
position pull, speed-envelope clamp) is unchanged from scale_corrector.py -- these already
play the role of FoundLoc's "long-term pose memory" (anchored trajectory) + "instant pose
observation" (latest VPR fix) EKF corrector (III-C), just as a causal blend rather than a
literal Kalman update; a literal EKF gave the same qualitative result in bench testing and is
not the piece FoundLoc's ablations show mattering most (VPR filtering + alignment robustness
are), so it was not re-implemented to keep this port scoped to the two proven-valuable pieces.

Realistic anchor noise (NEW here, not in scale_corrector.py): scale_corrector.py's anchor
stream was a clean 50m-quantized GPS proxy with ZERO false positives -- unrealistically
generous (real AnyLoc on this project's own footage: mean error 703-943m off-domain, see
memory real_video_constrained_search_failure; 29-399m variance even in the better-case
production runs, memory anyloc_vo_fusion_investigation). This script instead simulates a
mixture: 80% "good" matches (50m-grid-quantized true position + small noise) and 20% "false
positive" matches (large error drawn from the project's own documented worse-case range,
200-900m) at each 2s query -- grounded in this project's real measurements, not arbitrary.
This is the actual test of whether DBSCAN filtering earns its keep, matching FoundLoc's own
ablation methodology.

2026-07-25 refactor: the causal loop (`run_corrector()`) and the simulated-anchor generator
(`simulate_anchor_stream()`) were factored out of the CLI script body so a REAL (non-simulated)
anchor stream -- e.g. from actual AnyLoc retrieval -- can be fed through the exact same,
already-verified fusion logic by importing `run_corrector()` directly, instead of hand-copying
the loop (a hand-copy for a different comparison script earlier tonight had 3 subtle bugs that
went undetected until checked against this script's own trusted output -- see session notes).
Importing this module does NOT parse argv or run anything -- all of that is now inside
`if __name__ == "__main__":`. The CLI's numerical behavior is unchanged (verified: `--seed 42`
still reproduces the previously-shipped `vio_full_gyropredict_foundloc_final.csv` bit-for-bit).
"""
import argparse, csv, json, math
import numpy as np
from sklearn.cluster import DBSCAN

S = "/home/jetson/Ardupilot_no_GPS_drone_project/field_data/survey32"
V = f"{S}/vio_eval"
T0 = json.load(open(f"{S}/meta.json"))["video_start_unix"]


def quat_yaw(q):
    x, y, z, w = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


# --- baro/AGL reference (project-wide constant, independent of trajectory/anchor source) ---
agl = np.genfromtxt(f"{V}/agl.csv", delimiter=",", names=True)


def baro_at(t):
    return np.interp(t, agl["unix_time"], agl["agl_m"])


# --- ground-truth telemetry + local ENU frame (lat0/lon0/latm/lonm) -----------------------
# Shared by BOTH the simulated-anchor generator below AND real-anchor callers converting real
# AnyLoc lat/lon matches into the same local-meters frame -- reuse these, do not recompute.
_tel_rows = []
with open(f"{S}/telemetry.csv") as _f:
    for _r in csv.DictReader(_f):
        try:
            _tel_rows.append((float(_r["unix_time"]), float(_r["lat"]), float(_r["lon"])))
        except ValueError:
            pass
tel = np.array(_tel_rows)
lat0, lon0 = tel[0, 1], tel[0, 2]
latm = 111132.954 - 559.822 * math.cos(2 * math.radians(lat0))
lonm = 111412.84 * math.cos(math.radians(lat0)) - 93.5 * math.cos(3 * math.radians(lat0))
gxy = np.column_stack([(tel[:, 2] - lon0) * lonm, (tel[:, 1] - lat0) * latm])


def build_argparser():
    ap = argparse.ArgumentParser()
    ap.add_argument("traj")
    ap.add_argument("out")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--fp-rate", type=float, default=0.20, help="fraction of anchor queries that are false positives")
    ap.add_argument("--fp-err-range", type=float, nargs=2, default=[200.0, 900.0])
    ap.add_argument("--good-err-std", type=float, default=15.0)
    ap.add_argument("--anchor-period", type=float, default=2.0)
    ap.add_argument("--anchor-grid", type=float, default=50.0)
    ap.add_argument("--anchor-win", type=float, default=40.0)
    ap.add_argument("--anchor-min-disp", type=float, default=100.0)
    ap.add_argument("--baro-win", type=float, default=20.0)
    ap.add_argument("--baro-min-dz", type=float, default=3.0)
    ap.add_argument("--ema", type=float, default=0.5)
    ap.add_argument("--clamp", type=float, nargs=2, default=[0.2, 5.0])
    ap.add_argument("--anchor-pull", type=float, default=0.60,
                    help="Swept 0.3-0.8 across 6 seeds after diagnosing the 137m shipped result: raw VIO "
                         "drift between anchor fixes is large enough (up to 76m within a single 2s gap when "
                         "scale is badly wrong) that pull=0.30 was too weak/slow to keep up -- 0.60 is the "
                         "robust minimum (mean rmse 81.2m vs 120.0m at 0.30); higher (0.8-0.9) trusts individual "
                         "noisy anchors too much and stops improving/gets worse under realistic false-positive "
                         "rates, even with DBSCAN filtering (that trade-off doesn't show up under clean anchors, "
                         "where 0.9 keeps helping).")
    ap.add_argument("--vmax", type=float, default=20.0)
    ap.add_argument("--dbscan-eps", type=float, default=200.0,
                    help="m, residual-space clustering radius. Original sweep (pre-gyro-predict-tracking "
                         "raw VIO signal, 60-800, 5 seeds) found 300 optimal. RETUNED 2026-07-25 after the "
                         "gyro-predicted-KLT tracking fix (see run_video_msckf_aglprior's gyro_predict_track "
                         "flag) made the raw VIO signal much cleaner: residuals used for clustering are "
                         "tighter now, so 200 discriminates false positives better (rejects 57% of true FPs "
                         "vs 40-45% at 300, 0 good anchors wrongly rejected either way). 12-seed mean rmse "
                         "77.9m at eps=200 vs 87.2m at the old eps=300 default, both on the gyro-predict "
                         "signal -- eps must be retuned per raw-signal-quality, it is not a universal "
                         "constant. Sweep was 150-800 x pull 0.5-0.8; flat optimum region eps 150-225.")
    ap.add_argument("--dbscan-min-samples", type=int, default=2)
    ap.add_argument("--dbscan-window", type=int, default=8, help="# trailing anchor samples clustered together")
    ap.add_argument("--no-filter", action="store_true", help="ablation: use noisy anchors WITHOUT DBSCAN filtering (FoundLoc-NF analogue)")
    ap.add_argument("--lock-min-arc", type=float, default=150.0)
    ap.add_argument("--lock-collinearity-ratio", type=float, default=4.0, help="eigenvalue ratio above which the lock window is treated as degenerate")
    return ap


def simulate_anchor_stream(args, seed=None):
    """Simulated AnyLoc-style anchor stream: 80% good (50m-grid-quantized + small noise) /
    20% false-positive (200-900m error) matches at --anchor-period cadence. Returns
    (anchor_t, anchor_xy, is_fp) -- exact same computation as the original inline script body,
    unchanged. `seed` overrides args.seed if given."""
    rng = np.random.default_rng(args.seed if seed is None else seed)

    anchor_t = np.arange(tel[0, 0], tel[-1, 0], args.anchor_period)
    true_xy = np.column_stack([np.interp(anchor_t, tel[:, 0], gxy[:, i]) for i in range(2)])
    grid_xy = np.round(true_xy / args.anchor_grid) * args.anchor_grid

    is_fp = rng.random(len(anchor_t)) < args.fp_rate
    noise = rng.normal(0, args.good_err_std, size=(len(anchor_t), 2))
    fp_ang = rng.uniform(0, 2 * math.pi, size=len(anchor_t))
    fp_mag = rng.uniform(args.fp_err_range[0], args.fp_err_range[1], size=len(anchor_t))
    fp_offset = np.column_stack([fp_mag * np.cos(fp_ang), fp_mag * np.sin(fp_ang)])

    anchor_xy = grid_xy + noise
    anchor_xy[is_fp] = true_xy[is_fp] + fp_offset[is_fp]
    n_fp_injected = int(is_fp.sum())
    print(f"injected {n_fp_injected}/{len(anchor_t)} false-positive anchor queries "
          f"(errors {args.fp_err_range[0]:.0f}-{args.fp_err_range[1]:.0f}m)")
    return anchor_t, anchor_xy, is_fp


def run_corrector(traj_path, anchor_t, anchor_xy, args, is_fp=None, out_path=None, verbose=True):
    """The verified FoundLoc-style causal loop, unchanged from the original inline script body
    -- only lifted into a function so real (non-simulated) anchor streams can reuse it exactly.

    traj_path : path to the OpenVINS trajectory CSV (t,px,py,pz,qx,qy,qz,qw,...)
    anchor_t  : (M,) anchor query unix timestamps, ascending
    anchor_xy : (M,2) anchor position fixes in the SAME local-ENU-meters frame as `p` below
                (see module-level lat0/lon0/latm/lonm -- x=(lon-lon0)*lonm, y=(lat-lat0)*latm)
    args      : argparse.Namespace with the tunables (see build_argparser()) -- args.traj/args.out
                are NOT read here, only the tunable fields
    is_fp     : optional (M,) bool array, ground-truth false-positive flags for diagnostic
                reporting only (simulated runs have this; real runs don't -- pass None)
    out_path  : if given, write the corrected trajectory CSV here (else no file is written)

    Returns: dict(pc=(N,3) corrected positions, scales=(N,) scale estimates, tv=(N,) times,
                  names=list of output CSV column names excluding scale_est, v=structured input
                  array, n_anchor_seen=int, n_anchor_used=int, n_anchor_rejected=int,
                  n_fp_rejected=int or None, n_good_rejected=int or None)
    """
    if is_fp is None:
        is_fp = np.zeros(len(anchor_t), dtype=bool)

    # --- inputs ---
    v = np.genfromtxt(traj_path, delimiter=",", names=True)
    tv = v["t"]
    p = np.column_stack([v["px"], v["py"], v["pz"]])
    quat = np.column_stack([v["qx"], v["qy"], v["qz"], v["qw"]])  # JPL q_GtoI, per existing CSV convention
    yaw_vio = np.array([quat_yaw(q) for q in quat])
    yaw_vio_unwrapped = np.unwrap(yaw_vio)

    # --- causal loop ---
    s_est = 1.0
    n_baro = n_anchor = n_anchor_seen = n_anchor_rejected = 0
    last_anchor_idx = -1
    last_pull_idx = -1
    anchor_R = anchor_Rinv = anchor_off = None
    lock_yaw_source = None
    history_window = []  # (t, anchor_idx, residual_xy_in_vio_frame) for DBSCAN
    pc = np.zeros_like(p)
    pc[0] = p[0]
    scales = np.ones(len(tv))
    rejected_log = []
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
                    n_baro += 1

        t_past = t - args.anchor_win
        ia_now = np.searchsorted(anchor_t, t) - 1
        ia_past = np.searchsorted(anchor_t, t_past) - 1
        if t_past >= tv[0] and ia_past >= 0 and ia_now > ia_past and ia_now != last_anchor_idx:
            last_anchor_idx = ia_now
            n_anchor_seen += 1

            # --- FoundLoc piece 1: DBSCAN false-positive filtering over the trailing window ---
            # residual = anchor position minus what the current scale/track would predict for that
            # same instant, expressed in a common (VIO-local) frame via the best rotation known so
            # far -- consistent (real) matches cluster tightly around the true residual; scattered
            # false positives don't cluster with them or each other. MUST use the CORRECTED running
            # estimate (pc), not raw VIO (p) -- anchor_R/anchor_off were fit against pc, and raw p's
            # own scale error grows over time regardless of match quality, which would swamp the
            # false-positive signal with unrelated VIO drift (found + fixed during integration).
            xy_at_anchor_t = pc[k - 1, :2]
            if anchor_R is not None:
                pred_anchor_frame = anchor_R @ xy_at_anchor_t + anchor_off
                residual = anchor_xy[ia_now] - pred_anchor_frame
            else:
                residual = anchor_xy[ia_now] - anchor_xy[ia_now] * 0  # pre-lock: no filtering possible yet
            history_window.append((anchor_t[ia_now], ia_now, residual))
            history_window = history_window[-args.dbscan_window:]

            accept = True
            if anchor_R is not None and not args.no_filter and len(history_window) >= args.dbscan_min_samples:
                pts = np.array([h[2] for h in history_window])
                labels = DBSCAN(eps=args.dbscan_eps, min_samples=args.dbscan_min_samples).fit_predict(pts)
                this_label = labels[-1]
                if this_label == -1:
                    accept = False
                else:
                    sizes = {l: (labels == l).sum() for l in set(labels) if l != -1}
                    largest = max(sizes, key=sizes.get)
                    if this_label != largest:
                        accept = False
            if not accept:
                n_anchor_rejected += 1
                rejected_log.append((float(anchor_t[ia_now] - T0), bool(is_fp[ia_now])))

            if accept:
                seg = anchor_xy[ia_past:ia_now + 1]
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

        # --- FoundLoc piece 2: degeneracy-aware (gyro-heading-assisted) frame lock ---
        if args.anchor_pull > 0 and anchor_R is None:
            arc_pc = float(np.linalg.norm(np.diff(pc[: k + 1, :2], axis=0), axis=1).sum())
            a_hist = np.column_stack([np.interp(tv[: k + 1], anchor_t, anchor_xy[:, i]) for i in range(2)])
            arc_a = float(np.linalg.norm(np.diff(a_hist, axis=0), axis=1).sum())
            if arc_pc >= args.lock_min_arc and arc_a >= args.lock_min_arc:
                ms_, md_ = pc[: k + 1, :2].mean(0), a_hist.mean(0)
                s_, d_ = pc[: k + 1, :2] - ms_, a_hist - md_
                # collinearity check on the VIO-side window: eigenvalue ratio of point spread
                cov = np.cov(s_.T)
                eigval = np.linalg.eigvalsh(cov)
                collin_ratio = eigval[-1] / max(eigval[0], 1e-6)
                yaw_posfit = math.atan2(np.sum(s_[:, 0] * d_[:, 1] - s_[:, 1] * d_[:, 0]),
                                        np.sum(s_[:, 0] * d_[:, 0] + s_[:, 1] * d_[:, 1]))
                if collin_ratio > args.lock_collinearity_ratio:
                    # degenerate window -- position-fit yaw is unreliable. Cross-check against the
                    # gyro-integrated heading change over the same window (well-conditioned even in
                    # straight flight): if position-fit disagrees with the gyro-relative heading by
                    # more than a coarse sanity bound, prefer extending the window over locking on a
                    # bad yaw. This is the direct analogue of FoundLoc's gravity term: using
                    # independent IMU-derived rotation info to break a position-only ambiguity.
                    dyaw_gyro = yaw_vio_unwrapped[k] - yaw_vio_unwrapped[0]
                    # gyro gives RELATIVE heading change of the vehicle body, not the map-alignment
                    # yaw directly, but a genuine position-fit solution should be consistent with it
                    # up to a constant offset -- check self-consistency isn't wildly off (e.g. fit
                    # implies far more net turning than the gyro actually saw).
                    if abs(abs(yaw_posfit) - abs(dyaw_gyro) % (2 * math.pi)) > math.radians(60) and arc_pc < 3 * args.lock_min_arc:
                        lock_yaw_source = "deferred (collinear, gyro mismatch) at arc=%.0fm" % arc_pc
                    else:
                        lock_yaw_source = f"position-fit (collinearity_ratio={collin_ratio:.1f}, gyro-consistent, extended to arc={arc_pc:.0f}m)"
                        yw = yaw_posfit
                else:
                    yw = yaw_posfit
                    lock_yaw_source = f"position-fit (collinearity_ratio={collin_ratio:.1f}, well-conditioned)"

                if lock_yaw_source and not lock_yaw_source.startswith("deferred"):
                    c_, si_ = math.cos(yw), math.sin(yw)
                    anchor_R = np.array([[c_, -si_], [si_, c_]])
                    anchor_off = md_ - anchor_R @ ms_
                    anchor_Rinv = anchor_R.T
                    if verbose:
                        print(f"anchor frame locked at rel t={tv[k]-tv[0]:.0f}s, yaw={math.degrees(yw):.1f} deg | {lock_yaw_source}")

        do_pull = (args.anchor_pull > 0 and anchor_R is not None and last_anchor_idx >= 0
                   and last_anchor_idx != last_pull_idx)
        if do_pull:
            # only pull using DBSCAN-accepted anchors (the whole point of filtering)
            accepted_this_round = (args.no_filter or accept)
            if accepted_this_round:
                last_pull_idx = last_anchor_idx
                a_vio_frame = anchor_Rinv @ (anchor_xy[last_anchor_idx] - anchor_off)
                pc[k, :2] += args.anchor_pull * (a_vio_frame - pc[k, :2])

    names = list(v.dtype.names)
    if out_path:
        with open(out_path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(names + ["scale_est"])
            for k in range(len(tv)):
                row = [f"{v[n][k]:.6f}" for n in names]
                row[names.index("px")] = f"{pc[k,0]:.4f}"
                row[names.index("py")] = f"{pc[k,1]:.4f}"
                row[names.index("pz")] = f"{pc[k,2]:.4f}"
                w.writerow(row + [f"{scales[k]:.4f}"])

    n_fp_rejected = sum(1 for _, is_true_fp in rejected_log if is_true_fp) if is_fp is not None else None
    n_good_rejected = (len(rejected_log) - n_fp_rejected) if n_fp_rejected is not None else None
    if verbose:
        print(f"anchors: {n_anchor_seen} seen, {n_anchor} used for scale update, "
              f"{n_anchor_rejected} rejected by DBSCAN ({'DISABLED (--no-filter)' if args.no_filter else 'enabled'})")
        if n_anchor_seen and is_fp is not None:
            n_fp_injected = int(np.sum(is_fp))
            print(f"  of {n_fp_injected} true false-positives: {n_fp_rejected} correctly rejected "
                  f"({100*n_fp_rejected/max(n_fp_injected,1):.0f}%); "
                  f"{n_good_rejected} good anchors wrongly rejected")
        print(f"scale: min {scales.min():.2f} max {scales.max():.2f} final {scales[-1]:.2f}")
        if out_path:
            print(f"wrote {out_path}")

    return dict(pc=pc, scales=scales, tv=tv, names=names, v=v,
                n_anchor_seen=n_anchor_seen, n_anchor_used=n_anchor, n_anchor_rejected=n_anchor_rejected,
                n_fp_rejected=n_fp_rejected, n_good_rejected=n_good_rejected)


def run_corrector_relock(traj_path, anchor_t, anchor_xy, args, is_fp=None, out_path=None, verbose=True):
    """Variant of run_corrector() adding PERIODIC RE-LOCKING of the SE(2) alignment transform,
    instead of fitting it once and freezing it forever.

    Found 2026-07-25: on real (non-simulated) high-accuracy same-domain anchors (8.6m mean
    retrieval error, see instructions/vpe_jump_runaway_diagnosis.md SS14-20), fusing through the
    original one-time-lock run_corrector() gave 62-65m rmse in a window where the best-possible
    NON-CAUSAL fixed-transform fit of the SAME raw VIO trajectory achieves 16.5m -- i.e. the
    corrector was leaving ~45m on the table that isn't an inherent VIO-shape limit. Root cause:
    the one-time lock, fit from early-flight data, goes stale as the flight progresses (VIO scale/
    heading drift after the lock point is real even with good anchors), and per-anchor position
    PULL can't fix a stale ROTATION -- pull only nudges position at each anchor instant, it can't
    correct the systematic misalignment of everything between anchors. This is also the literal
    gap between this file's original port and FoundLoc's own design: their "Map Alignment" (SS
    III-B) explicitly runs over a SLIDING WINDOW of the N most recent keyframes -- i.e. it
    re-fits continuously. This project's port only made the DBSCAN filtering piece sliding-window
    (see module docstring point 1); the rotation/offset fit (point 2) stayed a one-time lock.
    This function fixes that specific gap; everything else is identical to run_corrector().

    Extra args (on top of run_corrector()'s): args.relock_arc (float, meters of NEW pc arc-length
    since the last (re)lock before attempting another one; smaller than args.lock_min_arc is fine
    for re-locks since a working transform already exists to sanity-check against -- only the
    FIRST lock needs the full lock_min_arc/collinearity caution) and args.relock_window_s (float,
    seconds of TRAILING pc/anchor history used for each re-fit, so the transform reflects recent
    geometry, not stale early-flight data diluted by everything since).
    """
    if is_fp is None:
        is_fp = np.zeros(len(anchor_t), dtype=bool)

    v = np.genfromtxt(traj_path, delimiter=",", names=True)
    tv = v["t"]
    p = np.column_stack([v["px"], v["py"], v["pz"]])
    quat = np.column_stack([v["qx"], v["qy"], v["qz"], v["qw"]])
    yaw_vio = np.array([quat_yaw(q) for q in quat])
    yaw_vio_unwrapped = np.unwrap(yaw_vio)

    s_est = 1.0
    n_baro = n_anchor = n_anchor_seen = n_anchor_rejected = 0
    last_anchor_idx = -1
    last_pull_idx = -1
    anchor_R = anchor_Rinv = anchor_off = None
    arc_pc_at_last_lock = 0.0
    n_relocks = 0
    history_window = []
    pc = np.zeros_like(p)
    pc[0] = p[0]
    scales = np.ones(len(tv))
    rejected_log = []
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
                    n_baro += 1

        t_past = t - args.anchor_win
        ia_now = np.searchsorted(anchor_t, t) - 1
        ia_past = np.searchsorted(anchor_t, t_past) - 1
        if t_past >= tv[0] and ia_past >= 0 and ia_now > ia_past and ia_now != last_anchor_idx:
            last_anchor_idx = ia_now
            n_anchor_seen += 1

            xy_at_anchor_t = pc[k - 1, :2]
            if anchor_R is not None:
                pred_anchor_frame = anchor_R @ xy_at_anchor_t + anchor_off
                residual = anchor_xy[ia_now] - pred_anchor_frame
            else:
                residual = anchor_xy[ia_now] - anchor_xy[ia_now] * 0
            history_window.append((anchor_t[ia_now], ia_now, residual))
            history_window = history_window[-args.dbscan_window:]

            accept = True
            if anchor_R is not None and not args.no_filter and len(history_window) >= args.dbscan_min_samples:
                pts = np.array([h[2] for h in history_window])
                labels = DBSCAN(eps=args.dbscan_eps, min_samples=args.dbscan_min_samples).fit_predict(pts)
                this_label = labels[-1]
                if this_label == -1:
                    accept = False
                else:
                    sizes = {l: (labels == l).sum() for l in set(labels) if l != -1}
                    largest = max(sizes, key=sizes.get)
                    if this_label != largest:
                        accept = False
            if not accept:
                n_anchor_rejected += 1
                rejected_log.append((float(anchor_t[ia_now] - T0), bool(is_fp[ia_now])))

            if accept:
                seg = anchor_xy[ia_past:ia_now + 1]
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

        arc_pc_total = float(np.linalg.norm(np.diff(pc[: k + 1, :2], axis=0), axis=1).sum())
        need_first_lock = anchor_R is None
        need_relock = (anchor_R is not None and args.relock_arc > 0
                       and (arc_pc_total - arc_pc_at_last_lock) >= args.relock_arc)

        if args.anchor_pull > 0 and (need_first_lock or need_relock):
            if need_first_lock:
                win_mask = np.ones(k + 1, dtype=bool)  # from-start correspondence, same as run_corrector()
                min_arc_needed = args.lock_min_arc
            else:
                t_lo = t - args.relock_window_s
                win_mask = tv[: k + 1] >= t_lo
                min_arc_needed = args.lock_min_arc * 0.4  # re-locks need less data, a working transform already exists

            a_hist_full = np.column_stack([np.interp(tv[: k + 1], anchor_t, anchor_xy[:, i]) for i in range(2)])
            pc_win = pc[: k + 1, :2][win_mask]
            a_win = a_hist_full[win_mask]
            arc_pc_win = float(np.linalg.norm(np.diff(pc_win, axis=0), axis=1).sum()) if len(pc_win) > 1 else 0.0
            arc_a_win = float(np.linalg.norm(np.diff(a_win, axis=0), axis=1).sum()) if len(a_win) > 1 else 0.0

            if arc_pc_win >= min_arc_needed and arc_a_win >= min_arc_needed:
                ms_, md_ = pc_win.mean(0), a_win.mean(0)
                s_, d_ = pc_win - ms_, a_win - md_
                cov = np.cov(s_.T)
                eigval = np.linalg.eigvalsh(cov)
                collin_ratio = eigval[-1] / max(eigval[0], 1e-6)
                yaw_posfit = math.atan2(np.sum(s_[:, 0] * d_[:, 1] - s_[:, 1] * d_[:, 0]),
                                        np.sum(s_[:, 0] * d_[:, 0] + s_[:, 1] * d_[:, 1]))
                do_lock = True
                if collin_ratio > args.lock_collinearity_ratio:
                    dyaw_gyro = yaw_vio_unwrapped[k] - yaw_vio_unwrapped[np.argmax(win_mask)]
                    if (abs(abs(yaw_posfit) - abs(dyaw_gyro) % (2 * math.pi)) > math.radians(60)
                            and arc_pc_win < 3 * min_arc_needed):
                        do_lock = False  # deferred, same caution as the first lock
                if do_lock:
                    c_, si_ = math.cos(yaw_posfit), math.sin(yaw_posfit)
                    anchor_R = np.array([[c_, -si_], [si_, c_]])
                    anchor_off = md_ - anchor_R @ ms_
                    anchor_Rinv = anchor_R.T
                    arc_pc_at_last_lock = arc_pc_total
                    if not need_first_lock:
                        n_relocks += 1
                    if verbose:
                        kind = "LOCK" if need_first_lock else f"RELOCK#{n_relocks}"
                        print(f"{kind} at rel t={tv[k]-tv[0]:.0f}s, yaw={math.degrees(yaw_posfit):.1f} deg, "
                              f"collin={collin_ratio:.1f}, window_arc={arc_pc_win:.0f}m")

        do_pull = (args.anchor_pull > 0 and anchor_R is not None and last_anchor_idx >= 0
                   and last_anchor_idx != last_pull_idx)
        if do_pull:
            accepted_this_round = (args.no_filter or accept)
            if accepted_this_round:
                last_pull_idx = last_anchor_idx
                a_vio_frame = anchor_Rinv @ (anchor_xy[last_anchor_idx] - anchor_off)
                pc[k, :2] += args.anchor_pull * (a_vio_frame - pc[k, :2])

    names = list(v.dtype.names)
    if out_path:
        with open(out_path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(names + ["scale_est"])
            for k in range(len(tv)):
                row = [f"{v[n][k]:.6f}" for n in names]
                row[names.index("px")] = f"{pc[k,0]:.4f}"
                row[names.index("py")] = f"{pc[k,1]:.4f}"
                row[names.index("pz")] = f"{pc[k,2]:.4f}"
                w.writerow(row + [f"{scales[k]:.4f}"])

    n_fp_rejected = sum(1 for _, is_true_fp in rejected_log if is_true_fp) if is_fp is not None else None
    n_good_rejected = (len(rejected_log) - n_fp_rejected) if n_fp_rejected is not None else None
    if verbose:
        print(f"anchors: {n_anchor_seen} seen, {n_anchor} used for scale update, "
              f"{n_anchor_rejected} rejected by DBSCAN, {n_relocks} re-locks performed")
        print(f"scale: min {scales.min():.2f} max {scales.max():.2f} final {scales[-1]:.2f}")
        if out_path:
            print(f"wrote {out_path}")

    return dict(pc=pc, scales=scales, tv=tv, names=names, v=v, n_relocks=n_relocks,
                n_anchor_seen=n_anchor_seen, n_anchor_used=n_anchor, n_anchor_rejected=n_anchor_rejected,
                n_fp_rejected=n_fp_rejected, n_good_rejected=n_good_rejected)


if __name__ == "__main__":
    args = build_argparser().parse_args()
    anchor_t, anchor_xy, is_fp = simulate_anchor_stream(args)
    run_corrector(args.traj, anchor_t, anchor_xy, args, is_fp=is_fp, out_path=args.out)
