#!/usr/bin/env python3
"""survey42: FoundLoc-style fusion corrector (He et al., CMU AirLab, arXiv:2310.16299), fed
with REAL (not simulated) AnyLoc anchors -- global (unconstrained) whole-DB queries against
anyloc/database_survey40_samedomain_vits14 (built from survey40, a separate ~100m AGL map-build
flight over the same zone, same day, same camera) at a fixed 2s cadence, matching the project's
established pattern (survey25/survey32 used a simulated 80/20 good/false-positive stream;
survey42 uses genuinely observed AnyLoc scores/positions instead -- see
anyloc/logs/survey42_samedomain_vo_fusion.json for the prior run that showed this DB gives weak,
low-confidence matches on survey42, mean 159m raw error, scores 0.14-0.42 not separating good
from bad matches).

Corrector logic (run_corrector / run_corrector_relock) is copied unchanged from
field_data/survey32/vio_eval/foundloc_corrector_survey32.py -- only the module-level
paths/telemetry/agl setup and the anchor-stream source are survey42-specific (real AnyLoc
queries instead of simulate_anchor_stream()). See that file's docstring for what the two
FoundLoc pieces (DBSCAN false-positive filtering, gyro-assisted degeneracy-aware frame lock) do
and why.

Uses the REAL Kalibr-calibrated OpenVINS trajectory (vio_full_survey42_kalibr.csv,
2026-08-12 recalibration) as the raw VIO input, not a simulated/approximate one.
"""
import argparse, csv, json, math, os, sys
import numpy as np
from sklearn.cluster import DBSCAN

HERE = os.path.dirname(os.path.abspath(__file__))
S = "/home/jetson/Ardupilot_no_GPS_drone_project/field_data/survey42"
ROOT = "/home/jetson/Ardupilot_no_GPS_drone_project"
sys.path.insert(0, ROOT)
T0 = json.load(open(f"{S}/meta.json"))["video_start_unix"]

# --- ground-truth telemetry + local ENU frame + AGL series (agl.csv not needed -- alt_agl
#     column in telemetry.csv IS the same baro-derived AGL used to build agl.csv elsewhere) ---
_tel_rows = []
with open(f"{S}/telemetry.csv") as _f:
    for _r in csv.DictReader(_f):
        try:
            _tel_rows.append((float(_r["unix_time"]), float(_r["lat"]), float(_r["lon"]),
                               float(_r["alt_agl"])))
        except ValueError:
            pass
tel = np.array(_tel_rows)
lat0, lon0 = tel[0, 1], tel[0, 2]
latm = 111132.954 - 559.822 * math.cos(2 * math.radians(lat0))
lonm = 111412.84 * math.cos(math.radians(lat0)) - 93.5 * math.cos(3 * math.radians(lat0))
gxy = np.column_stack([(tel[:, 2] - lon0) * lonm, (tel[:, 1] - lat0) * latm])
tg = tel[:, 0]
agl_t, agl_v = tel[:, 0], tel[:, 3]


def baro_at(t):
    return np.interp(t, agl_t, agl_v)


def quat_yaw(q):
    x, y, z, w = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def err_curve(xy_t, xy, align_win=20.0):
    """Same fixed-4DOF (yaw+translation, no scale) alignment used throughout this project
    (vio_agl_comparison.py, foundloc_eval.py) -- applied here as the final honest-reporting
    step on top of the corrector's own internal anchor-lock alignment, same as foundloc_eval.py
    does for survey25."""
    gt = np.column_stack([np.interp(xy_t, tg, gxy[:, i]) for i in range(2)])
    ma = xy_t <= xy_t[0] + align_win
    ms, md = xy[ma].mean(0), gt[ma].mean(0)
    s, d = xy[ma] - ms, gt[ma] - md
    yaw = math.atan2(np.sum(s[:, 0] * d[:, 1] - s[:, 1] * d[:, 0]),
                      np.sum(s[:, 0] * d[:, 0] + s[:, 1] * d[:, 1]))
    c, si = math.cos(yaw), math.sin(yaw)
    R = np.array([[c, -si], [si, c]])
    est = (R @ (xy - ms).T).T + md
    e = np.linalg.norm(est - gt, axis=1)
    return e, est, gt


def build_argparser():
    ap = argparse.ArgumentParser()
    ap.add_argument("--traj", default=f"{S}/vio_eval/vio_full_survey42_kalibr.csv")
    ap.add_argument("--db-dir", default=f"{ROOT}/anyloc/database_survey40_samedomain_vits14")
    ap.add_argument("--anchor-start-agl", type=float, default=90.0)
    ap.add_argument("--anchor-end-t", type=float, default=229.4, help="video-relative seconds")
    ap.add_argument("--anchor-period", type=float, default=2.0)
    ap.add_argument("--anchor-win", type=float, default=40.0)
    ap.add_argument("--anchor-min-disp", type=float, default=100.0)
    ap.add_argument("--baro-win", type=float, default=20.0)
    ap.add_argument("--baro-min-dz", type=float, default=3.0)
    ap.add_argument("--ema", type=float, default=0.5)
    ap.add_argument("--clamp", type=float, nargs=2, default=[0.2, 5.0])
    ap.add_argument("--anchor-pull", type=float, default=0.60)
    ap.add_argument("--vmax", type=float, default=20.0)
    ap.add_argument("--dbscan-eps", type=float, default=200.0)
    ap.add_argument("--dbscan-min-samples", type=int, default=2)
    ap.add_argument("--dbscan-window", type=int, default=8)
    ap.add_argument("--no-filter", action="store_true")
    ap.add_argument("--lock-min-arc", type=float, default=150.0)
    ap.add_argument("--lock-collinearity-ratio", type=float, default=4.0)
    ap.add_argument("--relock-arc", type=float, default=0.0, help="0 disables relocking (run_corrector); >0 uses run_corrector_relock")
    ap.add_argument("--relock-window-s", type=float, default=40.0)
    ap.add_argument("--out", default="")
    return ap


def real_anchor_stream(args):
    """Query the REAL AnyLoc localizer (global, unconstrained whole-DB search -- same as the
    'global' column in the prior test_vo_fusion_compare.py run) at --anchor-period cadence
    against --db-dir, over the AGL>=--anchor-start-agl cruise band up to --anchor-end-t.
    Returns (anchor_t, anchor_xy_local_enu, scores, true_err_m) -- true_err_m is diagnostic
    only (ground truth is known here because this is an offline replay), not used by the
    corrector itself."""
    import torch
    from PIL import Image
    import cv2
    from anyloc.localizer import AnyLocLocalizer
    from anyloc.test_vo_fusion_compare import load_survey_full, nearest_tel

    meta, frame_times, telemetry = load_survey_full(S)
    frame_by_idx = dict(frame_times)
    start_t = next(t for t in telemetry if t['alt_agl'] >= args.anchor_start_agl)
    t0_query = start_t['unix_time']
    t1_query = T0 + args.anchor_end_t

    loc = AnyLocLocalizer(args.db_dir)
    cap = cv2.VideoCapture(os.path.join(S, 'video.mkv'))

    query_times = np.arange(t0_query, t1_query, args.anchor_period)
    anchor_t, anchor_xy, scores, true_err = [], [], [], []
    tel_hint = 0
    print(f"[anchor] querying {len(query_times)} real AnyLoc fixes vs {args.db_dir} "
          f"({t0_query - T0:.1f}-{t1_query - T0:.1f}s, period {args.anchor_period}s)")
    for qi, qt in enumerate(query_times):
        fidx = min(frame_times, key=lambda ft: abs(ft[1] - qt))[0]
        cap.set(cv2.CAP_PROP_POS_FRAMES, fidx)
        ok, frame_bgr = cap.read()
        if not ok:
            continue
        pil_img = Image.fromarray(cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB))
        tel, tel_hint = nearest_tel(telemetry, frame_by_idx[fidx], tel_hint)
        est_lat, est_lon, _, _, score, _ = loc.localize(pil_img, agl_m=tel['alt_agl'])
        x = (est_lon - lon0) * lonm
        y = (est_lat - lat0) * latm
        true_x = (tel['lon'] - lon0) * lonm
        true_y = (tel['lat'] - lat0) * latm
        err = math.hypot(x - true_x, y - true_y)
        anchor_t.append(frame_by_idx[fidx])
        anchor_xy.append((x, y))
        scores.append(score)
        true_err.append(err)
        if (qi + 1) % 20 == 0 or qi == len(query_times) - 1:
            print(f"  {qi+1}/{len(query_times)}  t={frame_by_idx[fidx]-T0:6.1f}s  "
                  f"score={score:.3f}  err={err:6.1f}m")
    cap.release()
    return (np.array(anchor_t), np.array(anchor_xy), np.array(scores), np.array(true_err))


def run_corrector(traj_path, anchor_t, anchor_xy, args, out_path=None, verbose=True):
    """Unchanged from foundloc_corrector_survey32.py's run_corrector() -- see that file for the
    full explanation of both FoundLoc pieces."""
    v = np.genfromtxt(traj_path, delimiter=",", names=True)
    tv = v["t"]
    p = np.column_stack([v["px"], v["py"], v["pz"]])
    quat = np.column_stack([v["qx"], v["qy"], v["qz"], v["qw"]])
    yaw_vio_unwrapped = np.unwrap(np.array([quat_yaw(q) for q in quat]))

    s_est = 1.0
    n_baro = n_anchor = n_anchor_seen = n_anchor_rejected = 0
    last_anchor_idx = -1
    last_pull_idx = -1
    anchor_R = anchor_Rinv = anchor_off = None
    lock_yaw_source = None
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

        if args.anchor_pull > 0 and anchor_R is None:
            arc_pc = float(np.linalg.norm(np.diff(pc[: k + 1, :2], axis=0), axis=1).sum())
            a_hist = np.column_stack([np.interp(tv[: k + 1], anchor_t, anchor_xy[:, i]) for i in range(2)])
            arc_a = float(np.linalg.norm(np.diff(a_hist, axis=0), axis=1).sum())
            if arc_pc >= args.lock_min_arc and arc_a >= args.lock_min_arc:
                ms_, md_ = pc[: k + 1, :2].mean(0), a_hist.mean(0)
                s_, d_ = pc[: k + 1, :2] - ms_, a_hist - md_
                cov = np.cov(s_.T)
                eigval = np.linalg.eigvalsh(cov)
                collin_ratio = eigval[-1] / max(eigval[0], 1e-6)
                yaw_posfit = math.atan2(np.sum(s_[:, 0] * d_[:, 1] - s_[:, 1] * d_[:, 0]),
                                        np.sum(s_[:, 0] * d_[:, 0] + s_[:, 1] * d_[:, 1]))
                if collin_ratio > args.lock_collinearity_ratio:
                    dyaw_gyro = yaw_vio_unwrapped[k] - yaw_vio_unwrapped[0]
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

    if verbose:
        print(f"anchors: {n_anchor_seen} seen, {n_anchor} used for scale update, "
              f"{n_anchor_rejected} rejected by DBSCAN ({'DISABLED (--no-filter)' if args.no_filter else 'enabled'})")
        print(f"scale: min {scales.min():.2f} max {scales.max():.2f} final {scales[-1]:.2f}")

    return dict(pc=pc, scales=scales, tv=tv, n_anchor_seen=n_anchor_seen,
                n_anchor_used=n_anchor, n_anchor_rejected=n_anchor_rejected)


if __name__ == "__main__":
    args = build_argparser().parse_args()
    anchor_t, anchor_xy, scores, true_err = real_anchor_stream(args)
    print(f"\n[anchor] real AnyLoc stream: n={len(anchor_t)}  "
          f"score mean={scores.mean():.3f}  err mean={true_err.mean():.1f}m median={np.median(true_err):.1f}m\n")

    for label, no_filter in [("FoundLoc-style (DBSCAN-filtered)", False),
                              ("FoundLoc-NF (no filter, raw pull)", True)]:
        args.no_filter = no_filter
        print(f"=== {label} ===")
        res = run_corrector(args.traj, anchor_t, anchor_xy, args,
                             out_path=(args.out or None) if not no_filter else None)
        m = res["tv"] >= res["tv"][0]
        e, est, gt = err_curve(res["tv"][m] - T0, res["pc"][m, :2])
        rmse = float(np.sqrt((e ** 2).mean()))
        print(f"  final (post fixed-4DOF align): rmse={rmse:.1f}m mean={e.mean():.1f}m "
              f"median={np.median(e):.1f}m max={e.max():.1f}m\n")
