#!/usr/bin/env python3
"""Compass-heading-corrected variant of foundloc_corrector_survey32.run_corrector.

Frank's observation looking at best_config_path_vs_gps.png: the fused path's overshoot/loop
artifacts still look like a VIO problem. Verified true by code inspection -- `run_corrector`'s
per-tick dead-reckoning is `inc = s_est * (p[k] - p[k-1])`, i.e. VIO's own raw translation
DIRECTION is used completely unmodified between anchor pulls; only its magnitude gets a slowly-
EMA-updated scale correction, and only its absolute POSITION gets periodically snapped toward an
anchor. Nothing in the existing corrector ever corrects VIO's own heading/direction estimate,
which is exactly the thing this project has already shown gets corrupted during turns (rolling
shutter, see instructions/vpe_jump_runaway_diagnosis.md sec 14-11) -- confirmed here too: the
tick-to-tick heading-CHANGE of the fused output correlates 0.69 with raw VIO's own heading-change,
and is even NOISIER (44.6 deg/tick vs VIO's own 31.3 deg/tick) because each anchor pull adds an
extra non-physical direction discontinuity on top.

Frank's proposed fix: the compass (telemetry.csv heading_deg, 5 Hz, gap-free, already trusted
elsewhere in this project for north-up frame rotation) is an INDEPENDENT measurement of vehicle
yaw, immune to the vision-side corruption that hits VIO's own filter yaw during turns. Use it to
correct the DIRECTION of each per-tick displacement while keeping VIO's own DISPLACEMENT
MAGNITUDE (which comes from accel/visual scale, not from the yaw estimate, and isn't the thing
shown to be corrupted).

Mechanism: once the rotation lock (anchor_R) is established (same gating as the existing pull
mechanism -- this fix can't apply before a fixed VIO-frame<->true-north relationship is known),
compute the angular difference between what VIO's own filter believes its yaw is (`quat_yaw`,
already computed for the existing gyro-consistency check) and what the compass says it actually
is (converted into VIO's own frame via the SAME lock rotation `anchor_R` already fit for anchor
pulls), and rotate the raw per-tick displacement vector by that difference before scaling it by
s_est. If VIO's yaw is already correct, the correction is a no-op (difference ~0).

Caveat this fix does NOT address: this assumes the vehicle's direction of travel matches its
compass heading (no significant crab/sideslip) -- true for most controlled multirotor cruise
flight but not guaranteed under wind compensation or aggressive maneuvering. Not verified here,
just noted.
"""
import csv, math, sys
import numpy as np
from sklearn.cluster import DBSCAN

sys.path.insert(0, "/home/jetson/Ardupilot_no_GPS_drone_project/field_data/survey32/vio_eval")
from foundloc_corrector_survey32 import S, V, T0, quat_yaw, baro_at  # noqa: E402

# --- compass heading, 5 Hz, telemetry-native (same source already trusted for north-up rotation
# in tools/extract_frames.py / anyloc/test_accuracy_survey25_time.py) ---
_hdg_rows = []
with open(f"{S}/telemetry.csv") as _f:
    for _r in csv.DictReader(_f):
        try:
            _hdg_rows.append((float(_r["unix_time"]), float(_r["heading_deg"])))
        except (ValueError, KeyError):
            pass
_hdg = np.array(_hdg_rows)


def compass_math_angle_at(t):
    """Compass bearing (deg, CW from true North) -> standard math angle (rad, CCW from East),
    consistent with this project's x=East/y=North local-ENU convention (gxy/anchor_xy)."""
    heading_deg = np.interp(t, _hdg[:, 0], _hdg[:, 1])
    return np.radians(90.0 - heading_deg)


def run_corrector_compass(traj_path, anchor_t, anchor_xy, args, is_fp=None, out_path=None, verbose=True):
    """Identical to foundloc_corrector_survey32.run_corrector except for ONE change: the per-tick
    dead-reckoning increment's DIRECTION is corrected using compass heading once the rotation
    lock is established (magnitude is untouched -- still VIO's own s_est-scaled displacement
    norm). See module docstring for the full mechanism and rationale.
    """
    if is_fp is None:
        is_fp = np.zeros(len(anchor_t), dtype=bool)

    v = np.genfromtxt(traj_path, delimiter=",", names=True)
    tv = v["t"]
    p = np.column_stack([v["px"], v["py"], v["pz"]])
    quat = np.column_stack([v["qx"], v["qy"], v["qz"], v["qw"]])
    yaw_vio = np.array([quat_yaw(q) for q in quat])
    yaw_vio_unwrapped = np.unwrap(yaw_vio)
    compass_true_angle = compass_math_angle_at(tv)  # precompute once, vectorized

    s_est = 1.0
    n_baro = n_anchor = n_anchor_seen = n_anchor_rejected = 0
    n_compass_corrected = 0
    last_anchor_idx = -1
    last_pull_idx = -1
    anchor_R = anchor_Rinv = anchor_off = None
    lock_yaw = None  # scalar angle (VIO-frame -> true-north-frame rotation), extracted from anchor_R
    lock_yaw_source = None
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
        raw_disp = p[k, :2] - p[k - 1, :2]
        if lock_yaw is not None:
            # correct DIRECTION only using compass, keep VIO's own magnitude
            compass_yaw_in_vio_frame = compass_true_angle[k] - lock_yaw
            dyaw = compass_yaw_in_vio_frame - yaw_vio[k]
            dyaw = math.atan2(math.sin(dyaw), math.cos(dyaw))  # wrap to [-pi,pi]
            c_, si_ = math.cos(dyaw), math.sin(dyaw)
            Rdyaw = np.array([[c_, -si_], [si_, c_]])
            raw_disp = Rdyaw @ raw_disp
            n_compass_corrected += 1
        inc = np.array([raw_disp[0], raw_disp[1], p[k, 2] - p[k - 1, 2]]) * s_est
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
                    lock_yaw = yw
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

    n_fp_rejected = sum(1 for _, is_true_fp in rejected_log if is_true_fp) if is_fp is not None else None
    n_good_rejected = (len(rejected_log) - n_fp_rejected) if n_fp_rejected is not None else None
    if verbose:
        print(f"anchors: {n_anchor_seen} seen, {n_anchor} used for scale update, "
              f"{n_anchor_rejected} rejected by DBSCAN")
        print(f"compass-corrected ticks: {n_compass_corrected}/{len(tv)-1}")
        print(f"scale: min {scales.min():.2f} max {scales.max():.2f} final {scales[-1]:.2f}")
        if out_path:
            print(f"wrote {out_path}")

    return dict(pc=pc, scales=scales, tv=tv, names=names, v=v,
                n_anchor_seen=n_anchor_seen, n_anchor_used=n_anchor, n_anchor_rejected=n_anchor_rejected,
                n_fp_rejected=n_fp_rejected, n_good_rejected=n_good_rejected,
                n_compass_corrected=n_compass_corrected)
