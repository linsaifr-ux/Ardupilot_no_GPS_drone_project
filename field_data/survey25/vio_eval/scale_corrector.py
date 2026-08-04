#!/usr/bin/env python3
"""survey25: causal, GPS-free scale corrector applied to a raw VIO trajectory CSV.

Rationale (Frank, 2026-07-24): the VIO path *shape* is right, the monocular *scale* is
what's wrong -- so correct scale in output space instead of more filter surgery.

Corrected path: p_corr[k] = p_corr[k-1] + s[k] * (p_vio[k] - p_vio[k-1]) -- increments
rescaled causally, position never touched directly, all in the VIO frame (GPS is used
only afterwards by the standard evaluation alignment, same as every baseline).

Two GPS-free scale references (see instructions/vpe_jump_runaway_diagnosis.md SS12-2):
- baro: trailing-window ratio dz_baro/dz_vio -- observable only during vertical motion
  (climb/descent); held between episodes. On survey25's level cruise the baro span is
  <2 m for 90 s, so this mode alone cannot track the cruise scale swing (measured).
- anchor: AnyLoc-style absolute fixes. AnyLoc wasn't running during this recording, so
  the anchor stream is SIMULATED from telemetry GPS quantized to the 50 m grid AnyLoc
  actually outputs (memory: localizer output is 50m-grid-quantized), at 2 s cadence.
  Scale = anchor displacement / VIO displacement over a trailing window, gated on both
  displacements being large enough to beat the quantization. Honest label: this is a
  proxy for what plan-B would provide in a real GPS-denied mission, not a real AnyLoc run.

Usage: scale_corrector.py <traj.csv> <out.csv> [--mode baro|anchor|both] [--t0-offset 200]
"""
import argparse, csv, json, math
import numpy as np

S = "/home/jetson/Ardupilot_no_GPS_drone_project/field_data/survey25"
V = f"{S}/vio_eval"
T0 = json.load(open(f"{S}/meta.json"))["video_start_unix"]

ap = argparse.ArgumentParser()
ap.add_argument("traj")
ap.add_argument("out")
ap.add_argument("--mode", default="both", choices=["baro", "anchor", "both"])
ap.add_argument("--anchor-period", type=float, default=2.0)
ap.add_argument("--anchor-grid", type=float, default=50.0)
ap.add_argument("--anchor-win", type=float, default=40.0)
ap.add_argument("--anchor-min-disp", type=float, default=100.0, help="min anchor displacement (m) to trust a ratio, vs 50m grid")
ap.add_argument("--baro-win", type=float, default=20.0)
ap.add_argument("--baro-min-dz", type=float, default=3.0)
ap.add_argument("--ema", type=float, default=0.25, help="EMA weight per accepted scale sample")
ap.add_argument("--clamp", type=float, nargs=2, default=[0.2, 5.0])
ap.add_argument("--anchor-pull", type=float, default=0.0,
                help="per-anchor-sample position blend toward the (50m-quantized) absolute fix; 0=off. "
                     "This is the plan-B fusion role, simulated -- fixes heading drift that scale cannot.")
ap.add_argument("--outage", type=float, nargs=2, default=None,
                help="video-relative [lo hi] seconds during which the anchor stream is unavailable "
                     "(simulates AnyLoc dropout; scale updates and pulls both masked)")
ap.add_argument("--vmax", type=float, default=0.0,
                help="speed envelope clamp m/s (0=off): scaled increments implying faster motion than the "
                     "vehicle can fly (WPNAV_SPEED=12) are clamped -- GPS-free physical-plausibility gate")
args = ap.parse_args()

# --- inputs ---
v = np.genfromtxt(args.traj, delimiter=",", names=True)
tv = v["t"]
p = np.column_stack([v["px"], v["py"], v["pz"]])

agl = np.genfromtxt(f"{V}/agl.csv", delimiter=",", names=True)


def baro_at(t):
    return np.interp(t, agl["unix_time"], agl["agl_m"])


# anchor stream: 50m-quantized GPS at fixed cadence (AnyLoc plan-B proxy)
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
anchor_xy = np.column_stack([np.interp(anchor_t, tel[:, 0], gxy[:, i]) for i in range(2)])
anchor_xy = np.round(anchor_xy / args.anchor_grid) * args.anchor_grid  # 50m grid quantization

# --- causal loop ---
s_est = 1.0
n_baro = n_anchor = 0
last_anchor_idx = -1
last_pull_idx = -1
n_pull = 0
anchor_R = anchor_Rinv = anchor_off = None
pc = np.zeros_like(p)
pc[0] = p[0]
scales = np.ones(len(tv))
for k in range(1, len(tv)):
    t = tv[k]

    if args.mode in ("baro", "both"):
        t_past = t - args.baro_win
        if t_past >= tv[0]:
            dz_b = baro_at(t) - baro_at(t_past)
            z_past = np.interp(t_past, tv, p[:, 2])
            dz_v = p[k, 2] - z_past
            if abs(dz_b) >= args.baro_min_dz and abs(dz_v) > 0.3:
                r = dz_b / dz_v
                if args.clamp[0] < r < args.clamp[1]:
                    s_est = (1 - args.ema) * s_est + args.ema * r
                    n_baro += 1

    anchor_ok = not (args.outage and args.outage[0] <= (t - T0) <= args.outage[1])
    if args.mode in ("anchor", "both") and anchor_ok:
        t_past = t - args.anchor_win
        ia_now = np.searchsorted(anchor_t, t) - 1
        ia_past = np.searchsorted(anchor_t, t_past) - 1
        # arc length (not chord): chord ratios are turn-geometry-dependent and swing wildly
        # through the survey's 150-deg turns; path length is turn-invariant. EMA updates only
        # when a NEW anchor sample lands (2s cadence), not per 15Hz frame, so smoothing time
        # constants mean what they say.
        if t_past >= tv[0] and ia_past >= 0 and ia_now > ia_past and ia_now != last_anchor_idx:
            last_anchor_idx = ia_now
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
    # absolute-anchor pull (plan-B role): needs the corrected path expressed in the anchor
    # frame; the corrector runs in the VIO frame, so pull toward the anchor via a fixed
    # rigid transform locked ONCE, causally, from the first 150 m of common motion (in a
    # real mission this is known upfront: VIO starts at the EKF origin with FC compass
    # heading). Never re-fit afterwards.
    if args.anchor_pull > 0 and anchor_R is None:
        m_hist = tv[: k + 1] >= tv[0]
        arc_pc = float(np.linalg.norm(np.diff(pc[: k + 1, :2], axis=0), axis=1).sum())
        a_hist = np.column_stack([np.interp(tv[: k + 1], anchor_t, anchor_xy[:, i]) for i in range(2)])
        arc_a = float(np.linalg.norm(np.diff(a_hist, axis=0), axis=1).sum())
        if arc_pc >= 150.0 and arc_a >= 150.0:
            ms_, md_ = pc[: k + 1, :2].mean(0), a_hist.mean(0)
            s_, d_ = pc[: k + 1, :2] - ms_, a_hist - md_
            yw = math.atan2(np.sum(s_[:, 0] * d_[:, 1] - s_[:, 1] * d_[:, 0]),
                            np.sum(s_[:, 0] * d_[:, 0] + s_[:, 1] * d_[:, 1]))
            c_, si_ = math.cos(yw), math.sin(yw)
            anchor_R = np.array([[c_, -si_], [si_, c_]])   # vio->anchor rotation
            anchor_off = md_ - anchor_R @ ms_               # vio->anchor: a = R@p + off
            anchor_Rinv = anchor_R.T
            print(f"anchor frame locked at t={t - (anchor_t[0] if False else 0):.1f} (rel {tv[k]-tv[0]:.0f}s in), yaw={math.degrees(yw):.1f} deg")
    if args.anchor_pull > 0 and anchor_R is not None and last_anchor_idx >= 0 and last_anchor_idx != last_pull_idx and anchor_ok:
        last_pull_idx = last_anchor_idx
        a_vio_frame = anchor_Rinv @ (anchor_xy[last_anchor_idx] - anchor_off)
        pc[k, :2] += args.anchor_pull * (a_vio_frame - pc[k, :2])
        n_pull += 1

# --- write corrected trajectory (same columns as input, px..pz replaced) ---
names = list(v.dtype.names)
with open(args.out, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(names + ["scale_est"])
    for k in range(len(tv)):
        row = [f"{v[n][k]:.6f}" for n in names]
        row[names.index("px")] = f"{pc[k,0]:.4f}"
        row[names.index("py")] = f"{pc[k,1]:.4f}"
        row[names.index("pz")] = f"{pc[k,2]:.4f}"
        w.writerow(row + [f"{scales[k]:.4f}"])

print(f"mode={args.mode}: {n_baro} baro updates, {n_anchor} anchor updates, {n_pull} anchor pulls over {len(tv)} samples")
print(f"scale trajectory: start 1.00, min {scales.min():.2f}, max {scales.max():.2f}, final {scales[-1]:.2f}")
print(f"wrote {args.out}")
