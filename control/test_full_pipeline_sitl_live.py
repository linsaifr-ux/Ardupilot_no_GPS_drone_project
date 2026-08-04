#!/usr/bin/env python3
"""
LIVE, position-aware SITL closed-loop test -- fixes a real design flaw in
test_full_pipeline_sitl.py (that script replays a PRE-RECORDED error curve
indexed by a fixed time offset; if the SITL flight paces differently than the
original recording, the time-index silently walks past the database's real
coverage window even while the vehicle is still physically nearby -- found
2026-07-25 on the samedomain run, see instructions/vpe_jump_runaway_diagnosis.md
SS14-23 point 1 and the session log).

Method
------
Every ANCHOR_PERIOD_S (2.0s, matches foundloc_corrector.py's --anchor-period
default and this project's real anchor cadence) while SITL flies the survey25
AUTO mission:
  1. Read SITL's own TRUE position (SIMSTATE -- exactly as the existing harness).
  2. Find the REAL survey25 video frame whose RECORDED telemetry position is
     nearest (by ground distance, not by time) to that true position -- this is
     what makes the loop "live" and "position-aware": if the SITL vehicle drifts
     somewhere the real flight never went near, the nearest match degrades,
     honestly testing self-correction/coverage instead of assuming it.
  3. Decode that exact frame from video.mkv, rotate it North-up using ITS
     recorded heading with the CORRECT sign (angle = -heading -- the bug found
     and fixed earlier tonight in tools/extract_frames.py and
     anyloc/test_accuracy_survey25_time.py; do not reintroduce +heading).
  4. Run REAL AnyLoc (anyloc.localizer.AnyLocLocalizer, database
     anyloc/database_survey25_z20_vits14, default feature mode) on that frame --
     genuine live inference, not a JSON lookup.
  5. Record anyloc_error_m = |AnyLoc estimate - SITL true position| (available
     because this is simulation -- directly tests whether retrieval quality
     degrades with distance from the recorded track).
  6. Feed the AnyLoc estimate as an anchor into LiveCorrector, a live
     re-implementation of foundloc_corrector.py's run_corrector() causal loop
     (DBSCAN-filtered anchor residuals, one-time gyro-consistency-checked SE(2)
     frame lock, anchor-pull blend) -- run_corrector() itself operates on whole
     pre-loaded arrays in one batch pass and cannot be called incrementally, so
     the anchor-handling math is reproduced here tick-by-tick using the same
     formulas and the same tuned defaults (via foundloc_corrector.build_argparser()),
     not hand-typed numbers. NOT reproduced: the baro-vertical-motion scale
     term, which is dropped deliberately -- it indexes agl.csv by ABSOLUTE
     original-recording time, which is exactly the kind of real-vs-SITL-pacing
     mismatch this rewrite exists to avoid reintroducing.
  7. Record localizer_error_m = |corrected estimate - SITL true position| at
     the same tick.
  8. Publish the corrected estimate as VISION_POSITION_ESTIMATE.

Between anchor ticks, position is dead-reckoned from vio_full_aglprior_gyropredict.csv's
OWN frame-to-frame increments, consumed as a queue paced by REAL elapsed SITL
time (not original recording time) -- i.e. "assume live VIO would produce
increments of similar per-second character to what it produced on the real
flight", explicitly a proxy (no live OpenVINS in this environment), but at
least decoupled from absolute-time indexing.

pc's coordinate frame is local-ENU relative to survey25's own home
(lat0,lon0) -- the SAME frame SITL's EKF origin is set to (see build_mission()
in test_full_pipeline_sitl.py). So pc is published DIRECTLY as the position
estimate (not "truth + delta" like the old harness) -- this is a genuinely
independent estimate, not derived from SITL truth at all except through the
nearest-frame search closing the loop. Before the SE(2) lock fires, pc is
just the raw dead-reckoned VIO queue starting from (0,0) -- since both VIO's
own recorded start and SITL's own liftoff point are each near their
respective local origins, this is a reasonable (not perfect) coincidental
frame match; documented here rather than hidden.

Postview: every anchor tick renders a 4-panel PNG (path graph so far / the
real survey25 frame just used / the matched AnyLoc database tile / both error
numbers as text) to field_data/survey25/vio_eval/sitl/postview/frame_%04d.png,
stitched into an MP4 at the end via ffmpeg.

Usage
-----
    python3 control/test_full_pipeline_sitl_live.py run live
    python3 control/test_full_pipeline_sitl_live.py run live_slew   # + vpe_slew.py
"""

import csv
import json
import math
import os
import subprocess
import sys
import time

import cv2
import numpy as np
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "field_data", "survey25", "vio_eval"))
sys.path.insert(0, os.path.join(ROOT, "anyloc"))

from vpe_slew import VpeSlewLimiter
import test_full_pipeline_sitl as base   # reuse mission-building + SITL plumbing (not modified)
from foundloc_corrector import quat_yaw, build_argparser
from localizer import AnyLocLocalizer
from sklearn.cluster import DBSCAN

S, V = base.S, base.V
OUT_DIR = os.path.join(V, "sitl")
POSTVIEW_DIR = os.path.join(OUT_DIR, "postview")

# 2026-07-25: run at real-time (1x), not base.SPEEDUP's 4x. The VIO increment queue only
# has 360s of pre-recorded material; at 4x it's consumed at real_dt*4 (required -- see
# sim_dt comment below -- 1x consumption let dead-reckoning fall behind and diverge on its
# own), so it only covers 90 real wall-clock seconds regardless of how long the flight
# actually takes. Even the well-behaved zero-error control from Round 11 took ~74.5s wall
# clock -- too close to that ceiling for comfort. At SPEEDUP=1 the 360s budget lasts a full
# 360 real seconds, giving real headroom independent of how the flight behaves. Costs wall-
# clock test runtime, not correctness. Overrides base.SPEEDUP locally -- does not touch
# control/test_full_pipeline_sitl.py, which keeps its own already-published 4x results.
SPEEDUP = 1
os.makedirs(POSTVIEW_DIR, exist_ok=True)

VIDEO_PATH = os.path.join(S, "video.mkv")
DB_DIR     = os.path.join(ROOT, "anyloc", "database_survey25_z20_vits14")
VIO_CSV    = os.path.join(V, "vio_full_aglprior_gyropredict.csv")

ANCHOR_PERIOD_S = 2.0
CRUISE_AGL      = base.CRUISE_AGL

# 2026-07-25 bootstrap-alignment fix: `corrector.pc` is dead-reckoned in VIO's OWN
# arbitrary internal frame (never rotated to true North/East) -- this is true of the
# VERIFIED offline foundloc_corrector.py too (its CSV px/py output is raw `pc`, see
# run_corrector() lines ~343-344). Every offline rmse number reported tonight relied on
# a SEPARATE, evaluation-only post-hoc alignment: real_anchor_eval.py's err_curve(fn,
# align_win=20) fits a rigid yaw+offset from a 20s ground-truth window and applies it to
# the WHOLE trajectory before scoring. The live SITL harness never had an equivalent step
# -- it published raw `pc` directly, which is only meaningful once some alignment exists.
# This is legitimate to fix using SITL truth for the same reason it's legitimate on real
# hardware: real flights have GPS at arm/takeoff, before GPS-denial -- ALIGN_WIN_S mirrors
# that same real-world assumption (and the exact align_win=20 value used in every offline
# evaluation tonight), not a form of cheating with ground truth during the GPS-denied
# phase itself.
ALIGN_WIN_S = 20.0


# ── survey25 frame/position index (for nearest-by-GPS lookup, not nearest-by-time) ──
def load_frame_position_index(lat0, lon0, latm, lonm):
    frame_times = []
    with open(os.path.join(S, "frame_times.csv")) as f:
        for row in csv.DictReader(f):
            frame_times.append((int(row["frame_idx"]), float(row["unix_time"])))
    frame_times.sort(key=lambda x: x[1])

    telem = []
    with open(os.path.join(S, "telemetry.csv")) as f:
        for row in csv.DictReader(f):
            try:
                heading = float(row["heading_deg"]) if row.get("heading_deg") else None
            except ValueError:
                heading = None
            try:
                telem.append((float(row["unix_time"]), float(row["lat"]), float(row["lon"]), heading))
            except ValueError:
                pass
    telem.sort(key=lambda r: r[0])
    tt = np.array([r[0] for r in telem])

    entries = []  # (frame_idx, x_m, y_m, lat, lon, heading)
    for fidx, ftime in frame_times:
        i = int(np.searchsorted(tt, ftime))
        i = min(max(i, 0), len(telem) - 1)
        if i > 0 and abs(telem[i - 1][0] - ftime) < abs(telem[i][0] - ftime):
            i -= 1
        _, lat, lon, heading = telem[i]
        if heading is None:
            continue
        x_m = (lon - lon0) * lonm
        y_m = (lat - lat0) * latm
        entries.append((fidx, x_m, y_m, lat, lon, heading))
    xs = np.array([e[1] for e in entries])
    ys = np.array([e[2] for e in entries])
    print(f"[live] frame/position index: {len(entries)} frames with heading")
    return entries, xs, ys


def nearest_frame(entries, xs, ys, x_m, y_m):
    d2 = (xs - x_m) ** 2 + (ys - y_m) ** 2
    i = int(np.argmin(d2))
    fidx, ex, ey, lat, lon, heading = entries[i]
    return fidx, lat, lon, heading, math.sqrt(d2[i])


def decode_frame(frame_idx, heading):
    cap = cv2.VideoCapture(VIDEO_PATH)
    cap.set(cv2.CAP_PROP_POS_FRAMES, frame_idx)
    ok, frame_bgr = cap.read()
    cap.release()
    if not ok:
        return None
    h, w = frame_bgr.shape[:2]
    cx, cy = w // 2, h // 2
    # sign fixed 2026-07-25: -heading, not +heading (see tools/extract_frames.py,
    # anyloc/test_accuracy_survey25_time.py -- verified via live matching-accuracy A/B)
    M = cv2.getRotationMatrix2D((cx, cy), -heading, 1.0)
    rot = cv2.warpAffine(frame_bgr, M, (w, h))
    rgb = cv2.cvtColor(rot, cv2.COLOR_BGR2RGB)
    return Image.fromarray(rgb), rot


# ── live re-implementation of foundloc_corrector.run_corrector()'s anchor logic ──
class LiveCorrector:
    """Same math as foundloc_corrector.run_corrector() for: DBSCAN-filtered
    anchor residual rejection, one-time gyro-consistency-checked SE(2) frame
    lock, anchor-pull blend, anchor arc-ratio scale update. Reimplemented for
    incremental (tick-by-tick) use since run_corrector() takes whole pre-loaded
    arrays and cannot be called per-tick. Deliberately DROPS the baro-vertical-
    motion scale term (see module docstring)."""

    def __init__(self):
        a = build_argparser().parse_args(["_dummy_traj", "_dummy_out"])
        self.args = a
        self.pc = np.zeros(2)               # local-ENU, same frame as SITL home
        self.s_est = 1.0
        self.anchor_R = self.anchor_Rinv = self.anchor_off = None
        self.history_window = []            # (t, residual_xy) trailing anchor residuals for DBSCAN
        self.anchor_hist = []                # (t, anchor_xy) ALL anchors seen (for lock fit, unfiltered -- matches run_corrector's a_hist semantics)
        self.pc_hist = []                    # (t, pc.copy()) dense dead-reckon samples (for lock fit)
        self.yaw_accum = 0.0                 # cumulative dead-reckoned heading change, for gyro-consistency check
        self.last_pull_t = None
        self.arc_pc_since_start = 0.0

    def dead_reckon(self, dpx, dpy, dyaw, dt):
        inc = self.s_est * np.array([dpx, dpy])
        if self.args.vmax > 0 and dt > 0:
            sp = np.linalg.norm(inc) / dt
            if sp > self.args.vmax:
                inc *= self.args.vmax / sp
        self.pc = self.pc + inc
        self.yaw_accum += dyaw
        self.arc_pc_since_start += float(np.linalg.norm(inc))

    def sample_pc_hist(self, t):
        self.pc_hist.append((t, self.pc.copy()))
        if len(self.pc_hist) > 4000:
            self.pc_hist = self.pc_hist[-4000:]

    def bootstrap_align(self, pc_truth_pairs):
        """Seed anchor_R/anchor_off from a short (pc, truth) window instead of waiting
        for the multi-anchor lock -- same rigid-yaw-fit math as
        real_anchor_eval.py's err_curve()/foundloc_corrector's own lock fit, just given
        truth instead of noisy anchors for this one-time bootstrap (see ALIGN_WIN_S
        comment for why that's legitimate). No-op if anchor_R is already set."""
        if self.anchor_R is not None or len(pc_truth_pairs) < 2:
            return False
        pc_arr = np.array([p for p, _ in pc_truth_pairs])
        tr_arr = np.array([t for _, t in pc_truth_pairs])
        ms, md = pc_arr.mean(0), tr_arr.mean(0)
        s_, d_ = pc_arr - ms, tr_arr - md
        yaw = math.atan2(np.sum(s_[:, 0] * d_[:, 1] - s_[:, 1] * d_[:, 0]),
                         np.sum(s_[:, 0] * d_[:, 0] + s_[:, 1] * d_[:, 1]))
        c_, si_ = math.cos(yaw), math.sin(yaw)
        self.anchor_R = np.array([[c_, -si_], [si_, c_]])
        self.anchor_off = md - self.anchor_R @ ms
        self.anchor_Rinv = self.anchor_R.T
        print(f"[live-corrector] BOOTSTRAP ALIGN yaw={math.degrees(yaw):.1f}deg "
              f"from {len(pc_truth_pairs)} truth-referenced samples over "
              f"{ALIGN_WIN_S:.0f}s (real anchors take over the pull from here)")
        return True

    def published_xy(self):
        """Real-ENU position for publishing: pc transformed by anchor_R/off once known
        (bootstrap or real lock), else raw pc (only during the brief pre-bootstrap tail)."""
        if self.anchor_R is not None:
            return self.anchor_R @ self.pc + self.anchor_off
        return self.pc.copy()

    def on_anchor(self, t, anchor_xy):
        """anchor_xy: (2,) local-ENU meters. Returns accepted(bool)."""
        self.anchor_hist.append((t, anchor_xy.copy()))

        # --- DBSCAN false-positive filtering, over trailing residuals vs the
        # CURRENT running estimate (matches run_corrector: must use corrected pc,
        # not raw dead-reckoning, or VIO's own drift swamps the FP signal) ---
        if self.anchor_R is not None:
            pred = self.anchor_R @ self.pc + self.anchor_off
            residual = anchor_xy - pred
        else:
            residual = anchor_xy * 0
        self.history_window.append((t, residual))
        self.history_window = self.history_window[-self.args.dbscan_window:]

        accept = True
        if self.anchor_R is not None and len(self.history_window) >= self.args.dbscan_min_samples:
            pts = np.array([h[1] for h in self.history_window])
            labels = DBSCAN(eps=self.args.dbscan_eps, min_samples=self.args.dbscan_min_samples).fit_predict(pts)
            this_label = labels[-1]
            if this_label == -1:
                accept = False
            else:
                sizes = {lb: (labels == lb).sum() for lb in set(labels) if lb != -1}
                largest = max(sizes, key=sizes.get)
                if this_label != largest:
                    accept = False

        # --- anchor arc-ratio scale update (accepted anchors only) ---
        if accept and len(self.anchor_hist) >= 2:
            t_past = t - self.args.anchor_win
            seg = [a for a in self.anchor_hist if a[0] >= t_past]
            if len(seg) >= 2:
                arc_anchor = float(np.sum(np.linalg.norm(np.diff([s[1] for s in seg], axis=0), axis=1)))
                pc_win = [p for p in self.pc_hist if p[0] >= t_past]
                if len(pc_win) >= 2:
                    arc_vio_c = float(np.sum(np.linalg.norm(np.diff([p[1] for p in pc_win], axis=0), axis=1)))
                    # undo current scale to get the raw-VIO arc this scale was derived from
                    arc_vio = arc_vio_c / max(self.s_est, 1e-3)
                    if arc_anchor >= self.args.anchor_min_disp and arc_vio > 5.0:
                        r = arc_anchor / arc_vio
                        if self.args.clamp[0] < r < self.args.clamp[1]:
                            self.s_est = (1 - self.args.ema) * self.s_est + self.args.ema * r

        # --- one-time gyro-consistency-checked SE(2) frame lock ---
        if self.args.anchor_pull > 0 and self.anchor_R is None and len(self.pc_hist) >= 2:
            pc_arr = np.array([p[1] for p in self.pc_hist])
            t_arr = np.array([p[0] for p in self.pc_hist])
            at_arr = np.array([a[0] for a in self.anchor_hist])
            axy_arr = np.array([a[1] for a in self.anchor_hist])
            if len(at_arr) >= 2:
                a_hist = np.column_stack([np.interp(t_arr, at_arr, axy_arr[:, i]) for i in range(2)])
                arc_pc = float(np.sum(np.linalg.norm(np.diff(pc_arr, axis=0), axis=1)))
                arc_a = float(np.sum(np.linalg.norm(np.diff(a_hist, axis=0), axis=1)))
                if arc_pc >= self.args.lock_min_arc and arc_a >= self.args.lock_min_arc:
                    ms, md = pc_arr.mean(0), a_hist.mean(0)
                    s_, d_ = pc_arr - ms, a_hist - md
                    cov = np.cov(s_.T)
                    eigval = np.linalg.eigvalsh(cov)
                    collin = eigval[-1] / max(eigval[0], 1e-6)
                    yaw_posfit = math.atan2(np.sum(s_[:, 0] * d_[:, 1] - s_[:, 1] * d_[:, 0]),
                                            np.sum(s_[:, 0] * d_[:, 0] + s_[:, 1] * d_[:, 1]))
                    do_lock = True
                    if collin > self.args.lock_collinearity_ratio:
                        if (abs(abs(yaw_posfit) - abs(self.yaw_accum) % (2 * math.pi)) > math.radians(60)
                                and arc_pc < 3 * self.args.lock_min_arc):
                            do_lock = False
                    if do_lock:
                        c_, si_ = math.cos(yaw_posfit), math.sin(yaw_posfit)
                        self.anchor_R = np.array([[c_, -si_], [si_, c_]])
                        self.anchor_off = md - self.anchor_R @ ms
                        self.anchor_Rinv = self.anchor_R.T
                        print(f"[live-corrector] LOCK at t={t:.1f}s yaw={math.degrees(yaw_posfit):.1f}deg "
                              f"collin={collin:.1f} arc_pc={arc_pc:.0f}m arc_a={arc_a:.0f}m")

        # --- anchor pull ---
        if accept and self.args.anchor_pull > 0 and self.anchor_R is not None:
            a_vio_frame = self.anchor_Rinv @ (anchor_xy - self.anchor_off)
            self.pc = self.pc + self.args.anchor_pull * (a_vio_frame - self.pc)

        return accept


def load_vio_increments():
    v = np.genfromtxt(VIO_CSV, delimiter=",", names=True)
    tv = v["t"]
    px, py = v["px"], v["py"]
    quat = np.column_stack([v["qx"], v["qy"], v["qz"], v["qw"]])
    yaw = np.unwrap(np.array([quat_yaw(q) for q in quat]))
    dt = np.diff(tv)
    dpx = np.diff(px)
    dpy = np.diff(py)
    dyaw = np.diff(yaw)
    print(f"[live] VIO increment queue: {len(dt)} steps, "
          f"total span {tv[-1]-tv[0]:.0f}s, total path "
          f"{np.sum(np.hypot(dpx,dpy)):.0f}m")
    return dt, dpx, dpy, dyaw


class VioQueue:
    """Consumes vio_full_aglprior_gyropredict.csv's own relative increments,
    paced by REAL elapsed SITL time (not original recording absolute time)."""

    def __init__(self, dt, dpx, dpy, dyaw):
        self.dt, self.dpx, self.dpy, self.dyaw = dt, dpx, dpy, dyaw
        self.i = 0
        self.acc = 0.0
        self.n = len(dt)
        self._exhausted_logged = False

    def step(self, real_dt):
        """Returns cumulative (dpx, dpy, dyaw) to apply for this real_dt slice."""
        self.acc += real_dt
        cpx = cpy = cdyaw = 0.0
        while self.i < self.n and self.acc >= self.dt[self.i]:
            cpx += self.dpx[self.i]
            cpy += self.dpy[self.i]
            cdyaw += self.dyaw[self.i]
            self.acc -= self.dt[self.i]
            self.i += 1
        if self.i >= self.n and not self._exhausted_logged:
            # queue exhausted -- holds (no further dead-reckoning motion) for the rest of
            # the flight. Flagged loudly, not hidden: this is a real limitation (the VIO
            # recording is only 360s long) if the SITL flight's sim-time-equivalent
            # duration exceeds that.
            print(f"[live] WARNING: VIO increment queue exhausted (all {self.n} steps "
                  f"consumed) -- dead-reckoning will hold still for the remainder of "
                  f"this flight.")
            self._exhausted_logged = True
        return cpx, cpy, cdyaw


# ── postview rendering ───────────────────────────────────────────────────────
def render_postview(idx, route_t, route_m, truth_hist, ekf_hist, pub_hist,
                     query_frame_rgb, matched_tile_path, anyloc_error_m,
                     localizer_error_m, match_gps_dist_m):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(2, 2, figsize=(11, 9))
    ax_path, ax_frame = axes[0]
    ax_tile, ax_text = axes[1]

    ax_path.plot([p[0] for p in route_m], [p[1] for p in route_m], "k--", lw=1.2, label="mission route")
    if truth_hist:
        ax_path.plot([p[0] for p in truth_hist], [p[1] for p in truth_hist], color="tab:blue", lw=1.5, label="SITL truth")
    if ekf_hist:
        ax_path.plot([p[0] for p in ekf_hist], [p[1] for p in ekf_hist], color="tab:red", lw=1.0, alpha=0.7, label="EKF")
    if pub_hist:
        ax_path.plot([p[0] for p in pub_hist], [p[1] for p in pub_hist], color="tab:green", lw=1.0, alpha=0.5, label="published (live corrector)")
    ax_path.set_title(f"path so far  (route t={route_t:.1f}s)")
    ax_path.axis("equal"); ax_path.grid(alpha=0.3); ax_path.legend(fontsize=8)

    if query_frame_rgb is not None:
        ax_frame.imshow(query_frame_rgb)
    ax_frame.set_title("real survey25 frame used (N-up rotated)")
    ax_frame.axis("off")

    if matched_tile_path and os.path.exists(matched_tile_path):
        ax_tile.imshow(Image.open(matched_tile_path).convert("RGB"))
    ax_tile.set_title("AnyLoc matched DB tile")
    ax_tile.axis("off")

    ax_text.axis("off")
    txt = (f"route time: {route_t:.1f} s\n\n"
           f"anyloc_error_m:     {anyloc_error_m:.1f} m\n"
           f"localizer_error_m:  {localizer_error_m:.1f} m\n\n"
           f"nearest-frame GPS search error: {match_gps_dist_m:.1f} m\n"
           f"(distance between query position and the\n"
           f" recorded position of the frame that was used)")
    ax_text.text(0.02, 0.7, txt, fontsize=12, family="monospace", va="top")

    fig.suptitle("survey25 LIVE position-aware SITL closed loop", fontsize=13)
    fig.tight_layout()
    out = os.path.join(POSTVIEW_DIR, f"frame_{idx:04d}.png")
    fig.savefig(out, dpi=100)
    plt.close(fig)
    return out


def assemble_video():
    pattern = os.path.join(POSTVIEW_DIR, "frame_%04d.png")
    out = os.path.join(OUT_DIR, "postview_live.mp4")
    r = subprocess.run(["ffmpeg", "-y", "-framerate", "2", "-i", pattern,
                        "-c:v", "libx264", "-pix_fmt", "yuv420p", out],
                       capture_output=True, text=True)
    if r.returncode != 0:
        print("[live] ffmpeg failed:\n", r.stderr[-2000:])
        return None
    print(f"[live] postview video -> {out}")
    return out


# ── SITL flight ──────────────────────────────────────────────────────────────
def run_flight(run_name: str):
    from mavlink_ctrl import MAVLinkCtrl
    from pymavlink import mavutil

    wp_latlon, lat0, lon0, home_alt_msl, T0, latm, lonm = base.build_mission()
    route_m = [((lon - lon0) * lonm, (lat - lat0) * latm) for lat, lon in wp_latlon]

    print("[live] loading AnyLoc localizer + database …")
    loc = AnyLocLocalizer(DB_DIR)

    entries, fxs, fys = load_frame_position_index(lat0, lon0, latm, lonm)
    dt_q, dpx_q, dpy_q, dyaw_q = load_vio_increments()
    vio_queue = VioQueue(dt_q, dpx_q, dpy_q, dyaw_q)
    corrector = LiveCorrector()

    print(f"[live] run={run_name} home=({lat0:.6f},{lon0:.6f}) {len(wp_latlon)} cruise wps")

    workdir = os.path.join(os.environ.get("TMPDIR", "/tmp"), f"full_pipeline_sitl_{run_name}")
    os.makedirs(workdir, exist_ok=True)
    parm = os.path.join(workdir, "test.parm")
    with open(parm, "w") as f:
        for k, v in base.SITL_PARAMS.items():
            f.write(f"{k} {v}\n")

    proc = subprocess.Popen(
        [base.ARDUCOPTER, "--model", "quad", "--speedup", str(SPEEDUP),
         "--defaults", parm, "--home", f"{lat0},{lon0},{home_alt_msl},0", "-I0"],
        cwd=workdir, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
    time.sleep(4)

    log = dict(run=run_name, home=[lat0, lon0], rows=[], anchors=[])
    truth_hist, ekf_hist, pub_hist = [], [], []
    postview_i = 0

    try:
        ctrl = MAVLinkCtrl(base.CONNECT)
        if not ctrl.wait_heartbeat(30):
            raise RuntimeError("no heartbeat from SITL")
        mav = ctrl._mav
        for msg_id, hz in ((mavutil.mavlink.MAVLINK_MSG_ID_SIMSTATE, 25),
                           (mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 25),
                           (mavutil.mavlink.MAVLINK_MSG_ID_MISSION_CURRENT, 5),
                           (mavutil.mavlink.MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT, 5)):
            mav.mav.command_long_send(
                mav.target_system, mav.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                msg_id, int(1e6 / hz), 0, 0, 0, 0, 0)

        ctrl.set_ekf_origin(lat0, lon0, home_alt_msl)
        ctrl.set_home_position(lat0, lon0, home_alt_msl)

        slew = VpeSlewLimiter() if run_name == "live_slew" else None

        def sim_truth_en():
            m = mav.messages.get("SIMSTATE")
            if m is None:
                return None
            la, lo_ = m.lat, m.lng
            if abs(la) > 1000:
                la, lo_ = la / 1e7, lo_ / 1e7
            return (lo_ - lon0) * lonm, (la - lat0) * latm

        state = dict(last_tick=None, next_anchor=0.0, align_pairs=[])

        def vpe_tick(route_t=None):
            te_n = sim_truth_en()
            if te_n is None:
                return None
            te, tn = te_n

            now_mono = time.monotonic()
            real_dt = 0.0 if state["last_tick"] is None else (now_mono - state["last_tick"])
            state["last_tick"] = now_mono
            real_dt = max(0.0, min(real_dt, 0.5))
            # SITL runs at --speedup Nx wall-clock (base.SPEEDUP): the vehicle's OWN
            # motion/dynamics advance N seconds of "flight time" per 1 wall-clock second.
            # The VIO increment queue is paced against flight time (it's a proxy for what
            # live VIO would produce as the vehicle actually moves), so it must be fed
            # sim_dt = real_dt * SPEEDUP, not raw wall-clock real_dt -- BUG found + fixed
            # 2026-07-25: without this, dead-reckoned pc fell ~4x behind the vehicle's
            # actual progress, diverging without bound and dragging the whole loop
            # (pc vs truth blew up to km-scale, corrupting anyloc_error_m too via the
            # position-controller feedback loop). route_t / ANCHOR_PERIOD_S / timeouts
            # deliberately stay wall-clock (they represent real-world elapsed seconds,
            # matching the real system's actual 2s anchor cadence).
            sim_dt = real_dt * SPEEDUP

            anyloc_err = localizer_err = match_gps_err = None
            frame_rgb = matched_tile_path = None

            if route_t is not None and route_t >= state["next_anchor"]:
                state["next_anchor"] = route_t + ANCHOR_PERIOD_S
                fidx, flat, flon, fheading, match_gps_err = nearest_frame(entries, fxs, fys, te, tn)
                decoded = decode_frame(fidx, fheading)
                if decoded is not None:
                    pil_img, frame_rgb = decoded
                    est_lat, est_lon, _, match_img, score, db_idx = loc.localize(pil_img, agl_m=CRUISE_AGL)
                    anchor_xy = np.array([(est_lon - lon0) * lonm, (est_lat - lat0) * latm])
                    anyloc_err = float(math.hypot(anchor_xy[0] - te, anchor_xy[1] - tn))
                    accepted = corrector.on_anchor(route_t, anchor_xy)
                    matched_tile_path = os.path.join(loc.img_dir, f"{db_idx:06d}.jpg")
                    log["anchors"].append(dict(t=round(route_t, 2), frame_idx=fidx,
                                               match_gps_err_m=round(match_gps_err, 1),
                                               anyloc_error_m=round(anyloc_err, 1),
                                               score=round(score, 3), accepted=bool(accepted)))

            if route_t is not None:
                cdpx, cdpy, cdyaw = vio_queue.step(sim_dt)
                corrector.dead_reckon(cdpx, cdpy, cdyaw, sim_dt)
                corrector.sample_pc_hist(route_t)
                if corrector.anchor_R is None:
                    # bootstrap window (see ALIGN_WIN_S comment): publish truth directly,
                    # same legitimacy as pre-route -- collect (pc, truth) pairs to fit the
                    # one-time yaw/offset the instant the window closes.
                    state["align_pairs"].append((corrector.pc.copy(), np.array([te, tn])))
                    if route_t >= ALIGN_WIN_S:
                        corrector.bootstrap_align(state["align_pairs"])
                if corrector.anchor_R is not None:
                    pub_xy = corrector.published_xy()
                    pe_raw, pn_raw = float(pub_xy[0]), float(pub_xy[1])
                else:
                    pe_raw, pn_raw = te, tn
                localizer_err = float(math.hypot(pe_raw - te, pn_raw - tn))
            else:
                pe_raw, pn_raw = te, tn  # pre-route: publish truth so EKF can acquire cleanly

            if slew is not None:
                pe, pn = slew.update(pe_raw, pn_raw, time.monotonic())
            else:
                pe, pn = pe_raw, pn_raw

            lp = ctrl.local_pos
            down = lp.z if lp is not None else 0.0
            ctrl.send_vision_position(pn, pe, down, 0.0)

            if route_t is not None:
                mc = mav.messages.get("MISSION_CURRENT")
                nco = mav.messages.get("NAV_CONTROLLER_OUTPUT")
                log["rows"].append(dict(
                    t=round(route_t, 3), truth=[round(te, 2), round(tn, 2)],
                    ekf=[round(lp.y, 2), round(lp.x, 2)] if lp else None,
                    pub=[round(pe, 2), round(pn, 2)],
                    localizer_error_m=round(localizer_err, 1) if localizer_err is not None else None,
                    anyloc_error_m=round(anyloc_err, 1) if anyloc_err is not None else None,
                    mis_seq=int(mc.seq) if mc else None,
                    wp_dist=float(nco.wp_dist) if nco else None))
                truth_hist.append((te, tn))
                if lp:
                    ekf_hist.append((lp.y, lp.x))
                pub_hist.append((pe, pn))

                if anyloc_err is not None:
                    nonlocal postview_i
                    render_postview(postview_i, route_t, route_m, truth_hist, ekf_hist, pub_hist,
                                    frame_rgb, matched_tile_path, anyloc_err, localizer_err, match_gps_err)
                    postview_i += 1
            return pe, pn

        def pump(duration=None, cond=None, route_start=None, timeout=180):
            t_end = time.monotonic() + (duration if duration else timeout)
            next_vpe = 0.0
            while time.monotonic() < t_end:
                now = time.monotonic()
                ctrl.recv()
                if now >= next_vpe:
                    rt = (now - route_start) if route_start else None
                    vpe_tick(rt)
                    next_vpe = now + base.TICK_S
                if cond is not None and cond():
                    return True
                time.sleep(0.005)
            return duration is not None

        print("[live] waiting for EKF position…")
        if not pump(cond=lambda: ctrl.ekf_pos_valid, timeout=90):
            raise RuntimeError("EKF never got a position fix")
        pump(duration=5)

        for _ in range(10):
            if "GPS_GLOBAL_ORIGIN" not in mav.messages:
                ctrl.set_ekf_origin(lat0, lon0, home_alt_msl)
            else:
                mav.mav.command_long_send(
                    mav.target_system, mav.target_component,
                    mavutil.mavlink.MAV_CMD_DO_SET_HOME, 0, 0, 0, 0, 0, lat0, lon0, home_alt_msl)
            pump(duration=1)
            if "HOME_POSITION" in mav.messages:
                break
        else:
            raise RuntimeError("origin/home never accepted")

        items = base.mission_items_from_waypoints(wp_latlon, lat0, lon0, home_alt_msl)
        print(f"[live] uploading {len(items)}-item mission…")
        if not base.upload_mission(mav, items):
            raise RuntimeError("mission upload failed / rejected")

        ctrl.set_mode("GUIDED")
        pump(duration=1)
        for attempt in range(3):
            ctrl.arm(force=True)
            pump(duration=3)
            if ctrl.is_armed:
                break
        if not ctrl.is_armed:
            raise RuntimeError("arm failed")

        ctrl.takeoff(CRUISE_AGL)
        print("[live] climbing to cruise AGL via GUIDED takeoff…")
        lp_alt = lambda: (ctrl.local_pos is not None and -ctrl.local_pos.z > CRUISE_AGL - 3.0)
        if not pump(cond=lp_alt, timeout=90):
            raise RuntimeError("takeoff did not reach cruise altitude")
        pump(duration=3)

        mav.mav.mission_set_current_send(mav.target_system, mav.target_component, 2)
        pump(duration=1)
        ctrl.set_mode("AUTO")
        pump(duration=1)

        print(f"[live] flying survey25 mission ({run_name})…")
        route_start = time.monotonic()

        def landed():
            return route_start and (time.monotonic() - route_start) > 20 and not ctrl.is_armed
        pump(duration=None, cond=landed, route_start=route_start,
             timeout=(315.0 - 170.0) / max(SPEEDUP, 1) * 4 + 120)
        pump(duration=base.HOLD_S)
        print(f"[live] flight segment done, armed={ctrl.is_armed}, "
              f"{len(log['anchors'])} anchor queries, {postview_i} postview frames")
    finally:
        proc.terminate()
        try:
            proc.wait(5)
        except subprocess.TimeoutExpired:
            proc.kill()

    out = os.path.join(OUT_DIR, f"full_pipeline_sitl_{run_name}.json")
    with open(out, "w") as f:
        json.dump(log, f)
    print(f"[live] {len(log['rows'])} samples -> {out}")
    return out


def summarize(run_name):
    path = os.path.join(OUT_DIR, f"full_pipeline_sitl_{run_name}.json")
    d = json.load(open(path))
    rows = d["rows"]
    anyloc_errs = [r["anyloc_error_m"] for r in rows if r.get("anyloc_error_m") is not None]
    loc_errs = [r["localizer_error_m"] for r in rows if r.get("localizer_error_m") is not None]
    ekf_steps, glitches = [], []
    prev = None
    for r in rows:
        if r["ekf"] is None:
            continue
        if prev is not None:
            dd = math.hypot(r["ekf"][0] - prev[0], r["ekf"][1] - prev[1])
            ekf_steps.append(dd)
            if dd > 50:
                glitches.append((r["t"], dd))
        prev = r["ekf"]
    print(f"=== {run_name}: n_rows={len(rows)} n_anchor_ticks={len(anyloc_errs)} ===")
    if anyloc_errs:
        print(f"  anyloc_error_m:    mean={np.mean(anyloc_errs):.1f} max={np.max(anyloc_errs):.1f}")
    if loc_errs:
        print(f"  localizer_error_m: mean={np.mean(loc_errs):.1f} max={np.max(loc_errs):.1f}")
    if ekf_steps:
        print(f"  EKF per-tick step: mean={np.mean(ekf_steps):.2f} max={np.max(ekf_steps):.2f} m")
    print(f"  glitch(>50m) events: {len(glitches)}")


def main():
    args = sys.argv[1:]
    if args[:1] == ["run"] and len(args) == 2:
        run_flight(args[1])
        summarize(args[1])
        assemble_video()
    elif args[:1] == ["summarize"] and len(args) == 2:
        summarize(args[1])
    elif args[:1] == ["assemble"]:
        assemble_video()
    else:
        print(__doc__)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
