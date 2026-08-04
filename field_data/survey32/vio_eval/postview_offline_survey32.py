#!/usr/bin/env python3
"""
Postview video built from PURE survey32 data replayed offline -- no SITL, no simulated vehicle,
no ArduPilot EKF at all. Plays the REAL camera video (video.mkv) continuously at its native
30fps -- output video duration matches real elapsed time 1:1. Restricted to the 100m-AGL cruise
window only (t=116.0-197.2s, AGL>=97m, CRUISE_LO/CRUISE_HI) -- no takeoff/climb/descent/landing,
matching this project's real deployment (pilot-manual takeoff/landing, pipeline only covers
cruise, see memory project-overview). A path-so-far graph and the matched AnyLoc DB tile are
overlaid alongside the live camera feed, refreshed every 2s (this project's real anchor
cadence) rather than every video frame -- real AnyLoc queries exist for the whole of this
window (anyloc_vs_survey33_db_cruise.json, 41 real retrievals against the survey33-built
database, matching the real deployment's AGL>=50m gate), so every tick in this cruise-only cut
has a real match (unlike the full-flight version this replaced, where ticks outside 116-197.2s
showed "not queried").

"Truth" is survey32's own recorded GPS (telemetry.csv, real flight, not simulation). There is no
ArduPilot EKF in this replay, so the SITL version's red "EKF" line is replaced with raw
(uncorrected) VIO. The green "fused" line is vio_cruise_real_anchor_survey32.csv, the
already-verified (determinism-checked) real-anchor fusion output. localizer_error_m is shown
CONTINUOUSLY (interpolated every video frame, not held at tick resolution) since it doesn't
depend on a fresh AnyLoc query, unlike anyloc_error_m which only updates at real anchor ticks.

Both trajectories are aligned into the GPS-comparable frame via the same fixed-4DOF (yaw+offset)
alignment used everywhere else in this project's evaluation code, fit from the well-conditioned
t=116-136s window (climb-phase windows have too little horizontal motion for a well-determined
yaw fit) but applied to the full trajectory.

Performance note: matplotlib (slow) is only used to render the path-so-far graph once per 2s
tick (~127 times); everything else (camera decode/rotate, tile resize, text) is plain OpenCV/
numpy per video frame (~7500 frames) for speed.

Usage:
    /home/jetson/venv/anyloc/bin/python3 field_data/survey32/vio_eval/postview_offline_survey32.py
"""
import csv
import json
import math
import os
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(HERE)))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "anyloc"))

from foundloc_corrector_survey32 import S, V, T0, lat0, lon0, latm, lonm

VIDEO_PATH = os.path.join(S, "video.mkv")
DB_DIR = os.path.join(ROOT, "anyloc", "database_survey33_vits14")
RAW_VIO_CSV = os.path.join(V, "vio_full_survey32.csv")          # stride=2, project-standard
FUSED_CSV = os.path.join(V, "vio_cruise_real_anchor_survey32.csv")
ANCHORS_JSON = os.path.join(V, "anyloc_vs_survey33_db_cruise.json")
OUT_MP4 = os.path.join(V, "postview_offline.mp4")

ALIGN_LO = 116.0        # well-conditioned fit window start (video-relative s) -- cruise onset
CRUISE_LO = 116.0       # 100m-AGL cruise window (AGL>=97m), video-relative seconds
CRUISE_HI = 197.2
TICK_S = 2.0            # matches this project's AnyLoc anchor cadence everywhere else
PANEL = 480              # each quadrant tile is PANEL x PANEL px (except camera keeps its own AR)


# ── real telemetry (heading lookup + GPS truth in the same local-ENU frame, full flight) ──
def load_telemetry():
    rows = []
    with open(os.path.join(S, "telemetry.csv")) as f:
        for r in csv.DictReader(f):
            try:
                heading = float(r["heading_deg"]) if r.get("heading_deg") else None
            except ValueError:
                heading = None
            try:
                rows.append(dict(t=float(r["unix_time"]), lat=float(r["lat"]), lon=float(r["lon"]),
                                  heading=heading))
            except ValueError:
                pass
    rows.sort(key=lambda r: r["t"])
    return rows


TEL = load_telemetry()
TG = np.array([r["t"] for r in TEL])
GXY = np.column_stack([[(r["lon"] - lon0) * lonm for r in TEL],
                        [(r["lat"] - lat0) * latm for r in TEL]])
HEAD = np.array([r["heading"] if r["heading"] is not None else 0.0 for r in TEL])


def nearest_idx(arr, t):
    i = int(np.searchsorted(arr, t))
    i = min(max(i, 0), len(arr) - 1)
    if i > 0 and abs(arr[i - 1] - t) < abs(arr[i] - t):
        i -= 1
    return i


# ── fixed 4DOF (yaw+offset) alignment, fit from a well-conditioned window, applied to the
# FULL trajectory (not truncated to the fitting window) ──
def err_curve_full(fn, align_lo, align_win=20):
    v = np.genfromtxt(fn, delimiter=",", names=True)
    tv = v["t"]
    xy = np.column_stack([v["px"], v["py"]])
    gt = np.column_stack([np.interp(tv, TG, GXY[:, i]) for i in range(2)])
    ma = (tv >= T0 + align_lo) & (tv <= T0 + align_lo + align_win)
    ms, md = xy[ma].mean(0), gt[ma].mean(0)
    s, d = xy[ma] - ms, gt[ma] - md
    yaw = math.atan2(np.sum(s[:, 0] * d[:, 1] - s[:, 1] * d[:, 0]), np.sum(s[:, 0] * d[:, 0] + s[:, 1] * d[:, 1]))
    c, si = math.cos(yaw), math.sin(yaw)
    R = np.array([[c, -si], [si, c]])
    est = (R @ (xy - ms).T).T + md
    return tv - T0, est, gt


def render_path_panel(route_t, gt_hist, raw_hist, fused_hist):
    """Renders just the path-so-far graph to a PANELxPANEL RGB numpy array (matplotlib, slow --
    called once per 2s tick only, not per video frame)."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig = plt.figure(figsize=(PANEL / 100, PANEL / 100), dpi=100)
    ax = fig.add_subplot(111)
    ax.plot([p[0] for p in gt_hist], [p[1] for p in gt_hist], color="tab:blue", lw=1.8, label="GPS truth (real)")
    ax.plot([p[0] for p in raw_hist], [p[1] for p in raw_hist], color="tab:red", lw=1.0, alpha=0.7, label="raw VIO (aligned)")
    ax.plot([p[0] for p in fused_hist], [p[1] for p in fused_hist], color="tab:green", lw=1.2, alpha=0.8, label="fused (aligned)")
    ax.set_title(f"path so far  t={route_t:.1f}s", fontsize=10)
    ax.axis("equal"); ax.grid(alpha=0.3); ax.legend(fontsize=6, loc="best")
    fig.tight_layout(pad=0.5)
    fig.canvas.draw()
    buf = np.asarray(fig.canvas.buffer_rgba())[:, :, :3].copy()
    plt.close(fig)
    return cv2.resize(buf, (PANEL, PANEL))


def render_tile_panel(matched_tile_path):
    if matched_tile_path and os.path.exists(matched_tile_path):
        img = cv2.imread(matched_tile_path)
        return cv2.resize(img, (PANEL, PANEL))[:, :, ::-1]  # BGR->RGB
    canvas = np.full((PANEL, PANEL, 3), 40, dtype=np.uint8)
    cv2.putText(canvas, "not queried this tick", (20, PANEL // 2 - 10),
               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (180, 180, 180), 1, cv2.LINE_AA)
    cv2.putText(canvas, "(outside AGL>=50m cruise window)", (20, PANEL // 2 + 15),
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 150, 150), 1, cv2.LINE_AA)
    return canvas


def render_text_panel(route_t, anyloc_error_m, localizer_error_m, anyloc_queried):
    canvas = np.full((PANEL, PANEL, 3), 255, dtype=np.uint8)
    anyloc_txt = f"{anyloc_error_m:.1f} m" if anyloc_queried else "n/a (not queried)"
    lines = [f"route time: {route_t:6.1f} s", "",
            f"anyloc_error_m:     {anyloc_txt}",
            f"localizer_error_m:  {localizer_error_m:6.1f} m"]
    for i, line in enumerate(lines):
        cv2.putText(canvas, line, (15, 40 + i * 30), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                   (0, 0, 0), 1, cv2.LINE_AA)
    return canvas


def main():
    print("[offline] aligning raw VIO + fused trajectories to GPS frame ...")
    raw_t, raw_est, raw_gt = err_curve_full(RAW_VIO_CSV, ALIGN_LO)
    fused_t, fused_est, fused_gt = err_curve_full(FUSED_CSV, ALIGN_LO)
    # 100m-AGL cruise window only (AGL>=97m, precisely measured -- see
    # field_data/survey32/vio_eval/README.md section 1), not the full flight (no takeoff/
    # climb/descent/landing -- this project's real deployment is pilot-manual for those).
    t_start = max(raw_t.min(), fused_t.min(), CRUISE_LO)
    t_end = min(raw_t.max(), fused_t.max(), CRUISE_HI)
    print(f"  span: {t_start:.1f}-{t_end:.1f}s (cruise-only)")

    print(f"[offline] loading {ANCHORS_JSON} ...")
    anchors = json.load(open(ANCHORS_JSON))["results"]
    anchors.sort(key=lambda r: r["t_rel"])
    anchor_ts = np.array([a["t_rel"] for a in anchors])
    print(f"[offline] {len(anchors)} real anchor ticks in {anchor_ts.min():.1f}-{anchor_ts.max():.1f}s")

    # ── precompute path/tile panels once per 2s tick (matplotlib is slow; do it ~127x not ~7500x) ──
    ticks = np.arange(t_start, t_end, TICK_S)
    tick_assets = []  # (path_img, tile_img, anyloc_error_m or None, anyloc_queried)
    print(f"[offline] precomputing {len(ticks)} tick panels ...")
    for i, route_t in enumerate(ticks):
        t_unix = T0 + route_t
        gt_hist = [(GXY[j, 0], GXY[j, 1]) for j in range(len(TG)) if TG[j] <= t_unix]
        raw_hist = [(raw_est[j, 0], raw_est[j, 1]) for j in range(len(raw_t)) if raw_t[j] <= route_t]
        fused_hist = [(fused_est[j, 0], fused_est[j, 1]) for j in range(len(fused_t)) if fused_t[j] <= route_t]
        path_img = render_path_panel(route_t, gt_hist, raw_hist, fused_hist)

        ai = int(np.argmin(np.abs(anchor_ts - route_t)))
        anyloc_queried = abs(anchor_ts[ai] - route_t) <= TICK_S / 2
        if anyloc_queried:
            tile_path = os.path.join(DB_DIR, "db_images", f"{anchors[ai]['db_idx']:06d}.jpg")
            anyloc_err = anchors[ai]["error_m"]
        else:
            tile_path = None
            anyloc_err = None
        tile_img = render_tile_panel(tile_path)
        tick_assets.append((path_img, tile_img, anyloc_err, anyloc_queried))
        if (i + 1) % 20 == 0 or i == len(ticks) - 1:
            print(f"  {i+1}/{len(ticks)} tick panels done")

    # ── continuous localizer_error_m interpolation (every video frame, not tick-held) ──
    fused_interp_x = np.interp(np.arange(0, t_end + 1, 0.05), fused_t, fused_est[:, 0])
    fused_interp_y = np.interp(np.arange(0, t_end + 1, 0.05), fused_t, fused_est[:, 1])

    def localizer_error_at(route_t):
        te = np.interp(route_t, fused_t, fused_est[:, 0])
        tn = np.interp(route_t, fused_t, fused_est[:, 1])
        ge = np.interp(T0 + route_t, TG, GXY[:, 0])
        gn = np.interp(T0 + route_t, TG, GXY[:, 1])
        return float(math.hypot(te - ge, tn - gn))

    # ── sweep the REAL video continuously, native framerate, from t_start to t_end ──
    with open(os.path.join(S, "frame_times.csv")) as f:
        frame_times = [(int(r["frame_idx"]), float(r["unix_time"])) for r in csv.DictReader(f)]
    frame_times.sort(key=lambda x: x[1])
    ft_idx = np.array([x[0] for x in frame_times])
    ft_time = np.array([x[1] for x in frame_times])

    start_fi = frame_times[nearest_idx(ft_time, T0 + t_start)][0]
    end_t_unix = T0 + t_end

    cap = cv2.VideoCapture(VIDEO_PATH)
    cap.set(cv2.CAP_PROP_POS_FRAMES, start_fi)
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0

    writer = cv2.VideoWriter(OUT_MP4.replace(".mp4", "_raw.avi"),
                             cv2.VideoWriter_fourcc(*"MJPG"), fps, (PANEL * 2, PANEL * 2))

    print(f"[offline] sweeping real video from frame {start_fi}, native {fps:.1f}fps, "
          f"writing composited frames ...")
    n_written = 0
    fi = start_fi
    while True:
        ok, frame_bgr = cap.read()
        if not ok:
            break
        # cheap running lookup of this frame's real timestamp (frame_times is sorted, fi increases
        # monotonically as we read sequentially -- avoid a fresh searchsorted every frame)
        ti = min(fi - frame_times[0][0], len(ft_time) - 1)
        if ti < 0 or ti >= len(ft_time) or ft_idx[ti] != fi:
            ti = nearest_idx(ft_idx, fi)
        t_unix = ft_time[ti]
        if t_unix > end_t_unix:
            break
        route_t = t_unix - T0

        hi = nearest_idx(TG, t_unix)
        heading = HEAD[hi]
        h, w = frame_bgr.shape[:2]
        Mrot = cv2.getRotationMatrix2D((w // 2, h // 2), -heading, 1.0)
        rot = cv2.warpAffine(frame_bgr, Mrot, (w, h))
        cam_panel = cv2.resize(rot, (PANEL, PANEL))  # BGR, matches writer's BGR expectation

        tick_i = min(max(int((route_t - t_start) / TICK_S), 0), len(tick_assets) - 1)
        path_img, tile_img, anyloc_err, anyloc_queried = tick_assets[tick_i]
        loc_err = localizer_error_at(route_t)
        text_panel = render_text_panel(route_t, anyloc_err, loc_err, anyloc_queried)

        top = np.hstack([path_img[:, :, ::-1], cam_panel])          # path(RGB->BGR), camera(BGR)
        bot = np.hstack([tile_img[:, :, ::-1], text_panel[:, :, ::-1]])
        canvas = np.vstack([top, bot])
        writer.write(canvas)
        n_written += 1
        fi += 1
        if n_written % 500 == 0:
            print(f"  {n_written} video frames written (t={route_t:.1f}s)")

    cap.release()
    writer.release()
    print(f"[offline] {n_written} frames written, re-encoding to H.264 mp4 ...")

    import subprocess
    r = subprocess.run(["ffmpeg", "-y", "-i", OUT_MP4.replace(".mp4", "_raw.avi"),
                        "-c:v", "libx264", "-pix_fmt", "yuv420p", OUT_MP4],
                       capture_output=True, text=True)
    if r.returncode != 0:
        print("[offline] ffmpeg re-encode failed, keeping .avi:\n", r.stderr[-2000:])
    else:
        os.remove(OUT_MP4.replace(".mp4", "_raw.avi"))
        print(f"[offline] postview video -> {OUT_MP4}")


if __name__ == "__main__":
    main()
