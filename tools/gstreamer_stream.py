#!/usr/bin/env python3
"""
GStreamer H.265 streamer for drone survey camera.
Structure mirrors yolo_anyloc.py: direct OpenCV camera, combined left|right panels.

  Left  (640×480): live camera + AnyLoc telemetry overlay
  Right (640×480): AnyLoc matched map tile (anyloc/latest_match.jpg) or status text

The AnyLoc ros2_node saves anyloc/latest_match.jpg each time it gets a new match.
Run this script alongside launch_real_hw.sh — it opens the camera independently,
so do NOT also run launch_camera.sh (that would cause "Device busy").

Usage:
    python3 tools/gstreamer_stream.py [--host 10.181.156.237] [--camera 0] [--port 5000]

Receive on ground station:
    gst-launch-1.0 udpsrc port=5000 ! \\
        application/x-rtp,encoding-name=H265,payload=96 ! \\
        rtph265depay ! h265parse ! avdec_h265 ! \\
        videoconvert ! autovideosink sync=false

    # Or VLC: Media → Open Network Stream → rtp://@:5000
"""

import argparse
import json
import os
import sys
import time

import cv2
import numpy as np

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst

# ── Config ───────────────────────────────────────────────────────────────────

GROUND_IP  = os.environ.get("GROUND_IP", "10.181.156.237")
CAM_W      = 1280
CAM_H      = 720
FPS        = 30
CAM_FOURCC = cv2.VideoWriter_fourcc(*'MJPG')
PANEL_W    = 640
PANEL_H    = 480
STREAM_W   = PANEL_W * 2   # 1280
STREAM_H   = PANEL_H       # 480

PROJECT_DIR   = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ESTIMATE_JSON = os.path.join(PROJECT_DIR, "anyloc", "latest_estimate.json")
MATCH_JPG     = os.path.join(PROJECT_DIR, "anyloc", "latest_match.jpg")


# ── GStreamer pipeline ────────────────────────────────────────────────────────

def build_pipeline(host: str, port: int, bitrate: int) -> Gst.Pipeline:
    return Gst.parse_launch(
        f'appsrc name=src format=time is-live=true block=true '
        f'caps=video/x-raw,format=BGR,width={STREAM_W},height={STREAM_H},framerate={FPS}/1 ! '
        f'videoconvert ! '
        f'nvvidconv ! video/x-raw(memory:NVMM),format=NV12 ! '
        f'nvv4l2h265enc bitrate={bitrate} preset-level=UltraFastPreset '
        f'idrinterval={FPS} iframeinterval={FPS} ! '
        f'rtph265pay config-interval=-1 mtu=1200 ! '
        f'udpsink host={host} port={port} sync=false'
    )


# ── Overlay helpers ───────────────────────────────────────────────────────────

def draw_text_box(frame, lines, origin=(10, 10), font_scale=0.55, thickness=2):
    """Black-backed text labels — same style as yolo_anyloc.py."""
    x, y = origin
    lh = int(font_scale * 30)
    for line in lines:
        (tw, th), _ = cv2.getTextSize(line, cv2.FONT_HERSHEY_SIMPLEX, font_scale, thickness)
        cv2.rectangle(frame, (x - 2, y - th - 4), (x + tw + 4, y + 4), (0, 0, 0), -1)
        cv2.putText(frame, line, (x, y), cv2.FONT_HERSHEY_SIMPLEX,
                    font_scale, (0, 255, 0), thickness)
        y += lh + 6


def read_estimate() -> dict:
    try:
        with open(ESTIMATE_JSON) as f:
            return json.load(f)
    except Exception:
        return {}


def load_match_panel(est: dict) -> np.ndarray:
    """Load latest_match.jpg for the right panel; fall back to status text."""
    panel = np.zeros((PANEL_H, PANEL_W, 3), dtype=np.uint8)

    if os.path.exists(MATCH_JPG):
        try:
            img = cv2.imread(MATCH_JPG)
            if img is not None:
                panel = cv2.resize(img, (PANEL_W, PANEL_H))
        except Exception:
            pass

    # Overlay telemetry on the match image (or black panel)
    if est:
        age     = time.time() - est.get("timestamp", 0)
        stale   = "  STALE" if age > 5.0 else ""
        color   = (80, 255, 80) if age <= 5.0 else (80, 80, 255)
        lines   = [
            f"ANYLOC MATCH   score {est.get('score', 0.0):.3f}{stale}",
            f"LAT   {est.get('est_lat', 0.0):.5f} N",
            f"LON   {est.get('est_lon', 0.0):.5f} E",
            f"ALT   {est.get('alt_msl_m', 0.0):.1f} m MSL    AGL {est.get('agl_m', 0.0):.1f} m",
            f"ERR   {est.get('error_m', 0.0):.0f} m    age {age:.1f} s",
        ]
        x, y = 10, 10
        lh = int(0.55 * 30)
        for line in lines:
            (tw, th), _ = cv2.getTextSize(line, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 2)
            cv2.rectangle(panel, (x - 2, y - th - 4), (x + tw + 4, y + 4), (0, 0, 0), -1)
            cv2.putText(panel, line, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
            y += lh + 6
    else:
        draw_text_box(panel, ["AnyLoc: no data yet", "waiting for estimate …"],
                      origin=(10, PANEL_H // 2 - 30))

    return panel


# ── Main ─────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description="Drone camera GStreamer H.265 streamer")
    ap.add_argument('--host',    default=GROUND_IP,
                    help=f'Ground station IP (default: GROUND_IP env or {GROUND_IP})')
    ap.add_argument('--port',    type=int, default=5000, help='UDP port (default: 5000)')
    ap.add_argument('--camera',  type=int, default=0,    help='Camera index (default: 0)')
    ap.add_argument('--bitrate', type=int, default=2_000_000,
                    help='H.265 bitrate bits/s (default: 2000000)')
    args = ap.parse_args()

    Gst.init(None)
    pipeline = build_pipeline(args.host, args.port, args.bitrate)
    appsrc   = pipeline.get_by_name('src')
    pipeline.set_state(Gst.State.PLAYING)
    print(f"[stream] Streaming 1280×480 H.265 → {args.host}:{args.port}")
    print()
    print("Receive on ground station:")
    print(f"  gst-launch-1.0 udpsrc port={args.port} ! \\")
    print( "    application/x-rtp,encoding-name=H265,payload=96 ! \\")
    print( "    rtph265depay ! h265parse ! avdec_h265 ! \\")
    print( "    videoconvert ! autovideosink sync=false")
    print()

    cap = cv2.VideoCapture(args.camera, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, CAM_FOURCC)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  CAM_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_H)
    cap.set(cv2.CAP_PROP_FPS, FPS)
    if not cap.isOpened():
        sys.exit(f"[!] Cannot open camera {args.camera}")
    print(f"[stream] Camera {args.camera} ready  ({CAM_W}×{CAM_H} MJPG). Press Ctrl-C to stop.")

    frame_count = 0
    fps_t0      = time.perf_counter()
    fps_val     = 0.0

    try:
        while True:
            ret, bgr = cap.read()
            if not ret:
                print("[!] Camera read failed — exiting.")
                break

            frame_count += 1

            if frame_count % 30 == 0:
                fps_val = 30.0 / (time.perf_counter() - fps_t0 + 1e-9)
                fps_t0  = time.perf_counter()

            # ── Left panel: camera + telemetry ───────────────────────────────
            left = cv2.resize(bgr, (PANEL_W, PANEL_H))
            est  = read_estimate()

            if est:
                draw_text_box(left, [
                    f"DRONE CAMERA    FPS {fps_val:.1f}",
                    f"LAT   {est.get('est_lat', 0.0):.5f} N",
                    f"LON   {est.get('est_lon', 0.0):.5f} E",
                    f"ALT   {est.get('alt_msl_m', 0.0):.1f} m MSL    AGL {est.get('agl_m', 0.0):.1f} m",
                    f"YAW   {est.get('yaw_deg', 0.0):.1f} deg    score {est.get('score', 0.0):.3f}",
                ])
            else:
                draw_text_box(left, [
                    f"DRONE CAMERA    FPS {fps_val:.1f}",
                    "AnyLoc: waiting for first estimate …",
                ])

            # ── Right panel: AnyLoc matched tile ─────────────────────────────
            right = load_match_panel(est)

            # ── Stream combined 1280×480 ─────────────────────────────────────
            combined = np.hstack([left, right])
            buf = Gst.Buffer.new_wrapped(combined.tobytes())
            buf.pts      = frame_count * Gst.SECOND // FPS
            buf.duration = Gst.SECOND // FPS
            flow = appsrc.emit('push-buffer', buf)
            if flow != Gst.FlowReturn.OK:
                print(f"[!] GStreamer error: {flow}")
                break

    except KeyboardInterrupt:
        print("\n[stream] Stopping …")
    finally:
        cap.release()
        appsrc.emit('end-of-stream')
        pipeline.set_state(Gst.State.NULL)
        print("[stream] Done.")


if __name__ == '__main__':
    main()
