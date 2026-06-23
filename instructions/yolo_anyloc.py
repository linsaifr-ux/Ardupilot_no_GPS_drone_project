#!/usr/bin/env python3
"""
Combined YOLO detection + AnyLoc localizer over a single USB camera feed.
Streams a 1280x480 H.265 frame (YOLO left | AnyLoc match right) via RTP/UDP.

  - Every frame   : YOLO car detection on left panel
  - AnyLoc runs in a background thread every N frames; never blocks main loop
  - VO accumulates between AnyLoc anchors

Usage:
    python yolo_anyloc.py [--agl 65] [--camera 0] [--interval 30]

Press Ctrl-C to stop.
"""

import argparse, os, sys, time, threading, queue
import cv2
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst
import numpy as np
import torch
from PIL import Image
from ultralytics import YOLO

HERE   = os.path.dirname(os.path.abspath(__file__))
ANYLOC = os.path.join(HERE, 'anyloc')
DB_DIR = os.path.join(ANYLOC, 'database_vits14')
MODEL  = os.path.join(HERE, 'yolo_test', 'car_v8s_1280', 'weights', 'best.engine')
CLASS_NAMES = {0: 'car'}

sys.path.insert(0, ANYLOC)
from localizer  import AnyLocLocalizer
from vo_refiner import VORefiner

# ── Config ───────────────────────────────────────────────────────────────────
GROUND_IP  = "10.181.156.237"
CAM_W      = 1280
CAM_H      = 720
FPS        = 30
CAM_FOURCC = cv2.VideoWriter_fourcc(*'MJPG')
PANEL_W    = 640
PANEL_H    = 480
STREAM_W   = PANEL_W * 2
STREAM_H   = PANEL_H


# ── Helpers ──────────────────────────────────────────────────────────────────

def draw_text_box(frame, lines, origin=(10, 10), font_scale=0.55, thickness=2):
    x, y = origin
    lh = int(font_scale * 30)
    for line in lines:
        (tw, th), _ = cv2.getTextSize(line, cv2.FONT_HERSHEY_SIMPLEX, font_scale, thickness)
        cv2.rectangle(frame, (x - 2, y - th - 4), (x + tw + 4, y + 4), (0, 0, 0), -1)
        cv2.putText(frame, line, (x, y), cv2.FONT_HERSHEY_SIMPLEX,
                    font_scale, (0, 255, 0), thickness)
        y += lh + 6


def pil_to_bgr(pil_img, w=PANEL_W, h=PANEL_H):
    arr = pil_img.resize((w, h), Image.LANCZOS).convert('RGB')
    t = torch.frombuffer(bytearray(arr.tobytes()), dtype=torch.uint8).reshape(h, w, 3)
    return cv2.cvtColor(t.numpy(), cv2.COLOR_RGB2BGR)


# ── GStreamer ─────────────────────────────────────────────────────────────────

def build_pipeline():
    return Gst.parse_launch(
        f'appsrc name=src format=time is-live=true block=true '
        f'caps=video/x-raw,format=BGR,width={STREAM_W},height={STREAM_H},framerate={FPS}/1 ! '
        f'videoconvert ! '
        f'nvvidconv ! video/x-raw(memory:NVMM),format=NV12 ! '
        f'nvv4l2h265enc bitrate=2000000 preset-level=UltraFastPreset idrinterval=30 iframeinterval=30 ! '
        f'rtph265pay config-interval=-1 mtu=1200 ! '
        f'udpsink host={GROUND_IP} port=5000 sync=false'
    )


# ── AnyLoc background worker ──────────────────────────────────────────────────

class AnyLocWorker:
    """Runs AnyLoc in a daemon thread. Main loop submits frames; never waits."""

    def __init__(self, loc):
        self._loc    = loc
        self._queue  = queue.Queue(maxsize=1)
        self._lock   = threading.Lock()
        self._result = None
        self._busy   = False
        threading.Thread(target=self._run, daemon=True).start()

    def submit(self, pil_img, agl, center_lat, center_lon):
        try:
            self._queue.put_nowait((pil_img, agl, center_lat, center_lon))
        except queue.Full:
            pass  # AnyLoc still running; drop this frame

    @property
    def busy(self):
        return self._busy

    def get_result(self):
        with self._lock:
            return self._result

    def _run(self):
        while True:
            pil_img, agl, clat, clon = self._queue.get()
            self._busy = True
            try:
                t0 = time.perf_counter()
                lat, lon, alt, matched, score, idx = self._loc.localize(
                    pil_img, agl_m=agl,
                    center_lat=clat, center_lon=clon, radius_m=200.0)
                elapsed_ms = (time.perf_counter() - t0) * 1000.0
                with self._lock:
                    self._result = dict(lat=lat, lon=lon, match=matched,
                                        score=score, idx=idx, elapsed_ms=elapsed_ms)
            except Exception as e:
                print(f"[AnyLoc] error: {e}")
            finally:
                self._busy = False


# ── Main ─────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--agl',      type=float, default=65.0)
    ap.add_argument('--camera',   type=int,   default=0)
    ap.add_argument('--interval', type=int,   default=10,
                    help='Submit to AnyLoc every N frames (default: 10)')
    args = ap.parse_args()

    print("[*] Loading YOLO model …")
    yolo = YOLO(MODEL)

    print("[*] Loading AnyLoc database and DINOv2 …")
    loc    = AnyLocLocalizer(DB_DIR)
    vo     = VORefiner(cam_w=CAM_W, cam_h=CAM_H)
    worker = AnyLocWorker(loc)

    Gst.init(None)
    pipeline = build_pipeline()
    appsrc   = pipeline.get_by_name('src')
    pipeline.set_state(Gst.State.PLAYING)
    print(f"[*] Streaming 1280×480 H.265 → {GROUND_IP}:5000")

    cap = cv2.VideoCapture(args.camera, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, CAM_FOURCC)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  CAM_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_H)
    cap.set(cv2.CAP_PROP_FPS, FPS)
    if not cap.isOpened():
        sys.exit(f"[!] Cannot open camera {args.camera}")
    print(f"[*] Camera {args.camera} ready. Press Ctrl-C to stop.")

    frame_count  = 0
    anchor_lat   = None
    anchor_lon   = None
    anchor_score = 0.0
    anchor_idx   = 0
    anchor_ms    = 0.0
    accum_dlat   = 0.0
    accum_dlon   = 0.0
    right_bgr    = np.zeros((PANEL_H, PANEL_W, 3), dtype=np.uint8)
    last_result  = None
    fps_t0       = time.perf_counter()
    fps_val      = 0.0

    try:
        while True:
            ret, bgr = cap.read()
            if not ret:
                print("[!] Camera read failed.")
                break

            raw_pil = Image.fromarray(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))
            frame_count += 1

            # FPS
            if frame_count % 10 == 0:
                fps_val = 10.0 / (time.perf_counter() - fps_t0 + 1e-9)
                fps_t0  = time.perf_counter()

            # ── YOLO ─────────────────────────────────────────────────────────
            results  = yolo(bgr, verbose=False, imgsz=1280)[0]
            left_bgr = cv2.resize(bgr, (PANEL_W, PANEL_H))
            sx, sy   = PANEL_W / CAM_W, PANEL_H / CAM_H
            for box in results.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                x1, x2 = int(x1*sx), int(x2*sx)
                y1, y2 = int(y1*sy), int(y2*sy)
                conf   = float(box.conf[0])
                label  = f"{CLASS_NAMES.get(int(box.cls[0]), int(box.cls[0]))} {conf:.2f}"
                cv2.rectangle(left_bgr, (x1, y1), (x2, y2), (0, 255, 0), 2)
                (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 2)
                ty = max(y1 - 4, th + 4)
                cv2.rectangle(left_bgr, (x1, ty-th-4), (x1+tw+4, ty+2), (0, 0, 0), -1)
                cv2.putText(left_bgr, label, (x1+2, ty),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
            status = "AnyLoc…" if worker.busy else f"FPS: {fps_val:.1f}"
            draw_text_box(left_bgr, [f"Objects: {len(results.boxes)}", status])

            # ── VO (every frame) ─────────────────────────────────────────────
            dlat, dlon, n_vo = vo.update(raw_pil, args.agl, yaw_deg=0.0)
            if anchor_lat is not None:
                accum_dlat += dlat
                accum_dlon += dlon

            # ── Consume fresh AnyLoc result ───────────────────────────────────
            cur = worker.get_result()
            if cur is not None and cur is not last_result:
                last_result  = cur
                anchor_lat   = cur['lat']
                anchor_lon   = cur['lon']
                anchor_score = cur['score']
                anchor_idx   = cur['idx']
                anchor_ms    = cur['elapsed_ms']
                accum_dlat = accum_dlon = 0.0
                vo.reset()
                right_bgr = pil_to_bgr(cur['match'])  # cache once per result

            # ── Submit to AnyLoc every N frames (non-blocking) ───────────────
            if (frame_count == 1) or (frame_count % args.interval == 0):
                clat = (anchor_lat + accum_dlat) if anchor_lat is not None else None
                clon = (anchor_lon + accum_dlon) if anchor_lat is not None else None
                worker.submit(raw_pil.copy(), args.agl, clat, clon)

            # ── Right panel ──────────────────────────────────────────────────
            if anchor_lat is not None:
                final_lat = anchor_lat + accum_dlat
                final_lon = anchor_lon + accum_dlon
                vo_frames = frame_count % args.interval
                mode      = 'ANYLOC' if vo_frames == 0 else f'VO +{vo_frames}f'
                panel     = right_bgr.copy()
                draw_text_box(panel, [
                    f"{mode}  score {anchor_score:.3f}  #{anchor_idx}",
                    f"LAT  {final_lat:.5f} N",
                    f"LON  {final_lon:.5f} E",
                    f"AGL  {args.agl:.1f} m    VO pts {n_vo}    {anchor_ms:.0f} ms",
                ])
            else:
                panel = right_bgr

            # ── Stream ───────────────────────────────────────────────────────
            combined = np.hstack([left_bgr, panel])
            buf = Gst.Buffer.new_wrapped(combined.tobytes())
            buf.pts      = frame_count * Gst.SECOND // FPS
            buf.duration = Gst.SECOND // FPS
            flow = appsrc.emit('push-buffer', buf)
            if flow != Gst.FlowReturn.OK:
                print(f"[!] GStreamer error: {flow}")
                break

    except KeyboardInterrupt:
        print("\n[*] Stopping …")
    finally:
        cap.release()
        appsrc.emit('end-of-stream')
        pipeline.set_state(Gst.State.NULL)
        print("[*] Done.")


if __name__ == '__main__':
    main()
