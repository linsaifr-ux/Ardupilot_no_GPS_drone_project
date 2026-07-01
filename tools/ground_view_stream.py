#!/usr/bin/env python3
"""
Ground view streamer — composite debug viewport → GStreamer H.265 → network.

Layout (1280×720):
  Left  (640×720)
    ├─ Top    (640×360): live camera with YOLO bounding boxes + drone position
    └─ Bottom (640×360): AnyLoc latest match tile + localizer telemetry
  Right (640×720)
    ├─ Slot 0 (640×240): most recent YOLO detection crop ─┐
    ├─ Slot 1 (640×240): 2nd most recent                  ├ class / conf / lat / lon
    └─ Slot 2 (640×240): 3rd most recent                 ─┘

Subscribes to /drone/camera/image_raw (published by control/launch_camera.sh,
i.e. csi_camera_node.py). Does NOT open the camera directly — run
launch_camera.sh separately (handled automatically by launch_real_hw.sh).

Stream mode A — direct UDP to ground station (ZeroTier / same LAN):
    python3 tools/ground_view_stream.py --host <GS_IP>

    Receive:
        gst-launch-1.0 udpsrc port=5000 ! \\
            application/x-rtp,encoding-name=H265,payload=96 ! \\
            rtph265depay ! h265parse ! avdec_h265 ! \\
            videoconvert ! autovideosink sync=false

Stream mode B — RTSP push to MediaMTX relay server (LTE / internet):
    python3 tools/ground_view_stream.py --stream-server 118.232.160.227

    Watch (no install needed on ground station):
        VLC:     rtsp://118.232.160.227:8554/drone
        Browser: http://118.232.160.227:8889/drone  (WebRTC ~200 ms)
        Browser: http://118.232.160.227:8888/drone  (HLS ~5 s, very reliable)

--host and --stream-server are mutually exclusive.
"""

import argparse
import collections
import json
import os
import sys
import threading
import time

_ROS2_SITE = "/opt/ros/humble/lib/python3.10/site-packages"
if os.path.isdir(_ROS2_SITE) and _ROS2_SITE not in sys.path:
    sys.path.insert(0, _ROS2_SITE)

import cv2
import numpy as np

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst

import rclpy
import rclpy.node
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from vision_msgs.msg import Detection2DArray

# ── Constants ─────────────────────────────────────────────────────────────────

GROUND_IP = os.environ.get("GROUND_IP", "10.181.156.237")
FPS       = 30

STREAM_W  = 1280
STREAM_H  = 720
PANEL_W   = STREAM_W // 2   # 640
PANEL_H   = STREAM_H        # 720
HALF_H    = PANEL_H // 2    # 360  — each left sub-panel height
CROP_H    = PANEL_H // 3    # 240  — each right crop slot height
CROP_IMG_H = CROP_H - 44   # 196  — image area inside each slot

MAX_CROPS = 3

PROJECT_DIR   = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ESTIMATE_JSON = os.path.join(PROJECT_DIR, "anyloc", "latest_estimate.json")
MATCH_JPG     = os.path.join(PROJECT_DIR, "anyloc", "latest_match.jpg")

_SENSOR_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                          durability=DurabilityPolicy.VOLATILE, depth=1)

# ── GStreamer pipelines ───────────────────────────────────────────────────────

_ENC = (
    f'videoconvert ! '
    f'nvvidconv ! video/x-raw(memory:NVMM),format=NV12 ! '
    f'nvv4l2h265enc preset-level=UltraFastPreset '
    f'idrinterval={FPS} iframeinterval={FPS} '
)
_APPSRC = (
    f'appsrc name=src format=time is-live=true block=true '
    f'caps=video/x-raw,format=BGR,width={STREAM_W},height={STREAM_H},framerate={FPS}/1 ! '
)


def _build_udp_pipeline(host: str, port: int, bitrate: int):
    """Mode A — direct RTP/UDP to ground station."""
    pipeline = Gst.parse_launch(
        _APPSRC +
        _ENC + f'bitrate={bitrate} ! '
        f'rtph265pay config-interval=-1 mtu=1200 ! '
        f'udpsink host={host} port={port} sync=false'
    )
    return pipeline, pipeline.get_by_name('src')


def _build_server_pipeline(server: str, rtsp_path: str, bitrate: int):
    """Mode B — RTSP push to MediaMTX relay server via TCP."""
    pipeline = Gst.parse_launch(
        _APPSRC +
        _ENC + f'bitrate={bitrate} ! '
        f'h265parse ! '
        f'rtspclientsink location=rtsp://{server}:8554{rtsp_path} protocols=tcp'
    )
    return pipeline, pipeline.get_by_name('src')

# ── Overlay helper ────────────────────────────────────────────────────────────

def _put(img, lines, x, y, scale=0.5, thickness=1, color=(0, 255, 80)):
    """Draw black-backed text lines starting at (x, y) baseline."""
    lh = int(scale * 28) + 4
    for line in lines:
        (tw, th), _ = cv2.getTextSize(line, cv2.FONT_HERSHEY_SIMPLEX, scale, thickness)
        cv2.rectangle(img, (x - 2, y - th - 3), (x + tw + 3, y + 3), (0, 0, 0), -1)
        cv2.putText(img, line, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, thickness)
        y += lh

# ── File-read caches (avoid 30 Hz disk reads) ────────────────────────────────

_est_cache: dict   = {}
_est_cache_t: float = 0.0
_match_img: np.ndarray | None = None
_match_mtime: float = 0.0


def _read_estimate() -> dict:
    global _est_cache, _est_cache_t
    if time.time() - _est_cache_t < 0.5:
        return _est_cache
    try:
        with open(ESTIMATE_JSON) as f:
            _est_cache = json.load(f)
        _est_cache_t = time.time()
    except Exception:
        pass
    return _est_cache


def _read_match() -> np.ndarray | None:
    global _match_img, _match_mtime
    try:
        mtime = os.path.getmtime(MATCH_JPG)
        if mtime != _match_mtime:
            img = cv2.imread(MATCH_JPG)
            if img is not None:
                _match_img   = img
                _match_mtime = mtime
    except Exception:
        pass
    return _match_img

# ── ROS2 node ─────────────────────────────────────────────────────────────────

class GroundViewNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("ground_view_streamer")
        self._lock = threading.Lock()

        self._lat: float = 0.0
        self._lon: float = 0.0
        self._agl: float = 0.0
        self._latest_bgr: np.ndarray | None = None
        self._latest_bboxes: list[dict]     = []   # from most recent /yolo/detections
        # Deque of detection crop dicts, newest at index 0
        self._crops: collections.deque = collections.deque(maxlen=MAX_CROPS)

        self.create_subscription(Image,           "/drone/camera/image_raw", self._cb_img,  _SENSOR_QOS)
        self.create_subscription(PoseStamped,     "/drone/pose",             self._cb_pose, _SENSOR_QOS)
        self.create_subscription(Float64,         "/drone/agl",              self._cb_agl,  _SENSOR_QOS)
        self.create_subscription(Detection2DArray, "/yolo/detections",       self._cb_det,  1)

    # ── subscribers ───────────────────────────────────────────────────────────

    def _cb_img(self, msg: Image):
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        # csi_camera_node.py publishes rgb8; convert to BGR for OpenCV
        bgr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
        with self._lock:
            self._latest_bgr = bgr

    def _cb_pose(self, msg):
        with self._lock:
            self._lat = msg.pose.position.x
            self._lon = msg.pose.position.y

    def _cb_agl(self, msg):
        with self._lock:
            self._agl = msg.data

    def _cb_det(self, msg):
        with self._lock:
            frame    = self._latest_bgr
            lat, lon = self._lat, self._lon
        if frame is None:
            return

        bboxes     = []
        new_crops  = []
        fh, fw     = frame.shape[:2]

        for det in msg.detections:
            if not det.results:
                continue
            label = det.results[0].hypothesis.class_id
            conf  = det.results[0].hypothesis.score
            cx    = det.bbox.center.position.x
            cy    = det.bbox.center.position.y
            bw    = det.bbox.size_x
            bh    = det.bbox.size_y
            x1 = max(0,  int(cx - bw / 2))
            y1 = max(0,  int(cy - bh / 2))
            x2 = min(fw, int(cx + bw / 2))
            y2 = min(fh, int(cy + bh / 2))
            if x2 <= x1 or y2 <= y1:
                continue
            bboxes.append(dict(x1=x1, y1=y1, x2=x2, y2=y2, label=label, conf=conf))
            new_crops.append(dict(
                img=frame[y1:y2, x1:x2].copy(),
                label=label, conf=conf,
                lat=lat, lon=lon,
                ts=time.time(),
            ))

        with self._lock:
            self._latest_bboxes = bboxes
            for crop in new_crops:
                self._crops.appendleft(crop)

    # ── composite builder ─────────────────────────────────────────────────────

    def build_composite(self) -> np.ndarray:
        with self._lock:
            frame   = self._latest_bgr
            lat     = self._lat
            lon     = self._lon
            agl     = self._agl
            bboxes  = list(self._latest_bboxes)
            crops   = list(self._crops)

        # ── Left-top: YOLO live feed with bounding boxes ──────────────────────
        if frame is not None:
            yolo_panel = cv2.resize(frame, (PANEL_W, HALF_H))
            sx = PANEL_W / frame.shape[1]
            sy = HALF_H  / frame.shape[0]
            for b in bboxes:
                pt1 = (int(b['x1'] * sx), int(b['y1'] * sy))
                pt2 = (int(b['x2'] * sx), int(b['y2'] * sy))
                cv2.rectangle(yolo_panel, pt1, pt2, (0, 255, 0), 2)
                label_y = max(pt1[1] - 4, 14)
                _put(yolo_panel, [f"{b['label']} {b['conf']:.0%}"],
                     pt1[0], label_y, scale=0.44, color=(0, 255, 0))
        else:
            yolo_panel = np.zeros((HALF_H, PANEL_W, 3), dtype=np.uint8)
        n_det = len(bboxes)
        _put(yolo_panel, [
            f"YOLO  {n_det} det   AGL {agl:.0f} m",
            f"LAT {lat:.5f}   LON {lon:.5f}",
        ], 8, 18, scale=0.5)
        cv2.line(yolo_panel, (0, HALF_H - 1), (PANEL_W - 1, HALF_H - 1), (60, 60, 60), 1)

        # ── Left-bottom: AnyLoc match tile ───────────────────────────────────
        match_src = _read_match()
        if match_src is not None:
            anyloc_panel = cv2.resize(match_src, (PANEL_W, HALF_H))
        else:
            anyloc_panel = np.zeros((HALF_H, PANEL_W, 3), dtype=np.uint8)
        est = _read_estimate()
        if est:
            age   = time.time() - est.get("timestamp", 0)
            color = (80, 255, 80) if age <= 5.0 else (80, 80, 255)
            stale = "  STALE" if age > 5.0 else ""
            _put(anyloc_panel, [
                f"ANYLOC  score {est.get('score', 0.0):.3f}{stale}",
                f"LAT {est.get('est_lat', 0.0):.5f}   LON {est.get('est_lon', 0.0):.5f}",
                f"ERR {est.get('error_m', 0.0):.0f} m   age {age:.1f} s",
            ], 8, 18, scale=0.5, color=color)
        else:
            _put(anyloc_panel, ["AnyLoc: waiting for first estimate …"],
                 8, HALF_H // 2, scale=0.5, color=(120, 120, 120))

        left = np.vstack([yolo_panel, anyloc_panel])

        # ── Right: 3 most recent YOLO detection crops ─────────────────────────
        right = np.full((PANEL_H, PANEL_W, 3), 18, dtype=np.uint8)
        for i in range(MAX_CROPS):
            y0 = i * CROP_H
            y1 = y0 + CROP_H
            if i < len(crops):
                d = crops[i]
                # Scale crop to fit PANEL_W × CROP_IMG_H preserving aspect ratio
                ch, cw = d['img'].shape[:2]
                scale_f = min(PANEL_W / max(cw, 1), CROP_IMG_H / max(ch, 1))
                nw = max(1, int(cw * scale_f))
                nh = max(1, int(ch * scale_f))
                patch = np.zeros((CROP_IMG_H, PANEL_W, 3), dtype=np.uint8)
                resized = cv2.resize(d['img'], (nw, nh))
                ox = (PANEL_W - nw) // 2
                oy = (CROP_IMG_H - nh) // 2
                patch[oy:oy+nh, ox:ox+nw] = resized
                right[y0:y0+CROP_IMG_H] = patch
                # Text strip (44 px below the image)
                age_s  = time.time() - d['ts']
                text_y = y0 + CROP_IMG_H + 20
                _put(right, [
                    f"{d['label']}  {d['conf']:.0%}   {age_s:.0f}s ago",
                    f"LAT {d['lat']:.5f}   LON {d['lon']:.5f}",
                ], 6, text_y, scale=0.44, color=(0, 210, 255))
            else:
                _put(right, [f"─── no detection {i + 1} ───"],
                     8, y0 + CROP_H // 2, scale=0.5, color=(60, 60, 60))
            if i < MAX_CROPS - 1:
                cv2.line(right, (0, y1 - 1), (PANEL_W - 1, y1 - 1), (45, 45, 45), 1)

        composite = np.hstack([left, right])
        cv2.line(composite, (PANEL_W, 0), (PANEL_W, STREAM_H - 1), (80, 80, 80), 2)
        return composite

# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description="Ground view streamer (YOLO + AnyLoc → H.265)")
    # Mode A — direct UDP
    ap.add_argument('--host',    default='',
                    help='Ground station IP for direct UDP (mode A)')
    ap.add_argument('--port',    type=int, default=5000)
    # Mode B — RTSP relay server
    ap.add_argument('--stream-server', default='',   metavar='IP',
                    help='MediaMTX relay server IP for RTSP push (mode B)')
    ap.add_argument('--rtsp-path',     default='/drone', metavar='PATH',
                    help='RTSP stream path on server (default: /drone)')
    # Common
    ap.add_argument('--bitrate', type=int, default=1_000_000)
    args = ap.parse_args()

    if args.host and args.stream_server:
        ap.error('--host and --stream-server are mutually exclusive')
    if not args.host and not args.stream_server:
        args.host = GROUND_IP   # default to direct UDP

    Gst.init(None)
    if args.stream_server:
        pipeline, appsrc = _build_server_pipeline(
            args.stream_server, args.rtsp_path, args.bitrate)
    else:
        pipeline, appsrc = _build_udp_pipeline(args.host, args.port, args.bitrate)
    pipeline.set_state(Gst.State.PLAYING)

    rclpy.init()
    node = GroundViewNode()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    if args.stream_server:
        url = f'rtsp://{args.stream_server}:8554{args.rtsp_path}'
        print(f'[stream] /drone/camera/image_raw  →  {STREAM_W}×{STREAM_H} H.265 → {url}')
        print()
        print('Watch on ground station (no install needed):')
        print(f'  VLC:     {url}')
        print(f'  Browser: http://{args.stream_server}:8889{args.rtsp_path}  (WebRTC ~200 ms)')
        print(f'  Browser: http://{args.stream_server}:8888{args.rtsp_path}  (HLS ~5 s)')
    else:
        print(f'[stream] /drone/camera/image_raw  →  {STREAM_W}×{STREAM_H} H.265 → {args.host}:{args.port}')
        print()
        print('Receive on ground station:')
        print(f'  gst-launch-1.0 udpsrc port={args.port} ! \\')
        print( '      application/x-rtp,encoding-name=H265,payload=96 ! \\')
        print( '      rtph265depay ! h265parse ! avdec_h265 ! \\')
        print( '      videoconvert ! autovideosink sync=false')
    print()
    print('[stream] Waiting for /drone/camera/image_raw …')

    frame_interval = 1.0 / FPS
    next_frame_t   = time.monotonic()
    frame_count    = 0
    warned_no_cam  = False

    try:
        while True:
            composite = node.build_composite()

            if frame_count == 0 and node._latest_bgr is None and not warned_no_cam:
                pass  # still waiting — silence until first frame arrives
            elif frame_count == 0 and node._latest_bgr is not None:
                print('[stream] First camera frame received — streaming.')
                warned_no_cam = True

            buf          = Gst.Buffer.new_wrapped(composite.tobytes())
            buf.pts      = frame_count * Gst.SECOND // FPS
            buf.duration = Gst.SECOND // FPS
            flow = appsrc.emit('push-buffer', buf)
            if flow != Gst.FlowReturn.OK:
                print(f'[!] GStreamer pipeline error: {flow}')
                break
            frame_count += 1

            next_frame_t += frame_interval
            sleep_t = next_frame_t - time.monotonic()
            if sleep_t > 0:
                time.sleep(sleep_t)
            else:
                # fell behind; reset to avoid spiral
                next_frame_t = time.monotonic()

    except KeyboardInterrupt:
        print('\n[stream] Stopping …')
    finally:
        appsrc.emit('end-of-stream')
        pipeline.set_state(Gst.State.NULL)
        rclpy.shutdown()
        print('[stream] Done.')


if __name__ == '__main__':
    main()
