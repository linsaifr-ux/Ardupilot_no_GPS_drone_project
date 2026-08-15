#!/usr/bin/env python3
"""
Ground view streamer — composite debug viewport → GStreamer H.265 → network.

Layout (1280×720):
  Left  (640×720)
    ├─ Top    (640×360): camera with YOLO bounding boxes + drone position
    │                    (always the live frame; boxes are the latest
    │                     detection, so they can trail by ~1 inference during
    │                     fast motion; drop out entirely if YOLO dies)
    └─ Bottom (640×360): AnyLoc latest match tile + localizer telemetry
  Right (640×720)
    ├─ Slot 0 (640×240): most recent YOLO detection crop ─┐
    ├─ Slot 1 (640×240): 2nd most recent                  ├ class / conf / lat / lon
    └─ Slot 2 (640×240): 3rd most recent                 ─┘

Subscribes to /drone/camera/image_raw (published by control/launch_camera.sh,
i.e. usb_camera_node.py). Does NOT open the camera directly — run
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

Stream mode C — OpenHD (H.264 RTP/UDP, runs ALONGSIDE mode A or B):
    python3 tools/ground_view_stream.py --stream-server 118.232.160.227 \\
        --stream-openhd [IP]

    Streams the same composite as mode A/B to an OpenHD ground station
    (default IP 192.168.2.2 when the flag is given without a value). This is
    the same mode C tools/record_field.py already has -- ported here rather
    than shared, since this tool subscribes to the live camera topic instead
    of owning the device, so it can run this alongside anything.

--host and --stream-server are mutually exclusive.

A local copy of the streamed composite is always recorded to
recordings/ground_view_<timestamp>.mkv (same encoded H.265 — no extra GPU
cost; crash-safe streamable MKV). Disable with --no-record; change the
directory with --record-dir.
"""

import argparse
import collections
import json
import os
import sys
import threading
import time
from datetime import datetime

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

# Reconnect backoff when the pipeline errors out (e.g. rtspclientsink losing
# the TCP connection on an LTE drop) — doubles per consecutive failure, capped.
# The backoff SLEEP sits directly in the compositing loop (main() for the
# primary stream, its own loop in _openhd_thread for mode C), so it isn't
# just "wait before retrying the network" -- it fully freezes that leg
# (composite generation, local recording, since they're tee'd off the same
# pipeline as the primary stream) for its entire duration. The old 30s cap
# was tuned to be polite to a struggling connection, but for a live
# monitoring stream that's the wrong tradeoff -- found live (2026-08-14):
# repeated RTSP reconnects under heavy system load produced single freezes
# up to 48s (backoff climbing to the 30s cap plus reconnect/handshake
# overhead on top). This relay is privately controlled (not a rate-limited
# public API), so there's little cost to retrying fast; 5s bounds the worst
# case to something a live viewer can actually tolerate.
RECONNECT_BACKOFF_S     = 1.0
RECONNECT_BACKOFF_MAX_S = 5.0

STREAM_W  = 1280
STREAM_H  = 720
PANEL_W   = STREAM_W // 2   # 640
PANEL_H   = STREAM_H        # 720
HALF_H    = PANEL_H // 2    # 360  — each left sub-panel height
CROP_H    = PANEL_H // 3    # 240  — each right crop slot height
CROP_IMG_H = CROP_H - 44   # 196  — image area inside each slot

MAX_CROPS = 3

# Shadow-mode fused/GPS XY track history for the bottom-left panel. Sampled
# at whatever rate _read_shadow_estimate()'s own file-cache actually turns
# over (2 Hz, see _shadow_cache_t below) rather than fusion_live_node's
# 20 Hz write rate -- plenty dense for a survey-speed drone. 4000 points at
# 2 Hz is ~33 min of flight; trivial memory (two floats each).
TRACK_MAXLEN = 4000

# Recent camera frames kept for stamp-matching against /yolo/detections
# (whose header is copied from the source image). YOLO inference is ~60 ms,
# so detections arrive ~2-4 frames after their source at 30 fps; 12 frames
# (~0.4 s) of slack covers scheduling hiccups. ~3.7 MB per 1280×960 frame.
FRAME_BUF_LEN = 12
# If no detections message (even an empty one arrives per processed frame)
# for this long, YOLO is considered down → show the live feed, no boxes.
DET_STALE_S = 2.0

PROJECT_DIR   = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ESTIMATE_JSON = os.path.join(PROJECT_DIR, "anyloc", "latest_estimate.json")
MATCH_JPG     = os.path.join(PROJECT_DIR, "anyloc", "latest_match.jpg")
# Written by vio_vpe/fusion_live_node.py -- the shadow-mode VIO+VPE observer
# (see ~/.claude/plans/robust-squishing-forest.md). That stack never publishes
# to the flight controller; this is read-only, display-only, same as the
# AnyLoc estimate above.
SHADOW_ESTIMATE_JSON = os.path.join(PROJECT_DIR, "vio_vpe", "latest_estimate.json")

_SENSOR_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                          durability=DurabilityPolicy.VOLATILE, depth=1)

# ── GStreamer pipelines ───────────────────────────────────────────────────────

# _ENC/_APPSRC/_OPENHD_APPSRC are functions, not module-level strings, because
# they embed FPS -- and FPS is only known once main() parses --fps, which
# happens after module import. A frozen f-string here would silently keep
# declaring "framerate=30/1" in the GStreamer caps even after --fps changed
# the actual push rate, a real mismatch (found while adding --fps, before it
# ever shipped).
def _enc():
    return (
        f'videoconvert ! '
        f'nvvidconv ! video/x-raw(memory:NVMM),format=NV12 ! '
        f'nvv4l2h265enc preset-level=UltraFastPreset '
        f'idrinterval={FPS} iframeinterval={FPS} '
    )


def _appsrc():
    return (
        f'appsrc name=src format=time is-live=true block=true '
        f'caps=video/x-raw,format=BGR,width={STREAM_W},height={STREAM_H},framerate={FPS}/1 ! '
        # Bound end-to-end latency: if the network sink (TCP push over LTE) can't
        # keep up, drop stale frames here instead of blocking appsrc and letting
        # delay grow unbounded with no catch-up.
        f'queue leaky=downstream max-size-buffers=2 max-size-bytes=0 max-size-time=0 ! '
    )


def _rec_branch(record_path: str) -> str:
    """tee branch writing the encoded H.265 to a local MKV (no re-encode).

    matroskamux streamable=true writes cluster-by-cluster with no end-of-file
    index seek, so the file stays playable after a crash / power loss
    mid-flight — same rationale as record_field.py's video.mkv.
    """
    return (f' t. ! queue ! h265parse ! matroskamux streamable=true ! '
            f'filesink location="{record_path}" sync=false')


def _build_udp_pipeline(host: str, port: int, bitrate: int, record_path: str = ''):
    """Mode A — direct RTP/UDP to ground station (+ optional local MKV)."""
    net = (f'rtph265pay config-interval=-1 mtu=1200 ! '
           f'udpsink host={host} port={port} sync=false')
    if record_path:
        desc = (_appsrc() + _enc() + f'bitrate={bitrate} vbv-size={bitrate} ! tee name=t '
                f't. ! queue ! ' + net + _rec_branch(record_path))
    else:
        desc = _appsrc() + _enc() + f'bitrate={bitrate} vbv-size={bitrate} ! ' + net
    pipeline = Gst.parse_launch(desc)
    return pipeline, pipeline.get_by_name('src')


def _build_server_pipeline(server: str, rtsp_path: str, bitrate: int,
                           record_path: str = ''):
    """Mode B — RTSP push to MediaMTX relay server via TCP (+ optional local MKV).

    Requires the `gstreamer1.0-rtsp` apt package (provides rtspclientsink) —
    it's not part of gstreamer1.0-plugins-bad on Ubuntu.
    """
    net = (f'h265parse ! '
           f'rtspclientsink location=rtsp://{server}:8554{rtsp_path} protocols=tcp')
    if record_path:
        desc = (_appsrc() + _enc() + f'bitrate={bitrate} vbv-size={bitrate} ! tee name=t '
                f't. ! queue ! ' + net + _rec_branch(record_path))
    else:
        desc = _appsrc() + _enc() + f'bitrate={bitrate} vbv-size={bitrate} ! ' + net
    pipeline = Gst.parse_launch(desc)
    return pipeline, pipeline.get_by_name('src')


def _build_pipeline(args, record_path: str):
    """Build the appsrc → encode → [net, rec] pipeline for the selected mode."""
    if args.stream_server:
        return _build_server_pipeline(args.stream_server, args.rtsp_path,
                                       args.bitrate, record_path)
    return _build_udp_pipeline(args.host, args.port, args.bitrate, record_path)


def _openhd_appsrc():
    return (
        f'appsrc name=openhd format=time is-live=true block=true '
        f'caps=video/x-raw,format=BGR,width={STREAM_W},height={STREAM_H},framerate={FPS}/1 ! '
        f'queue leaky=downstream max-size-buffers=2 max-size-bytes=0 max-size-time=0 ! '
    )


def _build_openhd_pipeline(host: str, port: int, bitrate: int):
    """Mode C — H.264 RTP/UDP to an OpenHD ground station, runs ALONGSIDE
    mode A/B. Encoder/payloader settings match tools/record_field.py's
    field-tested OpenHD pipeline (_build_openhd_pipeline there) -- ported
    rather than imported since that one owns the camera device directly and
    this one subscribes to the topic instead. idrinterval=15 here is
    deliberately independent of FPS (OpenHD's own keyframe-cadence
    convention, matches record_field.py) -- only unlike _enc()'s
    idrinterval={FPS}, not a bug.
    """
    desc = (_openhd_appsrc() +
            f'videoconvert ! '
            f'nvvidconv ! video/x-raw(memory:NVMM),format=NV12 ! '
            f'nvv4l2h264enc bitrate={bitrate} control-rate=1 insert-sps-pps=true '
            f'idrinterval=15 ! '
            f'h264parse ! rtph264pay config-interval=1 pt=96 mtu=1024 ! '
            f'udpsink host={host} port={port} sync=false')
    pipeline = Gst.parse_launch(desc)
    return pipeline, pipeline.get_by_name('openhd')


class _LatestFrame:
    """Single-slot handoff between the main compositing loop and the OpenHD
    thread. Deliberately drop-old (not a queue) -- OpenHD only ever wants the
    newest composite, same intent as the leaky queue inside each pipeline."""

    def __init__(self):
        self._lock = threading.Lock()
        self._frame = None

    def set(self, frame):
        with self._lock:
            self._frame = frame

    def get(self):
        with self._lock:
            return self._frame


def _openhd_thread(host, port, bitrate, latest_frame, stop_event):
    """Owns the OpenHD pipeline end to end, on its own thread with its own
    frame pacing, reading only the latest composite via _LatestFrame.

    This exists because appsrc.emit('push-buffer', ...) is a BLOCKING call
    (block=true) -- an earlier version pushed to this pipeline from inside
    the main loop, right after the primary stream's own blocking push. Any
    stall on this leg (OpenHD ground station unreachable, or two simultaneous
    hardware encode sessions -- H.265 for mode A/B, H.264 here -- contending
    on the same Jetson) delayed every subsequent primary-stream push too,
    since they were sequential on one thread: found live as "the stream
    lags until Ctrl+C, then jumps" -- the jump was the backlog finally
    flushing on shutdown. Full isolation is the fix, not a tighter timeout:
    a stall here must be able to persist indefinitely without the primary
    stream ever noticing.
    """
    pipeline, appsrc = _build_openhd_pipeline(host, port, bitrate)
    pipeline.set_state(Gst.State.PLAYING)
    bus = pipeline.get_bus()
    print(f'[stream] OpenHD (mode C) → {host}:{port}  (H.264 RTP/UDP)')

    def reconnect(reason):
        nonlocal pipeline, appsrc, bus, stream_start_t, last_pts, last_reconnect_t
        print(f'[stream] OpenHD connection lost ({reason}) — reconnecting …')
        try:
            pipeline.set_state(Gst.State.NULL)
        except Exception:
            pass
        # The REBUILD can fail too (e.g. the ground station's port still
        # refusing right after a reboot, or the NVENC session not fully
        # released yet from the pipeline just torn down above) -- this must
        # not escape. This is a daemon thread with no supervisor: an
        # uncaught exception here silently kills it, and the OpenHD leg
        # would stay dead until the whole program is restarted, even once
        # the ground station comes back. Found live: a ground-station
        # reboot did exactly that. Leaving pipeline/appsrc/bus pointed at
        # the old (already-NULL) pipeline on failure means the next
        # push-buffer call fails immediately too, which re-enters
        # reconnect() on the next loop iteration -- so this keeps retrying
        # at the same backoff cadence instead of dying outright.
        try:
            pipeline, appsrc = _build_openhd_pipeline(host, port, bitrate)
            pipeline.set_state(Gst.State.PLAYING)
            bus = pipeline.get_bus()
            stream_start_t = time.monotonic()
            last_pts = 0
        except Exception as e:
            print(f'[stream] OpenHD pipeline rebuild failed ({e}) — will retry')
        last_reconnect_t = time.monotonic()

    # Same wall-clock-PTS rationale as the primary loop in main() -- see its
    # comment. A nominal frame_count*interval PTS here would let this leg's
    # lag grow independently of and in addition to the primary stream's.
    stream_start_t = time.monotonic()
    last_pts = 0
    frame_interval = 1.0 / FPS
    next_frame_t = time.monotonic()
    frame_count = 0
    consec_fail = 0
    last_reconnect_t = time.monotonic()

    while not stop_event.is_set():
        msg = bus.timed_pop_filtered(0, Gst.MessageType.ERROR | Gst.MessageType.EOS)
        if msg is not None:
            reason = ('EOS' if msg.type == Gst.MessageType.EOS
                      else msg.parse_error()[0].message)
            consec_fail += 1
            backoff_s = min(RECONNECT_BACKOFF_S * (2 ** (consec_fail - 1)),
                             RECONNECT_BACKOFF_MAX_S)
            time.sleep(backoff_s)
            reconnect(reason)
            frame_count = 0
            next_frame_t = time.monotonic()
            continue

        frame = latest_frame.get()
        if frame is not None:
            pts = int((time.monotonic() - stream_start_t) * Gst.SECOND)
            buf = Gst.Buffer.new_wrapped(frame.tobytes())
            buf.pts = pts
            buf.duration = max(pts - last_pts, 1)
            flow = appsrc.emit('push-buffer', buf)
            if flow != Gst.FlowReturn.OK:
                consec_fail += 1
                backoff_s = min(RECONNECT_BACKOFF_S * (2 ** (consec_fail - 1)),
                                 RECONNECT_BACKOFF_MAX_S)
                time.sleep(backoff_s)
                reconnect(f'push-buffer returned {flow}')
                frame_count = 0
                next_frame_t = time.monotonic()
                continue
            last_pts = pts
            frame_count += 1
            if consec_fail and time.monotonic() - last_reconnect_t >= 15.0:
                consec_fail = 0

        next_frame_t += frame_interval
        sleep_t = next_frame_t - time.monotonic()
        if sleep_t > 0:
            time.sleep(sleep_t)
        else:
            next_frame_t = time.monotonic()

    try:
        appsrc.emit('end-of-stream')
        bus.timed_pop_filtered(2 * Gst.SECOND, Gst.MessageType.EOS | Gst.MessageType.ERROR)
        pipeline.set_state(Gst.State.NULL)
    except Exception:
        pass


def _new_record_path(args) -> str:
    """Fresh timestamped MKV path, or '' if recording is disabled."""
    if args.no_record:
        return ''
    os.makedirs(args.record_dir, exist_ok=True)
    return os.path.join(args.record_dir,
                         f"ground_view_{time.strftime('%Y%m%d_%H%M%S')}.mkv")

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
_shadow_cache: dict   = {}
_shadow_cache_t: float = 0.0
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


def _read_shadow_estimate() -> dict:
    """Same poll-a-JSON-file pattern as _read_estimate(), for the VIO+VPE
    shadow-mode observer instead of AnyLoc."""
    global _shadow_cache, _shadow_cache_t
    if time.time() - _shadow_cache_t < 0.5:
        return _shadow_cache
    try:
        with open(SHADOW_ESTIMATE_JSON) as f:
            _shadow_cache = json.load(f)
        _shadow_cache_t = time.time()
    except Exception:
        pass
    return _shadow_cache


_SCALE_BAR_NICE_M = [1, 2, 5, 10, 20, 50, 100, 200, 500, 1000, 2000, 5000]


def _draw_track_panel(fused_track, gps_track, sh: dict, age_s) -> np.ndarray:
    """Bottom-left shadow-mode panel: live 2D XY plot of the fused VIO+VPE
    path (green) against the GPS ground-truth path (white), both in the same
    imx900_geo_common site ENU frame fusion_live_node.py computes them in.
    Equal east/north scale (no distortion) with a plain meter scale bar
    instead of numeric axes -- this is a shape/drift-at-a-glance panel, not a
    precision one; the per-source numbers (yaw, vpe fix age, err vio/vpe --
    previously the whole panel, before this became a graph) are overlaid in
    the top-right corner instead."""
    panel = np.zeros((HALF_H, PANEL_W, 3), dtype=np.uint8)
    pts = list(fused_track) + list(gps_track)
    if len(pts) < 2:
        _put(panel, ["collecting VIO+VPE / GPS track ..."],
             8, HALF_H // 2, scale=0.5, color=(120, 120, 120))
        return panel

    es = [p[0] for p in pts]
    ns = [p[1] for p in pts]
    e_min, e_max = min(es), max(es)
    n_min, n_max = min(ns), max(ns)
    e_range = max(e_max - e_min, 1.0)
    n_range = max(n_max - n_min, 1.0)

    margin_l, margin_r, margin_t, margin_b = 10, 10, 46, 10
    usable_w = PANEL_W - margin_l - margin_r
    usable_h = HALF_H - margin_t - margin_b
    scale = min(usable_w / e_range, usable_h / n_range)
    plot_w, plot_h = e_range * scale, n_range * scale
    ox = margin_l + (usable_w - plot_w) / 2
    oy = margin_t + (usable_h - plot_h) / 2

    def _px(pt):
        e, n = pt
        return (int(ox + (e - e_min) * scale), int(oy + (n_max - n) * scale))

    cv2.rectangle(panel, (margin_l, margin_t), (PANEL_W - margin_r, HALF_H - margin_b),
                  (50, 50, 50), 1)

    if len(gps_track) >= 2:
        cv2.polylines(panel, [np.array([_px(p) for p in gps_track], dtype=np.int32)],
                      False, (200, 200, 200), 2, cv2.LINE_AA)
    if gps_track:
        cv2.circle(panel, _px(gps_track[-1]), 5, (255, 255, 255), -1)

    if len(fused_track) >= 2:
        cv2.polylines(panel, [np.array([_px(p) for p in fused_track], dtype=np.int32)],
                      False, (0, 255, 0), 2, cv2.LINE_AA)
    if fused_track:
        cv2.circle(panel, _px(fused_track[-1]), 5, (0, 255, 0), -1)

    # Scale bar: nearest "nice" round meter value to ~1/4 of the plot width.
    target_m = (usable_w * 0.25) / scale
    nice_m = min(_SCALE_BAR_NICE_M, key=lambda v: abs(v - target_m))
    bar_px = max(1, int(nice_m * scale))
    bx0, by = PANEL_W - margin_r - bar_px - 10, HALF_H - margin_b - 8
    cv2.line(panel, (bx0, by), (bx0 + bar_px, by), (150, 150, 150), 2)
    _put(panel, [f"{nice_m} m"], bx0, by - 6, scale=0.4, thickness=1, color=(150, 150, 150))

    def _fmt_err(v):
        return f"{v:.0f} m" if v is not None else "--"

    # Kept short (<330 px @ this scale) so it can't run into the stats block
    # on the right -- verified against cv2.getTextSize, not eyeballed.
    _put(panel, [
        "SHADOW (not published to FC): fused vs GPS",
        f"err fused {_fmt_err(sh.get('err_fused_m'))}   age {age_s:.1f}s   n={len(fused_track)}",
    ], 8, 18, scale=0.45, color=(0, 255, 80))

    yaw = sh.get("yaw_deg")
    yaw_txt = (f"{yaw:.0f} deg" if sh.get("yaw_resolved") and yaw is not None
               else "unresolved")
    _put(panel, [
        f"yaw {yaw_txt}  vpe age {sh.get('vpe_age_s', 0.0) or 0.0:.1f}s",
        f"err vio {_fmt_err(sh.get('err_vio_m'))}  err vpe {_fmt_err(sh.get('err_vpe_m'))}",
    ], PANEL_W - 230, 18, scale=0.4, color=(80, 200, 255))

    return panel


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
        # Recent frames as (stamp_key, bgr) for pairing detections with the
        # exact frame they were computed on -- used only for the crop
        # thumbnails (see _cb_det), which are static per-detection snapshots
        # where source accuracy matters. The main video panel draws boxes on
        # the live frame instead (see build_composite).
        self._frame_buf: collections.deque = collections.deque(maxlen=FRAME_BUF_LEN)
        self._det_time: float = 0.0                # wall time of last detections msg
        # Deque of detection crop dicts, newest at index 0
        self._crops: collections.deque = collections.deque(maxlen=MAX_CROPS)

        # Shadow-mode XY track history (see _draw_track_panel) -- built up
        # here, not in fusion_live_node.py, since it's purely a display
        # concern; fusion_live_node.py only ever needs to write its latest
        # single estimate.
        self._shadow_track_fused: collections.deque = collections.deque(maxlen=TRACK_MAXLEN)
        self._shadow_track_gps: collections.deque = collections.deque(maxlen=TRACK_MAXLEN)
        self._shadow_track_last_t: float | None = None

        self.create_subscription(Image,           "/drone/camera/image_raw", self._cb_img,  _SENSOR_QOS)
        self.create_subscription(PoseStamped,     "/drone/pose",             self._cb_pose, _SENSOR_QOS)
        self.create_subscription(Float64,         "/drone/agl",              self._cb_agl,  _SENSOR_QOS)
        self.create_subscription(Detection2DArray, "/yolo/detections",       self._cb_det,  1)

    # ── subscribers ───────────────────────────────────────────────────────────

    def _cb_img(self, msg: Image):
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        # usb_camera_node.py publishes rgb8; convert to BGR for OpenCV
        bgr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
        key = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        with self._lock:
            self._latest_bgr = bgr
            self._frame_buf.append((key, bgr))

    def _cb_pose(self, msg):
        with self._lock:
            self._lat = msg.pose.position.x
            self._lon = msg.pose.position.y

    def _cb_agl(self, msg):
        with self._lock:
            self._agl = msg.data

    def _cb_det(self, msg):
        # The detector copies the source image's header, so the stamp tells us
        # exactly which buffered frame these boxes were computed on. Drawing on
        # that frame (not the newest one) keeps boxes glued to their objects —
        # the panel just trails live by one YOLO inference (~60 ms).
        key = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        with self._lock:
            frame = next((f for k, f in self._frame_buf if k == key),
                         self._latest_bgr)
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

            # Crop window: square, 50% larger than the bbox's longer side so
            # the thumbnail shows surrounding context instead of a tight cut.
            side = min(max(bw, bh) * 1.5, fw, fh)
            half = side / 2
            wx1, wy1, wx2, wy2 = cx - half, cy - half, cx + half, cy + half
            if wx1 < 0:
                wx2 -= wx1; wx1 = 0
            if wy1 < 0:
                wy2 -= wy1; wy1 = 0
            if wx2 > fw:
                wx1 -= (wx2 - fw); wx2 = fw
            if wy2 > fh:
                wy1 -= (wy2 - fh); wy2 = fh
            wx1, wy1 = max(0, int(wx1)), max(0, int(wy1))
            wx2, wy2 = min(fw, int(wx2)), min(fh, int(wy2))
            if wx2 <= wx1 or wy2 <= wy1:
                continue

            crop_img = frame[wy1:wy2, wx1:wx2].copy()
            # Mark the actual detection box within the crop, same style as
            # the live YOLO panel.
            bx1, by1 = x1 - wx1, y1 - wy1
            bx2, by2 = x2 - wx1, y2 - wy1
            cv2.rectangle(crop_img, (bx1, by1), (bx2, by2), (0, 255, 0), 2)
            _put(crop_img, [f"{label} {conf:.0%}"],
                 bx1, max(by1 - 4, 18), scale=0.7, thickness=2, color=(0, 255, 0))

            new_crops.append(dict(
                img=crop_img,
                label=label, conf=conf,
                lat=lat, lon=lon,
                ts=time.time(),
            ))

        with self._lock:
            self._latest_bboxes = bboxes
            self._det_time  = time.time()
            for crop in new_crops:
                self._crops.appendleft(crop)

    # ── composite builder ─────────────────────────────────────────────────────

    def build_composite(self) -> np.ndarray:
        with self._lock:
            frame     = self._latest_bgr
            lat       = self._lat
            lon       = self._lon
            agl       = self._agl
            bboxes    = list(self._latest_bboxes)
            crops     = list(self._crops)
            det_age   = time.time() - self._det_time

        # ── Left-top: YOLO feed with bounding boxes ───────────────────────────
        # Always the live frame -- smooth 30fps video, never freezes/jumps
        # when a new detection lands. Boxes are the most recent
        # /yolo/detections result drawn straight onto it, so during fast
        # motion they can trail the live frame by ~1 YOLO inference (~60ms,
        # 2-4 frames @30fps). Traded deliberately: this panel is cosmetic
        # (ground-crew situational awareness only, not the localization
        # path), so smooth video wins over pixel-perfect box registration.
        # Was previously stamp-matched to the exact source frame instead
        # (see ground_view_bbox_sync_fix.md) -- that kept boxes glued to
        # their objects but made the panel visibly step/freeze between
        # detections since it only updated at YOLO's inference rate.
        det_fresh = det_age <= DET_STALE_S
        if frame is not None:
            yolo_panel = cv2.resize(frame, (PANEL_W, HALF_H))
            sx = PANEL_W / frame.shape[1]
            sy = HALF_H  / frame.shape[0]
            if det_fresh:
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
        status = f"YOLO  {n_det} det" if det_fresh else "YOLO  STALE (no boxes)"
        clock = datetime.now().strftime('%H:%M:%S')
        _put(yolo_panel, [
            f"{status}   AGL {agl:.0f} m",
            f"LAT {lat:.5f}   LON {lon:.5f}   {clock}",
        ], 8, 18, scale=0.5,
             color=(0, 255, 80) if det_fresh else (80, 80, 255))
        cv2.line(yolo_panel, (0, HALF_H - 1), (PANEL_W - 1, HALF_H - 1), (60, 60, 60), 1)

        # ── Left-bottom: localizer panel -- AnyLoc match tile OR the
        # shadow-mode VIO+VPE observer, whichever is actually producing
        # fresh data right now. These two never run together
        # (control/launch_real_hw.sh starts anyloc/ros2_node.py;
        # vio_vpe/launch_shadow_mode.sh starts fusion_live_node.py instead
        # and never touches anyloc/ at all), so showing AnyLoc's match crop
        # unconditionally would mean displaying a stale image left over from
        # a past AnyLoc run during a shadow-mode flight -- actively
        # misleading, not just an empty panel. Pick whichever source has a
        # fix newer than 5s (matches the STALE threshold each source already
        # uses for its own age color-coding).
        est = _read_estimate()
        anyloc_age = (time.time() - est.get("timestamp", 0)) if est else 1e9
        sh = _read_shadow_estimate()
        shadow_age = (time.time() - sh.get("t", 0)) if sh else 1e9

        # Accumulate the fused/GPS XY track for _draw_track_panel. Runs
        # whenever a new sample shows up (dedup by fusion_live_node's own
        # 't'), independent of which panel branch is active below, so the
        # track survives a brief anyloc_age<=5 blip mid shadow-mode run.
        # t going backwards (not just repeating) means fusion_live_node.py
        # restarted -- a fresh flight, not a continuation -- so drop the old
        # track instead of drawing a line across the gap between them.
        t = sh.get("t")
        if t is not None and t != self._shadow_track_last_t:
            if self._shadow_track_last_t is not None and t < self._shadow_track_last_t - 1.0:
                self._shadow_track_fused.clear()
                self._shadow_track_gps.clear()
            self._shadow_track_last_t = t
            fe, fn = sh.get("fused_east"), sh.get("fused_north")
            if fe is not None and fn is not None:
                self._shadow_track_fused.append((fe, fn))
            ge, gn = sh.get("gps_east"), sh.get("gps_north")
            if ge is not None and gn is not None:
                self._shadow_track_gps.append((ge, gn))

        if anyloc_age <= 5.0:
            match_src = _read_match()
            anyloc_panel = (cv2.resize(match_src, (PANEL_W, HALF_H)) if match_src is not None
                            else np.zeros((HALF_H, PANEL_W, 3), dtype=np.uint8))
            _put(anyloc_panel, [
                f"ANYLOC  score {est.get('score', 0.0):.3f}",
                f"LAT {est.get('est_lat', 0.0):.5f}   LON {est.get('est_lon', 0.0):.5f}",
                f"ERR {est.get('error_m', 0.0):.0f} m   age {anyloc_age:.1f} s",
            ], 8, 18, scale=0.5, color=(80, 255, 80))
        elif shadow_age <= 5.0:
            anyloc_panel = _draw_track_panel(
                self._shadow_track_fused, self._shadow_track_gps,
                sh, shadow_age)
        else:
            anyloc_panel = np.zeros((HALF_H, PANEL_W, 3), dtype=np.uint8)
            _put(anyloc_panel, ["waiting for AnyLoc or shadow-mode estimate …"],
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
    global FPS
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
    # Mode C — OpenHD, runs alongside mode A or B
    ap.add_argument('--stream-openhd', nargs='?', const='192.168.2.2',
                    default='', metavar='IP',
                    help='Also stream (mode C) to an OpenHD ground station, '
                         'default IP 192.168.2.2 when given without a value '
                         '-- same convention as record_field.py')
    ap.add_argument('--openhd-port',    type=int, default=5601)
    ap.add_argument('--openhd-bitrate', type=int, default=4_000_000)
    # Common
    ap.add_argument('--bitrate', type=int, default=1_000_000)
    ap.add_argument('--fps', type=int, default=FPS, metavar='N',
                    help=f'Compositing/encode frame rate for BOTH the primary '
                         f'stream and OpenHD (default {FPS}). Lower this before '
                         f'reaching for --no-yolo/--no-openhd on a CPU-tight '
                         f'run (e.g. alongside vio_vpe/) -- build_composite() '
                         f'runs once per frame at this rate and is real, '
                         f'measurable CPU cost (found live: 30fps here + YOLO '
                         f'+ the vio_vpe stack together saturated an 8-core '
                         f'Orin NX, load avg ~9, stream falling ~50s behind '
                         f'real time).')
    ap.add_argument('--record-dir', default=os.path.join(PROJECT_DIR, 'recordings'),
                    metavar='DIR',
                    help='Directory for the local MKV copy of the stream '
                         '(default: <project>/recordings)')
    ap.add_argument('--no-record', action='store_true',
                    help='Disable the local recording (stream only)')
    args = ap.parse_args()

    if args.host and args.stream_server:
        ap.error('--host and --stream-server are mutually exclusive')
    if not args.host and not args.stream_server:
        args.host = GROUND_IP   # default to direct UDP

    FPS = args.fps

    record_path = _new_record_path(args)

    Gst.init(None)
    pipeline, appsrc = _build_pipeline(args, record_path)
    pipeline.set_state(Gst.State.PLAYING)
    bus = pipeline.get_bus()

    # Mode C runs on its own thread with its own pacing -- see _openhd_thread's
    # docstring for why sharing the main loop's blocking push caused the
    # primary stream to lag until the process was killed.
    openhd_latest_frame = _LatestFrame()
    openhd_stop = threading.Event()
    openhd_thread = None
    if args.stream_openhd:
        openhd_thread = threading.Thread(
            target=_openhd_thread,
            args=(args.stream_openhd, args.openhd_port, args.openhd_bitrate,
                  openhd_latest_frame, openhd_stop),
            daemon=True)
        openhd_thread.start()

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
    if record_path:
        print(f'[stream] Recording local copy → {record_path}')
    # (mode C's own "OpenHD (mode C) → ..." line prints from its thread,
    # asynchronously, once that pipeline is actually up)
    print()
    print('[stream] Waiting for /drone/camera/image_raw …')

    def _reconnect(reason: str):
        """Tear down the broken pipeline and rebuild it from scratch.

        rtspclientsink doesn't recover from a lost TCP connection on its own
        (e.g. an LTE drop) — it leaves the pipeline stuck in an error state
        forever with nothing consuming appsrc's buffers. Rebuilding is the
        simplest robust fix; the local recording starts a fresh segment
        rather than trying to splice back into the old (now orphaned) file.
        """
        nonlocal pipeline, appsrc, bus, record_path, stream_start_t, last_pts
        print(f'[stream] Connection lost ({reason}) — reconnecting …')
        try:
            pipeline.set_state(Gst.State.NULL)
        except Exception:
            pass
        # The REBUILD can fail too, not just the teardown above -- e.g. the
        # relay/ground station's port still refusing right after it reboots.
        # This runs inside main()'s try/except KeyboardInterrupt block, which
        # does NOT catch anything else: an uncaught exception here falls
        # through to the `finally:` cleanup and kills the WHOLE program,
        # OpenHD leg included, even though that leg was fine. Same live
        # finding as _openhd_thread's reconnect(). Leaving pipeline/appsrc/
        # bus pointed at the old (already-NULL) pipeline on failure means
        # the next push-buffer call fails immediately too, re-entering
        # _reconnect() on the next loop iteration -- keeps retrying at the
        # same backoff cadence instead of taking the process down.
        try:
            record_path = _new_record_path(args)
            pipeline, appsrc = _build_pipeline(args, record_path)
            pipeline.set_state(Gst.State.PLAYING)
            bus = pipeline.get_bus()
            stream_start_t = time.monotonic()   # fresh pipeline, fresh timeline
            last_pts = 0
            if record_path:
                print(f'[stream] Recording local copy → {record_path}')
        except Exception as e:
            print(f'[stream] Pipeline rebuild failed ({e}) — will retry')

    # PTS is derived from the wall clock (time.monotonic() - stream_start_t),
    # NOT from frame_count * frame_interval. That distinction is the actual
    # fix for lag that grows without bound: a fixed-rate PTS counter has no
    # way to represent "this iteration ran long" except by falling further
    # behind real time forever (the pacing loop below resets its OWN schedule
    # after an overrun so it doesn't spiral into a catch-up burst, but that
    # only stops things from getting WORSE -- it never reclaims time already
    # lost, because nothing tied PTS to real elapsed time in the first
    # place). Deriving PTS from the wall clock instead means an overrun
    # iteration just produces one frame with a bigger PTS jump than usual --
    # a momentary framerate dip, not a permanent, accumulating delay. This
    # is also just the documented-correct way to timestamp a live appsrc
    # source (is-live=true) in the first place, not a special-case patch.
    stream_start_t = time.monotonic()
    last_pts       = 0
    frame_interval = 1.0 / FPS
    next_frame_t   = time.monotonic()
    frame_count    = 0
    warned_no_cam  = False
    consec_fail    = 0
    last_reconnect_t = time.monotonic()

    try:
        while True:
            bus_msg = bus.timed_pop_filtered(0, Gst.MessageType.ERROR | Gst.MessageType.EOS)
            if bus_msg is not None:
                reason = ('EOS' if bus_msg.type == Gst.MessageType.EOS
                          else bus_msg.parse_error()[0].message)
                consec_fail += 1
                backoff_s = min(RECONNECT_BACKOFF_S * (2 ** (consec_fail - 1)),
                                 RECONNECT_BACKOFF_MAX_S)
                time.sleep(backoff_s)
                _reconnect(reason)
                frame_count  = 0
                next_frame_t = time.monotonic()
                last_reconnect_t = time.monotonic()
                continue

            composite = node.build_composite()

            if frame_count == 0 and node._latest_bgr is None and not warned_no_cam:
                pass  # still waiting — silence until first frame arrives
            elif frame_count == 0 and node._latest_bgr is not None:
                print('[stream] First camera frame received — streaming.')
                warned_no_cam = True

            pts = int((time.monotonic() - stream_start_t) * Gst.SECOND)
            buf          = Gst.Buffer.new_wrapped(composite.tobytes())
            buf.pts      = pts
            buf.duration = max(pts - last_pts, 1)
            flow = appsrc.emit('push-buffer', buf)
            if flow != Gst.FlowReturn.OK:
                consec_fail += 1
                backoff_s = min(RECONNECT_BACKOFF_S * (2 ** (consec_fail - 1)),
                                 RECONNECT_BACKOFF_MAX_S)
                time.sleep(backoff_s)
                _reconnect(f'push-buffer returned {flow}')
                frame_count  = 0
                next_frame_t = time.monotonic()
                last_reconnect_t = time.monotonic()
                continue
            last_pts     = pts
            frame_count += 1

            # Hand the OpenHD thread the newest composite -- cheap (a lock +
            # reference swap), never blocks, and that thread paces/pushes on
            # its own schedule entirely independent of this loop.
            if openhd_thread is not None:
                openhd_latest_frame.set(composite)

            # 15s of clean streaming since the last reconnect → forgive past
            # failures so a later, unrelated blip doesn't inherit a long
            # backoff. Wall-clock based (not frame_count == FPS*15) since the
            # actual achieved framerate can vary -- frame_count would no
            # longer reliably mean "15 real seconds" once it does.
            if consec_fail and time.monotonic() - last_reconnect_t >= 15.0:
                consec_fail = 0

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
        # Let the muxer flush its last cluster before tearing down — the MKV
        # is streamable (playable even without this), but this avoids losing
        # the final ~1 s.
        bus.timed_pop_filtered(2 * Gst.SECOND,
                               Gst.MessageType.EOS | Gst.MessageType.ERROR)
        pipeline.set_state(Gst.State.NULL)
        if openhd_thread is not None:
            openhd_stop.set()
            openhd_thread.join(timeout=3.0)
        # see vio_vpe/fusion_live_node.py's identical guard: rclpy's own
        # SIGINT handler (spin_thread runs rclpy.spin(node)) may already have
        # shut the context down by the time this runs.
        if rclpy.ok():
            rclpy.shutdown()
        if record_path:
            print(f'[stream] Local recording saved → {record_path}')
        print('[stream] Done.')


if __name__ == '__main__':
    main()
