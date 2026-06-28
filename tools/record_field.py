#!/usr/bin/env python3
"""
Field database collection recorder.

Records 2048×1536 30fps H.264 video alongside a telemetry CSV, and
optionally streams a 1280×720 H.265 preview with a telemetry overlay
to a ground station or a MediaMTX relay server.

Video and stream share a single OpenCV capture of /dev/video0.
Do NOT run launch_camera.sh or any AnyLoc/YOLO node at the same time.

Requires MAVROS + hw_bridge.py (publishes /drone/agl and /drone/pose).

Usage:
    source /opt/ros/humble/setup.bash
    python3 tools/record_field.py [OPTIONS]

    --output DIR           output directory  (default: field_data/<timestamp>)
    --bitrate BPS          H.264 record bitrate (default: 8000000)
    --duration SECS        stop after N s    (default: 0 = Ctrl+C)

  Stream mode A — direct UDP to ground station:
    --stream-host IP       stream preview to this ground station IP
    --stream-port PORT     UDP port          (default: 5000)
    --stream-bitrate BPS   H.265 stream bitrate (default: 1000000)

  Stream mode B — RTSP push to MediaMTX relay server:
    --stream-server IP     push RTSP to this server IP (e.g. 118.232.160.227)
    --stream-rtsp-path P   RTSP stream path  (default: /drone)
    --stream-bitrate BPS   H.265 stream bitrate (default: 1000000)

Output files in DIR/:
    video.mkv          H.264, 2048×1536 30fps (MKV — crash-safe)
    telemetry.csv      unix_time, lat, lon, alt_amsl, alt_agl, heading_deg  (5 Hz)
    meta.json          video_start_unix, fps, width, height

Stream mode A receiver (ground station):
    gst-launch-1.0 udpsrc port=5000 ! \\
      application/x-rtp,encoding-name=H265,payload=96 ! \\
      rtph265depay ! h265parse ! avdec_h265 ! \\
      videoconvert ! autovideosink sync=false

Stream mode B viewers (MediaMTX server):
    VLC:     rtsp://118.232.160.227:8554/drone
    Browser: http://118.232.160.227:8889/drone  (WebRTC, ~200 ms)
    Browser: http://118.232.160.227:8888/drone  (HLS, ~5 s, mobile-friendly)

Post-processing:
    python3 tools/extract_frames.py field_data/<timestamp>/
"""

import argparse
import csv
import json
import math
import os
import sys
import threading
import time
from datetime import datetime

import cv2
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64

Gst.init(None)

# ── Camera / pipeline constants ────────────────────────────────────────────────
REC_W,    REC_H,    FPS  = 2048, 1536, 30
STREAM_W, STREAM_H       = 1280,  720
OVERLAY_H                = 44     # height of black telemetry bar at bottom of stream


# ── Telemetry ──────────────────────────────────────────────────────────────────

class TelemetryLogger(Node):
    def __init__(self, csv_path):
        super().__init__('field_recorder')

        self._lock    = threading.Lock()
        self._lat     = None
        self._lon     = None
        self._alt_msl = None
        self._agl     = None
        self._heading = None

        self._file   = open(csv_path, 'w', newline='')
        self._writer = csv.writer(self._file)
        self._writer.writerow(['unix_time', 'lat', 'lon', 'alt_amsl', 'alt_agl', 'heading_deg'])

        self.create_subscription(NavSatFix, '/mavros/global_position/global',    self._cb_gps, qos_profile_sensor_data)
        self.create_subscription(Float64,   '/mavros/global_position/rel_alt',  self._cb_agl, qos_profile_sensor_data)
        self.create_subscription(Float64,   '/mavros/global_position/compass_hdg', self._cb_hdg, qos_profile_sensor_data)

        self.create_timer(0.2, self._log_row)

    def _cb_gps(self, msg: NavSatFix):
        with self._lock:
            self._lat     = msg.latitude
            self._lon     = msg.longitude
            self._alt_msl = msg.altitude

    def _cb_agl(self, msg: Float64):
        with self._lock:
            self._agl = msg.data

    def _cb_hdg(self, msg: Float64):
        with self._lock:
            self._heading = msg.data

    def _log_row(self):
        with self._lock:
            if self._lat is None:
                return
            self._writer.writerow([
                f'{time.time():.3f}',
                f'{self._lat:.8f}',
                f'{self._lon:.8f}',
                f'{self._alt_msl:.2f}' if self._alt_msl is not None else '',
                f'{self._agl:.2f}'     if self._agl     is not None else '',
                f'{self._heading:.1f}' if self._heading  is not None else '',
            ])
            self._file.flush()

    def snapshot(self):
        """Return current telemetry values (thread-safe)."""
        with self._lock:
            return (self._lat, self._lon, self._alt_msl, self._agl, self._heading)

    def status(self):
        lat, lon, _, agl, hdg = self.snapshot()
        if lat is None:
            return 'waiting for GPS …'
        agl_s = f'{agl:.1f} m' if agl is not None else '---'
        hdg_s = f'{hdg:.0f}°'  if hdg is not None else '---'
        return f'lat={lat:.6f}  lon={lon:.6f}  agl={agl_s}  hdg={hdg_s}'

    def close(self):
        self._file.close()


# ── Stream overlay ─────────────────────────────────────────────────────────────

def _make_stream_frame(bgr_full, telem):
    """Resize to stream resolution and draw telemetry bar at bottom."""
    lat, lon, _, agl, hdg = telem

    frame = cv2.resize(bgr_full, (STREAM_W, STREAM_H))

    # Black bar
    y0 = STREAM_H - OVERLAY_H
    cv2.rectangle(frame, (0, y0), (STREAM_W, STREAM_H), (0, 0, 0), -1)

    line1 = f'LAT {lat:.6f}   LON {lon:.6f}' if lat is not None else 'GPS: waiting ...'
    line2 = (f'AGL {agl:.1f} m   HDG {hdg:.0f} deg' if agl is not None and hdg is not None
             else f'AGL {agl:.1f} m' if agl is not None
             else 'AGL ---   HDG ---')

    cv2.putText(frame, line1, (10, y0 + 16),
                cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(frame, line2, (10, y0 + 34),
                cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 220, 255), 1, cv2.LINE_AA)

    return frame


# ── Camera helpers ────────────────────────────────────────────────────────────

def _open_camera(retries=10, delay=2.0):
    """Open /dev/video0 at REC_W×REC_H. Returns cap or None after all retries."""
    for attempt in range(retries):
        cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FOURCC,      cv2.VideoWriter_fourcc('Y', 'U', 'Y', 'V'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  REC_W)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, REC_H)
        cap.set(cv2.CAP_PROP_FPS,          FPS)
        if cap.isOpened():
            w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            if w == REC_W and h == REC_H:
                return cap
        cap.release()
        if attempt < retries - 1:
            time.sleep(delay)
    return None


# ── GStreamer pipeline builders ────────────────────────────────────────────────

def _build_rec_pipeline(video_path, bitrate):
    # matroskamux writes clusters incrementally — the file stays playable even
    # after a hard power-off.  mp4mux requires a clean EOS to write the moov
    # atom; a crash leaves the file unplayable.
    return Gst.parse_launch(
        f'appsrc name=rec format=time is-live=true block=true '
        f'caps=video/x-raw,format=BGR,width={REC_W},height={REC_H},framerate={FPS}/1 ! '
        f'videoconvert ! '
        f'nvvidconv ! video/x-raw(memory:NVMM),format=NV12 ! '
        f'nvv4l2h264enc bitrate={bitrate} ! '
        f'h264parse ! matroskamux ! '
        f'filesink location={video_path}'
    )


def _build_stream_pipeline(host, port, bitrate):
    return Gst.parse_launch(
        f'appsrc name=stream format=time is-live=true block=true '
        f'caps=video/x-raw,format=BGR,width={STREAM_W},height={STREAM_H},framerate={FPS}/1 ! '
        f'videoconvert ! '
        f'nvvidconv ! video/x-raw(memory:NVMM),format=NV12 ! '
        f'nvv4l2h265enc bitrate={bitrate} preset-level=UltraFastPreset '
        f'idrinterval=30 iframeinterval=30 ! '
        f'rtph265pay config-interval=-1 mtu=1200 ! '
        f'udpsink host={host} port={port} sync=false'
    )


def _build_server_pipeline(server, rtsp_path, bitrate):
    """Push H.265 RTSP stream to a MediaMTX relay server via TCP."""
    return Gst.parse_launch(
        f'appsrc name=stream format=time is-live=true block=true '
        f'caps=video/x-raw,format=BGR,width={STREAM_W},height={STREAM_H},framerate={FPS}/1 ! '
        f'videoconvert ! '
        f'nvvidconv ! video/x-raw(memory:NVMM),format=NV12 ! '
        f'nvv4l2h265enc bitrate={bitrate} preset-level=UltraFastPreset '
        f'idrinterval=30 iframeinterval=30 ! '
        f'h265parse ! '
        f'rtspclientsink location=rtsp://{server}:8554{rtsp_path} protocols=tcp'
    )


def _push(appsrc, frame_bgr, frame_idx):
    buf = Gst.Buffer.new_wrapped(frame_bgr.tobytes())
    buf.pts      = frame_idx * Gst.SECOND // FPS
    buf.duration = Gst.SECOND // FPS
    return appsrc.emit('push-buffer', buf)


# ── Main ───────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--output',           default='')
    ap.add_argument('--bitrate',          type=int, default=8_000_000)
    ap.add_argument('--duration',         type=int, default=0)
    # Stream mode A — direct UDP to ground station
    ap.add_argument('--stream-host',      default='',    metavar='IP')
    ap.add_argument('--stream-port',      type=int, default=5000)
    # Stream mode B — RTSP push to MediaMTX relay server
    ap.add_argument('--stream-server',    default='',    metavar='IP')
    ap.add_argument('--stream-rtsp-path', default='/drone', metavar='PATH')
    # Shared stream option
    ap.add_argument('--stream-bitrate',   type=int, default=1_000_000)
    args = ap.parse_args()

    if args.stream_host and args.stream_server:
        ap.error('--stream-host and --stream-server are mutually exclusive')

    ts  = datetime.now().strftime('%Y%m%d_%H%M%S')
    out = args.output or os.path.join('field_data', ts)
    os.makedirs(out, exist_ok=True)

    video_path = os.path.join(out, 'video.mkv')
    telem_path = os.path.join(out, 'telemetry.csv')
    meta_path  = os.path.join(out, 'meta.json')

    # ── ROS2 telemetry ─────────────────────────────────────────────────────────
    rclpy.init()
    logger = TelemetryLogger(telem_path)
    threading.Thread(target=rclpy.spin, args=(logger,), daemon=True).start()

    # ── Camera ─────────────────────────────────────────────────────────────────
    cap = _open_camera()
    if cap is None:
        rclpy.shutdown()
        import subprocess
        holders = subprocess.run(['fuser', '/dev/video0'],
                                 capture_output=True, text=True).stdout.strip()
        hint = (f' — PIDs holding /dev/video0: {holders} (kill them first)'
                if holders else ' — no other process holds it; try replugging')
        sys.exit(f'[REC] Cannot open camera at {REC_W}×{REC_H}{hint}')

    # ── GStreamer pipelines ────────────────────────────────────────────────────
    rec_pipe = _build_rec_pipeline(video_path, args.bitrate)
    rec_src  = rec_pipe.get_by_name('rec')
    rec_pipe.set_state(Gst.State.PLAYING)

    stream_pipe = None
    stream_src  = None
    if args.stream_host:
        stream_pipe = _build_stream_pipeline(
            args.stream_host, args.stream_port, args.stream_bitrate)
        stream_src = stream_pipe.get_by_name('stream')
        stream_pipe.set_state(Gst.State.PLAYING)
    elif args.stream_server:
        stream_pipe = _build_server_pipeline(
            args.stream_server, args.stream_rtsp_path, args.stream_bitrate)
        stream_src = stream_pipe.get_by_name('stream')
        stream_pipe.set_state(Gst.State.PLAYING)

    # ── Print header ───────────────────────────────────────────────────────────
    print(f'[REC] Output  → {out}/')
    if args.stream_host:
        print(f'[REC] Stream  → {args.stream_host}:{args.stream_port}  (H.265 RTP/UDP)')
    elif args.stream_server:
        url = f'rtsp://{args.stream_server}:8554{args.stream_rtsp_path}'
        print(f'[REC] Stream  → {url}  (H.265 RTSP push)')
        print(f'[REC] Watch   → http://{args.stream_server}:8889{args.stream_rtsp_path}  (WebRTC browser)')
    print('[REC] Press Ctrl+C to stop\n')

    video_start = time.time()
    with open(meta_path, 'w') as f:
        json.dump({
            'video_start_unix': video_start,
            'fps': FPS, 'width': REC_W, 'height': REC_H,
            'bitrate': args.bitrate,
        }, f, indent=2)

    # ── Frame loop ─────────────────────────────────────────────────────────────
    stop_event  = threading.Event()
    frame_idx   = 0
    max_frames  = args.duration * FPS if args.duration > 0 else 0

    def _stop(*_):
        stop_event.set()

    import signal
    signal.signal(signal.SIGINT,  _stop)
    signal.signal(signal.SIGTERM, _stop)

    try:
        while not stop_event.is_set():
            ret, frame = cap.read()
            if not ret:
                print('\n[REC] Camera dropout — reconnecting...', flush=True)
                cap.release()
                cap = _open_camera()
                if cap is None:
                    print('[REC] Camera could not be recovered — stopping')
                    break
                print('[REC] Camera reconnected', flush=True)
                continue

            frame = cv2.rotate(frame, cv2.ROTATE_180)

            # Push full-res to recording pipeline
            flow = _push(rec_src, frame, frame_idx)
            if flow != Gst.FlowReturn.OK:
                print(f'[REC] Record pipeline error: {flow}')
                break

            # Push overlay frame to stream pipeline
            if stream_src is not None:
                telem = logger.snapshot()
                stream_frame = _make_stream_frame(frame, telem)
                _push(stream_src, stream_frame, frame_idx)

            frame_idx += 1

            if frame_idx % FPS == 0:
                elapsed = frame_idx // FPS
                print(f'\r[REC] {elapsed:5d}s  {logger.status()}   ', end='', flush=True)

            if max_frames and frame_idx >= max_frames:
                break

    finally:
        if cap is not None:
            cap.release()
        rec_src.emit('end-of-stream')
        # Wait for EOS to propagate so matroskamux flushes the final cluster
        # before the pipeline goes to NULL.
        rec_bus = rec_pipe.get_bus()
        rec_bus.timed_pop_filtered(10 * Gst.SECOND,
                                   Gst.MessageType.EOS | Gst.MessageType.ERROR)
        rec_pipe.set_state(Gst.State.NULL)
        if stream_pipe:
            stream_src.emit('end-of-stream')
            stream_bus = stream_pipe.get_bus()
            stream_bus.timed_pop_filtered(5 * Gst.SECOND,
                                          Gst.MessageType.EOS | Gst.MessageType.ERROR)
            stream_pipe.set_state(Gst.State.NULL)
        logger.close()
        logger.destroy_node()
        rclpy.shutdown()

    print()
    size_mb = os.path.getsize(video_path) / 1e6 if os.path.exists(video_path) else 0
    print(f'[REC] Done — {size_mb:.1f} MB  ({out}/)')


if __name__ == '__main__':
    main()
