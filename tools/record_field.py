#!/usr/bin/env python3
"""
Field database collection recorder.

Records 2048×1536 30fps H.265 video alongside a telemetry CSV, and
optionally streams a 1280×720 H.265 preview with a telemetry overlay
to a ground station or a MediaMTX relay server.

Video and stream share a single OpenCV capture of /dev/video0
(AP-IMX900-Mini-USB3-I5, USB3/v4l2, YUYV).
Do NOT run launch_camera.sh or any AnyLoc/YOLO node at the same time.

Optionally (--imx219) also records the IMX219 CSI camera concurrently, as a
second, fully independent GStreamer pipeline (nvarguscamerasrc → HW H.265
encode → its own file) — it never touches the Python frame loop above, so it
can't stall the primary recording. The two cameras sit on separate buses
(USB3 vs CSI/MIPI) and separate HW encode sessions; bench-tested 2026-08-13
running both concurrently at full res/fps for 20s: 0 dropped frames on
either side, CPU <25%, GPU 0-27%, no thermal throttling.

Requires MAVROS only — reads GPS/AGL/heading directly from /mavros/global_position/*,
and RC input from /mavros/rc/in (to log which EKF source switch position was active).
hw_bridge.py is not needed.

Usage:
    source /opt/ros/humble/setup.bash
    python3 tools/record_field.py [OPTIONS]

    --output DIR           output directory  (default: field_data/<timestamp>)
    --bitrate BPS          H.265 record bitrate (default: 8000000)
    --duration SECS        stop after N s    (default: 0 = Ctrl+C)

  Second camera (IMX219, CSI) — independent GStreamer pipeline, runs alongside:
    --imx219                   also record the IMX219 CSI camera → video_imx219.mkv
    --imx219-sensor-id N       Argus sensor index (default: 0)
    --imx219-bitrate BPS       H.265 bitrate       (default: 8000000)

  Stream mode A — direct UDP to ground station:
    --stream-host IP       stream preview to this ground station IP
    --stream-port PORT     UDP port          (default: 5000)
    --stream-bitrate BPS   H.265 stream bitrate (default: 1000000)

  Stream mode B — RTSP push to MediaMTX relay server:
    --stream-server IP     push RTSP to this server IP (e.g. 118.232.160.227)
    --stream-rtsp-path P   RTSP stream path  (default: /drone)
    --stream-bitrate BPS   H.265 stream bitrate (default: 1000000)

  Stream mode C — OpenHD (H.264 RTP/UDP, runs ALONGSIDE mode A or B):
    --stream-openhd [IP]   stream the same overlay view as mode A/B (telemetry
                           bar + IMU rate + clock) to the OpenHD ground
                           station (default IP 192.168.2.2 when the flag is
                           given without a value)
    --openhd-port PORT     UDP port     (default: 5601)
    --openhd-bitrate BPS   H.264 bitrate (default: 4000000)

Output files in DIR/:
    video.mkv          H.265, 2048×1536 30fps (MKV — crash-safe)
    telemetry.csv      unix_time, lat, lon, alt_amsl, alt_agl, heading_deg, rc_channels  (5 Hz)
                       rc_channels is the raw /mavros/rc/in PWM list (space-separated) —
                       check the EKF-source switch channel (RCx_OPTION=90) stayed LOW
                       (GPS) throughout if this recording is meant to be GPS ground truth.
    meta.json          video_start_unix, fps, width, height, frame_rotation_deg,
                       achieved IMU/attitude rates (written again at shutdown)
    frame_times.csv    frame_idx, unix_time — actual capture time per frame, logged
                       directly (not reconstructed from fps) so it stays correct even
                       across camera dropouts/reconnects. Prefer this over
                       video_start_unix + frame_idx/fps for any timing-sensitive
                       analysis (e.g. VO drift).
    imu.csv            stamp_ros, recv_unix, wx, wy, wz, ax, ay, az —
                       /mavros/imu/data_raw (FC gyro rad/s + accel m/s², mavros
                       timesync-mapped stamp AND Jetson arrival time). Written by
                       the tools/imu_logger.py sidecar process (in-process logging
                       is GIL-starved to ~100 Hz by the camera loop); it requests
                       RAW_IMU at 200 Hz via SET_MESSAGE_INTERVAL. Achieved rate
                       is shown live and stored in meta.json — OpenVINS-grade VIO
                       needs ≥100 Hz (see instructions/vio_data_collection.md).
    attitude.csv       stamp_ros, recv_unix, qw, qx, qy, qz — fused FC attitude
                       (/mavros/imu/data), for VIO initialization/sanity checks.
    imu_rates.json     live 2 s rate report from the sidecar (imu_hz, att_hz).
    video_imx219.mkv         (--imx219 only) H.265, 1640×1232 30fps, IMX219 CSI camera
    frame_times_imx219.csv   (--imx219 only) frame_idx, unix_time — same as frame_times.csv
                              but for the IMX219 stream. Shares telemetry.csv/imu.csv/
                              attitude.csv with the primary (IMX900) recording — those are
                              per-flight, not per-camera.

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
import subprocess
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
from mavros_msgs.msg import RCIn, HomePosition

from imu_logger import IMU_OK_FRACTION, IMU_REQUEST_HZ   # sidecar (same dir)

Gst.init(None)

# ── Camera / pipeline constants ────────────────────────────────────────────────
REC_W,    REC_H,    FPS  = 2048, 1536, 30
STREAM_W, STREAM_H       = 1280,  720
OVERLAY_H                = 44     # height of black telemetry bar at bottom of stream

# IMX219 (CSI) second-camera recording — full-FOV 2x2-binned mode, matches
# csi_camera_node.py's default (62.2°x48.8° HFOV/VFOV spec used for GSD math).
IMX219_W, IMX219_H = 1640, 1232

# Reconnect backoff for the network stream sinks (RTSP relay / OpenHD) when
# their pipeline errors out (e.g. an LTE drop) — doubles per consecutive
# failure, capped. Mirrors ground_view_stream.py's _reconnect().
RECONNECT_BACKOFF_S     = 2.0
RECONNECT_BACKOFF_MAX_S = 30.0


# ── Telemetry ──────────────────────────────────────────────────────────────────

class TelemetryLogger(Node):
    def __init__(self, csv_path):
        super().__init__('field_recorder')

        self._lock       = threading.Lock()
        self._lat        = None
        self._lon        = None
        self._alt_msl    = None
        self._agl        = None
        self._heading    = None
        self._rc_channels = None   # raw RCIn.channels — includes the EKF-source switch
        self._home_lat   = None    # from /mavros/home_position/home (set at arm)
        self._home_lon   = None

        self._file   = open(csv_path, 'w', newline='')
        self._writer = csv.writer(self._file)
        self._writer.writerow(['unix_time', 'lat', 'lon', 'alt_amsl', 'alt_agl', 'heading_deg', 'rc_channels'])

        self.create_subscription(NavSatFix, '/mavros/global_position/global',    self._cb_gps, qos_profile_sensor_data)
        self.create_subscription(Float64,   '/mavros/global_position/rel_alt',  self._cb_agl, qos_profile_sensor_data)
        self.create_subscription(Float64,   '/mavros/global_position/compass_hdg', self._cb_hdg, qos_profile_sensor_data)
        self.create_subscription(RCIn,      '/mavros/rc/in',                    self._cb_rc,  qos_profile_sensor_data)
        self.create_subscription(HomePosition, '/mavros/home_position/home',    self._cb_home, qos_profile_sensor_data)

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

    def _cb_rc(self, msg: RCIn):
        with self._lock:
            self._rc_channels = list(msg.channels)

    def _cb_home(self, msg: HomePosition):
        with self._lock:
            self._home_lat = msg.geo.latitude
            self._home_lon = msg.geo.longitude

    def _log_row(self):
        with self._lock:
            if self._lat is None:
                return
            rc_str = ' '.join(str(c) for c in self._rc_channels) if self._rc_channels else ''
            self._writer.writerow([
                f'{time.time():.3f}',
                f'{self._lat:.8f}',
                f'{self._lon:.8f}',
                f'{self._alt_msl:.2f}' if self._alt_msl is not None else '',
                f'{self._agl:.2f}'     if self._agl     is not None else '',
                f'{self._heading:.1f}' if self._heading  is not None else '',
                rc_str,
            ])
            self._file.flush()

    def snapshot(self):
        """Return current telemetry values (thread-safe)."""
        with self._lock:
            return (self._lat, self._lon, self._alt_msl, self._agl, self._heading,
                    self._home_lat, self._home_lon)

    def status(self):
        lat, lon, _, agl, hdg, _, _ = self.snapshot()
        if lat is None:
            return 'waiting for GPS …'
        agl_s = f'{agl:.1f} m' if agl is not None else '---'
        hdg_s = f'{hdg:.0f}°'  if hdg is not None else '---'
        return f'lat={lat:.6f}  lon={lon:.6f}  agl={agl_s}  hdg={hdg_s}'

    def close(self):
        self._file.close()


# ── Stream overlay ─────────────────────────────────────────────────────────────

def _crop_resize_stream(bgr_full):
    """Center-crop 4:3 → 16:9 and resize to stream resolution."""
    # Crop vertically to 16:9 before resizing (avoids horizontal stretch)
    src_h, src_w = bgr_full.shape[:2]
    crop_h = src_w * 9 // 16          # e.g. 2048 → 1152
    y0c = (src_h - crop_h) // 2
    return cv2.resize(bgr_full[y0c:y0c + crop_h, :], (STREAM_W, STREAM_H))


def _haversine_m(lat1, lon1, lat2, lon2):
    R = 6371000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * R * math.asin(math.sqrt(a))


def _make_stream_frame(frame, telem, imu_hz=None):
    """Draw the telemetry bar onto a _crop_resize_stream() frame (in place)."""
    lat, lon, _, agl, hdg, home_lat, home_lon = telem

    # Black bar
    y0 = STREAM_H - OVERLAY_H
    cv2.rectangle(frame, (0, y0), (STREAM_W, STREAM_H), (0, 0, 0), -1)

    clock = datetime.now().strftime('%H:%M:%S')
    line1 = (f'LAT {lat:.6f}   LON {lon:.6f}   {clock}'
             if lat is not None else f'GPS: waiting ...   {clock}')
    imu_s = f'{imu_hz:.0f} Hz' if imu_hz is not None else '---'
    if lat is not None and home_lat is not None:
        dist_s = f'{_haversine_m(lat, lon, home_lat, home_lon):.0f} m'
    else:
        dist_s = '---'
    line2 = (f'AGL {agl:.1f} m   HDG {hdg:.0f} deg' if agl is not None and hdg is not None
             else f'AGL {agl:.1f} m' if agl is not None
             else 'AGL ---   HDG ---')
    line2 += f'   HOME {dist_s}   IMU {imu_s}'

    cv2.putText(frame, line1, (10, y0 + 16),
                cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(frame, line2, (10, y0 + 34),
                cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 220, 255), 1, cv2.LINE_AA)

    return frame


# ── Camera helpers ────────────────────────────────────────────────────────────

def _find_camera_device():
    """Find the AP-IMX900 by USB descriptor name instead of assuming
    /dev/video0 — the Jetson's other camera (IMX219, CSI) can be connected
    at the same time and /dev/videoN enumeration order isn't guaranteed
    stable across reboots/replugs. Falls back to /dev/video0 (the prior
    hardcoded default) if v4l2-ctl isn't available or finds no match.
    """
    try:
        out = subprocess.run(['v4l2-ctl', '--list-devices'],
                             capture_output=True, text=True, timeout=5).stdout
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return '/dev/video0'
    found = False
    for line in out.splitlines():
        if 'APPROPHO' in line or 'IMX900' in line:
            found = True
            continue
        if found and '/dev/video' in line:
            return line.strip()
        if found and line and not line[0].isspace():
            found = False
    return '/dev/video0'


def _open_camera(retries=10, delay=2.0):
    """Open the AP-IMX900 at REC_W×REC_H. Returns cap or None after all retries.

    MJPG, not YUYV — this camera only exposes raw YUYV at 640x480/360; at
    REC_W×REC_H it's MJPG/H264-only. OpenCV's V4L2 backend decodes MJPG
    transparently, so this just works like any other fourcc here.
    """
    device = _find_camera_device()
    for attempt in range(retries):
        cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FOURCC,      cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
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
    # atom; a crash leaves the file unplayable. (Fragmented MP4 was tried as
    # an alternative but produces corrupt trun/sample tables with
    # nvv4l2h265enc on this hardware/GStreamer combo — do not switch to it
    # without re-verifying.)
    # idrinterval/iframeinterval=FPS forces one keyframe/second so every
    # ~1 s window has a seek point, matching the stream pipelines below.
    return Gst.parse_launch(
        f'appsrc name=rec format=time is-live=true block=true '
        f'caps=video/x-raw,format=BGR,width={REC_W},height={REC_H},framerate={FPS}/1 ! '
        f'videoconvert ! '
        f'nvvidconv ! video/x-raw(memory:NVMM),format=NV12 ! '
        f'nvv4l2h265enc bitrate={bitrate} idrinterval={FPS} iframeinterval={FPS} ! '
        f'h265parse ! matroskamux ! '
        f'filesink location={video_path}'
    )


def _build_imx219_pipeline(video_path, bitrate, sensor_id):
    """IMX219 CSI camera → HW H.265 encode → file, entirely self-contained
    (nvarguscamerasrc drives its own streaming thread — no appsrc, no Python
    in the per-frame path). flip-method=2 matches the 180° mount-orientation
    correction applied to the primary camera's frames before encode, and to
    every other IMX219 consumer in this codebase (csi_camera_node.py).

    'ts' is an identity element used only to timestamp frames as they pass
    (via its handoff signal) for frame_times_imx219.csv — it does not touch
    the buffer contents, so it costs nothing in the encode path.
    """
    return Gst.parse_launch(
        f'nvarguscamerasrc sensor-id={sensor_id} ! '
        f'video/x-raw(memory:NVMM),width={IMX219_W},height={IMX219_H},'
        f'framerate={FPS}/1,format=NV12 ! '
        f'nvvidconv flip-method=2 ! video/x-raw(memory:NVMM),format=NV12 ! '
        f'identity name=ts signal-handoffs=true ! '
        f'nvv4l2h265enc bitrate={bitrate} idrinterval={FPS} iframeinterval={FPS} ! '
        f'h265parse ! matroskamux ! '
        f'filesink location={video_path}'
    )


# Bound end-to-end latency AND decouple appsrc from downstream stalls: if the
# network sink (UDP/TCP over LTE) can't keep up or hangs entirely (e.g. a
# dropped RTSP TCP connection), drop stale frames here instead of blocking the
# appsrc push — which would otherwise freeze the whole capture loop (including
# local recording) since everything runs on one thread. Without this, a lost
# relay connection hung push-buffer indefinitely (survey43, 2026-08-11).
_NET_QUEUE = ('queue leaky=downstream max-size-buffers=2 max-size-bytes=0 '
              'max-size-time=0 ! ')


def _build_stream_pipeline(host, port, bitrate):
    return Gst.parse_launch(
        f'appsrc name=stream format=time is-live=true block=true '
        f'caps=video/x-raw,format=BGR,width={STREAM_W},height={STREAM_H},framerate={FPS}/1 ! '
        + _NET_QUEUE +
        f'videoconvert ! '
        f'nvvidconv ! video/x-raw(memory:NVMM),format=NV12 ! '
        f'nvv4l2h265enc bitrate={bitrate} preset-level=UltraFastPreset '
        f'idrinterval=30 iframeinterval=30 ! '
        f'rtph265pay config-interval=-1 mtu=1200 ! '
        f'udpsink host={host} port={port} sync=false'
    )


def _build_server_pipeline(server, rtsp_path, bitrate):
    """Push H.265 RTSP stream to a MediaMTX relay server via TCP.

    Requires the `gstreamer1.0-rtsp` apt package (provides rtspclientsink) —
    it's not part of gstreamer1.0-plugins-bad on Ubuntu.
    """
    return Gst.parse_launch(
        f'appsrc name=stream format=time is-live=true block=true '
        f'caps=video/x-raw,format=BGR,width={STREAM_W},height={STREAM_H},framerate={FPS}/1 ! '
        + _NET_QUEUE +
        f'videoconvert ! '
        f'nvvidconv ! video/x-raw(memory:NVMM),format=NV12 ! '
        f'nvv4l2h265enc bitrate={bitrate} preset-level=UltraFastPreset '
        f'idrinterval=30 iframeinterval=30 ! '
        f'h265parse ! '
        f'rtspclientsink location=rtsp://{server}:8554{rtsp_path} protocols=tcp'
    )


def _build_openhd_pipeline(host, port, bitrate):
    """H.264 RTP/UDP for the OpenHD ground station.

    Encoder/payloader settings mirror the field-tested standalone command
    (nvv4l2h264enc bitrate=4000000 control-rate=1 insert-sps-pps=true
    idrinterval=15, rtph264pay config-interval=1 pt=96 mtu=1024) — only the
    source differs: frames come from the shared capture, since the CSI
    camera cannot be opened by two processes.
    """
    return Gst.parse_launch(
        f'appsrc name=openhd format=time is-live=true block=true '
        f'caps=video/x-raw,format=BGR,width={STREAM_W},height={STREAM_H},framerate={FPS}/1 ! '
        + _NET_QUEUE +
        f'videoconvert ! '
        f'nvvidconv ! video/x-raw(memory:NVMM),format=NV12 ! '
        f'nvv4l2h264enc bitrate={bitrate} control-rate=1 insert-sps-pps=true '
        f'idrinterval=15 ! '
        f'h264parse ! rtph264pay config-interval=1 pt=96 mtu=1024 ! '
        f'udpsink host={host} port={port} sync=false'
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
    ap.add_argument('--imu-hz', type=int, default=IMU_REQUEST_HZ,
                    help='RAW_IMU stream rate; 333 = grantable max (~346 Hz '
                         'actual), removes the 400→200 stream-decimation '
                         'aliasing; 400 is DENIED by the FC (diagnosis doc '
                         '§14-3/14-4/14-6)')
    ap.add_argument('--calib', action='store_true',
                    help='tag this recording as a camera-IMU calibration '
                         'session (AprilGrid/checkerboard footage for Kalibr; '
                         'see instructions/vio_data_collection.md)')
    # Second camera — IMX219 (CSI), independent pipeline, runs alongside
    ap.add_argument('--imx219', action='store_true',
                    help='also record the IMX219 CSI camera → video_imx219.mkv '
                         '(separate HW encode session, bench-verified no '
                         'contention with the primary IMX900 recording)')
    ap.add_argument('--imx219-sensor-id', type=int, default=0, metavar='N')
    ap.add_argument('--imx219-bitrate',   type=int, default=8_000_000)
    # Stream mode A — direct UDP to ground station
    ap.add_argument('--stream-host',      default='',    metavar='IP')
    ap.add_argument('--stream-port',      type=int, default=5000)
    # Stream mode B — RTSP push to MediaMTX relay server
    ap.add_argument('--stream-server',    default='',    metavar='IP')
    ap.add_argument('--stream-rtsp-path', default='/drone', metavar='PATH')
    # Shared stream option
    ap.add_argument('--stream-bitrate',   type=int, default=1_000_000)
    # Stream mode C — OpenHD (can run alongside mode A or B)
    ap.add_argument('--stream-openhd',    nargs='?', const='192.168.2.2',
                    default='', metavar='IP')
    ap.add_argument('--openhd-port',      type=int, default=5601)
    ap.add_argument('--openhd-bitrate',   type=int, default=4_000_000)
    args = ap.parse_args()

    if args.stream_host and args.stream_server:
        ap.error('--stream-host and --stream-server are mutually exclusive')

    ts  = datetime.now().strftime('%Y%m%d_%H%M%S')
    out = args.output or os.path.join(
        'field_data', ('calib_' if args.calib else '') + ts)
    os.makedirs(out, exist_ok=True)

    video_path       = os.path.join(out, 'video.mkv')
    telem_path       = os.path.join(out, 'telemetry.csv')
    meta_path        = os.path.join(out, 'meta.json')
    frame_times_path = os.path.join(out, 'frame_times.csv')
    # imu.csv + attitude.csv are written by the imu_logger.py sidecar
    imx219_video_path       = os.path.join(out, 'video_imx219.mkv')
    imx219_frame_times_path = os.path.join(out, 'frame_times_imx219.csv')

    # ── ROS2 telemetry ─────────────────────────────────────────────────────────
    rclpy.init()
    logger = TelemetryLogger(telem_path)
    threading.Thread(target=rclpy.spin, args=(logger,), daemon=True).start()

    # ── High-rate IMU sidecar (separate process) ───────────────────────────────
    # In-process logging tops out at ~60-105 Hz of the 200 Hz stream: the
    # camera/encode loop's GIL contention starves the ROS executor
    # (bench-measured 2026-07-18). A dedicated process sustains 199.8 Hz.
    imu_proc = subprocess.Popen(
        [sys.executable, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                      'imu_logger.py'), '--out', out,
         '--imu-hz', str(args.imu_hz)])
    imu_rates_path = os.path.join(out, 'imu_rates.json')
    imu_ok_hz = args.imu_hz * IMU_OK_FRACTION

    def _imu_status():
        try:
            with open(imu_rates_path) as f:
                r = json.load(f)
            mark = '' if r['imu_hz'] >= imu_ok_hz else '⚠'
            return f"  imu={r['imu_hz']:.0f}Hz{mark}", r
        except (FileNotFoundError, json.JSONDecodeError, KeyError):
            return '  imu=--', None

    # ── Camera ─────────────────────────────────────────────────────────────────
    cap = _open_camera()
    if cap is None:
        rclpy.shutdown()
        device = _find_camera_device()
        holders = subprocess.run(['fuser', device],
                                 capture_output=True, text=True).stdout.strip()
        hint = (f' — PIDs holding {device}: {holders} (kill them first)' if holders else
                ' — no other process holds it; try replugging')
        sys.exit(f'[REC] Cannot open camera ({device}) at {REC_W}×{REC_H}{hint}')

    # ── GStreamer pipelines ────────────────────────────────────────────────────
    rec_pipe = _build_rec_pipeline(video_path, args.bitrate)
    rec_src  = rec_pipe.get_by_name('rec')
    rec_pipe.set_state(Gst.State.PLAYING)

    # ── Second camera (IMX219, CSI) — independent pipeline, its own frame
    # timestamps via the 'ts' identity element's handoff signal (fires on
    # that pipeline's own streaming thread, so it can't block the primary
    # camera loop below).
    imx219_pipe             = None
    imx219_frame_times_file = None
    imx219_frame_idx        = [0]   # mutable cell, mutated from the handoff callback
    if args.imx219:
        imx219_pipe = _build_imx219_pipeline(
            imx219_video_path, args.imx219_bitrate, args.imx219_sensor_id)

        # Wire up frame-time logging BEFORE set_state(PLAYING) — buffers
        # start flowing through 'identity' the moment the pipeline goes
        # PLAYING (well before the error-check wait below returns), and a
        # GObject signal connected later simply misses every handoff that
        # already fired. Connecting late here silently truncated the CSV to
        # ~3s short of the actual encoded frame count (347 rows logged vs
        # 428 real frames in a 14s clip, bench-caught 2026-08-13).
        imx219_frame_times_file = open(imx219_frame_times_path, 'w', newline='')
        imx219_frame_times_writer = csv.writer(imx219_frame_times_file)
        imx219_frame_times_writer.writerow(['frame_idx', 'unix_time'])

        def _on_imx219_frame(_identity, _buf):
            imx219_frame_times_writer.writerow(
                [imx219_frame_idx[0], f'{time.time():.6f}'])
            imx219_frame_times_file.flush()
            imx219_frame_idx[0] += 1

        imx219_pipe.get_by_name('ts').connect('handoff', _on_imx219_frame)
        imx219_pipe.set_state(Gst.State.PLAYING)

        # nvarguscamerasrc reports a bad sensor-id / already-in-use CSI
        # link as an ERROR shortly after PLAYING, not as a failed state
        # change — give it a moment and check the bus before committing to
        # recording it, so a missing/busy second camera degrades to
        # primary-only instead of taking the whole recording down.
        imx219_bus = imx219_pipe.get_bus()
        imx219_startup_msg = imx219_bus.timed_pop_filtered(
            3 * Gst.SECOND, Gst.MessageType.ERROR)
        imx219_failed = imx219_startup_msg is not None
        if not imx219_failed:
            # No ERROR message doesn't guarantee frames are actually
            # flowing — a second concurrent nvarguscamerasrc session on an
            # already-open CSI sensor (e.g. csi_camera_node.py left
            # running) was bench-observed 2026-08-13 to sit PLAYING
            # indefinitely with zero buffers and no error, silently
            # producing a corrupt .mkv. Confirm at least one real frame
            # arrives before trusting it.
            deadline = time.time() + 2.0
            while imx219_frame_idx[0] == 0 and time.time() < deadline:
                time.sleep(0.1)
            imx219_failed = imx219_frame_idx[0] == 0

        if imx219_failed:
            err = (imx219_startup_msg.parse_error()[0].message
                   if imx219_startup_msg is not None else
                   'no frames received — CSI camera busy or not responding '
                   '(check nothing else, e.g. csi_camera_node.py, holds it)')
            print(f'[REC] WARNING: IMX219 camera failed to start ({err}) — '
                  'continuing with primary (IMX900) recording only.')
            imx219_pipe.set_state(Gst.State.NULL)
            imx219_pipe = None
            imx219_frame_times_file.close()
            imx219_frame_times_file = None
            for p in (imx219_frame_times_path, imx219_video_path):
                if os.path.exists(p):
                    os.remove(p)

    stream_pipe = None
    stream_src  = None
    stream_bus  = None
    if args.stream_host:
        stream_pipe = _build_stream_pipeline(
            args.stream_host, args.stream_port, args.stream_bitrate)
        stream_src = stream_pipe.get_by_name('stream')
        stream_pipe.set_state(Gst.State.PLAYING)
        stream_bus = stream_pipe.get_bus()
    elif args.stream_server:
        stream_pipe = _build_server_pipeline(
            args.stream_server, args.stream_rtsp_path, args.stream_bitrate)
        stream_src = stream_pipe.get_by_name('stream')
        stream_pipe.set_state(Gst.State.PLAYING)
        stream_bus = stream_pipe.get_bus()

    openhd_pipe = None
    openhd_src  = None
    openhd_bus  = None
    if args.stream_openhd:
        openhd_pipe = _build_openhd_pipeline(
            args.stream_openhd, args.openhd_port, args.openhd_bitrate)
        openhd_src = openhd_pipe.get_by_name('openhd')
        openhd_pipe.set_state(Gst.State.PLAYING)
        openhd_bus = openhd_pipe.get_bus()

    stream_consec_fail = 0
    stream_ok_count    = 0
    openhd_consec_fail = 0
    openhd_ok_count     = 0

    def _reconnect_stream(reason: str):
        """Rebuild the stream pipeline after its TCP/UDP sink errors out.

        Mirrors ground_view_stream.py's _reconnect(): rtspclientsink doesn't
        recover from a lost TCP connection on its own, leaving the pipeline
        stuck with nothing consuming appsrc's buffers (the leaky queue above
        keeps the frame loop itself unblocked, but the stream stays dead
        until the pipeline is torn down and rebuilt).
        """
        nonlocal stream_pipe, stream_src, stream_bus
        print(f'\n[REC] Stream connection lost ({reason}) — reconnecting …', flush=True)
        try:
            stream_pipe.set_state(Gst.State.NULL)
        except Exception:
            pass
        if args.stream_host:
            stream_pipe = _build_stream_pipeline(
                args.stream_host, args.stream_port, args.stream_bitrate)
        else:
            stream_pipe = _build_server_pipeline(
                args.stream_server, args.stream_rtsp_path, args.stream_bitrate)
        stream_src = stream_pipe.get_by_name('stream')
        stream_pipe.set_state(Gst.State.PLAYING)
        stream_bus = stream_pipe.get_bus()

    def _reconnect_openhd(reason: str):
        nonlocal openhd_pipe, openhd_src, openhd_bus
        print(f'\n[REC] OpenHD connection lost ({reason}) — reconnecting …', flush=True)
        try:
            openhd_pipe.set_state(Gst.State.NULL)
        except Exception:
            pass
        openhd_pipe = _build_openhd_pipeline(
            args.stream_openhd, args.openhd_port, args.openhd_bitrate)
        openhd_src = openhd_pipe.get_by_name('openhd')
        openhd_pipe.set_state(Gst.State.PLAYING)
        openhd_bus = openhd_pipe.get_bus()

    # ── Print header ───────────────────────────────────────────────────────────
    print(f'[REC] Output  → {out}/')
    if args.stream_host:
        print(f'[REC] Stream  → {args.stream_host}:{args.stream_port}  (H.265 RTP/UDP)')
    elif args.stream_server:
        url = f'rtsp://{args.stream_server}:8554{args.stream_rtsp_path}'
        print(f'[REC] Stream  → {url}  (H.265 RTSP push)')
        print(f'[REC] Watch   → http://{args.stream_server}:8889{args.stream_rtsp_path}  (WebRTC browser)')
    if args.stream_openhd:
        print(f'[REC] OpenHD  → {args.stream_openhd}:{args.openhd_port}  (H.264 RTP/UDP)')
    if imx219_pipe is not None:
        print(f'[REC] IMX219  → {imx219_video_path}  ({IMX219_W}x{IMX219_H} H.265)')
    print('[REC] Press Ctrl+C to stop\n')

    video_start = time.time()
    meta = {
        'video_start_unix': video_start,
        'fps': FPS, 'width': REC_W, 'height': REC_H,
        'bitrate': args.bitrate,
        # cv2.rotate(ROTATE_180) is applied before encoding — any camera
        # calibration (Kalibr) must be run on the recorded orientation
        'frame_rotation_deg': 180,
        'purpose': 'calibration' if args.calib else 'survey',
        'imu_requested_hz': args.imu_hz,
        'imx219_enabled': imx219_pipe is not None,
    }
    if imx219_pipe is not None:
        meta['imx219_video']     = os.path.basename(imx219_video_path)
        meta['imx219_width']     = IMX219_W
        meta['imx219_height']    = IMX219_H
        meta['imx219_fps']       = FPS
        meta['imx219_bitrate']   = args.imx219_bitrate
        meta['imx219_sensor_id'] = args.imx219_sensor_id
        # Same 180° correction as the primary camera — applied in-pipeline
        # via nvvidconv flip-method=2 rather than post-hoc.
        meta['imx219_frame_rotation_deg'] = 180
    with open(meta_path, 'w') as f:
        json.dump(meta, f, indent=2)

    # ── Frame loop ─────────────────────────────────────────────────────────────
    stop_event  = threading.Event()
    frame_idx   = 0
    max_frames  = args.duration * FPS if args.duration > 0 else 0
    imu_hz_cached = None    # refreshed 1×/s in the status block, shown in overlay

    # Real per-frame capture time — video_start_unix + frame_idx/fps assumes a
    # perfectly constant frame rate, which breaks silently after any camera
    # dropout (frame_idx isn't incremented during the reconnect gap, so every
    # frame after that point gets an increasingly wrong reconstructed time).
    frame_times_file = open(frame_times_path, 'w', newline='')
    frame_times_writer = csv.writer(frame_times_file)
    frame_times_writer.writerow(['frame_idx', 'unix_time'])

    def _stop(*_):
        stop_event.set()

    import signal
    signal.signal(signal.SIGINT,  _stop)
    signal.signal(signal.SIGTERM, _stop)
    # Closing the terminal window (rather than Ctrl+C) sends SIGHUP, not
    # SIGINT — without trapping it the process dies without emitting EOS,
    # so matroskamux never writes Cues/duration and the file is left
    # unseekable (this is what happened to survey8/survey9).
    signal.signal(signal.SIGHUP,  _stop)

    try:
        while not stop_event.is_set():
            ret, frame = cap.read()
            capture_time = time.time()
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

            frame_times_writer.writerow([frame_idx, f'{capture_time:.6f}'])
            frame_times_file.flush()

            # Push full-res to recording pipeline
            flow = _push(rec_src, frame, frame_idx)
            if flow != Gst.FlowReturn.OK:
                print(f'[REC] Record pipeline error: {flow}')
                break

            # Push the same overlay view (telemetry bar + IMU rate + clock)
            # to every active stream sink. Each sink's appsrc feeds straight
            # into its own leaky queue, so a stalled network write can't
            # block this loop — but the pipeline still needs an explicit
            # rebuild once it errors out, or it stays dark for the rest of
            # the flight.
            if stream_bus is not None:
                msg = stream_bus.timed_pop_filtered(
                    0, Gst.MessageType.ERROR | Gst.MessageType.EOS)
                if msg is not None:
                    reason = ('EOS' if msg.type == Gst.MessageType.EOS
                              else msg.parse_error()[0].message)
                    stream_consec_fail += 1
                    stream_ok_count = 0
                    time.sleep(min(RECONNECT_BACKOFF_S * (2 ** (stream_consec_fail - 1)),
                                   RECONNECT_BACKOFF_MAX_S))
                    _reconnect_stream(reason)

            if openhd_bus is not None:
                msg = openhd_bus.timed_pop_filtered(
                    0, Gst.MessageType.ERROR | Gst.MessageType.EOS)
                if msg is not None:
                    reason = ('EOS' if msg.type == Gst.MessageType.EOS
                              else msg.parse_error()[0].message)
                    openhd_consec_fail += 1
                    openhd_ok_count = 0
                    time.sleep(min(RECONNECT_BACKOFF_S * (2 ** (openhd_consec_fail - 1)),
                                   RECONNECT_BACKOFF_MAX_S))
                    _reconnect_openhd(reason)

            # Unlike the streams above, a dead IMX219 pipeline isn't rebuilt —
            # nvarguscamerasrc/ISP faults aren't the transient network drops
            # the reconnect logic above is designed for, and restarting mid-
            # flight would just fragment the footage across multiple files.
            # Drop it and keep the primary recording going.
            if imx219_pipe is not None:
                msg = imx219_bus.timed_pop_filtered(
                    0, Gst.MessageType.ERROR | Gst.MessageType.EOS)
                if msg is not None:
                    reason = ('EOS' if msg.type == Gst.MessageType.EOS
                              else msg.parse_error()[0].message)
                    print(f'\n[REC] IMX219 pipeline died ({reason}) — dropping '
                          f'it, primary recording continues.', flush=True)
                    imx219_pipe.set_state(Gst.State.NULL)
                    imx219_pipe = None
                    imx219_frame_times_file.close()
                    imx219_frame_times_file = None

            if stream_src is not None or openhd_src is not None:
                view = _crop_resize_stream(frame)
                _make_stream_frame(view, logger.snapshot(), imu_hz_cached)
                if openhd_src is not None:
                    flow = _push(openhd_src, view, frame_idx)
                    if flow != Gst.FlowReturn.OK:
                        openhd_consec_fail += 1
                        openhd_ok_count = 0
                        time.sleep(min(RECONNECT_BACKOFF_S * (2 ** (openhd_consec_fail - 1)),
                                       RECONNECT_BACKOFF_MAX_S))
                        _reconnect_openhd(f'push-buffer returned {flow}')
                    else:
                        openhd_ok_count += 1
                        if openhd_consec_fail and openhd_ok_count >= FPS * 15:
                            openhd_consec_fail = 0
                if stream_src is not None:
                    flow = _push(stream_src, view, frame_idx)
                    if flow != Gst.FlowReturn.OK:
                        stream_consec_fail += 1
                        stream_ok_count = 0
                        time.sleep(min(RECONNECT_BACKOFF_S * (2 ** (stream_consec_fail - 1)),
                                       RECONNECT_BACKOFF_MAX_S))
                        _reconnect_stream(f'push-buffer returned {flow}')
                    else:
                        stream_ok_count += 1
                        if stream_consec_fail and stream_ok_count >= FPS * 15:
                            stream_consec_fail = 0

            frame_idx += 1

            if frame_idx % FPS == 0:
                elapsed = frame_idx // FPS
                imu_s, rates = _imu_status()
                imu_hz_cached = rates['imu_hz'] if rates else None
                print(f'\r[REC] {elapsed:5d}s  {logger.status()}{imu_s}   ',
                      end='', flush=True)

            if max_frames and frame_idx >= max_frames:
                break

    finally:
        frame_times_file.close()
        if cap is not None:
            cap.release()
        rec_src.emit('end-of-stream')
        # Wait for EOS to propagate so matroskamux flushes the final cluster
        # (Cues/duration) before the pipeline goes to NULL.
        rec_bus = rec_pipe.get_bus()
        rec_msg = rec_bus.timed_pop_filtered(10 * Gst.SECOND,
                                             Gst.MessageType.EOS | Gst.MessageType.ERROR)
        if rec_msg is None or rec_msg.type != Gst.MessageType.EOS:
            print('[REC] WARNING: record pipeline did not reach EOS cleanly — '
                  'video.mkv may be missing its seek index (Cues)/duration.')
        rec_pipe.set_state(Gst.State.NULL)
        if stream_pipe:
            stream_src.emit('end-of-stream')
            stream_bus = stream_pipe.get_bus()
            stream_bus.timed_pop_filtered(5 * Gst.SECOND,
                                          Gst.MessageType.EOS | Gst.MessageType.ERROR)
            stream_pipe.set_state(Gst.State.NULL)
        if openhd_pipe:
            openhd_src.emit('end-of-stream')
            openhd_pipe.get_bus().timed_pop_filtered(
                5 * Gst.SECOND, Gst.MessageType.EOS | Gst.MessageType.ERROR)
            openhd_pipe.set_state(Gst.State.NULL)
        if imx219_pipe:
            # No appsrc here — nvarguscamerasrc is a live source, so EOS has
            # to be injected into the pipeline itself rather than emitted by
            # the source element (mirrors the standard GStreamer pattern for
            # stopping a live-source-to-file pipeline cleanly).
            imx219_pipe.send_event(Gst.Event.new_eos())
            imx219_msg = imx219_pipe.get_bus().timed_pop_filtered(
                10 * Gst.SECOND, Gst.MessageType.EOS | Gst.MessageType.ERROR)
            if imx219_msg is None or imx219_msg.type != Gst.MessageType.EOS:
                print('[REC] WARNING: IMX219 pipeline did not reach EOS cleanly — '
                      'video_imx219.mkv may be missing its seek index (Cues)/duration.')
            imx219_pipe.set_state(Gst.State.NULL)
        if imx219_frame_times_file is not None:
            imx219_frame_times_file.close()
        _, rates = _imu_status()
        imu_proc.send_signal(signal.SIGINT)     # sidecar flushes and exits
        try:
            imu_proc.wait(5)
        except subprocess.TimeoutExpired:
            imu_proc.terminate()
        logger.close()
        logger.destroy_node()
        rclpy.shutdown()
        imu_rate = rates['imu_hz'] if rates else 0.0
        meta['imu_achieved_hz_at_stop'] = imu_rate
        meta['att_achieved_hz_at_stop'] = rates['att_hz'] if rates else 0.0
        with open(meta_path, 'w') as f:
            json.dump(meta, f, indent=2)

    print()
    size_mb = os.path.getsize(video_path) / 1e6 if os.path.exists(video_path) else 0
    print(f'[REC] Done — {size_mb:.1f} MB  ({out}/)')
    if meta.get('imx219_enabled') and os.path.exists(imx219_video_path):
        imx219_size_mb = os.path.getsize(imx219_video_path) / 1e6
        print(f'[REC] IMX219 — {imx219_size_mb:.1f} MB  ({imx219_video_path})')
    if imu_rate < imu_ok_hz:
        print(f'[REC] WARNING: IMU rate at stop was {imu_rate:.0f} Hz '
              f'(< {imu_ok_hz:.0f} Hz) — insufficient for OpenVINS-grade VIO. '
              'Check the FC link / SET_MESSAGE_INTERVAL support.')


if __name__ == '__main__':
    main()
