#!/usr/bin/env python3
"""
Field database collection recorder.

Records 2048×1536 30fps H.264 video alongside a telemetry CSV containing
GPS position, AGL, and compass heading — all synchronized by Unix timestamp.

Requires MAVROS + ardupilot_commander running (they publish /drone/agl and
/drone/pose).  The camera must be on /dev/video0.

Usage:
    source /opt/ros/humble/setup.bash
    python3 tools/record_field.py [OPTIONS]

    --output DIR       output directory  (default: field_data/<timestamp>)
    --bitrate BPS      H.264 bitrate     (default: 8000000)
    --duration SECS    stop after N s    (default: 0 = Ctrl+C)

Output files in DIR/:
    video.mp4          H.264, 2048×1536 30fps
    telemetry.csv      unix_time, lat, lon, alt_amsl, alt_agl, heading_deg
    meta.json          video_start_unix, fps, width, height

Post-processing:
    python3 tools/extract_frames.py field_data/<timestamp>/
"""

import argparse
import csv
import json
import math
import os
import signal
import subprocess
import sys
import threading
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64


def _yaw_from_quat(qz, qw):
    """Extract yaw (rad) from PoseStamped quaternion (same convention as ros2_node.py)."""
    return 2.0 * math.atan2(qz, qw)


class TelemetryLogger(Node):
    def __init__(self, csv_path):
        super().__init__('field_recorder')

        self._lock    = threading.Lock()
        self._lat     = None
        self._lon     = None
        self._alt_msl = None
        self._agl     = None
        self._heading = None   # degrees, 0 = North, CW

        self._file   = open(csv_path, 'w', newline='')
        self._writer = csv.writer(self._file)
        self._writer.writerow(['unix_time', 'lat', 'lon', 'alt_amsl', 'alt_agl', 'heading_deg'])

        self.create_subscription(NavSatFix,   '/mavros/global_position/global', self._cb_gps,  10)
        self.create_subscription(Float64,     '/drone/agl',                     self._cb_agl,  10)
        self.create_subscription(PoseStamped, '/drone/pose',                    self._cb_pose, 10)

        # Log at 5 Hz
        self.create_timer(0.2, self._log_row)
        self.get_logger().info(f'Telemetry → {csv_path}')

    def _cb_gps(self, msg: NavSatFix):
        with self._lock:
            self._lat     = msg.latitude
            self._lon     = msg.longitude
            self._alt_msl = msg.altitude

    def _cb_agl(self, msg: Float64):
        with self._lock:
            self._agl = msg.data

    def _cb_pose(self, msg: PoseStamped):
        q  = msg.pose.orientation
        # /drone/pose encodes yaw as −compass_bearing_rad (same as ros2_node.py line 233)
        yaw_rad = _yaw_from_quat(q.z, q.w)
        heading = (-math.degrees(yaw_rad)) % 360.0
        with self._lock:
            self._heading = heading

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

    def status(self):
        with self._lock:
            lat = f'{self._lat:.6f}'     if self._lat     is not None else '---'
            lon = f'{self._lon:.6f}'     if self._lon     is not None else '---'
            agl = f'{self._agl:.1f} m'   if self._agl     is not None else '--- m'
            hdg = f'{self._heading:.0f}°' if self._heading is not None else '---°'
        return f'lat={lat}  lon={lon}  agl={agl}  hdg={hdg}'

    def close(self):
        self._file.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--output',   default='',  help='Output directory')
    ap.add_argument('--bitrate',  type=int, default=8_000_000)
    ap.add_argument('--duration', type=int, default=0,
                    help='Recording duration in seconds (0 = Ctrl+C)')
    args = ap.parse_args()

    ts  = datetime.now().strftime('%Y%m%d_%H%M%S')
    out = args.output or os.path.join('field_data', ts)
    os.makedirs(out, exist_ok=True)

    video_path = os.path.join(out, 'video.mp4')
    telem_path = os.path.join(out, 'telemetry.csv')
    meta_path  = os.path.join(out, 'meta.json')

    # ROS2 init
    rclpy.init()
    logger = TelemetryLogger(telem_path)
    spin_thread = threading.Thread(target=rclpy.spin, args=(logger,), daemon=True)
    spin_thread.start()

    # GStreamer pipeline
    num_buffers = args.duration * 30 if args.duration > 0 else 0
    gst_cmd = ['gst-launch-1.0', '-e']
    gst_cmd += ['v4l2src', 'device=/dev/video0']
    if num_buffers:
        gst_cmd += [f'num-buffers={num_buffers}']
    gst_cmd += [
        '!', 'video/x-raw,width=2048,height=1536,framerate=30/1',
        '!', 'nvvidconv',
        '!', 'nvv4l2h264enc', f'bitrate={args.bitrate}',
        '!', 'h264parse',
        '!', 'mp4mux',
        '!', 'filesink', f'location={video_path}',
    ]

    print(f'[REC] Output  → {out}/')
    print(f'[REC] Video   → video.mp4')
    print(f'[REC] Telem   → telemetry.csv')
    if args.duration:
        print(f'[REC] Duration: {args.duration} s')
    else:
        print('[REC] Press Ctrl+C to stop')
    print()

    video_start = time.time()
    proc = subprocess.Popen(gst_cmd, stderr=subprocess.DEVNULL)

    with open(meta_path, 'w') as f:
        json.dump({
            'video_start_unix': video_start,
            'fps': 30,
            'width': 2048,
            'height': 1536,
            'bitrate': args.bitrate,
        }, f, indent=2)

    stop_event = threading.Event()

    def _stop(sig=None, frame=None):
        if not stop_event.is_set():
            stop_event.set()
            print('\n[REC] Stopping …')
            proc.send_signal(signal.SIGINT)

    signal.signal(signal.SIGINT,  _stop)
    signal.signal(signal.SIGTERM, _stop)

    # Status line while recording
    try:
        while proc.poll() is None:
            elapsed = time.time() - video_start
            print(f'\r[REC] {elapsed:6.0f}s  {logger.status()}   ', end='', flush=True)
            time.sleep(1.0)
    except Exception:
        pass

    proc.wait()
    logger.close()
    rclpy.shutdown()

    print()
    size_mb = os.path.getsize(video_path) / 1e6 if os.path.exists(video_path) else 0
    print(f'[REC] Done — {size_mb:.1f} MB  ({out}/)')


if __name__ == '__main__':
    main()
