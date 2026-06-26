# Jetson → MediaMTX Server Video Streaming

**Replace ZeroTier VPN with a relay server on Frank's PC.**  
Jetson pushes RTSP to the server. Ground station pulls from the server URL — no VPN, no direct P2P.

---

## Architecture

```
Jetson (LTE or field WiFi)              Frank's PC (public IP / LAN)     Ground Station
  GStreamer                                  MediaMTX :8554                  VLC / browser
  nvv4l2h265enc → rtspclientsink ─────────▶  /drone  ──────────────────────▶ rtsp://SERVER:8554/drone
```

---

## Server Info

| Item | Value |
|---|---|
| Server public IP | `118.232.160.227` |
| RTSP push/pull port | `8554` |
| SRT push port | `8890` (use if RTSP is unstable on LTE) |
| WebRTC browser view | `http://118.232.160.227:8889/drone` |
| HLS browser view | `http://118.232.160.227:8888/drone` |

IPv6 is also available at `2407:4d00:cc00:e15:558f:1e96:207d:5a30` if both endpoints support it.  
Router port forwarding must point 8554/TCP+UDP (and 8890/UDP for SRT) to `192.168.0.134` (this PC's LAN IP).

---

## Verify Server Is Running (from Jetson or ground station)

```bash
# RTSP port reachable:
nc -zv 118.232.160.227 8554 && echo "RTSP OK"

# Or pull a test stream (should block waiting — means server is alive):
gst-launch-1.0 rtspsrc location=rtsp://118.232.160.227:8554/drone latency=200 ! fakesink
# Expected output: "Setting pipeline to PLAYING" then wait (no stream yet = normal)
```

---

## Task 1 — Modify the GStreamer Pipeline

The only change from the existing script (`Gstreamer_opencv.py`) is the **sink**:

| | Before (ZeroTier) | After (MediaMTX relay) |
|---|---|---|
| Sink | `udpsink host=GROUND_IP port=5000` | `rtspclientsink location=rtsp://SERVER_IP:8554/drone protocols=tcp` |
| Receiver setup | GStreamer pipeline on ground station | VLC / browser — no setup |

### Updated GStreamer sink line

Replace the last two lines of `encode_pipeline`:
```python
# BEFORE
f'rtph265pay config-interval=-1 mtu=1200 ! '
f'udpsink host={ground_ip} port=5000 sync=false'

# AFTER
f'rtph265pay config-interval=-1 mtu=1200 ! '
f'rtspclientsink location=rtsp://{server_ip}:8554/drone protocols=tcp'
```

### Full updated script: `streaming/jetson_streamer.py`

Create this file on the Jetson:

```python
#!/usr/bin/env python3
"""
Drone camera → H.265 → RTSP push to MediaMTX relay server.

Changes from Gstreamer_opencv.py:
  - udpsink replaced with rtspclientsink (pushes to relay server)
  - Face detection removed (use YOLO node instead for vehicle detection)
  - Added --headless flag for contest flight (no display)
  - SERVER_IP configurable via env var or argument

Run:
  python3 streaming/jetson_streamer.py
  python3 streaming/jetson_streamer.py --server 118.232.160.227
  SERVER_IP=1.2.3.4 python3 streaming/jetson_streamer.py
"""
import argparse
import os
import sys

import cv2
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst

Gst.init(None)

# ── Config ────────────────────────────────────────────────────────────────────
DEFAULT_SERVER = os.environ.get("SERVER_IP", "118.232.160.227")
DEFAULT_DEVICE = int(os.environ.get("CAMERA_DEV", "0"))
WIDTH          = 848
HEIGHT         = 480
FPS            = 30
BITRATE        = 1_000_000   # 1 Mbps — fits LTE uplink
IDR_INTERVAL   = 30          # keyframe every 1 s — fast recovery on packet loss

def build_pipeline(server_ip: str) -> tuple:
    pipeline_str = (
        f'appsrc name=src format=time is-live=true block=true '
        f'caps=video/x-raw,format=BGR,width={WIDTH},height={HEIGHT},framerate={FPS}/1 ! '
        f'videoconvert ! '
        f'nvvidconv ! video/x-raw(memory:NVMM),format=NV12 ! '
        f'nvv4l2h265enc bitrate={BITRATE} preset-level=UltraFastPreset '
        f'    idrinterval={IDR_INTERVAL} iframeinterval={IDR_INTERVAL} ! '
        f'rtph265pay config-interval=-1 mtu=1200 ! '
        f'rtspclientsink location=rtsp://{server_ip}:8554/drone protocols=tcp'
    )
    pipeline = Gst.parse_launch(pipeline_str)
    appsrc   = pipeline.get_by_name('src')
    return pipeline, appsrc


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--server',   default=DEFAULT_SERVER,
                        help='MediaMTX server IP (default: %(default)s)')
    parser.add_argument('--device',   default=DEFAULT_DEVICE, type=int,
                        help='V4L2 camera index (default: %(default)s)')
    parser.add_argument('--headless', action='store_true',
                        help='No OpenCV preview window (use for contest flight)')
    args = parser.parse_args()

    print(f"[stream] Camera /dev/video{args.device} → rtsp://{args.server}:8554/drone")

    pipeline, appsrc = build_pipeline(args.server)

    cap = cv2.VideoCapture(args.device)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
    cap.set(cv2.CAP_PROP_FPS,          FPS)

    if not cap.isOpened():
        print(f"[stream] ERROR: cannot open /dev/video{args.device}")
        sys.exit(1)

    # Verify actual resolution (camera may round to nearest supported mode)
    ret, probe = cap.read()
    if ret:
        print(f"[stream] Camera resolution: {probe.shape[1]}×{probe.shape[0]}")

    pipeline.set_state(Gst.State.PLAYING)
    print("[stream] Streaming ... press Ctrl+C to stop")

    frame_count = 0
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("[stream] Camera read failed — retrying")
                continue

            buf          = Gst.Buffer.new_wrapped(frame.tobytes())
            buf.pts      = frame_count * Gst.SECOND // FPS
            buf.duration = Gst.SECOND // FPS
            flow         = appsrc.emit('push-buffer', buf)
            if flow != Gst.FlowReturn.OK:
                print(f"[stream] Pipeline error: {flow}")
                break

            if not args.headless:
                cv2.imshow('Drone Camera', frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            frame_count += 1
            if frame_count % (FPS * 10) == 0:
                print(f"[stream] {frame_count // FPS} s streamed  "
                      f"→ rtsp://{args.server}:8554/drone")

    except KeyboardInterrupt:
        print("\n[stream] Stopping ...")
    finally:
        cap.release()
        if not args.headless:
            cv2.destroyAllWindows()
        appsrc.emit('end-of-stream')
        pipeline.set_state(Gst.State.NULL)
        print("[stream] Done.")


if __name__ == '__main__':
    main()
```

---

## Task 2 — Create Launch Script: `streaming/launch_stream.sh`

```bash
#!/bin/bash
# Launch drone camera stream to MediaMTX relay server.
# Usage: bash streaming/launch_stream.sh [server_ip]
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SERVER_IP="${1:-118.232.160.227}"

source /opt/ros/jazzy/setup.bash   # needed if ROS2 env affects PYTHONPATH

echo "[launch_stream] Pushing to rtsp://${SERVER_IP}:8554/drone"
conda run -n isaac_sim_test --no-capture-output \
    python3 -u "$SCRIPT_DIR/jetson_streamer.py" \
    --server "$SERVER_IP" \
    --headless
```

Make executable: `chmod +x streaming/launch_stream.sh`

---

## Task 3 — Integrate into `control/launch_real_hw.sh`

Add the stream after the camera driver launch (before the commander):

```bash
# After: CAMERA_PID=$!
# Add:
echo "[launch] Starting video stream to relay server ..."
bash "$PROJECT_DIR/streaming/launch_stream.sh" "${SERVER_IP:-118.232.160.227}" &
STREAM_PID=$!
sleep 2

# And add to the cleanup section:
kill $STREAM_PID ...
```

---

## Ground Station — How to Watch

No software installation needed on the ground station.

**Option A — VLC (lowest latency for RTSP):**
```
vlc rtsp://118.232.160.227:8554/drone
```
Or via VLC menu: Media → Open Network Stream → paste the URL.

**Option B — Browser (WebRTC, ~200 ms latency):**
```
http://118.232.160.227:8889/drone
```
Open in Chrome or Edge. Works on any OS with no install.

**Option C — Browser (HLS, ~5 s latency but very reliable):**
```
http://118.232.160.227:8888/drone
```
Works on mobile phones too.

**Option D — GStreamer on ground station:**
```bash
gst-launch-1.0 rtspsrc location=rtsp://118.232.160.227:8554/drone latency=100 ! \
  decodebin ! videoconvert ! autovideosink sync=false
```

---

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `rtspclientsink` element not found | Missing GStreamer plugin | `sudo apt install gstreamer1.0-rtsp` on Jetson |
| Connection refused on port 8554 | Server not running or firewall | Start `./mediamtx mediamtx.yml` on PC; open port: `sudo ufw allow 8554/tcp` |
| Stream starts then freezes | LTE packet loss breaks RTSP TCP | Switch to SRT (see below) |
| Black screen on VLC | H.265 not decoded | Install H.265 codec; or use `--no-hw-dec` flag in VLC |
| High latency (>3 s) in VLC | VLC jitter buffer | VLC → Tools → Preferences → Input/Codecs → Network caching → 300 ms |
| `nvv4l2h265enc` not found | Missing Jetson GStreamer plugins | `sudo apt install nvidia-l4t-gstreamer` |

---

## SRT Alternative (better on lossy LTE)

If RTSP TCP drops frames on a mobile link, switch the Jetson sink to SRT:

```python
# In jetson_streamer.py, change rtspclientsink line to:
f'srtsink uri="srt://118.232.160.227:8890?streamid=publish:drone&mode=caller" '
f'latency=200'
```

Ground station VLC:
```
vlc srt://118.232.160.227:8890?streamid=read:drone
```

SRT handles retransmissions internally — much more resilient than RTSP over lossy links.

---

## Server Start Commands (Frank's PC)

```bash
# Manual start:
cd ~/Ardupilot_no_GPS_drone_project/streaming
./mediamtx mediamtx.yml

# Auto-start as systemd user service:
mkdir -p ~/.config/systemd/user
cp mediamtx-drone.service ~/.config/systemd/user/
systemctl --user daemon-reload
systemctl --user enable --now mediamtx-drone

# Check status:
systemctl --user status mediamtx-drone

# Firewall (run once):
sudo ufw allow 8554/tcp comment "MediaMTX RTSP"
sudo ufw allow 8554/udp comment "MediaMTX RTSP UDP"
sudo ufw allow 8890/udp comment "MediaMTX SRT"
sudo ufw allow 8889/tcp comment "MediaMTX WebRTC"
sudo ufw allow 8888/tcp comment "MediaMTX HLS"
sudo ufw allow 8000:8001/udp comment "MediaMTX RTP/RTCP"
```
