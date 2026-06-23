# tools/ — Flight Monitoring and Ground Station Tools

Standalone tools for monitoring, streaming, and analysing drone flights.

---

## record_field.py — Field database collection recorder

Records 2048×1536 30fps H.264 video from `/dev/video0` directly (via OpenCV + GStreamer `appsrc`) alongside a telemetry CSV (lat/lon/AGL/heading at 5 Hz via ROS2). Optionally streams a 1280×720 H.265 preview with a telemetry overlay bar to a ground station.

**Do NOT run `launch_camera.sh` at the same time** — both open `/dev/video0`.  
Requires **MAVROS only** — reads GPS/AGL/heading directly from `/mavros/global_position/*`. `hw_bridge.py` is not needed.

```bash
# Terminal 1
bash control/launch_mavros_real.sh

# Terminal 2 — record only
source /opt/ros/humble/setup.bash
python3 tools/record_field.py --output field_data/survey1

# Terminal 2 — record + stream preview to ground station
source /opt/ros/humble/setup.bash
python3 tools/record_field.py --output field_data/survey1 --stream-host 10.181.156.237
```

**Ground station receiver:**
```bash
gst-launch-1.0 udpsrc port=5000 ! \
    application/x-rtp,encoding-name=H265,payload=96 ! \
    rtph265depay ! h265parse ! avdec_h265 ! \
    videoconvert ! autovideosink sync=false
```

| Flag | Default | Description |
|---|---|---|
| `--output DIR` | `field_data/<timestamp>` | Output directory |
| `--stream-host IP` | off | Ground station IP — enables preview stream |
| `--stream-port N` | 5000 | UDP port |
| `--bitrate N` | 8000000 | H.264 recording bitrate (bps) |
| `--stream-bitrate N` | 2000000 | H.265 stream bitrate (bps) |
| `--duration N` | 0 | Stop after N seconds (0 = Ctrl+C) |

**Output:** `video.mkv`, `telemetry.csv`, `meta.json` in the output directory.  
**Storage:** ~58 MB/min at default bitrate.

---

## extract_frames.py — Geo-tagged frame extractor

Reads a `record_field.py` session directory and extracts one frame every N metres of ground track, matched to GPS position and heading from the telemetry CSV.

```bash
python3 tools/extract_frames.py field_data/survey1/ --rotate --min-dist 25
```

| Flag | Default | Description |
|---|---|---|
| `--min-dist M` | 30 m | Minimum ground distance between saved frames |
| `--min-agl M` | 50 m | Skip frames below this AGL |
| `--rotate` | off | Rotate each frame to North-up using heading |

**Output:** `frames/000000.jpg …` and `frames.csv` (path, lat, lon, alt_agl, heading_deg).  
Feed directly to `anyloc/build_database_real.py`.

---

## ekf_monitor.py — EKF status monitor

Watches raw MAVLink from `/uas1/mavlink_source` and decodes `EKF_STATUS_REPORT` (msgid 193).  
Shows which EKF flags are active and whether `POS_ABS` has been accepted.

```bash
source /opt/ros/humble/setup.bash
python3 tools/ekf_monitor.py
```

**Output when EKF accepts VPE:**
```
flags=0x037  ✓ POS_ABS accepted
  active : ATTITUDE, VEL_HORIZ, VEL_VERT, POS_REL, POS_ABS, POS_VERT
  var    : vel=0.08  pos_h=0.12  pos_v=0.11  compass=0.01
```

Use this to verify EKF accepts VPE before flight. Flip RC aux switch to HIGH (SRC2 = ExternalNav)  
and confirm `POS_ABS` appears in the active list with `pos_h` variance < 0.5.

**Requires:** MAVROS running (`launch_mavros_real.sh`)

---

## gstreamer_stream.py — H.265 camera stream to ground station

Opens the camera directly with OpenCV and streams a 1280×480 two-panel view to a ground station PC via H.265/RTP/UDP.

```
Left panel  (640×480): live camera + AnyLoc telemetry overlay
Right panel (640×480): AnyLoc matched satellite tile (from anyloc/latest_match.jpg)
```

```bash
bash control/launch_gstreamer.sh --host 10.181.156.237
# or:
python3 tools/gstreamer_stream.py --host 10.181.156.237 --port 5000
```

**Receive on ground station:**
```bash
gst-launch-1.0 udpsrc port=5000 ! \
    application/x-rtp,encoding-name=H265,payload=96 ! \
    rtph265depay ! h265parse ! avdec_h265 ! \
    videoconvert ! autovideosink sync=false
# Or VLC: Media → Open Network Stream → rtp://@:5000
```

**Important:** opens `/dev/video0` directly — do NOT also run `launch_camera.sh` (device busy).  
The right panel shows `anyloc/latest_match.jpg` saved by `anyloc/ros2_node.py` on each AnyLoc match.

| Flag | Default | Description |
|---|---|---|
| `--host IP` | `GROUND_IP` env or `10.181.156.237` | Ground station IP |
| `--port N` | 5000 | UDP port |
| `--camera N` | 0 | Camera index (`/dev/video0`) |
| `--bitrate N` | 2000000 | H.265 bitrate (bits/s) |

**Requires:** nvidia-l4t-gstreamer, python3-gi (both on JetPack 36.x)

---

## live_trace.py — real-time simulation trace viewer

Open before or during a **simulation** flight to watch the trace as it grows.

```bash
python3 tools/live_trace.py              # auto-attach to newest trace
python3 tools/live_trace.py <file.csv>  # specific file
```

**Display:**
- Left panel: top view (East vs North) — accumulating path, home marker
- Right panel: AGL vs time
- Updates every 200 ms; axes auto-expand as drone moves

---

## plot_trace.py — post-flight simulation plotter

```bash
python3 tools/plot_trace.py              # latest trace
python3 tools/plot_trace.py <file.csv>  # specific trace
python3 tools/plot_trace.py --all        # overlay all traces
```

Saves `simulator/flight_traces/trace_plot.png`.

---

## anyloc_gps_compare.py — AnyLoc accuracy checker

Compares `anyloc/latest_estimate.json` against live GPS from MAVROS.

```bash
source /opt/ros/humble/setup.bash
python3 tools/anyloc_gps_compare.py
```

---

## Trace CSV format (simulation only)

```
t_s, east_m, north_m, agl_m, vn_ms, ve_ms
```

Written by `control/drone_sim.py` and `simulator/cesium_scene.py` at 5 Hz.
