# tools/ — Flight Monitoring and Ground Station Tools

Standalone tools for monitoring, streaming, and analysing drone flights.

---

## record_field.py — Field database collection recorder

Records 1640×1232 30fps H.265 video (H.264→H.265 2026-07-07) from the IMX219 CSI camera directly (via OpenCV + GStreamer `nvarguscamerasrc`/ISP + `appsrc`) alongside a telemetry CSV (lat/lon/AGL/heading/RC-channels at 5 Hz via ROS2) and a per-frame capture-timestamp CSV. Frames are rotated 180° after capture. Auto-spawns the `imu_logger.py` sidecar (FC IMU + attitude at 50 Hz for VIO — its own process, never in-process; see `instructions/vio_data_collection.md`). IMU rate is `--imu-hz` (default 200; field standard since 2026-07-23 is `--imu-hz 333` → ~346 Hz actual, the firmware's grantable max — removes the RAW_IMU stream-decimation vibration aliasing, `instructions/vpe_jump_runaway_diagnosis.md` §14-6; the Desktop launcher passes it). Optionally streams a 1280×720 H.265 preview with a telemetry overlay bar to a ground station or a MediaMTX relay server, and/or H.264 RTP to an OpenHD ground station.

**Do NOT run `launch_camera.sh` at the same time** — both open an Argus CaptureSession on the same sensor.  
Requires **MAVROS only** — reads GPS/AGL/heading directly from `/mavros/global_position/*` and RC input from `/mavros/rc/in`. `hw_bridge.py` is not needed.

> **Known issue:** prints `waiting for GPS …` until `/mavros/global_position/global` receives a fix. On a no-GPS flight (GPS jammed), the status line stays stuck but **recording continues normally** — video and AGL/heading still write to CSV. Fix pending: replace lat/lon source with AnyLoc pose.

```bash
# Terminal 1
bash control/launch_mavros_real.sh

# Terminal 2 — record only
source /opt/ros/humble/setup.bash
source control/ros2_env.sh
python3 tools/record_field.py --output field_data/survey1

# Terminal 2 — record + stream direct to ground station (UDP)
source /opt/ros/humble/setup.bash
source control/ros2_env.sh
python3 tools/record_field.py --output field_data/survey1 --stream-host 10.181.156.237

# Terminal 2 — record + stream via MediaMTX relay server (RTSP)
source /opt/ros/humble/setup.bash
source control/ros2_env.sh
python3 tools/record_field.py --output field_data/survey1 --stream-server 118.232.160.227
```

**Stream mode A — ground station receiver (UDP):**
```bash
gst-launch-1.0 udpsrc port=5000 ! \
    application/x-rtp,encoding-name=H265,payload=96 ! \
    rtph265depay ! h265parse ! avdec_h265 ! \
    videoconvert ! autovideosink sync=false
```

**Stream mode B — viewers (MediaMTX relay):**
```
VLC:     rtsp://118.232.160.227:8554/drone
Browser: http://118.232.160.227:8889/drone  (WebRTC, ~200 ms)
Browser: http://118.232.160.227:8888/drone  (HLS, ~5 s, mobile-friendly)
```

| Flag | Default | Description |
|---|---|---|
| `--output DIR` | `field_data/<timestamp>` | Output directory |
| `--stream-host IP` | off | Ground station IP — direct UDP preview stream |
| `--stream-port N` | 5000 | UDP port (mode A only) |
| `--stream-server IP` | off | MediaMTX relay server IP — RTSP push stream |
| `--stream-rtsp-path P` | `/drone` | RTSP path (mode B only) |
| `--stream-bitrate N` | 2000000 | H.265 stream bitrate (bps, modes A/B) |
| `--stream-openhd [IP]` | off | Mode C: H.264 RTP/UDP to OpenHD ground station (default 192.168.2.2), coexists with A/B — shares the recorder's capture |
| `--openhd-port N` | 5601 | OpenHD UDP port |
| `--openhd-bitrate N` | 4000000 | OpenHD H.264 bitrate (bps) |
| `--bitrate N` | 8000000 | H.265 recording bitrate (bps) |
| `--duration N` | 0 | Stop after N seconds (0 = Ctrl+C) |
| `--calib` | off | Tag as camera-IMU calibration session (`field_data/calib_<ts>/`) |

`--stream-host` and `--stream-server` are mutually exclusive.

**Output:** `video.mkv`, `telemetry.csv` (now includes an `rc_channels` column — raw `/mavros/rc/in` PWM list, useful for confirming the EKF-source switch stayed on GPS for the whole recording), `meta.json`, `frame_times.csv` (`frame_idx, unix_time` — real per-frame capture time, more reliable than `meta.json`'s `video_start_unix + frame_idx/fps` across camera dropouts), and the sidecar's `imu.csv` (200 Hz), `attitude.csv` (50 Hz), `imu_rates.json` in the output directory.  
**Storage:** ~60 MB/min at default bitrate.

---

## gen_survey_waypoints.py — Field survey waypoint generator

Generates Mission Planner QGC WPL 110 `.waypoints` files for the AnyLoc database collection lawnmower. Strips run **E-W (long side ~1 743 m)**, advancing N-S between strips.

```bash
python3 tools/gen_survey_waypoints.py                 # full mission → field_data/survey_mission_full.waypoints
python3 tools/gen_survey_waypoints.py --split 4       # 4 equal N-S sub-missions
python3 tools/gen_survey_waypoints.py --spacing 22    # 22 m spacing (72 % sidelap, denser)
```

| Flag | Default | Description |
|---|---|---|
| `--spacing M` | 39.2 m | Strip spacing in metres (39.2 m = 50 % sidelap of the IMX219's 78.4 m footprint @ 65 m AGL) |
| `--split N` | 1 | Split into N equal-width N-S sub-missions |
| `--outdir DIR` | `field_data` | Output directory |

Default output (27 strips, 39.2 m spacing, 50 % sidelap):
```
Survey area  : 2091 m (E-W) × 1025 m (N-S)  = 2.14 km²
Strip spacing: 39 m  →  50 % sidelap
Total distance: 57.5 km  ~319 min  (~16 batteries @ 20 min each)
```

Load the output `.waypoints` file in Mission Planner or pass it directly to `ardupilot_commander.py --waypoint-file`.

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
source control/ros2_env.sh
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

## ground_view_stream.py — Composite ground view stream (YOLO + AnyLoc)

Subscribes to `/drone/camera/image_raw` (`launch_camera.sh` must already be running — this script doesn't open the camera) and streams a 1280×720 composite viewport. Two stream modes: direct UDP to a ground station, or RTSP push to the MediaMTX relay server (no GStreamer needed on the receiver).

**Local recording (default-on, added 2026-07-06):** every run also saves the exact streamed composite to `recordings/ground_view_<timestamp>.mkv` — a tee of the same H.265 encode (no extra GPU load), streamable MKV so it stays playable after power loss mid-flight, new timestamped file per run, gitignored. `--no-record` disables; `--record-dir DIR` relocates. This is the overlay-burned 720p stream view — for clean full-res camera footage (database building, offline replay) use `record_field.py`; both can run together.

```
Left  (640×720)
  ├─ Top    (640×360): camera with YOLO bounding boxes + drone lat/lon/AGL
  │                    (frame is stamp-matched to the boxes — see below)
  └─ Bottom (640×360): AnyLoc latest match satellite tile + localizer telemetry
Right (640×720)
  ├─ Slot 0 (640×240): most recent YOLO detection crop ─┐
  ├─ Slot 1 (640×240): 2nd most recent                  ├ class / conf / lat / lon / age
  └─ Slot 2 (640×240): 3rd most recent                 ─┘
```

This script only **subscribes** to `/drone/camera/image_raw` — it does not open the camera. `launch_camera.sh` must already be running (handled automatically by `launch_real_hw.sh --stream-host/--stream-server`); YOLO and AnyLoc receive camera frames from the same topic normally.

```bash
# Mode A — direct UDP to ground station (ZeroTier / LAN)
source /opt/ros/humble/setup.bash
source control/ros2_env.sh
python3 tools/ground_view_stream.py --host 10.181.156.237

# Mode B — RTSP push to MediaMTX relay server (LTE / internet)
source /opt/ros/humble/setup.bash
source control/ros2_env.sh
python3 tools/ground_view_stream.py --stream-server 118.232.160.227
```

**Mode A — receive on ground station:**
```bash
gst-launch-1.0 udpsrc port=5000 ! \
    application/x-rtp,encoding-name=H265,payload=96 ! \
    rtph265depay ! h265parse ! avdec_h265 ! \
    videoconvert ! autovideosink sync=false
```

**Mode B — watch on ground station (no install needed):**
```
VLC:     rtsp://118.232.160.227:8554/drone
Browser: http://118.232.160.227:8889/drone  (WebRTC, ~200 ms)
Browser: http://118.232.160.227:8888/drone  (HLS, ~5 s, mobile-friendly)
```

| Flag | Default | Description |
|---|---|---|
| `--host IP` | `GROUND_IP` env or `10.181.156.237` | Ground station IP — direct UDP (mode A) |
| `--port N` | 5000 | UDP port (mode A only) |
| `--stream-server IP` | off | MediaMTX relay server IP — RTSP push (mode B) |
| `--rtsp-path P` | `/drone` | RTSP stream path (mode B only) |
| `--bitrate N` | 1000000 | H.265 bitrate (bits/s) |

`--host` and `--stream-server` are mutually exclusive. Without either, defaults to direct UDP using `GROUND_IP` env var.

**Latency bounding (2026-07-07):** over the LTE relay (mode B), a growing, non-recovering delay was traced to two unbounded buffers: `appsrc` pushed with `block=true` and no queue after it (a downstream stall just blocked the whole compositing loop indefinitely), and the encoder's `vbv-size` defaulting to 4 MB regardless of `--bitrate` (let it burst ~4 s of data above target rate on complex frames, e.g. right when YOLO draws a detection). Fixed by adding a `leaky=downstream max-size-buffers=2` queue right after `appsrc` (drops stale frames instead of blocking) and setting `vbv-size` equal to `--bitrate` (~1 s of burst allowance instead of ~4 s). Both scale automatically with `--bitrate` — no new flag.

**Auto-reconnect (2026-07-09):** `rtspclientsink` (mode B) doesn't recover on its own from a lost TCP connection to the relay (LTE drop) — it left the pipeline stuck in an error state forever, silently discarding every subsequent frame with nothing to signal it (visible as a `GLib-GIO-CRITICAL **: g_socket_set_timeout: assertion 'G_IS_SOCKET (socket)' failed` log line, then the stream just stops). The main loop now polls the GStreamer bus each frame for `ERROR`/`EOS` and, on either, tears down and rebuilds the whole pipeline (exponential backoff: 2 s → capped at 30 s, reset after 15 s of clean streaming). Recording rolls into a new timestamped `.mkv` segment on each reconnect. Applies to both modes, though mode A (connectionless UDP) rarely triggers it.

**Detection-synced overlay (2026-07-09):** the top-left panel used to draw the most recent `/yolo/detections` boxes on the *newest* camera frame — but those boxes were computed on a frame 2–4 frames older (30 fps camera vs ~17 fps YOLO + ~60 ms inference), so boxes visibly trailed moving objects; the right-panel crops had the same bug (cut from the newest frame with old box coordinates, so a moving car could sit off-center or outside its crop). The node now buffers the last 12 frames by header stamp and, since the detector copies the source image's header into `Detection2DArray`, both draws the boxes on and cuts the crops from the exact frame they were computed on. The panel therefore updates at YOLO's rate (~17 fps) and trails live by one inference (~60 ms — invisible next to the LTE latency), with the boxes glued to their objects. If no detections message arrives for 2 s (YOLO publishes every processed frame, even with zero detections, so silence means it's down), the panel falls back to the live 30 fps feed with no boxes and a red `YOLO STALE (live view)` header instead of freezing.

**Via launch script** (integrates into full flight stack):
```bash
bash control/launch_real_hw.sh --stream-host 10.181.156.237       # mode A
bash control/launch_real_hw.sh --stream-server 118.232.160.227    # mode B
```

**Requires:** nvidia-l4t-gstreamer, python3-gi, ROS2 Humble with vision_msgs

---

## gstreamer_stream.py — Simple H.265 camera stream (camera + AnyLoc only)

Opens the IMX219 CSI camera directly with OpenCV/GStreamer (`nvarguscamerasrc`) and streams a 1280×480 two-panel view via H.265/RTP/UDP. Simpler than `ground_view_stream.py` — no YOLO boxes, no detection crops, no ROS2 node.

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

**Important:** opens an Argus CaptureSession directly — do NOT also run `launch_camera.sh` (only one CaptureSession is allowed on the sensor at a time). `ground_view_stream.py` doesn't open the camera itself, so it's fine to run alongside this one.

| Flag | Default | Description |
|---|---|---|
| `--host IP` | `GROUND_IP` env or `10.181.156.237` | Ground station IP |
| `--port N` | 5000 | UDP port |
| `--camera N` | 0 | Argus sensor-id |
| `--bitrate N` | 1000000 | H.265 bitrate (bits/s) |

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
source control/ros2_env.sh
python3 tools/anyloc_gps_compare.py
```

---

## Trace CSV format (simulation only)

```
t_s, east_m, north_m, agl_m, vn_ms, ve_ms
```

Written by `control/drone_sim.py` and `simulator/cesium_scene.py` at 5 Hz.
