# tools/ — Flight Monitoring and Ground Station Tools

Standalone tools for monitoring, streaming, and analysing drone flights.

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
