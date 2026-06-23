# anyloc/ — Visual Localisation (GPS-Denied)

Visual place recognition for GPS-denied drone navigation.  
Uses **DINOv2** patch features + **VLAD** aggregation + **FAISS** nearest-neighbour search against a geo-tagged satellite image database.

Active backbone: **ViT-S/14** (`dinov2_vits14`) — database lives in `anyloc/database_vits14/` with a symlink `anyloc/database → anyloc/database_vits14`. The localizer reads `model_name` from the database metadata automatically.

**Platform:** Jetson Orin NX, JetPack 36.x, ROS2 Humble, Python 3.10  
**Python env:** `/home/jetson/venv/anyloc` (torch + faiss + pillow)

---

## How it fits in the pipeline

```
/drone/camera/image_raw  →  ros2_node.py  →  /anyloc/pose_estimate
                                           →  latest_estimate.json   (read by ardupilot_commander.py)
                                           →  latest_match.jpg       (read by gstreamer_stream.py postview)
```

AnyLoc only runs inference when AGL ≥ 50 m (configurable via `MIN_AGL`).  
Below 50 m the postview still shows the live camera feed.

VPE to MAVROS (`/mavros/vision_pose/pose_cov`) is published by `ardupilot_commander.py`,  
which reads `latest_estimate.json` — **not** by this node (avoids duplicate EKF inputs).  
Exception: `--test` mode publishes directly to MAVROS so the commander is not needed for ground tests.

---

## Requirements

| Component | Version | Notes |
|---|---|---|
| OS | Ubuntu 22.04 | JetPack 36.x |
| Python | 3.10 | `/home/jetson/venv/anyloc` |
| PyTorch | JetPack wheel | NVIDIA JetPack PyTorch |
| faiss-cpu | ≥ 1.7 | pip install faiss-cpu |
| Pillow | ≥ 9.0 | |
| NumPy | ≥ 1.24 | |
| OpenCV | ≥ 4.7 | |
| ROS2 Humble | — | `/opt/ros/humble` |

---

## 1. Build the Image Database

### Option A — Satellite tiles (default, no flight needed)

```bash
/home/jetson/venv/anyloc/bin/python3 anyloc/build_database.py --model vits14
```

Database lands in `anyloc/database_vits14/`. Create the symlink once:

```bash
ln -s database_vits14 anyloc/database
```

### Option B — Real drone footage (better match at inference time)

Fly a grid survey with the recorder, extract frames, then build:

```bash
# 1. Record survey flight
source /opt/ros/humble/setup.bash
python3 tools/record_field.py --output field_data/survey1 --stream-host <GS_IP>

# 2. Extract geo-tagged frames
python3 tools/extract_frames.py field_data/survey1/ --rotate --min-dist 25

# 3. Build database
/home/jetson/venv/anyloc/bin/python3 anyloc/build_database_real.py field_data/survey1/

# 4. Activate
ln -sfn database_real anyloc/database
```

See `instructions/field_database_collection.md` for the full guide including flight plan, FOV/overlap analysis, and terminal setup.

### Switching databases

```bash
ln -sfn database_vits14 anyloc/database   # satellite
ln -sfn database_real   anyloc/database   # real-field
```

### Verify

```bash
ls anyloc/database/database_vlads.pt && echo "DB OK"
```

---

## 2. Run the ROS2 Node

```bash
bash anyloc/run_ros2_localizer.sh [OPTIONS]
```

| Flag | Description |
|---|---|
| *(none)* | Show matplotlib postview window (requires display / SSH -X) |
| `--headless` | No display, no stream — flight mode |
| `--stream-host IP` | Stream postview as H.265/RTP to ground station instead of local window |
| `--stream-port N` | UDP port for stream (default: 5000) |
| `--test` | **Ground test mode**: bypass 50 m AGL gate, run AnyLoc on every frame, publish VPE directly to `/mavros/vision_pose/pose_cov` |
| `--test-agl N` | Fake AGL (m) used when on ground in `--test` mode (default: 65) |

### Postview streaming to ground station

```bash
bash anyloc/run_ros2_localizer.sh --stream-host 10.181.156.237
```

Receive on ground station:
```bash
gst-launch-1.0 udpsrc port=5000 ! \
    application/x-rtp,encoding-name=H265,payload=96 ! \
    rtph265depay ! h265parse ! avdec_h265 ! \
    videoconvert ! autovideosink sync=false
```

### Ground test (no flight needed)

```bash
# Terminal 1 — MAVROS
bash control/launch_mavros_real.sh

# Terminal 2 — Camera
bash control/launch_camera.sh

# Terminal 3 — AnyLoc test mode (publishes VPE to MAVROS directly)
bash anyloc/run_ros2_localizer.sh --test

# Terminal 4 — EKF monitor
source /opt/ros/humble/setup.bash && python3 tools/ekf_monitor.py
```

Then flip RC aux switch to HIGH (SRC2 = ExternalNav) and watch for `✓ POS_ABS accepted`.  
`hw_bridge.py` is **not needed** for `--test` mode.

---

## 3. ROS2 Topics

| Direction | Topic | Type | Notes |
|---|---|---|---|
| Subscribe | `/drone/camera/image_raw` | `sensor_msgs/Image` | rgb8, 1280×960, 30 fps |
| Subscribe | `/drone/pose` | `geometry_msgs/PoseStamped` | WGS84 (lat, lon, alt_msl) from hw_bridge |
| Subscribe | `/drone/agl` | `std_msgs/Float64` | AGL from hw_bridge (barometer) |
| Publish | `/anyloc/pose_estimate` | `geometry_msgs/PoseWithCovarianceStamped` | WGS84 estimate (monitoring) |
| Publish | `/mavros/vision_pose/pose_cov` | `geometry_msgs/PoseWithCovarianceStamped` | **test mode only** — ENU metres |

---

## 4. Output Files

| File | Description |
|---|---|
| `anyloc/latest_estimate.json` | Latest AnyLoc estimate — read by `ardupilot_commander.py` VPE thread |
| `anyloc/latest_match.jpg` | Latest matched satellite tile — read by `tools/gstreamer_stream.py` for right panel |

### latest_estimate.json format

```json
{
  "timestamp": 1748991234.5,
  "est_lat": 23.4512,
  "est_lon": 120.2847,
  "alt_msl_m": 93.3,
  "agl_m": 65.1,
  "yaw_deg": 0.0,
  "score": 0.847,
  "error_m": 32.4
}
```

---

## 5. VPE Integration (normal flight)

`ardupilot_commander.py` reads `latest_estimate.json` in its VPE background thread at 20 Hz:

- **Phase 1** (AGL < 50 m): sends home-anchor VPE at (0, 0), cov = 20 m²
- **Phase 2** (AGL ≥ 50 m): sends AnyLoc estimate, cov = 20 m²

The commander also switches EKF source from GPS (SRC1) to ExternalNav (SRC2) automatically after reaching cruise altitude via `MAV_CMD_DO_AUX_FUNCTION`.

---

## 6. Troubleshooting

| Symptom | Fix |
|---|---|
| `[PostView] Waiting for first frame` / black window | Camera not running — start `launch_camera.sh` first |
| `ImportError: No module named 'faiss'` | Use `/home/jetson/venv/anyloc/bin/python3`, not system python |
| `latest_estimate.json` not updating | Normal below 50 m AGL in normal mode; use `--test` to bypass |
| Database not found | Check symlink: `ls -la anyloc/database` → should point to `database_vits14` |
| Wrong model loaded | `model_name` is in `database_meta.pt`; localizer reads it automatically |
| Postview window black on Jetson screen | Use `--stream-host` to stream to ground PC, or `ssh -X` for X11 forwarding |
