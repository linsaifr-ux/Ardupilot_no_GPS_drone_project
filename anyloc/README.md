# anyloc/ — Visual Localisation (GPS-Denied)

Visual place recognition for GPS-denied drone navigation.  
Uses **DINOv2** patch features + **VLAD** aggregation + **FAISS** nearest-neighbour search against a geo-tagged satellite image database.

Active backbone: **ViT-S/14** (`dinov2_vits14`) — database lives in `anyloc/database_zone_vits14/` (right-sized to the mission zone, 882 entries) with a symlink `anyloc/database → anyloc/database_zone_vits14`. The localizer reads `model_name` from the database metadata automatically. The old full-radius `anyloc/database_vits14/` (2821 entries, covers a 1502 m circle around home) is kept on disk as a fallback.

**Platform:** Jetson Orin NX, JetPack 36.x, ROS2 Humble, Python 3.10  
**Python env:** `/home/jetson/venv/anyloc` (torch + faiss + pillow)

---

## How it fits in the pipeline

```
/drone/camera/image_raw  →  ros2_node.py                →  /anyloc/pose_estimate
                            OR ros2_node_vo_primary.py   →  latest_estimate.json   (read by ardupilot_commander.py)
                            (run ONE, never both)        →  latest_match.jpg       (read by gstreamer_stream.py postview)
```

Two fusion nodes exist — identical topics/outputs, different policy; run **one or the other, never both** (they write the same `latest_estimate.json`):

- **`ros2_node.py` (plan A, anchor-chain):** AnyLoc every `ANYLOC_INTERVAL=10` frames re-anchors *unconditionally*; VO fills between anchors and resets on each anchor. Since 2026-07-06 the first search is seeded from the EKF position (`/drone/pose` — GPS truth on SRC1 at handover) instead of a global whole-DB search.
- **`ros2_node_vo_primary.py` (plan B, VO-primary + gated AnyLoc) — preferred:** position seeds once from the EKF position, VO integrates every frame and is never reset; AnyLoc every 10 frames (constrained ±200 m around the VO position) supplies *corrections* that must pass two gates: cosine `score ≥ --gate` (default 0.32) **and** a jump-plausibility gate — the candidate must lie within `--jump-base + --drift-rate × dt` metres of the VO position (defaults 45 m + 1.0 m/s since the last accepted correction), because VO can't be more wrong than its drift allows, so a farther "good-scoring" match is a false match. Accepted corrections are **blended** (`--blend`, default 0.4 of the way toward the candidate) rather than snapped, since the localizer returns raw DB-entry coordinates on a ~50 m grid — a correct match can sit ~35 m from truth and snapping teleports the EKF to each new grid point. If `--reacquire-n` (default 3) consecutive jump-rejected candidates agree within 30 m, the position relocates there fully (a consistent far signal means VO/seed was wrong, not the matcher). Launcher: `run_ros2_localizer_vo.sh`. Benchmarked on `field_data/survey13` real footage (`test_vo_fusion_compare.py`, logs `anyloc/logs/survey13_vo_fusion*.json`): 13–15 m mean error vs 29–399 m for plan A, which can permanently lock onto a wrong DB entry. **The gate is site/database-specific** (real-footage scores span ~0.16–0.34; 0.32 calibrated on survey13 + `database_test20_vits14`) — recalibrate from a GPS shadow flight's accuracy CSV before contest: set it just above the highest bad-match score.

The Desktop `full_run.sh` launches plan B; `control/launch_real_hw.sh` still launches plan A.

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
# Zone-sized (recommended — matches the active database):
/home/jetson/venv/anyloc/bin/python3 anyloc/build_database.py --model vits14 \
    --db-dir anyloc/database_zone_vits14 \
    --n-min -262 --n-max 762 --e-min -1501 --e-max 589
# Full-radius circle around CENTER_LAT/LON (larger, mostly wasted — old default):
/home/jetson/venv/anyloc/bin/python3 anyloc/build_database.py --model vits14
```

The zone-sized build lands in `anyloc/database_zone_vits14/`; the plain command lands in `anyloc/database_vits14/`. Create the symlink once:

```bash
ln -s database_zone_vits14 anyloc/database
```

The `--n-min/--n-max/--e-min/--e-max` bounds above are specific to the current mission zone + 20% margin — recompute them if the zone changes (see `tools/gen_contest_survey.py`'s bounding-box math).

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

### Option C — Satellite tiles for a different area (testing)

`build_database.py` defaults to the mission-zone `CENTER_LAT`/`CENTER_LON` baked into the script, but `--center-lat`/`--center-lon`/`--grid-radius-m` override it for a one-off test database anywhere in Taiwan (NLSC PHOTO2 coverage). `--sat-path` defaults to `<db-dir>/satellite.jpg` in this mode so it never clobbers `simulator/satellite_ground.jpg` (the mission-zone mosaic):

```bash
/home/jetson/venv/anyloc/bin/python3 anyloc/build_database.py --model vits14 \
    --db-dir anyloc/database_test_<name>_vits14 \
    --center-lat <lat> --center-lon <lon> \
    --grid-radius-m 1000
```

This builds a circular grid (not the rectangular mission-zone shape) of the given radius. `--n-min/--n-max/--e-min/--e-max` still work too, measured from the new center.

### Switching databases

```bash
ln -sfn database_zone_vits14 anyloc/database   # satellite, zone-sized (active)
ln -sfn database_vits14      anyloc/database   # satellite, old full-radius fallback
ln -sfn database_real        anyloc/database   # real-field
ln -sfn database_test_<name>_vits14 anyloc/database   # ad-hoc test area (Option C)
```

> `anyloc/database` is just a symlink — check `readlink anyloc/database` before a real flight to make sure it's not still pointed at a test database from a previous session.

### Verify

```bash
ls anyloc/database/database_vlads.pt && echo "DB OK"
```

---

## 2. Run the ROS2 Node

```bash
bash anyloc/run_ros2_localizer_vo.sh [OPTIONS]   # plan B (preferred)
bash anyloc/run_ros2_localizer.sh    [OPTIONS]   # plan A (anchor-chain)
```

| Flag | Description |
|---|---|
| *(none)* | Show matplotlib postview window (requires display / SSH -X) |
| `--headless` | No display, no stream — flight mode |
| `--gate S` | **plan B only** — AnyLoc accept score gate (default 0.32; recalibrate per site/DB) |
| `--jump-base M` | **plan B only** — jump gate base (m): score-passing candidate must lie within `jump-base + drift-rate×dt` of the VO position (default 45) |
| `--drift-rate R` | **plan B only** — jump gate growth (m/s) since last accepted correction (default 1.0) |
| `--blend A` | **plan B only** — fraction of (candidate − position) applied per accepted correction (default 0.4; 1.0 = old snap behavior) |
| `--reacquire-n N` | **plan B only** — consecutive agreeing jump-rejects that force a relocation (default 3) |
| `--stream-host IP` | Stream postview as H.265/RTP to ground station instead of local window |
| `--stream-port N` | UDP port for stream (default: 5000) |
| `--test` | **Ground test mode**: bypass 50 m AGL gate, run AnyLoc on every frame, publish VPE directly to `/mavros/vision_pose/pose_cov` |
| `--test-agl N` | Fake AGL (m) used when on ground in `--test` mode (default: 65) |

Both nodes need `hw_bridge.py` running: plan B waits for `/drone/pose` to seed its position (prints a warning until it arrives), and plan A uses it to seed the first search window.

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
| Subscribe | `/drone/camera/image_raw` | `sensor_msgs/Image` | rgb8, 1640×1232, 30 fps |
| Subscribe | `/drone/pose` | `geometry_msgs/PoseStamped` | WGS84 (lat, lon, alt_amsl) — hw_bridge relays ArduPilot's own EKF output (`/mavros/global_position/global`), whichever source (GPS or ExternalNav/VPE) is active |
| Subscribe | `/drone/agl` | `std_msgs/Float64` | AGL — hw_bridge relays `/mavros/global_position/rel_alt` (ArduPilot's own relative altitude) |
| Publish | `/anyloc/pose_estimate` | `geometry_msgs/PoseWithCovarianceStamped` | WGS84 estimate (monitoring) |
| Publish | `/mavros/vision_pose/pose_cov` | `geometry_msgs/PoseWithCovarianceStamped` | **test mode only** — ENU metres |

---

## 4. Output Files

| File | Description |
|---|---|
| `anyloc/latest_estimate.json` | Latest AnyLoc estimate — read by `ardupilot_commander.py` VPE thread |
| `anyloc/latest_match.jpg` | Latest matched satellite tile — read by `tools/gstreamer_stream.py` for right panel |
| `anyloc/logs/accuracy_<timestamp>.csv` | Per-frame AnyLoc-vs-GPS log, one row per camera frame, written for the life of the run (see below) |

### accuracy_<timestamp>.csv format

One file per node run (either fusion node — same format), created at startup (path printed as `[AnyLoc] Logging accuracy to ...`), flushed every row so no data is lost on a crash. Columns:

```
timestamp, drone_lat, drone_lon, gps_lat, gps_lon, est_lat, est_lon, err_m, score, mode_tag, agl_m, n_vo, elapsed_ms
```

`gps_lat`/`gps_lon` come from `/mavros/global_position/global` (ground truth); `err_m` is the great-circle distance to `est_lat`/`est_lon`. `mode_tag`: plan A writes `ANYLOC` on retrieval frames and `VO +Nf` in between; plan B writes `ANYLOC-ACC` (correction blended in), `ANYLOC-REJ` (score below gate), `ANYLOC-JREJ` (score passed but jump implausible — likely false match), `ANYLOC-REACQ` (agreeing far candidates forced a relocation), or `VO`. The plan-B tags + `score` column are what you use to recalibrate `--gate` after a shadow flight; frequent `JREJ` runs ending in `REACQ` back at the same spot suggest `--jump-base` is too tight, while isolated `JREJ`s are the filter doing its job. Not committed to git (`anyloc/logs/` is gitignored).

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

- **Phase 1** (AGL < 50 m): sends home-anchor VPE at (0, 0), cov = 0.5 m²
- **Phase 2** (AGL ≥ 50 m): sends the estimate, cov = max(1, err_m²)

VPE metres are computed relative to the commander's `local_frame_ref()` (EKF origin / arm GPS — see `control/README.md`), not the hardcoded HOME.

In the scripted-survey flow the commander also switches EKF source SRC1→SRC2 automatically at cruise altitude; in the Mission-Planner-AUTO flow you flip the RC aux switch yourself.

> **Known landmine (unfixed):** Phase 2 only accepts estimates with `error_m < 100`, and `error_m` is measured against the last GPS fix. Under real GPS jamming the fix freezes, so >100 m from the jam point every estimate gets rejected and the thread republishes a stale position. Replace this gate before a real GPS-denied flight.

---

## 6. Troubleshooting

| Symptom | Fix |
|---|---|
| `[PostView] Waiting for first frame` / black window | Camera not running — start `launch_camera.sh` first |
| `ImportError: No module named 'faiss'` | Use `/home/jetson/venv/anyloc/bin/python3`, not system python |
| `latest_estimate.json` not updating | Normal below 50 m AGL in normal mode; use `--test` to bypass |
| Database not found | Check symlink: `ls -la anyloc/database` → should point to `database_zone_vits14` |
| Wrong model loaded | `model_name` is in `database_meta.pt`; localizer reads it automatically |
| Postview window black on Jetson screen | Use `--stream-host` to stream to ground PC, or `ssh -X` for X11 forwarding |
