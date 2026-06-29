# How to Run — Jetson Real Hardware

**Target:** Jetson Orin NX + ArduPilot FC (USB-to-TTL on **Serial6**, `/dev/ttyUSB0:921600`) + AP-IMX900 camera (`/dev/video0`)
**ROS2:** Humble (`/opt/ros/humble`)
**Python envs:** `/home/jetson/venv/anyloc` (torch + faiss) · `/home/jetson/venv/yolo` (torch + ultralytics)
**Goal:** GPS-denied autonomous survey: takeoff 65 m → boustrophedon pattern → YOLO detection → land

---

## Architecture

```
Mission Planner (PC)
  └─ survey.waypoints ──scp──▶  Jetson Orin NX
                                  launch_real_hw.sh
                                  ├─ [1] launch_mavros_real.sh       → serial:///dev/ttyUSB0:921600
                                  ├─ [2a] launch_camera.sh           → /dev/video0 → /drone/camera/image_raw   (no ground stream)
                                  │  OR
                                  │  [2b] ground_view_stream.py      → /dev/video0 → /drone/camera/image_raw   (+ H.265 stream)
                                  │         --stream-host GS_IP      →   RTP/UDP → ground station
                                  │         --stream-server SERVER_IP →   RTSP push → MediaMTX relay
                                  ├─ [3] hw_bridge.py                → /mavros/local_position/pose → /drone/state /drone/pose /drone/agl
                                  ├─ [4] anyloc/ros2_node.py         → venv/anyloc → /drone/camera/image_raw → AnyLoc VPE
                                  ├─ [5] detection/ros2_node.py      → venv/yolo   → /drone/camera/image_raw → detections.csv
                                  └─ [6] ardupilot_commander.py      → VPE → GUIDED → survey
                                         ↕ MAVLink
                              /dev/ttyUSB0:921600
                                         ↕
                             ArduPilot FC (real_hw.parm: SRC1=GPS, SRC2=ExternalNav)
```

VPE phases:
- **Phase 1** (ground → 50 m AGL): commander anchors EKF at home (0, 0) — static hold
- **Phase 2** (≥ 50 m AGL): AnyLoc DINOv2+VLAD visual estimates fused into EKF3

EKF source switching (real_hw.parm):
- **SRC1** (RC switch LOW): GPS — used for arming and takeoff
- **SRC2** (RC switch HIGH): ExternalNav/AnyLoc — flip at cruise altitude once AnyLoc is confident

---

## One-Time Setup (do once per Jetson)

### 1. Permissions for serial and camera

```bash
sudo usermod -aG dialout $USER   # serial access — logout + login after
# Or per-session:
sudo chmod 666 /dev/ttyUSB0
sudo chmod 666 /dev/video0
```

### 2. Verify ROS2 and MAVROS

```bash
source /opt/ros/humble/setup.bash
ros2 pkg list | grep mavros   # should show: mavros  mavros_extras  mavros_msgs
```

### 3. Verify Python venvs

```bash
/home/jetson/venv/anyloc/bin/python3 -c "import torch, faiss; print('anyloc venv OK')"
/home/jetson/venv/yolo/bin/python3  -c "import torch, ultralytics; print('yolo venv OK')"
```

### 4. Upload ArduPilot parameters to FC

Upload `control/real_hw.parm` — **not** `no_gps.parm` (that has SITL-only settings).

```bash
# Via Mission Planner: Config → Full Parameter List → Load from file → real_hw.parm
# Or via MAVProxy:
mavproxy.py --master=/dev/ttyUSB0,921600
  > param load control/real_hw.parm
  > param save
  > reboot
```

Key parameters in `real_hw.parm`:

| Parameter | Value | Why |
|---|---|---|
| `GPS_TYPE` | 1 | GPS enabled — used for SRC1 arming |
| `EK3_SRC1_POSXY` | 3 | SRC1 = GPS (arm + takeoff) |
| `EK3_SRC2_POSXY` | 6 | SRC2 = ExternalNav/AnyLoc (survey) |
| `VISO_TYPE` | 1 | MAVLink visual odometry enabled |
| `BRD_SAFETYENABLE` | 1 | Physical safety button required |
| `PSC_NE_VEL_I` | 0.0 | Must be 0 — non-zero causes integral windup |
| `GUID_TIMEOUT` | 30 | Prevents failsafe on Jetson CPU spikes |
| `EK3_GLITCH_RAD` | 50 | Accept AnyLoc jumps up to 50 m |
| `ARMING_CHECK` | 0 | Skip software pre-arm (physical safety switch is protection) |

Set RC aux switch for EKF source:
- In Mission Planner: Config → Full Parameter List → find `RCx_OPTION` on a 2/3-pos switch → set to **90** (EKF Source Select)
- Switch LOW → SRC1 (GPS), Switch HIGH → SRC2 (ExternalNav)

### 5. AnyLoc database

Database lives at `anyloc/database_vits14/` (satellite tiles) with a symlink `anyloc/database → anyloc/database_vits14` (already created).

To rebuild satellite database for a different site:
```bash
# Update CENTER_LAT / CENTER_LON in anyloc/build_database.py
/home/jetson/venv/anyloc/bin/python3 anyloc/build_database.py --rebuild
ls -lh anyloc/database_vits14/   # expect database.pt, database_vlads.pt, db_meta.json, db_images/
```

To build a **real-field database** from actual drone footage (better match at inference time):
```bash
# 1. Record survey flight (MAVROS only, no launch_camera.sh)
# Frames are rotated 180° automatically.
source /opt/ros/humble/setup.bash
# Direct UDP to ground station:
python3 tools/record_field.py --output field_data/survey1 --stream-host <GS_IP>
# Or push RTSP to MediaMTX relay (watch in VLC/browser, no GStreamer on ground station):
python3 tools/record_field.py --output field_data/survey1 --stream-server 118.232.160.227
# → writes field_data/survey1/video.mkv  telemetry.csv  meta.json
# MKV format: stays playable even after power-off mid-flight

# 2. Extract geo-tagged frames
python3 tools/extract_frames.py field_data/survey1/ --rotate --min-dist 25

# 3. Build database
/home/jetson/venv/anyloc/bin/python3 anyloc/build_database_real.py field_data/survey1/

# 4. Activate
ln -sfn database_real anyloc/database
```
See `instructions/field_database_collection.md` for the complete guide (flight plan, FOV/overlap analysis, terminal setup).

---

## Before Every Flight

### Step A — Update home position

Edit `control/home_elevation.json` with the **actual contest takeoff point**:

```json
{"lat": 23.450868, "lon": 120.286135, "centre_elev_m": 28.17}
```

Get values from Mission Planner → right-click takeoff point → elevation shown in status bar.

### Step B — Load survey waypoints from Mission Planner

On the Mission Planner PC:
1. Flight Plan → right-click → Survey (Grid)
2. Draw polygon, set altitude 65 m relative, turn radius 0 m
3. File → Save Waypoints → `survey.waypoints`
4. Copy to Jetson:

```bash
scp survey.waypoints jetson@JETSON_IP:~/Ardupilot_no_GPS_drone_project/control/
```

Verify on Jetson:

```bash
cd ~/Ardupilot_no_GPS_drone_project
python3 -c "
import sys, json; sys.path.insert(0,'control')
from mission_loader import load_mission_planner_waypoints
h = json.load(open('control/home_elevation.json'))
wps = load_mission_planner_waypoints('control/survey.waypoints', h['lat'], h['lon'], h['centre_elev_m'])
print(f'{len(wps)} waypoints loaded')
"
```

### Step C — Hardware checks

```bash
# FC adapter
ls /dev/ttyUSB*       # expect /dev/ttyUSB0

# Camera
ls /dev/video*        # expect /dev/video0 and /dev/video1 (video1 is metadata — use video0)

# AnyLoc database
ls anyloc/database/database_vlads.pt && echo "DB OK" || echo "DB MISSING — rebuild"

# Kill any stale camera processes before starting
pkill -f v4l2_camera_node 2>/dev/null; echo "camera clear"
```

---

## Launch Sequence

### Option 1 — One command (recommended)

```bash
cd ~/Ardupilot_no_GPS_drone_project

# No ground stream (camera only)
bash control/launch_real_hw.sh --manual-takeoff --waypoint-file control/survey.waypoints

# With ground view stream — direct UDP to ground station (ZeroTier / LAN)
bash control/launch_real_hw.sh --manual-takeoff --waypoint-file control/survey.waypoints \
    --stream-host 10.181.156.237

# With ground view stream — RTSP push to MediaMTX relay (LTE / internet)
bash control/launch_real_hw.sh --manual-takeoff --waypoint-file control/survey.waypoints \
    --stream-server 118.232.160.227
```

`--stream-host` and `--stream-server` are mutually exclusive. Both replace `launch_camera.sh` with `ground_view_stream.py`, which opens the camera and also publishes `/drone/camera/image_raw`.

Launch order with waits:

| Step | Process | Python | Wait |
|---|---|---|---|
| 1 | `launch_mavros_real.sh` → MAVROS @ ttyUSB0:921600 | system | 6 s |
| 2 | `launch_camera.sh` **or** `ground_view_stream.py` → `/drone/camera/image_raw` (+ optional stream) | system / python3 | 3 s |
| 3 | `hw_bridge.py` → `/drone/state`, `/drone/pose`, `/drone/agl` | system python3 | 2 s |
| 4 | `anyloc/ros2_node.py --headless` → AnyLoc VPE | `venv/anyloc` | 4 s |
| 5 | `detection/ros2_node.py --headless` → YOLO | `venv/yolo` | 2 s |
| 6 | `ardupilot_commander.py` → mission (foreground) | system python3 | — |

Ctrl+C kills the commander; script then kills all background processes.

### Option 2 — tmux (recommended for contest — see each process separately)

```bash
tmux new-session -s flight

# Pane 0: MAVROS
bash control/launch_mavros_real.sh

# Ctrl-B " to split panes. In each new pane:

# Pane 1: Camera — choose ONE of the following:

#   No stream (camera driver only)
bash control/launch_camera.sh

#   OR: Ground view stream — direct UDP (ZeroTier / LAN)
#   Shows YOLO live + AnyLoc match + 3 most recent detection crops at 1280×720
#   Receive: gst-launch-1.0 udpsrc port=5000 ! ... (see tools/README.md)
source /opt/ros/humble/setup.bash
python3 tools/ground_view_stream.py --host <GROUND_IP>

#   OR: Ground view stream — RTSP push to MediaMTX relay (LTE / internet)
#   Watch: vlc rtsp://118.232.160.227:8554/drone  OR  http://118.232.160.227:8889/drone
source /opt/ros/humble/setup.bash
python3 tools/ground_view_stream.py --stream-server 118.232.160.227

# Pane 2: HW Bridge
source /opt/ros/humble/setup.bash
python3 control/hw_bridge.py

# Pane 3: AnyLoc (~20 min startup — loading VLAD database)
bash anyloc/run_ros2_localizer.sh --headless

# Pane 4: YOLO
bash detection/run_ros2_detector.sh --headless

# Pane 5: Commander
source /opt/ros/humble/setup.bash
python3 control/ardupilot_commander.py --manual-takeoff --waypoint-file control/survey.waypoints

# Pane 6: EKF monitor (optional — separate terminal)
source /opt/ros/humble/setup.bash
python3 tools/ekf_monitor.py
```

### Option 3 — AnyLoc + EKF ground test (`--test` mode, no flying needed)

Minimal 4-terminal test to verify AnyLoc runs and EKF accepts VPE — **no hw_bridge, no commander needed**:

```bash
# T1: MAVROS
bash control/launch_mavros_real.sh

# T2: Camera
bash control/launch_camera.sh

# T3: AnyLoc in test mode (bypasses 50 m AGL gate, sends VPE to MAVROS directly)
bash anyloc/run_ros2_localizer.sh --test

# T4: EKF monitor
source /opt/ros/humble/setup.bash && python3 tools/ekf_monitor.py
```

Flip RC aux switch to HIGH (SRC2 = ExternalNav) and watch `ekf_monitor` for `✓ POS_ABS accepted`.

`--test` mode differences from normal:
- AGL gate bypassed — AnyLoc runs on every camera frame
- Fake AGL = 65 m used for scale when on ground (`--test-agl N` to change)
- VPE published directly to `/mavros/vision_pose/pose_cov` — no commander needed
- `hw_bridge.py` not required

---

## Manual Takeoff Mode (`--manual-takeoff`)

All flying is done with your RC. The only thing Jetson does is send VPE to the FC.

```bash
python3 control/ardupilot_commander.py --manual-takeoff
```

Commander prints and waits up to 10 min:
```
[APCmd] === MANUAL TAKEOFF MODE ===
[APCmd]   1. Arm with RC (GPS — STABILIZE or LOITER)
[APCmd]      Jetson will set EKF origin from GPS the moment you arm
[APCmd]   2. Climb to cruise altitude (~65 m AGL)
[APCmd]   3. Flip RC aux switch HIGH → SRC2 (ExternalNav/VPE)
[APCmd]   4. Switch FC to GUIDED — survey starts automatically
[APCmd] waiting for arm …
```

When you arm, the commander reads your live GPS position and sets EKF origin from it:
```
[APCmd] Armed ✓  GPS: 23.450912 N  120.286201 E  28.3 m MSL
[APCmd] EKF origin set from live GPS ✓
[APCmd] waiting for GUIDED + AGL > 5 m …
```

Once you switch to GUIDED at altitude, commander starts the survey automatically.

**Your RC sequence:**

| You do | Commander does |
|---|---|
| Arm in STABILIZE/LOITER | Sets EKF origin from GPS, publishes VPE at 20 Hz |
| Fly to 65 m AGL | Publishing VPE at 20 Hz |
| Flip RC aux switch HIGH → SRC2 | Publishing VPE at 20 Hz |
| Switch FC to GUIDED | Detects GUIDED → runs survey |

---

## What to Watch During Startup

### MAVROS — expect within 5 s

```
[mavros_real] Connecting to ArduPilot FC at /dev/ttyUSB0:921600 ...
[mavros_router]: link[1000] detected remote address 1.1
[mavros.sys]: VER: 1.1: Flight software: ...
[mavros_real] Waiting for MAVROS to connect...
[mavros_real] Connected — requesting data streams at 10 Hz...
[mavros_real] Data streams enabled.
```

> **Why the stream rate step:** MAVROS sends `REQUEST_DATA_STREAM rate=0` on startup which clears ArduPilot's SR6_* flash params. The launch script re-requests all stream types individually (IDs 1–4, 6, 10–12) after connection — without this, all `/mavros/*` data topics stay silent. `stream_id=0` (ALL) alone is insufficient.

### Camera — expected warnings (harmless)

```
[WARN] slow conversion: yuv422_yuy2 => rgb8   ← expected, YUYV 1280×960→rgb8 conversion
[ERROR] Camera calibration file ... not found  ← harmless, AnyLoc doesn't need intrinsics
```
Confirm images flow: `ros2 topic hz /drone/camera/image_raw` → expect ~30 Hz

### Commander — startup sequence

```
[APCmd] HOME_ALT_MSL = 28.2 m
[APCmd] 14 waypoints from survey.waypoints
[APCmd] MAVROS connected ✓
[APCmd] vision thread started (Phase 1 — home-anchor)   ← VPE publishing at 20 Hz

── manual-takeoff mode ──
[APCmd] === MANUAL TAKEOFF MODE ===
[APCmd] waiting for arm …
[APCmd] Armed ✓  GPS: 23.450912 N  120.286201 E  28.3 m MSL
[APCmd] EKF origin set from live GPS ✓
[APCmd] waiting for GUIDED + AGL > 5 m …

── auto mode ──
[APCmd] Armed ✓
[APCmd] GUIDED ✓
[APCmd] switching EKF source → SRC2 (ExternalNav/AnyLoc) …
[APCmd] AGL 65 m ≥ 50 m — VPE → AnyLoc               ← Phase 2 activates
[APCmd] SURVEY WP 1/14 ...
```

### EKF monitor

```bash
source /opt/ros/humble/setup.bash
python3 tools/ekf_monitor.py
```

When EKF accepts VPE:
```
flags=0x037  ✓ POS_ABS accepted
  active : ATTITUDE, VEL_HORIZ, VEL_VERT, POS_REL, POS_ABS, POS_VERT
  var    : vel=0.08  pos_h=0.12  pos_v=0.11  compass=0.01
```

`POS_ABS` in active list + `pos_h` variance < 0.5 = EKF healthy.

### Ground view stream — composite YOLO + AnyLoc viewport

`tools/ground_view_stream.py` streams a 1280×720 composite viewport showing YOLO live detection feed, AnyLoc match tile, and the last 3 detection crops with class/location labels. It also publishes `/drone/camera/image_raw` — so it **replaces** `launch_camera.sh` (do not run both).

**Mode A — direct UDP (ZeroTier / LAN):**
```bash
# Replace Pane 1 (camera) with:
source /opt/ros/humble/setup.bash
python3 tools/ground_view_stream.py --host 192.168.1.50

# Or via launch script:
bash control/launch_real_hw.sh --stream-host 192.168.1.50 --manual-takeoff
```

Receive on ground station:
```bash
gst-launch-1.0 udpsrc port=5000 ! \
    application/x-rtp,encoding-name=H265,payload=96 ! \
    rtph265depay ! h265parse ! avdec_h265 ! \
    videoconvert ! autovideosink sync=false
```

**Mode B — RTSP push to MediaMTX relay (LTE / internet, no GStreamer on receiver):**
```bash
source /opt/ros/humble/setup.bash
python3 tools/ground_view_stream.py --stream-server 118.232.160.227

# Or via launch script:
bash control/launch_real_hw.sh --stream-server 118.232.160.227 --manual-takeoff
```

Watch on any device — no install needed:
```
VLC:     rtsp://118.232.160.227:8554/drone
Browser: http://118.232.160.227:8889/drone  (WebRTC ~200 ms)
Browser: http://118.232.160.227:8888/drone  (HLS ~5 s, works on mobile)
```

Bitrate: 1 Mbps H.265, keyframe every 1 s. Override with `--bitrate N`.

**Simple camera-only stream** (no YOLO, no detection crops — `gstreamer_stream.py`):
```bash
# Only if running launch_camera.sh separately (not ground_view_stream.py):
bash control/launch_gstreamer.sh --host 192.168.1.50
```

### VPE topic verify

```bash
source /opt/ros/humble/setup.bash
ros2 topic hz /mavros/vision_pose/pose_cov    # expect 20 Hz
ros2 topic echo /mavros/vision_pose/pose_cov --once   # check x,y,z
```

### AnyLoc estimate

```bash
cat anyloc/latest_estimate.json
# {"north_m": ..., "east_m": ..., "error_m": ..., "agl_m": ...}
```

### Detection output

```bash
tail -f detections.csv
```

---

## Contest Day Checklist

```
T-30 min
  [ ] scp survey.waypoints from Mission Planner PC to Jetson control/
  [ ] Verify waypoints load (python3 verify command — see Step B above)
  [ ] Update control/home_elevation.json with actual contest site lat/lon/elev
  [ ] Confirm AnyLoc database matches contest site
  [ ] Confirm RC aux switch channel set to RCx_OPTION=90 (EKF Source Select)

T-15 min
  [ ] Power on Jetson, connect USB-to-TTL adapter and camera
  [ ] Verify /dev/ttyUSB0 and /dev/video0 present
  [ ] pkill -f v4l2_camera_node (clear stale camera processes)
  [ ] Start: bash control/launch_real_hw.sh --manual-takeoff (or tmux layout)
  [ ] MAVROS pane: "detected remote address 1.1" ✓
  [ ] Camera pane: running (slow conversion warning OK)
  [ ] ros2 topic hz /drone/camera/image_raw → ~30 Hz ✓
  [ ] HW Bridge pane: "HW bridge ready" ✓
  [ ] AnyLoc pane: "AnyLoc node ready" ✓
  [ ] YOLO pane: "YOLO Waiting for image" ✓
  [ ] Commander pane: "MAVROS connected ✓" and "waiting for GUIDED mode" ✓
  [ ] EKF monitor: SRC1 (GPS, RC switch LOW) → "POS_ABS accepted" from GPS ✓

T-5 min
  [ ] VPE publishing: ros2 topic hz /mavros/vision_pose/pose_cov → 20 Hz ✓
  [ ] AnyLoc estimate updating: cat anyloc/latest_estimate.json ✓

Contest start — jammer ON (GPS LOST — expected)
  [ ] Press physical safety button on FC → LED green
  [ ] Arm with RC in STABILIZE or LOITER (GPS)
  [ ] Fly manually to 65 m AGL
  [ ] Flip RC aux switch to HIGH → SRC2 (ExternalNav/AnyLoc)
  [ ] Watch EKF monitor: "✓ POS_ABS accepted" from VPE
  [ ] Switch FC to GUIDED → commander takes over survey automatically
  [ ] Monitor: "WP 01/N →" for each waypoint
  [ ] detections.csv populated (tail -f)
  [ ] "Survey complete" → RTL/LAND → disarm

Emergency
  RC transmitter: switch to STABILIZE → full manual control
  Ctrl+C in commander: sends velocity=0, drone holds → then RC to LAND
```

---

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| All `/mavros/*` data topics silent (no IMU, pose, altitude) | MAVROS reset SR6_* stream rates on startup | `launch_mavros_real.sh` handles this automatically — if running manually, call `ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 6, message_rate: 10, on_off: true}"` for each stream ID 1–4, 6, 10–12 |
| `/dev/ttyUSB0` permission denied | Not in `dialout` group | `sudo chmod 666 /dev/ttyUSB0` |
| `/dev/ttyUSB0` not found | Adapter unplugged or driver missing | `dmesg \| tail -20` → look for cp210x/ch341 |
| `launch_mavros_real.sh` exits with no output | `set -e` + stale pkill returning 1 | Fixed; if recurs: check script has `pkill ... \|\| true` |
| MAVROS not connecting | Wrong baud or device | Try `FCU_DEV=/dev/ttyUSB1 bash control/launch_mavros_real.sh`; verify FC serial baud = 921600 |
| Camera "Device or resource busy" | Stale v4l2 instance | `pkill -f v4l2_camera_node; sleep 1; bash control/launch_camera.sh` |
| Camera calibration file not found | No intrinsics YAML | Harmless — AnyLoc doesn't use camera intrinsics |
| AnyLoc cv_bridge error "Unrecognized encoding" | Wrong pixel_format in launch_camera.sh | Verify `pixel_format:=YUYV` and `output_encoding:=rgb8` in launch_camera.sh |
| Arm rejected: Safety Switch | Safety button not pressed | Press physical button; LED must go green |
| Arm rejected: Need Position | VPE not publishing or EKF not converged | Check `ros2 topic hz /mavros/vision_pose/pose_cov` (expect 20 Hz); wait 30 s |
| EKF never reaches POS_ABS on SRC2 | ExternalNav params wrong | Verify `VISO_TYPE=1`, `EK3_SRC2_POSXY=6` in FC params; check VPE topic hz |
| EKF failsafe during survey | AnyLoc jump > glitch radius | Verify `EK3_GLITCH_RAD=50` in real_hw.parm |
| AnyLoc venv import error | Wrong Python used | Confirm script uses `/home/jetson/venv/anyloc/bin/python3` |
| YOLO venv import error | Wrong Python used | Confirm script uses `/home/jetson/venv/yolo/bin/python3` |
| AnyLoc not activating | AGL below 50 m threshold | Normal — activates above `MIN_AGL=50`; use `--test` flag to bypass on ground |
| AnyLoc database error | Wrong path | Check symlink: `ls -la anyloc/database` → should point to `database_vits14` |
| Survey strips curved | Wrong setpoint type | `go_to_ned()` uses velocity setpoints — don't switch to position setpoints during survey |
| Drone drifts in hover | `PSC_NE_VEL_I` non-zero | Verify `PSC_NE_VEL_I=0.0` in uploaded params |
| No detections logged | YOLO not running or AGL < 50 m | Check YOLO pane; verify `MIN_AGL=50.0` in `detection/ros2_node.py` |
| Wrong survey area | `home_elevation.json` mismatch | Update lat/lon/elev to actual takeoff point |
| Commander hangs at "waiting for drone state" | `hw_bridge.py` not running | hw_bridge must start before commander |
| GStreamer "no such element: nvv4l2h265enc" | Missing Jetson GStreamer plugins | `sudo apt install nvidia-l4t-gstreamer` |
| GStreamer stream no video on receiver | Firewall or wrong IP | Check `--host` matches ground PC IP; open port 5000/udp |
| GStreamer "Camera not running" | Camera pane not started | Start `launch_camera.sh` before `launch_gstreamer.sh` |
| GStreamer "appsrc push returned GST_FLOW_ERROR" | Ground IP unreachable | Ping ground station first; udpsink drops silently |
| `ground_view_stream.py` "Cannot open camera" | `launch_camera.sh` already running | Kill it first — both cannot open `/dev/video0` simultaneously |
| `ground_view_stream.py` YOLO panel shows no boxes | YOLO node not started yet | Wait for YOLO node to load model (~30 s); boxes appear once AGL > 50 m |
| `ground_view_stream.py` AnyLoc panel black | AnyLoc node not running or no match yet | Wait for first AnyLoc match; `anyloc/latest_match.jpg` must exist |
| `ground_view_stream.py` RTSP: `rtspclientsink not found` | Missing GStreamer RTSP plugin | `sudo apt install gstreamer1.0-rtsp` |
| `ground_view_stream.py` RTSP: connection refused | MediaMTX server not running | Start `./mediamtx mediamtx.yml` on Frank's PC; verify port 8554 open |

---

## Key Invariants

1. **ENU everywhere**: setpoints use `x=East, y=North, z=Up`. MAVROS converts to NED — never send raw NED.
2. **VPE yaw = π/2**: ENU yaw π/2 → MAVROS → NED yaw=0 (North). AnyLoc's yaw output is always ignored.
3. **`PSC_NE_VEL_I = 0.0`**: default 1.0 causes integral windup under ExternalNav. Must be zero.
4. **`GUID_TIMEOUT = 30`**: default 3 s causes failsafe on Jetson CPU spikes during VPE inference.
5. **Safety button required**: `BRD_SAFETYENABLE=1`; force-arm (`ALLOW_FORCE_ARM=1`) is SITL only.
6. **`home_elevation.json` must match takeoff point**: all waypoints are ENU offsets from this origin.
7. **`real_hw.parm` not `no_gps.parm`**: `no_gps.parm` has SITL-only entries — never upload to real FC.
8. **hw_bridge before commander**: commander waits for `/drone/state`; hw_bridge must be running first.
9. **AnyLoc fuses at ≥ 50 m only**: `MIN_LOCALISATION_AGL=50.0` in commander and `MIN_AGL=50.0` in anyloc node must match.
10. **venv/anyloc for AnyLoc, venv/yolo for YOLO**: system Python3 lacks torch/faiss/ultralytics. Do not use `conda run`.
11. **Kill stale camera processes**: running `launch_camera.sh` twice causes "Device busy" — always `pkill -f v4l2_camera_node` first.
