# How to Run — Jetson Real Hardware

**Target:** Jetson Orin NX + ArduPilot FC (USB-to-TTL on **Serial6**, `/dev/ttyUSB0:921600`) + IMX219 CSI camera (nvarguscamerasrc, sensor-id 0; raw node `/dev/video0`)
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
                                  ├─ [2a] launch_camera.sh           → CSI (nvarguscamerasrc) → /drone/camera/image_raw   (no ground stream)
                                  │  OR
                                  │  [2b] ground_view_stream.py      → subscribes /drone/camera/image_raw (needs 2a running) (+ H.265 stream)
                                  │         --stream-host GS_IP      →   RTP/UDP → ground station
                                  │         --stream-server SERVER_IP →   RTSP push → MediaMTX relay
                                  ├─ [3] hw_bridge.py                → /mavros/local_position/pose → /drone/state
                                  │                                    /mavros/global_position/global, rel_alt → /drone/pose, /drone/agl
                                  ├─ [4] anyloc/ros2_node.py         → venv/anyloc → /drone/camera/image_raw → AnyLoc VPE
                                  │      (plan A — launch_real_hw.sh default; Desktop full_run.sh uses
                                  │       plan B anyloc/ros2_node_vo_primary.py, VO-primary + score/jump gates — preferred)
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
sudo usermod -aG video $USER     # CSI/nvargus camera access — logout + login after
# Or per-session:
sudo chmod 666 /dev/ttyUSB0
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
| `FRAME_CLASS` / `FRAME_TYPE` | 2 / 1 | Hexarotor X — matches the real airframe (was wrongly quad-X in this file until 2026-07-06; the FC itself was always correct) |
| `GPS_TYPE` | 1 | GPS enabled — used for SRC1 arming |
| `EK3_SRC1_POSXY` | 3 | SRC1 = GPS (arm + takeoff) |
| `EK3_SRC2_POSXY` | 6 | SRC2 = ExternalNav/AnyLoc (survey) |
| `EK3_SRC2_VELXY` | 0 | **No velocity on SRC2** — the Jetson vision_speed is differentiated EKF output on real hw (circular); IMU + 20 Hz VPE position suffices. Was 6 in SITL only. |
| `EK3_SRC2_YAW` | 1 | Compass on SRC2 — never 6; VPE yaw is hardcoded North, not a measurement |
| `VISO_TYPE` | 1 | MAVLink visual odometry enabled |
| `BRD_SAFETYENABLE` | 1 | Physical safety button required |
| `PSC_NE_VEL_I` | 0.0 | Must be 0 — non-zero causes integral windup |
| `GUID_TIMEOUT` | 30 | Prevents failsafe on Jetson CPU spikes |
| `EK3_GLITCH_RAD` | 50 | Accept AnyLoc jumps up to 50 m |
| `ARMING_CHECK` | 0 | Skip software pre-arm (physical safety switch is protection) |
| `SERIAL6_OPTIONS` | 1024 | Don't forward MAVLink to/from the Jetson port. Without it the FC copies the commander's 20 Hz VPE broadcasts onto the telemetry radio/USB, and Mission Planner hangs on "Getting Params". Needs FC reboot to take effect |
| `SR3_*` (EXT_STAT, EXTRA1-3, POSITION, RAW_SENS, RC_CHAN) | 10 | Persistent 10 Hz stream rates for the Jetson link (SERIAL6 = 4th MAVLink port → SR**3**). Keeps telemetry flowing after an FC reboot mid-session — the launch script's stream request is runtime-only |

Set RC aux switch for EKF source:
- In Mission Planner: Config → Full Parameter List → find `RCx_OPTION` on a 2/3-pos switch → set to **90** (EKF Source Select)
- Switch LOW → SRC1 (GPS), Switch HIGH → SRC2 (ExternalNav)

### 5. AnyLoc database

Active database is `anyloc/database_zone_z20_vits14/` (NLSC zoom-20 satellite tiles ≈ 0.19 m/px, right-sized to the mission zone, 882 entries) with a symlink `anyloc/database → anyloc/database_zone_z20_vits14` (already created). Fallbacks kept on disk: `anyloc/database_zone_vits14/` (same 882-entry grid, coarser zoom-18) and the old full-radius `anyloc/database_vits14/` (2821 entries, 1502 m circle) — `ln -sfn database_zone_vits14 anyloc/database` etc. to switch back.

To rebuild satellite database for a different site or zone:
```bash
# Update CENTER_LAT / CENTER_LON in anyloc/build_database.py if the site changed.
# --model vits14 is required (default is vitb14, the wrong backbone for this project).
# --n-min/--n-max/--e-min/--e-max (metres from CENTER_LAT/LON) build a rectangular
# region instead of the default full-radius circle — use this to stay zone-sized.
# The values below are specific to the CURRENT mission zone + 20% margin — recompute
# them (see tools/gen_contest_survey.py's bounding-box math) if the zone changes.
# --sat-zoom 20 needs --grid-radius-m to shrink the tile fetch (MAX_TEX cap) and an
# explicit --sat-path (else it silently reuses the zoom-18 simulator mosaic).
/home/jetson/venv/anyloc/bin/python3 anyloc/build_database.py --model vits14 \
    --db-dir anyloc/database_zone_z20_vits14 \
    --n-min -262.2 --n-max 737.8 --e-min -1501.3 --e-max 548.7 \
    --agl-min 65 --agl-max 65 --sat-zoom 20 --grid-radius-m 780 \
    --sat-path anyloc/database_zone_z20_vits14/satellite.jpg --rebuild
ls -lh anyloc/database_zone_z20_vits14/   # expect database.pt, database_vlads.pt, db_meta.json, db_images/
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
# → writes field_data/survey1/video.mkv  telemetry.csv  meta.json  frame_times.csv
# MKV format: stays playable even after power-off mid-flight

# 2. Extract geo-tagged frames
python3 tools/extract_frames.py field_data/survey1/ --rotate --min-dist 25

# 3. Build database
/home/jetson/venv/anyloc/bin/python3 anyloc/build_database_real.py field_data/survey1/

# 4. Activate
ln -sfn database_real anyloc/database
```
See `instructions/field_database_collection.md` for the complete guide (flight plan, FOV/overlap analysis, terminal setup).

### 6. YOLO TensorRT engine

`YOLODetector` (`detection/detector.py`) runs the model as a TensorRT FP16 engine, not the raw `.pt` file. The engine is auto-exported next to the `.pt` weights the first time `detection/ros2_node.py` loads them — takes ~15 min on Orin NX and blocks that pane's startup. Pre-build it once so contest-day launches don't stall:

```bash
/home/jetson/venv/yolo/bin/python3 -c "from detection.detector import YOLODetector; YOLODetector('Car_visdrone1280.pt', conf=0.50)"
ls -lh Car_visdrone1280.engine   # confirm it exists before flight day
```

Re-run this (or just delete `Car_visdrone1280.engine`) any time the `.pt` weights are retrained/replaced — a stale engine built from old weights will silently keep being used since the cache check is by filename only.

**`jetson_clocks` matters more than the engine.** `nvpmodel MAXN_SUPER` only raises the clock ceiling — it doesn't force max clocks. Without `jetson_clocks`, DVFS throttling gives back most of the TensorRT speedup (measured 47 ms/frame vs 18.6 ms/frame with clocks locked). **Automated as of 2026-07-08** via `control/jetson_clocks.service` (systemd, ordered after `nvpmodel.service`) — runs on every boot automatically. To (re)install it (e.g. after an SSD reflash):
```bash
sudo cp control/jetson_clocks.service /etc/systemd/system/
sudo systemctl daemon-reload && sudo systemctl enable --now jetson_clocks.service
```
Verify before flying with `systemctl is-active jetson_clocks.service` (expect `active`) rather than re-running the command by hand.

### 7. MAVLink relay over the internet (optional — backup GCS link)

Gives Mission Planner a second path to the FC (telemetry **and** arm/RTL/mode/param control) via the Jetson's own FC link (`SERIAL6`) and Frank's PC (`118.232.160.227`), for when the 915MHz SiK radio is out of range. Full setup in `streaming/mavlink_relay_setup.md`; summary:

```bash
# Once, any machine with openssl:
bash streaming/generate_relay_certs.sh 118.232.160.227
# Copy the right cert/key files to Frank's PC, the Jetson, and the MP machine (see doc).

# Frank's PC:
python3 streaming/mavlink_relay_server.py

# Jetson — either standalone or via --mavlink-relay in the Launch Sequence below:
python3 control/mavlink_relay_client.py --role vehicle

# MP machine:
python3 control/mavlink_relay_client.py --role controller
# Then in Mission Planner: add a TCP connection to 127.0.0.1:5760, alongside the radio link.
```

**Why this doesn't use FC-level MAVLink signing:** ArduPilot's signing is all-or-nothing across every serial port ([confirmed via source + an exact-match upstream issue](https://github.com/ArduPilot/ardupilot/issues/28736)) — turning it on would also require mavros's unsigned `SERIAL6` VPE/EKF feed to be signed, which mavros/libmavconn has no support for, and would break the whole no-GPS localization stack. Instead, both relay hops are authenticated with mutual TLS (private CA, client certs) — the FC and mavros are never touched or made aware this exists.

**This is a backup, not a replacement for the radio.** It depends on Jetson LTE + Frank's PC being reachable. Bench-test (props off) before ever trusting arm/RTL through it — see the testing checklist in `streaming/mavlink_relay_setup.md`.

**Known limitation:** Mission Planner's Messages tab (prearm/warning `STATUSTEXT`) stays empty on this connection — telemetry and control both work fully, but `SERIAL6_OPTIONS=1024` (step 4 of "Upload ArduPilot parameters", set to stop the commander's VPE stream from flooding the radio) also excludes SERIAL6 from ArduPilot's `STATUSTEXT` distribution (verified via source — one option bit controls both). This is a permanent tradeoff, not a bug; see `streaming/mavlink_relay_setup.md` for details. Don't clear `SERIAL6_OPTIONS` to fix it — that reopens the VPE-flooding problem.

Cross-machine tested 2026-07-08: both the vehicle leg (Jetson) and controller leg (MP machine) confirmed carrying live telemetry and control over the real deployed link.

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

# Camera (CSI — check the sensor is detected by the Argus/tegra-camera stack)
v4l2-ctl --list-devices 2>&1 | grep -i imx219   # expect "vi-output, imx219 ..." → /dev/video0

# AnyLoc database
ls anyloc/database/database_vlads.pt && echo "DB OK" || echo "DB MISSING — rebuild"

# Kill any stale camera processes before starting
pkill -f csi_camera_node.py 2>/dev/null; echo "camera clear"
# If capture still fails with "Device 0 (of 1) is in use" / "Failed to create
# CaptureSession", a stale Argus session is held by nvargus-daemon — clear it:
#   sudo systemctl restart nvargus-daemon
```

---

## Launch Sequence

### Option 1 — One command (recommended)

> **Fusion-node note:** `launch_real_hw.sh` starts the **plan-A** localizer
> (`anyloc/ros2_node.py`). The Desktop **Full Run** icon (`~/Desktop/full_run.sh`)
> starts the **plan-B** node (`ros2_node_vo_primary.py`, VO-primary + score gate 0.32
> + jump gate/blended corrections, preferred) plus the `--manual-takeoff` commander and the RTSP ground stream —
> use that for the Mission-Planner-AUTO flow.

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

# Plus a backup Mission Planner link over the internet (see step 7 above) —
# combine with any of the above:
bash control/launch_real_hw.sh --manual-takeoff --waypoint-file control/survey.waypoints \
    --mavlink-relay
```

`--stream-host` and `--stream-server` are mutually exclusive. Either adds `ground_view_stream.py` alongside `launch_camera.sh` (not instead of it) — `ground_view_stream.py` only subscribes to `/drone/camera/image_raw`, it doesn't open the camera. `--mavlink-relay` is independent of both — it's a GCS link, not a video stream — and requires certs already generated/copied per step 7.

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
# Plan B (VO-primary + score-gated AnyLoc, preferred — see anyloc/README.md):
bash anyloc/run_ros2_localizer_vo.sh --headless
# or plan A (anchor-chain):
#   bash anyloc/run_ros2_localizer.sh --headless
# Run ONE of the two, never both — they write the same latest_estimate.json.

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

> **Mission Planner AUTO flow (alternative to the scripted survey):** plan the
> survey in Mission Planner and **Write** it to the FC (no scp needed — the
> mission lives on the FC). Launch this same `--manual-takeoff` commander as
> the VPE feeder, arm on GPS, climb, flip the aux switch to SRC2, then switch
> to **AUTO** from Mission Planner. **Never switch to GUIDED** in this flow —
> GUIDED triggers the commander's scripted survey. Mission Planner can stay
> connected the whole time (radio or USB) — `SERIAL6_OPTIONS=1024` stops the
> FC from forwarding the VPE flood onto MP's link. The commander sets the EKF
> origin *and the VPE reference frame* from your arm GPS position; when
> Phase 2 activates it prints `VPE reference: … (EKF origin|arm GPS)` — if it
> says `HOME const` on real hardware, positions will be offset; abort.
> Flip back to SRC1 (GPS) before descending below 50 m AGL — below that the
> localizer stops and the VPE falls back to a home-anchor that would drag the
> EKF toward the origin.

**The 10-minute arm wait is a hard timeout**: if you don't arm within 10 min the
commander prints `ABORT: timed out waiting for arm` and **exits** — VPE stops and
the FC's "computer vision position" health flips to Fail. Restart the commander
shortly before you actually intend to arm.

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
GST_ARGUS: Available Sensor modes: ...             ← expected, ISP mode enumeration on startup
[ WARN:0] ... Cannot query video position ...       ← harmless, OpenCV GStreamer backend quirk
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
[APCmd] VPE reference: 23.450912°N 120.286201°E (arm GPS)   ← must NOT say "HOME const"
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

`tools/ground_view_stream.py` streams a 1280×720 composite viewport showing YOLO live detection feed, AnyLoc match tile, and the last 3 detection crops with class/location labels. It only **subscribes** to `/drone/camera/image_raw` — it does not open the camera — so `launch_camera.sh` must already be running (handled automatically by `launch_real_hw.sh --stream-host/--stream-server`).

It also always saves a local copy of the exact streamed composite to `recordings/ground_view_<timestamp>.mkv` (tee of the same H.265 encode — no extra GPU load; crash-safe MKV, playable even after power loss; gitignored). `--no-record` disables it, `--record-dir DIR` relocates it. The path is printed at startup and on exit.

**Mode A — direct UDP (ZeroTier / LAN):**
```bash
# In addition to Pane 1 (launch_camera.sh), add a new pane:
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

Bitrate: 1 Mbps H.265, keyframe every 1 s. Override with `--bitrate N`. Latency is bounded (leaky queue after `appsrc` + `vbv-size` capped to `--bitrate`, 2026-07-07) so a momentary LTE stall drops stale frames instead of building up an ever-growing, non-recovering delay.

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
ros2 topic hz /uas1/mavlink_sink              # Jetson→FC MAVLink: ~40 msg/s with
                                              # commander running; 1 msg/s = heartbeat
                                              # only (commander dead / VPE not reaching FC)
```

FC-side confirmation that the EKF is receiving the feed: `/diagnostics` → `mavros: System` → `computer vision position: Ok` (Fail = no VPE arriving).

### AnyLoc estimate

```bash
cat anyloc/latest_estimate.json
# {"timestamp": ..., "est_lat": ..., "est_lon": ..., "alt_msl_m": ..., "agl_m": ..., "yaw_deg": ..., "score": ..., "error_m": ...}
```

For a post-flight accuracy record (not just the current instant), `ros2_node.py` also appends one row per frame to `anyloc/logs/accuracy_<timestamp>.csv` — see `anyloc/README.md` §4 for the column layout.

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
  [ ] Verify jetson_clocks applied: `systemctl is-active jetson_clocks.service` → `active` (automated since 2026-07-08 — no manual command needed, but confirm the service actually ran; if not `active`, `sudo jetson_clocks` by hand)
  [ ] Verify /dev/ttyUSB0 present and `v4l2-ctl --list-devices` shows imx219
  [ ] ls Car_visdrone1280.engine (pre-built — if missing, first YOLO launch will stall ~15 min exporting it)
  [ ] pkill -f csi_camera_node.py (clear stale camera processes)
  [ ] Start: bash control/launch_real_hw.sh --manual-takeoff (or tmux layout)
  [ ] MAVROS pane: "detected remote address 1.1" ✓
  [ ] Camera pane: running (Argus sensor-mode enumeration on startup is OK)
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
  [ ] Commander printed "VPE reference: … (EKF origin)" or "(arm GPS)" — NOT "(HOME const)"
  [ ] Flip RC aux switch to HIGH → SRC2 (ExternalNav/AnyLoc)
  [ ] Watch EKF monitor: "✓ POS_ABS accepted" from VPE
  [ ] Scripted flow: switch FC to GUIDED → commander runs its survey
      Mission Planner flow: switch FC to AUTO (mission stored on FC) — NEVER GUIDED
  [ ] Before descending below 50 m AGL: flip aux switch back LOW (SRC1/GPS)
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
| All `/mavros/*` data topics silent (no IMU, pose, altitude) | MAVROS sends REQUEST_DATA_STREAM rate=0 on startup, or an FC reboot dropped the runtime stream request | `launch_mavros_real.sh` re-requests 10 Hz automatically at connect; `SR3_*=10` in real_hw.parm keeps streams alive across FC reboots. Manual fix: `ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 0, message_rate: 10, on_off: true}"` (stream_id 0 = all) |
| Mission Planner stuck on "Getting Params" / mission upload stalls while the stack runs | FC forwards the commander's 20 Hz VPE broadcasts onto MP's link (57600 radio saturates) | `SERIAL6_OPTIONS=1024` on the FC (in real_hw.parm since 2026-07-06) + reboot FC. Verified: MP param download + mission upload work with the full stack running |
| FC reports "computer vision position: Fail" / VPE gone though stack looks up | Commander hit its 10-min arm timeout and exited (`ABORT: timed out waiting for arm`) | Restart the commander; healthy check: `ros2 topic hz /uas1/mavlink_sink` ≈ 40 msg/s (1 msg/s = only heartbeat, commander dead) |
| `ros2 topic hz/echo` shows nothing for a topic that is actually publishing | DDS discovery latency on the loaded Jetson (load ~6 with full stack) | Wait — use 25 s+ timeouts before concluding a topic is dead |
| `/dev/ttyUSB0` permission denied | Not in `dialout` group | `sudo chmod 666 /dev/ttyUSB0` |
| `/dev/ttyUSB0` not found | Adapter unplugged or driver missing | `dmesg \| tail -20` → look for cp210x/ch341 |
| `launch_mavros_real.sh` exits with no output | `set -e` + stale pkill returning 1 | Fixed; if recurs: check script has `pkill ... \|\| true` |
| MAVROS not connecting | Wrong baud or device | Try `FCU_DEV=/dev/ttyUSB1 bash control/launch_mavros_real.sh`; verify FC serial baud = 921600 |
| Camera "Failed to create CaptureSession" / "Device 0 (of 1) is in use" | Stale Argus session (nvargus-daemon) | `pkill -f csi_camera_node.py; sudo systemctl restart nvargus-daemon; bash control/launch_camera.sh` |
| Camera calibration file not found | No intrinsics YAML | Harmless — AnyLoc doesn't use camera intrinsics |
| Arm rejected: Safety Switch | Safety button not pressed | Press physical button; LED must go green |
| Arm rejected: Need Position | VPE not publishing or EKF not converged | Check `ros2 topic hz /mavros/vision_pose/pose_cov` (expect 20 Hz); wait 30 s |
| EKF never reaches POS_ABS on SRC2 | ExternalNav params wrong | Verify `VISO_TYPE=1`, `EK3_SRC2_POSXY=6`, `EK3_SRC2_VELXY=0` in FC params; check VPE topic hz |
| EKF failsafe during survey | AnyLoc jump > glitch radius | Verify `EK3_GLITCH_RAD=50` in real_hw.parm |
| AnyLoc venv import error | Wrong Python used | Confirm script uses `/home/jetson/venv/anyloc/bin/python3` |
| YOLO venv import error | Wrong Python used | Confirm script uses `/home/jetson/venv/yolo/bin/python3` |
| AnyLoc not activating | AGL below 50 m threshold | Normal — activates above `MIN_AGL=50`; use `--test` flag to bypass on ground |
| AnyLoc database error | Wrong path | Check symlink: `ls -la anyloc/database` → should point to `database_zone_z20_vits14` |
| Survey strips curved | Wrong setpoint type | `go_to_ned()` uses velocity setpoints — don't switch to position setpoints during survey |
| Drone drifts in hover | `PSC_NE_VEL_I` non-zero | Verify `PSC_NE_VEL_I=0.0` in uploaded params |
| No detections logged | YOLO not running, or `/drone/camera/image_raw` not flowing | Check YOLO pane started; `ros2 topic hz /drone/camera/image_raw` (YOLO runs at any AGL — no altitude gate) |
| Wrong survey area | `home_elevation.json` mismatch | Update lat/lon/elev to actual takeoff point |
| Commander hangs at "waiting for drone state" | `hw_bridge.py` not running | hw_bridge must start before commander |
| GStreamer "no such element: nvv4l2h265enc" | Missing Jetson GStreamer plugins | `sudo apt install nvidia-l4t-gstreamer` |
| GStreamer stream no video on receiver | Firewall or wrong IP | Check `--host` matches ground PC IP; open port 5000/udp |
| GStreamer "Camera not running" | Camera pane not started | Start `launch_camera.sh` before `launch_gstreamer.sh` |
| GStreamer "appsrc push returned GST_FLOW_ERROR" | Ground IP unreachable | Ping ground station first; udpsink drops silently |
| `gstreamer_stream.py` "Cannot open CSI camera" | `launch_camera.sh` already running | Kill it first — both open an Argus CaptureSession on the same sensor and only one is allowed |
| `ground_view_stream.py` stuck at "Waiting for /drone/camera/image_raw" | `launch_camera.sh` not running | Start it first — `ground_view_stream.py` only subscribes, it doesn't open the camera (fixed automatically by `launch_real_hw.sh`) |
| `ground_view_stream.py` YOLO panel shows no boxes | YOLO node not started yet | Wait for YOLO node to load model (~30 s); boxes appear once AGL > 50 m |
| `ground_view_stream.py` AnyLoc panel black | AnyLoc node not running or no match yet | Wait for first AnyLoc match; `anyloc/latest_match.jpg` must exist |
| `ground_view_stream.py` / `record_field.py` RTSP: `no element "rtspclientsink"` | `gstreamer1.0-rtsp` not installed — it's a separate package from `gstreamer1.0-plugins-bad` on Ubuntu | `sudo apt install gstreamer1.0-rtsp` (verified fix — confirms `gst-inspect-1.0 rtspclientsink` afterward) |
| `ground_view_stream.py` RTSP: connection refused | MediaMTX server not running | Start `./mediamtx mediamtx.yml` on Frank's PC; verify port 8554 open |
| `ground_view_stream.py` stream delay grows and never recovers (LTE relay) | `appsrc` blocked on a stalled TCP push + encoder `vbv-size` let it burst above `--bitrate` on complex frames | Fixed 2026-07-07 (leaky queue + `vbv-size=bitrate`) — pull latest code. If it recurs, lower `--bitrate` for more headroom below actual uplink throughput |

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
9. **AnyLoc fuses at ≥ 50 m only**: `MIN_LOCALISATION_AGL=50.0` in commander and `MIN_AGL=50.0` in anyloc node must match. (YOLO has no altitude gate — it runs at any AGL.)
10. **venv/anyloc for AnyLoc, venv/yolo for YOLO**: system Python3 lacks torch/faiss/ultralytics. Do not use `conda run`.
11. **Kill stale camera processes**: running `launch_camera.sh` twice causes an Argus "CaptureSession" conflict — always `pkill -f csi_camera_node.py` first (and `sudo systemctl restart nvargus-daemon` if the session is stuck).
