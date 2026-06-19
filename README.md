# No-GPS Drone Project

Autonomous drone that localises itself and detects objects without GPS, validated in Isaac Sim before deployment to real hardware.

**Location:** Chiayi, Taiwan — 23.4509°N, 120.2861°E  
**Stack:** Isaac Sim 6.0.0 · AnyLoc (DINOv2 ViT-S/14 + VLAD) · YOLO11s · **PX4 SITL** · **ArduPilot SITL** · ROS2 Jazzy · MAVROS2

> **Autopilot:** PX4 (primary, fully validated) + ArduPilot (**AP-3 HOLD GATE passed 2026-06-19**, 0.1 m drift; AP-4–AP-6 pending).
> Original WP nav inversion fixed: old `flight_commander.py` sent NED to MAVROS2 (which always applies ENU→NED), axis-swapping the target.
> `ardupilot_commander.py` sends ENU identically to `px4_commander.py`.
> **PSC rename (V4.8):** `PSC_POSXY_*`/`PSC_VELXY_*` → `PSC_NE_*`/`PSC_NE_VEL_*`; `no_gps.parm` updated (old names silently ignored → default `PSC_NE_VEL_I=1.0` caused integral windup → growing oscillation).

---

## Architecture

```
┌──────────────────────────────────────────────────────────────────────┐
│  simulator/cesium_scene.py  (Isaac Sim — physics + visualiser)      │
│  100 Hz background thread: 6-DOF kinematic model                     │
│  ArduPilot: SITLBridge  UDP 9002  (binary servo in / JSON out)      │
│  PX4:       PX4SimBridge TCP 4560  (HIL_ACTUATOR_CONTROLS in /       │
│                                     HIL_SENSOR out)                  │
│  Publishes: /drone/state (ENU PoseStamped, 100 Hz)                  │
│             /drone/camera/image_raw (2-axis gimbal nadir, 1024×768)  │
│             /drone/pose  /drone/agl                                  │
└───────┬──────────────────────────────────────────────────────────────┘
        │ /drone/state
  ┌─────┴──────────────────────┐
  ▼                            ▼
anyloc/ros2_node.py       detection/ros2_node.py
DINOv2+VLAD localisation  YOLOv8 detection
→ /mavros/vision_pose/    → /yolo/detections
  pose_cov  (VPE)

 ── ArduPilot path ────────────────────────────────────────────────────
 MAVProxy (TCP 5760) → UDP 14550 → MAVROS2
 /mavros/vision_pose/pose_cov → EKF3 (ExternalNav)
 ardupilot_commander.py: STABILIZE→arm→GUIDED→NAV_TAKEOFF→7-strip E-W survey 12m/s→LAND

 ── PX4 path (active) ─────────────────────────────────────────────────
 PX4 SITL (TCP 4560 HIL) → UDP 14540/14580 → MAVROS2
 /mavros/vision_pose/pose_cov → EKF2 (EV_CTRL=15)
 px4_commander.py: stream setpoints→OFFBOARD→arm→climb 65m→7-strip E-W survey 12m/s→fly home→AUTO.LAND
```

**Headless fallback:** `control/drone_sim.py` provides the same kinematic bridge without Isaac Sim — used for fast control-loop testing. Not used when Isaac Sim runs.

---

## Repository Layout

```
no_GPS_drone_project/
├── run.sh                        # top-level launcher (ArduPilot + PX4 tmux modes)
├── simulator/                    # Isaac Sim — physics + visualiser
│   ├── cesium_scene.py           # Cesium terrain + 100 Hz kinematic thread + bridge
│   └── run_chiayi.sh             # launch: ./run_chiayi.sh [--px4]
├── control/                      # autopilot integration + mission
│   ├── drone_sim.py              # headless physics rig (PX4_SIM=0/1)
│   ├── px4_sim_bridge.py         # PX4 HIL bridge (TCP 4560, pymavlink)
│   ├── sitl_bridge.py            # ArduPilot SIM_JSON bridge (UDP 9002)
│   ├── px4_commander.py          # PX4 survey: OFFBOARD→65m→7-strip E-W 12m/s lawnmower (91.7m spacing, 33m overlap, ~10.2 min); YOLO logs via yaw-corrected pixel projection→fly home→AUTO.LAND
│   ├── ardupilot_commander.py    # ArduPilot survey: GUIDED→NAV_TAKEOFF→65m→7-strip E-W 12m/s lawnmower→LAND (ported from px4_commander.py; ENU setpoint fix)
│   ├── flight_commander.py       # ArduPilot mission (reference archive; superseded by ardupilot_commander.py)
│   ├── px4_no_gps.params         # PX4: EKF2_EV_CTRL=15, GPS off, no RC
│   ├── no_gps.parm               # ArduPilot: EK3 ExternalNav, GPS off, WPNAV_SPEED=1200 cm/s
│   ├── launch_px4_sitl.sh        # start PX4 SITL; saves PID → /tmp/px4_sitl.pid; overwrites /tmp/px4_sitl.log
│   ├── stop_px4_sitl.sh          # stop PX4 SITL (MAVLink shutdown → SIGTERM → SIGKILL)
│   ├── launch_mavros_px4.sh      # MAVROS2 → PX4 (UDP 14540)
│   ├── launch_commander_px4.sh   # run px4_commander.py
│   ├── apply_px4_params.sh       # set + save PX4 params, auto-reboot
│   ├── launch_sitl.sh            # ArduPilot SITL via MAVProxy
│   ├── launch_mavros.sh          # MAVROS2 → ArduPilot (UDP 14550)
│   ├── launch_commander.sh       # run flight_commander.py (legacy)
│   └── launch_commander_ardupilot.sh  # run ardupilot_commander.py
├── anyloc/                       # visual localisation
│   ├── build_database.py         # build VLAD database (--model vitb14|vits14; ~2 820 entries)
│   ├── localizer.py              # AnyLocLocalizer (DINOv2 ViT-S/14 + VLAD + FAISS)
│   ├── ros2_node.py              # ROS2: pub /mavros/vision_pose/pose_cov
│   ├── test_vit_comparison.py    # speed + accuracy benchmark: ViT-B vs ViT-S
│   ├── database_vits14/          # active database (ViT-S/14, ~265 MB VLADs)
│   └── run_ros2_localizer.sh     # launch script
├── detection/                    # object detection
│   ├── detector.py               # YOLODetector (auto class-map COCO/VisDrone)
│   ├── ros2_node.py              # ROS2: sub /drone/camera → pub /yolo/detections
│   ├── finetune.py               # train car_s_1280 (YOLOv8s) / car_11s_1280 (YOLO11s)
│   ├── test_map_car.py           # mAP benchmark: YOLOv8s vs YOLO11s car-only
│   └── run_ros2_detector.sh      # launch script
├── tools/                        # Post-flight and live analysis tools
│   ├── live_trace.py             # Real-time viewer: survey route + zone + detections overlay
│   └── plot_trace.py             # Post-flight two-panel plot (top view + altitude)
├── yolov8l_visdrone.pt           # YOLOv8l fine-tuned on VisDrone (active)
└── third_party/ardupilot/        # ArduPilot source (SITL binary inside)
```

---

## Milestones

| # | Milestone | Status |
|---|-----------|--------|
| 1 | Isaac Sim: Cesium terrain + NLSC imagery + OSM buildings | Done |
| 2 | Virtual drone + nadir camera | Done |
| 3 | AnyLoc database from simulated satellite views | Done |
| 4 | AnyLoc + VO localisation on simulated frames | Done |
| 5 | YOLOv8 detection on simulated frames (VisDrone model) | Done |
| 6a | ArduPilot SITL + Isaac Sim physics bridge (JSON FDM) | Done |
| 6b | GPS-denied EKF (EK3 ExternalNav) + VPE from AnyLoc | Done |
| 6c–n | ROS2/MAVROS2 migration, VPE tuning, takeoff, 90 m altitude | Done |
| 6m-wp | **ArduPilot WP nav** — root cause identified (MAVROS ENU→NED on NED input); fix in `ardupilot_commander.py` | Root cause fixed |
| PX4-1 | PX4 SITL ↔ HIL bridge validated (27k+ frames, EKF2 level) | Done |
| PX4-2 | Vision + MAVROS↔PX4 link established | Done |
| PX4-3 | **Position-hold gate passed** (<0.3 m drift, 40 s) | Done |
| PX4-4 | Waypoint nav ported to px4_commander.py (65 m, 699 m leg) | Done |
| PX4-5 | Isaac Sim pipeline wired (`run.sh --tmux --px4`) | Done |
| PX4-6 | End-to-end Isaac Sim waypoint flight (65 m AGL, 699 m leg, horiz_err < 60 m) | Done ✓ |
| PX4-7 | AnyLoc + detection integration in PX4 pipeline | In progress |
| PX4-8 | Survey mission plan: lawnmower + car detection response | Done ✓ |
| PX4-9 | Survey commander: 12 m/s, 7-strip E-W lawnmower (91.7 m spacing, 33 m overlap, ~10.2 min), YOLO log-in-flight (no divert) | Done ✓ |
| PX4-10 | Jetson distributed sim (Jetson = commander+AnyLoc+YOLO; PC = Isaac+PX4) | TODO |
| AP-1 | SITL + drone_sim.py: bridge connects, physics packets | Done ✓ |
| AP-2 | EKF origin set + arm in GUIDED succeeds | Done ✓ |
| AP-3 | HOLDTEST: 40 s hold at 3 m AGL, drift < 0.5 m | Done ✓ (0.1 m, 2026-06-19) |
| AP-4 | Full survey: 7-strip E-W lawnmower, YOLO log-in-flight | Pending |
| AP-5 | Isaac Sim pipeline: `run.sh --tmux --isaac` + full survey | Pending |
| AP-6 | AnyLoc + detection: `run.sh --tmux --isaac --anyloc --detection` | Pending |
| 8 | Deploy to real hardware | TODO |

---

## Port Map

| Port | Protocol | Owner | Direction |
|------|----------|-------|-----------|
| TCP 4560 | MAVLink HIL | PX4SimBridge (server) ↔ PX4 SITL (client) | physics bridge |
| UDP 9002 | JSON FDM | SITLBridge ↔ ArduPilot SITL | physics bridge |
| TCP 5760 | MAVLink | MAVProxy ↔ ArduPilot SITL | internal |
| UDP 14550 | MAVLink | MAVROS2 ← MAVProxy → ArduPilot | ArduPilot offboard |
| UDP 14540 | MAVLink | MAVROS2 receives from PX4 | PX4 offboard |
| UDP 14580 | MAVLink | PX4 SITL listens (onboard link) | PX4 offboard |
| UDP 18570 | MAVLink | PX4 SITL → GCS (QGC) | PX4 GCS |

---

## Quick Start — PX4 (recommended)

### Prerequisites

```bash
# ROS2 Jazzy + MAVROS2
sudo apt install ros-jazzy-mavros ros-jazzy-mavros-extras ros-jazzy-mavros-msgs
sudo /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh

# PX4 SITL (one-time build, ~20 min)
cd ~/PX4-Autopilot
PATH=~/.local/bin:$PATH make px4_sitl_nolockstep
```

### Run — Isaac Sim (full pipeline)

```bash
# First run — apply params and wipe saved state:
bash run.sh --tmux --px4 --params --wipe

# Subsequent runs (control loop only):
bash run.sh --tmux --px4

# With AnyLoc GPS-denied localisation (Phase 2 VPE from visual matching):
bash run.sh --tmux --px4 --anyloc

# With AnyLoc + YOLO vehicle detection:
bash run.sh --tmux --px4 --anyloc --detection

# Headless Isaac Sim (no display window — full camera/AnyLoc/YOLO still run):
bash run.sh --tmux --px4 --no-window
bash run.sh --tmux --px4 --no-window --anyloc --detection
```

tmux windows: **0 Isaac** · **1 PX4** · **2 MAVROS** · **3 Commander** · **4 AnyLoc** · **5 Detection**  
Switch with `Ctrl-B 0–5`. The commander prints `[PX4Cmd]` progress to window 3.

**Live flight viewer** (open in a separate terminal before or during the flight):
```bash
python3 tools/live_trace.py
```
Overlays the survey route, zone boundary, sim car positions, and YOLO detections in real time.

> **AnyLoc startup:** ~2,820-entry ViT-S/14 database (`database_vits14/`, 265 MB). Load time is much shorter than the old 36,673-entry ViT-B database. The localizer reads `model_name` from the database and loads the correct DINOv2 backbone automatically.

### Run — distributed (PC = sim only, Jetson = everything that runs on real drone)

```bash
# PC — Isaac Sim + PX4 SITL + MAVProxy bridge only
export ROS_DOMAIN_ID=0
bash run.sh --tmux --px4 --jetson-sim

# Jetson Orin NX — MAVROS + Commander + AnyLoc + YOLO (same as real hardware)
export ROS_DOMAIN_ID=0
bash run_jetson.sh
```

tmux on PC: **0 Isaac · 1 PX4 · 2 MAVProxy**  
tmux on Jetson: **0 MAVROS · 1 Commander · 2 AnyLoc · 3 Detection**

See `instructions/jetson_distributed_plan.md` for network setup, MAVProxy bridge details, code changes required, and real hardware transition notes.

### Run — headless (no Isaac Sim, for control-loop testing)

```bash
# First run (apply params):
bash run.sh --tmux --px4 --headless --params

# Subsequent runs:
bash run.sh --tmux --px4 --headless
```

tmux windows: **0 Bridge** · **1 PX4** · **2 MAVROS** · **3 Commander**  
Switch with `Ctrl-B 0/1/2/3`. The commander prints `[PX4Cmd]` progress to window 3.

<details>
<summary>Manual steps (without run.sh)</summary>

```bash
source /opt/ros/jazzy/setup.bash

# T1 — physics bridge (must own TCP 4560 before PX4 starts)
PX4_SIM=1 python3 control/drone_sim.py

# T2 — PX4 SITL
bash control/launch_px4_sitl.sh
bash control/apply_px4_params.sh     # first run only

# T3 — MAVROS
bash control/launch_mavros_px4.sh

# T4 — commander
source /opt/ros/jazzy/setup.bash
python3 control/px4_commander.py
```

</details>

### Stop PX4 SITL after mission

PX4 is launched with `setsid nohup` and survives terminal/tmux-window close. After the mission lands, stop it with:

```bash
bash control/stop_px4_sitl.sh
```

Tries (in order): MAVLink `MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN` (if MAVROS is up) → SIGTERM to saved PID → pkill SIGTERM → SIGKILL.

### Hold-gate test only (Phase 3 regression check)

```bash
HOLDTEST=1 python3 control/px4_commander.py
```

---

## Quick Start — ArduPilot

```bash
# Headless (drone_sim.py physics, no Isaac Sim window):
bash run.sh --tmux          # normal run
bash run.sh --tmux --wipe   # first run (wipe EEPROM)

# With Isaac Sim:
bash run.sh --tmux --isaac                            # Isaac Sim + full survey
bash run.sh --tmux --isaac --anyloc                   # + AnyLoc Phase-2 VPE
bash run.sh --tmux --isaac --anyloc --detection       # + YOLO detection log
bash run.sh --tmux --wipe --isaac                     # first run (Isaac Sim)
```

tmux windows: **0 Bridge/Isaac** · **1 SITL** · **2 MAVROS** · **3 Commander** · **4 AnyLoc** · **5 Detection**

> **First run:** type `reboot` in the MAVProxy console after params load. Drop `--wipe` subsequently.

```bash
# Manual steps (without run.sh):
bash control/launch_sitl.sh [--wipe]          # T1: ArduPilot SITL via MAVProxy
bash control/launch_mavros.sh                 # T2: MAVROS2 → UDP 14550
source /opt/ros/jazzy/setup.bash
python3 control/ardupilot_commander.py        # T3: mission commander
# or: HOLDTEST=1 python3 control/ardupilot_commander.py
```

> **ArduPilot V4.8 PSC parameter rename:** `no_gps.parm` uses `PSC_NE_POS_P`, `PSC_NE_VEL_P/I/D` (renamed from `PSC_POSXY_P`, `PSC_VELXY_*` in V4.8.0-dev). If you see growing horizontal oscillation on a fresh build, verify these names are accepted (`param show PSC_NE*` in MAVProxy). Setting `PSC_NE_VEL_I=0.0` and `PSC_NE_POS_P=0.2` is critical — the defaults (I=1.0, P=1.0) cause integral windup and underdamped oscillation. See `instructions/ap3_holdgate_solving_process.md` for the full debugging record.

---

## Key Design Decisions

### Autopilot bridge

`cesium_scene.py` and `drone_sim.py` both honour `PX4_SIM`:
- `PX4_SIM=0` (default): `SITLBridge` on UDP 9002 — binary servo packet in, JSON FDM out
- `PX4_SIM=1`: `PX4SimBridge` on TCP 4560 — `HIL_ACTUATOR_CONTROLS` in, `HIL_SENSOR` out

The bridge must own its port **before** the autopilot starts; otherwise SITL/PX4 exits immediately.

### VPE (vision position estimate) — two phases

Both commanders publish `PoseWithCovarianceStamped` to `/mavros/vision_pose/pose_cov`:

| Phase | Trigger | Position source | cov_xy |
|-------|---------|-----------------|--------|
| 1 | AGL < 50 m | `/drone/state` kinematic truth (ENU) | 0.1 m² |
| 2 | AGL ≥ 50 m | AnyLoc `latest_estimate.json` | max(1, err_m²) |

Altitude always comes from `/drone/state` (kinematic AGL); cov_z = 0.25 m².  
Frame: `"map"` (ENU) — MAVROS converts to NED for PX4/ArduPilot.

**Heading:** ENU yaw = π/2 (North) in **both** phases. `/drone/pose` encodes orientation as `qz = sin(−_kyaw_rad / 2)`, so a North-facing drone (`_kyaw_rad = 0`) produces `yaw_deg = 0` (East) in `latest_estimate.json` — a 90° error. Since the drone never yaws, the commander hardcodes π/2 for consistent, correct heading across both phases.

### MAVROS2 setpoint convention

`/mavros/setpoint_raw/local` with `FRAME_LOCAL_NED`:  
**MAVROS2 always applies ENU→NED** regardless of the frame flag. Send:
- `position.x = East`, `position.y = North`, `position.z = Up (AGL)` — MAVROS negates z to NED Down.

### PX4 OFFBOARD mode

PX4 requires setpoints streaming ≥ 2 Hz **before** switching to OFFBOARD. `px4_commander.py` pre-streams 40 setpoints at 20 Hz, then switches mode and arms. OFFBOARD is maintained by continuous setpoint publishing in `takeoff()` and `go_to_ned()`.

### Position carrot navigation (PX4)

`go_to_ned()` publishes a position target 25 m ahead of the drone toward the waypoint. This prevents PX4 from commanding max speed toward a 700 m jump and gives smooth, bounded velocity. Within 25 m the carrot snaps to the exact target.

### PX4 parameters (px4_no_gps.params)

| Param | Value | Reason |
|-------|-------|--------|
| `EKF2_GPS_CTRL` | 0 | disable GPS |
| `SYS_HAS_GPS` | 0 | GPS not present |
| `COM_ARM_WO_GPS` | 1 | allow arming without GPS |
| `EKF2_EV_CTRL` | 15 | fuse EV pos + height + vel + yaw |
| `EKF2_HGT_REF` | 3 | vision altitude reference |
| `EKF2_BARO_CTRL` | 0 | disable baro (EV handles altitude) |
| `COM_RC_IN_MODE` | 4 | no RC required |
| `NAV_RCL_ACT` | 0 | no RC loss failsafe |
| `NAV_DLL_ACT` | 0 | no datalink loss failsafe |

### Why 100 Hz physics matters

Isaac Sim renders at ~13 fps. If the physics + bridge ran in the render loop, the autopilot would see 13 Hz physics replies — too slow for stable PID control (altitude oscillates). The background thread at 100 Hz gives the autopilot a stable high-rate loop.

---

## Monitor Topics

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic hz /drone/state                      # physics rate (should be ~100 Hz)
ros2 topic hz /drone/camera/image_raw          # ~6 Hz from Isaac Sim
ros2 topic echo /mavros/state                  # connected, armed, mode
ros2 topic echo /mavros/local_position/pose    # EKF2 position estimate
ros2 topic echo /mavros/vision_pose/pose_cov   # VPE from commander
```

---

## Flight Trace Tools

Both `drone_sim.py` and `cesium_scene.py` write a CSV trace at 5 Hz to `simulator/flight_traces/trace_<timestamp>.csv`:

```
t_s, east_m, north_m, agl_m, vn_ms, ve_ms
```

**Live viewer** (open before or during flight):
```bash
python3 tools/live_trace.py              # auto-attach to latest trace
DISPLAY=:2 python3 tools/live_trace.py  # headless display
```

Overlays: planned 7-strip E-W survey route, raw zone boundary (solid white), buffered zone
boundary 30 m inward (orange dashed), sim car positions (yellow squares), detection markers
from `detections.csv` (refreshed live, filtered to current flight only), 65 m AGL target line.
Status bar shows nearest WP name + distance and running detection count.

**Post-flight plot** (saves `simulator/flight_traces/trace_plot.png`):
```bash
python3 tools/plot_trace.py             # latest trace
python3 tools/plot_trace.py --all       # overlay all traces
```

---

## Data Sources

| Layer | Source | License |
|-------|--------|---------|
| Terrain | Cesium World Terrain (asset 1) | © Cesium ion |
| Buildings | Cesium OSM Buildings (asset 96188) | © OpenStreetMap (ODbL) |
| Satellite imagery (sim + database build) | Taiwan NLSC PHOTO2 (zoom 18, ~0.60 m/px) | © 內政部國土測繪中心 (NLSC) |

### Target Vehicles (Isaac Sim)

Three procedural sedan models (`make_car()` in `cesium_scene.py`) placed inside the
detection zone for end-to-end survey pipeline testing. Each car's Z position is looked
up from the terrain mesh via `terrain_elev_at()` so wheels sit on the ground at their
actual lat/lon rather than at home-origin elevation:

| Model | NED (m) | Yaw | Strip area |
|-------|---------|-----|------------|
| `/World/Car_01` | N+350 E−700 | 45° NE | Strip 3 |
| `/World/Car_02` | N+150 E−900 | 270° W | Strip 1 |
| `/World/Car_03` | N+450 E−1100 | 135° SE | Strip 4 |
