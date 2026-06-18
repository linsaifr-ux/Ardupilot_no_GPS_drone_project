# No-GPS Drone Project — Plan

## Goal

Build a drone system that can localize itself and detect objects without GPS, using visual place recognition (AnyLoc), object detection (YOLO), and ArduPilot for flight control. The full pipeline is validated in Isaac Sim before deploying to real hardware.

---

## Project Structure

```
no_GPS_drone_project/
├── run.sh                        # Top-level tmux launcher (ArduPilot + PX4 modes)
├── instructions/                 # Plans, notes, history
├── simulator/
│   ├── cesium_scene.py           # Physics engine + visualiser
│   │                             #   100 Hz background thread: 6-DOF kinematic model + PX4SimBridge/SITLBridge
│   │                             #   render loop (~5 Hz): reads state, updates mesh, captures camera
│   │                             #   publishes /drone/state (100 Hz) + /drone/camera/image_raw (1024×768) + /drone/pose + /drone/agl
│   └── run_chiayi.sh             # Launch: sources ROS2 Jazzy, runs in conda env; --px4 flag
├── anyloc/
│   ├── build_database.py             # Build VLAD database (run once; NLSC PHOTO2 zoom-18)
│   ├── localizer.py                  # AnyLocLocalizer (DINOv2 + VLAD + FAISS)
│   ├── vo_refiner.py                 # VORefiner (LK optical flow)
│   ├── ros2_node.py                  # ROS2: sub camera/pose → write latest_estimate.json
│   ├── run_ros2_localizer.sh         # Launch script
│   ├── test_accuracy_esri.py         # accuracy benchmark — random points, NLSC PHOTO2 imagery
│   └── test_accuracy_constrained.py  # benchmark — anchor-chain constrained search vs global
├── detection/
│   ├── detector.py               # YOLODetector (auto class-map COCO/VisDrone)
│   └── ros2_node.py              # ROS2: sub /drone/camera → pub /yolo/detections
├── tools/
│   ├── live_trace.py             # Real-time viewer: survey route + zone + detections overlay
│   └── plot_trace.py             # Post-flight two-panel plot (top view + altitude)
├── control/
│   ├── px4_commander.py          # [ACTIVE] PX4 survey mission commander
│   ├── px4_sim_bridge.py         # PX4 HIL bridge — TCP 4560 server; HIL_ACTUATOR_CONTROLS in / HIL_SENSOR out
│   ├── px4_no_gps.params         # PX4: EKF2_EV_CTRL=15, GPS off, no RC
│   ├── launch_px4_sitl.sh        # Start PX4 SITL; saves PID → /tmp/px4_sitl.pid; overwrites /tmp/px4_sitl.log
│   ├── stop_px4_sitl.sh          # Stop PX4 SITL (MAVLink shutdown → SIGTERM → SIGKILL)
│   ├── apply_px4_params.sh       # Set + save PX4 params, auto-reboot
│   ├── launch_mavros_px4.sh      # MAVROS2 → PX4 (UDP 14540)
│   ├── launch_commander_px4.sh   # Run px4_commander.py
│   ├── drone_sim.py              # Headless-only fallback: kinematic physics + bridge (PX4_SIM=0/1)
│   │                             #   publishes /drone/state; NOT used when Isaac Sim runs
│   ├── sitl_bridge.py            # ArduPilot UDP :9002 server — binary servo in → JSON physics out
│   ├── ardupilot_commander.py    # [ACTIVE AP] ArduPilot survey mission commander — ported from px4_commander.py; ENU setpoint fix; GUIDED/NAV_TAKEOFF/LAND
│   ├── flight_commander.py       # [REFERENCE] ArduPilot mission archive (WP nav had axis-swap bug — fixed in ardupilot_commander.py)
│   ├── launch_mavros.sh          # MAVROS2 on UDP 14550 (ArduPilot)
│   ├── launch_sitl.sh            # ArduPilot SITL via MAVProxy
│   ├── launch_commander.sh       # Run flight_commander.py (legacy)
│   ├── launch_commander_ardupilot.sh  # Run ardupilot_commander.py
│   └── no_gps.parm               # ArduPilot: GPS_TYPE=0, EK3_SRC1_POSXY=6, VISO_TYPE=1, WPNAV_SPEED=1200
└── yolov8l_visdrone.pt           # YOLOv8l fine-tuned on VisDrone (active)
```

---

## System Architecture

### PX4 path (active)

```
simulator/cesium_scene.py  (or drone_sim.py headless)
  100 Hz background thread: kinematic model + PX4SimBridge
  TCP 4560 ◄──HIL_ACTUATOR_CONTROLS──► PX4 SITL ──HIL_SENSOR──►
  publishes /drone/state (ENU PoseStamped, 100 Hz)
            /drone/camera/image_raw  /drone/pose  /drone/agl
            │
            ├── px4_commander.py
            │     VPE: Phase1 kinematic → Phase2 AnyLoc latest_estimate.json
            │     → /mavros/vision_pose/pose_cov  (EKF2 VPE fusion)
            │     → /mavros/setpoint_raw/local    (PositionTarget, position carrot 25 m)
            │     ← /yolo/detections              (log via yaw-corrected GSD, no divert)
            │
            ├── anyloc/ros2_node.py  → latest_estimate.json  (AGL ≥ 50 m)
            └── detection/ros2_node.py → /yolo/detections    (AGL ≥ 50 m)

PX4 SITL
  ─UDP 14540/14580──► MAVROS2 ◄── /mavros/vision_pose/pose_cov
                      └── /mavros/setpoint_raw/local → SET_POSITION_TARGET_LOCAL_NED
```

### ArduPilot path (re-implemented 2026-06-19)

```
cesium_scene.py (or drone_sim.py headless)
  UDP 9002 ◄──PWM──► ArduPilot SITL
  ─TCP 5760──► MAVProxy ─UDP 14550──► MAVROS2
                          ◄── /mavros/vision_pose/pose_cov (EKF3 VPE, ENU→NED)
                          ◄── /mavros/global_position/set_gp_origin (EKF origin)
                          ◄── /mavros/setpoint_raw/local (ENU, MAVROS→NED)

ardupilot_commander.py
  VPE: Phase1 kinematic → Phase2 AnyLoc (identical to px4_commander.py)
  → /mavros/vision_pose/pose_cov   (EKF3 VPE fusion)
  → /mavros/setpoint_raw/local     (ENU velocity carrot)
  ← /yolo/detections               (log via yaw-corrected GSD, no divert)
  Sequence: STABILIZE→arm→GUIDED→EKF origin→wait EKF_POS_HORIZ_ABS→NAV_TAKEOFF→survey→LAND
```

### Coordinate conventions

| Frame | x | y | z | Used by |
|-------|---|---|---|---------|
| Isaac Sim / ENU | East | North | Up | cesium_scene.py, /drone/state, vision_pose VPE |
| PX4 / ArduPilot NED | North | East | Down (−=alt) | autopilot internal |
| MAVROS2 vision_pose | ENU in, NED out | | | converts correctly |
| MAVROS2 setpoint_raw/local (PositionTarget) | **explicit FRAME_LOCAL_NED** | | | MAVROS always applies ENU→NED; send x=East, y=North, z=Up |

### Port map

| Port | Protocol | Owner |
|------|----------|-------|
| TCP 4560 | MAVLink HIL | PX4SimBridge (server) ↔ PX4 SITL (client) |
| UDP 14540 | MAVLink | MAVROS2 receives from PX4 |
| UDP 14580 | MAVLink | PX4 SITL listens (onboard link) |
| UDP 18570 | MAVLink | PX4 SITL → GCS (QGC) |
| UDP 9002 | JSON FDM | SITLBridge ↔ ArduPilot SITL |
| TCP 5760 | MAVLink | MAVProxy ↔ ArduPilot SITL (internal; single client only) |
| UDP 14550 | MAVLink | MAVROS2 listens (MAVProxy → MAVROS2, ArduPilot) |

---

## Modules

### 1. Simulator / Physics (`simulator/cesium_scene.py`)

**Status:** Working — 100 Hz kinematic physics thread; owns SITL bridge; publishes /drone/state

Isaac Sim 6.0.0 scene centred on Chiayi, Taiwan (23.4509°N, 120.2861°E).

- **Terrain:** Cesium World Terrain (asset 1) — quantized-mesh-1.0, 9 tiles at level 13
- **Buildings:** Cesium OSM Buildings (asset 96188) — B3DM, ~83 buildings
- **Imagery:** Taiwan NLSC PHOTO2 WMTS (zoom 18, ~0.60 m/px effective); `satellite_ground.jpg` re-downloaded on first run after deletion; `MAX_TEX=16384`
- **Physics thread (100 Hz):** `_run_physics()` — kinematic 6-DOF model + `SITLBridge`; publishes `/drone/state`; rate decoupled from render loop
- **Render loop (~13 Hz):** reads shared kinematic state under lock; updates `/World/Drone` mesh position/orientation; captures nadir camera frames
- **Motor layout:** ArduCopter X-frame ch1=FR(NE), ch2=RL(SW), ch3=RR(SE), ch4=FL(NW)
- **SITLBridge:** `control/sitl_bridge.py` on UDP 9002 — binary servo in, JSON physics out
- **Camera:** `/World/Drone/Camera` — nadir, AP-IMX900-Mini-USB3-I5 (EFL 3.1 mm, 113.1°(D)×88°(H)×65.1°(V) from spec sheet, 2048×1536, GSD ≈ 75 mm/px @ 80 m); 2-axis gimbal-stabilised (cancel roll+pitch, preserve yaw — image is always level and top follows drone nose)
- **HUD:** `omni.ui` overlay showing live LAT / LON / ALT MSL / AGL
- **Frame output:** `drone_frames/latest.jpg` + `latest_meta.json` every 5 sim steps
- **ROS2 publishers:** `/drone/state`, `/drone/camera/image_raw`, `/drone/pose` (WGS84), `/drone/agl`

### 2. Drone Simulator Fallback (`control/drone_sim.py`)

**Status:** Headless fallback — not used when Isaac Sim is running

Standalone ROS2 node for headless SITL testing without Isaac Sim. Provides the same `/drone/state` topic and `SITLBridge` connection as `cesium_scene.py`.

- 6-DOF kinematic physics: PWM → thrust → roll/pitch → NED acceleration → ENU position
- Ground friction: horizontal velocity zeroed on ground contact (no sliding during motor spool-up)
- `SITLBridge` on UDP 9002: same protocol as used by `cesium_scene.py`
- Publishes `/drone/state` (`PoseStamped`, `frame_id="local_enu"`)

**Do not run alongside `cesium_scene.py`** — both bind UDP 9002.

### 3. Localisation (`anyloc/`)

**Status:** Working — AnyLoc + VO; ~2,820-entry database (65 m AGL only — single layer matching mission cruise altitude); ~15–20 m anchor error

**Database build** (`anyloc/build_database.py`) — self-contained, no Isaac Sim required:
- Downloads NLSC PHOTO2 tiles (zoom 18) automatically if `satellite_ground.jpg` missing
- Single AGL: `--agl-min 65 --agl-max 65` (1 level × ~2820 positions); was 13 levels × 36,673 entries
- 3-pass memory-safe: crop→disk, sample 2000 for codebook, batch VLADs (peak ~4.5 GB RAM)
- `db_meta.json` cache: Pass 1 skipped on subsequent `--rebuild` runs
- Runtime fully offline: `database.pt` + `satellite_ground.jpg` + cached DINOv2 weights

### 4. Object Detection (`detection/`)

**Status:** Working — `yolov8l_visdrone.pt` (VisDrone-trained), 10 aerial classes

### 5a. Flight Control — PX4 path (`control/px4_commander.py`) **[ACTIVE]**

**Status:** Survey mission implemented ✓ (PX4-9 Done). 12 m/s lawnmower, YOLO log-in-flight (no divert).

ROS2 node. MAVROS2 + PX4 OFFBOARD mode via `setpoint_raw/local` (position targets with 25 m carrot).

**Mission sequence (survey):**
1. Pre-stream 40 position setpoints at 20 Hz (PX4 requires setpoints before OFFBOARD)
2. Switch to OFFBOARD mode + arm
3. Climb to 65 m AGL (`takeoff()`)
4. Hold 5 s
5. `go_to_ned(speed=12)` — iterate 14 survey waypoints sequentially (7-strip boustrophedon)
   - YOLO detections: yaw-corrected GSD pixel projection → dedup (5 m) → log to `detections.csv`; survey never interrupted
   - On arrival (60 m radius) or timeout: advance waypoint index
6. Survey complete: `go_to_ned(0, 0, TAKEOFF_ALT)` fly home → `AUTO.LAND`
   (RTL not used — requires GPS-derived home; unreliable with external vision only)

**Survey constants (module level):**
- `SURVEY_WPS` — 14 boustrophedon waypoints at 65 m AGL, 7 E-W strips, 91.7 m N-S spacing (see `instructions/survey_mission_plan.md`)
- `SURVEY_SPEED = 12.0` m/s, `DEDUP_RADIUS = 5.0` m, `ZONE_VERTS` — buffered boundary (visualisation only)
- `CAM_W/H = 1024/768`, `HFOV_DEG = 88`, `VFOV_DEG = 65.1` — for GSD pixel-to-ground mapping
- `DET_LOG = detections.csv` — auto-header on first write

**Key methods:** `_cb_detections()` (project + log, no state change), `_log_detection()`, `_in_buffered_zone()` (live_trace.py only)  
**`go_to_ned(speed, radius)`** — no `interruptible` param; never returns early

**VPE two-phase:**
- Phase 1 (AGL < 50 m): kinematic truth, cov = 0.1 m²
- Phase 2 (AGL ≥ 50 m): AnyLoc `latest_estimate.json`, cov = max(1, err_m²)
- Heading: ENU yaw = π/2 (North) in both phases — `/drone/pose` encodes `−_kyaw_rad` (not `π/2−_kyaw_rad`), giving `yaw_deg=0` (East) for a North-facing drone. Hardcoding π/2 avoids a 90° VPE yaw jump at Phase 1→2.

**Physics fix (2026-06):** PX4 path requires second-order angular rate model in `drone_sim.py` and `cesium_scene.py`. First-order (τ=0.15 s) causes motor oscillation at 100 Hz steps → zero net horizontal force + slow altitude sink. Sign: `_kbfwd = -thrust * sin(pitch)` (FRD positive pitch = nose-UP = southward force = stable negative feedback).

**`px4_no_gps.params` highlights:**

| Param | Value | Reason |
|-------|-------|--------|
| `EKF2_GPS_CTRL` | 0 | disable GPS |
| `SYS_HAS_GPS` | 0 | no GPS hardware |
| `COM_ARM_WO_GPS` | 1 | arm without GPS |
| `EKF2_EV_CTRL` | 15 | fuse EV pos+height+vel+yaw |
| `EKF2_HGT_REF` | 3 | vision altitude reference |
| `EKF2_BARO_CTRL` | 0 | disable baro |
| `COM_RC_IN_MODE` | 4 | no RC required |

### 5b. Flight Control — ArduPilot path (`control/ardupilot_commander.py`) **[RE-IMPLEMENTED]**

**Status:** Implemented 2026-06-19 — pending test (AP-1 through AP-6)

ROS2 node. Ported from `px4_commander.py` with ArduPilot-specific additions (EKF origin, EKF_POS_HORIZ_ABS wait, NAV_TAKEOFF, LAND mode, force-arm fallback).

**Root cause of original ArduPilot WP nav failure (flight_commander.py):**  
`flight_commander.py` sent NED coordinates (`x=north, y=east`) to `setpoint_raw/local`. MAVROS2 always applies ENU→NED regardless of the `FRAME_LOCAL_NED` flag, so it treated `x=north_m` as "East" and swapped axes — ArduPilot received mirror-direction targets. The velocity-based `go_to_ned()` happened to work (it sent ENU-correct velocities). The fix in `ardupilot_commander.py`: send ENU (`x=East, y=North, z=Up`) identical to `px4_commander.py`.

**Full arming and flight sequence:**

1. Write stub estimate JSON; start VPE thread (Phase 1 kinematic 20 Hz)
2. Wait for MAVROS2 connection (60 s)
3. Spin to populate `/drone/state`; detect in-air restart
4. STABILIZE mode → arm (force-arm `CommandLong(400, param2=21196)` if needed)
5. GUIDED mode
6. Publish EKF origin to `/mavros/global_position/set_gp_origin` (10× / 5 s)
7. Wait for `EKF_POS_HORIZ_ABS` (bit 4) from EKF_STATUS_REPORT (msg 193) on `/uas1/mavlink_source`
8. `MAV_CMD_NAV_TAKEOFF` — ArduPilot climbs autonomously; monitor `_agl()` until AGL ≥ target − 2 m
9. Hold 5 s at cruise altitude; then 7-strip E-W boustrophedon survey at 12 m/s
10. Survey complete: fly home `go_to_ned(0,0,TAKEOFF_ALT)` → `LAND`

**MAVROS2 setpoint coordinate (corrected):**  
Send ENU (`x=East, y=North, z=Up(AGL)`) to `setpoint_raw/local`. MAVROS always applies ENU→NED — this is the correct convention. The old `flight_commander.py` incorrectly sent NED, causing MAVROS to double-convert and swap axes.

### 5b-archive. `flight_commander.py` **[REFERENCE ARCHIVE]**

**Status:** Superseded by `ardupilot_commander.py` — kept for historical reference only.

Original ArduPilot commander with the axis-swap bug. Velocity-based `go_to_ned()` worked correctly; position setpoints were broken due to NED→MAVROS→NED double-conversion.

**VPE covariance design:**
- `frame_id = "map"` (ENU): x = East, y = North, z = Up — the `vision_pose` plugin converts correctly to NED
- Phase 1 x/y covariance = **0.1 m²** (EKF sets POS_HORIZ_ABS immediately at ground)
- Phase 2 x/y covariance = **max(1, error_m²)** (scales with AnyLoc confidence)
- z = kinematic AGL from `/drone/state`; z covariance = **0.25 m²**; `EK3_SRC1_POSZ=6` routes to EKF altitude

**no_gps.parm highlights:**

| Param | Value | Reason |
|-------|-------|--------|
| `GPS_TYPE` | 0 | disable GPS |
| `EK3_SRC1_POSXY` | 6 | ExternalNav horizontal position |
| `EK3_SRC1_POSZ` | 6 | ExternalNav altitude (VPE z = kinematic AGL; barometer unreliable in SIM_JSON without `position` field) |
| `EK3_SRC1_YAW` | 6 | ExternalNav yaw |
| `VISO_TYPE` | 1 | MAVLink vision odometry |
| `FS_GPS_ENABLE` | 0 | no GPS failsafe |
| `ARMING_CHECK` | 0 | skip pre-arm (SITL only) |
| `MOT_THST_HOVER` | 0.5 | hover PWM = 1500 (matches Isaac Sim k_T and kinematic model) |
| `SCHED_LOOP_RATE` | 50 | matches Isaac Sim frame rate |
| `DISARM_DELAY` | 0 | enables NAV_TAKEOFF path (removes land-detector deadlock) |
| `WPNAV_SPEED` | 100 | 1 m/s horizontal max |
| `PSC_POSXY_P` | 0.3 | horizontal position P (default 1.0) |
| `PSC_VELXY_P` | 0.5 | horizontal velocity P (default 2.0) |
| `PSC_VELXY_I` | 0.0 | horizontal velocity I — zeroed to prevent windup during 90 s climb |
| `FS_CRASH_CHECK` | 0 | disable crash-detect disarm (kinematic model requires tilt angles that exceed the 30° threshold; re-enable on real hardware) |
| `ATC_ANG_RLL_P` | 1.5 | roll angle P (default 4.5 → windup → drift) |
| `ATC_ANG_PIT_P` | 1.5 | pitch angle P |
| `ATC_RAT_RLL_P` | 0.10 | roll rate P |
| `ATC_RAT_RLL_I` | 0.0 | roll rate I — **zeroed**: even 0.02 accumulates over 45 s climb → motor imbalance → crash-detect disarm at ~80 m |
| `ATC_RAT_PIT_P` | 0.10 | pitch rate P |
| `ATC_RAT_PIT_I` | 0.0 | pitch rate I — zeroed for same reason |

---

## Run order

### PX4 — with Isaac Sim (active path)

```bash
bash run.sh --tmux --px4 --params --wipe  # first run
bash run.sh --tmux --px4                  # subsequent runs
```

tmux windows: 0=Isaac Sim, 1=PX4 SITL, 2=MAVROS2, 3=Commander.

### PX4 — headless (no Isaac Sim)

```bash
source /opt/ros/jazzy/setup.bash

# T1 — physics bridge (must own TCP 4560 before PX4 starts)
PX4_SIM=1 python3 control/drone_sim.py

# T2 — PX4 SITL
bash control/launch_px4_sitl.sh
bash control/apply_px4_params.sh    # first run only

# T3 — MAVROS2
bash control/launch_mavros_px4.sh

# T4 — commander
python3 control/px4_commander.py
# or: HOLDTEST=1 python3 control/px4_commander.py  (Phase 3 regression)
```

### ArduPilot — headless (active path)

```bash
bash run.sh --tmux          # normal run (drone_sim.py physics)
bash run.sh --tmux --wipe   # first run (wipe EEPROM)
```

### ArduPilot — with Isaac Sim

```bash
bash run.sh --tmux --isaac                            # Isaac Sim + full survey
bash run.sh --tmux --isaac --anyloc                   # + AnyLoc Phase-2 VPE
bash run.sh --tmux --isaac --anyloc --detection       # + YOLO detection log
bash run.sh --tmux --wipe --isaac                     # first run (Isaac Sim)
```

tmux windows: 0=Bridge/Isaac, 1=SITL, 2=MAVROS, 3=Commander, 4=AnyLoc, 5=Detection.

```bash
# Manual steps:
# T1 — bridge (must own UDP 9002 before SITL)
python3 control/drone_sim.py        # headless
# or: cd simulator && ./run_chiayi.sh  # Isaac Sim (no --px4 flag)

# T2 — ArduPilot SITL via MAVProxy
bash control/launch_sitl.sh [--wipe]

# T3 — MAVROS2
bash control/launch_mavros.sh

# T4 — Commander
bash control/launch_commander_ardupilot.sh
# or: HOLDTEST=1 python3 control/ardupilot_commander.py
```

> Do **not** run `drone_sim.py` alongside `cesium_scene.py` — both bind UDP 9002.
> Start the bridge **before** ArduPilot SITL.

---

## Milestones

| # | Milestone | Status |
|---|-----------|--------|
| 1 | Isaac Sim scene: Cesium terrain + satellite imagery + OSM buildings | Done |
| 2 | Virtual drone + nadir camera + HUD publishing frames | Done |
| 3 | AnyLoc database built from simulated views | Done |
| 4 | AnyLoc localisation working + dual postview | Done |
| 5 | YOLO detection working on simulated frames | Done |
| 5a | VisDrone-trained YOLOv8l + auto class-map | Done |
| 5b | Top-down fine-tuning pipeline | Ready to run |
| 6a | ArduPilot SITL + JSON bridge (IMU + baro) | Done |
| 6b-i | pymavlink connection to ArduPilot | Done |
| 6b-ii | Disable GPS; strip position from bridge | Done |
| 6b-iii | AnyLoc → ArduPilot EKF3 via VPE | Done |
| 6b-iv | Flight commands via SET_POSITION_TARGET | Done |
| 6e | ROS2 migration: all IPC via topics + MAVROS2 | Done |
| 6f | Separate drone physics (drone_sim.py) from Isaac Sim | Done |
| 6g | Fix VPE: ENU coordinate order + 1 m² covariance for EKF POS_ABS | Done |
| 6h | Remove pymavlink; MAVROS2 raw MAVLink; DISARM_DELAY=0 | Done ✓ |
| 6i | NAV_TAKEOFF replaces P-controller; EKF origin blocking; reduced ATC gains | Done ✓ |
| 6j | Fix altitude runaway (EK3_SRC1_POSZ→ExternalNav) + fix 90° course error | Done ✓ |
| 6k | Fix waypoint direction inversion (MAVROS2 NED passthrough) + altitude drop on hold | Done ✓ |
| 6l | Integrate kinematic physics into cesium_scene.py (100 Hz thread); eliminate drone_sim.py | Done ✓ |
| 6m | Fix crash-detect disarm at ~80 m AGL (FS_CRASH_CHECK=0, ATC I-terms=0) | Done ✓ |
| 6n | database.pt truncation fix (_safe_save + split files); setpoint_raw/local FRAME_LOCAL_NED; launch scripts | Done ✓ |
| 6m-wp | ArduPilot WP nav — root cause identified (MAVROS ENU→NED applied to NED input = axis swap); fix in `ardupilot_commander.py` | Root cause fixed |
| PX4-1 | PX4 SITL HIL bridge (TCP 4560) + EKF2 no-GPS validated | Done ✓ |
| PX4-2 | Vision + MAVROS↔PX4 link; EKF tracks truth | Done ✓ |
| PX4-3 | Position-hold gate: 3 m AGL, 40 s, <0.3 m drift | Done ✓ |
| PX4-4 | Waypoint nav in `px4_commander.py`: 65 m AGL, 699 m leg | Done ✓ |
| PX4-5 | Isaac Sim pipeline wired (`run.sh --tmux --px4`) | Done ✓ |
| PX4-6 | End-to-end Isaac Sim waypoint flight (horiz_err < 60 m) | Done ✓ |
| PX4-7 | AnyLoc + detection end-to-end in PX4 pipeline | In progress (code ready; pending test) |
| PX4-8 | Survey mission plan: lawnmower + car detection response | Done ✓ |
| PX4-9 | Implement survey commander: 12 m/s, 7-strip E-W lawnmower (91.7 m spacing, 33 m overlap, ~10.2 min), YOLO log-in-flight (no divert) | Done ✓ |
| PX4-10 | Jetson distributed sim: Jetson runs commander+AnyLoc+YOLO, PC runs Isaac+PX4 | TODO |
| AP-1 | ArduPilot SITL + drone_sim.py: EKF origin + arm in GUIDED | Pending test |
| AP-2 | HOLDTEST: EKF_POS_HORIZ_ABS set; NAV_TAKEOFF; 3 m hold < 0.5 m drift | Pending test |
| AP-3 | Single-WP nav: ENU setpoint fix verified (no mirror-direction) | Pending test |
| AP-4 | Full survey: 7-strip E-W lawnmower 91.7 m spacing, YOLO log-in-flight | Pending test |
| AP-5 | Isaac Sim pipeline: `run.sh --tmux --isaac` + full survey | Pending test |
| AP-6 | Full pipeline: `run.sh --tmux --isaac --anyloc --detection` | Pending test |
| 6c | HIGHRES_IMU from ArduPilot → localization pipeline | TODO |
| 6d | IMU fusion: AnyLoc anchor validator + VO quality gate | TODO |
| 7 | Full pipeline: AnyLoc + VO + detection → PX4 commands | TODO |
| 8 | Deploy to real drone hardware | TODO |
