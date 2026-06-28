# control/ — Flight control, SITL bridges, and autopilot integration

This module connects the simulator physics to an autopilot (SITL) and runs the autonomous mission.
Two autopilots are supported via the `PX4_SIM` environment variable:

| Autopilot | Bridge | Transport | Commander | Status |
|-----------|--------|-----------|-----------|--------|
| **PX4** (`PX4_SIM=1`) | `px4_sim_bridge.py` | MAVLink HIL, TCP 4560 | `px4_commander.py` | **Full mission ready** — phases 1–5 complete; position-hold gate passed (<0.3 m drift); waypoint nav 90 m AGL / 699 m leg implemented |
| **ArduPilot** (default) | `sitl_bridge.py` | JSON FDM, UDP 9002 | `ardupilot_commander.py` | **AP-3–AP-6 all passed** 2026-06-19. AP-6: full stack (Isaac Sim + AnyLoc + YOLO), 14 WPs, landed ✓. |

Root cause of original ArduPilot failure: `flight_commander.py` sent NED coordinates to `setpoint_raw/local`; MAVROS2 always applies ENU→NED regardless of the frame flag, swapping axes before forwarding to ArduPilot. `ardupilot_commander.py` sends ENU (identical to `px4_commander.py`) and the inversion is resolved. `flight_commander.py` is kept as a reference archive.

---

## Files

### Bridges (simulator ↔ autopilot)

**`px4_sim_bridge.py`** — PX4 Simulator-MAVLink (HIL) bridge. TCP 4560 server (PX4 is the client).
- In: `HIL_ACTUATOR_CONTROLS` — 16 normalised motor outputs [0, 1]
- Out: `HIL_SENSOR` — accel/gyro body-FRD, synthetic mag rotated by attitude, baro
- `time_usec` must be `time.monotonic() * 1e6` — PX4 sets its CLOCK_MONOTONIC to it; a backward jump causes BARO/MAG STALE errors
- Motor decode for PX4 none_iris quad-X (CA_ROTOR geometry): `control[0]=FR(+,+)`, `[1]=RL(-,-)`, `[2]=FL(+,-)`, `[3]=RR(-,+)`. Roll = `(m1+m2)-(m0+m3)`, pitch = `(m0+m2)-(m1+m3)`. Decode is in `cesium_scene.py` and `drone_sim.py` under `_PX4_SIM`.

**`sitl_bridge.py`** — ArduPilot SIM_JSON bridge. UDP 9002 server.
- In: binary `servo_packet_16` (40 bytes, magic=18458)
- Out: JSON physics state terminated by `\n` — `velocity` included, `position` intentionally absent

### Physics rigs (headless — no Isaac Sim)

**`drone_sim.py`** — kinematic 6-DOF rig + SITL bridge. Honours `PX4_SIM`:
- `PX4_SIM=0` → ArduPilot bridge (UDP 9002)
- `PX4_SIM=1` → PX4 bridge (TCP 4560)

Publishes `/drone/state` (ENU PoseStamped, 100 Hz). Used for fast control-loop iteration without the full Isaac Sim render overhead. Not used when `cesium_scene.py` is running.

**PX4 physics (second-order angular rate model):** For `PX4_SIM=1`, attitude uses a second-order model (`K_PITCH_ACCEL=80 rad/s²`, `K_PITCH_DAMP=12 s⁻¹`) rather than first-order τ. The first-order model caused motor oscillation at 100 Hz (τ=0.15 s ≈ 15 steps), resulting in zero net horizontal force and a slow altitude sink. The sign of the horizontal thrust component is `_kbfwd = -thrust * sin(pitch)` — minus because PX4 FRD positive pitch is nose-UP (southward force = negative feedback for northward flight).

**Flight trace CSV:** Both `drone_sim.py` and `cesium_scene.py` write a 5 Hz trace to `simulator/flight_traces/trace_<timestamp>.csv` with columns `t_s, east_m, north_m, agl_m, vn_ms, ve_ms`. View live with `tools/live_trace.py` or post-flight with `tools/plot_trace.py`.

### Commanders (the mission)

**`px4_commander.py`** — PX4/MAVROS2 full mission commander.
- Vision injection: 20 Hz `PoseWithCovarianceStamped` to `/mavros/vision_pose/pose_cov` + velocity to `/mavros/vision_speed/speed_twist`
- Two-phase VPE: Phase 1 (AGL < 50 m) = kinematic truth, cov=0.1 m²; Phase 2 (≥ 50 m) = AnyLoc `latest_estimate.json`, cov = max(1, err_m²)
- VPE heading: ENU yaw = π/2 (North) in **both** phases. `/drone/pose` encodes `−_kyaw_rad` not `π/2−_kyaw_rad`, so `yaw_deg=0` in the JSON maps to East, not North. Since the drone never yaws, π/2 is always correct and avoids a 90° EKF2 heading jump at the Phase 1→2 transition.
- **Survey mission:** climb 65 m → 7-strip E-W lawnmower at 12 m/s / 91.7 m N-S spacing (~10.2 min, ~7.36 km); strips run east-west (long axis), enter from east, boustrophedon S→N; 91.7 m spacing < 125 m swath → 33 m overlap, zero coverage gaps; YOLO vehicle detection → yaw-corrected GSD pixel projection → log to `detections.csv` (timestamp, category, confidence, lat, lon, agl_m). No divert — survey continues unbroken. Dedup: `_logged_positions` list; detections within 5 m of an already-logged entry are discarded.
- See `instructions/survey_mission_plan.md` for zone geometry, strip table, and waypoint list.
- `HOLDTEST=1`: 3 m hold gate (Phase 3 regression test)
- `TAKEOFF_ALT=<m>`: override cruise altitude (default 65 m)
- In-air restart: detects AGL > 5 m at startup and skips takeoff

**`ardupilot_commander.py`** — ArduPilot/MAVROS2 full mission commander (ported from `px4_commander.py`).
- STABILIZE → arm → GUIDED → EKF origin → `EKF_POS_HORIZ_ABS` wait → NAV_TAKEOFF → 7-strip E-W survey 12 m/s → LAND
- ENU setpoints (identical to `px4_commander.py`); MAVROS converts to NED for ArduPilot
- Two-phase VPE: Phase 1 (AGL < 50 m) = home anchor, Phase 2 (≥ 50 m) = AnyLoc `latest_estimate.json`
- After reaching cruise altitude: switches EKF source to SRC2 (ExternalNav) via `MAV_CMD_DO_AUX_FUNCTION`
- Force-arm fallback via `CommandLong(400, param2=21196)` for SITL pre-arm bypass
- `--manual-takeoff`: skips auto arm/takeoff; waits for RC arm, sets EKF origin from live GPS, waits for GUIDED
- `HOLDTEST=1`: 3 m hold gate (ArduPilot Phase-3 regression test)
- Survey mission, YOLO detection callback, CSV logging — identical to `px4_commander.py`

**`flight_commander.py`** — ArduPilot/MAVROS2 commander (reference archive; superseded by `ardupilot_commander.py`).
- Original WP nav bug: sent NED coordinates to MAVROS2, causing ENU→NED double-conversion (axis swap)
- Velocity-based `go_to_ned()` happened to work directionally; position setpoints were broken
- Kept for historical reference; do not use for new flights

### Parameters

**`px4_no_gps.params`** — PX4 no-GPS external-vision params:
- `EKF2_GPS_CTRL=0`, `SYS_HAS_GPS=0`, `COM_ARM_WO_GPS=1`
- `EKF2_EV_CTRL=15` (fuse EV pos+height+vel+yaw), `EKF2_HGT_REF=3` (vision altitude)
- `EKF2_BARO_CTRL=0`, `COM_RC_IN_MODE=4`, failsafes disabled
- Apply once with `apply_px4_params.sh` — persists in `parameters.bson`

**`real_hw.parm`** — ArduPilot real-hardware params (dual-source EKF):
- `GPS_TYPE=1`, `EK3_SRC1_POSXY=3` (GPS for arming/takeoff), `EK3_SRC2_POSXY=6` (ExternalNav for survey)
- `VISO_TYPE=1`, `BRD_SAFETYENABLE=1`, `PSC_NE_VEL_I=0.0`, `GUID_TIMEOUT=30`
- RC aux channel: `RCx_OPTION=90` (EKF Source Select — LOW=SRC1/GPS, HIGH=SRC2/ExternalNav)
- Upload via Mission Planner or MAVProxy: `param load control/real_hw.parm`

**`no_gps.parm`** — ArduPilot SITL no-GPS params:
- `EK3_SRC1_POSXY=6`, `EK3_SRC1_POSZ=6` (ExternalNav), `GPS_TYPE=0`
- `FS_CRASH_CHECK=0`, `ARMING_CHECK=0`, `DISARM_DELAY=0`
- **Do not upload to real FC** — SITL-only settings

### Launch scripts

**Real hardware (Jetson + ArduPilot FC):**

| Script | Purpose |
|--------|---------|
| `launch_mavros_real.sh` | MAVROS2 → ArduPilot FC via `/dev/ttyUSB0:921600` |
| `launch_camera.sh` | v4l2_camera: `/dev/video0`, YUYV 1280×960 @ 30 fps → `/drone/camera/image_raw` (rgb8) |
| `hw_bridge.py` | Converts MAVROS EKF position to `/drone/state`, `/drone/pose`, `/drone/agl` |
| `launch_real_hw.sh` | Full real-hardware stack: MAVROS + camera (or streamer) + hw_bridge + AnyLoc + YOLO + commander. Pass `--stream-host IP` for direct UDP ground view stream, or `--stream-server IP` for RTSP push to MediaMTX relay — either replaces `launch_camera.sh`. |
| `launch_gstreamer.sh` | Simple H.265 camera stream to ground station — camera + AnyLoc tile only, no YOLO. Opens camera directly — don't run with `launch_camera.sh` or `ground_view_stream.py`. |

**Simulation (SITL):**

| Script | Purpose |
|--------|---------|
| `launch_px4_sitl.sh` | Start PX4 SITL (checks TCP 4560, waits for UDP 14580) |
| `stop_px4_sitl.sh` | Stop PX4 SITL gracefully |
| `apply_px4_params.sh` | Set + save PX4 params, auto-reboot PX4 |
| `launch_mavros_px4.sh` | MAVROS2 → PX4 (`fcu_url udp://:14540@127.0.0.1:14580`) |
| `launch_commander_px4.sh` | Run `px4_commander.py` (sources ROS2) |
| `launch_sitl.sh` | ArduPilot SITL via MAVProxy (`--wipe` flag) |
| `launch_mavros.sh` | MAVROS2 → ArduPilot SITL (UDP 14550) |
| `launch_commander_ardupilot.sh` | Run `ardupilot_commander.py` (sources ROS2, `PYTHONUNBUFFERED=1`) |
| `launch_commander.sh` | Run `flight_commander.py` (legacy reference) |

### Test / diagnostics

**`px4_bridge_test.py`** — standalone HIL link test (no ROS2, no MAVROS). Connects to TCP 4560, streams HIL_SENSOR for 30 s, prints frame count and EKF attitude. Use to verify the bridge/PX4 link before involving MAVROS.

---

## PX4 Launch Sequence

> **Critical:** the bridge must own TCP 4560 **before** PX4 starts.

```bash
# 1. Bridge first (TCP 4560 server)
PX4_SIM=1 python3 control/drone_sim.py          # headless
# or:  bash simulator/run_chiayi.sh --px4       # Isaac Sim

# 2. PX4 SITL
bash control/launch_px4_sitl.sh [--wipe]        # --wipe deletes parameters.bson; log overwrites /tmp/px4_sitl.log; PID → /tmp/px4_sitl.pid

# 3. Apply params (first run only — persists)
bash control/apply_px4_params.sh

# 4. MAVROS2
bash control/launch_mavros_px4.sh

# 5. Commander
source /opt/ros/jazzy/setup.bash
python3 control/px4_commander.py
# or: HOLDTEST=1 python3 control/px4_commander.py
```

Or use the top-level launcher:
```bash
bash run.sh --tmux --px4              # full Isaac Sim pipeline
bash run.sh --tmux --px4 --params     # + apply params (first run)
```

### Stopping PX4 SITL

PX4 is launched with `setsid nohup` and survives terminal/tmux-window close. To stop it after the mission:

```bash
bash control/stop_px4_sitl.sh
```

The script tries (in order): MAVLink `MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN` (if MAVROS is up) → SIGTERM to `/tmp/px4_sitl.pid` → pkill SIGTERM → SIGKILL.

### Hard-won PX4 notes

- **No `-d` flag**: using `px4 -d` changes the working directory, breaking the `px4-param` IPC socket path. Use `setsid nohup` without `-d`; run from the rootfs dir.
- **`fcu_protocol:="v2.0"`** must NOT be passed to MAVROS: PX4 denies `MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES` (520), causing MAVROS VER plugin to double-satisfy a future → `Promise already satisfied` crash.
- **ENU convention**: `setpoint_raw/local` MAVROS2 always converts ENU→NED regardless of `FRAME_LOCAL_NED` flag. Send `x=East, y=North, z=Up(AGL)`.
- **Stale bridge**: if a previous `drone_sim.py` is running on TCP 4560, PX4 silently connects to it. Always kill stale instances before starting the pipeline.
- **`run.sh` pkill pattern**: the pattern must be `'/px4 |bin/px4$|mavros_node|px4_commander'` — a wider pattern (e.g. `'px4'`) matches `bash run.sh --px4` and kills the launcher itself.
- **Commander stdout buffering**: `px4_commander.py` must be launched with `PYTHONUNBUFFERED=1` (already set in `launch_commander_px4.sh`) — without it, all `print()` output is held in a 4 kB pipe buffer when stdout is piped to `tee`, making the log appear silent for the entire flight.

---

## PX4 Phase Status

| Phase | Status | Description |
|-------|--------|-------------|
| 1 | Done | Bridge↔PX4 validated: 27k+ HIL_SENSOR frames, EKF2 level attitude |
| 2 | Done | Vision + MAVROS↔PX4 link; EKF tracks truth |
| 3 | Done | Position-hold gate: 3 m AGL, 40 s, <0.3 m drift |
| 4 | Done | Waypoint nav in `px4_commander.py`: 65 m AGL, 699 m leg, fly-home + AUTO.LAND |
| 5 | Done | Isaac Sim pipeline wired (`run_chiayi.sh --px4`, `run.sh --tmux --px4`) |
| 6 | Done ✓ | End-to-end Isaac Sim waypoint flight: horiz_err < 60 m at 699 m leg |
| 7 | In progress | AnyLoc + detection integration in full pipeline |
| 8 | Done ✓ | Survey mission: 7-strip E-W lawnmower 91.7 m spacing (33 m overlap) + YOLO log-in-flight (no divert) |

---

## ArduPilot Launch Sequence (ardupilot_commander.py)

```bash
bash run.sh --tmux                                    # headless (drone_sim.py)
bash run.sh --tmux --isaac                            # Isaac Sim
bash run.sh --tmux --isaac --anyloc --detection       # full pipeline
```

Or via the top-level launcher:
```bash
bash run.sh --tmux --wipe                             # first run (wipe EEPROM)
```

Manual steps:
```bash
# 1. Bridge (must own UDP 9002 before ArduPilot starts)
python3 control/drone_sim.py        # headless
# or: bash simulator/run_chiayi.sh  # Isaac Sim (no --px4 flag)

# 2. ArduPilot SITL via MAVProxy
bash control/launch_sitl.sh [--wipe]

# 3. MAVROS2
bash control/launch_mavros.sh

# 4. Commander
bash control/launch_commander_ardupilot.sh
# or: HOLDTEST=1 python3 control/ardupilot_commander.py
```

## ArduPilot Takeoff Sequence (ardupilot_commander.py)

**Auto mode (no flags):**
1. Start VPE thread (Phase 1: home-anchor at 20 Hz)
2. STABILIZE → arm (force-arm fallback via `CommandLong` for SITL)
3. Publish EKF global origin to `/mavros/global_position/set_gp_origin`
4. Wait for `EKF_POS_HORIZ_ABS` (GPS SRC1)
5. GUIDED mode → `MAV_CMD_NAV_TAKEOFF` to 65 m AGL
6. Hold 5 s at cruise altitude
7. Switch EKF source → SRC2 (ExternalNav) via `MAV_CMD_DO_AUX_FUNCTION`
8. Wait for `EKF_POS_HORIZ_ABS` re-confirmed on SRC2
9. Velocity-carrot survey navigation (ENU setpoints)

**Manual-takeoff mode (`--manual-takeoff`):**
1. Start VPE thread
2. Wait for RC arm → set EKF origin from live GPS position at arm moment
3. Wait for FC in GUIDED mode + AGL > 5 m
4. Survey starts automatically

## ArduPilot Phase Status

| Phase | Status | Description |
|-------|--------|-------------|
| AP-1 | Done ✓ | SITL + drone_sim.py: bridge connects, physics packets |
| AP-2 | Done ✓ | EKF origin + arm in GUIDED succeeds |
| AP-3 | **Done ✓** (2026-06-19) | HOLDTEST: 40 s hold at 3 m AGL — **0.1 m drift** (PSC_NE rename fix) |
| AP-4 | **Done ✓** (2026-06-19) | Full survey: 7-strip lawnmower, 14 WPs, 65 m AGL, 12 m/s, landed ✓ |
| AP-5 | Pending | Isaac Sim pipeline: `run.sh --tmux --isaac` + full survey |
| AP-6 | Pending | AnyLoc + detection: `run.sh --tmux --isaac --anyloc --detection` |

## Hard-won ArduPilot notes

- **PSC parameter rename (V4.8.0-dev)** — `PSC_POSXY_P` → `PSC_NE_POS_P`; `PSC_VELXY_P/I/D` → `PSC_NE_VEL_P/I/D`. Old names are **silently ignored** — no error, no warning. The defaults that activate (VEL_I=1.0, POS_P=1.0) cause integral windup and underdamped oscillation. Always verify `param show PSC_NE*` in MAVProxy after loading the parm file. Required: `PSC_NE_POS_P=0.2`, `PSC_NE_VEL_I=0.0`, `PSC_NE_VEL_D=0.5`.
- **ENU setpoints** — MAVROS2 always converts ENU→NED regardless of `FRAME_LOCAL_NED` flag. Send `x=East, y=North, z=Up(AGL)` (same as PX4). The original `flight_commander.py` bug: it sent NED (`x=north, y=east`) which MAVROS treated as ENU — axis-swapping the target.
- **EKF origin required** — ArduPilot (unlike PX4) requires explicit `/mavros/global_position/set_gp_origin` publication. PX4 auto-sets from the first EV pose.
- **EKF_POS_HORIZ_ABS** — wait for bit 4 (0x010) of `EKF_STATUS_REPORT` (MAVLink msg 193) via `/uas1/mavlink_source`. Not `local_pos.z < 5 m` as in PX4.
- **Force-arm fallback** — `CommandLong(command=400, param1=1.0, param2=21196.0)` bypasses all pre-arm checks for SITL.
- **NAV_TAKEOFF** — ArduPilot climbs autonomously to the requested AGL; commander only monitors. PX4 needs continuous position setpoints during climb.
- **LAND not AUTO.LAND** — ArduPilot's land mode string is `"LAND"`; `"AUTO.LAND"` is PX4-specific.
- **RTL unsafe** — same as PX4: RTL needs a GPS-derived home. Use explicit `go_to_ned(0,0,alt)` + `LAND`.
- **MAVROS stdout buffering** — launch with `PYTHONUNBUFFERED=1` (already set in `launch_commander_ardupilot.sh`).
- **MAVProxy in the path** — `launch_sitl.sh` starts MAVProxy; MAVROS connects to UDP 14550. PX4 has no MAVProxy.

---

## Coordinate Conventions

| Frame | Convention | Used by |
|-------|-----------|---------|
| `/drone/state` | ENU, MSL altitude (z = metres MSL) | cesium_scene.py, drone_sim.py |
| VPE to MAVROS | ENU `"map"` frame (MAVROS converts to NED) | commanders |
| `setpoint_raw/local` | ENU (MAVROS converts to NED) | commanders |
| PX4 EKF2 internal | NED | autopilot |
| ArduPilot EKF3 internal | NED | autopilot |
