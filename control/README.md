# control/ — Flight control, SITL bridges, and autopilot integration

This module connects the simulator physics to an autopilot (SITL) and runs the autonomous mission.
Two autopilots are supported via the `PX4_SIM` environment variable:

| Autopilot | Bridge | Transport | Commander | Status |
|-----------|--------|-----------|-----------|--------|
| **PX4** (`PX4_SIM=1`) | `px4_sim_bridge.py` | MAVLink HIL, TCP 4560 | `px4_commander.py` | **Full mission ready** — phases 1–5 complete; position-hold gate passed (<0.3 m drift); waypoint nav 90 m AGL / 699 m leg implemented |
| **ArduPilot** (default) | `sitl_bridge.py` | JSON FDM, UDP 9002 | `flight_commander.py` | Takeoff + 90 m alt OK; **horizontal WP nav has an unresolved `AC_PosControl` direction inversion** |

The PX4 migration was started because ArduPilot's position controller inversion defied exhaustive investigation (EKF/velocity/yaw all verified correct; only direct velocity commands tracked correctly — position setpoints and AUTO mode flew the mirror direction).

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

**`flight_commander.py`** — ArduPilot/MAVROS2 commander (reference; WP nav unresolved).
- STABILIZE → arm → GUIDED → EKF origin → NAV_TAKEOFF → velocity-carrot WP → RTL
- `HOLDTEST=1`, `CALIBRATE=1` diagnostic modes

### Parameters

**`px4_no_gps.params`** — PX4 no-GPS external-vision params:
- `EKF2_GPS_CTRL=0`, `SYS_HAS_GPS=0`, `COM_ARM_WO_GPS=1`
- `EKF2_EV_CTRL=15` (fuse EV pos+height+vel+yaw), `EKF2_HGT_REF=3` (vision altitude)
- `EKF2_BARO_CTRL=0`, `COM_RC_IN_MODE=4`, failsafes disabled
- Apply once with `apply_px4_params.sh` — persists in `parameters.bson`

**`no_gps.parm`** — ArduPilot no-GPS params:
- `EK3_SRC1_POSXY=6`, `EK3_SRC1_POSZ=6` (ExternalNav), `GPS_TYPE=0`
- `FS_CRASH_CHECK=0`, `ARMING_CHECK=0`, `DISARM_DELAY=0`

### Launch scripts

| Script | Purpose |
|--------|---------|
| `launch_px4_sitl.sh` | Start PX4 SITL (checks TCP 4560, waits for UDP 14580); saves PID to `/tmp/px4_sitl.pid`; overwrites `/tmp/px4_sitl.log` on each launch |
| `stop_px4_sitl.sh` | Stop PX4 SITL gracefully — tries MAVLink shutdown → SIGTERM (saved PID) → pkill SIGTERM → SIGKILL |
| `apply_px4_params.sh` | Set + save PX4 params, auto-reboot PX4 |
| `launch_mavros_px4.sh` | MAVROS2 → PX4 (`fcu_url udp://:14540@127.0.0.1:14580`) |
| `launch_commander_px4.sh` | Run `px4_commander.py` (sources ROS2) |
| `launch_sitl.sh` | ArduPilot SITL via MAVProxy (`--wipe` flag) |
| `launch_mavros.sh` | MAVROS2 → ArduPilot (UDP 14550) |
| `launch_commander.sh` | Run `flight_commander.py` |

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

## ArduPilot Takeoff Sequence (reference)

1. Start VPE thread (Phase 1: kinematic stub at 20 Hz)
2. Set EKF global origin — block up to 60 s
3. Arm in STABILIZE (bypasses GPS pre-arm checks)
4. Switch to GUIDED
5. Wait for `EKF_POS_HORIZ_ABS` flag (VPE accepted)
6. `MAV_CMD_NAV_TAKEOFF` — monitor `/drone/state` AGL
7. Hold 5 s, then velocity-carrot waypoint navigation

---

## Coordinate Conventions

| Frame | Convention | Used by |
|-------|-----------|---------|
| `/drone/state` | ENU, MSL altitude (z = metres MSL) | cesium_scene.py, drone_sim.py |
| VPE to MAVROS | ENU `"map"` frame (MAVROS converts to NED) | commanders |
| `setpoint_raw/local` | ENU (MAVROS converts to NED) | commanders |
| PX4 EKF2 internal | NED | autopilot |
| ArduPilot EKF3 internal | NED | autopilot |
