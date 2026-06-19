# ArduPilot Migration Plan — Based on Working PX4 Process

**Date:** 2026-06-19  
**Status:** AP-3 HOLD GATE passed ✓ (0.1 m drift). AP-4 full survey passed ✓. AP-5 Isaac Sim pipeline passed ✓. AP-6 full stack passed ✓ (2026-06-19). All phases complete.  
**Goal:** Port the working PX4 survey pipeline back to ArduPilot, fixing the root cause of the original AC_PosControl inversion bug.

---

## Root Cause of Original Failure (Now Identified)

The old `flight_commander.py` sent **NED** coordinates (`x=north, y=east, z=−AGL`) to
`setpoint_raw/local`. The PX4 investigation later revealed: **MAVROS2 always applies
ENU→NED conversion regardless of the `FRAME_LOCAL_NED` flag.** This means MAVROS
was treating `x=north_m` as "East" and swapping the axes before forwarding to ArduPilot.

What ArduPilot received for a target of N=+531, E=−454:
- Commanded: `x=531 (intended north), y=−454 (intended east)` in NED
- MAVROS read as ENU: East=531, North=−454
- MAVROS converted to NED: x_NED=north=−454, y_NED=east=531
- Result: ArduPilot tried to fly to NED north=−454 (south!), east=+531

This is the **exact mirror-direction inversion** observed. The AC_PosControl was
computing correctly — it was given the wrong target.

**Fix:** Send ENU (`x=East, y=North, z=Up`) to `setpoint_raw/local`, identical to
`px4_commander.py`. MAVROS does the ENU→NED conversion for both autopilots.

The velocity-based `go_to_ned()` in `flight_commander.py` already happened to work
because it sent velocity in ENU (`-dx/hdist`, `-dy/hdist` where dx/dy were ENU errors),
so MAVROS converted the ENU velocity to NED correctly and the drone moved in the right
direction. Position setpoints were broken; velocity setpoints were correct.

---

## What Changes vs What Stays the Same

| Component | PX4 (current) | ArduPilot (new) |
|---|---|---|
| Physics bridge | `PX4SimBridge` TCP 4560 HIL | `SITLBridge` UDP 9002 JSON (unchanged) |
| Physics model | Second-order (K_ACCEL=80, K_DAMP=12) | First-order τ=0.15 s (already in `cesium_scene.py` per `PX4_SIM` flag) |
| SITL→MAVROS path | PX4 SITL → UDP 14540/14580 → MAVROS | ArduPilot SITL → MAVProxy TCP 5760 → UDP 14550 → MAVROS |
| Autopilot mode | OFFBOARD | GUIDED |
| Pre-stream setpoints | 40 × 20 Hz before OFFBOARD | Not needed |
| Arm sequence | stream setpoints → OFFBOARD → arm | STABILIZE → arm → GUIDED |
| Takeoff | Climb via OFFBOARD position setpoints | `NAV_TAKEOFF` command |
| EKF origin | Auto-set from first EV pose | Must publish to `/mavros/global_position/set_gp_origin` |
| EKF ready check | Wait for `local_pos.z < 5 m` | Wait for `EKF_POS_HORIZ_ABS` flag via raw MAVLink |
| Setpoint convention | ENU (x=East, y=North, z=Up) — MAVROS converts | **Same** — this was the bug |
| VPE convention | ENU "map" frame, yaw=π/2 hardcoded | **Same** (unchanged) |
| `go_to_ned()` nav | Velocity carrot (v_e, v_n toward target) | **Same** (already worked in old commander) |
| Survey mission | 7-strip E-W, 12 m/s, 91.7 m spacing, 65 m AGL | **Port unchanged** |
| YOLO detection callback | `_cb_detections()` pixel projection + dedup | **Port unchanged** |
| Home fly-back | Explicit go_to_ned(0,0,alt) + AUTO.LAND | go_to_ned(0,0,alt) + LAND |
| Ctrl-C handler | go_to_ned home + AUTO.LAND | go_to_ned home + LAND |
| In-air restart | Detect AGL > 5 m, skip takeoff | **Port unchanged**, switch to GUIDED |

---

## Files to Create / Modify

| File | Action | Description |
|---|---|---|
| `control/ardupilot_commander.py` | **Create** | New commander ported from `px4_commander.py` with GUIDED mode |
| `control/launch_commander_ardupilot.sh` | **Create** | Mirrors `launch_commander_px4.sh` with `PYTHONUNBUFFERED=1` |
| `control/no_gps.parm` | **Update** | Add `WPNAV_SPEED` bump + verify survey-speed params |
| `run.sh` | **Update** | ArduPilot tmux path uses `ardupilot_commander.py` + add `--anyloc`/`--detection` windows |
| `README.md` | **Update** | ArduPilot Quick Start section |

`flight_commander.py` is kept as reference/archive (rename section in README).

---

## `ardupilot_commander.py` — Detailed Spec

### Module-level constants (identical to `px4_commander.py`)

```python
HOME_LAT, HOME_LON, HOME_ALT_MSL   # loaded from home_elevation.json
TAKEOFF_ALT = 65.0                  # m AGL
WAYPOINT_RADIUS = 60.0              # m
WAYPOINT_TIMEOUT = 900.0            # s
MIN_LOCALISATION_AGL = 50.0         # m
SURVEY_SPEED = 12.0                 # m/s
SURVEY_WPS = [...]                  # 14 boustrophedon waypoints (identical)
ZONE_VERTS = [...]                  # buffered boundary (identical)
CAM_W, CAM_H = 1024, 768
HFOV_DEG, VFOV_DEG = 88.0, 65.1
VEHICLE_CLASSES, DET_LOG, DEDUP_RADIUS, ESTIMATE_JSON   # identical
```

### Class `ArduPilotCommander(rclpy.node.Node)`

**Subscribers (same as PX4 commander):**
- `/mavros/state` → `_cb_state`
- `/mavros/local_position/pose` → `_cb_local` (EKF position)
- `/mavros/local_position/velocity_local` → `_cb_vel`
- `/drone/state` → `_cb_drone` (kinematic truth)
- `/yolo/detections` → `_cb_detections` (if vision_msgs available)
- `/drone/camera/image_raw` → `_cb_image` (if PIL available)

**Additional subscriber (ArduPilot-specific):**
- `/uas1/mavlink_source` → `_cb_mavlink`
  - Parses `EKF_STATUS_REPORT` (msg 193) for `EKF_POS_HORIZ_ABS` flag (bit 4)
  - Parses `GPS_GLOBAL_ORIGIN` (msg 49) echo

**Publishers (same as PX4 commander):**
- `/mavros/vision_pose/pose_cov` (PoseWithCovarianceStamped)
- `/mavros/vision_speed/speed_twist` (TwistStamped)
- `/mavros/setpoint_raw/local` (PositionTarget)

**Additional publisher (ArduPilot-specific):**
- `/mavros/global_position/set_gp_origin` (GeoPointStamped)

**Service clients:**
- `/mavros/cmd/arming` (CommandBool)
- `/mavros/set_mode` (SetMode)
- `/mavros/cmd/takeoff` (CommandTOL)  ← ArduPilot NAV_TAKEOFF
- `/mavros/cmd/command` (CommandLong) ← force arm fallback

### Methods — identical ports from `px4_commander.py`

- `_cb_state`, `_cb_local`, `_cb_vel`, `_cb_drone`, `_cb_image`
- `_cb_detections()` — yaw-corrected GSD projection + dedup + log
- `_log_detection()` — CSV append + crop save
- `start_vision()` — 20 Hz VPE thread (Phase 1/2, ENU, yaw=π/2 hardcoded)
- `_spin_until()`, `set_mode()`, `arm()`, `_agl()`
- `make_sp()` — build ENU PositionTarget (position-only mask)
- `go_to_ned()` — **velocity carrot** (already correct in old commander), port from `px4_commander.py`
- `_in_buffered_zone()`, `HOLDTEST` gate loop

### Methods — ArduPilot-specific (from `flight_commander.py`)

- `set_ekf_origin()` — publish to `/mavros/global_position/set_gp_origin`; retry 10× over 5 s; treat as success after 5 s even without GPS_GLOBAL_ORIGIN echo (SITL doesn't echo reliably)
- `_cb_mavlink()` — parse EKF_STATUS_REPORT flags and GPS_GLOBAL_ORIGIN echo
- `wait_ekf_pos()` — block until `EKF_POS_HORIZ_ABS` (bit 4) set; timeout 90 s
- `takeoff()` — send `MAV_CMD_NAV_TAKEOFF` via `/mavros/cmd/takeoff`; monitor `_agl()` until AGL ≥ target − 2 m

### `engage_guided()` — replaces `engage_offboard()`

```
1. set_mode("STABILIZE")
2. sleep 0.5 s
3. arm()  [+ force-arm fallback via CommandLong]
4. set_mode("GUIDED")
5. sleep 0.5 s
6. set_ekf_origin(HOME_LAT, HOME_LON, HOME_ALT_MSL)
7. wait_ekf_pos(timeout=60 s)
```

### `main()` sequence

```
1. rclpy.init(), create ArduPilotCommander
2. Write stub estimate JSON
3. start_vision(stop)
4. Wait MAVROS connected (60 s)
5. Spin 20 × 0.1 s to populate _drone
6. Detect in-air restart (AGL > 5 m)
   → if in-air: ensure GUIDED mode; skip to survey loop
7. engage_guided()
   → ABORT if any step fails
8. takeoff(TAKEOFF_ALT, timeout=180 s)
   → ABORT if fails
9. Hold 5 s at cruise alt (publish position setpoint)
10. HOLDTEST branch (env var HOLDTEST=1)
11. Survey loop:
    for each WP in SURVEY_WPS:
        go_to_ned(wn, we, wagl, timeout, speed=SURVEY_SPEED)
        advance wp_idx on arrival or timeout
12. go_to_ned(0, 0, TAKEOFF_ALT, timeout=300, speed=SURVEY_SPEED)
13. set_mode("LAND")
14. _spin_until(not armed, timeout=150 s)
15. cleanup
```

**KeyboardInterrupt handler:**
```
go_to_ned(0, 0, TAKEOFF_ALT, timeout=120, speed=SURVEY_SPEED)
set_mode("LAND")
_spin_until(not armed, timeout=150 s)
```

---

## `no_gps.parm` Updates

The current `no_gps.parm` is largely correct. Verify/update:

| Param | Current | New | Reason |
|---|---|---|---|
| `WPNAV_SPEED` | 100 (1 m/s) | 1200 (12 m/s) | Match survey speed (WPNAV_SPEED used as reference; actual nav is via velocity setpoints so this is informational) |
| `EK3_SRC1_VELXY` | 6 | 6 | Keep — velocity aiding active |
| All others | unchanged | unchanged | Already tuned |

Note: `WPNAV_SPEED` in cm/s. The survey uses velocity setpoints not WPNAV, but set it
consistent to avoid confusion if AUTO mode is ever used.

---

## `launch_commander_ardupilot.sh`

```bash
#!/bin/bash
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
echo "[Commander AP] Starting ardupilot_commander.py..."
PYTHONUNBUFFERED=1 python3 "$SCRIPT_DIR/ardupilot_commander.py" "$@"
```

---

## `run.sh` Changes

**ArduPilot tmux section:**
- Replace `launch_commander.sh` → `launch_commander_ardupilot.sh`
- Replace `flight_commander` in pkill pattern → `ardupilot_commander`
- Add `--anyloc` / `--detection` windows (windows 3/4) mirroring PX4 path
- Add Isaac Sim window (window 0) — currently ArduPilot tmux skips Isaac Sim

**New ArduPilot flags:**
```
bash run.sh --tmux                                    # ArduPilot + drone_sim.py (headless)
bash run.sh --tmux --wipe                             # + wipe EEPROM
bash run.sh --tmux --isaac                            # ArduPilot + Isaac Sim
bash run.sh --tmux --isaac --anyloc                   # + AnyLoc
bash run.sh --tmux --isaac --anyloc --detection       # + YOLO
```

Or keep existing flag structure and add `--isaac` as the only new flag for ArduPilot
(PX4 already has `--headless` for the inverse). Decision: keep backward compatibility —
`bash run.sh --tmux` stays headless ArduPilot (no Isaac Sim) to preserve existing workflow.

---

## Test Plan

Run these phases in order. Each must pass before the next.

| Phase | Command | Pass Criteria | Result |
|---|---|---|---|
| AP-1 | `python3 control/drone_sim.py` + `launch_sitl.sh` | ArduPilot logs "GPS Glitch" cleared, no crash | Done ✓ |
| AP-2 | + `launch_mavros.sh` + `ardupilot_commander.py HOLDTEST=1` | EKF_POS_HORIZ_ABS set; arm succeeds | Done ✓ |
| AP-3 | `HOLDTEST=1 python3 control/ardupilot_commander.py` | Drone holds 3 m AGL for 40 s; drift < 0.5 m | **PASSED ✓** (0.1 m drift, 2026-06-19) |
| AP-4 | Full 7-strip survey (14 WPs, 65m AGL, 12 m/s) | All 14 WPs navigated correct direction; YOLO log active; landed ✓ | **PASSED ✓** (2026-06-19) |
| AP-5 | Isaac Sim: `run.sh --tmux --isaac` + full survey | End-to-end with cesium_scene.py | **PASSED ✓** (2026-06-19) |
| AP-6 | `run.sh --tmux --isaac --anyloc --detection` | AnyLoc Phase 2 active; YOLO logs detections | **PASSED ✓** (2026-06-19) |

---

## Post-AP-3 Discovery: PSC Parameter Rename in ArduPilot V4.8

**Critical finding (2026-06-19):** ArduPilot V4.8.0-dev renamed the horizontal position controller parameters. The original `no_gps.parm` used V4.3-era names that were silently ignored, leaving dangerous defaults active.

| Old name (V4.3, silently ignored) | New name (V4.8+) | Default | Required |
|---|---|---|---|
| `PSC_POSXY_P` | `PSC_NE_POS_P` | 1.0 | **0.2** |
| `PSC_VELXY_P` | `PSC_NE_VEL_P` | 2.0 | 2.0 |
| `PSC_VELXY_I` | `PSC_NE_VEL_I` | **1.0** | **0.0** |
| `PSC_VELXY_D` | `PSC_NE_VEL_D` | 0.25 | **0.5** |

**Effect of wrong defaults:**
- `PSC_NE_VEL_I = 1.0`: integrator accumulates velocity error over 40 s → grows without bound → oscillation that never converges. Root cause of every growing-oscillation failure observed.
- `PSC_NE_POS_P = 1.0`: above the overdamped critical value (~0.44 with VEL_P=2.0, τ_att=0.15 s) → underdamped complex poles → sustained oscillation even with I=0.

**Stability analysis (corrected params: POS_P=0.2, VEL_I=0, VEL_D=0.5):**
- Inner velocity loop dominant pole: τ_vel ≈ 0.71 s (from `0.429s² + 3.01s + 3.0 = 0`)
- Outer loop characteristic: `1.065s² + 1.5s + 0.2 = 0`
- Discriminant: `2.25 − 4×1.065×0.2 = 1.398 > 0` → real poles → overdamped ✓
- Poles: τ₁ ≈ 6.7 s, τ₂ ≈ 0.79 s → exp(−40/6.7) ≈ 0.3% remaining at 40 s → passes 0.5 m criterion

**`no_gps.parm` was updated** to use the V4.8 names. Verify with `param show PSC_NE*` in MAVProxy after startup.

Full debugging record: `instructions/ap3_holdgate_solving_process.md`.

---

## Key Differences to Remember During Implementation

1. **No pre-stream loop** — GUIDED mode doesn't require 40 setpoints before activation.
2. **EKF origin** — must be published; PX4 doesn't need this (auto-sets from first EV pose).
3. **NAV_TAKEOFF** — ArduPilot climbs on its own; commander just monitors AGL.
4. **`wait_ekf_pos()`** — must wait for EKF_POS_HORIZ_ABS via raw MAVLink (not EKF z < 5 m).
5. **LAND not AUTO.LAND** — ArduPilot's land mode string is `"LAND"` not `"AUTO.LAND"`.
6. **RTL unsafe** — same as PX4: RTL needs GPS-derived home. Use explicit go_to_ned + LAND.
7. **Velocity setpoints** — send ENU velocity in `go_to_ned()`. MAVROS converts to NED.
   The old `flight_commander.py` `go_to_ned()` already did this correctly (it was only
   the position-setpoint hold loop that used the wrong NED convention).
8. **Force-arm fallback** — ArduPilot may refuse regular arm due to VisOdom pre-arm check;
   use `CommandLong` with `param2=21196.0` as fallback (already in `flight_commander.py`).
9. **SITL loop rate** — `SCHED_LOOP_RATE=50` in `no_gps.parm`; bridge provides JSON at
   100 Hz so this is fine.
10. **MAVProxy in the path** — `launch_sitl.sh` starts MAVProxy; MAVROS connects to UDP 14550.
    PX4 has no MAVProxy.

---

## Coordinate Convention Reference

| What | Send to MAVROS | MAVROS converts | ArduPilot receives |
|---|---|---|---|
| VPE position | ENU (x=East, y=North, z=Up=AGL) in frame "map" | → NED | NED pos (correct) |
| VPE yaw | ENU yaw = π/2 (North) | → NED yaw = 0 (North) | 0° heading (North) ✓ |
| Position setpoint | ENU (x=East, y=North, z=Up=AGL) | → NED | NED pos (correct) |
| Velocity setpoint | ENU (vx=East vel, vy=North vel, vz=Up vel) | → NED | NED vel (correct) |
| Old (broken) position | NED (x=North, y=East, z=−AGL) | → NED (treating as ENU) | Swapped axes ✗ |
