# Jetson Real-Hardware Survey — Mission Planner Integration Plan

**Date:** 2026-06-23  
**Target:** Jetson Orin NX → real ArduPilot FC via `/dev/ttyTHS1:921600`  
**Goal:** GPS-denied lawnmower survey, waypoints loaded from Mission Planner export  
**Contest:** 第二屆國防應用無人機挑戰賽 — 無GNSS自主偵蒐

---

## What Already Works (SITL — `control/ardupilot_commander.py`)

- `go_to_ned()` with velocity setpoints — straight strips, tuned decel
- `engage_guided()` STABILIZE→arm→GUIDED sequence
- `start_vision()` VPE thread publishing to `/mavros/vision_pose/pose_cov`
- YOLO detection + lat/lon logging to `detections.csv`
- EKF origin + `wait_ekf_pos()` handshake
- `WAYPOINT_RADIUS=30 m`, `APPROACH_DECEL=0.015` — confirmed no overshoot on SITL traces

## What Must Change for Real Hardware

| Item | SITL | Real Hardware |
|---|---|---|
| MAVROS FCU URL | `udp://:14550@` | `serial:///dev/ttyTHS1:921600` |
| Plugin denylist | `['param']` (workaround) | Remove — enable param plugin |
| Survey waypoints | Hardcoded `SURVEY_WPS` list | Loaded from `survey.waypoints` file |
| Phase 1 VPE source | `/drone/state` kinematic truth | Static "home anchor" (0, 0, MSL) |
| Phase 2 VPE source | AnyLoc (logger only in SITL) | AnyLoc fused to EKF |
| Detection position | `self._drone.pose` | `self._local_pos.pose` fallback |
| `BRD_SAFETYENABLE` | 0 | **1 — physical safety button required** |
| `FS_CRASH_CHECK` | 0 | **1 — re-enable for real flight** |
| Force-arm fallback | Used in SITL | **Disable — safety hazard on real HW** |

---

## Architecture

```
Mission Planner (PC)
  └─ survey.waypoints ──scp──▶  Jetson Orin NX
                                  ├─ launch_real_hw.sh
                                  │    ├─ launch_mavros_real.sh
                                  │    │    └─ MAVROS /dev/ttyTHS1:921600
                                  │    ├─ anyloc/run_ros2_localizer.sh
                                  │    └─ ardupilot_commander.py
                                  │         ├─ load_mission_planner_waypoints()
                                  │         ├─ start_vision() → VPE thread
                                  │         ├─ go_to_ned() velocity setpoints
                                  │         └─ _cb_detections() → detections.csv
                                  │
                              /dev/ttyTHS1:921600
                                  │
                             ArduPilot FC
                             (GPS_TYPE=0, EK3_SRC=ExternalNav)
```

---

## Step 1 — Mission Planner Survey Planning (on PC)

1. Open Mission Planner → **Flight Plan** tab
2. Right-click map → **Survey (Grid)**
3. Draw polygon over the contest survey area
4. Survey parameters:
   - **Altitude**: 65 m (relative to home)
   - **Angle**: 300° (NW-SE long axis) or let Mission Planner auto-detect
   - **Overlap**: set for your camera's FOV
   - **Turn radius**: 0 m (Python handles turns with velocity setpoints)
5. Click **Accept** → waypoints appear on map
6. **File → Save Waypoints** → save as `survey.waypoints`
7. Copy to Jetson:
   ```bash
   scp survey.waypoints jetson_user@jetson_ip:~/Ardupilot_no_GPS_drone_project/control/
   ```

### Waypoint File Format

Mission Planner exports QGC WPL 110 (tab-separated). Example:
```
QGC WPL 110
0	1	0	16	0	0	0	0	23.450868	120.286135	28.17	1
1	0	3	16	0	0	0	0	23.456780	120.273990	65.000000	1
2	0	3	16	0	0	0	0	23.455640	120.281690	65.000000	1
...
```

Column order: `index  current  coord_frame  command  p1  p2  p3  p4  lat  lon  alt  autocontinue`

- Index 0: home position (`current=1`, used as origin reference)
- Survey WPs: `command=16` (NAV_WAYPOINT), `coord_frame=3` (alt = metres AGL above home)

---

## Step 2 — New File: `control/mission_loader.py`

Create this file. It is imported by `ardupilot_commander.py`.

```python
import math
import os


def load_mission_planner_waypoints(filepath, home_lat, home_lon, home_alt_msl=0.0):
    """
    Parse a Mission Planner QGC WPL 110 .waypoints file.

    Returns list of (north_m, east_m, agl_m) in local ENU relative to
    (home_lat, home_lon). Only NAV_WAYPOINT (command=16) items are returned;
    home placeholder and non-survey commands are skipped.

    coord_frame=3 (FRAME_GLOBAL_RELATIVE_ALT): alt is directly AGL above home.
    coord_frame=0 (FRAME_GLOBAL): alt is MSL; subtract home_alt_msl to get AGL.
    """
    if not os.path.isfile(filepath):
        print(f"[mission_loader] File not found: {filepath}")
        return None

    cos_lat = math.cos(math.radians(home_lat))
    m_per_deg = 111_320.0

    # Read home from index-0 line if lat/lon non-zero; else use caller-supplied values
    file_home_lat = home_lat
    file_home_lon = home_lon

    wps = []
    with open(filepath) as f:
        header = f.readline().strip()
        if not header.startswith("QGC WPL"):
            print(f"[mission_loader] Not a QGC WPL file: {filepath}")
            return None

        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split("\t")
            if len(parts) < 11:
                continue
            try:
                idx   = int(parts[0])
                frame = int(parts[2])
                cmd   = int(parts[3])
                lat   = float(parts[8])
                lon   = float(parts[9])
                alt   = float(parts[10])
            except (ValueError, IndexError):
                continue

            # Use file's home row as reference if it has real coordinates
            if idx == 0 and lat != 0.0 and lon != 0.0:
                file_home_lat = lat
                file_home_lon = lon
                cos_lat = math.cos(math.radians(file_home_lat))
                continue

            if cmd != 16:       # only NAV_WAYPOINT
                continue
            if lat == 0.0 and lon == 0.0:
                continue        # skip dummy entries

            north = (lat - file_home_lat) * m_per_deg
            east  = (lon - file_home_lon) * m_per_deg * cos_lat

            if frame == 0:      # FRAME_GLOBAL: alt is MSL
                agl = alt - home_alt_msl
            else:               # FRAME_GLOBAL_RELATIVE_ALT: alt is AGL
                agl = alt

            wps.append((north, east, agl))

    print(f"[mission_loader] Loaded {len(wps)} waypoints from {os.path.basename(filepath)}")
    for i, (n, e, a) in enumerate(wps):
        print(f"  WP{i:02d}  N={n:+.1f}  E={e:+.1f}  AGL={a:.1f} m")
    return wps
```

---

## Step 3 — Changes to `control/ardupilot_commander.py`

### 3.1 Import `mission_loader` and add CLI argument (top of file, after existing imports)

Add after the existing `import` block:

```python
import argparse

# ── Mission Planner waypoint file (overrides hardcoded SURVEY_WPS) ─────────────
_DEFAULT_WP_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "survey.waypoints")
try:
    from mission_loader import load_mission_planner_waypoints as _load_wp
    _HAVE_LOADER = True
except ImportError:
    _HAVE_LOADER = False
```

### 3.2 Replace hardcoded `SURVEY_WPS` loading

Find the block that ends with the `SURVEY_WPS = [...]` list (lines ~99-114 in the current file). Keep the hardcoded list as a fallback but add dynamic loading **after** the list:

```python
# Attempt to load from Mission Planner file; hardcoded list is fallback only.
# Populated in main() after arg parsing — placeholder here for module-level access.
_MISSION_FILE = _DEFAULT_WP_FILE
```

Keep the `SURVEY_WPS = [...]` block exactly as-is. It becomes the fallback.

### 3.3 Update `start_vision()` for real hardware (Phase 1 VPE)

The current Phase 1 reads from `self._drone` (simulator kinematic truth). On real hardware `self._drone` is always `None`. Replace the VPE publishing block with a real-hardware-aware version.

**Find this block** (around line 386-409):
```python
                # Always use kinematic truth for VPE.  AnyLoc estimates ...
                if self._drone is not None:
                    east_v  = self._drone.pose.position.x
                    north_v = self._drone.pose.position.y
                else:
                    east_v, north_v = 0.0, 0.0
                yaw_v  = math.pi / 2.0
                cov_xy = 0.1
```

**Replace with:**
```python
                # Phase 1 VPE: if simulator kinematic truth available (SITL),
                # use it.  On real hardware (self._drone is None), publish a
                # static "home anchor" at (0,0) so EKF stays initialised while
                # climbing.  Phase 2: when AnyLoc has a valid estimate at
                # MIN_LOCALISATION_AGL, switch to that.
                use_anyloc = (anyloc_est is not None
                              and drone_agl >= MIN_LOCALISATION_AGL)

                if use_anyloc:
                    east_v, north_v, yaw_v, cov_xy = anyloc_est
                elif self._drone is not None:
                    east_v  = self._drone.pose.position.x
                    north_v = self._drone.pose.position.y
                    yaw_v   = math.pi / 2.0
                    cov_xy  = 0.1
                else:
                    # Real hardware Phase 1: anchor EKF at home position (0,0)
                    # with low covariance.  Drone is on the ground or climbing —
                    # small horizontal drift from home is acceptable here.
                    east_v  = 0.0
                    north_v = 0.0
                    yaw_v   = math.pi / 2.0
                    cov_xy  = 0.5   # slightly wider than SITL truth — still tight
```

Also **remove the AnyLoc-is-logger comment** immediately above the block (it's now wrong for real hardware) and update the print at the first publish:
```python
                # Change this:
                if n_sent == 1:
                    print("[APCmd] vision thread started (Phase 1 — truth)")
                # To:
                if n_sent == 1:
                    src = "AnyLoc" if use_anyloc else ("truth" if self._drone else "home-anchor")
                    print(f"[APCmd] vision thread started (Phase 1 — {src})")
```

Also update `msg.pose.pose.position.z` to use `agl` (not `drone_agl`) on real hardware:
```python
                # Current line:
                msg.pose.pose.position.z    = drone_agl
                # Change to:
                msg.pose.pose.position.z = drone_agl if self._drone is not None else agl
```

### 3.4 Fix `_cb_detections()` to work without `/drone/state`

**Find** (around line 255):
```python
    def _cb_detections(self, msg):
        """Project YOLO detections to world coords via yaw-corrected GSD and log."""
        if self._drone is None:
            return
        ...
        ds    = self._drone.pose.position
        cur_n = ds.y
        cur_e = ds.x
        agl   = max(1.0, ds.z - HOME_ALT_MSL)
        ...
        q       = self._drone.pose.orientation
```

**Replace** the guard and position extraction with:
```python
    def _cb_detections(self, msg):
        """Project YOLO detections to world coords via yaw-corrected GSD and log."""
        # Use kinematic truth if available (SITL); otherwise EKF local position
        if self._drone is not None:
            ds    = self._drone.pose.position
            cur_n = ds.y
            cur_e = ds.x
            agl   = max(1.0, ds.z - HOME_ALT_MSL)
            q     = self._drone.pose.orientation
        elif self._local_pos is not None:
            p     = self._local_pos.pose.position
            cur_n = p.y
            cur_e = p.x
            agl   = max(1.0, p.z)
            q     = self._local_pos.pose.orientation
        else:
            return   # no position estimate yet
```

### 3.5 Update `engage_guided()` — disable force-arm on real hardware

The force-arm bypass (`param2=21196`) is a SITL-only safety override. On real hardware it bypasses the physical safety button, which is dangerous. Add a guard:

**Find** (around line 510):
```python
        # Force-arm fallback: bypasses all pre-arm checks (SITL VisOdom health, GPS, etc.)
        self.get_logger().warn("regular arm failed — retrying with force arm …")
        ...
        req2.param2  = 21196.0   # force magic
```

**Replace** the force-arm section with:
```python
        # Force-arm: SITL only.  On real hardware this bypasses the physical
        # safety button — prohibited.
        if os.environ.get("ALLOW_FORCE_ARM"):
            self.get_logger().warn("regular arm failed — retrying with force arm …")
            ...
            req2.param2  = 21196.0
            ...
        else:
            self.get_logger().error(
                "Arm failed.  On real hardware: verify safety button pressed, "
                "VPE publishing, and EKF POS_ABS.  Set ALLOW_FORCE_ARM=1 for SITL only.")
            return False
```

### 3.6 Update `main()` — add argument parsing and dynamic waypoint loading

**Replace** the start of `main()`:
```python
def main():
    rclpy.init()
    cmd = ArduPilotCommander()
```

**With:**
```python
def main():
    parser = argparse.ArgumentParser(description="ArduPilot survey commander")
    parser.add_argument("--waypoint-file", default=_DEFAULT_WP_FILE,
                        help="Path to Mission Planner .waypoints file "
                             "(default: control/survey.waypoints)")
    args, _ = parser.parse_known_args()

    # Load Mission Planner waypoints if file exists; keep hardcoded list as fallback
    global SURVEY_WPS
    if _HAVE_LOADER:
        mp_wps = _load_wp(args.waypoint_file, HOME_LAT, HOME_LON, HOME_ALT_MSL)
        if mp_wps:
            SURVEY_WPS = mp_wps
            print(f"[APCmd] Using {len(SURVEY_WPS)} waypoints from "
                  f"{os.path.basename(args.waypoint_file)}")
        else:
            print(f"[APCmd] No valid waypoint file — using hardcoded SURVEY_WPS "
                  f"({len(SURVEY_WPS)} waypoints)")
    else:
        print("[APCmd] mission_loader not found — using hardcoded SURVEY_WPS")

    rclpy.init()
    cmd = ArduPilotCommander()
```

---

## Step 4 — New File: `control/launch_mavros_real.sh`

```bash
#!/bin/bash
# Launch MAVROS2 connected to real ArduPilot FC via UART on Jetson Orin NX.
#
# Hardware: /dev/ttyTHS1 at 921600 baud (Jetson 40-pin header)
# Run order:
#   Terminal 1: MAVROS    (this script)
#   Terminal 2: AnyLoc    (./anyloc/run_ros2_localizer.sh)
#   Terminal 3: Commander (python3 control/ardupilot_commander.py)

set -e
source /opt/ros/jazzy/setup.bash

# Verify UART is accessible
if [ ! -c /dev/ttyTHS1 ]; then
    echo "[mavros_real] ERROR: /dev/ttyTHS1 not found"
    exit 1
fi
sudo chmod 666 /dev/ttyTHS1

# Kill stale MAVROS instances
pkill -f mavros_node 2>/dev/null; sleep 1

echo "[mavros_real] Connecting to ArduPilot FC at /dev/ttyTHS1:921600 ..."

ros2 run mavros mavros_node \
    --ros-args \
    -p fcu_url:="serial:///dev/ttyTHS1:921600" \
    -p tgt_system:=1 \
    -p tgt_component:=1 \
    -p log_output:="screen" \
    -p fcu_protocol:="v2.0"
# NOTE: plugin_denylist removed — param plugin enabled for real hardware
# NOTE: gcs_url omitted; add -p gcs_url:="udp://@GCS_IP:14550" for telemetry to PC
```

---

## Step 5 — New File: `control/launch_real_hw.sh`

```bash
#!/bin/bash
# Full launch: MAVROS + AnyLoc + ardupilot_commander on real hardware.
# Usage: bash control/launch_real_hw.sh [--waypoint-file path/to/survey.waypoints]
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

echo "=== Real Hardware Launch ==="
echo "Project: $PROJECT_DIR"

# Step 1: MAVROS
echo "[real_hw] Launching MAVROS (UART)..."
bash "$SCRIPT_DIR/launch_mavros_real.sh" &
MAVROS_PID=$!
sleep 6

# Step 2: AnyLoc localizer
echo "[real_hw] Launching AnyLoc localizer..."
bash "$PROJECT_DIR/anyloc/run_ros2_localizer.sh" &
ANYLOC_PID=$!
sleep 3

# Step 3: Commander
echo "[real_hw] Launching ArduPilot commander..."
source /opt/ros/jazzy/setup.bash
python3 "$SCRIPT_DIR/ardupilot_commander.py" "$@"

# Cleanup on exit (Ctrl+C or normal exit)
echo "[real_hw] Shutting down..."
kill $ANYLOC_PID $MAVROS_PID 2>/dev/null || true
```

Make executable: `chmod +x control/launch_real_hw.sh control/launch_mavros_real.sh`

---

## Step 6 — Parameter Changes (`control/no_gps.parm`)

Apply these changes before uploading to real FC. The comments show what to change from the current SITL values:

```
# ── CHANGE from SITL values ─────────────────────────────────────────────────
BRD_SAFETYENABLE    1       # Real hardware: physical safety button required
FS_CRASH_CHECK      1       # Re-enable crash detection (was 0 in SITL)

# ── KEEP exactly as-is from SITL ─────────────────────────────────────────────
GPS_TYPE            0       # GPS disabled (jammer attached; ExternalNav only)
VISO_TYPE           1       # Enable MAVLink visual odometry (VPE from Jetson)
EK3_SRC1_POSXY      6       # ExternalNav → horizontal position
EK3_SRC1_POSZ       6       # ExternalNav → vertical (AnyLoc Z)
EK3_SRC1_VELXY      6       # ExternalNav → velocity aiding
EK3_SRC1_YAW        1       # Compass → yaw (or 6 for ExternalNav yaw)
ARMING_CHECK        0       # No GPS arming check (visual nav only)
PSC_NE_POS_P        0.2
PSC_NE_VEL_P        2.0
PSC_NE_VEL_I        0.0     # MUST be 0 — default 1.0 causes integral windup
PSC_NE_VEL_D        0.5
WPNAV_SPEED         1200    # 12 m/s (actual speed controlled by Python vel cmds)
GUID_TIMEOUT        30      # Prevent failsafe on Jetson CPU spikes
ATC_ANG_RLL_P       2.5
ATC_ANG_PIT_P       2.5
ATC_RAT_RLL_P       0.15
ATC_RAT_PIT_P       0.15
ATC_RAT_RLL_I       0.0     # Prevent I-term windup on climb
ATC_RAT_PIT_I       0.0
```

Upload without EEPROM wipe (preserves compass cal):
```bash
# Via MAVProxy on the PC side (GCS telemetry):
param load /path/to/no_gps.parm
```

---

## Step 7 — AnyLoc Phase 2 VPE Fusion (enable for real hardware)

On SITL, AnyLoc was intentionally kept as a background logger (not fused to EKF) because its covariance (~800 m²) caused EKF failsafe. For real hardware contest flight, AnyLoc must actually replace GPS as the position source above 50m AGL.

The `start_vision()` change in Step 3.3 already enables this: when `use_anyloc=True`, VPE is published with `cov_xy = max(1.0, err_m**2)`.

To prevent EKF failsafe on first AnyLoc transition, set these parameters:
```
EK3_POS_ERR_LIM     100     # Accept up to 100m position innovation (default 2m — too tight)
EK3_GLITCH_RAD      25      # Tolerate 25m glitch radius during VPE source switch
```

These relax EKF rejection thresholds. After the first AnyLoc-based flight confirms localisation error < 10m, tighten back to safe values.

---

## Step 8 — Pre-flight Checklist (Contest Day)

```
T-30 min
  [ ] scp survey.waypoints from Mission Planner PC to Jetson control/
  [ ] Verify survey.waypoints loads: python3 -c "
        from mission_loader import load_mission_planner_waypoints
        wps = load_mission_planner_waypoints('control/survey.waypoints', 23.450868, 120.286135, 28.17)
        print(f'{len(wps)} waypoints loaded')"

T-15 min
  [ ] Power on Jetson; verify /dev/ttyTHS1 exists
  [ ] Verify camera: ros2 topic hz /drone/camera/image_raw  (expect ~30 Hz)
  [ ] Check AnyLoc database matches contest site

T-5 min
  [ ] Open terminal: bash control/launch_real_hw.sh
  [ ] Verify MAVROS connected: ros2 topic echo /mavros/state  (connected=True)
  [ ] Verify VPE publishing: ros2 topic hz /mavros/vision_pose/pose_cov  (expect 20 Hz)
  [ ] Verify EKF POS_ABS: watch for "[APCmd] EKF POS_ABS ✓" in commander log

Contest start
  [ ] Contest organizer attaches GPS jammer (GPS LOST — expected)
  [ ] Press physical safety button on FC → LED changes from red to green
  [ ] Commander runs automatically: arm → GUIDED → takeoff → survey → land
  [ ] Monitor detections.csv in real time: tail -f detections.csv

Emergency
  RC transmitter: switch to STABILIZE (manual control, velocity setpoints ignored)
  Ctrl+C on commander: sends velocity=0; drone holds; then switch RC to LAND
```

---

## Step 9 — Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| MAVROS not connecting | Wrong baud or missing permissions | `ls /dev/ttyTHS1`; `sudo chmod 666 /dev/ttyTHS1` |
| Arm rejected: "Safety Switch" | Safety button not pressed | Press physical safety button; verify LED green |
| Arm rejected: "Need Position" | VPE not publishing or EKF not converged | Check `/mavros/vision_pose/pose_cov` is publishing; wait 30s |
| EKF failsafe during survey | AnyLoc covariance too large | Set `EK3_POS_ERR_LIM=100`, `EK3_GLITCH_RAD=25` |
| Strips still curved | Velocity setpoints not active | Verify `go_to_ned()` is called (not `run_auto_survey()`) |
| Drone drifts in hover | `PSC_NE_VEL_I` non-zero windup | Verify `PSC_NE_VEL_I=0.0` in uploaded params |
| No detections logged | YOLO node not running or AGL < 50m | Start YOLO node; check AGL gate in detection code |
| `survey.waypoints` not loaded | File path wrong | Pass `--waypoint-file /absolute/path/survey.waypoints` |
| Wrong survey area | HOME_LAT/LON mismatch with contest site | Update `HOME_LAT`, `HOME_LON`, `HOME_ALT_MSL` in commander to match actual takeoff point |

---

## Key Invariants (Do Not Break)

1. **ENU convention**: all setpoints to MAVROS are ENU (`x=East, y=North, z=Up`). MAVROS converts to NED. Never send raw NED.
2. **VPE yaw always π/2**: ENU yaw=π/2 → MAVROS converts → NED yaw=0 (North). Never use AnyLoc yaw (it's always 0 = East heading, which is wrong).
3. **Velocity setpoints during survey**: `go_to_ned()` uses `make_vel_sp()` → straight strips. Never switch to `make_sp()` (position setpoints) during the survey phase.
4. **`PSC_NE_VEL_I=0.0`**: default 1.0 causes I-term windup under ExternalNav. Must be zero.
5. **`GUID_TIMEOUT=30`**: prevents failsafe during processing spikes. Do not lower.
6. **Safety button on real hardware**: `BRD_SAFETYENABLE=1`; never use force-arm on real HW.
7. **HOME_LAT/LON must match actual takeoff point**: waypoints are offsets from HOME. If takeoff moves, update HOME and re-run the waypoint loader.
