# Jetson Full Real-Hardware Implementation Plan

**Date:** 2026-06-23  
**For:** Claude Code running on Jetson Orin NX  
**Goal:** Implement the complete no-GPS drone survey pipeline on real ArduPilot hardware  
**Contest:** 第二屆國防應用無人機挑戰賽 — 無GNSS自主偵蒐

---

## What This Project Does

A GPS-denied autonomous survey drone that:
1. Takes off to 65 m AGL
2. Flies a boustrophedon (lawnmower) survey pattern defined in Mission Planner
3. Detects vehicles (YOLO) and logs GPS-estimated positions to `detections.csv`
4. Localises itself using AnyLoc (DINOv2 + VLAD visual place recognition) above 50 m AGL
5. Lands and disarms automatically

---

## Repository Structure

```
Ardupilot_no_GPS_drone_project/
├── control/
│   ├── ardupilot_commander.py   ← MAIN flight controller (Python ROS2 node)
│   ├── no_gps.parm              ← ArduPilot parameters (SITL version)
│   ├── home_elevation.json      ← Home position: {lat, lon, centre_elev_m}
│   ├── launch_mavros.sh         ← SITL MAVROS launch (UDP)
│   └── launch_commander_ardupilot.sh
├── anyloc/
│   ├── ros2_node.py             ← AnyLoc ROS2 node (subscribes /drone/camera/image_raw)
│   ├── localizer.py             ← AnyLoc core (DINOv2 + VLAD retrieval)
│   ├── vo_refiner.py            ← Visual odometry inter-frame refinement
│   ├── build_database.py        ← Build geo-tagged image database from satellite imagery
│   ├── database/                ← Built database (database.pt, db_images/, db_meta.json)
│   └── latest_estimate.json     ← Written by ros2_node; read by ardupilot_commander
├── detection/
│   ├── ros2_node.py             ← YOLO ROS2 node (publishes /yolo/detections)
│   ├── detector.py              ← YOLOv8 inference wrapper
│   └── run_ros2_detector.sh
├── yolov8l_visdrone.pt          ← YOLOv8-L weights (VisDrone fine-tuned)
└── detections.csv               ← Output: detected vehicle positions
```

---

## Critical Gap: Simulation vs. Real Hardware Topics

The simulation uses these topics that **do not exist on real hardware**:

| Topic | Type | Published by | Used by |
|---|---|---|---|
| `/drone/state` | PoseStamped | `drone_sim.py` | `ardupilot_commander.py` VPE + detection position |
| `/drone/pose` | PoseStamped (lat/lon/alt_msl) | `drone_sim.py` | `anyloc/ros2_node.py`, `detection/ros2_node.py` |
| `/drone/agl` | Float64 | `drone_sim.py` | `anyloc/ros2_node.py`, `detection/ros2_node.py` |
| `/drone/camera/image_raw` | Image | Sim camera | All three |

On real hardware, position comes from MAVROS:
- `/mavros/local_position/pose` — EKF ENU position (x=East_m, y=North_m, z=Up_m from home)
- `/mavros/altitude` — mavros_msgs/Altitude; `.relative` field = AGL above home (m)

**Solution:** create `control/hw_bridge.py` that publishes the `/drone/*` topics from MAVROS, so AnyLoc and YOLO require zero changes to their subscription logic.

---

## Hardware Connections

| Hardware | Connection | Notes |
|---|---|---|
| ArduPilot FC (e.g. SDMODELH7V2) | USB-to-TTL adapter → `/dev/ttyUSB0` @ 921600 baud | CP2102/CH340/FT232 adapter; TX→RX, RX→TX, GND→GND |
| Camera (AP-IMX900-Mini-USB3-I5) | USB3 port → `/dev/video0` | UVC-compatible; `v4l2_camera` ROS2 driver |
| GPS jammer | Attached by contest organizers | Always on during flight — expected |

Verify USB-TTL device before proceeding:
```bash
# Find the adapter (plug/unplug to confirm):
ls /dev/ttyUSB*          # typically /dev/ttyUSB0
dmesg | tail -20         # look for "cp210x" / "ch341" / "FTDI" converter attached

# Check baud rate communication:
sudo chmod 666 /dev/ttyUSB0
# or permanent:
sudo usermod -aG dialout $USER   # logout + login after

# If multiple USB-serial devices, identify the FC adapter by unplugging it and comparing:
ls /dev/ttyUSB* before and after plugging in the adapter
```

If two USB-serial devices are present simultaneously (e.g. adapter on USB0, something else on USB1),
set `FCU_DEV=/dev/ttyUSB1` before launching (see `launch_mavros_real.sh` below).

---

## Software Prerequisites on Jetson

### ROS2 Humble

Jetson runs Ubuntu 22.04 (arm64) with ROS2 Humble:

```bash
# Check:
source /opt/ros/humble/setup.bash && echo "ROS2 OK"
```

### MAVROS

```bash
sudo apt install -y ros-humble-mavros ros-humble-mavros-extras ros-humble-mavros-msgs
sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh
```

### v4l2_camera (camera driver)

```bash
sudo apt install -y ros-humble-v4l2-camera
ls /dev/video*
```

### Python venvs (no conda on Jetson)

Two venvs with Jetson-specific PyTorch wheels are already set up:

| Venv | Path | Packages |
|---|---|---|
| anyloc | `/home/jetson/venv/anyloc` | torch 2.5 (NV), faiss, Pillow, numpy |
| yolo | `/home/jetson/venv/yolo` | torch 2.5 (NV), ultralytics, opencv |

Verify:
```bash
/home/jetson/venv/anyloc/bin/python3 -c "import torch, faiss; print('anyloc venv OK')"
/home/jetson/venv/yolo/bin/python3  -c "import torch, ultralytics; print('yolo venv OK')"
```

These venvs are used by:
- `anyloc/run_ros2_localizer.sh` → `/home/jetson/venv/anyloc/bin/python3`
- `detection/run_ros2_detector.sh` → `/home/jetson/venv/yolo/bin/python3`
- `control/launch_real_hw.sh` → both venvs for background processes

---

## Implementation Tasks (in order)

### Task 1 — Update `control/home_elevation.json`

Set this to the **actual contest takeoff point** GPS coordinates and elevation.  
Get elevation from: Mission Planner → right-click → "Set Home Here" shows lat/lon/alt.

```json
{"centre_elev_m": 28.17, "lat": 23.450868, "lon": 120.286135}
```

Replace the lat/lon/elevation with the real contest site values before any other task.  
All waypoint offsets and VPE origin depend on this file being correct.

---

### Task 2 — Create `control/hw_bridge.py` (NEW FILE)

This node bridges MAVROS → `/drone/*` topics so AnyLoc and YOLO work without modification.

```python
#!/usr/bin/env python3
"""
Hardware bridge: publishes /drone/state, /drone/pose, /drone/agl from MAVROS.

Converts EKF ENU local position (from /mavros/local_position/pose) to:
  /drone/state   PoseStamped  position=(East_m, North_m, alt_msl_m)  ← ardupilot_commander
  /drone/pose    PoseStamped  position=(lat, lon, alt_msl_m)          ← anyloc, yolo nodes
  /drone/agl     Float64      metres AGL above home                   ← anyloc, yolo nodes

Run: python3 control/hw_bridge.py
"""
import json
import math
import os
import sys

_ROS2_SITE = "/opt/ros/humble/lib/python3.10/site-packages"
if os.path.isdir(_ROS2_SITE) and _ROS2_SITE not in sys.path:
    sys.path.insert(0, _ROS2_SITE)

import rclpy
import rclpy.node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import Altitude
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Float64

_HOME_CFG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "home_elevation.json")
with open(_HOME_CFG) as _f:
    _h = json.load(_f)
HOME_LAT     = float(_h["lat"])
HOME_LON     = float(_h["lon"])
HOME_ALT_MSL = float(_h["centre_elev_m"])
COS_LAT      = math.cos(math.radians(HOME_LAT))
M_PER_DEG    = 111_320.0

_SENSOR_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                          durability=DurabilityPolicy.VOLATILE, depth=10)


class HWBridge(rclpy.node.Node):
    def __init__(self):
        super().__init__("hw_bridge")
        self._agl = 0.0

        self.create_subscription(PoseStamped, "/mavros/local_position/pose",
                                 self._cb_pose, _SENSOR_QOS)
        self.create_subscription(Altitude, "/mavros/altitude",
                                 self._cb_alt, _SENSOR_QOS)

        self._pub_state = self.create_publisher(PoseStamped, "/drone/state", 10)
        self._pub_pose  = self.create_publisher(PoseStamped, "/drone/pose",  10)
        self._pub_agl   = self.create_publisher(Float64,     "/drone/agl",   10)

        self.get_logger().info(
            f"HW bridge ready  HOME={HOME_LAT:.5f},{HOME_LON:.5f}  MSL={HOME_ALT_MSL:.1f} m")

    def _cb_alt(self, msg):
        # msg.relative = AGL above home from ArduPilot baro
        if msg.relative > -100:
            self._agl = float(msg.relative)

    def _cb_pose(self, msg):
        # EKF ENU: x=East, y=North, z=Up from home origin
        east_m  = msg.pose.position.x
        north_m = msg.pose.position.y
        up_m    = msg.pose.position.z   # AGL from home

        alt_msl = HOME_ALT_MSL + up_m
        lat     = HOME_LAT + north_m / M_PER_DEG
        lon     = HOME_LON + east_m  / (M_PER_DEG * COS_LAT)

        stamp = msg.header.stamp

        # /drone/state — ENU metres (used by ardupilot_commander.py)
        s = PoseStamped()
        s.header.stamp = stamp; s.header.frame_id = "map"
        s.pose.position.x = east_m
        s.pose.position.y = north_m
        s.pose.position.z = alt_msl
        s.pose.orientation = msg.pose.orientation
        self._pub_state.publish(s)

        # /drone/pose — WGS84 (used by anyloc and yolo nodes)
        p = PoseStamped()
        p.header.stamp = stamp; p.header.frame_id = "wgs84"
        p.pose.position.x = lat
        p.pose.position.y = lon
        p.pose.position.z = alt_msl
        p.pose.orientation = msg.pose.orientation
        self._pub_pose.publish(p)

        # /drone/agl — prefer barometer (smoother) over EKF z
        agl = self._agl if abs(self._agl) < 500 else up_m
        a = Float64(); a.data = float(max(0.0, agl))
        self._pub_agl.publish(a)


def main():
    rclpy.init()
    node = HWBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
```

---

### Task 3 — Create `control/mission_loader.py` (NEW FILE)

Parses Mission Planner QGC WPL 110 `.waypoints` files into `(north_m, east_m, agl_m)` tuples.

```python
#!/usr/bin/env python3
"""
Parse Mission Planner QGC WPL 110 .waypoints file into ENU survey waypoints.

Returns list of (north_m, east_m, agl_m) relative to (home_lat, home_lon).
Only NAV_WAYPOINT (command=16) items are returned.
coord_frame=3: alt is AGL above home (direct use).
coord_frame=0: alt is MSL; subtract home_alt_msl to get AGL.
"""
import math
import os


def load_mission_planner_waypoints(filepath, home_lat, home_lon, home_alt_msl=0.0):
    if not os.path.isfile(filepath):
        print(f"[mission_loader] File not found: {filepath}")
        return None

    cos_lat   = math.cos(math.radians(home_lat))
    m_per_deg = 111_320.0
    ref_lat   = home_lat
    ref_lon   = home_lon
    wps       = []

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

            # Index 0 is the home row — use it as coordinate origin
            if idx == 0:
                if lat != 0.0 and lon != 0.0:
                    ref_lat = lat; ref_lon = lon
                    cos_lat = math.cos(math.radians(ref_lat))
                continue

            if cmd != 16 or (lat == 0.0 and lon == 0.0):
                continue

            north = (lat - ref_lat) * m_per_deg
            east  = (lon - ref_lon) * m_per_deg * cos_lat
            agl   = alt if frame != 0 else (alt - home_alt_msl)
            wps.append((north, east, agl))

    print(f"[mission_loader] Loaded {len(wps)} waypoints from {os.path.basename(filepath)}")
    for i, (n, e, a) in enumerate(wps):
        print(f"  WP{i:02d}  N={n:+.1f} m  E={e:+.1f} m  AGL={a:.1f} m")
    return wps
```

---

### Task 4 — Modify `control/ardupilot_commander.py`

**4a. Add imports at top (after existing imports):**

```python
import argparse

# Mission Planner waypoint loading
_DEFAULT_WP_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "survey.waypoints")
try:
    from mission_loader import load_mission_planner_waypoints as _load_wp
    _HAVE_LOADER = True
except ImportError:
    _HAVE_LOADER = False
```

**4b. Replace `start_vision()` VPE source selection:**

Find the block starting with comment `# Always use kinematic truth for VPE` (around line 385).  
Replace the entire position selection block with:

```python
                # Phase 1 (SITL): use kinematic truth from /drone/state
                # Phase 2 (real hw, above MIN_LOCALISATION_AGL): use AnyLoc
                # Phase 1 (real hw, below MIN_LOCALISATION_AGL): anchor at home (0,0)
                use_anyloc = (anyloc_est is not None
                              and drone_agl >= MIN_LOCALISATION_AGL)

                if use_anyloc:
                    east_v, north_v, yaw_v, cov_xy = anyloc_est
                elif self._drone is not None:
                    # SITL: kinematic truth always available
                    east_v  = self._drone.pose.position.x
                    north_v = self._drone.pose.position.y
                    yaw_v   = math.pi / 2.0
                    cov_xy  = 0.1
                else:
                    # Real hardware Phase 1: anchor EKF at home; drone is on
                    # ground or climbing — small XY drift from home is acceptable
                    east_v  = 0.0
                    north_v = 0.0
                    yaw_v   = math.pi / 2.0
                    cov_xy  = 0.5

                # Use EKF z for altitude on real hardware when /drone/state absent
                vpe_z = (drone_agl if self._drone is not None
                         else (self._local_pos.pose.position.z
                               if self._local_pos else 0.0))
```

Then update the VPE message z field: change `msg.pose.pose.position.z = drone_agl`  
to: `msg.pose.pose.position.z = vpe_z`

Also change the Phase 2 check from `if drone_agl >= MIN_LOCALISATION_AGL` to use the
`use_anyloc` variable already computed.

Remove the AnyLoc-as-logger-only comment block (the large comment about EKF failsafe
that prevented AnyLoc VPE from being published) — on real hardware AnyLoc IS fused.

**4c. Fix `_cb_detections()` — add real-hardware fallback:**

Find the guard `if self._drone is None: return` at the top of `_cb_detections()`.
Replace it and the subsequent position extraction with:

```python
        if self._drone is not None:
            ds    = self._drone.pose.position
            cur_n, cur_e = ds.y, ds.x
            agl   = max(1.0, ds.z - HOME_ALT_MSL)
            q     = self._drone.pose.orientation
        elif self._local_pos is not None:
            p     = self._local_pos.pose.position
            cur_n, cur_e = p.y, p.x
            agl   = max(1.0, p.z)
            q     = self._local_pos.pose.orientation
        else:
            return
```

**4d. Add real-hardware guard in `engage_guided()` force-arm section:**

Find the force-arm fallback block (around line 512 — `param2=21196.0`).
Wrap it:

```python
        if not os.environ.get("ALLOW_FORCE_ARM"):
            self.get_logger().error(
                "Arm rejected. Real hardware: press safety button, verify VPE "
                "is publishing, check EKF POS_ABS. Set ALLOW_FORCE_ARM=1 for SITL only.")
            return False
        # Force-arm (SITL only):
        ...original force-arm code...
```

**4e. Update `main()` — add arg parsing + dynamic waypoint loading:**

Replace the first two lines of `main()`:
```python
def main():
    rclpy.init()
    cmd = ArduPilotCommander()
```
with:
```python
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--waypoint-file", default=_DEFAULT_WP_FILE)
    args, _ = parser.parse_known_args()

    global SURVEY_WPS
    if _HAVE_LOADER:
        mp_wps = _load_wp(args.waypoint_file, HOME_LAT, HOME_LON, HOME_ALT_MSL)
        if mp_wps:
            SURVEY_WPS = mp_wps
            print(f"[APCmd] {len(SURVEY_WPS)} waypoints from "
                  f"{os.path.basename(args.waypoint_file)}")
        else:
            print(f"[APCmd] No waypoint file — using hardcoded SURVEY_WPS "
                  f"({len(SURVEY_WPS)} wps)")

    rclpy.init()
    cmd = ArduPilotCommander()
```

---

### Task 5 — Modify `anyloc/ros2_node.py` — add `--headless` flag

The current node requires a display for matplotlib. On real hardware during a contest
flight there is no monitor. Add a headless flag.

**Find** in `main()`:
```python
def main():
    rclpy.init()
    node = AnyLocNode()

    # ROS2 spin in background thread so matplotlib can own the main thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        run_postview(node)
```

**Replace** `main()` with:
```python
def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true",
                        help="Disable matplotlib postview window (for flight)")
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = AnyLocNode()

    if args.headless:
        print("[AnyLoc] Running headless — no postview window")
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()
        return

    # ROS2 spin in background thread so matplotlib can own the main thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    try:
        run_postview(node)
```

Also add `import argparse` at the top of the file if not present.

---

### Task 6 — Modify `detection/ros2_node.py` — add `--headless` flag

Same pattern as AnyLoc. Find `main()` and replace:

```python
def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true")
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = YOLONode()

    if args.headless:
        print("[YOLO] Running headless — no postview window")
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()
        return

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    try:
        run_postview(node)
```

---

### Task 7 — Create `control/launch_mavros_real.sh` (NEW FILE)

```bash
#!/bin/bash
# MAVROS2 connected to real ArduPilot FC via USB-to-TTL adapter.
# Default device: /dev/ttyUSB0 — override with: FCU_DEV=/dev/ttyUSB1 bash launch_mavros_real.sh
set -e
source /opt/ros/humble/setup.bash

FCU_DEV="${FCU_DEV:-/dev/ttyUSB0}"

if [ ! -c "$FCU_DEV" ]; then
    echo "[mavros_real] ERROR: $FCU_DEV not found"
    echo "  Plug in USB-to-TTL adapter and check: ls /dev/ttyUSB*"
    exit 1
fi
sudo chmod 666 "$FCU_DEV"

pkill -f mavros_node 2>/dev/null; sleep 1

echo "[mavros_real] Connecting to ArduPilot FC at $FCU_DEV:921600 ..."
ros2 run mavros mavros_node \
    --ros-args \
    -p fcu_url:="serial://${FCU_DEV}:921600" \
    -p tgt_system:=1 \
    -p tgt_component:=1 \
    -p log_output:="screen" \
    -p fcu_protocol:="v2.0"
# plugin_denylist removed — param plugin enabled for real hardware
# Add -p gcs_url:="udp://@GCS_PC_IP:14550" for telemetry to Mission Planner
```

---

### Task 8 — Create `control/launch_camera.sh` (NEW FILE)

The camera (AP-IMX900-Mini-USB3-I5) is a UVC USB camera. Publish to `/drone/camera/image_raw`.

```bash
#!/bin/bash
# Launch USB camera driver on Jetson
source /opt/ros/humble/setup.bash

# Find camera device — AP-IMX900 typically appears as /dev/video0
CAMERA_DEV="${CAMERA_DEV:-/dev/video0}"
echo "[camera] Using $CAMERA_DEV"

ros2 run v4l2_camera v4l2_camera_node \
    --ros-args \
    -p video_device:="$CAMERA_DEV" \
    -p image_size:=[1024,768] \
    -p pixel_format:="YUYV" \
    -r /image_raw:=/drone/camera/image_raw \
    -r /camera_info:=/drone/camera/camera_info
```

**Verify camera before using:**
```bash
# List available devices:
v4l2-ctl --list-devices
# Preview camera (requires display or VNC):
ros2 run image_view image_view --ros-args -r image:=/drone/camera/image_raw
# Check topic is publishing:
ros2 topic hz /drone/camera/image_raw   # expect ~30 Hz
```

---

### Task 9 — Create `control/real_hw.parm` (NEW FILE)

Separate parameter file for real hardware (do not overwrite `no_gps.parm`).  
Upload to ArduPilot FC via Mission Planner or MAVProxy.

```
# ArduPilot parameters for real hardware, no-GPS survey
# Contest: GPS-denied via jammer; position from ExternalNav (AnyLoc VPE)

# Frame
FRAME_CLASS     1       # quadrotor
FRAME_TYPE      1       # quad-X

# GPS disabled (jammer attached)
GPS_TYPE        0
GPS_ARMING_MIN_SAT 0
FS_GPS_ENABLE   0
FENCE_ENABLE    0

# ExternalNav (VPE from Jetson AnyLoc)
VISO_TYPE       1       # enable MAVLink visual odometry
EK3_SRC1_POSXY  6       # ExternalNav → XY position
EK3_SRC1_VELXY  6       # ExternalNav → XY velocity (damping)
EK3_SRC1_POSZ   6       # ExternalNav → Z (AnyLoc altitude)
EK3_SRC1_YAW    1       # Compass → yaw (more stable than ExternalNav yaw)

# Real hardware safety (CHANGE FROM SITL VALUES)
BRD_SAFETYENABLE  1     # Physical safety button required before arm
FS_CRASH_CHECK    1     # Re-enable crash detection
ARMING_CHECK      0     # Disable GPS arming check (visual nav only)
DISARM_DELAY      10    # Disarm after 10 s on ground (real HW safety)

# Loop rate — real hardware handles 400 Hz
# Do NOT set SCHED_LOOP_RATE — use ArduPilot default (400 Hz)

# Hover thrust — must calibrate for actual airframe; default 0.35 is usually OK
MOT_THST_HOVER  0.35    # adjust via autotune or trial

# Attitude controller — start conservative; tune after first hover test
ATC_ANG_RLL_P   4.5     # default — tune down if oscillations
ATC_ANG_PIT_P   4.5
ATC_RAT_RLL_P   0.135
ATC_RAT_RLL_I   0.135
ATC_RAT_RLL_D   0.0036
ATC_RAT_PIT_P   0.135
ATC_RAT_PIT_I   0.135
ATC_RAT_PIT_D   0.0036

# Horizontal position controller (V4.8+ param names)
PSC_NE_POS_P    0.2     # below critical P → overdamped
PSC_NE_VEL_P    2.0
PSC_NE_VEL_I    0.0     # MUST be 0 — default 1.0 causes integral windup
PSC_NE_VEL_D    0.5

# Speed limits
WPNAV_SPEED     1200    # 12 m/s max (Python controls actual speed via velocity setpoints)
WPNAV_SPEED_UP  300     # 3 m/s climb
WPNAV_SPEED_DN  150     # 1.5 m/s descent

# EKF failsafe — relax to allow AnyLoc VPE with higher covariance
EK3_POS_ERR_LIM   100   # 100 m position innovation tolerance (default 2 m — too tight for AnyLoc)
EK3_GLITCH_RAD    25    # 25 m glitch radius (default 25 m — keep)

# Timeouts
GUID_TIMEOUT    30      # 30 s GUIDED setpoint timeout (prevent failsafe on CPU spikes)
```

**Upload to FC (choose one method):**
```bash
# Via MAVProxy (connect to FC via USB or telemetry first):
mavproxy.py --master=/dev/ttyTHS1,921600
  > param load control/real_hw.parm
  > param save

# Or via Mission Planner: Config → Full Parameter List → Load from file
```

---

### Task 10 — Create `control/launch_real_hw.sh` (NEW FILE)

```bash
#!/bin/bash
# Full system launch for real hardware contest flight.
#
# Usage:
#   bash control/launch_real_hw.sh                           (uses control/survey.waypoints)
#   bash control/launch_real_hw.sh --waypoint-file my.waypoints
#
# Terminal layout (recommend tmux):
#   Pane 0: MAVROS         (this script pane 0)
#   Pane 1: Camera         (pane 1)
#   Pane 2: HW Bridge      (pane 2)
#   Pane 3: AnyLoc         (pane 3)
#   Pane 4: YOLO           (pane 4)
#   Pane 5: Commander      (pane 5 — foreground)

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

source /opt/ros/humble/setup.bash

echo "=== Real Hardware Launch ==="
echo "Project: $PROJECT_DIR"

# 1. MAVROS
echo "[launch] Starting MAVROS ..."
bash "$SCRIPT_DIR/launch_mavros_real.sh" &
MAVROS_PID=$!
echo "[launch] MAVROS PID=$MAVROS_PID; waiting 6 s ..."
sleep 6

# 2. Camera
echo "[launch] Starting camera driver ..."
bash "$SCRIPT_DIR/launch_camera.sh" &
CAMERA_PID=$!
sleep 3

# 3. Hardware bridge
echo "[launch] Starting hardware bridge ..."
python3 "$SCRIPT_DIR/hw_bridge.py" &
BRIDGE_PID=$!
sleep 2

# 4. AnyLoc (headless — no display needed during flight)
echo "[launch] Starting AnyLoc localizer ..."
/home/jetson/venv/anyloc/bin/python3 -u "$PROJECT_DIR/anyloc/ros2_node.py" --headless &
ANYLOC_PID=$!
sleep 4

# 5. YOLO detector (headless)
echo "[launch] Starting YOLO detector ..."
/home/jetson/venv/yolo/bin/python3 -u "$PROJECT_DIR/detection/ros2_node.py" --headless &
YOLO_PID=$!
sleep 2

# 6. Commander (foreground — shows live flight log)
echo "[launch] Starting ArduPilot commander ..."
python3 "$SCRIPT_DIR/ardupilot_commander.py" "$@"
CMD_EXIT=$?

echo "[launch] Commander exited ($CMD_EXIT) — shutting down ..."
kill $YOLO_PID $ANYLOC_PID $BRIDGE_PID $CAMERA_PID $MAVROS_PID 2>/dev/null || true
exit $CMD_EXIT
```

Make executable:
```bash
chmod +x control/launch_real_hw.sh control/launch_mavros_real.sh control/launch_camera.sh
```

---

### Task 11 — Rebuild AnyLoc Database for Contest Site

The existing database is for the simulation site (HOME_LAT=23.450868, HOME_LON=120.286135).  
If the contest is at a different location, rebuild it.

**The database builder fetches satellite imagery from NLSC PHOTO2 (Taiwan government tile server).**

```bash
# Update CENTER_LAT / CENTER_LON in anyloc/build_database.py
/home/jetson/venv/anyloc/bin/python3 anyloc/build_database.py --rebuild

# Or pass AGL range matching your flight altitude (65 m):
/home/jetson/venv/anyloc/bin/python3 anyloc/build_database.py \
    --agl-min 60 --agl-max 70 --agl-step 5 --rebuild
```

Database lives at `anyloc/database_vits14/` with a symlink `anyloc/database → anyloc/database_vits14`.

After rebuilding, verify:
```bash
ls -lh anyloc/database_vits14/   # database.pt, database_vlads.pt (~265 MB), db_images/
/home/jetson/venv/anyloc/bin/python3 -c "
import torch
meta = torch.load('anyloc/database_vits14/database_meta.pt')
print(f'DB entries: {len(meta[\"lats\"])}')
"
```

---

### Task 12 — Mission Planner Survey Workflow (on PC)

1. Open Mission Planner → **Flight Plan** tab
2. Right-click map → **Survey (Grid)**
3. Draw 4-corner polygon over contest survey area
4. Settings:
   - **Altitude**: 65 m (relative to home)
   - **Angle**: auto-detect or set to long-axis bearing
   - **Side overlap / Sidelap**: set for coverage (e.g. 30%)
   - **Turn radius**: 0 m (Python handles turns with velocity setpoints)
5. **Accept** → verify waypoints on map
6. **File → Save Waypoints** → `survey.waypoints`
7. Transfer to Jetson:
   ```bash
   scp survey.waypoints USER@JETSON_IP:~/Ardupilot_no_GPS_drone_project/control/
   ```

**Verify the file on Jetson:**
```bash
python3 -c "
import sys; sys.path.insert(0,'control')
from mission_loader import load_mission_planner_waypoints
import json
h = json.load(open('control/home_elevation.json'))
wps = load_mission_planner_waypoints('control/survey.waypoints',
    h['lat'], h['lon'], h['centre_elev_m'])
print(f'{len(wps)} waypoints loaded')
"
```

---

## Pre-flight Verification Sequence

Run these checks BEFORE every flight. All must pass.

```bash
# 1. UART accessible
ls /dev/ttyTHS1 && echo "UART OK" || echo "UART MISSING"

# 2. Camera accessible
ls /dev/video* && echo "Camera device OK"

# 3. MAVROS connects and FC responds
bash control/launch_mavros_real.sh &
sleep 8
ros2 topic echo /mavros/state --once
# Expected: connected: true, mode: "STABILIZE" or similar

# 4. MAVROS local position publishing
ros2 topic hz /mavros/local_position/pose   # expect 30-50 Hz

# 5. Hardware bridge re-publishing
python3 control/hw_bridge.py &
sleep 2
ros2 topic echo /drone/agl --once   # expect a float (AGL m)
ros2 topic echo /drone/state --once # expect position with East/North coords

# 6. Camera publishing
bash control/launch_camera.sh &
sleep 3
ros2 topic hz /drone/camera/image_raw   # expect ~30 Hz

# 7. VPE publishing (start commander first, it publishes VPE)
# Look for: "[APCmd] vision thread started (Phase 1 — home-anchor)" in commander log

# 8. Survey waypoints load correctly
python3 control/ardupilot_commander.py --waypoint-file control/survey.waypoints &
sleep 3; kill $!   # just check it loads

# 9. AnyLoc database exists
ls anyloc/database/database.pt && echo "DB OK"

# 10. EKF reaches POS_ABS before flight
# Watch commander log for: "[APCmd] EKF POS_ABS ✓"
```

---

## Contest Day Launch Sequence

```
T-30 min
  [ ] scp survey.waypoints from Mission Planner PC to Jetson control/
  [ ] Verify waypoint count matches expected: python3 (see Task 12 verify command)
  [ ] Verify home_elevation.json has correct contest site coordinates
  [ ] Power on Jetson; connect to drone via SSH or local terminal

T-15 min
  [ ] Open tmux: tmux new-session -s flight
  [ ] bash control/launch_real_hw.sh --waypoint-file control/survey.waypoints
  [ ] Watch MAVROS pane — "connected: true" must appear
  [ ] Watch camera pane — "image_raw" topic at 30 Hz
  [ ] Watch AnyLoc pane — "AnyLoc node ready — waiting for camera"
  [ ] Watch YOLO pane — "YOLO Waiting for /drone/camera/image_raw"

T-5 min
  [ ] Commander pane shows: "MAVROS connected ✓"
  [ ] Commander pane shows: "waiting for ArduPilot mode to initialize"
  [ ] Commander pane shows: "ArduPilot mode: STABILIZE ✓"

Contest start (jammer ON — GPS LOST — expected, EKF continues from VPE)
  [ ] Press physical safety button on FC → LED changes red → green
  [ ] Commander runs automatically:
        STABILIZE → arm → EKF origin → wait EKF_POS_ABS → GUIDED → takeoff → survey → land
  [ ] Monitor: "[APCmd] EKF POS_ABS ✓" (must appear before GUIDED mode)
  [ ] Monitor: AGL readings climbing to 65 m in takeoff log
  [ ] At 65 m: AnyLoc activates — watch for "[APCmd] AGL N m ≥ 50 m — VPE → AnyLoc"
  [ ] Survey runs: "[APCmd] WP 1/N → ..." printed for each waypoint
  [ ] detections.csv populated during flight (tail -f detections.csv in another pane)
  [ ] Final: "[APCmd] Survey complete" → return to home → LAND → disarm

Emergency stop: Ctrl+C in commander pane → drone holds velocity=0
RC override: flip transmitter to STABILIZE → manual control
```

---

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `ttyUSB0` permission denied | Missing group membership | `sudo chmod 666 /dev/ttyUSB0` or `sudo usermod -aG dialout $USER` |
| `/dev/ttyUSB0` not found | Adapter not plugged in or driver missing | `dmesg | tail -20` to check; try `ls /dev/ttyUSB*` |
| MAVROS not connecting | Wrong device or baud rate | Set `FCU_DEV=/dev/ttyUSB1` if multiple adapters; verify 921600 baud matches FC SERIAL config |
| Arm rejected: Safety Switch | Safety button not pressed | Press physical button until LED is green |
| Arm rejected: Need Position | VPE not publishing or EKF not converged | Check `/mavros/vision_pose/pose_cov` Hz; wait 30 s |
| `/drone/state` not published | hw_bridge not running | Start `python3 control/hw_bridge.py` before commander |
| Camera not found | Device not `/dev/video0` | `ls /dev/video*`; set `CAMERA_DEV=/dev/videoN` |
| AnyLoc not activating | AGL below 50 m threshold | Normal; activates after drone climbs above MIN_LOCALISATION_AGL |
| AnyLoc database error | Wrong site or not rebuilt | Run `build_database.py --rebuild` with correct CENTER_LAT/LON |
| EKF failsafe during survey | AnyLoc covariance too large | Set `EK3_POS_ERR_LIM=100` in real_hw.parm |
| Strips not straight | Commander in position mode | Verify `go_to_ned()` called (velocity setpoints), not `run_auto_survey()` |
| `PSC_NE_VEL_I` windup | Non-zero I term | Verify `PSC_NE_VEL_I=0.0` in uploaded params |
| No detections logged | YOLO below AGL gate or node crashed | Check `MIN_AGL=50.0` in detection/ros2_node.py; check YOLO pane |
| Wrong survey area | home_elevation.json mismatch | Update lat/lon/elev to actual takeoff point |

---

## Key Invariants — Never Break These

1. **ENU to MAVROS**: All setpoints in ardupilot_commander.py use ENU (`x=East, y=North, z=Up`). MAVROS converts to NED. Never send raw NED.
2. **VPE yaw = π/2**: ENU yaw π/2 → MAVROS → NED yaw=0 (North). Never use AnyLoc's yaw field (always 0=East, wrong).
3. **`PSC_NE_VEL_I = 0.0`**: Default 1.0 causes integral windup under ExternalNav. Zero is mandatory.
4. **`GUID_TIMEOUT = 30`**: Default 3 s causes failsafe during Jetson CPU spikes. Must be 30+.
5. **Safety button on real HW**: `BRD_SAFETYENABLE=1`; arm WILL fail without pressing the button. Force-arm (`param2=21196`) is SITL only.
6. **home_elevation.json must match actual takeoff point**: All ENU offsets and VPE origin are relative to this. Wrong home = wrong waypoints.
7. **Velocity setpoints during survey**: `go_to_ned()` uses `make_vel_sp()` → straight strips. Position setpoints (`make_sp()`) cause curved paths via PSC — do not use during the survey phase.
8. **AnyLoc fused above 50 m only**: `MIN_AGL=50.0` in both `anyloc/ros2_node.py` and `MIN_LOCALISATION_AGL=50.0` in `ardupilot_commander.py` must match.
9. **hw_bridge.py must start before commander**: The commander waits for `/drone/state`; if hw_bridge is not running, commander hangs at "waiting for drone state (up to 30 s)".
10. **`real_hw.parm` not `no_gps.parm`**: Upload `control/real_hw.parm` to real FC. `no_gps.parm` has SITL-only settings (SCHED_LOOP_RATE=50, SIM_* entries) that must not go to real hardware.
