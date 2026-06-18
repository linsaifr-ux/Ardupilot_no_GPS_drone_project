# Plan: Distributed Sim — PC runs Isaac Sim + PX4 SITL, Jetson runs everything else

## Goal

Simulate the real drone software stack on Jetson Orin NX while the PC provides the
virtual environment. On real hardware the Jetson runs MAVROS, commander, AnyLoc, and
YOLO — so the sim must run those same components on Jetson.

---

## What Runs Where

| Component | Sim (PC) | Sim (Jetson) | Real drone |
|---|---|---|---|
| Isaac Sim (`cesium_scene.py`) | ✓ | — | — (real camera) |
| PX4 SITL | ✓ | — | — (Pixhawk) |
| **MAVProxy bridge** | ✓ (new) | — | — |
| MAVROS | — | ✓ | ✓ → Pixhawk serial |
| `px4_commander.py` | — | ✓ | ✓ |
| `anyloc/ros2_node.py` | — | ✓ | ✓ |
| `detection/ros2_node.py` | — | ✓ | ✓ |

Everything in the Jetson column is identical to what runs on the real drone.

---

## Network

- Jetson Orin NX RJ45 → same router/switch as PC
- `ROS_DOMAIN_ID=0` on both machines
- Both source `/opt/ros/jazzy/setup.bash`

```
PC                          Jetson Orin NX
────────────────────────    ──────────────────────────
Isaac Sim                   MAVROS
PX4 SITL                    px4_commander.py
MAVProxy  ←──UDP──────────► anyloc/ros2_node.py
          ──UDP──────────►   detection/ros2_node.py
          ◄─────────────── 
```

---

## MAVLink Routing — MAVProxy Bridge (Option A)

PX4 SITL and MAVROS normally talk over localhost UDP. With MAVROS on Jetson,
MAVProxy on PC bridges between PX4 SITL and the network.

**How it works:**

```
PX4 SITL (listens :14580, sends to :14540)
    ↕  UDP localhost
MAVProxy on PC
  --master udp:127.0.0.1:14580   ← connects to PX4's listening port
  --out    udp:<JETSON_IP>:14540  → forwards MAVLink to Jetson
    ↕  UDP over LAN
MAVROS on Jetson (binds :14540, replies to source automatically)
```

MAVProxy is fully bidirectional — MAVROS commands flow back through it to PX4.
PX4 SITL still sends to :14540 on localhost; those packets are harmlessly dropped
since nothing binds that port on PC anymore.

**MAVProxy command (PC):**
```bash
mavproxy.py \
    --master=udp:127.0.0.1:14580 \
    --out=udp:<JETSON_IP>:14540 \
    --daemon
```

**MAVROS fcu_url (Jetson):**
```
udp://:14540@
```
Same as current — MAVROS binds :14540 and learns the reply address from received packets.

---

## ROS2 Topic Data Flows

ROS2 DDS handles these automatically with matching `ROS_DOMAIN_ID`:

```
PC (Isaac Sim)                       Jetson
─────────────────────────────────────────────────────
/drone/camera/image_raw  ──────────► anyloc/ros2_node.py
/drone/pose              ──────────► anyloc/ros2_node.py
/drone/agl               ──────────► anyloc/ros2_node.py
                                     detection/ros2_node.py
/drone/state             ──────────► px4_commander.py  (Phase 1 VPE truth)

                         Jetson-internal (no network hop):
                         /anyloc/pose_estimate → px4_commander.py
                         /mavros/* ↔ px4_commander.py
```

No ROS2 topics need to flow Jetson → PC. MAVLink (not ROS2) carries PX4 commands.

---

## Code Changes Required

### 1. `anyloc/ros2_node.py` — dynamic covariance in `_publish()`

Currently uses hardcoded XY covariance (20 m²). Commander needs `max(1, error_m²)`.

```python
# _cb_image passes error_m:
self._publish(est_lat, est_lon, drone_alt, error_m=err_m)

# _publish signature:
def _publish(self, lat, lon, alt_msl, error_m: float = 20.0):
    ...
    xy_var = max(1.0, error_m ** 2)
    cov[0] = cov[7] = xy_var
    cov[14] = 0.25
```

### 2. `px4_commander.py` — subscribe to `/anyloc/pose_estimate` instead of JSON

Phase 1 VPE (kinematic truth from `/drone/state`) is unchanged — DDS delivers it from PC.
Phase 2 switches from JSON file polling to ROS2 subscription:

```python
from geometry_msgs.msg import PoseWithCovarianceStamped

# In __init__:
self._anyloc_msg   = None
self._anyloc_stamp = 0.0
self.create_subscription(
    PoseWithCovarianceStamped, "/anyloc/pose_estimate",
    self._cb_anyloc, 1)

def _cb_anyloc(self, msg):
    self._anyloc_msg   = msg
    self._anyloc_stamp = time.time()
```

In the VPE thread Phase 2 block, replace JSON read with:
```python
msg = self._anyloc_msg
if msg is not None and (time.time() - self._anyloc_stamp) < 2.0:
    # use msg.pose.pose.position.{x,y,z} and msg.pose.covariance directly
```

Remove the JSON polling loop entirely.

### 3. `run.sh` — add `--jetson-sim` mode

On PC, `--jetson-sim` skips MAVROS and Commander windows and adds a MAVProxy window:

```bash
bash run.sh --tmux --px4 --jetson-sim            # PC side
bash run.sh --tmux --px4 --jetson-sim --no-window # PC side, headless Isaac Sim
```

Windows on PC: **0 Isaac · 1 PX4 · 2 MAVProxy**

### 4. New `run_jetson.sh` — launch all drone-side services on Jetson

```bash
bash run_jetson.sh          # MAVROS + Commander + AnyLoc + YOLO in tmux
```

Windows on Jetson: **0 MAVROS · 1 Commander · 2 AnyLoc · 3 Detection**

---

## Jetson Setup

```bash
# 1. Copy repo (exclude sim-only files and third_party)
rsync -av --exclude=third_party/ --exclude=simulator/ \
    /path/to/no_GPS_drone_project/ jetson:~/no_GPS_drone_project/

# 2. Copy AnyLoc database (~2 820 entries at AGL 65 m, small)
rsync -av anyloc/database/ jetson:~/no_GPS_drone_project/anyloc/database/

# 3. Copy YOLO model weights
rsync -av yolov8l_visdrone.pt jetson:~/no_GPS_drone_project/

# 4. Environment on Jetson
echo 'export ROS_DOMAIN_ID=0' >> ~/.bashrc
source ~/.bashrc
```

---

## Full Launch Sequence

**PC:**
```bash
export ROS_DOMAIN_ID=0
bash run.sh --tmux --px4 --jetson-sim    # Isaac Sim + PX4 SITL + MAVProxy
```

**Jetson:**
```bash
export ROS_DOMAIN_ID=0
bash run_jetson.sh                        # MAVROS + Commander + AnyLoc + YOLO
```

---

## Verification

```bash
# Jetson: camera frames arriving from PC
ros2 topic hz /drone/camera/image_raw     # expect ~5 Hz

# PC: AnyLoc estimates arriving from Jetson
ros2 topic hz /anyloc/pose_estimate       # expect ~5 Hz

# Jetson: MAVROS connected (MAVProxy bridge working)
ros2 topic echo /mavros/state             # expect connected=True, armed=False
```

---

## Real Hardware Transition

Jetson `run_jetson.sh` is identical. Only two things change:

| | Sim | Real hardware |
|---|---|---|
| MAVROS `fcu_url` | `udp://:14540@` (via MAVProxy) | `/dev/ttyTHS1:921600` (Pixhawk serial) |
| Commander Phase 1 VPE | kinematic truth from `/drone/state` | skip (EKF2 uses real IMU + baro) |

Add `--real-hw` flag to commander to skip Phase 1 VPE injection.
The `run_jetson.sh` script passes `--real-hw` on real hardware and nothing in sim.
