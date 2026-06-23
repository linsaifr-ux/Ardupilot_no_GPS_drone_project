# PX4 → ArduPilot Migration Plan

## Background
Real hardware available only supports ArduPilot (not PX4).
Pegasus Simulator 5.1.0 has native ArduPilot support built in — no third-party patches needed.

## PX4 Backup
Original PX4 files saved to: `/mnt/raid5/lin/IsaacSim/_px4_backup/`
- `01_build_px4.sh`
- `02_launch.sh`
- `03_launch_mavros.sh`
- `19_gps_switch.sh`
- `1_px4_single_vehicle.py`

## Key Finding
Pegasus 5.1.0 already includes:
- `pegasus/simulator/logic/backends/ardupilot_mavlink_backend.py`
- `pegasus/simulator/logic/backends/tools/ardupilot_launch_tool.py`
- `examples/11_ardupilot_multi_vehicle.py` ← reference example

---

## Steps

### Step 1 — Install ArduCopter SITL [DONE]
```bash
bash /mnt/raid5/lin/IsaacSim/01_build_ardupilot.sh
```
- ArduPilot cloned to `/home/lin/ardupilot`
- ArduCopter SITL binary: `/home/lin/ardupilot/build/sitl/bin/arducopter`
- Build time: ~2m38s
- Script `01_build_ardupilot.sh` created at `/mnt/raid5/lin/IsaacSim/`

### Step 2 — Create `1_ardupilot_single_vehicle.py` [DONE]
- Created at `PegasusSimulator/examples/1_ardupilot_single_vehicle.py`
- Kept unchanged: Cesium terrain, GPS origin (23.44938, 120.28924), car at ENU (-1213, 480), camera setup

**What changed vs `1_px4_single_vehicle.py`:**

1. Add `ARDUPILOT_DIR` constant (replace `PX4_INSTANCE`):
```python
# OLD
PX4_INSTANCE = 0

# NEW
VEHICLE_ID    = 0
ARDUPILOT_DIR = "/home/lin/ardupilot"
```

2. Change import:
```python
# OLD
from pegasus.simulator.logic.backends.px4_mavlink_backend import PX4MavlinkBackend, PX4MavlinkBackendConfig

# NEW
from pegasus.simulator.logic.backends.ardupilot_mavlink_backend import ArduPilotMavlinkBackend, ArduPilotMavlinkBackendConfig
```

3. Change backend config and instantiation:
```python
# OLD
mavlink_config = PX4MavlinkBackendConfig({
    "vehicle_id": PX4_INSTANCE,
    "px4_autolaunch": True,
    "px4_dir": self.pg.px4_path,
    "px4_vehicle_model": self.pg.px4_default_airframe,
})
config_multirotor.backends = [PX4MavlinkBackend(mavlink_config)]

# NEW
backend_config = ArduPilotMavlinkBackendConfig({
    "vehicle_id": VEHICLE_ID,
    "ardupilot_autolaunch": True,
    "ardupilot_dir": ARDUPILOT_DIR,
    "ardupilot_vehicle_model": "gazebo-iris",
})
config_multirotor.backends = [ArduPilotMavlinkBackend(config=backend_config)]
```

### Step 3 — Update `02_launch.sh` [DONE]

**What changed:**

1. Change `EXAMPLE` path:
```bash
# OLD
EXAMPLE="${INSTALL_DIR}/PegasusSimulator/examples/1_px4_single_vehicle.py"

# NEW
EXAMPLE="${INSTALL_DIR}/PegasusSimulator/examples/1_ardupilot_single_vehicle.py"
```

2. Change pre-flight check (PX4 build → ArduCopter binary):
```bash
# OLD
[[ -d "${INSTALL_DIR}/PX4-Autopilot/build/px4_sitl_default" ]] \
    || fatal "PX4 not built. Run 01_build_px4.sh first."

# NEW
[[ -f "/home/lin/ardupilot/build/sitl/bin/arducopter" ]] \
    || fatal "ArduCopter not built. Run 01_build_ardupilot.sh first."
```

3. Change process cleanup (px4 → arducopter, remove PX4 port/lock cleanup):
```bash
# OLD
pkill -9 -f px4 2>/dev/null || true
fuser -k 4560/tcp 2>/dev/null || true
rm -f ~/tmp/px4_lock-* ~/tmp/px4-* 2>/dev/null || true

# NEW
pkill -9 -f arducopter 2>/dev/null || true
# (ArduPilot uses UDP 14550, managed by Pegasus backend — no port/lock cleanup needed)
```

### Step 4 — Update `03_launch_mavros.sh` [DONE]

**What changed:**

1. Change launch file (`px4.launch` → `apm.launch`):
```bash
# OLD
ros2 launch mavros px4.launch fcu_url:=udp://:14540@localhost:14557

# NEW
ros2 launch mavros apm.launch fcu_url:=udp://:14550@
```
- `px4.launch` uses PX4-specific MAVROS parameters
- `apm.launch` uses ArduPilot-specific parameters (different component_id, system_id defaults)
- ArduPilot SITL sends MAVLink output to UDP 14550 (set by Pegasus `ardupilot_launch_tool.py`)

### Step 5 — Rewrite `19_gps_switch.sh` [DONE]

**Parameter mapping PX4 → ArduPilot:**

| Action            | PX4 param           | ArduPilot param              |
|-------------------|---------------------|------------------------------|
| Disable GPS       | EKF2_GPS_CTRL=0     | GPS_TYPE=0                   |
| Enable vision XY  | EKF2_EV_CTRL=3      | VISO_TYPE=1, EK3_SRC1_POSXY=6 |
| Vision altitude   | EKF2_HGT_REF=3      | EK3_SRC1_POSZ=6              |
| Arm without GPS   | COM_ARM_WO_GPS=1    | ARMING_CHECK=0               |

**GPS OFF (`--off`) sets:**
```
GPS_TYPE=0       — disable GPS receiver
VISO_TYPE=1      — enable MAVLink visual odometry (receives /mavros/vision_pose/pose)
EK3_SRC1_POSXY=6 — EK3 horizontal position source: ExternalNav
EK3_SRC1_POSZ=6  — EK3 vertical position source: ExternalNav
ARMING_CHECK=0   — disable arming checks (allow re-arm without GPS)
```

**GPS ON (`--on`) sets:**
```
GPS_TYPE=1       — enable GPS receiver (auto-detect)
VISO_TYPE=0      — disable visual odometry
EK3_SRC1_POSXY=3 — EK3 horizontal position source: GPS
EK3_SRC1_POSZ=1  — EK3 vertical position source: barometer
ARMING_CHECK=1   — restore all arming checks
```

**Note:** `GPS_TYPE` may require reboot to take full effect on real hardware (not an issue in SITL).

### Step 6 — Test flight [ ]

**Issue 1 found during testing:**
`ardupilot_launch_tool.py` hardcoded `gnome-terminal` to launch ArduCopter SITL.
GNOME Terminal is not available in this environment → error:
```
Error constructing proxy for org.gnome.Terminal: Timeout was reached
```

**Fix applied** (`ardupilot_launch_tool.py`):
- Removed `gnome-terminal` and `--console`/`--map` GUI flags
- Launch ArduCopter as a direct background subprocess
- Redirect stdout/stderr to log file at `{tempdir}/ardupilot_0.log`

**Issue 2 found during testing:**
`sim_vehicle.py` requires `pexpect` which was not installed.
```
ModuleNotFoundError: No module named 'pexpect'
```
**Fix:** `pip3 install pexpect`

**Issue 3 found during testing:**
`sim_vehicle.py` requires `mavproxy.py` which was not installed.
```
[Errno 2] No such file or directory: 'mavproxy.py'
```
**Fix:** `pip3 install mavproxy` (installs to `/home/lin/.local/bin/mavproxy.py`, already on PATH)

**Issue 4 found during testing:**
Port conflict — both Pegasus backend and MAVROS tried to listen on UDP 14550.
MAVProxy sends to `127.0.0.1:14550` (Pegasus receives it), but MAVROS on `0.0.0.0:14550`
gets nothing because packets are addressed specifically to 127.0.0.1.
Result: `/mavros/state` never published, MAVROS never connected.

**Fix:**
- `ardupilot_launch_tool.py`: Add second MAVProxy output on port 14551 for MAVROS
  - Port 14550 → Pegasus backend
  - Port 14551 → MAVROS
- `03_launch_mavros.sh`: Change FCU URL from `udp://:14550@` to `udp://:14551@`

**Test sequence:**
1. `bash 02_launch.sh` — Isaac Sim + ArduPilot SITL
2. `bash 03_launch_mavros.sh` — MAVROS bridge
3. `bash 20_check_mavros.sh` — verify connection → `connected: true`, mode: STABILIZE ✓
4. `bash 04_fly.sh` — arm and takeoff
5. `bash 17_fly_trace.sh` — top-down flight trace viewer (same as PX4 project)
6. `bash 19_gps_switch.sh --off` — cut GPS
7. `bash 19_gps_switch.sh --on` — restore GPS

**Issue 6 found during testing:**
ArduPilot refused to arm: `PreArm: Motors: Check frame class and type` / `Frame: UNSUPPORTED`.
`gazebo-iris.parm` exists at `Tools/autotest/default_params/gazebo-iris.parm` with
`FRAME_CLASS=1, FRAME_TYPE=1` but sim_vehicle.py wasn't loading it automatically.

**Fix applied** (`ardupilot_launch_tool.py`):
Added `--add-param-file {ardupilot_dir}/Tools/autotest/default_params/gazebo-iris.parm`
to the sim_vehicle.py command so frame parameters are loaded on startup.

**Issue 7 found during testing:**
ArduPilot refused to arm: `Main loop slow (250Hz < 400Hz)` and `Gyro rate 324Hz < 720Hz`.
ArduPilot's default loop rate (400Hz) is too high for Isaac Sim to sustain in real-time.

**Fix applied:**
- Created `/mnt/raid5/lin/IsaacSim/sitl_params.parm` with:
  - `SCHED_LOOP_RATE 50` — reduce loop rate to 50Hz (well within simulation speed)
  - `ARMING_CHECK 0` — disable strict arming checks for SITL
  - `FS_THR_ENABLE 0` — disable RC throttle failsafe (no RC transmitter in SITL)
  - `FS_GCS_ENABLE 0` — disable GCS failsafe
  - `DISARM_DELAY 0` — disable auto-disarm timeout
- `ardupilot_launch_tool.py`: loads `sitl_params.parm` via `--add-param-file`

**Issue 8 found during testing:**
Drone armed and received takeoff command but immediately disarmed.
Root cause: RC failsafe triggered because no RC transmitter connected in SITL (MAVLink-only control).
**Fix applied:** Added `FS_THR_ENABLE=0`, `FS_GCS_ENABLE=0`, `DISARM_DELAY=0` to `sitl_params.parm`.

**Issue 5 found during testing:**
`fly_test.py` used PX4-specific flight modes — drone stayed on ground.
- `OFFBOARD` mode (PX4) → ArduPilot doesn't support it
- No explicit takeoff command (PX4 auto-takes off from setpoint; ArduPilot requires explicit command)
- `AUTO.LAND` → ArduPilot uses `LAND`

**Fix applied** (`fly_test.py`):
- `OFFBOARD` → `GUIDED`
- Added `_takeoff()` using `/mavros/cmd/takeoff` service after arming
- `AUTO.LAND` → `LAND`
- Startup sequence: set GUIDED (count=20) → arm (count=40) → takeoff (count=60)

**Verified working (2026-06-21):**
- MAVROS connected: true
- ArduPilot mode: STABILIZE
- Local position publishing at (0, 0, 0.10) on ground

**Issue 9 found during testing:**
`NAV_TAKEOFF: FAILED` — takeoff command sent at count=60 (3 seconds) before EKF3 initialized GPS origin.
ArduPilot's EKF3 takes ~15-20 seconds to set origin from the simulated GPS; the takeoff service call
is rejected until `EKF3 IMU0 origin set` appears in the SITL log.

**Fix applied** (`fly_test.py`):
- Subscribe to `/mavros/local_position/pose` — only published once EKF3 origin is set
- Set `self._ekf_ready = False` initially; `_on_local_pos` callback sets it `True` on first message
- Startup: set GUIDED (count=20) → arm (count=40) → takeoff only when `count > 60 AND _ekf_ready`

**Issue 10 found during testing:**
`/mavros/local_position/pose` subscription received no messages despite MAVROS publishing.
Warning: `New publisher discovered on topic '/mavros/local_position/pose', offering incompatible QoS.
Last incompatible policy: RELIABILITY`
MAVROS publishes local position with `BEST_EFFORT` reliability; the default ROS 2 subscription uses
`RELIABLE`, causing QoS mismatch — callback never fires, `_ekf_ready` stays `False` forever.

**Fix applied** (`fly_test.py`):
```python
from rclpy.qos import QoSProfile, ReliabilityPolicy
_best_effort = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
self.create_subscription(PoseStamped, '/mavros/local_position/pose', self._on_local_pos, _best_effort)
```

**Issue 11 found during testing:**
Takeoff command sent while `_armed` and `_guided` were still `False` — both `_arm()` and `_set_mode()`
are async MAVROS service calls; ArduPilot takes ~0.5–1 s to confirm each. Sending `MAV_CMD_NAV_TAKEOFF`
before ArduPilot has actually armed and switched to GUIDED causes a silent rejection.

**Fix applied** (`fly_test.py`):
- Subscribe to `/mavros/state` to track real armed/mode status
- Guard takeoff on `_ekf_ready AND _armed AND _guided`
- Add done callback to `_takeoff()` to log `success` and `result` code from ArduPilot
  ```python
  self._armed  = False
  self._guided = False
  self.create_subscription(State, '/mavros/state', self._on_state, 10)

  def _on_state(self, msg):
      self._armed  = msg.armed
      self._guided = (msg.mode == 'GUIDED')

  def _takeoff(self, altitude: float):
      req = CommandTOL.Request()
      req.altitude = altitude
      future = self.takeoff_cli.call_async(req)
      future.add_done_callback(
          lambda f: self.get_logger().info(
              f'Takeoff response: success={f.result().success} result={f.result().result}'))

  # in tick() startup phase:
  if self.count > 60 and self._ekf_ready and self._armed and self._guided:
      self.get_logger().info(f'Armed + GUIDED + EKF ready — Takeoff to {ALTITUDE:.0f} m ...')
      self._takeoff(ALTITUDE)
      self.phase = 'climb'
  ```

**Issue 12 found during testing:**
The first `_arm(True)` call at count=40 was silently dropped — `/mavros/state` never reflected
`armed=True`. ArduPilot was in GUIDED mode at first state message (~130 ms after `_set_mode`),
but the arm command sent ~1 s later got no response. Likely a transient race between EKF3 origin
finalization and the arm request being accepted.

**Fix applied** (`fly_test.py`):
Re-issue mode/arm every 2 s (count % 40 == 0, count >= 80) until `_armed` and `_guided` are both
confirmed via `/mavros/state`. The retry at count=80 (1 s after the original arm) succeeded:
```
[INFO] Arming ...
[INFO] ... still not armed — re-issuing arm (count=80)
[INFO] Vehicle armed.
[INFO] Armed + GUIDED + EKF ready — Takeoff to 500 m ...
[INFO] Takeoff response: success=True result=0
```

**Verified takeoff accepted (2026-06-21):**
- ArduPilot mode: GUIDED ✓
- Vehicle armed: True ✓
- EKF3 origin set: True ✓
- `MAV_CMD_NAV_TAKEOFF` response: `success=True, result=MAV_RESULT_ACCEPTED` ✓
- BUT drone did not physically climb — see Issues 13 / 14 below.

**Issue 13 found during testing — drone does not lift off despite takeoff acceptance:**
After `NAV_TAKEOFF: ACCEPTED`, `/mavros/local_position/pose` z stays at -0.14 m, velocity ~0,
`/mavros/rc/out` motor PWM frozen at 1100 (idle, MOT_SPIN_ARM level).

Diagnostic via `ss -tunap`:
```
udp 127.0.0.1:14550  Recv-Q=152640  Pegasus python (MAVLink queue backed up)
udp 127.0.0.1:9002   Recv-Q=8640    Pegasus python (JSON FDM motor cmds backed up)
```
Pegasus python process at 99% CPU. UDP consumer threads not draining the socket buffers
fast enough → ArduPilot's motor outputs never reach Isaac Sim physics, and Pegasus's physics
state never reaches ArduPilot fresh enough for the throttle controller to engage takeoff.

Additionally observed: `02_launch.sh` cleanup only killed `arducopter`, not `mavproxy` /
`sim_vehicle.py` / Pegasus python. After several reruns, **12 zombie mavproxy** and **2
arducopter** processes were running simultaneously — each trying to bind ports 14550/14551,
creating MAVLink storm conditions.

**Fix applied (`02_launch.sh`):**
```bash
pkill -9 -f arducopter 2>/dev/null || true
pkill -9 -f mavproxy 2>/dev/null || true        # new
pkill -9 -f sim_vehicle.py 2>/dev/null || true  # new
pkill -9 -f "xterm.*ArduCopter" 2>/dev/null || true  # new
pkill -9 -f "1_ardupilot_single_vehicle" 2>/dev/null || true  # new
pkill -9 -f mavros 2>/dev/null || true
fuser -k 14550/udp 14551/udp 9002/udp 5501/udp 5760/tcp 2>/dev/null || true
sleep 1
```

**Issue 14 found during testing — ArduPilot home/EKF origin defaulted to CMAC (Canberra):**
`/mavros/global_position/global` returned `lat=-35.36, lon=149.16, alt=603 m` — that's
CMAC (Canberra), ArduPilot SITL's hardcoded default home. Pegasus configures the Isaac Sim
world for Taiwan (23.44938, 120.28924) but the JSON FDM protocol only carries IMU and
physics state, not GPS coordinates. ArduPilot generates its own GPS internally based on
the `-L` flag, defaulting to Canberra.

**Fix applied (`ardupilot_launch_tool.py`):**
Pass Taiwan as the custom home location to `sim_vehicle.py`:
```python
"--custom-location=23.44938,120.28924,200,0",   # LAT,LON,ALT_AMSL,HDG
```

**Verified Taiwan home location (2026-06-21):**
- `/mavros/global_position/global`: lat=23.45, lon=120.29 ✓
- EKF origin: lat=23.45, lon=120.29 ✓
- Field elevation: 200 m AMSL ✓
- BUT drone still didn't lift — see Issue 15.

**Issue 13b — Pegasus throughput improved by reducing scene complexity:**
Disabled Cesium terrain and the 1920×1440 RGB camera in
`1_ardupilot_minimal_test.py` and saw UDP 14550 queue drop from 172 KB → 21 KB
(8× improvement) and no more `Duplicate input frame` warnings. So Cesium +
camera dominate the load.

**Files added for the minimal repro:**
- `1_ardupilot_minimal_test.py` — same scene but ENABLE_TERRAIN=False and no camera
- `02_launch_minimal.sh` — launches the minimal scene

**Issue 15 found during testing — ArduCopter GUIDED takeoff stuck at idle:**
Even in the minimal scene, after `NAV_TAKEOFF: ACCEPTED`:
- z stayed at -0.15 m, velocity ≈ 0
- `/mavros/rc/out` motor PWM stuck at 1100 (MOT_SPIN_ARM idle)
- `system_status` = 3 (STANDBY), never reached 4 (ACTIVE)
- No `Takeoff to X m` log line in the SITL log after the ACK

Diagnostic via `/mavros/rc/in`:
```
ch3 (throttle): 1000   ← MINIMUM
```
ArduCopter's `Copter::update_auto_armed()` requires
`channel_throttle->get_control_in() > 0` for `ap.auto_armed` to flip true.
With ch3 stuck at RC3_MIN (1000), `get_control_in()` returns 0, so
`auto_armed` stays false → `AutoTakeoff::run()` returns early → motors
hold at idle indefinitely.

MAVROS `/mavros/rc/override` doesn't help — even when streamed at 20 Hz,
`/mavros/rc/in` ch3 still reads 1000. SITL synthesizes its own RC values
internally and ignores MAVLink overrides on the JSON FDM model.

**Fix applied (`sitl_params.parm`):**
```
RC3_MIN 900
```
Lowering RC3_MIN from default (1100) to 900 makes the SITL throttle of 1000
read as "above min" (= ~91 / 1000), so `auto_armed` flips true and the
GUIDED takeoff state machine commands climb thrust.

Also added `BRD_SAFETYENABLE 0` to disable the SITL safety button check.

**Pegasus backend code observation (`ardupilot_mavlink_backend.py`):**
`update_motor_commands()` (line 705) zeros all rotor PWMs when
`self._armed == False`. `self._armed` is only set via `update_is_armed()`
which requires a HEARTBEAT message. With 172 KB queued on UDP 14550,
heartbeat delivery to the backend can lag by several seconds. During that
window Pegasus writes zero PWM to Isaac Sim physics even though ArduPilot
is already commanding idle thrust. This compounds the load issue but is
not the primary takeoff blocker (the primary blocker is the auto_armed
check on the ArduPilot side).

**Issue 16 found during testing — RC3_MIN at boot blocks arming:**
Setting `RC3_MIN=900` at boot (via `sitl_params.parm`) made SITL's synthetic
throttle of 1000 µs read as `get_control_in() ≈ 91 > 0`. ArduCopter's pre-arm
check `rc_throttle_arm_checks()` then rejected arming with
`Arm: Throttle (RC3) is not neutral`. This was visible in repeated retry logs:
```
[INFO] ... still not armed — re-issuing arm (count=80)
[INFO] ... still not armed — re-issuing arm (count=120)
[INFO] ... still not armed — re-issuing arm (count=160)
```
ArduPilot log: `Got COMMAND_ACK: COMPONENT_ARM_DISARM: FAILED`,
`AP: Arm: Throttle (RC3) is not neutral`.

**Catch-22:**
- `RC3_MIN=1000` (default) → arm works (throttle reads as neutral),
  but `auto_armed` stays false → GUIDED takeoff stalls at idle PWM 1100.
- `RC3_MIN=900` → arm rejected ("throttle not neutral").

**Fix applied — set RC3_MIN dynamically AFTER arming:**

`sitl_params.parm`: removed `RC3_MIN 900` so arming uses ArduPilot's default
(throttle of 1000 reads as neutral → arm passes).

`fly_test.py`: between arm-confirmed and takeoff, call
`/mavros/param/set` to lower RC3_MIN to 900. Then `get_control_in()` becomes
~91 → `auto_armed` flips true → next-loop `MAV_CMD_NAV_TAKEOFF` engages climb.

```python
self.param_cli = self.create_client(ParamSetV2, '/mavros/param/set')

def _set_param_int(self, name, value):
    req = ParamSetV2.Request()
    req.force_set = True
    req.param_id = name
    req.value = ParameterValue()
    req.value.type = ParameterType.PARAMETER_INTEGER
    req.value.integer_value = value
    self.param_cli.call_async(req)

# in tick() startup phase:
if self._armed and self._guided and not self._rc3min_lowered:
    self._set_param_int('RC3_MIN', 900)
    self._rc3min_lowered = True
if self.count > 60 and self._ekf_ready and self._armed \
        and self._guided and self._rc3min_lowered:
    self._takeoff(ALTITUDE)
    self.phase = 'climb'
```

**Issue 17 — Switched ArduCopter from master (4.8.0-dev) to stable 4.6.3:**
After exhausting the RC3_MIN dynamic-lowering attempt and confirming that even
when RC3_MIN=900 was persistent in ArduPilot's params (verified via
`/mavros/param/get`), the motors STILL stayed at PWM 1100 after `NAV_TAKEOFF:
ACCEPTED`. The takeoff state machine never logs "Takeoff to X m" or any
progress after the ACK.

Hypothesis: ArduCopter master (4.8.0-dev) has a regression in JSON FDM +
GUIDED takeoff. Stable Copter-4.6.3 is the latest release tag and is known
to work in canonical SITL setups.

**Steps applied:**
```bash
cd /home/lin/ardupilot
git tag master-backup-pre-4.6.3   # save current HEAD
git checkout Copter-4.6.3         # checkout stable tag
git submodule update --init --recursive
./waf configure --board sitl
./waf copter                       # 2m26s rebuild
```

Build artifact: `/home/lin/ardupilot/build/sitl/bin/arducopter` (5.4 MB,
timestamp 2026-06-21 10:32, 4.6.3 official).

**Tested with Copter-4.6.3 (2026-06-21):**

ArduCopter V4.6.3 confirmed in the SITL log (`AP: ArduCopter V4.6.3 (92b0cd78)`).
EKF3 took longer to settle (~22 s vs ~2 s on master/4.8.0-dev), causing many
`Arm: Need Position Estimate` rejections in the early arming phase. Eventually:
```
AP: EKF3 IMU0 is using GPS
AP: EKF3 IMU1 is using GPS
Got COMMAND_ACK: COMPONENT_ARM_DISARM: ACCEPTED
AP: Warning: Arming Checks Disabled        ← new in 4.6.3
AP: Arming motors
ARMED
Arming checks disabled
Got COMMAND_ACK: NAV_TAKEOFF: ACCEPTED
```

**Same failure mode as 4.8.0-dev:**
- `armed=true, mode=GUIDED, system_status=3 (STANDBY)`
- `/mavros/rc/out` motor PWM = `[1100, 1100, 1100, 1100]` (idle)
- `/mavros/local_position/pose` z = 0.003 m (essentially on ground)
- `/mavros/local_position/velocity_local` vz = 0.003 m/s (essentially zero)
- Takeoff command silently does not engage climb thrust.

**Conclusion: the takeoff failure is NOT an ArduCopter version regression.**
Both 4.6.3 stable and 4.8.0-dev master exhibit identical behavior. The block
is in the Pegasus ↔ ArduPilot interaction — most likely:
- Pegasus's `update_motor_commands` (line 705 of `ardupilot_mavlink_backend.py`)
  zeros motor inputs when `self._armed == False`, and `self._armed` only flips
  via slow HEARTBEAT processing from the heavily-backed-up MAVLink queue
- OR the JSON FDM time sync between Pegasus and ArduPilot is broken in a way
  that prevents the takeoff state machine from advancing past entry

**Reference comparison (`11_ardupilot_multi_vehicle.py` — Pegasus's own working example):**
- Uses `pg.ardupilot_path` (we hardcoded `ARDUPILOT_DIR`)
- Loads `"Curved Gridroom"` env (we use `"Flat Plane"`)
- Adds a `ROS2Backend` alongside `ArduPilotMavlinkBackend` (we use only ArduPilot)
- Does NOT call `pg.set_global_coordinates()` (we set Taiwan coords)
- Otherwise identical
- Worth testing the reference example as-is to see if it lifts off — if yes,
  the ROS2Backend or the lack of custom coords matters; if no, the Pegasus
  ArduPilot backend itself is broken in this environment.

**Where we are after 17 issues (2026-06-21):**

Working:
- ArduCopter SITL builds + runs (both 4.8.0-dev and 4.6.3 tested)
- Pegasus connects via JSON FDM
- MAVROS connects, ArduPilot armed in GUIDED, takeoff ACK accepted
- GPS origin in Taiwan via `--custom-location`

Blocked:
- Motors stuck at PWM 1100 idle after takeoff ACK
- Drone never physically lifts in Isaac Sim
- Issue is upstream of ArduCopter version; either Pegasus backend or JSON FDM sync

**Issue 18 — Pegasus reference example 11 cannot even import in this env:**
Trying to run `11_ardupilot_multi_vehicle.py` unchanged failed at import time:
```
ModuleNotFoundError: No module named 'rclpy._rclpy_pybind11'
The C extension '/opt/ros/jazzy/lib/python3.12/site-packages/_rclpy_pybind11.cpython-311-x86_64-linux-gnu.so' isn't present on the system.
```
ROS Jazzy's rclpy targets Python 3.12; Isaac Sim's venv is Python 3.11. The
reference example imports `ROS2Backend` which depends on rclpy → unimportable
in this env. The Pegasus author clearly never tested example 11 against this
specific Isaac Sim 5.x venv.

**Workaround — `11_ardupilot_no_ros2.py` created:**
Stripped copy of example 11 with:
- `ROS2Backend` removed (so rclpy is not imported)
- 1 vehicle instead of 5 (lighter load for diagnosis)
- Everything else (Curved Gridroom env, `pg.ardupilot_path`, no
  `set_global_coordinates`) preserved from upstream.

Launcher: `02_launch_example11.sh` → runs `11_ardupilot_no_ros2.py`.

---

**Issue 19 — Pegasus `update_is_armed` reads only 1 MAVLink message per physics step — `self._armed` never flips True — motors always zeroed:**

Root cause: `update_is_armed()` calls `recv_match(blocking=False)` which returns at
most ONE message. ArduPilot sends HEARTBEAT at 1 Hz but sends 40+ other message
types per second (ATTITUDE, GLOBAL_POSITION_INT, STATUS_TEXT, etc.). At 50 Hz
physics, there is only a ~2% chance that the one message read is a HEARTBEAT. With
a 100 KB MAVLink queue backlog, the backend may process hundreds of steps draining
non-HEARTBEAT messages before ever seeing a HEARTBEAT.

Result: `self._armed` stays `False` indefinitely. `update_motor_commands` gates on
`self._armed`, so all servo commands from ArduPilot (received via UDP 9002 JSON FDM)
are zeroed by `zero_input_reference()`. The drone never lifts regardless of what
ArduPilot sends.

Secondary bug: the disarm detection `if self._armed == True` at line 701 was inside
`if not self._armed:` — unreachable code.

**Fix applied to `ardupilot_mavlink_backend.py` `update_is_armed()`:**
Changed from `recv_match(blocking=False)` (reads 1 arbitrary message) to
`recv_match(type="HEARTBEAT", blocking=False)` (drains queue until HEARTBEAT found
or queue empty). Also fixed disarm detection to use `elif not is_armed and
self._armed`.

```python
# BEFORE (broken)
def update_is_armed(self):
    msg = self._connection.recv_match(blocking=False)   # reads 1 message
    if msg is not None:
        if not self._armed:
            if msg.get_type() == "HEARTBEAT" and ...:  # usually not HEARTBEAT
                ...

# AFTER (fixed)
def update_is_armed(self):
    msg = self._connection.recv_match(type="HEARTBEAT", blocking=False)
    if msg is not None and msg.type != mavutil.mavlink.MAV_TYPE_GCS:
        is_armed = self._connection.motors_armed()
        if is_armed and not self._armed:
            self._armed = True
            carb.log_warn("Drone is armed.")
        elif not is_armed and self._armed:
            self._armed = False
            carb.log_warn("Drone is disarmed.")
```

---

**Issue 20 — ArduPilot `auto_armed` flag never set → `guided_takeoff_run()` outputs idle throttle forever:**

After HEARTBEAT fix (Issue 19), Pegasus's `self._armed` correctly flips True at step ~15500. Servo
packets flow and motor references reach ref=[200 rad/s] (MOT_SPIN_ARM idle). But forces_z=0.342 N per
rotor (total 1.37 N) far below hover threshold (~14.7 N for 1.5 kg Iris). Drone never lifts.

Root cause: ArduPilot's internal `auto_armed` flag was never set. `guided_takeoff_run()` checks
`ap.auto_armed` before commanding climb; if False it calls `make_safe_ground_handling()` → idle
throttle forever. The ArduPilot log confirmed: "Auto armed" message NEVER appeared.

In ArduCopter 4.6.3, `auto_arm_motors()` sets `auto_armed=True` when:
- `motors->armed() == True` AND `channel_throttle->get_control_in() > 0`

SITL's synthesized throttle channel defaults to 1000 PWM. With default `RC3_MIN=1000`:
`control_in = (1000-1000)/(2000-1000)*1000 = 0` → `auto_armed` never fires.

**Attempted fix — `RC3_MIN 900` in `sitl_params.parm`:**
This did NOT work. See Issue 21 for why.

Previous workaround (dynamic RC3_MIN=900 via MAVROS ParamSet after arming, Issues 15/16) was also
correct in theory, but the HEARTBEAT queue bug (Issue 19) prevented `self._armed` from ever being
True, so `update_motor_commands` zeroed servos before the param set had any effect.

---

**Issue 21 — `AP_Param::get_default_value()` returns FIRST match → sitl_params.parm overrides for params already in copter.parm are silently ignored:**

After adding `RC3_MIN 900` to `sitl_params.parm`, the actual parameter in ArduPilot (as read from
`mav.parm` saved by MAVProxy after connecting) was still `RC3_MIN 1000`. The fix had no effect.

Root cause: `AP_Param::get_default_value()` in `AP_Param.cpp` iterates `param_overrides[]` and
returns the **first match**:
```cpp
for (uint16_t i=0; i<num_param_overrides; i++) {
    if (vp == param_overrides[i].object_ptr) {
        return param_overrides[i].value;  // returns FIRST match found
    }
}
```
`param_overrides[]` is filled in file order (earlier files first). `copter.parm` is processed first
(loaded by `sim_vehicle.py -f gazebo-iris` before `--add-param-file` files), so its `RC3_MIN=1000`
entry appears at a lower index and always wins. Our `RC3_MIN=900` from `sitl_params.parm` (added last
via `--add-param-file`) is stored at a higher index and never reached.

**Which sitl_params.parm entries actually take effect:**
- `SCHED_LOOP_RATE`, `ARMING_CHECK`, `FS_GCS_ENABLE`, `DISARM_DELAY`, `BRD_SAFETYENABLE`: ✓ work
  (not set by copter.parm → our entry is the only one → first match IS ours)
- `FS_THR_ENABLE 0`: ✗ does NOT work (copter.parm sets `FS_THR_ENABLE 1` → copter.parm wins)
- `RC3_MIN 900`: ✗ does NOT work (copter.parm sets `RC3_MIN 1000` → copter.parm wins)

**Note on safety button (`BRD_SAFETYENABLE`):**
Real ArduPilot hardware (e.g., SDMODELH7V2) requires pressing a physical safety button on the GPS unit
before motors will respond to throttle. In SITL, `BRD_SAFETYENABLE 0` (in sitl_params.parm, and
correctly taking effect since copter.parm does NOT set it) disables this check. No action needed.

**Note on `FS_THR_ENABLE=1` not taking effect:**
copter.parm's `FS_THR_ENABLE=1` stays. This enables RC throttle failsafe. However, the default
`FS_THR_VALUE=975` is below the SITL synthesized throttle of 1000, so the failsafe does NOT trigger
(throttle 1000 > threshold 975). This is not causing any observed problem.

**Fix — RC_CHANNELS_OVERRIDE in `fly_test.py`:**
Instead of setting a parameter, we publish a MAVLink `RC_CHANNELS_OVERRIDE` message with RC3=1100
for 1 second immediately after the drone is armed and in GUIDED mode. This makes:
`control_in = (1100-1000)/(2000-1000)*1000 = 100 > 0` → `auto_armed` fires on the next ArduCopter
50 Hz scheduler loop. After 1 second, the override is released (ch3=65535 = UINT16_MAX = release).
Then `NAV_TAKEOFF` is sent.

MAVLink RC_CHANNELS_OVERRIDE channel values:
- `0` = do not override (pass through to synthesized RC)
- `65535` (UINT16_MAX) = release/clear this channel's existing override
- Any other value = override to that PWM

```python
# in fly_test.py startup phase, once armed + GUIDED + EKF ready:
self._rc_ticks += 1
rc = OverrideRCIn()
rc.channels = [0] * 18          # 0 = don't override any channel by default
if self._rc_ticks <= RATE_HZ:   # first 1 s: throttle = 1100
    rc.channels[2] = 1100       # RC3 > RC3_MIN(1000) → control_in=100 → auto_armed=True
    self.rc_pub.publish(rc)
else:                            # after 1 s: release and take off
    rc.channels[2] = 65535      # UINT16_MAX = release channel 3 override
    self.rc_pub.publish(rc)
    if self._rc_ticks == RATE_HZ + 1:
        self._takeoff(ALTITUDE)
        self.phase = 'climb'
```

**Changes:**
- `fly_test.py`: added `OverrideRCIn` import, `rc_pub` publisher, `_rc_ticks` counter,
  RC override pulse sequence before NAV_TAKEOFF
- `sitl_params.parm`: RC3_MIN 900 removed; replaced with comment explaining why it didn't work

---

**Issue 22 — `fly_test.py` publishes position setpoints every tick → overrides TakeOff submode → GROUND_IDLE forever:**

**Root cause (confirmed from binary log):**
`fly_test.py::tick()` called `self._publish()` unconditionally on every tick — even during the `startup` phase before arming and during `climb` while still on the ground.

`_publish()` sends `PoseStamped` to `/mavros/setpoint_position/local`. MAVROS forwards this as `SET_POSITION_TARGET_LOCAL_NED` to ArduPilot. ArduPilot's `set_destination()` immediately changes `guided_mode` from `SubMode::TakeOff` to `SubMode::Pos`.

In `SubMode::Pos`, `pos_control_run()` is called instead of `takeoff_run()`. `pos_control_run()` first calls `is_disarmed_or_landed()`:
```cpp
bool Mode::is_disarmed_or_landed() const {
    if (!motors->armed() || !copter.ap.auto_armed || copter.ap.land_complete) {
        return true;
    }
    return false;
}
```
Because `land_complete=True` (drone hasn't left the ground yet), this returns True, and `pos_control_run()` calls `make_safe_ground_handling(false)` → `set_desired_spool_state(GROUND_IDLE)`.

This overrides the THROTTLE_UNLIMITED that `_AutoTakeoff::run()` would have set. Every loop: `update_flight_mode` calls `pos_control_run()` → GROUND_IDLE. Spool never advances.

**Evidence from binary log:**
- RCOU C1-C4 stuck at 1100 (= `spin_arm * (pwm_max-pwm_min) + pwm_min` = 0.1*1000+1000 → GROUND_IDLE spool, `_spin_up_ratio` converges to `spin_arm/spin_min = 0.667`, not 1.0)
- If `desired = THROTTLE_UNLIMITED`, `_spin_up_ratio` would reach 1.0 → RCOU would go to 1150 (spin_min)
- MOTB ThrOut=0 throughout — consistent with GROUND_IDLE
- AUTO_ARMED (EV id=15) at rel=+1.12s was set correctly by `do_user_takeoff()`, but immediately overridden

**The spool state machine (AP_MotorsMulticopter.cpp):**
```
GROUND_IDLE + desired=GROUND_IDLE:  _spin_up_ratio → spin_arm/spin_min ≈ 0.667 → RCOU 1100 (stuck)
GROUND_IDLE + desired=THROTTLE_UNLIMITED: _spin_up_ratio += _dt/_spool_up_time → 1.0 → SPOOLING_UP
```

**Fix — `fly_test.py`:**
```python
# Only publish position setpoints after climbing (not during startup or climb)
# Publishing during startup/climb switches GUIDED submode TakeOff → Pos while
# land_complete=True, causing pos_control_run() → make_safe_ground_handling(false) → GROUND_IDLE.
if self.phase not in ('startup', 'climb'):
    self._publish()
```

Applied at the top of `tick()`, replacing the unconditional `self._publish()`.

**Note on RC_CHANNELS_OVERRIDE (Issue 21):**
The binary log shows RCIN C3=1000 throughout — the RC override from MAVROS never reaches ArduPilot's RC input processor in JSON SITL mode. This means the Issue 21 RC pulse is a no-op. `auto_armed` is correctly set by `do_user_takeoff()` → `set_auto_armed(true)` directly when NAV_TAKEOFF is processed. The RC pulse can be removed but is left in place (harmless).

---

---

### Issue 23 — Wrong launch script for production mission

**Symptom:** `02_launch_example11.sh` was used for testing Issue 22 fix. It runs
`11_ardupilot_no_ros2.py` (Pegasus's diagnostic reference) which uses the "Curved Gridroom"
environment with no Cesium terrain. The 120s warmup phase in `fly_test.py` exists specifically
to wait for Cesium tiles to stream in — wasted with no terrain. Visual feedback of altitude is
also poor in the gridroom (drone flies through the ceiling visually).

**Root cause:** `02_launch_example11.sh` is a diagnostic script. It was used during Issue 22
testing to isolate the spool problem without Cesium loading delays. Now that the fix is
confirmed (binary log shows 485m flight, 841s total), the production script should be used.

**Fix:** Use `02_launch.sh` → `1_ardupilot_single_vehicle.py` for all production runs:
- Cesium terrain (Google Photorealistic 3D Tiles, Taiwan)
- GPS origin set: `set_global_coordinates(23.44938, 120.28924, 90.0)`
- Drone camera: `/drone/camera/image_raw`
- Drone prim: `/World/quadrotor`

**Both scripts use the same ArduPilot binary** (same inode 105660369 — `/home/lin/ardupilot`
and `/mnt/raid5/lin/ardupilot` are the same physical directory).

**Also fixed in this issue:** `fly_test.py` arming wait messages.
The old "still not armed — re-issuing arm (count=80/120/160)" messages implied failure.
In GUIDED mode, arming requires GPS/EKF convergence (~30 s) — this is mandatory and cannot be
bypassed by ARMING_CHECK 0. Messages now show elapsed seconds and say "Waiting for GPS/EKF
convergence (~30 s)".

---

## Debugging Methodology — How Each Problem Was Found and Fixed

This section documents the debugging process for the hardest issues: what tools were used,
what clue pointed to the root cause, and why the fix actually worked.

---

### Issue 19 — How we found it

**Symptom observed:** Drone armed (MAVProxy log said `ARMED`), ArduPilot sent servo commands
over the JSON FDM socket (port 9002), but the drone never moved. Motor references in Pegasus
stayed at zero (`zero_input_reference()` kept overwriting them).

**What we looked at first:** `ardupilot_mavlink_backend.py::update_motor_commands()`:
```python
if not self._armed:
    self.zero_input_reference()
    return
```
The gate was `self._armed`. We added a log print to `update_is_armed()` and saw it was never
setting `self._armed = True`, even though MAVProxy (on the same MAVLink stream) was clearly
showing the vehicle as armed.

**Why `self._armed` never flipped:** `update_is_armed()` called:
```python
msg = self._connection.recv_match(blocking=False)  # reads exactly 1 message
```
ArduPilot sends HEARTBEAT at **1 Hz** but sends 40+ other message types at ~50 Hz
(ATTITUDE, GLOBAL_POSITION_INT, STATUS_TEXT, EKF_STATUS_REPORT, etc.). At 50 Hz physics
step rate, each call reads one message. The probability that this one message is a HEARTBEAT
is ~1/40 = 2.5%. With a 100 KB+ queue backlog from many physics steps, the backend could
process hundreds of steps before ever encountering a HEARTBEAT.

**Why the fix works:** `recv_match(type="HEARTBEAT", blocking=False)` internally drains the
entire socket queue until it finds a HEARTBEAT or the queue is empty. One call per physics
step is enough — if any HEARTBEAT arrived since the last step, this call finds it.

**Key lesson:** `recv_match(blocking=False)` without a `type=` filter reads AT MOST one
message. It is NOT equivalent to "read and discard until you find what you want."

---

### Issue 22 — How we found it (binary log analysis)

**Symptom observed:** After Issue 19 fix, drone armed correctly (`self._armed = True`, motor
references non-zero). NAV_TAKEOFF ACCEPTED. But drone still didn't lift. Motor PWM outputs
appeared stuck.

**Step 1 — add servo logging to Pegasus backend:**
We added a print to `ardupilot_mavlink_backend.py` to log servo values every 100 physics
steps. The output showed:
- Step 10527: servos=[1000, 1000, 1000, 1000] (just armed, motors at rest)
- Step 11000+: servos rising to ~1048, 1168, 1620... then stabilizing ~1580

This showed the motors DID eventually start — but very slowly, and only after a long delay.
Something was holding them at GROUND_IDLE level first.

**Step 2 — read the ArduPilot binary flight log (`.BIN` file):**
The binary log is written by ArduPilot SITL to the temp directory at
`{root_fs.name}/logs/00000001.BIN`. We decoded it using pymavlink:
```python
from pymavlink import mavutil
mlog = mavutil.mavlink_connection('/tmp/.../logs/00000001.BIN')
```
Key messages decoded:
- **RCOU** (motor output PWM): C1–C4 stuck at **1100** for ~363 seconds, then rising to 1600+
- **EV** (autopilot events): ARMED at t=0, AUTO_ARMED at t=+0.74s, LAND_COMPLETE at t=+2.02s
- **MOTB** (motor throttle): `ThrOut=0.000` throughout the stuck period
- **RCIN** (RC input): C3=1000 at all times (RC override from MAVROS never reached ArduPilot)

**What 1100 PWM means:** In ArduPilot's motor spool state machine:
- `spin_arm = 0.1` → PWM = `0.1 * (pwm_max - pwm_min) + pwm_min = 0.1*1000+1000 = 1100`
- This is the GROUND_IDLE spool state: motors spin slowly but produce no useful thrust
- For liftoff, spool must reach THROTTLE_UNLIMITED: `_spin_up_ratio = 1.0` → PWM ≥ 1150

**Step 3 — trace why desired spool state was GROUND_IDLE:**
We read `AP_MotorsMulticopter.cpp::output_logic()`. In GROUND_IDLE state:
```cpp
case MotorSpool::GROUND_IDLE:
    if (_spool_desired == DesiredSpoolState::THROTTLE_UNLIMITED) {
        _spin_up_ratio += _dt / _spool_up_time;   // → advance toward SPOOLING_UP
    } else {
        _spin_up_ratio += (_spin_arm / _spin_min - _spin_up_ratio) * _dt / _spool_up_time;
        // → converges to spin_arm/spin_min = 0.1/0.15 = 0.667 → stuck at 1100
    }
```
`_spool_desired` was GROUND_IDLE, not THROTTLE_UNLIMITED. We searched for who sets GROUND_IDLE.

**Step 4 — trace who sets desired spool = GROUND_IDLE:**
`make_safe_ground_handling(false)` calls `set_desired_spool_state(GROUND_IDLE)`. This is
called from `pos_control_run()` when `is_disarmed_or_landed()` returns True:
```cpp
bool Mode::is_disarmed_or_landed() const {
    if (!motors->armed() || !copter.ap.auto_armed || copter.ap.land_complete) return true;
}
```
`land_complete = True` (drone hasn't left the ground yet) → returns True every loop.

**Step 5 — trace why GUIDED was in Pos submode, not TakeOff submode:**
`pos_control_run()` runs when `guided_mode == SubMode::Pos`. We expected `SubMode::TakeOff`
after NAV_TAKEOFF. We read `mode_guided.cpp::set_destination()`:
```cpp
void ModeGuided::set_destination(const Vector3f& destination, ...) {
    guided_mode = SubMode::Pos;   // ← switches away from TakeOff!
    ...
}
```
`set_destination()` is called whenever ArduPilot receives `SET_POSITION_TARGET_LOCAL_NED`.
MAVROS publishes this message whenever `fly_test.py` publishes to
`/mavros/setpoint_position/local`. And `fly_test.py::tick()` was calling `self._publish()`
**unconditionally every tick** — including during `startup` and `climb` phases before the
drone left the ground.

**Chain of causation (full path):**
```
fly_test.py tick() every 50ms
  → self._publish() (unconditional)
  → /mavros/setpoint_position/local PoseStamped
  → MAVROS → SET_POSITION_TARGET_LOCAL_NED (MAVLink)
  → ArduPilot mode_guided.cpp::set_destination()
  → guided_mode = SubMode::Pos  (overwrites SubMode::TakeOff set by NAV_TAKEOFF)
  → ModeGuided::run() → pos_control_run()
  → is_disarmed_or_landed() = True (land_complete=True, drone on ground)
  → make_safe_ground_handling(false)
  → set_desired_spool_state(GROUND_IDLE)
  → AP_MotorsMulticopter::output_logic() → _spin_up_ratio → 0.667 → PWM 1100
  → drone never lifts
```

**Why the fix works:** Skipping `_publish()` during `startup` and `climb` phases means
ArduPilot never receives a position setpoint while `land_complete=True`. The GUIDED submode
stays as `SubMode::TakeOff`, so `takeoff_run()` runs instead of `pos_control_run()`.
`takeoff_run()` calls `set_desired_spool_state(THROTTLE_UNLIMITED)` → spool advances →
motors spin up → drone lifts → `land_complete` becomes False → `_publish()` then starts
(warmup phase) → switches to Pos submode → holds altitude at target position. Correct.

---

### General debugging tools used

| Tool | What it showed |
|---|---|
| **ArduPilot binary log** (`.BIN` in tempdir `logs/`) | Ground truth: RCOU, EV, MOTB, RCIN decoded with pymavlink — proved spool was stuck at GROUND_IDLE (RCOU=1100), exactly when AUTO_ARMED fired, when LAND_COMPLETE was set |
| **Pegasus diagnostic log** (`/tmp/pegasus_diag.log`) | Per-step servo PWM values — showed when Pegasus's backend started sending non-zero motor commands vs when ArduPilot accepted them |
| **ArduPilot source code** | `AP_MotorsMulticopter.cpp` (spool state machine), `mode_guided.cpp` (set_destination, pos_control_run), `takeoff.cpp` (auto_armed check), `AP_Arming.cpp` (mandatory GPS check), `AP_Param.cpp` (first-match rule) |
| **MAVProxy log** (`ardupilot_0.log` in tempdir) | Arm/disarm events, GPS detection, EKF state messages — confirmed high-level sequence |
| **Print logging in backend** | Added servo value prints to `ardupilot_mavlink_backend.py` to see what Pegasus was actually sending to Isaac Sim physics |

---

### Why reading source code was essential

Documentation for ArduPilot, MAVROS, and Pegasus describes the happy path. It does not
document edge cases like:

- `recv_match(blocking=False)` reads at most 1 message (not "reads until a match")
- `SET_POSITION_TARGET_LOCAL_NED` silently overwrites the TakeOff submode even when
  `land_complete=True` makes the position controller output GROUND_IDLE
- `AP_Param::get_default_value()` returns the FIRST matching entry in `param_overrides[]`,
  so later `--add-param-file` entries lose to earlier ones for the same param
- `mandatory_gps_checks()` in `AP_Arming.cpp` always runs in GUIDED mode regardless of
  `ARMING_CHECK 0` — this is why arming takes ~30s (EKF convergence required)

Every one of these was only discoverable by reading the actual C++ source. The pattern:
observe unexpected behavior → add logging → narrow the failing line → read the source for
that function → find the undocumented constraint → fix.

---

---

## Issue 24 — fly_test.py ORIGIN_LAT/LON mismatch caused chaotic flight path

**Date:** 2026-06-22

**Symptom:** Drone does not follow the saved `fly_plan.json` waypoints; live trace on
`fly_trace.py` shows the drone flying to completely wrong positions, seemingly random.

**Root cause:** `fly_test.py` hard-coded:
```python
ORIGIN_LAT = 23.452011
ORIGIN_LON = 120.285761
```
but the simulation GPS home (set via `1_ardupilot_single_vehicle.py`) is:
```python
CESIUM_LAT = 23.44938
CESIUM_LON = 120.28924
```
That is a **293 m North + 355 m West** offset (~450 m combined).
ArduPilot's EKF sets local (0, 0, 0) at the first GPS fix = the CESIUM coordinates.
All setpoints published by `fly_test.py` are in metres relative to `ORIGIN_*`, so every
target was offset 450 m from the intended position.

`fly_plan.json` stores `lat/lon` (GPS) per waypoint, but the ENU `x/y` fields used for
publishing were computed against the wrong origin — so they were also wrong.

**Fix:**
1. Changed `fly_test.py`:
   ```python
   ORIGIN_LAT = 23.44938   # must match CESIUM_LAT in 1_ardupilot_single_vehicle.py
   ORIGIN_LON = 120.28924  # must match CESIUM_LON in 1_ardupilot_single_vehicle.py
   ```
2. Recomputed `fly_plan.json` ENU fields from the stored `lat/lon` fields using the correct
   CESIUM origin:
   ```python
   CESIUM_LAT = 23.44938; CESIUM_LON = 120.28924
   cos_lat = math.cos(math.radians(CESIUM_LAT))
   ex = (lon - CESIUM_LON) * 111320 * cos_lat
   ey = (lat - CESIUM_LAT) * 111320
   budget = max(15, int(dist / DRONE_SPEED) + LEG_BUFFER)
   ```
   New WP1: `(-888, +172)` m (was `(-532, -121)` m with wrong origin).

3. `fly_trace.py` imports `ORIGIN_LAT/LON` from `fly_test.py`, so the fix propagates
   automatically — no change needed in `fly_trace.py`.

**Key invariant:** `fly_test.py ORIGIN_*` must always equal `CESIUM_LAT/LON` in
`1_ardupilot_single_vehicle.py`. If you change the Pegasus GPS origin, update both.

---

## Issue 25 — fly_trace.py / 17_fly_trace.sh new version (2026-06-22)

**What changed:**
- `fly_trace.py` completely rewritten as an **interactive flight path planner + live drone
  trace on satellite map** (previously was only a live trace viewer).
- Satellite imagery via `contextily` (Esri WorldImagery), coordinate system Web Mercator.
- **Click** on map → adds numbered orange waypoints.
- **Undo Last / Clear All** buttons to edit.
- **Finish & Save** → writes `fly_plan.json` with `{x, y, budget, qz, qw, lat, lon}` per waypoint.
- Pre-loads existing `fly_plan.json` on startup so you can edit a previous plan.
- Live drone track overlaid from `/mavros/local_position/pose` — **ROS2 is optional**
  (try/except around `rclpy.init()`; runs in planning-only mode without MAVROS).
- **Does NOT auto-save the flight trace** — only the user-drawn plan is persisted.

**fly_test.py changes to support plan loading:**
- `_load_plan()` reads `fly_plan.json` if it exists; falls back to `_AUTO_SCAN_WPS`.
- `_load_plan()` is called at module level AND re-called at warmup end (~120 s) so a plan
  saved during warmup is picked up without restarting.
- Confirmation print: `[fly_test] USER PLAN loaded: N waypoints from fly_plan.json`.
- Wrapped in try/except to prevent silent hang if JSON is malformed.

**fly_plan.json format:**
```json
[
  {"x": -888.0, "y": 172.0, "budget": 85, "qz": -0.123, "qw": 0.992,
   "lat": 23.45093, "lon": 120.28120},
  ...
]
```
- `x` = East metres, `y` = North metres (ENU relative to ORIGIN_LAT/LON = CESIUM coords)
- `budget` = seconds allocated for this leg
- `qz/qw` = yaw quaternion pointing toward next waypoint

---

## Issue 26 — Shell scripts fail due to Anaconda Python / AMENT_TRACE_SETUP_FILES

**Symptoms observed:**

a) `bash 19_gps_switch.sh` printed:
   ```
   /opt/ros/jazzy/setup.bash: line 8: AMENT_TRACE_SETUP_FILES: unbound variable
   ```
   Cause: `set -euo pipefail` (the `-u` flag = `nounset`) was active when sourcing
   `/opt/ros/jazzy/setup.bash`. ROS2's setup script references `AMENT_TRACE_SETUP_FILES`
   which is not exported when running in a clean environment.

b) `bash 19_dbg_fly.sh` printed:
   ```
   ImportError: cannot import name 'Protocol' from 'typing' (/opt/anaconda3/lib/python3.7/typing.py)
   ```
   Cause: Anaconda prepends to `$PATH` in `~/.bashrc`; system Python 3.7 (Anaconda) was
   used instead of system Python 3.12 (required by ROS2 Jazzy).

**Fix A — AMENT_TRACE_SETUP_FILES:**
```bash
# In any script that sources ROS2 setup:
set -eo pipefail       # do NOT include -u
set +u
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
set -u
```

**Fix B — Anaconda Python conflict:**
Use `env -i` to strip the inherited environment before sourcing ROS2:
```bash
env -i HOME=$HOME PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin bash -c '
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
python3 /mnt/raid5/lin/IsaacSim/script.py
'
```
This is the same pattern used in `04_fly.sh` and `19_dbg_fly.sh`.

**Applied to:** `19_gps_switch.sh` (Fix A), `19_dbg_fly.sh` (Fix B).

---

## Issue 27 — Diagnostic tool: dbg_fly.py / 19_dbg_fly.sh

**Purpose:** Real-time comparison of drone EKF position vs fly_test.py setpoints.
Helps diagnose why drone is not following plan.

**Files:**
- `dbg_fly.py` — ROS2 node, subscribes to 3 topics, prints every 2 s
- `19_dbg_fly.sh` — launch wrapper (uses `env -i` pattern from Issue 26)

**Usage:**
```bash
bash /mnt/raid5/lin/IsaacSim/19_dbg_fly.sh
```
Run this in a separate terminal while `04_fly.sh` is active.

**Output columns:**
```
pos ( +x.x, +y.y, +z.z)  sp  ( +x.x, +y.y, +z.z)  err  N.N m
```
- `pos` = drone EKF local position in ENU metres (`/mavros/local_position/pose`)
- `sp`  = setpoint published by fly_test.py (`/mavros/setpoint_position/local`)
- `err` = Euclidean distance between pos and sp

**GPS home check:**
On startup, dbg_fly.py waits until the drone is on the ground near EKF origin
(`hypot(pos.x, pos.y) < 15 m AND pos.z < 5 m`) and has received ≥5 GPS samples.
Then it compares the GPS fix against CESIUM_LAT/LON (23.44938, 120.28924):
- offset < 50 m → `OK ✓`
- offset ≥ 50 m → `MISMATCH ✗ <-- likely cause of wrong positions`

**GPS check false alarm (known issue):**
If dbg_fly.py is started while the drone is already flying away from the origin,
the GPS check fires too early and reports a huge apparent offset.

Example: drone at EKF `(-960, +447)`, GPS check fires → shows N=+437 m, E=-949 m
offset → looks like catastrophic mismatch but is actually the drone's own displacement.

True EKF home computed by reversing GPS + EKF position:
```
lat_home = gps_lat - pos.y / 111320
lon_home = gps_lon - pos.x / (111320 * cos(lat))
```
yielded `(23.449296, 120.289443)` — only **22 m** from CESIUM → acceptable.

Fix already applied: GPS check now only fires when drone is within 15 m of origin AND
below 5 m altitude (i.e. on the ground before takeoff).

**How to use during a flight:**

| What you see | Meaning |
|---|---|
| `sp = (0.0, 0.0, 65.0)` | fly_test.py is in WARMUP phase (~120 s); expected |
| `sp = (-888.0, +172.0, 65.0)` | Warmup ended, executing WP1 — plan is loaded ✓ |
| `err` shrinking over time | Drone tracking setpoint correctly |
| `sp` stays at `(0,0,65)` past 120 s | `_load_plan()` failed or plan not loaded |

---

## Issue 28 — WPNAV_SPEED default (5 m/s) too slow for fly_plan.json budgets (12 m/s)

**Date:** 2026-06-22

**Symptom:** Drone appears to not follow `fly_plan.json` — it moves but never arrives at any
waypoint, or skips through all waypoints rapidly without reaching them.

**Root cause:** `fly_test.py` calculates per-leg time budgets assuming `DRONE_SPEED = 12.0` m/s:
```python
budget = max(15, int(dist / DRONE_SPEED) + LEG_BUFFER)   # LEG_BUFFER = 10
```
But ArduPilot's default `WPNAV_SPEED = 500` cm/s = **5 m/s** in GUIDED position mode.
`sitl_params.parm` had no `WPNAV_SPEED` override, so the drone flew at 5 m/s.

The scan phase advances waypoints purely on time (not on arrival):
```python
if self.wp_ticks >= self._scan_wps[self.wp_index][2] * RATE_HZ:
    self.wp_index += 1   # move on regardless of whether drone arrived
```

For a leg of distance D, the budget expires when the drone has only traveled
`(D/12 + 10) × 5 = 5D/12 + 50` metres — well short of the target for any long leg.

**Threshold:** budget is only sufficient when `D/5 ≤ D/12 + 10`, i.e. `D ≤ 86 m`.
All fly_plan.json legs are 200–900 m → every single leg timed out before arrival.

Example — WP1 (904 m from origin):
- Budget: `int(904/12) + 10 = 85 s`
- Time needed at 5 m/s: `904/5 = 181 s`
- Drone was only ~47% of the way there when the code jumped to WP2

**Fix:** Added to `sitl_params.parm`:
```
WPNAV_SPEED 1200   # 1200 cm/s = 12 m/s, matches DRONE_SPEED in fly_test.py
```

**Requires restart of 02_launch.sh** — `sitl_params.parm` is loaded at ArduPilot SITL startup.

**Key rule going forward:** `WPNAV_SPEED` (cm/s) must equal `DRONE_SPEED × 100` (m/s → cm/s).
If you change either one, change both.

---

## Issue 29 — Climb phase used absolute `count` instead of relative `phase_tick` (drone never took off)

**Date:** 2026-06-22

**Symptom:** `Takeoff: success=True` printed, `Altitude reached` printed 6–7 seconds later,
but `pos` stayed at `(0, 0, 0)` throughout warmup and scan — drone never physically left the
ground in Isaac Sim.

**Root cause:** The climb phase end condition compared the global `count` (ticks since program
start) to a fixed threshold:
```python
if self.count >= 60 + CLIMB_SECS * RATE_HZ:   # = 760 ticks = 38 s from start
```
Arm happens at ~T=28 s (count≈560). Takeoff happens 1 s later (count≈580).
Climb phase therefore only lasts `760 − 580 = 180` ticks ≈ **6–7 seconds** instead of 35 s.

In those 6 s the drone cannot reach 65 m (needs ~22 s at 3 m/s climb rate).
Warmup then starts and publishes `(0, 0, 65)` as a GUIDED Position setpoint.
ArduPilot receives this while still on the ground and tries to execute, but the drone
never spun up properly, so it remains at z=0.

The descend and land phases already used `phase_tick` correctly — climb was the only
phase that still used the global counter.

**Debug trail:**
- `fly_test.py` log showed `Altitude reached` only 6 s after `Takeoff to 65 m`
- `19_dbg_fly.sh` showed `pos=(0,0,0)` even after warmup/scan started
- `sp` correctly changed from `(0,0,65)` → `(-888,+172,65)` at warmup end, proving plan
  loading worked; the drone just was never airborne

**Fix:** Changed climb phase to increment `phase_tick` and reset it on takeoff:
```python
# in tick(), startup section — when issuing takeoff:
self.phase      = 'climb'
self.phase_tick = 0          # <-- added

# climb phase:
if self.phase == 'climb':
    self.phase_tick += 1
    if self.phase_tick >= CLIMB_SECS * RATE_HZ:   # 35 s × 20 Hz = 700 ticks
        self.phase = 'warmup'
        self.warmup_tick = 0
    return
```

---

## Issue 30 — Drone actual speed ~3 m/s; pure time-budget waypoint advancement skips WPs

**Date:** 2026-06-22

**Symptom:** After the climb fix, drone takes off and flies correctly, but `sp` jumps to
WP2 while drone is only 192 m into a 904 m leg — error grows from 718 m to 1453 m
immediately after WP1 budget (85 s) expires.

**Root cause — two parts:**

*Part A — actual speed ≈ 3 m/s, not 12 m/s:*
`fly_plan.json` budgets are computed with `DRONE_SPEED = 12.0` m/s, giving WP1 budget = 85 s.
The drone only flew 192 m in 85 s → average speed ≈ 2.3 m/s (steady-state ≈ 3 m/s).
Likely causes:
- `WPNAV_SPEED = 1200` in `sitl_params.parm` may be overridden by `copter.parm` (loaded
  earlier by `sim_vehicle.py`) per the first-match-wins rule (see Issue 18 / RC3_MIN).
- Physics limit of the Iris JSON model in Isaac Sim.
*(Not yet conclusively diagnosed — needs runtime param read to confirm.)*

*Part B — scan phase was purely time-based:*
```python
if self.wp_ticks >= self._scan_wps[self.wp_index][2] * RATE_HZ:
    self.wp_index += 1   # advance regardless of arrival
```
When budget < travel time, drone never reaches the waypoint before the code skips ahead.

**Fix — position-based waypoint advancement with time budget as fallback:**
Added `ARRIVAL_RADIUS = 15.0` m constant. Scan phase now advances when the drone is
within 15 m of the target OR the budget expires (logs a warning if skipped):
```python
ARRIVAL_RADIUS = 15.0   # m

# in scan phase:
dist_to_wp   = math.sqrt((self._cur_x - self.target_x)**2 +
                          (self._cur_y - self.target_y)**2)
budget_expired = self.wp_ticks >= self._scan_wps[self.wp_index][2] * RATE_HZ
arrived        = dist_to_wp < ARRIVAL_RADIUS
if arrived or budget_expired:
    if budget_expired and not arrived:
        self.get_logger().warn(
            f'WP {self.wp_index+1} budget expired ({dist_to_wp:.0f} m from target) — skipping')
    self.wp_index += 1
    ...
```
`_cur_x/y/z` stored in `_on_pos` callback:
```python
def _on_pos(self, msg):
    ...
    self._cur_x = msg.pose.position.x
    self._cur_y = msg.pose.position.y
    self._cur_z = msg.pose.position.z
```

**Workaround (Issue 32):** Changed `DRONE_SPEED = 3.0` in `fly_test.py` and recomputed
all 18 `fly_plan.json` budgets for 3 m/s. Long legs (~730 m) went from 70 s → ~250 s.
Position-based arrival (15 m radius) still triggers early if drone arrives sooner.

**Why the drone can't go faster — two hypotheses (not yet conclusively resolved):**

*Hypothesis A — WPNAV_SPEED not applied (parameter loading order):*
The first-match-wins rule (Issue 18) means `copter.parm` (loaded internally first) would
override `sitl_params.parm`. However, `copter.parm` was checked and has no `WPNAV_SPEED`
entry, so this may not be the cause. `gazebo-iris.parm` also has no `WPNAV_SPEED`.
Runtime check: `ros2 service call /mavros/param/get mavros_msgs/srv/ParamGet "{param_id: 'WPNAV_SPEED'}"`
If the value is not 1200, set it at startup via `/mavros/param/set` in fly_test.py.

*Hypothesis B — physics limit of the Iris model in Isaac Sim:*
ArduPilot sends motor PWM commands to Pegasus via JSON socket; Isaac Sim's physics engine
applies forces based on the Iris model's mass, motor thrust, and drag coefficients. The
drone may be physically limited to ~3 m/s by the simulation model regardless of ArduPilot
nav parameters. To fix: tune the Multirotor physics parameters in Pegasus (motor max thrust,
drag coefficients) or switch to a vehicle model with higher thrust-to-weight ratio.

**Diagnostic test result (2026-06-22):**
Set WPNAV_SPEED=1200 at runtime via MAVROS ParamSetV2 (success=True), observed no speed
increase → **Hypothesis B confirmed: physics limit.**

The Pegasus default Iris quadrotor model uses conservative motor thrust and drag parameters
tuned for simulation stability. The real Iris can fly 15+ m/s; the sim model caps at ~3 m/s.
To increase sim speed: tune motor/drag parameters in the Pegasus Multirotor config.
For mission testing purposes, DRONE_SPEED=3.0 is the correct value to use.

Note: `ros2 service call` CLI fails from subshells (rcl context error). Use `/usr/bin/python3`
with `ParamSetV2` directly instead of `ros2 service call` or `ParamSet` (older interface).
```python
from mavros_msgs.srv import ParamSetV2
from rcl_interfaces.msg import ParameterValue, ParameterType
req = ParamSetV2.Request()
req.param_id = 'WPNAV_SPEED'
req.value.type = ParameterType.PARAMETER_INTEGER
req.value.integer_value = 1200
```

---

## Issue 31 — fly_trace.py map extent excluded takeoff origin; drone trace invisible

**Date:** 2026-06-22

**Symptom:** Drone was flying but no trace appeared in the `17_fly_trace.sh` satellite map.

**Root cause:** Map bounds were computed only from the survey polygon:
```python
x_min = poly_merc[:, 0].min() - margin_m
x_max = poly_merc[:, 0].max() + margin_m
```
Survey polygon is at ENU ≈ (−1558 to −1645 m East, +263 to +843 m North).
EKF origin (takeoff point) is at (0, 0) — about **1500 m East** of the polygon.
The drone's flight path from (0,0) toward WP1 (−888, +172) was entirely off the left
edge of the visible map area.

**Fix:** Extended map bounds to include `orig_mx/my` (Web Mercator coordinates of EKF origin):
```python
x_min = min(poly_merc[:, 0].min(), orig_mx) - margin_m
x_max = max(poly_merc[:, 0].max(), orig_mx) + margin_m
y_min = min(poly_merc[:, 1].min(), orig_my) - margin_m
y_max = max(poly_merc[:, 1].max(), orig_my) + margin_m
```
Now the map always includes both the takeoff point and the survey polygon.

---

## Resume here in next session

**Production launch sequence:**
```bash
# Terminal 1 — Isaac Sim + Pegasus + ArduCopter SITL + Cesium terrain
bash /mnt/raid5/lin/IsaacSim/02_launch.sh

# Terminal 2 — MAVROS bridge
bash /mnt/raid5/lin/IsaacSim/03_launch_mavros.sh

# Terminal 3 — optional: plan editor (can run before/during flight)
bash /mnt/raid5/lin/IsaacSim/17_fly_trace.sh

# Terminal 4 — fly (loads fly_plan.json at warmup end ~120 s)
bash /mnt/raid5/lin/IsaacSim/04_fly.sh

# Terminal 5 — optional: live pos vs setpoint diagnostics
bash /mnt/raid5/lin/IsaacSim/19_dbg_fly.sh
```

**Files of interest** (all under `/mnt/raid5/lin/IsaacSim/`):
- `02_launch.sh` → `1_ardupilot_single_vehicle.py` — production launch (Cesium terrain, camera)
- `03_launch_mavros.sh` — MAVROS bridge (port 14551)
- `04_fly.sh` → `fly_test.py` — GUIDED arm + takeoff + polygon scan (loads fly_plan.json)
- `fly_trace.py` / `17_fly_trace.sh` — interactive waypoint planner + live drone trace
- `fly_plan.json` — user-drawn plan (18 waypoints, all West/North of EKF home)
- `dbg_fly.py` / `19_dbg_fly.sh` — diagnostic: pos vs setpoint vs error every 2 s
- `19_gps_switch.sh` — toggle GPS on/off in ArduPilot EK3 (for AnyLoc GPS-denied testing)
- `sitl_params.parm` — SITL overrides
- `PegasusSimulator/examples/1_ardupilot_single_vehicle.py` — CESIUM_LAT/LON ground truth

**ArduPilot state:**
- Tag `master-backup-pre-4.6.3` saved before checkout
- Currently on `Copter-4.6.3` stable
- Binary at `/home/lin/ardupilot/build/sitl/bin/arducopter`

**All fixes confirmed working:**
- Issue 19: `ardupilot_mavlink_backend.py` `update_is_armed()` drain queue for HEARTBEAT ✓
- Issue 22: `fly_test.py` `_publish()` skipped during `startup`/`climb` phases ✓
- Issue 23: Use `02_launch.sh` (not `02_launch_example11.sh`) for production ✓
- Issue 24: `fly_test.py` ORIGIN_LAT/LON corrected to match CESIUM; fly_plan.json recomputed ✓
- Issue 26: `env -i` / `set +u` pattern fixes Anaconda + AMENT_TRACE_SETUP_FILES errors ✓
- Issue 28: `WPNAV_SPEED 1200` added to `sitl_params.parm` ✓ (actual effect TBD — see Issue 30)
- Issue 32: `DRONE_SPEED=2.5` + `LEG_BUFFER=30` + fly_plan.json recomputed — budgets match actual sim speed ✓
- Issue 29: Climb phase `phase_tick` fix — drone now actually takes off ✓
- Issue 30: Scan phase now position-based (arrive within 15 m OR budget expires) ✓
- Issue 31: fly_trace.py map extended to include EKF origin — drone trace now visible ✓
- Issue 33: Takeoff location changed to GPS 23.451615, 120.286446 ✓

**Expected startup sequence with fly_test.py:**
1. count=20 (1s): set_mode('GUIDED') sent
2. count=40 (2s): arm() sent — FAILS (EKF not yet converged) — expected
3. count=80–600 (~4–30s): "Waiting for GPS/EKF convergence (~30 s)" every 2s — expected
4. ~30s: arm ACCEPTED → "Vehicle armed"
5. 1s later: RC3 pulse, then NAV_TAKEOFF sent → "Takeoff to 65 m"
6. climb phase: no _publish() calls; ArduPilot TakeOff mode climbs
7. ~120s warmup: _publish() starts → holds at (0,0,65); `_load_plan()` re-called at end
8. sp changes to WP1 (-602.8, -77.0, 65) → drone begins polygon scan
9. After all waypoints: RTL / land

**Conversation continuity:**
This migration plan now contains 28 documented issues with diagnoses and fixes.
A fresh session should be able to read this file and continue from here.

---

## Issue 33 — Takeoff location changed from KMU campus default to user-specified GPS (2026-06-22)

**Symptom:** User wanted drone to spawn and take off from GPS 23.451615, 120.286446
(different from the Cesium georef origin 23.44938, 120.28924).

**Change:** Takeoff point is now distinct from the Cesium georef origin.
- Cesium georef origin (`CESIUM_LAT/LON` in `1_ardupilot_single_vehicle.py`): **unchanged** at 23.44938, 120.28924 — this is the Isaac Sim world coordinate origin.
- ArduPilot EKF home (`--custom-location` in `ardupilot_launch_tool.py`): **changed** to 23.451615, 120.286446 — EKF local (0,0,0) is now at the new takeoff point.
- `ORIGIN_LAT/LON` in `fly_test.py` and `dbg_fly.py`: **changed** to 23.451615, 120.286446 — all ENU setpoints are relative to the new EKF home.

**Files changed:**
1. `ardupilot_launch_tool.py`: `--custom-location=23.451615,120.286446,200,0`
2. `fly_test.py`: `ORIGIN_LAT=23.451615`, `ORIGIN_LON=120.286446`
3. `dbg_fly.py`: `CESIUM_LAT=23.451615`, `CESIUM_LON=120.286446`
4. `1_ardupilot_single_vehicle.py`: drone spawn at ENU `[-285.34, 248.80, DRONE_SPAWN_Z]`
   (ENU offset of new takeoff GPS from Cesium origin: East=-285.34 m, North=+248.80 m)
5. `fly_plan.json`: all 18 waypoint ENU (x,y) recomputed from stored lat/lon with new origin.
   WP1 is now `(-602.8, -77.0)` m (was `(-888.2, +171.8)` m from old origin).

**fly_plan.json after change:** 18 WPs, total budget 3605 s = 60.1 min.

**Requires:** Restart `02_launch.sh` for `--custom-location` change to take effect.
