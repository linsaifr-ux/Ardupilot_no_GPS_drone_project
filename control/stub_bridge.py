#!/usr/bin/env python3
"""
DEPRECATED — replaced by control/drone_sim.py.

drone_sim.py is a full 6-DOF ROS2 node that replaces this stub:
  - Full roll/pitch/yaw kinematics (not just vertical)
  - Publishes /drone/state so cesium_scene.py can visualise without physics coupling
  - Use drone_sim.py for all new work

This file is kept for reference only.  It will be removed in a future cleanup.

──────────────────────────────────────────────────────────────────────────────
Minimal SITL bridge stub — keeps ArduPilot SITL alive without Isaac Sim.

Simulates a kinematic drone model driven by ArduPilot's PWM outputs:
  - Altitude: total thrust from avg motor PWM, integrated to altitude
  - Horizontal: fixed at scene origin (drone hovers in place)

This lets ArduPilot arm, take off, and hold altitude without Isaac Sim.
Horizontal waypoint flight requires the real Isaac Sim loop.

Usage:
  # Terminal 1 — SITL
  python3 third_party/ardupilot/Tools/autotest/sim_vehicle.py \
      -v ArduCopter --model=JSON --no-rebuild --console --map \
      -l 23.450868,120.286135,28.17,0 \
      --add-param-file=control/no_gps.parm --wipe \
      --out tcp:localhost:5763

  # Terminal 2 — stub (kinematic drone)
  python3 control/stub_bridge.py

  # Terminal 3 — vision (maintains EKF POS_ABS)
  python3 control/run_vision.py

  # Terminal 4 — flight commands
  python3 control/run_flight.py
"""

import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from control.sitl_bridge import SITLBridge

_HOME_ELEV    = 28.17  # metres MSL — matches -l 23.450868,120.286135,28.17,0
_GRAVITY      = 9.81   # m/s²
_MAX_TILT_ACC = 5.0    # m/s² horizontal acceleration cap (attitude-derived)
_MAX_VEL      = 10.0   # m/s velocity clamp

try:
    bridge = SITLBridge(centre_elev=_HOME_ELEV)
except OSError as e:
    if e.errno == 98:   # EADDRINUSE
        import subprocess
        try:
            who = subprocess.check_output(
                ["fuser", "9002/udp"], stderr=subprocess.DEVNULL
            ).decode().strip()
        except Exception:
            who = "unknown"
        print(f"[stub] ERROR: UDP port 9002 is already in use (pid {who}).")
        print("[stub] Stop Isaac Sim (cesium_scene.py) first.")
        sys.exit(1)
    raise

bridge.debug_hz = 1.0   # print physics state once per second

# ── Kinematic state ────────────────────────────────────────────────────────────
_x_enu = 0.0           # east  of home (m)
_y_enu = 0.0           # north of home (m)
_z_abs = _HOME_ELEV    # altitude MSL  (m) — starts on ground
_vn    = 0.0           # velocity north (m/s)
_ve    = 0.0           # velocity east  (m/s)
_vd    = 0.0           # velocity NED down (m/s); negative = ascending
_prev_t = time.time()

print("[stub] Kinematic drone on ground. Ctrl-C to stop.")
print(f"[stub] Home: AGL=0, MSL={_HOME_ELEV:.0f} m")

try:
    while True:
        t = time.time()
        dt = min(t - _prev_t, 0.05)   # cap at 50 ms to avoid large jumps
        _prev_t = t

        # ── Step bridge: sends physics state, receives servo PWM ──────────────
        servos = bridge.step(_x_enu, _y_enu, _z_abs, 0.0, wall_time=t)

        # ── Update kinematics from PWM ────────────────────────────────────────
        if servos is not None and dt > 0:
            pwm = servos["pwm"]

            # Total thrust from 4 main motors (indices 0-3)
            mean_pwm = sum(pwm[:4]) / 4
            # PWM 1000 → thrust_norm 0.0, PWM 1500 → 0.5 (hover), PWM 2000 → 1.0
            thrust_norm = max(0.0, (mean_pwm - 1000) / 1000)
            thrust_accel = thrust_norm * 2.0 * _GRAVITY   # 0 – 2g

            # NED vertical: positive accel_d = accelerate downward
            # Net = gravity (down) - thrust (up)
            accel_d = _GRAVITY - thrust_accel

            # Integrate vertical velocity and altitude
            _vd   += accel_d * dt
            _vd    = max(-_MAX_VEL, min(_MAX_VEL, _vd))
            _z_abs -= _vd * dt   # vd positive → z_abs decreases (falling)

            # Ground constraint
            if _z_abs <= _HOME_ELEV:
                _z_abs = _HOME_ELEV
                _vd    = min(0.0, _vd)

        time.sleep(0.01)   # 100 Hz

except KeyboardInterrupt:
    agl = _z_abs - _HOME_ELEV
    print(f"\n[stub] Stopped. Final AGL={agl:.1f} m")
finally:
    bridge.close()
