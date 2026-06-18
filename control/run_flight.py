#!/usr/bin/env python3
"""
6b-iv: Autonomous flight + vision position bridge in one process.

Vision sending (VISION_POSITION_ESTIMATE at 5 Hz) runs in a background
thread sharing the same MAVLinkCtrl — no second TCP connection needed.
This replaces running run_vision.py separately.

If anyloc/latest_estimate.json does not exist a stub estimate at the home
position is written automatically so EKF3 gets a position fix without the
full AnyLoc pipeline.

Usage:
  # Terminal 1 — Isaac Sim first (prints centre_elev, writes control/home_elevation.json)
  cd simulator && ./run_chiayi.sh

  # Terminal 2 — SITL
  # FIRST RUN (fresh setup or param change): load params into EEPROM with --wipe,
  # then type "reboot" in the MAVProxy console so RebootRequired params (VISO_TYPE,
  # SCHED_LOOP_RATE, FRAME_CLASS, SIM_GPS1_ENABLE) take effect.
  #
  #   python3 third_party/ardupilot/Tools/autotest/sim_vehicle.py \
  #       -v ArduCopter --model=JSON --no-rebuild --console --map \
  #       -l 23.450868,120.286135,28.17,0 \
  #       --add-param-file=control/no_gps.parm --wipe
  #   (wait for "Saved 1 params" in console, then type: reboot)
  #
  # SUBSEQUENT RUNS (params already in EEPROM — no --wipe needed):
  python3 third_party/ardupilot/Tools/autotest/sim_vehicle.py \
      -v ArduCopter --model=JSON --no-rebuild --console --map \
      -l 23.450868,120.286135,<centre_elev>,0

  # Terminal 3 (optional) — AnyLoc localizer
  DISPLAY=:2 conda run -n isaac_sim_test python anyloc/run_localizer.py

  # Terminal 4 — this script (vision + flight)
  python3 control/run_flight.py
"""

import json
import math
import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from control.mavlink_ctrl import MAVLinkCtrl
from pymavlink import mavutil

# ── Home position (must match SITL -l flag) ────────────────────────────────────
HOME_LAT     = 23.450868
HOME_LON     = 120.286135

# Read terrain elevation written by cesium_scene.py; fall back to 46 m if not found.
_HOME_CFG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "home_elevation.json")
try:
    with open(_HOME_CFG) as _f:
        HOME_ALT_MSL = float(json.load(_f)["centre_elev_m"])
    print(f"[Flight] HOME_ALT_MSL = {HOME_ALT_MSL:.1f} m (from {_HOME_CFG})")
except (FileNotFoundError, KeyError):
    HOME_ALT_MSL = 46.0
    print(f"[Flight] HOME_ALT_MSL = {HOME_ALT_MSL:.1f} m (default — run Isaac Sim first to set terrain elevation)")

COS_LAT      = math.cos(math.radians(HOME_LAT))
M_PER_DEG    = 111_320.0

ESTIMATE_JSON = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "anyloc", "latest_estimate.json",
)

# ── Mission parameters ──────────────────────────────────────────────────────────
TAKEOFF_ALT      = 10.0   # metres AGL
WAYPOINT_RADIUS  = 3.0    # metres — how close counts as "reached"
WAYPOINT_TIMEOUT = 60.0   # seconds per waypoint before giving up

# NED waypoints (north m, east m, down m) — down = -alt_agl
WAYPOINTS = [
    ( 20.0,   0.0, -10.0),
    ( 20.0,  20.0, -10.0),
    (  0.0,  20.0, -10.0),
    (  0.0,   0.0, -10.0),
]

VISION_HZ = 5   # VISION_POSITION_ESTIMATE send rate


# ── Helpers ────────────────────────────────────────────────────────────────────

def _lat_lon_to_ned(lat: float, lon: float, alt_msl: float):
    north = (lat - HOME_LAT) * M_PER_DEG
    east  = (lon - HOME_LON) * M_PER_DEG * COS_LAT
    down  = -(alt_msl - HOME_ALT_MSL)
    return north, east, down


def _write_stub_estimate() -> None:
    est = {
        "timestamp": time.time(),
        "est_lat":   HOME_LAT,
        "est_lon":   HOME_LON,
        "alt_msl_m": HOME_ALT_MSL,
        "agl_m":     0.0,
        "yaw_deg":   0.0,
        "score":     0.9,
        "error_m":   0.0,
    }
    tmp = ESTIMATE_JSON + ".tmp"
    with open(tmp, "w") as fh:
        json.dump(est, fh)
    os.replace(tmp, ESTIMATE_JSON)
    print(f"[Flight] Stub estimate written to {ESTIMATE_JSON}")


def _vision_loop(ctrl: MAVLinkCtrl, stop: threading.Event) -> None:
    """
    Background thread — reads latest_estimate.json and sends
    VISION_POSITION_ESTIMATE at VISION_HZ to keep EKF3 fusion alive.
    Shares ctrl with the main thread; send and recv use independent
    socket directions so no lock is needed.
    """
    last_mtime   = 0.0
    current_est  = None
    send_interval = 1.0 / VISION_HZ
    n_sent = 0

    while not stop.is_set():
        t0 = time.time()

        try:
            mtime = os.path.getmtime(ESTIMATE_JSON)
            if mtime != last_mtime:
                with open(ESTIMATE_JSON) as fh:
                    est = json.load(fh)
                north, east, down = _lat_lon_to_ned(
                    est["est_lat"], est["est_lon"], est["alt_msl_m"])
                yaw_rad     = math.radians(est.get("yaw_deg", 0.0))
                current_est = (north, east, down, yaw_rad)
                last_mtime  = mtime
        except (FileNotFoundError, KeyError, json.JSONDecodeError):
            pass

        if current_est is not None:
            n, e, d, yaw_rad = current_est
            # Use EKF's own altitude (baro-based) for the vision Z component so
            # there is no vertical discrepancy that would trigger an innovation
            # gate rejection and kill XY fusion during flight.
            lp = ctrl.local_pos
            if lp is not None:
                d = lp.z
            ctrl.send_vision_position(n, e, d, yaw_rad)
            n_sent += 1
            if n_sent == 1:
                print("[Vision] First VISION_POSITION_ESTIMATE sent")

        elapsed = time.time() - t0
        time.sleep(max(0.0, send_interval - elapsed))


# ── Main flight sequence ────────────────────────────────────────────────────────

def main():
    ctrl = MAVLinkCtrl()   # tcp:localhost:5762

    # 1 — Connect
    if not ctrl.wait_heartbeat(timeout=60.0):
        print("[Flight] No HEARTBEAT — is SITL running?")
        return

    # 2 — Set EKF origin so EKF3 has an absolute NED reference frame.
    # Without this, VISION_POSITION_ESTIMATE has nowhere to anchor and
    # ArduPilot reports "EKF attitude is bad" / "VisOdom: not healthy".
    ctrl.set_ekf_origin(HOME_LAT, HOME_LON, HOME_ALT_MSL)
    ctrl.set_home_position(HOME_LAT, HOME_LON, HOME_ALT_MSL)
    time.sleep(0.5)   # give SITL time to process before VPE starts arriving

    # 4 — Ensure estimate file exists and is fresh, then start vision thread
    # Treat files older than 10 s as stale (leftover from a previous localizer run).
    _needs_stub = True
    if os.path.exists(ESTIMATE_JSON):
        age = time.time() - os.path.getmtime(ESTIMATE_JSON)
        if age < 10.0:
            _needs_stub = False
        else:
            print(f"[Flight] {ESTIMATE_JSON} is {age:.0f} s old — overwriting with stub")
    if _needs_stub:
        print(f"[Flight] Writing stub estimate at home position")
        _write_stub_estimate()

    stop_ev = threading.Event()
    vt = threading.Thread(target=_vision_loop, args=(ctrl, stop_ev), daemon=True)
    vt.start()
    print(f"[Flight] Vision thread started — sending at {VISION_HZ} Hz")

    # 5 — Wait for EKF POS_ABS
    print("[Flight] Waiting for EKF POS_ABS …")
    if not ctrl.wait_ekf_pos(timeout=60.0):
        print("[Flight] EKF never reached POS_ABS — aborting")
        stop_ev.set(); ctrl.close(); return
    print("[Flight] EKF POS_ABS ✓")

    # Wait for PRED_POS_ABS (bit 9) — set only when AP_VisualOdom::healthy() is
    # satisfied. Without this, regular arm fails with result=FAILED even when
    # ARMING_CHECK=0, because the VisOdom health check is mandatory in AP 4.x+.
    print("[Flight] Waiting for VisOdom healthy (EKF PRED_POS_ABS) …")
    if not ctrl.wait_visodom_healthy(timeout=30.0):
        print("[Flight] VisOdom did not become healthy — will use force arm")
    else:
        print("[Flight] VisOdom healthy ✓")

    # 6 — GUIDED mode
    ctrl.set_mode("GUIDED")
    time.sleep(1.0)
    ctrl.recv()

    # 7 — Arm
    print("[Flight] Arming …")
    ctrl.arm()
    result = ctrl.wait_command_ack(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, timeout=10.0)

    if result != 0:
        names = {0:"ACCEPTED",1:"TEMPORARILY_REJECTED",2:"DENIED",3:"UNSUPPORTED",4:"FAILED"}
        print(f"[Flight] Regular arm {names.get(result, result)} — retrying with force arm …")
        ctrl.arm(force=True)
        result = ctrl.wait_command_ack(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, timeout=10.0)

    if result is None or result != 0:
        names = {0:"ACCEPTED",1:"TEMPORARILY_REJECTED",2:"DENIED",3:"UNSUPPORTED",4:"FAILED"}
        print(f"[Flight] ARM rejected: {names.get(result, result)}")
        stop_ev.set(); ctrl.close(); return
    print("[Flight] Armed ✓")

    # 8 — Takeoff
    print(f"[Flight] Takeoff → {TAKEOFF_ALT} m AGL …")
    ctrl.takeoff(TAKEOFF_ALT)
    if ctrl.wait_altitude(TAKEOFF_ALT, tolerance=1.5, timeout=30.0):
        print(f"[Flight] Reached {TAKEOFF_ALT} m AGL ✓")
    else:
        print(f"[Flight] Altitude timeout — continuing")
    time.sleep(2.0)

    # 9 — Waypoints
    for i, (n, e, d) in enumerate(WAYPOINTS):
        print(f"[Flight] WP {i+1}/{len(WAYPOINTS)}  N={n:+.0f} E={e:+.0f} ALT={-d:.0f} m AGL")
        ctrl.set_position_ned(n, e, d)
        if ctrl.wait_position(n, e, d, radius=WAYPOINT_RADIUS, timeout=WAYPOINT_TIMEOUT):
            print(f"[Flight] WP {i+1} ✓")
        else:
            print(f"[Flight] WP {i+1} timeout")
        time.sleep(1.0)

    # 10 — RTL
    print("[Flight] Mission complete — RTL")
    ctrl.set_mode("RTL")

    deadline = time.time() + 60.0
    while time.time() < deadline:
        ctrl.recv()
        if not ctrl.is_armed:
            print("[Flight] Disarmed — landed ✓")
            break
        time.sleep(0.2)
    else:
        print("[Flight] Disarm timeout")

    stop_ev.set()
    ctrl.close()
    print("[Flight] Done.")


if __name__ == "__main__":
    main()
