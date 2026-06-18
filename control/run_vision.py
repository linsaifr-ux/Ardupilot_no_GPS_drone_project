#!/usr/bin/env python3
"""
Vision position bridge — AnyLoc → ArduPilot EKF3 (standalone).

NOTE: run_flight.py now includes this logic in a background thread on the
same MAVLink connection. Use this file only when you want to test vision
position fusion WITHOUT running the full flight sequence.

Reads anyloc/latest_estimate.json (written by anyloc/run_localizer.py each
AnyLoc anchor frame) and sends VISION_POSITION_ESTIMATE to ArduPilot at 5 Hz.
Repeating the latest estimate at 5 Hz keeps EKF3 fusion alive between AnyLoc
updates (~every 10–20 s at typical sim frame rates).

Requires SITL launched with --add-param-file=control/no_gps.parm so that
EK3_SRC1_POSXY=6 (ExtNav) and VISO_TYPE=1 are set.

NOTE: Uses tcp:localhost:5763 (not 5762) so it can run alongside run_mavlink.py.
Start SITL with --out tcp:localhost:5763 to open that port.

Watch for EKF_POS_HORIZ_ABS (0x0010) going high — that means EKF3 accepted
the vision position and has a valid absolute fix.

Usage:
  # Terminal 1: SITL  (--out opens the extra port for run_vision.py)
  python3 third_party/ardupilot/Tools/autotest/sim_vehicle.py \
      -v ArduCopter --model=JSON --no-rebuild --console --map \
      -l 23.450868,120.286135,28.17,0 --add-param-file=control/no_gps.parm \
      --out tcp:localhost:5763

  # Terminal 2: Isaac Sim (bridge)
  cd simulator && ./run_chiayi.sh

  # Terminal 3: AnyLoc localizer (writes latest_estimate.json)
  DISPLAY=:2 conda run -n isaac_sim_test python anyloc/run_localizer.py

  # Terminal 4: vision bridge (this file)  ← uses port 5763
  python3 control/run_vision.py

  # Terminal 5: MAVLink monitor            ← uses port 5762 (no conflict)
  python3 control/run_mavlink.py
"""

import json
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from control.mavlink_ctrl import (
    MAVLinkCtrl,
    EKF_POS_HORIZ_ABS, EKF_PRED_POS_HORIZ_ABS, EKF_UNINITIALIZED,
)

# ── Constants ──────────────────────────────────────────────────────────────────

HOME_LAT     = 23.450868     # SITL home (matches -l flag)
HOME_LON     = 120.286135
HOME_ALT_MSL = 28.17         # metres MSL (from control/home_elevation.json)
COS_LAT      = math.cos(math.radians(HOME_LAT))
M_PER_DEG    = 111_320.0

ESTIMATE_JSON = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "anyloc", "latest_estimate.json",
)

SEND_HZ   = 5       # rate to (re)send the latest estimate to keep EKF3 alive
POLL_HZ   = 10      # how often to check for a new estimate file


def lat_lon_to_ned(lat: float, lon: float, alt_msl: float) -> tuple[float, float, float]:
    """Convert lat/lon/alt to NED metres from SITL home."""
    north = (lat - HOME_LAT) * M_PER_DEG
    east  = (lon - HOME_LON) * M_PER_DEG * COS_LAT
    down  = -(alt_msl - HOME_ALT_MSL)
    return north, east, down


def main():
    if not os.path.exists(ESTIMATE_JSON):
        print(f"[Vision] Waiting for {ESTIMATE_JSON}")
        print("[Vision] Start anyloc/run_localizer.py first.")

    ctrl = MAVLinkCtrl(connection_str="tcp:localhost:5763")
    if not ctrl.wait_heartbeat(timeout=60.0):
        print("[Vision] No HEARTBEAT — is SITL running?")
        return

    # Set EKF origin so VPE messages have an absolute NED reference frame.
    ctrl.set_ekf_origin(HOME_LAT, HOME_LON, HOME_ALT_MSL)
    ctrl.set_home_position(HOME_LAT, HOME_LON, HOME_ALT_MSL)
    time.sleep(0.5)

    print("\n[Vision] Sending VISION_POSITION_ESTIMATE at 5 Hz")
    print(f"[Vision] Reading from: {ESTIMATE_JSON}")
    print("[Vision] Watch for EKF flag POS_ABS (0x0010) — Ctrl-C to quit\n")

    current_est   = None
    last_mtime    = 0.0
    last_send_t   = 0.0
    last_ekf_flag = 0
    n_sent        = 0

    send_interval = 1.0 / SEND_HZ
    poll_interval = 1.0 / POLL_HZ

    try:
        while True:
            loop_start = time.time()

            # ── Drain MAVLink (EKF flag monitoring) ───────────────────────────
            ctrl.recv()
            flags = ctrl.ekf_flags
            if flags != last_ekf_flag:
                ekf_str = _ekf_summary(flags)
                marker  = " *** POS_ABS acquired!" if (flags & EKF_POS_HORIZ_ABS) else ""
                print(f"[Vision] EKF flags changed → 0x{flags:04x}  {ekf_str}{marker}")
                last_ekf_flag = flags

            # ── Poll for new estimate ──────────────────────────────────────────
            try:
                mtime = os.path.getmtime(ESTIMATE_JSON)
                if mtime != last_mtime:
                    with open(ESTIMATE_JSON) as fh:
                        est = json.load(fh)
                    north, east, down = lat_lon_to_ned(
                        est["est_lat"], est["est_lon"], est["alt_msl_m"])
                    yaw_rad = math.radians(est.get("yaw_deg", 0.0))
                    current_est = (north, east, down, yaw_rad)
                    last_mtime  = mtime
                    age_ms = (time.time() - est["timestamp"]) * 1000
                    print(f"[Vision] New estimate  N={north:+.1f} E={east:+.1f} D={down:+.1f} m"
                          f"  err={est.get('error_m', 0):.0f} m  age={age_ms:.0f} ms")
            except (FileNotFoundError, KeyError, json.JSONDecodeError):
                pass

            # ── Send at SEND_HZ ────────────────────────────────────────────────
            now = time.time()
            if current_est is not None and (now - last_send_t) >= send_interval:
                ctrl.send_vision_position(*current_est)
                last_send_t = now
                n_sent += 1
                if n_sent == 1:
                    print("[Vision] First VISION_POSITION_ESTIMATE sent")
                elif n_sent % 50 == 0:
                    print(f"[Vision] {n_sent} estimates sent  "
                          f"EKF=0x{flags:04x}  {_ekf_summary(flags)}")

            # ── Sleep remainder of poll interval ──────────────────────────────
            elapsed = time.time() - loop_start
            sleep_t = max(0.0, poll_interval - elapsed)
            if sleep_t > 0:
                time.sleep(sleep_t)

    except KeyboardInterrupt:
        print(f"\n[Vision] Stopped. Sent {n_sent} estimates.")
    finally:
        ctrl.close()


def _ekf_summary(flags: int) -> str:
    if flags & EKF_UNINITIALIZED:
        return "UNINIT"
    parts = []
    if flags & 0x0001: parts.append("ATT")
    if flags & 0x0002: parts.append("VEL")
    if flags & 0x0010: parts.append("POS_ABS")
    if flags & EKF_PRED_POS_HORIZ_ABS: parts.append("PRED_ABS")
    return ",".join(parts) if parts else "none"


if __name__ == "__main__":
    main()
