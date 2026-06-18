#!/usr/bin/env python3
"""
Live MAVLink state monitor (terminal).

Connects to ArduPilot SITL and prints attitude, NED position, IMU, and
EKF status in real time. Run while SITL + Isaac Sim are active.

Usage:
  python3 control/run_mavlink.py
"""

import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from control.mavlink_ctrl import (
    MAVLinkCtrl,
    EKF_ATTITUDE, EKF_VEL_HORIZ, EKF_VEL_VERT,
    EKF_POS_HORIZ_REL, EKF_POS_HORIZ_ABS, EKF_POS_VERT_ABS,
    EKF_PRED_POS_HORIZ_ABS, EKF_UNINITIALIZED,
)

_NAN = float("nan")


def _ekf_label(flags: int) -> str:
    if flags & EKF_UNINITIALIZED:
        return "UNINIT"
    parts = []
    if flags & EKF_ATTITUDE:           parts.append("ATT")
    if flags & EKF_VEL_HORIZ:          parts.append("VEL_H")
    if flags & EKF_VEL_VERT:           parts.append("VEL_V")
    if flags & EKF_POS_HORIZ_REL:      parts.append("POS_REL")
    if flags & EKF_POS_HORIZ_ABS:      parts.append("POS_ABS")
    if flags & EKF_POS_VERT_ABS:       parts.append("ALT")
    if flags & EKF_PRED_POS_HORIZ_ABS: parts.append("PRED_ABS")
    return ",".join(parts) if parts else "none"


def main():
    ctrl = MAVLinkCtrl()
    ok = ctrl.wait_heartbeat(timeout=60.0)
    if not ok:
        print("\nNo HEARTBEAT received.")
        print("Make sure SITL AND the bridge are both running:")
        print("  Terminal 1: python3 third_party/ardupilot/Tools/autotest/sim_vehicle.py "
              "-v ArduCopter --model=JSON --no-rebuild --console --map "
              "-l 23.450868,120.286135,46,0 --out tcp:localhost:5763")
        print("  Terminal 2: python3 control/stub_bridge.py  "
              "# (or start Isaac Sim with cesium_scene.py)")
        return

    print("\n[MAVLink] Monitoring — Ctrl-C to quit\n")
    hdr = (f"{'TIME':>8}  "
           f"{'ROLL°':>7} {'PCH°':>7} {'YAW°':>7}  "
           f"{'N m':>9} {'E m':>9} {'D m':>9}  "
           f"{'Ax':>7} {'Ay':>7} {'Az':>7}  "
           f"EKF flags")
    print(hdr)
    print("-" * len(hdr))

    t0 = time.time()
    try:
        while True:
            ctrl.recv()

            att = ctrl.attitude
            pos = ctrl.local_pos
            imu = ctrl.imu
            flags = ctrl.ekf_flags

            r = math.degrees(att.roll)  if att else _NAN
            p = math.degrees(att.pitch) if att else _NAN
            y = math.degrees(att.yaw)   if att else _NAN

            n = pos.x if pos else _NAN
            e = pos.y if pos else _NAN
            d = pos.z if pos else _NAN

            ax = imu.xacc if imu else _NAN
            ay = imu.yacc if imu else _NAN
            az = imu.zacc if imu else _NAN

            line = (f"{time.time() - t0:8.1f}  "
                    f"{r:7.2f} {p:7.2f} {y:7.2f}  "
                    f"{n:9.2f} {e:9.2f} {d:9.2f}  "
                    f"{ax:7.2f} {ay:7.2f} {az:7.2f}  "
                    f"0x{flags:04x} {_ekf_label(flags)}")
            print(f"\r{line}", end="", flush=True)
            time.sleep(0.1)  # 10 Hz display rate

    except KeyboardInterrupt:
        print("\n[MAVLink] Stopped.")
    finally:
        ctrl.close()


if __name__ == "__main__":
    main()
