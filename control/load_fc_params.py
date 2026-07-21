#!/usr/bin/env python3
"""
Load a .parm file onto the FC over the Jetson serial link, verifying each
value by read-back. mavros must NOT be running (it owns the serial port).

    python3 control/load_fc_params.py [--file control/real_hw_pixhawk6c.parm]
                                      [--device /dev/ttyUSB0] [--baud 921600]
                                      [--reboot]

--reboot sends MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN afterwards (refused by the
FC while armed) — needed for SERIALn_* changes to take effect.

Params that the firmware doesn't know are reported and skipped, so a file
written for a newer firmware fails loudly per-param instead of silently.
"""
import argparse
import re
import sys
import time

from pymavlink import mavutil

SYS, COMP = 1, 1


def parse_parm(path):
    out = []
    with open(path) as f:
        for line in f:
            m = re.match(r'^([A-Z][A-Z0-9_]*)\s+(-?[0-9.]+)', line)
            if m:
                out.append((m.group(1), float(m.group(2))))
    return out


def get_param(conn, name, tries=3, timeout=2):
    for _ in range(tries):
        conn.mav.param_request_read_send(SYS, COMP, name.encode(), -1)
        t0 = time.time()
        while time.time() - t0 < timeout:
            pv = conn.recv_match(type='PARAM_VALUE', blocking=True, timeout=timeout)
            if pv and pv.param_id == name:
                return pv
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--file', default='control/real_hw_pixhawk6c.parm')
    ap.add_argument('--device', default='/dev/ttyUSB0')
    ap.add_argument('--baud', type=int, default=921600)
    ap.add_argument('--reboot', action='store_true')
    args = ap.parse_args()

    wanted = parse_parm(args.file)
    print(f'{len(wanted)} params in {args.file}')

    conn = mavutil.mavlink_connection(args.device, baud=args.baud,
                                      source_system=255, source_component=190)
    if conn.wait_heartbeat(timeout=15) is None:
        sys.exit(f'no heartbeat on {args.device}@{args.baud}')

    unknown, failed, changed, already = [], [], [], []
    for name, val in wanted:
        pv = get_param(conn, name)
        if pv is None:
            unknown.append(name)
            print(f'  {name:16s} UNKNOWN to firmware — skipped')
            continue
        if abs(pv.param_value - val) < 1e-4:
            already.append(name)
            continue
        old = pv.param_value
        conn.mav.param_set_send(SYS, COMP, name.encode(), val, pv.param_type)
        time.sleep(0.1)
        rb = get_param(conn, name)
        if rb is not None and abs(rb.param_value - val) < 1e-4:
            changed.append(name)
            print(f'  {name:16s} {old:g} -> {rb.param_value:g}  OK')
        else:
            failed.append(name)
            print(f'  {name:16s} SET FAILED (still '
                  f'{rb.param_value if rb else "?":g})')

    print(f'\nset={len(changed)}  already-correct={len(already)}  '
          f'unknown={len(unknown)}  FAILED={len(failed)}')
    if args.reboot and not failed:
        print('Rebooting FC ...')
        conn.mav.command_long_send(SYS, COMP,
                                   mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
                                   0, 1, 0, 0, 0, 0, 0, 0)
        time.sleep(0.5)
    conn.close()
    sys.exit(1 if (failed or unknown) else 0)


if __name__ == '__main__':
    main()
