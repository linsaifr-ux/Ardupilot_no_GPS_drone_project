#!/usr/bin/env python3
"""
Standalone PX4 bridge connectivity test (no Isaac Sim).

Feeds PX4SimBridge a stationary, level, grounded drone state at 250 Hz so we can
verify the PX4 SITL HIL link in isolation: PX4 should connect on TCP 4560, receive
HIL_SENSOR, and its EKF2 should initialise (level, on the ground) before we wire in
the heavy Cesium/Isaac physics.

Run order:
  1. python3 control/px4_bridge_test.py          # bridge listens on 4560
  2. (separate) launch PX4 SITL with SYS_AUTOSTART=10016 → connects to 4560
"""
import sys, os, time
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from control.px4_sim_bridge import PX4SimBridge

CENTRE_ELEV = 0.0          # sea level for the test
RATE_HZ     = 250.0        # PX4 IMU_INTEG_RATE default

def main():
    br = PX4SimBridge(listen_port=4560, centre_elev=CENTRE_ELEV)
    dt = 1.0 / RATE_HZ
    n = 0
    last_report = 0.0
    while True:
        t = time.time()
        # stationary, level, grounded drone: x=y=0, z=centre_elev (agl 0), no attitude
        motors = br.step(0.0, 0.0, CENTRE_ELEV, 0.0, 0.0, 0.0, t)
        n += 1
        if br.connected and t - last_report > 2.0:
            last_report = t
            mx = max(motors) if motors else 0.0
            print(f"[test] connected, HIL_SENSOR sent={br._n_sent}, "
                  f"motors max={mx:.3f}  ({'ARMED/spinning' if mx > 0.05 else 'idle'})",
                  flush=True)
        elapsed = time.time() - t
        time.sleep(max(0.0, dt - elapsed))

if __name__ == "__main__":
    main()
