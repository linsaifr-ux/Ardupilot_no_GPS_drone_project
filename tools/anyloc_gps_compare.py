#!/usr/bin/env python3
"""
Compare AnyLoc estimate against GPS ground truth.

Subscribes:
  /mavros/global_position/global  (sensor_msgs/NavSatFix) — GPS position
  /drone/agl                      (std_msgs/Float64)       — altitude gate

Reads:
  anyloc/latest_estimate.json     — AnyLoc position estimate

Logs to: anyloc_gps_compare.csv

Run:
  source /opt/ros/humble/setup.bash
  python3 tools/anyloc_gps_compare.py

Requires GPS_TYPE != 0 in ArduPilot and GPS fix. Revert GPS_TYPE=0 before contest.
"""
import json
import math
import os
import sys
import time

_ROS2_SITE = "/opt/ros/humble/lib/python3.10/site-packages"
if os.path.isdir(_ROS2_SITE) and _ROS2_SITE not in sys.path:
    sys.path.insert(0, _ROS2_SITE)

import rclpy
import rclpy.node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64

HERE         = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ESTIMATE_JSON = os.path.join(HERE, "anyloc", "latest_estimate.json")
LOG_CSV      = os.path.join(HERE, "anyloc_gps_compare.csv")

M_PER_DEG = 111_320.0

_SENSOR_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                          durability=DurabilityPolicy.VOLATILE, depth=10)


def _geo_dist_m(lat1, lon1, lat2, lon2):
    cos_lat = math.cos(math.radians((lat1 + lat2) / 2.0))
    dn = (lat1 - lat2) * M_PER_DEG
    de = (lon1 - lon2) * M_PER_DEG * cos_lat
    return math.hypot(dn, de), dn, de


class CompareNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("anyloc_gps_compare")

        self._gps_lat  = None
        self._gps_lon  = None
        self._gps_fix  = False
        self._agl      = 0.0
        self._last_mtime = 0.0
        self._n_logged = 0

        self.create_subscription(NavSatFix, "/mavros/global_position/global",
                                 self._cb_gps, _SENSOR_QOS)
        self.create_subscription(Float64, "/drone/agl",
                                 self._cb_agl, _SENSOR_QOS)

        self.create_timer(1.0, self._compare)

        need_header = not os.path.exists(LOG_CSV)
        self._log = open(LOG_CSV, "a")
        if need_header:
            self._log.write(
                "timestamp,agl_m,gps_lat,gps_lon,"
                "anyloc_lat,anyloc_lon,anyloc_score,anyloc_internal_err_m,"
                "horiz_err_m,north_err_m,east_err_m\n")

        print(f"[compare] Logging to {LOG_CSV}")
        print("[compare] Waiting for GPS fix and AnyLoc estimates ...")

    def _cb_gps(self, msg):
        # status: -1=no fix, 0=fix, 1=SBAS, 2=GBAS
        self._gps_fix = msg.status.status >= 0
        if self._gps_fix:
            self._gps_lat = msg.latitude
            self._gps_lon = msg.longitude

    def _cb_agl(self, msg):
        self._agl = msg.data

    def _compare(self):
        if not self._gps_fix or self._gps_lat is None:
            if self._n_logged == 0:
                print("[compare] waiting for GPS fix ...")
            return

        try:
            mtime = os.path.getmtime(ESTIMATE_JSON)
        except FileNotFoundError:
            if self._n_logged == 0:
                print("[compare] waiting for anyloc/latest_estimate.json ...")
            return

        if mtime == self._last_mtime:
            return   # no new AnyLoc estimate

        try:
            with open(ESTIMATE_JSON) as f:
                est = json.load(f)
        except (json.JSONDecodeError, KeyError):
            return

        est_lat   = est.get("est_lat")
        est_lon   = est.get("est_lon")
        score     = est.get("score", 0.0)
        int_err_m = est.get("error_m", 999.0)   # AnyLoc's own error vs drone pose
        est_agl   = est.get("agl_m", 0.0)

        if est_lat is None or est_lon is None:
            return

        self._last_mtime = mtime

        dist_m, dn, de = _geo_dist_m(self._gps_lat, self._gps_lon, est_lat, est_lon)

        ts = time.time()
        self._log.write(
            f"{ts:.3f},{self._agl:.1f},"
            f"{self._gps_lat:.7f},{self._gps_lon:.7f},"
            f"{est_lat:.7f},{est_lon:.7f},"
            f"{score:.4f},{int_err_m:.1f},"
            f"{dist_m:.1f},{dn:.1f},{de:.1f}\n")
        self._log.flush()
        self._n_logged += 1

        flag = "OK" if dist_m < 50 else ("MARGINAL" if dist_m < 100 else "BAD")
        print(f"[compare] AGL={self._agl:.0f}m  GPS=({self._gps_lat:.5f},{self._gps_lon:.5f})"
              f"  AnyLoc=({est_lat:.5f},{est_lon:.5f})"
              f"  err={dist_m:.0f}m (N={dn:+.0f} E={de:+.0f})"
              f"  score={score:.3f}  [{flag}]")

    def destroy_node(self):
        self._log.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = CompareNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print(f"[compare] Logged {node._n_logged} samples to {LOG_CSV}")


if __name__ == "__main__":
    main()
