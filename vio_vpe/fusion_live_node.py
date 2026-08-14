#!/usr/bin/env python3
"""Shadow-mode fusion node -- Phase 3 of the plan at
~/.claude/plans/robust-squishing-forest.md.

Owns FusionFilter (fusion_filter.py, unmodified) and drives it from live
/vio/odom + /vpe/fix, exactly like a real onboard deployment would -- EXCEPT
it never sends anything to the flight controller. This is a passive observer:
it computes the same estimate a real system would produce, logs it, and
publishes it for the ground overlay, but the pilot flies manually the whole
time and this node could be killed at any point with zero effect on the
aircraft.

################################################################################
# HARD INVARIANT: this file must never import mavlink_ctrl, ardupilot_commander,
# or anything that publishes to /mavros/vision_pose/* or sends a MAVLink
# VISION_POSITION_ESTIMATE. Verify with:
#   grep -n "vision_pose\|vision_position_estimate\|mavlink_ctrl\|ardupilot_commander" fusion_live_node.py
# should return nothing. See the plan's verification step 2 for the runtime
# check (ros2 topic hz /mavros/vision_pose/pose_cov must show zero traffic).
################################################################################

Axis convention (get this wrong and every number is silently mis-scaled or
rotated -- see METHOD_imx900_vio_vpe.md Sec 3, "the single most expensive
class of mistake in this project"):
  - Everywhere in vio_vpe/* OUTSIDE this file: positions are (east, north)
    tuples, matching imx900_geo_common.enu() and vpe_localize_imx900's
    dict keys.
  - FusionFilter's own state is x=[n, e, vn, ve] -- north FIRST. The swap
    happens only at the FusionFilter call boundary, right here, and nowhere
    else.
  - /vio/odom's position/velocity are in OpenVINS' own gravity-aligned,
    YAW-ARBITRARY world frame (vio_live_node's px,py) until rotated by the
    OnlineYawAligner -- never treat them as east/north before that.

Ground truth: /mavros/global_position/global (raw GPS), never /drone/pose,
so the error metric never depends on which source EKF3 currently has active
-- see the plan's Phase 3 note on this.

Run (see vio_vpe/launch_shadow_mode.sh for the full stack):
  source /opt/ros/humble/setup.bash
  ~/venv/vio_vpe/bin/python3 vio_vpe/fusion_live_node.py
"""
import argparse
import csv
import json
import math
import os
import sys
import time
from collections import deque

_ROS2_SITE = "/opt/ros/humble/lib/python3.10/site-packages"
if os.path.isdir(_ROS2_SITE) and _ROS2_SITE not in sys.path:
    sys.path.insert(0, _ROS2_SITE)

import rclpy
import rclpy.node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from fusion_filter import FusionFilter
from yaw_align import OnlineYawAligner
import imx900_geo_common as G

HERE = os.path.dirname(os.path.abspath(__file__))
LOG_DIR = os.path.join(HERE, "logs")
ESTIMATE_JSON = os.path.join(HERE, "latest_estimate.json")

PREDICT_HZ = 20.0
# Open parameter (see the plan): default to the doc's WORSE-case sigmas
# (survey45/48-class flights) rather than the best case (survey43), so the
# filter starts skeptical. Override with --sigma-vio/--sigma-vpe once a
# specific flight's regime is known.
DEFAULT_SIGMA_VIO_MS = 36.0
DEFAULT_SIGMA_VPE_M = 10.0
VIO_HISTORY_S = 5.0   # trailing buffer for nearest-time VIO/VPE pairing


class FusionLiveNode(rclpy.node.Node):
    def __init__(self, sigma_vio_ms, max_fix_age_s, yaw_min_span_m):
        super().__init__("fusion_live_node")

        self._sigma_vio_ms = sigma_vio_ms
        self.filt = FusionFilter(max_fix_age_s=max_fix_age_s)
        self.aligner = OnlineYawAligner(min_span_m=yaw_min_span_m)

        # trailing VIO samples for nearest-time pairing against VPE fixes,
        # and the latest sample for computing err_vio right now.
        self._vio_hist = deque()       # (t, x, y, vx, vy)
        self._vio_latest = None        # (t, x, y, vx, vy)
        self._vpe_latest = None        # (t, east, north)
        self._gps_latest = None        # (t, east, north)

        self._last_predict_t = None

        os.makedirs(LOG_DIR, exist_ok=True)
        log_path = os.path.join(LOG_DIR, f"shadow_{time.strftime('%Y%m%d_%H%M%S')}.csv")
        self._log_fh = open(log_path, "w", newline="")
        self._log = csv.writer(self._log_fh)
        self._log.writerow([
            "t", "vio_x", "vio_y", "vio_east", "vio_north",
            "vpe_east", "vpe_north", "vpe_age_s",
            "fused_east", "fused_north",
            "gps_east", "gps_north",
            "err_vio_m", "err_vpe_m", "err_fused_m",
            "yaw_resolved", "yaw_deg", "yaw_span_m",
            "vel_health", "pos_ok", "pos_rej", "vel_ok", "vel_rej",
            "resets", "slew_limited", "suppressed",
        ])
        self.get_logger().info(f"[fusion_live] shadow-mode -- logging to {log_path}")
        self.get_logger().warn(
            "[fusion_live] SHADOW MODE: this node never publishes to the flight "
            "controller. Nothing here affects the aircraft.")

        # publishes the prior vpe_live_node uses for its search radius --
        # NOT anything MAVROS/FC-facing.
        self._pub_state = self.create_publisher(PoseWithCovarianceStamped, "/fusion/state", 10)

        self.create_subscription(Odometry, "/vio/odom", self._cb_vio, 10)
        self.create_subscription(PoseWithCovarianceStamped, "/vpe/fix", self._cb_vpe, 10)
        self.create_subscription(NavSatFix, "/mavros/global_position/global", self._cb_gps,
                                 qos_profile_sensor_data)

        self.create_timer(1.0 / PREDICT_HZ, self._tick)

    def destroy_node(self):
        self._log_fh.close()
        super().destroy_node()

    # -- subscriptions ---------------------------------------------------------
    def _cb_gps(self, msg):
        if msg.status.status < 0:   # STATUS_NO_FIX
            return
        east, north = G.enu(msg.latitude, msg.longitude)
        t = rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        self._gps_latest = (t, east, north)

    def _cb_vio(self, msg):
        t = rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        vx, vy = msg.twist.twist.linear.x, msg.twist.twist.linear.y
        self._vio_latest = (t, x, y, vx, vy)
        self._vio_hist.append((t, x, y, vx, vy))
        while self._vio_hist and t - self._vio_hist[0][0] > VIO_HISTORY_S:
            self._vio_hist.popleft()

        if not self.aligner.resolved:
            return  # velocity is meaningless (arbitrary yaw) until resolved
        ve, vn = self.aligner.rotate_velocity(vx, vy)
        # FusionFilter boundary: swap to its (vn, ve) convention here only.
        ekf_speed_ref = None  # deliberately not GPS-derived -- see file header
        self.filt.update_velocity(vn, ve, self._sigma_vio_ms, ekf_speed_ref)

    def _cb_vpe(self, msg):
        t = rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        east, north = msg.pose.pose.position.x, msg.pose.pose.position.y
        sigma_m = math.sqrt(max(msg.pose.covariance[0], msg.pose.covariance[7]))
        self._vpe_latest = (t, east, north)

        # FusionFilter boundary: swap to its (n, e) convention here only.
        self.filt.update_position(north, east, sigma_m)

        # pair with the nearest-in-time buffered VIO sample for the yaw fit
        if self._vio_hist:
            vio_t, vx, vy, _, _ = min(self._vio_hist, key=lambda s: abs(s[0] - t))
            self.aligner.add_pair((vx, vy), (east, north))

    # -- main loop ---------------------------------------------------------
    def _tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        dt = (now - self._last_predict_t) if self._last_predict_t is not None else 1.0 / PREDICT_HZ
        self._last_predict_t = now
        self.filt.predict(dt)

        out = self.filt.output(dt)   # README's "not optional #2": never state()
        fused_e = fused_n = None
        if out is not None:
            fused_n, fused_e = float(out[0]), float(out[1])  # filter order is (n,e)
            sigma = self.filt.pos_sigma()
            m = PoseWithCovarianceStamped()
            m.header.stamp = self.get_clock().now().to_msg()
            m.header.frame_id = "imx900_site_enu"
            m.pose.pose.position.x = fused_e
            m.pose.pose.position.y = fused_n
            m.pose.covariance[0] = sigma ** 2
            m.pose.covariance[7] = sigma ** 2
            self._pub_state.publish(m)

        self._write_log_and_json(now, fused_e, fused_n)

    def _write_log_and_json(self, now, fused_e, fused_n):
        gps = self._gps_latest
        gps_e, gps_n = (gps[1], gps[2]) if gps else (None, None)

        vio_e = vio_n = None
        err_vio = None
        if self._vio_latest and self.aligner.resolved:
            _, vx, vy, _, _ = self._vio_latest
            vio_e, vio_n = self.aligner.project_position(vx, vy)
            if gps_e is not None:
                err_vio = math.hypot(vio_e - gps_e, vio_n - gps_n)

        vpe_e = vpe_n = vpe_age = None
        err_vpe = None
        if self._vpe_latest:
            vt, vpe_e, vpe_n = self._vpe_latest
            vpe_age = now - vt
            if gps_e is not None:
                err_vpe = math.hypot(vpe_e - gps_e, vpe_n - gps_n)

        err_fused = None
        if fused_e is not None and gps_e is not None:
            err_fused = math.hypot(fused_e - gps_e, fused_n - gps_n)

        st = self.filt.stats()
        self._log.writerow([
            now, self._vio_latest[1] if self._vio_latest else "",
            self._vio_latest[2] if self._vio_latest else "",
            vio_e, vio_n, vpe_e, vpe_n, vpe_age, fused_e, fused_n, gps_e, gps_n,
            err_vio, err_vpe, err_fused,
            self.aligner.resolved, self.aligner.yaw_deg, self.aligner.span_m,
            st["vel_health"], st["pos_ok"], st["pos_rej"], st["vel_ok"], st["vel_rej"],
            st["resets"], st["slew_limited"], st["suppressed"],
        ])
        self._log_fh.flush()

        try:
            with open(ESTIMATE_JSON, "w") as fh:
                json.dump(dict(
                    t=now, fused_east=fused_e, fused_north=fused_n, err_fused_m=err_fused,
                    vio_east=vio_e, vio_north=vio_n, err_vio_m=err_vio,
                    vpe_east=vpe_e, vpe_north=vpe_n, err_vpe_m=err_vpe, vpe_age_s=vpe_age,
                    yaw_resolved=self.aligner.resolved, yaw_deg=self.aligner.yaw_deg,
                    yaw_span_m=self.aligner.span_m, vel_health=st["vel_health"],
                    initialised=st["initialised"],
                ), fh)
        except OSError:
            pass  # never let a logging hiccup take down the observer


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sigma-vio", type=float, default=DEFAULT_SIGMA_VIO_MS,
                    help="VIO velocity sigma (m/s), conservative default -- see "
                         "the plan's 'Open parameter' note")
    ap.add_argument("--max-fix-age", type=float, default=5.0,
                    help="FusionFilter max_fix_age_s -- must scale with the real "
                         "VPE rate (>=5x median inter-fix interval), 5s default "
                         "matches offline evaluation at ~1Hz VPE")
    ap.add_argument("--yaw-min-span", type=float, default=150.0)
    args, ros_args = ap.parse_known_args()

    rclpy.init(args=ros_args)
    node = FusionLiveNode(args.sigma_vio, args.max_fix_age, args.yaw_min_span)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # rclpy's own SIGINT handler (see launch_shadow_mode.sh's staged
        # cleanup) may already have shut the context down by the time this
        # runs -- rclpy.shutdown() raises if called twice, harmless but a
        # scary-looking traceback on every normal Ctrl+C/stop.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
