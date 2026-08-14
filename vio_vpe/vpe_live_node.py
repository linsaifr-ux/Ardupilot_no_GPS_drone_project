#!/usr/bin/env python3
"""Live VPE (SuperPoint+LightGlue) localizer node -- Phase 2 of the shadow-mode
observer described in the plan at ~/.claude/plans/robust-squishing-forest.md.

Wraps PrebuiltVpeLocalizer (vpe_localize_imx900.py) around live topics instead
of a recorded flight. Deliberately does not touch GPS at all -- ground-truth
comparison happens once, centrally, in fusion_live_node -- this node's only
job is to turn camera+AGL+heading into VPE fixes as fast as it reliably can.

Subscribes:
  /drone/camera/image_raw            (sensor_msgs/Image, rgb8)
  /drone/agl                         (std_msgs/Float64)
  /mavros/global_position/compass_hdg (std_msgs/Float64, degrees)
  /fusion/state                      (geometry_msgs/PoseWithCovarianceStamped)
                                      -- prior position + sigma, used only to
                                      set the search radius (README's "not
                                      optional #1"); absent/stale -> brute force.

Publishes:
  /vpe/fix  (geometry_msgs/PoseWithCovarianceStamped) -- ENU east/north in the
            imx900_geo_common site frame, one per accepted fix.

Writes:
  vio_vpe/logs/vpe_live_<timestamp>.csv -- every query attempt, fixed or not,
  with the inlier stats PrebuiltVpeLocalizer.localize() already returns.

Run (see vio_vpe/launch_shadow_mode.sh for the full stack):
  source /opt/ros/humble/setup.bash
  ~/venv/vio_vpe/bin/python3 vio_vpe/vpe_live_node.py \
      --map vio_vpe/maps/imx900_survey47.vpemap
"""
import argparse
import csv
import os
import sys
import threading
import time

_ROS2_SITE = "/opt/ros/humble/lib/python3.10/site-packages"
if os.path.isdir(_ROS2_SITE) and _ROS2_SITE not in sys.path:
    sys.path.insert(0, _ROS2_SITE)

import rclpy
import rclpy.node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Image
from std_msgs.msg import Float64

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from vpe_localize_imx900 import PrebuiltVpeLocalizer

HERE = os.path.dirname(os.path.abspath(__file__))
LOG_DIR = os.path.join(HERE, "logs")

# README's "not optional #1": search_radius = 3 sigma from the fusion filter's
# covariance, floor 35 m. MIN_SEARCH_RADIUS_M is that floor; used until the
# first /fusion/state arrives (brute force, matching PrebuiltVpeLocalizer's
# own no-prior fallback) and as a floor forever after.
MIN_SEARCH_RADIUS_M = 35.0
# Conservative default fix sigma -- see the plan's "Open parameter" note:
# the doc's per-flight VPE error ranged 4.4-18.9 m, so default to the worse
# case rather than the best one.
DEFAULT_SIGMA_M = 10.0


def _cov_pose_msg(east, north, sigma_m, frame_id="imx900_site_enu", stamp=None):
    msg = PoseWithCovarianceStamped()
    if stamp is not None:
        msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.pose.pose.position.x = float(east)
    msg.pose.pose.position.y = float(north)
    msg.pose.covariance[0] = float(sigma_m) ** 2   # xx
    msg.pose.covariance[7] = float(sigma_m) ** 2   # yy
    return msg


class VpeLiveNode(rclpy.node.Node):
    def __init__(self, map_path, sigma_m, min_radius_m, device):
        super().__init__("vpe_live_node")

        self._sigma_m = sigma_m
        self._min_radius_m = min_radius_m
        self._loc = PrebuiltVpeLocalizer(map_path, device=device,
                                         search_radius_m=min_radius_m)

        # latest inputs, updated by their own callbacks
        self._lk = threading.Lock()
        self._agl = None
        self._hdg = None
        self._prior = None          # (east, north) from /fusion/state
        self._prior_sigma = None
        self._pending = None        # (frame_bgr, t) newest unconsumed frame
        self._new_frame_ev = threading.Event()
        self._shutdown = False

        os.makedirs(LOG_DIR, exist_ok=True)
        log_path = os.path.join(LOG_DIR, f"vpe_live_{time.strftime('%Y%m%d_%H%M%S')}.csv")
        self._log_fh = open(log_path, "w", newline="")
        self._log = csv.writer(self._log_fh)
        self._log.writerow(["t", "agl", "hdg", "search_radius_m", "n_cand", "fixed",
                            "east", "north", "n_inl", "ratio", "scale", "rot",
                            "query_ms"])
        self.get_logger().info(f"[vpe_live] logging to {log_path}")

        self._pub_fix = self.create_publisher(PoseWithCovarianceStamped, "/vpe/fix", 10)

        self.create_subscription(Image, "/drone/camera/image_raw", self._cb_image,
                                 qos_profile_sensor_data)
        self.create_subscription(Float64, "/drone/agl", self._cb_agl, 10)
        self.create_subscription(Float64, "/mavros/global_position/compass_hdg",
                                 self._cb_hdg, qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, "/fusion/state",
                                 self._cb_fusion_state, 10)

        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()

    def destroy_node(self):
        self._shutdown = True
        self._new_frame_ev.set()
        self._worker.join(timeout=2.0)
        self._log_fh.close()
        super().destroy_node()

    # -- callbacks: cheap, no SuperPoint/LightGlue calls here -----------------
    def _cb_agl(self, msg):
        with self._lk:
            self._agl = float(msg.data)

    def _cb_hdg(self, msg):
        with self._lk:
            self._hdg = float(msg.data)

    def _cb_fusion_state(self, msg):
        sigma = max(msg.pose.covariance[0], msg.pose.covariance[7]) ** 0.5
        with self._lk:
            self._prior = (msg.pose.pose.position.x, msg.pose.pose.position.y)
            self._prior_sigma = sigma

    def _cb_image(self, msg):
        if msg.encoding not in ("rgb8", "bgr8"):
            self.get_logger().warn(f"unexpected image encoding '{msg.encoding}'",
                                   throttle_duration_sec=5.0)
            return
        import numpy as np
        import cv2 as cv
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        bgr = arr if msg.encoding == "bgr8" else cv.cvtColor(arr, cv.COLOR_RGB2BGR)
        t = rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        with self._lk:
            self._pending = (bgr.copy(), t)
        self._new_frame_ev.set()

    # -- worker: the only thread that calls into SuperPoint/LightGlue ---------
    def _worker_loop(self):
        while not self._shutdown:
            if not self._new_frame_ev.wait(timeout=1.0):
                continue
            self._new_frame_ev.clear()
            with self._lk:
                pending = self._pending
                self._pending = None
                agl, hdg = self._agl, self._hdg
                prior, prior_sigma = self._prior, self._prior_sigma
            if pending is None or self._shutdown:
                continue
            frame, t = pending
            if agl is None or hdg is None:
                continue  # no AGL/heading yet -- can't build a north-up tile

            radius = self._min_radius_m
            if prior is not None and prior_sigma is not None:
                radius = max(self._min_radius_m, 3.0 * prior_sigma)
            self._loc.radius = radius

            t0 = time.time()
            r = self._loc.localize(frame, agl, hdg, prior=prior)
            query_ms = (time.time() - t0) * 1000.0

            if r:
                self._pub_fix.publish(_cov_pose_msg(r["east"], r["north"], self._sigma_m))
                self._log.writerow([t, agl, hdg, radius, r["n_cand"], 1,
                                    r["east"], r["north"], r["n_inl"], r["ratio"],
                                    r["scale"], r["rot"], query_ms])
            else:
                self._log.writerow([t, agl, hdg, radius, "", 0, "", "", "", "", "", "",
                                    query_ms])
            self._log_fh.flush()
            self.get_logger().info(
                f"[vpe_live] {'FIX' if r else 'no fix'} agl={agl:.0f} radius={radius:.0f} "
                f"took={query_ms:.0f}ms" + (f" err_tiles={r['n_cand']}" if r else ""))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", default=os.path.join(HERE, "maps", "imx900_survey47.vpemap"))
    ap.add_argument("--sigma-vpe", type=float, default=DEFAULT_SIGMA_M,
                    help="fixed sigma (m) reported on /vpe/fix, conservative default "
                         "per the plan's 'Open parameter' note")
    ap.add_argument("--min-search-radius", type=float, default=MIN_SEARCH_RADIUS_M)
    ap.add_argument("--device", default="cuda")
    args, ros_args = ap.parse_known_args()

    rclpy.init(args=ros_args)
    node = VpeLiveNode(args.map, args.sigma_vpe, args.min_search_radius, args.device)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # see fusion_live_node.py's identical guard: rclpy's own SIGINT
        # handler may have already shut the context down by here.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
