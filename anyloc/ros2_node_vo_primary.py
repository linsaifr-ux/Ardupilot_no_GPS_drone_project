#!/usr/bin/env python3
"""
Plan-B fusion node: VO-primary + score-gated AnyLoc.

Same topics, files, and outputs as anyloc/ros2_node.py (plan A) — drop-in
replacement, run ONE of the two, never both. Differences in the fusion policy:

  - Position starts from the EKF position (/drone/pose = GPS truth on SRC1 at
    the moment inference starts) and is integrated by VO on every frame.
  - VO is never reset by AnyLoc. AnyLoc runs every ANYLOC_INTERVAL frames,
    constrained to SEARCH_RADIUS_M around the current VO position, and only
    REPLACES the position when its cosine score ≥ --gate.

Why: benchmarked on field_data/survey13 real flight video vs GPS truth
(anyloc/test_vo_fusion_compare.py, logs in anyloc/logs/survey13_vo_fusion*.json,
2026-07-06): this scheme @gate 0.32 gave 13–15 m mean error vs 29–399 m for the
plan-A anchor-chain, whose unconditional re-anchoring can lock onto a wrong DB
entry permanently.

The gate is site/database-specific: real-footage scores live in ~0.16–0.34.
Calibrate from a shadow flight's accuracy CSV: set the gate just above the
highest score seen on bad matches (survey13/test20: worst bad match = 0.300).

Run:
  bash anyloc/run_ros2_localizer_vo.sh --headless [--gate 0.32]
"""

import math
import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import rclpy
from PIL import Image as PILImage

from anyloc.ros2_node import (
    AnyLocNode, run_postview, run_stream,
    ANYLOC_INTERVAL, SEARCH_RADIUS_M, MIN_AGL, MATCH_JPG, _geo_dist_m,
)

DEFAULT_GATE = 0.32


class VOPrimaryNode(AnyLocNode):
    """Overrides _cb_image with the VO-primary + score-gated-AnyLoc policy.

    Everything else — subscriptions, publishers, latest_estimate.json,
    accuracy CSV, postview state — is inherited from AnyLocNode.
    """

    def __init__(self, gate: float, test_mode: bool = False,
                 test_agl: float = 65.0):
        super().__init__(test_mode=test_mode, test_agl=test_agl)
        self._gate = gate
        self._pos_lat = None    # fused estimate — VO-integrated, AnyLoc-corrected
        self._pos_lon = None
        self._n_accept = 0
        self._n_reject = 0
        self._seed_warned = False
        self.get_logger().info(
            f"[VO-primary] fusion: VO every frame, AnyLoc every "
            f"{ANYLOC_INTERVAL} frames accepted iff score ≥ {gate:.2f}")

    # ── Plan-B fusion loop ────────────────────────────────────────────────────

    def _cb_image(self, msg):
        try:
            pil_img = PILImage.frombytes(
                "RGB", (msg.width, msg.height), bytes(msg.data))
        except Exception as e:
            self.get_logger().warn(f"Image decode: {e}")
            return

        with self.lock:
            self.latest_frame = pil_img

        agl_m = self._drone_agl
        if self._test_mode:
            if agl_m < 2.0:
                agl_m = self._test_agl
            if not self._agl_logged:
                print(f"[VO-primary] TEST MODE — running (agl={agl_m:.0f} m)")
                self._agl_logged = True
        else:
            if agl_m < MIN_AGL:
                return
            if not self._agl_logged:
                print(f"[VO-primary] AGL {agl_m:.0f} m ≥ {MIN_AGL:.0f} m — starting")
                self._agl_logged = True

        yaw_deg   = math.degrees(self._drone_yaw)
        drone_lat = self._drone_lat
        drone_lon = self._drone_lon
        drone_alt = self._drone_alt

        # Seed once from the EKF position — GPS truth on SRC1 at handover.
        if self._pos_lat is None:
            if not self._pose_received:
                if not self._seed_warned:
                    print("[VO-primary] waiting for /drone/pose to seed position "
                          "— check hw_bridge is running")
                    self._seed_warned = True
                return
            self._pos_lat, self._pos_lon = drone_lat, drone_lon
            print(f"[VO-primary] seeded from EKF position "
                  f"{drone_lat:.6f}, {drone_lon:.6f}")

        self._frame_count += 1
        run_anyloc = (self._frame_count == 1 or
                      self._frame_count % ANYLOC_INTERVAL == 0)

        # VO every frame — integrates the position directly, never reset.
        # Yaw convention identical to ros2_node.py (compass bearing degrees).
        _vo_yaw = -math.degrees(self._drone_yaw)
        dlat, dlon, n_vo = self._vo.update(pil_img, agl_m, _vo_yaw)
        self._pos_lat += dlat
        self._pos_lon += dlon

        # AnyLoc candidate every ANYLOC_INTERVAL frames — gate on score.
        t0 = time.perf_counter()
        match_img = None
        score     = self._last_score
        db_idx    = 0
        accepted  = False
        if run_anyloc:
            result = self._loc.localize(
                pil_img, agl_m=agl_m,
                center_lat=self._pos_lat, center_lon=self._pos_lon,
                radius_m=SEARCH_RADIUS_M)
            if result is None:
                return
            m_lat, m_lon, _, match_img, score, db_idx = result
            self._last_score = score
            accepted = score >= self._gate
            if accepted:
                jump_m = _geo_dist_m(self._pos_lat, self._pos_lon, m_lat, m_lon)
                self._pos_lat, self._pos_lon = m_lat, m_lon
                self._n_accept += 1
                print(f"[VO-primary] AnyLoc ACCEPT score={score:.3f} "
                      f"jump={jump_m:.1f} m  "
                      f"({self._n_accept}✓/{self._n_reject}✗)")
            else:
                self._n_reject += 1
        elapsed_ms = (time.perf_counter() - t0) * 1000.0

        est_lat, est_lon = self._pos_lat, self._pos_lon
        gps_lat, gps_lon = self._gps_lat, self._gps_lon

        self._write_estimate(est_lat, est_lon, drone_alt, agl_m, yaw_deg,
                             score, gps_lat, gps_lon)
        self._publish(est_lat, est_lon, drone_alt)

        err_m = _geo_dist_m(gps_lat, gps_lon, est_lat, est_lon)
        if run_anyloc:
            mode_tag = 'ANYLOC-ACC' if accepted else 'ANYLOC-REJ'
        else:
            mode_tag = 'VO'

        self._log_writer.writerow([
            time.time(), drone_lat, drone_lon, gps_lat, gps_lon,
            est_lat, est_lon, err_m, score, mode_tag, agl_m,
            n_vo, elapsed_ms,
        ])
        self._log_fh.flush()

        with self.lock:
            if match_img is not None:
                self.latest_match = match_img
                try:
                    _tmp = MATCH_JPG + ".tmp"
                    match_img.save(_tmp, "JPEG", quality=85)
                    os.replace(_tmp, MATCH_JPG)
                except Exception:
                    pass
            self.latest_result = dict(
                drone_lat=drone_lat, drone_lon=drone_lon,
                drone_alt=drone_alt, drone_agl=agl_m,
                drone_yaw=math.degrees(self._drone_yaw),
                gps_lat=gps_lat, gps_lon=gps_lon,
                est_lat=est_lat, est_lon=est_lon,
                err_m=err_m, score=score, db_idx=db_idx,
                n_vo=n_vo, elapsed_ms=elapsed_ms,
                mode_tag=mode_tag, run_anyloc=run_anyloc,
            )


# ── Entry point (mirrors ros2_node.main, plus --gate) ─────────────────────────

def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--gate", type=float, default=DEFAULT_GATE,
                        help=f"AnyLoc accept score gate (default: {DEFAULT_GATE}"
                             " — calibrated on survey13/database_test20)")
    parser.add_argument("--headless", action="store_true",
                        help="Disable postview (flight mode — no display, no stream)")
    parser.add_argument("--stream-host", metavar="IP", default=None,
                        help="Stream postview as H.265/RTP to this ground station IP")
    parser.add_argument("--stream-port", type=int, default=5000,
                        help="UDP port for GStreamer stream (default: 5000)")
    parser.add_argument("--test", action="store_true",
                        help="Test mode: bypass AGL gate, publish VPE directly "
                             "to /mavros/vision_pose/pose_cov")
    parser.add_argument("--test-agl", type=float, default=65.0,
                        help="Fake AGL (m) used when on ground in --test mode")
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = VOPrimaryNode(gate=args.gate, test_mode=args.test,
                         test_agl=args.test_agl)

    if args.headless:
        print("[VO-primary] Running headless — no postview window")
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()
        return

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        if args.stream_host:
            run_stream(node, args.stream_host, args.stream_port)
        else:
            run_postview(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
