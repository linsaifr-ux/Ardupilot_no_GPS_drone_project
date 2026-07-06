#!/usr/bin/env python3
"""
Plan-B fusion node: VO-primary + score-gated AnyLoc.

Same topics, files, and outputs as anyloc/ros2_node.py (plan A) — drop-in
replacement, run ONE of the two, never both. Differences in the fusion policy:

  - Position starts from the EKF position (/drone/pose = GPS truth on SRC1 at
    the moment inference starts) and is integrated by VO on every frame.
  - VO is never reset by AnyLoc. AnyLoc runs every ANYLOC_INTERVAL frames,
    constrained to SEARCH_RADIUS_M around the current VO position, and a
    candidate is only used when its cosine score ≥ --gate.
  - Jump gate: a score-passing candidate must also be physically plausible —
    within --jump-base + --drift-rate × (seconds since last accepted
    correction) of the current VO position. VO tracks real motion, so this
    bound only has to cover VO error growth plus AnyLoc's grid quantization
    (localizer output snaps to DB entries on a ~50 m grid → a correct match
    can sit ~35 m off truth); a single aliased far match can never teleport
    the estimate. The bound grows over time so long-accumulated VO drift can
    still be corrected.
  - Re-acquisition: if --reacquire-n consecutive jump-rejected candidates
    agree with each other (within 30 m), that's a consistent signal, not
    aliasing noise — VO is the wrong one, so the position relocates there.
  - Blend: accepted corrections move the position by --blend × (candidate −
    position) instead of snapping. Grid quantization scatters correct
    candidates ±half a cell around truth; blending averages that out so the
    EKF sees a smooth track instead of ~50 m grid-point steps.

Why: benchmarked on field_data/survey13 real flight video vs GPS truth
(anyloc/test_vo_fusion_compare.py, logs in anyloc/logs/survey13_vo_fusion*.json,
2026-07-06): this scheme @gate 0.32 gave 13–15 m mean error vs 29–399 m for the
plan-A anchor-chain, whose unconditional re-anchoring can lock onto a wrong DB
entry permanently. Jump gate + blend added 2026-07-06 after a real flight on
this node showed the EKF position teleporting in Mission Planner (false matches
above the score gate + grid-quantized accepts).

The gate is site/database-specific: real-footage scores live in ~0.16–0.34.
Calibrate from a shadow flight's accuracy CSV: set the gate just above the
highest score seen on bad matches (survey13/test20: worst bad match = 0.300).

Run:
  bash anyloc/run_ros2_localizer_vo.sh --headless [--gate 0.32]
      [--jump-base 45] [--drift-rate 1.0] [--blend 0.4] [--reacquire-n 3]
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

DEFAULT_GATE        = 0.32
DEFAULT_JUMP_BASE   = 45.0   # m — covers DB grid half-diagonal (~35 m) + VO error
DEFAULT_DRIFT_RATE  = 1.0    # m/s — ~2× the VO drift measured on survey13 (0.4 m/s)
DEFAULT_BLEND       = 0.4    # fraction of (candidate − position) applied per accept
DEFAULT_REACQUIRE_N = 3      # consecutive agreeing jump-rejects → relocate
AGREE_RADIUS_M      = 30.0   # jump-rejected candidates within this = "agreeing"


class VOPrimaryNode(AnyLocNode):
    """Overrides _cb_image with the VO-primary + score-gated-AnyLoc policy.

    Everything else — subscriptions, publishers, latest_estimate.json,
    accuracy CSV, postview state — is inherited from AnyLocNode.
    """

    def __init__(self, gate: float, jump_base: float = DEFAULT_JUMP_BASE,
                 drift_rate: float = DEFAULT_DRIFT_RATE,
                 blend: float = DEFAULT_BLEND,
                 reacquire_n: int = DEFAULT_REACQUIRE_N,
                 test_mode: bool = False, test_agl: float = 65.0):
        super().__init__(test_mode=test_mode, test_agl=test_agl)
        self._gate        = gate
        self._jump_base   = jump_base
        self._drift_rate  = drift_rate
        self._blend       = blend
        self._reacquire_n = reacquire_n
        self._pos_lat = None    # fused estimate — VO-integrated, AnyLoc-corrected
        self._pos_lon = None
        self._last_accept_t = None   # set at seed, updated on ACC/REACQ
        self._rej_lat   = None       # last jump-rejected candidate (agreement check)
        self._rej_lon   = None
        self._rej_agree = 0          # consecutive agreeing jump-rejects
        self._n_accept   = 0
        self._n_reject   = 0
        self._n_jump_rej = 0
        self._seed_warned = False
        self.get_logger().info(
            f"[VO-primary] fusion: VO every frame, AnyLoc every "
            f"{ANYLOC_INTERVAL} frames accepted iff score ≥ {gate:.2f} and "
            f"jump ≤ {jump_base:.0f} m + {drift_rate:.1f} m/s×dt "
            f"(blend {blend:.2f}, re-acquire after {reacquire_n} agreeing rejects)")

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
            self._last_accept_t = time.time()
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

        # AnyLoc candidate every ANYLOC_INTERVAL frames — gate on score, then
        # on jump plausibility (see module docstring).
        t0 = time.perf_counter()
        match_img = None
        score     = self._last_score
        db_idx    = 0
        verdict   = None    # 'ACC' | 'REJ' | 'JREJ' | 'REACQ'
        if run_anyloc:
            result = self._loc.localize(
                pil_img, agl_m=agl_m,
                center_lat=self._pos_lat, center_lon=self._pos_lon,
                radius_m=SEARCH_RADIUS_M)
            if result is None:
                return
            m_lat, m_lon, _, match_img, score, db_idx = result
            self._last_score = score
            now    = time.time()
            counts = lambda: (f"({self._n_accept}✓/{self._n_reject}✗"
                              f"/{self._n_jump_rej}⤫)")
            if score < self._gate:
                verdict = 'REJ'
                self._n_reject += 1
            else:
                jump_m   = _geo_dist_m(self._pos_lat, self._pos_lon,
                                       m_lat, m_lon)
                max_jump = (self._jump_base
                            + self._drift_rate * (now - self._last_accept_t))
                if jump_m <= max_jump:
                    # Blend, don't snap: correct candidates scatter ±half a
                    # grid cell around truth, so pull partway and let
                    # successive corrections average the quantization out.
                    self._pos_lat += self._blend * (m_lat - self._pos_lat)
                    self._pos_lon += self._blend * (m_lon - self._pos_lon)
                    self._last_accept_t = now
                    self._rej_agree = 0
                    self._n_accept += 1
                    verdict = 'ACC'
                    print(f"[VO-primary] AnyLoc ACCEPT score={score:.3f} "
                          f"jump={jump_m:.1f} m ×{self._blend:.2f}  {counts()}")
                elif (self._rej_agree > 0
                      and _geo_dist_m(self._rej_lat, self._rej_lon,
                                      m_lat, m_lon) <= AGREE_RADIUS_M
                      and self._rej_agree + 1 >= self._reacquire_n):
                    # N consecutive far candidates agreeing with each other:
                    # not aliasing noise — VO/seed is the wrong one. Relocate
                    # fully (no blend); this is a deliberate one-time reset.
                    self._pos_lat, self._pos_lon = m_lat, m_lon
                    self._last_accept_t = now
                    self._rej_agree = 0
                    self._n_accept += 1
                    verdict = 'REACQ'
                    print(f"[VO-primary] AnyLoc RE-ACQUIRE score={score:.3f} "
                          f"jump={jump_m:.1f} m after {self._reacquire_n} "
                          f"agreeing rejects  {counts()}")
                else:
                    # Score passed but the move is physically implausible for
                    # VO error — false match unless it keeps repeating.
                    if (self._rej_agree > 0
                            and _geo_dist_m(self._rej_lat, self._rej_lon,
                                            m_lat, m_lon) <= AGREE_RADIUS_M):
                        self._rej_agree += 1
                    else:
                        self._rej_agree = 1
                    self._rej_lat, self._rej_lon = m_lat, m_lon
                    self._n_jump_rej += 1
                    verdict = 'JREJ'
                    print(f"[VO-primary] AnyLoc JUMP-REJ score={score:.3f} "
                          f"jump={jump_m:.1f} m > {max_jump:.1f} m "
                          f"(agree {self._rej_agree}/{self._reacquire_n})  "
                          f"{counts()}")
        elapsed_ms = (time.perf_counter() - t0) * 1000.0

        est_lat, est_lon = self._pos_lat, self._pos_lon
        gps_lat, gps_lon = self._gps_lat, self._gps_lon

        self._write_estimate(est_lat, est_lon, drone_alt, agl_m, yaw_deg,
                             score, gps_lat, gps_lon)
        self._publish(est_lat, est_lon, drone_alt)

        err_m = _geo_dist_m(gps_lat, gps_lon, est_lat, est_lon)
        if run_anyloc:
            mode_tag = {'ACC':   'ANYLOC-ACC',   # blended in
                        'REJ':   'ANYLOC-REJ',   # score below gate
                        'JREJ':  'ANYLOC-JREJ',  # score ok, jump implausible
                        'REACQ': 'ANYLOC-REACQ', # agreeing rejects → relocated
                        }[verdict]
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
    parser.add_argument("--jump-base", type=float, default=DEFAULT_JUMP_BASE,
                        help="Jump gate base (m): candidate must lie within "
                             "jump-base + drift-rate×dt of the VO position "
                             f"(default: {DEFAULT_JUMP_BASE:.0f})")
    parser.add_argument("--drift-rate", type=float, default=DEFAULT_DRIFT_RATE,
                        help="Jump gate growth (m/s) since last accepted "
                             f"correction (default: {DEFAULT_DRIFT_RATE:.1f})")
    parser.add_argument("--blend", type=float, default=DEFAULT_BLEND,
                        help="Fraction of (candidate − position) applied per "
                             f"accepted correction (default: {DEFAULT_BLEND:.1f}; "
                             "1.0 = old snap behavior)")
    parser.add_argument("--reacquire-n", type=int, default=DEFAULT_REACQUIRE_N,
                        help="Consecutive agreeing jump-rejects that force a "
                             f"relocation (default: {DEFAULT_REACQUIRE_N})")
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
    node = VOPrimaryNode(gate=args.gate, jump_base=args.jump_base,
                         drift_rate=args.drift_rate, blend=args.blend,
                         reacquire_n=args.reacquire_n,
                         test_mode=args.test, test_agl=args.test_agl)

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
