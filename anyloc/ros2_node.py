#!/usr/bin/env python3
"""
AnyLoc + VO localization as a ROS2 node with live postview.

Subscribes:
  /drone/camera/image_raw  (sensor_msgs/Image, rgb8, 640×480)
  /drone/pose              (geometry_msgs/PoseStamped, frame_id="wgs84",
                            position=(lat,lon,alt_msl), orientation=yaw quat)
  /drone/agl               (std_msgs/Float64, metres above ground)

Publishes:
  /anyloc/pose_estimate    (geometry_msgs/PoseWithCovarianceStamped)

Writes:
  anyloc/latest_estimate.json — read by px4_commander.py Phase 2 VPE thread.
  VPE publishing to MAVROS is intentionally handled by the commander (not here)
  to avoid duplicate EKF2 inputs when both processes run together.

Run:
  source /opt/ros/jazzy/setup.bash
  DISPLAY=:2 conda run -n isaac_sim_test python3 anyloc/ros2_node.py
"""

import json
import math
import os
import sys
import threading
import time

# ROS2 Jazzy site-packages (Python 3.12) — add when running inside conda env
_ROS2_SITE = "/opt/ros/jazzy/lib/python3.12/site-packages"
if os.path.isdir(_ROS2_SITE) and _ROS2_SITE not in sys.path:
    sys.path.insert(0, _ROS2_SITE)

import rclpy
import rclpy.node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import Image
from std_msgs.msg import Float64

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import numpy as np
import torch
from PIL import Image as PILImage, ImageDraw, ImageFont

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from anyloc.localizer import AnyLocLocalizer
from anyloc.vo_refiner import VORefiner

# ── Constants ─────────────────────────────────────────────────────────────────
HOME_LAT     = 23.450868
HOME_LON     = 120.286135
HOME_ALT_MSL = 28.17
COS_LAT      = math.cos(math.radians(HOME_LAT))
M_PER_DEG    = 111_320.0

ANYLOC_INTERVAL = 10
SEARCH_RADIUS_M = 200.0
MIN_AGL         = 50.0   # m — below this AGL skip inference (matches px4_commander Phase 2)

HERE          = os.path.dirname(os.path.abspath(__file__))
DB_PATH       = os.path.join(HERE, "database")
ESTIMATE_JSON = os.path.join(HERE, "latest_estimate.json")


# ── Helpers (identical to run_localizer.py) ───────────────────────────────────

def _pil_overlay(pil_img, lines, text_color='white', bg_alpha=140):
    img  = pil_img.copy().convert('RGBA')
    draw = ImageDraw.Draw(img)
    try:
        font = ImageFont.truetype(
            '/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf', 15)
    except Exception:
        font = ImageFont.load_default()
    line_h  = 20
    pad     = 8
    max_w   = max(draw.textlength(ln, font=font) for ln in lines)
    panel_h = pad + line_h * len(lines) + pad // 2
    panel_w = int(max_w) + pad * 2
    overlay = PILImage.new('RGBA', img.size, (0, 0, 0, 0))
    ov_draw = ImageDraw.Draw(overlay)
    ov_draw.rectangle((0, 0, panel_w, panel_h), fill=(0, 0, 0, bg_alpha))
    img = PILImage.alpha_composite(img, overlay)
    draw2 = ImageDraw.Draw(img)
    for i, ln in enumerate(lines):
        draw2.text((pad, pad + i * line_h), ln, fill=text_color, font=font)
    return img.convert('RGB')


def _pil_to_array(pil_img, size=(640, 480)):
    img = pil_img.resize(size, PILImage.LANCZOS).convert('RGB')
    t   = torch.frombuffer(bytearray(img.tobytes()), dtype=torch.uint8) \
               .reshape(size[1], size[0], 3)
    return t.numpy()


def _geo_dist_m(lat1, lon1, lat2, lon2):
    return math.hypot((lat1 - lat2) * M_PER_DEG,
                      (lon1 - lon2) * M_PER_DEG * COS_LAT)


def _yaw_from_quat(qz, qw):
    return 2.0 * math.atan2(qz, qw)


# ── ROS2 node ─────────────────────────────────────────────────────────────────

class AnyLocNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("anyloc_localizer")

        self._loc = AnyLocLocalizer(DB_PATH)
        self._vo  = VORefiner()

        # Localization state
        self._frame_count  = 0
        self._anchor_lat   = None
        self._anchor_lon   = None
        self._accum_dlat   = 0.0
        self._accum_dlon   = 0.0
        self._last_score   = 0.0   # score from most recent AnyLoc anchor (reused for VO frames)

        # Drone state (updated by pose/agl callbacks)
        self._drone_lat = HOME_LAT
        self._drone_lon = HOME_LON
        self._drone_alt = HOME_ALT_MSL
        self._drone_yaw = 0.0      # radians
        self._drone_agl = 0.0
        self._agl_logged = False   # one-time "AGL reached" print

        # Latest results shared with postview thread
        self.lock         = threading.Lock()
        self.latest_frame  = None   # PIL drone camera image
        self.latest_match  = None   # PIL AnyLoc satellite crop
        self.latest_result = None   # dict with all display fields

        # Subscribers
        self.create_subscription(Image,       "/drone/camera/image_raw", self._cb_image, 1)
        self.create_subscription(PoseStamped, "/drone/pose",             self._cb_pose,  10)
        self.create_subscription(Float64,     "/drone/agl",              self._cb_agl,   10)

        # Publishers
        self.pub_est = self.create_publisher(
            PoseWithCovarianceStamped, "/anyloc/pose_estimate", 1)
        # Note: VPE to MAVROS is handled by px4_commander.py reading latest_estimate.json.
        # Publishing directly to /mavros/vision_pose/pose here caused duplicate VPE when
        # running alongside the commander, confusing EKF2. The file-based interface is the
        # correct integration point.

        self.get_logger().info("AnyLoc node ready — waiting for /drone/camera/image_raw")

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _cb_pose(self, msg):
        self._drone_lat = msg.pose.position.x
        self._drone_lon = msg.pose.position.y
        self._drone_alt = msg.pose.position.z
        self._drone_yaw = _yaw_from_quat(
            msg.pose.orientation.z, msg.pose.orientation.w)

    def _cb_agl(self, msg):
        self._drone_agl = msg.data if msg.data > 0.5 else self._drone_agl

    def _cb_image(self, msg):
        agl_m = self._drone_agl
        if agl_m < MIN_AGL:
            return

        if not self._agl_logged:
            print(f"[AnyLoc] AGL {agl_m:.0f} m ≥ {MIN_AGL:.0f} m — starting inference")
            self._agl_logged = True

        try:
            pil_img = PILImage.frombytes(
                "RGB", (msg.width, msg.height), bytes(msg.data))
        except Exception as e:
            self.get_logger().warn(f"Image decode: {e}")
            return

        agl_m     = self._drone_agl
        yaw_deg   = math.degrees(self._drone_yaw)
        drone_lat = self._drone_lat
        drone_lon = self._drone_lon
        drone_alt = self._drone_alt

        self._frame_count += 1
        run_anyloc = (self._frame_count == 1 or
                      self._frame_count % ANYLOC_INTERVAL == 0)

        # VO every frame — gimbal preserves drone yaw so camera top = drone nose.
        # VORefiner yaw convention: 0 = North-pointing camera (image top=North),
        # which equals the drone's compass bearing (CW degrees from North).
        # /drone/pose encodes orientation as −_kyaw_rad (NED CW), so
        # self._drone_yaw = −_kyaw_rad = −(compass_bearing_rad).
        # Therefore: compass_bearing_deg = −math.degrees(self._drone_yaw).
        # Simulation: _drone_yaw=0 → compass=0° (North-facing) → VO yaw=0. ✓
        _vo_yaw = -math.degrees(self._drone_yaw)
        dlat, dlon, n_vo = self._vo.update(pil_img, agl_m, _vo_yaw)
        if self._anchor_lat is not None:
            self._accum_dlat += dlat
            self._accum_dlon += dlon

        # AnyLoc retrieval every ANYLOC_INTERVAL frames
        t0 = time.perf_counter()
        if run_anyloc:
            clat = (self._anchor_lat + self._accum_dlat
                    if self._anchor_lat is not None else None)
            clon = (self._anchor_lon + self._accum_dlon
                    if self._anchor_lat is not None else None)
            result = self._loc.localize(
                pil_img, agl_m=agl_m,
                center_lat=clat, center_lon=clon,
                radius_m=SEARCH_RADIUS_M if clat is not None else None)
            if result is None:
                return
            est_lat, est_lon, est_alt, match_img, score, db_idx = result
            self._anchor_lat  = est_lat
            self._anchor_lon  = est_lon
            self._accum_dlat  = 0.0
            self._accum_dlon  = 0.0
            self._last_score  = score
            self._vo.reset()
        else:
            match_img = None
            score     = self._last_score
            db_idx    = 0
            est_lat   = (self._anchor_lat + self._accum_dlat
                         if self._anchor_lat is not None else None)
            est_lon   = (self._anchor_lon + self._accum_dlon
                         if self._anchor_lat is not None else None)

        elapsed_ms = (time.perf_counter() - t0) * 1000.0

        if est_lat is None:
            return

        # Write JSON every frame with VO-smoothed position so the commander's
        # VPE thread gets a continuous update stream instead of a stale anchor
        # that jumps every ANYLOC_INTERVAL frames (~2 s), causing EKF2 lurches.
        self._write_estimate(est_lat, est_lon, drone_alt, agl_m, yaw_deg,
                             score, drone_lat, drone_lon)

        self._publish(est_lat, est_lon, drone_alt)

        err_m      = _geo_dist_m(drone_lat, drone_lon, est_lat, est_lon)
        anchor_age = 0 if run_anyloc else (self._frame_count % ANYLOC_INTERVAL)
        mode_tag   = 'ANYLOC' if run_anyloc else f'VO +{anchor_age}f'

        with self.lock:
            self.latest_frame = pil_img
            if match_img is not None:
                self.latest_match = match_img
            self.latest_result = dict(
                drone_lat=drone_lat, drone_lon=drone_lon,
                drone_alt=drone_alt, drone_agl=agl_m,
                drone_yaw=math.degrees(self._drone_yaw),
                est_lat=est_lat, est_lon=est_lon,
                err_m=err_m, score=score, db_idx=db_idx,
                n_vo=n_vo, elapsed_ms=elapsed_ms,
                mode_tag=mode_tag, run_anyloc=run_anyloc,
            )

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _publish(self, lat, lon, alt_msl):
        now = self.get_clock().now().to_msg()
        hy  = self._drone_yaw / 2.0

        est = PoseWithCovarianceStamped()
        est.header.stamp = now; est.header.frame_id = "wgs84"
        est.pose.pose.position.x = lat
        est.pose.pose.position.y = lon
        est.pose.pose.position.z = alt_msl
        est.pose.pose.orientation.z = math.sin(hy)
        est.pose.pose.orientation.w = math.cos(hy)
        cov = [0.0] * 36
        cov[0] = cov[7] = 20.0**2; cov[14] = 5.0**2
        cov[21] = cov[28] = cov[35] = 0.3**2
        est.pose.covariance = cov
        self.pub_est.publish(est)

    def _write_estimate(self, est_lat, est_lon, alt_msl, agl_m,
                        yaw_deg, score, drone_lat, drone_lon):
        est = {
            "timestamp": time.time(),
            "est_lat":   est_lat, "est_lon":  est_lon,
            "alt_msl_m": alt_msl, "agl_m":    agl_m,
            "yaw_deg":   yaw_deg, "score":    float(score),
            "error_m":   float(_geo_dist_m(drone_lat, drone_lon,
                                           est_lat, est_lon)),
        }
        tmp = ESTIMATE_JSON + ".tmp"
        with open(tmp, "w") as fh:
            json.dump(est, fh)
        os.replace(tmp, ESTIMATE_JSON)


# ── Postview (main thread) ────────────────────────────────────────────────────

def run_postview(node: AnyLocNode):
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5.2),
                                   gridspec_kw={'wspace': 0.04},
                                   layout='constrained')
    fig.patch.set_facecolor('#1a1a1a')
    for ax in (ax1, ax2):
        ax.axis('off')
        ax.set_facecolor('#1a1a1a')

    blank = np.zeros((480, 640, 3), dtype=np.uint8)
    im1 = ax1.imshow(blank)
    im2 = ax2.imshow(blank)
    ax1.set_title('Drone Camera', color='white', fontsize=11, pad=4)
    ax2.set_title('AnyLoc+VO',   color='white', fontsize=11, pad=4)
    plt.ion()
    plt.show()

    print("[PostView] Waiting for first frame …  (Ctrl-C or close window to quit)")

    while plt.fignum_exists(fig.number):
        with node.lock:
            frame  = node.latest_frame
            match  = node.latest_match
            result = node.latest_result

        if frame is None or result is None or match is None:
            plt.pause(0.15)
            continue

        r = result
        err_m   = r['err_m']
        color   = '#50ff50' if err_m < 200 else '#5050ff'
        mode    = r['mode_tag']

        v1 = _pil_overlay(frame.resize((640, 480), PILImage.LANCZOS), [
            'DRONE CAMERA',
            f"LAT   {r['drone_lat']:.5f} N",
            f"LON   {r['drone_lon']:.5f} E",
            f"ALT   {r['drone_alt']:.1f} m MSL    AGL {r['drone_agl']:.1f} m",
            f"YAW   {r['drone_yaw']:.1f} deg",
        ], text_color='white')
        im1.set_data(_pil_to_array(v1))

        v2 = _pil_overlay(match.resize((640, 480), PILImage.LANCZOS), [
            f"{mode}   score {r['score']:.3f}   #{r['db_idx']}",
            f"LAT   {r['est_lat']:.5f} N",
            f"LON   {r['est_lon']:.5f} E",
            f"ALT   {r['drone_agl']:.1f} m AGL",
            f"ERR   {err_m:.0f} m    VO pts {r['n_vo']}    {r['elapsed_ms']:.0f} ms",
        ], text_color=color)
        im2.set_data(_pil_to_array(v2))

        ax2.set_title(f'AnyLoc+VO [{mode}]  —  ERR {err_m:.0f} m',
                      color=color, fontsize=11, pad=4)
        fig.canvas.draw_idle()
        plt.pause(0.15)

    print("[PostView] Closed.")


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    rclpy.init()
    node = AnyLocNode()

    # ROS2 spin in background thread so matplotlib can own the main thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        run_postview(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
