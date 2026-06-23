#!/usr/bin/env python3
"""
PX4 flight commander (MAVROS2) — external-vision no-GPS, survey mission.

Full mission: 6-strip lawnmower survey of the detection zone west of home,
at 12 m/s / 65 m AGL / ~7.8 min. YOLO detections are projected to world
coordinates via yaw-corrected GSD and logged to detections.csv without
interrupting the survey route.

Key differences from ArduPilot flight_commander.py:
  - No set_gp_origin: PX4 EKF2 auto-sets its local frame origin from first EV pose.
  - No STABILIZE→GUIDED arm dance: stream setpoints ≥2 Hz → OFFBOARD → arm.
  - No NAV_TAKEOFF: climb via OFFBOARD position setpoints (monitored from drone_state).
  - Vision via /mavros/vision_pose/pose_cov (PoseWithCovarianceStamped) so EKF2
    can weight Phase-1 vs Phase-2 VPE correctly.
  - go_to_ned() uses velocity setpoints — PX4 velocity controller closes the loop.

Environment variables:
  HOLDTEST=1         run Phase-3 hold gate (HOLD_AGL m) instead of full mission
  TAKEOFF_ALT=<m>    override mission cruise altitude (default 65.0 m)

Run:
  source /opt/ros/humble/setup.bash
  python3 control/px4_commander.py              # survey mission
  HOLDTEST=1 python3 control/px4_commander.py   # hold-gate only
"""
import json
import math
import os
import sys
import threading
import time

_ROS2_SITE = "/opt/ros/humble/lib/python3.10/site-packages"
if os.path.isdir(_ROS2_SITE) and _ROS2_SITE not in sys.path:
    sys.path.insert(0, _ROS2_SITE)

import rclpy
import rclpy.node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import CommandBool, SetMode

try:
    from vision_msgs.msg import Detection2DArray
    _HAVE_VISION_MSGS = True
except ImportError:
    _HAVE_VISION_MSGS = False

try:
    from sensor_msgs.msg import Image as _RosImage
    from PIL import Image as _PilImage
    _HAVE_PIL = True
except ImportError:
    _HAVE_PIL = False

_SENSOR_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE, depth=10)

# ── Home position ──────────────────────────────────────────────────────────────
HOME_LAT  = 23.450868
HOME_LON  = 120.286135
_HOME_CFG = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         "home_elevation.json")
try:
    with open(_HOME_CFG) as _f:
        HOME_ALT_MSL = float(json.load(_f)["centre_elev_m"])
    print(f"[PX4Cmd] HOME_ALT_MSL = {HOME_ALT_MSL:.1f} m  (from {_HOME_CFG})")
except (FileNotFoundError, KeyError):
    HOME_ALT_MSL = 28.17
    print(f"[PX4Cmd] HOME_ALT_MSL = {HOME_ALT_MSL:.1f} m  (default)")

# ── Mission parameters ─────────────────────────────────────────────────────────
TAKEOFF_ALT          = float(os.environ.get("TAKEOFF_ALT", "65.0"))
HOLD_AGL             = 3.0    # m — Phase-3 gate altitude (HOLDTEST mode)
WAYPOINT_RADIUS      = 60.0   # m — survey waypoint arrival threshold
WAYPOINT_TIMEOUT     = 900.0  # s per waypoint
MIN_LOCALISATION_AGL = 50.0   # m — below this use truth VPE; above, use AnyLoc

SURVEY_SPEED   = 12.0   # m/s — strip cruise speed

COS_LAT   = math.cos(math.radians(HOME_LAT))
M_PER_DEG = 111_320.0

# ── Survey waypoints (north_m, east_m, agl_m relative to home) ────────────────
# 7-strip E-W boustrophedon lawnmower; 91.7 m N-S spacing; 65 m AGL; ~7.36 km.
# Strips run east-west (long axis); enter from east (closest to home), S→N.
# 91.7 m spacing < 125 m swath → 33 m overlap between consecutive strips (no gaps).
# E limits at each N clipped to the 30 m-buffered zone polygon.
#   Strip S (N= 60): partial SE boundary,  399 m  E→W
#   Strip 1 (N=152): full,                 732 m  W→E
#   Strip 2 (N=243): full,                 736 m  E→W
#   Strip 3 (N=335): full,                 740 m  W→E
#   Strip 4 (N=427): full,                 743 m  E→W
#   Strip 5 (N=518): full,                 686 m  W→E  (east limit on north edge)
#   Strip N (N=610): partial NW corner,    177 m  E→W
# T6 diagonal (WP11→WP12) is ~504 m — parity flip: strip 5 exits E, strip N starts E.
SURVEY_WPS = [
    ( 60.0,   -573.0,  TAKEOFF_ALT),  # ENTRY: E end strip S (SE boundary)  → fly W
    ( 60.0,   -972.0,  TAKEOFF_ALT),  # WP01 : W end strip S
    (152.0,  -1288.0,  TAKEOFF_ALT),  # WP02 : W end strip 1 [diag NW]      → fly E
    (152.0,   -556.0,  TAKEOFF_ALT),  # WP03 : E end strip 1
    (243.0,   -539.0,  TAKEOFF_ALT),  # WP04 : E end strip 2                → fly W
    (243.0,  -1275.0,  TAKEOFF_ALT),  # WP05 : W end strip 2
    (335.0,  -1261.0,  TAKEOFF_ALT),  # WP06 : W end strip 3                → fly E
    (335.0,   -521.0,  TAKEOFF_ALT),  # WP07 : E end strip 3
    (427.0,   -504.0,  TAKEOFF_ALT),  # WP08 : E end strip 4                → fly W
    (427.0,  -1247.0,  TAKEOFF_ALT),  # WP09 : W end strip 4
    (518.0,  -1234.0,  TAKEOFF_ALT),  # WP10 : W end strip 5                → fly E
    (518.0,   -548.0,  TAKEOFF_ALT),  # WP11 : E end strip 5 [long T6 diag NW]
    (610.0,  -1043.0,  TAKEOFF_ALT),  # WP12 : E end strip N                → fly W
    (610.0,  -1220.0,  TAKEOFF_ALT),  # WP13 : W end strip N
]

# ── Detection zone — buffered boundary (30 m inward from raw corners) ──────────
# CW vertices in (north_m, east_m): NW'→NE'→SE'→SW'
ZONE_VERTS = [
    (642.0, -1215.0),   # NW'
    (507.0,  -489.0),   # NE'
    (-13.0,  -587.0),   # SE'
    (121.0, -1293.0),   # SW'
]

# Camera parameters (AP-IMX900-Mini-USB3-I5 at 1280×960 publish resolution)
CAM_W     = 1024
CAM_H     = 768
HFOV_DEG  = 88.0
VFOV_DEG  = 65.1

VEHICLE_CLASSES = {"car", "van", "truck", "bus"}

DET_LOG = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "detections.csv"
)
CROP_DIR     = os.path.join(os.path.dirname(DET_LOG), "det_crops")
DEDUP_RADIUS = 5.0   # metres — skip re-logging the same vehicle within this distance

ESTIMATE_JSON = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "anyloc", "latest_estimate.json"
)


# ── Zone boundary helper ───────────────────────────────────────────────────────
def _in_buffered_zone(north_m, east_m):
    """Ray-casting point-in-polygon test against the buffered zone boundary."""
    verts = ZONE_VERTS
    n = len(verts)
    inside = False
    j = n - 1
    for i in range(n):
        ni, ei = verts[i]
        nj, ej = verts[j]
        if ((ei > east_m) != (ej > east_m)) and \
           (north_m < (nj - ni) * (east_m - ei) / (ej - ei) + ni):
            inside = not inside
        j = i
    return inside


class PX4Commander(rclpy.node.Node):
    def __init__(self):
        super().__init__("px4_commander")
        self._state     = State()
        self._local_pos = None   # /mavros/local_position/pose  (ENU, from EKF2)
        self._local_vel = None   # /mavros/local_position/velocity_local (ENU)
        self._drone     = None   # /drone/state  (ENU, kinematic truth)

        self._latest_frame    = None  # cached PIL image for crop saving
        self._det_count       = 0     # monotonic crop file index
        self._logged_positions = []   # (north_m, east_m) list for dedup

        # Subscribers
        from geometry_msgs.msg import PoseStamped
        self.create_subscription(State, "/mavros/state", self._cb_state, 10)
        self.create_subscription(PoseStamped, "/mavros/local_position/pose",
                                 self._cb_local, _SENSOR_QOS)
        self.create_subscription(TwistStamped, "/mavros/local_position/velocity_local",
                                 self._cb_vel, _SENSOR_QOS)
        self.create_subscription(PoseStamped, "/drone/state",
                                 self._cb_drone, _SENSOR_QOS)

        if _HAVE_VISION_MSGS:
            self.create_subscription(Detection2DArray, "/yolo/detections",
                                     self._cb_detections, _SENSOR_QOS)
            self.get_logger().info("YOLO detection subscriber active")
        else:
            self.get_logger().warn("vision_msgs not found — YOLO detection disabled")

        if _HAVE_PIL:
            _img_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                  durability=DurabilityPolicy.VOLATILE, depth=1)
            self.create_subscription(_RosImage, "/drone/camera/image_raw",
                                     self._cb_image, _img_qos)
        else:
            self.get_logger().warn("PIL not available — detection crops will not be saved")

        # Publishers
        self._vpe_pub  = self.create_publisher(
            PoseWithCovarianceStamped, "/mavros/vision_pose/pose_cov", 1)
        self._vspd_pub = self.create_publisher(
            TwistStamped, "/mavros/vision_speed/speed_twist", 1)
        self._sp_pub   = self.create_publisher(
            PositionTarget, "/mavros/setpoint_raw/local", 1)

        # Service clients
        self._arm_cli  = self.create_client(CommandBool, "/mavros/cmd/arming")
        self._mode_cli = self.create_client(SetMode,     "/mavros/set_mode")

        self.get_logger().info("PX4 commander ready")

    # ── Callbacks ──────────────────────────────────────────────────────────────
    def _cb_state(self, m):  self._state     = m
    def _cb_local(self, m):  self._local_pos = m
    def _cb_vel(self, m):    self._local_vel = m
    def _cb_drone(self, m):  self._drone     = m

    def _cb_image(self, msg):
        try:
            self._latest_frame = _PilImage.frombytes(
                "RGB", (msg.width, msg.height), bytes(msg.data))
        except Exception:
            pass

    def _cb_detections(self, msg):
        """
        Project YOLO detections to world coordinates via yaw-corrected GSD and log
        to detections.csv. Does not alter the flight path.
        """
        if self._drone is None:
            return

        vehicles = [d for d in msg.detections
                    if d.results and
                       d.results[0].hypothesis.class_id in VEHICLE_CLASSES]
        if not vehicles:
            return

        ds    = self._drone.pose.position
        cur_n = ds.y
        cur_e = ds.x
        agl   = max(1.0, ds.z - HOME_ALT_MSL)

        gsd_x = 2.0 * agl * math.tan(math.radians(HFOV_DEG / 2.0)) / CAM_W
        gsd_y = 2.0 * agl * math.tan(math.radians(VFOV_DEG / 2.0)) / CAM_H

        best = max(vehicles, key=lambda d: d.results[0].hypothesis.score)
        cx = best.bbox.center.position.x
        cy = best.bbox.center.position.y

        # Yaw-corrected pixel-to-ENU projection.
        # Camera top always follows drone nose; h = NED heading (CW from north).
        q       = self._drone.pose.orientation
        yaw_enu = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y ** 2 + q.z ** 2))
        h    = -yaw_enu
        dx_m =  (cx - CAM_W / 2.0) * gsd_x   # drone-right  (+east when h=0)
        dy_m = -(cy - CAM_H / 2.0) * gsd_y   # drone-forward (+north when h=0)
        de   = dx_m * math.cos(h) + dy_m * math.sin(h)
        dn   = -dx_m * math.sin(h) + dy_m * math.cos(h)

        obj_n = cur_n + dn
        obj_e = cur_e + de
        cat   = best.results[0].hypothesis.class_id
        conf  = best.results[0].hypothesis.score

        for pn, pe in self._logged_positions:
            if math.hypot(obj_n - pn, obj_e - pe) < DEDUP_RADIUS:
                return

        bbox = (cx, cy, best.bbox.size_x, best.bbox.size_y)
        self._log_detection(cat, conf, obj_n, obj_e, agl, bbox=bbox)
        print(f"[PX4Cmd] {cat} conf={conf:.2f}  N={obj_n:+.1f} E={obj_e:+.1f}"
              f"  (Δn={dn:+.1f} Δe={de:+.1f} m)")

    def _log_detection(self, category, confidence, north_m, east_m, agl_m, bbox=None):
        """Append one row to detections.csv and save crop image."""
        lat = HOME_LAT + north_m / M_PER_DEG
        lon = HOME_LON + east_m  / (M_PER_DEG * COS_LAT)

        crop_path = ""
        if _HAVE_PIL and bbox is not None and self._latest_frame is not None:
            try:
                cx, cy, bw, bh = bbox
                pad = 20
                x1 = max(0, int(cx - bw / 2) - pad)
                y1 = max(0, int(cy - bh / 2) - pad)
                x2 = min(self._latest_frame.width,  int(cx + bw / 2) + pad)
                y2 = min(self._latest_frame.height, int(cy + bh / 2) + pad)
                crop = self._latest_frame.crop((x1, y1, x2, y2))
                os.makedirs(CROP_DIR, exist_ok=True)
                crop_path = os.path.join(CROP_DIR, f"det_{self._det_count:03d}.jpg")
                crop.save(crop_path, "JPEG", quality=90)
            except Exception as e:
                print(f"[PX4Cmd] crop save failed: {e}")
                crop_path = ""

        need_header = not os.path.exists(DET_LOG)
        with open(DET_LOG, "a") as f:
            if need_header:
                f.write("timestamp,category,confidence,lat,lon,agl_m,crop_path\n")
            f.write(f"{time.time():.3f},{category},{confidence:.3f},"
                    f"{lat:.6f},{lon:.6f},{agl_m:.1f},{crop_path}\n")
        self._logged_positions.append((north_m, east_m))
        self._det_count += 1
        print(f"[PX4Cmd] logged: {category} conf={confidence:.2f}"
              f"  lat={lat:.6f} lon={lon:.6f}  agl={agl_m:.1f} m")

    # ── Vision injection thread ────────────────────────────────────────────────
    def start_vision(self, stop):
        """
        20 Hz background thread: publish VPE + velocity to MAVROS → PX4 EKF2.

        Two-phase strategy:
          Phase 1 (AGL < MIN_LOCALISATION_AGL):
            position = drone_state kinematic truth, cov_xy = 0.1 m²
          Phase 2 (AGL ≥ MIN_LOCALISATION_AGL):
            position = AnyLoc estimate from latest_estimate.json, cov_xy = err_m²

        Heading quaternion: ENU yaw = π/2 (North) in both phases.
        /drone/pose encodes -_kyaw_rad (not π/2-_kyaw_rad), so the kinematic
        yaw_deg=0 maps to East, not North. Since the drone never yaws, π/2 is
        always correct and keeps Phase 1→2 VPE yaw continuous.
        MAVROS converts ENU yaw=π/2 → NED yaw=0 (North).
        """
        def loop():
            last_ds      = None
            anyloc_est   = None
            last_mtime   = 0.0
            n_sent       = 0
            phase_logged = False

            while not stop.is_set():
                t0 = time.time()

                agl = 0.0
                if self._local_pos is not None:
                    agl = max(0.0, self._local_pos.pose.position.z)

                drone_agl = 0.0
                if self._drone is not None:
                    drone_agl = max(0.0, self._drone.pose.position.z - HOME_ALT_MSL)

                if drone_agl >= MIN_LOCALISATION_AGL:
                    if not phase_logged:
                        print(f"[PX4Cmd] AGL {drone_agl:.0f} m ≥ {MIN_LOCALISATION_AGL:.0f} m"
                              " — VPE → AnyLoc")
                        phase_logged = True
                    try:
                        mtime = os.path.getmtime(ESTIMATE_JSON)
                        if mtime != last_mtime:
                            with open(ESTIMATE_JSON) as fh:
                                est = json.load(fh)
                            err_m = est.get("error_m", 999.0)
                            if (est.get("agl_m", 0.0) >= MIN_LOCALISATION_AGL
                                    and err_m < 100.0):
                                lat  = est["est_lat"]; lon = est["est_lon"]
                                yaw  = math.pi / 2.0
                                n_v  = (lat - HOME_LAT) * M_PER_DEG
                                e_v  = (lon - HOME_LON) * M_PER_DEG * COS_LAT
                                cov  = max(1.0, err_m ** 2)
                                anyloc_est = (e_v, n_v, yaw, cov)
                                last_mtime = mtime
                                if n_sent < 2:
                                    print(f"[PX4Cmd] AnyLoc VPE: N={n_v:+.1f}"
                                          f" E={e_v:+.1f} m  err={err_m:.1f} m")
                    except (FileNotFoundError, KeyError, json.JSONDecodeError):
                        pass

                if agl >= MIN_LOCALISATION_AGL and anyloc_est is not None:
                    east_v, north_v, yaw_v, cov_xy = anyloc_est
                else:
                    if self._drone is not None:
                        east_v  = self._drone.pose.position.x
                        north_v = self._drone.pose.position.y
                    else:
                        east_v, north_v = 0.0, 0.0
                    yaw_v  = math.pi / 2.0
                    cov_xy = 0.1

                hy  = yaw_v / 2.0
                msg = PoseWithCovarianceStamped()
                msg.header.stamp    = self.get_clock().now().to_msg()
                msg.header.frame_id = "map"
                msg.pose.pose.position.x    = east_v
                msg.pose.pose.position.y    = north_v
                msg.pose.pose.position.z    = drone_agl
                msg.pose.pose.orientation.z = math.sin(hy)
                msg.pose.pose.orientation.w = math.cos(hy)
                cov = [0.0] * 36
                cov[0]  = cov_xy; cov[7]  = cov_xy; cov[14] = 0.25
                cov[21] = 0.09;   cov[28] = 0.09;   cov[35] = 0.09
                msg.pose.covariance = cov
                self._vpe_pub.publish(msg)
                n_sent += 1
                if n_sent == 1:
                    print("[PX4Cmd] vision thread started (Phase 1 — truth)")

                if self._drone is not None:
                    ds = self._drone.pose.position
                    now_t = time.time()
                    if last_ds is not None:
                        dt_v = now_t - last_ds[3]
                        if dt_v > 1e-3:
                            tw = TwistStamped()
                            tw.header.stamp    = msg.header.stamp
                            tw.header.frame_id = "map"
                            tw.twist.linear.x  = (ds.x - last_ds[0]) / dt_v
                            tw.twist.linear.y  = (ds.y - last_ds[1]) / dt_v
                            tw.twist.linear.z  = (ds.z - last_ds[2]) / dt_v
                            self._vspd_pub.publish(tw)
                    last_ds = (ds.x, ds.y, ds.z, now_t)

                elapsed = time.time() - t0
                time.sleep(max(0.0, 0.05 - elapsed))

        t = threading.Thread(target=loop, daemon=True)
        t.start()
        return t

    # ── Helpers ────────────────────────────────────────────────────────────────
    def _spin_until(self, cond, timeout):
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if cond():
                return True
        return False

    def set_mode(self, mode, timeout=8.0):
        req = SetMode.Request(); req.custom_mode = mode
        fut = self._mode_cli.call_async(req)
        end = time.time() + timeout
        while not fut.done() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        ok = fut.done() and fut.result().mode_sent
        self.get_logger().info(f"set_mode {mode}: {'✓' if ok else 'FAIL'}")
        return ok

    def arm(self, value=True, timeout=8.0):
        req = CommandBool.Request(); req.value = value
        fut = self._arm_cli.call_async(req)
        end = time.time() + timeout
        while not fut.done() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        ok = fut.done() and fut.result().success
        self.get_logger().info(f"{'arm' if value else 'disarm'}: {'✓' if ok else 'FAIL'}")
        return ok

    def _agl(self):
        if self._drone is not None:
            return self._drone.pose.position.z - HOME_ALT_MSL
        if self._local_pos is not None:
            return self._local_pos.pose.position.z
        return 0.0

    def make_sp(self, east, north, up):
        """
        Build a position PositionTarget in ENU.

        MAVROS2 applies ENU→NED for setpoint_raw/local regardless of FRAME_LOCAL_NED.
        Send x=East, y=North, z=Up(AGL); MAVROS converts to NED: x=North, y=East, z=-Up.
        """
        sp = PositionTarget()
        sp.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        sp.type_mask = (PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY |
                        PositionTarget.IGNORE_VZ | PositionTarget.IGNORE_AFX |
                        PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
                        PositionTarget.IGNORE_YAW | PositionTarget.IGNORE_YAW_RATE)
        sp.position.x = float(east)
        sp.position.y = float(north)
        sp.position.z = float(up)
        return sp

    # ── Pre-stream + switch to OFFBOARD + arm ──────────────────────────────────
    def engage_offboard(self, east, north, up, n_pre=40):
        sp = self.make_sp(east, north, up)
        for _ in range(n_pre):
            sp.header.stamp = self.get_clock().now().to_msg()
            self._sp_pub.publish(sp)
            rclpy.spin_once(self, timeout_sec=0.05)

        if not self.set_mode("OFFBOARD"):
            return False

        settle_end = time.time() + 2.0
        while time.time() < settle_end:
            sp.header.stamp = self.get_clock().now().to_msg()
            self._sp_pub.publish(sp)
            rclpy.spin_once(self, timeout_sec=0.05)

        for attempt in range(10):
            if self.arm():
                return True
            print(f"[PX4Cmd] arm attempt {attempt+1}/10 failed — retrying in 1 s …")
            retry_end = time.time() + 1.0
            while time.time() < retry_end:
                sp.header.stamp = self.get_clock().now().to_msg()
                self._sp_pub.publish(sp)
                rclpy.spin_once(self, timeout_sec=0.05)

        return False

    # ── Takeoff ────────────────────────────────────────────────────────────────
    def takeoff(self, alt_agl, timeout=180.0):
        self.get_logger().info(f"Climbing to {alt_agl:.0f} m AGL …")
        e0 = self._drone.pose.position.x if self._drone else 0.0
        n0 = self._drone.pose.position.y if self._drone else 0.0
        sp = self.make_sp(e0, n0, alt_agl)

        deadline   = time.time() + timeout
        last_print = time.time()

        while time.time() < deadline:
            sp.header.stamp = self.get_clock().now().to_msg()
            self._sp_pub.publish(sp)
            rclpy.spin_once(self, timeout_sec=0.05)

            agl = self._agl()
            now = time.time()
            if now - last_print > 3.0:
                print(f"[PX4Cmd] AGL={agl:.1f} m  target={alt_agl:.0f} m"
                      f"  mode={self._state.mode}  armed={self._state.armed}")
                last_print = now

            if agl >= alt_agl - 2.0:
                self.get_logger().info(f"Reached {alt_agl:.0f} m AGL ✓")
                return True

        self.get_logger().warn("Takeoff timeout")
        return False

    # ── Waypoint navigation ────────────────────────────────────────────────────
    def go_to_ned(self, north, east, agl, timeout=WAYPOINT_TIMEOUT,
                  speed=5.0, radius=None):
        """
        Fly to (north, east, agl) via OFFBOARD velocity setpoints.
        MAVROS converts ENU velocity → NED; PX4 velocity controller closes the loop.

        speed   horizontal cruise speed in m/s (default 5.0; survey uses 12.0)
        radius  arrival distance in m (default WAYPOINT_RADIUS = 60 m)

        Returns True when within radius of target; False on timeout.
        """
        NAV_SPEED_V = 2.0
        ALT_KP      = 0.4
        arrival_r   = radius if radius is not None else WAYPOINT_RADIUS

        _VMASK = (PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY |
                  PositionTarget.IGNORE_PZ |
                  PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY |
                  PositionTarget.IGNORE_AFZ |
                  PositionTarget.IGNORE_YAW | PositionTarget.IGNORE_YAW_RATE)

        deadline   = time.time() + timeout
        last_print = time.time()

        while time.time() < deadline:
            if self._drone is not None:
                ds = self._drone.pose.position
                cur_e, cur_n = ds.x, ds.y
                drone_agl    = ds.z - HOME_ALT_MSL
            elif self._local_pos is not None:
                p = self._local_pos.pose.position
                cur_e, cur_n = p.x, p.y
                drone_agl    = p.z
            else:
                rclpy.spin_once(self, timeout_sec=0.1)
                continue

            dx = cur_e - east
            dy = cur_n - north
            hdist = math.hypot(dx, dy)

            spd = min(speed, hdist)
            if hdist > 0.5:
                v_e = -dx / hdist * spd
                v_n = -dy / hdist * spd
            else:
                v_e = v_n = 0.0

            v_up = max(-NAV_SPEED_V, min(NAV_SPEED_V, ALT_KP * (agl - drone_agl)))

            sp = PositionTarget()
            sp.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
            sp.type_mask = _VMASK
            sp.velocity.x = float(v_e)
            sp.velocity.y = float(v_n)
            sp.velocity.z = float(v_up)
            sp.header.stamp = self.get_clock().now().to_msg()
            self._sp_pub.publish(sp)
            rclpy.spin_once(self, timeout_sec=0.05)

            now = time.time()
            if now - last_print > 5.0:
                ekf = ""
                if self._local_pos:
                    lp = self._local_pos.pose.position
                    ekf = f"  EKF=({lp.x:+.0f},{lp.y:+.0f},{lp.z:+.0f})"
                vel_s = ""
                if self._local_vel:
                    lv = self._local_vel.twist.linear
                    vel_s = f"  vm=({lv.x:+.1f},{lv.y:+.1f})"
                print(f"[PX4Cmd] errN={dy:+.1f} errE={dx:+.1f}"
                      f"  AGL={drone_agl:.1f} m  dist={hdist:.1f} m{ekf}{vel_s}")
                last_print = now

            if hdist <= arrival_r:
                return True

        return False


# ── Main ───────────────────────────────────────────────────────────────────────

def main():
    rclpy.init()
    cmd = PX4Commander()
    stop = threading.Event()

    try:
        os.makedirs(os.path.dirname(ESTIMATE_JSON), exist_ok=True)
        with open(ESTIMATE_JSON, "w") as _ef:
            json.dump({"agl_m": -1.0, "error_m": 999.0}, _ef)
    except OSError:
        pass

    cmd.start_vision(stop)

    print("[PX4Cmd] waiting for MAVROS connection …")
    if not cmd._spin_until(lambda: cmd._state.connected, 60.0):
        print("[PX4Cmd] MAVROS not connected — start PX4 + MAVROS first")
        stop.set(); cmd.destroy_node(); rclpy.shutdown(); return
    print("[PX4Cmd] MAVROS connected ✓")

    print("[PX4Cmd] waiting for EKF local position (up to 120 s) …")
    _last_diag = [time.time()]
    def _wait_cond():
        now = time.time()
        if now - _last_diag[0] > 10.0:
            _last_diag[0] = now
            print(f"[PX4Cmd] diag: drone={'OK' if cmd._drone is not None else 'None'}"
                  f"  local_pos={'OK' if cmd._local_pos is not None else 'None'}"
                  f"  t={now-start_t:.0f}s")
        return cmd._local_pos is not None and cmd._drone is not None
    start_t = time.time()
    if not cmd._spin_until(_wait_cond, 120.0):
        print(f"[PX4Cmd] no /drone/state or /mavros/local_position after 120 s")
        stop.set(); cmd.destroy_node(); rclpy.shutdown(); return

    print("[PX4Cmd] waiting for EKF z to converge to ground …")
    def _ekf_near_ground():
        return (cmd._local_pos is not None and
                abs(cmd._local_pos.pose.position.z) < 5.0)
    if not cmd._spin_until(_ekf_near_ground, timeout=15.0):
        ekf_z = cmd._local_pos.pose.position.z if cmd._local_pos else 999.0
        print(f"[PX4Cmd] WARNING: EKF z={ekf_z:.1f} m after 15 s — proceeding anyway")
    else:
        ekf_z = cmd._local_pos.pose.position.z if cmd._local_pos else 0.0
        print(f"[PX4Cmd] EKF z={ekf_z:.2f} m — converged ✓")

    start_agl = cmd._agl()
    in_air = start_agl > 5.0
    if in_air:
        print(f"[PX4Cmd] in-air restart at {start_agl:.0f} m AGL — skipping takeoff")
        if cmd._state.mode != "OFFBOARD":
            e0 = cmd._drone.pose.position.x if cmd._drone else 0.0
            n0 = cmd._drone.pose.position.y if cmd._drone else 0.0
            sp = cmd.make_sp(e0, n0, start_agl)
            for _ in range(40):
                sp.header.stamp = cmd.get_clock().now().to_msg()
                cmd._sp_pub.publish(sp)
                rclpy.spin_once(cmd, timeout_sec=0.05)
            cmd.set_mode("OFFBOARD")

    # HOLDTEST mode: position-hold gate only (Phase 3 test)
    if os.environ.get("HOLDTEST"):
        e0 = cmd._drone.pose.position.x if cmd._drone else 0.0
        n0 = cmd._drone.pose.position.y if cmd._drone else 0.0

        if not in_air:
            if not cmd.engage_offboard(e0, n0, HOLD_AGL):
                print("[PX4Cmd] engage_offboard failed"); stop.set()
                cmd.destroy_node(); rclpy.shutdown(); return

        sp  = cmd.make_sp(e0, n0, HOLD_AGL)
        print(f"[PX4Cmd] === HOLD GATE: {HOLD_AGL:.0f} m AGL for 40 s ===")
        t_end = time.time() + 40.0; t_log = 0.0
        while time.time() < t_end:
            sp.header.stamp = cmd.get_clock().now().to_msg()
            cmd._sp_pub.publish(sp)
            rclpy.spin_once(cmd, timeout_sec=0.02)
            if time.time() - t_log > 3.0 and cmd._drone is not None:
                t_log = time.time()
                ds = cmd._drone.pose.position
                agl = ds.z - HOME_ALT_MSL
                print(f"[PX4Cmd] drift E={ds.x-e0:+6.1f} N={ds.y-n0:+6.1f}"
                      f"  AGL={agl:4.1f}  dist={math.hypot(ds.x-e0,ds.y-n0):5.1f} m"
                      f"  mode={cmd._state.mode} armed={cmd._state.armed}")
        print("[PX4Cmd] === gate done ===")
        stop.set(); cmd.destroy_node(); rclpy.shutdown(); return

    # ── Full survey mission ───────────────────────────────────────────────────
    try:
        if not in_air:
            e0 = cmd._drone.pose.position.x if cmd._drone else 0.0
            n0 = cmd._drone.pose.position.y if cmd._drone else 0.0
            print(f"[PX4Cmd] engaging OFFBOARD at ground  E={e0:+.1f} N={n0:+.1f} …")
            if not cmd.engage_offboard(e0, n0, 0.5):
                print("[PX4Cmd] ABORT: engage_offboard failed")
                stop.set(); cmd.destroy_node(); rclpy.shutdown(); return

            if not cmd.takeoff(TAKEOFF_ALT):
                print("[PX4Cmd] ABORT: takeoff failed")
                stop.set(); cmd.destroy_node(); rclpy.shutdown(); return

        # Hold briefly at cruise altitude before starting survey
        if cmd._drone is not None:
            hold_e = cmd._drone.pose.position.x
            hold_n = cmd._drone.pose.position.y
        else:
            hold_e, hold_n = 0.0, 0.0
        print(f"[PX4Cmd] holding 5 s at {TAKEOFF_ALT:.0f} m AGL …")
        t_hold = time.time() + 5.0
        sp = cmd.make_sp(hold_e, hold_n, TAKEOFF_ALT)
        while time.time() < t_hold:
            sp.header.stamp = cmd.get_clock().now().to_msg()
            cmd._sp_pub.publish(sp)
            rclpy.spin_once(cmd, timeout_sec=0.05)

        # ── Survey loop ───────────────────────────────────────────────────────
        print(f"[PX4Cmd] === SURVEY START  {len(SURVEY_WPS)} waypoints"
              f"  speed={SURVEY_SPEED:.0f} m/s ===")
        wp_idx = 0
        while wp_idx < len(SURVEY_WPS):
            wn, we, wagl = SURVEY_WPS[wp_idx]
            print(f"[PX4Cmd] SURVEY WP {wp_idx+1}/{len(SURVEY_WPS)}"
                  f"  N={wn:+.0f} E={we:+.0f} AGL={wagl:.0f} m")

            reached = cmd.go_to_ned(wn, we, wagl,
                                    timeout=WAYPOINT_TIMEOUT,
                                    speed=SURVEY_SPEED)

            if reached:
                if cmd._drone is not None:
                    ds  = cmd._drone.pose.position
                    dx  = ds.x - we; dy = ds.y - wn
                    print(f"[PX4Cmd] WP {wp_idx+1} ARRIVED ✓"
                          f"  E={ds.x:+.1f} N={ds.y:+.1f}"
                          f"  horiz_err={math.hypot(dx, dy):.1f} m")
            else:
                print(f"[PX4Cmd] WP {wp_idx+1} TIMEOUT — skipping")
            wp_idx += 1

        # Survey complete — fly back to home, then land
        # RTL mode requires a GPS-derived home and is unreliable with external
        # vision only. Stay in OFFBOARD and command home explicitly instead.
        print("[PX4Cmd] === SURVEY COMPLETE — returning home ===")
        cmd.go_to_ned(0.0, 0.0, TAKEOFF_ALT, timeout=300.0, speed=SURVEY_SPEED)
        print("[PX4Cmd] Over home — AUTO.LAND")
        cmd.set_mode("AUTO.LAND")
        cmd._spin_until(lambda: not cmd._state.armed, timeout=150.0)
        print("[PX4Cmd] Disarmed — landed ✓")

    except KeyboardInterrupt:
        print("[PX4Cmd] Ctrl-C — returning home")
        cmd.go_to_ned(0.0, 0.0, TAKEOFF_ALT, timeout=120.0, speed=SURVEY_SPEED)
        cmd.set_mode("AUTO.LAND")
        cmd._spin_until(lambda: not cmd._state.armed, timeout=150.0)
        print("[PX4Cmd] Disarmed ✓")
    except Exception as exc:
        print(f"[PX4Cmd] mission aborted: {exc}")
    finally:
        stop.set()
        try:
            cmd.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
