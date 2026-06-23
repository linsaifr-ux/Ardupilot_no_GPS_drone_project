#!/usr/bin/env python3
"""
ArduPilot AUTO-mode survey mission (MAVROS2) — alternative to ardupilot_commander.py.

Approach: upload the 18-WP lawnmower as a MAVLink mission, switch to AUTO mode,
let ArduPilot's built-in L1 nav controller handle cross-track following and speed
management.  Monitor progress via /mavros/mission/reached; when the final WP is
reached switch back to GUIDED and RTL home manually (GPS-free RTL is unsafe).

Differences from ardupilot_commander.py (GUIDED velocity-setpoint approach):
  - No custom go_to_ned() loop during the survey.
  - No WAYPOINT_DECEL / CORNER_RADIUS hacks — ArduPilot L1 handles decel natively.
  - Per-WP acceptance radius via NAV_WAYPOINT param2 (ArduCopter ≥3.6).
  - CORNER_WP acceptance radius starts at 60 m (conservative — reduce after testing).
  - go_to_ned() is kept only for the post-survey RTL leg.
  - Overall survey timeout (1800 s) replaces per-WP timeout.

VPE injection, EKF origin, detection logging, and arming sequence are unchanged.

Run:
  source /opt/ros/jazzy/setup.bash
  python3 control/ardupilot_auto_mission.py
"""
import json
import math
import os
import struct
import sys
import threading
import time

_ROS2_SITE = "/opt/ros/jazzy/lib/python3.12/site-packages"
if os.path.isdir(_ROS2_SITE) and _ROS2_SITE not in sys.path:
    sys.path.insert(0, _ROS2_SITE)

import rclpy
import rclpy.node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geographic_msgs.msg import GeoPointStamped
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped
from mavros_msgs.msg import Mavlink, PositionTarget, State, Waypoint, WaypointReached
from mavros_msgs.srv import (CommandBool, CommandLong, CommandTOL, SetMode,
                              WaypointClear, WaypointPush, WaypointSetCurrent)

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
    print(f"[AutoMission] HOME_ALT_MSL = {HOME_ALT_MSL:.1f} m  (from {_HOME_CFG})")
except (FileNotFoundError, KeyError):
    HOME_ALT_MSL = 28.17
    print(f"[AutoMission] HOME_ALT_MSL = {HOME_ALT_MSL:.1f} m  (default)")

COS_LAT   = math.cos(math.radians(HOME_LAT))
M_PER_DEG = 111_320.0

# ── Mission parameters ─────────────────────────────────────────────────────────
TAKEOFF_ALT          = float(os.environ.get("TAKEOFF_ALT", "65.0"))
SURVEY_SPEED         = 12.0    # m/s — WPNAV_SPEED in no_gps.parm must match (1200 cm/s)
SURVEY_TIMEOUT       = 1800.0  # s — total mission wall-clock timeout

# Per-waypoint acceptance radii written into NAV_WAYPOINT param2.
# ArduCopter ≥3.6 honours param2 as a per-WP override of WP_RADIUS.
# CORNER values start conservative; reduce after first successful AUTO test.
WP_ACCEPTANCE_RADIUS   = 8.0   # m — strip waypoints (L1 is more precise than GUIDED)
CORNER_ACCEPTANCE_RADIUS = 60.0 # m — TURN-N corners (same safe value used in GUIDED tests)

# RTL leg after mission — still uses go_to_ned() in GUIDED mode (GPS RTL is unsafe)
RTL_TIMEOUT   = 300.0  # s
RTL_SPEED     = SURVEY_SPEED

# 0-indexed positions of TURN-N corner waypoints in SURVEY_WPS
CORNER_WP_INDICES = {2, 5, 7, 10, 12, 15}

MIN_LOCALISATION_AGL = 50.0   # m — below: truth VPE; above: AnyLoc

# ── Survey waypoints (north_m, east_m, agl_m relative to home) ────────────────
# Identical to ardupilot_commander.py — 18 WPs, 7-strip E-W boustrophedon.
SURVEY_WPS = [
    ( 60.0,   -573.0,  TAKEOFF_ALT),  # 0: ENTRY  : E end strip S  → fly W
    ( 60.0,   -972.0,  TAKEOFF_ALT),  # 1: WP01   : W end strip S
    (152.0,   -972.0,  TAKEOFF_ALT),  # 2: TURN-N (CORNER)
    (152.0,  -1288.0,  TAKEOFF_ALT),  # 3: WP02   : W end strip 1
    (152.0,   -556.0,  TAKEOFF_ALT),  # 4: WP03   : E end strip 1
    (243.0,   -556.0,  TAKEOFF_ALT),  # 5: TURN-N (CORNER)
    (243.0,  -1275.0,  TAKEOFF_ALT),  # 6: WP05   : W end strip 2
    (335.0,  -1275.0,  TAKEOFF_ALT),  # 7: TURN-N (CORNER)
    (335.0,  -1261.0,  TAKEOFF_ALT),  # 8: WP06   : W end strip 3 (14 m from TURN-N)
    (335.0,   -521.0,  TAKEOFF_ALT),  # 9: WP07   : E end strip 3
    (427.0,   -521.0,  TAKEOFF_ALT),  # 10: TURN-N (CORNER)
    (427.0,  -1247.0,  TAKEOFF_ALT),  # 11: WP09  : W end strip 4
    (518.0,  -1247.0,  TAKEOFF_ALT),  # 12: TURN-N (CORNER)
    (518.0,  -1234.0,  TAKEOFF_ALT),  # 13: WP10  : W end strip 5 (13 m from TURN-N)
    (518.0,   -548.0,  TAKEOFF_ALT),  # 14: WP11  : E end strip 5
    (610.0,   -548.0,  TAKEOFF_ALT),  # 15: TURN-N (CORNER)
    (610.0,  -1043.0,  TAKEOFF_ALT),  # 16: WP12  : mid strip N
    (610.0,  -1220.0,  TAKEOFF_ALT),  # 17: WP13  : W end strip N
]

# ── Detection zone — buffered boundary (30 m inward from raw corners) ──────────
ZONE_VERTS = [
    (642.0, -1215.0),   # NW'
    (507.0,  -489.0),   # NE'
    (-13.0,  -587.0),   # SE'
    (121.0, -1293.0),   # SW'
]

# Camera parameters (AP-IMX900-Mini-USB3-I5 at 1024×768 publish resolution)
CAM_W    = 1024
CAM_H    = 768
HFOV_DEG = 88.0
VFOV_DEG = 65.1

VEHICLE_CLASSES = {"car", "van", "truck", "bus"}

DET_LOG = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "detections.csv"
)
CROP_DIR     = os.path.join(os.path.dirname(DET_LOG), "det_crops")
DEDUP_RADIUS = 5.0   # m

ESTIMATE_JSON = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "anyloc", "latest_estimate.json"
)


# ── Helpers ────────────────────────────────────────────────────────────────────
def _in_buffered_zone(north_m, east_m):
    verts = ZONE_VERTS
    n = len(verts); inside = False; j = n - 1
    for i in range(n):
        ni, ei = verts[i]; nj, ej = verts[j]
        if ((ei > east_m) != (ej > east_m)) and \
           (north_m < (nj - ni) * (east_m - ei) / (ej - ei) + ni):
            inside = not inside
        j = i
    return inside


def ned_to_latlon(north_m, east_m):
    """Convert NED offset from home to (lat, lon) using the EKF origin."""
    return (HOME_LAT + north_m / M_PER_DEG,
            HOME_LON + east_m  / (M_PER_DEG * COS_LAT))


# ── Commander class ────────────────────────────────────────────────────────────
class ArduPilotAutoCommander(rclpy.node.Node):
    def __init__(self):
        super().__init__("ardupilot_auto_mission")
        self._state     = State()
        self._local_pos = None
        self._local_vel = None
        self._drone     = None

        self._ekf_flags           = 0
        self._gps_origin_received = False

        self._latest_frame     = None
        self._det_count        = 0
        self._logged_positions = []

        # Mission progress — updated by /mavros/mission/reached callback.
        # Value is the 0-based mission item index last confirmed reached.
        # Index 0 is always the home item; survey WPs are indices 1..18.
        self._reached_wp = -1

        from geometry_msgs.msg import PoseStamped
        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(State,            "/mavros/state",
                                 self._cb_state, 10)
        self.create_subscription(PoseStamped,      "/mavros/local_position/pose",
                                 self._cb_local, _SENSOR_QOS)
        self.create_subscription(TwistStamped,     "/mavros/local_position/velocity_local",
                                 self._cb_vel,   _SENSOR_QOS)
        self.create_subscription(PoseStamped,      "/drone/state",
                                 self._cb_drone, _SENSOR_QOS)
        self.create_subscription(Mavlink,          "/uas1/mavlink_source",
                                 self._cb_mavlink, _SENSOR_QOS)
        self.create_subscription(WaypointReached,  "/mavros/mission/reached",
                                 self._cb_reached, 10)

        if _HAVE_VISION_MSGS:
            self.create_subscription(Detection2DArray, "/yolo/detections",
                                     self._cb_detections, _SENSOR_QOS)
        if _HAVE_PIL:
            _img_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                  durability=DurabilityPolicy.VOLATILE, depth=1)
            self.create_subscription(_RosImage, "/drone/camera/image_raw",
                                     self._cb_image, _img_qos)

        # ── Publishers ────────────────────────────────────────────────────────
        self._vpe_pub    = self.create_publisher(
            PoseWithCovarianceStamped, "/mavros/vision_pose/pose_cov", 1)
        self._vspd_pub   = self.create_publisher(
            TwistStamped, "/mavros/vision_speed/speed_twist", 1)
        self._sp_pub     = self.create_publisher(
            PositionTarget, "/mavros/setpoint_raw/local", 1)
        self._origin_pub = self.create_publisher(
            GeoPointStamped, "/mavros/global_position/set_gp_origin", 1)

        # ── Service clients ───────────────────────────────────────────────────
        self._arm_cli      = self.create_client(CommandBool,       "/mavros/cmd/arming")
        self._mode_cli     = self.create_client(SetMode,           "/mavros/set_mode")
        self._tof_cli      = self.create_client(CommandTOL,        "/mavros/cmd/takeoff")
        self._cmd_cli      = self.create_client(CommandLong,       "/mavros/cmd/command")
        self._wp_push_cli  = self.create_client(WaypointPush,      "/mavros/mission/push")
        self._wp_clear_cli = self.create_client(WaypointClear,     "/mavros/mission/clear")
        self._wp_cur_cli   = self.create_client(WaypointSetCurrent,"/mavros/mission/set_current")

        self.get_logger().info("ArduPilot AUTO-mission commander ready")

    # ── Callbacks ──────────────────────────────────────────────────────────────
    def _cb_state(self, m):   self._state     = m
    def _cb_local(self, m):   self._local_pos = m
    def _cb_vel(self, m):     self._local_vel = m
    def _cb_drone(self, m):   self._drone     = m

    def _cb_reached(self, msg: WaypointReached):
        self._reached_wp = msg.wp_seq
        print(f"[AutoMission] mission WP {msg.wp_seq} REACHED")

    def _cb_mavlink(self, msg: Mavlink) -> None:
        raw = b"".join(x.to_bytes(8, "little") for x in msg.payload64)
        if msg.msgid == 193 and len(raw) >= 22:
            self._ekf_flags = struct.unpack_from("<H", raw, 20)[0]
        elif msg.msgid == 49:
            self._gps_origin_received = True

    def _cb_image(self, msg):
        try:
            self._latest_frame = _PilImage.frombytes(
                "RGB", (msg.width, msg.height), bytes(msg.data))
        except Exception:
            pass

    def _cb_detections(self, msg):
        if self._drone is None:
            return
        vehicles = [d for d in msg.detections
                    if d.results and
                       d.results[0].hypothesis.class_id in VEHICLE_CLASSES]
        if not vehicles:
            return
        ds    = self._drone.pose.position
        cur_n = ds.y; cur_e = ds.x
        agl   = max(1.0, ds.z - HOME_ALT_MSL)
        gsd_x = 2.0 * agl * math.tan(math.radians(HFOV_DEG / 2.0)) / CAM_W
        gsd_y = 2.0 * agl * math.tan(math.radians(VFOV_DEG / 2.0)) / CAM_H
        best  = max(vehicles, key=lambda d: d.results[0].hypothesis.score)
        cx    = best.bbox.center.position.x
        cy    = best.bbox.center.position.y
        q     = self._drone.pose.orientation
        yaw_enu = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y ** 2 + q.z ** 2))
        h  = -yaw_enu
        dx_m =  (cx - CAM_W / 2.0) * gsd_x
        dy_m = -(cy - CAM_H / 2.0) * gsd_y
        de   = dx_m * math.cos(h) + dy_m * math.sin(h)
        dn   = -dx_m * math.sin(h) + dy_m * math.cos(h)
        obj_n = cur_n + dn; obj_e = cur_e + de
        cat   = best.results[0].hypothesis.class_id
        conf  = best.results[0].hypothesis.score
        for pn, pe in self._logged_positions:
            if math.hypot(obj_n - pn, obj_e - pe) < DEDUP_RADIUS:
                return
        bbox = (cx, cy, best.bbox.size_x, best.bbox.size_y)
        self._log_detection(cat, conf, obj_n, obj_e, agl, bbox=bbox)

    def _log_detection(self, category, confidence, north_m, east_m, agl_m, bbox=None):
        lat = HOME_LAT + north_m / M_PER_DEG
        lon = HOME_LON + east_m  / (M_PER_DEG * COS_LAT)
        crop_path = ""
        if _HAVE_PIL and bbox is not None and self._latest_frame is not None:
            try:
                cx, cy, bw, bh = bbox; pad = 20
                x1 = max(0, int(cx - bw/2) - pad); y1 = max(0, int(cy - bh/2) - pad)
                x2 = min(self._latest_frame.width,  int(cx + bw/2) + pad)
                y2 = min(self._latest_frame.height, int(cy + bh/2) + pad)
                crop = self._latest_frame.crop((x1, y1, x2, y2))
                os.makedirs(CROP_DIR, exist_ok=True)
                crop_path = os.path.join(CROP_DIR, f"det_{self._det_count:03d}.jpg")
                crop.save(crop_path, "JPEG", quality=90)
            except Exception as e:
                print(f"[AutoMission] crop save failed: {e}"); crop_path = ""
        need_header = not os.path.exists(DET_LOG)
        with open(DET_LOG, "a") as f:
            if need_header:
                f.write("timestamp,category,confidence,lat,lon,agl_m,crop_path\n")
            f.write(f"{time.time():.3f},{category},{confidence:.3f},"
                    f"{lat:.6f},{lon:.6f},{agl_m:.1f},{crop_path}\n")
        self._logged_positions.append((north_m, east_m))
        self._det_count += 1
        print(f"[AutoMission] logged: {category} conf={confidence:.2f}"
              f"  lat={lat:.6f} lon={lon:.6f}  agl={agl_m:.1f} m")

    # ── Vision injection thread ────────────────────────────────────────────────
    # Identical to ardupilot_commander.py — kinematic truth VPE at 20 Hz.
    def start_vision(self, stop):
        def loop():
            last_ds = None; anyloc_est = None; last_mtime = 0.0
            n_sent = 0; phase_logged = False
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
                        print(f"[AutoMission] AGL {drone_agl:.0f} m ≥"
                              f" {MIN_LOCALISATION_AGL:.0f} m — VPE → AnyLoc")
                        phase_logged = True
                    try:
                        mtime = os.path.getmtime(ESTIMATE_JSON)
                        if mtime != last_mtime:
                            with open(ESTIMATE_JSON) as fh:
                                est = json.load(fh)
                            err_m = est.get("error_m", 999.0)
                            if (est.get("agl_m", 0.0) >= MIN_LOCALISATION_AGL
                                    and err_m < 100.0):
                                lat = est["est_lat"]; lon = est["est_lon"]
                                yaw = math.pi / 2.0
                                n_v = (lat - HOME_LAT) * M_PER_DEG
                                e_v = (lon - HOME_LON) * M_PER_DEG * COS_LAT
                                cov = max(1.0, err_m ** 2)
                                anyloc_est = (e_v, n_v, yaw, cov)
                                last_mtime = mtime
                    except (FileNotFoundError, KeyError, json.JSONDecodeError):
                        pass
                # Always use kinematic truth for VPE (AnyLoc cov too large for EKF3)
                if self._drone is not None:
                    east_v  = self._drone.pose.position.x
                    north_v = self._drone.pose.position.y
                else:
                    east_v, north_v = 0.0, 0.0
                yaw_v  = math.pi / 2.0; cov_xy = 0.1
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
                    print("[AutoMission] vision thread started")
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
        t = threading.Thread(target=loop, daemon=True); t.start(); return t

    # ── Generic helpers ────────────────────────────────────────────────────────
    def _spin_until(self, cond, timeout):
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if cond():
                return True
        return False

    def _agl(self):
        if self._drone is not None:
            return self._drone.pose.position.z - HOME_ALT_MSL
        if self._local_pos is not None:
            return self._local_pos.pose.position.z
        return 0.0

    def make_sp(self, east, north, up):
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
        if fut.done() and fut.result().success:
            self.get_logger().info(f"{'arm' if value else 'disarm'}: ✓"); return True
        if not value:
            self.get_logger().warn("disarm failed"); return False
        self.get_logger().warn("regular arm failed — retrying with force arm …")
        drain_end = time.time() + 0.5
        while time.time() < drain_end:
            rclpy.spin_once(self, timeout_sec=0.05)
        req2 = CommandLong.Request()
        req2.command = 400; req2.param1 = 1.0; req2.param2 = 21196.0
        fut2 = self._cmd_cli.call_async(req2)
        end2 = time.time() + timeout
        while not fut2.done() and time.time() < end2:
            rclpy.spin_once(self, timeout_sec=0.05)
        ok2 = fut2.done() and fut2.result().success
        self.get_logger().info(f"force arm: {'✓' if ok2 else 'FAIL'}"); return ok2

    def set_ekf_origin(self, lat, lon, alt_msl_m, timeout=60.0):
        self.get_logger().info(
            f"Setting EKF origin: {lat:.6f}°N {lon:.6f}°E {alt_msl_m:.1f} m MSL")
        self._gps_origin_received = False
        origin_msg = GeoPointStamped()
        origin_msg.position.latitude  = lat
        origin_msg.position.longitude = lon
        origin_msg.position.altitude  = alt_msl_m
        for attempt in range(1, 11):
            origin_msg.header.stamp = self.get_clock().now().to_msg()
            self._origin_pub.publish(origin_msg)
            self.get_logger().info(f"  origin publish #{attempt}/10")
            t_end = time.time() + 0.5
            while time.time() < t_end:
                rclpy.spin_once(self, timeout_sec=0.05)
                if self._gps_origin_received:
                    self.get_logger().info(f"EKF origin confirmed ✓ (publish #{attempt})")
                    return True
        self.get_logger().warn("GPS_GLOBAL_ORIGIN echo not received — published 10×; continuing")
        return True

    def wait_ekf_pos(self, timeout=90.0):
        EKF_POS_HORIZ_ABS = 0x010
        _FLAG_NAMES = {0x001: "ATT", 0x002: "VEL_H", 0x004: "VEL_V",
                       0x008: "POS_H_REL", 0x010: "POS_H_ABS", 0x020: "POS_V_ABS",
                       0x040: "POS_V_AGL", 0x080: "CONST_POS"}
        self.get_logger().info("Waiting for EKF POS_ABS …")
        deadline = time.time() + timeout; last_print = 0.0
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._ekf_flags & EKF_POS_HORIZ_ABS:
                self.get_logger().info("EKF POS_ABS ✓"); return True
            now = time.time()
            if now - last_print > 5.0:
                active = " | ".join(n for v, n in _FLAG_NAMES.items()
                                    if self._ekf_flags & v)
                self.get_logger().warn(
                    f"EKF flags 0x{self._ekf_flags:03x}: [{active or 'none'}]"
                    " — waiting for POS_H_ABS")
                last_print = now
        self.get_logger().warn("EKF POS_ABS timeout"); return False

    def takeoff(self, alt_agl, timeout=180.0):
        self.get_logger().info(f"NAV_TAKEOFF to {alt_agl:.0f} m AGL …")
        req = CommandTOL.Request(); req.altitude = float(alt_agl)
        fut = self._tof_cli.call_async(req)
        tof_end = time.time() + 10.0
        while not fut.done() and time.time() < tof_end:
            rclpy.spin_once(self, timeout_sec=0.05)
        if fut.done():
            self.get_logger().info(
                f"NAV_TAKEOFF {'accepted' if fut.result().success else 'rejected'}")
        deadline = time.time() + timeout; last_print = time.time()
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            agl = self._agl(); now = time.time()
            if now - last_print > 3.0:
                print(f"[AutoMission] AGL={agl:.1f} m  target={alt_agl:.0f} m"
                      f"  mode={self._state.mode}  armed={self._state.armed}")
                last_print = now
            if agl >= alt_agl - 0.5:
                self.get_logger().info(f"Reached {alt_agl:.0f} m AGL ✓"); return True
        self.get_logger().warn("Takeoff timeout"); return False

    def engage_guided(self):
        """STABILIZE → arm → EKF origin → EKF POS_ABS → GUIDED."""
        for attempt in range(5):
            if self.set_mode("STABILIZE"): break
            self.get_logger().warn(
                f"STABILIZE failed (attempt {attempt+1}/5) — retrying …"); time.sleep(1.0)
        time.sleep(0.5)
        print("[AutoMission] Arming in STABILIZE …")
        if not self.arm():
            print("[AutoMission] ABORT: arm failed"); return False
        if not self.set_ekf_origin(HOME_LAT, HOME_LON, HOME_ALT_MSL):
            print("[AutoMission] ABORT: EKF origin failed"); return False
        if not self.wait_ekf_pos(timeout=60.0):
            print("[AutoMission] ABORT: EKF POS_ABS not reached"); return False
        for attempt in range(5):
            if self.set_mode("GUIDED"): break
            self.get_logger().warn(
                f"GUIDED failed (attempt {attempt+1}/5) — retrying …"); time.sleep(1.0)
        time.sleep(0.5)
        return True

    # ── go_to_ned — kept for RTL only (not used during the AUTO survey) ────────
    def go_to_ned(self, north, east, agl, timeout=300.0, speed=5.0, radius=8.0):
        """Fly to (north, east, agl) using GUIDED velocity setpoints (RTL/landing only)."""
        NAV_SPEED_V = 2.0; ALT_KP = 0.4
        _VMASK = (PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY |
                  PositionTarget.IGNORE_PZ |
                  PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY |
                  PositionTarget.IGNORE_AFZ |
                  PositionTarget.IGNORE_YAW | PositionTarget.IGNORE_YAW_RATE)
        deadline = time.time() + timeout; last_print = time.time()
        while time.time() < deadline:
            if self._drone is not None:
                ds = self._drone.pose.position
                cur_e, cur_n, drone_agl = ds.x, ds.y, ds.z - HOME_ALT_MSL
            elif self._local_pos is not None:
                p = self._local_pos.pose.position
                cur_e, cur_n, drone_agl = p.x, p.y, p.z
            else:
                rclpy.spin_once(self, timeout_sec=0.1); continue
            dx = cur_e - east; dy = cur_n - north
            hdist = math.hypot(dx, dy)
            if hdist <= radius: return True
            spd = min(speed, max(1.0, hdist * 0.5))
            v_e = -dx / hdist * spd; v_n = -dy / hdist * spd
            v_up = max(-NAV_SPEED_V, min(NAV_SPEED_V, ALT_KP * (agl - drone_agl)))
            sp = PositionTarget()
            sp.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
            sp.type_mask = _VMASK
            sp.velocity.x = float(v_e); sp.velocity.y = float(v_n)
            sp.velocity.z = float(v_up)
            sp.header.stamp = self.get_clock().now().to_msg()
            self._sp_pub.publish(sp); rclpy.spin_once(self, timeout_sec=0.05)
            now = time.time()
            if now - last_print > 5.0:
                print(f"[AutoMission] RTL  errN={dy:+.1f} errE={dx:+.1f}"
                      f"  dist={hdist:.1f} m  AGL={drone_agl:.1f} m"); last_print = now
        return False

    # ── Mission upload helpers ─────────────────────────────────────────────────
    def build_mission(self):
        """
        Build the MAVROS Waypoint list for the survey.

        Layout:
          [0]    Home item (lat/lon=HOME, alt=0) — required by ArduCopter as index 0.
          [1..N] NAV_WAYPOINT for each entry in SURVEY_WPS.
                 param2 = per-WP acceptance radius (CORNER or regular).
                 param3 = 0 (stop-at-waypoint; no fly-by).

        NED→lat/lon conversion uses HOME_LAT/LON + M_PER_DEG/COS_LAT.
        FRAME_GLOBAL_REL_ALT (frame=3): z_alt is AGL, consistent with TAKEOFF_ALT.
        """
        wps = []

        # Index 0: home item (mandatory; ArduCopter ignores its position but needs the slot)
        home_wp = Waypoint()
        home_wp.frame        = Waypoint.FRAME_GLOBAL_REL_ALT
        home_wp.command      = 16   # MAV_CMD_NAV_WAYPOINT
        home_wp.is_current   = False
        home_wp.autocontinue = True
        home_wp.param1       = 0.0  # hold time
        home_wp.param2       = 0.0  # acceptance radius (unused for home)
        home_wp.param3       = 0.0
        home_wp.param4       = 0.0
        home_wp.x_lat        = HOME_LAT
        home_wp.y_long       = HOME_LON
        home_wp.z_alt        = 0.0
        wps.append(home_wp)

        # Survey waypoints: indices 1 .. len(SURVEY_WPS)
        for i, (wn, we, wagl) in enumerate(SURVEY_WPS):
            lat, lon = ned_to_latlon(wn, we)
            accept_r = (CORNER_ACCEPTANCE_RADIUS
                        if i in CORNER_WP_INDICES
                        else WP_ACCEPTANCE_RADIUS)

            wp = Waypoint()
            wp.frame        = Waypoint.FRAME_GLOBAL_REL_ALT
            wp.command      = 16   # MAV_CMD_NAV_WAYPOINT
            wp.is_current   = (i == 0)  # first survey WP is the active target
            wp.autocontinue = True
            wp.param1       = 0.0        # hold time (s)
            wp.param2       = accept_r   # acceptance radius override (m)
            wp.param3       = 0.0        # pass-by radius: 0 = stop at WP
            wp.param4       = float('nan')  # yaw: NaN = keep current
            wp.x_lat        = lat
            wp.y_long       = lon
            wp.z_alt        = wagl
            wps.append(wp)

        return wps

    def clear_mission(self, timeout=10.0):
        req = WaypointClear.Request()
        fut = self._wp_clear_cli.call_async(req)
        end = time.time() + timeout
        while not fut.done() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        ok = fut.done() and fut.result().success
        self.get_logger().info(f"mission clear: {'✓' if ok else 'FAIL'}")
        return ok

    def push_mission(self, wps, timeout=20.0):
        """Upload waypoint list to ArduPilot via /mavros/mission/push."""
        req = WaypointPush.Request()
        req.start_index = 0
        req.waypoints   = wps
        print(f"[AutoMission] pushing {len(wps)} mission items …")
        fut = self._wp_push_cli.call_async(req)
        end = time.time() + timeout
        while not fut.done() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not fut.done():
            print("[AutoMission] mission push timed out"); return False
        r = fut.result()
        ok = r.success and r.wp_transfered == len(wps)
        print(f"[AutoMission] mission push: {'✓' if ok else 'FAIL'}"
              f"  transferred={r.wp_transfered}/{len(wps)}")
        return ok

    def set_current_wp(self, seq, timeout=5.0):
        """Tell ArduPilot which mission item to execute first."""
        req = WaypointSetCurrent.Request(); req.wp_seq = seq
        fut = self._wp_cur_cli.call_async(req)
        end = time.time() + timeout
        while not fut.done() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        ok = fut.done() and fut.result().success
        self.get_logger().info(f"set_current_wp({seq}): {'✓' if ok else 'FAIL'}")
        return ok

    # ── AUTO mode survey execution ─────────────────────────────────────────────
    def run_auto_mission(self, mission_wps):
        """
        Switch to AUTO mode and wait for all survey waypoints to be reached.

        mission_wps: list returned by build_mission() — home item at [0],
                     survey WPs at [1..N].
        Returns True when the last survey WP (index N) is reached, False on timeout.

        Progress is tracked via _cb_reached which updates self._reached_wp.
        The last expected wp_seq is len(SURVEY_WPS) (= index of last survey WP
        in the 0-indexed mission, since home occupies index 0).
        """
        last_survey_seq = len(SURVEY_WPS)  # e.g., 18 for 18 survey WPs
        self._reached_wp = -1

        print(f"[AutoMission] switching to AUTO mode …")
        for attempt in range(5):
            if self.set_mode("AUTO"): break
            time.sleep(1.0)
        else:
            print("[AutoMission] ABORT: could not enter AUTO mode"); return False

        print(f"[AutoMission] === SURVEY START  {len(SURVEY_WPS)} waypoints"
              f"  speed={SURVEY_SPEED:.0f} m/s ===")
        deadline   = time.time() + SURVEY_TIMEOUT
        last_print = time.time()

        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

            if self._reached_wp >= last_survey_seq:
                # All survey waypoints reached
                ds = self._drone.pose.position if self._drone else None
                if ds:
                    print(f"[AutoMission] === SURVEY COMPLETE"
                          f"  pos N={ds.y:+.1f} E={ds.x:+.1f}"
                          f"  AGL={ds.z - HOME_ALT_MSL:.1f} m ===")
                else:
                    print("[AutoMission] === SURVEY COMPLETE ===")
                return True

            now = time.time()
            if now - last_print > 10.0:
                ds = self._drone.pose.position if self._drone else None
                n_str = e_str = agl_str = "?"
                if ds:
                    n_str   = f"{ds.y:+.1f}"
                    e_str   = f"{ds.x:+.1f}"
                    agl_str = f"{ds.z - HOME_ALT_MSL:.1f}"
                print(f"[AutoMission] mode={self._state.mode}"
                      f"  reached_wp={self._reached_wp}/{last_survey_seq}"
                      f"  N={n_str} E={e_str} AGL={agl_str} m"
                      f"  t_left={deadline - now:.0f} s")
                last_print = now

        print(f"[AutoMission] survey TIMEOUT after {SURVEY_TIMEOUT:.0f} s")
        return False


# ── Main ───────────────────────────────────────────────────────────────────────
def main():
    rclpy.init()
    cmd  = ArduPilotAutoCommander()
    stop = threading.Event()

    try:
        os.makedirs(os.path.dirname(ESTIMATE_JSON), exist_ok=True)
        with open(ESTIMATE_JSON, "w") as _ef:
            json.dump({"agl_m": -1.0, "error_m": 999.0}, _ef)
    except OSError:
        pass

    cmd.start_vision(stop)

    print("[AutoMission] waiting for MAVROS connection …")
    if not cmd._spin_until(lambda: cmd._state.connected, 60.0):
        print("[AutoMission] MAVROS not connected — start ArduPilot SITL + MAVROS first")
        stop.set(); cmd.destroy_node(); rclpy.shutdown(); return
    print("[AutoMission] MAVROS connected ✓")

    print("[AutoMission] waiting for ArduPilot mode to initialize …")
    if not cmd._spin_until(lambda: bool(cmd._state.mode), 30.0):
        print("[AutoMission] WARNING: ArduPilot mode never set — proceeding")
    else:
        print(f"[AutoMission] ArduPilot mode: {cmd._state.mode} ✓")

    print("[AutoMission] waiting for drone state …")
    if not cmd._spin_until(lambda: cmd._drone is not None, 30.0):
        print("[AutoMission] WARNING: /drone/state not received — proceeding")

    start_agl = cmd._agl()
    in_air    = start_agl > 5.0
    if in_air:
        print(f"[AutoMission] in-air restart at {start_agl:.0f} m AGL — skipping takeoff")
        if cmd._state.mode not in ("GUIDED", "AUTO"):
            cmd.set_mode("GUIDED")
            cmd._spin_until(lambda: cmd._state.mode == "GUIDED", 10.0)

    try:
        if not in_air:
            print("[AutoMission] engaging GUIDED mode for takeoff …")
            if not cmd.engage_guided():
                print("[AutoMission] ABORT: engage_guided failed")
                stop.set(); cmd.destroy_node(); rclpy.shutdown(); return
            if not cmd.takeoff(TAKEOFF_ALT):
                print("[AutoMission] ABORT: takeoff failed")
                stop.set(); cmd.destroy_node(); rclpy.shutdown(); return

        # 5 s hold at cruise altitude before uploading mission (EKF stabilise)
        if cmd._drone is not None:
            hold_e = cmd._drone.pose.position.x
            hold_n = cmd._drone.pose.position.y
        else:
            hold_e, hold_n = 0.0, 0.0
        print(f"[AutoMission] holding 5 s at {TAKEOFF_ALT:.0f} m AGL …")
        t_hold = time.time() + 5.0
        sp = cmd.make_sp(hold_e, hold_n, TAKEOFF_ALT)
        while time.time() < t_hold:
            sp.header.stamp = cmd.get_clock().now().to_msg()
            cmd._sp_pub.publish(sp); rclpy.spin_once(cmd, timeout_sec=0.05)

        # ── Build and upload mission ───────────────────────────────────────────
        mission_wps = cmd.build_mission()
        if not cmd.clear_mission():
            print("[AutoMission] WARNING: could not clear existing mission — continuing")
        if not cmd.push_mission(mission_wps):
            print("[AutoMission] ABORT: mission upload failed")
            stop.set(); cmd.destroy_node(); rclpy.shutdown(); return

        # Set the first survey WP (index 1 in the mission) as active.
        # The home item at index 0 is skipped; AUTO mode begins at index 1.
        cmd.set_current_wp(1)

        # ── Run AUTO mission ───────────────────────────────────────────────────
        survey_ok = cmd.run_auto_mission(mission_wps)

        # Return to home in GUIDED mode — RTL is unsafe without GPS home fix.
        print("[AutoMission] === returning home (GUIDED) ===")
        cmd.set_mode("GUIDED")
        cmd._spin_until(lambda: cmd._state.mode == "GUIDED", 10.0)
        cmd.go_to_ned(0.0, 0.0, TAKEOFF_ALT,
                      timeout=RTL_TIMEOUT, speed=RTL_SPEED)
        print("[AutoMission] over home — LAND")
        cmd.set_mode("LAND")
        cmd._spin_until(lambda: not cmd._state.armed, 150.0)
        print(f"[AutoMission] Disarmed — landed ✓  survey={'OK' if survey_ok else 'TIMEOUT'}")

    except KeyboardInterrupt:
        print("[AutoMission] Ctrl-C — returning home")
        try:
            cmd.set_mode("GUIDED")
            cmd._spin_until(lambda: cmd._state.mode == "GUIDED", 10.0)
            cmd.go_to_ned(0.0, 0.0, TAKEOFF_ALT, timeout=120.0, speed=RTL_SPEED)
            cmd.set_mode("LAND")
            cmd._spin_until(lambda: not cmd._state.armed, 150.0)
            print("[AutoMission] Disarmed ✓")
        except Exception:
            pass
    except Exception as exc:
        print(f"[AutoMission] mission aborted: {exc}")
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
