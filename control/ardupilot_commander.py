#!/usr/bin/env python3
"""
ArduPilot flight commander (MAVROS2) — external-vision no-GPS, survey mission.

Ported from px4_commander.py with ArduPilot-specific changes:
  - GUIDED mode instead of OFFBOARD.
  - Arm sequence: STABILIZE → arm → GUIDED (no pre-stream setpoints needed).
  - EKF origin published to /mavros/global_position/set_gp_origin.
  - EKF ready: wait for EKF_POS_HORIZ_ABS via /uas1/mavlink_source raw MAVLink.
  - Takeoff via NAV_TAKEOFF (CommandTOL) — ArduPilot climbs autonomously.
  - Land via "LAND" mode (not "AUTO.LAND" which is PX4-specific).
  - Force-arm fallback via CommandLong(400, param2=21196) to bypass SITL pre-arm.

Setpoint convention (this was the original inversion bug in flight_commander.py):
  MAVROS2 always applies ENU→NED on setpoint_raw/local regardless of FRAME_LOCAL_NED.
  Send x=East, y=North, z=Up(AGL); MAVROS converts to NED. Identical to px4_commander.py.
  The old flight_commander.py's hold block mistakenly sent NED (x=north, y=east) which
  caused MAVROS to swap axes, producing mirror-direction position-hold divergence.

Environment variables:
  HOLDTEST=1         run Phase-3 hold gate (HOLD_AGL m) instead of full mission
  TAKEOFF_ALT=<m>    override mission cruise altitude (default 65.0 m)

Run:
  source /opt/ros/jazzy/setup.bash
  python3 control/ardupilot_commander.py
  HOLDTEST=1 python3 control/ardupilot_commander.py
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
from mavros_msgs.msg import Mavlink, PositionTarget, State
from mavros_msgs.srv import CommandBool, CommandLong, CommandTOL, SetMode

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
    print(f"[APCmd] HOME_ALT_MSL = {HOME_ALT_MSL:.1f} m  (from {_HOME_CFG})")
except (FileNotFoundError, KeyError):
    HOME_ALT_MSL = 28.17
    print(f"[APCmd] HOME_ALT_MSL = {HOME_ALT_MSL:.1f} m  (default)")

# ── Mission parameters ─────────────────────────────────────────────────────────
TAKEOFF_ALT          = float(os.environ.get("TAKEOFF_ALT", "65.0"))
HOLD_AGL             = 3.0    # m — Phase-3 gate altitude (HOLDTEST mode)
WAYPOINT_RADIUS      = 60.0   # m — survey waypoint arrival threshold
WAYPOINT_TIMEOUT     = 900.0  # s per waypoint
MIN_LOCALISATION_AGL = 50.0   # m — below this use truth VPE; above, use AnyLoc

SURVEY_SPEED = 12.0   # m/s — strip cruise speed

COS_LAT   = math.cos(math.radians(HOME_LAT))
M_PER_DEG = 111_320.0

# ── Survey waypoints (north_m, east_m, agl_m relative to home) ────────────────
# 7-strip E-W boustrophedon lawnmower; 91.7 m N-S spacing; 65 m AGL; ~7.36 km.
SURVEY_WPS = [
    ( 60.0,   -573.0,  TAKEOFF_ALT),  # ENTRY: E end strip S  → fly W
    ( 60.0,   -972.0,  TAKEOFF_ALT),  # WP01 : W end strip S
    (152.0,  -1288.0,  TAKEOFF_ALT),  # WP02 : W end strip 1  → fly E
    (152.0,   -556.0,  TAKEOFF_ALT),  # WP03 : E end strip 1
    (243.0,   -539.0,  TAKEOFF_ALT),  # WP04 : E end strip 2  → fly W
    (243.0,  -1275.0,  TAKEOFF_ALT),  # WP05 : W end strip 2
    (335.0,  -1261.0,  TAKEOFF_ALT),  # WP06 : W end strip 3  → fly E
    (335.0,   -521.0,  TAKEOFF_ALT),  # WP07 : E end strip 3
    (427.0,   -504.0,  TAKEOFF_ALT),  # WP08 : E end strip 4  → fly W
    (427.0,  -1247.0,  TAKEOFF_ALT),  # WP09 : W end strip 4
    (518.0,  -1234.0,  TAKEOFF_ALT),  # WP10 : W end strip 5  → fly E
    (518.0,   -548.0,  TAKEOFF_ALT),  # WP11 : E end strip 5
    (610.0,  -1043.0,  TAKEOFF_ALT),  # WP12 : E end strip N  → fly W
    (610.0,  -1220.0,  TAKEOFF_ALT),  # WP13 : W end strip N
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
DEDUP_RADIUS = 5.0   # m — skip re-logging same vehicle within this distance

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


class ArduPilotCommander(rclpy.node.Node):
    def __init__(self):
        super().__init__("ardupilot_commander")
        self._state     = State()
        self._local_pos = None   # /mavros/local_position/pose  (EKF, ENU)
        self._local_vel = None   # /mavros/local_position/velocity_local (ENU)
        self._drone     = None   # /drone/state  (ENU kinematic truth)

        self._ekf_flags           = 0
        self._gps_origin_received = False

        self._latest_frame     = None
        self._det_count        = 0
        self._logged_positions = []   # (north_m, east_m) for dedup

        # Subscribers
        from geometry_msgs.msg import PoseStamped
        self.create_subscription(State,        "/mavros/state",
                                 self._cb_state, 10)
        self.create_subscription(PoseStamped,  "/mavros/local_position/pose",
                                 self._cb_local, _SENSOR_QOS)
        self.create_subscription(TwistStamped, "/mavros/local_position/velocity_local",
                                 self._cb_vel,   _SENSOR_QOS)
        self.create_subscription(PoseStamped,  "/drone/state",
                                 self._cb_drone, _SENSOR_QOS)
        self.create_subscription(Mavlink,      "/uas1/mavlink_source",
                                 self._cb_mavlink, _SENSOR_QOS)

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

        # Publishers
        self._vpe_pub    = self.create_publisher(
            PoseWithCovarianceStamped, "/mavros/vision_pose/pose_cov", 1)
        self._vspd_pub   = self.create_publisher(
            TwistStamped, "/mavros/vision_speed/speed_twist", 1)
        self._sp_pub     = self.create_publisher(
            PositionTarget, "/mavros/setpoint_raw/local", 1)
        self._origin_pub = self.create_publisher(
            GeoPointStamped, "/mavros/global_position/set_gp_origin", 1)

        # Service clients
        self._arm_cli  = self.create_client(CommandBool, "/mavros/cmd/arming")
        self._mode_cli = self.create_client(SetMode,     "/mavros/set_mode")
        self._tof_cli  = self.create_client(CommandTOL,  "/mavros/cmd/takeoff")
        self._cmd_cli  = self.create_client(CommandLong, "/mavros/cmd/command")

        self.get_logger().info("ArduPilot commander ready")

    # ── Callbacks ──────────────────────────────────────────────────────────────
    def _cb_state(self, m):  self._state     = m
    def _cb_local(self, m):  self._local_pos = m
    def _cb_vel(self, m):    self._local_vel = m
    def _cb_drone(self, m):  self._drone     = m

    def _cb_mavlink(self, msg: Mavlink) -> None:
        raw = b"".join(x.to_bytes(8, "little") for x in msg.payload64)
        if msg.msgid == 193 and len(raw) >= 22:   # EKF_STATUS_REPORT flags at byte 20
            self._ekf_flags = struct.unpack_from("<H", raw, 20)[0]
        elif msg.msgid == 49:                      # GPS_GLOBAL_ORIGIN echo
            self._gps_origin_received = True

    def _cb_image(self, msg):
        try:
            self._latest_frame = _PilImage.frombytes(
                "RGB", (msg.width, msg.height), bytes(msg.data))
        except Exception:
            pass

    def _cb_detections(self, msg):
        """Project YOLO detections to world coords via yaw-corrected GSD and log."""
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

        q       = self._drone.pose.orientation
        yaw_enu = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y ** 2 + q.z ** 2))
        h    = -yaw_enu
        dx_m =  (cx - CAM_W / 2.0) * gsd_x
        dy_m = -(cy - CAM_H / 2.0) * gsd_y
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
        print(f"[APCmd] {cat} conf={conf:.2f}  N={obj_n:+.1f} E={obj_e:+.1f}"
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
                print(f"[APCmd] crop save failed: {e}")
                crop_path = ""

        need_header = not os.path.exists(DET_LOG)
        with open(DET_LOG, "a") as f:
            if need_header:
                f.write("timestamp,category,confidence,lat,lon,agl_m,crop_path\n")
            f.write(f"{time.time():.3f},{category},{confidence:.3f},"
                    f"{lat:.6f},{lon:.6f},{agl_m:.1f},{crop_path}\n")
        self._logged_positions.append((north_m, east_m))
        self._det_count += 1
        print(f"[APCmd] logged: {category} conf={confidence:.2f}"
              f"  lat={lat:.6f} lon={lon:.6f}  agl={agl_m:.1f} m")

    # ── Vision injection thread ────────────────────────────────────────────────
    def start_vision(self, stop):
        """
        20 Hz background thread: publish VPE + velocity to MAVROS → ArduPilot EKF3.

        Two-phase strategy:
          Phase 1 (AGL < MIN_LOCALISATION_AGL):
            position = drone_state kinematic truth, cov_xy = 0.1 m²
          Phase 2 (AGL ≥ MIN_LOCALISATION_AGL):
            position = AnyLoc estimate from latest_estimate.json, cov_xy = err_m²

        ENU yaw = π/2 (North) hardcoded in both phases.
        MAVROS converts ENU yaw=π/2 → NED yaw=0 (North) for ArduPilot EKF3.
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
                        print(f"[APCmd] AGL {drone_agl:.0f} m ≥ {MIN_LOCALISATION_AGL:.0f} m"
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
                                    print(f"[APCmd] AnyLoc VPE: N={n_v:+.1f}"
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
                    print("[APCmd] vision thread started (Phase 1 — truth)")

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
        """
        Position PositionTarget in ENU.
        MAVROS2 converts ENU→NED: send x=East, y=North, z=Up(AGL).
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
            self.get_logger().info(f"{'arm' if value else 'disarm'}: ✓")
            return True

        if not value:
            self.get_logger().warn("disarm failed")
            return False

        # Force-arm fallback: bypasses all pre-arm checks (SITL VisOdom health, GPS, etc.)
        self.get_logger().warn("regular arm failed — retrying with force arm …")
        drain_end = time.time() + 0.5
        while time.time() < drain_end:
            rclpy.spin_once(self, timeout_sec=0.05)

        req2 = CommandLong.Request()
        req2.command = 400       # MAV_CMD_COMPONENT_ARM_DISARM
        req2.param1  = 1.0       # arm
        req2.param2  = 21196.0   # force magic
        fut2 = self._cmd_cli.call_async(req2)
        end2 = time.time() + timeout
        while not fut2.done() and time.time() < end2:
            rclpy.spin_once(self, timeout_sec=0.05)
        ok2 = fut2.done() and fut2.result().success
        self.get_logger().info(f"force arm: {'✓' if ok2 else 'FAIL'}")
        return ok2

    # ── ArduPilot-specific helpers ─────────────────────────────────────────────
    def set_ekf_origin(self, lat, lon, alt_msl_m, timeout=60.0):
        """
        Publish GPS global origin 10× over 5 s and treat as success regardless
        of GPS_GLOBAL_ORIGIN echo — ArduPilot SITL accepts silently.
        """
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
                    self.get_logger().info(
                        f"EKF origin confirmed via GPS_GLOBAL_ORIGIN ✓ (publish #{attempt})")
                    return True

        self.get_logger().warn(
            "GPS_GLOBAL_ORIGIN echo not received (normal for this SITL) — "
            "published 10×; continuing")
        return True

    def wait_ekf_pos(self, timeout=90.0):
        """Block until EKF_POS_HORIZ_ABS (bit 4 of EKF_STATUS_REPORT) is set."""
        EKF_POS_HORIZ_ABS = 0x010
        _FLAG_NAMES = {
            0x001: "ATT", 0x002: "VEL_H", 0x004: "VEL_V",
            0x008: "POS_H_REL", 0x010: "POS_H_ABS", 0x020: "POS_V_ABS",
            0x040: "POS_V_AGL", 0x080: "CONST_POS",
        }
        self.get_logger().info("Waiting for EKF POS_ABS …")
        deadline   = time.time() + timeout
        last_print = 0.0
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._ekf_flags & EKF_POS_HORIZ_ABS:
                self.get_logger().info("EKF POS_ABS ✓")
                return True
            now = time.time()
            if now - last_print > 5.0:
                active = " | ".join(n for v, n in _FLAG_NAMES.items()
                                    if self._ekf_flags & v)
                self.get_logger().warn(
                    f"EKF flags 0x{self._ekf_flags:03x}: [{active or 'none'}]"
                    " — waiting for POS_H_ABS")
                last_print = now
        self.get_logger().warn("EKF POS_ABS timeout — check VPE flow and EKF origin")
        return False

    def takeoff(self, alt_agl, timeout=180.0):
        """Send NAV_TAKEOFF; ArduPilot climbs autonomously. Monitor AGL."""
        self.get_logger().info(f"NAV_TAKEOFF to {alt_agl:.0f} m AGL …")
        req = CommandTOL.Request()
        req.altitude = float(alt_agl)
        fut = self._tof_cli.call_async(req)
        tof_end = time.time() + 10.0
        while not fut.done() and time.time() < tof_end:
            rclpy.spin_once(self, timeout_sec=0.05)
        if fut.done():
            self.get_logger().info(
                f"NAV_TAKEOFF {'accepted' if fut.result().success else 'rejected'}")
        else:
            self.get_logger().warn("NAV_TAKEOFF send timed out — continuing")

        deadline   = time.time() + timeout
        last_print = time.time()
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            agl = self._agl()
            now = time.time()
            if now - last_print > 3.0:
                print(f"[APCmd] AGL={agl:.1f} m  target={alt_agl:.0f} m"
                      f"  mode={self._state.mode}  armed={self._state.armed}")
                last_print = now
                if now - deadline + timeout > 30.0 and agl < 2.0:
                    self.get_logger().warn(
                        "Drone not lifting after 30 s — check DISARM_DELAY=0 in params")
                    return False
            if agl >= alt_agl - 2.0:
                self.get_logger().info(f"Reached {alt_agl:.0f} m AGL ✓")
                return True

        self.get_logger().warn("Takeoff timeout")
        return False

    def engage_guided(self):
        """
        STABILIZE → arm → GUIDED → EKF origin → wait EKF_POS_HORIZ_ABS.
        Returns True on success, False on any failure.
        """
        self.set_mode("STABILIZE")
        time.sleep(0.5)

        print("[APCmd] Arming in STABILIZE …")
        if not self.arm():
            print("[APCmd] ABORT: arm failed")
            return False

        self.set_mode("GUIDED")
        time.sleep(0.5)

        if not self.set_ekf_origin(HOME_LAT, HOME_LON, HOME_ALT_MSL):
            print("[APCmd] ABORT: EKF origin failed")
            return False

        if not self.wait_ekf_pos(timeout=60.0):
            print("[APCmd] ABORT: EKF POS_ABS not reached — check VPE flow")
            return False

        return True

    # ── Waypoint navigation ────────────────────────────────────────────────────
    def go_to_ned(self, north, east, agl, timeout=WAYPOINT_TIMEOUT,
                  speed=5.0, radius=None):
        """
        Fly to (north, east, agl) via ENU velocity setpoints.

        MAVROS converts ENU velocity → NED; ArduPilot GUIDED velocity controller
        closes the loop. Velocity setpoints were confirmed to track direction correctly
        in prior testing (position setpoints flew mirror-direction due to axis swap).

        speed   horizontal cruise speed m/s
        radius  arrival distance m (default WAYPOINT_RADIUS)
        Returns True when within radius; False on timeout.
        """
        NAV_SPEED_V = 2.0    # m/s max vertical correction speed
        ALT_KP      = 0.4    # altitude P-gain for vertical velocity
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
            sp.velocity.x = float(v_e)   # ENU east  → MAVROS → NED east
            sp.velocity.y = float(v_n)   # ENU north → MAVROS → NED north
            sp.velocity.z = float(v_up)  # ENU up    → MAVROS → NED down (negated)
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
                print(f"[APCmd] errN={dy:+.1f} errE={dx:+.1f}"
                      f"  AGL={drone_agl:.1f} m  dist={hdist:.1f} m{ekf}{vel_s}")
                last_print = now

            if hdist <= arrival_r:
                return True

        return False


# ── Main ───────────────────────────────────────────────────────────────────────

def main():
    rclpy.init()
    cmd = ArduPilotCommander()
    stop = threading.Event()

    # Write stub estimate so VPE thread can read the file before AnyLoc starts
    try:
        os.makedirs(os.path.dirname(ESTIMATE_JSON), exist_ok=True)
        with open(ESTIMATE_JSON, "w") as _ef:
            json.dump({"agl_m": -1.0, "error_m": 999.0}, _ef)
    except OSError:
        pass

    cmd.start_vision(stop)

    print("[APCmd] waiting for MAVROS connection …")
    if not cmd._spin_until(lambda: cmd._state.connected, 60.0):
        print("[APCmd] MAVROS not connected — start ArduPilot SITL + MAVROS first")
        stop.set(); cmd.destroy_node(); rclpy.shutdown(); return
    print("[APCmd] MAVROS connected ✓")

    # Spin briefly to populate /drone/state
    print("[APCmd] waiting for drone state (up to 30 s) …")
    _last_diag = [time.time()]
    start_t = time.time()
    def _wait_drone():
        now = time.time()
        if now - _last_diag[0] > 8.0:
            _last_diag[0] = now
            print(f"[APCmd] diag: drone={'OK' if cmd._drone is not None else 'None'}"
                  f"  local_pos={'OK' if cmd._local_pos is not None else 'None'}"
                  f"  t={now-start_t:.0f}s")
        return cmd._drone is not None
    if not cmd._spin_until(_wait_drone, 30.0):
        print("[APCmd] WARNING: /drone/state not received — proceeding without kinematic truth")

    # Detect in-air restart
    start_agl = cmd._agl()
    in_air = start_agl > 5.0
    if in_air:
        print(f"[APCmd] in-air restart at {start_agl:.0f} m AGL — skipping takeoff")
        if cmd._state.mode != "GUIDED":
            print(f"[APCmd] mode={cmd._state.mode} — switching to GUIDED …")
            cmd.set_mode("GUIDED")
            cmd._spin_until(lambda: cmd._state.mode == "GUIDED", timeout=10.0)

    # ── HOLDTEST mode: Phase-3 position-hold gate ──────────────────────────────
    if os.environ.get("HOLDTEST"):
        e0 = cmd._drone.pose.position.x if cmd._drone else 0.0
        n0 = cmd._drone.pose.position.y if cmd._drone else 0.0

        if not in_air:
            print("[APCmd] === HOLDTEST: engaging GUIDED for hold gate ===")
            if not cmd.engage_guided():
                stop.set(); cmd.destroy_node(); rclpy.shutdown(); return
            if not cmd.takeoff(HOLD_AGL):
                print("[APCmd] HOLDTEST takeoff failed")
                stop.set(); cmd.destroy_node(); rclpy.shutdown(); return
            e0 = cmd._drone.pose.position.x if cmd._drone else 0.0
            n0 = cmd._drone.pose.position.y if cmd._drone else 0.0

        sp = cmd.make_sp(e0, n0, HOLD_AGL)
        print(f"[APCmd] === HOLD GATE: {HOLD_AGL:.0f} m AGL for 40 s ===")
        t_end = time.time() + 40.0; t_log = 0.0
        while time.time() < t_end:
            sp.header.stamp = cmd.get_clock().now().to_msg()
            cmd._sp_pub.publish(sp)
            rclpy.spin_once(cmd, timeout_sec=0.02)
            if time.time() - t_log > 3.0 and cmd._drone is not None:
                t_log = time.time()
                ds = cmd._drone.pose.position
                agl = ds.z - HOME_ALT_MSL
                print(f"[APCmd] drift E={ds.x-e0:+6.1f} N={ds.y-n0:+6.1f}"
                      f"  AGL={agl:4.1f}  dist={math.hypot(ds.x-e0,ds.y-n0):5.1f} m"
                      f"  mode={cmd._state.mode} armed={cmd._state.armed}")
        print("[APCmd] === gate done ===")
        stop.set(); cmd.destroy_node(); rclpy.shutdown(); return

    # ── Full survey mission ────────────────────────────────────────────────────
    try:
        if not in_air:
            print("[APCmd] engaging GUIDED mode …")
            if not cmd.engage_guided():
                print("[APCmd] ABORT: engage_guided failed")
                stop.set(); cmd.destroy_node(); rclpy.shutdown(); return

            if not cmd.takeoff(TAKEOFF_ALT):
                print("[APCmd] ABORT: takeoff failed")
                stop.set(); cmd.destroy_node(); rclpy.shutdown(); return

        # Hold 5 s at cruise altitude before starting survey
        if cmd._drone is not None:
            hold_e = cmd._drone.pose.position.x
            hold_n = cmd._drone.pose.position.y
        else:
            hold_e, hold_n = 0.0, 0.0
        print(f"[APCmd] holding 5 s at {TAKEOFF_ALT:.0f} m AGL …")
        t_hold = time.time() + 5.0
        sp = cmd.make_sp(hold_e, hold_n, TAKEOFF_ALT)
        while time.time() < t_hold:
            sp.header.stamp = cmd.get_clock().now().to_msg()
            cmd._sp_pub.publish(sp)
            rclpy.spin_once(cmd, timeout_sec=0.05)

        # ── Survey loop ────────────────────────────────────────────────────────
        print(f"[APCmd] === SURVEY START  {len(SURVEY_WPS)} waypoints"
              f"  speed={SURVEY_SPEED:.0f} m/s ===")
        wp_idx = 0
        while wp_idx < len(SURVEY_WPS):
            wn, we, wagl = SURVEY_WPS[wp_idx]
            print(f"[APCmd] SURVEY WP {wp_idx+1}/{len(SURVEY_WPS)}"
                  f"  N={wn:+.0f} E={we:+.0f} AGL={wagl:.0f} m")

            reached = cmd.go_to_ned(wn, we, wagl,
                                    timeout=WAYPOINT_TIMEOUT,
                                    speed=SURVEY_SPEED)

            if reached and cmd._drone is not None:
                ds  = cmd._drone.pose.position
                dx  = ds.x - we; dy = ds.y - wn
                print(f"[APCmd] WP {wp_idx+1} ARRIVED ✓"
                      f"  E={ds.x:+.1f} N={ds.y:+.1f}"
                      f"  horiz_err={math.hypot(dx, dy):.1f} m")
            else:
                print(f"[APCmd] WP {wp_idx+1} {'ARRIVED' if reached else 'TIMEOUT — skipping'}")
            wp_idx += 1

        # Survey complete — fly home explicitly; LAND mode does not need GPS home.
        # RTL is unsafe with external-vision-only localisation (requires GPS home fix).
        print("[APCmd] === SURVEY COMPLETE — returning home ===")
        cmd.go_to_ned(0.0, 0.0, TAKEOFF_ALT, timeout=300.0, speed=SURVEY_SPEED)
        print("[APCmd] Over home — LAND")
        cmd.set_mode("LAND")
        cmd._spin_until(lambda: not cmd._state.armed, timeout=150.0)
        print("[APCmd] Disarmed — landed ✓")

    except KeyboardInterrupt:
        print("[APCmd] Ctrl-C — returning home")
        try:
            cmd.go_to_ned(0.0, 0.0, TAKEOFF_ALT, timeout=120.0, speed=SURVEY_SPEED)
            cmd.set_mode("LAND")
            cmd._spin_until(lambda: not cmd._state.armed, timeout=150.0)
            print("[APCmd] Disarmed ✓")
        except Exception:
            pass
    except Exception as exc:
        print(f"[APCmd] mission aborted: {exc}")
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
