#!/usr/bin/env python3
"""
Autonomous flight commander — ROS2 / MAVROS2 version.

Replaces run_flight.py. Uses MAVROS2 topics and services for all flight
commands. Vision position injection is handled by anyloc/ros2_node.py
(publishes /mavros/vision_pose/pose → MAVROS2 → VISION_POSITION_ESTIMATE).

Architecture:
  anyloc/ros2_node.py  →  /mavros/vision_pose/pose  →  MAVROS2  →  ArduPilot EKF3
  this node            →  /mavros/setpoint_position/local  →  MAVROS2  →  ArduPilot

EKF origin and status are handled via MAVROS2 raw MAVLink topics
(/uas1/mavlink_source BEST_EFFORT) — no pymavlink or extra UDP port needed.

Run:
  source /opt/ros/jazzy/setup.bash
  python3 control/flight_commander.py

Prerequisites (running concurrently):
  - ArduPilot SITL  (sim_vehicle.py ... --add-param-file=control/no_gps.parm)
  - stub_bridge.py or Isaac Sim  (physics bridge on UDP 9002)
  - MAVROS2  (control/launch_mavros.sh)
  - AnyLoc ROS2 node  (anyloc/ros2_node.py)
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
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TwistStamped
from mavros_msgs.msg import Mavlink, PositionTarget, State, Waypoint
from mavros_msgs.srv import (CommandBool, CommandLong, CommandTOL, SetMode,
                             WaypointPush, WaypointClear)

_SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    depth=10,
)

# ── Home position ──────────────────────────────────────────────────────────────
HOME_LAT     = 23.450868
HOME_LON     = 120.286135
_HOME_CFG    = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             "home_elevation.json")
try:
    with open(_HOME_CFG) as _f:
        HOME_ALT_MSL = float(json.load(_f)["centre_elev_m"])
    print(f"[Commander] HOME_ALT_MSL = {HOME_ALT_MSL:.1f} m (from {_HOME_CFG})")
except (FileNotFoundError, KeyError):
    HOME_ALT_MSL = 28.17
    print(f"[Commander] HOME_ALT_MSL = {HOME_ALT_MSL:.1f} m (default)")

# ── Mission parameters ─────────────────────────────────────────────────────────
TAKEOFF_ALT          = 90.0   # metres AGL
WAYPOINT_RADIUS      = 60.0   # metres — how close counts as reached
WAYPOINT_TIMEOUT     = 900.0  # seconds per waypoint (699 m at 1 m/s ≈ 12 min)
MIN_LOCALISATION_AGL = 50.0   # metres AGL — below this, VPE is locked to home position;
                               # above this, AnyLoc estimates are used

# Target: 23.45564°N, 120.28169°E  (computed from HOME_LAT/LON below)
# N=+531.2 m  E=−453.9 m  dist≈699 m  AGL=90 m
WAYPOINTS = [
    (531.2, -453.9, -90.0),
]

COS_LAT   = math.cos(math.radians(HOME_LAT))
M_PER_DEG = 111_320.0

ESTIMATE_JSON = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "anyloc", "latest_estimate.json"
)


# ── Stub estimate (written at startup so VPE flows before AnyLoc starts) ──────

def _write_stub_estimate() -> None:
    est = {
        "timestamp": time.time(),
        "est_lat":   HOME_LAT, "est_lon":  HOME_LON,
        "alt_msl_m": HOME_ALT_MSL, "agl_m": 0.0,
        "yaw_deg": 0.0, "score": 1.0, "error_m": 0.0,
    }
    tmp = ESTIMATE_JSON + ".tmp"
    with open(tmp, "w") as fh:
        json.dump(est, fh)
    os.replace(tmp, ESTIMATE_JSON)


# ── Flight commander ROS2 node ─────────────────────────────────────────────────

class FlightCommander(rclpy.node.Node):
    def __init__(self):
        super().__init__("flight_commander")

        self._state               = State()
        self._local_pos           = None
        self._drone_state         = None  # from drone_sim.py — actual kinematic altitude
        self._ekf_flags           = 0     # from EKF_STATUS_REPORT (msg 193)
        self._gps_origin_received = False # set when GPS_GLOBAL_ORIGIN (msg 49) arrives
        self._last_motor_pwm      = None  # from SERVO_OUTPUT_RAW (msg 36)
        self._vpe_yaw             = math.pi / 2.0  # Phase-1 VPE yaw (ENU rad); mutable for calibration

        # Subscribers
        self.create_subscription(State, "/mavros/state",
                                 self._cb_state, 10)
        self.create_subscription(PoseStamped, "/mavros/local_position/pose",
                                 self._cb_local_pos, _SENSOR_QOS)
        # Direct kinematic altitude from drone_sim.py — position.z = MSL altitude
        # Use this for takeoff control to avoid EKF barometric divergence
        self.create_subscription(PoseStamped, "/drone/state",
                                 self._cb_drone_state, _SENSOR_QOS)
        # Raw MAVLink from FCU — BEST_EFFORT matches mavros_router's QoS
        self.create_subscription(Mavlink, "/uas1/mavlink_source",
                                 self._cb_mavlink, _SENSOR_QOS)

        # setpoint_raw/local with FRAME_LOCAL_NED and IGNORE_PZ.
        # MAVROS always converts ENU→NED even with FRAME_LOCAL_NED; send ENU
        # (x=East, y=North) so MAVROS produces the correct NED target.
        # IGNORE_PZ lets ArduPilot hold altitude from NAV_TAKEOFF (z convention ambiguous).
        self._pos_pub    = self.create_publisher(
            PositionTarget, "/mavros/setpoint_raw/local", 1)
        self._vpe_pub    = self.create_publisher(
            PoseWithCovarianceStamped, "/mavros/vision_pose/pose_cov", 1)
        # Horizontal velocity aiding (VISION_SPEED_ESTIMATE).  Without a velocity source
        # (EK3_SRC1_VELXY was 0) the EKF derives velocity only by integrating the bridge's
        # laggy double-differenced IMU accel → delayed damping → position-hold runaway.
        self._vspeed_pub = self.create_publisher(
            TwistStamped, "/mavros/vision_speed/speed_twist", 1)
        self._origin_pub = self.create_publisher(
            GeoPointStamped, "/mavros/global_position/set_gp_origin", 1)

        # Service clients
        self._arm_cli  = self.create_client(CommandBool, "/mavros/cmd/arming")
        self._cmd_cli  = self.create_client(CommandLong, "/mavros/cmd/command")
        self._mode_cli = self.create_client(SetMode,     "/mavros/set_mode")
        self._tof_cli  = self.create_client(CommandTOL,  "/mavros/cmd/takeoff")
        self._wp_push  = self.create_client(WaypointPush,  "/mavros/mission/push")
        self._wp_clear = self.create_client(WaypointClear, "/mavros/mission/clear")

        self.get_logger().info("Flight commander ready")

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def start_vision_thread(self, stop_event: threading.Event) -> threading.Thread:
        """
        Background thread: publishes VPE to /mavros/vision_pose/pose_cov at 5 Hz.

        Two-phase strategy:
          Phase 1 — below MIN_LOCALISATION_AGL:
            VPE = home position (0, 0) with 0.1 m² covariance.
            The drone is on the ground at the known home position so this is accurate.
            EKF sets POS_HORIZ_ABS and holds XY at home while baro handles altitude.

          Phase 2 — above MIN_LOCALISATION_AGL:
            VPE = AnyLoc estimate from latest_estimate.json.
            Camera can see landmarks at altitude → real visual localisation.
            Covariance scales with AnyLoc's reported error_m (min 1 m²).
        """
        def _loop():
            last_mtime   = 0.0
            anyloc_est   = None   # (east, north, yaw_rad, cov_xy) — set once above altitude
            n_sent       = 0
            n_anyloc     = 0
            phase_logged = False
            last_ds      = None   # (east, north, t) for velocity differencing

            _write_stub_estimate()

            while not stop_event.is_set():
                t0 = time.time()

                # Current AGL from EKF local position (ENU z = up from home = AGL)
                # Used only for Phase 1/2 switch; VPE altitude uses kinematic truth below.
                agl = 0.0
                if self._local_pos is not None:
                    agl = max(0.0, self._local_pos.pose.position.z)

                # True AGL from drone_sim.py kinematic state — sent as VPE z so ArduPilot's
                # EKF3 altitude estimate (EK3_SRC1_POSZ=6 ExternalNav) tracks the sim.
                # Without this, SIM_JSON barometer stays at 0 and ArduPilot climbs forever.
                drone_agl = 0.0
                if self._drone_state is not None:
                    drone_agl = max(0.0, self._drone_state.pose.position.z - HOME_ALT_MSL)

                # ── Phase 2: above localisation altitude — read AnyLoc estimate ──
                if agl >= MIN_LOCALISATION_AGL:
                    if not phase_logged:
                        print(f"[Commander] AGL {agl:.0f} m ≥ {MIN_LOCALISATION_AGL:.0f} m "
                              "— switching VPE to AnyLoc")
                        phase_logged = True
                    try:
                        mtime = os.path.getmtime(ESTIMATE_JSON)
                        if mtime != last_mtime:
                            with open(ESTIMATE_JSON) as fh:
                                est = json.load(fh)
                            err_m = est.get("error_m", 999.0)
                            # Require: taken at altitude AND error < 100 m.
                            # Estimates worse than 100 m are discarded — the EKF jump would
                            # destabilise the position controller mid-climb.
                            if (est.get("agl_m", 0.0) >= MIN_LOCALISATION_AGL
                                    and err_m < 100.0):
                                lat  = est["est_lat"]; lon = est["est_lon"]
                                yaw  = math.radians(est.get("yaw_deg", 0.0))
                                north = (lat - HOME_LAT) * M_PER_DEG
                                east  = (lon - HOME_LON) * M_PER_DEG * COS_LAT
                                cov_xy = max(1.0, err_m ** 2)
                                anyloc_est = (east, north, yaw, cov_xy)
                                last_mtime = mtime
                                n_anyloc += 1
                                if n_anyloc == 1:
                                    print(f"[Commander] First AnyLoc VPE: "
                                          f"N={north:+.1f} E={east:+.1f} m  "
                                          f"err={err_m:.1f} m")
                            elif est.get("agl_m", 0.0) >= MIN_LOCALISATION_AGL:
                                last_mtime = mtime  # advance mtime so we log once per file
                                print(f"[Commander] AnyLoc estimate rejected: "
                                      f"err={err_m:.1f} m ≥ 100 m — staying on Phase 1 VPE")
                    except (FileNotFoundError, KeyError, json.JSONDecodeError):
                        pass

                # ── Choose position and covariance ────────────────────────────────
                if agl >= MIN_LOCALISATION_AGL and anyloc_est is not None:
                    east_v, north_v, yaw_v, cov_xy = anyloc_est
                else:
                    # Phase 1 — track actual kinematic position from drone_sim.
                    # Sending fixed (0,0) caused EKF/kinematic mismatch: the drone
                    # drifted but ArduPilot believed it was still at home, making
                    # waypoints converge in the wrong frame.
                    # On the ground drone_state = (0,0) so EKF still inits at home;
                    # once airborne the VPE follows the real trajectory so the
                    # position controller can hold position and waypoints work.
                    if self._drone_state is not None:
                        east_v  = self._drone_state.pose.position.x
                        north_v = self._drone_state.pose.position.y
                    else:
                        east_v, north_v = 0.0, 0.0
                    # VPE yaw sets the EKF heading (no compass → external-nav is the yaw
                    # source).  The sim drone faces North (kinematic kyaw=0).  ENU yaw=π/2
                    # → MAVROS converts to EKF NED yaw=0 (North), matching the sim, so the
                    # position controller maps NED errors to the correct lean angles.
                    # (Requires the ch2<->ch3 motor-order fix in cesium_scene.py; without it
                    # the decoded roll/pitch are reflected and the drone flies away.)
                    yaw_v  = self._vpe_yaw
                    cov_xy = 0.1

                # ── Publish VPE ───────────────────────────────────────────────────
                hy  = yaw_v / 2.0
                msg = PoseWithCovarianceStamped()
                msg.header.stamp    = self.get_clock().now().to_msg()
                msg.header.frame_id = "map"   # ENU: x=East, y=North, z=Up
                msg.pose.pose.position.x    = east_v
                msg.pose.pose.position.y    = north_v
                msg.pose.pose.position.z    = drone_agl   # kinematic AGL → EKF altitude
                msg.pose.pose.orientation.z = math.sin(hy)
                msg.pose.pose.orientation.w = math.cos(hy)
                cov = [0.0] * 36
                cov[0]  = cov_xy  # x variance
                cov[7]  = cov_xy  # y variance
                cov[14] = 0.25    # z: 0.5 m std dev — EK3_SRC1_POSZ=6 uses this
                cov[21] = 0.09    # roll  (0.3 rad std)
                cov[28] = 0.09    # pitch
                cov[35] = 0.09    # yaw
                msg.pose.covariance = cov
                self._vpe_pub.publish(msg)
                n_sent += 1
                if n_sent == 1:
                    print("[Commander] VPE thread started — Phase 1 (home position)")

                # ── Horizontal velocity aiding (VISION_SPEED_ESTIMATE, ENU) ──────────
                # True velocity from differencing drone_state truth; MAVROS converts
                # ENU→NED.  Gives the EKF a direct, low-lag velocity measurement
                # (EK3_SRC1_VELXY=6) so the position controller has clean damping.
                if self._drone_state is not None:
                    ds = self._drone_state.pose.position
                    now_t = time.time()
                    if last_ds is not None:
                        dt_v = now_t - last_ds[2]
                        if dt_v > 1e-3:
                            vx = (ds.x - last_ds[0]) / dt_v   # ENU East  vel
                            vy = (ds.y - last_ds[1]) / dt_v   # ENU North vel
                            tw = TwistStamped()
                            tw.header.stamp = self.get_clock().now().to_msg()
                            tw.header.frame_id = "map"
                            tw.twist.linear.x = vx
                            tw.twist.linear.y = vy
                            tw.twist.linear.z = 0.0
                            self._vspeed_pub.publish(tw)
                    last_ds = (ds.x, ds.y, now_t)

                elapsed = time.time() - t0
                time.sleep(max(0.0, 0.05 - elapsed))  # 20 Hz — 5 Hz caused EKF dead-reckoning gaps

        t = threading.Thread(target=_loop, daemon=True)
        t.start()
        return t

    def _cb_state(self, msg: State):
        self._state = msg

    def _cb_local_pos(self, msg: PoseStamped):
        self._local_pos = msg

    def _cb_drone_state(self, msg: PoseStamped):
        self._drone_state = msg

    def _cb_mavlink(self, msg: Mavlink) -> None:
        # Use the full payload64 bytes (before MAVLink-2 zero truncation) for
        # size checks and unpacking.  MAVROS2 sets msg.len to the wire length
        # which may be smaller than the field span (e.g. EKF_STATUS_REPORT
        # msg.len=21 but flags sit at bytes [20:22]).  Padding bytes are zero
        # and don't affect flag values.
        raw = b"".join(x.to_bytes(8, "little") for x in msg.payload64)
        if msg.msgid == 193 and len(raw) >= 22:  # EKF_STATUS_REPORT: flags at byte 20
            self._ekf_flags = struct.unpack_from("<H", raw, 20)[0]
        elif msg.msgid == 49:  # GPS_GLOBAL_ORIGIN
            self._gps_origin_received = True
        elif msg.msgid == 36 and len(raw) >= 12:  # SERVO_OUTPUT_RAW: 4 motors after uint32
            ch = struct.unpack_from("<4H", raw, 4)
            self._last_motor_pwm = ch


    def set_ekf_origin(self, lat: float, lon: float, alt_msl_m: float,
                       timeout: float = 60.0) -> bool:
        """
        Publish GPS global origin to MAVROS2, which forwards it to ArduPilot SITL.

        ArduPilot SITL does NOT reliably echo GPS_GLOBAL_ORIGIN (msg 49) in response
        to a SET_GPS_GLOBAL_ORIGIN command — it only broadcasts msg 49 at boot or on
        its own schedule.  Blocking on the echo causes a 60 s timeout every run.

        Strategy: publish the origin repeatedly for 5 s (10 × 0.5 s) so SITL receives
        it even if it briefly misses the first packet.  If msg 49 arrives during that
        window we still log it and return early.  After 5 s of repeated publishes we
        treat the origin as set and return True — MAVROS2 forwarded it every time.
        """
        self.get_logger().info(
            f"Setting EKF origin: {lat:.6f}°N {lon:.6f}°E {alt_msl_m:.1f} m MSL")
        self._gps_origin_received = False

        origin_msg = GeoPointStamped()
        origin_msg.position.latitude  = lat
        origin_msg.position.longitude = lon
        origin_msg.position.altitude  = alt_msl_m

        # Phase 1 — publish 10× over 5 s; return early if ArduPilot echoes msg 49.
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

        # Phase 2 — no echo received, but origin was published 10 times.
        # ArduPilot SITL does not echo SET_GPS_GLOBAL_ORIGIN in this setup;
        # the origin is accepted silently.  Continue with a warning.
        self.get_logger().warn(
            "GPS_GLOBAL_ORIGIN echo not received (normal for this SITL build) — "
            "origin was published 10×; continuing. "
            "If waypoints land in the wrong place, restart SITL with --wipe.")
        return True

    # ── Blocking helpers ──────────────────────────────────────────────────────

    def _spin_until(self, condition, timeout=30.0, interval=0.05):
        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=interval)
            if condition():
                return True
        return False

    def wait_connected(self, timeout=60.0) -> bool:
        self.get_logger().info("Waiting for MAVROS2 connection …")
        ok = self._spin_until(lambda: self._state.connected, timeout)
        if ok:
            self.get_logger().info("MAVROS2 connected ✓")
        return ok

    def wait_ekf_pos(self, timeout=90.0) -> bool:
        """
        Block until EKF_POS_HORIZ_ABS is set (bit 4 of EKF_STATUS_REPORT flags).
        Uses _ekf_flags updated by _cb_mavlink via /uas1/mavlink_source.
        /mavros/estimator_status is NOT published in MAVROS2 Jazzy 2.14 at useful rate.
        """
        EKF_POS_HORIZ_ABS = 0x010
        _FLAG_NAMES = {
            0x001: "ATT", 0x002: "VEL_H", 0x004: "VEL_V",
            0x008: "POS_H_REL", 0x010: "POS_H_ABS", 0x020: "POS_V_ABS",
            0x040: "POS_V_AGL", 0x080: "CONST_POS",
        }
        self.get_logger().info(
            "Waiting for EKF POS_ABS — VPE must reach ArduPilot EKF3 …")

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

    def set_mode(self, mode: str, timeout=10.0) -> bool:
        req = SetMode.Request()
        req.custom_mode = mode
        future = self._mode_cli.call_async(req)
        deadline = time.time() + timeout
        while not future.done() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if future.done() and future.result().mode_sent:
            self.get_logger().info(f"Mode {mode} set ✓")
            return True
        self.get_logger().warn(f"Mode {mode} failed")
        return False

    def arm(self, timeout=10.0) -> bool:
        # Regular arm first
        req = CommandBool.Request()
        req.value = True
        future = self._arm_cli.call_async(req)
        deadline = time.time() + timeout
        while not future.done() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if future.done() and future.result().success:
            self.get_logger().info("Armed ✓")
            return True

        # Regular arm failed (GPS bad fix, VisOdom, etc.) — force arm.
        # Drain spin for 0.5 s before the second call: back-to-back service
        # calls with no gap can trigger a MAVROS2 internal 'Promise already
        # satisfied' crash when the first response arrives mid-second-call.
        self.get_logger().warn("Regular arm failed — retrying with force arm …")
        t_drain = time.time() + 0.5
        while time.time() < t_drain:
            rclpy.spin_once(self, timeout_sec=0.05)

        req2 = CommandLong.Request()
        req2.command  = 400      # MAV_CMD_COMPONENT_ARM_DISARM
        req2.param1   = 1.0      # arm
        req2.param2   = 21196.0  # force arm magic (bypasses all pre-arm checks)
        future2 = self._cmd_cli.call_async(req2)
        deadline = time.time() + timeout
        while not future2.done() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if future2.done() and future2.result().success:
            self.get_logger().info("Force armed ✓")
            return True

        self.get_logger().warn("Arm failed (regular + force)")
        return False

    def takeoff(self, alt_m: float, timeout: float = 180.0) -> bool:
        """
        Send NAV_TAKEOFF and wait — ArduPilot handles the climb and holds at alt_m.

        With DISARM_DELAY=0, ArduPilot's spool-up timer completes (0.5 s) and the
        altitude controller commands full throttle to reach the target.  We just
        monitor EKF altitude and return when done.
        """
        self.get_logger().info(f"Climbing to {alt_m:.0f} m AGL …")

        req = CommandTOL.Request()
        req.altitude = alt_m
        future = self._tof_cli.call_async(req)
        tof_deadline = time.time() + 10.0
        while not future.done() and time.time() < tof_deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if future.done():
            self.get_logger().info(
                f"NAV_TAKEOFF {'accepted' if future.result().success else 'rejected'}")
        else:
            self.get_logger().warn("NAV_TAKEOFF timed out — continuing anyway")

        deadline   = time.time() + timeout
        last_print = time.time()

        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

            agl = (self._drone_state.pose.position.z - HOME_ALT_MSL
                   if self._drone_state else
                   (self._local_pos.pose.position.z if self._local_pos else 0.0))

            now = time.time()
            if now - last_print > 3.0:
                mot = self._last_motor_pwm
                mot_str = f"  motors={list(mot)}" if mot else ""
                print(f"[Commander] AGL={agl:.1f} m  target={alt_m:.0f} m{mot_str}")
                last_print = now

                # If still on ground after 30 s, land detector deadlock — abort
                if now - deadline + timeout > 30.0 and agl < 2.0:
                    self.get_logger().warn(
                        "Drone not lifting after 30 s — land detector issue, "
                        "check DISARM_DELAY=0 loaded (param show DISARM_DELAY)")
                    return False

            if agl >= alt_m - 2.0:
                self.get_logger().info(f"Reached {alt_m:.0f} m AGL ✓")
                return True

        self.get_logger().warn("Takeoff timeout")
        return False

    def go_to_ned(self, north: float, east: float, down: float,
                  timeout=60.0) -> bool:
        """
        Send EKF position setpoint and wait until drone reaches it.

        Uses setpoint_raw/local with MAV_FRAME_LOCAL_NED so MAVROS passes
        x,y,z directly to ArduPilot with no coordinate conversion.
        NED: x=north, y=east, z=down (negative z = above origin).
        The vision_pose plugin converts ENU→NED correctly.

        Distance check uses /drone/state ENU truth (x=East, y=North, z=MSL).
        """
        # Speed-capped velocity "carrot" toward the target while holding altitude.
        # A raw 700 m position target makes ArduPilot command an unbounded, overshooting
        # velocity (WPNAV_SPEED is not enforced for setpoint_raw position), which drove a
        # growing oscillation/flyaway.  Instead command horizontal VELOCITY toward the
        # target, magnitude = min(SPEED_CAP, dist*APPROACH_GAIN), with position-Z holding
        # altitude.  Within WAYPOINT_RADIUS, fall back to a position-hold setpoint.
        # MAVROS setpoint_raw applies ftf::transform_frame_enu_ned, so send ENU and it
        # becomes the correct NED command.
        # Position "carrot": always command a position target only CARROT_DIST metres
        # ahead toward the goal, advancing it as the drone moves.  A near target keeps the
        # position controller in its gentle regime so it self-limits speed; a far raw
        # target made it command an unbounded, overshooting velocity (the velocity loop is
        # underdamped and won't track a velocity setpoint either).  Within CARROT_DIST,
        # command the real goal.  MAVROS applies ENU→NED, so send ENU.
        CARROT_DIST = 25.0      # metres ahead of the drone
        target_agl  = -down     # AGL metres

        _POS_MASK = (PositionTarget.IGNORE_VX  | PositionTarget.IGNORE_VY |
                     PositionTarget.IGNORE_VZ  | PositionTarget.IGNORE_AFX |
                     PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
                     PositionTarget.IGNORE_YAW | PositionTarget.IGNORE_YAW_RATE)

        deadline   = time.time() + timeout
        last_print = time.time()

        try:
            while time.time() < deadline:
                if self._drone_state is not None:
                    ds = self._drone_state.pose.position
                    cur_e, cur_n = ds.x, ds.y
                    dz = (ds.z - HOME_ALT_MSL) - target_agl
                elif self._local_pos is not None:
                    p  = self._local_pos.pose.position
                    cur_e, cur_n = p.x, p.y
                    dz = p.z - target_agl
                else:
                    rclpy.spin_once(self, timeout_sec=0.1)
                    continue

                dx = cur_e - east              # ENU east  error (drone − target)
                dy = cur_n - north             # ENU north error
                hdist = math.hypot(dx, dy)
                dist  = math.sqrt(dx**2 + dy**2 + dz**2)

                # Velocity "carrot" toward the goal (ENU), capped.  Empirically the velocity
                # loop tracks DIRECTION correctly (converges toward the WP) whereas raw
                # position setpoints are interpreted mirrored.  This is the best-performing
                # form so far: it converges from ~700 m to ~380 m, then the weak North/pitch
                # axis + underdamped velocity loop cause overshoot before the 60 m radius.
                # KNOWN-UNSOLVED: North/pitch authority is weak (ArduPilot pitch-for-North
                # decodes mostly as roll in the sim); see horizontal-flyaway-diagnosis memory.
                SPEED_CAP = 5.0
                msg = PositionTarget()
                msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
                msg.type_mask = (PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY |
                                 PositionTarget.IGNORE_VZ | PositionTarget.IGNORE_AFX |
                                 PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
                                 PositionTarget.IGNORE_YAW | PositionTarget.IGNORE_YAW_RATE)
                if hdist > 1e-3:
                    spd = min(SPEED_CAP, hdist * 0.4)
                    msg.velocity.x = float(-dx / hdist * spd)
                    msg.velocity.y = float(-dy / hdist * spd)
                msg.position.z = float(target_agl)
                msg.header.stamp = self.get_clock().now().to_msg()
                self._pos_pub.publish(msg)
                rclpy.spin_once(self, timeout_sec=0.1)

                now  = time.time()
                if now - last_print > 5.0:
                    agl = (self._drone_state.pose.position.z - HOME_ALT_MSL
                           if self._drone_state else float("nan"))
                    ekf = ""
                    if self._local_pos is not None:
                        p = self._local_pos.pose.position
                        ekf = f"  EKF=({p.x:+.0f},{p.y:+.0f})"
                    print(f"[Commander] WP  err N={dy:+.1f} E={dx:+.1f}"
                          f"  AGL={agl:.1f} m  dist={dist:.1f} m{ekf}")
                    last_print = now

                if dist <= WAYPOINT_RADIUS:
                    return True
        except Exception:
            pass
        return False

    def fly_auto_waypoint(self, north: float, east: float, agl: float,
                          timeout: float = 600.0) -> bool:
        """
        Fly to a waypoint using an AUTO-mode mission instead of GUIDED setpoints.

        Uploads a lat/lon mission and lets ArduPilot's own WPNAV controller fly it.
        This bypasses the MAVROS setpoint_raw frame conversion and GUIDED setpoint
        streaming entirely — a different navigation path than go_to_ned.
        """
        tgt_lat = HOME_LAT + north / M_PER_DEG
        tgt_lon = HOME_LON + east  / (M_PER_DEG * COS_LAT)
        self.get_logger().info(
            f"AUTO mission → {tgt_lat:.6f}°N {tgt_lon:.6f}°E {agl:.0f} m AGL")

        def mk(seq_current, cmd, lat, lon, alt, frame=3):
            wp = Waypoint()
            wp.frame = frame                     # 3 = GLOBAL_RELATIVE_ALT
            wp.command = cmd
            wp.is_current = seq_current
            wp.autocontinue = True
            wp.param1 = wp.param2 = wp.param3 = wp.param4 = 0.0
            wp.x_lat = float(lat); wp.y_long = float(lon); wp.z_alt = float(alt)
            return wp

        # seq0 = home (placeholder), seq1 = waypoint, seq2 = loiter at waypoint
        wps = [
            mk(False, 16, HOME_LAT, HOME_LON, 0.0, frame=0),     # NAV_WAYPOINT home
            mk(True,  16, tgt_lat,  tgt_lon,  agl),               # NAV_WAYPOINT target
            mk(False, 17, tgt_lat,  tgt_lon,  agl),               # NAV_LOITER_UNLIM
        ]

        # clear, then push
        for cli, name in ((self._wp_clear, "clear"), (self._wp_push, "push")):
            if not cli.wait_for_service(timeout_sec=5.0):
                self.get_logger().warn(f"mission {name} service unavailable")
                return False
        cf = self._wp_clear.call_async(WaypointClear.Request())
        self._spin_future(cf, 5.0)

        req = WaypointPush.Request(); req.start_index = 0; req.waypoints = wps
        pf = self._wp_push.call_async(req)
        if not self._spin_future(pf, 10.0) or not pf.result().success:
            self.get_logger().warn("mission push failed")
            return False
        self.get_logger().info(f"mission pushed ({pf.result().wp_transfered} items) ✓")

        if not self.set_mode("AUTO"):
            return False

        deadline = time.time() + timeout
        last_print = time.time()
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._drone_state is None:
                continue
            ds = self._drone_state.pose.position
            dx = ds.x - east; dy = ds.y - north
            dist = math.hypot(dx, dy)
            now = time.time()
            if now - last_print > 5.0:
                cur_agl = ds.z - HOME_ALT_MSL
                print(f"[Commander] AUTO  err N={dy:+.1f} E={dx:+.1f}  "
                      f"AGL={cur_agl:.1f} m  dist={dist:.1f} m")
                last_print = now
            if dist <= WAYPOINT_RADIUS:
                return True
        return False

    def _spin_future(self, future, timeout):
        deadline = time.time() + timeout
        while not future.done() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        return future.done()


    def velocity_probe(self):
        """
        Calibration: command known ENU velocity vectors and measure the actual
        ENU motion from /drone/state truth.  Reveals the commanded-direction →
        actual-motion transfer so the sim frame/sign bug can be pinned down
        empirically instead of by analysis.  Gated by env var CALIBRATE=1.
        """
        _IGNORE = (PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY |
                   PositionTarget.IGNORE_PZ | PositionTarget.IGNORE_AFX |
                   PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
                   PositionTarget.IGNORE_YAW | PositionTarget.IGNORE_YAW_RATE)

        def send_vel(vx, vy, secs):
            msg = PositionTarget()
            msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
            msg.type_mask = _IGNORE
            msg.velocity.x = float(vx); msg.velocity.y = float(vy); msg.velocity.z = 0.0
            t_end = time.time() + secs
            while time.time() < t_end:
                msg.header.stamp = self.get_clock().now().to_msg()
                self._pos_pub.publish(msg)
                rclpy.spin_once(self, timeout_sec=0.05)

        def pos():
            p = self._drone_state.pose.position
            return (p.x, p.y)  # East, North

        def att():
            # roll/pitch/yaw (deg) from drone_state ENU quaternion
            q = self._drone_state.pose.orientation
            sinr = 2*(q.w*q.x + q.y*q.z); cosr = 1-2*(q.x*q.x + q.y*q.y)
            roll = math.degrees(math.atan2(sinr, cosr))
            sinp = 2*(q.w*q.y - q.z*q.x)
            pitch = math.degrees(math.asin(max(-1, min(1, sinp))))
            siny = 2*(q.w*q.z + q.x*q.y); cosy = 1-2*(q.y*q.y + q.z*q.z)
            yaw = math.degrees(math.atan2(siny, cosy))
            return roll, pitch, yaw

        def vel_pulse(vx, vy):
            send_vel(0.0, 0.0, 3.0)        # settle/hover
            p0 = pos()
            t_end = time.time() + 4.0
            while time.time() < t_end:
                send_vel(vx, vy, 0.05)
            p1 = pos()
            send_vel(0.0, 0.0, 3.0)        # brake
            return (p1[0] - p0[0], p1[1] - p0[1])

        def ekf_yaw_enu():
            if self._local_pos is None:
                return float('nan')
            q = self._local_pos.pose.orientation
            siny = 2*(q.w*q.z + q.x*q.y); cosy = 1-2*(q.y*q.y + q.z*q.z)
            return math.degrees(math.atan2(siny, cosy))

        # Map VPE yaw command -> resulting EKF heading (no velocity cmds = no runaway).
        # The drone physically faces North (sim kyaw=0); we want the VPE yaw that makes
        # EKF heading = North (ENU 90°) so ArduPilot's NED axes line up.
        print("[CAL] ===== VPE-yaw -> EKF-heading map =====")
        for yaw_deg in (0, 45, 90, 135, 180, 225, 270, 315):
            self._vpe_yaw = math.radians(yaw_deg)
            send_vel(0.0, 0.0, 5.0)        # hold ~0 vel, let EKF adopt new yaw
            ey = ekf_yaw_enu()
            print(f"[CAL] vpe_yaw={yaw_deg:3d}° -> EKF_heading_ENU={ey:+6.1f}°  "
                  f"({'≈North(+90) ✓' if abs(((ey-90+180)%360)-180)<20 else ''})")
        print("[CAL] ===== yaw map done (want EKF_heading ≈ +90° = North) =====")
        self._vpe_yaw = math.pi / 2.0


# ── Main ───────────────────────────────────────────────────────────────────────

def main():
    rclpy.init()
    cmd = FlightCommander()

    # Step 1: start VPE thread immediately so VisOdom stays healthy
    stop_ev = threading.Event()
    cmd.start_vision_thread(stop_ev)

    # Step 2: wait for MAVROS2 to connect to SITL
    if not cmd.wait_connected(timeout=60.0):
        print("[Commander] MAVROS2 not connected — is SITL + launch_mavros.sh running?")
        stop_ev.set(); rclpy.shutdown(); return

    # Step 3: detect whether this is a fresh ground start or an in-air restart.
    # In-air restart: SITL + cesium are still running; only the commander was
    # restarted (e.g. after WP nav drifted the drone far from home at 90 m AGL).
    # In that case skip arming/takeoff and go straight to waypoints.
    for _ in range(20):
        rclpy.spin_once(cmd, timeout_sec=0.1)
        if cmd._drone_state is not None:
            break

    in_air_restart = False
    if cmd._drone_state is not None:
        start_agl = cmd._drone_state.pose.position.z - HOME_ALT_MSL
        if start_agl > 10.0:
            print(f"[Commander] In-air restart detected: drone at {start_agl:.0f} m AGL "
                  f"(E={cmd._drone_state.pose.position.x:+.0f} "
                  f"N={cmd._drone_state.pose.position.y:+.0f}) — "
                  "skipping takeoff sequence, proceeding to waypoints")
            in_air_restart = True
            # Ensure GUIDED mode — RTL or other modes may have triggered while
            # the commander was restarting and setpoints were briefly absent.
            if cmd._state.mode != "GUIDED":
                print(f"[Commander] Mode is {cmd._state.mode}, switching to GUIDED …")
                cmd.set_mode("GUIDED")
                cmd._spin_until(lambda: cmd._state.mode == "GUIDED", timeout=10.0)

    if not in_air_restart:
        # Step 4: wait for SITL EKF to initialize, then confirm EKF global origin.
        # Without confirmed origin, position setpoints use the wrong coordinate frame
        # and the drone flies to the wrong location (observed: 664 m displacement).
        # EKF typically needs 5-10 s to initialize after SITL starts; set_ekf_origin
        # retries every 2 s for up to 60 s and aborts if not confirmed.
        print("[Commander] Waiting 8 s for SITL EKF to initialize …")
        t_wait = time.time() + 8.0
        while time.time() < t_wait:
            rclpy.spin_once(cmd, timeout_sec=0.1)

        if not cmd.set_ekf_origin(HOME_LAT, HOME_LON, HOME_ALT_MSL, timeout=60.0):
            print("[Commander] ABORT: EKF origin not confirmed. Restart SITL with --wipe.")
            stop_ev.set(); cmd.destroy_node(); rclpy.shutdown(); return

        # Step 5: arm in STABILIZE first — only needs EKF attitude, not position.
        # This bypasses the GPS/VisOdom position requirement that blocks GUIDED arming.
        cmd.set_mode("STABILIZE")
        time.sleep(0.5)
        print("[Commander] Arming in STABILIZE (no EKF position required) …")
        if not cmd.arm():
            print("[Commander] Arm failed in STABILIZE — check IMU/bridge")
            stop_ev.set(); cmd.destroy_node(); rclpy.shutdown(); return

        # Step 6: switch to GUIDED now that motors are armed
        cmd.set_mode("GUIDED")
        time.sleep(0.5)

        # Step 6: wait for EKF POS_ABS — VPE must be accepted before takeoff command
        if not cmd.wait_ekf_pos(timeout=60.0):
            print("[Commander] EKF POS_ABS not reached — check VPE flow")
            stop_ev.set(); cmd.destroy_node(); rclpy.shutdown(); return

        # Step 7: Takeoff
        if not cmd.takeoff(TAKEOFF_ALT):
            print("[Commander] Takeoff failed")
            stop_ev.set(); cmd.destroy_node(); rclpy.shutdown(); return

        # Latch the hold NED target immediately when takeoff() returns (drone is at
        # TAKEOFF_ALT AGL).  Use drone_state for current horizontal position (should
        # be near 0,0 since we took off from home); fall back to exact home if the
        # topic is momentarily unavailable.  Altitude is latched from TAKEOFF_ALT —
        # more reliable than recomputing from MSL each iteration.
        # drone_state is ENU (x=East, y=North). Hold at current horizontal position.
        if cmd._drone_state is not None:
            _hold_north = cmd._drone_state.pose.position.y   # ENU y = North = NED x
            _hold_east  = cmd._drone_state.pose.position.x   # ENU x = East  = NED y
        else:
            _hold_north, _hold_east = 0.0, 0.0

        _IGNORE = (PositionTarget.IGNORE_VX  | PositionTarget.IGNORE_VY |
                   PositionTarget.IGNORE_VZ  | PositionTarget.IGNORE_AFX |
                   PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
                   PositionTarget.IGNORE_YAW | PositionTarget.IGNORE_YAW_RATE)
        _hold = PositionTarget()
        _hold.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        _hold.type_mask        = _IGNORE
        _hold.position.x       = float(_hold_east)   # ENU x = East  → MAVROS → NED East
        _hold.position.y       = float(_hold_north)  # ENU y = North → MAVROS → NED North
        _hold.position.z       = float(TAKEOFF_ALT)  # ENU z = Up → hold at takeoff altitude

        # Hold for 10 s — publish on every iteration so there is never a gap.
        print(f"[Commander] Holding 10 s at {TAKEOFF_ALT:.0f} m …")
        t_hold = time.time() + 10.0
        while time.time() < t_hold:
            _hold.header.stamp = cmd.get_clock().now().to_msg()
            cmd._pos_pub.publish(_hold)
            rclpy.spin_once(cmd, timeout_sec=0.05)

        # One final publish before handing off to go_to_ned.
        _hold.header.stamp = cmd.get_clock().now().to_msg()
        cmd._pos_pub.publish(_hold)

    # Calibration mode: probe the frame instead of flying the waypoint.
    if os.environ.get("CALIBRATE"):
        try:
            cmd.velocity_probe()
        except Exception as exc:
            print(f"[CAL] probe aborted: {exc}")
        print("[CAL] holding — Ctrl-C to exit")
        try:
            while True:
                rclpy.spin_once(cmd, timeout_sec=0.1)
        except KeyboardInterrupt:
            pass
        stop_ev.set(); cmd.destroy_node(); rclpy.shutdown(); return

    # Hold-in-place stability test: command the CURRENT position and log drift.
    # If it stays → position loop is stable (waypoint frame is the issue).
    # If it drifts away → the position loop itself is unstable.
    if os.environ.get("HOLDTEST"):
        for _ in range(10):
            rclpy.spin_once(cmd, timeout_sec=0.1)
        e0 = cmd._drone_state.pose.position.x if cmd._drone_state else 0.0
        n0 = cmd._drone_state.pose.position.y if cmd._drone_state else 0.0
        print(f"[HOLD] holding at E={e0:+.1f} N={n0:+.1f} AGL≈90 for 40 s")
        hmsg = PositionTarget()
        hmsg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        hmsg.type_mask = (PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY |
                          PositionTarget.IGNORE_VZ | PositionTarget.IGNORE_AFX |
                          PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
                          PositionTarget.IGNORE_YAW | PositionTarget.IGNORE_YAW_RATE)
        hmsg.position.x = float(e0); hmsg.position.y = float(n0); hmsg.position.z = float(TAKEOFF_ALT)
        t_end = time.time() + 40.0; t_log = 0.0
        while time.time() < t_end:
            hmsg.header.stamp = cmd.get_clock().now().to_msg()
            cmd._pos_pub.publish(hmsg)
            rclpy.spin_once(cmd, timeout_sec=0.05)
            if time.time() - t_log > 4.0 and cmd._drone_state is not None:
                t_log = time.time()
                e = cmd._drone_state.pose.position.x; n = cmd._drone_state.pose.position.y
                print(f"[HOLD] drift E={e-e0:+6.1f} N={n-n0:+6.1f}  dist={math.hypot(e-e0,n-n0):6.1f} m")
        print("[HOLD] done")
        stop_ev.set(); cmd.destroy_node(); rclpy.shutdown(); return

    # Step 8: Waypoints
    use_auto = bool(os.environ.get("AUTO_WP"))
    try:
        for i, (n, e, d) in enumerate(WAYPOINTS):
            print(f"[Commander] WP {i+1}/{len(WAYPOINTS)}  "
                  f"N={n:+.0f} E={e:+.0f} ALT={-d:.0f} m AGL  "
                  f"[{'AUTO mission' if use_auto else 'GUIDED setpoint'}]")
            if use_auto:
                reached = cmd.fly_auto_waypoint(n, e, -d, timeout=WAYPOINT_TIMEOUT)
            else:
                reached = cmd.go_to_ned(n, e, d, timeout=WAYPOINT_TIMEOUT)
            if reached and cmd._drone_state is not None:
                ds = cmd._drone_state.pose.position
                agl = ds.z - HOME_ALT_MSL
                dx = ds.x - e; dy = ds.y - n
                dist = math.sqrt(dx**2 + dy**2)
                print(f"[Commander] WP {i+1} ARRIVED ✓  "
                      f"pos E={ds.x:+.1f} N={ds.y:+.1f} AGL={agl:.1f} m  "
                      f"horiz_err={dist:.1f} m")
            else:
                print(f"[Commander] WP {i+1} {'✓' if reached else 'TIMEOUT — did not arrive'}")
            time.sleep(1.0)

        # Step 9: hold at target indefinitely (Ctrl-C to RTL)
        print("[Commander] Holding at 23.45564°N 120.28169°E — Ctrl-C to RTL")
        try:
            while True:
                rclpy.spin_once(cmd, timeout_sec=0.1)
        except KeyboardInterrupt:
            print("[Commander] Ctrl-C — RTL")
            cmd.set_mode("RTL")
            cmd._spin_until(lambda: not cmd._state.armed, timeout=150.0)
            print("[Commander] Disarmed — landed ✓")
    except Exception as exc:
        print(f"[Commander] Mission aborted: {exc}")
    finally:
        stop_ev.set()
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
