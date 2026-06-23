#!/usr/bin/env python3
"""
GPS-defined polygon lawnmower scan at 65 m altitude.
Scan lines run parallel to the long sides of the survey polygon.

Takeoff point : 23.452011, 120.285761
Survey polygon: 4 GPS corners — sorted CCW, long axis detected automatically.
"""
import json
import math
import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State, OverrideRCIn, PositionTarget
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL

# ── mission parameters ────────────────────────────────────────────────────────
ALTITUDE     = 65.0   # m AGL
CLIMB_SECS   = 35     # 65 m at ~3 m/s climb rate
WARMUP_SECS  = 120    # hover for Cesium tile warm-up
DESCEND_SECS = 60     # 65 m at ~1.5 m/s descent rate

ORIGIN_LAT   = 23.451615  # takeoff GPS — must match --custom-location in ardupilot_launch_tool.py
ORIGIN_LON   = 120.286446 # takeoff GPS — must match --custom-location in ardupilot_launch_tool.py

SURVEY_GPS = [
    (23.45695,  120.27399),
    (23.45174,  120.27314),
    (23.45564,  120.28169),
    (23.45044,  120.28062),
]
LINE_SPACING = 50.0   # m between scan lines (perpendicular to long axis)
DRONE_SPEED  = 2.5    # m/s — conservative budget speed (actual cruise ~4 m/s but avg ~2.5 m/s with transitions)
LEG_BUFFER   = 30     # extra seconds per leg (covers WP transition deceleration overhead)

PLAN_FILE      = '/mnt/raid5/franklin/IsaacSim/fly_plan.json'
ARRIVAL_RADIUS = 3.0    # m — enter arrival zone
ARRIVAL_SPEED  = 0.4    # m/s — must also be below this speed to confirm arrival;
                         #        prevents corner-cutting at cruise speed
APPROACH_DECEL = 0.05   # 1/s — vel_cmd = min(DRONE_SPEED, APPROACH_DECEL * dist)
                         #        ramp starts at 2.5/0.05=50 m; 0.15 m/s at 3 m zone entry

RATE_HZ = 20

# ── coordinate helpers ────────────────────────────────────────────────────────

def _gps_to_enu(lat, lon):
    north = (lat - ORIGIN_LAT) * 111320.0
    east  = (lon - ORIGIN_LON) * 111320.0 * math.cos(math.radians(ORIGIN_LAT))
    return east, north


def _sort_ccw(pts):
    cx = sum(p[0] for p in pts) / len(pts)
    cy = sum(p[1] for p in pts) / len(pts)
    return sorted(pts, key=lambda p: math.atan2(p[1] - cy, p[0] - cx))


def _find_long_axis(polygon):
    """
    Unit vector along the long axis of a quadrilateral polygon.
    Identifies the pair of opposite edges with the greatest combined length
    and averages their directions.
    """
    n = len(polygon)
    edges = []
    for i in range(n):
        x1, y1 = polygon[i]
        x2, y2 = polygon[(i + 1) % n]
        dx, dy = x2 - x1, y2 - y1
        L = math.sqrt(dx * dx + dy * dy)
        edges.append((L, dx / L, dy / L))

    # For a quadrilateral: opposite pairs are (0,2) and (1,3)
    if edges[0][0] + edges[2][0] >= edges[1][0] + edges[3][0]:
        a, b = (edges[0][1], edges[0][2]), (edges[2][1], edges[2][2])
    else:
        a, b = (edges[1][1], edges[1][2]), (edges[3][1], edges[3][2])

    # Make both directions point the same way
    if a[0] * b[0] + a[1] * b[1] < 0:
        b = (-b[0], -b[1])

    ux, uy = (a[0] + b[0]) / 2, (a[1] + b[1]) / 2
    mag = math.sqrt(ux * ux + uy * uy)
    return ux / mag, uy / mag


def _poly_u_at_v(polygon, u_hat, v_hat, v_coord):
    """U-axis intersections of the horizontal slice at v=v_coord within polygon."""
    us = []
    n = len(polygon)
    for i in range(n):
        x1, y1 = polygon[i]
        x2, y2 = polygon[(i + 1) % n]
        v1 = x1 * v_hat[0] + y1 * v_hat[1]
        v2 = x2 * v_hat[0] + y2 * v_hat[1]
        if v1 == v2:
            continue
        if min(v1, v2) <= v_coord <= max(v1, v2):
            t  = (v_coord - v1) / (v2 - v1)
            xi = x1 + t * (x2 - x1)
            yi = y1 + t * (y2 - y1)
            us.append(xi * u_hat[0] + yi * u_hat[1])
    return sorted(set(round(u, 3) for u in us))


def _rot_to_enu(u_coord, v_coord, u_hat, v_hat):
    return (u_coord * u_hat[0] + v_coord * v_hat[0],
            u_coord * u_hat[1] + v_coord * v_hat[1])


def _leg_budget(x0, y0, x1, y1):
    dist = math.sqrt((x1 - x0) ** 2 + (y1 - y0) ** 2)
    return max(15, int(dist / DRONE_SPEED) + LEG_BUDGET_EXTRA)

LEG_BUDGET_EXTRA = LEG_BUFFER


def _yaw_q(x0, y0, x1, y1):
    yaw = math.atan2(y1 - y0, x1 - x0)
    return math.sin(yaw / 2), math.cos(yaw / 2)

# ── build polygon and waypoints ───────────────────────────────────────────────

SURVEY_POLY = _sort_ccw([_gps_to_enu(lat, lon) for lat, lon in SURVEY_GPS])

_U_HAT = _find_long_axis(SURVEY_POLY)           # unit vector along long axis
_V_HAT = (-_U_HAT[1], _U_HAT[0])               # unit vector along short axis (scan spacing)


def _build_scan_waypoints():
    vs    = [p[0] * _V_HAT[0] + p[1] * _V_HAT[1] for p in SURVEY_POLY]
    v_min, v_max = min(vs), max(vs)

    wps = []
    prev_x, prev_y = 0.0, 0.0
    line_idx = 0
    v = v_min

    while v <= v_max + 1.0:
        us = _poly_u_at_v(SURVEY_POLY, _U_HAT, _V_HAT, v)
        if len(us) >= 2 and (us[-1] - us[0]) > 5.0:   # skip degenerate lines
            u_start = us[0]  if line_idx % 2 == 0 else us[-1]
            u_end   = us[-1] if line_idx % 2 == 0 else us[0]

            xs, ys = _rot_to_enu(u_start, v, _U_HAT, _V_HAT)
            xe, ye = _rot_to_enu(u_end,   v, _U_HAT, _V_HAT)

            turn_b = _leg_budget(prev_x, prev_y, xs, ys)
            qz_t, qw_t = _yaw_q(prev_x, prev_y, xs, ys)
            wps.append((xs, ys, turn_b, qz_t, qw_t))

            scan_b = _leg_budget(xs, ys, xe, ye)
            qz_s, qw_s = _yaw_q(xs, ys, xe, ye)
            wps.append((xe, ye, scan_b, qz_s, qw_s))

            prev_x, prev_y = xe, ye
            line_idx += 1
        v += LINE_SPACING
    return wps


_AUTO_SCAN_WPS = _build_scan_waypoints()   # polygon lawnmower — always available for fly_trace.py

def _load_plan():
    if os.path.isfile(PLAN_FILE):
        try:
            with open(PLAN_FILE) as f:
                data = json.load(f)
            wps = tuple((d['x'], d['y'], d['budget'], d['qz'], d['qw']) for d in data)
            print(f'[fly_test] USER PLAN loaded: {len(wps)} waypoints from fly_plan.json')
            return wps
        except Exception as e:
            print(f'[fly_test] WARNING: could not load fly_plan.json ({e}), using auto-scan')
    print('[fly_test] No fly_plan.json found — using auto polygon scan')
    return _AUTO_SCAN_WPS

_SCAN_WPS = _load_plan()

# ── ROS2 node ─────────────────────────────────────────────────────────────────

class FlyNode(Node):
    def __init__(self):
        super().__init__('fly_test')
        self.pub          = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self.sp_pub       = self.create_publisher(PositionTarget, '/mavros/setpoint_raw/local', 10)
        self.rc_pub       = self.create_publisher(OverrideRCIn, '/mavros/rc/override', 10)
        self.set_mode_cli = self.create_client(SetMode,     '/mavros/set_mode')
        self.arm_cli      = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.takeoff_cli  = self.create_client(CommandTOL,  '/mavros/cmd/takeoff')
        _be = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(PoseStamped,   '/mavros/local_position/pose',           self._on_pos, _be)
        self.create_subscription(TwistStamped,  '/mavros/local_position/velocity_local', self._on_vel, _be)
        self.create_subscription(State, '/mavros/state', self._on_state, 10)

        self._ekf_ready = False
        self._armed     = False
        self._guided    = False
        self._rc_ticks  = 0
        self._cur_x     = 0.0
        self._cur_y     = 0.0
        self._cur_z     = 0.0
        self._cur_spd   = 0.0

        self.count       = 0
        self.wp_index    = 0
        self.wp_ticks    = 0
        self.phase       = 'startup'
        self.phase_tick  = 0
        self.warmup_tick = 0
        self._scan_wps   = _SCAN_WPS   # reloaded fresh at warmup→scan

        self.target_x  = 0.0
        self.target_y  = 0.0
        self.target_z  = ALTITUDE
        self.target_qz = 0.0
        self.target_qw = 1.0

        angle_deg = math.degrees(math.atan2(_U_HAT[1], _U_HAT[0]))
        total_secs = sum(w[2] for w in _SCAN_WPS)
        self.get_logger().info(
            f'Scan: {len(_SCAN_WPS)} WPs, long-axis angle={angle_deg:.1f}°, '
            f'~{total_secs // 60} min scan time'
        )
        self.timer = self.create_timer(1.0 / RATE_HZ, self.tick)

    def _on_pos(self, msg):
        if not self._ekf_ready:
            self.get_logger().info('EKF3 origin set — local position available.')
        self._ekf_ready = True
        self._cur_x = msg.pose.position.x
        self._cur_y = msg.pose.position.y
        self._cur_z = msg.pose.position.z

    def _on_vel(self, msg):
        vx = msg.twist.linear.x
        vy = msg.twist.linear.y
        self._cur_spd = math.sqrt(vx * vx + vy * vy)

    def _on_state(self, msg):
        if not getattr(self, '_state_seen', False):
            self.get_logger().info(
                f'First /mavros/state: armed={msg.armed} mode={msg.mode!r}')
            self._state_seen = True
        prev_armed  = self._armed
        prev_guided = self._guided
        self._armed  = msg.armed
        self._guided = (msg.mode == 'GUIDED')
        if msg.armed and not prev_armed:
            self.get_logger().info('Vehicle armed.')
        if self._guided and not prev_guided:
            self.get_logger().info('Mode: GUIDED confirmed.')

    def _set_mode(self, mode):
        req = SetMode.Request(); req.custom_mode = mode
        self.set_mode_cli.call_async(req)

    def _arm(self, value: bool):
        req = CommandBool.Request(); req.value = value
        self.arm_cli.call_async(req)

    def _takeoff(self, altitude: float):
        req = CommandTOL.Request(); req.altitude = altitude
        future = self.takeoff_cli.call_async(req)
        future.add_done_callback(
            lambda f: self.get_logger().info(f'Takeoff: success={f.result().success}'))

    def _publish(self):
        msg = PoseStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = self.target_x
        msg.pose.position.y = self.target_y
        msg.pose.position.z = self.target_z
        msg.pose.orientation.z = self.target_qz
        msg.pose.orientation.w = self.target_qw
        self.pub.publish(msg)

    def _publish_vel(self):
        """
        Pure velocity setpoint toward target_x/y (ENU).
        MAVROS applies ENU→NED conversion before forwarding to ArduPilot.
        velocity.z=0 holds current altitude; position bits all ignored so
        ArduPilot stays in velocity sub-mode and flies a straight line.
        """
        dx   = self.target_x - self._cur_x
        dy   = self.target_y - self._cur_y
        dist = math.sqrt(dx * dx + dy * dy)
        if dist > 0.1:
            vel_mag = min(DRONE_SPEED, APPROACH_DECEL * dist)
            vel_x   = vel_mag * dx / dist
            vel_y   = vel_mag * dy / dist
        else:
            vel_x, vel_y = 0.0, 0.0

        sp = PositionTarget()
        sp.header.stamp    = self.get_clock().now().to_msg()
        sp.header.frame_id = 'map'
        sp.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        sp.type_mask = (
            PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ |
            PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW | PositionTarget.IGNORE_YAW_RATE
        )
        sp.velocity.x = vel_x   # ENU East  — MAVROS converts to NED
        sp.velocity.y = vel_y   # ENU North — MAVROS converts to NED
        sp.velocity.z = 0.0     # hold altitude
        self.sp_pub.publish(sp)

    def _set_target(self, x, y, budget, qz, qw):
        self.target_x  = x
        self.target_y  = y
        self.target_qz = qz
        self.target_qw = qw
        self.get_logger().info(
            f'WP {self.wp_index + 1}/{len(self._scan_wps)}: ({x:.0f}, {y:.0f}) m  {budget}s')

    # ── main loop ─────────────────────────────────────────────────────────────

    def tick(self):
        if self.phase == 'done':
            return

        # Skip publishing during startup/climb to avoid switching GUIDED submode
        # TakeOff→Pos while land_complete=True (Issue 22).
        if self.phase not in ('startup', 'climb'):
            if self.phase == 'scan':
                self._publish_vel()   # velocity setpoints → straight strip paths
            else:
                self._publish()       # position setpoints → hold during warmup/descend
        self.count += 1

        # ── startup ───────────────────────────────────────────────────────────
        if self.phase == 'startup':
            if self.count == 20:
                self.get_logger().info('Switching to GUIDED ...')
                self._set_mode('GUIDED')
            if self.count == 40:
                self.get_logger().info('Arming ...')
                self._arm(True)
            if self.count >= 80 and self.count % 40 == 0:
                elapsed = self.count / RATE_HZ
                if not self._guided:
                    self.get_logger().info(f'[{elapsed:.0f}s] Not yet GUIDED — re-issuing set_mode ...')
                    self._set_mode('GUIDED')
                elif not self._armed:
                    self.get_logger().info(
                        f'[{elapsed:.0f}s] Waiting for GPS/EKF convergence (~30 s) ...')
                    self._arm(True)
            if self.count > 60 and self._ekf_ready and self._armed and self._guided:
                self._rc_ticks += 1
                rc = OverrideRCIn()
                rc.channels = [0] * 18
                if self._rc_ticks <= RATE_HZ:
                    rc.channels[2] = 1100
                    self.rc_pub.publish(rc)
                    if self._rc_ticks == 1:
                        self.get_logger().info('Pulsing RC3=1100 to set auto_armed ...')
                else:
                    rc.channels[2] = 65535
                    self.rc_pub.publish(rc)
                    if self._rc_ticks == RATE_HZ + 1:
                        self.get_logger().info(f'Takeoff to {ALTITUDE:.0f} m ...')
                        self._takeoff(ALTITUDE)
                        self.phase      = 'climb'
                        self.phase_tick = 0
            return

        # ── climb ─────────────────────────────────────────────────────────────
        if self.phase == 'climb':
            self.phase_tick += 1
            if self.phase_tick >= CLIMB_SECS * RATE_HZ:
                self.get_logger().info(
                    f'Altitude reached — {WARMUP_SECS}s Cesium tile warm-up.')
                self.phase       = 'warmup'
                self.warmup_tick = 0
            return

        # ── warmup ────────────────────────────────────────────────────────────
        if self.phase == 'warmup':
            self.warmup_tick += 1
            if self.warmup_tick >= WARMUP_SECS * RATE_HZ:
                # Reload plan at the last moment so the user can save in fly_trace.py
                # during the warmup window and have it picked up here.
                self._scan_wps = _load_plan()
                self.get_logger().info(
                    f'Warm-up done — starting scan ({len(self._scan_wps)} WPs).')
                self._set_target(*self._scan_wps[0])
                self.wp_index = 0
                self.wp_ticks = 0
                self.phase    = 'scan'
            return

        # ── scan ──────────────────────────────────────────────────────────────
        if self.phase == 'scan':
            self.wp_ticks += 1
            dist_to_wp = math.sqrt(
                (self._cur_x - self.target_x) ** 2 +
                (self._cur_y - self.target_y) ** 2)
            budget_expired = self.wp_ticks >= self._scan_wps[self.wp_index][2] * RATE_HZ
            # Require both proximity AND low speed — prevents corner-cutting when
            # the drone enters the 3 m zone at cruise speed and would turn early.
            arrived = dist_to_wp < ARRIVAL_RADIUS and self._cur_spd < ARRIVAL_SPEED
            if arrived or budget_expired:
                if budget_expired and not arrived:
                    self.get_logger().warn(
                        f'WP {self.wp_index + 1} budget expired '
                        f'({dist_to_wp:.0f} m from target, spd={self._cur_spd:.2f} m/s) — skipping')
                self.wp_ticks  = 0
                self.wp_index += 1
                if self.wp_index < len(self._scan_wps):
                    self._set_target(*self._scan_wps[self.wp_index])
                else:
                    self.get_logger().info(f'Scan complete — descending from {ALTITUDE:.0f} m ...')
                    self.target_z   = 0.0
                    self.phase      = 'descend'
                    self.phase_tick = 0
            return

        # ── descend ───────────────────────────────────────────────────────────
        if self.phase == 'descend':
            self.phase_tick += 1
            if self.phase_tick >= DESCEND_SECS * RATE_HZ:
                self.get_logger().info('Switching to LAND ...')
                self._set_mode('LAND')
                self.phase      = 'land'
                self.phase_tick = 0
            return

        # ── land ──────────────────────────────────────────────────────────────
        if self.phase == 'land':
            self.phase_tick += 1
            if self.phase_tick >= 20 * RATE_HZ:
                self.get_logger().info('Landing complete — shutting down.')
                self._arm(False)
                self.phase = 'done'
                self.create_timer(2.0, lambda: rclpy.shutdown())


def main():
    rclpy.init()
    node = FlyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted — switching to LAND.')
        node._set_mode('LAND')
        import time; time.sleep(10)
    node.destroy_node()


if __name__ == '__main__':
    main()
