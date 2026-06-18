#!/usr/bin/env python3
"""
PX4 SITL simulator bridge (replaces sitl_bridge.py for the PX4 migration).

Speaks PX4's "Simulator MAVLink API" instead of ArduPilot's JSON FDM:

Protocol (PX4 is the CLIENT, this bridge is the SERVER on TCP 4560):
  1. Bridge listens on TCP 4560; PX4 SITL (`make px4_sitl none_iris`) connects.
  2. Bridge streams HIL_SENSOR (IMU accel/gyro body-FRD, mag, baro) to PX4.
  3. PX4 streams HIL_ACTUATOR_CONTROLS (16 normalised motor outputs) back.
  4. Bridge returns the motor outputs to the sim, which integrates physics.

Coordinate conventions (same scene as the ArduPilot bridge):
  Isaac/Cesium (ENU):  X=East,  Y=North, Z=Up
  PX4 sensors  (FRD body / NED earth):  body x=Forward, y=Right, z=Down

Reuses the kinematic→IMU math from sitl_bridge.py (specific force in body frame,
euler-rate gyro).  The actuator→motor decode is PX4 quad-X order (see _decode_motors).

Usage — embed in cesium_scene.py simulation loop, mirroring SITLBridge.step():
  bridge = PX4SimBridge(centre_elev=centre_elev)
  motors = bridge.step(x_enu, y_enu, z_abs, yaw_deg, roll_rad, pitch_rad, t)
  # motors: list of 4 normalised [0,1] motor outputs in PX4 order, or None.
"""

import math
import socket
import time

from pymavlink.dialects.v20 import common as mavlink2

_GRAVITY   = 9.80665   # m/s²
_MAX_SPEED = 30.0      # m/s — clamp computed velocity
_SEA_LEVEL_HPA = 1013.25

# HIL_SENSOR fields_updated bitmask: accel(0-2) gyro(3-5) mag(6-8)
# abs_pressure(9) diff_pressure(10) pressure_alt(11) temperature(12)
_FIELDS_ALL = 0x1FFF

# Earth magnetic field at the scene (Chiayi, Taiwan), NED, in Gauss (~0.45 G total).
# Roughly: north +0.39, east +0.0, down +0.23 (declination ~ -4°, inclination ~35°).
_MAG_NED = (0.390, -0.027, 0.225)


class PX4SimBridge:
    LISTEN_PORT = 4560

    def __init__(self, listen_port: int = LISTEN_PORT, centre_elev: float = 0.0):
        self._centre_elev = centre_elev
        self._port        = listen_port
        self._conn        = None          # accepted PX4 client socket
        self._connected   = False
        self._rx          = bytearray()   # TCP receive buffer
        self._last_motors = [0.0, 0.0, 0.0, 0.0]

        # Derivative state (for finite-difference velocity / accel / gyro)
        self._prev_pos_ned   = None
        self._prev_vel_ned   = None
        self._prev_accel_ned = None
        self._prev_yaw_rad   = None
        self._prev_roll_rad  = None
        self._prev_pitch_rad = None
        self._prev_t         = None
        self._start_t        = time.time()
        self._n_sent         = 0

        # MAVLink encoder/parser (file=None → we pack/parse manually over TCP)
        self._mav = mavlink2.MAVLink(None, srcSystem=1, srcComponent=1)

        # TCP server socket — PX4 connects to us
        self._srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._srv.setblocking(False)
        self._srv.bind(("0.0.0.0", listen_port))
        self._srv.listen(1)

        print(f"[PX4SITL] Bridge listening on TCP port {listen_port}  "
              f"(centre_elev={centre_elev:.1f} m MSL)")
        print("[PX4SITL] Waiting for PX4 SITL to connect …")

    # ── Public API ──────────────────────────────────────────────────────────
    def step(self, x_enu, y_enu, z_abs, yaw_deg,
             roll_rad=0.0, pitch_rad=0.0, wall_time=None):
        """One sim step: accept/serve PX4, send HIL_SENSOR, return motor outputs."""
        t = wall_time if wall_time is not None else time.time()
        self._accept_if_needed()

        # drain incoming HIL_ACTUATOR_CONTROLS (non-blocking)
        new_motors = self._recv_actuators()
        if new_motors is not None:
            self._last_motors = new_motors

        # build + send HIL_SENSOR
        if self._conn is not None:
            self._send_hil_sensor(x_enu, y_enu, z_abs, yaw_deg,
                                  roll_rad, pitch_rad, t)

        self._prev_t = t
        return list(self._last_motors)

    @property
    def connected(self) -> bool:
        return self._connected

    @property
    def last_motors(self):
        return list(self._last_motors)

    def close(self):
        try:
            if self._conn:
                self._conn.close()
        finally:
            self._srv.close()

    # ── TCP / MAVLink plumbing ──────────────────────────────────────────────
    def _accept_if_needed(self):
        if self._conn is not None:
            return
        try:
            conn, addr = self._srv.accept()
            conn.setblocking(False)
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self._conn = conn
            self._connected = True
            self._start_t = time.time()
            self._n_sent = 0
            print(f"[PX4SITL] PX4 connected from {addr} — streaming HIL_SENSOR")
        except (BlockingIOError, OSError):
            pass

    def _recv_actuators(self):
        if self._conn is None:
            return None
        # pull whatever bytes are available
        try:
            while True:
                chunk = self._conn.recv(4096)
                if not chunk:
                    print("[PX4SITL] PX4 disconnected")
                    self._conn = None; self._connected = False
                    return None
                self._rx.extend(chunk)
        except (BlockingIOError, OSError):
            pass

        latest = None
        try:
            msgs = self._mav.parse_buffer(bytes(self._rx)) or []
            self._rx.clear()
            for m in msgs:
                if m.get_type() == "HIL_ACTUATOR_CONTROLS":
                    latest = [max(0.0, min(1.0, float(c))) for c in m.controls[:4]]
        except Exception:
            self._rx.clear()
        return latest

    def _send_hil_sensor(self, x_enu, y_enu, z_abs, yaw_deg,
                         roll_rad, pitch_rad, t):
        acc_b, gyro_b, mag_b, pressure, palt = self._build_imu(
            x_enu, y_enu, z_abs, yaw_deg, roll_rad, pitch_rad, t)
        # PX4 sets its own CLOCK_MONOTONIC to this time_usec (even nolockstep).  Use a
        # large monotonic host timestamp so PX4's clock continues forward from boot
        # instead of jumping back to ~0 (which made baro/mag go STALE).
        ts = int(time.monotonic() * 1e6)
        msg = self._mav.hil_sensor_encode(
            ts,
            acc_b[0], acc_b[1], acc_b[2],
            gyro_b[0], gyro_b[1], gyro_b[2],
            mag_b[0], mag_b[1], mag_b[2],
            pressure, 0.0, palt, 25.0,
            _FIELDS_ALL, 0)
        try:
            self._conn.sendall(msg.pack(self._mav))
            self._n_sent += 1
            if self._n_sent == 1:
                print("[PX4SITL] First HIL_SENSOR sent ✓")
        except OSError as e:
            print(f"[PX4SITL] send error: {e}")
            self._conn = None; self._connected = False

    # ── Physics → IMU (ported from sitl_bridge._build_state) ────────────────
    def _build_imu(self, x_enu, y_enu, z_abs, yaw_deg, roll_rad, pitch_rad, t):
        north = y_enu; east = x_enu
        agl   = z_abs - self._centre_elev
        down  = -agl
        pos_ned = (north, east, down)

        if self._prev_pos_ned is not None and self._prev_t is not None:
            dt = t - self._prev_t
            if dt > 1e-6:
                vel_ned = tuple(max(-_MAX_SPEED, min(_MAX_SPEED,
                                (pos_ned[i] - self._prev_pos_ned[i]) / dt))
                                for i in range(3))
            else:
                vel_ned = self._prev_vel_ned or (0.0, 0.0, 0.0)
        else:
            vel_ned = (0.0, 0.0, 0.0)

        if self._prev_vel_ned is not None and self._prev_t is not None:
            dt = t - self._prev_t
            if dt > 1e-6:
                raw_a = tuple((vel_ned[i] - self._prev_vel_ned[i]) / dt
                              for i in range(3))
                if self._prev_accel_ned is not None:
                    a = 0.3
                    accel_ned = tuple(a*raw_a[i] + (1-a)*self._prev_accel_ned[i]
                                      for i in range(3))
                else:
                    accel_ned = raw_a
            else:
                accel_ned = self._prev_accel_ned or (0.0, 0.0, 0.0)
        else:
            accel_ned = (0.0, 0.0, 0.0)

        # body angular rates from euler-angle derivatives (small-angle ≈ body)
        yaw_rad = -math.radians(yaw_deg)   # CCW(ENU) → CW(NED)
        roll_rate = pitch_rate = yaw_rate = 0.0
        if self._prev_yaw_rad is not None and self._prev_t is not None:
            dt = t - self._prev_t
            if dt > 1e-6:
                dyaw = (yaw_rad - self._prev_yaw_rad + math.pi) % (2*math.pi) - math.pi
                yaw_rate   = dyaw / dt
                roll_rate  = (roll_rad  - self._prev_roll_rad)  / dt
                pitch_rate = (pitch_rad - self._prev_pitch_rad) / dt

        # specific force (accelerometer) in body frame: sf = accel - gravity
        sf_ned = (accel_ned[0], accel_ned[1], accel_ned[2] - _GRAVITY)
        cr, sr = math.cos(roll_rad),  math.sin(roll_rad)
        cp, sp = math.cos(pitch_rad), math.sin(pitch_rad)
        cy, sy = math.cos(yaw_rad),   math.sin(yaw_rad)
        sf_bx = sf_ned[0]*(cy*cp)            + sf_ned[1]*(sy*cp)            + sf_ned[2]*(-sp)
        sf_by = sf_ned[0]*(cy*sp*sr - sy*cr) + sf_ned[1]*(sy*sp*sr + cy*cr) + sf_ned[2]*(cp*sr)
        sf_bz = sf_ned[0]*(cy*sp*cr + sy*sr) + sf_ned[1]*(sy*sp*cr - cy*sr) + sf_ned[2]*(cp*cr)

        # magnetometer: Earth NED field rotated into body frame (same R_bn)
        mn, me, md = _MAG_NED
        mb_x = mn*(cy*cp)            + me*(sy*cp)            + md*(-sp)
        mb_y = mn*(cy*sp*sr - sy*cr) + me*(sy*sp*sr + cy*cr) + md*(cp*sr)
        mb_z = mn*(cy*sp*cr + sy*sr) + me*(sy*sp*cr - cy*sr) + md*(cp*cr)

        # barometer from altitude (ISA approximation)
        palt = agl + self._centre_elev
        pressure = _SEA_LEVEL_HPA * (1.0 - 2.25577e-5 * palt) ** 5.25588

        self._prev_pos_ned   = pos_ned
        self._prev_vel_ned   = vel_ned
        self._prev_accel_ned = accel_ned
        self._prev_yaw_rad   = yaw_rad
        self._prev_roll_rad  = roll_rad
        self._prev_pitch_rad = pitch_rad

        return ((sf_bx, sf_by, sf_bz), (roll_rate, pitch_rate, yaw_rate),
                (mb_x, mb_y, mb_z), pressure, palt)

    # ── Actuator decode (PX4 quad-X) ────────────────────────────────────────
    # Reference only — cesium_scene.py does the decode in the physics loop.
    # PX4 control allocation (none_iris CA_ROTOR geometry): controls[0..3] map to
    # motors at (PX=fwd, PY=right):  0=FR(+,+) 1=RL(-,-) 2=FL(+,-) 3=RR(-,+).
    # roll(+=right)=left(RL,FL)−right(FR,RR); pitch(+=nose up)=front(FR,FL)−rear(RL,RR).
    @staticmethod
    def _decode_motors(m):
        thrust = sum(m) / 4.0
        roll   = (m[1] + m[2]) - (m[0] + m[3])
        pitch  = (m[0] + m[2]) - (m[1] + m[3])
        return thrust, roll, pitch
