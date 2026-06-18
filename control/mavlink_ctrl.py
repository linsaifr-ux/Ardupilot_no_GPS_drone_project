#!/usr/bin/env python3
"""
ArduPilot MAVLink controller.

6b-i    pymavlink connection, non-blocking receive loop,
        HEARTBEAT / HIGHRES_IMU / ATTITUDE / LOCAL_POSITION_NED / EKF_STATUS_REPORT
6b-iii  send_vision_position() — AnyLoc → ArduPilot EKF3
6b-iv   set_mode(), arm(), takeoff(), set_position_ned() — flight commands
        wait_ekf_pos(), wait_command_ack(), wait_altitude(), wait_position()

Connection string examples
  "tcp:localhost:5762"  direct SITL TCP port (default, run_mavlink.py + run_flight.py)
  "tcp:localhost:5763"  second SITL TCP port (run_vision.py)

Usage:
  ctrl = MAVLinkCtrl()
  ctrl.wait_heartbeat()
  ctrl.wait_ekf_pos()
  ctrl.set_mode("GUIDED")
  ctrl.arm()
  ctrl.wait_command_ack(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM)
  ctrl.takeoff(10.0)
  ctrl.wait_altitude(10.0)
  ctrl.set_position_ned(20.0, 0.0, -10.0)
  ctrl.wait_position(20.0, 0.0, -10.0)
"""

import math
import time

from pymavlink import mavutil

# ── EKF_STATUS_REPORT flag bits (ArduPilot EKF3) ──────────────────────────────
EKF_ATTITUDE           = 1 << 0
EKF_VEL_HORIZ          = 1 << 1
EKF_VEL_VERT           = 1 << 2
EKF_POS_HORIZ_REL      = 1 << 3
EKF_POS_HORIZ_ABS      = 1 << 4   # GPS or vision absolute position fused
EKF_POS_VERT_ABS       = 1 << 5
EKF_POS_VERT_AGL       = 1 << 6
EKF_CONST_POS_MODE     = 1 << 7
EKF_PRED_POS_HORIZ_REL = 1 << 8
EKF_PRED_POS_HORIZ_ABS = 1 << 9   # vision estimate accepted by EKF3
EKF_UNINITIALIZED      = 1 << 10  # EKF has not finished initialising


class MAVLinkCtrl:
    """
    pymavlink wrapper for no-GPS drone control.

    Maintains latest state from the MAVLink stream. Non-blocking: call
    recv() each loop iteration to drain pending messages without stalling
    the main sim or localisation loop.
    """

    def __init__(self, connection_str: str = "tcp:localhost:5762",
                 source_system: int = 255):
        """
        connection_str : pymavlink connection string
            "tcp:localhost:5762"  — direct SITL TCP port 2 (default; no mavproxy needed)
            "tcp:localhost:5760"  — SITL TCP port 1 (mavproxy uses this; avoid conflict)
            "tcp:localhost:5763"  — SITL TCP port 3 (spare)
        source_system  : GCS MAVLink system ID (must not clash with vehicle = 1)
        """
        self._mav = mavutil.mavlink_connection(
            connection_str,
            source_system=source_system,
            dialect="ardupilotmega",
        )
        self._connected = False

        # Latest received messages (None until first arrival)
        self._heartbeat = None
        self._imu       = None   # HIGHRES_IMU
        self._attitude  = None   # ATTITUDE
        self._local_pos = None   # LOCAL_POSITION_NED
        self._ekf       = None   # EKF_STATUS_REPORT

        # 6b-iv state
        self._last_ack  = {}     # cmd_id → MAV_RESULT (populated by COMMAND_ACK)
        self._armed     = False  # updated from HEARTBEAT base_mode

        print(f"[MAVLink] Connecting to {connection_str} …")

    # ── Public API ─────────────────────────────────────────────────────────────

    def wait_heartbeat(self, timeout: float = 60.0) -> bool:
        """
        Block until the first HEARTBEAT arrives or timeout expires.
        Returns True on success, False on timeout.
        """
        print("[MAVLink] Waiting for HEARTBEAT …")
        deadline = time.time() + timeout
        while time.time() < deadline:
            msg = self._mav.recv_match(type="HEARTBEAT", blocking=True,
                                       timeout=1.0)
            if msg is not None:
                self._heartbeat = msg
                self._connected = True
                print(f"[MAVLink] HEARTBEAT  sysid={self._mav.target_system} "
                      f"compid={self._mav.target_component} "
                      f"type={msg.type} autopilot={msg.autopilot}")
                self._request_streams()
                return True
        print("[MAVLink] Timeout waiting for HEARTBEAT")
        return False

    def recv(self) -> list:
        """
        Non-blocking drain of all pending MAVLink messages.
        Updates internal state; returns list of type-name strings received.
        """
        received = []
        while True:
            msg = self._mav.recv_match(blocking=False)
            if msg is None:
                break
            t = msg.get_type()
            if   t == "HEARTBEAT":
                self._heartbeat = msg
                self._connected = True
                self._armed = bool(msg.base_mode &
                                   mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            elif t == "HIGHRES_IMU":        self._imu       = msg
            elif t == "ATTITUDE":           self._attitude  = msg
            elif t == "LOCAL_POSITION_NED": self._local_pos = msg
            elif t == "EKF_STATUS_REPORT":  self._ekf       = msg
            elif t == "COMMAND_ACK":        self._last_ack[msg.command] = msg.result
            received.append(t)
        return received

    # ── State properties ───────────────────────────────────────────────────────

    @property
    def connected(self) -> bool:
        return self._connected

    @property
    def imu(self):
        """Latest HIGHRES_IMU message, or None."""
        return self._imu

    @property
    def attitude(self):
        """Latest ATTITUDE message, or None."""
        return self._attitude

    @property
    def local_pos(self):
        """Latest LOCAL_POSITION_NED message, or None."""
        return self._local_pos

    @property
    def ekf_flags(self) -> int:
        """Latest EKF_STATUS_REPORT flags bitmask (0 until first message)."""
        return self._ekf.flags if self._ekf is not None else 0

    @property
    def ekf_pos_valid(self) -> bool:
        """True when EKF3 has accepted an absolute position source (GPS or vision)."""
        return bool(self.ekf_flags & EKF_POS_HORIZ_ABS)

    @property
    def is_armed(self) -> bool:
        """True when ArduPilot reports motors armed (from HEARTBEAT base_mode)."""
        return self._armed

    # ── 6b-iv: Blocking helpers ────────────────────────────────────────────────

    def set_mode(self, mode_name: str) -> None:
        """Set ArduPilot flight mode by name, e.g. 'GUIDED', 'RTL', 'LAND'."""
        mode_id = self._mav.mode_mapping().get(mode_name)
        if mode_id is None:
            raise ValueError(f"Unknown mode '{mode_name}'. "
                             f"Available: {list(self._mav.mode_mapping().keys())}")
        self._mav.mav.command_long_send(
            self._mav.target_system, self._mav.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id,
            0, 0, 0, 0, 0,
        )
        print(f"[MAVLink] SET_MODE {mode_name} (id={mode_id}) sent")

    def wait_ekf_pos(self, timeout: float = 60.0) -> bool:
        """Block until EKF3 has POS_ABS, or timeout. Returns True on success."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            self.recv()
            if self.ekf_pos_valid:
                return True
            time.sleep(0.05)
        return False

    def wait_visodom_healthy(self, timeout: float = 30.0) -> bool:
        """
        Block until EKF3 has both POS_ABS and PRED_POS_ABS set.
        PRED_POS_ABS (bit 9) means EKF is predicting future position from VPE,
        which implies AP_VisualOdom::healthy() is satisfied — allowing regular arm
        without needing force arm to bypass the mandatory VisOdom pre-arm check.
        """
        need = EKF_POS_HORIZ_ABS | EKF_PRED_POS_HORIZ_ABS
        deadline = time.time() + timeout
        while time.time() < deadline:
            self.recv()
            if (self.ekf_flags & need) == need:
                return True
            time.sleep(0.05)
        return False

    def wait_command_ack(self, cmd_id: int, timeout: float = 10.0) -> int | None:
        """
        Block until COMMAND_ACK for cmd_id arrives.
        Returns MAV_RESULT int (0 = ACCEPTED) or None on timeout.
        """
        deadline = time.time() + timeout
        while time.time() < deadline:
            self.recv()
            if cmd_id in self._last_ack:
                return self._last_ack.pop(cmd_id)
            time.sleep(0.05)
        return None

    def wait_altitude(self, target_agl: float, tolerance: float = 1.0,
                      timeout: float = 30.0) -> bool:
        """Block until LOCAL_POSITION_NED.z is within tolerance of -target_agl."""
        target_down = -target_agl
        deadline = time.time() + timeout
        while time.time() < deadline:
            self.recv()
            if self._local_pos is not None:
                if abs(self._local_pos.z - target_down) <= tolerance:
                    return True
            time.sleep(0.05)
        return False

    def wait_position(self, north: float, east: float, down: float,
                      radius: float = 2.0, timeout: float = 30.0) -> bool:
        """Block until drone is within radius metres of (north, east, down) NED."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            self.recv()
            p = self._local_pos
            if p is not None:
                dist = math.sqrt((p.x - north)**2 + (p.y - east)**2 + (p.z - down)**2)
                if dist <= radius:
                    return True
            time.sleep(0.05)
        return False

    # ── EKF origin (required for no-GPS operation) ────────────────────────────

    def set_ekf_origin(self, lat: float, lon: float, alt_msl_m: float) -> None:
        """
        Send SET_GPS_GLOBAL_ORIGIN to define the EKF3 NED reference frame.

        Must be called before sending VISION_POSITION_ESTIMATE. Without this
        ArduPilot has no absolute coordinate reference, so VPE messages cannot
        anchor the EKF — causing 'EKF attitude is bad' and 'VisOdom: not healthy'.
        """
        self._mav.mav.set_gps_global_origin_send(
            self._mav.target_system,
            int(lat * 1e7),          # 1e-7 degrees
            int(lon * 1e7),
            int(alt_msl_m * 1e3),    # mm MSL
            int(time.time() * 1e6),  # time_usec
        )
        print(f"[MAVLink] SET_GPS_GLOBAL_ORIGIN  lat={lat:.6f} lon={lon:.6f} alt={alt_msl_m:.1f} m")

    def set_home_position(self, lat: float, lon: float, alt_msl_m: float) -> None:
        """Send SET_HOME_POSITION to define the RTL target."""
        self._mav.mav.set_home_position_send(
            self._mav.target_system,
            int(lat * 1e7),
            int(lon * 1e7),
            int(alt_msl_m * 1e3),
            0.0, 0.0, 0.0,        # local NED x/y/z (unused when lat/lon/alt given)
            [1.0, 0.0, 0.0, 0.0], # quaternion (identity, not used)
            0.0, 0.0, 0.0,        # approach vector (unused)
            int(time.time() * 1e6),
        )
        print(f"[MAVLink] SET_HOME_POSITION      lat={lat:.6f} lon={lon:.6f} alt={alt_msl_m:.1f} m")

    # ── 6b-iii: Vision position ────────────────────────────────────────────────

    def send_vision_position(self, north: float, east: float, down: float,
                             yaw_rad: float,
                             covariance: list | None = None) -> None:
        """
        Send VISION_POSITION_ESTIMATE to ArduPilot EKF3.

        north, east, down : NED position in metres from the EKF origin (home)
        yaw_rad           : heading (rad, NED compass convention, CW-positive)
        covariance        : 21-element upper-triangle of 6×6 pose covariance [x,y,z,r,p,y].
                            Default: 20 m horizontal std (AnyLoc ~15–20 m error),
                            5 m vertical std, 0.3 rad yaw std.
        """
        if covariance is None:
            pxy = 20.0 ** 2   # 20 m horizontal std → 400 m² variance
            pz  =  5.0 ** 2   # 5 m vertical std
            ov  =  0.3 ** 2   # 0.3 rad (~17°) orientation std
            covariance = [
                pxy, 0,   0,   0,  0,  0,
                     pxy, 0,   0,  0,  0,
                          pz,  0,  0,  0,
                               ov, 0,  0,
                                   ov, 0,
                                       ov,
            ]
        self._mav.mav.vision_position_estimate_send(
            int(time.time() * 1e6),   # time_usec
            float(north), float(east), float(down),
            0.0, 0.0, float(yaw_rad),
            covariance,
            0,                         # reset_counter
        )

    # ── 6b-iv: Flight commands ─────────────────────────────────────────────────

    def arm(self, force: bool = False) -> None:
        """
        Send arm command.
        TODO (6b-iv): call only after ekf_pos_valid is True.
        """
        self._mav.mav.command_long_send(
            self._mav.target_system, self._mav.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,                  # param1: 1=arm
            21196 if force else 0,  # param2: force-arm magic (bypasses pre-arm checks)
            0, 0, 0, 0, 0,
        )
        print("[MAVLink] ARM command sent")

    def takeoff(self, alt_m: float) -> None:
        """
        Command takeoff to alt_m AGL.
        TODO (6b-iv): call after arm() ACK received.
        """
        self._mav.mav.command_long_send(
            self._mav.target_system, self._mav.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, alt_m,
        )
        print(f"[MAVLink] TAKEOFF {alt_m:.1f} m command sent")

    def set_position_ned(self, north: float, east: float, down: float,
                         yaw_rad: float | None = None) -> None:
        """
        Fly to NED position in metres from EKF origin.
        Optionally sets heading; velocity + acceleration fields are ignored.
        TODO (6b-iv): replaces keyboard control once EKF position is valid.
        """
        # type_mask: bit=1 → ignore that field
        # ignore velocity (bits 3-5), acceleration (bits 6-8), yaw_rate (bit 11)
        # keep position (bits 0-2); optionally keep yaw (bit 10)
        type_mask = 0b111111111000  # ignore vel + accel + yaw + yaw_rate
        if yaw_rad is not None:
            type_mask &= ~(1 << 10)  # clear "ignore yaw" → enable yaw

        self._mav.mav.set_position_target_local_ned_send(
            0,  # time_boot_ms (0 = use current time)
            self._mav.target_system, self._mav.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            type_mask,
            north, east, down,
            0, 0, 0,                # velocity (ignored)
            0, 0, 0,                # acceleration (ignored)
            yaw_rad if yaw_rad is not None else 0.0,
            0,                      # yaw_rate (ignored)
        )

    def close(self) -> None:
        self._mav.close()

    # ── Internal ───────────────────────────────────────────────────────────────

    def _request_streams(self) -> None:
        """Ask ArduPilot to send data streams at 10 Hz, HIGHRES_IMU at 50 Hz."""
        # Broad request — covers ATTITUDE, LOCAL_POSITION_NED, EKF_STATUS_REPORT
        self._mav.mav.request_data_stream_send(
            self._mav.target_system, self._mav.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            10,  # 10 Hz
            1,   # start
        )
        # HIGHRES_IMU is not included in any standard stream group — request explicitly.
        # 40 000 µs = 25 Hz, safely below SCHED_LOOP_RATE=50 to avoid "rate too fast" warning.
        self._mav.mav.command_long_send(
            self._mav.target_system, self._mav.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            mavutil.mavlink.MAVLINK_MSG_ID_HIGHRES_IMU,
            40000,  # 40 000 µs = 25 Hz
            0, 0, 0, 0, 0,
        )
        print("[MAVLink] Requested all streams at 10 Hz, HIGHRES_IMU at 25 Hz")
