#!/usr/bin/env python3
"""
Drone kinematic simulator — ROS2 / SITL bridge process.

Runs the 6-DOF kinematic physics model and the ArduPilot SITL bridge in a
single lightweight process, completely independent of Isaac Sim.

Publishes /drone/state (PoseStamped, frame_id="local_enu") so that
cesium_scene.py (pure Isaac Sim visualiser) can subscribe and move the drone
mesh.  All flight-critical work (physics + bridge) continues even if Isaac Sim
is not running.

Architecture:
  ArduPilot SITL  ←→ (UDP 9002)  ←→  drone_sim.py  →  /drone/state  →  cesium_scene.py
                                       ↑ reads
                                  home_elevation.json  (written by cesium_scene.py at startup;
                                  falls back to 28.17 m if absent)

Launch order (headless, no Isaac Sim):
  1. sim_vehicle.py  (ArduPilot SITL --model=JSON)
  2. python3 control/drone_sim.py
  3. launch_mavros.sh
  4. python3 control/flight_commander.py

Launch order (full, with Isaac Sim):
  1. cesium_scene.py      (writes home_elevation.json)
  2. sim_vehicle.py
  3. python3 control/drone_sim.py
  4. launch_mavros.sh
  5. anyloc/ros2_node.py
  6. python3 control/flight_commander.py
"""

import csv
import json
import math
import os
import sys
import time

# ROS2 Jazzy site-packages — must precede rclpy import when running in conda
_ROS2_SITE = "/opt/ros/jazzy/lib/python3.12/site-packages"
if os.path.isdir(_ROS2_SITE) and _ROS2_SITE not in sys.path:
    sys.path.insert(0, _ROS2_SITE)

import rclpy
import rclpy.node
from geometry_msgs.msg import PoseStamped

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# PX4_SIM=1 → PX4 SITL (MAVLink HIL on TCP 4560); else ArduPilot (JSON UDP 9002).
# Lightweight headless physics rig (no Isaac Sim) for control-loop validation.
_PX4_SIM = bool(os.environ.get("PX4_SIM"))
if _PX4_SIM:
    from control.px4_sim_bridge import PX4SimBridge
else:
    from control.sitl_bridge import SITLBridge

# ── Home position ──────────────────────────────────────────────────────────────
_HOME_CFG = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          "home_elevation.json")
try:
    with open(_HOME_CFG) as _f:
        _cfg = json.load(_f)
    HOME_LAT     = float(_cfg.get("lat",           23.450868))
    HOME_LON     = float(_cfg.get("lon",          120.286135))
    HOME_ALT_MSL = float(_cfg["centre_elev_m"])
    print(f"[drone_sim] HOME_ALT_MSL = {HOME_ALT_MSL:.1f} m  (from {_HOME_CFG})")
except (FileNotFoundError, KeyError):
    HOME_LAT     = 23.450868
    HOME_LON     = 120.286135
    HOME_ALT_MSL = 28.17
    print(f"[drone_sim] HOME_ALT_MSL = {HOME_ALT_MSL:.1f} m  (default — start cesium_scene.py first for real value)")

M_PER_DEG = 111_320.0
COS_LAT   = math.cos(math.radians(HOME_LAT))

# ── Physics constants (same as cesium_scene.py) ────────────────────────────────
_K_GRAVITY  = 9.81    # m/s²
_K_MAX_VEL  = 15.0    # m/s velocity clamp
_K_MAX_TILT = 0.35    # rad (~20°) max tilt from PWM differential
_K_TILT_TAU    = 0.15   # s first-order attitude time constant (ArduPilot only)
_K_PITCH_ACCEL = 80.0  # rad/s² per unit motor diff × mean_p (PX4 only)
_K_PITCH_DAMP  = 12.0  # angular damping s⁻¹ → time constant ≈ 83 ms (PX4 only)
_K_DRAG        = 0.35  # aerodynamic drag coefficient (s⁻¹)


def _euler_to_quat(roll: float, pitch: float, yaw: float):
    """ZYX Euler angles (rad) → quaternion (x, y, z, w)."""
    cy, sy = math.cos(yaw / 2),   math.sin(yaw / 2)
    cr, sr = math.cos(roll / 2),  math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    return (
        sr * cp * cy - cr * sp * sy,   # x
        cr * sp * cy + sr * cp * sy,   # y
        cr * cp * sy - sr * sp * cy,   # z
        cr * cp * cy + sr * sp * sy,   # w
    )


def main():
    rclpy.init()
    node = rclpy.node.Node("drone_sim")

    state_pub = node.create_publisher(PoseStamped, "/drone/state", 1)

    # SITL bridge — ArduPilot (UDP 9002) or PX4 (TCP 4560 MAVLink HIL)
    try:
        if _PX4_SIM:
            bridge = PX4SimBridge(listen_port=4560, centre_elev=HOME_ALT_MSL)
        else:
            bridge = SITLBridge(listen_port=9002, centre_elev=HOME_ALT_MSL)
            bridge.debug_hz = 0.2   # print physics state 5× per second
    except OSError as e:
        if e.errno == 98:   # EADDRINUSE
            print("[drone_sim] ERROR: bridge port already in use — another bridge running?")
            node.destroy_node(); rclpy.shutdown(); return
        raise

    # ── Kinematic state ────────────────────────────────────────────────────────
    _kx      = 0.0            # ENU east of home (m)
    _ky      = 0.0            # ENU north of home (m)
    _kz      = HOME_ALT_MSL   # altitude MSL (m) — starts at ground
    _kvn     = 0.0            # velocity north (m/s)
    _kve     = 0.0            # velocity east  (m/s)
    _kvd     = 0.0            # velocity NED down (m/s); negative = ascending
    _kroll       = 0.0         # estimated roll  (rad)
    _kpitch      = 0.0         # estimated pitch (rad)
    _kroll_rate  = 0.0         # roll angular rate (rad/s) — PX4 second-order model
    _kpitch_rate = 0.0         # pitch angular rate (rad/s) — PX4 second-order model
    _kyaw_rad = 0.0            # heading (rad, NED CW); no PWM-driven yaw torque
    _kprev_t = None
    _dbg_t   = 0.0            # last motor-debug print time
    _dbg_start = time.time()  # mission start for elapsed time

    # ── Flight trace CSV ──────────────────────────────────────────────────────
    _TRACE_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                              "simulator", "flight_traces")
    os.makedirs(_TRACE_DIR, exist_ok=True)
    _trace_path = os.path.join(_TRACE_DIR,
                               f"trace_{time.strftime('%Y%m%d_%H%M%S')}.csv")
    _trace_f   = open(_trace_path, "w", newline="", buffering=1)
    _trace_csv = csv.writer(_trace_f)
    _trace_csv.writerow(["t_s", "east_m", "north_m", "agl_m", "vn_ms", "ve_ms"])
    _trace_last_t = 0.0   # last time a row was written (5 Hz decimation)
    print(f"[drone_sim] Flight trace → {_trace_path}")

    bridge_name = "PX4 HIL (TCP 4560)" if _PX4_SIM else "ArduPilot (UDP 9002)"
    print(f"[drone_sim] Kinematic drone on ground — waiting for {bridge_name} …")
    print(f"[drone_sim] Home: AGL=0  MSL={HOME_ALT_MSL:.1f} m")

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            t_now = time.time()
            _kdt = min(t_now - _kprev_t, 0.05) if _kprev_t is not None else 0.0
            _kprev_t = t_now

            # ── Step bridge: send current physics state, receive servo PWM ────
            yaw_deg = math.degrees(-_kyaw_rad)   # NED CW rad → Isaac CCW deg
            servos = bridge.step(_kx, _ky, _kz, yaw_deg,
                                 _kroll, _kpitch, t_now)

            # ── Integrate 6-DOF kinematic model from motor outputs ────────────
            if _PX4_SIM:
                _p4 = [float(v) for v in servos[:4]] if servos else [0.0, 0.0, 0.0, 0.0]
            else:
                if servos is not None:
                    _p4 = [max(0.0, (v - 1000) / 1000.0) for v in servos["pwm"][:4]]
                else:
                    _p4 = None
            if _p4 is not None and _kdt > 0:
                _mean_p  = sum(_p4) / 4.0
                _kthrust = _mean_p * 2.0 * _K_GRAVITY   # 0 – 2 g

                if _PX4_SIM:
                    # Second-order angular dynamics: motor torque → rate → angle.
                    # pitch_diff>0 = front(m0,m2) > rear(m1,m3) = nose DOWN in FRD = forward.
                    # roll_diff>0  = left(m1,m2)  > right(m0,m3) = roll RIGHT = eastward.
                    pitch_diff = (_p4[0] + _p4[2]) - (_p4[1] + _p4[3])
                    roll_diff  = (_p4[1] + _p4[2]) - (_p4[0] + _p4[3])
                    _kpitch_rate += (_K_PITCH_ACCEL * _mean_p * pitch_diff
                                     - _K_PITCH_DAMP * _kpitch_rate) * _kdt
                    _kroll_rate  += (_K_PITCH_ACCEL * _mean_p * roll_diff
                                     - _K_PITCH_DAMP * _kroll_rate)  * _kdt
                    _kpitch = max(-_K_MAX_TILT, min(_K_MAX_TILT,
                                                     _kpitch + _kpitch_rate * _kdt))
                    _kroll  = max(-_K_MAX_TILT, min(_K_MAX_TILT,
                                                     _kroll  + _kroll_rate  * _kdt))

                    # Motor debug every 5 s
                    if t_now - _dbg_t >= 5.0:
                        _dbg_t = t_now
                        _agl = _kz - HOME_ALT_MSL
                        print(f"[SIM] t={t_now-_dbg_start:5.0f}s  "
                              f"m=[{_p4[0]:.3f},{_p4[1]:.3f},{_p4[2]:.3f},{_p4[3]:.3f}]"
                              f"  mean={_mean_p:.3f}  pd={pitch_diff:+.3f}  p={_kpitch:+.3f}"
                              f"  pr={_kpitch_rate:+.2f}  vN={_kvn:+.2f}  vE={_kve:+.2f}"
                              f"  AGL={_agl:.1f}m")
                else:
                    # ArduCopter QUAD-X (FRAME_TYPE=1): Motor→Servo mapping from AP_MotorsMatrix:
                    #   Motor 1(FR,45°)→Servo1=ch1, Motor2(RL,-135°)→Servo2=ch2,
                    #   Motor 3(RR,-45°)→Servo3=ch3, Motor4(FL,135°)→Servo4=ch4
                    # Roll  right = (FR+RR) - (RL+FL) = right_side - left_side
                    # Pitch fwd   = (FR+FL) - (RL+RR) = front - rear  → negative = nose down = northward
                    _roll_tgt  = ((_p4[0] + _p4[2]) - (_p4[1] + _p4[3])) * _K_MAX_TILT
                    _pitch_tgt = ((_p4[0] + _p4[3]) - (_p4[1] + _p4[2])) * _K_MAX_TILT
                    _ka = _kdt / (_K_TILT_TAU + _kdt)
                    _kroll  += _ka * (_roll_tgt  - _kroll)
                    _kpitch += _ka * (_pitch_tgt - _kpitch)

                # Thrust vector rotated to world-NED via yaw
                _kcy, _ksy = math.cos(_kyaw_rad), math.sin(_kyaw_rad)
                _kbfwd = -_kthrust * math.sin(_kpitch)
                _kbrgt =  _kthrust * math.sin(_kroll)
                _kan = _kbfwd * _kcy - _kbrgt * _ksy   # NED north accel
                _kae = _kbfwd * _ksy + _kbrgt * _kcy   # NED east  accel
                _kad = _K_GRAVITY - _kthrust * math.cos(_kroll) * math.cos(_kpitch)

                _kvn = max(-_K_MAX_VEL, min(_K_MAX_VEL, _kvn + _kan * _kdt))
                _kve = max(-_K_MAX_VEL, min(_K_MAX_VEL, _kve + _kae * _kdt))
                _kvd = max(-_K_MAX_VEL, min(_K_MAX_VEL, _kvd + _kad * _kdt))

                # Aerodynamic drag
                drag = 1.0 - _K_DRAG * _kdt
                _kvn *= drag; _kve *= drag; _kvd *= drag

                _ky += _kvn * _kdt   # ENU north = +Y
                _kx += _kve * _kdt   # ENU east  = +X
                _kz -= _kvd * _kdt   # NED down  → altitude increases when vd < 0

                if _kz <= HOME_ALT_MSL:   # ground constraint
                    _kz  = HOME_ALT_MSL
                    _kvd = min(0.0, _kvd)
                    _kvn = 0.0   # ground friction — no horizontal sliding
                    _kve = 0.0
                    _kpitch_rate = 0.0; _kroll_rate = 0.0

            # ── Trace CSV at 5 Hz ─────────────────────────────────────────────
            if t_now - _trace_last_t >= 0.2:
                _trace_last_t = t_now
                _trace_csv.writerow([
                    f"{t_now - _dbg_start:.2f}",
                    f"{_kx:.3f}", f"{_ky:.3f}",
                    f"{_kz - HOME_ALT_MSL:.3f}",
                    f"{_kvn:.3f}", f"{_kve:.3f}",
                ])

            # ── Publish /drone/state ───────────────────────────────────────────
            qx, qy, qz, qw = _euler_to_quat(_kroll, _kpitch, _kyaw_rad)
            msg = PoseStamped()
            msg.header.stamp    = node.get_clock().now().to_msg()
            msg.header.frame_id = "local_enu"
            msg.pose.position.x    = _kx
            msg.pose.position.y    = _ky
            msg.pose.position.z    = _kz
            msg.pose.orientation.x = qx
            msg.pose.orientation.y = qy
            msg.pose.orientation.z = qz
            msg.pose.orientation.w = qw
            state_pub.publish(msg)

            time.sleep(0.01)   # 100 Hz

    except KeyboardInterrupt:
        agl = _kz - HOME_ALT_MSL
        print(f"\n[drone_sim] Stopped. Final AGL = {agl:.1f} m")
    finally:
        _trace_f.close()
        bridge.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
