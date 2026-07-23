#!/usr/bin/env python3
"""
High-rate IMU/attitude logger sidecar for tools/record_field.py.

Runs as its OWN process: inside the recorder, the 30 fps camera/encode loop's
GIL contention caps a Python ROS executor at ~100 Hz (bench-measured 62-105 Hz
with 200 Hz requested); a dedicated process sustains the full 200 Hz.

Writes into --out DIR:
    imu.csv       stamp_ros, recv_unix, wx, wy, wz, ax, ay, az
                  (/mavros/imu/data_raw — FC gyro rad/s + accel m/s²)
    attitude.csv  stamp_ros, recv_unix, qw, qx, qy, qz  (/mavros/imu/data)
    imu_rates.json {"imu_hz": .., "att_hz": ..} refreshed every 2 s — the
                  recorder reads this for its status line and meta.json.

Requests RAW_IMU @--imu-hz (default 200) + ATTITUDE_QUATERNION @50 Hz via
SET_MESSAGE_INTERVAL, re-sending every 2 s until the measured rate holds
≥40 % of the request (the FC forgets the setting on reboot).

--imu-hz 333 streams near the FC loop rate: the RAW_IMU values are the
filtered loop-rate samples (post INS_ notch/LPF), but the default 400→200 Hz
stream decimation has no anti-alias filter — 100-200 Hz vibration folds
into the band VIO integrates (measured on survey17, see
instructions/vpe_jump_runaway_diagnosis.md §14-3). 333 is the grantable
max (bench-measured 2026-07-23: 400 is DENIED by the firmware's
cap_message_interval — needs interval_ms*800 >= loop_period_us, so 3 ms
is the floor at SCHED_LOOP_RATE=400; a 333 request actually delivers
~346 Hz = ~87% of all loop-rate samples, which removes the coherent
decimation fold).

Standalone use:  python3 tools/imu_logger.py --out somedir/ [--imu-hz 333]
Exits cleanly (flushing buffers) on SIGINT/SIGTERM.
"""

import argparse
import json
import os
import signal
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu
from mavros_msgs.srv import CommandLong

MAV_CMD_SET_MESSAGE_INTERVAL = 511
RAW_IMU_MSG_ID               = 27      # → /mavros/imu/data_raw
ATT_QUAT_MSG_ID              = 31      # → /mavros/imu/data
IMU_REQUEST_HZ               = 200     # default; 333 = grantable max
                                       # (~346 Hz actual, no coherent
                                       # stream-decimation aliasing;
                                       # 400 is DENIED by the FC)
ATT_REQUEST_HZ               = 50
IMU_OK_FRACTION              = 0.4     # stop re-requesting at this fraction
IMU_OK_HZ                    = int(IMU_REQUEST_HZ * IMU_OK_FRACTION)


class ImuLogger(Node):
    def __init__(self, out_dir, imu_hz=IMU_REQUEST_HZ):
        super().__init__('imu_logger')
        self._imu_hz = imu_hz
        self._imu_ok_hz = imu_hz * IMU_OK_FRACTION
        self._imu_file = open(os.path.join(out_dir, 'imu.csv'), 'w')
        self._imu_file.write('stamp_ros,recv_unix,wx,wy,wz,ax,ay,az\n')
        self._att_file = open(os.path.join(out_dir, 'attitude.csv'), 'w')
        self._att_file.write('stamp_ros,recv_unix,qw,qx,qy,qz\n')
        self._rates_path = os.path.join(out_dir, 'imu_rates.json')

        self._imu_buf, self._att_buf = [], []
        self._imu_count = self._att_count = 0
        self._imu_rate  = self._att_rate  = 0.0
        self._rate_t    = time.time()

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=64)
        self.create_subscription(Imu, '/mavros/imu/data_raw',
                                 self._cb_imu, qos)
        self.create_subscription(Imu, '/mavros/imu/data',
                                 self._cb_att, qos)
        self._cmd_cli = self.create_client(CommandLong, '/mavros/cmd/command')

        self.create_timer(1.0,  self._flush)
        self.create_timer(2.0,  self._update_rates)
        # 2 s retry: a fresh mavros resets the FC's message intervals to
        # defaults (~3 Hz RAW_IMU) and its command service comes up late —
        # with a 10 s timer the first ~10 s of every session recorded at
        # default rates (bench-measured). The ≥IMU_OK_HZ guard in
        # _request_rate stops the re-sends once the rate locks.
        self.create_timer(2.0,  self._request_rate)
        self._request_rate()

    def _cb_imu(self, msg):
        st = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        w, a = msg.angular_velocity, msg.linear_acceleration
        self._imu_buf.append(
            f'{st:.6f},{time.time():.6f},'
            f'{w.x:.6f},{w.y:.6f},{w.z:.6f},{a.x:.5f},{a.y:.5f},{a.z:.5f}\n')
        self._imu_count += 1

    def _cb_att(self, msg):
        st = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        q = msg.orientation
        self._att_buf.append(
            f'{st:.6f},{time.time():.6f},'
            f'{q.w:.6f},{q.x:.6f},{q.y:.6f},{q.z:.6f}\n')
        self._att_count += 1

    def _flush(self):
        if self._imu_buf:
            self._imu_file.writelines(self._imu_buf)
            self._imu_buf = []
            self._imu_file.flush()
        if self._att_buf:
            self._att_file.writelines(self._att_buf)
            self._att_buf = []
            self._att_file.flush()

    def _update_rates(self):
        now = time.time()
        dt = now - self._rate_t
        if dt > 0:
            self._imu_rate = self._imu_count / dt
            self._att_rate = self._att_count / dt
        self._imu_count = self._att_count = 0
        self._rate_t = now
        tmp = self._rates_path + '.tmp'
        with open(tmp, 'w') as f:
            json.dump({'imu_hz': round(self._imu_rate, 1),
                       'att_hz': round(self._att_rate, 1)}, f)
        os.replace(tmp, self._rates_path)

    def _send_interval(self, msg_id, hz, then=None):
        req = CommandLong.Request()
        req.command = MAV_CMD_SET_MESSAGE_INTERVAL
        req.param1  = float(msg_id)
        req.param2  = float(1e6 / hz)     # interval µs
        fut = self._cmd_cli.call_async(req)
        if then is not None:
            fut.add_done_callback(lambda _f: then())

    def _request_rate(self):
        # Keep requesting until BOTH rates hold. Two quirks (bench-measured):
        # ArduPilot serves only ~50 Hz after the FIRST request while the
        # REQUEST_DATA_STREAM regime is active — a repeat unlocks the full
        # rate; and two concurrent COMMAND_LONGs race in mavros (RAW_IMU
        # ended up at the attitude request's 50 Hz), so the second command
        # is chained on the first's completion, never sent in parallel.
        if self._imu_rate >= self._imu_ok_hz and self._att_rate >= ATT_REQUEST_HZ / 2:
            return
        if not self._cmd_cli.service_is_ready():
            return
        self._send_interval(
            RAW_IMU_MSG_ID, self._imu_hz,
            then=lambda: self._send_interval(ATT_QUAT_MSG_ID, ATT_REQUEST_HZ))

    def close(self):
        self._flush()
        self._imu_file.close()
        self._att_file.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--imu-hz', type=int, default=IMU_REQUEST_HZ)
    args = ap.parse_args()

    rclpy.init()
    node = ImuLogger(args.out, imu_hz=args.imu_hz)
    stop = []
    signal.signal(signal.SIGINT,  lambda *_: stop.append(1))
    signal.signal(signal.SIGTERM, lambda *_: stop.append(1))
    try:
        while not stop:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
