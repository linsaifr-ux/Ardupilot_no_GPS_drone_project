#!/usr/bin/env python3
"""Replay a recorded flight onto the same live topics launch_shadow_mode.sh
uses, for end-to-end testing of vio_live_node + vpe_live_node +
fusion_live_node without a real aircraft -- Task 9 / the plan's verification
step 1.

Publishes, all from field_data/<survey>/:
  /drone/camera/image_raw             (sensor_msgs/Image, rgb8, native res)
  /mavros/imu/data_raw                (sensor_msgs/Imu)
  /mavros/global_position/global      (sensor_msgs/NavSatFix)
  /mavros/global_position/compass_hdg (std_msgs/Float64)
  /drone/agl                          (std_msgs/Float64)

Message header.stamp always carries the ORIGINAL recorded time, regardless of
--speed. This matters: vio_live_node keys all its OpenVINS math off message
stamps (not wall-clock), so it is speed-agnostic -- but fusion_live_node's
predict()/output() timing runs on real wall-clock ticks (correctly so, since
in real flight wall-clock IS the data rate). Running this replay at anything
other than --speed 1.0 desyncs those two, making MAX_FIX_AGE_S / the slew
budget behave unrepresentatively. --speed exists for quick smoke tests of
message flow only; trust --speed 1.0 runs for anything about filter behavior.

Usage:
  source /opt/ros/humble/setup.bash
  python3 vio_vpe/replay_flight.py --survey survey43 --start-off 210 --end-off 320
"""
import argparse
import csv
import os
import sys
import time

_ROS2_SITE = "/opt/ros/humble/lib/python3.10/site-packages"
if os.path.isdir(_ROS2_SITE) and _ROS2_SITE not in sys.path:
    sys.path.insert(0, _ROS2_SITE)

import cv2 as cv
import rclpy
import rclpy.node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu, NavSatFix
from std_msgs.msg import Float64

HERE = os.path.dirname(os.path.abspath(__file__))
FIELD_DATA = os.path.join(os.path.dirname(HERE), "field_data")


def _stamp(t):
    from rclpy.time import Time
    sec = int(t)
    nsec = int(round((t - sec) * 1e9))
    return Time(seconds=sec, nanoseconds=nsec).to_msg()


def load_events(survey_dir, start_off, end_off, image_hz=15.0):
    """Returns (events, cap) where events is a time-sorted list of
    (t, kind, payload) and cap is an opened cv2.VideoCapture already seeked
    to the first frame in the window.

    Image events are subsampled to image_hz (matching vio_live_node's own
    internal throttle -- it discards frames faster than track_frequency
    anyway, so publishing all native ~30fps here only adds replay-harness
    decode/serialize/publish load with no downstream benefit). Publish()
    still fast-forwards the decoder through the skipped frames one at a time
    (self._last_decoded_idx), it just never builds/publishes a message for
    them -- cheap relative to the full rgb8-convert-and-DDS-send path this
    was originally spending on every single native-res frame."""
    ft = [(int(r["frame_idx"]), float(r["unix_time"]))
          for r in csv.DictReader(open(os.path.join(survey_dir, "frame_times.csv")))]
    t_video0 = ft[0][1]
    t_start = t_video0 + start_off
    t_end = t_video0 + end_off if end_off > 0 else ft[-1][1] + 1.0

    events = []
    first_idx = None
    last_img_t = None
    min_gap = 1.0 / image_hz
    for idx, t in ft:
        if t_start <= t <= t_end:
            if first_idx is None:
                first_idx = idx
            if last_img_t is None or t - last_img_t >= min_gap:
                events.append((t, "img", idx))
                last_img_t = t

    with open(os.path.join(survey_dir, "imu.csv")) as f:
        r = csv.reader(f)
        next(r)
        for row in r:
            t = float(row[0])
            if t_start - 1.0 <= t <= t_end + 1.0:
                events.append((t, "imu",
                              tuple(float(x) for x in row[2:8])))  # wx,wy,wz,ax,ay,az

    with open(os.path.join(survey_dir, "telemetry.csv")) as f:
        for row in csv.DictReader(f):
            try:
                t = float(row["unix_time"])
            except ValueError:
                continue
            if t_start <= t <= t_end:
                events.append((t, "gps", (float(row["lat"]), float(row["lon"]))))
                events.append((t, "hdg", float(row["heading_deg"])))
                if row["alt_agl"]:
                    events.append((t, "agl", float(row["alt_agl"])))

    events.sort(key=lambda e: e[0])

    cap = cv.VideoCapture(os.path.join(survey_dir, "video.mkv"))
    if first_idx is not None:
        cap.set(cv.CAP_PROP_POS_FRAMES, first_idx)
    return events, cap, (first_idx or 0) - 1


class ReplayNode(rclpy.node.Node):
    def __init__(self, last_decoded_idx):
        super().__init__("replay_flight_node")
        self.pub_img = self.create_publisher(Image, "/drone/camera/image_raw",
                                             qos_profile_sensor_data)
        self.pub_imu = self.create_publisher(Imu, "/mavros/imu/data_raw",
                                             qos_profile_sensor_data)
        self.pub_gps = self.create_publisher(NavSatFix, "/mavros/global_position/global",
                                             qos_profile_sensor_data)
        self.pub_hdg = self.create_publisher(Float64, "/mavros/global_position/compass_hdg",
                                             qos_profile_sensor_data)
        self.pub_agl = self.create_publisher(Float64, "/drone/agl", 10)
        self._last_decoded_idx = last_decoded_idx

    def publish(self, kind, t, payload, cap):
        if kind == "img":
            # fast-forward through any frames skipped by the image_hz
            # subsample -- decode-only, no message built for these.
            idx = payload
            for _ in range(idx - self._last_decoded_idx - 1):
                if not cap.read()[0]:
                    return False
            ok, frame = cap.read()
            self._last_decoded_idx = idx
            if not ok:
                return False
            rgb = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
            m = Image()
            m.header.stamp = _stamp(t)
            m.height, m.width = rgb.shape[:2]
            m.encoding = "rgb8"
            m.step = m.width * 3
            m.data = rgb.tobytes()
            self.pub_img.publish(m)
        elif kind == "imu":
            wx, wy, wz, ax, ay, az = payload
            m = Imu()
            m.header.stamp = _stamp(t)
            m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = wx, wy, wz
            m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = ax, ay, az
            self.pub_imu.publish(m)
        elif kind == "gps":
            lat, lon = payload
            m = NavSatFix()
            m.header.stamp = _stamp(t)
            m.status.status = 0
            m.latitude, m.longitude = lat, lon
            self.pub_gps.publish(m)
        elif kind == "hdg":
            m = Float64()
            m.data = float(payload)
            self.pub_hdg.publish(m)
        elif kind == "agl":
            m = Float64()
            m.data = float(payload)
            self.pub_agl.publish(m)
        return True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--survey", required=True)
    ap.add_argument("--start-off", type=float, default=0.0)
    ap.add_argument("--end-off", type=float, default=-1.0)
    ap.add_argument("--speed", type=float, default=1.0,
                    help="1.0 = real-time (the only speed representative of "
                         "filter timing behavior -- see file header)")
    ap.add_argument("--image-hz", type=float, default=15.0,
                    help="subsample rate for published camera frames -- see "
                         "load_events()'s docstring on why this isn't 30")
    args, ros_args = ap.parse_known_args()

    survey_dir = os.path.join(FIELD_DATA, args.survey)
    events, cap, last_decoded_idx = load_events(survey_dir, args.start_off, args.end_off,
                                                args.image_hz)
    if args.speed != 1.0:
        print(f"[replay] WARNING: --speed {args.speed} != 1.0 -- message flow "
              f"only, NOT representative of fusion_live_node's real-time gating")
    print(f"[replay] {len(events)} events over "
          f"{events[-1][0]-events[0][0]:.1f}s (data time), speed={args.speed}, "
          f"image_hz={args.image_hz}")

    rclpy.init(args=ros_args)
    node = ReplayNode(last_decoded_idx)

    t0_data = events[0][0]
    t0_wall = time.time()
    n_img = n_imu = n_gps = 0
    for t, kind, payload in events:
        target_wall = t0_wall + (t - t0_data) / args.speed
        delay = target_wall - time.time()
        if delay > 0:
            time.sleep(delay)
        if not node.publish(kind, t, payload, cap):
            print(f"[replay] video ended early at t={t-t0_data:.1f}s")
            break
        # ReplayNode has no subscriptions/timers of its own -- Publisher.publish()
        # does not need spin_once() to actually send. Calling it after every one
        # of ~16k events (many at ~300 Hz for IMU) was the actual bottleneck: a
        # first pass at this harness fell ~4-12x behind real-time even after
        # subsampling images, entirely from spin_once()'s per-call overhead, not
        # message payload size. rclpy still needs occasional spinning to process
        # its own internal work (e.g. QoS/discovery), so this keeps a light touch
        # rather than dropping it completely.
        if kind == "img":
            n_img += 1
        elif kind == "imu":
            n_imu += 1
            if n_imu % 100 == 0:
                rclpy.spin_once(node, timeout_sec=0)
        elif kind == "gps":
            n_gps += 1
        rclpy.spin_once(node, timeout_sec=0)
        if (n_img + n_imu) % 2000 == 0 and (n_img or n_imu):
            print(f"[replay] t={t-t0_data:.1f}s img={n_img} imu={n_imu} gps={n_gps}")

    print(f"[replay] done: img={n_img} imu={n_imu} gps={n_gps}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
