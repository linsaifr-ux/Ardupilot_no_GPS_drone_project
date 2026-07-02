#!/usr/bin/env python3
"""
CSI camera driver: captures frames from the IMX219 (CSI/MIPI) via the Jetson
ISP (nvarguscamerasrc) and publishes them as sensor_msgs/Image (rgb8) on
/drone/camera/image_raw.

nvarguscamerasrc replaces v4l2_camera_node here because the IMX219 exposes
raw Bayer (RG10) on /dev/video0 — v4l2_camera_node cannot debayer it. Going
through nvarguscamerasrc/ISP gives debayered, auto-exposed/white-balanced RGB.

Capture defaults to 1640x1232 — the sensor's full-FOV 2x2-binned mode
(matches the 62.2°x48.8° HFOV/VFOV spec used for GSD/AnyLoc math elsewhere;
narrower 16:9 modes like 1280x720 crop the vertical FOV).

Publishes with sensor-data QoS (BEST_EFFORT) — correct semantics for a live
feed (no point retransmitting a stale frame), and all subscribers must match
it or delivery silently fails. This alone is NOT what fixes throughput,
though — see read_and_publish() for the real bottleneck.

Run: python3 control/csi_camera_node.py [--sensor-id 0] [--width 1640] [--height 1232] [--fps 30]
"""
import argparse
import array
import os
import sys

_ROS2_SITE = "/opt/ros/humble/lib/python3.10/site-packages"
if os.path.isdir(_ROS2_SITE) and _ROS2_SITE not in sys.path:
    sys.path.insert(0, _ROS2_SITE)

import cv2
import rclpy
import rclpy.node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def build_pipeline(sensor_id: int, width: int, height: int, fps: int) -> str:
    return (
        f'nvarguscamerasrc sensor-id={sensor_id} ! '
        f'video/x-raw(memory:NVMM),width={width},height={height},'
        f'framerate={fps}/1,format=NV12 ! '
        f'nvvidconv ! video/x-raw,format=BGRx ! '
        f'videoconvert ! video/x-raw,format=BGR ! '
        f'appsink drop=true max-buffers=1 sync=false'
    )


class CsiCameraNode(rclpy.node.Node):
    def __init__(self, sensor_id: int, width: int, height: int, fps: int):
        super().__init__('csi_camera_node')
        self._pub = self.create_publisher(Image, '/drone/camera/image_raw', qos_profile_sensor_data)
        self._width, self._height = width, height

        pipeline = build_pipeline(sensor_id, width, height, fps)
        self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            raise RuntimeError(
                f'Cannot open CSI camera (sensor-id={sensor_id}) — check '
                f'nvargus-daemon is running and the ribbon cable is seated')

        self.get_logger().info(
            f'CSI camera ready: sensor-id={sensor_id} {width}x{height}@{fps}fps '
            f'→ /drone/camera/image_raw')

    def read_and_publish(self) -> bool:
        ok, bgr = self.cap.read()
        if not ok:
            self.get_logger().warn('Camera read failed', throttle_duration_sec=2.0)
            return False

        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'
        msg.height = self._height
        msg.width = self._width
        msg.encoding = 'rgb8'
        msg.is_bigendian = 0
        msg.step = self._width * 3
        # rosidl_generator_py's uint8[] setter validates every element in a
        # Python loop when given plain bytes — ~870ms for a 6 MB frame,
        # measured. array.array('B', ...) matches the field's expected type
        # exactly, so the setter skips validation entirely: ~4ms.
        msg.data = array.array('B', rgb.tobytes())
        self._pub.publish(msg)
        return True


def main():
    ap = argparse.ArgumentParser(description='IMX219 CSI camera → /drone/camera/image_raw')
    ap.add_argument('--sensor-id', type=int, default=0, help='Argus sensor index (default: 0)')
    ap.add_argument('--width',     type=int, default=1640)
    ap.add_argument('--height',    type=int, default=1232)
    ap.add_argument('--fps',       type=int, default=30)
    args = ap.parse_args()

    rclpy.init()
    node = CsiCameraNode(args.sensor_id, args.width, args.height, args.fps)
    try:
        while rclpy.ok():
            node.read_and_publish()
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
