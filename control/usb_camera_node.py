#!/usr/bin/env python3
"""
USB3 camera driver: captures frames from the AP-IMX900-Mini-USB3-I5 via
direct V4L2/OpenCV and publishes them as sensor_msgs/Image (rgb8) on
/drone/camera/image_raw.

A custom node replaces the stock v4l2_camera_node here because this camera
only exposes raw YUYV at 640x480/360 — at 1280x960 and above it only offers
MJPG/H264, and the ROS2 v4l2_camera package (0.6.2, confirmed via binary
inspection) has no MJPEG decode compiled in, so it silently negotiates down
to 640x480 instead of erroring. OpenCV's V4L2 backend decodes MJPG
transparently, so requesting MJPG directly gets the real 1280x960.

Capture defaults to 1280x960 (matches the 59.9°x46.7° HFOV/VFOV spec used
for GSD/AnyLoc math elsewhere — computed from sensor geometry, no lens
datasheet available for the 4mm CS-mount lens).

Frames are rotated 180° to correct the camera's physical mount orientation,
matching the convention already used in tools/record_field.py for this same
camera body.

Publishes with sensor-data QoS (BEST_EFFORT) — correct semantics for a live
feed (no point retransmitting a stale frame), and all subscribers must match
it or delivery silently fails.

Run: python3 control/usb_camera_node.py [--device /dev/video0] [--width 1280] [--height 960] [--fps 30]
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


class UsbCameraNode(rclpy.node.Node):
    def __init__(self, device: str, width: int, height: int, fps: int):
        super().__init__('usb_camera_node')
        self._pub = self.create_publisher(Image, '/drone/camera/image_raw', qos_profile_sensor_data)
        self._width, self._height = width, height

        self.cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        if not self.cap.isOpened():
            raise RuntimeError(f'Cannot open camera at {device}')

        got_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        got_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        if (got_w, got_h) != (width, height):
            self.get_logger().warn(
                f'Requested {width}x{height} but device negotiated '
                f'{got_w}x{got_h} — GSD/FOV math elsewhere assumes '
                f'{width}x{height} and will be wrong')

        self.get_logger().info(
            f'USB camera ready: {device} {got_w}x{got_h}@{fps}fps (MJPG) '
            f'→ /drone/camera/image_raw')

    def read_and_publish(self) -> bool:
        ok, bgr = self.cap.read()
        if not ok:
            self.get_logger().warn('Camera read failed', throttle_duration_sec=2.0)
            return False

        bgr = cv2.rotate(bgr, cv2.ROTATE_180)
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
        # Python loop when given plain bytes — array.array('B', ...) matches
        # the field's expected type exactly, so it skips validation. See
        # csi_camera_node.py for the measured cost of not doing this.
        msg.data = array.array('B', rgb.tobytes())
        self._pub.publish(msg)
        return True


def main():
    ap = argparse.ArgumentParser(description='AP-IMX900 USB3 camera → /drone/camera/image_raw')
    ap.add_argument('--device', default='/dev/video0', help='V4L2 device path (default: /dev/video0)')
    ap.add_argument('--width',  type=int, default=1280)
    ap.add_argument('--height', type=int, default=960)
    ap.add_argument('--fps',    type=int, default=30)
    args = ap.parse_args()

    rclpy.init()
    node = UsbCameraNode(args.device, args.width, args.height, args.fps)
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
