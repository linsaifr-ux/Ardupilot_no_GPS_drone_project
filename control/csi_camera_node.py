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

Run: python3 control/csi_camera_node.py [--sensor-id 0] [--width 1640] [--height 1232] [--fps 30]
"""
import argparse
import os
import sys

_ROS2_SITE = "/opt/ros/humble/lib/python3.10/site-packages"
if os.path.isdir(_ROS2_SITE) and _ROS2_SITE not in sys.path:
    sys.path.insert(0, _ROS2_SITE)

import cv2
import rclpy
import rclpy.node
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
        self._pub = self.create_publisher(Image, '/drone/camera/image_raw', 1)
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
        msg.data = rgb.tobytes()
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
