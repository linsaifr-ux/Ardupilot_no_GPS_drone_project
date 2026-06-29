#!/usr/bin/env python3
"""
YOLOv8 vehicle detector as a ROS2 node with live postview.

Subscribes:
  /drone/camera/image_raw  (sensor_msgs/Image, rgb8, 2048×1536)
  /drone/pose              (geometry_msgs/PoseStamped, frame_id="wgs84")

Publishes:
  /yolo/detections         (vision_msgs/Detection2DArray)

Run:
  ./detection/run_ros2_detector.sh
"""

import os
import sys
import threading
import time

_ROS2_SITE = "/opt/ros/humble/lib/python3.10/site-packages"
if os.path.isdir(_ROS2_SITE) and _ROS2_SITE not in sys.path:
    sys.path.insert(0, _ROS2_SITE)

import rclpy
import rclpy.node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import numpy as np
import torch
from PIL import Image as PILImage

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from detection.detector import YOLODetector

MODEL_PT = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "Car_visdrone1280.pt")

MIN_AGL = 50.0   # m — skip inference below this altitude


def _pil_to_array(pil_img, size=(1024, 768)):
    img = pil_img.resize(size, PILImage.LANCZOS).convert('RGB')
    t   = torch.frombuffer(bytearray(img.tobytes()), dtype=torch.uint8) \
               .reshape(size[1], size[0], 3)
    return t.numpy()


class YOLONode(rclpy.node.Node):
    def __init__(self):
        super().__init__("yolo_detector")

        self._det = YOLODetector(MODEL_PT, conf=0.50)

        self._drone_lat  = 0.0
        self._drone_lon  = 0.0
        self._drone_agl  = 0.0
        self._agl_logged = False
        self._frame_times: list[float] = []

        # Latest results shared with postview (main thread)
        self.lock           = threading.Lock()
        self.latest_frame   = None   # PIL annotated image
        self.latest_result  = None   # dict: n, elapsed_ms, fps, lat, lon, detections

        self.create_subscription(Image,       "/drone/camera/image_raw", self._cb_image, 1)
        self.create_subscription(PoseStamped, "/drone/pose",             self._cb_pose,  10)
        self.create_subscription(Float64,     "/drone/agl",              self._cb_agl,   10)

        self.pub = self.create_publisher(Detection2DArray, "/yolo/detections", 1)

        print(f"[YOLO] Model: {os.path.basename(MODEL_PT)}")
        print("[YOLO] Waiting for /drone/camera/image_raw …")
        print("[YOLO] Close the window or press Ctrl-C to quit.")

    def _cb_pose(self, msg):
        self._drone_lat = msg.pose.position.x
        self._drone_lon = msg.pose.position.y

    def _cb_agl(self, msg):
        self._drone_agl = msg.data

    def _cb_image(self, msg):
        _t_recv = time.perf_counter()
        if self._drone_agl < MIN_AGL:
            return

        if not self._agl_logged:
            print(f"[YOLO] AGL {self._drone_agl:.0f} m ≥ {MIN_AGL:.0f} m — starting detection")
            self._agl_logged = True

        try:
            pil_img = PILImage.frombytes(
                "RGB", (msg.width, msg.height), bytes(msg.data))
        except Exception as e:
            self.get_logger().warn(f"Image decode: {e}")
            return

        _t_decode_ms = (time.perf_counter() - _t_recv) * 1000.0
        t0 = time.perf_counter()
        detections = self._det.detect(pil_img)
        elapsed_ms = (time.perf_counter() - t0) * 1000.0

        self._frame_times.append(t0)
        if len(self._frame_times) > 30:
            self._frame_times.pop(0)
        fps = ((len(self._frame_times) - 1) /
               (self._frame_times[-1] - self._frame_times[0])
               if len(self._frame_times) >= 2 else 0.0)

        # Publish ROS2 detections
        arr = Detection2DArray()
        arr.header = msg.header
        for d in detections:
            det = Detection2D()
            det.header = msg.header
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = d["label"]
            hyp.hypothesis.score    = float(d["conf"])
            det.results.append(hyp)
            det.bbox.center.position.x = (d["x1"] + d["x2"]) / 2.0
            det.bbox.center.position.y = (d["y1"] + d["y2"]) / 2.0
            det.bbox.size_x = float(d["x2"] - d["x1"])
            det.bbox.size_y = float(d["y2"] - d["y1"])
            arr.detections.append(det)
        self.pub.publish(arr)

        # Terminal output every frame
        n = len(detections)
        if detections:
            for d in detections:
                print(f"[YOLO] {d['label']:12s}  conf={d['conf']:.2f}  "
                      f"box=({d['x1']:.0f},{d['y1']:.0f},"
                      f"{d['x2']:.0f},{d['y2']:.0f})  {fps:.1f} fps")
        else:
            print(f"[YOLO] no vehicles  decode {_t_decode_ms:.0f} ms  "
                  f"infer {elapsed_ms:.0f} ms  {fps:.1f} fps  "
                  f"lat={self._drone_lat:.5f} lon={self._drone_lon:.5f}")

        # Annotated frame for postview — scale boxes to half-res display
        _dw, _dh = 1024, 768
        _sx, _sy = _dw / msg.width, _dh / msg.height
        _scaled = [{**d, 'x1': d['x1']*_sx, 'y1': d['y1']*_sy,
                         'x2': d['x2']*_sx, 'y2': d['y2']*_sy}
                   for d in detections]
        annotated = self._det.draw(pil_img.resize((_dw, _dh), PILImage.LANCZOS),
                                   _scaled)
        with self.lock:
            self.latest_frame  = annotated
            self.latest_result = dict(
                n=n, elapsed_ms=elapsed_ms, fps=fps,
                lat=self._drone_lat, lon=self._drone_lon,
                detections=detections,
            )


def run_postview(node: YOLONode):
    fig, ax = plt.subplots(1, 1, figsize=(8, 6.4), layout='constrained')
    fig.patch.set_facecolor('#1a1a1a')
    ax.axis('off')
    ax.set_facecolor('#1a1a1a')

    blank = np.zeros((768, 1024, 3), dtype=np.uint8)
    im = ax.imshow(blank)
    ax.set_title('YOLO Vehicle Detection — waiting for frames …',
                 color='white', fontsize=11, pad=4)
    plt.ion()
    plt.show()

    while plt.fignum_exists(fig.number):
        with node.lock:
            frame  = node.latest_frame
            result = node.latest_result

        if frame is not None and result is not None:
            r     = result
            color = '#50ff50' if r['n'] > 0 else 'white'
            ax.set_title(
                f"YOLO  {r['n']} vehicle{'s' if r['n'] != 1 else ''}  —  "
                f"{r['elapsed_ms']:.0f} ms  {r['fps']:.1f} fps  |  "
                f"{r['lat']:.5f} N  {r['lon']:.5f} E",
                color=color, fontsize=11, pad=4)
            im.set_data(_pil_to_array(frame))
            fig.canvas.draw_idle()

        plt.pause(0.05)

    print("[YOLO] Closed.")


def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true")
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = YOLONode()

    if args.headless:
        print("[YOLO] Running headless — no postview window")
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()
        return

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        run_postview(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
