#!/usr/bin/env python3
"""
YOLOv8 vehicle detector as a ROS2 node with live postview.

Subscribes:
  /drone/camera/image_raw  (sensor_msgs/Image, rgb8, 1280×960)
  /drone/pose              (geometry_msgs/PoseStamped, frame_id="wgs84")

Publishes:
  /yolo/detections         (vision_msgs/Detection2DArray)
    Contract relied on downstream (tools/ground_view_stream.py): header is
    copied verbatim from the source image, and a message goes out for every
    processed frame, even with zero detections.

Pipelined (2026-07-09): the image callback only decodes; a preprocess thread
letterboxes + uploads the tensor (CPU, ~15 ms) while the inference thread runs
the TensorRT engine on the previous frame (GPU, ~30 ms) — the stages overlap,
so throughput is bounded by the slower stage instead of their sum. Each stage
keeps only the newest frame (depth-1 slots), same freshest-frame semantics as
the old QoS depth-1 callback.

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
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
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


def _pil_to_array(pil_img, size=(1024, 768)):
    img = pil_img.resize(size, PILImage.LANCZOS).convert('RGB')
    t   = torch.frombuffer(bytearray(img.tobytes()), dtype=torch.uint8) \
               .reshape(size[1], size[0], 3)
    return t.numpy()


class YOLONode(rclpy.node.Node):
    def __init__(self, headless: bool = False):
        super().__init__("yolo_detector")

        self._headless = headless
        # (960, 1280) rect engine matches the 1280×960 camera's aspect ratio —
        # same scale as the old 1280×1280 square letterbox but no padding, so
        # preprocess + inference both do ~25% less work at identical accuracy.
        self._det = YOLODetector(MODEL_PT, conf=0.50, imgsz=(960, 1280))

        self._drone_lat  = 0.0
        self._drone_lon  = 0.0
        self._frame_times: list[float] = []

        # Latest results shared with postview (main thread)
        self.lock           = threading.Lock()
        self.latest_frame   = None   # PIL annotated image
        self.latest_result  = None   # dict: n, elapsed_ms, fps, lat, lon, detections

        # Depth-1 hand-off slots between the pipeline stages (newest wins)
        self._frame_cond = threading.Condition()
        self._frame_slot = None      # (header, rgb ndarray)
        self._prep_cond  = threading.Condition()
        self._prep_slot  = None      # (header, rgb, tensor, meta, pre_ms)

        self.create_subscription(Image,       "/drone/camera/image_raw", self._cb_image, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, "/drone/pose",             self._cb_pose,  10)

        self.pub = self.create_publisher(Detection2DArray, "/yolo/detections", 1)

        threading.Thread(target=self._preproc_loop, daemon=True,
                         name="yolo_preproc").start()
        threading.Thread(target=self._infer_loop, daemon=True,
                         name="yolo_infer").start()

        print(f"[YOLO] Model: {os.path.basename(MODEL_PT)}")
        print("[YOLO] Waiting for /drone/camera/image_raw …")
        print("[YOLO] Close the window or press Ctrl-C to quit.")

    def _cb_pose(self, msg):
        self._drone_lat = msg.pose.position.x
        self._drone_lon = msg.pose.position.y

    def _cb_image(self, msg):
        try:
            rgb = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, 3)
        except ValueError as e:
            self.get_logger().warn(f"Image decode: {e}")
            return
        with self._frame_cond:
            self._frame_slot = (msg.header, rgb)
            self._frame_cond.notify()

    # ── pipeline stage 1: CPU letterbox + tensor upload ───────────────────────

    def _preproc_loop(self):
        while True:
            with self._frame_cond:
                while self._frame_slot is None:
                    self._frame_cond.wait()
                header, rgb = self._frame_slot
                self._frame_slot = None
            try:
                t0 = time.perf_counter()
                tensor, meta = self._det.preprocess(rgb)
                pre_ms = (time.perf_counter() - t0) * 1000.0
            except Exception as e:
                self.get_logger().warn(f"preprocess: {e}")
                continue
            with self._prep_cond:
                self._prep_slot = (header, rgb, tensor, meta, pre_ms)
                self._prep_cond.notify()

    # ── pipeline stage 2: GPU inference + publish + postview ─────────────────

    def _infer_loop(self):
        while True:
            with self._prep_cond:
                while self._prep_slot is None:
                    self._prep_cond.wait()
                header, rgb, tensor, meta, pre_ms = self._prep_slot
                self._prep_slot = None
            try:
                t0 = time.perf_counter()
                detections = self._det.detect_prepared(tensor, meta)
                elapsed_ms = (time.perf_counter() - t0) * 1000.0
            except Exception as e:
                self.get_logger().warn(f"inference: {e}")
                continue

            self._frame_times.append(t0)
            if len(self._frame_times) > 30:
                self._frame_times.pop(0)
            fps = ((len(self._frame_times) - 1) /
                   (self._frame_times[-1] - self._frame_times[0])
                   if len(self._frame_times) >= 2 else 0.0)

            # Publish ROS2 detections — header must stay the source image's
            # (ground_view_stream stamp-matches on it), and one message goes
            # out per processed frame even with zero detections.
            arr = Detection2DArray()
            arr.header = header
            for d in detections:
                det = Detection2D()
                det.header = header
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
                print(f"[YOLO] no vehicles  pre {pre_ms:.0f} ms  "
                      f"infer {elapsed_ms:.0f} ms  {fps:.1f} fps  "
                      f"lat={self._drone_lat:.5f} lon={self._drone_lon:.5f}")

            if self._headless:
                continue

            # Annotated frame for postview — scale boxes to half-res display
            pil_img = PILImage.fromarray(rgb)
            _dw, _dh = 1024, 768
            _sx, _sy = _dw / pil_img.width, _dh / pil_img.height
            _scaled = [{**d, 'x1': d['x1']*_sx, 'y1': d['y1']*_sy,
                             'x2': d['x2']*_sx, 'y2': d['y2']*_sy}
                       for d in detections]
            annotated = self._det.draw(
                pil_img.resize((_dw, _dh), PILImage.LANCZOS), _scaled)
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
    node = YOLONode(headless=args.headless)

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
