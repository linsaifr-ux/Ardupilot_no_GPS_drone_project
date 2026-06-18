#!/usr/bin/env python3
"""
YOLO vehicle detection postview.
Watches simulator/drone_frames/latest.jpg and runs YOLOv8 vehicle detection
on each new frame, showing an annotated live window.

Run in a separate terminal while Isaac Sim is running:
    DISPLAY=:2 conda run -n isaac_sim_test python detection/run_detector.py

Press Ctrl-C or close the window to quit.
"""

import json, os, sys, time
import numpy as np
import torch
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from PIL import Image

HERE      = os.path.dirname(os.path.abspath(__file__))
ROOT      = os.path.abspath(os.path.join(HERE, '..'))
FRAME_JPG = os.path.join(ROOT, 'simulator', 'drone_frames', 'latest.jpg')
META_JSON = os.path.join(ROOT, 'simulator', 'drone_frames', 'latest_meta.json')
MODEL_PT  = os.path.join(ROOT, 'yolov8l_visdrone.pt')

sys.path.insert(0, HERE)
from detector import YOLODetector


def pil_to_rgb_array(pil_img, size=(640, 480)):
    """PIL → (H, W, 3) uint8 via torch frombuffer (avoids np.array issue)."""
    img = pil_img.resize(size, Image.LANCZOS).convert('RGB')
    t   = torch.frombuffer(bytearray(img.tobytes()), dtype=torch.uint8) \
               .reshape(size[1], size[0], 3)
    return t.numpy()


def main():
    det = YOLODetector(model_name=MODEL_PT, conf=0.30)

    fig, ax = plt.subplots(1, 1, figsize=(8, 6.4), layout='constrained')
    fig.patch.set_facecolor('#1a1a1a')
    ax.axis('off')
    ax.set_facecolor('#1a1a1a')

    blank = np.zeros((480, 640, 3), dtype=np.uint8)
    im = ax.imshow(blank)
    ax.set_title('YOLO Vehicle Detection', color='white', fontsize=11, pad=4)
    plt.ion()
    plt.show()

    last_mtime = 0.0
    frame_times: list[float] = []  # timestamps of recent processed frames
    print(f"[YOLO] Watching {FRAME_JPG}")
    print("[YOLO] Close the window or press Ctrl-C to quit.")

    while plt.fignum_exists(fig.number):
        try:
            mtime = os.path.getmtime(FRAME_JPG)
        except FileNotFoundError:
            plt.pause(0.5)
            continue

        if mtime != last_mtime:
            try:
                frame = Image.open(FRAME_JPG).convert('RGB')
                frame.load()
                with open(META_JSON) as fh:
                    meta = json.load(fh)
            except Exception as exc:
                print(f"[YOLO] frame read error ({exc}) — retrying")
                plt.pause(0.1)
                continue

            last_mtime = mtime

            t0 = time.perf_counter()
            detections = det.detect(frame)
            elapsed_ms = (time.perf_counter() - t0) * 1000.0

            # rolling FPS over last 30 frames
            frame_times.append(t0)
            if len(frame_times) > 30:
                frame_times.pop(0)
            if len(frame_times) >= 2:
                fps = (len(frame_times) - 1) / (frame_times[-1] - frame_times[0])
            else:
                fps = 0.0

            annotated = det.draw(frame.resize((640, 480), Image.LANCZOS), detections)
            im.set_data(pil_to_rgb_array(annotated))

            n   = len(detections)
            lat = meta.get('lat', 0.0)
            lon = meta.get('lon', 0.0)
            ax.set_title(
                f'YOLO  {n} vehicle{"s" if n != 1 else ""}  —  '
                f'{elapsed_ms:.0f} ms  {fps:.1f} fps  |  {lat:.5f} N  {lon:.5f} E',
                color='#50ff50' if n > 0 else 'white', fontsize=11, pad=4)

            if detections:
                for d in detections:
                    print(f"[YOLO] {d['label']:12s}  conf={d['conf']:.2f}  "
                          f"box=({d['x1']:.0f},{d['y1']:.0f},"
                          f"{d['x2']:.0f},{d['y2']:.0f})  {fps:.1f} fps")
            else:
                print(f"[YOLO] no vehicles  {elapsed_ms:.0f} ms  {fps:.1f} fps")

            fig.canvas.draw()
            fig.canvas.flush_events()

        plt.pause(0.05)

    print("[YOLO] Closed.")


if __name__ == '__main__':
    main()
