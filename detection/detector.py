"""
YOLO vehicle detector — wraps ultralytics YOLOv8 and filters to vehicle classes.

Supports both COCO-trained models and VisDrone-trained models by reading the
model's own class names and mapping them to four canonical labels:
  car  motorcycle  bus  truck
"""
from __future__ import annotations

import shutil
import tempfile
from pathlib import Path

import cv2
import numpy as np
import torch
from PIL import Image, ImageDraw, ImageFont
from ultralytics import YOLO

# Map any known class name → canonical vehicle label.
# Covers COCO names, VisDrone names, and common fine-tuned variants.
_NAME_TO_LABEL: dict[str, str] = {
    'car':             'car',
    'van':             'car',         # VisDrone
    'motorcycle':      'motorcycle',  # COCO
    'motor':           'motorcycle',  # VisDrone
    'tricycle':        'motorcycle',  # VisDrone
    'awning-tricycle': 'motorcycle',  # VisDrone
    'bus':             'bus',
    'truck':           'truck',
}

_COLORS = {
    'car':        '#ff4444',
    'motorcycle': '#ff8800',
    'bus':        '#cc44ff',
    'truck':      '#ffee00',
}


class YOLODetector:
    """
    YOLOv8 vehicle detector.

    Usage:
        det = YOLODetector()
        detections = det.detect(pil_img)   # list of dicts
        annotated  = det.draw(pil_img, detections)
    """

    def __init__(self, model_name: str = 'yolov8n.pt', conf: float = 0.35,
                 imgsz: int | tuple[int, int] = 1280, use_tensorrt: bool = True):
        self.conf  = conf
        # (h, w) — a rectangular size matching the camera's aspect ratio avoids
        # burning compute on letterbox padding: 1280×960 into a 1280×1280
        # square wastes ~25% of the input on gray bars; (960, 1280) runs the
        # same pixels at the same scale with none.
        self.imgsz: tuple[int, int] = \
            (imgsz, imgsz) if isinstance(imgsz, int) else tuple(imgsz)

        # Eager PyTorch fp32 at imgsz=1280 is GPU-bound on Jetson (~8 fps for
        # yolov8s). A fp16 TensorRT engine fuses the conv/NMS graph and uses
        # the tensor cores properly, ~3x faster for the same weights. Export
        # once per (weights, imgsz) and cache the .engine next to the .pt.
        load_path = model_name
        if use_tensorrt and model_name.endswith('.pt') and torch.cuda.is_available():
            h, w = self.imgsz
            if h == w:   # legacy name — pre-existing square engines keep working
                engine_path = Path(model_name).with_suffix('.engine')
            else:
                engine_path = Path(model_name).with_name(
                    f"{Path(model_name).stem}_{h}x{w}.engine")
            if not engine_path.exists():
                print(f"[YOLO] No TensorRT engine at {engine_path} — exporting "
                      f"from {model_name} (imgsz={self.imgsz}, fp16). This "
                      f"takes a few minutes on first run …")
                # Export in a tempdir: ultralytics always writes <stem>.engine
                # next to the .pt, which would silently clobber an existing
                # engine of a different imgsz.
                with tempfile.TemporaryDirectory() as td:
                    tmp_pt = Path(td) / Path(model_name).name
                    shutil.copy2(model_name, tmp_pt)
                    YOLO(str(tmp_pt)).export(format='engine', imgsz=self.imgsz,
                                             half=True, device=0)
                    shutil.move(str(tmp_pt.with_suffix('.engine')), engine_path)
            load_path = str(engine_path)

        print(f"[YOLO] Loading {load_path} …")
        self.model = YOLO(load_path)

        # Build {class_id: canonical_label} from the model's own class names
        self._filter: dict[int, str] = {
            cid: _NAME_TO_LABEL[name]
            for cid, name in self.model.names.items()
            if name in _NAME_TO_LABEL
        }
        print(f"[YOLO] Model ready  classes={list(self._filter.values())}  "
              f"conf_threshold={conf}")

    def detect(self, pil_img: Image.Image, imgsz: int | None = None) -> list[dict]:
        """
        Run inference on a PIL image.

        Args:
            pil_img  input image (any resolution; YOLO letterboxes internally)
            imgsz    YOLO inference size; defaults to the size this detector
                     was constructed/exported with. If a TensorRT engine is
                     loaded, it is only valid at its export imgsz — pass a
                     different value only when using a .pt model.

        Returns list of dicts:
            label  str    — 'car', 'motorcycle', 'bus', or 'truck'
            conf   float  — confidence score
            x1 y1 x2 y2  float — bounding box pixels (xyxy, top-left origin)
        """
        results = self.model(pil_img, conf=self.conf, verbose=False,
                             imgsz=imgsz or self.imgsz)[0]
        out = []
        for box in results.boxes:
            cls_id = int(box.cls[0])
            if cls_id not in self._filter:
                continue
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            out.append({
                'label': self._filter[cls_id],
                'conf':  float(box.conf[0]),
                'x1': x1, 'y1': y1, 'x2': x2, 'y2': y2,
            })
        return out

    # ── Split-stage API (used by ros2_node.py to overlap CPU and GPU) ─────────
    #
    # detect() runs letterbox+normalize (CPU, ~20 ms) and inference (GPU,
    # ~30-40 ms) back-to-back on the caller's thread. preprocess() /
    # detect_prepared() expose the two halves so a producer thread can prepare
    # frame N+1 while the GPU runs frame N. Results are identical to detect().

    def preprocess(self, rgb: np.ndarray) -> tuple[torch.Tensor, dict]:
        """CPU stage: RGB HWC uint8 (any size) → normalized CHW fp16 CUDA
        tensor at self.imgsz, plus the letterbox meta needed to map boxes back.
        """
        h0, w0 = rgb.shape[:2]
        th, tw = self.imgsz
        gain = min(th / h0, tw / w0)
        nw, nh = round(w0 * gain), round(h0 * gain)
        if (nw, nh) != (w0, h0):
            rgb = cv2.resize(rgb, (nw, nh), interpolation=cv2.INTER_LINEAR)
        pad_x, pad_y = (tw - nw) // 2, (th - nh) // 2
        canvas = np.full((th, tw, 3), 114, dtype=np.uint8)
        canvas[pad_y:pad_y + nh, pad_x:pad_x + nw] = rgb
        t = torch.from_numpy(
            np.ascontiguousarray(canvas.transpose(2, 0, 1))).unsqueeze(0)
        t = t.to('cuda', non_blocking=True).half().div_(255.0)
        return t, dict(gain=gain, pad_x=pad_x, pad_y=pad_y, w0=w0, h0=h0)

    def detect_prepared(self, tensor: torch.Tensor, meta: dict) -> list[dict]:
        """GPU stage: run the model on a preprocess()ed tensor. Returns the
        same dict format as detect(), boxes in original-image pixels."""
        results = self.model(tensor, conf=self.conf, verbose=False)[0]
        gain, px, py = meta['gain'], meta['pad_x'], meta['pad_y']
        w0, h0 = meta['w0'], meta['h0']
        out = []
        for box in results.boxes:
            cls_id = int(box.cls[0])
            if cls_id not in self._filter:
                continue
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            out.append({
                'label': self._filter[cls_id],
                'conf':  float(box.conf[0]),
                'x1': min(max((x1 - px) / gain, 0), w0),
                'y1': min(max((y1 - py) / gain, 0), h0),
                'x2': min(max((x2 - px) / gain, 0), w0),
                'y2': min(max((y2 - py) / gain, 0), h0),
            })
        return out

    def draw(self, pil_img: Image.Image,
             detections: list[dict]) -> Image.Image:
        """Draw bounding boxes + labels on a PIL image. Returns new PIL RGB image."""
        img  = pil_img.copy().convert('RGB')
        draw = ImageDraw.Draw(img)
        try:
            font = ImageFont.truetype(
                '/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf', 13)
        except Exception:
            font = ImageFont.load_default()

        for det in detections:
            color = _COLORS.get(det['label'], '#ffffff')
            x1, y1, x2, y2 = det['x1'], det['y1'], det['x2'], det['y2']

            draw.rectangle([x1, y1, x2, y2], outline=color, width=2)

            label = f"{det['label']} {det['conf']:.2f}"
            lw    = draw.textlength(label, font=font)
            ty    = max(0, y1 - 16)
            draw.rectangle([x1, ty, x1 + lw + 4, ty + 16], fill=color)
            draw.text((x1 + 2, ty + 1), label, fill='black', font=font)

        return img
