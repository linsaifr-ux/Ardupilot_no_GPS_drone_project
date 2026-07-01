"""
YOLO vehicle detector — wraps ultralytics YOLOv8 and filters to vehicle classes.

Supports both COCO-trained models and VisDrone-trained models by reading the
model's own class names and mapping them to four canonical labels:
  car  motorcycle  bus  truck
"""
from __future__ import annotations

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

    def __init__(self, model_name: str = 'yolov8n.pt', conf: float = 0.35):
        print(f"[YOLO] Loading {model_name} …")
        self.model = YOLO(model_name)
        self.conf  = conf

        # Build {class_id: canonical_label} from the model's own class names
        self._filter: dict[int, str] = {
            cid: _NAME_TO_LABEL[name]
            for cid, name in self.model.names.items()
            if name in _NAME_TO_LABEL
        }
        print(f"[YOLO] Model ready  classes={list(self._filter.values())}  "
              f"conf_threshold={conf}")

    def detect(self, pil_img: Image.Image, imgsz: int = 1280) -> list[dict]:
        """
        Run inference on a PIL image.

        Args:
            pil_img  input image (any resolution; YOLO letterboxes internally)
            imgsz    YOLO inference size (default 1280 for IMX219 1640×1232 input;
                     use 640 for faster inference at lower accuracy)

        Returns list of dicts:
            label  str    — 'car', 'motorcycle', 'bus', or 'truck'
            conf   float  — confidence score
            x1 y1 x2 y2  float — bounding box pixels (xyxy, top-left origin)
        """
        results = self.model(pil_img, conf=self.conf, verbose=False,
                             imgsz=imgsz)[0]
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
