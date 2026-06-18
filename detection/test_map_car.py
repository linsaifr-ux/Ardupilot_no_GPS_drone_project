"""
mAP evaluation for car_s_1280 (YOLOv8s) vs car_11s_1280 (YOLO11s) on VisDrone val.

Both are single-class car-only models trained on the same dataset at imgsz=1280.

Run:
    python detection/test_map_car.py
"""
import sys
from pathlib import Path
_ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(_ROOT))

from ultralytics import YOLO

VD_CAR_DATA  = str(_ROOT / "detection/runs/wenting_visdrone_11s/data/visdrone_car/data.yaml")

MODELS = {
    # (weights_path, data_yaml, imgsz)
    "car_s_1280  (YOLOv8s)": (
        _ROOT / "detection/runs/car_s_1280/weights/best.pt",   VD_CAR_DATA, 1280),
    "car_11s_1280 (YOLO11s)": (
        _ROOT / "detection/runs/car_11s_1280/weights/best.pt", VD_CAR_DATA, 1280),
}


def evaluate(name: str, model_path: Path, data: str, imgsz: int) -> dict | None:
    if not model_path.exists():
        print(f"[SKIP] {name} — weights not found: {model_path}")
        return None

    print(f"\n{'='*60}")
    print(f"  Evaluating: {name}")
    print(f"  Weights   : {model_path}")
    print(f"  imgsz     : {imgsz}")
    print(f"{'='*60}")

    model = YOLO(str(model_path))
    results = model.val(data=data, imgsz=imgsz, verbose=False, plots=False)

    box = results.box
    spd = results.speed  # ms/image: preprocess, inference, postprocess
    inference_ms = spd.get("inference", float("nan"))
    total_ms     = sum(v for v in spd.values() if isinstance(v, float))
    return {
        "model"        : name,
        "mAP50"        : float(box.map50),
        "mAP50-95"     : float(box.map),
        "inference_ms" : inference_ms,
        "total_ms"     : total_ms,
        "fps"          : 1000.0 / total_ms if total_ms > 0 else float("nan"),
    }


def print_summary(rows: list[dict]):
    metrics = [
        ("mAP50",              "mAP50"),
        ("mAP50-95",           "mAP50-95"),
        ("inference (ms/img)", "inference_ms"),
        ("total    (ms/img)",  "total_ms"),
        ("FPS",                "fps"),
    ]

    col_w = 28
    name_w = 24
    header = f"{'Metric':<{col_w}}" + "".join(f"{r['model'][:name_w]:>{name_w}}" for r in rows)
    print(f"\n{'─'*len(header)}")
    print(header)
    print(f"{'─'*len(header)}")
    for label, key in metrics:
        line = f"{label:<{col_w}}"
        for r in rows:
            val = r.get(key, float("nan"))
            line += f"{val:>{name_w}.4f}"
        print(line)
    print(f"{'─'*len(header)}\n")


if __name__ == "__main__":
    rows = []
    for name, (path, data, imgsz) in MODELS.items():
        result = evaluate(name, path, data, imgsz)
        if result:
            rows.append(result)

    if rows:
        print_summary(rows)
    else:
        print("No models evaluated.")
        sys.exit(1)
