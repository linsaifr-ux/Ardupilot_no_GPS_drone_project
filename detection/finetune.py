"""
Fine-tune YOLOv8 for top-down vehicle detection.

Modes
-----
topdown         Start from yolov8n.pt + synthetic dataset (requires
                python detection/prepare_dataset.py first).

visdrone1280    Continue fine-tuning yolov8l_visdrone.pt on VisDrone at
                imgsz=1280 so inference resolution matches training.

visdrone_s_1280 Train yolov8s.pt from scratch on VisDrone (10 classes) at
                imgsz=1280.

car_s_1280      Train yolov8s.pt on car-only VisDrone dataset (car+van → class 0)
                at imgsz=1280. Single-class, faster and more accurate for car
                detection than the 10-class models.

car_11s_1280    Train yolo11s.pt on the same car-only dataset at imgsz=1280.
                YOLO11s backbone, otherwise identical setup to car_s_1280.

Run:
    python detection/finetune.py                          # topdown (default)
    python detection/finetune.py --mode visdrone1280      # L model at 1280
    python detection/finetune.py --mode visdrone_s_1280   # S model, 10-class, 1280
    python detection/finetune.py --mode car_s_1280        # YOLOv8s, car-only, 1280
    python detection/finetune.py --mode car_11s_1280      # YOLO11s, car-only, 1280
    python detection/finetune.py --mode <any> --resume

Best weights saved to:
    detection/runs/<name>/weights/best.pt
"""
import argparse
from pathlib import Path
from ultralytics import YOLO

_HERE = Path(__file__).parent
RUN_DIR = _HERE / "runs"

_VD_DATA = Path("/home/frank/文件/Project/Object_Detection_sim_in_Gazebo/datasets/visdrone.yaml")

# ── Mode: topdown ──────────────────────────────────────────────────────────────
_TOPDOWN_DATA = _HERE / "dataset" / "data.yaml"
_TOPDOWN_BASE = _HERE.parent / "yolov8n.pt"
_TOPDOWN_NAME = "topdown_v1"

# ── Mode: visdrone1280 (YOLOv8l) ──────────────────────────────────────────────
_VD_L_BASE = _HERE.parent / "yolov8l_visdrone.pt"
_VD_L_NAME = "visdrone_1280"
_VD_L_LAST = RUN_DIR / _VD_L_NAME / "weights" / "last.pt"

# ── Mode: visdrone_s_1280 (YOLOv8s, 10-class) ────────────────────────────────
_VD_S_BASE = "yolov8s.pt"   # auto-downloaded by Ultralytics on first run (~22 MB)
_VD_S_NAME = "visdrone_s_1280"
_VD_S_LAST = RUN_DIR / _VD_S_NAME / "weights" / "last.pt"

# ── Mode: car_s_1280 (YOLOv8s, car-only) ─────────────────────────────────────
_CAR_S_DATA = RUN_DIR / "wenting_visdrone_11s" / "data" / "visdrone_car" / "data.yaml"
_CAR_S_NAME = "car_s_1280"
_CAR_S_LAST = RUN_DIR / _CAR_S_NAME / "weights" / "last.pt"

# ── Mode: car_11s_1280 (YOLO11s, car-only) ────────────────────────────────────
_CAR_11S_BASE = "yolo11s.pt"   # auto-downloaded by Ultralytics on first run
_CAR_11S_NAME = "car_11s_1280"
_CAR_11S_LAST = RUN_DIR / _CAR_11S_NAME / "weights" / "last.pt"


def train_topdown():
    if not _TOPDOWN_DATA.exists():
        raise FileNotFoundError(
            f"{_TOPDOWN_DATA} not found — run  python detection/prepare_dataset.py  first"
        )
    model = YOLO(str(_TOPDOWN_BASE))
    model.train(
        data          = str(_TOPDOWN_DATA),
        epochs        = 100,
        imgsz         = 640,
        batch         = 16,
        lr0           = 1e-3,
        lrf           = 0.01,
        warmup_epochs = 3,
        degrees       = 45,
        flipud        = 0.5,
        fliplr        = 0.5,
        scale         = 0.5,
        mosaic        = 1.0,
        hsv_h         = 0.015,
        hsv_s         = 0.7,
        hsv_v         = 0.4,
        project       = str(RUN_DIR),
        name          = _TOPDOWN_NAME,
        exist_ok      = True,
    )
    best = RUN_DIR / _TOPDOWN_NAME / "weights" / "best.pt"
    print(f"\n[finetune] Best weights → {best}")


def _vd_train_args(name, batch):
    return dict(
        data          = str(_VD_DATA),
        epochs        = 50,
        imgsz         = 1280,
        batch         = batch,
        lr0           = 1e-3,
        lrf           = 0.01,
        warmup_epochs = 3,
        degrees       = 15,
        flipud        = 0.5,
        fliplr        = 0.5,
        scale         = 0.3,
        mosaic        = 1.0,
        hsv_h         = 0.015,
        hsv_s         = 0.5,
        hsv_v         = 0.3,
        project       = str(RUN_DIR),
        name          = name,
        exist_ok      = True,
    )


def train_visdrone1280(resume: bool = False):
    if not _VD_DATA.exists():
        raise FileNotFoundError(f"VisDrone yaml not found: {_VD_DATA}")

    if resume:
        if not _VD_L_LAST.exists():
            raise FileNotFoundError(f"No checkpoint to resume from: {_VD_L_LAST}")
        model = YOLO(str(_VD_L_LAST))
        model.train(resume=True)
    else:
        base = _VD_L_LAST if _VD_L_LAST.exists() else _VD_L_BASE
        model = YOLO(str(base))
        args = _vd_train_args(_VD_L_NAME, batch=3)
        args["lr0"] = 1e-4          # fine-tune LR — model already trained on VisDrone
        args["warmup_epochs"] = 1
        args["epochs"] = 30
        model.train(**args)

    best = RUN_DIR / _VD_L_NAME / "weights" / "best.pt"
    print(f"\n[finetune] Best weights → {best}")
    print(f"[finetune] Update ros2_node.py MODEL_PT and imgsz=1280")


def train_visdrone_s_1280(resume: bool = False):
    if not _VD_DATA.exists():
        raise FileNotFoundError(f"VisDrone yaml not found: {_VD_DATA}")

    if resume:
        if not _VD_S_LAST.exists():
            raise FileNotFoundError(f"No checkpoint to resume from: {_VD_S_LAST}")
        model = YOLO(str(_VD_S_LAST))
        model.train(resume=True)
    else:
        base = _VD_S_LAST if _VD_S_LAST.exists() else _VD_S_BASE
        model = YOLO(str(base))
        # Train from scratch (or COCO weights): higher LR, more epochs, more augmentation
        model.train(**_vd_train_args(_VD_S_NAME, batch=6))

    best = RUN_DIR / _VD_S_NAME / "weights" / "best.pt"
    print(f"\n[finetune] Best weights → {best}")
    print(f"[finetune] Update ros2_node.py MODEL_PT and imgsz=1280")


def train_car_s_1280(resume: bool = False):
    if not _CAR_S_DATA.exists():
        raise FileNotFoundError(
            f"Car-only dataset not found: {_CAR_S_DATA}\n"
            "Run: python detection/runs/wenting_visdrone_11s/prepare_visdrone.py"
        )

    if resume:
        if not _CAR_S_LAST.exists():
            raise FileNotFoundError(f"No checkpoint to resume from: {_CAR_S_LAST}")
        model = YOLO(str(_CAR_S_LAST))
        model.train(resume=True)
    else:
        base = _CAR_S_LAST if _CAR_S_LAST.exists() else _VD_S_BASE
        model = YOLO(str(base))
        args = _vd_train_args(_CAR_S_NAME, batch=6)
        args["data"] = str(_CAR_S_DATA)   # override: car-only (nc=1), not 10-class
        model.train(**args)

    best = RUN_DIR / _CAR_S_NAME / "weights" / "best.pt"
    print(f"\n[finetune] Best weights → {best}")
    print(f"[finetune] Update ros2_node.py MODEL_PT and imgsz=1280")


def train_car_11s_1280(resume: bool = False):
    if not _CAR_S_DATA.exists():
        raise FileNotFoundError(
            f"Car-only dataset not found: {_CAR_S_DATA}\n"
            "Run: python detection/runs/wenting_visdrone_11s/prepare_visdrone.py"
        )

    if resume:
        if not _CAR_11S_LAST.exists():
            raise FileNotFoundError(f"No checkpoint to resume from: {_CAR_11S_LAST}")
        model = YOLO(str(_CAR_11S_LAST))
        model.train(resume=True)
    else:
        base = _CAR_11S_LAST if _CAR_11S_LAST.exists() else _CAR_11S_BASE
        model = YOLO(str(base))
        args = _vd_train_args(_CAR_11S_NAME, batch=6)
        args["data"] = str(_CAR_S_DATA)   # car-only (nc=1)
        model.train(**args)

    best = RUN_DIR / _CAR_11S_NAME / "weights" / "best.pt"
    print(f"\n[finetune] Best weights → {best}")
    print(f"[finetune] Update ros2_node.py MODEL_PT and imgsz=1280")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode",
                    choices=["topdown", "visdrone1280", "visdrone_s_1280", "car_s_1280",
                             "car_11s_1280"],
                    default="topdown")
    ap.add_argument("--resume", action="store_true",
                    help="Resume from last checkpoint")
    args = ap.parse_args()

    if args.mode == "visdrone1280":
        train_visdrone1280(resume=args.resume)
    elif args.mode == "visdrone_s_1280":
        train_visdrone_s_1280(resume=args.resume)
    elif args.mode == "car_s_1280":
        train_car_s_1280(resume=args.resume)
    elif args.mode == "car_11s_1280":
        train_car_11s_1280(resume=args.resume)
    else:
        train_topdown()
