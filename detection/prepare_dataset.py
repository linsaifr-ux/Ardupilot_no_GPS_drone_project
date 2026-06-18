"""
Download VisDrone 2019 DET and convert to 4-class top-down YOLO format.

Classes: 0=car  1=motorcycle  2=bus  3=truck

VisDrone source classes remapped:
  4  car             → 0
  5  van             → 0  (looks like a car from above)
  6  truck           → 3
  7  tricycle        → 1
  8  awning-tricycle → 1
  9  bus             → 2
  10 motor           → 1

Run (outside Isaac Sim, in any env with ultralytics):
    python detection/prepare_dataset.py

Output layout:
    detection/dataset/
    ├── data.yaml
    ├── images/train/   ← VisDrone train images (symlinked)
    ├── images/val/     ← VisDrone val   images (symlinked)
    ├── labels/train/   ← remapped YOLO .txt files
    └── labels/val/
"""
from __future__ import annotations

import os
import shutil
from pathlib import Path

from PIL import Image

_VISDRONE_MAP = {
    4: 0,    # car
    5: 0,    # van → car
    6: 3,    # truck
    7: 1,    # tricycle → motorcycle
    8: 1,    # awning-tricycle → motorcycle
    9: 2,    # bus
   10: 1,    # motor
}

HERE        = Path(__file__).parent
DATASET_DIR = HERE / "dataset"


def _convert_split(img_dir: Path, ann_dir: Path,
                   out_img: Path, out_lbl: Path) -> None:
    out_img.mkdir(parents=True, exist_ok=True)
    out_lbl.mkdir(parents=True, exist_ok=True)

    imgs = sorted(img_dir.glob("*.jpg")) + sorted(img_dir.glob("*.png"))
    print(f"  {len(imgs)} images  ({img_dir})")
    converted = ignored = 0

    for img_path in imgs:
        ann_path = ann_dir / (img_path.stem + ".txt")
        if not ann_path.exists():
            continue

        with Image.open(img_path) as im:
            iw, ih = im.size

        labels: list[tuple] = []
        for line in ann_path.read_text().strip().splitlines():
            parts = line.strip().split(",")
            if len(parts) < 6:
                continue
            x1, y1    = int(parts[0]), int(parts[1])
            bw, bh    = int(parts[2]), int(parts[3])
            score     = int(parts[4])
            cat       = int(parts[5])

            if score == 0 or bw <= 0 or bh <= 0:
                ignored += 1
                continue
            if cat not in _VISDRONE_MAP:
                ignored += 1
                continue

            xc = (x1 + bw * 0.5) / iw
            yc = (y1 + bh * 0.5) / ih
            labels.append((_VISDRONE_MAP[cat], xc, yc, bw / iw, bh / ih))

        if not labels:
            continue

        dst_img = out_img / img_path.name
        if not dst_img.exists():
            os.symlink(img_path.resolve(), dst_img)

        lbl_file = out_lbl / (img_path.stem + ".txt")
        with open(lbl_file, "w") as f:
            for cls, xc, yc, w, h in labels:
                f.write(f"{cls} {xc:.6f} {yc:.6f} {w:.6f} {h:.6f}\n")
        converted += 1

    print(f"  → {converted} images written, {ignored} annotations skipped")


def _merge_synth() -> None:
    synth_img = DATASET_DIR / "synth" / "images"
    synth_lbl = DATASET_DIR / "synth" / "labels"
    if not synth_img.exists():
        return

    out_img = DATASET_DIR / "images" / "train"
    out_lbl = DATASET_DIR / "labels" / "train"
    n = 0
    for src in sorted(synth_img.glob("*.jpg")):
        dst = out_img / src.name
        if not dst.exists():
            shutil.copy2(src, dst)
            lbl_src = synth_lbl / (src.stem + ".txt")
            if lbl_src.exists():
                shutil.copy2(lbl_src, out_lbl / lbl_src.name)
            n += 1
    print(f"[prepare] Merged {n} synthetic images into train split")


def main() -> None:
    print("[prepare] Downloading / verifying VisDrone 2019 DET …")
    from ultralytics.data.utils import check_det_dataset
    info = check_det_dataset("VisDrone.yaml")
    vd_root = Path(info["path"])
    print(f"[prepare] VisDrone root: {vd_root}")

    DATASET_DIR.mkdir(parents=True, exist_ok=True)

    print("[prepare] Converting train split …")
    _convert_split(
        vd_root / "VisDrone2019-DET-train" / "images",
        vd_root / "VisDrone2019-DET-train" / "annotations",
        DATASET_DIR / "images" / "train",
        DATASET_DIR / "labels" / "train",
    )

    print("[prepare] Converting val split …")
    _convert_split(
        vd_root / "VisDrone2019-DET-val" / "images",
        vd_root / "VisDrone2019-DET-val" / "annotations",
        DATASET_DIR / "images" / "val",
        DATASET_DIR / "labels" / "val",
    )

    _merge_synth()

    data_yaml = DATASET_DIR / "data.yaml"
    data_yaml.write_text(
        f"path: {DATASET_DIR.resolve()}\n"
        f"train: images/train\n"
        f"val:   images/val\n"
        f"\n"
        f"nc: 4\n"
        f"names: [car, motorcycle, bus, truck]\n"
    )
    print(f"[prepare] Wrote {data_yaml}")

    n_train = len(list((DATASET_DIR / "images" / "train").glob("*")))
    n_val   = len(list((DATASET_DIR / "images" / "val").glob("*")))
    print(f"[prepare] Done — train={n_train}  val={n_val}")


if __name__ == "__main__":
    main()
