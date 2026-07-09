# detection/ — Vehicle Detection

Real-time vehicle detection from the drone's nadir camera. Uses YOLO with VisDrone-trained models and automatically maps model-specific class names to four canonical vehicle labels.

**Inference backend:** `YOLODetector` runs on a TensorRT FP16 engine, not raw PyTorch. On first load for a given `.pt` file it auto-exports a sibling `.engine` (one-time cost, ~15 min on Jetson Orin NX) and loads that on every run after. Eager PyTorch fp32 at `imgsz=1280` was GPU-bound at ~8 fps on Orin NX; the FP16 TensorRT engine gets ~17 fps for the same weights — but only if `sudo jetson_clocks` has been run, otherwise DVFS throttling gives back most of the gain. See [Performance](#performance) below.

---

## Files

| File | Purpose |
|------|---------|
| `detector.py` | `YOLODetector` class — wraps Ultralytics YOLO |
| `ros2_node.py` | ROS2 node — subscribes to camera, publishes detections |
| `run_ros2_detector.sh` | Launch script (sources ROS2 Jazzy, runs in conda env) |
| `run_detector.py` | Standalone test runner (no ROS2) |
| `collect_training_data.py` | Collect frames + pseudo-labels for fine-tuning |
| `prepare_dataset.py` | Convert collected data to YOLO format |
| `finetune.py` | Fine-tune YOLO on VisDrone dataset |
| `test_map_car.py` | mAP benchmark: YOLOv8s vs YOLO11s car-only models |
| `label_writer.py` | Write YOLO label files from detection dicts |

---

## Models

| Name | Backbone | Classes | imgsz | Notes |
|------|----------|---------|-------|-------|
| `car_s_1280` | YOLOv8s | 1 (car+van) | 1280 | trained on VisDrone car-only dataset |
| `car_11s_1280` | YOLO11s | 1 (car+van) | 1280 | same dataset, YOLO11s backbone |
| `visdrone_s_1280` | YOLOv8s | 10 | 1280 | full VisDrone 10-class |
| `visdrone_1280` | YOLOv8l | 10 | 1280 | full VisDrone 10-class, large |
| `wenting_11s` | YOLO11s | 1 (car+van) | 1536 | pre-trained reference |

Weights are saved to `detection/runs/<name>/weights/best.pt`.

---

## Requirements

```bash
conda run -n Drone_NV_Isaac_sim pip install ultralytics
```

ROS2 Jazzy + `ros-jazzy-vision-msgs` for the ROS2 node. TensorRT + ONNX come from JetPack (not pip) on the Jetson — confirm with `python3 -c "import tensorrt, onnx"`.

---

## Performance

`YOLODetector.__init__` (in `detector.py`) exports `<model>.engine` next to any `.pt` weights the first time they're loaded, then loads the engine on every subsequent run (skips re-export if the `.engine` file already exists). Delete the `.engine` file to force a re-export, e.g. after retraining the weights. Rectangular sizes get a size-tagged name (`<model>_<h>x<w>.engine`) so they can't be confused with the square engine, and the export runs in a tempdir so it can't clobber an existing engine of a different size.

**Rectangular engine (2026-07-09):** the production node now runs `imgsz=(960, 1280)` instead of the 1280×1280 square. A 1640×1232 frame letterboxed into a square wastes ~25% of the input (and thus compute) on gray padding; the rect engine runs the same pixels at the same scale with none — ~25% faster preprocess *and* inference at identical accuracy (validated on survey13 footage: every square-engine detection matched at IoU>0.5, mean 0.89).

**Pipelined node (2026-07-09):** `ros2_node.py` splits the work into a CPU preprocess thread (letterbox + tensor upload, ~16 ms) and a GPU inference thread (~30-47 ms) with depth-1 hand-off slots, so throughput is bounded by the slower stage instead of their sum. Detection latency gains one stage (~1 frame); `ground_view_stream.py`'s stamp-matching absorbs that.

**Jetson clocks matter more than the engine itself.** `nvpmodel MAXN_SUPER` only raises the clock ceiling — it doesn't force the GPU/EMC to run at max. Without `jetson_clocks`, DVFS keeps clocks low between bursts and inference stays close to fp32 speeds even with the FP16 engine loaded (measured 47 ms/frame). After `jetson_clocks`, inference dropped to ~18.6 ms/frame. **Automated as of 2026-07-08** — `jetson_clocks.service` (systemd, `After=nvpmodel.service`) runs it on every boot; no manual step needed. Verify with `systemctl is-active jetson_clocks.service` before flying (should print `active`) rather than re-running it by hand.

Measured on `Car_visdrone1280.pt` at `imgsz=1280` on Jetson Orin NX:

| Backend | ms/frame | fps |
|---|---|---|
| PyTorch fp32 (old default) | ~121 | ~8.3 |
| TensorRT fp16, clocks not locked | ~83 | ~12 |
| TensorRT fp16, `jetson_clocks` run | ~58 | ~17.3 |

Preprocessing (CPU-side letterbox resize) is ~16 ms/frame — with the pipelined node it overlaps inference instead of adding to it. If more headroom is needed later, the next knob is an INT8 engine (needs a calibration set + an mAP check with `test_map_car.py` — small objects are the first casualty of INT8), not a smaller model.

Note the end-to-end rate is capped by what the camera actually delivers — see the FastDDS SHM profile note in `control/ros2_env.sh` (2026-07-09): without it, each subscriber silently drops ~30% of the 6 MB frames in transport and no amount of detector speed helps.

**`--headless` skips the postview render entirely (2026-07-07):** `ros2_node.py` used to build the annotated/resized postview frame every callback even with `--headless`, even though nothing displayed it — a ~37 ms/frame PIL LANCZOS resize wasted on every frame (comparable to inference itself). `--headless` now returns right after publishing detections, before that work runs.

---

## Training

### Fine-tune a model

```bash
conda activate Drone_NV_Isaac_sim

# YOLOv8s, car-only, imgsz=1280
python detection/finetune.py --mode car_s_1280

# YOLO11s, car-only, imgsz=1280
python detection/finetune.py --mode car_11s_1280

# Resume a run
python detection/finetune.py --mode car_11s_1280 --resume
```

| Mode | Backbone | Dataset | imgsz |
|------|----------|---------|-------|
| `topdown` | YOLOv8n | synthetic | 640 |
| `visdrone1280` | YOLOv8l | VisDrone 10-class | 1280 |
| `visdrone_s_1280` | YOLOv8s | VisDrone 10-class | 1280 |
| `car_s_1280` | YOLOv8s | VisDrone car-only | 1280 |
| `car_11s_1280` | YOLO11s | VisDrone car-only | 1280 |

The car-only dataset maps VisDrone classes `car` and `van` → class 0. Required before training car modes:
```bash
python detection/runs/wenting_visdrone_11s/prepare_visdrone.py
```

### mAP benchmark (YOLOv8s vs YOLO11s)

```bash
python detection/test_map_car.py
```

Evaluates `car_s_1280` and `car_11s_1280` on the VisDrone val set and prints mAP50 / mAP50-95 side by side.

---

## Canonical Labels

`YOLODetector` maps both COCO and VisDrone class names to four labels:

| Canonical | COCO name | VisDrone names |
|-----------|-----------|----------------|
| `car` | `car` | `car`, `van` |
| `motorcycle` | `motorcycle` | `motor`, `tricycle`, `awning-tricycle` |
| `bus` | `bus` | `bus` |
| `truck` | `truck` | `truck` |

All other classes are filtered out. This makes the node model-agnostic — drop in any COCO or VisDrone model without code changes.

---

## Run the ROS2 Node

**Prerequisites:** `cesium_scene.py` (or `drone_sim.py`) must be publishing `/drone/camera/image_raw`.

```bash
bash detection/run_ros2_detector.sh
```

Or manually:
```bash
source /opt/ros/jazzy/setup.bash
conda run -n isaac_sim_test --no-capture-output python3 detection/ros2_node.py
```

### ROS2 Topics

| Direction | Topic | Type | Notes |
|---|---|---|---|
| Subscribe | `/drone/camera/image_raw` | `sensor_msgs/Image` | rgb8, 1640×1232 (IMX219 native) |
| Subscribe | `/drone/pose` | `geometry_msgs/PoseStamped` | WGS84, `frame_id="wgs84"` — `position.x/y` = lat/lon (for geo-tagging) |
| Publish | `/yolo/detections` | `vision_msgs/Detection2DArray` | bounding boxes + class + confidence |

Inference runs on every frame regardless of altitude — no AGL gate (unlike AnyLoc, which only fuses ≥ 50 m).

**Contract relied on downstream (don't break):** the array's `header` is copied verbatim from the source image, and a message is published for **every** processed frame, even with zero detections. `tools/ground_view_stream.py` (2026-07-09) stamp-matches detections to its frame buffer for a lag-free overlay and treats >2 s of silence as "YOLO down".

**Survey mission integration:** Both `px4_commander.py` and `ardupilot_commander.py` subscribe to `/yolo/detections`. On vehicle detection each commander projects the bounding-box centre to world coordinates via yaw-corrected GSD, deduplicates within 5 m, and appends to `detections.csv` (timestamp, category, confidence, lat, lon, agl_m). The survey route is never interrupted.

### Detection2D fields

Each detection in the array:
- `bbox.center.position.x/y` — bounding box centre in pixels
- `bbox.size_x/y` — bounding box width/height in pixels
- `results[0].hypothesis.class_id` — canonical label (`car`, `bus`, etc.)
- `results[0].hypothesis.score` — confidence [0, 1]

---

## Standalone Usage

```python
from PIL import Image
from detection.detector import YOLODetector

det = YOLODetector(model_name='detection/runs/car_11s_1280/weights/best.pt', conf=0.35)

img = Image.open('frame.jpg')
detections = det.detect(img)
# [{'label': 'car', 'conf': 0.72, 'box': [x1, y1, x2, y2]}, ...]

annotated = det.draw(img, detections)
annotated.save('annotated.jpg')
```
