# detection/ — Vehicle Detection

Real-time vehicle detection from the drone's nadir camera. Uses YOLO with VisDrone-trained models and automatically maps model-specific class names to four canonical vehicle labels.

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

ROS2 Jazzy + `ros-jazzy-vision-msgs` for the ROS2 node.

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
| Subscribe | `/drone/camera/image_raw` | `sensor_msgs/Image` | rgb8, 1024×768 |
| Subscribe | `/drone/pose` | `geometry_msgs/PoseStamped` | ENU pose (for geo-tagging) |
| Subscribe | `/drone/agl` | `std_msgs/Float64` | AGL in metres — inference gated on AGL ≥ 50 m |
| Publish | `/yolo/detections` | `vision_msgs/Detection2DArray` | bounding boxes + class + confidence |

**Survey mission integration:** `px4_commander.py` subscribes to `/yolo/detections` and on
vehicle detection projects the bounding-box centre to world coordinates via yaw-corrected
GSD, deduplicates within 5 m, and appends to `detections.csv` (timestamp, category,
confidence, lat, lon, agl_m). The survey route is never interrupted.

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
