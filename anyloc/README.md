# anyloc/ — Visual Localisation (GPS-Denied)

Visual place recognition for GPS-denied drone navigation.  
Uses **DINOv2** patch features + **VLAD** aggregation + **FAISS** nearest-neighbour search against a geo-tagged satellite image database.

Active backbone: **ViT-B/14** (`dinov2_vitb14`) — the built database is in `anyloc/database/` and `ros2_node.py` points there. The localizer reads `model_name` from the database metadata and loads the correct backbone automatically.

In the full pipeline, AnyLoc runs as a **Phase 2 logger** when the drone is above 50 m AGL: it subscribes to `/drone/camera/image_raw`, retrieves the nearest geo-tagged satellite crop, and writes `latest_estimate.json` with the estimated lat/lon, error, and confidence score. The flight commander logs these estimates but does **not** fuse them into the EKF — kinematic truth is used for VPE throughout the survey. Fusing AnyLoc VPE (cov ≈ 800 m², 20–60 m error) destabilises EKF3 (`position lost` failsafe).

---

## Requirements

| Component | Version | Notes |
|---|---|---|
| OS | Ubuntu 22.04 / 24.04 | Tested on 24.04 |
| Python | 3.10–3.12 | Via conda `isaac_sim_test` env |
| PyTorch | ≥ 2.0 | Pre-installed by Isaac Sim |
| torchvision | ≥ 0.15 | Pre-installed by Isaac Sim |
| Pillow | ≥ 9.0 | |
| NumPy | ≥ 1.24 | |
| faiss-cpu | ≥ 1.7 | Install via conda-forge |
| OpenCV (cv2) | ≥ 4.7 | Pre-installed by Isaac Sim |
| requests | any | For NLSC tile download |
| ROS2 Jazzy | — | Required for `ros2_node.py` |
| MAVROS2 | — | Required for VPE publishing |

---

## 1. Conda Environment Setup

Uses the `isaac_sim_test` conda environment created by Isaac Sim. For standalone use without Isaac Sim:

```bash
conda create -n isaac_sim_test python=3.10 -y
conda activate isaac_sim_test
pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu
pip install pillow numpy requests opencv-python
conda install -n isaac_sim_test -c conda-forge faiss-cpu -y
```

---

## 2. Build the Image Database

Build once before running the localizer. Downloads NLSC PHOTO2 tiles (zoom 18) and encodes them as VLAD vectors.

Run from the **project root**:

```bash
# ViT-B/14 (active — used by ros2_node.py; larger VLAD dim = 49152)
conda run -n isaac_sim_test python anyloc/build_database.py

# ViT-S/14 (alternative — faster, smaller VLAD dim = 24576)
conda run -n isaac_sim_test python anyloc/build_database.py --model vits14
```

### Build options

| Flag | Default | Description |
|---|---|---|
| `--model vitb14\|vits14` | vitb14 | DINOv2 backbone |
| `--db-dir PATH` | auto | Output directory (default: `database/` for vitb14, `database_vits14/` for vits14) |
| `--grid-step N` | 50 | Grid spacing in metres |
| `--agl-min N` | 65 | Minimum AGL altitude |
| `--agl-max N` | 65 | Maximum AGL altitude |
| `--agl-step N` | 5 | AGL increment |
| `--rebuild` | off | Overwrite existing database |

### What the build does

1. Downloads NLSC PHOTO2 tiles at zoom 18 and stitches them into a mosaic (~0.60 m/px effective after MAX_TEX=16384 cap)
2. Crops satellite patches for each (lat, lon, AGL) grid point simulating a nadir drone camera
3. Encodes each crop with DINOv2 ViT-S/14 or ViT-B/14 (model downloaded on first run)
4. Clusters patch descriptors into a VLAD codebook (k=64) with FAISS k-means
5. Saves metadata (including `model_name`) to the output directory

### Output

```
anyloc/database/              ← active (ViT-B/14, used by ros2_node.py)
├── database.pt               # split-file pointer
├── database_meta.pt          # lats, lons, alts, codebook, model_name
├── database_vlads.pt         # VLAD vectors (N × 49152, ~550 MB)
├── db_images/                # satellite crop JPEGs
└── db_meta.json              # build cache (skip re-crop if present)

anyloc/database_vits14/       ← alternative (ViT-S/14, ~265 MB, faster)
├── database_meta.pt          # VLAD vectors (N × 24576)
└── …
```

Current databases: **~2 820 entries**, 50 m grid, AGL 65 m only (single altitude layer matches mission cruise altitude).

---

## 3. Run the Localizer (standalone)

```bash
conda run -n isaac_sim_test python anyloc/run_localizer.py
```

---

## 4. Accuracy Benchmark (NLSC PHOTO2)

Measures localizer accuracy at known ground-truth coordinates using NLSC imagery.

```bash
conda run -n isaac_sim_test python anyloc/test_accuracy_esri.py
```

| Flag | Default | Description |
|---|---|---|
| `--samples N` | 20 | Number of random test points |
| `--agl N` | 80 | AGL in metres (0 = randomise 60–120 m) |
| `--seed N` | 42 | Random seed |
| `--output FILE` | — | Save results to JSON |
| `--plot` | off | Show error histogram + spatial map |

---

## 5. Constrained-Search Benchmark

Measures accuracy and speed of the **anchor-chain constrained search** used in `ros2_node.py`. Instead of searching all ~2.8k entries, each retrieval considers only DB entries within 200 m of the previous estimate.

```bash
conda run -n isaac_sim_test python anyloc/test_accuracy_constrained.py
```

| Flag | Default | Description |
|---|---|---|
| `--steps N` | 20 | Trajectory steps |
| `--agl N` | 80 | AGL in metres |
| `--radius N` | 200 | Search radius in metres |
| `--seed N` | 42 | Random seed |
| `--output FILE` | — | Save results to JSON |
| `--plot` | off | Show per-step error and latency charts |

Typical speedup: **~4×** faster vs global search; constrained RMSE lower than global (anchor eliminates far-away false positives).

---

## 6. ViT-B vs ViT-S Comparison

Benchmarks feature-extraction speed and (if both databases are built) full localization accuracy for both backbones side by side.

```bash
# Speed test only (no ViT-S DB needed):
conda run -n isaac_sim_test python anyloc/test_vit_comparison.py --speed-only

# Full accuracy + speed comparison:
conda run -n isaac_sim_test python anyloc/test_vit_comparison.py
```

| Flag | Default | Description |
|---|---|---|
| `--steps N` | 20 | Trajectory steps for accuracy test |
| `--agl N` | 80 | AGL in metres |
| `--seed N` | 42 | Random seed |
| `--speed-only` | off | Skip accuracy test |

---

## 7. Run the ROS2 Node (full pipeline)

The ROS2 node processes camera frames, runs AnyLoc retrieval, and publishes VPE to MAVROS.

**Prerequisites:** the autopilot SITL and MAVROS2 must be running first (see `run.sh`).

```bash
bash anyloc/run_ros2_localizer.sh
```

Or manually:
```bash
source /opt/ros/jazzy/setup.bash
DISPLAY=:2 conda run -n isaac_sim_test --no-capture-output python3 -u anyloc/ros2_node.py
```

### ROS2 topics

| Direction | Topic | Type | Notes |
|---|---|---|---|
| Subscribe | `/drone/camera/image_raw` | `sensor_msgs/Image` | rgb8, 1024×768 (AP-IMX900-Mini-USB3-I5 at half native; optics: 88°×65.1° FOV, EFL 3.1 mm) |
| Subscribe | `/drone/pose` | `geometry_msgs/PoseStamped` | WGS84 (lat, lon, alt_msl) |
| Subscribe | `/drone/agl` | `std_msgs/Float64` | AGL in metres |
| Publish | `/anyloc/pose_estimate` | `geometry_msgs/PoseWithCovarianceStamped` | AnyLoc estimate (monitoring) |

**VPE to MAVROS is not published by this node.** `px4_commander.py` reads `latest_estimate.json` and publishes `/mavros/vision_pose/pose_cov` with correct per-axis covariance. Publishing from both processes caused duplicate EKF2 inputs.

### latest_estimate.json format

```json
{
  "est_lat": 23.4512,
  "est_lon": 120.2847,
  "yaw_deg": 0.0,
  "agl_m": 82.3,
  "error_m": 55.1,
  "timestamp": 1748991234.5
}
```

---

## VPE Integration with Flight Commander

`px4_commander.py` reads `latest_estimate.json` in its vision thread:
- Phase 1 (AGL < `MIN_LOCALISATION_AGL` = 50 m): sends kinematic truth, cov = 0.1 m²
- Phase 2 (AGL ≥ 50 m): sends AnyLoc estimate, cov = max(1, error_m²)

The covariance difference lets PX4's EKF2 automatically weight the two sources: tight covariance on ground truth during climb, loose covariance on AnyLoc during cruise.

---

## Troubleshooting

**`ImportError: No module named 'faiss'`**  
→ `conda install -n isaac_sim_test -c conda-forge faiss-cpu`

**`RuntimeError: CUDA out of memory`**  
→ DINOv2 runs on CPU by default. GPU: ensure PyTorch+CUDA is installed.

**Database build fails on tile download**  
→ Check connectivity to `wmts.nlsc.gov.tw` (NLSC). Script retries 3× per tile; persists on failure.

**`latest_estimate.json` not updating**  
→ The node only publishes when AGL ≥ 50 m. The file is written as a stub at startup; the VPE guard requires the altitude threshold to be satisfied.

**Wrong model loaded after switching databases**  
→ `model_name` is stored in `database_meta.pt`. The localizer reads it automatically — no code changes needed when switching between `database/` and `database_vits14/`.
