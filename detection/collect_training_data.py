"""
Headless Isaac Sim synthetic data collector for top-down vehicle detection.

Builds a flat scene with coloured vehicle boxes, flies a nadir camera on a grid
at multiple altitudes, and exports YOLO-format images + labels.

Output:
    detection/dataset/synth/images/  ← JPEG frames
    detection/dataset/synth/labels/  ← YOLO .txt files

Run:
    OMNI_KIT_ACCEPT_EULA=Y conda run -n isaac_sim_test \\
        python detection/collect_training_data.py

After collection, re-run prepare_dataset.py to merge synth into the main dataset.
"""
from __future__ import annotations

import math
import random
import sys
from pathlib import Path
from PIL import Image

HERE = Path(__file__).parent
sys.path.insert(0, str(HERE))
from label_writer import (
    CLASS_CAR, CLASS_MOTO, CLASS_BUS, CLASS_TRUCK,
    CAM_W, CAM_H, vehicle_label, write_label,
)

# ── output dirs ────────────────────────────────────────────────────────────────
SYNTH_IMG = HERE / "dataset" / "synth" / "images"
SYNTH_LBL = HERE / "dataset" / "synth" / "labels"
SYNTH_IMG.mkdir(parents=True, exist_ok=True)
SYNTH_LBL.mkdir(parents=True, exist_ok=True)

# ── scene constants ─────────────────────────────────────────────────────────────
GROUND_Z     = 0.0
SCENE_RADIUS = 150.0
RANDOM_SEED  = 42

# Vehicle dims: (length_m, width_m, height_m)
_DIMS = {
    CLASS_CAR:   (4.64, 1.775, 1.45),
    CLASS_MOTO:  (2.20, 0.80,  1.20),
    CLASS_BUS:   (12.0, 2.50,  3.50),
    CLASS_TRUCK: (8.00, 2.40,  3.00),
}

# Vehicle counts per class
_COUNTS = {CLASS_CAR: 25, CLASS_MOTO: 8, CLASS_BUS: 4, CLASS_TRUCK: 6}

# Grid altitudes (AGL) and settle steps before each capture
ALTITUDES_M      = [30.0, 60.0, 100.0]
CAM_SETTLE_STEPS = 4

# ── Isaac Sim ──────────────────────────────────────────────────────────────────
from isaacsim import SimulationApp
simulation_app = SimulationApp({
    "headless": True,
    "width":    CAM_W,
    "height":   CAM_H,
    "window_title": "Synthetic data collector",
})

import omni.usd
from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdShade
import omni.replicator.core as rep

stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)

# ── USD helpers ────────────────────────────────────────────────────────────────
_used_paths: set = set()

def _upath(base: str) -> str:
    p, i = base, 0
    while p in _used_paths:
        i += 1
        p = f"{base}_{i}"
    _used_paths.add(p)
    return p

def _pbr(path: str, rgb: tuple, metallic: float = 0.0, roughness: float = 0.7):
    mat = UsdShade.Material.Define(stage, path)
    sh  = UsdShade.Shader.Define(stage, path + "/S")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
    sh.CreateInput("metallic",     Sdf.ValueTypeNames.Float).Set(metallic)
    sh.CreateInput("roughness",    Sdf.ValueTypeNames.Float).Set(roughness)
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    return mat

def _vehicle_box(path: str, x: float, y: float, z_base: float,
                 length: float, width: float, height: float,
                 mat, yaw_deg: float = 0.0) -> None:
    """Place one vehicle as a coloured box; z_base is the bottom of the vehicle."""
    b = UsdGeom.Cube.Define(stage, path)
    b.CreateSizeAttr(1.0)
    xf = UsdGeom.Xformable(b)
    xf.AddTranslateOp().Set(Gf.Vec3d(x, y, z_base + height * 0.5))
    xf.AddRotateZOp().Set(float(-yaw_deg))      # compass → USD
    xf.AddScaleOp().Set(Gf.Vec3f(width, length, height))
    UsdShade.MaterialBindingAPI(b.GetPrim()).Bind(mat)

# ── scene setup ────────────────────────────────────────────────────────────────
ground_mat = _pbr("/Mat/Ground", (0.13, 0.13, 0.13), roughness=0.95)
gnd = UsdGeom.Cube.Define(stage, "/World/Ground")
gnd.CreateSizeAttr(1.0)
xf_gnd = UsdGeom.Xformable(gnd)
xf_gnd.AddScaleOp().Set(Gf.Vec3f(400.0, 400.0, 0.20))
xf_gnd.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, -0.10))
UsdShade.MaterialBindingAPI(gnd.GetPrim()).Bind(ground_mat)

sky = UsdLux.DomeLight.Define(stage, "/World/Sky")
sky.CreateIntensityAttr(200)
sky.CreateColorAttr(Gf.Vec3f(0.52, 0.68, 1.0))
sun = UsdLux.DistantLight.Define(stage, "/World/Sun")
sun.CreateIntensityAttr(2500)
sun.CreateColorAttr(Gf.Vec3f(1.0, 0.97, 0.87))
UsdGeom.Xformable(sun).AddRotateXYZOp().Set(Gf.Vec3d(-55.0, 0.0, 30.0))

# ── vehicle materials ──────────────────────────────────────────────────────────
_CAR_COLORS = [
    (0.85, 0.10, 0.08),   # red
    (0.10, 0.25, 0.75),   # blue
    (0.92, 0.92, 0.90),   # white/silver
    (0.08, 0.08, 0.08),   # black
    (0.55, 0.55, 0.55),   # grey
]
_car_mats  = [_pbr(f"/Mat/Car{i}", c, metallic=0.4, roughness=0.3)
              for i, c in enumerate(_CAR_COLORS)]
_moto_mat  = _pbr("/Mat/Moto",  (0.12, 0.12, 0.14), roughness=0.5)
_bus_mat   = _pbr("/Mat/Bus",   (0.92, 0.88, 0.10), roughness=0.6)
_truck_mat = _pbr("/Mat/Truck", (0.70, 0.70, 0.72), metallic=0.1, roughness=0.5)

# ── place vehicles and build registry ─────────────────────────────────────────
rng = random.Random(RANDOM_SEED)

def _rand_pos() -> tuple[float, float]:
    r   = rng.uniform(10.0, SCENE_RADIUS)
    ang = rng.uniform(0.0, math.tau)
    return r * math.cos(ang), r * math.sin(ang)

# vehicle_registry: list of (class_id, x, y, ground_z, yaw_deg)
vehicle_registry: list[tuple] = []

for i in range(_COUNTS[CLASS_CAR]):
    x, y = _rand_pos()
    yaw  = rng.uniform(0.0, 360.0)
    l, w, h = _DIMS[CLASS_CAR]
    _vehicle_box(_upath(f"/World/Vehicles/Car{i:03d}"),
                 x, y, GROUND_Z, l, w, h, _car_mats[i % len(_car_mats)], yaw)
    vehicle_registry.append((CLASS_CAR, x, y, GROUND_Z, yaw))

for i in range(_COUNTS[CLASS_MOTO]):
    x, y = _rand_pos()
    yaw  = rng.uniform(0.0, 360.0)
    l, w, h = _DIMS[CLASS_MOTO]
    _vehicle_box(_upath(f"/World/Vehicles/Moto{i:03d}"),
                 x, y, GROUND_Z, l, w, h, _moto_mat, yaw)
    vehicle_registry.append((CLASS_MOTO, x, y, GROUND_Z, yaw))

for i in range(_COUNTS[CLASS_BUS]):
    x, y = _rand_pos()
    yaw  = rng.uniform(0.0, 360.0)
    l, w, h = _DIMS[CLASS_BUS]
    _vehicle_box(_upath(f"/World/Vehicles/Bus{i:03d}"),
                 x, y, GROUND_Z, l, w, h, _bus_mat, yaw)
    vehicle_registry.append((CLASS_BUS, x, y, GROUND_Z, yaw))

for i in range(_COUNTS[CLASS_TRUCK]):
    x, y = _rand_pos()
    yaw  = rng.uniform(0.0, 360.0)
    l, w, h = _DIMS[CLASS_TRUCK]
    _vehicle_box(_upath(f"/World/Vehicles/Truck{i:03d}"),
                 x, y, GROUND_Z, l, w, h, _truck_mat, yaw)
    vehicle_registry.append((CLASS_TRUCK, x, y, GROUND_Z, yaw))

print(f"[collect] Placed {len(vehicle_registry)} vehicles "
      f"({_COUNTS[CLASS_CAR]} cars, {_COUNTS[CLASS_MOTO]} motos, "
      f"{_COUNTS[CLASS_BUS]} buses, {_COUNTS[CLASS_TRUCK]} trucks)")

# ── drone + nadir camera ───────────────────────────────────────────────────────
drone_root   = UsdGeom.Xform.Define(stage, "/World/Drone")
drone_pos_op = drone_root.AddTranslateOp()
drone_pos_op.Set(Gf.Vec3d(0.0, 0.0, GROUND_Z + ALTITUDES_M[0]))

cam = UsdGeom.Camera.Define(stage, "/World/Drone/Camera")
cam.CreateFocalLengthAttr(18.0)
cam.CreateHorizontalApertureAttr(36.0)
cam.CreateVerticalApertureAttr(27.0)
cam.CreateClippingRangeAttr(Gf.Vec2f(0.1, 2000.0))
UsdGeom.Xformable(cam).AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, -0.15))

rp  = rep.create.render_product("/World/Drone/Camera", (CAM_W, CAM_H))
rgb_ann = rep.AnnotatorRegistry.get_annotator("rgb")
rgb_ann.attach([rp])

# ── build waypoints ────────────────────────────────────────────────────────────
def _grid_positions(alt: float) -> list[tuple[float, float]]:
    """(x, y) grid covering ±SCENE_RADIUS with 35 % lateral overlap."""
    step = 2.0 * alt * 0.65   # 90° HFOV → footprint = 2H; 35 % overlap
    positions = []
    x = -SCENE_RADIUS
    while x <= SCENE_RADIUS + 1.0:
        y = -SCENE_RADIUS
        while y <= SCENE_RADIUS + 1.0:
            positions.append((x, y))
            y += step
        x += step
    return positions

waypoints: list[tuple[float, float, float]] = []
for alt in ALTITUDES_M:
    for gx, gy in _grid_positions(alt):
        waypoints.append((gx, gy, GROUND_Z + alt))

print(f"[collect] {len(waypoints)} capture positions "
      f"({len(ALTITUDES_M)} altitudes × grid)")

# ── warm up ─────────────────────────────────────────────────────────────────────
for _ in range(10):
    simulation_app.update()

# ── capture loop ────────────────────────────────────────────────────────────────
n_saved = 0

for wx, wy, wz in waypoints:
    if not simulation_app.is_running():
        break

    drone_pos_op.Set(Gf.Vec3d(wx, wy, wz))
    for _ in range(CAM_SETTLE_STEPS):
        simulation_app.update()

    rep.orchestrator.step(rt_subframes=2, delta_time=0.0)
    raw = rgb_ann.get_data()

    if raw is None:
        print(f"[collect] ({wx:.0f},{wy:.0f},{wz:.0f}): no data — skipping")
        continue
    arr = raw if not isinstance(raw, dict) else raw.get("data")
    if arr is None or arr.size == 0:
        print(f"[collect] ({wx:.0f},{wy:.0f},{wz:.0f}): empty — skipping")
        continue

    # PIL conversion via bytes (safe: avoids .astype() on broken numpy)
    pil_img = Image.frombytes("RGBA", (CAM_W, CAM_H), arr.tobytes()).convert("RGB")

    # Project all vehicles; camera is at drone position (0.15 m offset negligible)
    cam_xyz = (wx, wy, wz)
    labels  = []
    for cls_id, vx, vy, vz, yaw in vehicle_registry:
        lab = vehicle_label(cam_xyz, (vx, vy, vz), yaw, cls_id)
        if lab is not None:
            labels.append(lab)

    stem = f"synth_{n_saved:05d}"
    pil_img.save(str(SYNTH_IMG / (stem + ".jpg")), "JPEG", quality=90)
    write_label(str(SYNTH_LBL / (stem + ".txt")), labels)

    agl = wz - GROUND_Z
    print(f"[collect] {stem}  ({wx:+.0f},{wy:+.0f})  agl={agl:.0f}m  "
          f"labels={len(labels)}")
    n_saved += 1

print(f"\n[collect] Done — {n_saved} frames → {SYNTH_IMG.parent}")
print(f"[collect] Next: python detection/prepare_dataset.py")
simulation_app.close()
