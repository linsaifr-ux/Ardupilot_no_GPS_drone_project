#!/usr/bin/env python3
"""
Chiayi, Taiwan — pure Cesium ion 3D Tiles scene.

All geometry comes from Cesium ion REST API:
  Asset 1      — Cesium World Terrain (quantized-mesh-1.0)
  Asset 96188  — Cesium OSM Buildings (3D Tiles B3DM)

Satellite imagery: Taiwan NLSC PHOTO2 WMTS (zoom 18, ~0.60 m/px).

No OSM, no SRTM, no Overpass.  Just Cesium.

Run:
    DISPLAY=:2 OMNI_KIT_ACCEPT_EULA=Y conda run -n isaac_sim_test python cesium_scene.py
"""

import csv
import io as _io, json, math, os, queue as _queue, struct, sys, threading, time, urllib.parse, urllib.request

# Project root on path so control/ package is importable from simulator/
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# PX4_SIM=1 → talk to PX4 SITL (MAVLink HIL on TCP 4560); else ArduPilot (JSON UDP 9002)
_PX4_SIM = bool(os.environ.get("PX4_SIM"))
if _PX4_SIM:
    from control.px4_sim_bridge import PX4SimBridge
else:
    from control.sitl_bridge import SITLBridge

# ── Drone physics constants ────────────────────────────────────────────────────
DRONE_MASS  = 1.0          # kg — hover at PWM 1500 (p_norm=0.5) matches MOT_THST_HOVER=0.5
_K_THRUST   = DRONE_MASS * 9.81 / 2.0   # N per unit normalized PWM per motor
_MOTOR_ARM  = 0.40         # m from centre to motor (matches visual arm length)
_SQ2        = math.sqrt(2.0) / 2.0
# ArduCopter X-frame FRAME_TYPE=1: ch1=FR, ch2=RL, ch3=RR, ch4=FL
# In scene ENU body frame (x=East-local, y=North-local, z=Up)
_MOTOR_LOCAL = [
    ( _MOTOR_ARM * _SQ2,  _MOTOR_ARM * _SQ2, 0.0),  # ch1 M1 Front-Right NE
    (-_MOTOR_ARM * _SQ2, -_MOTOR_ARM * _SQ2, 0.0),  # ch2 M2 Rear-Left   SW
    ( _MOTOR_ARM * _SQ2, -_MOTOR_ARM * _SQ2, 0.0),  # ch3 M3 Rear-Right  SE
    (-_MOTOR_ARM * _SQ2,  _MOTOR_ARM * _SQ2, 0.0),  # ch4 M4 Front-Left  NW
]

# ROS2 Jazzy Python packages (Python 3.12) — compatible with Isaac Sim 6.0 (Python 3.12).
# run_chiayi.sh sources /opt/ros/humble/setup.bash so ROS2 shared libs are on LD_LIBRARY_PATH.
_ROS2_SITE = "/opt/ros/humble/lib/python3.10/site-packages"
if os.path.isdir(_ROS2_SITE) and _ROS2_SITE not in sys.path:
    sys.path.insert(0, _ROS2_SITE)
try:
    import rclpy
    import rclpy.node
    from sensor_msgs.msg import Image as RosImage
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import Float64
    _ROS2_OK = True
except ImportError as _e:
    print(f"[ROS2] rclpy not available ({_e}) — falling back to file output")
    _ROS2_OK = False

import numpy as np
import requests
from PIL import Image

# ── SCENE CONSTANTS ────────────────────────────────────────────────────────────
CENTER_LAT = 23.450868
CENTER_LON = 120.286135
RADIUS_M   = 2000.0
R_EARTH    = 6_371_000.0
COS_LAT    = math.cos(math.radians(CENTER_LAT))
HERE             = os.path.dirname(os.path.abspath(__file__))
DRONE_FRAME_DIR  = os.path.join(HERE, "drone_frames")
DRONE_CAM_W, DRONE_CAM_H = 1024, 768    # render res (optics stay 88°×65.1°; native=2048×1536)
DRONE_SAVE_EVERY  = 1      # capture every render frame (rep.step dominates; no benefit skipping)
DRONE_SAVE_FRAMES = False  # write drone_frames/latest.jpg to disk (debug; off by default)
DRONE_SPEED_M    = 5.0  # keyboard move step (m)

# WGS-84
_A  = 6_378_137.0
_E2 = 0.006_694_379_990_14

CESIUM_ION_TOKEN = (
    "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9"
    ".eyJqdGkiOiIzY2E1NmFkNC1jNjg3LTRjMmUtOWJlMi1hODhmNjY1NjcxMDMiLCJpZCI6NDMxNzYzLCJpc3MiOiJodHRwczovL2lvbi5jZXNpdW0uY29tIiwiYXVkIjoidW5kZWZpbmVkX2RlZmF1bHQiLCJpYXQiOjE3Nzg4MDQzNDF9"
    ".OLYEHi742XKljoOW6vPi7HnLcokgZuUr9M0BIbQheHI"
)
CESIUM_TERRAIN_ASSET   = 1
CESIUM_BUILDINGS_ASSET = 96188

# ── ENU helpers ────────────────────────────────────────────────────────────────
def to_xy(lat, lon):
    return (math.radians(lon - CENTER_LON) * R_EARTH * COS_LAT,
            math.radians(lat - CENTER_LAT) * R_EARTH)

def to_latlon(x, y):
    return (CENTER_LAT + (y / R_EARTH) * (180 / math.pi),
            CENTER_LON + (x / (R_EARTH * COS_LAT)) * (180 / math.pi))

# ── ECEF → geodetic (vectorised) ───────────────────────────────────────────────
def ecef_to_geodetic_vec(xyz: np.ndarray):
    x, y, z = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    lon = np.degrees(np.arctan2(y, x))
    p   = np.sqrt(x**2 + y**2)
    lat = np.arctan2(z, p * (1.0 - _E2))
    for _ in range(10):
        N   = _A / np.sqrt(1.0 - _E2 * np.sin(lat)**2)
        lat = np.arctan2(z + _E2 * N * np.sin(lat), p)
    N   = _A / np.sqrt(1.0 - _E2 * np.sin(lat)**2)
    alt = np.where(np.abs(np.cos(lat)) > 1e-10,
                   p / np.cos(lat) - N,
                   np.abs(z) / np.abs(np.where(np.sin(lat) == 0, 1e-30, np.sin(lat)))
                   - N * (1 - _E2))
    return np.degrees(lat), lon, alt

# ── Isaac Sim ──────────────────────────────────────────────────────────────────
# Parse --no-window before SimulationApp (must happen before Kit initialises)
_NO_WINDOW = "--no-window" in sys.argv
if _NO_WINDOW:
    sys.argv.remove("--no-window")

_RASTERIZE = "--rasterize" in sys.argv
if _RASTERIZE:
    sys.argv.remove("--rasterize")

from isaacsim import SimulationApp
simulation_app = SimulationApp({
    "headless":     _NO_WINDOW,
    "renderer":     "FullRasterization" if _RASTERIZE else "RaytracingLighting",
    "width":        1280,
    "height":       720,
    "window_title": "Isaac Sim — Cesium ion  23.45°N 120.29°E",
})

# Disable RTX denoiser — it is the ~190 ms fixed floor in rep.orchestrator.step().
# With FullRasterization this is a no-op; with RaytracingLighting it saves ~50-80 ms.
import carb as _carb
_rs = _carb.settings.get_settings()
_rs.set("/rtx/post/dlss/execMode", "0")   # 0 = DLSS off
_rs.set("/rtx/denoise/enabled", False)

import omni.usd
from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdPhysics, UsdShade, Vt
import omni.replicator.core as rep

stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)

# Print and remove any default prims Isaac Sim adds (ground planes, default lights)
print("[STAGE] Prims at startup:")
for p in stage.Traverse():
    print(f"  {p.GetPath()}  ({p.GetTypeName()})")
for default_path in ["/World/defaultGroundPlane", "/groundPlane", "/World/GroundPlane",
                     "/World/groundPlane", "/Environment", "/World/Environment"]:
    prim = stage.GetPrimAtPath(default_path)
    if prim.IsValid():
        stage.RemovePrim(default_path)
        print(f"[STAGE] Removed: {default_path}")

# ── USD helpers ────────────────────────────────────────────────────────────────
def bind(prim, mat):
    UsdShade.MaterialBindingAPI(prim).Bind(mat)

def pbr_mat(path, rgb, metallic=0.0, roughness=0.7):
    mat = UsdShade.Material.Define(stage, path)
    sh  = UsdShade.Shader.Define(stage, path + "/S")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
    sh.CreateInput("metallic",     Sdf.ValueTypeNames.Float).Set(metallic)
    sh.CreateInput("roughness",    Sdf.ValueTypeNames.Float).Set(roughness)
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    return mat

_used = set()
def upath(base):
    p, i = base, 0
    while p in _used: i += 1; p = f"{base}_{i}"
    _used.add(p); return p

def _box(path, w, l, h, tx=0.0, ty=0.0, tz=0.0, mat=None):
    """Axis-aligned box: w=X width, l=Y length, h=Z height, center at (tx,ty,tz)."""
    b = UsdGeom.Cube.Define(stage, path)
    b.CreateSizeAttr(1.0)
    xf = UsdGeom.Xformable(b)
    xf.AddTranslateOp().Set(Gf.Vec3d(tx, ty, tz))
    xf.AddScaleOp().Set(Gf.Vec3f(w, l, h))
    if mat:
        bind(b.GetPrim(), mat)
    return b

def _cyl(path, r, h, axis="Z", tx=0.0, ty=0.0, tz=0.0, mat=None):
    """Cylinder with given radius, height, and long-axis direction."""
    c = UsdGeom.Cylinder.Define(stage, path)
    c.CreateRadiusAttr(r)
    c.CreateHeightAttr(h)
    c.CreateAxisAttr(axis)
    UsdGeom.Xformable(c).AddTranslateOp().Set(Gf.Vec3d(tx, ty, tz))
    if mat:
        bind(c.GetPrim(), mat)
    return c


def make_car(root_path, car_lat, car_lon, yaw_deg=0.0):
    """
    Build a white sedan (Toyota Altis class) from USD primitives.

    Coordinate convention (local frame):
        +Y = car front,  +X = car right,  +Z = up
        root placed at (car_x, car_y, terrain_elev_at(car_x, car_y)) so wheels touch terrain.

    yaw_deg: compass heading of front (0=N, 90=E, 180=S, 270=W).
    In this scene X=East, Y=North, Z=Up; front=+Y at yaw=0 → RotateZ(0) is north.
    Compass to USD RotateZ: usd_rot = -(compass - 0) since rotating CCW from +Y to +X
    is the same as +yaw in compass. Actually: RotateZ(θ) rotates +X toward +Y by θ.
    Front is +Y; compass N=0 wants front=+Y → RotateZ(0). Compass E=90 wants front=+X
    → RotateZ(-90). So usd_rot = -yaw_deg (compass).

    Approximate dimensions (Toyota Altis):
        Length 4.64 m, Width 1.775 m, Height 1.45 m, Wheelbase 2.70 m
    """
    car_x, car_y = to_xy(car_lat, car_lon)
    car_z = terrain_elev_at(car_x, car_y)  # terrain height at this car's position

    # ── Materials ─────────────────────────────────────────────────────────────
    P = root_path
    m_body  = pbr_mat(P+"/M_Body",  (0.92, 0.92, 0.90), metallic=0.5,  roughness=0.25)
    m_glass = pbr_mat(P+"/M_Glass", (0.04, 0.06, 0.16), metallic=0.0,  roughness=0.08)
    m_tire  = pbr_mat(P+"/M_Tire",  (0.07, 0.07, 0.07), metallic=0.0,  roughness=0.95)
    m_rim   = pbr_mat(P+"/M_Rim",   (0.72, 0.73, 0.78), metallic=0.90, roughness=0.20)
    m_tail  = pbr_mat(P+"/M_Tail",  (0.88, 0.06, 0.04), metallic=0.1,  roughness=0.25)
    m_head  = pbr_mat(P+"/M_Head",  (0.95, 0.93, 0.82), metallic=0.2,  roughness=0.12)
    m_logo  = pbr_mat(P+"/M_Logo",  (0.06, 0.09, 0.22), metallic=0.0,  roughness=0.55)
    m_under = pbr_mat(P+"/M_Under", (0.10, 0.10, 0.10), metallic=0.0,  roughness=0.90)

    # ── Root xform ────────────────────────────────────────────────────────────
    car = UsdGeom.Xform.Define(stage, root_path)
    xf  = UsdGeom.Xformable(car)
    xf.AddTranslateOp().Set(Gf.Vec3d(car_x, car_y, car_z))
    xf.AddRotateZOp().Set(float(-yaw_deg))   # compass → USD

    # ── Body ──────────────────────────────────────────────────────────────────
    #                                W      L      H      cx    cy     cz
    _box(P+"/Under",    1.50,  4.00,  0.18,   0.0,  0.0,   0.09, m_under)  # chassis floor
    _box(P+"/Body",     1.775, 4.40,  0.64,   0.0,  0.0,   0.49, m_body)   # lower body / doors
    _box(P+"/Cabin",    1.62,  2.20,  0.62,   0.0, -0.15,  1.12, m_body)   # greenhouse / roof
    _box(P+"/Hood",     1.72,  1.10,  0.04,   0.0,  1.62,  0.83, m_body)   # hood top surface
    _box(P+"/Trunk",    1.72,  0.82,  0.04,   0.0, -1.74,  0.80, m_body)   # trunk lid
    _box(P+"/BumpF",    1.76,  0.12,  0.44,   0.0,  2.26,  0.25, m_body)   # front bumper
    _box(P+"/BumpR",    1.76,  0.12,  0.38,   0.0, -2.26,  0.22, m_body)   # rear bumper

    # ── Windows (dark tinted glass) ───────────────────────────────────────────
    _box(P+"/WinF",  0.05, 1.50, 0.48,   0.0,  1.00,  1.11, m_glass)  # windshield
    _box(P+"/WinR",  0.05, 1.48, 0.38,   0.0, -0.88,  1.10, m_glass)  # rear window
    _box(P+"/WinL",  1.58, 0.05, 0.34,  -0.81,-0.15,  1.10, m_glass)  # left side
    _box(P+"/WinRt", 1.58, 0.05, 0.34,   0.81,-0.15,  1.10, m_glass)  # right side

    # ── Lights ────────────────────────────────────────────────────────────────
    _box(P+"/HdL",   0.34, 0.08, 0.18,  -0.60, 2.25, 0.68, m_head)   # headlight L
    _box(P+"/HdR",   0.34, 0.08, 0.18,   0.60, 2.25, 0.68, m_head)   # headlight R
    _box(P+"/TlL",   0.44, 0.07, 0.16,  -0.55,-2.25, 0.66, m_tail)   # taillight L
    _box(P+"/TlR",   0.44, 0.07, 0.16,   0.55,-2.25, 0.66, m_tail)   # taillight R

    # ── Wheels — axis=X so the disc lies in the Y-Z plane; front is +Y ───────
    WR = 0.32   # tire radius
    WT = 0.22   # tire width
    RR = 0.20   # rim radius
    RT = 0.18   # rim width
    WX = 0.88   # lateral offset from car centreline (half-track ≈ 1.52 m)
    for sx, fy, tag in [(-WX, 1.35, "FL"), (WX, 1.35, "FR"),
                         (-WX,-1.35, "RL"), (WX,-1.35, "RR")]:
        _cyl(P+f"/Tire{tag}", WR, WT, "X", sx, fy, WR, m_tire)
        _cyl(P+f"/Rim{tag}",  RR, RT, "X", sx, fy, WR, m_rim)

    # ── Contest logo panel on roof centre ─────────────────────────────────────
    # Represents the UAV Defense Challenge logo placed on the roof for identification.
    _box(P+"/Logo", 0.40, 0.40, 0.02,  0.0, -0.15, 1.45, m_logo)

    print(f"[CAR] {root_path}  ({car_lat:.6f}N, {car_lon:.6f}E)"
          f"  ENU=({car_x:.1f}, {car_y:.1f}) m  yaw={yaw_deg}°")

# ── CESIUM ION ENDPOINT ────────────────────────────────────────────────────────
def fetch_ion_endpoint(asset_id: int):
    """Return (tileset_url, access_token, base_url) for a Cesium ion asset."""
    resp = requests.get(
        f"https://api.cesium.com/v1/assets/{asset_id}/endpoint",
        headers={"Authorization": f"Bearer {CESIUM_ION_TOKEN}"},
        timeout=30,
    )
    resp.raise_for_status()
    ep = resp.json()
    url  = ep["url"]
    base = url.rsplit("/", 1)[0] + "/"
    return url, ep["accessToken"], base

# ── SATELLITE IMAGERY (Taiwan NLSC PHOTO2 WMTS) ───────────────────────────────
# Free, no API key required. URL: https://wmts.nlsc.gov.tw/wmts/PHOTO2/default/GoogleMapsCompatible/{z}/{y}/{x}
SAT_ZOOM  = 18   # ~0.60 m/px effective (mosaic downsampled to MAX_TEX=16384)
SAT_CACHE = os.path.join(HERE, "satellite_ground.jpg")
NLSC_TILE_URL = "https://wmts.nlsc.gov.tw/wmts/PHOTO2/default/GoogleMapsCompatible/{z}/{y}/{x}"

def _deg2tile(lat, lon, z):
    n  = 1 << z
    x  = int((lon + 180.0) / 360.0 * n)
    lr = math.log(math.tan(math.radians(lat)) + 1.0 / math.cos(math.radians(lat)))
    y  = int((1.0 - lr / math.pi) / 2.0 * n)
    return x, y

def _tile2deg(tx, ty, z):
    n   = 1 << z
    lon = tx / n * 360.0 - 180.0
    lat = math.degrees(math.atan(math.sinh(math.pi * (1 - 2 * ty / n))))
    return lat, lon

def fetch_satellite(margin_factor=1.5):
    """Download NLSC PHOTO2 tiles for the scene area."""
    d_lat = RADIUS_M * margin_factor / 111_320.0
    d_lon = RADIUS_M * margin_factor / (111_320.0 * COS_LAT)
    tx_min, ty_min = _deg2tile(CENTER_LAT + d_lat, CENTER_LON - d_lon, SAT_ZOOM)
    tx_max, ty_max = _deg2tile(CENTER_LAT - d_lat, CENTER_LON + d_lon, SAT_ZOOM)
    nw_lat, nw_lon = _tile2deg(tx_min,     ty_min,     SAT_ZOOM)
    se_lat, se_lon = _tile2deg(tx_max + 1, ty_max + 1, SAT_ZOOM)

    bounds = dict(nw_lat=nw_lat, nw_lon=nw_lon, se_lat=se_lat, se_lon=se_lon)
    if os.path.exists(SAT_CACHE):
        print(f"[SAT] Using cached {SAT_CACHE}")
        return SAT_CACHE, bounds

    nx = tx_max - tx_min + 1; ny = ty_max - ty_min + 1
    print(f"[SAT] Downloading {nx}×{ny} NLSC PHOTO2 tiles at zoom {SAT_ZOOM} …")
    TILE   = 256
    mosaic = Image.new("RGB", (nx * TILE, ny * TILE))
    sess   = requests.Session()
    sess.headers.update({"User-Agent": "IsaacSimCesium/1.0"})
    for tx in range(tx_min, tx_max + 1):
        for ty in range(ty_min, ty_max + 1):
            url = NLSC_TILE_URL.format(z=SAT_ZOOM, y=ty, x=tx)
            for attempt in range(3):
                try:
                    r = sess.get(url, timeout=15)
                    r.raise_for_status()
                    tile = Image.open(_io.BytesIO(r.content)).convert("RGB")
                    mosaic.paste(tile, ((tx - tx_min) * TILE, (ty - ty_min) * TILE))
                    break
                except Exception as e:
                    if attempt == 2: print(f"  [SAT] skip {tx},{ty}: {e}")
                    else: time.sleep(0.5)
            time.sleep(0.03)
    # RTX renderer caps usable texture size at ~8192; resize if larger
    MAX_TEX = 16384
    if mosaic.width > MAX_TEX or mosaic.height > MAX_TEX:
        scale = MAX_TEX / max(mosaic.width, mosaic.height)
        new_w, new_h = int(mosaic.width * scale), int(mosaic.height * scale)
        mosaic = mosaic.resize((new_w, new_h), Image.LANCZOS)
        print(f"[SAT] Resized to {new_w}×{new_h} for GPU texture limit")
    mosaic.save(SAT_CACHE, "JPEG", quality=92)
    print(f"[SAT] Saved {mosaic.width}×{mosaic.height} → {SAT_CACHE}")
    return SAT_CACHE, bounds

sat_path, sat_bounds = fetch_satellite()
SAT_NW_LAT = sat_bounds["nw_lat"]; SAT_NW_LON = sat_bounds["nw_lon"]
SAT_SE_LAT = sat_bounds["se_lat"]; SAT_SE_LON = sat_bounds["se_lon"]

def geo_to_uv(lon_arr, lat_arr):
    """Map lon/lat arrays to texture UV within the satellite image bounds.
    USD UsdUVTexture uses OpenGL convention: v=0 = bottom of image.
    Our JPEG has north at top, so v must be flipped (1 - ...) so north → v=1 (top).
    """
    u = (lon_arr - SAT_NW_LON) / (SAT_SE_LON - SAT_NW_LON)
    v = 1.0 - (SAT_NW_LAT - lat_arr) / (SAT_NW_LAT - SAT_SE_LAT)
    return np.clip(u, 0, 1), np.clip(v, 0, 1)

# ── CESIUM WORLD TERRAIN ───────────────────────────────────────────────────────
TERRAIN_LEVEL = 13      # each tile ≈ 2.4 km;  3×3 grid → 7.2 km coverage
TERRAIN_CACHE_DIR = os.path.join(HERE, "cesium_terrain_cache")
os.makedirs(TERRAIN_CACHE_DIR, exist_ok=True)
TERRAIN_TILE_LIST = os.path.join(HERE, "cesium_terrain_list.json")

def _tms_tile_bounds(z, x, y):
    """Geographic bounds of a TMS tile (y=0 at south)."""
    nx = 1 << (z + 1); ny = 1 << z
    west  =  x      / nx * 360.0 - 180.0
    east  = (x + 1) / nx * 360.0 - 180.0
    south =  y      / ny * 180.0 -  90.0
    north = (y + 1) / ny * 180.0 -  90.0
    return west, south, east, north

def compute_terrain_tile_coords(level):
    """Return list of (x, y_tms) covering the scene area with a small margin."""
    nx = 1 << (level + 1)
    ny = 1 << level
    margin = RADIUS_M * 1.3
    d_lat  = margin / 111_320.0
    d_lon  = margin / (111_320.0 * COS_LAT)
    x_min = int((CENTER_LON - d_lon + 180) / 360 * nx)
    x_max = int((CENTER_LON + d_lon + 180) / 360 * nx)
    # TMS y: south = 0, north = ny-1
    y_min = int((CENTER_LAT - d_lat + 90) / 180 * ny)
    y_max = int((CENTER_LAT + d_lat + 90) / 180 * ny)
    coords = [(x, y) for x in range(x_min, x_max + 1)
                      for y in range(y_min, y_max + 1)]
    return coords

def zigzag_decode(n: np.ndarray) -> np.ndarray:
    return (n >> 1).astype(np.int32) ^ -(n.astype(np.int32) & 1)

def parse_quantized_mesh(data: bytes, west, south, east, north):
    """
    Parse Cesium quantized-mesh-1.0.
    Returns (verts_enu Nx3, faces Mx3, lons Nx1, lats Nx1) or None on failure.
    Vertex lat/lon returned for UV mapping.
    """
    if len(data) < 88:
        return None
    # Header: bytes 24-31 = minHeight, maxHeight (float32)
    min_h, max_h = struct.unpack_from("<ff", data, 24)
    off = 88

    vertex_count = struct.unpack_from("<I", data, off)[0]; off += 4
    if vertex_count == 0:
        return None

    u_raw = np.frombuffer(data, dtype="<u2", count=vertex_count, offset=off).copy(); off += vertex_count * 2
    v_raw = np.frombuffer(data, dtype="<u2", count=vertex_count, offset=off).copy(); off += vertex_count * 2
    h_raw = np.frombuffer(data, dtype="<u2", count=vertex_count, offset=off).copy(); off += vertex_count * 2

    # Decode: zigzag then prefix-sum (delta encoding)
    u = np.cumsum(zigzag_decode(u_raw)).clip(0, 32767).astype(np.float64)
    v = np.cumsum(zigzag_decode(v_raw)).clip(0, 32767).astype(np.float64)
    h = np.cumsum(zigzag_decode(h_raw)).clip(0, 32767).astype(np.float64)

    lon    = west  + u / 32767.0 * (east  - west)
    lat    = south + v / 32767.0 * (north - south)
    height = min_h + h / 32767.0 * (max_h - min_h)

    triangle_count = struct.unpack_from("<I", data, off)[0]; off += 4
    if triangle_count == 0:
        return None

    idx_size  = 4 if vertex_count > 65536 else 2
    idx_dtype = "<u4" if vertex_count > 65536 else "<u2"
    raw_idx   = np.frombuffer(data, dtype=idx_dtype,
                               count=triangle_count * 3, offset=off).astype(np.int32)

    # High-watermark index decode
    high = 0
    indices = np.empty(len(raw_idx), dtype=np.int32)
    for i, code in enumerate(raw_idx.tolist()):
        indices[i] = high - code
        if code == 0:
            high += 1
    faces = indices.reshape(-1, 3)

    # ENU
    x_enu = np.radians(lon - CENTER_LON) * R_EARTH * COS_LAT
    y_enu = np.radians(lat - CENTER_LAT) * R_EARTH
    verts_enu = np.column_stack([x_enu, y_enu, height])

    return verts_enu, faces, lon, lat

def fetch_terrain_tiles():
    """Return list of dicts: {url, x, y_tms, bounds, cache_path}."""
    if os.path.exists(TERRAIN_TILE_LIST):
        with open(TERRAIN_TILE_LIST) as f:
            cached = json.load(f)
        print(f"[TERRAIN] Using cached tile list ({len(cached['tiles'])} tiles)")
        return cached["tiles"], None   # token fetched separately

    print("[TERRAIN] Fetching Cesium World Terrain endpoint …")
    tileset_url, access_token, base_url = fetch_ion_endpoint(CESIUM_TERRAIN_ASSET)

    # layer.json tells us the tile URL template
    layer_url = base_url + "layer.json"
    r = requests.get(layer_url, params={"access_token": access_token}, timeout=30)
    if r.ok:
        layer = r.json()
        tile_template = layer.get("tiles", ["{z}/{x}/{y}.terrain"])[0]
        # Resolve relative URLs (e.g. "{z}/{x}/{y}.terrain?v={version}")
        if tile_template.startswith("//"):
            tile_template = "https:" + tile_template
        elif not tile_template.startswith("http"):
            tile_template = base_url + tile_template
        # Replace {version} with the version in the base URL path (e.g. "v1.2.0")
        tile_template = tile_template.replace("{version}", "1.2.0")
    else:
        tile_template = base_url + "{z}/{x}/{y}.terrain"

    coords = compute_terrain_tile_coords(TERRAIN_LEVEL)
    tiles  = []
    for x, y_tms in coords:
        west, south, east, north = _tms_tile_bounds(TERRAIN_LEVEL, x, y_tms)
        url = (tile_template
               .replace("{z}", str(TERRAIN_LEVEL))
               .replace("{x}", str(x))
               .replace("{y}", str(y_tms)))
        cache_key = f"{TERRAIN_LEVEL}_{x}_{y_tms}.terrain"
        tiles.append({"url": url, "x": x, "y_tms": y_tms,
                      "bounds": [west, south, east, north],
                      "cache": cache_key})
    with open(TERRAIN_TILE_LIST, "w") as f:
        json.dump({"tiles": tiles}, f)
    print(f"[TERRAIN] Computed {len(tiles)} tile URLs (level {TERRAIN_LEVEL})")
    return tiles, access_token

def download_terrain_tile(tile_info: dict, access_token: str):
    cache_p = os.path.join(TERRAIN_CACHE_DIR, tile_info["cache"])
    if os.path.exists(cache_p):
        with open(cache_p, "rb") as f:
            return f.read()
    url = tile_info["url"]
    r   = requests.get(url, params={"access_token": access_token},
                        timeout=60, headers={"Accept": "application/vnd.quantized-mesh"})
    if r.status_code == 404:
        return None
    r.raise_for_status()
    with open(cache_p, "wb") as f:
        f.write(r.content)
    return r.content

# ── USD TERRAIN MESH with satellite texture ────────────────────────────────────
def make_terrain_mesh(prim_path, verts_enu, faces, lons, lats, img_path):
    u_tex, v_tex = geo_to_uv(lons, lats)
    uvs  = Vt.Vec2fArray([Gf.Vec2f(float(u), float(v)) for u, v in zip(u_tex, v_tex)])
    pts  = Vt.Vec3fArray([Gf.Vec3f(float(v[0]), float(v[1]), float(v[2])) for v in verts_enu])
    flat_idx = [int(i) for f in faces for i in f]

    mesh = UsdGeom.Mesh.Define(stage, prim_path)
    mesh.CreatePointsAttr(pts)
    mesh.CreateFaceVertexCountsAttr([3] * len(faces))
    mesh.CreateFaceVertexIndicesAttr(flat_idx)
    mesh.CreateSubdivisionSchemeAttr("none")
    st_pv = UsdGeom.PrimvarsAPI(mesh.GetPrim()).CreatePrimvar(
        "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex)
    st_pv.Set(uvs)
    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())

    mp  = prim_path + "/SatMat"
    mat = UsdShade.Material.Define(stage, mp)
    pbr = UsdShade.Shader.Define(stage, mp + "/PBR")
    pbr.CreateIdAttr("UsdPreviewSurface")
    pbr.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.90)
    pbr.CreateInput("metallic",  Sdf.ValueTypeNames.Float).Set(0.0)
    tex = UsdShade.Shader.Define(stage, mp + "/Tex")
    tex.CreateIdAttr("UsdUVTexture")
    tex.CreateInput("file",             Sdf.ValueTypeNames.Asset).Set(img_path)
    tex.CreateInput("wrapS",            Sdf.ValueTypeNames.Token).Set("clamp")
    tex.CreateInput("wrapT",            Sdf.ValueTypeNames.Token).Set("clamp")
    tex.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("sRGB")
    st_r = UsdShade.Shader.Define(stage, mp + "/STReader")
    st_r.CreateIdAttr("UsdPrimvarReader_float2")
    st_r.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(
        st_r.ConnectableAPI(), "result")
    pbr.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(
        tex.ConnectableAPI(), "rgb")
    mat.CreateSurfaceOutput().ConnectToSource(pbr.ConnectableAPI(), "surface")
    bind(mesh.GetPrim(), mat)
    return mesh

# ── CESIUM OSM BUILDINGS ───────────────────────────────────────────────────────
BUILDING_LEVEL    = 12
BUILDING_TILE_DIR = os.path.join(HERE, "cesium_tile_cache")
BUILDING_TILE_LIST = os.path.join(HERE, "cesium_tile_list.json")
os.makedirs(BUILDING_TILE_DIR, exist_ok=True)

# Building type → material colour
BLD_TYPE_MAT = {
    "hospital": (0.85, 0.30, 0.30),   # red
    "clinic":   (0.85, 0.30, 0.30),
    "school":   (0.90, 0.65, 0.15),   # amber
    "college":  (0.90, 0.65, 0.15),
    "university": (0.90, 0.65, 0.15),
    "kindergarten": (0.90, 0.65, 0.15),
    "church":   (0.68, 0.38, 0.88),   # purple
    "temple":   (0.68, 0.38, 0.88),
    "mosque":   (0.68, 0.38, 0.88),
    "shrine":   (0.68, 0.38, 0.88),
    "retail":   (0.88, 0.82, 0.18),   # yellow
    "shop":     (0.88, 0.82, 0.18),
    "commercial": (0.30, 0.58, 0.88), # steel-blue
    "office":   (0.30, 0.58, 0.88),
    "hotel":    (0.30, 0.58, 0.88),
    "government": (0.30, 0.58, 0.88),
    "industrial": (0.55, 0.50, 0.42), # grey-brown
    "warehouse": (0.55, 0.50, 0.42),
    "factory":  (0.55, 0.50, 0.42),
    "apartments": (0.60, 0.60, 0.68), # blue-grey
    "dormitory": (0.60, 0.60, 0.68),
    "house":    (0.80, 0.72, 0.58),   # cream
    "residential": (0.80, 0.72, 0.58),
    "yes":      (0.75, 0.73, 0.70),   # light stone
}
_DEFAULT_BLD_RGB = (0.75, 0.73, 0.70)

_bld_mats = {}
def bld_mat(btype):
    if btype not in _bld_mats:
        rgb = BLD_TYPE_MAT.get(btype, _DEFAULT_BLD_RGB)
        _bld_mats[btype] = pbr_mat(f"/Mat/Bld_{btype}", rgb, roughness=0.80)
    return _bld_mats[btype]

def _read_accessor(accs, bvs, bin_data, idx, dtype, nc):
    acc = accs[idx]; bv = bvs[acc["bufferView"]]
    stride = bv.get("byteStride", nc * np.dtype(dtype).itemsize)
    off    = bv["byteOffset"] + acc.get("byteOffset", 0)
    n      = acc["count"]; sz = np.dtype(dtype).itemsize
    if stride == nc * sz:
        return np.frombuffer(bin_data[off: off + n * stride],
                             dtype=dtype).reshape(n, nc)
    return np.vstack([np.frombuffer(
        bin_data[off + i * stride: off + i * stride + nc * sz], dtype=dtype)
        for i in range(n)])

def parse_b3dm_buildings(data: bytes):
    """Parse B3DM tile → list of {verts_enu, faces, type}."""
    if len(data) < 28 or data[:4] != b"b3dm":
        return []
    ft_jl, ft_bl, bt_jl, bt_bl = struct.unpack_from("<IIII", data, 12)
    bt_raw = data[28 + ft_jl + ft_bl: 28 + ft_jl + ft_bl + bt_jl]
    glb    = data[28 + ft_jl + ft_bl + bt_jl + bt_bl:]
    if len(glb) < 12 or glb[:4] != b"glTF":
        return []

    # Batch-table hierarchy → _BATCHID → building type
    bid_to_type = {}
    if bt_jl > 0:
        try:
            hier = json.loads(bt_raw).get("extensions", {}).get(
                "3DTILES_batch_table_hierarchy", {})
            cls_map   = {c["name"]: c for c in hier.get("classes", [])}
            class_ids = hier.get("classIds", [])
            parent_ids = hier.get("parentIds", [])
            bld_inst   = cls_map.get("Building", {}).get("instances", {})
            bld_types  = bld_inst.get("building", [])
            bi = 0
            for i, cid in enumerate(class_ids):
                if cid == 0:
                    bid_to_type[i] = bld_types[bi] if bi < len(bld_types) else "yes"
                    bi += 1
                elif cid == 2:
                    p = parent_ids[i] if i < len(parent_ids) else -1
                    bid_to_type[i] = bid_to_type.get(p, "yes")
        except Exception:
            pass

    json_len = struct.unpack_from("<I", glb, 12)[0]
    gltf     = json.loads(glb[20: 20 + json_len])
    bin_off  = 20 + json_len
    if bin_off % 4: bin_off += 4 - (bin_off % 4)
    bin_data = bytes(glb[bin_off + 8: bin_off + 8 + struct.unpack_from("<I", glb, bin_off)[0]])
    bvs  = gltf.get("bufferViews", [])
    accs = gltf.get("accessors",   [])

    node_mat = np.eye(4)
    for node in gltf.get("nodes", []):
        if "mesh" in node and "matrix" in node:
            node_mat = np.array(node["matrix"], dtype=np.float64).reshape(4, 4).T
            break

    all_ecef = []; all_bids = []; all_faces = []; voff = 0
    for mesh in gltf.get("meshes", []):
        for prim in mesh.get("primitives", []):
            attrs   = prim.get("attributes", {})
            pos_idx = attrs.get("POSITION"); bid_idx = attrs.get("_BATCHID")
            tri_idx = prim.get("indices")
            if pos_idx is None: continue
            nv  = accs[pos_idx]["count"]
            raw = _read_accessor(accs, bvs, bin_data, pos_idx, np.float32, 3).astype(np.float64)
            gw  = (node_mat @ np.hstack([raw, np.ones((nv, 1))]).T).T[:, :3]
            # glTF Y-up → ECEF Z-up
            all_ecef.append(np.column_stack([gw[:, 0], -gw[:, 2], gw[:, 1]]))
            bids = (_read_accessor(accs, bvs, bin_data, bid_idx, np.float32, 1)
                    .flatten().astype(np.int32)
                    if bid_idx is not None else np.zeros(nv, np.int32))
            all_bids.append(bids)
            if tri_idx is not None:
                fmt = np.uint16 if accs[tri_idx]["componentType"] == 5123 else np.uint32
                ia  = _read_accessor(accs, bvs, bin_data, tri_idx, fmt, 1).flatten().astype(np.int32)
                all_faces.append(ia.reshape(-1, 3) + voff)
            voff += nv

    if not all_ecef:
        return []
    verts_ecef = np.vstack(all_ecef)
    batch_ids  = np.concatenate(all_bids)
    faces_all  = np.vstack(all_faces) if all_faces else np.empty((0, 3), np.int32)

    lat_d, lon_d, alt_m = ecef_to_geodetic_vec(verts_ecef)
    x_enu = np.radians(lon_d - CENTER_LON) * R_EARTH * COS_LAT
    y_enu = np.radians(lat_d - CENTER_LAT) * R_EARTH
    verts_enu = np.column_stack([x_enu, y_enu, alt_m])

    buildings = []
    for bid in np.unique(batch_ids):
        vm = batch_ids == bid; vi = np.where(vm)[0]
        v_remap = np.full(len(verts_enu), -1, np.int32)
        v_remap[vi] = np.arange(len(vi), dtype=np.int32)
        fm = np.all(vm[faces_all], axis=1)
        if not fm.any(): continue
        buildings.append({
            "verts_enu": verts_enu[vi],
            "faces":     v_remap[faces_all[fm]],
            "type":      bid_to_type.get(int(bid), "yes"),
        })
    return buildings

def fetch_building_tiles():
    """Return (tile_urls, fresh_access_token). Tile URLs cached locally."""
    print("[CESIUM] Fetching OSM Buildings endpoint …")
    tileset_url, access_token, base_url = fetch_ion_endpoint(CESIUM_BUILDINGS_ASSET)
    if os.path.exists(BUILDING_TILE_LIST):
        with open(BUILDING_TILE_LIST) as f:
            tiles = json.load(f)["tiles"]
        print(f"[CESIUM] Using cached building tile list ({len(tiles)} tiles)")
        return tiles, access_token

    level = BUILDING_LEVEL
    nx = 1 << (level + 1); ny = 1 << level
    margin = RADIUS_M * 1.3
    d_lat  = margin / 111_320.0
    d_lon  = margin / (111_320.0 * COS_LAT)
    x_min = int((CENTER_LON - d_lon + 180) / 360 * nx)
    x_max = int((CENTER_LON + d_lon + 180) / 360 * nx)
    # CWT: y=0 at north
    y_min = int((90 - (CENTER_LAT + d_lat)) / 180 * ny)
    y_max = int((90 - (CENTER_LAT - d_lat)) / 180 * ny)
    tiles = [f"{base_url}{level}/{x}/{y}.b3dm"
             for x in range(x_min, x_max + 1)
             for y in range(y_min, y_max + 1)]
    print(f"[CESIUM] Computed {len(tiles)} building tile URLs (level {level})")
    with open(BUILDING_TILE_LIST, "w") as f:
        json.dump({"tiles": tiles}, f)
    return tiles, access_token

def download_tile(url, access_token, cache_dir, cache_key):
    cache_p = os.path.join(cache_dir, cache_key)
    if os.path.exists(cache_p):
        with open(cache_p, "rb") as f: return f.read()
    r = requests.get(url, params={"access_token": access_token}, timeout=60)
    if r.status_code == 404: return None
    r.raise_for_status()
    with open(cache_p, "wb") as f: f.write(r.content)
    return r.content

# ── LIGHTS ─────────────────────────────────────────────────────────────────────
sky = UsdLux.DomeLight.Define(stage, "/World/Lights/Sky")
sky.CreateIntensityAttr(200)
sky.CreateColorAttr(Gf.Vec3f(0.52, 0.68, 1.0))
sun = UsdLux.DistantLight.Define(stage, "/World/Lights/Sun")
sun.CreateIntensityAttr(2500)
sun.CreateColorAttr(Gf.Vec3f(1.0, 0.97, 0.87))
UsdGeom.Xformable(sun).AddRotateXYZOp().Set(Gf.Vec3d(-48.0, 0.0, 35.0))

# Keep auto-exposure but clamp range so bright outdoor scenes don't blow out to white
import carb.settings
_rs = carb.settings.get_settings()
_rs.set("/rtx/post/histogram/enabled",     True)
_rs.set("/rtx/post/histogram/exposureMin", -4.0)   # EV stops
_rs.set("/rtx/post/histogram/exposureMax",  0.0)   # never boost above 0 EV
_rs.set("/rtx/post/tonemap/op",             6)     # ACES filmic
_rs.set("/rtx/post/tonemap/whitePoint",     1.0)

# ── LOAD CESIUM WORLD TERRAIN ─────────────────────────────────────────────────
print("[TERRAIN] Loading Cesium World Terrain …")
terrain_tiles, terrain_token = fetch_terrain_tiles()

# If tile list was cached we need a fresh token
if terrain_token is None:
    _, terrain_token, _ = fetch_ion_endpoint(CESIUM_TERRAIN_ASSET)

n_terrain_tiles     = 0
centre_elev         = 0.0   # elevation at scene origin, used for camera height
_all_terrain_verts  = []    # accumulated for per-position elevation lookup

for tile_info in terrain_tiles:
    bounds = tile_info["bounds"]   # [west, south, east, north]
    data   = download_terrain_tile(tile_info, terrain_token)
    if data is None:
        continue

    result = parse_quantized_mesh(data, *bounds)
    if result is None:
        continue
    verts_enu, faces, lons, lats = result

    # Record elevation at scene origin (closest vertex to centre)
    if n_terrain_tiles == 0:
        dists = np.hypot(verts_enu[:, 0], verts_enu[:, 1])
        centre_elev = float(verts_enu[dists.argmin(), 2])

    _all_terrain_verts.append(verts_enu)
    prim_path = upath(f"/World/Terrain/T{n_terrain_tiles:04d}")
    make_terrain_mesh(prim_path, verts_enu, faces, lons, lats, sat_path)
    n_terrain_tiles += 1

_terrain_verts_all = (np.vstack(_all_terrain_verts)
                      if _all_terrain_verts else np.zeros((1, 3)))

def terrain_elev_at(x, y):
    """Terrain elevation (m MSL) at ENU (x, y) by nearest-vertex lookup."""
    dists = np.hypot(_terrain_verts_all[:, 0] - x, _terrain_verts_all[:, 1] - y)
    return float(_terrain_verts_all[dists.argmin(), 2])

print(f"[TERRAIN] Loaded {n_terrain_tiles} terrain tiles  ({len(_terrain_verts_all)} vertices)")
print(f"[TERRAIN] centre_elev = {centre_elev:.1f} m MSL  (use this for SITL -l and HOME_ALT_MSL)")

# Write terrain elevation so run_flight.py and SITL can use the real value.
_home_cfg = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         "..", "control", "home_elevation.json")
with open(_home_cfg, "w") as _f:
    json.dump({"centre_elev_m": centre_elev,
               "lat": CENTER_LAT, "lon": CENTER_LON}, _f)

# ── ROS2 publishers — cesium_scene now PUBLISHES /drone/state (no subscription) ──
_ros2_node = None
_img_pub   = None
_pose_pub  = None
_agl_pub   = None
_state_pub = None   # replaces drone_sim.py's /drone/state publisher

def _cb_drone_reset(msg):
    """Reset drone to home position (E=0, N=0, AGL=0) without restarting Isaac Sim."""
    global _kx, _ky, _kz, _kvn, _kve, _kvd, _kroll, _kpitch, _kyaw_rad
    with _kin_lock:
        _kx, _ky    = 0.0, 0.0
        _kz         = float(centre_elev)
        _kvn, _kve, _kvd = 0.0, 0.0, 0.0
        _kroll, _kpitch  = 0.0, 0.0
    print("[DRONE] Reset to home position (0, 0, ground)")

if _ROS2_OK:
    try:
        rclpy.init()
        _ros2_node = rclpy.create_node("isaac_sim_drone")
        _img_pub   = _ros2_node.create_publisher(RosImage,    "/drone/camera/image_raw", 1)
        _pose_pub  = _ros2_node.create_publisher(PoseStamped, "/drone/pose",              1)
        _agl_pub   = _ros2_node.create_publisher(Float64,     "/drone/agl",               1)
        _state_pub = _ros2_node.create_publisher(PoseStamped, "/drone/state",             1)
        from std_msgs.msg import Bool as _BoolMsg
        _ros2_node.create_subscription(_BoolMsg, "/drone/reset", lambda msg: _cb_drone_reset(msg), 1)
        print("[ROS2] Publishers ready: /drone/camera/image_raw, /drone/pose, /drone/agl, /drone/state")
        print("[ROS2] Subscriber ready: /drone/reset (publish any Bool to reset to home)")
    except Exception as _e:
        print(f"[ROS2] Node init failed: {_e} — falling back to file output")
        _ros2_node = None

# ── Background publish thread ─────────────────────────────────────────────────
# Encode + ROS2 serialise + publish happen here so the render loop is never
# blocked by CPU-heavy tobytes() or PIL JPEG encode.
# Queue depth = 1: put_nowait() in the render loop silently drops a frame when
# the worker is still busy — always processes the freshest available frame.
_pub_q: _queue.Queue = _queue.Queue(maxsize=1)

def _publish_worker():
    _first = True
    while True:
        item = _pub_q.get()
        if item is None:
            break
        rgb_arr, lat, lon, alt, agl, yaw_CCW, step = item
        try:
            if DRONE_SAVE_FRAMES:
                Image.fromarray(rgb_arr, "RGB").save(
                    os.path.join(DRONE_FRAME_DIR, "latest.jpg"), "JPEG", quality=85)
                with open(os.path.join(DRONE_FRAME_DIR, "latest_meta.json"), "w") as _mf:
                    json.dump({
                        "step": step, "lat": lat, "lon": lon,
                        "alt_m": alt, "agl_m": agl,
                        "centre_elev": centre_elev,
                        "yaw_deg": math.degrees(yaw_CCW),
                        "frame_w": DRONE_CAM_W, "frame_h": DRONE_CAM_H,
                    }, _mf)
            if _ros2_node is not None:
                _now = _ros2_node.get_clock().now().to_msg()
                img_msg = RosImage()
                img_msg.header.stamp    = _now
                img_msg.header.frame_id = "drone_camera"
                img_msg.height   = DRONE_CAM_H
                img_msg.width    = DRONE_CAM_W
                img_msg.encoding = "rgb8"
                img_msg.step     = DRONE_CAM_W * 3
                img_msg.data     = rgb_arr.flatten().tobytes()
                _img_pub.publish(img_msg)
                _hy = yaw_CCW / 2.0
                pose_msg = PoseStamped()
                pose_msg.header.stamp    = _now
                pose_msg.header.frame_id = "wgs84"
                pose_msg.pose.position.x = lat
                pose_msg.pose.position.y = lon
                pose_msg.pose.position.z = alt
                pose_msg.pose.orientation.z = math.sin(_hy)
                pose_msg.pose.orientation.w = math.cos(_hy)
                _pose_pub.publish(pose_msg)
                agl_msg = Float64()
                agl_msg.data = float(agl)
                _agl_pub.publish(agl_msg)
                if _first:
                    print("[ROS2] First frame published to /drone/camera/image_raw")
                    _first = False
        except Exception as _pub_e:
            print(f"[PUB] Worker error: {_pub_e}")
        finally:
            _pub_q.task_done()

_pub_thread = threading.Thread(target=_publish_worker, name="cam-pub", daemon=True)
_pub_thread.start()

# ── LOAD CESIUM OSM BUILDINGS ─────────────────────────────────────────────────
print("[CESIUM] Loading Cesium OSM Buildings …")
building_tiles, bld_token = fetch_building_tiles()

n_bld = 0
for tile_url in building_tiles:
    parts    = tile_url.split("?")[0].split("/")
    cache_key = "_".join(parts[-3:])
    data      = download_tile(tile_url, bld_token, BUILDING_TILE_DIR, cache_key)
    if data is None:
        continue

    for bld in parse_b3dm_buildings(data):
        verts = bld["verts_enu"]
        faces = bld["faces"]
        cx    = float(verts[:, 0].mean())
        cy    = float(verts[:, 1].mean())
        if math.hypot(cx, cy) > RADIUS_M * 1.5:
            continue

        # Ground building on terrain elevation at its centroid.
        # Terrain elevation ≈ the ellipsoidal altitude of the lowest terrain vertex
        # near the building; we approximate it with the building's own minimum z
        # adjusted for geoid undulation via the closest terrain vertex lookup.
        # Simpler: use the minimum vertex z as the ellipsoidal ground and offset
        # so the floor sits at ≈ centre_elev (works for flat Chiayi plain).
        terrain_est = centre_elev   # flat-plain approximation
        z_off = terrain_est - float(verts[:, 2].min())
        verts = verts.copy(); verts[:, 2] += z_off
        bld_h = float(verts[:, 2].max() - terrain_est)
        if bld_h < 0.5: continue

        mat = bld_mat(bld["type"])
        prim_path = upath(f"/World/Buildings/B{n_bld:05d}")
        pts      = Vt.Vec3fArray([Gf.Vec3f(float(v[0]), float(v[1]), float(v[2])) for v in verts])
        flat_idx = [int(i) for f in faces for i in f]
        mesh = UsdGeom.Mesh.Define(stage, prim_path)
        mesh.CreatePointsAttr(pts)
        mesh.CreateFaceVertexCountsAttr([3] * len(faces))
        mesh.CreateFaceVertexIndicesAttr(flat_idx)
        mesh.CreateSubdivisionSchemeAttr("none")
        bind(mesh.GetPrim(), mat)
        UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
        n_bld += 1

print(f"[CESIUM] Loaded {n_bld} buildings")

# ── TARGET VEHICLES (inside detection zone) ───────────────────────────────────
# Three cars spread across the lawnmower survey zone (~NED: N-48→677, E-1327→-454).
# Positions verified inside the buffered zone polygon.
make_car("/World/Car_01", 23.454017, 120.279272, yaw_deg=45.0)   # N=350 E=-700  strip-1 area
make_car("/World/Car_02", 23.452218, 120.277309, yaw_deg=270.0)  # N=150 E=-900  strip-2 area
make_car("/World/Car_03", 23.454917, 120.275348, yaw_deg=135.0)  # N=450 E=-1100 strip-4 area

# ── VIEWPORT CAMERA ────────────────────────────────────────────────────────────
cam = UsdGeom.Camera.Define(stage, "/World/Camera")
cam.CreateFocalLengthAttr(28.0)
cxf = UsdGeom.Xformable(cam)
cam_z = centre_elev + 800.0
cxf.AddTranslateOp().Set(Gf.Vec3d(0.0, -600.0, cam_z))
cxf.AddRotateXYZOp().Set(Gf.Vec3d(-48.0, 0.0, 0.0))
try:
    from omni.kit.viewport.utility import get_active_viewport
    get_active_viewport().camera_path = "/World/Camera"
except Exception:
    pass

# ── DRONE + DRONE CAMERA ───────────────────────────────────────────────────────
os.makedirs(DRONE_FRAME_DIR, exist_ok=True)

# Root Xform — position and orientation set each frame from the kinematic model.
drone_root      = UsdGeom.Xform.Define(stage, "/World/Drone")
drone_pos_op    = drone_root.AddTranslateOp()
drone_orient_op = drone_root.AddOrientOp(UsdGeom.XformOp.PrecisionDouble)
drone_pos_op.Set(Gf.Vec3d(0.0, 0.0, centre_elev))
drone_orient_op.Set(Gf.Quatd(1, 0, 0, 0))

# Quadcopter: central body + 4 arms + motor pods + propeller discs
# Overall span ≈ 0.8 m (realistic DJI Phantom class)
_drone_dark = pbr_mat("/Mat/DroneMat",  (0.12, 0.12, 0.12), roughness=0.5)
_drone_prop = pbr_mat("/Mat/DroneProp", (0.25, 0.25, 0.28), roughness=0.3)

_qbody = UsdGeom.Cube.Define(stage, "/World/Drone/Body")
_qbody.CreateSizeAttr(1.0)
UsdGeom.Xformable(_qbody).AddScaleOp().Set(Gf.Vec3f(0.28, 0.28, 0.08))
bind(_qbody.GetPrim(), _drone_dark)

_ARM_CR = 0.275   # arm-box centre distance from drone origin (m)
_ARM_TR = 0.40    # motor-pod centre distance from drone origin (m)
for _an, _ad in [("NE", 45), ("NW", 135), ("SW", 225), ("SE", 315)]:
    _r = math.radians(_ad)
    _cx, _cy = _ARM_CR * math.cos(_r), _ARM_CR * math.sin(_r)
    _mx, _my = _ARM_TR * math.cos(_r), _ARM_TR * math.sin(_r)

    # Arm — unit cube: scale first, rotate to direction, translate to position
    _arm = UsdGeom.Cube.Define(stage, f"/World/Drone/Arm_{_an}")
    _arm.CreateSizeAttr(1.0)
    _axf = UsdGeom.Xformable(_arm)
    _axf.AddTranslateOp().Set(Gf.Vec3d(_cx, _cy, 0.0))
    _axf.AddRotateZOp().Set(float(_ad))
    _axf.AddScaleOp().Set(Gf.Vec3f(0.25, 0.05, 0.03))
    bind(_arm.GetPrim(), _drone_dark)

    # Motor pod (upright cylinder)
    _mot = UsdGeom.Cylinder.Define(stage, f"/World/Drone/Motor_{_an}")
    _mot.CreateRadiusAttr(0.035)
    _mot.CreateHeightAttr(0.05)
    _mot.CreateAxisAttr("Z")
    UsdGeom.Xformable(_mot).AddTranslateOp().Set(Gf.Vec3d(_mx, _my, 0.0))
    bind(_mot.GetPrim(), _drone_dark)

    # Propeller disc (flat cylinder above motor)
    _prop = UsdGeom.Cylinder.Define(stage, f"/World/Drone/Prop_{_an}")
    _prop.CreateRadiusAttr(0.13)
    _prop.CreateHeightAttr(0.008)
    _prop.CreateAxisAttr("Z")
    UsdGeom.Xformable(_prop).AddTranslateOp().Set(Gf.Vec3d(_mx, _my, 0.03))
    bind(_prop.GetPrim(), _drone_prop)

# Orange beacon light — renders as a coloured dot from the overview camera
_beacon = UsdLux.SphereLight.Define(stage, "/World/Drone/Beacon")
_beacon.CreateIntensityAttr(5000.0)
_beacon.CreateRadiusAttr(0.05)
_beacon.CreateColorAttr(Gf.Vec3f(1.0, 0.4, 0.0))
UsdGeom.Xformable(_beacon).AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.15))

# Nadir camera with 2-axis gimbal stabilisation.
# The camera is a child of /World/Drone, so it inherits the drone's attitude.
# A gimbal orient op (updated every render frame) cancels roll and pitch while
# preserving yaw, so the camera always looks straight down AND the top of the
# image follows the drone nose direction.
# Math: camera_local = conj(drone_quat) * yaw_only_quat
#       → camera world orient = yaw_only (nadir + heading-aligned).
# The translate offset keeps it 15 cm below the drone centre in drone-local space.
# AP-IMX900-Mini-USB3-I5: EFL 3.1 mm, 113.1°(D)×88°(H)×65.1°(V) — from spec sheet.
# Pinhole aperture: 2×3.1×tan(44°)=5.987 mm H, 2×3.1×tan(32.55°)=3.957 mm V.
# GSD ≈ 75 mm/px @ 80 m AGL.  2048×1536 output.
drone_cam = UsdGeom.Camera.Define(stage, "/World/Drone/Camera")
drone_cam.CreateFocalLengthAttr(3.1)              # AP-IMX900: EFL 3.1 mm
drone_cam.CreateHorizontalApertureAttr(5.987)     # 2×3.1×tan(44°): HFOV=88°
drone_cam.CreateVerticalApertureAttr(3.957)       # 2×3.1×tan(32.55°): VFOV=65.1°
drone_cam.CreateClippingRangeAttr(Gf.Vec2f(0.1, 5000.0))
_dcam_xf = UsdGeom.Xformable(drone_cam)
_dcam_xf.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, -0.15))
drone_cam_orient_op = _dcam_xf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble)
drone_cam_orient_op.Set(Gf.Quatd(1, 0, 0, 0))   # identity until first frame

# Replicator render product for the drone camera
_rp  = rep.create.render_product("/World/Drone/Camera", (DRONE_CAM_W, DRONE_CAM_H))
_rgb = rep.AnnotatorRegistry.get_annotator("rgb")
_rgb.attach([_rp])

print(f"[DRONE] Nadir camera {DRONE_CAM_W}×{DRONE_CAM_H}  →  {DRONE_FRAME_DIR}/")

# ── Kinematic physics constants ────────────────────────────────────────────────
_K_GRAVITY     = 9.81
_K_MAX_VEL     = 15.0
_K_MAX_TILT    = 0.35    # rad — max tilt from PWM differential
_K_TILT_TAU    = 0.15    # s   — attitude first-order time constant (ArduPilot only)
_K_PITCH_ACCEL = 80.0    # rad/s² per unit motor diff × mean_p (PX4 only)
_K_PITCH_DAMP  = 12.0    # angular damping s⁻¹ (PX4 only)
_K_DRAG        = 0.35    # s⁻¹ — aerodynamic drag coefficient

# Kinematic state — ENU (x=East m, y=North m, z=MSL altitude m)
# Protected by _kin_lock; written by physics thread, read by render loop.
_kin_lock                   = threading.Lock()
_kx, _ky                    = 0.0, 0.0
_kz                         = float(centre_elev)
_kvn, _kve, _kvd            = 0.0, 0.0, 0.0
_kroll, _kpitch, _kyaw_rad  = 0.0, 0.0, 0.0
_kpitch_rate, _kroll_rate   = 0.0, 0.0             # angular rates (PX4 only)
_kqx, _kqy, _kqz, _kqw     = 0.0, 0.0, 0.0, 1.0   # quaternion for mesh

# ── SITL bridge ───────────────────────────────────────────────────────────────
if _PX4_SIM:
    _bridge = PX4SimBridge(listen_port=4560, centre_elev=centre_elev)
else:
    _bridge = SITLBridge(listen_port=9002, centre_elev=centre_elev)
    _bridge.debug_hz = 0.2
_latest_pwm = [1000] * 16   # motors off until autopilot connects and arms

# ── Flight trace CSV ──────────────────────────────────────────────────────────
_TRACE_DIR  = os.path.join(HERE, "flight_traces")
os.makedirs(_TRACE_DIR, exist_ok=True)
_trace_path = os.path.join(_TRACE_DIR, f"trace_{time.strftime('%Y%m%d_%H%M%S')}.csv")
_trace_f    = open(_trace_path, "w", newline="", buffering=1)
_trace_csv  = csv.writer(_trace_f)
_trace_csv.writerow(["t_s", "east_m", "north_m", "agl_m", "vn_ms", "ve_ms"])
_trace_start_t  = time.time()
_trace_last_t   = 0.0
print(f"[DRONE] Flight trace → {_trace_path}")

# ── 100 Hz physics thread — decoupled from Isaac Sim render rate (~13 Hz) ────
def _run_physics():
    """Run kinematic model + SITL bridge at 100 Hz in background thread."""
    global _kx, _ky, _kz, _kvn, _kve, _kvd
    global _kroll, _kpitch, _kyaw_rad
    global _kpitch_rate, _kroll_rate
    global _kqx, _kqy, _kqz, _kqw, _latest_pwm
    global _trace_last_t

    _kprev_t_loc = None

    while True:
        t_now = time.time()
        _kdt_loc = min(t_now - _kprev_t_loc, 0.05) if _kprev_t_loc is not None else 0.0
        _kprev_t_loc = t_now

        with _kin_lock:
            kx = _kx;  ky = _ky;  kz = _kz
            kvn = _kvn; kve = _kve; kvd = _kvd
            kroll = _kroll; kpitch = _kpitch; kyaw = _kyaw_rad
            kpitch_rate = _kpitch_rate; kroll_rate = _kroll_rate
            pwm = list(_latest_pwm)

        ret = _bridge.step(kx, ky, kz,
                           math.degrees(-kyaw), kroll, kpitch, t_now)
        if _PX4_SIM:
            # PX4 bridge returns a list of 4 normalised motor outputs [0,1]
            _p4 = [float(v) for v in ret[:4]] if ret else [0.0, 0.0, 0.0, 0.0]
        else:
            if ret is not None:
                pwm = ret["pwm"]
            _p4 = [max(0.0, (v - 1000) / 1000.0) for v in pwm[:4]]

        if _kdt_loc > 0:
            _mean_p  = sum(_p4) / 4.0
            _kthrust = _mean_p * 2.0 * _K_GRAVITY

            if _PX4_SIM:
                # Second-order angular dynamics: motor torque → rate → angle.
                # PX4 none_iris CA_ROTOR: 0=FR(+,+) 1=RL(-,-) 2=FL(+,-) 3=RR(-,+).
                # pitch_diff>0 = front > rear = nose DOWN in FRD = forward/northward.
                # roll_diff>0  = left > right = roll right = eastward.
                pitch_diff = (_p4[0] + _p4[2]) - (_p4[1] + _p4[3])
                roll_diff  = (_p4[1] + _p4[2]) - (_p4[0] + _p4[3])
                new_pitch_rate = kpitch_rate + (_K_PITCH_ACCEL * _mean_p * pitch_diff
                                               - _K_PITCH_DAMP * kpitch_rate) * _kdt_loc
                new_roll_rate  = kroll_rate  + (_K_PITCH_ACCEL * _mean_p * roll_diff
                                               - _K_PITCH_DAMP * kroll_rate)  * _kdt_loc
                new_pitch = max(-_K_MAX_TILT, min(_K_MAX_TILT,
                                                   kpitch + new_pitch_rate * _kdt_loc))
                new_roll  = max(-_K_MAX_TILT, min(_K_MAX_TILT,
                                                   kroll  + new_roll_rate  * _kdt_loc))
            else:
                # ArduCopter QUAD-X: Motor1(FR)→ch1, Motor2(RL)→ch2, Motor3(RR)→ch3, Motor4(FL)→ch4
                # Roll right = (FR+RR)-(RL+FL); Pitch fwd = (FR+FL)-(RL+RR), negative = northward
                _roll_tgt  = ((_p4[0] + _p4[2]) - (_p4[1] + _p4[3])) * _K_MAX_TILT
                _pitch_tgt = ((_p4[0] + _p4[3]) - (_p4[1] + _p4[2])) * _K_MAX_TILT
                _ka        = _kdt_loc / (_K_TILT_TAU + _kdt_loc)
                new_roll   = kroll  + _ka * (_roll_tgt  - kroll)
                new_pitch  = kpitch + _ka * (_pitch_tgt - kpitch)
                new_pitch_rate = 0.0; new_roll_rate = 0.0

            _kcy, _ksy = math.cos(kyaw), math.sin(kyaw)
            _kbfwd = -_kthrust * math.sin(new_pitch)
            _kbrgt =  _kthrust * math.sin(new_roll)
            _kan   = _kbfwd * _kcy - _kbrgt * _ksy
            _kae   = _kbfwd * _ksy + _kbrgt * _kcy
            _kad   = _K_GRAVITY - _kthrust * math.cos(new_roll) * math.cos(new_pitch)

            new_kvn = max(-_K_MAX_VEL, min(_K_MAX_VEL, kvn + _kan * _kdt_loc))
            new_kve = max(-_K_MAX_VEL, min(_K_MAX_VEL, kve + _kae * _kdt_loc))
            new_kvd = max(-_K_MAX_VEL, min(_K_MAX_VEL, kvd + _kad * _kdt_loc))

            _drag = 1.0 - _K_DRAG * _kdt_loc
            new_kvn *= _drag; new_kve *= _drag; new_kvd *= _drag

            new_ky = ky + new_kvn * _kdt_loc
            new_kx = kx + new_kve * _kdt_loc
            new_kz = kz - new_kvd * _kdt_loc

            if new_kz <= centre_elev:
                new_kz  = float(centre_elev)
                new_kvd = min(0.0, new_kvd)
                new_kvn = 0.0; new_kve = 0.0
                new_pitch_rate = 0.0; new_roll_rate = 0.0

            _yaw_CCW_loc = -kyaw
            _cy = math.cos(_yaw_CCW_loc / 2); _sy = math.sin(_yaw_CCW_loc / 2)
            _cr = math.cos(new_roll  / 2);    _sr = math.sin(new_roll  / 2)
            _cp = math.cos(new_pitch / 2);    _sp = math.sin(new_pitch / 2)
            nqx = _sr*_cp*_cy - _cr*_sp*_sy
            nqy = _cr*_sp*_cy + _sr*_cp*_sy
            nqz = _cr*_cp*_sy - _sr*_sp*_cy
            nqw = _cr*_cp*_cy + _sr*_sp*_sy

            # Trace at 5 Hz
            if t_now - _trace_last_t >= 0.2:
                _trace_last_t = t_now
                _trace_csv.writerow([
                    f"{t_now - _trace_start_t:.2f}",
                    f"{new_kx:.3f}", f"{new_ky:.3f}",
                    f"{new_kz - centre_elev:.3f}",
                    f"{new_kvn:.3f}", f"{new_kve:.3f}",
                ])

            with _kin_lock:
                _kx, _ky, _kz            = new_kx, new_ky, new_kz
                _kvn, _kve, _kvd         = new_kvn, new_kve, new_kvd
                _kroll, _kpitch          = new_roll, new_pitch
                _kpitch_rate, _kroll_rate = new_pitch_rate, new_roll_rate
                _latest_pwm              = pwm
                _kqx, _kqy, _kqz, _kqw  = nqx, nqy, nqz, nqw

        # Publish /drone/state at physics rate (100 Hz)
        if _state_pub is not None and _ros2_node is not None:
            _sm = PoseStamped()
            _sm.header.stamp    = _ros2_node.get_clock().now().to_msg()
            _sm.header.frame_id = "local_enu"
            with _kin_lock:
                _sm.pose.position.x    = _kx
                _sm.pose.position.y    = _ky
                _sm.pose.position.z    = _kz
                _sm.pose.orientation.w = _kqw
                _sm.pose.orientation.x = _kqx
                _sm.pose.orientation.y = _kqy
                _sm.pose.orientation.z = _kqz
            _state_pub.publish(_sm)

        elapsed = time.time() - t_now
        time.sleep(max(0.0, 0.01 - elapsed))   # 100 Hz

_physics_thread = threading.Thread(target=_run_physics, daemon=True)
_physics_thread.start()
print("[DRONE] Physics thread started at 100 Hz — drone_sim.py no longer needed")

# Write geo metadata
stage.SetMetadata("customLayerData", {
    "geo:centerLat":   CENTER_LAT,   "geo:centerLon":   CENTER_LON,
    "geo:radiusM":     RADIUS_M,     "geo:metersPerUnit": 1.0,
    "geo:upAxis":      "Z",          "geo:xIsEast":     True,
    "geo:yIsNorth":    True,
    "geo:source":      "Cesium ion (terrain=asset 1, buildings=asset 96188)",
})
with open(os.path.join(HERE, "geo_metadata.json"), "w") as f:
    json.dump({
        "center": {"lat": CENTER_LAT, "lon": CENTER_LON},
        "radius_m": RADIUS_M,
        "data_sources": {
            "terrain":   "Cesium World Terrain (asset 1) quantized-mesh-1.0",
            "imagery":   "Taiwan NLSC PHOTO2 orthophoto zoom-18",
            "buildings": "Cesium OSM Buildings (asset 96188) 3D Tiles B3DM",
        },
    }, f, indent=2)

# ── SIMULATION LOOP ────────────────────────────────────────────────────────────

# ── HUD overlay (omni.ui window pinned to top-left corner) ────────────────────
try:
    import omni.ui as ui
    _hud = ui.Window(
        "DroneHUD", width=400, height=78,
        flags=(ui.WINDOW_FLAGS_NO_TITLE_BAR       |
               ui.WINDOW_FLAGS_NO_RESIZE           |
               ui.WINDOW_FLAGS_NO_SCROLLBAR        |
               ui.WINDOW_FLAGS_NO_FOCUS_ON_APPEARING),
    )
    _hud.position_x = 10
    _hud.position_y = 10
    _LS = {"color": 0xFFFFFFFF, "font_size": 15}   # white text
    with _hud.frame:
        with ui.ZStack():
            ui.Rectangle(style={"background_color": 0xCC000000, "border_radius": 4})
            with ui.VStack(spacing=1):
                ui.Spacer(height=5)
                _lbl_latlon = ui.Label("  LAT --         LON --",  style=_LS)
                _lbl_alt    = ui.Label("  ALT -- m MSL   AGL -- m", style=_LS)
                _lbl_cam    = ui.Label("  CAM  Overview",           style=_LS)
    _hud_ok = True
except Exception as _e:
    print(f"[HUD] omni.ui unavailable: {_e}")
    _hud_ok = False

print(f"[SCENE] {n_terrain_tiles} terrain tiles | {n_bld} Cesium buildings")
print(f"[GEO] Camera: lat={to_latlon(0,-600)[0]:.4f}°N  lon={CENTER_LON:.4f}°E  alt={cam_z:.1f} m")
print("[CESIUM] © Cesium ion | © OpenStreetMap contributors | © 內政部國土測繪中心 (NLSC)")
print("[SCENE] Running — close the window to exit")

simulation_app.update()

_step = 0
_t_update_ms = 0.0
_t_rep_ms    = 0.0
while simulation_app.is_running():
    _t0 = time.perf_counter()
    simulation_app.update()   # advances physics + rendering
    _t_update_ms = (time.perf_counter() - _t0) * 1000.0
    _step += 1

    # ── Read physics state from background thread ──────────────────────────────
    with _kin_lock:
        _x_enu = _kx;  _y_enu = _ky;  _z_abs = _kz
        _qx = _kqx;  _qy = _kqy;  _qz = _kqz;  _qw = _kqw
        _yaw_CCW = -_kyaw_rad

    # ── Update drone mesh ──────────────────────────────────────────────────────
    drone_pos_op.Set(Gf.Vec3d(_x_enu, _y_enu, _z_abs))
    drone_orient_op.Set(Gf.Quatd(_qw, _qx, _qy, _qz))
    # Gimbal stabilisation: cancel roll+pitch but preserve yaw so the camera
    # always points straight down AND the top of the image follows the drone nose.
    # camera_local = conj(drone_quat) * yaw_only_quat
    # → camera world orient = yaw_only, which is nadir + heading-aligned.
    _cy = math.cos(_yaw_CCW / 2.0)
    _sy = math.sin(_yaw_CCW / 2.0)
    drone_cam_orient_op.Set(Gf.Quatd(
         _qw * _cy + _qz * _sy,   # w
        -_qx * _cy - _qy * _sy,   # x
         _qx * _sy - _qy * _cy,   # y
         _qw * _sy - _qz * _cy,   # z
    ))

    # ── Drain pending ROS2 events ──────────────────────────────────────────────
    if _ros2_node is not None:
        rclpy.spin_once(_ros2_node, timeout_sec=0.0)

    # ── HUD + geo coordinates ──────────────────────────────────────────────────
    _lat, _lon = to_latlon(_x_enu, _y_enu)
    _alt     = _z_abs
    _agl     = _alt - centre_elev
    _yaw_deg = math.degrees(-_kyaw_rad)   # CCW-positive for HUD / meta.json

    if _hud_ok:
        _lbl_latlon.text = f"  LAT  {_lat:.5f}°N    LON  {_lon:.5f}°E"
        _lbl_alt.text    = f"  ALT  {_alt:.1f} m MSL    AGL  {_agl:.1f} m"
        _lbl_cam.text    = f"  CAM  Overview"

    # ── Frame capture ──────────────────────────────────────────────────────────────
    if _step % DRONE_SAVE_EVERY == 0:
        _t0 = time.perf_counter()
        rep.orchestrator.step(rt_subframes=0, delta_time=0.0)
        _t_step_ms = (time.perf_counter() - _t0) * 1000.0
        raw = _rgb.get_data()
        _t_rep_ms = (time.perf_counter() - _t0) * 1000.0
        _t_get_ms = _t_rep_ms - _t_step_ms
        if _step % (DRONE_SAVE_EVERY * 20) == 0:
            print(f"[PERF] sim.update {_t_update_ms:.0f} ms | "
                  f"rep.step {_t_step_ms:.0f} ms | get_data {_t_get_ms:.0f} ms | "
                  f"loop {(_t_update_ms + _t_rep_ms):.0f} ms total")
        if raw is None:
            print(f"[DRONE] step {_step}: get_data() returned None — frame skipped")
        else:
            arr = raw if isinstance(raw, np.ndarray) else raw.get("data")
            if arr is None or arr.size == 0:
                print(f"[DRONE] step {_step}: empty frame — skipped")
            else:
                rgb_arr = arr[:, :, :3].astype(np.uint8)
                if _step == DRONE_SAVE_EVERY:
                    print(f"[DRONE] Frame capture working — publishing via background thread")
                # Encode + publish in background; drop frame if worker is still busy
                try:
                    _pub_q.put_nowait((rgb_arr, _lat, _lon, _alt, _agl, _yaw_CCW, _step))
                except _queue.Full:
                    pass

_pub_q.put(None)   # signal publish worker to exit cleanly
_pub_thread.join(timeout=2.0)
_trace_f.close()
print(f"[DRONE] Flight trace saved → {_trace_path}")
simulation_app.close()
