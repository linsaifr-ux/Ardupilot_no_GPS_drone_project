"""
Geographic coordinate utilities for the realistic Isaac Sim scene.
No Isaac Sim dependency — safe to import in training code.

Coordinate system
-----------------
  Origin  : CENTER_LAT, CENTER_LON
  X axis  : East  (positive = east,  negative = west)
  Y axis  : North (positive = north, negative = south)
  Z axis  : Up    (positive = up,    0 ≈ sea level)
  Units   : metres

Usage
-----
    from geo_utils import world_to_latlon, latlon_to_world, camera_geo

    lat, lon = world_to_latlon(x_m, y_m)   # camera XY → geo
    x, y     = latlon_to_world(lat, lon)    # geo → camera XY
    info     = camera_geo(cam_x, cam_y, cam_z)   # full dict for training label
"""

import math, json, os

CENTER_LAT = 23.450868
CENTER_LON = 120.286135
R_EARTH    = 6_371_000.0
COS_LAT    = math.cos(math.radians(CENTER_LAT))

def world_to_latlon(x_m: float, y_m: float):
    """Convert world-space (x, y) in metres → (latitude, longitude)."""
    lat = CENTER_LAT + (y_m / R_EARTH) * (180.0 / math.pi)
    lon = CENTER_LON + (x_m / (R_EARTH * COS_LAT)) * (180.0 / math.pi)
    return lat, lon

def latlon_to_world(lat: float, lon: float):
    """Convert (latitude, longitude) → world-space (x, y) in metres."""
    x = math.radians(lon - CENTER_LON) * R_EARTH * COS_LAT
    y = math.radians(lat - CENTER_LAT) * R_EARTH
    return x, y

def camera_geo(cam_x: float, cam_y: float, cam_z: float) -> dict:
    """Return a dict with full geographic label for a camera position."""
    lat, lon = world_to_latlon(cam_x, cam_y)
    return {
        "latitude":  lat,
        "longitude": lon,
        "altitude_m": cam_z,
        "world_x_m": cam_x,
        "world_y_m": cam_y,
    }

def load_metadata(scene_dir: str = None) -> dict:
    """Load geo_metadata.json from the scene directory."""
    path = os.path.join(scene_dir or os.path.dirname(__file__), "geo_metadata.json")
    with open(path) as f:
        return json.load(f)
