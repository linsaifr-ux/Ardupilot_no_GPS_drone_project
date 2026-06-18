#!/usr/bin/env python3
"""
Realistic 3D scene — satellite imagery + SRTM terrain + OSM buildings
Centre : 23.450868, 120.286135 (Chiayi, Taiwan)
Radius : 2 km

Every world-space point (x, y, z) maps to a real (lat, lon, alt).
Geographic metadata is written to geo_metadata.json for model training.

Run:
    DISPLAY=:2 OMNI_KIT_ACCEPT_EULA=Y conda run -n isaac_sim_test python realistic_scene.py
"""

import json, math, os, random, re, time, urllib.parse, urllib.request
import io as _io

random.seed(42)

# ── CONSTANTS ─────────────────────────────────────────────────────────────────
CENTER_LAT = 23.450868
CENTER_LON = 120.286135
RADIUS_M   = 2000.0
R_EARTH    = 6_371_000.0
COS_LAT    = math.cos(math.radians(CENTER_LAT))
HERE       = os.path.dirname(os.path.abspath(__file__))

def to_xy(lat, lon):
    return (math.radians(lon - CENTER_LON) * R_EARTH * COS_LAT,
            math.radians(lat - CENTER_LAT) * R_EARTH)

def to_latlon(x, y):
    return (CENTER_LAT + (y / R_EARTH) * (180 / math.pi),
            CENTER_LON + (x / (R_EARTH * COS_LAT)) * (180 / math.pi))

# ── ISAAC SIM ─────────────────────────────────────────────────────────────────
from isaacsim import SimulationApp
simulation_app = SimulationApp({
    "headless": False,
    "width": 1920,
    "height": 1080,
    "window_title": "Isaac Sim — Realistic Scene 23.45°N 120.29°E",
})

import omni.usd
from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdPhysics, UsdShade, Vt

stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)

# ── SATELLITE TILES ───────────────────────────────────────────────────────────
from PIL import Image

SAT_ZOOM  = 17
SAT_CACHE = os.path.join(HERE, "satellite_ground.jpg")

def _deg2tile(lat, lon, z):
    n = 1 << z
    x = int((lon + 180.0) / 360.0 * n)
    lr = math.log(math.tan(math.radians(lat)) + 1.0 / math.cos(math.radians(lat)))
    y = int((1.0 - lr / math.pi) / 2.0 * n)
    return x, y

def _tile2deg(tx, ty, z):
    n = 1 << z
    lon = tx / n * 360.0 - 180.0
    lat = math.degrees(math.atan(math.sinh(math.pi * (1 - 2 * ty / n))))
    return lat, lon

def fetch_satellite():
    d_lat = RADIUS_M / 111_320.0
    d_lon = RADIUS_M / (111_320.0 * COS_LAT)
    tx_min, ty_min = _deg2tile(CENTER_LAT + d_lat, CENTER_LON - d_lon, SAT_ZOOM)
    tx_max, ty_max = _deg2tile(CENTER_LAT - d_lat, CENTER_LON + d_lon, SAT_ZOOM)
    nw_lat, nw_lon = _tile2deg(tx_min,     ty_min,     SAT_ZOOM)
    se_lat, se_lon = _tile2deg(tx_max + 1, ty_max + 1, SAT_ZOOM)
    wx_w, wy_n = to_xy(nw_lat, nw_lon)
    wx_e, wy_s = to_xy(se_lat, se_lon)
    bounds = dict(wx_w=wx_w, wy_s=wy_s, wx_e=wx_e, wy_n=wy_n,
                  nw_lat=nw_lat, nw_lon=nw_lon, se_lat=se_lat, se_lon=se_lon)

    if os.path.exists(SAT_CACHE):
        print(f"[SAT] Using cached {SAT_CACHE}")
        return SAT_CACHE, bounds

    nx = tx_max - tx_min + 1
    ny = ty_max - ty_min + 1
    print(f"[SAT] Downloading {nx}×{ny}={nx*ny} tiles at zoom {SAT_ZOOM} …")
    TILE = 256
    mosaic = Image.new("RGB", (nx * TILE, ny * TILE))
    for tx in range(tx_min, tx_max + 1):
        for ty in range(ty_min, ty_max + 1):
            url = (f"https://server.arcgisonline.com/ArcGIS/rest/services"
                   f"/World_Imagery/MapServer/tile/{SAT_ZOOM}/{ty}/{tx}")
            for attempt in range(3):
                try:
                    req = urllib.request.Request(url, headers={"User-Agent": "IsaacSimScene/1.0"})
                    with urllib.request.urlopen(req, timeout=15) as r:
                        tile = Image.open(_io.BytesIO(r.read())).convert("RGB")
                    mosaic.paste(tile, ((tx - tx_min) * TILE, (ty - ty_min) * TILE))
                    break
                except Exception as e:
                    if attempt == 2: print(f"  [SAT] skip {tx},{ty}: {e}")
                    else: time.sleep(0.5)
            time.sleep(0.04)
    mosaic.save(SAT_CACHE, "JPEG", quality=92)
    print(f"[SAT] Saved {mosaic.width}×{mosaic.height} mosaic → {SAT_CACHE}")
    return SAT_CACHE, bounds

sat_path, sat_bounds = fetch_satellite()
wx_w = sat_bounds["wx_w"]; wy_s = sat_bounds["wy_s"]
wx_e = sat_bounds["wx_e"]; wy_n = sat_bounds["wy_n"]

# ── ELEVATION (SRTM 30 m via Open Topo Data) ──────────────────────────────────
ELEV_CACHE = os.path.join(HERE, "elevation_cache.json")
GRID_N     = 40   # 40×40 grid = 1600 points, 16 batch requests

def fetch_elevation():
    if os.path.exists(ELEV_CACHE):
        print(f"[ELEV] Using cached {ELEV_CACHE}")
        with open(ELEV_CACHE) as f:
            return json.load(f)

    # Sample points: j=0 → south (wy_s), j=GRID_N-1 → north (wy_n)
    points = []
    for j in range(GRID_N):
        for i in range(GRID_N):
            x = wx_w + i * (wx_e - wx_w) / (GRID_N - 1)
            y = wy_s + j * (wy_n - wy_s) / (GRID_N - 1)
            lat, lon = to_latlon(x, y)
            points.append((lat, lon))

    BATCH = 100
    total  = len(points)
    n_bat  = math.ceil(total / BATCH)
    print(f"[ELEV] Fetching {total} elevation points ({n_bat} requests) …")
    elevs  = []

    for b in range(0, total, BATCH):
        batch = points[b : b + BATCH]
        locs  = "|".join(f"{lat},{lon}" for lat, lon in batch)
        url   = f"https://api.opentopodata.org/v1/srtm30m?locations={locs}"
        ok    = False
        for attempt in range(3):
            try:
                req = urllib.request.Request(url, headers={"User-Agent": "IsaacSimScene/1.0"})
                with urllib.request.urlopen(req, timeout=30) as r:
                    data = json.loads(r.read())
                for res in data["results"]:
                    elevs.append(float(res["elevation"] or 0.0))
                ok = True
                break
            except Exception as e:
                print(f"  [ELEV] batch {b//BATCH+1} attempt {attempt+1} failed: {e}")
                time.sleep(2)
        if not ok:
            elevs.extend([0.0] * len(batch))
        time.sleep(1.1)   # Open Topo Data: 1 req/s limit

    # Reshape → grid[j][i]
    grid = [[elevs[j * GRID_N + i] for i in range(GRID_N)] for j in range(GRID_N)]
    with open(ELEV_CACHE, "w") as f:
        json.dump(grid, f)
    print(f"[ELEV] Saved elevation grid → {ELEV_CACHE}")
    return grid

elev_grid = fetch_elevation()

def sample_elev(x, y):
    """Bilinear interpolation of elevation at world (x, y)."""
    fi = (x - wx_w) / (wx_e - wx_w) * (GRID_N - 1)
    fj = (y - wy_s) / (wy_n - wy_s) * (GRID_N - 1)
    i0 = max(0, min(GRID_N - 2, int(fi)))
    j0 = max(0, min(GRID_N - 2, int(fj)))
    di, dj = fi - i0, fj - j0
    return (elev_grid[j0][i0]   * (1-di) * (1-dj) +
            elev_grid[j0][i0+1] * di     * (1-dj) +
            elev_grid[j0+1][i0] * (1-di) * dj     +
            elev_grid[j0+1][i0+1] * di   * dj)

# ── OSM DATA ──────────────────────────────────────────────────────────────────
OSM_CACHE = os.path.join(HERE, "osm_cache.json")

def fetch_osm():
    if os.path.exists(OSM_CACHE):
        print(f"[OSM] Using cached {OSM_CACHE}")
        with open(OSM_CACHE) as f:
            return json.load(f)
    d_lat = RADIUS_M / 111_320.0
    d_lon = RADIUS_M / (111_320.0 * COS_LAT)
    s, n  = CENTER_LAT - d_lat, CENTER_LAT + d_lat
    w, e  = CENTER_LON - d_lon, CENTER_LON + d_lon
    query = ("[out:json][timeout:120];\n("
             f'  way["building"]({s},{w},{n},{e});\n'
             f'  way["highway"]({s},{w},{n},{e});\n'
             f'  way["landuse"]({s},{w},{n},{e});\n'
             f'  way["natural"~"water|wood|scrub|grassland|wetland"]({s},{w},{n},{e});\n'
             f'  way["leisure"~"park|garden|playground|sports_centre"]({s},{w},{n},{e});\n'
             f'  way["waterway"~"river|stream|canal|drain"]({s},{w},{n},{e});\n'
             ");\nout geom;")
    data = urllib.parse.urlencode({"data": query}).encode()
    req  = urllib.request.Request("https://overpass-api.de/api/interpreter",
                                   data=data, headers={"User-Agent": "IsaacSimScene/1.0"})
    print(f"[OSM] Fetching OSM data …")
    with urllib.request.urlopen(req, timeout=180) as r:
        raw = json.loads(r.read())
    with open(OSM_CACHE, "w") as f:
        json.dump(raw, f)
    print(f"[OSM] Saved {len(raw.get('elements',[]))} elements → {OSM_CACHE}")
    return raw

osm   = fetch_osm()
elems = osm.get("elements", [])
print(f"[OSM] {len(elems)} elements to process")

# ── GEO METADATA (for training pipeline) ──────────────────────────────────────
n_bld = sum(1 for e in elems if e.get("type")=="way" and "building" in e.get("tags",{}))
n_road= sum(1 for e in elems if e.get("type")=="way" and "highway"  in e.get("tags",{}))
n_land= len(elems) - n_bld - n_road

geo_meta = {
    "version": "1.0",
    "description": (
        "Geographic metadata for the Isaac Sim realistic scene. "
        "Every world-space (x, y, z) maps to (lat, lon, alt_m)."
    ),
    "center": {"lat": CENTER_LAT, "lon": CENTER_LON,
                "description": "World origin (0,0,0) in geographic coordinates"},
    "radius_m": RADIUS_M,
    "coordinate_system": {
        "x_axis": "east  (+ = east,  − = west)",
        "y_axis": "north (+ = north, − = south)",
        "z_axis": "up    (+ = up,   ≈0 = sea level)",
        "units": "metres",
    },
    "conversion": {
        "world_to_latlon": {
            "lat": f"lat = {CENTER_LAT} + (y_m / {R_EARTH}) * (180 / pi)",
            "lon": f"lon = {CENTER_LON} + (x_m / ({R_EARTH} * {COS_LAT:.6f})) * (180 / pi)",
        },
        "latlon_to_world": {
            "x_m": f"x = (lon − {CENTER_LON}) * pi/180 * {R_EARTH} * {COS_LAT:.6f}",
            "y_m": f"y = (lat − {CENTER_LAT}) * pi/180 * {R_EARTH}",
        },
        "python_helper": "from geo_utils import world_to_latlon, latlon_to_world, camera_geo",
    },
    "satellite": {
        "source": "ESRI World Imagery (ArcGIS MapServer)",
        "zoom_level": SAT_ZOOM,
        "approx_resolution_m_per_px": 1.5,
        "file": "satellite_ground.jpg",
        "world_bounds": {
            "wx_w": wx_w, "wy_s": wy_s, "wx_e": wx_e, "wy_n": wy_n,
        },
        "latlon_bounds": {
            "north": sat_bounds["nw_lat"], "south": sat_bounds["se_lat"],
            "west":  sat_bounds["nw_lon"], "east":  sat_bounds["se_lon"],
        },
    },
    "elevation": {
        "source": "Open Topo Data — SRTM 30 m",
        "grid_n": GRID_N,
        "file": "elevation_cache.json",
    },
    "osm": {
        "source": "OpenStreetMap via Overpass API",
        "file": "osm_cache.json",
        "approx_counts": {"buildings": n_bld, "roads": n_road, "landuse": n_land},
    },
}

with open(os.path.join(HERE, "geo_metadata.json"), "w") as f:
    json.dump(geo_meta, f, indent=2)
print("[GEO] Wrote geo_metadata.json")

# ── USD GEO METADATA ──────────────────────────────────────────────────────────
stage.SetMetadata("customLayerData", {
    "geo:centerLat":    CENTER_LAT,
    "geo:centerLon":    CENTER_LON,
    "geo:radiusMeters": RADIUS_M,
    "geo:metersPerUnit": 1.0,
    "geo:upAxis":       "Z",
    "geo:xIsEast":      True,
    "geo:yIsNorth":     True,
    "geo:earthRadiusM": R_EARTH,
    "geo:cosLat":       COS_LAT,
    "geo:description":  (
        "world_to_lat = centerLat + y/earthRadiusM*(180/pi); "
        "world_to_lon = centerLon + x/(earthRadiusM*cosLat)*(180/pi)"
    ),
})

# ── USD HELPERS ───────────────────────────────────────────────────────────────
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

def signed_area(pts):
    a = 0.0
    n = len(pts)
    for i in range(n):
        j = (i + 1) % n
        a += pts[i][0] * pts[j][1] - pts[j][0] * pts[i][1]
    return a * 0.5

def ccw(pts):
    return pts if signed_area(pts) > 0 else pts[::-1]

_used = set()
def upath(base):
    p, i = base, 0
    while p in _used: i += 1; p = f"{base}_{i}"
    _used.add(p); return p

def extrude(path, pts, base_z, h, mat=None):
    pts = ccw(pts)
    n   = len(pts)
    if n < 3 or h <= 0: return None
    verts = ([Gf.Vec3f(x, y, base_z)     for x, y in pts] +
             [Gf.Vec3f(x, y, base_z + h) for x, y in pts])
    cnt, idx = [], []
    cnt.append(n); idx.extend(range(n - 1, -1, -1))
    cnt.append(n); idx.extend(range(n, 2 * n))
    for i in range(n):
        j = (i + 1) % n
        cnt.append(4); idx.extend([i, j, j + n, i + n])
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr(Vt.Vec3fArray(verts))
    mesh.CreateFaceVertexCountsAttr(cnt)
    mesh.CreateFaceVertexIndicesAttr(idx)
    mesh.CreateSubdivisionSchemeAttr("none")
    if mat: bind(mesh.GetPrim(), mat)
    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
    return mesh

def flat_poly(path, pts, z, mat=None):
    pts = ccw(pts)
    n   = len(pts)
    if n < 3: return None
    verts = [Gf.Vec3f(x, y, z) for x, y in pts]
    mesh  = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr(Vt.Vec3fArray(verts))
    mesh.CreateFaceVertexCountsAttr([n])
    mesh.CreateFaceVertexIndicesAttr(list(range(n)))
    mesh.CreateSubdivisionSchemeAttr("none")
    if mat: bind(mesh.GetPrim(), mat)
    return mesh

ROAD_W = {
    "motorway":16,"motorway_link":8,"trunk":13,"trunk_link":6.5,
    "primary":11,"primary_link":5.5,"secondary":9,"secondary_link":4.5,
    "tertiary":7.5,"tertiary_link":3.5,"residential":5.5,"living_street":4.5,
    "service":3.5,"unclassified":5,"pedestrian":5,
    "footway":2,"cycleway":2,"path":1.5,"track":3,
}

def road_strip(path, pts, width, mat, z_offset=0.15):
    hw = width / 2.0
    verts, cnt, idx, vi = [], [], [], 0
    for i in range(len(pts) - 1):
        x0, y0 = pts[i]; x1, y1 = pts[i + 1]
        dx, dy = x1 - x0, y1 - y0
        L = math.hypot(dx, dy)
        if L < 0.01: continue
        px, py = -dy / L * hw, dx / L * hw
        # Elevate each endpoint to terrain height
        z0 = sample_elev(x0, y0) + z_offset
        z1 = sample_elev(x1, y1) + z_offset
        verts += [Gf.Vec3f(x0+px, y0+py, z0), Gf.Vec3f(x0-px, y0-py, z0),
                  Gf.Vec3f(x1-px, y1-py, z1), Gf.Vec3f(x1+px, y1+py, z1)]
        cnt.append(4); idx.extend([vi, vi+1, vi+2, vi+3]); vi += 4
    if not verts: return None
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr(Vt.Vec3fArray(verts))
    mesh.CreateFaceVertexCountsAttr(cnt)
    mesh.CreateFaceVertexIndicesAttr(idx)
    mesh.CreateSubdivisionSchemeAttr("none")
    bind(mesh.GetPrim(), mat)
    return mesh

# ── TERRAIN MESH WITH SATELLITE TEXTURE ───────────────────────────────────────
def make_terrain(path, img_path):
    """40×40 grid terrain mesh with SRTM elevation, draped with satellite texture."""
    N = GRID_N
    verts, uvs = [], []
    for j in range(N):
        for i in range(N):
            x = wx_w + i * (wx_e - wx_w) / (N - 1)
            y = wy_s + j * (wy_n - wy_s) / (N - 1)
            z = elev_grid[j][i]
            verts.append(Gf.Vec3f(x, y, z))
            # UV: U=0 west→east=1; V=0 north (j=N-1) → south (j=0)=1
            u = i / (N - 1)
            v = 1.0 - j / (N - 1)
            uvs.append(Gf.Vec2f(u, v))

    cnt, idx = [], []
    for j in range(N - 1):
        for i in range(N - 1):
            # CCW (normal = +Z): SW→NW→NE→SE
            sw = j     * N + i
            nw = (j+1) * N + i
            ne = (j+1) * N + (i+1)
            se = j     * N + (i+1)
            cnt.append(4); idx.extend([sw, nw, ne, se])

    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr(Vt.Vec3fArray(verts))
    mesh.CreateFaceVertexCountsAttr(cnt)
    mesh.CreateFaceVertexIndicesAttr(idx)
    mesh.CreateSubdivisionSchemeAttr("none")

    st_pv = UsdGeom.PrimvarsAPI(mesh.GetPrim()).CreatePrimvar(
        "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex)
    st_pv.Set(Vt.Vec2fArray(uvs))

    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())

    # Satellite texture material
    mp  = path + "/SatMat"
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

print("[SCENE] Building terrain mesh …")
make_terrain("/World/Terrain", sat_path)

# ── LIGHTS ────────────────────────────────────────────────────────────────────
sky = UsdLux.DomeLight.Define(stage, "/World/Lights/Sky")
sky.CreateIntensityAttr(500)
sky.CreateColorAttr(Gf.Vec3f(0.52, 0.68, 1.0))

sun = UsdLux.DistantLight.Define(stage, "/World/Lights/Sun")
sun.CreateIntensityAttr(6000)
sun.CreateColorAttr(Gf.Vec3f(1.0, 0.97, 0.87))
UsdGeom.Xformable(sun).AddRotateXYZOp().Set(Gf.Vec3d(-48.0, 0.0, 35.0))

# ── MATERIALS ─────────────────────────────────────────────────────────────────
M = {}
def m(k, *a, **kw): M[k] = pbr_mat(f"/Mat/{k}", *a, **kw)

m("asphalt",  (0.14, 0.14, 0.14), roughness=0.95)
m("footway",  (0.72, 0.68, 0.58), roughness=0.92)
m("bld_lo",   (0.80, 0.72, 0.60), roughness=0.82)
m("bld_mid",  (0.65, 0.64, 0.63), roughness=0.72)
m("bld_hi",   (0.58, 0.63, 0.70), metallic=0.18, roughness=0.40)
m("bld_ind",  (0.55, 0.50, 0.45), roughness=0.90)
m("park",     (0.20, 0.50, 0.16), roughness=0.95)
m("forest",   (0.09, 0.32, 0.09), roughness=0.97)
m("farmland", (0.60, 0.54, 0.28), roughness=0.97)
m("water",    (0.10, 0.30, 0.62), metallic=0.25, roughness=0.06)
m("gravel",   (0.48, 0.45, 0.40), roughness=0.93)

LU_MAT = {
    "grass":"park","park":"park","garden":"park","recreation_ground":"park",
    "playground":"park","sports_centre":"park","meadow":"park","grassland":"park",
    "forest":"forest","wood":"forest","scrub":"forest",
    "farmland":"farmland","farmyard":"farmland","orchard":"farmland",
    "water":"water","reservoir":"water","basin":"water","wetland":"water",
    "river":"water","stream":"water","canal":"water","drain":"water",
    "residential":"gravel","commercial":"gravel","industrial":"gravel","retail":"gravel",
}

# ── BUILD OSM GEOMETRY ────────────────────────────────────────────────────────
print("[SCENE] Building OSM geometry …")
n_bld_built = n_road_built = n_land_built = 0

for el in elems:
    if el.get("type") != "way": continue
    tags = el.get("tags", {})
    geom = el.get("geometry", [])
    if len(geom) < 2: continue
    nodes = [to_xy(g["lat"], g["lon"]) for g in geom]
    eid   = el["id"]

    # BUILDINGS — base sits on terrain elevation
    if "building" in tags:
        if len(nodes) < 4: continue
        pts = nodes[:-1] if nodes[0] == nodes[-1] else nodes
        if len(pts) < 3: continue

        h = None
        for k in ("building:height", "height"):
            if k in tags:
                try: h = float(str(tags[k]).rstrip("m").strip()); break
                except ValueError: pass
        if h is None and "building:levels" in tags:
            try: h = float(tags["building:levels"]) * 3.5
            except ValueError: pass
        if h is None:
            bt = tags.get("building", "yes")
            if bt in ("apartments","residential","house","detached"): h = random.uniform(6, 20)
            elif bt in ("commercial","office","hotel"):               h = random.uniform(12, 60)
            elif bt in ("retail","shop"):                             h = random.uniform(4, 12)
            elif bt in ("industrial","warehouse"):                    h = random.uniform(5, 14)
            elif bt in ("church","cathedral","temple","shrine"):      h = random.uniform(8, 22)
            elif bt in ("school","university","hospital"):            h = random.uniform(10, 30)
            else:                                                      h = random.uniform(5, 18)

        # Base z = terrain elevation at building centroid
        cx = sum(x for x, y in pts) / len(pts)
        cy = sum(y for x, y in pts) / len(pts)
        base_z = sample_elev(cx, cy)

        bm = (M["bld_hi"]  if h > 40 else
              M["bld_mid"] if h > 18 else
              M["bld_ind"] if tags.get("building","") in ("industrial","warehouse") else
              M["bld_lo"])
        extrude(upath(f"/World/Buildings/B{eid}"), pts, base_z, h, mat=bm)
        n_bld_built += 1

    # ROADS — follow terrain elevation
    elif "highway" in tags:
        ht   = tags["highway"]
        w    = ROAD_W.get(ht, 4.0)
        foot = ht in ("footway","cycleway","path","pedestrian")
        rm   = M["footway"] if foot else M["asphalt"]
        road_strip(upath(f"/World/Roads/R{eid}"), nodes, w, rm,
                   z_offset=0.12 if foot else 0.08)
        n_road_built += 1

    # LANDUSE
    else:
        lu = (tags.get("landuse") or tags.get("natural") or
              tags.get("leisure") or tags.get("waterway"))
        if not lu: continue
        mk = LU_MAT.get(lu)
        if not mk: continue
        pts = nodes[:-1] if (len(nodes) > 2 and nodes[0] == nodes[-1]) else nodes
        if len(pts) < 3: continue
        cx = sum(x for x, y in pts) / len(pts)
        cy = sum(y for x, y in pts) / len(pts)
        z = sample_elev(cx, cy) + (-0.1 if mk == "water" else 0.02)
        flat_poly(upath(f"/World/Landuse/L{eid}"), pts, z, M[mk])
        n_land_built += 1

print(f"[SCENE] {n_bld_built} buildings | {n_road_built} roads | {n_land_built} landuse areas")

# ── VIEWPORT CAMERA ────────────────────────────────────────────────────────────
# Positioned at a real lat/lon — (0,0,z) = exactly CENTER_LAT, CENTER_LON
centre_elev = sample_elev(0, 0)
cam = UsdGeom.Camera.Define(stage, "/World/Camera")
cam.CreateFocalLengthAttr(28.0)
cxf = UsdGeom.Xformable(cam)
# 600 m north, 800 m up from centre — maps to a real lat/lon
cam_x, cam_y, cam_z = 0.0, -600.0, centre_elev + 800.0
cxf.AddTranslateOp().Set(Gf.Vec3d(cam_x, cam_y, cam_z))
cxf.AddRotateXYZOp().Set(Gf.Vec3d(-48.0, 0.0, 0.0))

# Store camera's real lat/lon as USD custom data
cam_lat, cam_lon = to_latlon(cam_x, cam_y)
cam.GetPrim().SetCustomData({
    "geo:latitude":  cam_lat,
    "geo:longitude": cam_lon,
    "geo:altitude_m": cam_z,
})

try:
    from omni.kit.viewport.utility import get_active_viewport
    get_active_viewport().camera_path = "/World/Camera"
except Exception:
    pass

# ── SIMULATION LOOP ────────────────────────────────────────────────────────────
from isaacsim.core.api import World
world = World()
world.reset()
print(f"[GEO] Camera at lat={cam_lat:.6f}, lon={cam_lon:.6f}, alt={cam_z:.1f} m")
print("[SCENE] Simulation running — close the window to exit")

while simulation_app.is_running():
    world.step(render=True)

simulation_app.close()
