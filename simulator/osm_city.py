#!/usr/bin/env python3
"""
Real-world city scene from OpenStreetMap + ESRI satellite imagery
Centre: 23.450868, 120.286135 (Chiayi, Taiwan)  Radius: 2 km
Run: DISPLAY=:2 OMNI_KIT_ACCEPT_EULA=Y conda run -n isaac_sim_test python osm_city.py
"""

import json, math, os, random, re, time, urllib.parse, urllib.request
import io as _io

random.seed(42)

# ── ISAAC SIM ─────────────────────────────────────────────────────────────────
from isaacsim import SimulationApp
simulation_app = SimulationApp({
    "headless": False,
    "width": 1920,
    "height": 1080,
    "window_title": "Isaac Sim — OSM+Satellite Chiayi 23.45°N 120.29°E",
})

import omni.usd
from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdPhysics, UsdShade, Vt

stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)

# ── CONSTANTS ─────────────────────────────────────────────────────────────────
CENTER_LAT = 23.450868
CENTER_LON = 120.286135
RADIUS_M   = 2000.0
R_EARTH    = 6_371_000.0
COS_LAT    = math.cos(math.radians(CENTER_LAT))
HERE       = os.path.dirname(os.path.abspath(__file__))

def to_xy(lat, lon):
    x = math.radians(lon - CENTER_LON) * R_EARTH * COS_LAT
    y = math.radians(lat - CENTER_LAT) * R_EARTH
    return (x, y)

# ── SATELLITE TILE FETCH ──────────────────────────────────────────────────────
from PIL import Image

SAT_ZOOM  = 17          # ~1.5 m/px at lat 23°
SAT_CACHE = os.path.join(HERE, "satellite_ground.jpg")

def _deg2tile(lat, lon, z):
    n = 1 << z
    x = int((lon + 180.0) / 360.0 * n)
    lr = math.log(math.tan(math.radians(lat)) + 1.0 / math.cos(math.radians(lat)))
    y = int((1.0 - lr / math.pi) / 2.0 * n)
    return x, y

def _tile2deg(tx, ty, z):
    """NW corner of tile (tx, ty)."""
    n = 1 << z
    lon = tx / n * 360.0 - 180.0
    lat = math.degrees(math.atan(math.sinh(math.pi * (1 - 2 * ty / n))))
    return lat, lon

def fetch_satellite():
    """Download ESRI World Imagery tiles and stitch into one JPEG.
    Returns (image_path, wx_west, wy_south, wx_east, wy_north) world-space bounds."""

    d_lat = RADIUS_M / 111_320.0
    d_lon = RADIUS_M / (111_320.0 * COS_LAT)
    # NW tile covers the northern edge (smaller tile-y), SE the southern
    tx_min, ty_min = _deg2tile(CENTER_LAT + d_lat, CENTER_LON - d_lon, SAT_ZOOM)
    tx_max, ty_max = _deg2tile(CENTER_LAT - d_lat, CENTER_LON + d_lon, SAT_ZOOM)

    # World-space extent of the complete mosaic
    nw_lat, nw_lon = _tile2deg(tx_min,     ty_min,     SAT_ZOOM)
    se_lat, se_lon = _tile2deg(tx_max + 1, ty_max + 1, SAT_ZOOM)
    wx_west,  wy_north = to_xy(nw_lat, nw_lon)
    wx_east,  wy_south = to_xy(se_lat, se_lon)

    if os.path.exists(SAT_CACHE):
        print(f"[SAT] Using cached satellite image: {SAT_CACHE}")
        return SAT_CACHE, wx_west, wy_south, wx_east, wy_north

    nx = tx_max - tx_min + 1
    ny = ty_max - ty_min + 1
    print(f"[SAT] Downloading {nx}×{ny} = {nx*ny} tiles (zoom {SAT_ZOOM}) …")

    TILE_PX = 256
    mosaic = Image.new("RGB", (nx * TILE_PX, ny * TILE_PX))

    for tx in range(tx_min, tx_max + 1):
        for ty in range(ty_min, ty_max + 1):
            url = (f"https://server.arcgisonline.com/ArcGIS/rest/services"
                   f"/World_Imagery/MapServer/tile/{SAT_ZOOM}/{ty}/{tx}")
            for attempt in range(3):
                try:
                    req = urllib.request.Request(
                        url, headers={"User-Agent": "IsaacSimOSM/1.0"})
                    with urllib.request.urlopen(req, timeout=15) as r:
                        tile = Image.open(_io.BytesIO(r.read())).convert("RGB")
                    px = (tx - tx_min) * TILE_PX
                    py = (ty - ty_min) * TILE_PX
                    mosaic.paste(tile, (px, py))
                    break
                except Exception as e:
                    if attempt == 2:
                        print(f"  [SAT] Skip tile {tx},{ty}: {e}")
                    else:
                        time.sleep(0.5)
            time.sleep(0.04)   # polite rate limiting

    mosaic.save(SAT_CACHE, "JPEG", quality=92)
    print(f"[SAT] Saved {mosaic.width}×{mosaic.height} mosaic → {SAT_CACHE}")
    return SAT_CACHE, wx_west, wy_south, wx_east, wy_north

sat_path, wx_w, wy_s, wx_e, wy_n = fetch_satellite()

# ── OSM FETCH (cached) ────────────────────────────────────────────────────────
OSM_CACHE = os.path.join(HERE, "osm_cache.json")

def fetch_osm():
    if os.path.exists(OSM_CACHE):
        print(f"[OSM] Loading cached data from {OSM_CACHE}")
        with open(OSM_CACHE) as f:
            return json.load(f)
    d_lat = RADIUS_M / 111_320.0
    d_lon = RADIUS_M / (111_320.0 * COS_LAT)
    s, n  = CENTER_LAT - d_lat, CENTER_LAT + d_lat
    w, e  = CENTER_LON - d_lon, CENTER_LON + d_lon
    query = (
        "[out:json][timeout:120];\n("
        f'  way["building"]({s},{w},{n},{e});\n'
        f'  way["highway"]({s},{w},{n},{e});\n'
        f'  way["landuse"]({s},{w},{n},{e});\n'
        f'  way["natural"~"water|wood|scrub|grassland|wetland"]({s},{w},{n},{e});\n'
        f'  way["leisure"~"park|garden|playground|sports_centre"]({s},{w},{n},{e});\n'
        f'  way["waterway"~"river|stream|canal|drain"]({s},{w},{n},{e});\n'
        ");\nout geom;"
    )
    data = urllib.parse.urlencode({"data": query}).encode()
    req  = urllib.request.Request(
        "https://overpass-api.de/api/interpreter", data=data,
        headers={"User-Agent": "IsaacSimOSM/1.0"},
    )
    print(f"[OSM] Fetching 2 km radius around ({CENTER_LAT}, {CENTER_LON}) …")
    with urllib.request.urlopen(req, timeout=180) as r:
        raw = json.loads(r.read())
    with open(OSM_CACHE, "w") as f:
        json.dump(raw, f)
    print(f"[OSM] Saved {len(raw.get('elements',[]))} elements → {OSM_CACHE}")
    return raw

osm   = fetch_osm()
elems = osm.get("elements", [])
print(f"[OSM] {len(elems)} elements to process")

# ── USD HELPERS ───────────────────────────────────────────────────────────────
def bind(prim, mat):
    UsdShade.MaterialBindingAPI(prim).Bind(mat)

def pbr_mat(path, rgb, metallic=0.0, roughness=0.7):
    mat = UsdShade.Material.Define(stage, path)
    sh  = UsdShade.Shader.Define(stage, path + "/PBR")
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

def extrude(path, pts, h, mat=None, collide=True):
    pts = ccw(pts)
    n   = len(pts)
    if n < 3 or h <= 0:
        return None
    verts = ([Gf.Vec3f(x, y, 0.0) for x, y in pts] +
             [Gf.Vec3f(x, y, h)   for x, y in pts])
    cnt, idx = [], []
    cnt.append(n);  idx.extend(range(n - 1, -1, -1))   # bottom cap
    cnt.append(n);  idx.extend(range(n, 2 * n))          # top cap
    for i in range(n):
        j = (i + 1) % n
        cnt.append(4); idx.extend([i, j, j + n, i + n]) # sides
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr(Vt.Vec3fArray(verts))
    mesh.CreateFaceVertexCountsAttr(cnt)
    mesh.CreateFaceVertexIndicesAttr(idx)
    mesh.CreateSubdivisionSchemeAttr("none")
    if mat:   bind(mesh.GetPrim(), mat)
    if collide: UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
    return mesh

def flat_poly(path, pts, z=0.0, mat=None):
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
    "motorway": 16.0, "motorway_link": 8.0,
    "trunk": 13.0,    "trunk_link": 6.5,
    "primary": 11.0,  "primary_link": 5.5,
    "secondary": 9.0, "secondary_link": 4.5,
    "tertiary": 7.5,  "tertiary_link": 3.5,
    "residential": 5.5, "living_street": 4.5,
    "service": 3.5,   "unclassified": 5.0,
    "pedestrian": 5.0,
    "footway": 2.0, "cycleway": 2.0, "path": 1.5, "track": 3.0,
}

def road_strip(path, pts, width, mat, z=0.02):
    hw = width / 2.0
    verts, cnt, idx, vi = [], [], [], 0
    for i in range(len(pts) - 1):
        x0, y0 = pts[i]; x1, y1 = pts[i + 1]
        dx, dy = x1 - x0, y1 - y0
        L = math.hypot(dx, dy)
        if L < 0.01: continue
        px, py = -dy / L * hw, dx / L * hw
        verts += [Gf.Vec3f(x0+px,y0+py,z), Gf.Vec3f(x0-px,y0-py,z),
                  Gf.Vec3f(x1-px,y1-py,z), Gf.Vec3f(x1+px,y1+py,z)]
        cnt.append(4); idx.extend([vi,vi+1,vi+2,vi+3]); vi += 4
    if not verts: return None
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr(Vt.Vec3fArray(verts))
    mesh.CreateFaceVertexCountsAttr(cnt)
    mesh.CreateFaceVertexIndicesAttr(idx)
    mesh.CreateSubdivisionSchemeAttr("none")
    bind(mesh.GetPrim(), mat)
    return mesh

# ── SATELLITE-TEXTURED GROUND ─────────────────────────────────────────────────
def make_sat_ground(path, img_path, gx_w, gy_s, gx_e, gy_n):
    """Quad covering the satellite mosaic extent with a UV-mapped texture."""
    # Vertices: SW, SE, NE, NW  (CCW from above → normal = +Z)
    pts = [
        Gf.Vec3f(gx_w, gy_s, -0.01),   # 0  SW
        Gf.Vec3f(gx_e, gy_s, -0.01),   # 1  SE
        Gf.Vec3f(gx_e, gy_n, -0.01),   # 2  NE
        Gf.Vec3f(gx_w, gy_n, -0.01),   # 3  NW
    ]
    # Omniverse RTX: V=0 at image top (DirectX convention).
    # Image top = north, so NW/NE get V=0, SW/SE get V=1.
    uvs = Vt.Vec2fArray([
        Gf.Vec2f(0.0, 1.0),   # 0  SW
        Gf.Vec2f(1.0, 1.0),   # 1  SE
        Gf.Vec2f(1.0, 0.0),   # 2  NE
        Gf.Vec2f(0.0, 0.0),   # 3  NW
    ])

    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr(Vt.Vec3fArray(pts))
    mesh.CreateFaceVertexCountsAttr([4])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    mesh.CreateSubdivisionSchemeAttr("none")

    st_pv = UsdGeom.PrimvarsAPI(mesh.GetPrim()).CreatePrimvar(
        "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex)
    st_pv.Set(uvs)

    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())

    # ── Texture material ──────────────────────────────────────────────────────
    mp   = path + "/SatMat"
    mat  = UsdShade.Material.Define(stage, mp)

    pbr  = UsdShade.Shader.Define(stage, mp + "/PBR")
    pbr.CreateIdAttr("UsdPreviewSurface")
    pbr.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.88)
    pbr.CreateInput("metallic",  Sdf.ValueTypeNames.Float).Set(0.0)

    tex  = UsdShade.Shader.Define(stage, mp + "/DiffTex")
    tex.CreateIdAttr("UsdUVTexture")
    tex.CreateInput("file",            Sdf.ValueTypeNames.Asset).Set(img_path)
    tex.CreateInput("wrapS",           Sdf.ValueTypeNames.Token).Set("clamp")
    tex.CreateInput("wrapT",           Sdf.ValueTypeNames.Token).Set("clamp")
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

make_sat_ground("/World/Ground", sat_path, wx_w, wy_s, wx_e, wy_n)

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

# ── LANDUSE MAP ───────────────────────────────────────────────────────────────
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
n_bld = n_road = n_land = 0
_used = set()

def upath(base):
    p, i = base, 0
    while p in _used: i += 1; p = f"{base}_{i}"
    _used.add(p); return p

for el in elems:
    if el.get("type") != "way": continue
    tags = el.get("tags", {})
    geom = el.get("geometry", [])
    if len(geom) < 2: continue
    nodes = [to_xy(g["lat"], g["lon"]) for g in geom]
    eid   = el["id"]

    # ── BUILDINGS ─────────────────────────────────────────────────────────────
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
            if bt in ("apartments", "residential", "house", "detached"):
                h = random.uniform(6, 20)
            elif bt in ("commercial", "office", "hotel"):
                h = random.uniform(12, 60)
            elif bt in ("retail", "shop"):
                h = random.uniform(4, 12)
            elif bt in ("industrial", "warehouse"):
                h = random.uniform(5, 14)
            elif bt in ("church", "cathedral", "temple", "shrine"):
                h = random.uniform(8, 22)
            elif bt in ("school", "university", "hospital"):
                h = random.uniform(10, 30)
            else:
                h = random.uniform(5, 18)

        if   h > 40: bm = M["bld_hi"]
        elif h > 18: bm = M["bld_mid"]
        elif tags.get("building","") in ("industrial","warehouse"): bm = M["bld_ind"]
        else:        bm = M["bld_lo"]

        extrude(upath(f"/World/Buildings/B{eid}"), pts, h, mat=bm)
        n_bld += 1

    # ── ROADS ─────────────────────────────────────────────────────────────────
    elif "highway" in tags:
        ht   = tags["highway"]
        w    = ROAD_W.get(ht, 4.0)
        foot = ht in ("footway", "cycleway", "path", "pedestrian")
        rm   = M["footway"] if foot else M["asphalt"]
        rz   = 0.03 if foot else 0.02
        road_strip(upath(f"/World/Roads/R{eid}"), nodes, w, rm, rz)
        n_road += 1

    # ── LANDUSE / NATURAL / LEISURE / WATERWAY ────────────────────────────────
    else:
        lu = (tags.get("landuse") or tags.get("natural") or
              tags.get("leisure") or tags.get("waterway"))
        if not lu: continue
        mk = LU_MAT.get(lu)
        if not mk: continue
        pts = nodes[:-1] if (len(nodes) > 2 and nodes[0] == nodes[-1]) else nodes
        if len(pts) < 3: continue
        z = -0.08 if mk == "water" else 0.005
        flat_poly(upath(f"/World/Landuse/L{eid}"), pts, z, M[mk])
        n_land += 1

print(f"[OSM] Scene: {n_bld} buildings | {n_road} roads | {n_land} landuse areas")

# ── VIEWPORT CAMERA ────────────────────────────────────────────────────────────
cam = UsdGeom.Camera.Define(stage, "/World/Camera")
cam.CreateFocalLengthAttr(28.0)
cxf = UsdGeom.Xformable(cam)
cxf.AddTranslateOp().Set(Gf.Vec3d(0.0, -800.0, 800.0))
cxf.AddRotateXYZOp().Set(Gf.Vec3d(-45.0, 0.0, 0.0))

try:
    from omni.kit.viewport.utility import get_active_viewport
    get_active_viewport().camera_path = "/World/Camera"
except Exception:
    pass

# ── SIMULATION LOOP ────────────────────────────────────────────────────────────
from isaacsim.core.api import World
world = World()
world.reset()
print("[OSM] Simulation running — close the window to exit")

while simulation_app.is_running():
    world.step(render=True)

simulation_app.close()
