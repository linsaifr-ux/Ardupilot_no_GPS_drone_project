#!/usr/bin/env python3
"""
City scene with a car — Isaac Sim 6.0.0.0

Run:
    DISPLAY=:0 OMNI_KIT_ACCEPT_EULA=Y conda run -n isaac_sim_test python city_scene.py
"""
from isaacsim import SimulationApp

simulation_app = SimulationApp({
    "headless": False,
    "width": 1920,
    "height": 1080,
    "window_title": "Isaac Sim — City Scene",
})

import omni.usd
from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdPhysics, UsdShade

stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)


# ── HELPERS ──────────────────────────────────────────────────────────────────

def pbr(path, color, metallic=0.0, roughness=0.7):
    mat = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, path + "/PBR")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    sh.CreateInput("metallic",     Sdf.ValueTypeNames.Float).Set(metallic)
    sh.CreateInput("roughness",    Sdf.ValueTypeNames.Float).Set(roughness)
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    return mat


def box(path, pos, dims, mat=None, collide=True):
    """Axis-aligned box.  dims = full (x, y, z) extents in metres."""
    c = UsdGeom.Cube.Define(stage, path)
    c.CreateSizeAttr(1.0)
    xf = UsdGeom.Xformable(c)
    xf.AddTranslateOp().Set(Gf.Vec3d(*pos))
    xf.AddScaleOp().Set(Gf.Vec3d(*dims))
    if mat:
        UsdShade.MaterialBindingAPI(c.GetPrim()).Bind(mat)
    if collide:
        UsdPhysics.CollisionAPI.Apply(c.GetPrim())
    return c


def cyl(path, pos, radius, height, axis="Z", mat=None, collide=False):
    cy = UsdGeom.Cylinder.Define(stage, path)
    cy.CreateRadiusAttr(radius)
    cy.CreateHeightAttr(height)
    cy.CreateAxisAttr(axis)
    UsdGeom.Xformable(cy).AddTranslateOp().Set(Gf.Vec3d(*pos))
    if mat:
        UsdShade.MaterialBindingAPI(cy.GetPrim()).Bind(mat)
    if collide:
        UsdPhysics.CollisionAPI.Apply(cy.GetPrim())
    return cy


# ── MATERIALS ─────────────────────────────────────────────────────────────────
asphalt   = pbr("/Mat/Asphalt",  (0.12, 0.12, 0.12), roughness=0.95)
concrete  = pbr("/Mat/Concrete", (0.65, 0.65, 0.68), roughness=0.88)
glass     = pbr("/Mat/Glass",    (0.40, 0.55, 0.72), metallic=0.05, roughness=0.08)
yellow    = pbr("/Mat/Yellow",   (0.92, 0.80, 0.00), roughness=0.60)
white     = pbr("/Mat/White",    (0.95, 0.95, 0.95), roughness=0.70)
car_red   = pbr("/Mat/CarRed",   (0.85, 0.10, 0.08), metallic=0.75, roughness=0.15)
car_glass = pbr("/Mat/CarGlass", (0.45, 0.60, 0.80), metallic=0.00, roughness=0.04)
rubber    = pbr("/Mat/Rubber",   (0.08, 0.08, 0.08), roughness=0.95)
steel     = pbr("/Mat/Steel",    (0.55, 0.55, 0.58), metallic=0.85, roughness=0.25)
foliage   = pbr("/Mat/Foliage",  (0.14, 0.48, 0.12), roughness=0.92)
bark      = pbr("/Mat/Bark",     (0.36, 0.22, 0.10), roughness=0.95)
lamp_mat  = pbr("/Mat/Lamp",     (0.95, 0.90, 0.70), roughness=0.30)


# ── LIGHTS ───────────────────────────────────────────────────────────────────
sky = UsdLux.DomeLight.Define(stage, "/World/Lights/Sky")
sky.CreateIntensityAttr(600)
sky.CreateColorAttr(Gf.Vec3f(0.55, 0.72, 1.0))

sun = UsdLux.DistantLight.Define(stage, "/World/Lights/Sun")
sun.CreateIntensityAttr(7000)
sun.CreateColorAttr(Gf.Vec3f(1.0, 0.96, 0.86))
UsdGeom.Xformable(sun).AddRotateXYZOp().Set(Gf.Vec3d(-55.0, 0.0, 40.0))


# ── ROAD ─────────────────────────────────────────────────────────────────────
# Road surface (dark asphalt). Top face sits at z = 0.
box("/World/City/Road",       (0,  0, -0.05), (100, 100, 0.10), asphalt)

# Raised concrete sidewalks on both sides of the road
box("/World/City/Sidewalk_N", (0,  11,  0.08), (100, 2, 0.16), concrete)
box("/World/City/Sidewalk_S", (0, -11,  0.08), (100, 2, 0.16), concrete)

# Yellow centre-line dashes
for i in range(-6, 7):
    box(f"/World/City/Dash{i+6}", (i * 7, 0, 0.01), (4.0, 0.12, 0.02), yellow, collide=False)

# White edge lines
box("/World/City/Edge_N", (0,  9.9, 0.01), (100, 0.10, 0.02), white, collide=False)
box("/World/City/Edge_S", (0, -9.9, 0.01), (100, 0.10, 0.02), white, collide=False)


# ── BUILDINGS ─────────────────────────────────────────────────────────────────
# (centre_x, centre_y, width, depth, height)  — all in metres
BUILDINGS = [
    ( 18,  20,  9, 12, 26),
    ( 18, -22,  8, 10, 18),
    (-20,  20, 11,  8, 32),
    (-20, -22,  9,  9, 22),
    ( 34,  18, 10, 14, 14),
    (-34, -18, 12, 10, 28),
    (  8,  36, 14, 11, 38),
    ( -8, -36, 10, 12, 20),
    ( 30,  30,  8,  8, 22),
    (-30, -30,  8,  8, 16),
    ( 24, -34,  9,  9, 24),
    (-36,  24,  8, 12, 30),
    ( 42, -10, 10, 10, 18),
    (-42,  10, 10, 10, 24),
]
for i, (bx, by, bw, bd, bh) in enumerate(BUILDINGS):
    box(f"/World/City/B{i}/Wall", (bx, by, bh / 2), (bw, bd, bh), concrete)
    # Horizontal glass window bands every 4 floors
    for f in range(3, int(bh) - 1, 4):
        box(f"/World/City/B{i}/Win{f}",
            (bx, by, f + 1.0), (bw + 0.02, bd + 0.02, 1.8),
            glass, collide=False)


# ── CAR ──────────────────────────────────────────────────────────────────────
# Wheel geometry: radius WHL_R, axis=Y (rolls in X direction).
# Wheel centre is WHL_OFFSET_Z below the car root xform.
# Car root z = WHL_R + WHL_OFFSET_Z so that wheel bottoms touch z=0 (road).
WHL_R        = 0.38
WHL_OFFSET_Z = -(WHL_R - 0.05)          # -0.33 m below car root
CAR_Z        = WHL_R + abs(WHL_OFFSET_Z) # = 0.71 m

# Compound rigid body: RigidBodyAPI on root xform, CollisionAPI on children.
car = UsdGeom.Xform.Define(stage, "/World/Car")
UsdGeom.Xformable(car).AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, CAR_Z))
UsdPhysics.RigidBodyAPI.Apply(car.GetPrim())
UsdPhysics.MassAPI.Apply(car.GetPrim()).CreateMassAttr().Set(1500.0)

# Low, wide chassis
box("/World/Car/Chassis", (0, 0, 0), (2.30, 1.05, 0.38), car_red)

# Narrower passenger cabin on top
box("/World/Car/Cabin",   (-0.15, 0, 0.53), (1.35, 0.90, 0.42), car_red)

# Glazing (visual only — no collision)
box("/World/Car/Windshield", ( 0.62, 0, 0.53), (0.04, 0.88, 0.40), car_glass, collide=False)
box("/World/Car/RearGlass",  (-0.97, 0, 0.53), (0.04, 0.86, 0.38), car_glass, collide=False)

# Four wheels as cylinders attached to the same rigid body
for name, wx, wy in [("FL",  1.15, -1.08), ("FR",  1.15,  1.08),
                      ("RL", -1.15, -1.08), ("RR", -1.15,  1.08)]:
    w = UsdGeom.Cylinder.Define(stage, f"/World/Car/Whl_{name}")
    w.CreateRadiusAttr(WHL_R)
    w.CreateHeightAttr(0.24)
    w.CreateAxisAttr("Y")
    UsdGeom.Xformable(w).AddTranslateOp().Set(Gf.Vec3d(wx, wy, WHL_OFFSET_Z))
    UsdShade.MaterialBindingAPI(w.GetPrim()).Bind(rubber)
    UsdPhysics.CollisionAPI.Apply(w.GetPrim())


# ── STREET LIGHTS ─────────────────────────────────────────────────────────────
for i, (lx, ly) in enumerate([
    ( 10,  10.5), (-10,  10.5), ( 25,  10.5), (-25,  10.5),
    ( 10, -10.5), (-10, -10.5), ( 25, -10.5), (-25, -10.5),
]):
    b    = f"/World/City/SL{i}"
    sign = 1.0 if ly > 0 else -1.0
    arm_y = ly + sign * 1.0
    tip_y = ly + sign * 1.6

    cyl(b + "/Pole", (lx, ly, 4.0), 0.08, 8.0, "Z", steel)
    box(b + "/Arm",  (lx, arm_y, 8.10), (0.10, 2.0, 0.10), steel, collide=False)
    box(b + "/Head", (lx, tip_y, 7.94), (0.55, 0.20, 0.12), lamp_mat, collide=False)

    pt = UsdLux.SphereLight.Define(stage, b + "/Light")
    pt.CreateRadiusAttr(0.12)
    pt.CreateIntensityAttr(4000)
    pt.CreateColorAttr(Gf.Vec3f(1.0, 0.95, 0.75))
    UsdGeom.Xformable(pt).AddTranslateOp().Set(Gf.Vec3d(lx, tip_y, 7.85))


# ── TREES ─────────────────────────────────────────────────────────────────────
for i, (tx, ty) in enumerate([
    ( 13,  11), (-13,  11),
    ( 13, -11), (-13, -11),
    ( 28,  11), (-28, -11),
]):
    t = f"/World/City/Tree{i}"
    cyl(t + "/Trunk", (tx, ty, 1.25), 0.14, 2.5, "Z", bark)
    for j, (tz, fs) in enumerate([(2.8, 2.8), (3.9, 2.1), (4.9, 1.4)]):
        box(t + f"/F{j}", (tx, ty, tz), (fs, fs, 1.2), foliage, collide=False)


# ── VIEWPORT CAMERA ──────────────────────────────────────────────────────────
# Angled view looking toward the car from 45°
cam = UsdGeom.Camera.Define(stage, "/World/Camera")
cam.CreateFocalLengthAttr(28.0)
cam_xf = UsdGeom.Xformable(cam)
cam_xf.AddTranslateOp().Set(Gf.Vec3d(12.0, -14.0, 6.5))
cam_xf.AddRotateXYZOp().Set(Gf.Vec3d(-22.0, 0.0, 42.0))

try:
    from omni.kit.viewport.utility import get_active_viewport
    get_active_viewport().camera_path = "/World/Camera"
except Exception:
    pass


# ── SIMULATION LOOP ───────────────────────────────────────────────────────────
from isaacsim.core.api import World

world = World()
world.reset()

while simulation_app.is_running():
    world.step(render=True)

simulation_app.close()
