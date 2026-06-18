"""
Projects USD-placed vehicles onto the nadir drone camera image plane.
Pure Python — no numpy, safe inside the isaac_sim_test conda env.

Camera intrinsics match cesium_scene.py exactly:
  focal_length=18 mm, h_aperture=36 mm, v_aperture=27 mm, 640×480
  fx = fy = 320 px  (square pixels)

Coordinate system: X=East, Y=North, Z=up.
Camera looks -Z (nadir); image U increases East, V increases South.
"""
import math

CAM_W  = 640
CAM_H  = 480
_FX    = CAM_W * 18.0 / 36.0   # 320
_FY    = CAM_H * 18.0 / 27.0   # 320
_CX    = CAM_W / 2              # 320
_CY    = CAM_H / 2              # 240

CLASS_CAR   = 0
CLASS_MOTO  = 1
CLASS_BUS   = 2
CLASS_TRUCK = 3

# (length_m, width_m) footprint per class
VEHICLE_DIMS = {
    CLASS_CAR:   (4.64, 1.775),
    CLASS_MOTO:  (2.20, 0.80),
    CLASS_BUS:   (12.0, 2.50),
    CLASS_TRUCK: (8.00, 2.40),
}


def _project_point(cam_xyz, point_xyz):
    """Return (u, v) pixel coords or None if point is above/at camera."""
    cx, cy, cz = cam_xyz
    px, py, pz = point_xyz
    depth = cz - pz
    if depth < 0.1:
        return None
    u = _CX + _FX * (px - cx) / depth
    v = _CY - _FY * (py - cy) / depth
    return u, v


def vehicle_label(cam_xyz, vehicle_xyz, yaw_deg, class_id):
    """
    Compute a YOLO-format label for one vehicle.

    Parameters
    ----------
    cam_xyz     : (cx, cy, cz) — camera ENU position in metres
    vehicle_xyz : (vx, vy, vz) — vehicle centre ENU
    yaw_deg     : compass heading of vehicle front (0=North, 90=East)
    class_id    : CLASS_CAR / CLASS_MOTO / CLASS_BUS / CLASS_TRUCK

    Returns
    -------
    (class_id, xc, yc, w, h) — all image coords normalised to [0,1],
    or None if the vehicle footprint lies entirely outside the frame.
    """
    length, width = VEHICLE_DIMS[class_id]
    half_l = length / 2.0
    half_w = width  / 2.0

    yr  = math.radians(yaw_deg)
    fwd = ( math.sin(yr),  math.cos(yr))   # forward unit vector (E, N)
    rgt = ( math.cos(yr), -math.sin(yr))   # right   unit vector

    vx, vy, vz = vehicle_xyz
    corners_xyz = [
        (vx + rgt[0]*sw + fwd[0]*sl,
         vy + rgt[1]*sw + fwd[1]*sl,
         vz)
        for sw in ( half_w, -half_w)
        for sl in ( half_l, -half_l)
    ]

    uvs = [_project_point(cam_xyz, c) for c in corners_xyz]
    uvs = [uv for uv in uvs if uv is not None]
    if not uvs:
        return None

    us = [uv[0] for uv in uvs]
    vs = [uv[1] for uv in uvs]
    u0 = max(0.0, min(us));  u1 = min(float(CAM_W), max(us))
    v0 = max(0.0, min(vs));  v1 = min(float(CAM_H), max(vs))

    if u1 <= u0 or v1 <= v0:
        return None

    xc = (u0 + u1) * 0.5 / CAM_W
    yc = (v0 + v1) * 0.5 / CAM_H
    w  = (u1 - u0) / CAM_W
    h  = (v1 - v0) / CAM_H
    return (class_id, xc, yc, w, h)


def write_label(path, labels):
    """
    Write a YOLO-format label file.

    Parameters
    ----------
    path   : str or Path — output .txt file
    labels : list of (class_id, xc, yc, w, h)
    """
    with open(path, "w") as f:
        for cls, xc, yc, w, h in labels:
            f.write(f"{cls} {xc:.6f} {yc:.6f} {w:.6f} {h:.6f}\n")
