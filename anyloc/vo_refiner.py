"""
Frame-to-frame Visual Odometry for nadir drone camera.

Tracks Shi-Tomasi corner features with Lucas-Kanade optical flow and
converts median pixel displacement to a ground-plane lat/lon delta.

Coordinate convention (yaw = 0, north-pointing camera):
    image +x = east    image +y = south
    raw_east  = -median_dx_px × m_per_px_x   (feature right  → drone moved west)
    raw_north = +median_dy_px × m_per_px_y   (feature down   → drone moved north)

For non-zero yaw, raw (east, north) is rotated to world ENU:
    east  =  raw_east × cos(yaw) + raw_north × sin(yaw)
    north = -raw_east × sin(yaw) + raw_north × cos(yaw)

Derivation is analytic; verify signs empirically by flying in a known
direction and confirming the reported delta matches ground truth.
"""

import math
import cv2
import torch
from PIL import Image

_LK_PARAMS = dict(
    winSize=(21, 21),
    maxLevel=3,
    criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01),
)
_FEAT_PARAMS = dict(maxCorners=300, qualityLevel=0.01, minDistance=8, blockSize=7)
_MIN_PTS     = 8    # re-detect features when tracked count falls below this

_COS_LAT = math.cos(math.radians(23.450868))   # Chiayi, Taiwan


class VORefiner:
    """
    Accumulates LK optical-flow deltas between AnyLoc re-anchors.

    Typical usage inside the localizer loop:
        vo = VORefiner()
        dlat, dlon, n = vo.update(frame_pil, agl_m, yaw_deg)
        accum_dlat += dlat
        accum_dlon += dlon
        ...
        # after AnyLoc re-anchor:
        vo.reset()
        accum_dlat = accum_dlon = 0.0
    """

    def __init__(self, cam_w: int = 1640, cam_h: int = 1232,
                 hfov_deg: float = 62.2, vfov_deg: float = 48.8):
        self._cam_w    = cam_w
        self._cam_h    = cam_h
        self._hfov_deg = hfov_deg
        self._vfov_deg = vfov_deg
        self._prev_gray = None   # uint8 numpy (H, W)
        self._prev_pts  = None   # float32 numpy (N, 1, 2)

    # ── helpers ────────────────────────────────────────────────────────────────

    def _m_per_px(self, agl_m: float):
        mx = 2.0 * agl_m * math.tan(math.radians(self._hfov_deg / 2.0)) / self._cam_w
        my = 2.0 * agl_m * math.tan(math.radians(self._vfov_deg / 2.0)) / self._cam_h
        return mx, my

    def _to_gray(self, pil_img: Image.Image):
        """PIL → uint8 numpy (H, W) via torch frombuffer — avoids np.array(pil)."""
        g = pil_img.convert('L').resize((self._cam_w, self._cam_h), Image.LANCZOS)
        return (torch.frombuffer(bytearray(g.tobytes()), dtype=torch.uint8)
                     .reshape(self._cam_h, self._cam_w)
                     .numpy())

    def _detect(self, gray):
        self._prev_pts = cv2.goodFeaturesToTrack(gray, **_FEAT_PARAMS)

    # ── public API ─────────────────────────────────────────────────────────────

    def reset(self):
        """Clear tracked state after an AnyLoc re-anchor."""
        self._prev_gray = None
        self._prev_pts  = None

    def update(self, frame_pil: Image.Image, agl_m: float, yaw_deg: float = 0.0):
        """
        Estimate drone displacement from the previous frame to this one.

        Returns:
            delta_lat  float   degrees latitude  (north positive)
            delta_lon  float   degrees longitude (east  positive)
            n_pts      int     tracked feature count; 0 on first call or failure
        """
        gray = self._to_gray(frame_pil)

        # First call or state was reset — detect features and return zero delta
        if self._prev_gray is None or self._prev_pts is None:
            self._prev_gray = gray
            self._detect(gray)
            return 0.0, 0.0, 0

        if len(self._prev_pts) < _MIN_PTS:
            self._prev_gray = gray
            self._detect(gray)
            return 0.0, 0.0, 0

        new_pts, status, _ = cv2.calcOpticalFlowPyrLK(
            self._prev_gray, gray, self._prev_pts, None, **_LK_PARAMS)

        ok   = status.flatten() == 1
        n_ok = sum(ok.tolist())   # .sum() hits broken numpy _methods.py — use tolist()

        if n_ok < _MIN_PTS:
            self._prev_gray = gray
            self._detect(gray)
            return 0.0, 0.0, 0

        displ = (new_pts - self._prev_pts)[ok].reshape(-1, 2)   # (N, 2) numpy

        # Median via torch — avoids broken numpy dispatch in isaac_sim_test env
        dx_px = float(torch.tensor(displ[:, 0].tolist()).median())
        dy_px = float(torch.tensor(displ[:, 1].tolist()).median())

        # Advance state; re-detect when few points remain
        self._prev_gray = gray
        if n_ok < _MIN_PTS * 2:
            self._detect(gray)
        else:
            self._prev_pts = new_pts[ok].reshape(-1, 1, 2)

        # Pixel displacement → ground metres (yaw = 0 convention)
        mx, my     = self._m_per_px(agl_m)
        raw_east   = -dx_px * mx
        raw_north  =  dy_px * my

        # Rotate to world ENU by drone heading
        yaw_r      = math.radians(yaw_deg)
        c, s       = math.cos(yaw_r), math.sin(yaw_r)
        east_m     =  raw_east * c + raw_north * s
        north_m    = -raw_east * s + raw_north * c

        delta_lat  = north_m / 111_320.0
        delta_lon  = east_m  / (111_320.0 * _COS_LAT)

        return delta_lat, delta_lon, n_ok
