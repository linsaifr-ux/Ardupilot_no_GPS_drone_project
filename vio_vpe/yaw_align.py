#!/usr/bin/env python3
"""Online VIO->ENU yaw fit, for fusion_live_node.

OpenVINS' world frame is gravity-aligned but yaw-arbitrary (it starts wherever
the filter happened to initialise). Before VIO velocity can be fed into
FusionFilter.update_velocity() -- which expects NED/ENU-referenced n/e -- that
arbitrary yaw has to be resolved against something that IS geo-referenced.

Per the method doc (METHOD_imx900_vio_vpe.md Sec 8.3) and the plan, that
"something" is the VPE fixes, never GPS: using GPS to align a system meant to
run without GPS makes the live error-vs-GPS number circular and dishonest.
This module is the live, incremental version of the same idea --
~/openvins_ws/compare_vio_gps.py's yaw_align() does the identical closed-form
fit, but as a one-shot batch call over a whole recorded flight; here it runs
on a trailing window of (VIO xy, VPE xy) pairs, re-fit as new VPE fixes come
in, with a spatial-span gate before the result is trusted at all.

That gate exists because a time-only window can be almost pure climb: the doc
measured a 40s/3.8m window returning a yaw 160 degrees wrong on survey48. This
module gates on the actual spread of the VPE fixes seen so far, not on
elapsed time or fix count.

Known limitation: this assumes the VIO->world yaw offset is a single constant
for the flight, which is true only as long as OpenVINS never re-initialises.
A VIO re-init (e.g. after total feature-track loss) would silently invalidate
every pair collected before it; detecting that is not implemented here.
"""
import math
from collections import deque

import numpy as np

MIN_SPAN_M = 150.0   # doc's --yaw-fit-span default
MAX_PAIRS = 300       # ~5 min of VPE fixes at 1 Hz -- a trailing window, not
                       # an ever-growing batch


def _yaw_align_2d(src, dst):
    """4-DOF (yaw+translation) closed-form fit minimising |dst-(R@src+t)|.

    Identical math to ~/openvins_ws/compare_vio_gps.py's yaw_align(), reduced
    to 2D (that version embeds a 3x3 R with z untouched; here z/altitude is
    ArduPilot's EKF's job, never this filter's -- see fusion_filter.py's own
    "horizontal only, deliberately" comment).
    """
    src = np.asarray(src, dtype=float)
    dst = np.asarray(dst, dtype=float)
    ms, md = src.mean(0), dst.mean(0)
    s, d = src - ms, dst - md
    num = np.sum(s[:, 0] * d[:, 1] - s[:, 1] * d[:, 0])
    den = np.sum(s[:, 0] * d[:, 0] + s[:, 1] * d[:, 1])
    yaw = math.atan2(num, den)
    c, si = math.cos(yaw), math.sin(yaw)
    R = np.array([[c, -si], [si, c]])
    t = md - R @ ms
    return R, t, yaw


class OnlineYawAligner:
    """Buffers (vio_xy, vpe_xy) pairs and fits VIO->ENU yaw once they span
    enough ground to be trustworthy (see module docstring)."""

    def __init__(self, min_span_m=MIN_SPAN_M, max_pairs=MAX_PAIRS):
        self.min_span_m = float(min_span_m)
        self._vio = deque(maxlen=max_pairs)
        self._vpe = deque(maxlen=max_pairs)
        self._R = None
        self._t = None
        self._yaw = None

    def add_pair(self, vio_xy, vpe_xy):
        """Call once per accepted VPE fix, with the VIO position nearest that
        fix's timestamp and the VPE fix's own (east, north)."""
        self._vio.append(tuple(vio_xy))
        self._vpe.append(tuple(vpe_xy))
        if self._span_m() >= self.min_span_m and len(self._vpe) >= 4:
            self._R, self._t, self._yaw = _yaw_align_2d(list(self._vio), list(self._vpe))

    def _span_m(self):
        if len(self._vpe) < 2:
            return 0.0
        pts = np.asarray(self._vpe)
        return float(np.hypot(*(pts.max(0) - pts.min(0))))

    @property
    def resolved(self):
        return self._R is not None

    @property
    def yaw_deg(self):
        return math.degrees(self._yaw) if self._yaw is not None else None

    @property
    def span_m(self):
        return self._span_m()

    def rotate_velocity(self, vx_vio, vy_vio):
        """Rotation only (no translation) -- velocity is a direction, not a
        position. Inputs are VIO's own raw (vx,vy), which have no north/east
        meaning until rotated; the return value does. Caller must check
        .resolved first."""
        v = self._R @ np.array([vx_vio, vy_vio])
        return float(v[0]), float(v[1])

    def project_position(self, x_vio, y_vio):
        """Rotation + translation, for comparing VIO's own integrated
        position against GPS (the err_vio diagnostic in fusion_live_node) --
        not fed into FusionFilter itself, which only ever sees VIO velocity."""
        p = self._R @ np.array([x_vio, y_vio]) + self._t
        return float(p[0]), float(p[1])
