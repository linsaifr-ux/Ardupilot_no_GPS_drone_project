#!/usr/bin/env python3
"""ENU frame for the imx900 flights (survey43/44/45).

These flights are at a DIFFERENT SITE from survey25/32/33 -- 22.5749 N versus
22.7775 N, about 22 km apart -- so survey33_geo_common's origin does not apply
and positions from the two sites are not comparable. There is also no mosaic
here: survey44 was flown to build a map but only the raw frames exist, so this
module carries no GeoTIFF at all, just the local tangent plane the frame map
and every evaluation share.

The origin is fixed as a literal, not computed from whichever flight happens to
be loaded, so survey43, survey44 and survey45 all land in one common frame.
"""
import math

# survey44's first airborne GPS fix, rounded. Fixed so every flight at this
# site shares one origin regardless of which is loaded first.
LAT0, LON0 = 22.574900, 120.550600

_LATM = 111132.954 - 559.822 * math.cos(2 * math.radians(LAT0))
_LONM = (111412.84 * math.cos(math.radians(LAT0))
         - 93.5 * math.cos(3 * math.radians(LAT0)))


def enu(lat, lon):
    """(east, north) metres from the shared imx900-site origin."""
    return (lon - LON0) * _LONM, (lat - LAT0) * _LATM


def enu_to_lonlat(e, n):
    return LON0 + e / _LONM, LAT0 + n / _LATM


# --- camera constants, measured in session_2026-08-12_imx900_vio.md ---------

# mpp = K_GSD * agl. From the Kalibr cam-only solve calib_20260812_124944
# (1873.6 +/- 4.4 px at 2048x1536), adopted 2026-08-13 after it cut survey43's
# VIO rmse from 3.40 m to 1.07 m. The earlier value was OpenVINS' own online
# estimate of 1790 px; the flight-derived wide-baseline measurement was
# 1738 +/- 218 px, which brackets both and does not discriminate.
K_GSD = 1.0 / 1873.6        # 5.337e-04 m/px per m AGL

# Radial/tangential distortion from the same Kalibr solve. Leaving this at zero
# is what made VIO diverge (3437 m vs 4.4 m rmse), and it bends a 65 m map tile
# by metres at the corners if ignored here too. Previous value was OpenVINS'
# online estimate (-0.422, 0.152, -0.002, 0.0).
DIST_K = (-0.48083899, 0.21935635, 0.00137044, -0.00241865)

# The camera is in the CANONICAL nadir orientation in the raw decoded video:
# image-up is aircraft-forward, image-right is aircraft-right (measured at
# +91.5 deg with 0.98 angular concentration over 40 wide-baseline pairs).
# meta.json still says "frame_rotation_deg": 180, but that describes the OLD
# 1640x1232 rig -- applying it here would put every tile 180 deg out.
FRAME_ROT_DEG = 0.0
