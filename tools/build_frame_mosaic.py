#!/usr/bin/env python3
"""
Build a georeferenced visual mosaic from extract_frames.py output.

Reads <session_dir>/frames.csv (path,lat,lon,alt_amsl,alt_agl,heading_deg) — frames must
already be North-up (i.e. extracted with `extract_frames.py --rotate`), since this script does
no further rotation, only placement and scaling. Each frame's ground footprint is computed from
its AGL and the AP-IMX900's known FOV (same HFOV_DEG/VFOV_DEG constants as build_database.py /
anyloc/localizer.py), then pasted onto a local-ENU canvas at its GPS position. Compositing is
sequential alpha-over in flight/time order, with each frame's edges feathered (soft-blended
over --feather-px pixels, full opacity in the interior) — this is NOT true photogrammetric
orthorectification/feature-based registration (this camera has no gimbal, so any roll/pitch
skews the true ground footprint away from the flat-plate rectangle this script assumes, and
GPS/heading noise causes real frame-to-frame misalignment); feathering only softens the seams
between misaligned frames so overlapping flight legs read as one surface instead of a harsh
terraced/duplicated look — it does not correct the underlying misalignment.

No GDAL/rasterio in this environment, so georeferencing is done the dependency-free way:
a world file (.jgw/.pgw, standard 6-line affine transform) + a .prj (WGS84 WKT) next to the
raster — QGIS and most GIS tools auto-load these as a georeferenced layer without needing an
embedded-metadata format like GeoTIFF. mosaic_meta.json also records the bounds/resolution in
plain text for anything that just wants the numbers.

Usage:
    /home/jetson/venv/anyloc/bin/python3 tools/build_frame_mosaic.py field_data/survey33/ \\
        [--res 0.15] [--out field_data/survey33/mosaic.png]
"""
import argparse
import csv
import json
import math
import os

import cv2
import numpy as np

# Drone camera: AP-IMX900 4mm CS-mount, computed — same constants as anyloc/build_database.py and anyloc/localizer.py
HFOV_DEG = 59.9
VFOV_DEG = 46.7

NODATA_FILL = (60, 60, 60)  # dark gray, visually distinct from real (usually darker/brighter) image content

PRJ_WGS84 = (
    'GEOGCS["GCS_WGS_1984",DATUM["D_WGS_1984",SPHEROID["WGS_1984",6378137.0,298.257223563]],'
    'PRIMEM["Greenwich",0.0],UNIT["Degree",0.0174532925199433]]'
)


def latlon_to_m(lat, lon, lat0, lon0):
    latm = 111132.954 - 559.822 * math.cos(2 * math.radians(lat0))
    lonm = 111412.84 * math.cos(math.radians(lat0)) - 93.5 * math.cos(3 * math.radians(lat0))
    x = (lon - lon0) * lonm
    y = (lat - lat0) * latm
    return x, y, latm, lonm


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('session_dir', help='e.g. field_data/survey33')
    ap.add_argument('--res', type=float, default=0.15, help='mosaic resolution, metres/pixel (default 0.15)')
    ap.add_argument('--feather-px', type=int, default=25,
                    help='feather width in canvas pixels at each frame edge (default 25, '
                         '~3.75m at default --res) — softens seams between overlapping frames')
    ap.add_argument('--out', default='', help='output raster path (default <session_dir>/mosaic.png)')
    args = ap.parse_args()

    session_dir = args.session_dir.rstrip('/')
    frames_csv = os.path.join(session_dir, 'frames.csv')
    out_path = args.out or os.path.join(session_dir, 'mosaic.png')

    print(f"[MOSAIC] Reading {frames_csv} ...")
    rows = []
    with open(frames_csv) as f:
        for r in csv.DictReader(f):
            rows.append(dict(
                path=r['path'], lat=float(r['lat']), lon=float(r['lon']),
                agl=float(r['alt_agl']),
            ))
    if not rows:
        raise SystemExit("no frames in frames.csv")
    print(f"  {len(rows)} frames")

    lat0, lon0 = rows[0]['lat'], rows[0]['lon']
    for r in rows:
        x, y, latm, lonm = latlon_to_m(r['lat'], r['lon'], lat0, lon0)
        r['x'], r['y'] = x, y
        r['half_w'] = r['agl'] * math.tan(math.radians(HFOV_DEG / 2.0))
        r['half_h'] = r['agl'] * math.tan(math.radians(VFOV_DEG / 2.0))

    xmin = min(r['x'] - r['half_w'] for r in rows)
    xmax = max(r['x'] + r['half_w'] for r in rows)
    ymin = min(r['y'] - r['half_h'] for r in rows)
    ymax = max(r['y'] + r['half_h'] for r in rows)
    print(f"  local-ENU bounds: x=[{xmin:.1f},{xmax:.1f}]m  y=[{ymin:.1f},{ymax:.1f}]m "
          f"({xmax-xmin:.1f} x {ymax-ymin:.1f} m)")

    res = args.res
    canvas_w = int(math.ceil((xmax - xmin) / res))
    canvas_h = int(math.ceil((ymax - ymin) / res))
    print(f"  canvas: {canvas_w} x {canvas_h} px @ {res} m/px "
          f"({canvas_w*canvas_h*3/1e6:.1f} MB uncompressed)")
    canvas_f = np.full((canvas_h, canvas_w, 3), NODATA_FILL, dtype=np.float32)
    feather_px = args.feather_px

    print(f"[MOSAIC] Pasting frames (flight order, feathered alpha-over, feather={feather_px}px) ...")
    for i, r in enumerate(rows):
        img = cv2.imread(os.path.join(session_dir, r['path']))
        if img is None:
            print(f"  [!] skip unreadable {r['path']}")
            continue
        fw_px = max(1, int(round(2 * r['half_w'] / res)))
        fh_px = max(1, int(round(2 * r['half_h'] / res)))
        img_r = cv2.resize(img, (fw_px, fh_px), interpolation=cv2.INTER_AREA)

        # extract_frames.py --rotate warps each frame to North-up, leaving black corner
        # triangles where the rotated content doesn't reach the original frame edge -- exclude
        # those near-black border pixels from the valid region entirely (distance-transform
        # below then also feathers the frame's OWN true (non-rectangular) content edge, not
        # just the canvas-crop edge).
        valid_u8 = (img_r.astype(np.int32).sum(axis=2) > 15).astype(np.uint8) * 255
        dist = cv2.distanceTransform(valid_u8, cv2.DIST_L2, 5)
        alpha_full = np.clip(dist / max(feather_px, 1), 0.0, 1.0).astype(np.float32)

        col0 = int(round((r['x'] - r['half_w'] - xmin) / res))
        row0 = int(round((ymax - (r['y'] + r['half_h'])) / res))  # row 0 = north (top), matches
                                                                    # build_database.py's NW-corner convention
        col1, row1 = col0 + fw_px, row0 + fh_px

        sc0, sr0 = max(0, -col0), max(0, -row0)
        dc0, dr0 = max(0, col0), max(0, row0)
        dc1, dr1 = min(canvas_w, col1), min(canvas_h, row1)
        if dc1 <= dc0 or dr1 <= dr0:
            continue
        sc1, sr1 = sc0 + (dc1 - dc0), sr0 + (dr1 - dr0)
        src = img_r[sr0:sr1, sc0:sc1][:, :, ::-1].astype(np.float32)  # BGR (cv2) -> RGB (canvas)
        alpha = alpha_full[sr0:sr1, sc0:sc1][:, :, None]

        # sequential "over" compositing: later (in flight order) frames blend on top of
        # earlier ones with a soft edge, instead of a hard last-write-wins cut or a full
        # running average (which would blur/ghost the ~80%+ overlap between consecutive
        # same-leg frames into mush) -- each frame stays dominant in its own interior.
        dst = canvas_f[dr0:dr1, dc0:dc1]
        canvas_f[dr0:dr1, dc0:dc1] = dst * (1.0 - alpha) + src * alpha

        if (i + 1) % 20 == 0 or i == len(rows) - 1:
            print(f"  {i+1}/{len(rows)} frames pasted")

    canvas = np.clip(canvas_f, 0, 255).astype(np.uint8)
    cv2.imwrite(out_path, canvas[:, :, ::-1])  # RGB canvas -> BGR for cv2.imwrite
    print(f"[MOSAIC] Saved {out_path}")

    # -- georeferencing: world file + .prj (no GDAL available in this environment) --
    latm = 111132.954 - 559.822 * math.cos(2 * math.radians(lat0))
    lonm = 111412.84 * math.cos(math.radians(lat0)) - 93.5 * math.cos(3 * math.radians(lat0))
    a = res / lonm            # deg/px, +x (east)
    e = -res / latm           # deg/px, -y (south, since row increases downward/south)
    c_lon = lon0 + (xmin / lonm) + a / 2.0   # centre of upper-left pixel
    c_lat = lat0 + (ymax / latm) + e / 2.0

    base, _ = os.path.splitext(out_path)
    ext = os.path.splitext(out_path)[1].lower()
    wld_ext = {'.png': '.pgw', '.jpg': '.jgw', '.jpeg': '.jgw', '.tif': '.tfw', '.tiff': '.tfw'}.get(ext, '.wld')
    wld_path = base + wld_ext
    with open(wld_path, 'w') as f:
        f.write(f"{a:.12f}\n0.0\n0.0\n{e:.12f}\n{c_lon:.10f}\n{c_lat:.10f}\n")
    prj_path = base + '.prj'
    with open(prj_path, 'w') as f:
        f.write(PRJ_WGS84)
    print(f"[MOSAIC] World file -> {wld_path}")
    print(f"[MOSAIC] CRS (.prj, WGS84) -> {prj_path}")
    print("  NOTE: not a true embedded-metadata GeoTIFF (no GDAL/rasterio in this env) — this is "
          "a plain raster + sidecar world file/.prj, the dependency-free equivalent most GIS "
          "tools (QGIS etc.) load as a georeferenced layer automatically.")

    meta = dict(
        session_dir=session_dir, n_frames=len(rows), resolution_m_per_px=res,
        canvas_w=canvas_w, canvas_h=canvas_h,
        bounds=dict(nw_lat=lat0 + ymax / latm, nw_lon=lon0 + xmin / lonm,
                    se_lat=lat0 + ymin / latm, se_lon=lon0 + xmax / lonm),
        compositing=f"sequential alpha-over (frames.csv/flight order), feather={feather_px}px at each frame edge",
        feather_px=feather_px,
        nodata_fill_rgb=list(NODATA_FILL),
        hfov_deg=HFOV_DEG, vfov_deg=VFOV_DEG,
    )
    meta_path = base + '_meta.json'
    with open(meta_path, 'w') as f:
        json.dump(meta, f, indent=2)
    print(f"[MOSAIC] Meta -> {meta_path}")


if __name__ == '__main__':
    main()
