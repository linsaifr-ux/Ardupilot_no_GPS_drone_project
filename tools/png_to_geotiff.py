#!/usr/bin/env python3
"""
Convert a raster + world file (.pgw/.jgw/.wld, as written by build_frame_mosaic.py or any
other tool using the same convention) into a real GeoTIFF, with embedded GeoTIFF tags
(ModelPixelScaleTag, ModelTiepointTag, GeoKeyDirectoryTag) readable by QGIS/gdalinfo/etc.
without needing a world file alongside it.

No GDAL/rasterio in this environment -- uses `tifffile` (pure Python + numpy, `pip install
tifffile`) to write the TIFF tags directly instead. Assumes the raster's CRS is WGS84
(EPSG:4326, plain lat/lon degrees) -- true for every world file this project currently
produces (see build_frame_mosaic.py) -- and that the world file has no rotation terms
(B=D=0), i.e. it's a plain north-up raster.

Usage:
    /home/jetson/venv/anyloc/bin/python3 tools/png_to_geotiff.py field_data/survey33/mosaic.png
        [--out field_data/survey33/mosaic.tif]
"""
import argparse
import os

import numpy as np
import tifffile
from PIL import Image

WORLD_EXT = {'.png': '.pgw', '.jpg': '.jgw', '.jpeg': '.jgw'}

# Minimal GeoTIFF GeoKeyDirectoryTag: geographic (lat/lon) model, pixel-is-area, WGS84 (EPSG:4326)
GEOKEY_DIRECTORY = [
    1, 1, 0, 3,          # KeyDirectoryVersion, KeyRevision, MinorRevision, NumberOfKeys
    1024, 0, 1, 2,        # GTModelTypeGeoKey = 2 (Geographic)
    1025, 0, 1, 1,        # GTRasterTypeGeoKey = 1 (RasterPixelIsArea)
    2048, 0, 1, 4326,      # GeographicTypeGeoKey = 4326 (WGS84)
]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('raster', help='e.g. field_data/survey33/mosaic.png')
    ap.add_argument('--out', default='', help='output .tif path (default: same basename, .tif)')
    args = ap.parse_args()

    base, ext = os.path.splitext(args.raster)
    wld_path = base + WORLD_EXT.get(ext.lower(), '.wld')
    out_path = args.out or base + '.tif'

    print(f"[GEOTIFF] Reading world file {wld_path} ...")
    with open(wld_path) as f:
        a, d, b, e, c_lon, c_lat = (float(x) for x in f.read().split())
    if d != 0.0 or b != 0.0:
        raise SystemExit("world file has rotation terms (B/D != 0) -- not supported, this "
                          "converter assumes a plain north-up raster")

    # World-file (a,e,c_lon,c_lat) describes the CENTRE of the upper-left pixel.
    # GeoTIFF's ModelTiepointTag at raster point (0,0) is the OUTER CORNER of that same pixel
    # -- shift back by half a pixel in each axis.
    corner_lon = c_lon - a / 2.0
    corner_lat = c_lat - e / 2.0
    print(f"  pixel size: {a:.10f} x {abs(e):.10f} deg/px")
    print(f"  upper-left corner: lat={corner_lat:.8f} lon={corner_lon:.8f}")

    print(f"[GEOTIFF] Reading raster {args.raster} ...")
    img = np.array(Image.open(args.raster).convert('RGB'))
    print(f"  {img.shape[1]} x {img.shape[0]} px")

    extratags = [
        (33550, 'd', 3, (a, abs(e), 0.0), True),                    # ModelPixelScaleTag
        (33922, 'd', 6, (0.0, 0.0, 0.0, corner_lon, corner_lat, 0.0), True),  # ModelTiepointTag
        (34735, 'H', len(GEOKEY_DIRECTORY), GEOKEY_DIRECTORY, True),  # GeoKeyDirectoryTag
    ]
    tifffile.imwrite(out_path, img, photometric='rgb', extratags=extratags)
    print(f"[GEOTIFF] Wrote {out_path}")


if __name__ == '__main__':
    main()
