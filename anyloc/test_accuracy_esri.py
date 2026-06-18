#!/usr/bin/env python3
"""
AnyLoc accuracy benchmark using Esri World Imagery as ground truth.
No API key required.

For each test point (true_lat, true_lon, agl_m):
  1. Fetch Esri World Imagery tiles, stitch and crop to the drone nadir footprint.
  2. Run AnyLoc localizer on that image.
  3. Compute Euclidean distance (metres) between the true and estimated position.
  4. Report per-sample results and aggregate statistics.

Usage:
    conda run -n isaac_sim_test python anyloc/test_accuracy_bing.py \\
        [--samples 20] [--agl 80] [--seed 42] [--output results.json] [--plot]
"""

import argparse
import io
import json
import math
import os
import random
import sys
import time

import requests
from PIL import Image

HERE   = os.path.dirname(os.path.abspath(__file__))
DB_DIR = os.path.join(HERE, 'database')

# Esri World Imagery — key-free XYZ tile service
ESRI_TILE_URL = (
    "https://server.arcgisonline.com/ArcGIS/rest/services"
    "/World_Imagery/MapServer/tile/{z}/{y}/{x}"
)
TILE_PX = 256   # Esri tile size in pixels

# ── Scene constants (must match cesium_scene.py / localizer.py) ───────────────
CENTER_LAT = 23.450868
CENTER_LON = 120.286135
RADIUS_M   = 2000.0
COS_LAT    = math.cos(math.radians(CENTER_LAT))

HFOV_DEG = 90.0    # drone camera horizontal FOV
VFOV_DEG = 73.7    # drone camera vertical FOV


# ── Tile math (Web Mercator, same convention as build_database.py) ─────────────

def _deg2tile(lat, lon, z):
    """(lat, lon) → (tile_x, tile_y) at zoom z.  y=0 at north."""
    n   = 1 << z
    tx  = int((lon + 180.0) / 360.0 * n)
    lr  = math.log(math.tan(math.radians(lat)) + 1.0 / math.cos(math.radians(lat)))
    ty  = int((1.0 - lr / math.pi) / 2.0 * n)
    return tx, ty


def _tile2deg(tx, ty, z):
    """Return the (lat, lon) of the NW corner of tile (tx, ty) at zoom z."""
    n   = 1 << z
    lon = tx / n * 360.0 - 180.0
    lat = math.degrees(math.atan(math.sinh(math.pi * (1.0 - 2.0 * ty / n))))
    return lat, lon


def _zoom_for_agl(agl_m, lat_deg, img_w=640):
    """
    Return the integer zoom whose ground resolution matches the drone footprint.

    m_per_px_camera = 2 * AGL * tan(HFOV/2) / img_w
    m_per_px_tile   = 156543.03392 * cos(lat) / 2^z   (Web-Mercator)
    Solve for z, round, clamp to [17, 20].
    """
    footprint_w = 2.0 * agl_m * math.tan(math.radians(HFOV_DEG / 2.0))
    m_per_px    = footprint_w / img_w
    z = math.log2(156_543.03392 * math.cos(math.radians(lat_deg)) / m_per_px)
    return max(17, min(20, int(round(z))))


# ── Esri tile fetcher ──────────────────────────────────────────────────────────

_tile_cache: dict = {}   # (z, tx, ty) → PIL Image, shared across all samples


def _is_blank_tile(tile: Image.Image, std_threshold: float = 8.0) -> bool:
    """Return True if the tile is a gray 'Map data not yet available' placeholder."""
    import numpy as np
    arr = np.array(tile, dtype=np.float32)
    return float(arr.std()) < std_threshold


def _fetch_tile(z, tx, ty, retries=3):
    """Download one Esri World Imagery tile, with in-memory cache."""
    key = (z, tx, ty)
    if key in _tile_cache:
        return _tile_cache[key]

    url = ESRI_TILE_URL.format(z=z, y=ty, x=tx)
    for attempt in range(retries):
        try:
            resp = requests.get(url, timeout=15,
                                headers={'User-Agent': 'AnyLocAccuracyTest/1.0'})
            resp.raise_for_status()
            tile = Image.open(io.BytesIO(resp.content)).convert('RGB')
            _tile_cache[key] = tile
            return tile
        except Exception as exc:
            if attempt == retries - 1:
                raise RuntimeError(f"Esri tile ({z}/{ty}/{tx}) failed: {exc}") from exc
            time.sleep(0.5 * (attempt + 1))


def fetch_esri_image(lat, lon, agl_m, img_w=640, img_h=480):
    """
    Build a nadir-view image centred at (lat, lon) from Esri World Imagery tiles.

    The zoom level is chosen so the pixel resolution matches the drone camera
    footprint at agl_m.  If tiles at the computed zoom are gray 'not available'
    placeholders, the zoom is decremented until real imagery is found (min zoom 14).
    Returns (PIL RGB Image, zoom).
    """
    zoom = _zoom_for_agl(agl_m, lat, img_w)
    min_zoom = 14

    # Footprint half-extents in degrees
    half_w_m = agl_m * math.tan(math.radians(HFOV_DEG / 2.0))
    half_h_m = agl_m * math.tan(math.radians(VFOV_DEG / 2.0))
    d_lat    = half_h_m / 111_320.0
    d_lon    = half_w_m / (111_320.0 * COS_LAT)

    north, south = lat + d_lat, lat - d_lat
    west,  east  = lon - d_lon, lon + d_lon

    while zoom >= min_zoom:
        # Tile range covering the footprint (NW → SE)
        tx_min, ty_min = _deg2tile(north, west, zoom)
        tx_max, ty_max = _deg2tile(south, east, zoom)

        # Download and stitch
        nx = tx_max - tx_min + 1
        ny = ty_max - ty_min + 1
        mosaic = Image.new('RGB', (nx * TILE_PX, ny * TILE_PX))
        blank_count = 0
        for tx in range(tx_min, tx_max + 1):
            for ty in range(ty_min, ty_max + 1):
                tile = _fetch_tile(zoom, tx, ty)
                if _is_blank_tile(tile):
                    blank_count += 1
                mosaic.paste(tile, ((tx - tx_min) * TILE_PX, (ty - ty_min) * TILE_PX))

        total_tiles = nx * ny
        if blank_count > total_tiles // 2:
            print(f"  [zoom {zoom}] {blank_count}/{total_tiles} blank tiles — trying zoom {zoom - 1}")
            # Evict blank tiles from cache so they are re-fetched at the new zoom
            for tx in range(tx_min, tx_max + 1):
                for ty in range(ty_min, ty_max + 1):
                    _tile_cache.pop((zoom, tx, ty), None)
            zoom -= 1
            continue

        # Geographic extent of the mosaic
        nw_lat, nw_lon = _tile2deg(tx_min,     ty_min,     zoom)
        se_lat, se_lon = _tile2deg(tx_max + 1, ty_max + 1, zoom)
        lon_span = se_lon - nw_lon
        lat_span = nw_lat - se_lat   # positive (north − south)
        mw, mh   = mosaic.size

        # Crop to the exact footprint
        x1 = int((west  - nw_lon) / lon_span * mw)
        x2 = int((east  - nw_lon) / lon_span * mw)
        y1 = int((nw_lat - north) / lat_span * mh)
        y2 = int((nw_lat - south) / lat_span * mh)
        x1, y1 = max(0, x1), max(0, y1)
        x2, y2 = min(mw, x2), min(mh, y2)

        if x2 - x1 < 4 or y2 - y1 < 4:
            raise RuntimeError(f"Footprint crop too small at zoom {zoom}: "
                               f"({x1},{y1})→({x2},{y2}) in {mw}×{mh} mosaic")

        return mosaic.crop((x1, y1, x2, y2)).resize((img_w, img_h), Image.LANCZOS), zoom

    raise RuntimeError(f"No Esri imagery available for ({lat:.5f}, {lon:.5f}) "
                       f"down to zoom {min_zoom}")


# ── Geo helpers ────────────────────────────────────────────────────────────────

def euclidean_m(lat1, lon1, lat2, lon2):
    """Flat-earth Euclidean distance in metres between two (lat, lon) points."""
    dlat_m = (lat1 - lat2) * 111_320.0
    dlon_m = (lon1 - lon2) * 111_320.0 * COS_LAT
    return math.sqrt(dlat_m ** 2 + dlon_m ** 2)


def random_point_in_circle(center_lat, center_lon, max_r_m, min_r_m=0.0, rng=None):
    """Uniform random (lat, lon) within an annulus [min_r_m, max_r_m] of centre."""
    rng   = rng or random
    angle = rng.uniform(0, 2 * math.pi)
    r_m   = math.sqrt(rng.uniform(min_r_m ** 2, max_r_m ** 2))
    lat   = center_lat + (r_m * math.sin(angle)) / 111_320.0
    lon   = center_lon + (r_m * math.cos(angle)) / (111_320.0 * COS_LAT)
    return lat, lon


# ── Statistics ─────────────────────────────────────────────────────────────────

def _stats(values):
    n    = len(values)
    mean = sum(values) / n
    var  = sum((v - mean) ** 2 for v in values) / n
    rmse = math.sqrt(sum(v ** 2 for v in values) / n)
    s    = sorted(values)
    med  = s[n // 2] if n % 2 else (s[n // 2 - 1] + s[n // 2]) / 2
    return dict(n=n, mean=mean, median=med, std=math.sqrt(var),
                rmse=rmse, min=min(values), max=max(values))


# ── Live viewport ─────────────────────────────────────────────────────────────

class _Viewport:
    """
    Live side-by-side window updated after every test sample.
    Left  — Esri World Imagery at the true coordinate (validation image).
    Right — AnyLoc database match at the estimated coordinate.
    """

    def __init__(self, n_samples):
        import matplotlib
        matplotlib.use('TkAgg')
        import matplotlib.pyplot as plt
        self._plt = plt

        self.fig, (self.ax_val, self.ax_match) = plt.subplots(
            1, 2, figsize=(12, 5.5))
        self.fig.canvas.manager.set_window_title('AnyLoc Accuracy Test')
        self.fig.suptitle(
            f'AnyLoc Accuracy Test — Esri World Imagery  |  0 / {n_samples}',
            fontsize=11)
        for ax in (self.ax_val, self.ax_match):
            ax.axis('off')
        plt.tight_layout()
        plt.show(block=False)
        plt.pause(0.01)
        self._n = n_samples

    def update(self, idx, val_img, match_img,
               true_lat, true_lon, est_lat, est_lon,
               err_m, score, agl_m):
        plt = self._plt

        err_color = ('limegreen' if err_m < 100
                     else 'orange' if err_m < 250 else 'tomato')

        self.ax_val.clear()
        self.ax_val.imshow(val_img)
        self.ax_val.set_title(
            f"Esri validation image  [{idx} / {self._n}]\n"
            f"True:  {true_lat:.6f},  {true_lon:.6f}   AGL {agl_m:.0f} m",
            fontsize=9)
        self.ax_val.axis('off')

        self.ax_match.clear()
        self.ax_match.imshow(match_img)
        self.ax_match.set_title(
            f"AnyLoc match\n"
            f"Est:  {est_lat:.6f},  {est_lon:.6f}   "
            f"Err: {err_m:.1f} m   Score: {score:.3f}",
            fontsize=9, color=err_color)
        self.ax_match.axis('off')

        self.fig.suptitle(
            f'AnyLoc Accuracy Test — Esri World Imagery  |  {idx} / {self._n}',
            fontsize=11)
        plt.tight_layout()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def close(self):
        self._plt.ioff()


# ── Benchmark ──────────────────────────────────────────────────────────────────

def run_benchmark(n_samples, agl_m, seed, output_path, plot, show_viewport):
    rng = random.Random(seed)

    sys.path.insert(0, os.path.dirname(HERE))
    from anyloc.localizer import AnyLocLocalizer

    print(f"\n{'='*62}")
    print(f"  AnyLoc Accuracy Benchmark — Esri World Imagery")
    print(f"  Samples : {n_samples}  |  AGL : {agl_m if agl_m > 0 else 'random 60-120'} m"
          f"  |  Seed : {seed}")
    print(f"{'='*62}\n")

    print("[1/3] Loading AnyLoc database and DINOv2 model …")
    loc = AnyLocLocalizer(DB_DIR)
    print()

    print("[2/3] Generating test points …")
    test_points = []
    for i in range(n_samples):
        lat, lon = random_point_in_circle(
            CENTER_LAT, CENTER_LON,
            max_r_m=RADIUS_M * 0.85,
            min_r_m=50.0,
            rng=rng,
        )
        h = agl_m if agl_m > 0 else rng.choice([60, 70, 80, 90, 100, 110, 120])
        test_points.append(dict(idx=i + 1, true_lat=lat, true_lon=lon, agl_m=h))
    print(f"  {n_samples} points inside {RADIUS_M * 0.85:.0f} m radius\n")

    print("[3/3] Fetching imagery and running localizer …\n")
    header  = f"  {'#':>3}  {'True lat':>10}  {'True lon':>11}  "
    header += f"{'Est lat':>10}  {'Est lon':>11}  {'Err (m)':>8}  {'Score':>6}  AGL"
    print(header)
    print("  " + "-" * (len(header) - 2))

    results  = []
    errors_m = []

    viewport = None
    if show_viewport:
        try:
            viewport = _Viewport(n_samples)
        except Exception as exc:
            print(f"  [viewport] could not open window: {exc}")

    for pt in test_points:
        i, true_lat, true_lon, h = pt['idx'], pt['true_lat'], pt['true_lon'], pt['agl_m']

        # ── Fetch Esri tiles ──────────────────────────────────────────────────
        try:
            img, zoom = fetch_esri_image(true_lat, true_lon, h)
        except RuntimeError as exc:
            print(f"  {i:>3}  [SKIP] {exc}")
            continue

        # ── AnyLoc localization ───────────────────────────────────────────────
        t0 = time.perf_counter()
        est_lat, est_lon, _, match_img, score, db_idx = loc.localize(img, agl_m=h)
        elapsed = time.perf_counter() - t0

        # ── Euclidean error in metres ─────────────────────────────────────────
        err_m = euclidean_m(true_lat, true_lon, est_lat, est_lon)
        errors_m.append(err_m)

        results.append(dict(
            idx=i,
            true_lat=true_lat, true_lon=true_lon,
            est_lat=est_lat,   est_lon=est_lon,
            agl_m=h, zoom=zoom,
            error_m=err_m, score=score,
            db_idx=db_idx, inference_s=round(elapsed, 3),
        ))

        print(f"  {i:>3}  {true_lat:>10.6f}  {true_lon:>11.6f}  "
              f"{est_lat:>10.6f}  {est_lon:>11.6f}  "
              f"{err_m:>8.1f}  {score:>6.3f}  {h} m")

        if viewport is not None:
            viewport.update(i, img, match_img,
                            true_lat, true_lon, est_lat, est_lon,
                            err_m, score, h)

    if viewport is not None:
        viewport.close()

    if not errors_m:
        print("\nNo successful localizations.")
        return

    # ── Print statistics ──────────────────────────────────────────────────────
    st = _stats(errors_m)
    print(f"\n{'='*62}")
    print(f"  Results  ({st['n']} / {n_samples} samples succeeded)")
    print(f"{'='*62}")
    print(f"  Mean error   : {st['mean']:>8.2f} m")
    print(f"  Median error : {st['median']:>8.2f} m")
    print(f"  RMSE         : {st['rmse']:>8.2f} m")
    print(f"  Std dev      : {st['std']:>8.2f} m")
    print(f"  Min error    : {st['min']:>8.2f} m")
    print(f"  Max error    : {st['max']:>8.2f} m")
    print(f"{'='*62}\n")

    # ── JSON output ───────────────────────────────────────────────────────────
    report = dict(
        config=dict(n_samples=n_samples, agl_m=agl_m, seed=seed,
                    imagery='Esri World Imagery',
                    center_lat=CENTER_LAT, center_lon=CENTER_LON,
                    radius_m=RADIUS_M),
        statistics=st,
        results=results,
    )
    if output_path:
        with open(output_path, 'w') as f:
            json.dump(report, f, indent=2)
        print(f"Results saved → {output_path}")

    if plot:
        _plot_results(results, st)

    return report


# ── Plot ───────────────────────────────────────────────────────────────────────

def _plot_results(results, st):
    try:
        import matplotlib
        matplotlib.use('TkAgg')
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not available — skipping plot.")
        return

    errors    = [r['error_m']  for r in results]
    true_lats = [r['true_lat'] for r in results]
    true_lons = [r['true_lon'] for r in results]
    est_lats  = [r['est_lat']  for r in results]
    est_lons  = [r['est_lon']  for r in results]

    fig, axes = plt.subplots(1, 2, figsize=(13, 5))
    fig.suptitle(
        f"AnyLoc Accuracy — Esri World Imagery  |  "
        f"N={st['n']}  mean={st['mean']:.1f} m  RMSE={st['rmse']:.1f} m",
        fontsize=12,
    )

    # Error histogram
    ax = axes[0]
    ax.hist(errors, bins=max(5, len(errors) // 3),
            color='steelblue', edgecolor='white', linewidth=0.5)
    ax.axvline(st['mean'],   color='red',    linestyle='--',
               label=f"Mean {st['mean']:.1f} m")
    ax.axvline(st['median'], color='orange', linestyle='--',
               label=f"Median {st['median']:.1f} m")
    ax.set_xlabel('Euclidean error (m)')
    ax.set_ylabel('Count')
    ax.set_title('Error distribution')
    ax.legend()

    # Spatial error map
    ax = axes[1]
    sc = ax.scatter(true_lons, true_lats, c=errors, cmap='RdYlGn_r',
                    s=60, zorder=3, label='True position')
    for r in results:
        ax.plot([r['true_lon'], r['est_lon']], [r['true_lat'], r['est_lat']],
                'k-', linewidth=0.6, alpha=0.4)
    ax.scatter(est_lons, est_lats, marker='x', s=40, color='navy',
               linewidths=0.8, zorder=4, label='AnyLoc estimate')
    ax.scatter([CENTER_LON], [CENTER_LAT], marker='*', s=150,
               color='gold', edgecolors='black', linewidths=0.5,
               zorder=5, label='Scene centre')
    fig.colorbar(sc, ax=ax).set_label('Error (m)')
    ax.set_xlabel('Longitude')
    ax.set_ylabel('Latitude')
    ax.set_title('Spatial error map')
    ax.legend(fontsize=8)

    plt.tight_layout()
    plt.show()


# ── CLI ────────────────────────────────────────────────────────────────────────

def main():
    global DB_DIR
    _default_db = DB_DIR   # snapshot before parse_args touches the global

    parser = argparse.ArgumentParser(
        description='AnyLoc accuracy test — Esri World Imagery, no API key required.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument('--samples', type=int, default=20,
                        help='Number of test points (default: 20)')
    parser.add_argument('--agl', type=float, default=80.0,
                        help='Drone AGL in metres (default: 80). '
                             'Pass 0 to randomise across [60, 120] m.')
    parser.add_argument('--seed', type=int, default=42,
                        help='Random seed for reproducible test points (default: 42)')
    parser.add_argument('--output', default='',
                        help='Save results to this JSON file (default: none)')
    parser.add_argument('--plot', action='store_true',
                        help='Show error histogram and spatial map after benchmark')
    parser.add_argument('--no-viewport', action='store_true',
                        help='Disable the live side-by-side image viewport')
    parser.add_argument('--db-dir', default=_default_db,
                        help=f'AnyLoc database directory (default: {_default_db})')
    args = parser.parse_args()

    DB_DIR = args.db_dir

    run_benchmark(
        n_samples=args.samples,
        agl_m=args.agl,
        seed=args.seed,
        output_path=args.output or None,
        plot=args.plot,
        show_viewport=not args.no_viewport,
    )


if __name__ == '__main__':
    main()
