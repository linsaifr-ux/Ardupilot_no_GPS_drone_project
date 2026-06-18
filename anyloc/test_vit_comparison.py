#!/usr/bin/env python3
"""
Speed and accuracy comparison: DINOv2 ViT-B/14 vs ViT-S/14 for AnyLoc.

Speed test  — feature-extraction latency (ms/image) measured on Esri imagery.
              Runs using only the ViT-B database (always available).

Accuracy test — full localization accuracy + latency for both models on the
              same trajectory.  Requires a ViT-S database at:
                  anyloc/database_vits14/
              Build it first (takes ~same time as the ViT-B database):
                  conda run -n isaac_sim_test python anyloc/build_database.py \\
                      --model vits14

Usage:
    conda run -n isaac_sim_test python anyloc/test_vit_comparison.py \\
        [--steps 20] [--agl 80] [--seed 42] [--speed-only]
"""

import argparse
import io
import math
import os
import random
import sys
import time

import requests
from PIL import Image
import torch

HERE   = os.path.dirname(os.path.abspath(__file__))
DB_B   = os.path.join(HERE, 'database')
DB_S   = os.path.join(HERE, 'database_vits14')

CENTER_LAT = 23.450868
CENTER_LON = 120.286135
RADIUS_M   = 2000.0
COS_LAT    = math.cos(math.radians(CENTER_LAT))
HFOV_DEG   = 90.0
VFOV_DEG   = 73.7
TILE_PX    = 256

ESRI_TILE_URL = (
    "https://server.arcgisonline.com/ArcGIS/rest/services"
    "/World_Imagery/MapServer/tile/{z}/{y}/{x}"
)

# ── Tile helpers ──────────────────────────────────────────────────────────────

def _deg2tile(lat, lon, z):
    n  = 1 << z
    tx = int((lon + 180.0) / 360.0 * n)
    lr = math.log(math.tan(math.radians(lat)) + 1.0 / math.cos(math.radians(lat)))
    ty = int((1.0 - lr / math.pi) / 2.0 * n)
    return tx, ty

def _tile2deg(tx, ty, z):
    n   = 1 << z
    lon = tx / n * 360.0 - 180.0
    lat = math.degrees(math.atan(math.sinh(math.pi * (1.0 - 2.0 * ty / n))))
    return lat, lon

def _zoom_for_agl(agl_m, lat_deg, img_w=640):
    footprint_w = 2.0 * agl_m * math.tan(math.radians(HFOV_DEG / 2.0))
    m_per_px    = footprint_w / img_w
    z = math.log2(156_543.03392 * math.cos(math.radians(lat_deg)) / m_per_px)
    return max(17, min(20, int(round(z))))

_tile_cache: dict = {}

def _fetch_tile(z, tx, ty, retries=3):
    key = (z, tx, ty)
    if key in _tile_cache:
        return _tile_cache[key]
    url = ESRI_TILE_URL.format(z=z, y=ty, x=tx)
    for attempt in range(retries):
        try:
            resp = requests.get(url, timeout=15,
                                headers={'User-Agent': 'AnyLocVitCompare/1.0'})
            resp.raise_for_status()
            tile = Image.open(io.BytesIO(resp.content)).convert('RGB')
            _tile_cache[key] = tile
            return tile
        except Exception as exc:
            if attempt == retries - 1:
                raise RuntimeError(f"Esri tile ({z}/{ty}/{tx}) failed: {exc}") from exc
            time.sleep(0.5 * (attempt + 1))

def fetch_esri_image(lat, lon, agl_m, img_w=640, img_h=480):
    zoom     = _zoom_for_agl(agl_m, lat, img_w)
    min_zoom = 14
    half_w_m = agl_m * math.tan(math.radians(HFOV_DEG / 2.0))
    half_h_m = agl_m * math.tan(math.radians(VFOV_DEG / 2.0))
    d_lat    = half_h_m / 111_320.0
    d_lon    = half_w_m / (111_320.0 * COS_LAT)
    north, south = lat + d_lat, lat - d_lat
    west,  east  = lon - d_lon, lon + d_lon
    while zoom >= min_zoom:
        tx_min, ty_min = _deg2tile(north, west, zoom)
        tx_max, ty_max = _deg2tile(south, east, zoom)
        nx = tx_max - tx_min + 1
        ny = ty_max - ty_min + 1
        mosaic = Image.new('RGB', (nx * TILE_PX, ny * TILE_PX))
        import numpy as np
        blank_count = 0
        for tx in range(tx_min, tx_max + 1):
            for ty in range(ty_min, ty_max + 1):
                tile = _fetch_tile(zoom, tx, ty)
                if float(np.array(tile, dtype=np.float32).std()) < 8.0:
                    blank_count += 1
                mosaic.paste(tile, ((tx - tx_min) * TILE_PX, (ty - ty_min) * TILE_PX))
        if blank_count > (nx * ny) // 2:
            zoom -= 1
            continue
        nw_lat, nw_lon = _tile2deg(tx_min,     ty_min,     zoom)
        se_lat, se_lon = _tile2deg(tx_max + 1, ty_max + 1, zoom)
        lon_span = se_lon - nw_lon
        lat_span = nw_lat - se_lat
        mw, mh   = mosaic.size
        x1 = int((west  - nw_lon) / lon_span * mw)
        x2 = int((east  - nw_lon) / lon_span * mw)
        y1 = int((nw_lat - north) / lat_span * mh)
        y2 = int((nw_lat - south) / lat_span * mh)
        x1, y1 = max(0, x1), max(0, y1)
        x2, y2 = min(mw, x2), min(mh, y2)
        if x2 - x1 < 4 or y2 - y1 < 4:
            zoom -= 1
            continue
        return mosaic.crop((x1, y1, x2, y2)).resize((img_w, img_h), Image.LANCZOS)
    raise RuntimeError(f"No Esri imagery for ({lat:.5f}, {lon:.5f})")

# ── Geo helpers ────────────────────────────────────────────────────────────────

def euclidean_m(lat1, lon1, lat2, lon2):
    dlat_m = (lat1 - lat2) * 111_320.0
    dlon_m = (lon1 - lon2) * 111_320.0 * COS_LAT
    return math.sqrt(dlat_m ** 2 + dlon_m ** 2)

def _stats(values):
    n    = len(values)
    mean = sum(values) / n
    rmse = math.sqrt(sum(v ** 2 for v in values) / n)
    s    = sorted(values)
    med  = s[n // 2] if n % 2 else (s[n // 2 - 1] + s[n // 2]) / 2
    return dict(mean=mean, median=med, rmse=rmse, min=min(values), max=max(values))

# ── Trajectory ─────────────────────────────────────────────────────────────────

def _trajectory(n_steps, agl_m, seed):
    rng   = random.Random(seed)
    angle = rng.uniform(0, 2 * math.pi)
    r0    = rng.uniform(0.3, 0.7) * RADIUS_M
    r1    = rng.uniform(0.3, 0.7) * RADIUS_M
    lat0  = CENTER_LAT + r0 * math.sin(angle)            / 111_320.0
    lon0  = CENTER_LON + r0 * math.cos(angle)            / (111_320.0 * COS_LAT)
    lat1  = CENTER_LAT + r1 * math.sin(angle + math.pi)  / 111_320.0
    lon1  = CENTER_LON + r1 * math.cos(angle + math.pi)  / (111_320.0 * COS_LAT)
    pts = []
    for i in range(n_steps):
        t = i / max(n_steps - 1, 1)
        pts.append(dict(idx=i + 1,
                        true_lat=lat0 + t * (lat1 - lat0),
                        true_lon=lon0 + t * (lon1 - lon0),
                        agl_m=agl_m))
    return pts

# ── Speed test (feature extraction only, no DB needed) ────────────────────────

def run_speed_test(images, device):
    """Load both models and time forward_features on the same image list."""
    from anyloc.localizer import _pil_to_tensor

    results = {}
    for model_id in ('dinov2_vitb14', 'dinov2_vits14'):
        print(f"\n  Loading {model_id} …")
        model = torch.hub.load('facebookresearch/dinov2', model_id, pretrained=True)
        model.eval().to(device)

        # warm-up
        x_warm = _pil_to_tensor(images[0]).unsqueeze(0).to(device)
        with torch.no_grad():
            model.forward_features(x_warm)
        if device == 'cuda':
            torch.cuda.synchronize()

        times = []
        for img in images:
            x = _pil_to_tensor(img).unsqueeze(0).to(device)
            t0 = time.perf_counter()
            with torch.no_grad():
                model.forward_features(x)
            if device == 'cuda':
                torch.cuda.synchronize()
            times.append((time.perf_counter() - t0) * 1000.0)

        results[model_id] = _stats(times)
        del model
        if device == 'cuda':
            torch.cuda.empty_cache()

    return results

# ── Accuracy + latency test ────────────────────────────────────────────────────

def run_accuracy_test(trajectory, loc_b, loc_s):
    rows_b, rows_s = [], []
    hdr = (f"  {'#':>3}  {'True lat':>10}  {'True lon':>11}  "
           f"{'Err_B':>8}  {'T_B ms':>8}  {'Err_S':>8}  {'T_S ms':>8}")
    print(hdr)
    print("  " + "-" * (len(hdr) - 2))

    for pt in trajectory:
        i, lat, lon, h = pt['idx'], pt['true_lat'], pt['true_lon'], pt['agl_m']
        try:
            img = fetch_esri_image(lat, lon, h)
        except RuntimeError as exc:
            print(f"  {i:>3}  [SKIP] {exc}")
            continue

        t0 = time.perf_counter()
        b_lat, b_lon, *_ = loc_b.localize(img, agl_m=h)
        t_b = (time.perf_counter() - t0) * 1000.0

        t0 = time.perf_counter()
        s_lat, s_lon, *_ = loc_s.localize(img, agl_m=h)
        t_s = (time.perf_counter() - t0) * 1000.0

        err_b = euclidean_m(lat, lon, b_lat, b_lon)
        err_s = euclidean_m(lat, lon, s_lat, s_lon)

        print(f"  {i:>3}  {lat:>10.6f}  {lon:>11.6f}  "
              f"{err_b:>8.1f}  {t_b:>8.1f}  {err_s:>8.1f}  {t_s:>8.1f}")

        rows_b.append(dict(err=err_b, ms=t_b))
        rows_s.append(dict(err=err_s, ms=t_s))

    return rows_b, rows_s

# ── Summary printer ────────────────────────────────────────────────────────────

def _print_summary(speed_b, speed_s, acc_b=None, acc_s=None):
    W = 18
    print(f"\n{'='*54}")
    print(f"  {'Metric':<24}  {'ViT-B/14':>{W}}  {'ViT-S/14':>{W}}")
    print(f"  {'-'*24}  {'-'*W}  {'-'*W}")

    def row(label, vb, vs, fmt='.2f'):
        print(f"  {label:<24}  {vb:>{W}{fmt}}  {vs:>{W}{fmt}}")

    if speed_b and speed_s:
        print(f"  {'── Feature extraction ──'}")
        row('mean  (ms/img)',    speed_b['mean'],   speed_s['mean'])
        row('median (ms/img)',   speed_b['median'], speed_s['median'])
        row('min   (ms/img)',    speed_b['min'],    speed_s['min'])
        speedup = speed_b['mean'] / speed_s['mean'] if speed_s['mean'] > 0 else float('nan')
        print(f"  {'speedup (S vs B)':<24}  {speedup:>{W}.2f}x")

    if acc_b and acc_s:
        sb = _stats([r['err'] for r in acc_b])
        ss = _stats([r['err'] for r in acc_s])
        tb = _stats([r['ms']  for r in acc_b])
        ts = _stats([r['ms']  for r in acc_s])
        print(f"  {'── Localization accuracy ──'}")
        row('mean error (m)',    sb['mean'],   ss['mean'])
        row('median error (m)',  sb['median'], ss['median'])
        row('RMSE (m)',          sb['rmse'],   ss['rmse'])
        row('max error (m)',     sb['max'],    ss['max'])
        print(f"  {'── Localization latency ──'}")
        row('mean  (ms/query)',  tb['mean'],   ts['mean'])
        row('median (ms/query)', tb['median'], ts['median'])
        speedup_loc = tb['mean'] / ts['mean'] if ts['mean'] > 0 else float('nan')
        print(f"  {'speedup (S vs B)':<24}  {speedup_loc:>{W}.2f}x")

    print(f"{'='*54}\n")

# ── Main ───────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--steps',      type=int,   default=20,
                    help='Trajectory steps for accuracy test (default: 20)')
    ap.add_argument('--agl',        type=float, default=80.0,
                    help='Drone AGL in metres (default: 80)')
    ap.add_argument('--seed',       type=int,   default=42)
    ap.add_argument('--speed-only', action='store_true',
                    help='Run speed test only (skip accuracy test)')
    args = ap.parse_args()

    sys.path.insert(0, os.path.dirname(HERE))
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    print(f"\nDevice: {device}")

    has_vits_db = os.path.exists(os.path.join(DB_S, 'database.pt'))

    # ── Fetch images ──────────────────────────────────────────────────────────
    traj = _trajectory(args.steps, args.agl, args.seed)
    print(f"\nFetching {args.steps} Esri images …")
    images, valid_pts = [], []
    for pt in traj:
        try:
            img = fetch_esri_image(pt['true_lat'], pt['true_lon'], pt['agl_m'])
            images.append(img)
            valid_pts.append(pt)
            print(f"  {pt['idx']:>3}/{args.steps}  fetched", end='\r', flush=True)
        except RuntimeError as exc:
            print(f"  {pt['idx']:>3}  [SKIP] {exc}")
    print(f"\nFetched {len(images)} / {args.steps} images.")

    if not images:
        print("No images fetched — check internet connection.")
        sys.exit(1)

    # ── Speed test ────────────────────────────────────────────────────────────
    print(f"\n{'='*54}")
    print(f"  Speed test: feature extraction ({len(images)} images)")
    print(f"{'='*54}")
    speed = run_speed_test(images, device)
    speed_b = speed['dinov2_vitb14']
    speed_s = speed['dinov2_vits14']

    # ── Accuracy test ─────────────────────────────────────────────────────────
    acc_b, acc_s = None, None
    if not args.speed_only:
        if not has_vits_db:
            print(f"\n[SKIP] ViT-S accuracy test — database not found at {DB_S}")
            print(f"  Build it with:")
            print(f"    conda run -n isaac_sim_test python anyloc/build_database.py --model vits14")
        else:
            from anyloc.localizer import AnyLocLocalizer
            print(f"\n{'='*54}")
            print(f"  Accuracy test: {len(valid_pts)} steps, AGL={args.agl} m")
            print(f"{'='*54}\n")

            print("[1/2] Loading ViT-B localizer …")
            loc_b = AnyLocLocalizer(DB_B)
            print("[2/2] Loading ViT-S localizer …")
            loc_s = AnyLocLocalizer(DB_S)
            print()

            acc_b, acc_s = run_accuracy_test(valid_pts, loc_b, loc_s)

    _print_summary(speed_b, speed_s, acc_b, acc_s)


if __name__ == '__main__':
    main()
