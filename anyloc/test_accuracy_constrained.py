#!/usr/bin/env python3
"""
Benchmark for AnyLoc constrained-search optimization (no VO).

The project's localization pipeline (ros2_node.py) runs AnyLoc every
ANYLOC_INTERVAL frames with a restricted search radius centred on the
last known estimate.  This test isolates that anchor-chain optimization
from the VO layer:

  step 0  — global FAISS search (cold start, no prior)
  step n  — constrained search within SEARCH_RADIUS_M of the previous
             AnyLoc estimate (pure anchor-chain, no VO drift)

Both modes are run on every step so the numbers are directly comparable.

Metrics reported per step and in aggregate:
  • error_global_m   — Euclidean error with full FAISS search
  • error_const_m    — Euclidean error with constrained search
  • time_global_ms   — inference wall-time, global
  • time_const_ms    — inference wall-time, constrained
  • speedup          — time_global / time_const
  • in_window        — whether the true position was inside the search window

Usage:
    conda run -n isaac_sim_test python anyloc/test_accuracy_constrained.py \\
        [--steps 20] [--agl 80] [--radius 200] [--seed 42] \\
        [--output results_constrained.json] [--plot] [--no-viewport]
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

# ── Scene constants (must match cesium_scene.py / localizer.py) ───────────────
CENTER_LAT = 23.450868
CENTER_LON = 120.286135
RADIUS_M   = 2000.0
COS_LAT    = math.cos(math.radians(CENTER_LAT))

HFOV_DEG = 90.0
VFOV_DEG = 73.7

# Default search radius used in ros2_node.py
SEARCH_RADIUS_M = 200.0

# Esri World Imagery tile service (same as test_accuracy_esri.py)
ESRI_TILE_URL = (
    "https://server.arcgisonline.com/ArcGIS/rest/services"
    "/World_Imagery/MapServer/tile/{z}/{y}/{x}"
)
TILE_PX = 256


# ── Tile math ──────────────────────────────────────────────────────────────────

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


# ── Esri tile fetcher (identical logic to test_accuracy_esri.py) ───────────────

_tile_cache: dict = {}


def _is_blank_tile(tile: Image.Image, std_threshold: float = 8.0) -> bool:
    import numpy as np
    arr = np.array(tile, dtype=np.float32)
    return float(arr.std()) < std_threshold


def _fetch_tile(z, tx, ty, retries=3):
    key = (z, tx, ty)
    if key in _tile_cache:
        return _tile_cache[key]
    url = ESRI_TILE_URL.format(z=z, y=ty, x=tx)
    for attempt in range(retries):
        try:
            resp = requests.get(url, timeout=15,
                                headers={'User-Agent': 'AnyLocConstrained/1.0'})
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
        blank_count = 0
        for tx in range(tx_min, tx_max + 1):
            for ty in range(ty_min, ty_max + 1):
                tile = _fetch_tile(zoom, tx, ty)
                if _is_blank_tile(tile):
                    blank_count += 1
                mosaic.paste(tile, ((tx - tx_min) * TILE_PX, (ty - ty_min) * TILE_PX))

        if blank_count > (nx * ny) // 2:
            print(f"  [zoom {zoom}] {blank_count}/{nx * ny} blank tiles — "
                  f"trying zoom {zoom - 1}")
            for tx in range(tx_min, tx_max + 1):
                for ty in range(ty_min, ty_max + 1):
                    _tile_cache.pop((zoom, tx, ty), None)
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
            raise RuntimeError(f"Footprint crop too small at zoom {zoom}: "
                               f"({x1},{y1})→({x2},{y2}) in {mw}×{mh} mosaic")

        return mosaic.crop((x1, y1, x2, y2)).resize((img_w, img_h), Image.LANCZOS), zoom

    raise RuntimeError(f"No Esri imagery available for ({lat:.5f}, {lon:.5f}) "
                       f"down to zoom {min_zoom}")


# ── Geo helpers ────────────────────────────────────────────────────────────────

def euclidean_m(lat1, lon1, lat2, lon2):
    dlat_m = (lat1 - lat2) * 111_320.0
    dlon_m = (lon1 - lon2) * 111_320.0 * COS_LAT
    return math.sqrt(dlat_m ** 2 + dlon_m ** 2)


def _stats(values):
    n    = len(values)
    mean = sum(values) / n
    var  = sum((v - mean) ** 2 for v in values) / n
    rmse = math.sqrt(sum(v ** 2 for v in values) / n)
    s    = sorted(values)
    med  = s[n // 2] if n % 2 else (s[n // 2 - 1] + s[n // 2]) / 2
    return dict(n=n, mean=mean, median=med, std=math.sqrt(var),
                rmse=rmse, min=min(values), max=max(values))


# ── Trajectory generator ───────────────────────────────────────────────────────

def _linear_trajectory(n_steps, agl_m, seed):
    """
    Simple linear trajectory across the scene.
    Starts and ends within 85 % of RADIUS_M from centre.
    The anchor-chain benefit is most visible on a correlated path — each
    step is close to the previous one, exactly like a real drone flight.
    """
    rng   = random.Random(seed)
    angle = rng.uniform(0, 2 * math.pi)
    r0    = rng.uniform(0.3, 0.7) * RADIUS_M
    r1    = rng.uniform(0.3, 0.7) * RADIUS_M

    lat0 = CENTER_LAT + r0 * math.sin(angle)     / 111_320.0
    lon0 = CENTER_LON + r0 * math.cos(angle)     / (111_320.0 * COS_LAT)
    lat1 = CENTER_LAT + r1 * math.sin(angle + math.pi) / 111_320.0
    lon1 = CENTER_LON + r1 * math.cos(angle + math.pi) / (111_320.0 * COS_LAT)

    pts = []
    for i in range(n_steps):
        t    = i / max(n_steps - 1, 1)
        lat  = lat0 + t * (lat1 - lat0)
        lon  = lon0 + t * (lon1 - lon0)
        h    = agl_m if agl_m > 0 else rng.choice([60, 70, 80, 90, 100, 110, 120])
        pts.append(dict(idx=i + 1, true_lat=lat, true_lon=lon, agl_m=h))
    return pts


# ── Live viewport ─────────────────────────────────────────────────────────────

class _Viewport:
    """
    Three-panel live window:
      Left   — Esri validation image (true position)
      Centre — Global AnyLoc match
      Right  — Constrained AnyLoc match
    """

    def __init__(self, n_steps):
        import matplotlib
        matplotlib.use('TkAgg')
        import matplotlib.pyplot as plt
        self._plt = plt

        self.fig, (self.ax_val, self.ax_glob, self.ax_const) = plt.subplots(
            1, 3, figsize=(17, 5))
        self.fig.canvas.manager.set_window_title('AnyLoc Constrained-Search Test')
        self.fig.suptitle(f'AnyLoc Constrained Test  |  0 / {n_steps}', fontsize=11)
        for ax in (self.ax_val, self.ax_glob, self.ax_const):
            ax.axis('off')
        plt.tight_layout()
        plt.show(block=False)
        plt.pause(0.01)
        self._n = n_steps

    def update(self, idx, val_img, glob_img, const_img,
               true_lat, true_lon,
               est_glob_lat, est_glob_lon, err_glob,
               est_const_lat, est_const_lon, err_const,
               in_window, agl_m):
        plt = self._plt

        def _color(e):
            return 'limegreen' if e < 100 else ('orange' if e < 250 else 'tomato')

        self.ax_val.clear()
        self.ax_val.imshow(val_img)
        self.ax_val.set_title(
            f"Esri validation  [{idx}/{self._n}]\n"
            f"True  {true_lat:.6f}  {true_lon:.6f}   AGL {agl_m:.0f} m",
            fontsize=8.5)
        self.ax_val.axis('off')

        self.ax_glob.clear()
        self.ax_glob.imshow(glob_img)
        self.ax_glob.set_title(
            f"Global search\n"
            f"Est  {est_glob_lat:.6f}  {est_glob_lon:.6f}   "
            f"Err {err_glob:.1f} m",
            fontsize=8.5, color=_color(err_glob))
        self.ax_glob.axis('off')

        win_tag = '✓ in window' if in_window else '✗ out of window'
        self.ax_const.clear()
        self.ax_const.imshow(const_img)
        self.ax_const.set_title(
            f"Constrained search  [{win_tag}]\n"
            f"Est  {est_const_lat:.6f}  {est_const_lon:.6f}   "
            f"Err {err_const:.1f} m",
            fontsize=8.5, color=_color(err_const))
        self.ax_const.axis('off')

        self.fig.suptitle(
            f'AnyLoc Constrained Test  |  {idx} / {self._n}  '
            f'  glob {err_glob:.0f} m  const {err_const:.0f} m',
            fontsize=11)
        plt.tight_layout()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def close(self):
        self._plt.ioff()


# ── Benchmark ──────────────────────────────────────────────────────────────────

def run_benchmark(n_steps, agl_m, radius_m, seed, output_path, plot, show_viewport):
    sys.path.insert(0, os.path.dirname(HERE))
    from anyloc.localizer import AnyLocLocalizer

    print(f"\n{'='*66}")
    print(f"  AnyLoc Constrained-Search Benchmark  (no VO)")
    print(f"  Steps  : {n_steps}  |  AGL : {agl_m if agl_m > 0 else 'random 60-120'} m"
          f"  |  Radius : {radius_m} m  |  Seed : {seed}")
    print(f"  Method : anchor-chain constrained search vs full global search")
    print(f"{'='*66}\n")

    print("[1/3] Loading AnyLoc database and DINOv2 model …")
    loc = AnyLocLocalizer(DB_DIR)
    print()

    print("[2/3] Generating linear trajectory …")
    trajectory = _linear_trajectory(n_steps, agl_m, seed)
    start = trajectory[0]
    end   = trajectory[-1]
    span  = euclidean_m(start['true_lat'], start['true_lon'],
                        end['true_lat'],   end['true_lon'])
    print(f"  {n_steps} steps  |  path length ≈ {span:.0f} m\n")

    print("[3/3] Running localizer (global + constrained on each step) …\n")

    hdr  = (f"  {'#':>3}  {'True lat':>10}  {'True lon':>11}  "
            f"{'Err_glob':>9}  {'Err_const':>9}  "
            f"{'T_glob ms':>10}  {'T_const ms':>10}  {'Speedup':>7}  "
            f"{'InWin':>5}  AGL")
    print(hdr)
    print("  " + "-" * (len(hdr) - 2))

    results  = []
    err_glob_list  = []
    err_const_list = []
    t_glob_list    = []
    t_const_list   = []

    anchor_lat = None
    anchor_lon = None

    viewport = None
    if show_viewport:
        try:
            viewport = _Viewport(n_steps)
        except Exception as exc:
            print(f"  [viewport] could not open window: {exc}")

    for pt in trajectory:
        i, true_lat, true_lon, h = pt['idx'], pt['true_lat'], pt['true_lon'], pt['agl_m']

        # ── Fetch Esri image ──────────────────────────────────────────────────
        try:
            img, zoom = fetch_esri_image(true_lat, true_lon, h)
        except RuntimeError as exc:
            print(f"  {i:>3}  [SKIP] {exc}")
            continue

        # ── Global search ─────────────────────────────────────────────────────
        t0 = time.perf_counter()
        g_lat, g_lon, _, g_match, g_score, g_idx = loc.localize(img, agl_m=h)
        t_glob_ms = (time.perf_counter() - t0) * 1000.0

        # ── Constrained search ────────────────────────────────────────────────
        # On step 0 there is no anchor yet — fall back to global
        if anchor_lat is None:
            t0 = time.perf_counter()
            c_lat, c_lon, _, c_match, c_score, c_idx = loc.localize(img, agl_m=h)
            t_const_ms = (time.perf_counter() - t0) * 1000.0
            in_window  = True   # trivially true on cold start
        else:
            in_window = euclidean_m(true_lat, true_lon,
                                    anchor_lat, anchor_lon) <= radius_m
            t0 = time.perf_counter()
            c_lat, c_lon, _, c_match, c_score, c_idx = loc.localize(
                img, agl_m=h,
                center_lat=anchor_lat, center_lon=anchor_lon,
                radius_m=radius_m)
            t_const_ms = (time.perf_counter() - t0) * 1000.0

        # Update anchor from constrained result for the next step
        anchor_lat = c_lat
        anchor_lon = c_lon

        err_glob  = euclidean_m(true_lat, true_lon, g_lat, g_lon)
        err_const = euclidean_m(true_lat, true_lon, c_lat, c_lon)
        speedup   = t_glob_ms / t_const_ms if t_const_ms > 0 else float('nan')

        err_glob_list.append(err_glob)
        err_const_list.append(err_const)
        t_glob_list.append(t_glob_ms)
        t_const_list.append(t_const_ms)

        win_ch = 'Y' if in_window else 'N'
        print(f"  {i:>3}  {true_lat:>10.6f}  {true_lon:>11.6f}  "
              f"{err_glob:>9.1f}  {err_const:>9.1f}  "
              f"{t_glob_ms:>10.1f}  {t_const_ms:>10.1f}  {speedup:>7.2f}x  "
              f"{win_ch:>5}  {h:.0f} m")

        results.append(dict(
            idx=i,
            true_lat=true_lat, true_lon=true_lon,
            agl_m=h, zoom=zoom,
            est_glob_lat=g_lat,  est_glob_lon=g_lon,
            est_const_lat=c_lat, est_const_lon=c_lon,
            error_glob_m=err_glob, error_const_m=err_const,
            score_glob=g_score,    score_const=c_score,
            db_idx_glob=g_idx,     db_idx_const=c_idx,
            time_glob_ms=round(t_glob_ms, 2),
            time_const_ms=round(t_const_ms, 2),
            speedup=round(speedup, 3),
            in_window=in_window,
        ))

        if viewport is not None:
            viewport.update(
                i, img, g_match, c_match,
                true_lat, true_lon,
                g_lat, g_lon, err_glob,
                c_lat, c_lon, err_const,
                in_window, h)

    if viewport is not None:
        viewport.close()

    if not results:
        print("\nNo successful localizations.")
        return

    # ── Aggregate statistics ──────────────────────────────────────────────────
    st_glob  = _stats(err_glob_list)
    st_const = _stats(err_const_list)
    st_t_glob  = _stats(t_glob_list)
    st_t_const = _stats(t_const_list)
    in_window_pct = 100.0 * sum(r['in_window'] for r in results) / len(results)
    mean_speedup  = sum(r['speedup'] for r in results) / len(results)

    print(f"\n{'='*66}")
    print(f"  Results  ({len(results)} / {n_steps} steps)")
    print(f"{'='*66}")
    print(f"  {'Metric':<24}  {'Global':>10}  {'Constrained':>12}  {'Delta':>8}")
    print(f"  {'-'*24}  {'-'*10}  {'-'*12}  {'-'*8}")
    print(f"  {'Mean error (m)':<24}  {st_glob['mean']:>10.2f}  "
          f"{st_const['mean']:>12.2f}  "
          f"{st_const['mean'] - st_glob['mean']:>+8.2f}")
    print(f"  {'Median error (m)':<24}  {st_glob['median']:>10.2f}  "
          f"{st_const['median']:>12.2f}  "
          f"{st_const['median'] - st_glob['median']:>+8.2f}")
    print(f"  {'RMSE (m)':<24}  {st_glob['rmse']:>10.2f}  "
          f"{st_const['rmse']:>12.2f}  "
          f"{st_const['rmse'] - st_glob['rmse']:>+8.2f}")
    print(f"  {'Max error (m)':<24}  {st_glob['max']:>10.2f}  "
          f"{st_const['max']:>12.2f}")
    print(f"  {'Mean latency (ms)':<24}  {st_t_glob['mean']:>10.1f}  "
          f"{st_t_const['mean']:>12.1f}  "
          f"  {mean_speedup:.2f}x speedup")
    print(f"  {'True pos in window':<24}  {'—':>10}  "
          f"{in_window_pct:>11.1f} %")
    print(f"{'='*66}\n")

    report = dict(
        config=dict(
            n_steps=n_steps, agl_m=agl_m, radius_m=radius_m, seed=seed,
            imagery='Esri World Imagery',
            center_lat=CENTER_LAT, center_lon=CENTER_LON,
            radius_scene_m=RADIUS_M,
        ),
        statistics=dict(
            global_search=st_glob,
            constrained_search=st_const,
            latency_global_ms=st_t_glob,
            latency_constrained_ms=st_t_const,
            mean_speedup=mean_speedup,
            in_window_pct=in_window_pct,
        ),
        results=results,
    )
    if output_path:
        with open(output_path, 'w') as f:
            json.dump(report, f, indent=2)
        print(f"Results saved → {output_path}")

    if plot:
        _plot_results(results, st_glob, st_const)

    return report


# ── Plot ───────────────────────────────────────────────────────────────────────

def _plot_results(results, st_glob, st_const):
    try:
        import matplotlib
        matplotlib.use('TkAgg')
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not available — skipping plot.")
        return

    idxs        = [r['idx']           for r in results]
    err_glob    = [r['error_glob_m']  for r in results]
    err_const   = [r['error_const_m'] for r in results]
    t_glob      = [r['time_glob_ms']  for r in results]
    t_const     = [r['time_const_ms'] for r in results]
    true_lats   = [r['true_lat']      for r in results]
    true_lons   = [r['true_lon']      for r in results]
    in_windows  = [r['in_window']     for r in results]

    fig, axes = plt.subplots(1, 3, figsize=(18, 5))
    fig.suptitle(
        f"AnyLoc Constrained-Search Benchmark  |  N={len(results)}\n"
        f"Global mean={st_glob['mean']:.1f} m   Constrained mean={st_const['mean']:.1f} m   "
        f"Constrained RMSE={st_const['rmse']:.1f} m",
        fontsize=11,
    )

    # Error vs step
    ax = axes[0]
    ax.plot(idxs, err_glob,  'o--', color='steelblue', linewidth=1.2,
            markersize=5, label=f"Global  (mean {st_glob['mean']:.1f} m)")
    ax.plot(idxs, err_const, 's-',  color='tomato',    linewidth=1.5,
            markersize=5, label=f"Constrained  (mean {st_const['mean']:.1f} m)")
    # Mark steps where true pos was outside the window
    out_idxs = [r['idx'] for r in results if not r['in_window']]
    out_err  = [r['error_const_m'] for r in results if not r['in_window']]
    if out_idxs:
        ax.scatter(out_idxs, out_err, marker='x', s=80, color='black',
                   linewidths=1.5, zorder=5, label='Outside window')
    ax.set_xlabel('Step')
    ax.set_ylabel('Euclidean error (m)')
    ax.set_title('Error per step')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # Latency vs step
    ax = axes[1]
    ax.plot(idxs, t_glob,  'o--', color='steelblue', linewidth=1.2,
            markersize=5, label='Global')
    ax.plot(idxs, t_const, 's-',  color='tomato',    linewidth=1.5,
            markersize=5, label='Constrained')
    ax.set_xlabel('Step')
    ax.set_ylabel('Inference time (ms)')
    ax.set_title('Latency per step')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # Spatial error map
    ax = axes[2]
    sc_g = ax.scatter(
        [r['est_glob_lon']  for r in results],
        [r['est_glob_lat']  for r in results],
        c=err_glob, cmap='Blues', s=50, marker='o', zorder=3,
        label='Global estimate', vmin=0, vmax=max(err_glob + [1]))
    sc_c = ax.scatter(
        [r['est_const_lon'] for r in results],
        [r['est_const_lat'] for r in results],
        c=err_const, cmap='Reds', s=50, marker='s', zorder=4,
        label='Constrained estimate', vmin=0, vmax=max(err_const + [1]))
    ax.plot(true_lons, true_lats, 'k-', linewidth=1.0, alpha=0.5, zorder=2,
            label='True trajectory')
    ax.scatter(true_lons, true_lats, c='black', s=20, zorder=5)
    # Mark cold start (step 0)
    ax.scatter([true_lons[0]], [true_lats[0]], marker='*', s=200,
               color='gold', edgecolors='black', linewidths=0.5,
               zorder=6, label='Start')
    fig.colorbar(sc_g, ax=ax, fraction=0.03, pad=0.01).set_label('Err global (m)')
    ax.set_xlabel('Longitude')
    ax.set_ylabel('Latitude')
    ax.set_title('Spatial error map')
    ax.legend(fontsize=7)

    plt.tight_layout()
    plt.show()


# ── CLI ────────────────────────────────────────────────────────────────────────

def main():
    global DB_DIR
    _default_db = DB_DIR

    parser = argparse.ArgumentParser(
        description='AnyLoc constrained-search benchmark — no VO, anchor-chain only.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument('--steps',  type=int,   default=20,
                        help='Number of trajectory steps (default: 20)')
    parser.add_argument('--agl',    type=float, default=80.0,
                        help='Drone AGL in metres (default: 80). '
                             'Pass 0 to randomise across [60, 120] m.')
    parser.add_argument('--radius', type=float, default=SEARCH_RADIUS_M,
                        help=f'Constrained search radius in metres '
                             f'(default: {SEARCH_RADIUS_M})')
    parser.add_argument('--seed',   type=int,   default=42,
                        help='Random seed for trajectory generation (default: 42)')
    parser.add_argument('--output', default='',
                        help='Save results to this JSON file (default: none)')
    parser.add_argument('--plot', action='store_true',
                        help='Show error and latency plots after benchmark')
    parser.add_argument('--no-viewport', action='store_true',
                        help='Disable live three-panel image viewport')
    parser.add_argument('--db-dir', default=_default_db,
                        help=f'AnyLoc database directory (default: {_default_db})')
    args = parser.parse_args()

    DB_DIR = args.db_dir

    run_benchmark(
        n_steps=args.steps,
        agl_m=args.agl,
        radius_m=args.radius,
        seed=args.seed,
        output_path=args.output or None,
        plot=args.plot,
        show_viewport=not args.no_viewport,
    )


if __name__ == '__main__':
    main()
