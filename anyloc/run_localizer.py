#!/usr/bin/env python3
"""
AnyLoc postview — two live windows:
  [Drone Camera]  — what the drone camera captured + ground-truth geo info
  [AnyLoc Match]  — the database image AnyLoc matched + estimated geo info

Run in a separate terminal while Isaac Sim is running:
    DISPLAY=:2 conda run -n isaac_sim_test python anyloc/run_localizer.py

Press Ctrl-C or close the window to quit.
"""

import json, math, os, sys, time
import numpy as np
import torch
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from PIL import Image, ImageDraw, ImageFont

HERE          = os.path.dirname(os.path.abspath(__file__))
FRAME_JPG     = os.path.abspath(os.path.join(HERE, '..', 'simulator', 'drone_frames', 'latest.jpg'))
META_JSON     = os.path.abspath(os.path.join(HERE, '..', 'simulator', 'drone_frames', 'latest_meta.json'))
DB_DIR        = os.path.join(HERE, 'database_vits14')
ESTIMATE_JSON = os.path.join(HERE, 'latest_estimate.json')  # read by control/run_vision.py

COS_LAT = math.cos(math.radians(23.450868))

sys.path.insert(0, HERE)
from localizer  import AnyLocLocalizer
from vo_refiner import VORefiner

ANYLOC_INTERVAL = 10   # run full AnyLoc retrieval every N frames; VO fills in between


# ── PIL text overlay (avoids numpy ops) ───────────────────────────────────────

def _pil_overlay(pil_img, lines, text_color='white', bg_alpha=140):
    """
    Draw a semi-transparent dark panel in the top-left of a PIL image.
    Returns a new PIL RGB image.
    """
    img  = pil_img.copy().convert('RGBA')
    draw = ImageDraw.Draw(img)

    try:
        font = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf', 15)
    except Exception:
        font = ImageFont.load_default()

    line_h  = 20
    pad     = 8
    max_w   = max(draw.textlength(ln, font=font) for ln in lines)
    panel_h = pad + line_h * len(lines) + pad // 2
    panel_w = int(max_w) + pad * 2

    # Semi-transparent black panel
    overlay = Image.new('RGBA', img.size, (0, 0, 0, 0))
    ov_draw = ImageDraw.Draw(overlay)
    ov_draw.rectangle((0, 0, panel_w, panel_h), fill=(0, 0, 0, bg_alpha))
    img = Image.alpha_composite(img, overlay)

    draw2 = ImageDraw.Draw(img)
    for i, ln in enumerate(lines):
        draw2.text((pad, pad + i * line_h), ln, fill=text_color, font=font)

    return img.convert('RGB')


def pil_to_rgb_array(pil_img, size=(640, 480)):
    """PIL → (H, W, 3) uint8 array via torch frombuffer (avoids np.array issue)."""
    img = pil_img.resize(size, Image.LANCZOS).convert('RGB')
    t   = torch.frombuffer(bytearray(img.tobytes()), dtype=torch.uint8) \
               .reshape(size[1], size[0], 3)
    return t.numpy()


# ── Geo helper ─────────────────────────────────────────────────────────────────

def geo_dist_m(lat1, lon1, lat2, lon2):
    return math.hypot((lat1 - lat2) * 111_320.0,
                      (lon1 - lon2) * 111_320.0 * COS_LAT)


# ── Main loop ──────────────────────────────────────────────────────────────────

def main():
    if not os.path.exists(DB_DIR):
        sys.exit(f"[PostView] Database not found: {DB_DIR}\n"
                 "           Run build_database.py first.")

    loc = AnyLocLocalizer(DB_DIR)
    vo  = VORefiner()

    # AnyLoc anchor state
    frame_count  = 0
    anchor_lat   = None
    anchor_lon   = None
    anchor_match = None
    anchor_score = 0.0
    anchor_idx   = 0
    accum_dlat   = 0.0
    accum_dlon   = 0.0

    # Create figure with two side-by-side axes
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5.2),
                                   gridspec_kw={'wspace': 0.04},
                                   layout='constrained')
    fig.patch.set_facecolor('#1a1a1a')
    for ax in (ax1, ax2):
        ax.axis('off')
        ax.set_facecolor('#1a1a1a')

    # Placeholder arrays until first frame arrives
    blank = np.zeros((480, 640, 3), dtype=np.uint8)
    im1 = ax1.imshow(blank)
    im2 = ax2.imshow(blank)
    ax1.set_title('Drone Camera', color='white', fontsize=11, pad=4)
    ax2.set_title('AnyLoc+VO', color='white', fontsize=11, pad=4)
    plt.ion()
    plt.show()

    last_mtime = 0.0
    print(f"[PostView] Watching {FRAME_JPG}")
    print("[PostView] Close the window or press Ctrl-C to quit.")

    while plt.fignum_exists(fig.number):
        # Poll for new frame
        try:
            mtime = os.path.getmtime(FRAME_JPG)
        except FileNotFoundError:
            plt.pause(0.5)
            continue

        if mtime != last_mtime:
            # Guard against mid-write reads: simulator may be writing the file
            # right now.  Catch bad JPEG / truncated JSON and retry next tick.
            try:
                frame = Image.open(FRAME_JPG).convert('RGB')
                frame.load()   # force full decode before the file changes again
                with open(META_JSON) as fh:
                    meta = json.load(fh)
            except Exception as exc:
                print(f"[PostView] frame read error ({exc}) — retrying next tick")
                plt.pause(0.1)
                continue

            last_mtime = mtime

            drone_lat = meta['lat']
            drone_lon = meta['lon']
            drone_alt = meta.get('alt_m', 0.0)
            drone_agl = meta.get('agl_m', drone_alt)
            drone_yaw = meta.get('yaw_deg', 0.0)

            frame_count += 1
            run_anyloc = (frame_count == 1) or (frame_count % ANYLOC_INTERVAL == 0)

            # ── VO update (every frame) ───────────────────────────────────────
            dlat, dlon, n_vo = vo.update(frame, drone_agl, drone_yaw)
            if anchor_lat is not None:
                accum_dlat += dlat
                accum_dlon += dlon

            # ── AnyLoc retrieval (every ANYLOC_INTERVAL frames) ───────────────
            t0 = time.perf_counter()
            if run_anyloc:
                clat = (anchor_lat + accum_dlat) if anchor_lat is not None else None
                clon = (anchor_lon + accum_dlon) if anchor_lat is not None else None
                est_lat, est_lon, est_alt, matched, score, db_idx = loc.localize(
                    frame, agl_m=drone_agl,
                    center_lat=clat, center_lon=clon, radius_m=200.0)
                anchor_lat   = est_lat
                anchor_lon   = est_lon
                anchor_match = matched
                anchor_score = score
                anchor_idx   = db_idx
                accum_dlat   = 0.0
                accum_dlon   = 0.0
                vo.reset()
                # Write estimate for control/run_vision.py (atomic rename)
                _est = {
                    "timestamp":  time.time(),
                    "est_lat":    est_lat,
                    "est_lon":    est_lon,
                    "alt_msl_m":  drone_alt,
                    "agl_m":      drone_agl,
                    "yaw_deg":    drone_yaw,
                    "score":      float(score),
                    "error_m":    float(geo_dist_m(drone_lat, drone_lon, est_lat, est_lon)),
                }
                _tmp = ESTIMATE_JSON + ".tmp"
                with open(_tmp, 'w') as _fh:
                    json.dump(_est, _fh)
                os.replace(_tmp, ESTIMATE_JSON)
            elapsed_ms = (time.perf_counter() - t0) * 1000.0

            # Skip display until first anchor is available
            if anchor_lat is None:
                plt.pause(0.15)
                continue

            final_lat = anchor_lat + accum_dlat
            final_lon = anchor_lon + accum_dlon
            err_m     = geo_dist_m(drone_lat, drone_lon, final_lat, final_lon)
            anchor_age = 0 if run_anyloc else (frame_count % ANYLOC_INTERVAL)

            # ── View 1: Drone Camera ──────────────────────────────────────────
            v1_pil = _pil_overlay(frame.resize((640, 480), Image.LANCZOS), [
                'DRONE CAMERA',
                f'LAT   {drone_lat:.5f} N',
                f'LON   {drone_lon:.5f} E',
                f'ALT   {drone_alt:.1f} m MSL    AGL {drone_agl:.1f} m',
                f'YAW   {drone_yaw:.1f} deg',
            ], text_color='white')
            im1.set_data(pil_to_rgb_array(v1_pil))

            # ── View 2: AnyLoc + VO estimate ─────────────────────────────────
            good_match  = err_m < 200
            match_color = '#50ff50' if good_match else '#5050ff'
            mode_tag    = 'ANYLOC' if run_anyloc else f'VO +{anchor_age}f'
            v2_pil = _pil_overlay(anchor_match.resize((640, 480), Image.LANCZOS), [
                f'{mode_tag}   score {anchor_score:.3f}   #{anchor_idx}',
                f'LAT   {final_lat:.5f} N',
                f'LON   {final_lon:.5f} E',
                f'ALT   {drone_agl:.1f} m AGL',
                f'ERR   {err_m:.0f} m    VO pts {n_vo}    {elapsed_ms:.0f} ms',
            ], text_color=match_color)
            im2.set_data(pil_to_rgb_array(v2_pil))

            ax2.set_title(f'AnyLoc+VO [{mode_tag}]  —  ERR {err_m:.0f} m',
                          color=match_color, fontsize=11, pad=4)

            fig.canvas.draw_idle()

        plt.pause(0.15)

    print("[PostView] Closed.")


if __name__ == '__main__':
    main()
