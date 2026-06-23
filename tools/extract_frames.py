#!/usr/bin/env python3
"""
Extract geo-tagged frames from a field recording for AnyLoc database building.

Reads the output of record_field.py (video.mp4 + telemetry.csv + meta.json)
and produces:
  frames/000000.jpg, frames/000001.jpg, …
  frames.csv   — path, lat, lon, alt_amsl, alt_agl, heading_deg

Frame selection strategy:
  - One frame every MIN_DIST metres of ground track (default 30 m)
  - Skip frames where AGL < MIN_AGL (default 50 m)
  - Skip frames where the drone was tilted > MAX_TILT deg  (needs pitch/roll — skipped if absent)

Usage:
    python3 tools/extract_frames.py field_data/20260623_120000/ [OPTIONS]

    --min-dist M     minimum distance between saved frames in metres (default 30)
    --min-agl  M     skip frames below this AGL (default 50)
    --rotate         rotate each frame to North-up using heading (recommended for DB)
"""

import argparse
import csv
import json
import math
import os
import sys


def _haversine(lat1, lon1, lat2, lon2):
    R = 6_371_000.0
    d1, d2 = math.radians(lat2 - lat1), math.radians(lon2 - lon1)
    a = math.sin(d1/2)**2 + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(d2/2)**2
    return 2 * R * math.asin(math.sqrt(a))


def load_telemetry(telem_path):
    rows = []
    with open(telem_path) as f:
        for row in csv.DictReader(f):
            try:
                rows.append({
                    't':       float(row['unix_time']),
                    'lat':     float(row['lat']),
                    'lon':     float(row['lon']),
                    'alt_msl': float(row['alt_amsl'])   if row['alt_amsl']   else None,
                    'agl':     float(row['alt_agl'])    if row['alt_agl']    else None,
                    'heading': float(row['heading_deg']) if row['heading_deg'] else None,
                })
            except (ValueError, KeyError):
                continue
    return rows


def nearest_telem(rows, t):
    lo, hi = 0, len(rows) - 1
    while lo < hi:
        mid = (lo + hi) // 2
        if rows[mid]['t'] < t:
            lo = mid + 1
        else:
            hi = mid
    if lo > 0 and abs(rows[lo-1]['t'] - t) < abs(rows[lo]['t'] - t):
        lo -= 1
    return rows[lo]


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('session_dir', help='Output directory from record_field.py')
    ap.add_argument('--min-dist', type=float, default=30.0,
                    help='Minimum ground distance between frames in metres (default 30)')
    ap.add_argument('--min-agl',  type=float, default=50.0,
                    help='Minimum AGL to save a frame (default 50)')
    ap.add_argument('--rotate', action='store_true',
                    help='Rotate frames to North-up using heading')
    args = ap.parse_args()

    d = args.session_dir
    meta_path  = os.path.join(d, 'meta.json')
    video_path = os.path.join(d, 'video.mp4')
    telem_path = os.path.join(d, 'telemetry.csv')

    for p in (meta_path, video_path, telem_path):
        if not os.path.exists(p):
            sys.exit(f'Missing: {p}')

    with open(meta_path) as f:
        meta = json.load(f)

    fps         = meta['fps']
    video_start = meta['video_start_unix']

    telem = load_telemetry(telem_path)
    if not telem:
        sys.exit('Telemetry CSV is empty.')

    frames_dir = os.path.join(d, 'frames')
    os.makedirs(frames_dir, exist_ok=True)
    out_csv = os.path.join(d, 'frames.csv')

    # Use OpenCV for video reading and optional rotation
    try:
        import cv2
    except ImportError:
        sys.exit('pip install opencv-python')

    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        sys.exit(f'Cannot open {video_path}')

    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    print(f'[EXT] Video: {total_frames} frames @ {fps} fps')
    print(f'[EXT] Telemetry: {len(telem)} rows')
    print(f'[EXT] Min dist: {args.min_dist} m  |  Min AGL: {args.min_agl} m')
    print(f'[EXT] Rotate to North-up: {args.rotate}')

    saved        = 0
    last_lat     = None
    last_lon     = None
    frame_idx    = 0

    with open(out_csv, 'w', newline='') as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(['path', 'lat', 'lon', 'alt_amsl', 'alt_agl', 'heading_deg'])

        while True:
            ok, frame = cap.read()
            if not ok:
                break

            t = video_start + frame_idx / fps
            frame_idx += 1

            tel = nearest_telem(telem, t)

            # Skip if telemetry is stale (>2 s gap)
            if abs(tel['t'] - t) > 2.0:
                continue

            # Skip below min AGL
            if tel['agl'] is None or tel['agl'] < args.min_agl:
                continue

            # Skip if too close to last saved frame
            if last_lat is not None:
                dist = _haversine(last_lat, last_lon, tel['lat'], tel['lon'])
                if dist < args.min_dist:
                    continue

            # Optionally rotate to North-up
            if args.rotate and tel['heading'] is not None:
                h, w = frame.shape[:2]
                cx, cy = w // 2, h // 2
                M = cv2.getRotationMatrix2D((cx, cy), tel['heading'], 1.0)
                frame = cv2.warpAffine(frame, M, (w, h))

            fname = os.path.join(frames_dir, f'{saved:06d}.jpg')
            cv2.imwrite(fname, frame, [cv2.IMWRITE_JPEG_QUALITY, 92])
            writer.writerow([
                os.path.relpath(fname, d),
                f'{tel["lat"]:.8f}',
                f'{tel["lon"]:.8f}',
                f'{tel["alt_msl"]:.2f}' if tel['alt_msl'] is not None else '',
                f'{tel["agl"]:.2f}',
                f'{tel["heading"]:.1f}' if tel['heading'] is not None else '',
            ])
            csv_file.flush()

            last_lat, last_lon = tel['lat'], tel['lon']
            saved += 1

            if saved % 50 == 0:
                pct = frame_idx / total_frames * 100 if total_frames else 0
                print(f'  {pct:.0f}%  {saved} frames saved …')

    cap.release()
    print(f'[EXT] Done — {saved} frames → {frames_dir}/')
    print(f'[EXT] Frame list → {out_csv}')


if __name__ == '__main__':
    main()
