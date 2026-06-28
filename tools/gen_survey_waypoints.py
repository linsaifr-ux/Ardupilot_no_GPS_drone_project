#!/usr/bin/env python3
"""
Generate Mission Planner lawnmower waypoints for AnyLoc database collection.
Strips run E-W (long side ~1 743 m), advancing N-S.

Usage:
  python3 tools/gen_survey_waypoints.py                 # one full file
  python3 tools/gen_survey_waypoints.py --split 4       # 4 equal-width N-S sub-missions
  python3 tools/gen_survey_waypoints.py --spacing 62.75 # 62.75 m strip spacing (50 % sidelap)
"""

import argparse, math, sys
from pathlib import Path

# ── Operational boundary corners (lat, lon) ────────────────────────
CORNERS = [
    (23.45695, 120.27399),
    (23.45174, 120.27314),
    (23.45424, 120.29020),
    (23.44928, 120.28824),
]

# ── Survey parameters ──────────────────────────────────────────────
ALTITUDE_M     = 65.0   # AGL – must match operational mission altitude
SPEED_MS       = 3.0    # m/s, ≤ 3 reduces motion blur
MARGIN_PCT     = 0.10   # 10 % each side = 20 % total expansion

def build_waypoints(slat_min, slat_max, slon_min, slon_max,
                    strip_spacing_m, m_lat, altitude, speed):
    # Strips run E-W (long side); advance N-S between strips
    step_deg = strip_spacing_m / m_lat
    ns_m = (slat_max - slat_min) * m_lat
    n_strips = math.ceil(ns_m / strip_spacing_m) + 1

    wpts = []
    for i in range(n_strips):
        lat = min(slat_min + i * step_deg, slat_max)
        if i % 2 == 0:
            wpts.append((lat, slon_min))
            wpts.append((lat, slon_max))
        else:
            wpts.append((lat, slon_max))
            wpts.append((lat, slon_min))

    rows = ["QGC WPL 110"]
    rows.append("0\t1\t0\t16\t0\t0\t0\t0\t0.000000\t0.000000\t0.000000\t1")
    rows.append(f"1\t0\t3\t178\t0\t{speed:.1f}\t-1\t0\t0.000000\t0.000000\t0.000000\t1")
    s_lat, s_lon = wpts[0]
    rows.append(f"2\t0\t3\t22\t0\t0\t0\t0\t{s_lat:.6f}\t{s_lon:.6f}\t{altitude:.1f}\t1")
    for k, (lat, lon) in enumerate(wpts):
        rows.append(f"{k+3}\t0\t3\t16\t0\t2\t0\t0\t{lat:.6f}\t{lon:.6f}\t{altitude:.1f}\t1")
    rows.append(f"{len(wpts)+3}\t0\t3\t20\t0\t0\t0\t0\t0.000000\t0.000000\t{altitude:.1f}\t1")
    return rows, n_strips

def stats(rows, m_lat, m_lon, speed):
    wpts = [(float(r.split('\t')[8]), float(r.split('\t')[9]))
            for r in rows[1:] if r.split('\t')[3] == '16' and float(r.split('\t')[8]) != 0]
    dist = sum(
        math.sqrt(((wpts[j][0]-wpts[j-1][0])*m_lat)**2 + ((wpts[j][1]-wpts[j-1][1])*m_lon)**2)
        for j in range(1, len(wpts))
    )
    return dist, dist / speed / 60.0

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--split',   type=int, default=1,       help='split into N N-S sub-missions')
    ap.add_argument('--spacing', type=float, default=62.75, help='strip spacing in metres (default 62.75 = 50 %% sidelap)')
    ap.add_argument('--outdir',  default='field_data',   help='output directory')
    args = ap.parse_args()

    lat_ref = sum(c[0] for c in CORNERS) / len(CORNERS)
    m_lat = 111320.0
    m_lon = 111320.0 * math.cos(math.radians(lat_ref))

    lat_min = min(c[0] for c in CORNERS);  lat_max = max(c[0] for c in CORNERS)
    lon_min = min(c[1] for c in CORNERS);  lon_max = max(c[1] for c in CORNERS)

    lat_mg = (lat_max - lat_min) * MARGIN_PCT
    lon_mg = (lon_max - lon_min) * MARGIN_PCT

    slat_min = lat_min - lat_mg;  slat_max = lat_max + lat_mg
    slon_min = lon_min - lon_mg;  slon_max = lon_max + lon_mg

    ns_m = (slat_max - slat_min) * m_lat
    ew_m = (slon_max - slon_min) * m_lon

    sidelap = (1 - args.spacing / 125.5) * 100
    print(f"Survey area  : {ew_m:.0f} m (E-W) × {ns_m:.0f} m (N-S)  = {ew_m*ns_m/1e6:.2f} km²", file=sys.stderr)
    print(f"Expanded box : lat [{slat_min:.6f}, {slat_max:.6f}]", file=sys.stderr)
    print(f"               lon [{slon_min:.6f}, {slon_max:.6f}]", file=sys.stderr)
    print(f"Strip spacing: {args.spacing:.0f} m  →  {sidelap:.0f} % sidelap", file=sys.stderr)

    outdir = Path(args.outdir)
    outdir.mkdir(parents=True, exist_ok=True)

    lat_width = (slat_max - slat_min) / args.split
    total_dist = 0.0

    for seg in range(args.split):
        seg_lat_min = slat_min + seg * lat_width
        seg_lat_max = slat_min + (seg + 1) * lat_width
        rows, n_strips = build_waypoints(
            seg_lat_min, seg_lat_max, slon_min, slon_max,
            args.spacing, m_lat, ALTITUDE_M, SPEED_MS)
        dist_m, t_min = stats(rows, m_lat, m_lon, SPEED_MS)
        total_dist += dist_m

        suffix = f"_part{seg+1}of{args.split}" if args.split > 1 else "_full"
        fname = outdir / f"survey_mission{suffix}.waypoints"
        fname.write_text('\n'.join(rows) + '\n')

        print(f"  Part {seg+1}/{args.split}: {n_strips} strips, {dist_m/1000:.1f} km, "
              f"~{t_min:.0f} min (~{math.ceil(t_min/20)} batteries)  → {fname}", file=sys.stderr)

    print(f"Total distance: {total_dist/1000:.1f} km  "
          f"~{total_dist/SPEED_MS/60:.0f} min  "
          f"(~{math.ceil(total_dist/SPEED_MS/60/20)} batteries @ 20 min each)", file=sys.stderr)

if __name__ == '__main__':
    main()
