#!/usr/bin/env python3
"""
Regenerate the contest survey lawnmower (SURVEY_WPS) used by
control/ardupilot_commander.py and tools/live_trace.py.

Does proper convex-polygon line clipping against the buffered detection zone
(ZONE_VERTS) — correct at any strip spacing, unlike the original hand-built
pattern's perpendicular-translation approximation between "corner" turns.

Re-run this whenever the camera FOV, survey altitude, or zone boundary
changes, and paste the printed SURVEY_WPS block into:
  - control/ardupilot_commander.py   (3-tuples: north_m, east_m, TAKEOFF_ALT)
  - control/ardupilot_auto_mission.py  — NOT covered by this script; it uses a
    different right-angle-turn algorithm. Update separately.
  - tools/live_trace.py              (2-tuples: north_m, east_m; also update
    _CORNER_IDX to {even indices 2..len-2})

Usage:
  python3 tools/gen_contest_survey.py                    # default: 50% sidelap of IMX219 footprint
  python3 tools/gen_contest_survey.py --spacing 53.9      # explicit spacing in metres
  python3 tools/gen_contest_survey.py --hfov 62.2 --altitude 65 --sidelap-ratio 0.687

  # Also write a Mission Planner-importable QGC WPL 110 file (and loadable by
  # ardupilot_commander.py's --waypoint-file, which only reads NAV_WAYPOINT rows):
  python3 tools/gen_contest_survey.py --waypoints-file control/survey.waypoints
"""
import argparse
import math
import os

# Buffered detection zone (30 m inward from raw corners), (north_m, east_m).
# Mirrors ZONE_VERTS in control/ardupilot_commander.py — keep in sync.
ZONE_VERTS = [
    (642.0, -1215.0),   # NW'
    (507.0,  -489.0),   # NE'
    (-13.0,  -587.0),   # SE'
    (121.0, -1293.0),   # SW'
]

# Raw zone corners — only used to derive strip direction (parallel to the
# NW->NE edge, i.e. the zone's long axis).
RAW_NW = (677.0, -1240.0)
RAW_NE = (531.0,  -454.0)

ALTITUDE_M = 65.0   # AGL — must match operational mission altitude
HFOV_DEG   = 62.2   # IMX219 CSI camera
SURVEY_SPEED = 12.0  # m/s — matches SURVEY_SPEED in control/ardupilot_commander.py

# Mirrors HOME_LAT/HOME_LON in control/ardupilot_commander.py / home_elevation.json
HOME_LAT  = 23.450868
HOME_LON  = 120.286135
M_PER_DEG = 111_320.0


def _strip_and_advance_dirs():
    dn = RAW_NE[0] - RAW_NW[0]
    de = RAW_NE[1] - RAW_NW[1]
    length = math.hypot(dn, de)
    strip_dir = (dn / length, de / length)
    advance_dir = (strip_dir[1], -strip_dir[0])
    if advance_dir[0] < 0:   # keep advance pointing north-ish
        advance_dir = (-advance_dir[0], -advance_dir[1])
    return strip_dir, advance_dir


STRIP_DIR, ADVANCE_DIR = _strip_and_advance_dirs()


def _to_local(pt):
    n, e = pt
    x = n * STRIP_DIR[0] + e * STRIP_DIR[1]
    y = n * ADVANCE_DIR[0] + e * ADVANCE_DIR[1]
    return x, y


def _to_world(x, y):
    n = x * STRIP_DIR[0] + y * ADVANCE_DIR[0]
    e = x * STRIP_DIR[1] + y * ADVANCE_DIR[1]
    return n, e


def _clip_row(y, poly_local):
    """Intersect horizontal line at local-y with a convex polygon (local coords)."""
    xs = []
    n = len(poly_local)
    for i in range(n):
        x1, y1 = poly_local[i]
        x2, y2 = poly_local[(i + 1) % n]
        if (y1 <= y <= y2) or (y2 <= y <= y1):
            if y2 == y1:
                continue
            t = (y - y1) / (y2 - y1)
            xs.append(x1 + t * (x2 - x1))
    if len(xs) < 2:
        return None
    return min(xs), max(xs)


def generate(spacing_m: float, edge_margin_m: float = 6.0):
    """Return (list of (north_m, east_m) strip-endpoint pairs, n_strips)."""
    poly_local = [_to_local(p) for p in ZONE_VERTS]
    ys = [p[1] for p in poly_local]
    y_min, y_max = min(ys), max(ys)

    rows = []
    y = y_min + edge_margin_m
    while y <= y_max - edge_margin_m + 1.0:
        span = _clip_row(y, poly_local)
        if span is not None:
            rows.append((y, span[0], span[1]))
        y += spacing_m

    wps = []
    for i, (y, x_lo, x_hi) in enumerate(rows):
        x_a, x_b = (x_hi, x_lo) if i % 2 == 0 else (x_lo, x_hi)
        wps.append(_to_world(x_a, y))
        wps.append(_to_world(x_b, y))
    return wps, len(rows)


def _print_survey_wps_block(wps, n_strips):
    print(f'# {n_strips}-strip boundary-parallel boustrophedon; strip spacing as generated above.')
    idx = 0
    for strip in range(n_strips):
        n1, e1 = wps[strip * 2]
        n2, e2 = wps[strip * 2 + 1]
        if strip == 0:
            print(f'    ({n1:7.1f}, {e1:8.1f},  TAKEOFF_ALT),  # {idx:<2d} ENTRY : E end strip 1  → fly W')
            idx += 1
            print(f'    ({n2:7.1f}, {e2:8.1f},  TAKEOFF_ALT),  # {idx:<2d} WP{idx:02d}  : W end strip 1')
            idx += 1
        else:
            side = 'W' if strip % 2 == 1 else 'E'
            other = 'E' if side == 'W' else 'W'
            turn_dir = 'NE' if side == 'W' else 'NW'
            print(f'    ({n1:7.1f}, {e1:8.1f},  TAKEOFF_ALT),  # {idx:<2d} WP{idx:02d}  : {side} boundary corner → fly {turn_dir}')
            idx += 1
            last = strip == n_strips - 1
            suffix = '  (final)' if last else f'  → fly {other}'
            print(f'    ({n2:7.1f}, {e2:8.1f},  TAKEOFF_ALT),  # {idx:<2d} WP{idx:02d}  : {other} end strip {strip + 1}{suffix}')
            idx += 1


def _north_east_to_latlon(n, e):
    cos_lat = math.cos(math.radians(HOME_LAT))
    lat = HOME_LAT + n / M_PER_DEG
    lon = HOME_LON + e / (M_PER_DEG * cos_lat)
    return lat, lon


def write_qgc_file(wps, altitude, speed, out_path):
    """QGC WPL 110 format — importable in Mission Planner and readable by
    control/mission_loader.py (only NAV_WAYPOINT / cmd=16 rows are used by the
    latter; frame=3 = MAV_FRAME_GLOBAL_RELATIVE_ALT, i.e. altitude is AGL)."""
    rows = ["QGC WPL 110"]
    home_lat, home_lon = HOME_LAT, HOME_LON
    rows.append(f"0\t1\t0\t16\t0\t0\t0\t0\t{home_lat:.6f}\t{home_lon:.6f}\t0.000000\t1")
    rows.append(f"1\t0\t3\t178\t0\t{speed:.1f}\t-1\t0\t0.000000\t0.000000\t0.000000\t1")
    first_lat, first_lon = _north_east_to_latlon(*wps[0])
    rows.append(f"2\t0\t3\t22\t0\t0\t0\t0\t{first_lat:.6f}\t{first_lon:.6f}\t{altitude:.1f}\t1")
    for k, (n, e) in enumerate(wps):
        lat, lon = _north_east_to_latlon(n, e)
        rows.append(f"{k+3}\t0\t3\t16\t0\t2\t0\t0\t{lat:.6f}\t{lon:.6f}\t{altitude:.1f}\t1")
    rows.append(f"{len(wps)+3}\t0\t3\t20\t0\t0\t0\t0\t0.000000\t0.000000\t{altitude:.1f}\t1")

    os.makedirs(os.path.dirname(out_path) or '.', exist_ok=True)
    with open(out_path, 'w') as f:
        f.write('\n'.join(rows) + '\n')
    print(f'# Wrote {len(wps)} waypoints -> {out_path}')


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--hfov', type=float, default=HFOV_DEG, help='camera HFOV in degrees')
    ap.add_argument('--altitude', type=float, default=ALTITUDE_M, help='survey AGL in metres')
    ap.add_argument('--sidelap-ratio', type=float, default=0.687,
                     help='spacing/swath ratio (default 0.687 preserves the original '
                          'AP-IMX900-era ~31%% sidelap)')
    ap.add_argument('--spacing', type=float, default=None,
                     help='explicit strip spacing in metres (overrides --hfov/--altitude/--sidelap-ratio)')
    ap.add_argument('--waypoints-file', default=None, metavar='PATH',
                     help='also write a QGC WPL 110 .waypoints file (Mission Planner import + '
                          'control/mission_loader.py) to this path')
    ap.add_argument('--speed', type=float, default=SURVEY_SPEED, help='cruise speed in m/s (for the .waypoints file)')
    args = ap.parse_args()

    swath_m = 2.0 * args.altitude * math.tan(math.radians(args.hfov / 2.0))
    spacing = args.spacing if args.spacing is not None else swath_m * args.sidelap_ratio

    wps, n_strips = generate(spacing)
    total = sum(math.hypot(wps[i][0] - wps[i - 1][0], wps[i][1] - wps[i - 1][1])
                for i in range(1, len(wps)))

    print(f'# swath={swath_m:.1f} m  spacing={spacing:.1f} m  sidelap={(1 - spacing/swath_m)*100:.0f}%')
    print(f'# {n_strips} strips, {len(wps)} WPs, {total:.0f} m path '
          f'(~{total/12/60:.1f} min @ 12 m/s, ~{total/3/60:.1f} min @ 3 m/s)')
    print()
    _print_survey_wps_block(wps, n_strips)

    if args.waypoints_file:
        print()
        write_qgc_file(wps, args.altitude, args.speed, args.waypoints_file)


if __name__ == '__main__':
    main()
