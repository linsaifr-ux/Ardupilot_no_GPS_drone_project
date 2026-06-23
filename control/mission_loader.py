#!/usr/bin/env python3
"""
Parse Mission Planner QGC WPL 110 .waypoints file into ENU survey waypoints.

Returns list of (north_m, east_m, agl_m) relative to (home_lat, home_lon).
Only NAV_WAYPOINT (command=16) items are returned.
coord_frame=3: alt is AGL above home (direct use).
coord_frame=0: alt is MSL; subtract home_alt_msl to get AGL.
"""
import math
import os


def load_mission_planner_waypoints(filepath, home_lat, home_lon, home_alt_msl=0.0):
    if not os.path.isfile(filepath):
        print(f"[mission_loader] File not found: {filepath}")
        return None

    cos_lat   = math.cos(math.radians(home_lat))
    m_per_deg = 111_320.0
    ref_lat   = home_lat
    ref_lon   = home_lon
    wps       = []

    with open(filepath) as f:
        header = f.readline().strip()
        if not header.startswith("QGC WPL"):
            print(f"[mission_loader] Not a QGC WPL file: {filepath}")
            return None

        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split("\t")
            if len(parts) < 11:
                continue
            try:
                idx   = int(parts[0])
                frame = int(parts[2])
                cmd   = int(parts[3])
                lat   = float(parts[8])
                lon   = float(parts[9])
                alt   = float(parts[10])
            except (ValueError, IndexError):
                continue

            # Index 0 is the home row — use it as coordinate origin
            if idx == 0:
                if lat != 0.0 and lon != 0.0:
                    ref_lat = lat; ref_lon = lon
                    cos_lat = math.cos(math.radians(ref_lat))
                continue

            if cmd != 16 or (lat == 0.0 and lon == 0.0):
                continue

            north = (lat - ref_lat) * m_per_deg
            east  = (lon - ref_lon) * m_per_deg * cos_lat
            agl   = alt if frame != 0 else (alt - home_alt_msl)
            wps.append((north, east, agl))

    print(f"[mission_loader] Loaded {len(wps)} waypoints from {os.path.basename(filepath)}")
    for i, (n, e, a) in enumerate(wps):
        print(f"  WP{i:02d}  N={n:+.1f} m  E={e:+.1f} m  AGL={a:.1f} m")
    return wps
