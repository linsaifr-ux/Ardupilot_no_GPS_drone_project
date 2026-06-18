# Survey Mission Plan — Detection Zone Lawnmower + Car Detection Response

## Overview

Autonomous lawnmower survey of a defined detection zone west of home. When YOLO detects a
vehicle inside the zone, its ground position is computed via yaw-corrected GSD projection
and logged to `detections.csv`. The drone never diverts — the survey route runs uninterrupted.

| Parameter | Value |
|-----------|-------|
| Cruise AGL | 65 m |
| Cruise speed | 12 m/s |
| Strip orientation | E-W (long axis of zone) |
| Strip spacing | 91.7 m N-S |
| Strips | 7 (5 full + 2 partial edge strips) |
| Total distance | ≈ 7.36 km |
| Estimated flight time | ≈ 10.2 min |
| AnyLoc error | ~20 m → 30 m inward buffer |
| Camera footprint at 65 m | 125 m × 83 m (HFOV 88°, VFOV 65.1°) |
| Cross-track swath (N-S, ⊥ to flight) | 125 m → 33 m overlap between strips |

---

## Detection Zone

### Raw boundary corners (lat, lon)

| Corner | Lat | Lon |
|--------|-----|-----|
| NW | 23.45695 | 120.27399 |
| NE | 23.45564 | 120.28169 |
| SE | 23.45044 | 120.28062 |
| SW | 23.45174 | 120.27314 |

### In NED metres from home (23.450868°N, 120.286135°E)

COS_LAT ≈ 0.9175; M_PER_DEG_LAT = 111 320; M_PER_DEG_LON ≈ 102 136.

| Corner | North (m) | East (m) |
|--------|-----------|----------|
| NW | +677 | −1240 |
| NE | +531 | −454 |
| SE | −48  | −563 |
| SW | +97  | −1327 |

The NE corner (531, −454) is the existing single test waypoint — coordinate math confirmed.

### Buffered boundary (30 m inward — exceeds AnyLoc ~20 m error)

Each boundary edge shifted 30 m toward the polygon interior. New corners at edge
intersections:

| Corner | North (m) | East (m) |
|--------|-----------|----------|
| NW' | +642 | −1215 |
| NE' | +507 | −489 |
| SE' | −13  | −587 |
| SW' | +121 | −1293 |

Polygon edges (CW):
- **Northern:** NW'(642,−1215) → NE'(507,−489)
- **Eastern:**  NE'(507,−489)  → SE'(−13,−587)
- **Southern:** SE'(−13,−587)  → SW'(121,−1293)
- **Western:**  SW'(121,−1293) → NW'(642,−1215)

---

## Lawnmower Strip Plan

### Design rationale

- Strips run E-W (long axis of zone, ~730 m) rather than N-S (~540 m), reducing
  the number of strips and simplifying transitions.
- N-S spacing 91.7 m, cross-track swath 125 m → 33 m overlap between strips (no gaps).
- 7 strips with uniform 91.7 m spacing (= 550 m / 6 gaps) span N=60→610, covering
  the full buffered zone N-S extent.
- At 12 m/s + 5 fps the drone advances 2.4 m between frames — unchanged.
- 12 m/s is within PX4's default `MPC_XY_VEL_MAX` (12 m/s).
- Enter from east side (closest to home); boustrophedon S→N.

### Strip limits

East limits at each strip N position clipped to the buffered polygon edges.
Strips at N ≥ 507 use the northern edge (NW'→NE') for the east boundary.

| Strip | North (m) | E west (m) | E east (m) | Direction | Length |
|-------|-----------|------------|------------|-----------|--------|
| S  |  60 | −972  | −573  | E→W | 399 m — partial (SE boundary) |
| 1  | 152 | −1288 | −556  | W→E | 732 m — full |
| 2  | 243 | −1275 | −539  | E→W | 736 m — full |
| 3  | 335 | −1261 | −521  | W→E | 740 m — full |
| 4  | 427 | −1247 | −504  | E→W | 743 m — full |
| 5  | 518 | −1234 | −548  | W→E | 686 m — full (east limit on north edge) |
| N  | 610 | −1220 | −1043 | E→W | 177 m — partial (NW corner) |

> **T6 diagonal note:** strip 5 exits east (E=−548) and strip N starts east (E=−1043) due
> to boustrophedon parity with 7 strips. The WP11→WP12 transit is ~504 m (~42 s), versus
> ~92 m for all other inter-strip transitions. Unavoidable given the zone geometry.

---

## Ordered Waypoint Sequence

All coordinates: `(north_m, east_m, 65.0)` — relative to home, metres.

```
HOME    (0, 0)               takeoff to 65 m AGL, fly at 12 m/s

ENTRY:  ( 60,   −573)        E end strip S                     → fly W
WP01:   ( 60,   −972)        W end strip S
WP02:   (152,  −1288)        W end strip 1  (transition NW)   → fly E
WP03:   (152,   −556)        E end strip 1
WP04:   (243,   −539)        E end strip 2  (transition NE)   → fly W
WP05:   (243,  −1275)        W end strip 2
WP06:   (335,  −1261)        W end strip 3  (transition NE)   → fly E
WP07:   (335,   −521)        E end strip 3
WP08:   (427,   −504)        E end strip 4  (transition NE)   → fly W
WP09:   (427,  −1247)        W end strip 4
WP10:   (518,  −1234)        W end strip 5  (transition NE)   → fly E
WP11:   (518,   −548)        E end strip 5  [T6: 504 m diagonal]
WP12:   (610,  −1043)        E end strip N  (transition NW)   → fly W
WP13:   (610,  −1220)        W end strip N

HOME    (0, 0)               fly home → AUTO.LAND
```

### Reference lat/lon for each waypoint

lat = 23.450868 + N/111320;  lon = 120.286135 + E/(111320 × 0.9175)

| WP    | North (m) | East (m) | Lat       | Lon        |
|-------|-----------|----------|-----------|------------|
| ENTRY |  60 | −573  | 23.451407 | 120.280525 |
| WP01  |  60 | −972  | 23.451407 | 120.276618 |
| WP02  | 152 | −1288 | 23.452233 | 120.273524 |
| WP03  | 152 | −556  | 23.452233 | 120.280691 |
| WP04  | 243 | −539  | 23.453051 | 120.280858 |
| WP05  | 243 | −1275 | 23.453051 | 120.273651 |
| WP06  | 335 | −1261 | 23.453877 | 120.273788 |
| WP07  | 335 | −521  | 23.453877 | 120.281034 |
| WP08  | 427 | −504  | 23.454703 | 120.281201 |
| WP09  | 427 | −1247 | 23.454703 | 120.273925 |
| WP10  | 518 | −1234 | 23.455520 | 120.274053 |
| WP11  | 518 | −548  | 23.455520 | 120.280769 |
| WP12  | 610 | −1043 | 23.456347 | 120.275923 |
| WP13  | 610 | −1220 | 23.456347 | 120.274190 |

---

## Detection Response

### Trigger

Subscribe to `/yolo/detections` (`vision_msgs/Detection2DArray`).  
Trigger on any detection whose canonical label is `car`, `van`, `truck`, or `bus`.  
**The survey route is never interrupted.** Car positions are logged in-flight from the
pixel offset alone; the drone continues to the next waypoint without diverting.

### Ground position from bounding box (yaw-corrected)

```python
GSD_x = 2 * AGL * tan(radians(HFOV / 2)) / CAM_W   # ≈ 0.1226 m/px at 65 m
GSD_y = 2 * AGL * tan(radians(VFOV / 2)) / CAM_H   # ≈ 0.1082 m/px at 65 m

# Drone heading from pose quaternion (camera top follows drone nose)
q       = drone.pose.orientation
yaw_enu = atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y² + q.z²))
h       = -yaw_enu                        # NED heading (rad, CW from north)

dx_m =  (bbox_cx − CAM_W / 2) * GSD_x   # drone-right  (+east when h=0)
dy_m = −(bbox_cy − CAM_H / 2) * GSD_y   # drone-forward (+north when h=0)

Δeast  = dx_m * cos(h) + dy_m * sin(h)
Δnorth = −dx_m * sin(h) + dy_m * cos(h)

obj_north = cur_north + Δnorth
obj_east  = cur_east  + Δeast
```

### Log procedure

1. Compute `(obj_north, obj_east)` using the yaw-corrected formula above.
2. Check dedup: skip if within `DEDUP_RADIUS = 5 m` of any already-logged position.
3. Log highest-confidence vehicle to `detections.csv`:
   ```
   timestamp, category, confidence, lat, lon, agl_m
   ```
   where lat/lon derived from `(obj_north, obj_east)`.
4. Append `(obj_north, obj_east)` to `_logged_positions`.

### Deduplication guard

After a vehicle is logged its position `(north_m, east_m)` is appended to
`self._logged_positions`. Any subsequent detection whose estimated ground position
falls within `DEDUP_RADIUS = 5 m` of an already-logged entry is silently discarded.
The 5 m radius suppresses duplicate logs from successive frames detecting the same
physical car without discarding nearby distinct vehicles.

### Boundary visualisation

`_in_buffered_zone()` and `ZONE_VERTS` are kept in `px4_commander.py` for use by
`tools/live_trace.py` (zone polygon overlay). They are no longer used to gate
detection logging.

---

## Implementation (`control/px4_commander.py`)

### Constants

```python
SURVEY_SPEED  = 12.0   # m/s — strip cruise speed
DEDUP_RADIUS  = 5.0    # m — suppress duplicate log within this radius
SURVEY_WPS = [         # (north_m, east_m, agl_m)
    ( 60.0,   -573.0,  65.0),  # ENTRY: E end strip S
    ( 60.0,   -972.0,  65.0),  # WP01 : W end strip S
    (152.0,  -1288.0,  65.0),  # WP02 : W end strip 1 [diag NW]
    (152.0,   -556.0,  65.0),  # WP03 : E end strip 1
    (243.0,   -539.0,  65.0),  # WP04 : E end strip 2
    (243.0,  -1275.0,  65.0),  # WP05 : W end strip 2
    (335.0,  -1261.0,  65.0),  # WP06 : W end strip 3
    (335.0,   -521.0,  65.0),  # WP07 : E end strip 3
    (427.0,   -504.0,  65.0),  # WP08 : E end strip 4
    (427.0,  -1247.0,  65.0),  # WP09 : W end strip 4
    (518.0,  -1234.0,  65.0),  # WP10 : W end strip 5
    (518.0,   -548.0,  65.0),  # WP11 : E end strip 5 [long T6 diag NW]
    (610.0,  -1043.0,  65.0),  # WP12 : E end strip N
    (610.0,  -1220.0,  65.0),  # WP13 : W end strip N
]
```

### YOLO subscription

```python
from vision_msgs.msg import Detection2DArray

self._logged_positions = []   # (north_m, east_m) dedup list
self.create_subscription(Detection2DArray, "/yolo/detections",
                         self._cb_detections, sensor_qos)
```

### `_cb_detections(msg)`

```python
def _cb_detections(self, msg):
    if self._drone is None:
        return
    vehicles = [d for d in msg.detections
                if d.results[0].hypothesis.class_id in VEHICLE_CLASSES]
    if not vehicles:
        return
    ds    = self._drone.pose.position
    cur_n, cur_e = ds.y, ds.x
    agl   = max(1.0, ds.z - HOME_ALT_MSL)
    gsd_x = 2 * agl * math.tan(math.radians(HFOV_DEG / 2)) / CAM_W
    gsd_y = 2 * agl * math.tan(math.radians(VFOV_DEG / 2)) / CAM_H
    best  = max(vehicles, key=lambda d: d.results[0].hypothesis.score)
    cx, cy = best.bbox.center.position.x, best.bbox.center.position.y
    q       = self._drone.pose.orientation
    yaw_enu = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y**2 + q.z**2))
    h = -yaw_enu
    dx_m =  (cx - CAM_W/2) * gsd_x
    dy_m = -(cy - CAM_H/2) * gsd_y
    de = dx_m*math.cos(h) + dy_m*math.sin(h)
    dn = -dx_m*math.sin(h) + dy_m*math.cos(h)
    obj_n, obj_e = cur_n + dn, cur_e + de
    for ln, le in self._logged_positions:
        if math.hypot(obj_n - ln, obj_e - le) < DEDUP_RADIUS:
            return
    self._log_detection(best.results[0].hypothesis.class_id,
                        best.results[0].hypothesis.score,
                        obj_n, obj_e, agl)
```

### Survey loop

```python
wp_idx = 0
while wp_idx < len(SURVEY_WPS):
    wn, we, wagl = SURVEY_WPS[wp_idx]
    reached = cmd.go_to_ned(wn, we, wagl, timeout=WAYPOINT_TIMEOUT,
                            speed=SURVEY_SPEED)
    if reached:
        ...log arrival...
    else:
        print(f"WP {wp_idx+1} TIMEOUT — skipping")
    wp_idx += 1
```

---

## Flight Time Budget

| Segment | Distance | Time (12 m/s) |
|---------|----------|----------------|
| Home → ENTRY | 576 m | 48.0 s |
| Strip S (partial) | 399 m | 33.2 s |
| WP01 → WP02 (diagonal NW) | 329 m | 27.4 s |
| Strip 1 | 732 m | 61.0 s |
| WP03 → WP04 (short NE) | 93 m | 7.7 s |
| Strip 2 | 736 m | 61.3 s |
| WP05 → WP06 (short NE) | 93 m | 7.7 s |
| Strip 3 | 740 m | 61.7 s |
| WP07 → WP08 (short NE) | 94 m | 7.8 s |
| Strip 4 | 743 m | 61.9 s |
| WP09 → WP10 (short NE) | 92 m | 7.7 s |
| Strip 5 | 686 m | 57.2 s |
| WP11 → WP12 (long T6 NW) | 504 m | 42.0 s |
| Strip N (partial) | 177 m | 14.8 s |
| WP13 → Home | 1364 m | 113.7 s |
| **Total** | **≈ 7358 m** | **≈ 613 s ≈ 10.2 min** |

No detection diversions — cars are logged in-flight; flight time is fixed at ~10.2 min.

---

## Coverage Summary

| Metric | Value |
|--------|-------|
| Buffered zone area | ≈ 0.46 km² |
| Strips | 7 (5 full + 2 partial edge strips) |
| Zone N-S extent covered | 655 m of 655 m (100% — 33 m uniform overlap between strips) |
| Strip spacing / footprint | 91.7 m / 125 m → 33 m overlap |
| Along-track: footprint / advance per frame | 83 m / 2.4 m → heavy overlap |
| AnyLoc buffer from boundary | 30 m |
| Estimated flight time (no detections) | ≈ 10.2 min |
