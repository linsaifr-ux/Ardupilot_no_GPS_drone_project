# Project History

## 2026-06-19 — ArduPilot AP-3 HOLD GATE passed (PSC_NE parameter rename fix)

**Result:** HOLDTEST passed — 0.1 m horizontal drift over 40 s at 3 m AGL. Full debugging record in `instructions/ap3_holdgate_solving_process.md`.

**Root cause discovered:** ArduPilot V4.8.0-dev renamed the horizontal position controller parameters from `PSC_POSXY_*`/`PSC_VELXY_*` to `PSC_NE_*`/`PSC_NE_VEL_*`. The old `no_gps.parm` used V4.3-era names that were **silently ignored** — no warning, no error, params simply not applied. This left two dangerous defaults active:
- `PSC_NE_VEL_I = 1.0` (default): integrator accumulates velocity error across the 40 s gate → grows without bound → spiral oscillation
- `PSC_NE_POS_P = 1.0` (default): above the overdamped critical P ≈ 0.44 → underdamped → sustained oscillation even if the integrator were zero

**Fix:** Updated `control/no_gps.parm` to use V4.8 parameter names:
```
PSC_NE_POS_P    0.2     # below P_crit≈0.44 → real eigenvalues → overdamped
PSC_NE_VEL_P    2.0     # keep default
PSC_NE_VEL_I    0.0     # ZERO — default 1.0 causes integral windup
PSC_NE_VEL_D    0.5     # raised from 0.25 for extra damping
```

**HOLDTEST gate:** rewritten to use `make_sp(e0, n0, HOLD_AGL)` position setpoints — lets ArduPilot's PSC loop stabilise internally rather than fighting it with external velocity commands.

**Files changed:** `control/no_gps.parm`

---

## 2026-06-19 — ArduPilot migration: ardupilot_commander.py (root cause fix)

**Root cause identified** for the ArduPilot WP nav inversion that led to the PX4 migration.
`flight_commander.py` sent NED coordinates (`x=north, y=east`) to `setpoint_raw/local`.
MAVROS2 always applies ENU→NED regardless of the `FRAME_LOCAL_NED` flag — so it treated
`x=north_m` as "East" and swapped axes before forwarding to ArduPilot. This is the exact
mirror-direction inversion observed. AC_PosControl was computing correctly; it received the
wrong target. The velocity-based `go_to_ned()` in `flight_commander.py` happened to work
because it already sent ENU-correct velocities.

**Fix:** send ENU (`x=East, y=North, z=Up`) to MAVROS — identical to `px4_commander.py`.

**Files created/modified:**

- `control/ardupilot_commander.py` — new ~850-line ArduPilot commander ported from
  `px4_commander.py`. Key additions vs PX4: EKF origin publication, EKF_POS_HORIZ_ABS wait
  via raw MAVLink on `/uas1/mavlink_source`, `CommandTOL` NAV_TAKEOFF, force-arm fallback
  via `CommandLong(400, param2=21196)`, `"LAND"` mode (not `"AUTO.LAND"`). Survey mission,
  YOLO detection callback, VPE two-phase — all identical to `px4_commander.py`.

- `control/launch_commander_ardupilot.sh` — launch script mirroring `launch_commander_px4.sh`;
  sets `PYTHONUNBUFFERED=1`.

- `control/no_gps.parm` — `WPNAV_SPEED` changed from 100 → 1200 cm/s (12 m/s, matches
  `SURVEY_SPEED` in `ardupilot_commander.py`).

- `run.sh` — ArduPilot section overhauled: added `--isaac` flag (window 0 = Isaac Sim or
  `drone_sim.py` depending on flag), window 3 = `launch_commander_ardupilot.sh`, windows 4/5
  = AnyLoc/Detection (when `--anyloc`/`--detection` passed). Updated pkill pattern and help.

- `instructions/ardupilot_migration_plan.md` — implementation plan documenting the root cause,
  change comparison table, detailed spec, 6-phase test plan.

**Status after this session:** Implementation complete; all syntax checks passed. Testing
(AP-1 through AP-6) not yet run.

---

## 2026-06-12 — Dedup radius 30 m → 5 m; live_trace detection crop text enlarged

`DEDUP_RADIUS` in `control/px4_commander.py` reduced from 30 m to 5 m. The 30 m radius was
originally sized to cover ~20 m AnyLoc position uncertainty, but it also suppressed logging
of genuinely distinct vehicles parked within 30 m of each other. 5 m suppresses same-vehicle
duplicate frames while allowing nearby distinct cars to be logged separately.

`tools/live_trace.py` detection crop panel text sizes increased: panel title (label + category)
from 8 → 15 pt, coordinate line from 7 → 15 pt (green `#44ff88`).

---

## 2026-06-09 — Imagery: Esri World Imagery → NLSC PHOTO2

Switched satellite imagery back from Esri World Imagery to Taiwan NLSC PHOTO2 WMTS in
both `simulator/cesium_scene.py` and `anyloc/build_database.py`.

```python
NLSC_TILE_URL = "https://wmts.nlsc.gov.tw/wmts/PHOTO2/default/GoogleMapsCompatible/{z}/{y}/{x}"
SAT_ZOOM = 18   # ~0.60 m/px effective
```

The previous Esri zoom-19 mosaic gave ~0.37 m/px but NLSC zoom-18 at ~0.60 m/px is
sufficient and uses the authoritative Taiwan government orthophoto.

**Action required after pulling:** delete `simulator/satellite_ground.jpg` so it
re-downloads as NLSC on the next run. AnyLoc database must also be rebuilt:
```bash
rm simulator/satellite_ground.jpg
conda run -n isaac_sim_test python anyloc/build_database.py --rebuild
```

**Files changed:** `simulator/cesium_scene.py`, `anyloc/build_database.py`, `README.md`.

---

## 2026-06-09 — Survey route: 6-strip → 7-strip (91.7 m spacing, 33 m overlap)

**Motivation:** 6-strip 110 m spacing gave only 15 m cross-track overlap between adjacent
125 m swaths. Adding a 7th strip reduces N-S spacing to 91.7 m (= 550 m / 6 gaps) within
the same N=60→610 extent, increasing overlap to 33 m.

**Strip parameters (7 strips):**

| Strip | N (m) | E east | E west | Length | Direction |
|-------|-------|--------|--------|--------|-----------|
| S  |  60 | −573  | −972  | 399 m  | E→W (partial SE) |
| 1  | 152 | −556  | −1288 | 732 m  | W→E (full) |
| 2  | 243 | −539  | −1275 | 736 m  | E→W (full) |
| 3  | 335 | −521  | −1261 | 740 m  | W→E (full) |
| 4  | 427 | −504  | −1247 | 743 m  | E→W (full) |
| 5  | 518 | −548  | −1234 | 686 m  | W→E (full) |
| N  | 610 | −1043 | −1220 | 177 m  | E→W (partial NW) |

**T6 diagonal note:** with 7 (odd) strips the boustrophedon parity means strip 5 exits
east (E=−548) and strip N also starts east (E=−1043). The WP11→WP12 transit is ~504 m
(~42 s) — unavoidable given the zone geometry.

**Route totals:** 14 waypoints; ~7.36 km; ~10.2 min (was 12 WPs / ~6.1 km / ~8.5 min).

**Files changed:** `control/px4_commander.py` (SURVEY_WPS 12→14 entries), `tools/live_trace.py`
(SURVEY_WPS mirror), `instructions/survey_mission_plan.md` (full rewrite for 7-strip),
`README.md`, `control/README.md`.

---

## 2026-06-09 — Fix sim cars floating above terrain

**Problem:** `make_car()` placed all three cars at `centre_elev` (home-origin elevation,
~28 m MSL) regardless of their actual lat/lon. The cars are 700–1100 m west of home
where terrain elevation differs slightly, causing them to float visibly above the ground
(shadow visible below the car body).

**Fix:** accumulate all terrain tile vertices into `_terrain_verts_all` during the tile
loading loop. Add `terrain_elev_at(x, y)` which finds the nearest vertex by Euclidean
distance and returns its Z. `make_car()` now calls `terrain_elev_at(car_x, car_y)` for
each car's root Z so wheels sit on the actual terrain surface at that position.

---

## 2026-06-09 — PX4 SITL stop script + launch improvements

Added `control/stop_px4_sitl.sh` so users can stop PX4 SITL without using `pkill` directly.
Two improvements to `control/launch_px4_sitl.sh`:

- **PID file:** saves the PX4 process PID to `/tmp/px4_sitl.pid` on every launch, so the stop script can target it precisely.
- **Log overwrite:** changed `>>` to `>` so `/tmp/px4_sitl.log` is overwritten fresh on each launch instead of accumulating across sessions.

`stop_px4_sitl.sh` stop sequence:
1. MAVLink `MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN` via MAVROS (graceful — if MAVROS is up)
2. SIGTERM to `/tmp/px4_sitl.pid`
3. pkill SIGTERM fallback
4. SIGKILL if still alive

Usage: `bash control/stop_px4_sitl.sh`

---

## 2026-06-08 — Sim car positions added to live_trace.py overlay

Three static yellow square markers (■) now appear in `tools/live_trace.py` at the
simulator car positions, with Car_01/02/03 labels and a "Sim car" legend entry.

| Marker | NED (m) | Strip area |
|--------|---------|------------|
| Car_01 | N+350 E−700 | Strip 2 (N=300) |
| Car_02 | N+150 E−900 | Strip 1 (N=180) |
| Car_03 | N+450 E−1100 | Strip 3 (N=420) |

Markers are drawn once at startup alongside the zone boundary and route — not
refreshed each frame. Added to `_legend_handles` as `_Line2D` with marker `"s"`,
color `#ffe066`.

---

## 2026-06-08 — Strip spacing equalized to 110 m (was 120 m with unequal last gap)

**Problem:** the 6-strip 120 m spacing plan placed strip N at N=600, only 60 m above
strip 4 at N=540 — half the intended spacing. The zone's northern boundary (N=642)
prevents placing the 6th strip at N=660 (120 m above strip 4).

**Fix:** recompute uniform spacing to fit 6 strips within the zone:
- Zone span N=−13 to N=642 (655 m)
- Desired: first swath covers zone south boundary, last swath covers zone north boundary
- Strip S stays at N=60 (clips to SE boundary); strip N at N=610 (swath reaches 672, clips to 642)
- Uniform gap: (610−60)/5 = **110 m** between all consecutive strips

| Strip | N (m) | E east | E west | Width |
|-------|-------|--------|--------|-------|
| S |  60 | −573  | −972  | 399 m (partial SE) |
| 1 | 170 | −553  | −1286 | 733 m (full) |
| 2 | 280 | −532  | −1269 | 737 m (full) |
| 3 | 390 | −511  | −1253 | 742 m (full) |
| 4 | 500 | −490  | −1236 | 746 m (full) |
| N | 610 | −1043 | −1220 | 177 m (partial NW) |

Strip overlap: 125 m swath − 110 m spacing = **15 m** between every consecutive pair.
~6.1 km / ~8.5 min. Files changed: `control/px4_commander.py`, `tools/live_trace.py`.

---

## 2026-06-08 — Survey route: 5 E-W strips → 6 E-W strips (120 m spacing)

**Motivation:** 150 m strip spacing left a 25 m gap between consecutive 125 m
swaths. Reducing to 120 m spacing gives a 5 m overlap — zero coverage gaps across
the entire detection zone.

**Old route (5 strips, 150 m spacing):**
- Strips at N = 60, 210, 360, 510, 580 m; 10 waypoints; ~5.9 km; ~8.2 min

**New route (6 strips, 120 m spacing):**
- Strips at N = 60, 180, 300, 420, 540, 600 m; 12 waypoints; ~6.0 km; ~8.3 min

| Strip | N (m) | E west | E east | Direction | Length |
|-------|-------|--------|--------|-----------|--------|
| S  |  60 | −972  | −573  | E→W | 399 m (partial SE) |
| 1  | 180 | −1284 | −551  | W→E | 733 m (full) |
| 2  | 300 | −1266 | −528  | E→W | 738 m (full) |
| 3  | 420 | −1248 | −505  | W→E | 743 m (full) |
| 4  | 540 | −1230 | −667  | E→W | 563 m (partial, NE corner) |
| N  | 600 | −1221 | −989  | W→E | 232 m (partial NW) |

Strip 4's east limit retreats to −667 (not the polygon eastern edge) because at
N>507 the eastern polygon boundary is the northern edge running NW, which pushes
the clipped E limit westward.

**Car strip assignments updated** (with 120 m spacing):
Car_01 (N=350) → Strip 2; Car_02 (N=150) → Strip 1; Car_03 (N=450) → Strip 3.

**Files changed:** `control/px4_commander.py`, `tools/live_trace.py`

---

## 2026-06-07 — Survey route changed from N-S strips to E-W strips

**Motivation:** The detection zone is ~873 m E-W × 655 m N-S. Flying E-W strips
(along the long axis) gives ~730 m per full strip vs ~537 m for N-S strips, so
the same area is covered with fewer strips and simpler transitions.

**Old route (N-S strips, 6 strips):**
- Strips at fixed E positions: −545, −695, −845, −995, −1145, −1250 m
- 150 m E-W spacing; strips ran N-S
- 12 waypoints; ~5.4 km; ~7.6 min

**New route (E-W strips, 5 strips):**
- Strips at fixed N positions: 60, 210, 360, 510, 580 m
- 150 m N-S spacing; strips run E-W; enter from east (closest to home), boustrophedon S→N

| Strip | N (m) | E west | E east | Length | Type |
|-------|-------|--------|--------|--------|------|
| S  |  60 | −972  | −573  | 399 m | partial (SE boundary) |
| 1  | 210 | −1280 | −545  | 735 m | full |
| 2  | 360 | −1257 | −517  | 740 m | full |
| 3  | 510 | −1235 | −505  | 730 m | full |
| N  | 580 | −1224 | −882  | 342 m | partial (NW corner) |

- 10 waypoints; ~5.9 km; ~8.2 min
- Extra ~460 m vs old route comes from two diagonal transitions at the edge partials
  (WP01→WP02: 342 m NW; WP07→WP08: 383 m NW)
- All three target vehicles remain in coverage: Car_01 (N=350) → strip 2,
  Car_02 (N=150) → strip 1, Car_03 (N=450) → strip 3

**Cross-track swath unchanged** — 125 m (HFOV) perpendicular to flight regardless of
heading; 150 m spacing → 25 m gap, same as before.

**Files changed:** `control/px4_commander.py` (SURVEY_WPS), `tools/live_trace.py` (SURVEY_WPS mirror)

---

## 2026-06-07 — Three target vehicles placed in detection zone (cesium_scene.py)

Added three procedural sedan models inside the YOLO detection zone for end-to-end
survey-detection pipeline testing.

**Previous state:** one car at `(23.452028, 120.283829)` — approximately N=125, E=−235 m
from home. This is **outside** the detection zone (nearest edge is at E≈−454).

**New state:** three cars spread across the zone, one per strip region:

| Prim | lat | lon | NED north | NED east | Yaw | Strip |
|------|-----|-----|-----------|----------|-----|-------|
| `/World/Car_01` | 23.454017 | 120.279272 | +350 m | −700 m | 45° (NE) | Strip 1 |
| `/World/Car_02` | 23.452218 | 120.277309 | +150 m | −900 m | 270° (W) | Strip 2 |
| `/World/Car_03` | 23.454917 | 120.275348 | +450 m | −1100 m | 135° (SE) | Strip 4 |

All positions verified inside the buffered zone polygon (30 m inward from raw corners).
Varying yaw angles give realistic parked orientations.

Each model is built by `make_car()`: USD `Cube`/`Cylinder` primitives — white sedan
body, cabin, hood, trunk, bumpers, dark tinted windows, head/tail-lights, four
wheels with rims, and a roof identification panel. Approximate dimensions: Toyota Altis
class (4.64 m × 1.775 m × 1.45 m).

**Files changed:** `simulator/cesium_scene.py`

---

## 2026-06-07 — live_trace.py: raw zone boundary + explicit legend

Added the actual detection zone boundary (before the 30 m inward buffer) as a second
polygon overlay to `tools/live_trace.py`.

**Changes:**
- Added `RAW_ZONE_VERTS` constant (NW/NE/SE/SW raw corners in N/E metres from home):
  ```python
  RAW_ZONE_VERTS = [
      (677.0, -1240.0),  # NW
      (531.0,  -454.0),  # NE
      (-48.0,  -563.0),  # SE
      ( 97.0, -1327.0),  # SW
  ]
  ```
- Added solid white polygon (`fill=False, edgecolor="#ffffff", linewidth=1.5`) for the
  raw zone boundary — drawn before the existing orange dashed buffered zone polygon.
- Replaced auto-generated legend with an explicit `handles=` list using `Line2D` proxy
  artists, covering both zone boundaries, planned route, actual path, home marker, and
  detection marker. (Auto-legend lost the zone patches; proxy artists fix this.)
- Updated docstring to list both overlays.

**Live_trace top view now shows:**
- **Solid white** = actual detection area boundary (raw corners)
- **Orange dashed with faint fill** = 30 m inward buffered boundary (was the only boundary before)

**Files changed:** `tools/live_trace.py`

---

## 2026-06-07 — Survey complete: RTL replaced by explicit fly-home + AUTO.LAND

**Problem:** After completing all 12 survey waypoints the drone kept flying south instead
of returning home.

**Root cause:** `set_mode("RTL")` in PX4 requires a GPS-derived home position. With
external-vision-only localisation there is no GPS home. When RTL silently fails or is
ignored, the last active velocity setpoint (southward, toward WP11) continues to execute
indefinitely in OFFBOARD mode.

**Fix:** Stay in OFFBOARD mode and command the return explicitly:
```python
# Survey complete — fly back to home, then land
cmd.go_to_ned(0.0, 0.0, TAKEOFF_ALT, timeout=300.0, speed=SURVEY_SPEED)
cmd.set_mode("AUTO.LAND")
cmd._spin_until(lambda: not cmd._state.armed, timeout=150.0)
```
Same fix applied to the `KeyboardInterrupt` handler. `go_to_ned()` continuously publishes
position setpoints during the return, so OFFBOARD mode never drops and the drone reliably
reaches home before switching to AUTO.LAND.

**Files changed:** `control/px4_commander.py`

---

## 2026-06-07 — Refactor: DIVERT state machine removed; yaw-corrected pixel projection only

**What changed:** The previous detection response (divert to centre the car in frame,
then log) was replaced with a pure position-calculation approach. The drone never
interrupts the survey route.

**Removed:**
- `SurveyState` enum (`SURVEY` / `DIVERT`)
- `DETECT_RADIUS = 10.0` constant
- `interruptible` parameter from `go_to_ned()`
- `_survey_state`, `_divert_n`, `_divert_e`, `_divert_cat`, `_divert_conf` from `__init__`
- ~80 lines of DIVERT branch in `main()` survey loop

**New `_cb_detections()`** — project pixel offset to world coordinates using
yaw-corrected GSD, apply dedup, log. No state changes, no flight interruption:
```python
q       = self._drone.pose.orientation
yaw_enu = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y**2 + q.z**2))
h    = -yaw_enu                              # NED heading (CW from north)
dx_m =  (cx - CAM_W / 2) * gsd_x            # drone-right
dy_m = -(cy - CAM_H / 2) * gsd_y            # drone-forward
de   = dx_m * math.cos(h) + dy_m * math.sin(h)
dn   = -dx_m * math.sin(h) + dy_m * math.cos(h)
obj_n = cur_n + dn;  obj_e = cur_e + de
```

**Survey loop** simplified to sequential `wp_idx += 1` — no DIVERT branch:
```python
while wp_idx < len(SURVEY_WPS):
    reached = cmd.go_to_ned(wn, we, wagl, timeout=WAYPOINT_TIMEOUT, speed=SURVEY_SPEED)
    wp_idx += 1
```

**Why:** The divert approach required re-acquiring AnyLoc anchor at the off-route position,
then navigating back — adding ~30 s per detection and risking position drift. Pixel
projection gives sufficient (~30 m) accuracy for a car location log without any flight
interruption. The 30 m `DEDUP_RADIUS` prevents re-logging from successive passes.

**YOLO confidence threshold** also raised 0.30 → 0.50 in this session to reduce false
positives from satellite imagery watermarks.

**Files changed:** `control/px4_commander.py`

---

## 2026-06-07 — live_trace.py: filter detections to current flight only

**Problem:** `detections.csv` accumulates across flights (px4_commander.py appends,
never clears). On a new flight, `live_trace.py` showed all markers from every previous
flight, cluttering the map.

**Fix:** Parse the flight start time from the trace filename
(`trace_YYYYMMDD_HHMMSS.csv` → Unix epoch) and pass it as `min_timestamp` to
`read_detections()`. Any CSV row whose `timestamp` field predates the current flight
is skipped. If the filename format cannot be parsed (e.g. a renamed file), the filter
falls back to `min_timestamp=0.0` and all rows are shown.

```python
# in main():
dt = datetime.datetime.strptime(fname, "trace_%Y%m%d_%H%M%S.csv")
_flight_start_epoch[0] = dt.timestamp()

# in read_detections():
if float(row["timestamp"]) < min_timestamp:
    continue
```

**Files changed:** `tools/live_trace.py`

---

## 2026-06-07 — Detection deduplication added to px4_commander.py

**Problem:** After logging a vehicle and resuming the survey route, the same car
remained in the camera frame and `_cb_detections()` immediately fired again,
triggering a second divert to the same target. The drone would yo-yo between the
survey waypoint and the car indefinitely.

**Fix:** Location-based deduplication in `_cb_detections()`:

```python
DEDUP_RADIUS = 30.0   # m — suppress re-divert within this of a logged position

# in __init__:
self._logged_positions = []   # (north_m, east_m) of logged detections

# in _cb_detections(), before setting DIVERT:
for ln, le in self._logged_positions:
    if math.hypot(obj_n - ln, obj_e - le) < DEDUP_RADIUS:
        return   # already logged — skip divert

# in _log_detection():
self._logged_positions.append((north_m, east_m))
```

The 30 m radius covers the ~20 m AnyLoc position uncertainty so the same physical
car does not re-trigger even if its estimated position drifts between passes.
A different car more than 30 m away still triggers a normal divert.

**Files changed:** `control/px4_commander.py`

---

## 2026-06-07 — live_trace.py updated for survey mission

`tools/live_trace.py` updated to match the survey mission in `px4_commander.py`.

**Top-view additions:**
- **Buffered zone boundary** — orange dashed polygon (30 m inward from raw zone corners)
  filled with a faint orange tint; `ZONE_VERTS` mirrored from `px4_commander.py`
- **Planned survey route** — grey dotted line from Home through all 12 `SURVEY_WPS` in order,
  so flown path vs planned route can be compared visually
- **WP labels** — small grey dots with labels (ENTRY, WP01–WP11)
- **Detection markers** — red stars plotted from `detections.csv`; file mtime is checked each
  frame so new detections appear within 200 ms; filtered to current flight only by
  comparing `timestamp` against flight start epoch parsed from the trace filename

**Other changes:**
- `TARGET_AGL` 90 m → 65 m (dashed reference in altitude panel)
- Initial axis limits pre-set to cover the full survey zone (E −1450..250, N −200..800)
- Status bar: `nearest=WPxx(dist m)` instead of `dist_to_WP` (single hardcoded point);
  appends `dets=N` when detections are logged
- `HOME_LAT/LON` + geodetic conversion added so detection lat/lon → plot coordinates

**Files changed:** `tools/live_trace.py`

---

## 2026-06-07 — Survey mission implemented: px4_commander.py PX4-9 ✓

### `control/px4_commander.py` — complete rewrite for 6-strip lawnmower survey

Previous file: `WAYPOINTS` with a single test WP at (531.2, −453.9); `go_to_ned()` at 5 m/s hardcoded.

New file replaces the single-WP test with a full survey mission at 12 m/s / 65 m AGL:

#### Module-level additions

| Symbol | Value | Purpose |
|--------|-------|---------|
| `SURVEY_WPS` | 12 entries | 6-strip boustrophedon waypoints at 65 m AGL |
| `SURVEY_SPEED` | 12.0 m/s | strip cruise speed |
| `DETECT_RADIUS` | 10.0 m | vehicle centring arrival threshold |
| `ZONE_VERTS` | 4 CW corners | 30 m inward buffered boundary (NW'→NE'→SE'→SW') |
| `CAM_W/H` | 1024 / 768 | camera resolution for GSD computation |
| `HFOV_DEG / VFOV_DEG` | 88.0 / 65.1 | camera half-angle for ground offset |
| `VEHICLE_CLASSES` | car,van,truck,bus | classes that trigger a divert |
| `DET_LOG` | `detections.csv` | detection log path (project root) |
| `SurveyState` | enum SURVEY/DIVERT | state machine for main survey loop |
| `_in_buffered_zone(n, e)` | module function | ray-casting point-in-polygon |

#### New methods

- **`_cb_detections(msg)`**: subscribes to `/yolo/detections` (vision_msgs 5.x Jazzy —
  `bbox.center.position.x/y`). Computes ground offset from bbox centre using GSD at current AGL.
  If object inside `ZONE_VERTS`: sets `_survey_state = DIVERT` and records divert target.
  If outside zone: logs position only (no flight divert).
  Ignores new detections while already in DIVERT state.

- **`_log_detection(cat, conf, n, e, agl)`**: appends one CSV row to `detections.csv`
  (`timestamp, category, confidence, lat, lon, agl_m`). Auto-writes header on first row.

#### `go_to_ned()` signature change

`go_to_ned(north, east, agl, timeout, speed=5.0, radius=None, interruptible=False)`

- `speed`: horizontal cruise (survey uses 12.0, divert uses 12.0)
- `radius`: arrival distance (survey uses 60 m default; divert uses `DETECT_RADIUS = 10 m`)
- `interruptible`: if `True`, returns `False` immediately when `_survey_state == DIVERT`

#### Survey state machine in `main()`

```
while wp_idx < len(SURVEY_WPS):
    go_to_ned(WP, speed=12, interruptible=True)
    if DIVERT:
        go_to_ned(divert_target, radius=10, interruptible=False)
        _log_detection(...)
        _survey_state = SURVEY        # resume same wp_idx
    elif reached:
        wp_idx += 1                   # advance
    else:                             # timeout
        wp_idx += 1                   # skip
→ set_mode("RTL") → wait disarm
```

Ctrl-C at any point → RTL.

---

## 2026-06-07 — Survey mission planned: lawnmower route + car detection response

### Plan: detection zone survey at 12 m/s, 6 strips, 65 m AGL

Detection zone defined by four GPS corners west of home (23.450868°N, 120.286135°E):

| Corner | Lat | Lon | NED north (m) | NED east (m) |
|--------|-----|-----|---------------|--------------|
| NW | 23.45695 | 120.27399 | +677 | −1240 |
| NE | 23.45564 | 120.28169 | +531 | −454 |
| SE | 23.45044 | 120.28062 | −48  | −563 |
| SW | 23.45174 | 120.27314 | +97  | −1327 |

Zone dimensions: ≈ 800 m E-W × 650 m N-S (≈ 0.52 km²). AnyLoc error ~20 m → 30 m
inward buffer applied to all boundary edges. Buffered corners:
NW'(642,−1215), NE'(507,−489), SE'(−13,−587), SW'(121,−1293).

**Original plan (9 strips, 100 m spacing, 5 m/s) → ≈ 24 min flight time.**  
**Revised plan (6 strips, 150 m spacing, 12 m/s) → ≈ 7.8 min flight time.**

Trade-off: 25 m unscanned gap between strips (150 m spacing vs 125 m footprint). Cars
in the exact centre of a gap could be missed; cars on roads or in typical parking areas
will be in camera coverage on at least one strip.

#### Ordered waypoints (north_m, east_m, 65.0 m AGL)

```
HOME   (0, 0)          takeoff at 12 m/s cruise
ENTRY: (210,  −545)    south end strip E (partial, NE wedge)
WP01:  (517,  −545)    north end strip E
WP02:  (545,  −695)    north end strip 1
WP03:  (8,    −695)    south end strip 1
WP04:  (36,   −845)    south end strip 2
WP05:  (573,  −845)    north end strip 2
WP06:  (601,  −995)    north end strip 3
WP07:  (65,   −995)    south end strip 3
WP08:  (93,   −1145)   south end strip 4
WP09:  (629,  −1145)   north end strip 4
WP10:  (408,  −1250)   north end strip W (partial, SW wedge)
WP11:  (113,  −1250)   south end strip W
HOME   (0, 0)          RTL / land
```

#### Detection response

When YOLO detects a vehicle (car/van/truck/bus) inside the buffered zone:
1. Compute ground offset from bounding box centre (GSD_x≈0.123 m/px, GSD_y≈0.108 m/px).
2. Fly to centre the vehicle in frame; arrival threshold 10 m.
3. Append to `detections.csv`: `timestamp, category, confidence, lat, lon, agl_m`.
4. Resume survey from saved waypoint index.

If divert target is outside buffered zone, log position only — no divert flight.

#### Code changes (implemented 2026-06-07 — PX4-9 ✓)

| File | Change |
|------|--------|
| `control/px4_commander.py` | Full survey mission — see section below |

Full plan: `instructions/survey_mission_plan.md`.

---

## 2026-06-07 — Mission AGL 90 m → 65 m; AnyLoc database single-altitude; Jetson distributed plan

### Change: mission cruise altitude 90 m → 65 m

`TAKEOFF_ALT` default in `px4_commander.py` changed from 90.0 → 65.0 m. The waypoint
inherits this via `TAKEOFF_ALTS` in `WAYPOINTS`. `TAKEOFF_ALT=<m>` env override still works.

Files: `control/px4_commander.py`.

---

### Change: AnyLoc database single AGL layer (65 m only)

Database was 36,673 entries spanning AGL 60–120 m in 5 m steps (13 layers × ~2,821
positions). Since the mission now flies at a fixed 65 m AGL, all other layers are unused
weight. Defaults changed to `--agl-min 65 --agl-max 65` → single layer → **~2,820
entries**. Database loads ~13× faster; FAISS search is ~13× faster.

Rebuild required:
```bash
conda run -n isaac_sim_test python anyloc/build_database.py --rebuild
```

Files: `anyloc/build_database.py`, `anyloc/README.md`.

---

### Plan: distributed sim — PC runs Isaac Sim + PX4 SITL, Jetson runs everything else

Written to `instructions/jetson_distributed_plan.md`. Jetson runs the identical software
stack it will run on the real drone: MAVROS, commander, AnyLoc, YOLO. PC is pure
simulation infrastructure.

- **Network:** Jetson Orin NX RJ45 → same router as PC. `ROS_DOMAIN_ID=0` on both.
- **MAVLink bridge (Option A):** MAVProxy on PC connects to PX4 SITL (:14580) and
  forwards MAVLink to Jetson (:14540). MAVROS on Jetson uses `udp://:14540@` unchanged.
  No PX4 SITL config changes needed.
- **Topics PC → Jetson (DDS):** `/drone/camera/image_raw`, `/drone/pose`, `/drone/agl`,
  `/drone/state` (for Phase 1 kinematic truth VPE)
- **No topics Jetson → PC** — MAVLink (not ROS2) carries PX4 commands via MAVProxy
- **Code change 1:** `anyloc/ros2_node.py` `_publish()` — dynamic covariance
  `max(1, error_m²)` instead of hardcoded 20 m²
- **Code change 2:** `px4_commander.py` — subscribe to `/anyloc/pose_estimate` instead
  of polling `latest_estimate.json`
- **Code change 3:** `run.sh` — `--jetson-sim` flag: PC launches Isaac + PX4 + MAVProxy
  (no MAVROS, no commander windows)
- **Code change 4:** new `run_jetson.sh` — Jetson launches MAVROS + Commander + AnyLoc
  + YOLO in tmux
- **Real hardware:** `run_jetson.sh` passes `--real-hw` to commander (skips Phase 1 VPE);
  MAVROS `fcu_url` changed to `/dev/ttyTHS1:921600`

Files: `instructions/jetson_distributed_plan.md` (new).

---

## 2026-06-07 — Isaac Sim FPS diagnosis; --no-window and --rasterize modes; background publish thread

### Investigation: Isaac Sim render loop FPS bottleneck

**Symptom:** Isaac Sim camera loop running at ~2 fps initially, target 20 fps for reliable VO.

**Diagnosis — three-stage profiling:**

1. Added `[PERF]` timing around `simulation_app.update()` and `rep.orchestrator.step()` + `get_data()` separately.
2. Varied resolution (2048×1536 → 1024×768 → 512×384), rt_subframes (1→0), viewport size (1920×1080 → 1280×720), RTX denoiser (on→off via carb settings), renderer mode (RaytracingLighting → FullRasterization), headless mode (`--no-window`), and `pause_timeline=True`.
3. Split `rep.step` and `get_data` timing: `rep.step=186ms`, `get_data=1ms` — GPU→CPU transfer is instant; entire bottleneck is inside `rep.orchestrator.step()`.

**Result:** `rep.orchestrator.step()` has a **~190 ms fixed floor** invariant to all of the above. It is not GPU rendering (FullRasterization made zero difference), not timeline stepping (`pause_timeline=True` had zero effect), not the denoiser, and not the resolution. The floor is internal to the replicator annotator pipeline.

**Attempted bypass:** replacing `rep.orchestrator.step()` with multiple `simulation_app.update()` calls → annotator returns empty frames regardless of count. `rep.orchestrator.preview()` → also empty. The annotator requires `rep.orchestrator.step()` and cannot be populated by `simulation_app.update()` alone.

**Conclusion:** 5 fps is the hard ceiling with the current replicator setup. The right fix is making navigation robust at 5 fps (larger LK window, VPE at 20 Hz via VO dead-reckoning, vision velocity publishing) — not chasing camera FPS. These are deferred to a later session.

**Optimisations kept (cumulative improvement ~416 ms → ~210 ms):**
- Camera render resolution: 2048×1536 → 1024×768
- `rt_subframes`: 1 → 0
- Viewport: 1920×1080 → 1280×720
- RTX denoiser disabled via carb settings (`/rtx/post/dlss/execMode=0`, `/rtx/denoise/enabled=False`)

Files: `simulator/cesium_scene.py`.

---

### New: --no-window flag (Isaac Sim headless — no display, full camera)

`--no-window` runs Isaac Sim with `"headless": True` in `SimulationApp`. The display window is suppressed; camera, physics, Replicator annotator, and ROS2 topics all run normally. GPU is freed from viewport rendering — `sim.update` drops from 44 ms to 20 ms, but `rep.step` is unchanged at 190 ms.

Parsed before `SimulationApp` initialises (Kit must not have opened a window yet):
```python
_NO_WINDOW = "--no-window" in sys.argv
if _NO_WINDOW:
    sys.argv.remove("--no-window")
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": _NO_WINDOW, ...})
```

`run_chiayi.sh` already forwards unknown flags via `FWDARGS`, so no change needed there. `run.sh` gains `NO_WINDOW` flag, `--no-window` case, `NOWIN_ARG` variable injected into the `run_chiayi.sh` call.

```bash
bash run.sh --tmux --px4 --no-window
bash run.sh --tmux --px4 --no-window --anyloc --detection
```

`--anyloc` and `--detection` work normally with `--no-window` (both require Isaac Sim camera, not the display window). They are only blocked by `--headless` (which runs `drone_sim.py` — no camera at all).

Files: `simulator/cesium_scene.py`, `run.sh`.

---

### New: --rasterize flag (FullRasterization renderer)

`--rasterize` passes `"renderer": "FullRasterization"` to `SimulationApp`. Tested but made zero difference to `rep.step` timing (confirmed the 190 ms is not GPU rendering). Kept as an option for future investigation.

```bash
bash run.sh --tmux --px4 --no-window --rasterize
```

Files: `simulator/cesium_scene.py`, `run.sh`.

---

### New: background publish thread for ROS2 camera publishing

`tobytes()`, PIL JPEG encode, and ROS2 message serialisation are moved to a background daemon thread. A depth-1 queue (`_pub_q`) decouples the render loop from publishing: `put_nowait()` drops a frame silently if the worker is still busy, so the render loop never blocks on publishing. `DRONE_SAVE_FRAMES = False` (was always-on disk write) skips `drone_frames/latest.jpg` by default.

Shutdown: `_pub_q.put(None); _pub_thread.join(timeout=2.0)` before `_trace_f.close()`.

Files: `simulator/cesium_scene.py`.

---

### Files modified

| File | Change |
|------|--------|
| `simulator/cesium_scene.py` | `--no-window` + `--rasterize` flags; RTX denoiser off; background publish thread; `DRONE_SAVE_FRAMES=False`; render res 1024×768; `rt_subframes=0`; viewport 1280×720; `[PERF]` split timing |
| `run.sh` | `--no-window`, `--rasterize` flags; help text; `NOWIN_ARG`/`RASTER_ARG` wired to `run_chiayi.sh` |
| `anyloc/vo_refiner.py` | Default `cam_w=1024, cam_h=768` (matches published camera resolution) |
| `detection/ros2_node.py` | Add `_t_decode_ms` instrumentation; print `decode N ms  infer N ms` |

---

## 2026-06-06 — PX4-7 prep: AnyLoc + detection integration fixes

### Bug: duplicate VPE when AnyLoc node runs alongside commander

**Symptom:** `anyloc/ros2_node.py` published `PoseStamped` (no covariance) to `/mavros/vision_pose/pose` while `px4_commander.py` publishes `PoseWithCovarianceStamped` to `/mavros/vision_pose/pose_cov`. Running both simultaneously sent two conflicting VPE streams into PX4 EKF2.

**Fix:** Removed the `pub_vpe` publisher and `_ned_from_geopose()` helper from `anyloc/ros2_node.py`. The AnyLoc node now writes only to `anyloc/latest_estimate.json`; the commander is the sole MAVROS VPE publisher.

Files: `anyloc/ros2_node.py`, `anyloc/README.md`.

---

### New: AGL gate on AnyLoc and YOLO nodes (MIN_AGL = 50 m)

Both `anyloc/ros2_node.py` and `detection/ros2_node.py` subscribe to `/drone/agl` (std_msgs/Float64) and skip `_cb_image` processing until AGL ≥ 50 m. A one-time log line confirms when inference starts. Initial `_drone_agl = 0.0` (not 50.0) so the gate is always closed at startup until the first AGL reading arrives.

Files: `anyloc/ros2_node.py`, `detection/ros2_node.py`.

---

### Bug: non-straight flight with AnyLoc active (stale JSON updates)

**Symptom:** With AnyLoc running, the drone deviated from the straight-line path to the waypoint. Position corrections appeared as ~2-second bursts orthogonal to the intended course.

**Root cause:** `_write_estimate()` (which writes `latest_estimate.json`) was only called on AnyLoc anchor frames (every 10 camera frames, ~2 s). Between anchors, VO accumulated `_accum_dlat/_accum_dlon` internally but never wrote them to the file. The commander's VPE thread (reading JSON at 20 Hz) received a stale anchor position for ~2 s, then a sudden jump when the next AnyLoc frame fired. This EKF2 position jump triggered a position-controller correction burst in the wrong direction.

**Fix:** Moved `_write_estimate()` outside the `if run_anyloc:` block so it runs on every camera frame. VO-frame writes use `est_lat = anchor_lat + accum_dlat` / `est_lon = anchor_lon + accum_dlon`. The commander's VPE thread now receives a smooth ~6 Hz update stream instead of 2-second jumps. Added `self._last_score = 0.0` to carry the most recent AnyLoc score for reuse on VO frames.

Files: `anyloc/ros2_node.py`.

---

### Bug: 90° VPE yaw discontinuity at Phase 1→2 transition

**Symptom:** Phase 1 VPE sent `yaw = π/2` (ENU North). Phase 2 read `yaw_deg` from `latest_estimate.json`, which was always 0° (ENU East) — causing a 90° yaw jump in EKF2 at the 50 m AGL threshold.

**Root cause:** `cesium_scene.py` encodes `/drone/pose` orientation as `qz = sin(-_kyaw_rad / 2)`. For `_kyaw_rad = 0` (North in NED), this gives `qz = 0, qw = 1` → ENU yaw = 0 (East), not π/2 (North). The `anyloc` node extracted this as `yaw_deg = 0.0` and wrote it to `latest_estimate.json`. The commander used it as the VPE heading, telling EKF2 the drone faces East when it actually faces North.

**Fix:** `px4_commander.py` Phase 2 now hardcodes `yaw = math.pi / 2.0` instead of reading `yaw_deg` from JSON. Since the kinematic drone never yaws (`_kyaw_rad` stays 0 throughout all flights), π/2 (North) is always correct and keeps the VPE heading consistent between phases.

Files: `control/px4_commander.py`.

---

### New: run.sh --anyloc / --detection flags

`run.sh` gains `--anyloc` and `--detection` flags. When Isaac Sim mode is active, `--anyloc` opens tmux window 4 running `anyloc/run_ros2_localizer.sh`; `--detection` (requires `--anyloc`) opens window 5 running `detection/run_ros2_detector.sh`. In headless mode, `--anyloc` is ignored with a warning (no camera frames). Full pipeline: `bash run.sh --tmux --px4 --anyloc --detection`.

Files: `run.sh`.

---

### New: 2-axis gimbal-stabilised nadir camera

**Problem:** The drone camera is a child of `/World/Drone` in USD. It inherits the drone's pitch and roll, so camera frames tilt with the drone during flight. AnyLoc and YOLO expect a level nadir image. Additionally, the image top should follow the drone nose direction so that VO pixel displacements map cleanly to the drone's heading frame.

**Fix:** Added a gimbal orient op to `/World/Drone/Camera`. Each render loop frame it is set to cancel roll and pitch while preserving yaw:

```
camera_local = conj(drone_quat) × yaw_only_quat
→ camera world orient = yaw_only  (nadir + heading-aligned)
```

Quaternion components each frame:
```python
_cy = cos(_yaw_CCW / 2);  _sy = sin(_yaw_CCW / 2)
drone_cam_orient_op.Set(Gf.Quatd(
     _qw*_cy + _qz*_sy,   # w
    -_qx*_cy - _qy*_sy,   # x
     _qx*_sy - _qy*_cy,   # y
     _qw*_sy - _qz*_cy,   # z
))
```

Result: camera looks straight down regardless of pitch/roll; image top always points toward the drone's nose. The 15 cm translate offset is unaffected (TranslateOp precedes OrientOp in the xformOp stack).

Files: `simulator/cesium_scene.py`.

---

### Fix: VO yaw convention corrected for heading-aligned gimbal

`anyloc/ros2_node.py` was passing `yaw_deg = 0` to `VORefiner.update()`. VORefiner's `yaw=0` convention means "North-pointing camera". With the 2-axis gimbal, the camera now follows the drone's heading — so the yaw fed to VO should be the drone's compass bearing (CW from North).

`/drone/pose` encodes orientation as `−_kyaw_rad` (not the correct `π/2 − _kyaw_rad`), so `self._drone_yaw = −_kyaw_rad = −(compass_bearing_rad)`. Therefore:

```python
_vo_yaw = -math.degrees(self._drone_yaw)  # = compass_bearing_deg
```

In simulation (`_kyaw_rad=0`, drone faces North): `_vo_yaw = 0` ✓ (North-pointing camera convention). On real hardware with drone yaw, this correctly rotates raw pixel displacements to world ENU.

Files: `anyloc/ros2_node.py`.

---

### New: AP-IMX900-Mini-USB3-I5 camera resolution match

Updated all modules to match the real hardware camera. Final values are from the AP-IMX900-Mini-USB3-I5 spec sheet (initial implementation incorrectly computed FOV from raw IMX900 sensor pitch; corrected after reading the actual datasheet):

| Spec | Old (sim default) | New (spec sheet) |
|------|-------------------|-----------------|
| Resolution | 640×480 | **2048×1536** |
| HFOV | 90° | **88°** |
| VFOV | 73.7° | **65.1°** |
| DFOV | — | 113.1° |
| Focal length | 18 mm | **3.1 mm (EFL)** |
| Pinhole aperture H | 36 mm | **5.987 mm** (2×3.1×tan 44°) |
| Pinhole aperture V | 27 mm | **3.957 mm** (2×3.1×tan 32.55°) |
| GSD @ 80 m AGL | ~250 mm/px | **~75 mm/px** |

> **Note on FOV:** The spec-sheet H/V values (88°×65.1°) are consistent with an equisolid wide-angle lens, not a rectilinear pinhole — `sin(44°)/sin(32.55°) = 1.337 ≈ 2048/1536 = 1.333`. Isaac Sim uses a pinhole (rectilinear) model; the spec-sheet H and V angles are used directly as the pinhole FOV, approximating the real lens.

**YOLO:** `detect()` now accepts `imgsz=1280` (default). VisDrone targets are small; upsizing the inference resolution recovers small-object detections that letterbox-at-640 misses. Ultralytics letterboxes internally so no aspect-ratio distortion.

**Detection postview:** rendered at 1024×768 (half native) with bounding boxes scaled proportionally (`_sx = _sy = 0.5`).

> **⚠ AnyLoc database rebuild required.** The old database was built with FOV 90°×73.7°; the correct FOV is 88°×65.1°. Satellite patches in the DB correspond to the old footprint and will produce wrong matches at runtime.
> ```bash
> conda run -n isaac_sim_test python anyloc/build_database.py --rebuild
> ```

Files: `simulator/cesium_scene.py`, `anyloc/localizer.py`, `anyloc/build_database.py`, `anyloc/vo_refiner.py`, `detection/detector.py`, `detection/ros2_node.py`.

---

### Files modified

| File | Change |
|------|--------|
| `anyloc/ros2_node.py` | Remove duplicate VPE publisher; AGL gate (MIN_AGL=50m); write JSON every frame (VO-smoothed); VO yaw = compass bearing |
| `detection/ros2_node.py` | Add `/drone/agl` subscriber; AGL gate (MIN_AGL=50m); postview at 1024×768 with scaled boxes |
| `control/px4_commander.py` | Phase 2 VPE yaw hardcoded π/2 (was reading yaw_deg=0 from JSON) |
| `run.sh` | Add `--anyloc` and `--detection` flags; windows 4 and 5 |
| `anyloc/README.md` | Remove `/mavros/vision_pose/pose` row; add file-based VPE note |
| `simulator/cesium_scene.py` | 2-axis gimbal: cancel roll+pitch, preserve yaw; camera top follows drone nose; AP-IMX900 2048×1536, aperture 5.987×3.957 mm, HFOV=88°, VFOV=65.1° |
| `anyloc/localizer.py` | HFOV=88°, VFOV=65.1° |
| `anyloc/build_database.py` | HFOV=88°, VFOV=65.1° |
| `anyloc/vo_refiner.py` | Default cam_w=2048, cam_h=1536, hfov=88°, vfov=65.1° |
| `detection/detector.py` | `detect(pil_img, imgsz=1280)` for better small-object detection |

---

## 2026-06-05 — PX4 physics sign fix; second-order model in Isaac Sim; flight trace tools; end-to-end confirmed

### Bug: northward runaway on takeoff (physics sign inversion)

**Symptom:** After PX4 arms and commands forward flight, the drone accelerated northward uncontrollably. Headless `drone_sim.py` run showed N position increasing exponentially despite WP being NE.

**Root cause:** In PX4 body-FRD frame, positive pitch = nose UP. A nose-up pitch produces a southward horizontal thrust. The code had `_kbfwd = +_kthrust * math.sin(_kpitch)`, which mapped nose-up to northward acceleration — positive feedback instead of negative. PX4 commands nose-up to brake northward motion, but the inverted sign accelerated it further.

**Fix:** Reverted to `_kbfwd = -_kthrust * math.sin(_kpitch)`. This gives the correct negative-feedback loop: nose-up → southward force → brakes northward motion.

Files: `control/drone_sim.py`, `simulator/cesium_scene.py`.

---

### Bug: Isaac Sim drone hovers but altitude slowly sinks (first-order motor model oscillation)

**Symptom:** With Isaac Sim, drone reached 90 m AGL via VPE but AGL slowly decayed to ~78 m over 60 s. Horizontal motion worked but altitude was not maintained.

**Root cause:** `cesium_scene.py` had not received the second-order angular rate model that was already in `drone_sim.py`. It still used the first-order τ=0.15 s model. At 100 Hz step rate (dt=10 ms), τ=0.15 s = 15 steps. The motor commands oscillate at 5–10 Hz; the first-order low-pass averages them. Due to Jensen's inequality on `cos(pitch)`, the average of `cos(oscillating pitch)` < `cos(0)` = 1, so mean vertical thrust is less than the commanded value → altitude sink.

**Fix:** Ported the second-order angular rate model from `drone_sim.py` to `cesium_scene.py`'s `_run_physics()`:
- Added `_K_PITCH_ACCEL = 80.0` and `_K_PITCH_DAMP = 12.0` constants
- Added `_kpitch_rate, _kroll_rate = 0.0, 0.0` state variables
- Replaced PX4 first-order block with: `dω/dt = K_ACCEL * mean_p * diff − K_DAMP * ω`, then `θ += ω * dt`
- Ground constraint resets rates to 0

Files: `simulator/cesium_scene.py`.

---

### Fixes: run.sh pkill self-kill; commander stdout buffering

**pkill self-kill:** `pkill -9 -f 'px4'` matched `bash run.sh --px4` and killed the run.sh process itself. Fixed pattern to `'/px4 |bin/px4$|mavros_node|px4_commander'` — specific enough to not match the launcher.

**Commander stdout buffering:** `px4_commander.py` piped through `tee` used Python's block buffer (4 kB). The log showed only 6 lines for the entire flight — all remaining output was trapped in the buffer. Fixed by adding `PYTHONUNBUFFERED=1` to `control/launch_commander_px4.sh`.

Files: `run.sh`, `control/launch_commander_px4.sh`.

---

### New: flight trace CSV + live viewer + post-flight plotter

Both `drone_sim.py` and `cesium_scene.py` now write a 5 Hz CSV trace to `simulator/flight_traces/trace_<YYYYmmdd_HHMMSS>.csv`:

```
t_s, east_m, north_m, agl_m, vn_ms, ve_ms
```

**`tools/live_trace.py`** — real-time viewer using `matplotlib.animation.FuncAnimation` at 200 ms:
- Dark theme; top view + altitude vs time panel
- Overlays: survey route, buffered zone boundary, detection markers (updated 2026-06-07)
- Status bar: `t / E / N / AGL / nearest WP / det count`
- Auto-expands axes; auto-finds newest CSV (waits if none exists)
- Usage: `python3 tools/live_trace.py [<file>]`

**`tools/plot_trace.py`** — post-flight two-panel plot:
- Top view (East vs North) + altitude vs time; can overlay multiple traces with `--all`
- Saves `simulator/flight_traces/trace_plot.png`
- Usage: `python3 tools/plot_trace.py [<file>] [--all]`

---

### End-to-end flight results

**Headless (drone_sim.py + PX4 SITL):**
- Takeoff → 90 m AGL: stable, ~45 s
- WP nav (N=531, E=−454): horiz_err < 60 m confirmed
- Motor oscillation: zero (second-order model); yaw stable at 0°

**Isaac Sim (cesium_scene.py + PX4 SITL):**
- Same 699 m waypoint leg, horiz_err < 60 m confirmed
- Altitude stable at 90 m (second-order model fixed the sink)
- Flight trace saved and verified with `tools/plot_trace.py`

### Files modified

| File | Change |
|------|--------|
| `control/drone_sim.py` | Physics sign fix (`_kbfwd = -thrust*sin(pitch)`); flight trace CSV at 5 Hz |
| `simulator/cesium_scene.py` | Second-order angular rate model ported; physics sign fix; flight trace CSV at 5 Hz |
| `run.sh` | Narrowed pkill pattern; added tee log for commander |
| `control/launch_commander_px4.sh` | Added `PYTHONUNBUFFERED=1` |
| `tools/live_trace.py` | New — real-time flight trace viewer |
| `tools/plot_trace.py` | New — post-flight two-panel plotter |

---

## 2026-06-03 — AnyLoc database rebuilt from NLSC; crash-detect disarm fixed; NED convention confirmed

### Bug: Drone disarms mid-flight at ~80 m AGL (motors cut to 1000 PWM)

**Symptom:** During NAV_TAKEOFF climb, motors reach [1704, 1324, 1395, 1866] at ~80 m AGL, then cut to 1000 PWM (disarmed). Drone falls to ground.

**Root cause:** `ATC_RAT_RLL_I = 0.02` and `ATC_RAT_PIT_I = 0.02` accumulate over the 45-second climb. The I-terms drive a persistent motor differential that grows with altitude, eventually exceeding ArduPilot's crash-detect tilt threshold (~30°). `FS_CRASH_CHECK` defaults to 1 (enabled) — disarms on crash detection.

**Fix:**
- `ATC_RAT_RLL_I 0.0` and `ATC_RAT_PIT_I 0.0` — zeroed to prevent I-term windup over long climbs
- `FS_CRASH_CHECK 0` — disabled (kinematic model + VPE position corrections require tilt angles that exceed the threshold; re-enable on real hardware)

**Note:** The original "drop to 0 m" symptom reported was this crash-detect disarm, not a z-convention error. The NED setpoint convention (z = -AGL) is correct.

### Investigation: MAVROS2 setpoint z-convention (confirmed NED passthrough)

To diagnose the "drop to 0 m" symptom, the z-axis convention in position setpoints was tested:

- Sent `z = +90` (ENU-style, positive up): ArduPilot received NED z = +90 → 90 m underground → drone descended to 0 m. **WRONG.**
- Sent `z = -90` (NED, negative = altitude): drone held at 90 m AGL. **CORRECT.**

**Confirmed:** `setpoint_position/local` in MAVROS2 Jazzy is NED passthrough — it passes x,y,z directly to `SET_POSITION_TARGET_LOCAL_NED` without ENU→NED axis swap. `vision_pose` plugin does convert ENU→NED correctly. All position setpoints must be in NED.

### AnyLoc database: self-contained NLSC build, multi-AGL, memory-efficient

**Problem:** `build_database.py` required `satellite_ground.jpg` to exist (written by Isaac Sim) and stored all 36k images + features in RAM → OOM kill at 2320/36673.

**Fixes:**

1. **Self-contained download:** Added `fetch_satellite()` to `build_database.py`. If `satellite_ground.jpg` is missing, downloads NLSC PHOTO2 tiles directly — no Isaac Sim needed.

2. **Multi-AGL range:** Changed `--agl` (single value) to `--agl-min/max/step`. Default: 60–120 m, step 5 m = 13 levels × ~2821 positions = **36673 entries**. Database covers the full climb and waypoint altitude.

3. **3-pass memory-efficient build:** Replaced single-pass (all images + features in RAM) with:
   - Pass 1: crop → disk (JPEG), keep only path/lat/lon/alt
   - Pass 2: load 2000 random images, build codebook, discard
   - Pass 3: load 8 images at a time, compute VLAD, discard
   - Peak RAM: ~4.5 GB (Pass 2)

4. **`db_meta.json` cache:** Pass 1 writes `anyloc/database/db_meta.json` (lat/lon/alt/path lists). Subsequent `--rebuild` runs skip Pass 1 entirely.

5. **Corrupted `database.pt` fix:** OOM-killed `torch.save()` leaves partial ZIP — `RuntimeError: not a ZIP archive`. Fix: `rm database.pt && python anyloc/build_database.py --rebuild` (db_meta.json skips cropping).

**Satellite imagery upgrade:**

- Old `satellite_ground.jpg`: 4096×4096 px, 1.63 m/px effective (downloaded when RADIUS_M was smaller)
- New: 11264×11264 px, 28 MB, **0.60 m/px** (native zoom-18 resolution, no downscaling)
- `MAX_TEX` raised from 8192 → **16384** in both `cesium_scene.py` and `build_database.py`

**Runtime is fully offline** after build: `database.pt` + `satellite_ground.jpg` + DINOv2 (cached at `~/.cache/torch/hub/`). NLSC is never contacted at runtime.

### Files modified

| File | Change |
|------|--------|
| `control/no_gps.parm` | `ATC_RAT_RLL_I 0.0`, `ATC_RAT_PIT_I 0.0`, `FS_CRASH_CHECK 0` |
| `anyloc/build_database.py` | `fetch_satellite()` added; multi-AGL args; 3-pass memory-safe build; `db_meta.json` cache; `MAX_TEX=16384`; `Image.MAX_IMAGE_PIXELS=None` |
| `simulator/cesium_scene.py` | `MAX_TEX 8192 → 16384` |
| `simulator/satellite_ground.jpg` | Re-downloaded: 11264×11264 px, 0.60 m/px, 28 MB |
| `.gitignore` | Added `dumpcore.sh_*.out`, `dumpstack.sh_*.out` (SITL crash dumps) |

---

## 2026-06-03 (continued) — database.pt truncation bug; AnyLoc node startup; launch scripts; setpoint_raw/local fix

### Bug: database.pt silently truncated to ~2.1 GB (PyTorch/miniz multibyte-path overflow)

**Symptom:** Every rebuild produced a `database.pt` of ~2.1 GB instead of the expected ~7.2 GB. Loading it gave a FAISS index with far fewer entries than 36673, or a silent partial load.

**Root cause:** PyTorch bundles the `miniz` compression library. When the output file path contains multibyte UTF-8 characters (the project lives under `文件/`), a signed 32-bit integer overflow in miniz silently caps writes at ~2 GB and returns success — no exception is raised.

**Fix in `anyloc/build_database.py`:**
- Added `_safe_save(obj, dest_path)` helper: saves to `tempfile.mktemp(dir='/tmp')` (ASCII path, no overflow risk) then uses `shutil.move()` to the final destination.
- Split the save into two files to keep each well under the 2 GB limit:
  - `database_meta.pt` — lats/lons/alts/codebook (~0.6 MB)
  - `database_vlads.pt` — VLAD matrix (7.21 GB, saved via `_safe_save`)
  - `database.pt` — thin wrapper `{'_split': True, 'meta': 'database_meta.pt', 'vlads': 'database_vlads.pt'}` pointing to the two files
- Also frees the model from GPU before saving (`del model; torch.cuda.empty_cache()`) and calls `del vlad_list` before stacking to reduce peak RAM.

**Fix in `anyloc/localizer.py`:**
- `_load_db()` detects `_split=True` in the wrapper dict and loads `database_meta.pt` and `database_vlads.pt` separately before merging.

---

### AnyLoc ROS2 node startup — output buffering and startup time

**Symptom:** After launching the AnyLoc node via `conda run`, the terminal showed 0 bytes of output for the full ~20-minute startup period (loading 6.8 GB VLADs + building FAISS index + loading DINOv2 on GPU). It was impossible to tell whether the node had started or crashed.

**Root cause:** Python output is block-buffered when stdout is a pipe (as in `conda run ... | anything`). All `print()` calls are held in a 4 kB buffer until the node emits a large burst.

**Fix:** Launch with `python3 -u` (unbuffered stdout/stderr). Alternatively, confirm the node is live with `ros2 node list` — `/anyloc_localizer` appears as soon as ROS2 spins up, before the heavy model load completes.

**Confirmed working node output (once ready):**
```
[AnyLoc] DB: 36673 entries, VLAD dim=49152
[AnyLoc] FAISS index: 49152D × 36673
[AnyLoc] Model ready on cuda
[PostView] Waiting for first frame
```

---

### New launch scripts

Three new convenience scripts added to simplify startup:

**`control/launch_sitl.sh`**
Starts ArduPilot SITL directly (calling the `arducopter` binary rather than `sim_vehicle.py`, which corrupts the `--out` port argument). Starts MAVProxy separately with explicit `--out udp:127.0.0.1:14550`.

**`control/launch_commander.sh`**
Sources ROS2 Jazzy and runs `flight_commander.py`. Equivalent to the manual `source /opt/ros/jazzy/setup.bash && python3 control/flight_commander.py` step.

**`run.sh` — top-level tmux launcher**
`--tmux` mode: creates a tmux session and auto-starts cesium_scene.py → SITL → MAVROS2 → flight_commander.py in separate panes.
`--wipe` flag: auto-sends `reboot` to the MAVProxy pane and waits for the second boot (needed for `VISO_TYPE` and `SCHED_LOOP_RATE` to activate after a `--wipe` run).

> **Important:** `cesium_scene.py` must be started **before** SITL. It needs to open UDP 9002 first; if SITL starts first and finds no listener it exits immediately.

---

### Bug: waypoint direction wrong — setpoint_raw/local replaces setpoint_position/local

**Symptom:** After takeoff, drone flew SE (distance increasing) instead of NW toward waypoint N=+531 m, E=−454 m. Multiple NED/ENU swap attempts (sp_x=east/sp_y=north and vice versa) both produced SE motion.

**Root cause uncertainty:** `setpoint_position/local` (via `PoseStamped` on `/mavros/setpoint_position/local`) has contradictory behaviour in MAVROS2 Jazzy. Even after confirming the NED convention (2026-06-01 entry), the waypoint direction remained wrong. The exact mapping of PoseStamped fields to `SET_POSITION_TARGET_LOCAL_NED` is ambiguous in this MAVROS2 version.

**Fix:** Switched `_pos_pub` from `PoseStamped` on `/mavros/setpoint_position/local` to `PositionTarget` on `/mavros/setpoint_raw/local` with:
- `coordinate_frame = PositionTarget.FRAME_LOCAL_NED` (= 1) — explicit, unambiguous NED
- `type_mask` set to ignore velocity, acceleration, yaw, and yaw rate (position-only)
- `position.x = north`, `position.y = east`, `position.z = down` (negative = above origin)

This is a direct `SET_POSITION_TARGET_LOCAL_NED` passthrough with no coordinate conversion at all — the frame is declared explicitly in the message itself, removing all ambiguity.

The hold block was also updated to use `PositionTarget` with `position.z = -TAKEOFF_ALT` (NED down, negative = above origin).

`PositionTarget` was added to the `mavros_msgs.msg` import in `flight_commander.py`.

### Files modified

| File | Change |
|------|--------|
| `anyloc/build_database.py` | `_safe_save()` workaround for PyTorch/miniz multibyte-path truncation; split save into `database_meta.pt` + `database_vlads.pt` + thin wrapper `database.pt`; free GPU model before save; `del vlad_list` before stack |
| `anyloc/localizer.py` | `_load_db()` handles `_split=True` wrapper format |
| `control/flight_commander.py` | `_pos_pub` → `setpoint_raw/local` with `PositionTarget FRAME_LOCAL_NED`; `go_to_ned` and hold block use `position.x=north, y=east, z=down`; `PositionTarget` imported from `mavros_msgs.msg` |
| `control/launch_sitl.sh` | New — starts ArduPilot SITL directly + MAVProxy with explicit UDP out |
| `control/launch_commander.sh` | New — sources ROS2 and runs flight_commander.py |
| `run.sh` | New — tmux launcher with `--wipe` auto-reboot support |

### Current milestone status

- **6m-wp (WIP):** Drone takes off ✓, holds at 90 m ✓, waypoint direction bug fixed via `setpoint_raw/local FRAME_LOCAL_NED`; clean end-to-end run not yet confirmed.

---

## 2026-06-01 — Physics thread at 100 Hz; fix altitude oscillation; debug MAVROS/EKF startup

### Bug: Drone oscillates 0–4 m AGL, never reaches 90 m

**Symptom:** After NAV_TAKEOFF accepted, motors cycle between ~1950 (max) and ~1150 (ground-idle). AGL bounces between 0 and 4 m, never climbs toward 90 m target.

**Root cause:** The kinematic model + SITL bridge ran in Isaac Sim's render loop (~13 Hz). ArduPilot saw physics replies at 13 Hz instead of 100 Hz. The altitude PID I-term (`PSC_ACCZ_I=0.3`) accumulated too aggressively at 13 Hz steps (77 ms each), causing bang-bang oscillation between max and minimum throttle.

**Fix:** Moved the kinematic model and bridge to a dedicated background thread (`_run_physics`, daemon) that runs at 100 Hz using `time.sleep(0.01)`. The render loop now only reads the current state under a `threading.Lock` and updates the drone mesh. `/drone/state` is published from the physics thread at 100 Hz.

### Bug: Isaac Sim PhysX `apply_force_at_pos` silently failed

Earlier in this session, milestone 6l was implemented using `UsdPhysics.RigidBodyAPI` + `omni.physx.apply_force_at_pos`. The drone spawn height stayed frozen at exactly the initial value (0.3 m AGL) — neither gravity nor motor thrust moved it. The exception from `apply_force_at_pos` was silently swallowed by `except Exception: pass`.

**Fix:** Abandoned PhysX force API (incompatible with Isaac Sim 6.0 in this configuration). Replaced with the 6-DOF kinematic model from `drone_sim.py` inlined into `cesium_scene.py`. This avoids the PhysX C++ API entirely while achieving the same result — one fewer process, same physics fidelity.

### Startup lessons learned

| Issue | Root cause | Fix |
|-------|-----------|-----|
| MAVROS not connected | Old `cesium_scene.py` (no bridge on UDP 9002); ArduPilot had no physics state | Restart Isaac Sim with new code; or run `drone_sim.py` as temp fallback |
| EKF flags 0x000 | `VISO_TYPE=1` not active after `--wipe` (needs second boot) | Type `reboot` in MAVProxy console after `--wipe`; wait for "Saved N params" |
| EKF flags 0x000 | ArduPilot SITL not running (`/drone/state` frozen, no ardupilot process) | Start SITL before flight_commander |
| `drone_sim.py` conflict | Both `drone_sim.py` and new `cesium_scene.py` try to bind UDP 9002 | Never run `drone_sim.py` when Isaac Sim is up |

### Files modified

| File | Change |
|------|--------|
| `simulator/cesium_scene.py` | Removed PhysX RigidBodyAPI + apply_force_at_pos; added 100 Hz `_run_physics` thread; render loop simplified to state-read + mesh-update |

---

## 2026-06-01 — Waypoint direction inverted + altitude drop + Isaac Sim physics (milestones 6k, 6l)

### Bug 1: Waypoint flies in opposite direction (NW target → SE movement)

**Symptom:** After reaching 90 m AGL and commanding WP N=+531 E=−454, the drone flew SE with growing error (N=−737→−916, E=+487→+561) and dropped to 0 m AGL.

**Root cause — direction:** MAVROS2 Jazzy `setpoint_position/local` plugin passes the `PoseStamped` x,y,z directly into `SET_POSITION_TARGET_LOCAL_NED` **without** performing the ENU→NED axis swap that the `vision_pose` plugin does correctly. Sending ENU (x=east=−453.9, y=north=531.2, z=+90) arrived at ArduPilot as NED (north=−453.9=south, east=531.2, down=+90=underground), causing the drone to fly in the exact opposite horizontal direction and descend to ground.

**Root cause — altitude drop:** The 5-second post-takeoff hold loop sent no position setpoints. Some ArduPilot firmware variants interpret "no setpoint" as "descend to z=0". Combined with the first waypoint setpoint arriving with z=+90 treated as 90 m underground, the drone immediately hit the ground constraint.

**Root cause — SE drift before liftoff:** `drone_sim.py` ground constraint zeroed vertical velocity but not horizontal velocity. Motor imbalance during spool-up caused horizontal sliding before liftoff.

**Fixes:**

| File | Change |
|------|--------|
| `control/flight_commander.py` | `go_to_ned`: send NED coords (x=north, y=east, z=down); distance check uses ENU drone_state unchanged |
| `control/flight_commander.py` | Post-takeoff hold loop now publishes current-position setpoints in NED at ~10 Hz |
| `control/drone_sim.py` | Ground constraint: zero `_kvn` and `_kve` when on ground (friction — stops pre-liftoff sliding) |

---

### Milestone 6l: Replace drone_sim.py with Isaac Sim PhysX rigid body

**Motivation:** Since Isaac Sim is always running, maintaining a separate kinematic physics process (`drone_sim.py`) added complexity and introduced bugs (missing ground friction, approximate motor model). Isaac Sim's PhysX engine handles all of this correctly for free.

**Changes to `simulator/cesium_scene.py`:**

- Drone root `/World/Drone` gains `UsdPhysics.RigidBodyAPI` (mass=1 kg, diagonal inertia) and an invisible collision sphere (r=0.2 m). PhysX now integrates gravity and thrust forces each simulation step.
- `SITLBridge` instantiated directly in `cesium_scene.py`; each render frame: read rigid body state → send to ArduPilot → receive PWM → apply 4 per-motor thrust forces via `omni.physx.apply_force_at_pos`.
- Publishes `/drone/state` (ENU PoseStamped from PhysX position + quaternion orientation).
- Removed `/drone/state` subscription and `_cb_drone_state` callback (no longer reads from `drone_sim.py`).
- Added `_quat_to_rpy()` and `_quat_rotate_vec()` helpers for attitude extraction and force rotation.
- Motor thrust coefficient: `k_T = M·g/2 = 4.905 N` per unit normalised PWM, hover at p_norm=0.5 (PWM 1500), matching `MOT_THST_HOVER=0.5`.

**drone_sim.py:** Retained as headless-only fallback. Do not run alongside `cesium_scene.py` (both bind UDP 9002).

**New run order (with Isaac Sim):** T1=cesium_scene.py, T2=ArduPilot SITL, T3=MAVROS2, T4=AnyLoc, T5=flight_commander.py. `drone_sim.py` is not in this sequence.

---

## 2026-06-01 — Two critical flight bugs fixed: altitude runaway + 90° waypoint error

### Bug 1: AGL ascends forever (never stabilises at 90 m)

**Symptom:** After NAV_TAKEOFF, the kinematic drone climbed past 90 m with no sign of ArduPilot reducing throttle.

**Root cause:** `EK3_SRC1_POSZ = 1` (barometer). In SIM_JSON mode, the barometric altitude derives from the `"position"` field in the JSON state. That field is intentionally omitted (it acts as a GPS substitute and disrupts EKF3 ExtNav fusion). Without it, the simulated barometer stays permanently at 0 m AGL. ArduPilot's altitude controller always sees `current = 0, target = 90` and commands maximum climb indefinitely.

**Fix:**
- `EK3_SRC1_POSZ 1 → 6` (ExternalNav) in `no_gps.parm`
- VPE thread (`flight_commander.py`): send kinematic AGL as `position.z` (from `_drone_state`); `cov[14] = 1e6 → 0.25` (0.5 m std dev)

ArduPilot's EKF now receives the true altitude via the existing VPE channel and correctly throttles back at 90 m.

---

### Bug 2: Waypoint navigation 90° off course

**Symptom:** `WP 1/4 N=+20 E=+0 ALT=90 m` → `err N=+2.7 E=−42.0 dist=42.1 m`. North command → east movement.

**Root cause:** `drone_sim.py` motor groupings assumed the wrong ArduCopter X-frame layout. Actual assignments from `AP_Motors_Matrix.cpp` (angles from forward/north, clockwise positive):
- ch1 / `pwm[0]` = M1 Front-Right (45°)
- ch2 / `pwm[1]` = M2 Rear-Left (−135°)
- ch3 / `pwm[2]` = M3 **Rear**-Right (−45°) ← code assumed Front-Left
- ch4 / `pwm[3]` = M4 **Front**-Left (135°) ← code assumed Rear-Right

With M3 and M4 positions swapped, the "roll formula" actually computed `rear − front` (pitch differential) and the "pitch formula" computed `right − left` (roll differential). When ArduPilot pitched nose-down to go north, the kinematic model applied it as roll-right — drone moved east. This also caused the 42 m east drift during the 90 m climb (90 s of swapped corrections).

**Fix:** `drone_sim.py` — corrected groupings:

```python
# Before (wrong — M3/M4 positions assumed incorrectly):
_roll_tgt  = ((_p4[1] + _p4[2]) - (_p4[0] + _p4[3])) * _K_MAX_TILT  # was rear − front
_pitch_tgt = ((_p4[0] + _p4[2]) - (_p4[1] + _p4[3])) * _K_MAX_TILT  # was right − left

# After (correct — ch1=M1 FR, ch2=M2 RL, ch3=M3 RR, ch4=M4 FL):
_roll_tgt  = ((_p4[1] + _p4[3]) - (_p4[0] + _p4[2])) * _K_MAX_TILT  # left(RL+FL) − right(FR+RR)
_pitch_tgt = ((_p4[0] + _p4[3]) - (_p4[1] + _p4[2])) * _K_MAX_TILT  # front(FR+FL) − rear(RL+RR)
```

### Files modified

| File | Change |
|------|--------|
| `control/no_gps.parm` | `EK3_SRC1_POSZ 1 → 6` (barometer → ExternalNav) |
| `control/flight_commander.py` | VPE thread: `position.z = drone_agl` (kinematic AGL from `_drone_state`), `cov[14] = 0.25` |
| `control/drone_sim.py` | Roll/pitch motor groupings corrected for ArduCopter X-frame FRAME_TYPE=1 |

---

## 2026-05-31 — Milestone 6i: NAV_TAKEOFF replaces hand-rolled P-controller

### What was done

**Removed SET_ATTITUDE_TARGET P-controller from takeoff()**

With `DISARM_DELAY=0` (added in milestone 6h), the land-detector deadlock no longer occurs. ArduPilot's built-in altitude controller handles the climb to 90 m AGL via NAV_TAKEOFF (CommandTOL). `takeoff()` now only monitors progress and returns when AGL reaches the target. `AttitudeTarget` import, `_att_pub` publisher, and `climb_rate` parameter removed.

**EKF origin now blocking**

`set_ekf_origin()` now retries every 2 s for up to 60 s and aborts the mission if GPS_GLOBAL_ORIGIN (msg 49) echo is not received. Previously it published 10× over 2 s without requiring confirmation, which allowed the drone to fly to the wrong location when the origin was silently rejected (observed: 664 m displacement on subsequent SITL runs).

**Attitude controller gains reduced in no_gps.parm**

Default gains (ATC_ANG_*_P=4.5, ATC_RAT_*=0.135) caused I-term windup with the kinematic drone model, producing motor imbalances and 180 m horizontal drift. Reduced to ATC_ANG_RLL/PIT_P=1.5, ATC_RAT_*_P/I=0.04, D=0.

**launch_mavros.sh hardened**

`pkill -f mavros_node` before launch prevents "Promise already satisfied" crash on repeated runs. `sleep 5` after kill waits for SITL to be ready before MAVROS2 connects.

### Files modified

| File | Change |
|------|--------|
| `control/flight_commander.py` | Removed P-controller, AttitudeTarget import, _att_pub, climb_rate; set_ekf_origin() now blocking; 30s ground abort check in takeoff(); waypoint progress logging |
| `control/no_gps.parm` | Added ATC_ANG_RLL/PIT_P=1.5, ATC_RAT_*_P/I=0.04, D=0 |
| `control/launch_mavros.sh` | pkill stale mavros_node; 5s wait before connecting |
| `README.md`, `instructions/project_plan.md`, `instructions/history.md` | Updated takeoff section, milestone table |

---

## 2026-05-31 — Waypoint instability: position controller runaway + fixes

### What was done

**Symptom:** After successful 90 m AGL takeoff, `go_to_ned()` sent position setpoints → ArduPilot switched from Guided_Attitude to Guided_Pos → position controller applied aggressive horizontal corrections → drone flew to **1458 m AGL** and kept climbing.

**Root cause:** Default `WPNAV_SPEED = 500 cm/s` (5 m/s horizontal) combined with untuned horizontal PIDs (PSC_POSXY_P=1.0, PSC_VELXY_P=2.0) caused extreme tilt (motors: one at 1950 PWM, opposite at 1150 PWM). At 30°+ tilt, vertical thrust drops and horizontal acceleration is large. With no position reference correction (EKF origin unconfirmed), the drone flew away unchecked.

**Additional discovery:** Restarting flight_commander.py without restarting SITL left the drone at 1458 m AGL from the previous run. The new run connected to SITL mid-flight. Startup AGL was 1458 m.

**Fixes applied:**

| Fix | File | Change |
|-----|------|--------|
| Horizontal speed limit | `no_gps.parm` | `WPNAV_SPEED 100` (1 m/s) |
| Horizontal PID reduction | `no_gps.parm` | `PSC_POSXY_P 0.3`, `PSC_VELXY_P 0.5`, `PSC_VELXY_I 0.3`, `PSC_VELXY_D 0.0` |
| EKF origin non-blocking | `flight_commander.py` | Publish 10× over 2 s; no GPS_GLOBAL_ORIGIN echo required |
| go_to_ned distance check | `flight_commander.py` | Use `/drone/state` (kinematic truth) not EKF `_local_pos` |
| ExternalShutdownException | `flight_commander.py` | `try/except` in go_to_ned() and main() waypoint loop |
| Startup AGL sanity check | `flight_commander.py` | Abort if `/drone/state` AGL > 10 m at launch |

**Status:** Fixes applied; not yet tested on clean --wipe SITL.

---

## 2026-05-31 — TAKEOFF SOLVED: attitude P-control to 90 m AGL ✓

### What was done

**Root cause of takeoff failure — land-detector deadlock:**

ArduPilot's land detector kept motors at GROUND_IDLE (1100 PWM). At 1100 PWM, drone_sim.py's kinematic model produces net downward acceleration (thrust < gravity), so the drone never lifts. Baro stays constant → land detector sees no motion → stays at GROUND_IDLE → DISARM_DELAY (10 s) fires → motors drop to 1000 (disarmed). Classic circular deadlock.

Diagnostic that revealed this: SERVO_OUTPUT_RAW (msg 36) motor PWM logging added to the AGL print line. Showed motors going 1000 → 1085 → 1100 → 1000 (exactly 10-second cycle), confirming DISARM_DELAY was the final cause.

**Fix 1: SET_ATTITUDE_TARGET bypasses land detector**

NAV_TAKEOFF (CommandTOL) sets `auto_armed=True` inside ArduPilot. Then immediately publishing SET_ATTITUDE_TARGET via `/mavros/setpoint_raw/attitude` (AttitudeTarget) switches to Guided_Attitude mode. In this mode, ArduPilot calls `set_desired_spool_state(THROTTLE_UNLIMITED)` directly — bypassing the land-detector check entirely. Motors spool up to commanded thrust within 0.5s (MOT_SPOOL_TIME).

**Fix 2: /drone/state for altitude feedback**

After liftoff, EKF barometric altitude (`/mavros/local_position/pose`) diverges when the drone briefly touches the ground. EKF integrates downward velocity and reports negative AGL (observed: −37 m). The P-controller fed with wrong altitude then drives wrong thrust and the drone crashes.

Fix: subscribe to `/drone/state` (published by drone_sim.py at 100 Hz) and read `pose.position.z − HOME_ALT_MSL` as actual AGL. This is the kinematic truth from the physics model, immune to EKF drift.

**Fix 3: DISARM_DELAY 0 in no_gps.parm**

Added `DISARM_DELAY 0` to prevent auto-disarm while debugging. Requires `--wipe` on SITL restart to activate.

**Why position setpoints fail (dead end):**

Both approaches tried:
- Phase A (attitude liftoff to 5 m) + Phase B (position setpoints): position controller switches ArduPilot from Guided_TakeOff → Guided_Pos. Position controller adds aggressive attitude corrections (motors: 1950 vs 1150 PWM), causing oscillation and crash at ~5 m AGL.
- Full P-controller with `/mavros/local_position/pose` altitude: AGL diverges to −37 m after first ground contact, P-controller drives wrong thrust.

**Solution: attitude control for the entire climb**

P-controller in flight_commander.py (NOT ArduPilot's position controller):
- `thrust = 0.50 + 0.004 × (target_agl − agl)` clamped to [0.30, 0.70]
- Below 2 m AGL: minimum thrust = 0.65 (ensures land detector releases)
- SET_ATTITUDE_TARGET at 100 Hz with `orientation.w=1.0` (level)

**Confirmed result:** Drone reached 90 m AGL ✓. Motor balance throughout: ~1563 PWM (Phase A) → ~1640 PWM (climb). No oscillation.

**Remaining issues after 90 m AGL:**
- Waypoints (go_to_ned) timeout because EKF horizontal reference is wrong when GPS_GLOBAL_ORIGIN echo fails (SITL degraded state without `--wipe`)
- ExternalShutdownException during waypoint loop (SITL crashes after extended flight)

### Files modified

| File | Change |
|------|--------|
| `control/flight_commander.py` | Added `AttitudeTarget` publisher (`/mavros/setpoint_raw/attitude`); `/drone/state` subscriber for kinematic altitude; rewrote `takeoff()` as attitude P-controller; removed two-phase position-setpoint approach |
| `control/no_gps.parm` | Added `DISARM_DELAY 0` |
| `README.md` | Updated takeoff sequence, milestone 6h Done |
| `instructions/project_plan.md` | Updated flight control section and milestone table |
| `instructions/history.md` | This entry |

---

## 2026-05-31 — Remove pymavlink; MAVROS2 raw MAVLink for EKF origin + status; two-phase VPE

### What was done

**Removed all pymavlink dependencies from `flight_commander.py`**

[See earlier entry — this session's first half]

---

## 2026-05-31 — Separate drone physics from Isaac Sim; fix VPE + takeoff

### What was done

**Separated `drone_sim.py` from `cesium_scene.py`**

The kinematic physics model and SITL bridge (previously embedded in `cesium_scene.py`) were extracted into a standalone ROS2 node `control/drone_sim.py`. Isaac Sim is now a pure visualiser: it subscribes to `/drone/state` (ENU PoseStamped, 100 Hz) and moves the USD drone mesh. This makes headless flight possible without Isaac Sim running.

- **New:** `control/drone_sim.py` — 6-DOF kinematic model + `SITLBridge` + `/drone/state` publisher
- **Modified:** `simulator/cesium_scene.py` — removed kinematic physics, SITL bridge, and keyboard control; added `/drone/state` subscriber + `_cb_drone_state()` callback
- **Deprecated:** `control/stub_bridge.py` — replaced by `drone_sim.py`

**Switched MAVROS2 and pymavlink from TCP to UDP**

`tcpin:localhost:5762` in `launch_mavros.sh` caused `PermissionError: [Errno 13] Permission denied` on socket bind inside `mavproxy`. Root cause unresolved (pure Python socket tests passed), so switched to UDP to avoid MAVProxy's `tcpin:` binding path entirely.

- `launch_mavros.sh`: `fcu_url:="tcp://localhost:5762"` → `fcu_url:="udp://:14550@"`
- `flight_commander.py`: all `udp:localhost:14550` → `udpin:0.0.0.0:14551`
- SITL command: `--out tcpin:localhost:5762` → `--out udp:127.0.0.1:14550 --out udp:127.0.0.1:14551`
- Removed `--console --map` flags (MAVProxy GUI modules not installed)

**Fixed `MAV_CMD_NAV_TAKEOFF` missing from takeoff sequence**

`flight_commander.py`'s `takeoff()` was publishing position setpoints but never sending `MAV_CMD_NAV_TAKEOFF`. ArduPilot keeps motors at idle in "landed" state regardless of setpoint altitude. Added a `CommandTOL` call at the top of `takeoff()` before the position ramp. `_tof_cli` was already wired up but unused.

**Fixed VPE coordinate order and covariance (EKF POS_ABS was never set)**

Two bugs in `flight_commander.py`'s VPE thread:

1. **x/y swap** — `position.x = north, position.y = east` instead of ENU (x=East, y=North).
2. **Covariance 400 m² too large** — EKF3 only sets `EKF_POS_HORIZ_ABS` when internal position uncertainty is below a few metres. With 20 m std dev measurement covariance, the EKF's uncertainty stays ~20 m and the flag is never set. Reduced to 1 m² (1 m std dev). z covariance unchanged at 1e6 m².

Added diagnostic logging to `wait_ekf_pos()`: prints active EKF flags every 5 s if stuck, e.g. `EKF flags 0x00f: [ATT | VEL_H | VEL_V | POS_H_REL] — waiting for POS_H_ABS`.

### Bugs fixed

| Bug | Symptom | Fix |
|-----|---------|-----|
| Drone mesh frozen at ground in Isaac Sim | `flight_commander` sent setpoints but Isaac Sim AGL didn't change | Extracted kinematic model to `drone_sim.py`; cesium_scene.py subscribes `/drone/state` |
| `PermissionError` on MAVProxy `tcpin:` bind | SITL crashes with `[Errno 13] Permission denied` | Switched to UDP 14550/14551 |
| `Connection refused` on `--out tcp:localhost:5763` | MAVProxy exits immediately | Changed to `--out udp:` |
| `No module named 'console'`/`'map'` | MAVProxy exits | Removed `--console --map` flags |
| `link 1 down` after SITL start | ArduPilot waiting for physics bridge | `drone_sim.py` must start within ~10 s of SITL |
| EKF POS_ABS never set | `flight_commander` stuck on "Waiting for EKF POS_ABS" | Fixed VPE x/y order + reduced covariance to 1 m² |
| Drone never lifts off | AGL stays near 0 despite climbing setpoints | Added `MAV_CMD_NAV_TAKEOFF` to `takeoff()` |

---

## 2026-05-30 — ROS2 migration (Milestone 6e)

### What was done

Migrated all IPC from file polling + direct pymavlink to ROS2 topics + MAVROS2.

- **New:** `control/flight_commander.py` — full ROS2 node replacing `run_flight.py`
  - EKF origin via pymavlink (MAVROS2 Jazzy 2.14 has no service for this)
  - VPE thread publishes `PoseWithCovarianceStamped` to `/mavros/vision_pose/pose_cov`
  - Position setpoints via `/mavros/setpoint_position/local`
  - STABILIZE arm → GUIDED → EKF POS_ABS → takeoff → waypoints → RTL
- **New:** `control/launch_mavros.sh` — MAVROS2 launch script
- **Modified:** `simulator/cesium_scene.py` — added ROS2 node, publishes `/drone/camera/image_raw`, `/drone/pose`, `/drone/agl`; kinematic model driven by ArduPilot PWM via embedded SITL bridge
- **Modified:** `anyloc/ros2_node.py` — subscribes ROS2 camera/pose; publishes VPE + AnyLoc estimates

### Key design: VPE with z=1e6 covariance

`PoseWithCovarianceStamped` on `/mavros/vision_pose/pose_cov` allows setting per-axis covariance. z covariance = 1e6 m² tells EKF3 to ignore VPE altitude and rely on barometer. This prevents EKF innovation gate failures when the stub VPE z differs from baro.

---

## 2026-05-15 — Simulator working

### What was done

Built a working Isaac Sim 6.0.0 scene for Chiayi, Taiwan centred at 23.450868°N, 120.286135°E.

**Data sources (all via Cesium ion REST API — no Cesium for Omniverse extension):**
- Terrain: Cesium World Terrain (asset 1), quantized-mesh-1.0, 9 tiles at level 13
- Buildings: Cesium OSM Buildings (asset 96188), B3DM format, 83 buildings from 4 tiles at level 12
- Imagery: Taiwan NLSC PHOTO2 aerial orthophoto WMTS, zoom 18, resized to 4096×4096

**Why no Cesium for Omniverse extension:**
Cesium for Omniverse v0.22–0.26 targets Kit 105.1/106.5 with Python 3.10. Isaac Sim 6.0.0 uses Kit 106 / Python 3.12. No compatible version exists.

---

### Bugs fixed

**1. Quantized mesh triangle count always 0**
- Cause: erroneous 4-byte alignment padding inserted between vertex data and triangle count in `parse_quantized_mesh()`
- Fix: removed `if off % 4: off += 4 - (off % 4)` — Cesium terrain tiles have no padding there
- File: `simulator/cesium_scene.py` → `parse_quantized_mesh()`

**2. `np.arange` TypeError in building parser**
- Cause: `np.arange(len(vi), np.int32)` passes `np.int32` as stop value, not dtype
- Fix: `np.arange(len(vi), dtype=np.int32)`
- File: `simulator/cesium_scene.py` → `parse_b3dm_buildings()`

**3. Stale terrain tile list with bad URLs**
- Cause: `cesium_terrain_list.json` was cached with relative URLs containing literal `{version}` placeholder
- Fix: deleted the stale cache file; added URL resolution logic in `fetch_terrain_tiles()` to prepend `base_url` for relative templates and replace `{version}` with `"1.2.0"`
- File: `simulator/cesium_scene.py` → `fetch_terrain_tiles()`

**4. Satellite imagery — switched from ESRI to Bing to NLSC**
- ESRI World Imagery and Bing Maps Aerial both use Maxar source for Taiwan — visually identical
- Switched to Taiwan NLSC PHOTO2 orthophoto WMTS (free, no API key, up to zoom 20)
- URL pattern: `https://wmts.nlsc.gov.tw/wmts/PHOTO2/default/GoogleMapsCompatible/{z}/{y}/{x}`
- Note: Bing Maps Aerial via Cesium ion asset 2 returns `externalType: BING` with a Bing API key (not a Cesium tile server) — requires quadkey conversion and Bing Imagery Metadata API call to get tile URL template
- File: `simulator/cesium_scene.py` → `fetch_satellite()`

**5. White wash / overexposure**
- Cause: RTX auto-exposure histogram boosting gain on bright outdoor scene until everything washed white
- Fix:
  - DomeLight intensity: 500 → 200
  - DistantLight intensity: 6000 → 2500
  - Enabled RTX histogram auto-exposure with clamped range: `exposureMin=-4.0`, `exposureMax=0.0`
  - Set ACES filmic tonemapper: `/rtx/post/tonemap/op = 6`
- File: `simulator/cesium_scene.py` → lights section + `carb.settings`

**6. Terrain texture mirrored**
- Cause: USD `UsdUVTexture` uses OpenGL convention where `v=0` = bottom of image. Our JPEG has north at the top, but we were mapping north to `v=0`, so north terrain got south pixels — entire texture was north-south flipped, appearing as a mirror from the camera's viewpoint
- Fix: `v = 1.0 - (SAT_NW_LAT - lat_arr) / (SAT_NW_LAT - SAT_SE_LAT)`
- File: `simulator/cesium_scene.py` → `geo_to_uv()`

---

### Project structure created

```
no_GPS_drone_project/
├── instructions/
│   ├── project_plan.md    # module plans + milestones
│   └── history.md         # this file
├── simulator/             # Isaac Sim — WORKING
├── localization/          # AnyLoc — TODO
├── detection/             # YOLO — TODO
├── control/               # ArduPilot — TODO
└── .gitignore
```

---

---

## 2026-05-17 — Drone + camera + HUD (Milestone 2)

### What was done

Added a controllable quadcopter drone with nadir camera, viewport HUD, and camera toggle to `simulator/cesium_scene.py`.

**USD prims — quadcopter model (~0.8 m span):**
- `/World/Drone` — `Xform` with `TranslateOp` + `RotateZOp` (yaw); starts at `centre_elev + 50 m`
- `/World/Drone/Body` — flat `Cube` (0.28 × 0.28 × 0.08 m), dark-grey
- `/World/Drone/Arm_NE/NW/SW/SE` — thin `Cube` arms at 45°/135°/225°/315°, dark-grey
- `/World/Drone/Motor_NE/…` — upright `Cylinder` pods at arm tips (r=0.035 m)
- `/World/Drone/Prop_NE/…` — flat `Cylinder` propeller discs above each motor (r=0.13 m)
- `/World/Drone/Beacon` — `SphereLight` (orange, 5000 cd) — visible as a coloured dot from the overview camera
- `/World/Drone/Camera` — `Camera` prim, 18 mm focal length, 36×27 mm aperture → **90°×73.7° FOV**, 640×480, clipping 0.1–5000 m

**Nadir orientation:** In a Z-up stage, default USD camera looks along local −Z = world −Z (straight down). No rotation op needed; yawing the parent `Xform` rotates the image around the nadir axis.

**Frame output (`omni.replicator.core`):**
- `rep.create.render_product("/World/Drone/Camera", (640, 480))`
- RGB annotator: RGBA → strip alpha → JPEG → `drone_frames/latest.jpg` every 5 sim steps
- `drone_frames/latest_meta.json` — `{step, lat, lon, alt_m, yaw_deg, frame_w, frame_h}`
- Viewport (Tab, 1920×1080) and render product (640×480) are **intentionally separate** — same camera and 90° HFOV, different aspect ratio and resolution. Viewport is for visual inspection; render product is the ML input.

**HUD overlay (`omni.ui`):**
- Semi-transparent dark window pinned to top-left corner, always on top
- Shows live: `LAT` / `LON` (5 dp) · `ALT` (MSL + AGL) · active `CAM` name
- Updates every sim step; wrapped in try/except so sim still runs if `omni.ui` fails

**Keyboard controls (`carb.input` + `omni.appwindow`):**
- Tab = toggle viewport: overview ↔ drone nadir (edge-detected, one press = one toggle)
- W/S = N/S · A/D = W/E · Q/E = down/up · Z/X = yaw ±1°/step · all ±5 m/step

---

### Bugs fixed

**1. `carb.input.IInput` has no `get_keyboard()` method**
- Cause: `get_keyboard()` lives on the app window, not the input interface
- Fix: `omni.appwindow.get_default_app_window().get_keyboard()`
- File: `simulator/cesium_scene.py` → keyboard setup block

**2. Camera FOV stated as 84°×65° — wrong**
- Cause: arithmetic error; 24 mm / 36×27 mm aperture gives 73.7°×58.7°, not 84°×65°
- Fix: corrected FOV formula `2 × arctan(aperture / (2 × focalLength))` and changed focal length to 18 mm to achieve the desired 90°×73.7°
- Files: `cesium_scene.py` comment, `project_plan.md`, `README.md`

---

---

## 2026-05-18 — Frame capture fix

### Bug fixed

**`_rgb.get_data()` silently returning `None` — no frames saved**
- Cause: `omni.replicator.core` does not render into the render product automatically during a manual `simulation_app.update()` loop. Without an explicit replicator step, `get_data()` always returns `None` and the save block was silently skipped.
- Fix: call `rep.orchestrator.step(rt_subframes=1, delta_time=0.0)` immediately before `get_data()` each capture cycle. This forces the RTX renderer to produce one frame into the render product.
- Added explicit `print` warnings when `get_data()` returns `None` or an empty array, so silent failures are visible in the terminal.
- Added a one-time confirmation message (`[DRONE] Frame capture working`) on the first successful save.
- File: `simulator/cesium_scene.py` → frame capture block in simulation loop

---

---

## 2026-05-20 — AnyLoc localization + dual postview (Milestone 3)

### What was done

Created `anyloc/` with a working AnyLoc visual localization pipeline and two live postview windows.

**Files created:**
- `anyloc/build_database.py` — builds a geo-tagged image database from the NLSC satellite orthophoto
- `anyloc/localizer.py` — AnyLocLocalizer class: DINOv2 ViT-B/14 + intra-normalised VLAD + FAISS nearest-neighbour; `localize(img, agl_m)` re-crops satellite at drone's actual AGL
- `anyloc/run_localizer.py` — main loop: watches `drone_frames/latest.jpg`, runs localisation, shows two matplotlib windows
- `anyloc/requirements.txt` — dependency notes
- `anyloc/database/` — built database (172 entries, VLAD dim=49,152)

**Modified:**
- `simulator/cesium_scene.py` — `latest_meta.json` now also writes `agl_m` and `centre_elev`

**Database:**
- Grid: 200 m step, ±1500 m from scene centre → 172 positions
- Drone AGL: 50 m (sets ground footprint size for satellite crops)
- Satellite crop per position → resize to 640×480 → DINOv2 ViT-B/14 patch features (768-dim)
- faiss.Kmeans k=64 codebook → intra-normalised VLAD → 64×768=49,152-dim descriptors
- FAISS IndexFlatIP (cosine similarity)

**Two postview windows (`run_localizer.py`):**
- `[Drone Camera]` — live `latest.jpg` with ground-truth geo overlay (LAT/LON/ALT MSL/AGL/YAW)
- `[AnyLoc Match]` — satellite crop re-cropped at **drone's actual AGL** at the matched position, with estimated geo overlay (LAT/LON/ALT AGL/ERR/time)
- Text colour: green if error < 200 m, blue otherwise
- Display: matplotlib TkAgg (not cv2 — cv2 in this env is headless)

**Measured performance (RTX 2080 Ti, cuda):**
- DINOv2 inference + VLAD + FAISS search: ~183 ms per frame
- Typical localisation error at 50 m AGL: ~65 m (≈ 1 grid step = 200 m)

---

### Bugs fixed

**1. numpy dual-install conflict (pip numpy 2.3.1 vs conda numpy 1.26.4)**
- Cause: conda-forge faiss-cpu installation pulled in numpy 2.x files over the Isaac Sim numpy 1.26.4, corrupting `numpy/core/_dtype.py`. ANY numpy operation failed.
- Fix:
  - Avoided all numpy operations in the VLAD pipeline — use torch tensors throughout
  - Used `pil.tobytes() → torch.frombuffer()` instead of `np.array(pil_img)` everywhere
  - Used `torch.save()` / `torch.load()` instead of `np.savez_compressed()` for database
  - Used `tensor.numpy()` only at faiss call sites (torch's numpy binding is ABI-compatible with cv2)
  - Force-reinstalled numpy 1.26.4 from conda-forge to restore numpy itself
- Files: `anyloc/build_database.py`, `anyloc/localizer.py`, `anyloc/run_localizer.py`

**2. torchvision.ToTensor() TypeError**
- Cause: `T.ToTensor()` internally calls `np.array(pic, dtype, copy=True)` then `torch.from_numpy()`, both fail with the dual-numpy conflict
- Fix: replaced transform pipeline with `pil.tobytes() + torch.frombuffer()` approach
- Files: `anyloc/build_database.py`, `anyloc/localizer.py`

**3. torch.from_numpy() TypeError**
- Cause: torch checks `isinstance(obj, <torch-numpy>.ndarray)` but the array was from a different numpy install
- Fix: for faiss centroid output, copy via `bytearray(arr.tobytes()) → frombuffer`
- File: `anyloc/build_database.py`

**4. cv2.namedWindow crash — OpenCV headless**
- Cause: cv2 in `isaac_sim_test` was built without GUI support (`GUI: NONE`)
- Fix: replaced all cv2 display calls with **matplotlib (TkAgg backend)**; text overlays drawn with PIL `ImageDraw` to avoid numpy ops
- File: `anyloc/run_localizer.py`

**5. tight_layout UserWarning**
- Cause: `plt.tight_layout()` incompatible with image axes that have no labels
- Fix: replaced with `layout='constrained'` on the figure constructor
- File: `anyloc/run_localizer.py`

**6. UnidentifiedImageError — mid-write race condition**
- Cause: localiser reads `latest.jpg` while the simulator is still writing it, producing a truncated JPEG
- Fix: wrapped `Image.open` + `frame.load()` in `try/except`; on error, prints a warning and retries next 150 ms tick without updating `last_mtime`
- File: `anyloc/run_localizer.py`

**7. AnyLoc match altitude always 50 m**
- Cause: `localize()` returned the DB entry's fixed 50 m AGL regardless of the drone's actual altitude; the match image was a 50 m footprint crop
- Fix: `localize(img, agl_m)` now accepts the drone's AGL, re-crops the satellite orthophoto at that altitude centred on the matched position, and returns `agl_m` as `est_alt`
- Files: `anyloc/localizer.py` (added `_sat_crop`, `_load_sat`, `agl_m` param), `anyloc/run_localizer.py` (passes `drone_agl`)

---

---

---

## 2026-05-20 — AnyLoc grid densification + VO refinement

### What was done

**Grid step reduced 200 m → 50 m (`anyloc/build_database.py`):**
- Changed `--grid-step` default from 200 to 50
- Rebuilt database: 2,821 entries (was 172), VLAD dim=49,152 unchanged
- Expected localisation error: ~15–20 m (was ~65 m)
- Hard accuracy floor at this AGL: ~50 m grid ≈ camera footprint width (~100 m × 75 m at 50 m AGL); going finer produces overlapping images that are indistinguishable

Accuracy table (for reference):

| Grid step | Entries | Expected error |
|-----------|---------|----------------|
| 200 m | 172 | ~65 m |
| 100 m | ~688 | ~30–40 m |
| **50 m (current)** | **2,821** | **~15–20 m** |
| 25 m | ~11,000 | ~8–12 m |

**Visual Odometry (VO) refinement implemented:**

New file `anyloc/vo_refiner.py` — `VORefiner` class using LK optical flow:
- Detects Shi-Tomasi corner features (`cv2.goodFeaturesToTrack`)
- Tracks them with Lucas-Kanade optical flow (`cv2.calcOpticalFlowPyrLK`)
- Median pixel displacement → ground metres → Δlat/Δlon via AGL + FOV + yaw rotation:
  - `raw_east = -dx_px × m_per_px_x` (feature right → drone moved west)
  - `raw_north = +dy_px × m_per_px_y` (feature down → drone moved north)
  - World ENU: `east = raw_east·cos(yaw) + raw_north·sin(yaw)`, `north = -raw_east·sin(yaw) + raw_north·cos(yaw)`
- `reset()` clears tracked state after each AnyLoc re-anchor

Updated `anyloc/run_localizer.py`:
- `ANYLOC_INTERVAL = 10` — full AnyLoc retrieval every 10 frames (~2 s at 5-step sim)
- Between anchors: VO accumulates `accum_dlat / accum_dlon`; final position = anchor + accumulated delta
- Panel 2 mode tag: `ANYLOC` on anchor frames, `VO +Nf` otherwise; also shows tracked point count
- Expected combined accuracy: ~5–10 m between anchor fixes

**Docs and .gitignore updated:**
- `.gitignore` — added `anyloc/database/` and `anyloc/test_output/`
- `README.md`, `project_plan.md` — reflect Milestone 3 done, new database size, VO documented

---

### Bug fixed

**`ok.sum()` hits broken numpy `_core/_methods.py` (numpy 2.x stub)**
- Cause: `cv2.calcOpticalFlowPyrLK` returns a numpy array `status`; calling `.sum()` on `status.flatten() == 1` triggers numpy's Python-level dispatch in `_core/_methods.py`, which is a numpy 2.x file still present in the env
- Fix: `sum(ok.tolist())` — `.tolist()` is C-level (safe), `sum()` on a Python list is pure Python
- File: `anyloc/vo_refiner.py` → `update()`

---

## 2026-05-22 — Geo-constrained AnyLoc search

### Motivation

The original AnyLoc retrieval searches all 2,821 database entries every time. Because VLAD descriptors can confuse visually similar tiles (rice paddies, rooftops, road intersections), the top-1 match occasionally jumps hundreds of metres to the wrong tile on the other side of the scene. Once that happens, the VO accumulation starts from the wrong anchor and the error compounds.

The fix: after the first anchor is established, restrict the FAISS / similarity search to only the database entries that are geographically plausible given how far the drone could have moved since the last anchor.

---

### Implementation

**`anyloc/localizer.py` — `AnyLocLocalizer.localize()`**

New optional parameters:
```
center_lat  float  — latitude of the search centre (VO-refined estimate)
center_lon  float  — longitude of the search centre
radius_m    float  — search radius in metres (default unused = full search)
```

When all three are provided the method skips the FAISS index and does:

```python
# 1. Flat-Earth distance from every DB entry to the search centre
dlat     = (self.lats - center_lat) * 111_320.0          # metres north
dlon     = (self.lons - center_lon) * 111_320.0 * COS_LAT  # metres east
in_range = ((dlat**2 + dlon**2) <= radius_m**2)           # boolean mask
           .nonzero(as_tuple=False).squeeze(1)             # index tensor

# 2. Cosine similarity on the subset (both desc and vlads are L2-normalised)
sims  = self.vlads[in_range] @ desc   # (M,) — inner product = cosine sim
best  = int(sims.argmax())
idx   = int(in_range[best])           # index back into full DB
score = float(sims[best])
```

The flat-Earth approximation (`111,320 m per degree lat`, scaled by `cos(lat)` for lon) introduces < 0.1 % error over the 2 km scene radius — negligible.

All operations are pure torch tensors, keeping the numpy-safety rules of the `isaac_sim_test` env (no `np.array`, no numpy reductions). The subset is typically ~50 entries at 200 m radius, down from 2,821 — making this path faster than FAISS even without the index.

If `in_range` is empty (VO drifted badly or first frame), the code falls back to the full FAISS IndexFlatIP search automatically.

**`anyloc/run_localizer.py` — main loop**

On every AnyLoc frame (every 10th frame after the first), the VO-accumulated offset is added to the last anchor to form the search centre:

```python
clat = (anchor_lat + accum_dlat) if anchor_lat is not None else None
clon = (anchor_lon + accum_dlon) if anchor_lat is not None else None
loc.localize(frame, agl_m=drone_agl,
             center_lat=clat, center_lon=clon, radius_m=200.0)
```

- Frame 1: `anchor_lat is None` → `clat = None` → full FAISS search (2,821 entries)
- Frame 10+: `clat` = VO estimate → constrained torch search (~50 entries within 200 m)

---

### Why 200 m radius

| Factor | Value |
|--------|-------|
| DB grid spacing | 50 m |
| Grid steps covered by 200 m radius | 4 in each direction |
| Entries inside 200 m circle (approx.) | π × (200/50)² ≈ 50 |
| Max drone speed (sim) | ~20 m/s |
| Time between AnyLoc runs (10 f @ ~5 fps) | ~2 s |
| Max real displacement between runs | ~40 m |
| VO error on 40 m displacement | < 10 m typical |
| Safety margin (200 m vs 50 m max displacement) | ~4× |

200 m is the smallest radius that is robustly larger than any plausible true displacement + VO error, while still covering only ~2 % of the full database (50 / 2,821).

Going smaller (e.g. 100 m) risks clipping the true position when the drone moves fast or VO drifts. Going larger (e.g. 500 m) reduces the benefit — more wrong tiles enter the candidate set.

---

### Effect on accuracy

Without the constraint, a single wrong anchor propagates until the next large-error AnyLoc run corrects it — but that run is also unconstrained and can jump again. The constrained search makes each AnyLoc run self-correcting: even if the previous anchor was slightly off, the new search centre (anchor + VO) is close enough to the true position that the correct tile is almost always in the 200 m window.

---

## 2026-05-22 — YOLO vehicle detection module (Milestone 5)

### What was done

Created `detection/` with a working YOLOv8 vehicle detection pipeline and live postview.

**Files created:**
- `detection/detector.py` — `YOLODetector` class
- `detection/run_detector.py` — mtime-polling postview loop

**`YOLODetector` (`detector.py`):**
- Loads `yolov8n.pt` (ultralytics YOLOv8 nano, COCO pretrained, ~6 MB, auto-downloaded on first run)
- `detect(pil_img)` — runs inference, filters to COCO vehicle class IDs `{2: car, 3: motorcycle, 5: bus, 7: truck}`, returns list of `{label, conf, x1, y1, x2, y2}` dicts; coordinates extracted via `box.xyxy[0].tolist()` (torch-level, avoids numpy dispatch)
- `draw(pil_img, detections)` — PIL `ImageDraw` bounding boxes + filled label chips per class colour; returns new PIL RGB image; numpy-safe

**`run_detector.py`:**
- Same mtime-polling pattern as `run_localizer.py` (polls `drone_frames/latest.jpg` every 50 ms)
- Single matplotlib TkAgg window; `fig.canvas.draw()` + `flush_events()` for synchronous render
- Window title: vehicle count + inference time + drone lat/lon; green title when detections present
- Terminal: one `[YOLO]` line per detected vehicle with label, confidence, bounding box

**Dependency installed:**
- `ultralytics 8.4.52` installed via `python -m pip install ultralytics` in `isaac_sim_test`

**Known limitation:**
YOLOv8n was trained on eye-level COCO images. Nadir (top-down) vehicle views differ substantially in appearance and aspect ratio — detection confidence is lower from directly above. Fine-tuning on aerial imagery (DOTA, VisDrone) is needed for production accuracy.

---

---

## 2026-05-23 — Top-down YOLO fine-tuning pipeline

### What was done

Built a complete fine-tuning pipeline for adapting YOLOv8 to nadir (top-down) aerial vehicle detection. The existing `yolov8n.pt` was trained on eye-level COCO photos; this session adds the infrastructure to train on aerial imagery.

**Files created:**

- `detection/label_writer.py` — pure-Python nadir camera projection; given drone ENU position + vehicle position / yaw / class, projects the 4 footprint corners through the camera (fx=fy=320, 640×480) and returns a normalised YOLO bounding box. No numpy — safe inside `isaac_sim_test` env.

- `detection/collect_training_data.py` — Isaac Sim headless synthetic data collector. Builds a flat scene with 43 coloured vehicle boxes (25 cars, 8 motos, 4 buses, 6 trucks at random positions / yaws). Flies a grid at 30 m / 60 m / 100 m AGL with 35 % lateral overlap. At each of ~70 grid positions, captures a frame and writes a YOLO label via `label_writer`. Uses `Image.frombytes("RGBA", ...)` instead of `.astype()` to safely convert the replicator buffer inside the broken-numpy env.

- `detection/prepare_dataset.py` — downloads VisDrone 2019 DET via `ultralytics.data.utils.check_det_dataset("VisDrone.yaml")`; remaps 7 VisDrone classes to 4 targets (`car/motorcycle/bus/truck`); symlinks images and writes YOLO `.txt` labels into `detection/dataset/{images,labels}/{train,val}/`; merges any synthetic data from `detection/dataset/synth/`; writes `data.yaml`.

  VisDrone → canonical map: car(4)→car, van(5)→car, truck(6)→truck, tricycle(7)→moto, awning-tricycle(8)→moto, bus(9)→bus, motor(10)→moto.

- `detection/finetune.py` — loads `yolov8n.pt`, trains 100 epochs with augmentations tuned for nadir aerial: `degrees=45`, `flipud=0.5`, `scale=0.5` (altitude variation), `mosaic=1.0` (small objects), `hsv_v=0.4` (lighting variation). Saves to `detection/runs/topdown_v1/weights/best.pt`.

---

## 2026-05-24 — Switched to yolov8l_visdrone.pt; auto class-map in detector

### What was done

Switched the active detection model from `yolov8n.pt` (COCO) to `yolov8l_visdrone.pt` (YOLOv8-large, pre-trained on VisDrone 2019 DET). This immediately improves aerial vehicle detection without any training.

**`detection/detector.py` — refactored class mapping:**

Replaced the hardcoded COCO class ID dict `{2: 'car', 3: 'motorcycle', ...}` with a name-based lookup built at load time:

```python
_NAME_TO_LABEL = {
    'car': 'car', 'van': 'car',
    'truck': 'truck',
    'bus': 'bus',
    'motorcycle': 'motorcycle', 'motor': 'motorcycle',
    'tricycle': 'motorcycle', 'awning-tricycle': 'motorcycle',
}

self._filter = {
    cid: _NAME_TO_LABEL[name]
    for cid, name in self.model.names.items()
    if name in _NAME_TO_LABEL
}
```

`self._filter` is built from `model.names` so the same `YOLODetector` class works for both COCO and VisDrone models — no code change needed when swapping models.

VisDrone model class map: `{3: car, 4: car, 5: truck, 6: motorcycle, 7: motorcycle, 8: bus, 9: motorcycle}` — 7 aerial vehicle classes covered.

**`detection/run_detector.py`:**
- Added `MODEL_PT = os.path.join(ROOT, 'yolov8l_visdrone.pt')`
- Changed `YOLODetector('yolov8n.pt', conf=0.35)` → `YOLODetector(MODEL_PT, conf=0.30)` (lower threshold appropriate for a model already trained on aerial imagery)

---

## 2026-05-27 — Architecture decisions: ArduPilot SITL + MAVLink + IMU

### Decisions made

**1. ArduPilot SITL + MAVLink before IMU implementation**

On real hardware, IMU data arrives via MAVLink `HIGHRES_IMU` messages from the flight controller. Building the IMU reader against MAVLink now means zero interface changes at deployment. ArduPilot SITL's sensor pipeline also provides realistic noise, bias drift, and temperature effects that analytical position derivatives cannot replicate.

Build order:
1. `control/sitl_bridge.py` — Isaac Sim → ArduPilot SITL JSON/UDP physics state bridge
2. `control/mavlink_ctrl.py` — pymavlink subscriber + `SET_POSITION_TARGET_LOCAL_NED` sender
3. `control/imu_reader.py` — reads `HIGHRES_IMU` from MAVLink stream
4. `control/imu_fusion.py` — uses IMU to validate AnyLoc anchors + gate VO quality

**2. Physics-based IMU via ArduPilot SITL JSON backend (not analytical derivatives)**

ArduPilot SITL receives the drone's physics state from Isaac Sim each step (position, velocity, acceleration, attitude in NED), runs its own sensor models, and outputs `HIGHRES_IMU` over MAVLink — the same message format a real ArduPilot FC sends.

**3. IMU role in localization: sanity check on AnyLoc anchors**

Context: the geo-constrained AnyLoc search (200 m window) prevents most bad jumps, but if the constraint window itself drifts (wrong anchor accepted), the system cannot self-correct. IMU dead-reckoning provides an independent position estimate to validate new anchors:

- If new AnyLoc anchor deviates > `jump_threshold` from IMU-predicted position → reject anchor
- If IMU detects high angular velocity / acceleration spike → skip VO accumulation for that frame
- If both AnyLoc and VO fail → use IMU dead-reckoning for short bridging intervals

### Architecture

```
Isaac Sim physics state (JSON/UDP, each step)
    ↓
ArduPilot SITL (JSON backend)
    ↓ MAVLink UDP:14550
    ├─ HIGHRES_IMU → imu_reader.py → imu_fusion.py (anchor validator + VO gate)
    ├─ ATTITUDE, LOCAL_POSITION_NED → state estimation
    └─ accepts SET_POSITION_TARGET_LOCAL_NED (replaces keyboard control)
```

---

## 2026-05-27 — Milestone 6a: ArduPilot SITL JSON bridge

### What was done

Created `control/sitl_bridge.py` and wired it into `simulator/cesium_scene.py`.

**Files created:**
- `control/__init__.py` — makes `control/` a Python package
- `control/sitl_bridge.py` — `SITLBridge` class

**`SITLBridge` class:**
- Sends drone physics state to ArduPilot SITL JSON backend via UDP (port 9002) every sim step
- Receives servo/motor outputs from SITL on port 9003 (`recv_servos()`) — used in milestone 6b
- Takes Isaac Sim ENU state `(x_enu, y_enu, z_abs, yaw_deg)` each step and converts to ArduPilot NED JSON

**Coordinate conversions:**
- ENU → NED: `north = y_enu`, `east = x_enu`, `down = -(z_abs - centre_elev)`
- Yaw: Isaac Sim RotateZ CCW-positive → ArduPilot NED CW-positive: `yaw_rad = -radians(yaw_deg)`

**Computed quantities (no physics engine — finite difference):**
- Velocity NED: `Δpos / Δt`, clamped to ±30 m/s (prevents spikes when keyboard moves 5 m/step)
- Acceleration NED: `Δvel / Δt`, low-pass filtered (α=0.3 EMA) to smooth keyboard jump artifacts
- IMU specific force (body frame): `accel_ned - (0, 0, +g)` rotated by yaw into body frame
  - At hover: `[0, 0, -9.81]` ✓
- Yaw rate: `Δyaw / Δt` with wrap-to-`[-π, π]`
- Barometric pressure: ISA approximation `101325 × exp(-alt_msl / 8500)`

**`simulator/cesium_scene.py` changes (3 edits):**
1. Added `sys` to imports; added `sys.path.insert` so `control/` is importable from `simulator/`
2. After terrain load (when `centre_elev` is known): `_sitl = SITLBridge(centre_elev=centre_elev)`; wrapped in `try/except ImportError` so sim still runs without the bridge
3. In simulation loop after HUD update: `_sitl.step(x, y, alt, yaw, time.time())` called every step (not gated to DRONE_SAVE_EVERY)

**Known limitations:**
- The drone is a scripted Xform — position jumps 5 m per key press. Velocity/acceleration clamp and EMA filter prevent SITL from seeing implausible IMU values, but the motion is not physically realistic. Milestone 6b-iv replaces keyboard control with `SET_POSITION_TARGET_LOCAL_NED` commands from ArduPilot.
- The JSON bridge currently sends `position_xyz` (ground-truth position from Isaac Sim), which ArduPilot EKF3 treats as a GPS substitute. This is **not** the no-GPS pipeline. Milestone 6b-ii removes `position_xyz` from the bridge and milestone 6b-iii replaces it with AnyLoc estimates sent via `VISION_POSITION_ESTIMATE` MAVLink messages.

**Run order:**
```bash
# Terminal 1 — start ArduPilot SITL first
python3 third_party/ardupilot/Tools/autotest/sim_vehicle.py \
    -v ArduCopter --model=JSON --console --map

# Terminal 2 — start Isaac Sim (bridge auto-connects on first step)
cd simulator && ./run_chiayi.sh
```

---

## 2026-05-28 — ArduPilot SITL build, protocol fix, milestone restructure

### ArduPilot SITL build

`--depth=1` clone does not pull submodules. Required submodules and fixes:

```bash
# All submodules at once (avoids discovering them one by one as build fails)
git submodule update --init --depth=1 --recursive   # ~5 min

# Configure and build ArduCopter SITL binary
python3 waf configure --board sitl
python3 waf copter   # ~2 min, binary → build/sitl/bin/arducopter
```

System Python3 dependencies installed for SITL tooling:
```bash
pip3 install --user --break-system-packages pexpect mavproxy pymavlink future
```

### Bug fixed: sitl_bridge.py protocol was backwards

**Original (wrong):** bridge was a UDP client that pushed physics state to port 9002 (as if ArduPilot was listening there).

**Correct:** ArduPilot is the JSON **client** — it sends `{"pwm": [...], "frame_time_us": N}` to the simulator and waits for physics state back. The bridge must be a UDP **server** listening on port 9002, receiving servo packets, and replying with physics state.

Fix: rewrote `SITLBridge` from a push client to a request-response server:
- `self._sock.bind(("0.0.0.0", 9002))` — server binds
- Each `step()`: drains incoming servo packets (non-blocking), learns `_ap_addr` from first packet, replies to that address with current physics state
- `step()` returns the latest servo dict for use in milestone 6b-iv

The message "No JSON sensor message received, resending servos" is ArduPilot's normal retry output while waiting for the simulator — it stops once Isaac Sim is running and the bridge replies.

### Architecture clarification: position_xyz is GPS, not no-GPS

`position_xyz` and `velocity_xyz` in the JSON bridge packet act as a GPS substitute in ArduPilot's EKF3. Sending them defeats the no-GPS goal. They are intentionally omitted from the bridge.

The no-GPS position source is `VISION_POSITION_ESTIMATE` MAVLink messages, sent from `mavlink_ctrl.py` using AnyLoc position estimates. ArduPilot EKF3 fuses this as an external vision source — same mechanism as Intel RealSense T265 or OptiTrack on real hardware.

### Milestone 6b restructured into 4 ordered sub-steps

| Sub-step | What |
|---|---|
| 6b-i | pymavlink connection to ArduPilot MAVLink output (UDP:14550) |
| 6b-ii | Disable GPS (`GPS_TYPE=0`); bridge sends IMU+baro only |
| 6b-iii | `VISION_POSITION_ESTIMATE` from AnyLoc → ArduPilot EKF3 |
| 6b-iv | `SET_POSITION_TARGET_LOCAL_NED` flight commands (replaces keyboard) |

6b-iii must precede 6b-iv: ArduPilot refuses position commands until EKF3 has a valid position fix.

---

## 2026-05-28 — Milestone 6b-i: pymavlink connection (control/mavlink_ctrl.py)

### Files created

**`control/mavlink_ctrl.py`** — `MAVLinkCtrl` class:
- `__init__(connection_str="tcp:localhost:5762")` — connects directly to ArduPilot SITL
  TCP port 5762 (no mavproxy needed; UDP:14550 was found to not deliver packets reliably)
- `wait_heartbeat(timeout=60)` — blocking; learns `target_system` / `target_component`
  from first HEARTBEAT, then requests data streams
- `recv()` — non-blocking drain; updates `_imu`, `_attitude`, `_local_pos`, `_ekf`,
  `_heartbeat` from incoming MAVLink messages; returns list of type strings received
- `_request_streams()` — asks ArduPilot for all data streams at 10 Hz via
  `REQUEST_DATA_STREAM_ALL`; requests `HIGHRES_IMU` separately at 50 Hz via
  `MAV_CMD_SET_MESSAGE_INTERVAL`
- Properties: `connected`, `imu`, `attitude`, `local_pos`, `ekf_flags`, `ekf_pos_valid`
- Stubs for 6b-iii: `send_vision_position(north, east, down, yaw_rad, covariance)`
  — sends `VISION_POSITION_ESTIMATE`; default covariance 5 m position std, 0.2 rad
  orientation std (needs tuning once AnyLoc error is characterised)
- Stubs for 6b-iv: `arm()`, `takeoff(alt_m)`, `set_position_ned(north, east, down, yaw_rad)`
  — `set_position_ned` uses `SET_POSITION_TARGET_LOCAL_NED` with type_mask
  `0b111111111000` (position only) or `0b110111111000` (position + yaw)

**`control/run_mavlink.py`** — terminal monitor:
- Connects, waits for HEARTBEAT, then prints rolling single-line display at 10 Hz:
  roll/pitch/yaw (degrees), NED position (metres), IMU accelerations (m/s²), EKF flags
- EKF flags decoded to named labels: ATT, VEL, POS_REL, POS_ABS, PRED_ABS

### EKF_STATUS_REPORT flag constants (exported from mavlink_ctrl.py)

| Constant | Bit | Hex | Meaning |
|---|---|---|---|
| `EKF_ATTITUDE` | 0 | 0x0001 | Attitude valid |
| `EKF_VEL_HORIZ` | 1 | 0x0002 | Horizontal velocity valid |
| `EKF_POS_HORIZ_REL` | 3 | 0x0008 | Relative horizontal position valid |
| `EKF_POS_HORIZ_ABS` | 4 | 0x0010 | Absolute horizontal position valid (GPS or vision fused) |
| `EKF_PRED_POS_HORIZ_ABS` | 9 | 0x0200 | Vision position estimate accepted by EKF3 |
| `EKF_UNINITIALIZED` | 10 | 0x0400 | EKF has not finished initialising (normal at startup) |

`EKF_POS_HORIZ_ABS` going high is the signal that `VISION_POSITION_ESTIMATE` is being
fused — needed before 6b-iv flight commands will be accepted.

### Notes
- `"position"` and `"velocity"` are sent in the JSON bridge for now (GPS substitute using correct SIM_JSON key names).
  They are removed in milestone 6b-ii after `VISION_POSITION_ESTIMATE` is working.
- The bridge's `step()` returns the latest parsed servo dict; 6b-iv reads PWM from there.

---

## 2026-05-28 — Three SITL bridge bugs fixed; EKF_UNINITIALIZED added

### Root cause of "No JSON sensor message received, resending servos"

Three compounding bugs in `control/sitl_bridge.py` prevented ArduPilot from ever receiving physics replies:

**Bug 1 — Binary servo packets were being parsed as JSON (root cause of _ap_addr never set)**

ArduPilot's `SIM_JSON::output_servos()` sends a C struct `servo_packet_16` (40 bytes, little-endian):
```c
struct servo_packet_16 { uint16_t magic=18458; uint16_t frame_rate; uint32_t frame_count; uint16_t pwm[16]; };
```
The bridge called `json.loads(data.decode('utf-8'))` on this binary data — always failing.
With the previous session's `_ap_addr` fix (only set after valid JSON parse), `_ap_addr` was never learned, so no physics replies were ever sent.

Fix: added `_parse_servo_packet()` which uses `struct.unpack("<HHI16H", data)` to parse the binary packet and validates the magic number (18458 for 16-channel, 29569 for 32-channel) before setting `_ap_addr`.

**Bug 2 — Missing `\n` terminator on physics JSON**

ArduPilot's `recv_fdm()` (in `SIM_JSON.cpp`) processes messages by replacing `\n` with `\0` as a delimiter, then uses `memrchr(..., 0, ...)` to locate the last complete message. Without a trailing `\n`, `memrchr` returns `nullptr` and the function returns early without parsing — every physics packet silently discarded.

Fix: append `"\n"` to every physics JSON packet before sending.

**Bug 3 — Wrong JSON key names**

The bridge sent keys like `"imu_angular_velocity_rpy"`, `"velocity_xyz"`, `"attitude_rpy"` which don't exist in ArduPilot's `SIM_JSON` keytable. Required keys are:
- `"timestamp"` (root, required)
- `"imu": {"gyro": [...], "accel_body": [...]}` (section required)
- `"velocity": [vn, ve, vd]` (root, required)
- `"attitude": [roll, pitch, yaw]` (root, required for either attitude or quaternion)

Fix: rewrote `_build_state()` return dict to use the exact key names from `SIM_JSON.h`.

### Files modified

- `control/sitl_bridge.py` — all three fixes; removed `import json` fallback path for servos; added `struct` import and binary constants; physics send now appends `\n`
- `control/mavlink_ctrl.py` — added `EKF_UNINITIALIZED = 1 << 10`
- `control/run_mavlink.py` — `_ekf_label()` now returns `"UNINIT"` for bit 10 instead of `"none"`

---

## 2026-05-28 — Milestone 6b-ii: disable GPS, strip position from bridge

### What was done

Removed `"position"` and `"velocity"` from the JSON physics packet and added a SITL parameter file to disable the GPS sensor.

**`control/sitl_bridge.py`** — `_build_state()` no longer includes `"position"` or `"velocity"` in the returned dict. `vel_ned` and `accel_ned` are still computed internally because `accel_body` (the IMU specific force) is derived from them; they just aren't sent to ArduPilot.

**`control/no_gps.parm`** — ArduPilot SITL parameter file:
```
GPS_TYPE 0    # disable GPS sensor
```
Loaded at SITL startup with `--add-param-file=control/no_gps.parm`. Parameters persist in SITL's `eeprom.bin` after first load.

### Effect on EKF

Without `"position"` and `"velocity"`, ArduPilot EKF3 receives:
- IMU (`imu.gyro`, `imu.accel_body`) — attitude + short-term dead reckoning
- Attitude (`"attitude"`) — direct yaw/roll/pitch reference
- Rangefinder (`"rng_1"`) — altitude AGL
- Barometer — simulated internally from last-known Aircraft altitude (static after position is dropped)
- Compass — synthesised from attitude + Earth field model (approx. correct for small area)

Expected EKF state: `ATT` (attitude valid) without `VEL_HORIZ` or `POS_ABS`. Horizontal position will drift — that is the correct no-GPS baseline before 6b-iii adds `VISION_POSITION_ESTIMATE`.

### Next step

6b-iii: send AnyLoc position estimates to ArduPilot EKF3 via `VISION_POSITION_ESTIMATE` MAVLink messages. This requires setting `EK3_SRC1_POSXY=6` (ExtNav) and `VISO_TYPE=1` in `no_gps.parm`.

---

## 2026-05-28 — 6b-ii velocity fix; multi-client TCP; EKF UNINIT root cause; 6b-iii wired up

### Bug fixed: `"velocity"` incorrectly removed in 6b-ii

Milestone 6b-ii had removed `"velocity"` from the bridge JSON alongside `"position"`. This caused ArduPilot to print "Failed to find key /velocity" and revert to "resending servos".

Root cause: `"velocity"` is `required=true` in ArduPilot's `SIM_JSON.h` keytable — omitting it causes `received_bitmask==0` and the entire packet is rejected. `"position"` is `required=false` and IS a GPS substitute; `"velocity"` is not (with `GPS_TYPE=0` it feeds only SITL's internal physics model and is never fused by EKF3 via GPS).

Fix: added `"velocity": list(vel_ned)` back to `_build_state()` with an explanatory comment.

---

### Bug fixed: only one MAVLink client could connect at a time

Both `run_mavlink.py` and `run_vision.py` connected to `tcp:localhost:5762`. TCP port 5762 accepts one client at a time — the second connection hung waiting for HEARTBEAT forever.

Fix: `run_vision.py` now connects to `tcp:localhost:5763`. SITL must be started with `--out tcp:localhost:5763` to open that port:

```bash
python3 third_party/ardupilot/Tools/autotest/sim_vehicle.py \
    -v ArduCopter --model=JSON --no-rebuild --console --map \
    -l 23.450868,120.286135,46,0 \
    --add-param-file=control/no_gps.parm \
    --out tcp:localhost:5763
```

| Script | Port | Role |
|--------|------|------|
| `run_mavlink.py` | `tcp:localhost:5762` | monitor |
| `run_vision.py` | `tcp:localhost:5763` | vision sender |

---

### Bug fixed: EKF reset to UNINIT every ~20 seconds

After reaching `ATT,VEL`, EKF flags would drop back to UNINIT (0x0400) approximately every 20 seconds.

Root cause: `EK3_SRC1_POSXY` defaults to **3 (GPS)**. With `GPS_TYPE=0`, EKF3 expects GPS position fusion but never receives any. After its internal timeout (~20 s) it declares the lane unhealthy and resets.

Fix: uncomment EK3 ExtNav params in `control/no_gps.parm`:
```
EK3_SRC1_POSXY  6   # horizontal position from ExternalNav
EK3_SRC1_VELXY  0   # no horizontal velocity source
EK3_SRC1_POSZ   1   # altitude from barometer
EK3_SRC1_YAW    1   # yaw from compass
VISO_TYPE       1   # enable MAVLink ExternalNav processing
```

With `EK3_SRC1_POSXY=6`, EKF3 expects ExtNav position (from `VISION_POSITION_ESTIMATE`) instead of GPS. Always restart SITL with `--wipe` after changing `no_gps.parm` to flush old `eeprom.bin`.

---

### `run_mavlink.py` display improvements

- **TIME column**: was `time.time() % 10000` (wall clock, never resets). Changed to `time.time() - t0` (seconds since monitor started, resets each run).
- **EKF label**: added `VEL_V` (bit 2) and `ALT` (bit 5 = `EKF_POS_VERT_ABS`) to `_ekf_label()`.

---

### Milestone 6b-iii: run_vision.py deployed, EK3 params set

`control/run_vision.py` is now wired up and `no_gps.parm` has the full ExtNav params. The pipeline is ready to test end-to-end:

1. SITL with `--out tcp:localhost:5763 --add-param-file=control/no_gps.parm --wipe`
2. Isaac Sim (`run_chiayi.sh`) or `stub_bridge.py`
3. `run_localizer.py` (writes `anyloc/latest_estimate.json`)
4. `run_vision.py` (reads estimate, sends `VISION_POSITION_ESTIMATE` at 5 Hz on port 5763)
5. `run_mavlink.py` (monitors EKF flags on port 5762)

Watch for `POS_ABS` (0x0010) in EKF flags to confirm EKF3 is fusing the vision position.

**Confirmed:** EKF flags reached `ATT,VEL_H,VEL_V,POS_REL,POS_ABS,ALT,PRED_ABS` — all flags healthy, vision position fully fused. Milestone 6b-iii done.

---

## 2026-05-29 — 6b-iv bug fixes: GPS failsafe, physics accuracy, stale estimate, debug tooling

### Bugs fixed

**1. GPS failsafe silently switches GUIDED→LAND after arming**

Root cause: `FS_GPS_ENABLE` is enabled by default. After force-arming with GPS bad fix, the failsafe fires within seconds and changes GUIDED → LAND. The TAKEOFF command arrives in LAND mode and is ignored — drone stays on the ground. Motors output landing throttle (~30 %), below hover threshold (~50 % mean PWM), so the kinematic model produces no upward thrust.

Fix: added to `control/no_gps.parm`:
```
FS_GPS_ENABLE   0   # prevent GPS failsafe GUIDED→LAND switch after arming
FENCE_ENABLE    0   # prevent geofence blocking flight near origin
```

**2. ARM rejected with FAILED (result=4) but force-arm not triggered**

Original code only triggered force-arm when `wait_command_ack` returned `None` (timeout). A `FAILED` result (4) returned immediately and bypassed force-arm entirely.

Fix: changed condition from `if result is None` to `if result != 0` — triggers force-arm on any non-zero MAV_RESULT (TEMPORARILY_REJECTED, DENIED, UNSUPPORTED, FAILED).

**3. EKF initialises at wrong position — stale estimate file**

Cause: `anyloc/latest_estimate.json` left over from a previous AnyLoc run. Old check was `if not os.path.exists(...)` — a 30-minute-old file would init EKF at (350 m N, 1352 m E) from home.

Fix: added age check — if file older than 10 seconds, overwrite with stub at home position.

**4. VisOdom not healthy at arm time**

`EKF_POS_ABS` fires on the very first VPE message, but `AP_VisualOdom::healthy()` requires a continuous 1-second window of VPE messages. Without waiting, the VisOdom pre-arm health check could still block arming.

Fix: added 3-second settle wait after EKF_POS_ABS — 3 s @ 5 Hz = 15 VPEs, well above the 1-second health window.

**5. HIGHRES_IMU "rate too fast" warning**

Requested 50 Hz equals `SCHED_LOOP_RATE` limit. ArduPilot logged a warning and may silently cap it.

Fix: reduced to 25 Hz (40 000 µs interval) in `mavlink_ctrl.py`.

**6. Gyro missing roll/pitch rates**

`sitl_bridge.py` sent `[0, 0, yaw_rate]` as the gyro vector. When the drone tilted, the EKF saw attitude changing (from the `attitude` field) but gyro showed no rotation — innovation mismatch, degraded EKF attitude tracking.

Fix: added `_prev_roll_rad` and `_prev_pitch_rad` state; compute p and q from finite difference alongside r. Gyro now sends `[roll_rate, pitch_rate, yaw_rate]`.

**7. Accel body frame — yaw-only rotation**

IMU specific force was rotated from NED to body using yaw only. At 20° tilt this introduced ≈12 % horizontal force error, causing wrong heading dynamics during autonomous flight.

Fix: full 3-axis DCM: R_bn = (R_z(yaw)·R_y(pitch)·R_x(roll))ᵀ

### Feature added: SITLBridge.debug_hz

New `debug_hz` property prints the physics state being sent at the specified rate. `stub_bridge.py` sets `bridge.debug_hz = 1.0` by default for cross-checking.

Sample output (stationary on ground):
```
[SITL] t=   3.12s  gyro p=+0.000 q=+0.000 r=+0.000 rad/s  accel bx=+0.00 by=+0.00 bz=-9.81 m/s²  vel N=+0.00 E=+0.00 D=+0.00 m/s  att r=+0.0° p=+0.0°  rng=0.10m
```

Cross-check: `accel bz ≈ −9.81` on ground confirms correct specific force sign convention. Compare `accel bz` against `Az` column in `run_mavlink.py` — should match within 0.05 m/s².

---

## 2026-05-29 — Milestone 6b-iv: flight command pipeline implemented

### mavlink_ctrl.py — new methods

| Method | Purpose |
|--------|---------|
| `set_mode(mode_name)` | Set ArduPilot flight mode by name ('GUIDED', 'RTL', 'LAND', …) |
| `wait_ekf_pos(timeout)` | Block until EKF_POS_HORIZ_ABS is set |
| `wait_command_ack(cmd_id, timeout)` | Block until COMMAND_ACK for cmd_id; returns MAV_RESULT |
| `wait_altitude(target_agl, tolerance, timeout)` | Block until LOCAL_POSITION_NED.z ≈ -target_agl |
| `wait_position(n, e, d, radius, timeout)` | Block until drone is within radius m of NED target |
| `is_armed` | True when HEARTBEAT base_mode has MAV_MODE_FLAG_SAFETY_ARMED |

COMMAND_ACK messages are now tracked in `recv()` via `self._last_ack[cmd_id] = result`.
Armed status is updated from every HEARTBEAT.

### stub_bridge.py — kinematic altitude model

Replaced static hover with a kinematic simulation:
- Drone starts on the ground (AGL = 0, z_abs = HOME_ELEV)
- Each step: `mean_pwm` of 4 motors → `thrust_norm` (0–1) → `thrust_accel` (0–2g)
- Net vertical acceleration: `GRAVITY - thrust_accel` (NED down)
- Integrates vertical velocity and altitude at 100 Hz
- Ground constraint: z_abs ≥ HOME_ELEV, vd clamped to ≤ 0 on contact

This lets ArduPilot arm and take off in SITL without Isaac Sim. Horizontal position stays at origin — full horizontal kinematics require Isaac Sim.

### run_flight.py — merged vision + flight

`run_vision.py` functionality merged into `run_flight.py` as a background thread:
- Vision thread: polls `anyloc/latest_estimate.json`, sends `VISION_POSITION_ESTIMATE` at 5 Hz
- Main thread: wait POS_ABS → GUIDED → arm → takeoff → waypoints → RTL → wait disarm
- Both share one `MAVLinkCtrl` on `tcp:localhost:5762` — no second TCP port needed
- If `latest_estimate.json` doesn't exist, a stub estimate at home is written automatically

`run_vision.py` kept as standalone alternative for vision-only testing.

SITL command simplified — `--out tcp:localhost:5763` no longer needed:
```bash
python3 third_party/ardupilot/Tools/autotest/sim_vehicle.py \
    -v ArduCopter --model=JSON --no-rebuild --console --map \
    -l 23.450868,120.286135,28.17,0 \
    --add-param-file=control/no_gps.parm --wipe
```

---

## 2026-05-30 — EKF origin fix; VisOdom health; confirmed first autonomous flight

### Bugs fixed

**1. `SET_GPS_GLOBAL_ORIGIN` never sent — root cause of all arming failures**

- Cause: `run_flight.py` connected and immediately started sending `VISION_POSITION_ESTIMATE`, but ArduPilot's EKF3 had no NED reference frame. Without a known origin, VPE messages cannot be anchored to absolute coordinates — EKF3 discards them, reports "EKF attitude is bad" and "VisOdom: not healthy", and blocks arming.
- Fix: added `set_ekf_origin()` and `set_home_position()` to `MAVLinkCtrl`; both are called right after `wait_heartbeat()` in `run_flight.py` and `run_vision.py`, before the vision thread starts.
- Confirmed: SITL console shows `EKF3 IMU0 origin set`, `EKF3 IMU1 origin set`, `Field Elevation Set: 28m` immediately after connection.

```python
# mavlink_ctrl.py — new methods
def set_ekf_origin(lat, lon, alt_msl_m)   # sends SET_GPS_GLOBAL_ORIGIN
def set_home_position(lat, lon, alt_msl_m) # sends SET_HOME_POSITION
```

**2. Regular arm FAILED even with `ARMING_CHECK 0` — VisOdom mandatory check**

- Cause: In ArduPilot 4.x+, the VisOdom health pre-arm check is mandatory when `EK3_SRC1_POSXY=6`. `ARMING_CHECK 0` does not bypass it. `AP_VisualOdom::healthy()` requires a continuous 1-second window of VPE messages — the previous fixed 3-second sleep was not tight enough to guarantee this.
- Fix: replaced the fixed sleep with `wait_visodom_healthy()` which polls `EKF_PRED_POS_HORIZ_ABS` (bit 9). This flag is set only when EKF3 is predicting future position from VPE, which implies `AP_VisualOdom::healthy()` is satisfied. Regular arm now succeeds without needing force arm.

```python
# mavlink_ctrl.py — new method
def wait_visodom_healthy(timeout=30.0)  # waits for EKF_POS_ABS | EKF_PRED_POS_ABS
```

### Confirmed flight output

```
AP: EKF3 IMU0 origin set
AP: EKF3 IMU1 origin set
AP: Field Elevation Set: 28m
AP: EKF3 IMU0 is using external nav data
AP: EKF3 IMU0 initial pos NED = 350.4,1351.6,0.0 (m)   ← stale AnyLoc estimate
ARMED
AP: EKF3 IMU0 MAG0 in-flight yaw alignment complete

[Flight] EKF POS_ABS ✓
[Flight] VisOdom healthy ✓
[Flight] Armed ✓
[Flight] Takeoff → 10.0 m AGL …
[Flight] Reached 10.0 m AGL ✓
[Flight] WP 1/4  N=+20 E=+0 ALT=10 m AGL
```

**Note on initial NED offset (350.4, 1351.6):** The first VPE sent was a stale `anyloc/latest_estimate.json` from a previous AnyLoc run (position was not at home). Delete or overwrite this file before each test to ensure EKF initialises at NED (0, 0, 0).

### Files modified

| File | Change |
|------|--------|
| `control/mavlink_ctrl.py` | Added `set_ekf_origin()`, `set_home_position()`, `wait_visodom_healthy()` |
| `control/run_flight.py` | Calls `set_ekf_origin` + `set_home_position` after heartbeat; replaces 3 s sleep with `wait_visodom_healthy()` |
| `control/run_vision.py` | Same origin/home calls added; stale `HOME_ALT_MSL=46.0` → `28.17` |
| `README.md`, `project_plan.md`, `history.md` | SITL `-l` altitude placeholder `<centre_elev>`/`46` → `28.17` throughout |

---

## 2026-05-30 — Milestone 6e: ROS2 migration (all IPC via topics + MAVROS2)

### Motivation

All previous inter-process communication was file-based (JPEG frames + JSON estimates) or raw sockets (pymavlink TCP). This introduced polling latency, file-write race conditions, and non-standard interfaces. ROS2 pub/sub eliminates polling, provides introspectability (`ros2 topic echo`), and matches the standard deployment interface for real hardware.

### Environment

- **ROS2 Jazzy** already installed at `/opt/ros/jazzy` (Ubuntu 24.04)
- **MAVROS2 2.14.0** already installed (`ros-jazzy-mavros`, `ros-jazzy-mavros-extras`)
- **vision_msgs 4.1.1** already installed (`ros-jazzy-vision-msgs`)
- **rclpy** uses Python 3.12 — same as Isaac Sim 6.0 — so system rclpy is used directly inside Isaac Sim by adding `/opt/ros/jazzy/lib/python3.12/site-packages` to `sys.path`

### New ROS2 topic map

| Topic | Type | Publisher | Subscriber(s) |
|-------|------|-----------|---------------|
| `/drone/camera/image_raw` | `sensor_msgs/Image` (rgb8) | Isaac Sim | AnyLoc node, YOLO node |
| `/drone/pose` | `geometry_msgs/PoseStamped` (frame=wgs84, pos=lat/lon/alt) | Isaac Sim | AnyLoc node, YOLO node |
| `/drone/agl` | `std_msgs/Float64` | Isaac Sim | AnyLoc node |
| `/anyloc/pose_estimate` | `geometry_msgs/PoseWithCovarianceStamped` | AnyLoc node | (mission planner) |
| `/mavros/vision_pose/pose` | `geometry_msgs/PoseStamped` (frame=map, NED) | AnyLoc node | MAVROS2 → `VISION_POSITION_ESTIMATE` |
| `/yolo/detections` | `vision_msgs/Detection2DArray` | YOLO node | (mission planner) |
| `/mavros/state` | `mavros_msgs/State` | MAVROS2 | Flight commander |
| `/mavros/local_position/pose` | `geometry_msgs/PoseStamped` | MAVROS2 | Flight commander |
| `/mavros/setpoint_position/local` | `geometry_msgs/PoseStamped` | Flight commander | MAVROS2 → `SET_POSITION_TARGET` |

### Files created

| File | Purpose |
|------|---------|
| `simulator/cesium_scene.py` (modified) | Publishes `/drone/camera/image_raw`, `/drone/pose`, `/drone/agl` via system rclpy; falls back to file output if ROS2 unavailable |
| `simulator/run_chiayi.sh` (modified) | Sources `/opt/ros/jazzy/setup.bash` before `conda run` so ROS2 shared libs are on `LD_LIBRARY_PATH` |
| `anyloc/ros2_node.py` | rclpy node: subscribes to camera + pose → runs AnyLoc+VO → publishes to `/anyloc/pose_estimate` and `/mavros/vision_pose/pose` |
| `detection/ros2_node.py` | rclpy node: subscribes to camera → runs YOLOv8 → publishes to `/yolo/detections` |
| `control/launch_mavros.sh` | Starts MAVROS2 connected to SITL `tcp:localhost:5762` |
| `control/flight_commander.py` | rclpy node: GUIDED → arm → takeoff → waypoints → RTL via MAVROS2 services/topics |

### Architecture decision: pymavlink for EKF origin only

MAVROS2 Jazzy 2.14 has no `/mavros/global_position/set_gp_origin` service. `flight_commander.py` uses a thin pymavlink call only for `SET_GPS_GLOBAL_ORIGIN` + `SET_HOME_POSITION` at startup, then hands off to MAVROS2 for everything else.

### Legacy files kept (non-ROS2 fallback)

`anyloc/run_localizer.py`, `detection/run_detector.py`, `control/run_flight.py`, `control/run_vision.py`, `control/run_mavlink.py`, `control/mavlink_ctrl.py` — all kept as file-based / pymavlink fallbacks. Remove when ROS2 pipeline is validated on hardware.

### Run order (ROS2 mode)

```bash
# Terminal 1 — SITL
python3 third_party/ardupilot/Tools/autotest/sim_vehicle.py \
    -v ArduCopter --model=JSON --no-rebuild --console --map \
    -l 23.450868,120.286135,28.17,0 --add-param-file=control/no_gps.parm

# Terminal 2 — physics bridge (or Isaac Sim)
python3 control/stub_bridge.py

# Terminal 3 — MAVROS2
bash control/launch_mavros.sh

# Terminal 4 — AnyLoc ROS2 node
source /opt/ros/jazzy/setup.bash && python3 anyloc/ros2_node.py

# Terminal 5 — YOLO ROS2 node (optional)
source /opt/ros/jazzy/setup.bash && python3 detection/ros2_node.py

# Terminal 6 — Isaac Sim (publishes camera + pose topics)
cd simulator && ./run_chiayi.sh

# Terminal 7 — Flight commander
source /opt/ros/jazzy/setup.bash && python3 control/flight_commander.py
```

---

## 2026-05-31 — Remove pymavlink; MAVROS2 raw MAVLink for EKF origin + status; two-phase VPE

### What was done

**Removed all pymavlink dependencies from `flight_commander.py`**

The old code used pymavlink on UDP 14551 for three things: setting the EKF global origin, monitoring EKF status flags, and reading altitude during takeoff. All three are replaced by MAVROS2 infrastructure:

- **EKF origin**: publish `GeoPointStamped` to `/mavros/global_position/set_gp_origin`. Confirmed by monitoring GPS_GLOBAL_ORIGIN (msg 49) on `/uas1/mavlink_source` with BEST_EFFORT QoS. No extra UDP port required — MAVROS2's global_position plugin forwards to ArduPilot.

- **EKF status**: read EKF_STATUS_REPORT (msg 193) from `/uas1/mavlink_source`. Flags decoded at **byte offset 20** (after 5 floats × 4 bytes). `/mavros/estimator_status` is advertised in MAVROS2 Jazzy 2.14 but publishes no messages at a useful rate — confirmed by `ros2 topic echo` producing no output. The `/uas1/mavlink_source` approach works.

- **Altitude**: already reading `/mavros/local_position/pose` (was the case since 6f/6g).

- **Motor PWM**: also decoded from SERVO_OUTPUT_RAW (msg 36) via `_cb_mavlink` and printed alongside each AGL line during takeoff for diagnostics.

**Why TCP 5760 (MAVProxy master) cannot be used:**
ArduPilot SITL's TCP 5760 only serves one client (MAVProxy). Additional connections are accepted at the TCP level but receive no MAVLink data. Confirmed by pymavlink `wait_heartbeat` timing out despite a successful TCP socket connect.

**Two-phase VPE strategy**

The VPE thread now uses altitude-dependent covariance and position:
- **Phase 1 (below 50 m AGL):** position = home (east=0, north=0), cov_xy = 0.1 m². EKF sets POS_HORIZ_ABS immediately because the drone IS at the known home position on the ground.
- **Phase 2 (above 50 m AGL):** position = AnyLoc estimate from `latest_estimate.json`, cov_xy = max(1.0, error_m²). Only estimates with `agl_m >= 50` accepted (rejects ground-level stubs).

**`launch_mavros.sh` updated:**
Only `--out udp:127.0.0.1:14550` needed in the SITL command. The `--out udp:127.0.0.1:14551` line is removed.

### Key diagnostic findings from debugging session

| Finding | Detail |
|---------|--------|
| `/uas1/mavlink_source` QoS | Publisher uses BEST_EFFORT — subscription must match |
| EKF_STATUS_REPORT flags offset | Byte 20 (not 0) — after 5 floats (velocity_variance, pos_horiz_variance, pos_vert_variance, compass_variance, terrain_alt_variance) |
| GPS_GLOBAL_ORIGIN msg ID | 49 — only echoed when EKF successfully accepts the origin |
| SERVO_OUTPUT_RAW struct | 4 uint16 motors at byte offset 4 (after uint32 time_usec) |
| "Mode change to Guided failed: requires position" | MAVROS2 returns success but ArduPilot silently rejects — indicates EKF flags=0x000 (degraded SITL state) |
| SITL degradation pattern | After 180s failed takeoff, EKF flags drop to 0x000; GPS_GLOBAL_ORIGIN no longer echoed; must restart SITL + drone_sim + MAVROS2 |

### Status

Arming pipeline fully working: connect → EKF origin confirmed → STABILIZE arm → GUIDED → EKF POS_ABS → NAV_TAKEOFF accepted. Takeoff (actual climb) is **not yet working** — motors read from SERVO_OUTPUT_RAW during the climb will be printed in the next run to determine whether ArduPilot is commanding throttle.

### Files modified

| File | Change |
|------|--------|
| `control/flight_commander.py` | Removed pymavlink; added `/uas1/mavlink_source` subscription; `set_ekf_origin()` via GeoPointStamped + GPS_GLOBAL_ORIGIN confirmation; `wait_ekf_pos()` via EKF_STATUS_REPORT flags; two-phase VPE; motor PWM logging |
| `control/launch_mavros.sh` | Updated comments: only `--out udp:127.0.0.1:14550` needed |

---

## 2026-05-31 — flight_commander.py: dead code removed, cleanup fixes

### Bugs fixed

**1. `AltMonitor` class defined but never used**

- Cause: `AltMonitor` (a persistent pymavlink thread exposing live AGL) was created as a planned helper for the VPE thread, but `takeoff()` ended up with its own inline pymavlink connection for altitude polling. The class was left as dead code.
- Fix: deleted the class entirely (~30 lines).
- File: `control/flight_commander.py`

**2. Takeoff failure path missing cleanup**

- Cause: when `takeoff()` returns False, the code called `rclpy.shutdown()` and returned, but did not call `stop_ev.set()` or `cmd.destroy_node()`. Every other failure path (MAVROS2 not connected, EKF timeout) calls all three. The VPE daemon thread was left running and the node was not destroyed.
- Fix: added `stop_ev.set(); cmd.destroy_node()` before `rclpy.shutdown()` in the takeoff failure branch — matching all other failure paths.
- File: `control/flight_commander.py` → `main()` Step 7

**3. RTL disarm timeout too short**

- Cause: `_spin_until(lambda: not cmd._state.armed, timeout=60.0)` — the drone takes off to 90 m AGL and descends at ~1–1.5 m/s during RTL, which takes ~60–90 s to descend plus landing time. The 60 s timeout would expire during descent.
- Fix: increased to 150 s to cover the full 90 m descent + landing sequence.
- File: `control/flight_commander.py` → Step 9

---

## 2026-05-30 — ROS2 node bugs fixed; postview added; dual file+ROS2 output

### Bugs fixed

**1. `ros2_node.py` crashed with `ModuleNotFoundError: faiss`**

- Cause: run command was `source /opt/ros/jazzy/setup.bash && python3 anyloc/ros2_node.py`, which uses system Python 3. System Python has rclpy but not faiss, torch, or PIL (those are in `isaac_sim_test` conda env).
- Fix: run with `conda run -n isaac_sim_test python3` so ML libraries are available. Add `/opt/ros/jazzy/lib/python3.12/site-packages` to `sys.path` inside the script so rclpy is importable from the conda env. Same fix applied to `detection/ros2_node.py` and `control/flight_commander.py`.
- New launch script: `anyloc/run_ros2_localizer.sh` — sources ROS2 then calls `conda run -n isaac_sim_test`.

**2. `VORefiner.update()` called with wrong arguments**

- Cause: `ros2_node.py` was calling `self._vo.update(prev_bgr, curr_bgr, agl_m, yaw_rad)` — passing two BGR numpy arrays, 4 positional args, and yaw in radians.
- Actual signature: `update(self, frame_pil: PIL.Image, agl_m: float, yaw_deg: float)` — takes a single PIL image (stores previous frame internally), and expects yaw in degrees.
- Fix: `dlat, dlon, _ = self._vo.update(pil_img, agl_m, math.degrees(self._drone_yaw))`

**3. `cesium_scene.py` stopped writing files when ROS2 was available**

- Cause: ROS2 publish and file write were in an if/else — when `_ros2_node is not None`, files were never written, so `run_localizer.py` (legacy, polls files by mtime) saw no new frames and stayed stuck at starting position.
- Fix: always write files unconditionally; publish to ROS2 on top when available (dual output).

### Feature added: `ros2_node.py` postview

`anyloc/ros2_node.py` now includes the same dual-window matplotlib postview as `run_localizer.py`:
- Left panel: live drone camera with LAT/LON/ALT/AGL/YAW overlay
- Right panel: AnyLoc satellite match crop with ERR/score/VO pts; green < 200 m, blue otherwise

ROS2 spin runs in a daemon background thread; matplotlib owns the main thread.
Node also writes `anyloc/latest_estimate.json` on each anchor for legacy `run_flight.py` compatibility.

| File | Change |
|------|--------|
| `anyloc/ros2_node.py` | Added sys.path ROS2 fix; fixed VORefiner.update() call; added full postview |
| `anyloc/run_ros2_localizer.sh` | New launch script: `source /opt/ros/jazzy/setup.bash && conda run -n isaac_sim_test python3 anyloc/ros2_node.py` |
| `detection/ros2_node.py` | Added sys.path ROS2 fix |
| `control/flight_commander.py` | Added sys.path ROS2 fix |
| `simulator/cesium_scene.py` | Always writes files; publishes ROS2 on top (dual output) |

---

## 2026-06-07 — Imagery → Esri; YOLO threshold 0.50; detection logging without divert

### Imagery: NLSC → Esri World Imagery (zoom 19)

NLSC WMTS PHOTO2 tiles contain embedded watermarks (text/logos) that caused YOLO false
positives. Switched to **Esri World Imagery** in both `cesium_scene.py` and
`anyloc/build_database.py`:

```python
ESRI_TILE_URL = ("https://server.arcgisonline.com/ArcGIS/rest/services"
                 "/World_Imagery/MapServer/tile/{z}/{y}/{x}")
SAT_ZOOM = 19   # ~0.37 m/px effective after MAX_TEX=16384 downsampling
```

Zoom 18 (old) → 0.60 m/px; zoom 19 → ~0.37 m/px effective (zoom 20 hits the same cap
but requires 4× more tiles). Deleted cached `simulator/satellite_ground.jpg` (NLSC mosaic)
so it re-downloads as Esri on next run. AnyLoc database must be rebuilt after this change.

### YOLO confidence threshold: 0.30 → 0.50

`detection/ros2_node.py` line: `self._det = YOLODetector(MODEL_PT, conf=0.50)`

### Detection logging without flight divert

**Previous behaviour:** YOLO detection inside buffered zone → set `SurveyState.DIVERT` →
interrupt `go_to_ned()` early → fly to car position (10 m radius) → log at final centred
position → resume same waypoint.

**New behaviour:** YOLO detection → yaw-corrected GSD pixel projection → dedup check
(30 m) → log immediately → survey continues unbroken.

Removed: `SurveyState` enum, `DETECT_RADIUS`, `interruptible` param from `go_to_ned()`,
DIVERT branch in survey loop, zone check in `_cb_detections()`.

**Yaw-corrected projection formula** (`h` = NED heading CW from north):
```python
q       = self._drone.pose.orientation
yaw_enu = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y**2 + q.z**2))
h = -yaw_enu
dx_m =  (cx - CAM_W/2) * gsd_x   # drone-right
dy_m = -(cy - CAM_H/2) * gsd_y   # drone-forward
de = dx_m*math.cos(h) + dy_m*math.sin(h)
dn = -dx_m*math.sin(h) + dy_m*math.cos(h)
```

Derivation: camera top follows drone nose; forward direction in ENU = (sin(h), cos(h));
right direction = (cos(h), −sin(h)). Verified at h=0 (north), h=π/2 (east), h=π (south).

| File | Change |
|------|--------|
| `simulator/cesium_scene.py` | `ESRI_TILE_URL`, `SAT_ZOOM=19`; removed NLSC URL |
| `anyloc/build_database.py` | Same imagery constants |
| `detection/ros2_node.py` | `conf=0.50` |
| `control/px4_commander.py` | Removed `SurveyState`/`DETECT_RADIUS`/`interruptible`; `_cb_detections` projects + logs; survey loop sequential |

---

## 2026-06-07 — RTL → explicit fly-home + AUTO.LAND

**Symptom:** After completing all survey waypoints the drone flew south indefinitely and
never returned to the takeoff point.

**Root cause:** `set_mode("RTL")` is unreliable with external-vision-only PX4. PX4's RTL
mode requires a GPS-derived home position. When it fails or is ignored, the drone stays
in OFFBOARD mode executing the last published velocity setpoint — which pointed south
(toward WP11 at N=113) — and keeps flying that direction without stopping.

**Fix:** Stay in OFFBOARD mode after the survey; use `go_to_ned(0, 0, TAKEOFF_ALT)` to
fly home under continuous setpoint control, then `set_mode("AUTO.LAND")` for the final
descent. Same change applied to the Ctrl-C handler.

```python
# Before
cmd.set_mode("RTL")

# After
cmd.go_to_ned(0.0, 0.0, TAKEOFF_ALT, timeout=300.0, speed=SURVEY_SPEED)
cmd.set_mode("AUTO.LAND")
```

| File | Change |
|------|--------|
| `control/px4_commander.py` | Survey-complete + Ctrl-C: fly home via `go_to_ned` then `AUTO.LAND` |
