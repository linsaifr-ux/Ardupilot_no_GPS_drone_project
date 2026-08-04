#!/usr/bin/env python3
"""
Closed-loop SITL test: does the real survey25 VIO+AnyLoc fusion pipeline's
position estimate make ArduPilot's EKF (and therefore the flown path) jump
around, when flown as a real AUTO mission instead of scripted GUIDED setpoints?

Method
------
- A mission is built from survey25's own flown GPS track (telemetry.csv):
  HOME item, TAKEOFF to ~100 m AGL, a handful of cruise waypoints simplified
  from the real track (Douglas-Peucker), LAND at the last one. Uploaded over
  MAVLink's MISSION_ITEM_INT protocol (no prior mission-upload code existed
  in this repo -- implemented here).
- ArduCopter SITL boots with real_hw.parm-mirroring EK3/PSC params (no GPS,
  ExternalNav position source, SRC1 here since SITL boots straight into it),
  arms, and is set to AUTO -- ArduPilot itself flies the mission (real
  waypoint navigation, not us pushing setpoints).
- At 20 Hz (matching the real commander's cadence) VISION_POSITION_ESTIMATE
  = SITL's own true position (from SIMSTATE) + the REAL pipeline's error
  signal, replayed from field_data/survey25/vio_eval/vio_full_real_anchor_fix1plus2.csv
  (the actual real-AnyLoc full-pipeline output, verified end-to-end
  rmse=230.2 m over rel t=170-472s against telemetry GPS truth before this
  script trusted it -- NOT the idealized simulated-anchor proxy).
- Three runs: `zero` (no injected error -- sanity control that the mission/
  harness itself works), `raw` (real pipeline error, unfiltered), `slew`
  (same error through control/vpe_slew.py). SIMSTATE truth, EKF
  LOCAL_POSITION_NED, and published VPE are all logged every tick.

Usage
-----
    python3 control/test_full_pipeline_sitl.py run zero
    python3 control/test_full_pipeline_sitl.py run raw
    python3 control/test_full_pipeline_sitl.py run slew
    python3 control/test_full_pipeline_sitl.py analyze     # stats from saved logs
    python3 control/test_full_pipeline_sitl.py plot        # PNG from saved logs
    python3 control/test_full_pipeline_sitl.py all         # 3 runs + analyze + plot

Outputs: field_data/survey25/vio_eval/sitl/full_pipeline_sitl_<run>.json
         field_data/survey25/vio_eval/sitl/full_pipeline_sitl.png
"""

import csv
import json
import math
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, ROOT)

from vpe_slew import VpeSlewLimiter

ARDUCOPTER = os.path.join(ROOT, "third_party", "ardupilot",
                          "build", "sitl", "bin", "arducopter")
S          = os.path.join(ROOT, "field_data", "survey25")
V          = os.path.join(S, "vio_eval")
OUT_DIR    = os.path.join(V, "sitl")
os.makedirs(OUT_DIR, exist_ok=True)

FUSED_CSV  = os.path.join(V, "vio_full_real_anchor_fix1plus2.csv")
TELEMETRY  = os.path.join(S, "telemetry.csv")
META       = os.path.join(S, "meta.json")

CRUISE_AGL = 100.0
TICK_S     = 0.05          # 20 Hz vision feed (commander cadence)
SETP_HZ    = 4              # NAV_CONTROLLER_OUTPUT / MISSION_CURRENT poll rate share
HOLD_S     = 8.0            # settle time after LAND before ending capture
SPEEDUP    = 4               # SITL --speedup; verified below to still fly sanely
CONNECT    = "tcp:127.0.0.1:5760"   # SERIAL0: SITL blocks at boot until a client connects here

# EK3/PSC/WPNAV mirroring control/real_hw.parm (that file uses SRC2 for
# ExternalNav since SRC1=GPS there; here SITL boots straight into vision so
# it's SRC1 -- same numeric values, matches the precedent in
# control/test_vpe_slew_sitl.py), GPS fully disabled.
SITL_PARAMS = {
    "AHRS_EKF_TYPE":   3,
    "FRAME_CLASS":     1,
    "FRAME_TYPE":      1,
    "GPS_TYPE":        0,
    "GPS1_TYPE":       0,
    "SIM_GPS_DISABLE": 1,
    "SIM_GPS1_ENABLE": 0,
    "EK3_SRC1_POSXY":  6,      # ExternalNav position
    "EK3_SRC1_VELXY":  0,      # matches real_hw.parm SRC2_VELXY=0
    "EK3_SRC1_POSZ":   1,      # baro
    "EK3_SRC1_VELZ":   0,
    "EK3_SRC1_YAW":    1,      # compass
    "VISO_TYPE":       1,
    "EK3_GLITCH_RAD":  50,     # matches real_hw.parm
    "PSC_NE_POS_P":    0.2,    # real_hw.parm position controller
    "PSC_NE_VEL_P":    2.0,
    "PSC_NE_VEL_I":    0.0,
    "PSC_NE_VEL_D":    0.5,
    "WPNAV_SPEED":     1200,   # real_hw.parm: 12 m/s
    "WPNAV_SPEED_UP":  300,
    "WPNAV_SPEED_DN":  150,
    "WPNAV_RADIUS":    800,    # 8 m waypoint acceptance -- generous given injected VPE noise
    "ARMING_CHECK":    0,
    "GPS_ARMING_MIN_SAT": 0,
    "FS_GPS_ENABLE":   0,
    "FS_EKF_THRESH":   1.0,    # relaxed -- we want to observe the runaway, not failsafe out of it
    "FS_CRASH_CHECK":  0,
    "FENCE_ENABLE":    0,
    "DISARM_DELAY":    0,
    "LAND_SPEED":      50,     # cm/s
    "LOG_BACKEND_TYPE": 0,     # no dataflash logs in the scratch dir
}


# ── survey25 data ───────────────────────────────────────────────────────────
def load_telemetry():
    rows = []
    with open(TELEMETRY) as f:
        for r in csv.DictReader(f):
            try:
                rows.append((float(r["unix_time"]), float(r["lat"]), float(r["lon"]),
                             float(r["alt_amsl"]), float(r["alt_agl"])))
            except ValueError:
                pass
    return rows


def latlon_m_consts(lat0):
    latm = 111132.954 - 559.822 * math.cos(2 * math.radians(lat0))
    lonm = 111412.84 * math.cos(math.radians(lat0)) - 93.5 * math.cos(3 * math.radians(lat0))
    return latm, lonm


def rdp(points, eps):
    """Douglas-Peucker simplification. points: list of (x, y[, payload])."""
    if len(points) < 3:
        return points
    def perp_dist(p, a, b):
        ax, ay = a[0], a[1]
        bx, by = b[0], b[1]
        px, py = p[0], p[1]
        dx, dy = bx - ax, by - ay
        if dx == 0 and dy == 0:
            return math.hypot(px - ax, py - ay)
        t = ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)
        t = max(0.0, min(1.0, t))
        cx, cy = ax + t * dx, ay + t * dy
        return math.hypot(px - cx, py - cy)
    dmax, idx = 0.0, 0
    for i in range(1, len(points) - 1):
        d = perp_dist(points[i], points[0], points[-1])
        if d > dmax:
            dmax, idx = d, i
    if dmax > eps:
        left = rdp(points[:idx + 1], eps)
        right = rdp(points[idx:], eps)
        return left[:-1] + right
    return [points[0], points[-1]]


def build_mission():
    """Returns (mission_items_latlonalt, home_lat, home_lon, home_alt_msl, T0, latm, lonm)."""
    tel = load_telemetry()
    T0 = json.load(open(META))["video_start_unix"]
    lat0, lon0, alt0_msl, _ = tel[0][1], tel[0][2], tel[0][3], tel[0][4]
    latm, lonm = latlon_m_consts(lat0)

    # cruise phase, per session_2026-07-24 timeline: level cruise ~167-319s
    # video-relative; trim a bit inside that so we don't simplify the
    # climb-out or the start of descent as if they were cruise legs.
    cruise = [(t - T0, lat, lon) for (t, lat, lon, alt_msl, alt_agl) in tel
              if 170.0 <= (t - T0) <= 315.0]
    pts_m = [((lon - lon0) * lonm, (lat - lat0) * latm, lat, lon) for (_, lat, lon) in cruise]
    simplified = rdp(pts_m, eps=8.0)
    # cap to a sane waypoint count -- this is a coarse mission, not a survey grid
    if len(simplified) > 18:
        step = len(simplified) / 18.0
        simplified = [simplified[int(i * step)] for i in range(18)] + [simplified[-1]]

    wp_latlon = [(p[2], p[3]) for p in simplified]
    print(f"[mission] {len(cruise)} cruise telemetry points -> {len(wp_latlon)} waypoints")
    return wp_latlon, lat0, lon0, alt0_msl, T0, latm, lonm


# ── MAVLink mission upload (no prior implementation in this repo) ──────────
def upload_mission(mav, items, timeout=30.0):
    """items: list of (frame, command, current, autocontinue, p1, p2, p3, p4, x_lat_e7, y_lon_e7, z)."""
    from pymavlink import mavutil
    n = len(items)
    deadline = time.time() + timeout
    mav.mav.mission_count_send(mav.target_system, mav.target_component, n,
                                mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
    sent = set()
    while time.time() < deadline:
        msg = mav.recv_match(type=["MISSION_REQUEST_INT", "MISSION_REQUEST", "MISSION_ACK"],
                              blocking=True, timeout=2.0)
        if msg is None:
            # nudge again in case the initial COUNT was dropped
            mav.mav.mission_count_send(mav.target_system, mav.target_component, n,
                                        mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
            continue
        t = msg.get_type()
        if t == "MISSION_ACK":
            ok = msg.type == mavutil.mavlink.MAV_MISSION_ACCEPTED
            print(f"[mission] MISSION_ACK type={msg.type} "
                  f"({'ACCEPTED' if ok else 'REJECTED'})")
            return ok
        seq = msg.seq
        if seq in sent or seq >= n:
            continue
        frame, cmd, cur, ac, p1, p2, p3, p4, xlat, ylon, z = items[seq]
        mav.mav.mission_item_int_send(
            mav.target_system, mav.target_component, seq, frame, cmd, cur, ac,
            p1, p2, p3, p4, xlat, ylon, z)
        sent.add(seq)
    print("[mission] upload timed out")
    return False


def mission_items_from_waypoints(wp_latlon, home_lat, home_lon, home_alt_msl):
    from pymavlink import mavutil
    F = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
    items = []
    # seq 0: home placeholder (ArduPilot convention)
    items.append((mavutil.mavlink.MAV_FRAME_GLOBAL, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                  0, 1, 0, 0, 0, 0, int(home_lat * 1e7), int(home_lon * 1e7), home_alt_msl))
    # seq 1: takeoff
    items.append((F, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 1,
                  0, 0, 0, 0, int(home_lat * 1e7), int(home_lon * 1e7), CRUISE_AGL))
    # cruise waypoints
    for lat, lon in wp_latlon:
        items.append((F, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1,
                      0, 0, 0, 0, int(lat * 1e7), int(lon * 1e7), CRUISE_AGL))
    # land at the last waypoint
    last_lat, last_lon = wp_latlon[-1]
    items.append((F, mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 1,
                  0, 0, 0, 0, int(last_lat * 1e7), int(last_lon * 1e7), 0.0))
    return items


# ── real-pipeline error signal ──────────────────────────────────────────────
def load_pipeline_error(lat0, lon0, latm, lonm, fused_csv=None):
    """Returns (t0_orig, t_end_orig, err_at(t_orig) -> (east, north))."""
    v_rows = []
    with open(fused_csv or FUSED_CSV) as f:
        for r in csv.DictReader(f):
            v_rows.append((float(r["t"]), float(r["px"]), float(r["py"])))
    tel = load_telemetry()
    tg = [t for (t, _, _, _, _) in tel]
    gx = [(lon - lon0) * lonm for (_, _, lon, _, _) in tel]
    gy = [(lat - lat0) * latm for (_, lat, _, _, _) in tel]

    def truth_at(t):
        return _interp1(tg, gx, t), _interp1(tg, gy, t)

    ts = [r[0] for r in v_rows]
    px = [r[1] for r in v_rows]
    py = [r[2] for r in v_rows]

    def err_at(t):
        fx, fy = _interp1(ts, px, t), _interp1(ts, py, t)
        tx, ty = truth_at(t)
        return fx - tx, fy - ty

    return ts[0], ts[-1], err_at


def _interp1(xs, ys, x):
    if x <= xs[0]:
        return ys[0]
    if x >= xs[-1]:
        return ys[-1]
    lo, hi = 0, len(xs) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if xs[mid] <= x:
            lo = mid
        else:
            hi = mid
    a = (x - xs[lo]) / (xs[hi] - xs[lo])
    return ys[lo] + a * (ys[hi] - ys[lo])


# ── SITL flight ──────────────────────────────────────────────────────────────
def run_flight(run_name: str, fused_csv=None):
    from mavlink_ctrl import MAVLinkCtrl
    from pymavlink import mavutil

    wp_latlon, lat0, lon0, home_alt_msl, T0, latm, lonm = build_mission()
    _, err_t1, err_at = load_pipeline_error(lat0, lon0, latm, lonm, fused_csv=fused_csv)
    # route_t=0 (our AUTO cruise start, once climb finishes) is aligned to
    # the ORIGINAL recording's cruise start (T0+170s, matching
    # build_mission()'s cruise window), not the original climb start -- the
    # injected error must be phase-matched (cruise error during our cruise),
    # not offset by however long our own climb happened to take.
    err_t0 = T0 + 170.0
    print(f"[SITL-full] run={run_name} home=({lat0:.6f},{lon0:.6f}) "
          f"{len(wp_latlon)} cruise wps, pipeline error replay window "
          f"{err_t0:.1f}-{err_t1:.1f} ({err_t1 - err_t0:.0f}s, cruise-aligned)")

    workdir = os.path.join(os.environ.get("TMPDIR", "/tmp"),
                           f"full_pipeline_sitl_{run_name}")
    os.makedirs(workdir, exist_ok=True)
    parm = os.path.join(workdir, "test.parm")
    with open(parm, "w") as f:
        for k, v in SITL_PARAMS.items():
            f.write(f"{k} {v}\n")

    proc = subprocess.Popen(
        [ARDUCOPTER, "--model", "quad", "--speedup", str(SPEEDUP),
         "--defaults", parm, "--home", f"{lat0},{lon0},{home_alt_msl},0", "-I0"],
        cwd=workdir, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
    time.sleep(4)

    log = dict(run=run_name, home=[lat0, lon0], rows=[], mission_n=len(wp_latlon) + 3)
    try:
        ctrl = MAVLinkCtrl(CONNECT)
        if not ctrl.wait_heartbeat(30):
            raise RuntimeError("no heartbeat from SITL")
        mav = ctrl._mav
        for msg_id, hz in ((mavutil.mavlink.MAVLINK_MSG_ID_SIMSTATE, 25),
                           (mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 25),
                           (mavutil.mavlink.MAVLINK_MSG_ID_MISSION_CURRENT, 5),
                           (mavutil.mavlink.MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT, 5)):
            mav.mav.command_long_send(
                mav.target_system, mav.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                msg_id, int(1e6 / hz), 0, 0, 0, 0, 0)

        ctrl.set_ekf_origin(lat0, lon0, home_alt_msl)
        ctrl.set_home_position(lat0, lon0, home_alt_msl)

        slew = VpeSlewLimiter() if run_name == "slew" else None

        def sim_truth_en():
            m = mav.messages.get("SIMSTATE")
            if m is None:
                return None
            la, lo_ = m.lat, m.lng
            if abs(la) > 1000:          # int degE7 encoding
                la, lo_ = la / 1e7, lo_ / 1e7
            return (lo_ - lon0) * lonm, (la - lat0) * latm

        def vpe_tick(route_t=None):
            te_n = sim_truth_en()
            if te_n is None:
                return None
            te, tn = te_n
            if run_name == "zero" or route_t is None:
                ee, en = 0.0, 0.0
            else:
                orig_t = min(err_t0 + route_t, err_t1)
                ee, en = err_at(orig_t)
            tgt_e, tgt_n = te + ee, tn + en
            if slew is not None:
                pe, pn = slew.update(tgt_e, tgt_n, time.monotonic())
            else:
                pe, pn = tgt_e, tgt_n
            lp = ctrl.local_pos
            down = lp.z if lp is not None else 0.0
            ctrl.send_vision_position(pn, pe, down, 0.0)
            if route_t is not None:
                mc = mav.messages.get("MISSION_CURRENT")
                nco = mav.messages.get("NAV_CONTROLLER_OUTPUT")
                log["rows"].append(dict(
                    t=round(route_t, 3),
                    truth=[round(te, 2), round(tn, 2)],
                    ekf=[round(lp.y, 2), round(lp.x, 2)] if lp else None,
                    pub=[round(pe, 2), round(pn, 2)],
                    mis_seq=int(mc.seq) if mc else None,
                    wp_dist=float(nco.wp_dist) if nco else None))
            return pe, pn

        def pump(duration=None, cond=None, route_start=None, timeout=180):
            t_end = time.monotonic() + (duration if duration else timeout)
            next_vpe = 0.0
            while time.monotonic() < t_end:
                now = time.monotonic()
                ctrl.recv()
                if now >= next_vpe:
                    rt = (now - route_start) if route_start else None
                    vpe_tick(rt)
                    next_vpe = now + TICK_S
                if cond is not None and cond():
                    return True
                time.sleep(0.005)
            return duration is not None

        print("[SITL-full] waiting for EKF position…")
        if not pump(cond=lambda: ctrl.ekf_pos_valid, timeout=90):
            raise RuntimeError("EKF never got a position fix")
        pump(duration=5)

        for _ in range(10):
            if "GPS_GLOBAL_ORIGIN" not in mav.messages:
                ctrl.set_ekf_origin(lat0, lon0, home_alt_msl)
            else:
                mav.mav.command_long_send(
                    mav.target_system, mav.target_component,
                    mavutil.mavlink.MAV_CMD_DO_SET_HOME, 0,
                    0, 0, 0, 0, lat0, lon0, home_alt_msl)
            pump(duration=1)
            if "HOME_POSITION" in mav.messages:
                break
        else:
            raise RuntimeError("origin/home never accepted")

        items = mission_items_from_waypoints(wp_latlon, lat0, lon0, home_alt_msl)
        print(f"[SITL-full] uploading {len(items)}-item mission…")
        ok = upload_mission(mav, items)
        if not ok:
            raise RuntimeError("mission upload failed / rejected")

        ctrl.set_mode("GUIDED")
        pump(duration=1)
        for attempt in range(3):
            ctrl.arm(force=True)
            pump(duration=3)
            if ctrl.is_armed:
                break
            st = mav.messages.get("STATUSTEXT")
            print(f"[SITL-full] arm attempt {attempt + 1} failed; "
                  f"ack={ctrl._last_ack.get(400)} statustext={st.text if st else '-'}")
        if not ctrl.is_armed:
            raise RuntimeError("arm failed")

        # Auto-takeoff straight from an uploaded AUTO mission (arm while in
        # AUTO with a NAV_TAKEOFF current item) never left the ground in
        # testing -- ArduCopter's auto_armed gate needs either RC throttle
        # input (none exists here) or an explicit GUIDED takeoff. Do the
        # climb the same way control/test_vpe_slew_sitl.py does (GUIDED +
        # MAV_CMD_NAV_TAKEOFF), THEN hand off to AUTO at the first real
        # waypoint. Critically: never use MAVLinkCtrl.wait_altitude()/
        # wait_position() for this -- they poll without calling vpe_tick(),
        # which starves EKF3's ExternalNav input (no velocity source to fall
        # back on) for the whole wait and visibly stalls/derails the climb
        # (found empirically: climbed only 14 m in 60 s and then disarmed,
        # vs. a clean ~2.5 m/s climb once vpe_tick() ran every poll tick).
        ctrl.takeoff(CRUISE_AGL)
        print("[SITL-full] climbing to cruise AGL via GUIDED takeoff…")
        lp_alt = lambda: (ctrl.local_pos is not None
                          and -ctrl.local_pos.z > CRUISE_AGL - 3.0)
        if not pump(cond=lp_alt, timeout=90):
            raise RuntimeError("takeoff did not reach cruise altitude")
        pump(duration=3)

        # hand off to the uploaded mission at the first real cruise
        # waypoint (seq 0=home placeholder, 1=takeoff already flown, 2=first
        # real MAV_CMD_NAV_WAYPOINT)
        mav.mav.mission_set_current_send(mav.target_system, mav.target_component, 2)
        pump(duration=1)
        ctrl.set_mode("AUTO")
        pump(duration=1)
        hb = mav.messages.get("HEARTBEAT")
        print(f"[SITL-full] mode after AUTO request: custom_mode={hb.custom_mode if hb else '?'}")

        print(f"[SITL-full] flying survey25 mission ({run_name} VPE)…")
        route_start = time.monotonic()
        cruise_span_s = 315.0 - 170.0   # matches build_mission()'s cruise window
        # generous ceiling: mission should self-terminate at LAND (disarm);
        # stop early if disarmed, else hard-timeout well past the expected span
        def landed():
            return route_start and (time.monotonic() - route_start) > 20 and not ctrl.is_armed
        pump(duration=None, cond=landed, route_start=route_start,
             timeout=cruise_span_s / max(SPEEDUP, 1) * 4 + 120)
        pump(duration=HOLD_S)
        print(f"[SITL-full] flight segment done, armed={ctrl.is_armed}")
    finally:
        proc.terminate()
        try:
            proc.wait(5)
        except subprocess.TimeoutExpired:
            proc.kill()

    out = os.path.join(OUT_DIR, f"full_pipeline_sitl_{run_name}.json")
    with open(out, "w") as f:
        json.dump(log, f)
    n = len(log["rows"])
    print(f"[SITL-full] {n} samples -> {out}")
    return out


# ── analysis ─────────────────────────────────────────────────────────────────
def dist_to_polyline(p, pts):
    best = float("inf")
    px, py = p
    for (ax, ay), (bx, by) in zip(pts, pts[1:]):
        dx, dy = bx - ax, by - ay
        L2 = dx * dx + dy * dy
        u = 0.0 if L2 == 0 else max(0.0, min(1.0,
            ((px - ax) * dx + (py - ay) * dy) / L2))
        best = min(best, math.hypot(px - (ax + u * dx), py - (ay + u * dy)))
    return best


def analyze():
    wp_latlon, lat0, lon0, home_alt_msl, T0, latm, lonm = build_mission()
    route_m = [((lon - lon0) * lonm, (lat - lat0) * latm) for lat, lon in wp_latlon]

    for run_name in ("zero", "raw", "slew"):
        path = os.path.join(OUT_DIR, f"full_pipeline_sitl_{run_name}.json")
        if not os.path.exists(path):
            print(f"=== {run_name}: no log ===")
            continue
        with open(path) as f:
            rows = json.load(f)["rows"]
        if len(rows) < 2:
            print(f"=== {run_name}: only {len(rows)} samples, skipping ===")
            continue
        ekf_steps, pub_steps, xtrack = [], [], []
        prev_ekf = prev_pub = None
        glitch_events = []
        for r in rows:
            if r["ekf"] is not None:
                if prev_ekf is not None:
                    d = math.hypot(r["ekf"][0] - prev_ekf[0], r["ekf"][1] - prev_ekf[1])
                    ekf_steps.append(d)
                    if d > 50.0:
                        glitch_events.append((r["t"], d))
                prev_ekf = r["ekf"]
            if prev_pub is not None:
                pub_steps.append(math.hypot(r["pub"][0] - prev_pub[0], r["pub"][1] - prev_pub[1]))
            prev_pub = r["pub"]
            if r["ekf"] is not None:
                xtrack.append(dist_to_polyline(r["ekf"], route_m))
        import statistics as st
        print(f"=== {run_name}: n={len(rows)} span={rows[-1]['t']:.0f}s "
              f"final mis_seq={rows[-1]['mis_seq']} ===")
        if ekf_steps:
            print(f"  EKF per-tick step: mean={st.mean(ekf_steps):.2f} "
                  f"p95={sorted(ekf_steps)[int(0.95*len(ekf_steps))]:.2f} "
                  f"max={max(ekf_steps):.2f} m  (tick={TICK_S}s)")
        print(f"  glitch-threshold (>50m) EKF steps: {len(glitch_events)}"
              + (f", e.g. t={glitch_events[0][0]:.0f}s d={glitch_events[0][1]:.0f}m" if glitch_events else ""))
        if xtrack:
            print(f"  cross-track from mission route: mean={st.mean(xtrack):.1f} "
                  f"max={max(xtrack):.1f} m")


def plot():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    wp_latlon, lat0, lon0, home_alt_msl, T0, latm, lonm = build_mission()
    route_m = [((lon - lon0) * lonm, (lat - lat0) * latm) for lat, lon in wp_latlon]

    runs = [r for r in ("zero", "raw", "slew")
            if os.path.exists(os.path.join(OUT_DIR, f"full_pipeline_sitl_{r}.json"))]
    fig, axes = plt.subplots(2, len(runs), squeeze=False, figsize=(7 * len(runs), 10))
    for col, run_name in enumerate(runs):
        with open(os.path.join(OUT_DIR, f"full_pipeline_sitl_{run_name}.json")) as f:
            rows = json.load(f)["rows"]
        ax, axd = axes[0][col], axes[1][col]
        ax.plot([p[0] for p in route_m], [p[1] for p in route_m],
                "k--", lw=1.5, label="mission route (survey25-derived)")
        ax.plot([r["truth"][0] for r in rows], [r["truth"][1] for r in rows],
                color="tab:blue", lw=1.5, label="SITL flown (truth)")
        ekf_pts = [r["ekf"] for r in rows if r["ekf"] is not None]
        ax.plot([p[0] for p in ekf_pts], [p[1] for p in ekf_pts],
                color="tab:red", lw=1.0, alpha=0.7, label="EKF position")
        ax.set_title(f"{run_name}"); ax.set_xlabel("East [m]"); ax.set_ylabel("North [m]")
        ax.axis("equal"); ax.grid(alpha=0.3); ax.legend(fontsize=8)

        steps = []
        prev = None
        for r in rows:
            if r["ekf"] is not None:
                if prev is not None:
                    steps.append((r["t"], math.hypot(r["ekf"][0]-prev[0], r["ekf"][1]-prev[1])))
                prev = r["ekf"]
        axd.plot([s[0] for s in steps], [s[1] for s in steps], color="tab:red", lw=0.8)
        axd.axhline(50, color="k", ls=":", lw=1, label="EK3_GLITCH_RAD")
        axd.set_title("EKF per-tick step size"); axd.set_xlabel("route time [s]")
        axd.set_ylabel("step [m]"); axd.grid(alpha=0.3); axd.legend(fontsize=8)
    fig.suptitle("survey25 full pipeline, AUTO-mode SITL closed loop", fontsize=13)
    fig.tight_layout()
    out = os.path.join(OUT_DIR, "full_pipeline_sitl.png")
    fig.savefig(out, dpi=110)
    print(f"[SITL-full] plot -> {out}")
    return out


def main():
    args = sys.argv[1:]
    if args[:1] == ["run"] and len(args) == 2:
        run_flight(args[1])
    elif args[:1] == ["run"] and len(args) == 3:
        # e.g. `run samedomain field_data/survey25/vio_eval/samedomain_100m_ext/vio_full_samedomain_100m_ext_fused.csv`
        run_flight(args[1], fused_csv=args[2])
    elif args[:1] == ["analyze"]:
        analyze()
    elif args[:1] == ["plot"]:
        plot()
    elif args[:1] == ["all"]:
        for r in ("zero", "raw", "slew"):
            run_flight(r)
        analyze()
        plot()
    else:
        print(__doc__)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
