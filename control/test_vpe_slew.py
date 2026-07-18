#!/usr/bin/env python3
"""
Offline validation of the VPE slew limiter (control/vpe_slew.py) against the
survey13 localizer replay tracks.

Feeds the recorded localizer output (anyloc/logs/survey13_jumpgate_replay.json
and _stress.json — per-frame lat/lon of the plan-B "new" and legacy "old"
policies, 30 fps with timestamps) through the limiter at the commander's real
20 Hz cadence and compares against the current pass-through behaviour:

  step      largest single-tick move of the *published* position (what EKF3
            sees as an instantaneous displacement)
  corr_vel  largest 1 s-window speed of (published − GPS truth) — motion the
            EKF would perceive that the aircraft did not actually perform
            (this is what triggers the phantom-velocity lurch)
  err       accuracy vs GPS truth — the limiter must not materially degrade it

The limiter is self-contained (its speed allowance comes from the target's
own motion), so the replay needs no velocity feed.

Usage:
    python3 control/test_vpe_slew.py            # both survey13 logs
    python3 control/test_vpe_slew.py file.json  # specific replay log(s)
"""

import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from vpe_slew import VpeSlewLimiter, VPE_SLEW_CORR_MPS

M_PER_DEG = 111_320.0
TICK_S    = 0.05          # commander vision-thread period (20 Hz)
CORR_WIN  = 1.0           # s — window for the correction-speed metric

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_LOGS = [
    os.path.join(ROOT, "anyloc", "logs", "survey13_jumpgate_replay.json"),
    os.path.join(ROOT, "anyloc", "logs", "survey13_jumpgate_stress.json"),
]


# ── synthetic sanity checks ────────────────────────────────────────────────────
def unit_checks():
    print("Unit checks")
    ok = True

    # 30 m step while hovering: glide at corr rate, ~12 s to converge
    s = VpeSlewLimiter()
    s.reset(0.0, 0.0, 0.0)
    t, done_t, max_step, e, n = 0.0, None, 0.0, 0.0, 0.0
    while t < 20.0:
        t += TICK_S
        pe, pn = s.update(30.0, 0.0, t)
        max_step = max(max_step, math.hypot(pe - e, pn - n))
        e, n = pe, pn
        if done_t is None and math.hypot(30.0 - pe, pn) < 0.01:
            done_t = t
    exp = 30.0 / VPE_SLEW_CORR_MPS
    ok1 = abs(done_t - exp) < 0.5 and max_step <= VPE_SLEW_CORR_MPS * TICK_S + 1e-6
    ok &= ok1
    print(f"  hover 30 m step  : glide {done_t:.1f} s (expect ~{exp:.1f}), "
          f"max tick step {max_step:.3f} m  {'ok' if ok1 else 'FAIL'}")

    # cruise at 8 m/s, target moves with the drone: zero lag after the
    # speed-median warms up (~2 ticks)
    s = VpeSlewLimiter()
    s.reset(0.0, 0.0, 0.0)
    t, lag = 0.0, 0.0
    while t < 10.0:
        t += TICK_S
        tgt = 8.0 * t
        pe, _ = s.update(tgt, 0.0, t)
        if t > 1.0:
            lag = max(lag, abs(tgt - pe))
    ok2 = lag < 1e-6
    ok &= ok2
    print(f"  cruise 8 m/s     : max lag {lag:.6f} m  {'ok' if ok2 else 'FAIL'}")

    # 20 m step during 8 m/s cruise: correction closes at corr rate; the
    # jump itself must not inflate the speed estimate
    s = VpeSlewLimiter()
    s.reset(0.0, 0.0, 0.0)
    t, worst, prev_off = 0.0, 0.0, 0.0
    while t < 15.0:
        t += TICK_S
        tgt = 8.0 * t + (20.0 if t > 2.0 else 0.0)
        pe, _ = s.update(tgt, 0.0, t)
        off = abs(tgt - pe)
        if t > 2.0 + TICK_S:
            worst = max(worst, (prev_off - off) / TICK_S)
        prev_off = off
    ok3 = worst <= VPE_SLEW_CORR_MPS + 1e-6 and prev_off < 0.01
    ok &= ok3
    print(f"  20 m step @8 m/s : correction rate ≤ {worst:.2f} m/s "
          f"(cap {VPE_SLEW_CORR_MPS}), residual {prev_off:.3f} m  "
          f"{'ok' if ok3 else 'FAIL'}")

    # scheduler stall: dt clamp bounds a single-tick glide
    s = VpeSlewLimiter()
    s.reset(0.0, 0.0, 0.0)
    pe, _ = s.update(100.0, 0.0, 5.0)          # 5 s gap
    ok4 = pe <= VPE_SLEW_CORR_MPS * 0.5 + 1e-6
    ok &= ok4
    print(f"  5 s thread stall : single glide {pe:.2f} m "
          f"(cap {VPE_SLEW_CORR_MPS * 0.5:.2f})  {'ok' if ok4 else 'FAIL'}")
    return ok


# ── replay simulation ──────────────────────────────────────────────────────────
def to_en(lat, lon, lat0, lon0, cos_lat):
    return (lon - lon0) * M_PER_DEG * cos_lat, (lat - lat0) * M_PER_DEG


def make_truth_interp(rows, lat0, lon0, cos_lat):
    """Linear interpolation over the *changes* of the recorded truth track
    (it is piecewise-constant at telemetry rate; stepping it would put a
    ~flight-speed floor under the correction-velocity metric)."""
    ts  = [r["t"] for r in rows]
    tru = [to_en(r["true_lat"], r["true_lon"], lat0, lon0, cos_lat)
           for r in rows]
    ch = [0] + [k for k in range(1, len(tru)) if tru[k] != tru[k - 1]]
    cts = [ts[k] for k in ch]
    cen = [tru[k] for k in ch]

    def at(t):
        if t <= cts[0]:
            return cen[0]
        if t >= cts[-1]:
            return cen[-1]
        lo, hi = 0, len(cts) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if cts[mid] <= t:
                lo = mid
            else:
                hi = mid
        a = (t - cts[lo]) / (cts[hi] - cts[lo])
        return (cen[lo][0] + a * (cen[hi][0] - cen[lo][0]),
                cen[lo][1] + a * (cen[hi][1] - cen[lo][1]))
    return at


def simulate(rows, prefix, lat0, lon0, cos_lat):
    """Run raw pass-through and slewed publishing over one localizer track."""
    ts  = [r["t"] for r in rows]
    tgt = [to_en(r[f"{prefix}_lat"], r[f"{prefix}_lon"], lat0, lon0, cos_lat)
           for r in rows]
    tru_at = make_truth_interp(rows, lat0, lon0, cos_lat)

    slew = VpeSlewLimiter()
    pub = dict(raw=[], slew=[])
    tru_tick = []
    i = 0
    t = ts[0]
    while t <= ts[-1]:
        while i + 1 < len(ts) and ts[i + 1] <= t:
            i += 1
        e, n = tgt[i]
        pub["raw"].append((e, n))
        pub["slew"].append(slew.update(e, n, t))
        tru_tick.append(tru_at(t))
        t += TICK_S

    def metrics(p):
        steps = [math.hypot(p[k][0] - p[k - 1][0], p[k][1] - p[k - 1][1])
                 for k in range(1, len(p))]
        errs = [math.hypot(a[0] - b[0], a[1] - b[1])
                for a, b in zip(p, tru_tick)]
        off = [(a[0] - b[0], a[1] - b[1]) for a, b in zip(p, tru_tick)]
        w = max(1, int(CORR_WIN / TICK_S))
        corr = [math.hypot(off[k][0] - off[k - w][0],
                           off[k][1] - off[k - w][1]) / (w * TICK_S)
                for k in range(w, len(off))]
        errs_s = sorted(errs)
        return dict(max_step=max(steps), max_corr=max(corr),
                    err_mean=sum(errs) / len(errs),
                    err_med=errs_s[len(errs_s) // 2], err_max=max(errs))

    return {k: metrics(v) for k, v in pub.items()}


def main():
    logs = sys.argv[1:] or DEFAULT_LOGS
    all_ok = unit_checks()

    for path in logs:
        with open(path) as f:
            d = json.load(f)
        lat0, lon0 = d["gps0"]
        cos_lat = d["cos_lat"]
        rows = d["rows"]
        print(f"\n{os.path.basename(path)} — {len(rows)} frames, "
              f"{rows[-1]['t'] - rows[0]['t']:.0f} s "
              f"(gate {d['params']['gate']:g})")
        print(f"  {'track':<14} {'mode':<6} {'max step':>9} {'corr vel':>9} "
              f"{'err mean':>9} {'err med':>8} {'err max':>8}")
        for prefix, label in (("new", "plan-B (new)"), ("old", "plan-A (old)")):
            m = simulate(rows, prefix, lat0, lon0, cos_lat)
            for mode in ("raw", "slew"):
                r = m[mode]
                print(f"  {label:<14} {mode:<6} {r['max_step']:>8.2f}m "
                      f"{r['max_corr']:>7.2f}m/s {r['err_mean']:>8.1f}m "
                      f"{r['err_med']:>7.1f}m {r['err_max']:>7.1f}m")
            ratio = m["raw"]["max_corr"] / max(m["slew"]["max_corr"], 1e-9)
            print(f"  {'':<14} → correction-velocity reduction ×{ratio:.1f}")

    print(f"\nUnit checks: {'ALL OK' if all_ok else 'FAILURES — see above'}")
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())
