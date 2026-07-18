#!/usr/bin/env python3
"""
Synthesize an "OpenVINS-grade VIO + AnyLoc" localizer error track for the
SITL closed-loop experiment (control/test_vpe_slew_sitl.py).

Real OpenVINS cannot be run against this project's data: SITL renders no
camera imagery, and the survey13 recording has no high-rate IMU (telemetry is
5 Hz lat/lon/alt/heading — OpenVINS needs ~200 Hz IMU time-synced to the
camera).  This generator instead swaps the odometry *error model* under the
same recorded AnyLoc candidate stream:

  - Odometry drift: ~1 % of distance travelled (published OpenVINS-class
    accuracy), direction a slow random walk, plus 0.3 m jitter — smooth, no
    jump structure, in place of the LK-VO drift of the real plan-B track.
  - AnyLoc corrections: the *recorded* candidate stream from
    anyloc/logs/survey13_vo_fusion.json (constrained match score + error per
    step), gated plan-B-style: score ≥ 0.32, and a jump gate tightened to
    10 m + 0.2 m/s — affordable only because VIO drift rate is known-small;
    accepted candidates blend 0.4.

Output rows mirror anyloc/logs/survey13_jumpgate_*.json ("new" track = the
fused estimate) so the SITL harness loads it as scenario "vio".

Usage:  python3 control/gen_vio_track.py
Output: anyloc/logs/survey13_vio_synth.json
"""

import json
import math
import os
import random

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LOGS = os.path.join(ROOT, "anyloc", "logs")
BASE = os.path.join(LOGS, "survey13_jumpgate_replay.json")   # truth timeline
CAND = os.path.join(LOGS, "survey13_vo_fusion.json")         # AnyLoc stream
OUT  = os.path.join(LOGS, "survey13_vio_synth.json")

M_PER_DEG = 111_320.0

DRIFT_FRAC   = 0.01     # VIO drift: 1 % of distance travelled
JITTER_M     = 0.3      # high-frequency VIO noise
SCORE_GATE   = 0.32     # calibrated AnyLoc score gate (plan-B)
JUMP_BASE_M  = 10.0     # tight jump gate — affordable with VIO drift rates
JUMP_RATE    = 0.2      # m/s allowance growth since last accept
BLEND        = 0.4


def main():
    rng = random.Random(13)
    with open(BASE) as f:
        base = json.load(f)
    with open(CAND) as f:
        steps = {s["frame_idx"]: s for s in json.load(f)["anyloc_steps"]}

    lat0, lon0 = base["gps0"]
    cos_lat = base["cos_lat"]
    rows_in = base["rows"]

    def to_en(lat, lon):
        return ((lon - lon0) * M_PER_DEG * cos_lat, (lat - lat0) * M_PER_DEG)

    def to_ll(e, n):
        return (lat0 + n / M_PER_DEG, lon0 + e / (M_PER_DEG * cos_lat))

    drift_e = drift_n = 0.0
    drift_th = rng.uniform(0, 2 * math.pi)   # drift direction random walk
    cand_th  = rng.uniform(0, 2 * math.pi)   # AnyLoc error direction r-walk
    corr_e = corr_n = 0.0                    # accumulated AnyLoc blend pulls
    prev_en = None
    last_accept_t = rows_in[0]["t"]
    n_acc = n_rej = n_jrej = 0

    rows_out = []
    for r in rows_in:
        te, tn = to_en(r["true_lat"], r["true_lon"])
        if prev_en is not None:
            d = math.hypot(te - prev_en[0], tn - prev_en[1])
            drift_th += rng.gauss(0.0, 0.02)
            drift_e += DRIFT_FRAC * d * math.cos(drift_th)
            drift_n += DRIFT_FRAC * d * math.sin(drift_th)
        prev_en = (te, tn)

        est_e = te + drift_e + corr_e + rng.gauss(0.0, JITTER_M)
        est_n = tn + drift_n + corr_n + rng.gauss(0.0, JITTER_M)

        verdict = None
        step = steps.get(r["frame_idx"])
        if step is not None:
            score = step["constA_score"]
            cand_th += rng.gauss(0.0, 0.3)
            ce = te + step["constA_err_m"] * math.cos(cand_th)
            cn = tn + step["constA_err_m"] * math.sin(cand_th)
            if score < SCORE_GATE:
                verdict = "REJ"; n_rej += 1
            else:
                jump = math.hypot(ce - est_e, cn - est_n)
                max_jump = JUMP_BASE_M + JUMP_RATE * (r["t"] - last_accept_t)
                if jump <= max_jump:
                    corr_e += BLEND * (ce - est_e)
                    corr_n += BLEND * (cn - est_n)
                    last_accept_t = r["t"]
                    verdict = "ACC"; n_acc += 1
                else:
                    verdict = "JREJ"; n_jrej += 1

        est_lat, est_lon = to_ll(est_e, est_n)
        rows_out.append(dict(
            frame_idx=r["frame_idx"], t=r["t"], agl=r["agl"],
            true_lat=r["true_lat"], true_lon=r["true_lon"],
            new_lat=est_lat, new_lon=est_lon,
            old_lat=est_lat, old_lon=est_lon,     # single track
            verdict_new=verdict, score=(step or {}).get("constA_score")))

    errs = sorted(
        math.hypot(*(a - b for a, b in
                     zip(to_en(r["new_lat"], r["new_lon"]),
                         to_en(r["true_lat"], r["true_lon"]))))
        for r in rows_out)
    out = dict(
        cos_lat=cos_lat, gps0=[lat0, lon0],
        params=dict(gate=SCORE_GATE, jump_base=JUMP_BASE_M,
                    drift_rate=JUMP_RATE, blend=BLEND,
                    drift_frac=DRIFT_FRAC, model="synthetic OpenVINS-grade"),
        summary=dict(counters=dict(n_acc=n_acc, n_rej=n_rej, n_jrej=n_jrej),
                     err_mean=sum(errs) / len(errs),
                     err_med=errs[len(errs) // 2], err_max=errs[-1]),
        rows=rows_out)
    with open(OUT, "w") as f:
        json.dump(out, f)
    print(f"VIO-model track → {OUT}")
    print(f"  AnyLoc: {n_acc} acc / {n_rej} score-rej / {n_jrej} jump-rej")
    print(f"  err vs truth: mean {out['summary']['err_mean']:.1f} m  "
          f"median {out['summary']['err_med']:.1f} m  "
          f"max {out['summary']['err_max']:.1f} m")


if __name__ == "__main__":
    main()
