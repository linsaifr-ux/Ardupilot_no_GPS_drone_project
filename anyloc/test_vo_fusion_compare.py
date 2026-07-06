#!/usr/bin/env python3
"""
VO-fusion scheme comparison on a real recorded flight (video.mkv + telemetry.csv).

Compares, in ONE pass over the video, all starting from the GPS position at
chain start (mirrors the real SRC1→SRC2 handover — EKF is GPS truth when the
localizer takes over):

  A  anchor-chain (production ros2_node.py): constrained AnyLoc every
     --interval-frames re-anchors, VO fills between, VO reset on anchor.
  B  VO-primary: VO integrates continuously; constrained AnyLoc runs every
     --interval-frames but only resets the position when its cosine score
     ≥ threshold (one state per threshold in --thresholds).
  VO pure VO: never accepts AnyLoc — drift floor/reference.

Also logs, per AnyLoc step, the global (whole-DB) match + score and the
constrained match + score for every scheme, so score-vs-error correlation can
be analysed afterwards to pick a real threshold.

VO runs on every video frame (30 fps) — an upper bound on live VO quality,
where AnyLoc inference latency drops frames. Use --vo-stride to thin it.

Usage:
    /home/jetson/venv/anyloc/bin/python3 anyloc/test_vo_fusion_compare.py \\
        field_data/survey13 --db-dir anyloc/database_test20_vits14 \\
        --output anyloc/logs/survey13_vo_fusion.json
"""

import argparse
import csv
import json
import math
import os
import sys
import time

import cv2
import torch
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.dirname(HERE))
from test_accuracy_real_video import euclidean_m, _stats
from vo_refiner import VORefiner

M_PER_DEG = 111_320.0


def load_survey_full(survey_dir):
    """load_survey + heading_deg (needed for VO world-frame rotation)."""
    with open(os.path.join(survey_dir, 'meta.json')) as f:
        meta = json.load(f)
    frame_times = []
    with open(os.path.join(survey_dir, 'frame_times.csv')) as f:
        for row in csv.DictReader(f):
            frame_times.append((int(row['frame_idx']), float(row['unix_time'])))
    telemetry = []
    with open(os.path.join(survey_dir, 'telemetry.csv')) as f:
        for row in csv.DictReader(f):
            telemetry.append(dict(
                unix_time=float(row['unix_time']),
                lat=float(row['lat']), lon=float(row['lon']),
                alt_agl=float(row['alt_agl']),
                heading_deg=float(row['heading_deg']),
            ))
    frame_times.sort(key=lambda x: x[0])
    telemetry.sort(key=lambda t: t['unix_time'])
    return meta, frame_times, telemetry


def nearest_tel(telemetry, t, hint=0):
    """Two-pointer nearest telemetry sample; telemetry sorted by time."""
    i = hint
    n = len(telemetry)
    while i + 1 < n and abs(telemetry[i + 1]['unix_time'] - t) <= \
            abs(telemetry[i]['unix_time'] - t):
        i += 1
    return telemetry[i], i


def constrained_match(loc, desc, clat, clon, radius_m, cos_lat):
    """Mirror of AnyLocLocalizer.localize()'s constrained branch, reusing a
    precomputed VLAD descriptor so one DINOv2 forward serves all schemes."""
    dlat = (loc.lats - clat) * M_PER_DEG
    dlon = (loc.lons - clon) * M_PER_DEG * cos_lat
    in_range = ((dlat ** 2 + dlon ** 2) <= radius_m ** 2) \
        .nonzero(as_tuple=False).squeeze(1)
    if len(in_range) == 0:
        in_range = torch.arange(len(loc.lats))
    sims = loc.vlads[in_range] @ desc
    best = int(sims.argmax())
    idx = int(in_range[best])
    return float(loc.lats[idx]), float(loc.lons[idx]), float(sims[best]), idx


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('survey_dir')
    ap.add_argument('--db-dir', required=True)
    ap.add_argument('--start-agl', type=float, default=65.0)
    ap.add_argument('--start-tol', type=float, default=0.5)
    ap.add_argument('--min-agl', type=float, default=60.0)
    ap.add_argument('--interval-frames', type=int, default=10,
                    help="frames between AnyLoc calls (default 10 = ros2_node's "
                         'ANYLOC_INTERVAL)')
    ap.add_argument('--radius-m', type=float, default=200.0)
    ap.add_argument('--thresholds', default='0.4,0.5,0.6,0.7,0.8',
                    help='comma-separated score gates for scheme B')
    ap.add_argument('--vo-stride', type=int, default=1,
                    help='run VO every Nth frame (default 1 = every frame)')
    ap.add_argument('--max-frames', type=int, default=20000)
    ap.add_argument('--output', default='')
    args = ap.parse_args()

    thresholds = [float(t) for t in args.thresholds.split(',')]

    from anyloc.localizer import AnyLocLocalizer

    print(f"\n{'='*78}")
    print(f"  VO-Fusion Comparison  (A: anchor-chain | B: VO-primary+score gate | VO)")
    print(f"  Survey : {args.survey_dir}  |  DB : {args.db_dir}")
    print(f"  Interval : {args.interval_frames} f  |  Radius : {args.radius_m} m"
          f"  |  Thresholds : {thresholds}")
    print(f"{'='*78}\n")

    print('[1/3] Loading survey data …')
    meta, frame_times, telemetry = load_survey_full(args.survey_dir)
    frame_by_idx = dict(frame_times)

    start_t = next((t for t in telemetry
                    if t['alt_agl'] >= args.start_agl - args.start_tol), None)
    if start_t is None:
        sys.exit(f'No telemetry sample reaches {args.start_agl - args.start_tol} m AGL')
    start_frame = min(frame_times, key=lambda ft: abs(ft[1] - start_t['unix_time']))[0]
    cos_lat = math.cos(math.radians(start_t['lat']))
    print(f"  {len(frame_times)} frames, {len(telemetry)} telemetry samples; "
          f"chain starts frame {start_frame} @ {start_t['alt_agl']:.1f} m AGL")

    print('[2/3] Loading AnyLoc database and DINOv2 model …')
    loc = AnyLocLocalizer(args.db_dir)
    vo = VORefiner(cam_w=meta['width'], cam_h=meta['height'])

    # GPS start — every scheme begins at truth, like the real source switch
    gps0 = (start_t['lat'], start_t['lon'])
    stateA = dict(anchor=gps0, accum=(0.0, 0.0))          # anchor + VO accum
    statesB = {th: dict(pos=gps0, n_accept=0) for th in thresholds}
    vo_pos = gps0

    results = []
    errsA, errsVO = [], []
    errsB = {th: [] for th in thresholds}
    anyloc_rows = []      # per-AnyLoc-step score/error rows for threshold analysis
    tel_hint = 0
    n_anyloc = 0

    print('[3/3] Replaying …\n')
    hdr = (f"  {'frame':>7} {'AGL':>5} {'errA':>7} {'errVO':>7} "
           + ' '.join(f'errB@{th:g}'.rjust(8) for th in thresholds)
           + f" {'g_score':>7} {'c_score':>7}")
    print(hdr)
    print('  ' + '-' * (len(hdr) - 2))

    cap = cv2.VideoCapture(os.path.join(args.survey_dir, 'video.mkv'))
    cap.set(cv2.CAP_PROP_POS_FRAMES, start_frame)
    cur = start_frame
    n_done = 0

    while n_done < args.max_frames:
        ok, frame_bgr = cap.read()
        if not ok:
            print(f'  [!] video ended at frame {cur}')
            break
        ftime = frame_by_idx.get(cur)
        if ftime is None:
            cur += 1
            continue
        tel, tel_hint = nearest_tel(telemetry, ftime, tel_hint)
        if tel['alt_agl'] < args.min_agl:
            print(f"  [·] AGL {tel['alt_agl']:.1f} m < {args.min_agl} m — chain ends "
                  f'at frame {cur}')
            break

        pil_img = Image.fromarray(cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB))
        agl, heading = tel['alt_agl'], tel['heading_deg']
        truth = (tel['lat'], tel['lon'])

        # ── shared VO delta (heading = compass bearing, VORefiner convention) ──
        if n_done % args.vo_stride == 0:
            dlat, dlon, n_pts = vo.update(pil_img, agl, heading)
        else:
            dlat = dlon = 0.0; n_pts = -1
        stateA['accum'] = (stateA['accum'][0] + dlat, stateA['accum'][1] + dlon)
        for st in statesB.values():
            st['pos'] = (st['pos'][0] + dlat, st['pos'][1] + dlon)
        vo_pos = (vo_pos[0] + dlat, vo_pos[1] + dlon)

        run_anyloc = (n_done % args.interval_frames == 0)
        g_score = c_scoreA = None
        if run_anyloc:
            n_anyloc += 1
            feats = loc._patch_features(pil_img)
            desc = loc._vlad(feats)

            # global match (analysis only — no scheme uses it)
            g_scores, g_idxs = loc._index.search(desc.unsqueeze(0).numpy(), 1)
            g_idx = int(g_idxs[0, 0]); g_score = float(g_scores[0, 0])
            g_lat, g_lon = float(loc.lats[g_idx]), float(loc.lons[g_idx])

            # A: constrained around anchor+accum → always re-anchor, reset VO accum
            cA = (stateA['anchor'][0] + stateA['accum'][0],
                  stateA['anchor'][1] + stateA['accum'][1])
            a_lat, a_lon, c_scoreA, _ = constrained_match(
                loc, desc, cA[0], cA[1], args.radius_m, cos_lat)
            stateA['anchor'] = (a_lat, a_lon)
            stateA['accum'] = (0.0, 0.0)

            # B: constrained around each state's VO position → accept iff score ≥ th
            b_step = {}
            for th, st in statesB.items():
                b_lat, b_lon, b_score, _ = constrained_match(
                    loc, desc, st['pos'][0], st['pos'][1], args.radius_m, cos_lat)
                accepted = b_score >= th
                if accepted:
                    st['pos'] = (b_lat, b_lon)
                    st['n_accept'] += 1
                b_step[th] = dict(score=b_score, accepted=accepted,
                                  err_m=euclidean_m(truth[0], truth[1],
                                                    b_lat, b_lon, cos_lat))

            anyloc_rows.append(dict(
                frame_idx=cur, agl_m=agl,
                global_score=g_score,
                global_err_m=euclidean_m(truth[0], truth[1], g_lat, g_lon, cos_lat),
                constA_score=c_scoreA,
                constA_err_m=euclidean_m(truth[0], truth[1], a_lat, a_lon, cos_lat),
                b_candidates={str(th): b_step[th] for th in thresholds},
            ))

        posA = (stateA['anchor'][0] + stateA['accum'][0],
                stateA['anchor'][1] + stateA['accum'][1])
        eA = euclidean_m(truth[0], truth[1], posA[0], posA[1], cos_lat)
        eVO = euclidean_m(truth[0], truth[1], vo_pos[0], vo_pos[1], cos_lat)
        eB = {th: euclidean_m(truth[0], truth[1], st['pos'][0], st['pos'][1], cos_lat)
              for th, st in statesB.items()}
        errsA.append(eA); errsVO.append(eVO)
        for th in thresholds:
            errsB[th].append(eB[th])

        results.append(dict(frame_idx=cur, agl_m=round(agl, 1),
                            true_lat=truth[0], true_lon=truth[1],
                            errA_m=round(eA, 1), errVO_m=round(eVO, 1),
                            errB_m={str(th): round(eB[th], 1) for th in thresholds},
                            n_vo_pts=n_pts, anyloc=run_anyloc))

        if run_anyloc:
            print(f"  {cur:>7} {agl:>5.1f} {eA:>7.1f} {eVO:>7.1f} "
                  + ' '.join(f'{eB[th]:>8.1f}' for th in thresholds)
                  + f" {g_score:>7.3f} {c_scoreA:>7.3f}")
        cur += 1
        n_done += 1
    cap.release()

    if not results:
        print('\nNo frames processed.')
        return

    span_s = n_done / meta['fps']
    print(f"\n{'='*78}")
    print(f'  Results — {n_done} frames (~{span_s:.1f} s), {n_anyloc} AnyLoc steps, '
          f'GPS-seeded start')
    print(f"{'='*78}")
    print(f"  {'Scheme':<26} {'mean':>8} {'median':>8} {'p95':>8} {'max':>8} {'accept':>8}")
    print(f"  {'-'*26} {'-'*8} {'-'*8} {'-'*8} {'-'*8} {'-'*8}")

    def p95(v):
        s = sorted(v); return s[min(len(s) - 1, int(len(s) * 0.95))]

    def row(name, errs, accept=''):
        st = _stats(errs)
        print(f"  {name:<26} {st['mean']:>8.1f} {st['median']:>8.1f} "
              f"{p95(errs):>8.1f} {st['max']:>8.1f} {accept:>8}")

    row('A anchor-chain (prod)', errsA, f'{n_anyloc}/{n_anyloc}')
    row('VO pure (drift floor)', errsVO, '0')
    for th in thresholds:
        row(f'B VO+AnyLoc@{th:g}', errsB[th],
            f"{statesB[th]['n_accept']}/{n_anyloc}")

    # score-vs-error summary for the B candidates (all offered matches)
    all_cand = [(c['score'], c['err_m'])
                for r in anyloc_rows for c in r['b_candidates'].values()]
    good = [s for s, e in all_cand if e < 50]
    bad = [s for s, e in all_cand if e >= 50]
    print(f"\n  AnyLoc candidate score stats (constrained matches, all B states):")
    if good:
        print(f"    err<50 m  (n={len(good):>4}): score mean {sum(good)/len(good):.3f}"
              f"  min {min(good):.3f}  max {max(good):.3f}")
    if bad:
        print(f"    err≥50 m  (n={len(bad):>4}): score mean {sum(bad)/len(bad):.3f}"
              f"  min {min(bad):.3f}  max {max(bad):.3f}")
    print(f"{'='*78}\n")

    if args.output:
        report = dict(
            config=vars(args),
            summary=dict(
                n_frames=n_done, n_anyloc=n_anyloc, span_s=span_s,
                A=_stats(errsA), VO=_stats(errsVO),
                B={str(th): dict(**_stats(errsB[th]),
                                 n_accept=statesB[th]['n_accept'])
                   for th in thresholds},
            ),
            anyloc_steps=anyloc_rows,
            frames=results,
        )
        os.makedirs(os.path.dirname(args.output) or '.', exist_ok=True)
        with open(args.output, 'w') as f:
            json.dump(report, f, indent=1)
        print(f'Results saved → {args.output}')


if __name__ == '__main__':
    main()
