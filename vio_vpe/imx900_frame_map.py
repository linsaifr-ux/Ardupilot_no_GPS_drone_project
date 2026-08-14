#!/usr/bin/env python3
"""Frame-based VPE map for the imx900 flights (survey43/44/45).

Same idea as survey33_frame_map: each map tile is one real video frame, made
north-up and put on a common ground sample distance, tagged with its centre's
ENU position from telemetry. Three things differ for this rig:

  * frames are UNDISTORTED first (k1 = -0.422 is large; uncorrected it bends a
    65 m tile by metres at the corners, and it is what broke VIO -- see
    session_2026-08-12_imx900_vio.md);
  * no 180 deg pre-rotation -- this camera sits in the canonical nadir
    orientation, unlike the older 1640x1232 rig meta.json still describes;
  * K_GSD is 5.587e-04, not 4.5824e-04.

The de-rotation sign is not assumed. `--check-rotation` matches pairs of frames
taken at nearly the same place but very different headings: if the convention
is right the fitted residual rotation between prepared tiles is ~0, and if the
sign is flipped it comes out at twice the heading difference.

    python3 imx900_frame_map.py --survey survey44 --check-rotation
"""
import argparse
import csv
import hashlib
import math
import pathlib
import pickle

import cv2 as cv
import numpy as np

import imx900_geo_common as G

SURVEY_ROOT = pathlib.Path("/home/frank/visual_localizer")
CACHE_DIR = pathlib.Path("/tmp/imx900_map")
COMMON_GSD = 0.10                      # m/px for every tile and query


def paths(survey):
    b = SURVEY_ROOT / survey
    return b / "video.mkv", b / "frame_times.csv", b / "telemetry.csv"


_undistort_maps = {}


def undistort(frame):
    """Remove lens distortion before anything else touches the geometry."""
    h, w = frame.shape[:2]
    key = (w, h)
    if key not in _undistort_maps:
        f = 1.0 / G.K_GSD * (w / 2048.0)      # focal scales with resolution
        K = np.array([[f, 0, w / 2.0], [0, f, h / 2.0], [0, 0, 1.0]])
        d = np.array(G.DIST_K, dtype=np.float64)
        m1, m2 = cv.initUndistortRectifyMap(K, d, None, K, (w, h), cv.CV_16SC2)
        _undistort_maps[key] = (m1, m2)
    m1, m2 = _undistort_maps[key]
    return cv.remap(frame, m1, m2, cv.INTER_LINEAR)


def prepare(frame_bgr, agl, hdg, common_gsd=COMMON_GSD, do_undistort=True):
    """One frame -> a north-up tile at common_gsd, black-padded."""
    f = undistort(frame_bgr) if do_undistort else frame_bgr
    if G.FRAME_ROT_DEG:
        f = cv.rotate(f, cv.ROTATE_180)
    scale = (G.K_GSD * agl) / common_gsd
    f = cv.resize(f, (max(1, int(f.shape[1] * scale)),
                      max(1, int(f.shape[0] * scale))),
                  interpolation=cv.INTER_AREA)
    h, w = f.shape[:2]
    # Black-padded canvas keeps the whole field of view; an inscribed crop
    # would throw away ~50% of it and leave consecutive tiles non-overlapping.
    d = int(math.hypot(w, h))
    M = cv.getRotationMatrix2D((w / 2, h / 2), -hdg, 1.0)
    M[0, 2] += (d - w) / 2
    M[1, 2] += (d - h) / 2
    return cv.warpAffine(f, M, (d, d), flags=cv.INTER_LINEAR,
                         borderMode=cv.BORDER_CONSTANT, borderValue=(0, 0, 0))


def load_telemetry(survey, min_agl):
    rows = []
    for r in csv.DictReader(open(paths(survey)[2])):
        try:
            agl = float(r["alt_agl"])
            if agl < min_agl:
                continue
            rows.append(dict(t=float(r["unix_time"]), lat=float(r["lat"]),
                             lon=float(r["lon"]), agl=agl,
                             hdg=float(r["heading_deg"])))
        except ValueError:
            pass
    return rows


def frame_index(survey):
    return [(int(r["frame_idx"]), float(r["unix_time"]))
            for r in csv.DictReader(open(paths(survey)[1]))]


def sample_frames(survey, every_s, min_agl, phase_s=0.0):
    tel = load_telemetry(survey, min_agl)
    if not tel:
        return []
    ft = frame_index(survey)
    ft_t = np.array([f[1] for f in ft])
    picks, last = [], -1e9
    t0 = tel[0]["t"] + phase_s
    for r in tel:
        if r["t"] < t0 or r["t"] - last < every_s:
            continue
        idx = ft[int(np.argmin(np.abs(ft_t - r["t"])))][0]
        e, n = G.enu(r["lat"], r["lon"])
        picks.append(dict(idx=idx, east=e, north=n, **r))
        last = r["t"]
    return picks


def build_tiles(survey, picks, tag, common_gsd=COMMON_GSD):
    """Decode + prepare each pick. Cached -- video decode is the slow part."""
    key = hashlib.sha1(
        f"imx900-v2:{survey}:{tag}:{common_gsd}:{G.K_GSD}:{G.DIST_K}:"
        f"{[p['idx'] for p in picks]}".encode()).hexdigest()[:16]
    cache = CACHE_DIR / f"tiles_{survey}_{tag}_{key}.pkl"
    if cache.exists():
        with open(cache, "rb") as fh:
            tiles = pickle.load(fh)
        print(f"[map:{survey}/{tag}] {len(tiles)} tiles from cache")
        return tiles
    want = {p["idx"]: p for p in picks}
    cap = cv.VideoCapture(str(paths(survey)[0]))
    tiles, idx = [], 0
    hi = max(want)
    while idx <= hi:
        ok, fr = cap.read()
        if not ok:
            break
        if idx in want:
            p = want[idx]
            tiles.append(dict(img=prepare(fr, p["agl"], p["hdg"], common_gsd),
                              east=p["east"], north=p["north"], agl=p["agl"],
                              hdg=p["hdg"], t=p["t"], idx=idx))
        idx += 1
    cap.release()
    CACHE_DIR.mkdir(parents=True, exist_ok=True)
    with open(cache, "wb") as fh:
        pickle.dump(tiles, fh)
    print(f"[map:{survey}/{tag}] {len(tiles)} tiles decoded -> cached")
    return tiles


def check_rotation(survey, min_agl=25.0):
    """Verify the de-rotation sign without assuming it.

    Pairs of frames at nearly the same ground position but very different
    headings must, once prepared, differ by ~0 deg. A flipped sign shows up as
    a residual of about twice the heading difference.
    """
    tel = load_telemetry(survey, min_agl)
    ft = frame_index(survey)
    ft_t = np.array([f[1] for f in ft])
    pts = []
    for r in tel[::10]:
        e, n = G.enu(r["lat"], r["lon"])
        pts.append((r, e, n))
    pairs = []
    for i in range(len(pts)):
        ri, ei, ni = pts[i]
        for j in range(i + 1, len(pts)):
            rj, ej, nj = pts[j]
            if math.hypot(ej - ei, nj - ni) > 12.0:
                continue
            dh = (rj["hdg"] - ri["hdg"] + 180) % 360 - 180
            if abs(dh) < 50:
                continue
            pairs.append((ri, rj, dh))
    if not pairs:
        print("[check] no same-place/different-heading pairs in this flight")
        return
    pairs = pairs[:40]
    print(f"[check] {len(pairs)} same-place pairs with >50 deg heading change")
    cap = cv.VideoCapture(str(paths(survey)[0]))
    need = {}
    for ri, rj, _ in pairs:
        for r in (ri, rj):
            need[ft[int(np.argmin(np.abs(ft_t - r["t"])))][0]] = r
    frames, idx, hi = {}, 0, max(need)
    while idx <= hi:
        ok, fr = cap.read()
        if not ok:
            break
        if idx in need:
            r = need[idx]
            frames[idx] = prepare(fr, r["agl"], r["hdg"])
        idx += 1
    cap.release()
    orb = cv.ORB_create(nfeatures=4000, fastThreshold=7)
    bf = cv.BFMatcher(cv.NORM_HAMMING, crossCheck=True)
    res = []
    for ri, rj, dh in pairs:
        ii = ft[int(np.argmin(np.abs(ft_t - ri["t"])))][0]
        jj = ft[int(np.argmin(np.abs(ft_t - rj["t"])))][0]
        if ii not in frames or jj not in frames:
            continue
        a = cv.cvtColor(frames[ii], cv.COLOR_BGR2GRAY)
        b = cv.cvtColor(frames[jj], cv.COLOR_BGR2GRAY)
        k1, d1 = orb.detectAndCompute(a, None)
        k2, d2 = orb.detectAndCompute(b, None)
        if d1 is None or d2 is None:
            continue
        mm = bf.match(d1, d2)
        if len(mm) < 25:
            continue
        p1 = np.float32([k1[m.queryIdx].pt for m in mm])
        p2 = np.float32([k2[m.trainIdx].pt for m in mm])
        M, inl = cv.estimateAffinePartial2D(p1, p2, method=cv.RANSAC,
                                            ransacReprojThreshold=4.0)
        if M is None or int(inl.sum()) < 20:
            continue
        rot = math.degrees(math.atan2(M[1, 0], M[0, 0]))
        res.append((dh, rot, int(inl.sum())))
    if not res:
        print("[check] no pair matched -- cannot verify the sign this way")
        return
    dh = np.array([r[0] for r in res])
    rot = np.array([r[1] for r in res])
    print(f"[check] {len(res)} pairs matched")
    print(f"[check] residual rotation between north-up tiles: "
          f"median {np.median(np.abs(rot)):.1f} deg "
          f"(heading differences were {np.abs(dh).min():.0f}-{np.abs(dh).max():.0f} deg)")
    flipped = np.median(np.abs(rot - 2 * dh))
    print(f"[check] if the sign were flipped we would instead see ~2*dh; "
          f"residual against that model is {flipped:.1f} deg")
    ok = np.median(np.abs(rot)) < 15.0
    print(f"[check] {'PASS -- de-rotation sign is correct' if ok else 'FAIL -- de-rotation looks wrong'}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--survey", default="survey44")
    ap.add_argument("--every", type=float, default=3.0, help="seconds between tiles")
    ap.add_argument("--min-agl", type=float, default=25.0)
    ap.add_argument("--check-rotation", action="store_true")
    args = ap.parse_args()

    if args.check_rotation:
        check_rotation(args.survey, args.min_agl)
        return
    picks = sample_frames(args.survey, args.every, args.min_agl)
    tiles = build_tiles(args.survey, picks, f"db{args.every:g}")
    e = [t["east"] for t in tiles]
    n = [t["north"] for t in tiles]
    print(f"tiles {len(tiles)}, size {tiles[0]['img'].shape}, "
          f"footprint {tiles[0]['img'].shape[1]*COMMON_GSD:.1f} m")
    print(f"coverage: E {min(e):.0f}..{max(e):.0f}  N {min(n):.0f}..{max(n):.0f} m")


if __name__ == "__main__":
    main()
