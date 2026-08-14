#!/usr/bin/env python3
"""VPE fixes for the imx900 flights: SuperPoint + LightGlue against a frame map.

Builds the database from one flight and localizes another against it. Both the
map tiles and the queries are made north-up at a common ground sample distance
by imx900_frame_map.prepare(), so a match is a pure 2D translation problem and
the recovered offset converts straight to metres.

Default is CROSS-FLIGHT (survey44 builds, survey45 queries), which is the only
protocol that measures anything: a same-flight split lets a query match its own
frame and reverses which method looks better -- see the survey33 session.

    python3 vpe_localize_imx900.py --db-survey survey44 --q-survey survey45
"""
import argparse
import hashlib
import json
import math
import pathlib
import pickle

import cv2 as cv
import numpy as np
import torch

import imx900_frame_map as FM
import imx900_geo_common as G

CACHE = pathlib.Path("/tmp/imx900_map")
MAX_KP = 1024
MATCH_THRESHOLD = 0.5
MIN_INLIERS = 18
MIN_INLIER_RATIO = 0.15


def to_tensor(img_bgr, dev):
    g = cv.cvtColor(img_bgr, cv.COLOR_BGR2GRAY).astype(np.float32) / 255.0
    return torch.from_numpy(g)[None, None].to(dev)


class VpeLocalizer:
    def __init__(self, db_survey, every_s, min_agl, device="cuda",
                 search_radius_m=None):
        from lightglue import LightGlue, SuperPoint
        self.dev = device if torch.cuda.is_available() else "cpu"
        self.radius = search_radius_m
        self.extractor = SuperPoint(max_num_keypoints=MAX_KP).eval().to(self.dev)
        self.matcher = LightGlue(features="superpoint",
                                 filter_threshold=MATCH_THRESHOLD).eval().to(self.dev)
        picks = FM.sample_frames(db_survey, every_s, min_agl)
        self.tiles = FM.build_tiles(db_survey, picks, f"db{every_s:g}a{min_agl:g}")
        if not self.tiles:
            raise SystemExit("empty database")
        self.feats = self._features(db_survey, every_s, min_agl)
        self.centres = np.array([[t["east"], t["north"]] for t in self.tiles])
        e, n = self.centres[:, 0], self.centres[:, 1]
        print(f"[vpe] database {len(self.tiles)} tiles from {db_survey}, "
              f"coverage E {e.min():.0f}..{e.max():.0f} N {n.min():.0f}..{n.max():.0f} m")

    def _features(self, survey, every_s, min_agl):
        key = hashlib.sha1(f"sp-v1:{survey}:{every_s}:{min_agl}:{MAX_KP}:"
                           f"{G.K_GSD}:{G.DIST_K}".encode()).hexdigest()[:16]
        path = CACHE / f"spfeat_{survey}_{key}.pkl"
        if path.exists():
            with open(path, "rb") as fh:
                raw = pickle.load(fh)
            print(f"[vpe] {len(raw)} tile descriptors from cache")
            return [{k: torch.from_numpy(v).to(self.dev) for k, v in d.items()}
                    for d in raw]
        feats = []
        with torch.no_grad():
            for t in self.tiles:
                feats.append(self.extractor.extract(to_tensor(t["img"], self.dev)))
        CACHE.mkdir(parents=True, exist_ok=True)
        with open(path, "wb") as fh:
            pickle.dump([{k: v.cpu().numpy() for k, v in f.items()} for f in feats], fh)
        print(f"[vpe] extracted {len(feats)} tile descriptors -> cached")
        return feats

    def localize(self, query_bgr, agl, hdg, prior=None):
        """Return a dict with the ENU fix, or None if nothing matched."""
        from lightglue.utils import rbd
        q = FM.prepare(query_bgr, agl, hdg)
        with torch.no_grad():
            fq = self.extractor.extract(to_tensor(q, self.dev))
        qh, qw = q.shape[:2]
        qc = np.array([qw / 2.0, qh / 2.0])

        cand = range(len(self.tiles))
        if prior is not None and self.radius:
            d = np.linalg.norm(self.centres - np.asarray(prior), axis=1)
            cand = np.where(d <= self.radius)[0]
            if len(cand) == 0:
                cand = [int(np.argmin(d))]

        best = None
        for i in cand:
            with torch.no_grad():
                out = self.matcher({"image0": fq, "image1": self.feats[i]})
            f0, f1, m = rbd(fq), rbd(self.feats[i]), rbd(out)
            idx = m["matches"]
            if idx.shape[0] < MIN_INLIERS:
                continue
            p0 = f0["keypoints"][idx[:, 0]].cpu().numpy()
            p1 = f1["keypoints"][idx[:, 1]].cpu().numpy()
            # Both images are already north-up at the same scale, so the true
            # transform is a translation; a full similarity is fitted anyway so
            # that a wrong match shows up as absurd scale/rotation.
            M, inl = cv.estimateAffinePartial2D(p0, p1, method=cv.RANSAC,
                                                ransacReprojThreshold=4.0)
            if M is None:
                continue
            n_inl = int(inl.sum())
            ratio = n_inl / max(len(idx), 1)
            if n_inl < MIN_INLIERS or ratio < MIN_INLIER_RATIO:
                continue
            scale = math.hypot(M[0, 0], M[1, 0])
            rot = abs(math.degrees(math.atan2(M[1, 0], M[0, 0])))
            if abs(scale - 1.0) > 0.25 or min(rot, 360 - rot) > 25.0:
                continue
            if best is None or n_inl > best["n_inl"]:
                t = self.tiles[i]
                th, tw = t["img"].shape[:2]
                p = M[:, :2] @ qc + M[:, 2]        # query centre in tile pixels
                dx = p[0] - tw / 2.0
                dy = p[1] - th / 2.0
                best = dict(tile=i, n_inl=n_inl, ratio=float(ratio),
                            scale=float(scale), rot=float(rot),
                            east=float(t["east"] + dx * FM.COMMON_GSD),
                            north=float(t["north"] - dy * FM.COMMON_GSD),
                            tile_east=float(t["east"]), tile_north=float(t["north"]),
                            n_cand=len(cand))
        return best


def export_map(loc, path):
    """Write a self-contained map file: descriptors + tile geometry, no imagery.

    Matching never touches the tile pixels after extraction -- it needs the
    SuperPoint keypoints/descriptors plus each tile's centre and pixel size.
    Dropping the images takes the survey47 map from ~1.1 GB of cache to a file
    that fits comfortably on a companion computer, and removes the need to ship
    the source video or rebuild the map onboard.
    """
    blob = dict(
        gsd=FM.COMMON_GSD,
        k_gsd=G.K_GSD, dist_k=list(G.DIST_K),
        lat0=G.LAT0, lon0=G.LON0,
        tiles=[dict(east=t["east"], north=t["north"],
                    h=int(t["img"].shape[0]), w=int(t["img"].shape[1]))
               for t in loc.tiles],
        feats=[{k: v.cpu().numpy() for k, v in f.items()} for f in loc.feats],
    )
    path = pathlib.Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "wb") as fh:
        pickle.dump(blob, fh, protocol=4)
    print(f"[vpe] exported {len(blob['tiles'])} tiles -> {path} "
          f"({path.stat().st_size/1e6:.1f} MB, no imagery)")


class PrebuiltVpeLocalizer(VpeLocalizer):
    """Localizer backed by an exported map file -- no video, no map build.

    This is the deployment path: build the map on the desktop, ship one file.
    """

    def __init__(self, map_path, device="cuda", search_radius_m=None):
        from lightglue import LightGlue, SuperPoint
        self.dev = device if torch.cuda.is_available() else "cpu"
        self.radius = search_radius_m
        self.extractor = SuperPoint(max_num_keypoints=MAX_KP).eval().to(self.dev)
        self.matcher = LightGlue(features="superpoint",
                                 filter_threshold=MATCH_THRESHOLD).eval().to(self.dev)
        with open(map_path, "rb") as fh:
            blob = pickle.load(fh)
        if abs(blob["k_gsd"] - G.K_GSD) > 1e-9 or list(blob["dist_k"]) != list(G.DIST_K):
            raise SystemExit(
                "map was built with different camera constants than "
                "imx900_geo_common currently has -- rebuild it, or the fixes "
                "will be silently mis-scaled")
        # tiles carry geometry only; .img is never read after feature extraction
        self.tiles = [dict(east=t["east"], north=t["north"],
                           img=_ShapeOnly((t["h"], t["w"], 3)))
                      for t in blob["tiles"]]
        self.feats = [{k: torch.from_numpy(v).to(self.dev) for k, v in f.items()}
                      for f in blob["feats"]]
        self.centres = np.array([[t["east"], t["north"]] for t in self.tiles])
        e, n = self.centres[:, 0], self.centres[:, 1]
        print(f"[vpe] prebuilt map {map_path}: {len(self.tiles)} tiles, "
              f"E {e.min():.0f}..{e.max():.0f} N {n.min():.0f}..{n.max():.0f} m")


class _ShapeOnly:
    """Stands in for a tile image so only .shape is available -- anything that
    tries to read pixels from a prebuilt map fails loudly instead of silently."""

    def __init__(self, shape):
        self.shape = shape


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--db-survey", default="survey44")
    ap.add_argument("--q-survey", default="survey45")
    ap.add_argument("--db-every", type=float, default=3.0)
    ap.add_argument("--q-every", type=float, default=2.0)
    ap.add_argument("--min-agl", type=float, default=25.0)
    ap.add_argument("--search-radius", type=float, default=None,
                    help="metres around a prior; omit to match every tile")
    ap.add_argument("--out", default="vio_out/vpe_s44db_s45q.json")
    ap.add_argument("--export-map", default=None,
                    help="write the database to a self-contained file "
                         "(descriptors + geometry, no imagery) and exit")
    args = ap.parse_args()

    loc = VpeLocalizer(args.db_survey, args.db_every, args.min_agl,
                       search_radius_m=args.search_radius)
    if args.export_map:
        export_map(loc, args.export_map)
        return
    picks = FM.sample_frames(args.q_survey, args.q_every, args.min_agl)
    print(f"[vpe] {len(picks)} query frames from {args.q_survey}")

    cap = cv.VideoCapture(str(FM.paths(args.q_survey)[0]))
    want = {p["idx"]: p for p in picks}
    hi = max(want)
    rows, idx = [], 0
    n_ok = 0
    while idx <= hi:
        ok, fr = cap.read()
        if not ok:
            break
        if idx in want:
            p = want[idx]
            r = loc.localize(fr, p["agl"], p["hdg"])
            row = dict(t=p["t"], idx=idx, agl=p["agl"], hdg=p["hdg"],
                       gps_e=p["east"], gps_n=p["north"], fix=r)
            if r:
                row["err_m"] = float(math.hypot(r["east"] - p["east"],
                                                r["north"] - p["north"]))
                n_ok += 1
            rows.append(row)
            if len(rows) % 10 == 0:
                got = [x["err_m"] for x in rows if x.get("err_m") is not None]
                print(f"[vpe] {len(rows)}/{len(picks)} queries, {n_ok} fixed, "
                      f"median err {np.median(got):.1f} m" if got else
                      f"[vpe] {len(rows)}/{len(picks)} queries, 0 fixed")
        idx += 1
    cap.release()

    errs = np.array([r["err_m"] for r in rows if r.get("err_m") is not None])
    print(f"\n=== VPE: {args.db_survey} map -> {args.q_survey} queries ===")
    print(f"  fix rate {len(errs)}/{len(rows)} ({100*len(errs)/max(len(rows),1):.0f}%)")
    if len(errs):
        print(f"  error  mean {errs.mean():.1f} m  median {np.median(errs):.1f} m  "
              f"p90 {np.percentile(errs,90):.1f} m  max {errs.max():.1f} m")
        print(f"  under 5 m: {100*np.mean(errs<5):.0f}%   "
              f"under 15 m: {100*np.mean(errs<15):.0f}%")
    out = pathlib.Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    json.dump(dict(db_survey=args.db_survey, q_survey=args.q_survey,
                   db_every=args.db_every, q_every=args.q_every,
                   n_tiles=len(loc.tiles), rows=rows), open(out, "w"), indent=1)
    print("  wrote", out)


if __name__ == "__main__":
    main()
