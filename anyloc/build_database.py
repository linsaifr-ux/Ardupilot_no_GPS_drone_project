#!/usr/bin/env python3
"""
Build AnyLoc geo-tagged image database from NLSC PHOTO2 satellite orthophoto.

Creates anyloc/database/ with:
  database.pt    — torch save: lats, lons, alts, vlads (N×D), codebook (k×768)
  db_images/     — satellite crop JPEGs

Run once before starting the localizer:
    conda run -n isaac_sim_test python anyloc/build_database.py

Options:
    --grid-step 50    grid spacing in metres (default 50)
    --agl-min 65      minimum AGL in metres (default 65)
    --agl-max 65      maximum AGL in metres (default 65)
    --agl-step 5      AGL increment in metres (default 5)
    --rebuild         overwrite existing database
"""

import argparse, io, json, math, os, sys, time
import numpy as np
from PIL import Image, ImageFile
Image.MAX_IMAGE_PIXELS = None   # suppress decompression bomb warning for large satellite mosaics
import requests
import torch
import faiss

HERE    = os.path.dirname(os.path.abspath(__file__))
SIM_DIR = os.path.abspath(os.path.join(HERE, '..', 'simulator'))
DB_DIR  = os.path.join(HERE, 'database')
IMG_DIR = os.path.join(DB_DIR, 'db_images')

# Must match simulator/cesium_scene.py
CENTER_LAT = 23.450868
CENTER_LON = 120.286135
RADIUS_M   = 2000.0
R_EARTH    = 6_371_000.0
COS_LAT    = math.cos(math.radians(CENTER_LAT))
SAT_ZOOM   = 18   # ~0.60 m/px effective after MAX_TEX=16384 downsample

# Drone camera: AP-IMX900-Mini-USB3-I5, EFL 3.1 mm, 113.1°(D)×88°(H)×65.1°(V)
# GSD ≈ 75 mm/px @ 80 m AGL
HFOV_DEG = 88.0
VFOV_DEG = 65.1

# AnyLoc VLAD settings
DINO_IMG_W = 448    # must be divisible by 14 (ViT-B/14 patch size)
DINO_IMG_H = 336
VLAD_K     = 64


# ── Geo helpers ────────────────────────────────────────────────────────────────

def to_latlon(x_enu, y_enu):
    lat = CENTER_LAT + (y_enu / R_EARTH) * (180.0 / math.pi)
    lon = CENTER_LON + (x_enu / (R_EARTH * COS_LAT)) * (180.0 / math.pi)
    return lat, lon


# ── NLSC PHOTO2 tile download ─────────────────────────────────────────────────
# Taiwan NLSC WMTS — free, no API key required.
NLSC_TILE_URL = "https://wmts.nlsc.gov.tw/wmts/PHOTO2/default/GoogleMapsCompatible/{z}/{y}/{x}"

def fetch_satellite(sat_path, margin_factor=1.5):
    """Download NLSC PHOTO2 tiles and stitch to sat_path."""
    d_lat = RADIUS_M * margin_factor / 111_320.0
    d_lon = RADIUS_M * margin_factor / (111_320.0 * COS_LAT)
    tx_min, ty_min = _deg2tile(CENTER_LAT + d_lat, CENTER_LON - d_lon, SAT_ZOOM)
    tx_max, ty_max = _deg2tile(CENTER_LAT - d_lat, CENTER_LON + d_lon, SAT_ZOOM)

    nx = tx_max - tx_min + 1
    ny = ty_max - ty_min + 1
    TILE = 256
    mosaic = Image.new("RGB", (nx * TILE, ny * TILE))

    print(f"[DB] Downloading {nx}×{ny} NLSC PHOTO2 tiles at zoom {SAT_ZOOM} …")
    sess = requests.Session()
    sess.headers.update({"User-Agent": "AnyLocDB/1.0"})
    total = nx * ny
    done  = 0
    for tx in range(tx_min, tx_max + 1):
        for ty in range(ty_min, ty_max + 1):
            url = NLSC_TILE_URL.format(z=SAT_ZOOM, y=ty, x=tx)
            for attempt in range(3):
                try:
                    r = sess.get(url, timeout=15)
                    r.raise_for_status()
                    tile = Image.open(io.BytesIO(r.content)).convert("RGB")
                    mosaic.paste(tile, ((tx - tx_min) * TILE, (ty - ty_min) * TILE))
                    break
                except Exception as e:
                    if attempt == 2:
                        print(f"  [DB] skip tile {tx},{ty}: {e}")
                    else:
                        time.sleep(0.5)
            done += 1
            if done % 20 == 0 or done == total:
                print(f"  [DB] tiles: {done}/{total}")
            time.sleep(0.03)

    MAX_TEX = 16384
    if mosaic.width > MAX_TEX or mosaic.height > MAX_TEX:
        scale = MAX_TEX / max(mosaic.width, mosaic.height)
        mosaic = mosaic.resize((int(mosaic.width * scale), int(mosaic.height * scale)),
                               Image.LANCZOS)
    mosaic.save(sat_path, "JPEG", quality=92)
    print(f"[DB] Satellite saved: {mosaic.width}×{mosaic.height} → {sat_path}")


# ── Satellite tile bounds ─────────────────────────────────────────────────────

def _deg2tile(lat, lon, z):
    n  = 1 << z
    x  = int((lon + 180.0) / 360.0 * n)
    lr = math.log(math.tan(math.radians(lat)) + 1.0 / math.cos(math.radians(lat)))
    y  = int((1.0 - lr / math.pi) / 2.0 * n)
    return x, y


def _tile2deg(tx, ty, z):
    n   = 1 << z
    lon = tx / n * 360.0 - 180.0
    lat = math.degrees(math.atan(math.sinh(math.pi * (1 - 2 * ty / n))))
    return lat, lon


def compute_sat_bounds(margin_factor=1.5):
    d_lat = RADIUS_M * margin_factor / 111_320.0
    d_lon = RADIUS_M * margin_factor / (111_320.0 * COS_LAT)
    tx_min, ty_min = _deg2tile(CENTER_LAT + d_lat, CENTER_LON - d_lon, SAT_ZOOM)
    tx_max, ty_max = _deg2tile(CENTER_LAT - d_lat, CENTER_LON + d_lon, SAT_ZOOM)
    nw_lat, nw_lon = _tile2deg(tx_min,     ty_min,     SAT_ZOOM)
    se_lat, se_lon = _tile2deg(tx_max + 1, ty_max + 1, SAT_ZOOM)
    return dict(nw_lat=nw_lat, nw_lon=nw_lon, se_lat=se_lat, se_lon=se_lon)


# ── Satellite crop ─────────────────────────────────────────────────────────────

def get_satellite_crop(sat_img, bounds, lat, lon, agl_m, out_size=(640, 480)):
    """
    Return the satellite patch corresponding to a nadir drone view from (lat, lon, agl_m).
    Image coords: x=0 at west, y=0 at north (top-left = NW corner).
    """
    half_w_m = agl_m * math.tan(math.radians(HFOV_DEG / 2.0))
    half_h_m = agl_m * math.tan(math.radians(VFOV_DEG / 2.0))
    d_lat = half_h_m / 111_320.0
    d_lon = half_w_m / (111_320.0 * COS_LAT)

    img_w, img_h = sat_img.size
    lon_span = bounds['se_lon'] - bounds['nw_lon']
    lat_span = bounds['nw_lat'] - bounds['se_lat']   # positive (north − south)

    x1 = int((lon - d_lon - bounds['nw_lon']) / lon_span * img_w)
    x2 = int((lon + d_lon - bounds['nw_lon']) / lon_span * img_w)
    y1 = int((bounds['nw_lat'] - (lat + d_lat)) / lat_span * img_h)
    y2 = int((bounds['nw_lat'] - (lat - d_lat)) / lat_span * img_h)

    x1c, y1c = max(0, x1), max(0, y1)
    x2c, y2c = min(img_w, x2), min(img_h, y2)
    if x2c - x1c < 10 or y2c - y1c < 10:
        return None

    return sat_img.crop((x1c, y1c, x2c, y2c)).resize(out_size, Image.LANCZOS)


# ── PIL → torch tensor (avoids numpy dual-install issues) ─────────────────────

_MEAN = (0.485, 0.456, 0.406)
_STD  = (0.229, 0.224, 0.225)


def pil_to_tensor(pil_img):
    """PIL RGB → (C, H, W) normalised float32 tensor using PIL tobytes + frombuffer."""
    img = pil_img.resize((DINO_IMG_W, DINO_IMG_H), Image.LANCZOS).convert('RGB')
    t = torch.frombuffer(bytearray(img.tobytes()), dtype=torch.uint8) \
              .reshape(DINO_IMG_H, DINO_IMG_W, 3).float() / 255.0
    mean = torch.tensor(_MEAN).view(1, 1, 3)
    std  = torch.tensor(_STD).view(1, 1, 3)
    return ((t - mean) / std).permute(2, 0, 1)   # (H,W,C) → (C,H,W)


# ── numpy array → torch via buffer (avoids torch.from_numpy type check) ───────

def np_to_tensor(arr):
    """Convert a numpy float32 array to a torch tensor via raw bytes."""
    return torch.frombuffer(bytearray(arr.tobytes()), dtype=torch.float32) \
                .reshape(arr.shape)


# ── DINOv2 feature extraction ──────────────────────────────────────────────────

def load_dino(device, model_name='dinov2_vitb14'):
    print(f"[DB] Loading DINOv2 {model_name} on {device} …")
    model = torch.hub.load('facebookresearch/dinov2', model_name, pretrained=True)
    model.eval().to(device)
    return model


def extract_features(images, model, device, batch=8):
    """Return list of (N_patches, 768) float32 CPU tensors, one per image."""
    all_feats = []
    for i in range(0, len(images), batch):
        chunk = images[i:i + batch]
        x = torch.stack([pil_to_tensor(img) for img in chunk]).to(device)
        with torch.no_grad():
            out = model.forward_features(x)
        # out['x_norm_patchtokens']: (B, N_patches, 768)
        patches = out['x_norm_patchtokens'].cpu()   # keep as torch tensor
        for j in range(len(chunk)):
            all_feats.append(patches[j].float())
        if (i + batch) % 80 < batch:
            print(f"  features: {min(i + batch, len(images))}/{len(images)}")
    return all_feats


# ── VLAD codebook + descriptors (pure torch) ──────────────────────────────────

def build_codebook(all_feats_list, k=VLAD_K):
    """k-means via faiss on all patch features from the database images."""
    all_flat = torch.cat(all_feats_list, dim=0)   # (total_patches, 768)
    d = all_flat.shape[1]
    print(f"[DB] faiss k-means: {len(all_flat)} patches → k={k} clusters …")
    km = faiss.Kmeans(d, k, niter=50, verbose=False, gpu=False)
    km.train(all_flat.numpy())    # faiss needs numpy; .numpy() doesn't call numpy methods
    # centroids: numpy (k, d) → torch via frombuffer
    return np_to_tensor(km.centroids.astype('f4'))   # (k, d) float32 torch tensor


def compute_vlad(feats, codebook):
    """
    Intra-normalised VLAD descriptor, fully in torch.
    feats:    (N, d) float32 CPU tensor
    codebook: (k, d) float32 CPU tensor
    Returns:  (k*d,) float32 CPU tensor, L2-normalised
    """
    k, d = codebook.shape
    v = torch.zeros(k, d, dtype=torch.float32)

    ff = (feats ** 2).sum(1, keepdim=True)                        # (N, 1)
    cc = (codebook ** 2).sum(1, keepdim=True)                     # (k, 1)
    dists   = ff + cc.T - 2.0 * (feats @ codebook.T)             # (N, k)
    assigns = dists.argmin(1)                                      # (N,)

    for c in range(k):
        mask = assigns == c
        if mask.any():
            v[c] = (feats[mask] - codebook[c]).sum(0)

    norms = v.norm(dim=1, keepdim=True)
    v = v / (norms + 1e-8)
    v = v.flatten()
    v = v / (v.norm() + 1e-8)
    return v.float()


# ── Main ───────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--grid-step', type=float, default=50.0,
                    help='Grid spacing in metres (default 50)')
    ap.add_argument('--agl-min',  type=float, default=65.0,
                    help='Minimum AGL in metres (default 65)')
    ap.add_argument('--agl-max',  type=float, default=65.0,
                    help='Maximum AGL in metres (default 65)')
    ap.add_argument('--agl-step', type=float, default=5.0,
                    help='AGL step in metres (default 5)')
    ap.add_argument('--rebuild', action='store_true',
                    help='Rebuild even if database.pt already exists')
    ap.add_argument('--model', choices=['vitb14', 'vits14'], default='vitb14',
                    help='DINOv2 backbone (default: vitb14)')
    ap.add_argument('--db-dir', default='',
                    help='Output database directory (default: anyloc/database for vitb14, '
                         'anyloc/database_vits14 for vits14)')
    args = ap.parse_args()

    model_name = f'dinov2_{args.model}'
    if args.db_dir:
        db_dir = args.db_dir
    else:
        db_dir = DB_DIR if args.model == 'vitb14' \
            else os.path.join(HERE, 'database_vits14')
    img_dir = os.path.join(db_dir, 'db_images')

    db_file = os.path.join(db_dir, 'database.pt')
    if os.path.exists(db_file) and not args.rebuild:
        print(f"[DB] {db_file} already exists.  Use --rebuild to regenerate.")
        return

    agl_levels = np.arange(args.agl_min, args.agl_max + 1e-6, args.agl_step).tolist()
    print(f"[DB] AGL levels: {[f'{a:.0f}' for a in agl_levels]} m  ({len(agl_levels)} levels)")

    # Load satellite image — download from NLSC if not already on disk
    sat_path = os.path.join(SIM_DIR, 'satellite_ground.jpg')
    if not os.path.exists(sat_path):
        fetch_satellite(sat_path)
    sat_img = Image.open(sat_path).convert('RGB')
    bounds  = compute_sat_bounds()
    print(f"[DB] Satellite: {sat_img.size[0]}×{sat_img.size[1]}")
    print(f"[DB] Bounds:  {bounds['nw_lat']:.5f}°N – {bounds['se_lat']:.5f}°N  "
          f"{bounds['nw_lon']:.5f}°E – {bounds['se_lon']:.5f}°E")

    # Grid positions (same XY grid reused for every AGL level)
    step  = args.grid_step
    limit = RADIUS_M * 0.75
    coords = [(x, y)
              for x in np.arange(-limit, limit + 1, step)
              for y in np.arange(-limit, limit + 1, step)
              if math.hypot(x, y) <= limit]
    print(f"[DB] Grid step={step} m  →  {len(coords)} positions × {len(agl_levels)} AGL = "
          f"{len(coords) * len(agl_levels)} entries max")

    # ── Pass 1: crop → disk (skipped if metadata cache exists) ──────────────
    meta_file = os.path.join(db_dir, 'db_meta.json')
    os.makedirs(img_dir, exist_ok=True)

    if os.path.exists(meta_file) and not args.rebuild:
        print(f"[DB] Loading existing crop metadata from {meta_file}")
        with open(meta_file) as f:
            meta = json.load(f)
        db_lats  = meta['lats']
        db_lons  = meta['lons']
        db_alts  = meta['alts']
        db_paths = meta['paths']
        print(f"[DB] Loaded {len(db_paths)} entries from cache")
    else:
        db_lats, db_lons, db_alts, db_paths = [], [], [], []
        skipped = 0
        for agl in agl_levels:
            level_count = 0
            for x_enu, y_enu in coords:
                lat, lon = to_latlon(x_enu, y_enu)
                crop = get_satellite_crop(sat_img, bounds, lat, lon, agl)
                if crop is None:
                    skipped += 1
                    continue
                path = os.path.join(img_dir, f'{len(db_paths):06d}.jpg')
                crop.save(path, 'JPEG', quality=90)
                db_lats.append(lat); db_lons.append(lon)
                db_alts.append(agl); db_paths.append(path)
                level_count += 1
            print(f"[DB]   AGL={agl:.0f} m → {level_count} crops")
        print(f"[DB] Total: {len(db_paths)} patches  ({skipped} outside bounds)")
        with open(meta_file, 'w') as f:
            json.dump({'lats': db_lats, 'lons': db_lons,
                       'alts': db_alts, 'paths': db_paths}, f)
        print(f"[DB] Metadata saved → {meta_file}")

    n_total = len(db_paths)

    if n_total < VLAD_K:
        sys.exit(f"[DB] Too few images ({n_total}) for k={VLAD_K} codebook.")

    # ── DINOv2 model ──────────────────────────────────────────────────────────
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    model  = load_dino(device, model_name)

    # ── Pass 2: sample → codebook (load CODEBOOK_SAMPLE images, discard after) ─
    CODEBOOK_SAMPLE = 2000
    import random
    sample_paths = random.sample(db_paths, min(CODEBOOK_SAMPLE, n_total))
    print(f"[DB] Codebook sample: {len(sample_paths)} images …")
    sample_imgs  = [Image.open(p).convert('RGB') for p in sample_paths]
    sample_feats = extract_features(sample_imgs, model, device)
    del sample_imgs
    codebook = build_codebook(sample_feats, k=VLAD_K)
    del sample_feats
    print(f"[DB] Codebook: {tuple(codebook.shape)}")

    # ── Pass 3: all images → VLADs in batches (one batch in RAM at a time) ────
    BATCH = 8
    print(f"[DB] Computing VLAD descriptors …")
    vlad_list = []
    for i in range(0, n_total, BATCH):
        batch_imgs  = [Image.open(p).convert('RGB') for p in db_paths[i:i + BATCH]]
        batch_feats = extract_features(batch_imgs, model, device, batch=BATCH)
        for f in batch_feats:
            vlad_list.append(compute_vlad(f, codebook))
        del batch_imgs, batch_feats
        done = min(i + BATCH, n_total)
        if done % 500 < BATCH or done == n_total:
            print(f"  vlad: {done}/{n_total}")
    vlads = torch.stack(vlad_list)
    del vlad_list
    print(f"[DB] VLAD matrix: {tuple(vlads.shape)}  (dim={vlads.shape[1]})")

    # Free GPU model memory before the large disk write
    del model
    if device == 'cuda':
        torch.cuda.empty_cache()

    # ── Save ─────────────────────────────────────────────────────────────────────
    # PyTorch's miniz has a signed-32-bit overflow bug when the file path contains
    # multibyte (non-ASCII) characters — it silently caps writes at 2 GB.
    # Workaround: save to /tmp (ASCII path) then move into place.
    import shutil, tempfile
    os.makedirs(db_dir, exist_ok=True)
    meta_pt  = db_file.replace('database.pt', 'database_meta.pt')
    vlads_pt = db_file.replace('database.pt', 'database_vlads.pt')

    def _safe_save(obj, dst):
        tmp = tempfile.mktemp(suffix='.pt', dir='/tmp')
        torch.save(obj, tmp)
        shutil.move(tmp, dst)

    _safe_save({
        'lats':       torch.tensor(db_lats, dtype=torch.float32),
        'lons':       torch.tensor(db_lons, dtype=torch.float32),
        'alts':       torch.tensor(db_alts, dtype=torch.float32),
        'codebook':   codebook,
        'model_name': model_name,
    }, meta_pt)
    print(f"[DB] Meta saved → {meta_pt}  ({os.path.getsize(meta_pt)/1e6:.1f} MB)")

    _safe_save(vlads, vlads_pt)
    saved_size = os.path.getsize(vlads_pt)
    expected   = vlads.numel() * 4
    print(f"[DB] VLADs saved → {vlads_pt}  ({saved_size/1e9:.2f} GB, expected ~{expected/1e9:.2f} GB)")
    if saved_size < expected * 0.99:
        raise RuntimeError(f"database_vlads.pt is truncated: {saved_size} < {expected}")

    _safe_save({'_split': True, 'meta': meta_pt, 'vlads': vlads_pt}, db_file)
    print(f"[DB] Saved → {db_file}")
    print(f"[DB] Done: {n_total} entries, VLAD dim={vlads.shape[1]}")


if __name__ == '__main__':
    main()
