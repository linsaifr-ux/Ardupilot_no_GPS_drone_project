#!/usr/bin/env python3
"""
Build AnyLoc database from real drone footage collected with record_field.py
and extracted with extract_frames.py.

Produces the same split database format as build_database.py so localizer.py
works without any changes.

Usage:
    python3 anyloc/build_database_real.py <session_dir> [OPTIONS]

    <session_dir>      directory produced by record_field.py + extract_frames.py
                       must contain frames.csv and frames/

    --db-dir DIR       output database directory
                       (default: anyloc/database_real)
    --model vitb14|vits14
                       DINOv2 backbone (default: vits14, matches active DB)
    --rebuild          overwrite existing database

Example:
    python3 anyloc/build_database_real.py field_data/20260623_120000/
    ln -sfn database_real anyloc/database
"""

import argparse
import csv
import os
import shutil
import sys
import tempfile

import torch
from PIL import Image

# Reuse pipeline utilities from build_database.py (same directory)
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from build_database import (
    VLAD_K,
    load_dino,
    extract_features,
    build_codebook,
    compute_vlad,
    pil_to_tensor,
)

DB_THUMB_SIZE = (640, 480)   # size for db_images/ thumbnails (visualisation only)
CODEBOOK_SAMPLE = 2000


def read_frames_csv(session_dir):
    csv_path = os.path.join(session_dir, 'frames.csv')
    if not os.path.exists(csv_path):
        sys.exit(f'frames.csv not found in {session_dir}\n'
                 'Run tools/extract_frames.py first.')
    entries = []
    with open(csv_path) as f:
        for row in csv.DictReader(f):
            try:
                agl = float(row['alt_agl'])
                lat = float(row['lat'])
                lon = float(row['lon'])
            except (ValueError, KeyError) as e:
                print(f'  [DB] skip row (bad value: {e}): {row}')
                continue
            img_path = os.path.join(session_dir, row['path'])
            if not os.path.exists(img_path):
                print(f'  [DB] skip missing image: {img_path}')
                continue
            entries.append(dict(path=img_path, lat=lat, lon=lon, agl=agl))
    return entries


def _safe_save(obj, dst):
    """Save via /tmp to avoid PyTorch miniz 2 GB overflow on non-ASCII paths."""
    tmp = tempfile.mktemp(suffix='.pt', dir='/tmp')
    torch.save(obj, tmp)
    shutil.move(tmp, dst)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('session_dir',
                    help='Session directory from record_field.py + extract_frames.py')
    ap.add_argument('--db-dir', default='',
                    help='Output database directory (default: anyloc/database_real)')
    ap.add_argument('--model', choices=['vitb14', 'vits14'], default='vits14',
                    help='DINOv2 backbone (default: vits14)')
    ap.add_argument('--rebuild', action='store_true',
                    help='Rebuild even if database.pt already exists')
    args = ap.parse_args()

    model_name = f'dinov2_{args.model}'
    db_dir     = args.db_dir or os.path.join(HERE, 'database_real')
    img_dir    = os.path.join(db_dir, 'db_images')
    db_file    = os.path.join(db_dir, 'database.pt')
    meta_pt    = os.path.join(db_dir, 'database_meta.pt')
    vlads_pt   = os.path.join(db_dir, 'database_vlads.pt')

    if os.path.exists(db_file) and not args.rebuild:
        print(f'[DB] {db_file} already exists — use --rebuild to regenerate.')
        return

    # ── Load frame list ────────────────────────────────────────────────────────
    entries = read_frames_csv(args.session_dir)
    n_total = len(entries)
    if n_total == 0:
        sys.exit('[DB] No valid frames found.')
    if n_total < VLAD_K:
        sys.exit(f'[DB] Too few frames ({n_total}) for k={VLAD_K} codebook.')

    print(f'[DB] Session : {args.session_dir}')
    print(f'[DB] Frames  : {n_total}')
    print(f'[DB] Output  : {db_dir}')
    print(f'[DB] Backbone: {model_name}')

    os.makedirs(img_dir, exist_ok=True)

    # ── Copy thumbnails to db_images/ (localizer visualisation fallback) ───────
    print('[DB] Copying thumbnails …')
    db_lats, db_lons, db_alts, db_paths = [], [], [], []
    for i, e in enumerate(entries):
        thumb_path = os.path.join(img_dir, f'{i:06d}.jpg')
        if not os.path.exists(thumb_path) or args.rebuild:
            img = Image.open(e['path']).convert('RGB')
            img.thumbnail(DB_THUMB_SIZE, Image.LANCZOS)
            img.save(thumb_path, 'JPEG', quality=90)
        db_lats.append(e['lat'])
        db_lons.append(e['lon'])
        db_alts.append(e['agl'])
        db_paths.append(thumb_path)
        if (i + 1) % 200 == 0 or (i + 1) == n_total:
            print(f'  thumbnails: {i+1}/{n_total}')

    # ── DINOv2 model ───────────────────────────────────────────────────────────
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    model  = load_dino(device, model_name)

    # ── Codebook from a random sample ─────────────────────────────────────────
    import random
    sample_paths = random.sample(db_paths, min(CODEBOOK_SAMPLE, n_total))
    print(f'[DB] Building codebook from {len(sample_paths)} sample images …')
    sample_imgs  = [Image.open(p).convert('RGB') for p in sample_paths]
    sample_feats = extract_features(sample_imgs, model, device)
    del sample_imgs
    codebook = build_codebook(sample_feats, k=VLAD_K)
    del sample_feats
    print(f'[DB] Codebook: {tuple(codebook.shape)}')

    # ── VLAD for every frame ───────────────────────────────────────────────────
    BATCH = 8
    print('[DB] Computing VLAD descriptors …')
    vlad_list = []
    for i in range(0, n_total, BATCH):
        batch_imgs  = [Image.open(p).convert('RGB') for p in db_paths[i:i + BATCH]]
        batch_feats = extract_features(batch_imgs, model, device, batch=BATCH)
        for f in batch_feats:
            vlad_list.append(compute_vlad(f, codebook))
        del batch_imgs, batch_feats
        done = min(i + BATCH, n_total)
        if done % 200 < BATCH or done == n_total:
            print(f'  vlad: {done}/{n_total}')
    vlads = torch.stack(vlad_list)
    del vlad_list
    print(f'[DB] VLAD matrix: {tuple(vlads.shape)}  (dim={vlads.shape[1]})')

    del model
    if device == 'cuda':
        torch.cuda.empty_cache()

    # ── Save (split format — identical to build_database.py) ──────────────────
    _safe_save({
        'lats':       torch.tensor(db_lats, dtype=torch.float32),
        'lons':       torch.tensor(db_lons, dtype=torch.float32),
        'alts':       torch.tensor(db_alts, dtype=torch.float32),
        'codebook':   codebook,
        'model_name': model_name,
    }, meta_pt)
    print(f'[DB] Meta  → {meta_pt}  ({os.path.getsize(meta_pt)/1e6:.1f} MB)')

    _safe_save(vlads, vlads_pt)
    saved_size = os.path.getsize(vlads_pt)
    expected   = vlads.numel() * 4
    print(f'[DB] VLADs → {vlads_pt}  ({saved_size/1e6:.1f} MB)')
    if saved_size < expected * 0.99:
        raise RuntimeError(f'database_vlads.pt truncated: {saved_size} < {expected}')

    _safe_save({'_split': True, 'meta': meta_pt, 'vlads': vlads_pt}, db_file)
    print(f'[DB] Done  → {db_file}')
    print(f'[DB] {n_total} entries, VLAD dim={vlads.shape[1]}')
    print()
    print('To activate this database:')
    print(f'  ln -sfn {os.path.relpath(db_dir, HERE)} anyloc/database')


if __name__ == '__main__':
    main()
