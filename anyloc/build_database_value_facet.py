#!/usr/bin/env python3
"""
Build an AnyLoc-style database using the AnyLoc-paper "value facet" feature
extraction (arXiv:2308.00688 Sec III-B / Fig. 3) instead of anyloc/
build_database.py's default final-layer DINOv2 patch tokens (x_norm_patchtokens).

Reuses an EXISTING database's already-cropped images + lat/lon/alt metadata
(--src-db-dir/db_meta.json) so satellite tile download/cropping is NOT repeated
-- only feature extraction + VLAD codebook/descriptors are redone with the new
extraction method. This keeps the value-facet ablation cheap to run against a
DB that's already been rebuilt for satellite resolution (Fix 1).

Does NOT touch anyloc/build_database.py or anyloc/localizer.py's default
AnyLocLocalizer -- standalone tool for the offline value-facet ablation
(field_data/survey25/vio_eval). Query-side extraction must use the matching
anyloc.localizer.AnyLocLocalizerValueFacet(model_name=..., layer=...) so the
query and database live in the same feature space.

Usage:
    /home/jetson/venv/anyloc/bin/python3 anyloc/build_database_value_facet.py \\
        --src-db-dir anyloc/database_survey25_z20_vits14 \\
        --db-dir anyloc/database_survey25_z20_vf_L9_vits14 \\
        --model dinov2_vits14 --layer 9
"""
import argparse
import json
import os
import shutil
import sys

import faiss
import numpy as np
import torch
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
from anyloc.localizer import _pil_to_tensor, _value_facet   # shared w/ query-side extraction, byte-identical

VLAD_K = 64   # matches this project's existing DB convention (build_database.py module default)


def compute_vlad(feats: torch.Tensor, codebook: torch.Tensor) -> torch.Tensor:
    """Intra-normalised VLAD, identical formula to build_database.py/localizer.py."""
    k, d = codebook.shape
    v = torch.zeros(k, d, dtype=torch.float32)
    ff = (feats ** 2).sum(1, keepdim=True)
    cc = (codebook ** 2).sum(1, keepdim=True)
    dists = ff + cc.T - 2.0 * (feats @ codebook.T)
    assigns = dists.argmin(1)
    for c in range(k):
        mask = assigns == c
        if mask.any():
            v[c] = (feats[mask] - codebook[c]).sum(0)
    norms = v.norm(dim=1, keepdim=True)
    v = v / (norms + 1e-8)
    v = v.flatten()
    v = v / (v.norm() + 1e-8)
    return v.float()


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--src-db-dir', required=True,
                    help='existing DB dir to reuse crops + lat/lon/alt from (needs db_meta.json)')
    ap.add_argument('--db-dir', required=True, help='output DB dir')
    ap.add_argument('--model', default='dinov2_vits14')
    ap.add_argument('--layer', type=int, required=True, help='0-indexed transformer block to hook')
    ap.add_argument('--vlad-k', type=int, default=VLAD_K)
    args = ap.parse_args()

    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    meta = json.load(open(os.path.join(args.src_db_dir, 'db_meta.json')))
    paths, lats, lons, alts = meta['paths'], meta['lats'], meta['lons'], meta['alts']
    repo_root = os.path.dirname(HERE)

    print(f"[VF-DB] {len(paths)} source crops from {args.src_db_dir}")
    print(f"[VF-DB] Loading {args.model} on {device} ...")
    model = torch.hub.load('facebookresearch/dinov2', args.model, pretrained=True)
    model.eval().to(device)

    print(f"[VF-DB] extracting layer {args.layer} value facet for {len(paths)} images ...")
    all_feats = []
    for i, p in enumerate(paths):
        full_p = p if os.path.isabs(p) else os.path.join(repo_root, p)
        img = Image.open(full_p).convert('RGB')
        x = _pil_to_tensor(img).unsqueeze(0).to(device)
        feats = _value_facet(model, x, args.layer).squeeze(0).cpu().float()
        all_feats.append(feats)
        if (i + 1) % 40 == 0 or i + 1 == len(paths):
            print(f"  features: {i+1}/{len(paths)}")

    print(f"[VF-DB] faiss k-means: k={args.vlad_k} ...")
    all_flat = torch.cat(all_feats, dim=0)
    d = all_flat.shape[1]
    km = faiss.Kmeans(d, args.vlad_k, niter=50, verbose=False, gpu=False)
    km.train(all_flat.numpy())
    codebook = torch.frombuffer(bytearray(km.centroids.astype('f4').tobytes()), dtype=torch.float32) \
                    .reshape(args.vlad_k, d)

    print("[VF-DB] computing VLAD descriptors ...")
    vlads = torch.stack([compute_vlad(f, codebook) for f in all_feats])
    print(f"[VF-DB] VLAD matrix: {tuple(vlads.shape)}")

    os.makedirs(args.db_dir, exist_ok=True)
    img_dir = os.path.join(args.db_dir, 'db_images')
    os.makedirs(img_dir, exist_ok=True)
    new_paths = []
    for i, p in enumerate(paths):
        full_p = p if os.path.isabs(p) else os.path.join(repo_root, p)
        dst = os.path.join(img_dir, f'{i:06d}.jpg')
        if not os.path.exists(dst):
            shutil.copy(full_p, dst)
        new_paths.append(dst)

    db = {
        'lats': torch.tensor(lats, dtype=torch.float32),
        'lons': torch.tensor(lons, dtype=torch.float32),
        'alts': torch.tensor(alts, dtype=torch.float32),
        'vlads': vlads,
        'codebook': codebook,
        'model_name': args.model,
        'feature_mode': 'value_facet',
        'value_layer': args.layer,
    }
    torch.save(db, os.path.join(args.db_dir, 'database.pt'))
    with open(os.path.join(args.db_dir, 'db_meta.json'), 'w') as f:
        json.dump({'lats': lats, 'lons': lons, 'alts': alts, 'paths': new_paths,
                    'model': args.model, 'layer': args.layer}, f)
    print(f"[VF-DB] saved -> {args.db_dir}  ({len(paths)} entries, vlad dim {vlads.shape[1]})")


if __name__ == '__main__':
    main()
