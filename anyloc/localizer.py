"""
AnyLoc localizer: DINOv2 patch features + VLAD aggregation + FAISS nearest-neighbour.

All intermediate operations use torch tensors to avoid numpy dual-install issues
in the Isaac Sim conda environment.

Usage:
    from localizer import AnyLocLocalizer
    loc = AnyLocLocalizer('anyloc/database')
    lat, lon, alt, matched_img, score, idx = loc.localize(pil_img, agl_m=50.0)
"""

import math, os
import numpy as np
from PIL import Image
import torch
import faiss

DINO_IMG_W = 448    # must be multiple of 14 (ViT-B/14 patch size)
DINO_IMG_H = 336

_MEAN = (0.485, 0.456, 0.406)
_STD  = (0.229, 0.224, 0.225)

# Scene constants — must match cesium_scene.py
_CENTER_LAT = 23.450868
_CENTER_LON = 120.286135
_RADIUS_M   = 2000.0
_COS_LAT    = math.cos(math.radians(_CENTER_LAT))
_SAT_ZOOM   = 18
_HFOV_DEG   = 62.2    # IMX219 CSI spec: 62.2°(H)
_VFOV_DEG   = 48.8    # IMX219 CSI spec: 48.8°(V)


# ── PIL → tensor (avoids numpy dual-install conflict) ─────────────────────────

def _pil_to_tensor(pil_img):
    """PIL RGB → (C, H, W) normalised float32 tensor via PIL tobytes + frombuffer."""
    img = pil_img.resize((DINO_IMG_W, DINO_IMG_H), Image.LANCZOS).convert('RGB')
    t = torch.frombuffer(bytearray(img.tobytes()), dtype=torch.uint8) \
              .reshape(DINO_IMG_H, DINO_IMG_W, 3).float() / 255.0
    mean = torch.tensor(_MEAN).view(1, 1, 3)
    std  = torch.tensor(_STD).view(1, 1, 3)
    return ((t - mean) / std).permute(2, 0, 1)   # (H,W,C) → (C,H,W)


# ── Satellite crop helpers (duplicated from build_database for self-containment) ─

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


def _sat_bounds(margin_factor=1.5):
    d_lat = _RADIUS_M * margin_factor / 111_320.0
    d_lon = _RADIUS_M * margin_factor / (111_320.0 * _COS_LAT)
    tx_min, ty_min = _deg2tile(_CENTER_LAT + d_lat, _CENTER_LON - d_lon, _SAT_ZOOM)
    tx_max, ty_max = _deg2tile(_CENTER_LAT - d_lat, _CENTER_LON + d_lon, _SAT_ZOOM)
    nw_lat, nw_lon = _tile2deg(tx_min,     ty_min,     _SAT_ZOOM)
    se_lat, se_lon = _tile2deg(tx_max + 1, ty_max + 1, _SAT_ZOOM)
    return dict(nw_lat=nw_lat, nw_lon=nw_lon, se_lat=se_lat, se_lon=se_lon)


def _sat_crop(sat_img, bounds, lat, lon, agl_m, out_size=(640, 480)):
    """Crop satellite image to the nadir footprint at (lat, lon, agl_m)."""
    half_w_m = agl_m * math.tan(math.radians(_HFOV_DEG / 2.0))
    half_h_m = agl_m * math.tan(math.radians(_VFOV_DEG / 2.0))
    d_lat = half_h_m / 111_320.0
    d_lon = half_w_m / (111_320.0 * _COS_LAT)

    img_w, img_h = sat_img.size
    lon_span = bounds['se_lon'] - bounds['nw_lon']
    lat_span = bounds['nw_lat'] - bounds['se_lat']

    x1 = int((lon - d_lon - bounds['nw_lon']) / lon_span * img_w)
    x2 = int((lon + d_lon - bounds['nw_lon']) / lon_span * img_w)
    y1 = int((bounds['nw_lat'] - (lat + d_lat)) / lat_span * img_h)
    y2 = int((bounds['nw_lat'] - (lat - d_lat)) / lat_span * img_h)

    x1c, y1c = max(0, x1), max(0, y1)
    x2c, y2c = min(img_w, x2), min(img_h, y2)
    if x2c - x1c < 10 or y2c - y1c < 10:
        return None
    return sat_img.crop((x1c, y1c, x2c, y2c)).resize(out_size, Image.LANCZOS)


class AnyLocLocalizer:
    """
    Visual place recognition using DINOv2 patch tokens + intra-normalised VLAD.
    Call build_database.py once to create the database before using this class.
    """

    def __init__(self, db_dir: str, device: str = 'auto', model_name: str = None):
        self.db_dir = db_dir
        self.device = ('cuda' if torch.cuda.is_available() else 'cpu') \
            if device == 'auto' else device
        self._model_name_override = model_name
        self._load_db()
        self._load_sat()
        self._load_model()

    # ── Database ───────────────────────────────────────────────────────────────

    def _load_db(self):
        db_file = os.path.join(self.db_dir, 'database.pt')
        if not os.path.exists(db_file):
            raise FileNotFoundError(
                f"Database not found: {db_file}\n"
                "Build it first:\n"
                "  conda run -n isaac_sim_test python anyloc/build_database.py"
            )
        db = torch.load(db_file, map_location='cpu', weights_only=False)
        if db.get('_split'):
            # database.pt stores absolute paths baked in at build time — not
            # portable across machines (e.g. built on a dev PC, copied to the
            # Jetson). Resolve by filename relative to db_dir instead of
            # trusting the stored path.
            meta_path  = os.path.join(self.db_dir, os.path.basename(db['meta']))
            vlads_path = os.path.join(self.db_dir, os.path.basename(db['vlads']))
            meta  = torch.load(meta_path,  map_location='cpu', weights_only=False)
            vlads = torch.load(vlads_path, map_location='cpu', weights_only=False)
            db = {**meta, 'vlads': vlads}
        self.model_name = self._model_name_override or \
            db.get('model_name', 'dinov2_vitb14')
        self.lats     = db['lats']       # (N,) float32 tensor
        self.lons     = db['lons']
        self.alts     = db['alts']
        self.vlads    = db['vlads']      # (N, D) float32 tensor
        self.codebook = db['codebook']   # (k, d) float32 tensor
        self.img_dir  = os.path.join(self.db_dir, 'db_images')
        n, d = self.vlads.shape
        print(f"[AnyLoc] DB: {n} entries, VLAD dim={d}")

        self._index = faiss.IndexFlatIP(d)
        self._index.add(self.vlads.numpy())
        print(f"[AnyLoc] FAISS index: {d}D × {n}")

    # ── Satellite image (for altitude-corrected match crops) ───────────────────

    def _load_sat(self):
        sat_path = os.path.abspath(
            os.path.join(self.db_dir, '..', '..', 'simulator', 'satellite_ground.jpg'))
        if os.path.exists(sat_path):
            self._sat_img    = Image.open(sat_path).convert('RGB')
            self._sat_bounds = _sat_bounds()
            print(f"[AnyLoc] Satellite image loaded for altitude-corrected crops")
        else:
            self._sat_img    = None
            self._sat_bounds = None
            print(f"[AnyLoc] Satellite image not found — match crops use DB altitude (50 m)")

    # ── DINOv2 model ───────────────────────────────────────────────────────────

    def _load_model(self):
        print(f"[AnyLoc] Loading DINOv2 {self.model_name} on {self.device} …")
        self.model = torch.hub.load(
            'facebookresearch/dinov2', self.model_name, pretrained=True)
        self.model.eval().to(self.device)
        print(f"[AnyLoc] Model ready on {self.device}")

    # ── Feature extraction ─────────────────────────────────────────────────────

    def _patch_features(self, pil_img: Image.Image) -> torch.Tensor:
        """DINOv2 patch tokens. Returns (N_patches, 768) float32 CPU tensor."""
        x = _pil_to_tensor(pil_img.convert('RGB')).unsqueeze(0).to(self.device)
        with torch.no_grad():
            out = self.model.forward_features(x)
        return out['x_norm_patchtokens'].squeeze(0).cpu().float()

    # ── VLAD (pure torch) ──────────────────────────────────────────────────────

    def _vlad(self, feats: torch.Tensor) -> torch.Tensor:
        """
        Intra-normalised VLAD.
        feats: (N, d) float32 CPU tensor.
        Returns: (k*d,) float32 L2-normalised tensor.
        """
        k, d = self.codebook.shape
        v = torch.zeros(k, d, dtype=torch.float32)

        ff = (feats ** 2).sum(1, keepdim=True)
        cc = (self.codebook ** 2).sum(1, keepdim=True)
        assigns = (ff + cc.T - 2.0 * (feats @ self.codebook.T)).argmin(1)

        for c in range(k):
            mask = assigns == c
            if mask.any():
                v[c] = (feats[mask] - self.codebook[c]).sum(0)

        norms = v.norm(dim=1, keepdim=True)
        v = v / (norms + 1e-8)
        v = v.flatten()
        v = v / (v.norm() + 1e-8)
        return v.float()

    # ── Public API ─────────────────────────────────────────────────────────────

    def localize(self, pil_img: Image.Image, agl_m: float = None,
                 center_lat: float = None, center_lon: float = None,
                 radius_m: float = None):
        """
        Estimate geo position from a single PIL image.

        Args:
            pil_img     PIL image from the drone camera
            agl_m       drone altitude above ground in metres
            center_lat  if given, restrict search to DB entries within radius_m
            center_lon  of (center_lat, center_lon) — pass the VO-refined estimate
            radius_m    search radius in metres (ignored when center_* is None)

        Returns:
            est_lat   float  — estimated latitude
            est_lon   float  — estimated longitude
            est_alt   float  — agl_m if provided, else DB entry altitude (m AGL)
            match_img PIL    — satellite crop at the matched position & altitude
            score     float  — cosine similarity ∈ [-1, 1], higher = better
            db_idx    int    — index into the database
        """
        feats = self._patch_features(pil_img)
        desc  = self._vlad(feats)   # (D,) normalised float32 tensor

        if center_lat is not None and center_lon is not None and radius_m is not None:
            dlat    = (self.lats - center_lat) * 111_320.0
            dlon    = (self.lons - center_lon) * 111_320.0 * _COS_LAT
            in_range = ((dlat ** 2 + dlon ** 2) <= radius_m ** 2) \
                           .nonzero(as_tuple=False).squeeze(1)
            if len(in_range) == 0:
                in_range = torch.arange(len(self.lats))
            sims  = self.vlads[in_range] @ desc   # cosine sim — both L2-normed
            best  = int(sims.argmax())
            idx   = int(in_range[best])
            score = float(sims[best])
        else:
            scores_np, idxs_np = self._index.search(desc.unsqueeze(0).numpy(), k=1)
            idx   = int(idxs_np[0, 0])
            score = float(scores_np[0, 0])

        est_lat = float(self.lats[idx])
        est_lon = float(self.lons[idx])

        # Re-crop at the drone's actual AGL when satellite image is available
        match_img = None
        if agl_m is not None and self._sat_img is not None:
            match_img = _sat_crop(self._sat_img, self._sat_bounds,
                                  est_lat, est_lon, agl_m)

        if match_img is None:
            # Fallback: use the pre-built 50 m database crop
            img_path  = os.path.join(self.img_dir, f'{idx:06d}.jpg')
            match_img = Image.open(img_path).convert('RGB') \
                if os.path.exists(img_path) else pil_img

        est_alt = agl_m if agl_m is not None else float(self.alts[idx])
        return est_lat, est_lon, est_alt, match_img, score, idx
